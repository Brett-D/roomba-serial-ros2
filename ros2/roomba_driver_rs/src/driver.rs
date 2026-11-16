//! The roomba_driver node's logic without ROS (port of roomba_driver/node.py): sensor frames ->
//! odometry, mode tracking and the bump stop / backup reflex; the supervisor (contact loss, mode
//! changes, initial mode, drive command watchdog); drive and actuator topics; service results.
//! The robot is reached through [`Link`] (the serial [`crate::roomba::Roomba`]) and logs / the
//! mode topic through [`Out`], so the behaviour can be tested with fakes. Log lines and service
//! messages are the Python driver's, word for word.

use std::collections::VecDeque;
use std::sync::Mutex;

use crate::battery::{BatteryFields, BatteryGauge, Gauge, Reading};
use crate::kinematics::{twist_to_wheel_speeds, DiffDriveOdometry};
use crate::oi::{self, mode_name, mode_str, Field as F, SensorValues};
use crate::roomba::lock;

/// Sensor data older than this means contact with the robot is lost.
pub const STALE_AFTER_S: f64 = 1.0;
/// Period of the supervisor timer (contact loss, mode changes, bump reflex, drive watchdog).
pub const SUPERVISE_PERIOD_S: f64 = 0.05;
/// Longest wait for the charger packet (34, only in the primary stream layout) after a drop to
/// Passive was seen in a secondary frame.
const CHARGER_WAIT_S: f64 = 1.0;
const BUMP_FIELDS: [F; 2] = [F::BumpLeft, F::BumpRight];
/// (flag, signal) of the front light bumpers; the first two are on the left.
const LIGHT_BUMP_FIELDS: [(F, F); 4] = [
    (F::LightBumpFrontLeft, F::LightBumpFrontLeftSignal),
    (F::LightBumpCenterLeft, F::LightBumpCenterLeftSignal),
    (F::LightBumpCenterRight, F::LightBumpCenterRightSignal),
    (F::LightBumpFrontRight, F::LightBumpFrontRightSignal),
];
const BLOCKERS: [F; 8] = [
    F::WheelDropLeft, F::WheelDropRight, F::CliffLeft, F::CliffFrontLeft, F::CliffFrontRight, F::CliffRight,
    F::HomeBaseAvailable, F::InternalChargerAvailable,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Info,
    Warn,
    Error,
}

/// The robot as the node sees it.
pub trait Link {
    fn write(&self, data: &[u8]) -> Result<(), String>;
    fn is_open(&self) -> bool;
    fn streaming(&self) -> bool;
    fn wake_with_dtr(&self) -> bool;
    fn wake(&self) -> Result<(), String>;
    fn request_mode(&self, mode: u8) -> Result<(), String>;
    fn set_mode(&self, mode: u8, timeout: f64) -> Result<Option<i32>, String>;
    fn wait_for_frame(&self, timeout: f64) -> bool;
    /// Sensor values accumulated over every frame (without ignore_sensors).
    fn latest(&self) -> SensorValues;
}

/// Logging and the latched roomba/mode topic.
pub trait Out {
    fn log(&self, level: Level, msg: &str);
    fn publish_mode(&self, mode: &str);
}

fn is_control(mode: Option<i64>) -> bool {
    matches!(mode, Some(2) | Some(3))
}

/// Conditions that keep the robot out of (or drop it from) Safe mode.
pub fn blockers(values: &SensorValues) -> String {
    BLOCKERS.iter().filter(|&&f| values.flag(f)).map(|f| f.name()).collect::<Vec<_>>().join(", ")
}

/// Supervisor ticks until a bump reflex step ends: the first tick at or after its end.
/// A step ending exactly on a tick ends on it, rather than timer jitter deciding on one more command.
fn ticks(seconds: f64) -> i64 {
    (seconds / SUPERVISE_PERIOD_S - 1e-6).ceil() as i64
}

/// Node parameters used by the logic.
#[derive(Clone, Debug)]
pub struct Config {
    pub initial_mode: u8,
    pub auto_reenter_mode: bool,
    pub wheel_base: f64,
    pub wheel_diameter: f64,
    pub counts_per_rev: f64,
    /// Already limited to the hardware maximum.
    pub max_wheel_speed: f64,
    pub cmd_timeout: f64,
    pub bump_stop: bool,
    pub bump_stop_light: bool,
    pub light_stop_signal: i64,
    pub bump_backup: bool,
    pub backup_distance: f64,
    pub backup_speed: f64,
    pub backup_turn: f64,
    pub ignore_sensors: Vec<F>,
    pub gauge: Gauge,
}

impl Default for Config {
    /// The Python node's parameter defaults.
    fn default() -> Self {
        Self {
            initial_mode: oi::MODE_SAFE,
            auto_reenter_mode: false,
            wheel_base: 0.235,
            wheel_diameter: 0.072,
            counts_per_rev: 508.8,
            max_wheel_speed: 0.3,
            cmd_timeout: 0.5,
            bump_stop: true,
            bump_stop_light: false,
            light_stop_signal: 0,
            bump_backup: false,
            backup_distance: 0.08,
            backup_speed: 0.1,
            backup_turn: 0.4,
            ignore_sensors: Vec::new(),
            gauge: Gauge::Roomba,
        }
    }
}

/// Call sites of the throttled log lines (rclpy throttles each call site separately).
#[derive(Clone, Copy)]
enum Site {
    TryFailed,
    NoData,
    WrongMode,
    DuringBackup,
    ForwardBumped,
    NotFinite,
}

#[derive(Default)]
struct Throttle([Option<f64>; 6]);

impl Throttle {
    /// Log the first time, then at most once per period.
    fn allow(&mut self, site: Site, period: f64, now: f64) -> bool {
        let last = &mut self.0[site as usize];
        if last.is_some_and(|t| now < t + period) {
            return false;
        }
        *last = Some(now);
        true
    }
}

/// Wheel odometry to publish.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OdomSnapshot {
    pub stamp_ns: i64,
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub linear: f64,
    pub angular: f64,
    pub positions: [f64; 2],
    pub velocities: [f64; 2],
}

/// (previous mode, new mode, merged values, monotonic time since when it waits for the charger packet).
type ModeEvent = (Option<i64>, i64, SensorValues, Option<f64>);

pub struct Core {
    pub cfg: Config,
    values: SensorValues,
    latest_stamp: Option<i64>,
    pub odom: DiffDriveOdometry,
    odom_stamp: Option<i64>,
    pub mode: Option<i64>,
    mode_event: Option<ModeEvent>,
    pub desired_mode: Option<u8>,
    apply_initial_mode: bool,
    check_mode_at: Option<f64>,
    last_frame_mono: f64,
    contact_lost: bool,
    pub driving: bool,
    pub driving_forward: bool,
    pub bumped: bool,
    bump_side: i32,
    /// (left mm/s, right mm/s, end supervisor tick) steps of the bump reflex.
    backup: Option<VecDeque<(i64, i64, i64)>>,
    /// Monotonic time of the reflex's first supervisor tick.
    backup_start: f64,
    /// Side of a bump waiting for the backup reflex.
    backup_request: i32,
    /// Monotonic starts of recent reflexes (rate limit).
    backup_times: Vec<f64>,
    last_drive_time: f64,
    last_reenter: f64,
    /// (padded text, index)
    scroll: Option<(Vec<char>, usize)>,
    battery: BatteryGauge,
    throttle: Throttle,
}

impl Core {
    pub fn new(cfg: Config) -> Self {
        let odom = DiffDriveOdometry::new(cfg.wheel_base, cfg.wheel_diameter, cfg.counts_per_rev,
                                          oi::MAX_WHEEL_SPEED_MM_S as f64 / 1000.0 * 1.5);
        Self {
            values: SensorValues::default(),
            latest_stamp: None,
            odom,
            odom_stamp: None,
            mode: None,
            mode_event: None,
            desired_mode: Some(cfg.initial_mode),
            apply_initial_mode: true,
            check_mode_at: None,
            last_frame_mono: 0.0,
            contact_lost: false,
            driving: false,
            driving_forward: false,
            bumped: false,
            bump_side: 0,
            backup: None,
            backup_start: 0.0,
            backup_request: 0,
            backup_times: Vec::new(),
            last_drive_time: 0.0,
            last_reenter: 0.0,
            scroll: None,
            battery: BatteryGauge::new(cfg.gauge),
            throttle: Throttle::default(),
            cfg,
        }
    }

    fn desired_is_control(&self) -> bool {
        is_control(self.desired_mode.map(i64::from))
    }

    fn stopped(&mut self) {
        self.driving = false;
        self.driving_forward = false;
    }

    // ------------------------------------------------------------ connection / stream
    /// Reader-thread callback for every 15 ms sensor frame.
    pub fn on_frame(&mut self, frame: &SensorValues, stamp_ns: i64, monotonic: f64, link: &dyn Link, out: &dyn Out) {
        // Frames alternate between two layouts; keep the union.
        self.values.update(frame);
        for &f in &self.cfg.ignore_sensors {
            if self.values.has(f) {
                self.values.set(f, 0);
            }
        }
        self.latest_stamp = Some(stamp_ns);
        self.last_frame_mono = monotonic;
        let (Some(left), Some(right)) = (frame.get(F::LeftEncoderCounts), frame.get(F::RightEncoderCounts)) else {
            out.log(Level::Error, "sensor callback failed: KeyError('left_encoder_counts')");
            return;
        };
        if self.odom.update(left as i64, right as i64, stamp_ns as f64 * 1e-9) {
            self.odom_stamp = Some(stamp_ns);
        }
        let Some(mode) = frame.get(F::OiMode).map(i64::from) else {
            out.log(Level::Error, "sensor callback failed: KeyError('oi_mode')");
            return;
        };
        if Some(mode) != self.mode {
            // A drop from Safe/Full to Passive without a cliff or wheel drop is re-entered unless
            // the robot is on the charger, which only primary frames show. If a secondary frame
            // shows the drop first, wait for the next primary frame: the charger state merged so
            // far is from before the drop.
            let wait = is_control(self.mode)
                && mode == oi::MODE_PASSIVE as i64
                && !frame.has(F::HomeBaseAvailable)
                && blockers(&self.values).is_empty();
            self.mode_event = Some((self.mode, mode, self.values, wait.then_some(monotonic)));
            self.mode = Some(mode);
        } else if frame.has(F::HomeBaseAvailable) {
            if let Some(event) = self.mode_event.as_mut().filter(|e| e.3.is_some()) {
                event.2 = self.values;
                event.3 = None;
            }
        }
        let light = self.cfg.bump_stop_light;
        let bumped = BUMP_FIELDS.iter().any(|&f| self.values.flag(f))
            || (light && LIGHT_BUMP_FIELDS.iter().any(|&l| self.light_bumped(l)));
        let newly_bumped = bumped && !self.bumped;
        self.bumped = bumped;
        if newly_bumped {
            let left = self.values.flag(F::BumpLeft) || (light && LIGHT_BUMP_FIELDS[..2].iter().any(|&l| self.light_bumped(l)));
            let right = self.values.flag(F::BumpRight) || (light && LIGHT_BUMP_FIELDS[2..].iter().any(|&l| self.light_bumped(l)));
            self.bump_side = if left && !right { 1 } else if right && !left { -1 } else { 0 };
        }
        if newly_bumped
            && self.cfg.bump_stop
            && (self.driving_forward || (self.driving && self.cfg.bump_backup && self.backup.is_none()))
        {
            // Stop straight from the reader thread: waiting for the next timer adds latency.
            let r = link.write(&oi::drive_direct(0.0, 0.0));
            if self.try_result(r, "stop on bump", monotonic, out) {
                self.stopped();
            }
            if self.request_backup(monotonic) {
                out.log(Level::Warn, "bumper hit: backing up");
            } else {
                out.log(Level::Warn, "bumper hit: stopped (reverse or turn to get free)");
            }
        }
    }

    /// Queue the bump reflex unless it is off, running, or has fired too often lately.
    fn request_backup(&mut self, now: f64) -> bool {
        if !self.cfg.bump_backup || self.backup.is_some() || self.backup_request != 0 {
            return false;
        }
        self.backup_times.retain(|&t| now - t < 30.0);
        if self.backup_times.len() >= 4 || self.backup_times.last().is_some_and(|&t| now - t < 3.0) {
            return false; // still pressed after several tries (stuck sensor, wedged): just stop
        }
        self.backup_times.push(now);
        self.backup_request = if self.bump_side != 0 { self.bump_side } else { 1 };
        true
    }

    /// A light bumper counts as a bump (its flag, or its signal when a threshold is set).
    fn light_bumped(&self, (flag, signal): (F, F)) -> bool {
        if self.cfg.light_stop_signal > 0 {
            return self.values.int(signal) as i64 >= self.cfg.light_stop_signal;
        }
        self.values.flag(flag)
    }

    /// Plan the bump reflex: reverse, then turn away from the side that was hit.
    fn start_backup(&mut self, side: i32, now: f64) {
        let c = &self.cfg;
        let v = (c.backup_speed * 1000.0).round_ties_even() as i64;
        let reverse_s = if c.backup_speed != 0.0 { c.backup_distance / c.backup_speed } else { 0.0 };
        let turn_s = if c.backup_speed != 0.0 { c.backup_turn * c.wheel_base / 2.0 / c.backup_speed } else { 0.0 };
        // Hit on the left -> turn right (clockwise): left wheel forward, right wheel back.
        let turn = if side > 0 { (v, -v) } else { (-v, v) };
        let (reverse_s, turn_s) = (reverse_s.min(1.5), turn_s.min(1.0));
        self.backup_start = now;
        self.backup = Some(VecDeque::from([
            (-v, -v, ticks(reverse_s)),
            (turn.0, turn.1, ticks(reverse_s + turn_s)),
        ]));
    }

    /// Advance the bump reflex; returns true while it is in control of the wheels.
    fn run_backup(&mut self, now: f64, link: &dyn Link, out: &dyn Out) -> bool {
        if self.backup_request != 0 && self.backup.is_none() {
            let side = std::mem::take(&mut self.backup_request);
            if is_control(self.mode) {
                self.start_backup(side, now);
            }
        }
        // Steps end on a count of supervisor ticks since the start, not on a clock comparison.
        let tick = ((now - self.backup_start) / SUPERVISE_PERIOD_S).round_ties_even() as i64;
        let Some(steps) = self.backup.as_mut() else { return false };
        if !is_control(self.mode) {
            // cliff / wheel drop: the firmware already stopped it
            self.backup = None;
            return false;
        }
        while steps.front().is_some_and(|s| tick >= s.2) {
            steps.pop_front();
        }
        let Some(&(left, right, _)) = steps.front() else {
            self.backup = None;
            let r = link.write(&oi::drive_direct(0.0, 0.0));
            if self.try_result(r, "end of bump backup", now, out) {
                self.stopped();
            }
            out.log(Level::Info, "bump backup done");
            return false;
        };
        let r = link.write(&oi::drive_direct(left as f64, right as f64));
        if self.try_result(r, "bump backup", now, out) {
            self.driving = true;
            self.driving_forward = false;
            self.last_drive_time = now;
        }
        true
    }

    /// Contact loss, mode changes, initial mode, and the drive command watchdog (every 50 ms).
    pub fn supervise(&mut self, now: f64, link: &dyn Link, out: &dyn Out) {
        if self.run_backup(now, link, out) {
            return;
        }
        let lost = self.mode.is_some() && now - self.last_frame_mono > STALE_AFTER_S;
        if lost {
            self.mode = None;
            self.mode_event = None;
        }
        // (A drop seen in a secondary frame waits for the next primary frame: see on_frame.)
        let waiting = matches!(self.mode_event, Some((.., Some(since))) if now - since < CHARGER_WAIT_S);
        let event = if waiting { None } else { self.mode_event.take() };
        let have_data = self.mode.is_some();
        if lost {
            let r = link.write(&oi::drive_direct(0.0, 0.0));
            if self.try_result(r, "stop after losing contact", now, out) {
                self.stopped();
            }
            self.contact_lost = true;
            out.publish_mode("unknown");
            out.log(
                Level::Warn,
                "lost contact with the Roomba (asleep? In passive mode it sleeps after 5 minutes idle; \
                 press Clean to wake it, or wire DTR to pin 5 and set wake_with_dtr)",
            );
        }
        if let Some((previous, mode, values, _)) = event {
            self.report_mode_change(previous, mode, &values, now, link, out);
        }
        if have_data && self.apply_initial_mode {
            self.apply_initial_mode = false;
            if let Some(desired) = self.desired_mode.filter(|&d| is_control(Some(d as i64))) {
                let r = link.request_mode(desired);
                if self.try_result(r, "set initial mode", now, out) {
                    self.check_mode_at = Some(now + 1.0);
                }
            }
        }
        if self.check_mode_at.is_some_and(|t| now > t) {
            self.check_mode_at = None;
            if let Some(desired) = self.desired_mode.filter(|&d| is_control(Some(d as i64))) {
                if self.mode != Some(desired as i64) {
                    let b = blockers(&link.latest());
                    out.log(
                        Level::Warn,
                        &format!(
                            "robot did not enter {} mode ({}); call roomba/set_mode once it is on the floor and off the dock",
                            mode_str(Some(desired as i64)),
                            if b.is_empty() { "no reason reported" } else { &b }
                        ),
                    );
                }
            }
        }
        if self.driving && now - self.last_drive_time > self.cfg.cmd_timeout {
            // Keep retrying every cycle until a stop actually reaches the robot.
            let r = link.write(&oi::drive_direct(0.0, 0.0));
            if self.try_result(r, "stop after command timeout", now, out) {
                self.stopped();
                out.log(Level::Info, &format!("no drive command for {:.1} s; stopped", self.cfg.cmd_timeout));
            }
        }
    }

    fn report_mode_change(&mut self, previous: Option<i64>, mode: i64, values: &SensorValues, now: f64, link: &dyn Link, out: &dyn Out) {
        let name = mode_str(Some(mode));
        out.publish_mode(&name);
        let Some(previous) = previous else {
            if self.contact_lost {
                self.contact_lost = false;
                out.log(Level::Info, &format!("contact with the Roomba restored (mode {name})"));
                self.apply_initial_mode = true;
            }
            if mode == oi::MODE_PASSIVE as i64 && !link.wake_with_dtr() && !self.desired_is_control() {
                out.log(Level::Info, "in passive mode the Roomba sleeps after 5 minutes idle");
            }
            return;
        };
        let mut reason = String::new();
        if is_control(Some(previous)) && mode == oi::MODE_PASSIVE as i64 {
            let b = blockers(values);
            if !b.is_empty() {
                reason = format!(" ({b})");
            }
            self.stopped();
            if b.is_empty() && self.desired_is_control() {
                // Nothing unsafe happened (no cliff, wheel drop or charger): the OI was restarted
                // (e.g. a stream restart or a robot reset). Go back to the requested mode.
                let desired = self.desired_mode.unwrap_or(oi::MODE_SAFE);
                reason = format!(" (no safety reason; re-entering {})", mode_str(Some(desired as i64)));
                let r = link.request_mode(desired);
                self.try_result(r, "re-enter mode", now, out);
            }
        }
        out.log(Level::Info, &format!("OI mode {} -> {name}{reason}", mode_str(Some(previous))));
    }

    /// Keep-awake timer: whether the robot is in passive mode and should get a wake pulse.
    pub fn wants_keep_awake(&self) -> bool {
        self.mode == Some(oi::MODE_PASSIVE as i64)
    }

    // ------------------------------------------------------------ helpers
    /// node.py `_try`: true on success; otherwise a warning (throttled: one call site in Python).
    pub fn try_result(&mut self, result: Result<(), String>, what: &str, now: f64, out: &dyn Out) -> bool {
        match result {
            Ok(()) => true,
            Err(e) => {
                if self.throttle.allow(Site::TryFailed, 2.0, now) {
                    out.log(Level::Warn, &format!("{what} failed: {e}"));
                }
                false
            }
        }
    }

    /// True if the robot is in Safe/Full mode; otherwise warns (and may re-enter the mode).
    fn in_control(&mut self, what: &str, now: f64, link: &dyn Link, out: &dyn Out) -> bool {
        let Some(mode) = self.mode else {
            if self.throttle.allow(Site::NoData, 5.0, now) {
                out.log(Level::Warn, &format!("ignoring {what}: no sensor data from the Roomba yet"));
            }
            return false;
        };
        if is_control(Some(mode)) {
            return true;
        }
        if self.cfg.auto_reenter_mode && self.desired_is_control() && now - self.last_reenter > 1.0 {
            self.last_reenter = now;
            let desired = self.desired_mode.unwrap_or(oi::MODE_SAFE);
            out.log(Level::Warn, &format!("re-entering {} mode for {what}", mode_str(Some(desired as i64))));
            let r = link.request_mode(desired);
            self.try_result(r, "re-enter mode", now, out);
        } else if !self.cfg.auto_reenter_mode && self.throttle.allow(Site::WrongMode, 5.0, now) {
            out.log(
                Level::Warn,
                &format!(
                    "ignoring {what}: Roomba is in {} mode. Call roomba/set_mode with mode 2 (safe) or 3 (full).",
                    mode_str(Some(mode))
                ),
            );
        }
        false
    }

    #[allow(clippy::too_many_arguments)]
    fn drive(&mut self, cmd: Vec<u8>, moving: bool, what: &str, forward: bool, reverse: bool, now: f64, link: &dyn Link, out: &dyn Out) {
        if self.backup.is_some() || self.backup_request != 0 {
            if self.throttle.allow(Site::DuringBackup, 2.0, now) {
                out.log(Level::Info, &format!("ignoring {what} during the bump backup"));
            }
            return;
        }
        if !moving {
            // Stopping is always allowed, even if the mode is unknown.
            if link.is_open() {
                let r = link.write(&cmd);
                if self.try_result(r, what, now, out) {
                    self.stopped();
                }
            }
            return;
        }
        if self.cfg.bump_stop && self.bumped && !reverse {
            // Pressed against something: turning in place there can climb it (it once drove up a
            // railing's base rail while turning), so any motion except straight back backs off first.
            if self.request_backup(now) {
                out.log(Level::Warn, &format!("{what} while the bumper is pressed: backing up"));
                return;
            }
        }
        if forward && self.cfg.bump_stop && self.bumped {
            if self.throttle.allow(Site::ForwardBumped, 2.0, now) {
                out.log(Level::Warn, &format!("ignoring forward {what}: bumper pressed (reverse or turn to get free)"));
            }
            if self.driving_forward {
                let r = link.write(&oi::drive_direct(0.0, 0.0));
                if self.try_result(r, "stop on bump", now, out) {
                    self.stopped();
                }
            }
            return;
        }
        if !self.in_control(what, now, link, out) {
            return;
        }
        let r = link.write(&cmd);
        if self.try_result(r, what, now, out) {
            self.driving = true;
            self.driving_forward = forward;
            self.last_drive_time = now;
        }
    }

    // ------------------------------------------------------------ drive topics
    pub fn on_cmd_vel(&mut self, linear: f64, angular: f64, now: f64, link: &dyn Link, out: &dyn Out) {
        let (left, right) = twist_to_wheel_speeds(linear, angular, self.cfg.wheel_base, self.cfg.max_wheel_speed);
        if !(left.is_finite() && right.is_finite()) {
            // (The Python node raises here and dies.)
            if self.throttle.allow(Site::NotFinite, 2.0, now) {
                out.log(Level::Warn, "ignoring cmd_vel: not a finite velocity");
            }
            return;
        }
        let (l, r) = ((left * 1000.0).round_ties_even(), (right * 1000.0).round_ties_even());
        self.drive(oi::drive_direct(l, r), l != 0.0 || r != 0.0, "cmd_vel", l + r > 0.0, l < 0.0 && r < 0.0, now, link, out);
    }

    pub fn on_drive(&mut self, velocity_mm_s: i16, radius_mm: i16, now: f64, link: &dyn Link, out: &dyn Out) {
        let limit = self.cfg.max_wheel_speed * 1000.0;
        let velocity = (-limit).max(limit.min(velocity_mm_s as f64));
        let spin = radius_mm as i64 == oi::RADIUS_TURN_CW || radius_mm as i64 == oi::RADIUS_TURN_CCW;
        self.drive(oi::drive(velocity, radius_mm as i64), velocity != 0.0, "roomba/drive",
                   velocity > 0.0 && !spin, velocity < 0.0 && !spin, now, link, out);
    }

    pub fn on_drive_direct(&mut self, left_mm_s: i16, right_mm_s: i16, now: f64, link: &dyn Link, out: &dyn Out) {
        let limit = self.cfg.max_wheel_speed * 1000.0;
        let left = (-limit).max(limit.min(left_mm_s as f64));
        let right = (-limit).max(limit.min(right_mm_s as f64));
        self.drive(oi::drive_direct(left, right), left != 0.0 || right != 0.0, "roomba/drive_direct",
                   left + right > 0.0, left < 0.0 && right < 0.0, now, link, out);
    }

    pub fn on_drive_pwm(&mut self, left_pwm: i16, right_pwm: i16, now: f64, link: &dyn Link, out: &dyn Out) {
        self.drive(oi::drive_pwm(left_pwm as f64, right_pwm as f64), left_pwm != 0 || right_pwm != 0, "roomba/drive_pwm",
                   left_pwm as i32 + right_pwm as i32 > 0, false, now, link, out);
    }

    // ------------------------------------------------------------ other actuator topics
    #[allow(clippy::too_many_arguments)]
    pub fn on_cleaning_motors(&mut self, use_pwm: bool, main_brush: bool, main_brush_outward: bool, side_brush: bool,
                              side_brush_clockwise: bool, vacuum: bool, pwm: (i8, i8, u8), now: f64, link: &dyn Link, out: &dyn Out) {
        if !self.in_control("roomba/cleaning_motors", now, link, out) {
            return;
        }
        let cmd = if use_pwm {
            oi::pwm_motors(pwm.0 as f64, pwm.1 as f64, pwm.2 as f64)
        } else {
            oi::motors(side_brush, vacuum, main_brush, side_brush_clockwise, main_brush_outward)
        };
        let r = link.write(&cmd);
        self.try_result(r, "cleaning motors", now, out);
    }

    #[allow(clippy::too_many_arguments)]
    pub fn on_leds(&mut self, debris: bool, spot: bool, dock: bool, check_robot: bool, power_color: u8, power_intensity: u8,
                   now: f64, link: &dyn Link, out: &dyn Out) {
        if self.in_control("roomba/leds", now, link, out) {
            let r = link.write(&oi::leds(debris, spot, dock, check_robot, power_color as f64, power_intensity as f64));
            self.try_result(r, "LEDs", now, out);
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn on_scheduling_leds(&mut self, weekdays: u8, schedule: bool, clock: bool, am: bool, pm: bool, colon: bool,
                              now: f64, link: &dyn Link, out: &dyn Out) {
        if self.in_control("roomba/scheduling_leds", now, link, out) {
            let r = link.write(&oi::scheduling_leds(weekdays as i64, schedule, clock, am, pm, colon));
            self.try_result(r, "scheduling LEDs", now, out);
        }
    }

    pub fn on_display(&mut self, text: &str, now: f64, link: &dyn Link, out: &dyn Out) {
        if !self.in_control("roomba/display", now, link, out) {
            return;
        }
        let chars: Vec<char> = text.chars().collect();
        if chars.len() <= 4 {
            self.scroll = None;
            let r = link.write(&oi::digit_leds_ascii(text));
            self.try_result(r, "display", now, out);
        } else {
            let padded = "    ".chars().chain(chars).chain("    ".chars()).collect();
            self.scroll = Some((padded, 0));
        }
    }

    /// Scroll timer: show the next 4 characters of a long display text.
    pub fn scroll_display(&mut self, now: f64, link: &dyn Link, out: &dyn Out) {
        if !is_control(self.mode) {
            return;
        }
        let Some((text, i)) = self.scroll.as_ref() else { return };
        let (shown, next): (String, usize) = (text[*i..(*i + 4).min(text.len())].iter().collect(), (*i + 1) % (text.len() - 3));
        let r = link.write(&oi::digit_leds_ascii(&shown));
        self.try_result(r, "display", now, out);
        if let Some(s) = self.scroll.as_mut() {
            s.1 = next;
        }
    }

    pub fn on_display_raw(&mut self, data: &[u8], now: f64, link: &dyn Link, out: &dyn Out) {
        if data.len() != 4 {
            out.log(Level::Warn, "roomba/display_raw needs exactly 4 bytes (leftmost digit first)");
            return;
        }
        if self.in_control("roomba/display_raw", now, link, out) {
            self.scroll = None;
            let segments: Vec<i64> = data.iter().map(|&b| b as i64).collect();
            let r = oi::digit_leds_raw(&segments).and_then(|cmd| link.write(&cmd));
            self.try_result(r, "display", now, out);
        }
    }

    // ------------------------------------------------------------ services
    /// node.py `_service_result`: (success, message).
    pub fn service_result(&self, action: impl FnOnce() -> Result<(), String>, what: &str, need_control: bool) -> (bool, String) {
        if need_control && !is_control(self.mode) {
            return (false, format!("{what} needs Safe or Full mode; robot is in {} mode", mode_str(self.mode)));
        }
        result_message(action(), what)
    }

    /// clean / spot / max_clean / seek_dock / power_off: the robot leaves the requested mode.
    pub fn behaviour(&mut self, name: &str, opcode: u8, link: &dyn Link) -> (bool, String) {
        self.desired_mode = None;
        self.scroll = None;
        self.service_result(|| link.write(&[opcode]), name, false)
    }

    pub fn reset(&mut self, link: &dyn Link) -> (bool, String) {
        self.desired_mode = None;
        self.service_result(|| link.write(&[oi::RESET]), "reset (robot is rebooting)", false)
    }

    pub fn define_song(&self, number: u8, notes: &[u8], durations: &[u8], play: bool, link: &dyn Link) -> (bool, String) {
        if notes.len() != durations.len() {
            return (false, "notes and durations must be the same length".into());
        }
        let pairs: Vec<(i64, i64)> = notes.iter().zip(durations).map(|(&n, &d)| (n as i64, d as i64)).collect();
        let mut result = self.service_result(|| oi::song(number as i64, &pairs).and_then(|c| link.write(&c)),
                                             &format!("song {number}"), false);
        if result.0 && play {
            result = self.service_result(|| oi::play(number as i64).and_then(|c| link.write(&c)),
                                         &format!("song {number} defined and played"), true);
        }
        result
    }

    pub fn play_song(&self, number: u8, link: &dyn Link) -> (bool, String) {
        self.service_result(|| oi::play(number as i64).and_then(|c| link.write(&c)), &format!("play song {number}"), true)
    }

    pub fn press_buttons(&self, mask: u8, link: &dyn Link) -> (bool, String) {
        self.service_result(|| oi::buttons(mask as i64).and_then(|c| link.write(&c)), "button press", false)
    }

    pub fn set_schedule(&self, days: u8, hours: &[u8], minutes: &[u8], link: &dyn Link) -> (bool, String) {
        let times: Vec<(i64, i64)> = hours.iter().zip(minutes).map(|(&h, &m)| (h as i64, m as i64)).collect();
        self.service_result(|| oi::schedule(days as i64, &times).and_then(|c| link.write(&c)), "schedule", false)
    }

    pub fn set_day_time(&self, day: i64, hour: i64, minute: i64, link: &dyn Link) -> (bool, String) {
        self.service_result(|| oi::set_day_time(day, hour, minute).and_then(|c| link.write(&c)),
                            &format!("clock set to day {day} {hour:02}:{minute:02}"), false)
    }

    // ------------------------------------------------------------ publishers
    /// Odometry since the last call (None if no new encoder update).
    pub fn take_odometry(&mut self) -> Option<OdomSnapshot> {
        let stamp_ns = self.odom_stamp.take()?;
        let o = &self.odom;
        Some(OdomSnapshot {
            stamp_ns,
            x: o.x,
            y: o.y,
            theta: o.theta,
            linear: o.linear,
            angular: o.angular,
            positions: [o.left_pos, o.right_pos],
            velocities: [o.left_vel, o.right_vel],
        })
    }

    /// The merged sensor values and the stamp of the latest frame; never stale data.
    pub fn sensor_snapshot(&self) -> Option<(SensorValues, i64)> {
        self.mode?;
        Some((self.values, self.latest_stamp?))
    }

    /// Battery message fields and stamp (also advances the voltage gauge's filter).
    pub fn battery_state(&mut self) -> Option<(BatteryFields, i64)> {
        let (v, stamp) = self.sensor_snapshot()?;
        // The primary layout carries the battery packets; nothing to report before it arrived.
        v.get(F::VoltageMv)?;
        let reading = Reading {
            voltage_mv: v.int(F::VoltageMv),
            current_ma: v.int(F::CurrentMa),
            temperature_c: v.int(F::TemperatureC),
            charge_mah: v.int(F::ChargeMah),
            capacity_mah: v.int(F::CapacityMah),
            charging_state: v.int(F::ChargingState),
            on_charger: v.flag(F::HomeBaseAvailable) || v.flag(F::InternalChargerAvailable),
        };
        Some((self.battery.update(&reading), stamp))
    }
}

pub fn result_message(result: Result<(), String>, what: &str) -> (bool, String) {
    match result {
        Ok(()) => (true, format!("{what} sent")),
        Err(e) => (false, e),
    }
}

/// roomba/set_mode. Blocks (waiting for the robot), so the core is only locked briefly.
/// Returns (success, message, mode reported by the robot).
pub fn set_mode_service(core: &Mutex<Core>, link: &dyn Link, out: &dyn Out, mode: u8, now: impl Fn() -> f64) -> (bool, String, u8) {
    let current = lock(core).mode;
    if mode_name(mode as i64).is_none() {
        return (false, format!("unknown mode {mode}"), current.unwrap_or(0) as u8);
    }
    if current.is_none() && mode != oi::MODE_OFF {
        if link.streaming() && link.wake_with_dtr() {
            let r = link.wake();
            lock(core).try_result(r, "wake", now(), out);
        }
        if !link.wait_for_frame(2.0) {
            return (false, "no sensor data from the Roomba (asleep? press Clean to wake it)".into(), 0);
        }
    }
    lock(core).desired_mode = if is_control(Some(mode as i64)) { Some(mode) } else { None };
    let reported = match link.set_mode(mode, 1.0) {
        Ok(r) => r.unwrap_or(0) as i64,
        Err(e) => return (false, e, 0),
    };
    let got = mode_str(Some(reported));
    if reported == mode as i64 {
        return (true, format!("now in {got} mode"), reported as u8);
    }
    let b = blockers(&link.latest());
    let hint = if b.is_empty() { String::new() } else { format!("; active: {b}") };
    (false, format!("asked for {}, robot reports {got}{hint}", mode_str(Some(mode as i64))), reported as u8)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    #[derive(Default)]
    struct FakeLink {
        writes: RefCell<Vec<Vec<u8>>>,
        modes: RefCell<Vec<u8>>,
        fail: RefCell<Option<String>>,
        latest: SensorValues,
    }

    impl Link for FakeLink {
        fn write(&self, data: &[u8]) -> Result<(), String> {
            if let Some(e) = self.fail.borrow().clone() {
                return Err(e);
            }
            self.writes.borrow_mut().push(data.to_vec());
            Ok(())
        }
        fn is_open(&self) -> bool {
            true
        }
        fn streaming(&self) -> bool {
            true
        }
        fn wake_with_dtr(&self) -> bool {
            false
        }
        fn wake(&self) -> Result<(), String> {
            Ok(())
        }
        fn request_mode(&self, mode: u8) -> Result<(), String> {
            self.modes.borrow_mut().push(mode);
            Ok(())
        }
        fn set_mode(&self, mode: u8, _timeout: f64) -> Result<Option<i32>, String> {
            self.modes.borrow_mut().push(mode);
            Ok(Some(1))
        }
        fn wait_for_frame(&self, _timeout: f64) -> bool {
            false
        }
        fn latest(&self) -> SensorValues {
            self.latest
        }
    }

    #[derive(Default)]
    struct Log {
        lines: RefCell<Vec<(Level, String)>>,
        modes: RefCell<Vec<String>>,
    }

    impl Out for Log {
        fn log(&self, level: Level, msg: &str) {
            self.lines.borrow_mut().push((level, msg.to_string()));
        }
        fn publish_mode(&self, mode: &str) {
            self.modes.borrow_mut().push(mode.to_string());
        }
    }

    impl Log {
        fn take(&self) -> Vec<String> {
            self.lines.borrow_mut().drain(..).map(|l| l.1).collect()
        }
    }

    fn frame(mode: i32, enc: (i32, i32), set: &[(F, i32)]) -> SensorValues {
        let mut v = SensorValues::default();
        v.set(F::OiMode, mode);
        v.set(F::LeftEncoderCounts, enc.0);
        v.set(F::RightEncoderCounts, enc.1);
        for &(f, x) in set {
            v.set(f, x);
        }
        v
    }

    fn ns(t: f64) -> i64 {
        (t * 1e9) as i64
    }

    /// A frame of one of the driver's stream layouts: zeros, the mode, and values it carries.
    fn layout_frame(layout: &[u8], mode: i32, set: &[(F, i32)]) -> SensorValues {
        let mut v = SensorValues::default();
        for &pid in layout {
            v.update(&oi::decode(pid, &vec![0; oi::packet_size(pid).unwrap()]).unwrap());
        }
        v.set(F::OiMode, mode);
        for &(f, x) in set {
            assert!(v.has(f), "{} is not in this layout", f.name());
            v.set(f, x);
        }
        v
    }

    fn primary(mode: i32, set: &[(F, i32)]) -> SensorValues {
        layout_frame(&oi::STREAM_PRIMARY, mode, set)
    }

    /// A safe-mode frame just before a supervisor cycle, so contact isn't lost.
    fn tick(c: &mut Core, now: f64, link: &FakeLink, log: &Log) {
        c.on_frame(&frame(2, (0, 0), &[]), ns(now - 0.001), now - 0.001, link, log);
        c.supervise(now, link, log);
    }

    const STOP: [u8; 5] = [145, 0, 0, 0, 0];

    /// A core that has seen safe-mode frames, with the startup mode handling done (t = 11.1).
    fn running(cfg: Config) -> (Core, FakeLink, Log) {
        let (mut c, link, log) = (Core::new(cfg), FakeLink::default(), Log::default());
        c.on_frame(&frame(1, (0, 0), &[]), ns(10.0), 10.0, &link, &log);
        c.supervise(10.01, &link, &log);
        assert_eq!(*link.modes.borrow(), [2]); // initial mode requested
        c.on_frame(&frame(2, (0, 0), &[]), ns(10.02), 10.02, &link, &log);
        c.supervise(10.03, &link, &log);
        tick(&mut c, 11.1, &link, &log); // mode confirmed: no warning
        assert_eq!(log.take(), ["OI mode passive -> safe"]);
        assert_eq!(*log.modes.borrow(), ["passive", "safe"]);
        link.modes.borrow_mut().clear();
        (c, link, log)
    }

    fn robot_cfg() -> Config {
        Config {
            max_wheel_speed: 0.4,
            bump_stop_light: true,
            bump_backup: true,
            light_stop_signal: 200,
            ignore_sensors: vec![F::BumpRight],
            ..Config::default()
        }
    }

    #[test]
    fn cmd_vel_drives_and_times_out() {
        let (mut c, link, log) = running(Config::default());
        c.on_cmd_vel(0.2, 0.0, 12.0, &link, &log);
        c.on_cmd_vel(0.0, 1.0, 12.1, &link, &log);
        assert_eq!(*link.writes.borrow(), [oi::drive_direct(200.0, 200.0), oi::drive_direct(-118.0, 118.0)]);
        tick(&mut c, 12.5, &link, &log);
        assert!(c.driving);
        tick(&mut c, 12.65, &link, &log);
        assert!(!c.driving);
        assert_eq!(link.writes.borrow().last().unwrap(), &STOP);
        assert_eq!(log.take(), ["no drive command for 0.5 s; stopped"]);
    }

    #[test]
    fn commands_are_refused_outside_control_modes() {
        let (mut c, link, log) = (Core::new(Config::default()), FakeLink::default(), Log::default());
        c.on_cmd_vel(0.2, 0.0, 1.0, &link, &log);
        c.on_cmd_vel(0.2, 0.0, 1.1, &link, &log); // throttled
        c.on_cmd_vel(0.0, 0.0, 1.2, &link, &log); // stopping is always allowed
        assert_eq!(log.take(), ["ignoring cmd_vel: no sensor data from the Roomba yet"]);
        assert_eq!(*link.writes.borrow(), [STOP]);
        c.on_frame(&frame(1, (0, 0), &[]), 0, 2.0, &link, &log);
        c.on_leds(true, false, false, false, 0, 255, 2.0, &link, &log);
        assert_eq!(log.take(), ["ignoring roomba/leds: Roomba is in passive mode. Call roomba/set_mode with mode 2 (safe) or 3 (full)."]);
    }

    #[test]
    fn auto_reenter_requests_the_mode_once_a_second() {
        let cfg = Config { auto_reenter_mode: true, ..Config::default() };
        let (mut c, link, log) = (Core::new(cfg), FakeLink::default(), Log::default());
        c.on_frame(&frame(1, (0, 0), &[]), 0, 2.0, &link, &log);
        c.on_cmd_vel(0.1, 0.0, 2.0, &link, &log);
        c.on_cmd_vel(0.1, 0.0, 2.5, &link, &log);
        c.on_cmd_vel(0.1, 0.0, 3.1, &link, &log);
        assert_eq!(*link.modes.borrow(), [2, 2]);
        assert_eq!(log.take(), ["re-entering safe mode for cmd_vel", "re-entering safe mode for cmd_vel"]);
    }

    #[test]
    fn bump_while_driving_forward_stops_and_backs_up_away_from_the_hit() {
        let (mut c, link, log) = running(robot_cfg());
        c.on_cmd_vel(0.2, 0.0, 12.0, &link, &log);
        // The front-left light bumper reaches the stop signal: a hit on the left.
        c.on_frame(&frame(2, (0, 0), &[(F::LightBumpFrontLeftSignal, 230)]), ns(12.1), 12.1, &link, &log);
        assert_eq!(log.take(), ["bumper hit: backing up"]);
        assert_eq!(link.writes.borrow().last().unwrap(), &STOP);
        c.on_cmd_vel(0.2, 0.0, 12.12, &link, &log);
        assert_eq!(log.take(), ["ignoring cmd_vel during the bump backup"]);
        link.writes.borrow_mut().clear();
        tick(&mut c, 12.15, &link, &log); // reverse 0.8 s
        tick(&mut c, 13.0, &link, &log); // turn right 0.47 s
        tick(&mut c, 13.43, &link, &log);
        assert_eq!(log.take(), ["bump backup done"]);
        assert_eq!(*link.writes.borrow(), [oi::drive_direct(-100.0, -100.0), oi::drive_direct(100.0, -100.0), STOP.to_vec()]);
        assert!(!c.driving);
    }

    #[test]
    fn ignored_sensor_reads_as_off() {
        let (mut c, link, log) = running(robot_cfg());
        c.on_cmd_vel(0.2, 0.0, 12.0, &link, &log);
        c.on_frame(&frame(2, (0, 0), &[(F::BumpRight, 1)]), ns(12.1), 12.1, &link, &log);
        assert!(!c.bumped);
        assert_eq!(c.sensor_snapshot().unwrap().0.get(F::BumpRight), Some(0));
        assert!(log.take().is_empty());
    }

    #[test]
    fn pressed_bumper_backs_off_before_any_motion_and_rate_limits() {
        let (mut c, link, log) = running(Config { bump_backup: true, ..Config::default() });
        c.on_frame(&frame(2, (0, 0), &[(F::BumpRight, 1)]), ns(12.0), 12.0, &link, &log);
        c.on_cmd_vel(0.0, 1.0, 12.1, &link, &log);
        assert_eq!(log.take(), ["cmd_vel while the bumper is pressed: backing up"]);
        tick(&mut c, 12.15, &link, &log);
        assert_eq!(link.writes.borrow().last().unwrap(), &oi::drive_direct(-100.0, -100.0));
        tick(&mut c, 13.5, &link, &log);
        assert_eq!(log.take(), ["bump backup done"]);
        // Within 3 s of the last reflex: forward is refused, turning is allowed.
        c.on_cmd_vel(0.1, 0.0, 14.0, &link, &log);
        assert_eq!(log.take(), ["ignoring forward cmd_vel: bumper pressed (reverse or turn to get free)"]);
        c.on_cmd_vel(0.0, 1.0, 14.1, &link, &log);
        assert_eq!(link.writes.borrow().last().unwrap(), &oi::drive_direct(-118.0, 118.0));
        // Reversing never triggers the reflex.
        c.on_cmd_vel(-0.1, 0.0, 16.0, &link, &log);
        assert_eq!(link.writes.borrow().last().unwrap(), &oi::drive_direct(-100.0, -100.0));
        assert!(log.take().is_empty());
    }

    #[test]
    fn contact_loss_and_restore() {
        let (mut c, link, log) = running(Config::default());
        c.supervise(12.2, &link, &log);
        assert_eq!(c.mode, None);
        assert_eq!(link.writes.borrow().last().unwrap(), &STOP);
        assert!(log.take()[0].starts_with("lost contact with the Roomba (asleep?"));
        assert_eq!(log.modes.borrow().last().unwrap(), "unknown");
        assert!(c.sensor_snapshot().is_none());
        c.on_frame(&frame(1, (0, 0), &[]), ns(15.0), 15.0, &link, &log);
        c.supervise(15.01, &link, &log);
        assert_eq!(log.take(), ["contact with the Roomba restored (mode passive)"]);
        assert_eq!(*link.modes.borrow(), [2]); // initial mode again
        c.on_frame(&frame(1, (0, 0), &[]), ns(16.0), 16.0, &link, &log);
        c.supervise(16.1, &link, &log);
        assert_eq!(log.take(), ["robot did not enter safe mode (no reason reported); call roomba/set_mode once it is on the floor and off the dock"]);
    }

    #[test]
    fn safety_drop_is_reported_and_not_undone() {
        let (mut c, link, log) = running(Config::default());
        c.on_frame(&frame(1, (0, 0), &[(F::CliffFrontRight, 1)]), ns(12.0), 12.0, &link, &log);
        c.supervise(12.01, &link, &log);
        assert_eq!(log.take(), ["OI mode safe -> passive (cliff_front_right)"]);
        assert!(link.modes.borrow().is_empty());
        // Without a safety reason (OI restarted) the requested mode is re-entered.
        c.on_frame(&frame(2, (0, 0), &[(F::CliffFrontRight, 0)]), ns(12.1), 12.1, &link, &log);
        c.on_frame(&primary(1, &[]), ns(12.2), 12.2, &link, &log);
        c.supervise(12.21, &link, &log);
        assert_eq!(log.take(), ["OI mode safe -> passive (no safety reason; re-entering safe)"]);
        assert_eq!(*link.modes.borrow(), [2]);
    }

    #[test]
    fn light_bumper_stop_is_checked_on_every_primary_frame() {
        let (mut c, link, log) = running(robot_cfg());
        c.on_cmd_vel(0.25, 0.0, 12.0, &link, &log);
        c.on_frame(&primary(2, &[(F::LightBumpFrontLeftSignal, 230)]), ns(12.015), 12.015, &link, &log);
        assert_eq!(link.writes.borrow().last().unwrap(), &STOP);
        assert_eq!(log.take(), ["bumper hit: backing up"]);
    }

    #[test]
    fn drop_to_passive_first_seen_in_a_secondary_frame_waits_for_the_charger() {
        for name in ["v2", "legacy"] {
            let (prim, sec) = oi::stream_layouts(name).unwrap();
            let primary = |mode: i32, set: &[(F, i32)]| layout_frame(prim, mode, set);
            let secondary = |mode: i32, set: &[(F, i32)]| layout_frame(sec, mode, set);
            let (mut c, link, log) = running(Config::default());
            // Docked: the drop and the charger come together, but the first frame showing the drop is a
            // secondary one, without the charger packet.
            c.on_frame(&secondary(1, &[]), ns(12.0), 12.0, &link, &log);
            c.supervise(12.01, &link, &log);
            assert!(log.take().is_empty());
            c.on_frame(&primary(1, &[(F::HomeBaseAvailable, 1)]), ns(12.015), 12.015, &link, &log);
            c.supervise(12.06, &link, &log);
            assert_eq!(log.take(), ["OI mode safe -> passive (home_base_available)"]);
            assert!(link.modes.borrow().is_empty()); // safe is not asked for on the charger
            // A drop without a reason (the OI restarted) is still undone once a primary frame confirms it.
            tick(&mut c, 12.1, &link, &log);
            c.on_frame(&primary(2, &[]), ns(12.11), 12.11, &link, &log);
            c.on_frame(&secondary(1, &[]), ns(12.2), 12.2, &link, &log);
            c.supervise(12.21, &link, &log);
            assert_eq!(log.take(), ["OI mode passive -> safe"]);
            c.on_frame(&primary(1, &[]), ns(12.215), 12.215, &link, &log);
            c.supervise(12.26, &link, &log);
            assert_eq!(log.take(), ["OI mode safe -> passive (no safety reason; re-entering safe)"]);
            assert_eq!(*link.modes.borrow(), [2]);
            // Without any primary frame the drop is judged on what is known after CHARGER_WAIT_S.
            c.on_frame(&secondary(2, &[]), ns(12.3), 12.3, &link, &log);
            c.supervise(12.31, &link, &log);
            c.on_frame(&secondary(1, &[]), ns(12.4), 12.4, &link, &log);
            c.supervise(12.41, &link, &log);
            c.on_frame(&secondary(1, &[]), ns(13.0), 13.0, &link, &log);
            c.supervise(13.39, &link, &log);
            assert_eq!(log.take(), ["OI mode passive -> safe"]);
            c.supervise(13.41, &link, &log);
            assert_eq!(log.take(), ["OI mode safe -> passive (no safety reason; re-entering safe)"]);
        }
    }

    #[test]
    fn bump_reflex_steps_end_on_supervisor_ticks_whatever_the_timer_jitter() {
        let (mut c, link, log) = running(robot_cfg());
        c.on_cmd_vel(0.2, 0.0, 12.0, &link, &log);
        c.on_frame(&frame(2, (0, 0), &[(F::BumpLeft, 1)]), ns(12.1), 12.1, &link, &log);
        assert_eq!(log.take(), ["bumper hit: backing up"]);
        link.writes.borrow_mut().clear();
        // The reflex reverses 0.8 s = exactly 16 ticks. Its first tick ran 3 ms late, so by the clock
        // the tick 0.8 s later comes 3 ms early and would reverse once more.
        tick(&mut c, 12.153, &link, &log);
        for k in 1..=26 {
            tick(&mut c, 12.15 + k as f64 * 0.05, &link, &log);
        }
        let writes = link.writes.borrow();
        let count = |cmd: Vec<u8>| writes.iter().filter(|w| **w == cmd).count();
        assert_eq!(count(oi::drive_direct(-100.0, -100.0)), 16);
        assert_eq!(count(oi::drive_direct(100.0, -100.0)), 10);
        assert_eq!(writes.last().unwrap(), &STOP);
        assert_eq!(log.take(), ["bump backup done"]);
    }

    #[test]
    fn display_scrolls_long_text() {
        let (mut c, link, log) = running(Config::default());
        c.on_display("HI", 12.0, &link, &log);
        c.on_display("HELLO", 12.0, &link, &log);
        for _ in 0..10 {
            c.scroll_display(12.0, &link, &log);
        }
        let shown: Vec<String> = link.writes.borrow().iter().map(|w| String::from_utf8_lossy(&w[1..]).into_owned()).collect();
        assert_eq!(shown, ["HI  ", "    ", "   H", "  HE", " HEL", "HELL", "ELLO", "LLO ", "LO  ", "O   ", "    "]);
        c.on_display_raw(&[1, 2, 3], 12.0, &link, &log);
        assert_eq!(log.take(), ["roomba/display_raw needs exactly 4 bytes (leftmost digit first)"]);
    }

    #[test]
    fn service_messages() {
        let (mut c, link, log) = running(Config::default());
        assert_eq!(c.define_song(1, &[60, 62], &[16], false, &link), (false, "notes and durations must be the same length".into()));
        assert_eq!(c.define_song(1, &[60, 62], &[16, 16], true, &link), (true, "song 1 defined and played sent".into()));
        assert_eq!(c.define_song(7, &[60], &[16], false, &link), (false, "song number must be 0-4, got 7".into()));
        assert_eq!(c.set_day_time(7, 0, 5, &link), (false, "day must be 0-6, got 7".into()));
        assert_eq!(c.set_day_time(3, 9, 5, &link), (true, "clock set to day 3 09:05 sent".into()));
        assert_eq!(c.behaviour("seek_dock", oi::SEEK_DOCK, &link), (true, "seek_dock sent".into()));
        assert_eq!(c.desired_mode, None);
        c.on_frame(&frame(1, (0, 0), &[]), ns(12.0), 12.0, &link, &log);
        assert_eq!(c.play_song(0, &link), (false, "play song 0 needs Safe or Full mode; robot is in passive mode".into()));
        c.mode = None;
        assert_eq!(c.play_song(0, &link), (false, "play song 0 needs Safe or Full mode; robot is in None mode".into()));
        *link.fail.borrow_mut() = Some("serial port is not open".into());
        assert_eq!(c.press_buttons(1, &link), (false, "serial port is not open".into()));
    }

    #[test]
    fn set_mode_service_reports_blockers() {
        let (c, _, log) = running(Config::default());
        let mut latest = SensorValues::default();
        latest.set(F::WheelDropLeft, 1);
        latest.set(F::HomeBaseAvailable, 1);
        let link = FakeLink { latest, ..Default::default() };
        let core = Mutex::new(c);
        let r = set_mode_service(&core, &link, &log, 2, || 0.0);
        assert_eq!(r, (false, "asked for safe, robot reports passive; active: wheel_drop_left, home_base_available".into(), 1));
        assert_eq!(set_mode_service(&core, &link, &log, 9, || 0.0), (false, "unknown mode 9".into(), 2));
        lock(&core).mode = None;
        assert_eq!(set_mode_service(&core, &link, &log, 2, || 0.0).1, "no sensor data from the Roomba (asleep? press Clean to wake it)");
    }

    #[test]
    fn odometry_and_battery_snapshots() {
        let cfg = Config { gauge: Gauge::voltage(5, 0.0, 0.1, 1.0, 30.0), ..Config::default() };
        let (mut c, link, log) = running(cfg);
        assert!(c.take_odometry().is_some());
        assert!(c.take_odometry().is_none());
        c.on_frame(&frame(2, (100, 100), &[]), ns(12.0), 12.0, &link, &log);
        let o = c.take_odometry().unwrap();
        assert!(o.x > 0.04 && o.stamp_ns == ns(12.0));
        assert!(c.battery_state().is_none()); // no battery packets yet
        c.on_frame(&frame(2, (100, 100), &[(F::VoltageMv, 20000), (F::CurrentMa, -1000)]), ns(12.1), 12.1, &link, &log);
        let (b, _) = c.battery_state().unwrap();
        assert!((b.percentage - crate::battery::li_ion_fraction(20.1 / 5.0)).abs() < 1e-12);
    }
}
