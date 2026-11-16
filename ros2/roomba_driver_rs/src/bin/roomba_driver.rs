//! roomba_driver (Rust): ROS 2 driver for iRobot Roomba 500-series robots over the Open Interface.
//! Drop-in replacement for the Python roomba_driver node (roomba-serial-ros2): same node name,
//! topics, QoS, services, parameters, log lines, reconnect behaviour and shutdown sequence. The
//! protocol and logic live in roomba_driver_rs::{oi, roomba, driver, kinematics, battery}; this file only
//! connects them to ROS.
//!
//! Publishes:  wheel/odom, joint_states, battery_state, roomba/sensors, roomba/mode (latched),
//!             /tf (odom_frame -> base_frame) only with publish_tf
//! Subscribes: cmd_vel (Twist, or TwistStamped with use_stamped_cmd_vel), roomba/drive,
//!             roomba/drive_direct, roomba/drive_pwm, roomba/cleaning_motors, roomba/leds,
//!             roomba/scheduling_leds, roomba/display, roomba/display_raw
//! Services:   roomba/set_mode, clean, spot, max_clean, seek_dock, power_off, reset, wake,
//!             define_song, play_song, press_buttons, set_schedule, set_day_time
//!
//! Threads: the executor (topics, timers), the serial reader (sensor frames, bump stop), one
//! service thread (services run one at a time and may block, like the Python node's service
//! callback group), a connect retry thread while the port can't be opened, and a signal thread:
//! on SIGINT/SIGTERM the executor stops and the robot is stopped and handed back in Passive mode.

use std::any::Any;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{bail, Result};
use rclrs::*;
use ros_env::{builtin_interfaces, geometry_msgs, nav_msgs, roomba_msgs, sensor_msgs, std_msgs, std_srvs, tf2_msgs};

use roomba_driver_rs::battery::Gauge;
use roomba_driver_rs::driver::{self, set_mode_service, Config, Core, Level, Link, OdomSnapshot, Out};
use roomba_driver_rs::oi::{self, mode_str, Field as F, SensorValues};
use roomba_driver_rs::roomba::{lock, mono, tty_opener, Roomba};

fn time_msg(ns: i64) -> builtin_interfaces::msg::Time {
    builtin_interfaces::msg::Time { sec: ns.div_euclid(1_000_000_000) as i32, nanosec: ns.rem_euclid(1_000_000_000) as u32 }
}

fn header(ns: i64, frame: &str) -> std_msgs::msg::Header {
    std_msgs::msg::Header { stamp: time_msg(ns), frame_id: frame.into() }
}

/// Declares the parameters, keeping the handles for the node's lifetime (rclrs undeclares a
/// parameter when its handle is dropped; `ros2 param get/list` should work as for the Python node).
struct Params<'a> {
    node: &'a Node,
    handles: Vec<Box<dyn Any>>,
}

impl Params<'_> {
    fn keep<T: ParameterVariant + 'static>(&mut self, p: MandatoryParameter<T>) -> T {
        let v = p.get();
        self.handles.push(Box::new(p));
        v
    }
    fn f(&mut self, name: &str, d: f64) -> Result<f64> {
        let p = self.node.declare_parameter(name).default(d).mandatory()?;
        Ok(self.keep(p))
    }
    fn i(&mut self, name: &str, d: i64) -> Result<i64> {
        let p = self.node.declare_parameter(name).default(d).mandatory()?;
        Ok(self.keep(p))
    }
    fn b(&mut self, name: &str, d: bool) -> Result<bool> {
        let p = self.node.declare_parameter(name).default(d).mandatory()?;
        Ok(self.keep(p))
    }
    fn s(&mut self, name: &str, d: &str) -> Result<String> {
        let p = self.node.declare_parameter::<Arc<str>>(name).default(d.into()).mandatory()?;
        Ok(self.keep(p).to_string())
    }
    fn strings(&mut self, name: &str, d: &[&str]) -> Result<Vec<String>> {
        let default: Arc<[Arc<str>]> = d.iter().map(|&s| Arc::from(s)).collect();
        let p = self.node.declare_parameter::<Arc<[Arc<str>]>>(name).default(default).mandatory()?;
        Ok(self.keep(p).iter().map(|s| s.to_string()).collect())
    }
}

/// Log lines go to the node's logger; the mode to the latched roomba/mode topic.
struct NodeOut {
    node: Node,
    pub_mode: Publisher<std_msgs::msg::String>,
}

fn log_to(node: &Node, level: Level, msg: &str) {
    match level {
        Level::Info => log!(node.info(), "{msg}"),
        Level::Warn => log!(node.warn(), "{msg}"),
        Level::Error => log!(node.error(), "{msg}"),
    }
}

impl Out for NodeOut {
    fn log(&self, level: Level, msg: &str) {
        log_to(&self.node, level, msg)
    }
    fn publish_mode(&self, mode: &str) {
        self.pub_mode.publish(std_msgs::msg::String { data: mode.into() }).ok();
    }
}

/// Everything a callback needs.
#[derive(Clone)]
struct Ctx {
    core: Arc<Mutex<Core>>,
    robot: Arc<Roomba>,
    out: Arc<NodeOut>,
}

impl Ctx {
    fn with<R>(&self, f: impl FnOnce(&mut Core, &dyn Link, &dyn Out, f64) -> R) -> R {
        let now = mono();
        let mut core = lock(&self.core);
        f(&mut core, &*self.robot, &*self.out, now)
    }
}

type Job = Box<dyn FnOnce() + Send>;

/// A service whose handler runs on the service thread (one request at a time, may block).
fn service<T, H>(node: &Node, name: &str, jobs: &mpsc::Sender<Job>, handler: H) -> Result<Service<T>>
where
    T: rosidl_runtime_rs::Service,
    T::Request: Send + 'static,
    T::Response: Default + Send + 'static,
    H: Fn(T::Request) -> T::Response + Send + Sync + 'static,
{
    let handler = Arc::new(handler);
    let jobs = jobs.clone();
    Ok(node.create_async_service::<T, _>(name, move |request: T::Request| {
        let (tx, rx) = futures::channel::oneshot::channel();
        let handler = Arc::clone(&handler);
        let _ = jobs.send(Box::new(move || {
            let _ = tx.send(handler(request));
        }));
        async move { rx.await.unwrap_or_default() }
    })?)
}

fn trigger(r: (bool, String)) -> std_srvs::srv::Trigger_Response {
    std_srvs::srv::Trigger_Response { success: r.0, message: r.1 }
}

/// node.py `_connect`: open the port, start the OI and the stream. True once that worked.
fn connect(robot: &Arc<Roomba>, node: &Node, (primary, secondary): (&[u8], &[u8])) -> bool {
    if robot.streaming() {
        return true;
    }
    let started = robot.open().and_then(|_| robot.start()).and_then(|_| {
        std::thread::sleep(Duration::from_millis(50));
        robot.start_stream_with(primary, Some(secondary), 0.2) // every sensor between them
    });
    if let Err(e) = started {
        log!(node.warn().throttle(Duration::from_secs(30)), "cannot open {}: {e}; retrying", robot.port);
        robot.close();
        return false;
    }
    if Link::wait_for_frame(&**robot, 2.0) {
        let (v, _) = robot.latest();
        log!(
            node.info(),
            "connected: battery {:.2} V, {}/{} mAh, mode {}",
            v.int(F::VoltageMv) as f64 / 1000.0,
            v.int(F::ChargeMah),
            v.int(F::CapacityMah),
            mode_str(Some(v.int(F::OiMode) as i64))
        );
    } else {
        log!(node.warn(), "port open but no sensor data yet; is the Roomba awake? Will keep retrying.");
    }
    true
}

fn odometry_msg(s: &OdomSnapshot, odom_frame: &str, base_frame: &str) -> nav_msgs::msg::Odometry {
    let mut m = nav_msgs::msg::Odometry::default();
    m.header = header(s.stamp_ns, odom_frame);
    m.child_frame_id = base_frame.into();
    m.pose.pose.position.x = s.x;
    m.pose.pose.position.y = s.y;
    m.pose.pose.orientation = geometry_msgs::msg::Quaternion { x: 0.0, y: 0.0, z: (s.theta / 2.0).sin(), w: (s.theta / 2.0).cos() };
    m.twist.twist.linear.x = s.linear;
    m.twist.twist.angular.z = s.angular;
    let big = 1e6;
    #[rustfmt::skip]
    let pose = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, big, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, big, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, big, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.05];
    #[rustfmt::skip]
    let twist = [0.002, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, big, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, big, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, big, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, big, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.01];
    m.pose.covariance = pose;
    m.twist.covariance = twist;
    m
}

fn sensor_msg(v: &SensorValues, stamp: i64, frame: &str) -> roomba_msgs::msg::SensorState {
    let b = |f| v.flag(f);
    let i = |f| v.int(f);
    let mut m = roomba_msgs::msg::SensorState::default();
    m.header = header(stamp, frame);
    m.bump_left = b(F::BumpLeft);
    m.bump_right = b(F::BumpRight);
    m.wheel_drop_left = b(F::WheelDropLeft);
    m.wheel_drop_right = b(F::WheelDropRight);
    m.wall = b(F::Wall);
    m.cliff_left = b(F::CliffLeft);
    m.cliff_front_left = b(F::CliffFrontLeft);
    m.cliff_front_right = b(F::CliffFrontRight);
    m.cliff_right = b(F::CliffRight);
    m.virtual_wall = b(F::VirtualWall);
    m.overcurrent_left_wheel = b(F::OvercurrentLeftWheel);
    m.overcurrent_right_wheel = b(F::OvercurrentRightWheel);
    m.overcurrent_main_brush = b(F::OvercurrentMainBrush);
    m.overcurrent_side_brush = b(F::OvercurrentSideBrush);
    m.dirt_detect = i(F::DirtDetect) as u8;
    m.ir_omni = i(F::IrOmni) as u8;
    m.ir_left = i(F::IrLeft) as u8;
    m.ir_right = i(F::IrRight) as u8;
    m.button_clean = b(F::ButtonClean);
    m.button_spot = b(F::ButtonSpot);
    m.button_dock = b(F::ButtonDock);
    m.button_minute = b(F::ButtonMinute);
    m.button_hour = b(F::ButtonHour);
    m.button_day = b(F::ButtonDay);
    m.button_schedule = b(F::ButtonSchedule);
    m.button_clock = b(F::ButtonClock);
    m.distance_mm = i(F::DistanceMm) as i16;
    m.angle_deg = i(F::AngleDeg) as i16;
    m.charging_state = i(F::ChargingState) as u8;
    m.voltage_mv = i(F::VoltageMv) as u16;
    m.current_ma = i(F::CurrentMa) as i16;
    m.temperature_c = i(F::TemperatureC) as i8;
    m.charge_mah = i(F::ChargeMah) as u16;
    m.capacity_mah = i(F::CapacityMah) as u16;
    m.wall_signal = i(F::WallSignal) as u16;
    m.cliff_left_signal = i(F::CliffLeftSignal) as u16;
    m.cliff_front_left_signal = i(F::CliffFrontLeftSignal) as u16;
    m.cliff_front_right_signal = i(F::CliffFrontRightSignal) as u16;
    m.cliff_right_signal = i(F::CliffRightSignal) as u16;
    m.internal_charger_available = b(F::InternalChargerAvailable);
    m.home_base_available = b(F::HomeBaseAvailable);
    m.oi_mode = i(F::OiMode) as u8;
    m.song_number = i(F::SongNumber) as u8;
    m.song_playing = b(F::SongPlaying);
    m.stream_packet_count = i(F::StreamPacketCount) as u8;
    m.requested_velocity_mm_s = i(F::RequestedVelocityMmS) as i16;
    m.requested_radius_mm = i(F::RequestedRadiusMm) as i16;
    m.requested_right_velocity_mm_s = i(F::RequestedRightVelocityMmS) as i16;
    m.requested_left_velocity_mm_s = i(F::RequestedLeftVelocityMmS) as i16;
    m.left_encoder_counts = i(F::LeftEncoderCounts) as u16;
    m.right_encoder_counts = i(F::RightEncoderCounts) as u16;
    m.light_bump_left = b(F::LightBumpLeft);
    m.light_bump_front_left = b(F::LightBumpFrontLeft);
    m.light_bump_center_left = b(F::LightBumpCenterLeft);
    m.light_bump_center_right = b(F::LightBumpCenterRight);
    m.light_bump_front_right = b(F::LightBumpFrontRight);
    m.light_bump_right = b(F::LightBumpRight);
    m.light_bump_left_signal = i(F::LightBumpLeftSignal) as u16;
    m.light_bump_front_left_signal = i(F::LightBumpFrontLeftSignal) as u16;
    m.light_bump_center_left_signal = i(F::LightBumpCenterLeftSignal) as u16;
    m.light_bump_center_right_signal = i(F::LightBumpCenterRightSignal) as u16;
    m.light_bump_front_right_signal = i(F::LightBumpFrontRightSignal) as u16;
    m.light_bump_right_signal = i(F::LightBumpRightSignal) as u16;
    m.left_motor_current_ma = i(F::LeftMotorCurrentMa) as i16;
    m.right_motor_current_ma = i(F::RightMotorCurrentMa) as i16;
    m.main_brush_current_ma = i(F::MainBrushCurrentMa) as i16;
    m.side_brush_current_ma = i(F::SideBrushCurrentMa) as i16;
    m.stasis = b(F::Stasis);
    m
}

/// SIGINT and SIGTERM are blocked in every thread (call before any thread starts) and taken by
/// a signal thread instead, so the driver can stop the robot before it exits. Resetting them to
/// the default first matters: a background process inherits SIGINT ignored.
fn block_signals() -> libc::sigset_t {
    // SAFETY: plain signal mask manipulation on a local sigset_t.
    unsafe {
        libc::signal(libc::SIGINT, libc::SIG_DFL);
        libc::signal(libc::SIGTERM, libc::SIG_DFL);
        let mut set: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut set);
        libc::sigaddset(&mut set, libc::SIGINT);
        libc::sigaddset(&mut set, libc::SIGTERM);
        libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());
        set
    }
}

/// Spins the executor until `done()` holds after a spin ends (set the flag `done` reads before
/// calling `halt_spinning`). Jazzy's rcl_wait returns RCL_RET_TIMEOUT when a wait that ended for a
/// ROS-time timer finds the timer not due yet because the system clock was stepped back meanwhile
/// (WSL2's time sync steps it every ~30 s; NTP can too). rclrs 0.8 reports that as an error and
/// stops spinning, which ended the node: spin again when only timeouts were reported. Returns the
/// errors other than those timeouts.
fn spin_until(executor: &mut Executor, done: impl Fn() -> bool) -> Vec<RclrsError> {
    loop {
        let errors = executor.spin(SpinOptions::default());
        if errors.is_empty() || done() || errors.iter().any(|e| !e.is_timeout()) {
            return errors.into_iter().filter(|e| !e.is_timeout()).collect();
        }
    }
}

fn main() -> Result<()> {
    let signals = block_signals();
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("roomba_driver")?;
    let mut p = Params { node: &node, handles: Vec::new() };

    let port = p.s("port", "/dev/ttyUSB0")?;
    let baud = p.i("baud", 115200)?;
    let initial_mode = p.s("initial_mode", "safe")?.to_lowercase();
    let initial = match initial_mode.as_str() {
        "passive" => oi::MODE_PASSIVE,
        "safe" => oi::MODE_SAFE,
        "full" => oi::MODE_FULL,
        other => bail!("initial_mode must be passive, safe or full, not '{other}'"),
    };
    let auto_reenter_mode = p.b("auto_reenter_mode", false)?;
    let base_frame = p.s("base_frame", "base_link")?;
    let odom_frame = p.s("odom_frame", "odom")?;
    let publish_tf = p.b("publish_tf", false)?;
    let wheel_base = p.f("wheel_base_m", 0.235)?;
    let wheel_diameter = p.f("wheel_diameter_m", 0.072)?;
    let counts_per_rev = p.f("encoder_counts_per_rev", 508.8)?;
    let max_wheel_speed = p.f("max_wheel_speed_m_s", 0.3)?.min(oi::MAX_WHEEL_SPEED_MM_S as f64 / 1000.0);
    let cmd_timeout = p.f("cmd_vel_timeout_s", 0.5)?;
    let stamped = p.b("use_stamped_cmd_vel", false)?;
    let odom_rate = p.f("odom_rate_hz", 50.0)?;
    let sensor_rate = p.f("sensor_rate_hz", 20.0)?;
    let battery_rate = p.f("battery_rate_hz", 1.0)?;
    // Charge estimate. 'roomba': the robot's own coulomb counter (stock battery). 'voltage': the
    // pack voltage through a Li-ion discharge curve, for another pack wired to the battery
    // contacts (e.g. a power-tool battery), where the robot's mAh and temperature are made up.
    let battery_gauge = p.s("battery_gauge", "roomba")?.to_lowercase();
    if battery_gauge != "roomba" && battery_gauge != "voltage" {
        bail!("battery_gauge must be roomba or voltage, not '{battery_gauge}'");
    }
    let battery_cells = p.i("battery_cells", 5)?;
    let battery_capacity_ah = p.f("battery_capacity_ah", 0.0)?;
    let battery_resistance = p.f("battery_internal_resistance_ohm", 0.1)?;
    let smoothing_s = p.f("battery_smoothing_s", 30.0)?;
    let joint_names = vec![p.s("left_wheel_joint", "left_wheel_joint")?, p.s("right_wheel_joint", "right_wheel_joint")?];
    let wake_with_dtr = p.b("wake_with_dtr", false)?;
    let keep_awake_s = p.f("keep_awake_period_s", 60.0)?;
    let scroll_period = p.f("display_scroll_period_s", 0.35)?;
    // Sensor stream layouts (oi.rs): v2 has the light-bumper signals in every 15 ms frame; legacy
    // is the older set proven on the robot, where they come only ~5 times a second.
    let layouts = match oi::stream_layouts(&p.s("stream_layout", "v2")?.to_lowercase()) {
        Ok(layouts) => layouts,
        Err(e) => bail!("{e}"),
    };
    // Safe mode doesn't stop on bumps; the camera can't see the last ~30 cm, so the bumper is
    // the last line of defence. Forward motion is refused while a bumper is pressed.
    let bump_stop = p.b("bump_stop", true)?;
    // Also treat the front light-bumper (IR proximity) detections as bumps.
    let bump_stop_light = p.b("bump_stop_light_bumper", false)?;
    // 0: use the Roomba's own light-bumper flags. >0: a light bumper counts only when its signal
    // (0-4095) reaches this, e.g. so deep carpet pile (~100-200) doesn't block driving.
    let light_stop_signal = p.i("light_bumper_stop_signal", 0)?;
    // Reflex after a bump while driving forward: reverse a little, then turn away from the side
    // that was hit, ignoring drive commands until done (at most ~2 s).
    let bump_backup = p.b("bump_backup", false)?;
    let backup_distance = p.f("bump_backup_distance_m", 0.08)?;
    let backup_speed = p.f("bump_backup_speed_m_s", 0.1)?.abs();
    let backup_turn = p.f("bump_backup_turn_rad", 0.4)?.abs();
    // Faulty sensors to report as off everywhere (topics and safety logic), e.g. ['bump_right'].
    let ignore: Vec<String> = p.strings("ignore_sensors", &[""])?.into_iter().filter(|s| !s.is_empty()).collect();
    let mut unknown: Vec<&String> = ignore.iter().filter(|s| F::from_name(s).is_none() && *s != "header").collect();
    unknown.sort();
    unknown.dedup();
    if !unknown.is_empty() {
        let names: Vec<String> = unknown.iter().map(|s| format!("'{s}'")).collect();
        bail!("ignore_sensors: unknown sensor names [{}]", names.join(", "));
    }
    let params = p.handles;

    let cfg = Config {
        initial_mode: initial,
        auto_reenter_mode,
        wheel_base,
        wheel_diameter,
        counts_per_rev,
        max_wheel_speed,
        cmd_timeout,
        bump_stop,
        bump_stop_light,
        light_stop_signal,
        bump_backup,
        backup_distance,
        backup_speed,
        backup_turn,
        ignore_sensors: ignore.iter().filter_map(|s| F::from_name(s)).collect(),
        gauge: if battery_gauge == "voltage" {
            Gauge::voltage(battery_cells, battery_capacity_ah, battery_resistance, battery_rate, smoothing_s)
        } else {
            Gauge::Roomba
        },
    };

    let log_node = node.clone();
    let robot = Roomba::new(&port, baud as u32, Arc::new(move |level, msg: &str| log_to(&log_node, level, msg)), tty_opener());
    robot.set_wake_with_dtr(wake_with_dtr);

    let latched = QoSProfile::topics_default().keep_last(1).transient_local();
    let out = Arc::new(NodeOut { node: node.clone(), pub_mode: node.create_publisher("roomba/mode".qos(latched))? });
    let ctx = Ctx { core: Arc::new(Mutex::new(Core::new(cfg))), robot: Arc::clone(&robot), out: Arc::clone(&out) };
    let pub_odom = node.create_publisher::<nav_msgs::msg::Odometry>("wheel/odom".keep_last(10))?;
    let pub_joints = node.create_publisher::<sensor_msgs::msg::JointState>("joint_states".keep_last(10))?;
    let pub_battery = node.create_publisher::<sensor_msgs::msg::BatteryState>("battery_state".keep_last(10))?;
    let pub_sensors = node.create_publisher::<roomba_msgs::msg::SensorState>("roomba/sensors".keep_last(10))?;
    // tf2_ros TransformBroadcaster QoS: reliable, volatile, depth 100.
    let pub_tf = if publish_tf {
        Some(node.create_publisher::<tf2_msgs::msg::TFMessage>("/tf".qos(QoSProfile::topics_default().keep_last(100).reliable()))?)
    } else {
        None
    };

    {
        let c = ctx.clone();
        let clock = node.get_clock();
        robot.add_frame_callback(Arc::new(move |values: &SensorValues, t: f64, r: &Roomba| {
            let stamp = clock.now().nsec;
            lock(&c.core).on_frame(values, stamp, t, r, &*c.out);
        }));
    }

    // Depth 1: a backlog of old motion commands must never be replayed.
    let mut subs: Vec<Box<dyn Any>> = Vec::new();
    if stamped {
        let c = ctx.clone();
        subs.push(Box::new(node.create_subscription("cmd_vel".keep_last(1), move |m: geometry_msgs::msg::TwistStamped| {
            c.with(|core, l, o, now| core.on_cmd_vel(m.twist.linear.x, m.twist.angular.z, now, l, o))
        })?));
    } else {
        let c = ctx.clone();
        subs.push(Box::new(node.create_subscription("cmd_vel".keep_last(1), move |m: geometry_msgs::msg::Twist| {
            c.with(|core, l, o, now| core.on_cmd_vel(m.linear.x, m.angular.z, now, l, o))
        })?));
    }
    let c = ctx.clone();
    subs.push(Box::new(node.create_subscription("roomba/drive".keep_last(1), move |m: roomba_msgs::msg::Drive| {
        c.with(|core, l, o, now| core.on_drive(m.velocity_mm_s, m.radius_mm, now, l, o))
    })?));
    let c = ctx.clone();
    subs.push(Box::new(node.create_subscription("roomba/drive_direct".keep_last(1), move |m: roomba_msgs::msg::DriveDirect| {
        c.with(|core, l, o, now| core.on_drive_direct(m.left_mm_s, m.right_mm_s, now, l, o))
    })?));
    let c = ctx.clone();
    subs.push(Box::new(node.create_subscription("roomba/drive_pwm".keep_last(1), move |m: roomba_msgs::msg::DrivePwm| {
        c.with(|core, l, o, now| core.on_drive_pwm(m.left_pwm, m.right_pwm, now, l, o))
    })?));
    let c = ctx.clone();
    subs.push(Box::new(node.create_subscription("roomba/cleaning_motors".keep_last(10), move |m: roomba_msgs::msg::CleaningMotors| {
        c.with(|core, l, o, now| {
            core.on_cleaning_motors(m.use_pwm, m.main_brush, m.main_brush_outward, m.side_brush, m.side_brush_clockwise, m.vacuum,
                                    (m.main_brush_pwm, m.side_brush_pwm, m.vacuum_pwm), now, l, o)
        })
    })?));
    let c = ctx.clone();
    subs.push(Box::new(node.create_subscription("roomba/leds".keep_last(10), move |m: roomba_msgs::msg::Leds| {
        c.with(|core, l, o, now| core.on_leds(m.debris, m.spot, m.dock, m.check_robot, m.power_color, m.power_intensity, now, l, o))
    })?));
    let c = ctx.clone();
    subs.push(Box::new(node.create_subscription("roomba/scheduling_leds".keep_last(10), move |m: roomba_msgs::msg::SchedulingLeds| {
        c.with(|core, l, o, now| core.on_scheduling_leds(m.weekdays, m.schedule, m.clock, m.am, m.pm, m.colon, now, l, o))
    })?));
    let c = ctx.clone();
    subs.push(Box::new(node.create_subscription("roomba/display".keep_last(10), move |m: std_msgs::msg::String| {
        c.with(|core, l, o, now| core.on_display(&m.data, now, l, o))
    })?));
    let c = ctx.clone();
    subs.push(Box::new(node.create_subscription("roomba/display_raw".keep_last(10), move |m: std_msgs::msg::UInt8MultiArray| {
        c.with(|core, l, o, now| core.on_display_raw(&m.data, now, l, o))
    })?));

    // Services may block (mode confirmation, wake pulse): they run one at a time on their own
    // thread so they can never hold up the supervisor and odometry timers.
    let (jobs, job_rx) = mpsc::channel::<Job>();
    std::thread::Builder::new().name("services".into()).spawn(move || {
        for job in job_rx {
            job();
        }
    })?;
    let mut services: Vec<Box<dyn Any>> = Vec::new();
    let c = ctx.clone();
    services.push(Box::new(service::<roomba_msgs::srv::SetMode, _>(&node, "roomba/set_mode", &jobs, move |req| {
        let (success, message, mode) = set_mode_service(&c.core, &*c.robot, &*c.out, req.mode, mono);
        roomba_msgs::srv::SetMode_Response { success, message, mode }
    })?));
    for (name, opcode) in [("clean", oi::CLEAN), ("spot", oi::SPOT), ("max_clean", oi::MAX), ("seek_dock", oi::SEEK_DOCK),
                           ("power_off", oi::POWER)] {
        let c = ctx.clone();
        services.push(Box::new(service::<std_srvs::srv::Trigger, _>(&node, &format!("roomba/{name}"), &jobs, move |_| {
            trigger(c.with(|core, l, _, _| core.behaviour(name, opcode, l)))
        })?));
    }
    let c = ctx.clone();
    services.push(Box::new(service::<std_srvs::srv::Trigger, _>(&node, "roomba/reset", &jobs, move |_| {
        trigger(c.with(|core, l, _, _| core.reset(l)))
    })?));
    let c = ctx.clone();
    services.push(Box::new(service::<std_srvs::srv::Trigger, _>(&node, "roomba/wake", &jobs, move |_| {
        // The 1 s pulse runs without the lock.
        trigger(roomba_driver_rs::driver::result_message(Link::wake(&*c.robot), "wake pulse on DTR (needs DTR wired to pin 5)"))
    })?));
    let c = ctx.clone();
    services.push(Box::new(service::<roomba_msgs::srv::DefineSong, _>(&node, "roomba/define_song", &jobs, move |req| {
        let (success, message) = c.with(|core, l, _, _| core.define_song(req.song_number, &req.notes, &req.durations, req.play, l));
        roomba_msgs::srv::DefineSong_Response { success, message }
    })?));
    let c = ctx.clone();
    services.push(Box::new(service::<roomba_msgs::srv::PlaySong, _>(&node, "roomba/play_song", &jobs, move |req| {
        let (success, message) = c.with(|core, l, _, _| core.play_song(req.song_number, l));
        roomba_msgs::srv::PlaySong_Response { success, message }
    })?));
    let c = ctx.clone();
    services.push(Box::new(service::<roomba_msgs::srv::PressButtons, _>(&node, "roomba/press_buttons", &jobs, move |req| {
        let (success, message) = c.with(|core, l, _, _| core.press_buttons(req.buttons, l));
        roomba_msgs::srv::PressButtons_Response { success, message }
    })?));
    let c = ctx.clone();
    services.push(Box::new(service::<roomba_msgs::srv::SetSchedule, _>(&node, "roomba/set_schedule", &jobs, move |req| {
        let (success, message) = c.with(|core, l, _, _| core.set_schedule(req.days, &req.hours, &req.minutes, l));
        roomba_msgs::srv::SetSchedule_Response { success, message }
    })?));
    let c = ctx.clone();
    services.push(Box::new(service::<roomba_msgs::srv::SetDayTime, _>(&node, "roomba/set_day_time", &jobs, move |req| {
        let (day, hour, minute) = if req.use_system_time {
            // SAFETY: localtime_r only writes the tm passed to it.
            let tm = unsafe {
                let now = libc::time(std::ptr::null_mut());
                let mut tm: libc::tm = std::mem::zeroed();
                libc::localtime_r(&now, &mut tm);
                tm
            };
            (tm.tm_wday as i64, tm.tm_hour as i64, tm.tm_min as i64)
        } else {
            (req.day as i64, req.hour as i64, req.minute as i64)
        };
        let (success, message) = c.with(|core, l, _, _| core.set_day_time(day, hour, minute, l));
        roomba_msgs::srv::SetDayTime_Response { success, message }
    })?));

    let created = Instant::now();
    let sensor_frame = base_frame.clone();
    let mut timers: Vec<Box<dyn Any>> = Vec::new();
    let c = ctx.clone();
    timers.push(Box::new(node.create_timer_repeating(Duration::from_secs_f64(1.0 / odom_rate).node_time(), move || {
        let Some(s) = lock(&c.core).take_odometry() else { return };
        let msg = odometry_msg(&s, &odom_frame, &base_frame);
        let joints = sensor_msgs::msg::JointState {
            header: header(s.stamp_ns, ""),
            name: joint_names.clone(),
            position: s.positions.to_vec(),
            velocity: s.velocities.to_vec(),
            effort: Vec::new(),
        };
        let tf = pub_tf.as_ref().map(|_| geometry_msgs::msg::TransformStamped {
            header: msg.header.clone(),
            child_frame_id: msg.child_frame_id.clone(),
            transform: geometry_msgs::msg::Transform {
                translation: geometry_msgs::msg::Vector3 { x: s.x, y: s.y, z: 0.0 },
                rotation: msg.pose.pose.orientation.clone(),
            },
        });
        pub_odom.publish(msg).ok();
        pub_joints.publish(joints).ok();
        if let (Some(publisher), Some(tf)) = (&pub_tf, tf) {
            publisher.publish(tf2_msgs::msg::TFMessage { transforms: vec![tf] }).ok();
        }
    })?));
    let c = ctx.clone();
    timers.push(Box::new(node.create_timer_repeating(Duration::from_secs_f64(1.0 / sensor_rate).node_time(), move || {
        let Some((values, stamp)) = lock(&c.core).sensor_snapshot() else { return };
        pub_sensors.publish(sensor_msg(&values, stamp, &sensor_frame)).ok();
    })?));
    let c = ctx.clone();
    timers.push(Box::new(node.create_timer_repeating(Duration::from_secs_f64(1.0 / battery_rate).node_time(), move || {
        let Some((b, stamp)) = lock(&c.core).battery_state() else { return };
        let msg = sensor_msgs::msg::BatteryState {
            header: header(stamp, ""),
            voltage: b.voltage as f32,
            temperature: b.temperature as f32,
            current: b.current as f32,
            charge: b.charge as f32,
            capacity: b.capacity as f32,
            design_capacity: b.design_capacity as f32,
            percentage: b.percentage as f32,
            power_supply_status: b.status,
            power_supply_health: b.health,
            power_supply_technology: b.technology,
            present: true,
            ..Default::default()
        };
        pub_battery.publish(msg).ok();
    })?));
    let c = ctx.clone();
    timers.push(Box::new(node.create_timer_repeating(Duration::from_secs_f64(driver::SUPERVISE_PERIOD_S).node_time(), move || {
        c.with(|core, l, o, now| core.supervise(now, l, o))
    })?));
    let c = ctx.clone();
    timers.push(Box::new(node.create_timer_repeating(Duration::from_secs_f64(scroll_period).node_time(), move || {
        c.with(|core, l, o, now| core.scroll_display(now, l, o))
    })?));

    let shutting_down = Arc::new(AtomicBool::new(false));
    if wake_with_dtr && keep_awake_s > 0.0 {
        let (c, stop) = (ctx.clone(), Arc::clone(&shutting_down));
        std::thread::Builder::new().name("keep-awake".into()).spawn(move || loop {
            std::thread::sleep(Duration::from_secs_f64(keep_awake_s));
            if stop.load(Ordering::SeqCst) {
                break;
            }
            if lock(&c.core).wants_keep_awake() {
                let r = Link::wake(&*c.robot);
                lock(&c.core).try_result(r, "keep-awake pulse", mono(), &*c.out);
            }
        })?;
    }

    log!(node.info(), "Roomba driver starting on {port} at {baud} baud (initial mode {initial_mode})");
    if !connect(&robot, &node, layouts) {
        // Like the Python node's 2 s connect timer (created before the first attempt).
        let (r, n, stop) = (Arc::clone(&robot), node.clone(), Arc::clone(&shutting_down));
        std::thread::Builder::new().name("connect".into()).spawn(move || {
            let mut k = 1u32;
            loop {
                let due = created + Duration::from_secs(2) * k;
                k += 1;
                std::thread::sleep(due.saturating_duration_since(Instant::now()));
                if stop.load(Ordering::SeqCst) || connect(&r, &n, layouts) {
                    break;
                }
            }
        })?;
    }

    let stopping = Arc::new(AtomicBool::new(false));
    let (flag, commands) = (Arc::clone(&stopping), Arc::clone(executor.commands()));
    std::thread::Builder::new().name("signals".into()).spawn(move || {
        let mut sig = 0;
        // SAFETY: waits for one of the blocked signals in `signals`.
        unsafe { libc::sigwait(&signals, &mut sig) };
        flag.store(true, Ordering::SeqCst);
        commands.halt_spinning();
    })?;
    let errors = spin_until(&mut executor, || stopping.load(Ordering::SeqCst));

    // Stop everything and hand the robot back in Passive mode so its buttons work.
    shutting_down.store(true, Ordering::SeqCst);
    if Link::is_open(&*robot) {
        let _ = robot.write(&oi::drive_direct(0.0, 0.0));
        let _ = robot.write(&oi::pwm_motors(0.0, 0.0, 0.0));
        let _ = robot.write(&oi::digit_leds_ascii("    "));
        robot.stop_stream();
        let _ = robot.start();
    }
    robot.close();
    drop((subs, services, timers, params));
    if let Some(e) = errors.into_iter().next() {
        return Err(e.into());
    }
    Ok(())
}
