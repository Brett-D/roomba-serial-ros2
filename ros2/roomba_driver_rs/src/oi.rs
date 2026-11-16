//! iRobot Roomba 500-series Open Interface (OI): opcodes, sensor packets and command encoding.
//! Port of roomba_driver/oi.py: pure protocol code with no I/O. Multi-byte values are big-endian.
//! Errors carry the same messages as the Python ValueErrors (they reach service responses).

// ---------------------------------------------------------------- opcodes
pub const RESET: u8 = 7;
pub const START: u8 = 128;
pub const BAUD: u8 = 129;
pub const CONTROL: u8 = 130;
pub const SAFE: u8 = 131;
pub const FULL: u8 = 132;
pub const POWER: u8 = 133;
pub const SPOT: u8 = 134;
pub const CLEAN: u8 = 135;
pub const MAX: u8 = 136;
pub const DRIVE: u8 = 137;
pub const MOTORS: u8 = 138;
pub const LEDS: u8 = 139;
pub const SONG: u8 = 140;
pub const PLAY: u8 = 141;
pub const SENSORS: u8 = 142;
pub const SEEK_DOCK: u8 = 143;
pub const PWM_MOTORS: u8 = 144;
pub const DRIVE_DIRECT: u8 = 145;
pub const DRIVE_PWM: u8 = 146;
pub const STREAM: u8 = 148;
pub const QUERY_LIST: u8 = 149;
pub const PAUSE_RESUME_STREAM: u8 = 150;
pub const SCHEDULING_LEDS: u8 = 162;
pub const DIGIT_LEDS_RAW: u8 = 163;
pub const DIGIT_LEDS_ASCII: u8 = 164;
pub const BUTTONS: u8 = 165;
pub const SCHEDULE: u8 = 167;
pub const SET_DAY_TIME: u8 = 168;
pub const STOP: u8 = 173;

pub const MODE_OFF: u8 = 0;
pub const MODE_PASSIVE: u8 = 1;
pub const MODE_SAFE: u8 = 2;
pub const MODE_FULL: u8 = 3;

/// MODE_NAMES[mode] of oi.py.
pub fn mode_name(mode: i64) -> Option<&'static str> {
    match mode {
        0 => Some("off"),
        1 => Some("passive"),
        2 => Some("safe"),
        3 => Some("full"),
        _ => None,
    }
}

/// Python's `MODE_NAMES.get(mode, mode)` formatted with str(): the name, the number, or "None".
pub fn mode_str(mode: Option<i64>) -> String {
    match mode {
        Some(m) => mode_name(m).map_or_else(|| m.to_string(), str::to_string),
        None => "None".into(),
    }
}

pub const MAX_WHEEL_SPEED_MM_S: i64 = 500;
pub const MAX_RADIUS_MM: i64 = 2000;
pub const MAX_DRIVE_PWM: i64 = 255;
pub const MAX_BRUSH_PWM: i64 = 127;
pub const MAX_VACUUM_PWM: i64 = 127;
pub const RADIUS_STRAIGHT: i64 = 32767;
pub const RADIUS_TURN_CW: i64 = -1;
pub const RADIUS_TURN_CCW: i64 = 1;
pub const NUM_SONGS: i64 = 5;
pub const MAX_SONG_NOTES: usize = 16;

pub const STREAM_HEADER: u8 = 19;
pub const STREAM_PERIOD_S: f64 = 0.015;

// ---------------------------------------------------------------- sensor fields
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Bool,
    Int,
}

macro_rules! fields {
    ($($v:ident = $n:literal $k:ident),* $(,)?) => {
        /// Every decoded sensor value, in roomba_msgs/SensorState order (names = oi.py keys).
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
        #[repr(u8)]
        pub enum Field { $($v),* }
        /// (field, name, kind) for every field, indexed by `Field as usize`.
        pub const FIELDS: &[(Field, &str, Kind)] = &[$((Field::$v, $n, Kind::$k)),*];
    };
}

fields! {
    BumpLeft = "bump_left" Bool, BumpRight = "bump_right" Bool,
    WheelDropLeft = "wheel_drop_left" Bool, WheelDropRight = "wheel_drop_right" Bool,
    Wall = "wall" Bool, CliffLeft = "cliff_left" Bool, CliffFrontLeft = "cliff_front_left" Bool,
    CliffFrontRight = "cliff_front_right" Bool, CliffRight = "cliff_right" Bool, VirtualWall = "virtual_wall" Bool,
    OvercurrentLeftWheel = "overcurrent_left_wheel" Bool, OvercurrentRightWheel = "overcurrent_right_wheel" Bool,
    OvercurrentMainBrush = "overcurrent_main_brush" Bool, OvercurrentSideBrush = "overcurrent_side_brush" Bool,
    DirtDetect = "dirt_detect" Int, IrOmni = "ir_omni" Int, IrLeft = "ir_left" Int, IrRight = "ir_right" Int,
    ButtonClean = "button_clean" Bool, ButtonSpot = "button_spot" Bool, ButtonDock = "button_dock" Bool,
    ButtonMinute = "button_minute" Bool, ButtonHour = "button_hour" Bool, ButtonDay = "button_day" Bool,
    ButtonSchedule = "button_schedule" Bool, ButtonClock = "button_clock" Bool,
    DistanceMm = "distance_mm" Int, AngleDeg = "angle_deg" Int,
    ChargingState = "charging_state" Int, VoltageMv = "voltage_mv" Int, CurrentMa = "current_ma" Int,
    TemperatureC = "temperature_c" Int, ChargeMah = "charge_mah" Int, CapacityMah = "capacity_mah" Int,
    WallSignal = "wall_signal" Int, CliffLeftSignal = "cliff_left_signal" Int,
    CliffFrontLeftSignal = "cliff_front_left_signal" Int, CliffFrontRightSignal = "cliff_front_right_signal" Int,
    CliffRightSignal = "cliff_right_signal" Int,
    InternalChargerAvailable = "internal_charger_available" Bool, HomeBaseAvailable = "home_base_available" Bool,
    OiMode = "oi_mode" Int, SongNumber = "song_number" Int, SongPlaying = "song_playing" Bool,
    StreamPacketCount = "stream_packet_count" Int, RequestedVelocityMmS = "requested_velocity_mm_s" Int,
    RequestedRadiusMm = "requested_radius_mm" Int, RequestedRightVelocityMmS = "requested_right_velocity_mm_s" Int,
    RequestedLeftVelocityMmS = "requested_left_velocity_mm_s" Int,
    LeftEncoderCounts = "left_encoder_counts" Int, RightEncoderCounts = "right_encoder_counts" Int,
    LightBumpLeft = "light_bump_left" Bool, LightBumpFrontLeft = "light_bump_front_left" Bool,
    LightBumpCenterLeft = "light_bump_center_left" Bool, LightBumpCenterRight = "light_bump_center_right" Bool,
    LightBumpFrontRight = "light_bump_front_right" Bool, LightBumpRight = "light_bump_right" Bool,
    LightBumpLeftSignal = "light_bump_left_signal" Int, LightBumpFrontLeftSignal = "light_bump_front_left_signal" Int,
    LightBumpCenterLeftSignal = "light_bump_center_left_signal" Int,
    LightBumpCenterRightSignal = "light_bump_center_right_signal" Int,
    LightBumpFrontRightSignal = "light_bump_front_right_signal" Int, LightBumpRightSignal = "light_bump_right_signal" Int,
    LeftMotorCurrentMa = "left_motor_current_ma" Int, RightMotorCurrentMa = "right_motor_current_ma" Int,
    MainBrushCurrentMa = "main_brush_current_ma" Int, SideBrushCurrentMa = "side_brush_current_ma" Int,
    Stasis = "stasis" Bool,
}

pub const NUM_FIELDS: usize = 68;
const _: () = assert!(FIELDS.len() == NUM_FIELDS);

impl Field {
    pub fn name(self) -> &'static str {
        FIELDS[self as usize].1
    }
    pub fn kind(self) -> Kind {
        FIELDS[self as usize].2
    }
    pub fn from_name(name: &str) -> Option<Field> {
        FIELDS.iter().find(|f| f.1 == name).map(|f| f.0)
    }
}

/// Decoded sensor values with the set of fields seen (the Python dicts of name -> value).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorValues {
    values: [i32; NUM_FIELDS],
    present: u128,
}

impl Default for SensorValues {
    fn default() -> Self {
        Self { values: [0; NUM_FIELDS], present: 0 }
    }
}

impl SensorValues {
    pub fn set(&mut self, f: Field, v: i32) {
        self.values[f as usize] = v;
        self.present |= 1u128 << f as u32;
    }
    pub fn has(&self, f: Field) -> bool {
        self.present >> f as u32 & 1 == 1
    }
    pub fn get(&self, f: Field) -> Option<i32> {
        self.has(f).then_some(self.values[f as usize])
    }
    /// The value, 0 / false when never received (dict.get(k, 0)).
    pub fn int(&self, f: Field) -> i32 {
        self.values[f as usize]
    }
    /// Truthiness, false when never received (bool(dict.get(k))).
    pub fn flag(&self, f: Field) -> bool {
        self.values[f as usize] != 0
    }
    pub fn len(&self) -> usize {
        self.present.count_ones() as usize
    }
    pub fn is_empty(&self) -> bool {
        self.present == 0
    }
    /// dict.update(other)
    pub fn update(&mut self, other: &SensorValues) {
        for i in 0..NUM_FIELDS {
            if other.present >> i & 1 == 1 {
                self.values[i] = other.values[i];
            }
        }
        self.present |= other.present;
    }
    /// Fields that are present, with their values.
    pub fn iter(&self) -> impl Iterator<Item = (Field, i32)> + '_ {
        FIELDS.iter().filter(|f| self.has(f.0)).map(|f| (f.0, self.values[f.0 as usize]))
    }
}

// ---------------------------------------------------------------- sensor packets
#[derive(Clone, Copy)]
enum Fmt {
    U8,
    I8,
    U16,
    I16,
}

impl Fmt {
    const fn size(self) -> usize {
        match self {
            Fmt::U8 | Fmt::I8 => 1,
            Fmt::U16 | Fmt::I16 => 2,
        }
    }
}

#[derive(Clone, Copy)]
enum Dec {
    Value(Field),
    Bool(Field),
    Bits(&'static [(u8, Field)]),
    Masked(Field, i32),
    Skip,
}

use Field as F;
use Fmt::*;

/// Packets 7..=58: (format, decoding).
const PACKETS: [(Fmt, Dec); 52] = [
    (U8, Dec::Bits(&[(0, F::BumpRight), (1, F::BumpLeft), (2, F::WheelDropRight), (3, F::WheelDropLeft)])), // 7
    (U8, Dec::Bool(F::Wall)),
    (U8, Dec::Bool(F::CliffLeft)),
    (U8, Dec::Bool(F::CliffFrontLeft)), // 10
    (U8, Dec::Bool(F::CliffFrontRight)),
    (U8, Dec::Bool(F::CliffRight)),
    (U8, Dec::Bool(F::VirtualWall)),
    (U8, Dec::Bits(&[(0, F::OvercurrentSideBrush), (2, F::OvercurrentMainBrush), (3, F::OvercurrentRightWheel),
                     (4, F::OvercurrentLeftWheel)])),
    (U8, Dec::Value(F::DirtDetect)), // 15
    (U8, Dec::Skip),
    (U8, Dec::Value(F::IrOmni)),
    (U8, Dec::Bits(&[(0, F::ButtonClean), (1, F::ButtonSpot), (2, F::ButtonDock), (3, F::ButtonMinute),
                     (4, F::ButtonHour), (5, F::ButtonDay), (6, F::ButtonSchedule), (7, F::ButtonClock)])),
    (I16, Dec::Value(F::DistanceMm)),
    (I16, Dec::Value(F::AngleDeg)), // 20
    (U8, Dec::Value(F::ChargingState)),
    (U16, Dec::Value(F::VoltageMv)),
    (I16, Dec::Value(F::CurrentMa)),
    (I8, Dec::Value(F::TemperatureC)),
    (U16, Dec::Value(F::ChargeMah)), // 25
    (U16, Dec::Value(F::CapacityMah)),
    (U16, Dec::Value(F::WallSignal)),
    (U16, Dec::Value(F::CliffLeftSignal)),
    (U16, Dec::Value(F::CliffFrontLeftSignal)),
    (U16, Dec::Value(F::CliffFrontRightSignal)), // 30
    (U16, Dec::Value(F::CliffRightSignal)),
    (U8, Dec::Skip),
    (U16, Dec::Skip),
    (U8, Dec::Bits(&[(0, F::InternalChargerAvailable), (1, F::HomeBaseAvailable)])),
    (U8, Dec::Value(F::OiMode)), // 35
    (U8, Dec::Value(F::SongNumber)),
    (U8, Dec::Bool(F::SongPlaying)),
    (U8, Dec::Value(F::StreamPacketCount)),
    (I16, Dec::Value(F::RequestedVelocityMmS)),
    (I16, Dec::Value(F::RequestedRadiusMm)), // 40
    (I16, Dec::Value(F::RequestedRightVelocityMmS)),
    (I16, Dec::Value(F::RequestedLeftVelocityMmS)),
    (U16, Dec::Value(F::LeftEncoderCounts)),
    (U16, Dec::Value(F::RightEncoderCounts)),
    (U8, Dec::Bits(&[(0, F::LightBumpLeft), (1, F::LightBumpFrontLeft), (2, F::LightBumpCenterLeft),
                     (3, F::LightBumpCenterRight), (4, F::LightBumpFrontRight), (5, F::LightBumpRight)])), // 45
    (U16, Dec::Value(F::LightBumpLeftSignal)),
    (U16, Dec::Value(F::LightBumpFrontLeftSignal)),
    (U16, Dec::Value(F::LightBumpCenterLeftSignal)),
    (U16, Dec::Value(F::LightBumpCenterRightSignal)),
    (U16, Dec::Value(F::LightBumpFrontRightSignal)), // 50
    (U16, Dec::Value(F::LightBumpRightSignal)),
    (U8, Dec::Value(F::IrLeft)),
    (U8, Dec::Value(F::IrRight)),
    (I16, Dec::Value(F::LeftMotorCurrentMa)),
    (I16, Dec::Value(F::RightMotorCurrentMa)), // 55
    (I16, Dec::Value(F::MainBrushCurrentMa)),
    (I16, Dec::Value(F::SideBrushCurrentMa)),
    // stasis: bit 0 = caster toggling; bit 1 = "stasis disabled" on some firmware
    (U8, Dec::Masked(F::Stasis, 0x01)),
];

/// Individual packet ids contained in a packet or group id, as a half-open range.
pub fn packet_range(packet_id: u8) -> Result<std::ops::Range<u8>, String> {
    Ok(match packet_id {
        0 => 7..27,
        1 => 7..17,
        2 => 17..21,
        3 => 21..27,
        4 => 27..35,
        5 => 35..43,
        6 => 7..43,
        100 => 7..59,
        101 => 43..59,
        106 => 46..52,
        107 => 54..59,
        7..=58 => packet_id..packet_id + 1,
        _ => return Err(format!("unknown sensor packet id {packet_id}")),
    })
}

pub fn packet_ids(packet_id: u8) -> Result<Vec<u8>, String> {
    Ok(packet_range(packet_id)?.collect())
}

/// Number of data bytes the robot returns for a packet or group id.
pub fn packet_size(packet_id: u8) -> Result<usize, String> {
    Ok(packet_range(packet_id)?.map(|i| PACKETS[(i - 7) as usize].0.size()).sum())
}

fn decode_into(packet_id: u8, data: &[u8], out: &mut SensorValues) -> Result<(), String> {
    let size = packet_size(packet_id)?;
    if data.len() != size {
        return Err(format!("packet {packet_id}: expected {size} bytes, got {}", data.len()));
    }
    let mut offset = 0;
    for pid in packet_range(packet_id)? {
        let (fmt, dec) = PACKETS[(pid - 7) as usize];
        let b = &data[offset..offset + fmt.size()];
        offset += fmt.size();
        let value = match fmt {
            U8 => b[0] as i32,
            I8 => b[0] as i8 as i32,
            U16 => u16::from_be_bytes([b[0], b[1]]) as i32,
            I16 => i16::from_be_bytes([b[0], b[1]]) as i32,
        };
        match dec {
            Dec::Value(f) => out.set(f, value),
            Dec::Bool(f) => out.set(f, (value != 0) as i32),
            Dec::Bits(bits) => {
                for &(bit, f) in bits {
                    out.set(f, value >> bit & 1);
                }
            }
            Dec::Masked(f, mask) => out.set(f, (value & mask != 0) as i32),
            Dec::Skip => {}
        }
    }
    Ok(())
}

/// Decode the data bytes of one packet or group.
pub fn decode(packet_id: u8, data: &[u8]) -> Result<SensorValues, String> {
    let mut out = SensorValues::default();
    decode_into(packet_id, data, &mut out)?;
    Ok(out)
}

/// The data bytes the robot sends for a packet or group, from each packet's raw value (`raw(id)`
/// for ids 7..=58: the byte or 16-bit word as sent, e.g. the bump/wheel-drop bits of packet 7).
/// The inverse of [`decode`] (the twin's Open Interface uses it).
pub fn encode(packet_id: u8, raw: &impl Fn(u8) -> i32) -> Result<Vec<u8>, String> {
    let mut out = Vec::with_capacity(packet_size(packet_id)?);
    for pid in packet_range(packet_id)? {
        let v = raw(pid);
        match PACKETS[(pid - 7) as usize].0 {
            U8 | I8 => out.push(v as u8),
            U16 | I16 => out.extend_from_slice(&(v as u16).to_be_bytes()),
        }
    }
    Ok(out)
}

/// A whole stream frame [19][n][id][data]...[checksum] for the requested ids.
pub fn encode_stream_frame(ids: &[u8], raw: &impl Fn(u8) -> i32) -> Result<Vec<u8>, String> {
    let mut body = vec![STREAM_HEADER, 0];
    for &pid in ids {
        body.push(pid);
        body.extend(encode(pid, raw)?);
    }
    body[1] = (body.len() - 2) as u8;
    let sum = body.iter().fold(0u8, |a, &b| a.wrapping_add(b));
    body.push(sum.wrapping_neg());
    Ok(body)
}

/// Decode the [id][data][id][data]... body of a stream frame.
pub fn decode_stream_payload(payload: &[u8]) -> Result<SensorValues, String> {
    let mut out = SensorValues::default();
    let mut i = 0;
    while i < payload.len() {
        let pid = payload[i];
        let size = packet_size(pid)?;
        if i + 1 + size > payload.len() {
            return Err("truncated stream payload".into());
        }
        decode_into(pid, &payload[i + 1..i + 1 + size], &mut out)?;
        i += 1 + size;
    }
    Ok(out)
}

/// Value of the stream frame's byte-count field for a list of requested ids.
pub fn stream_payload_length(ids: &[u8]) -> Result<usize, String> {
    ids.iter().map(|&pid| packet_size(pid).map(|n| n + 1)).sum()
}

/// Streaming every sensor in one frame (group 100, 84 bytes) makes the robot send frames with bad
/// checksums in Safe/Full mode, so the driver streams a 70-byte primary set every 15 ms and swaps
/// in the remaining sensors (with the safety, mode and odometry packets) in a 46-byte frame a few
/// times a second. The primary set carries everything the driver acts on: bumps, wheel drops,
/// cliffs, the light bumpers with their signals (light_bumper_stop_signal is checked every frame),
/// the charger and the battery. This is stream_layout v2, the default.
pub const STREAM_PRIMARY: [u8; 8] = [0, 4, 35, 43, 44, 45, 106, 58];
pub const STREAM_SECONDARY: [u8; 11] = [7, 9, 10, 11, 12, 5, 43, 44, 52, 53, 107];
/// The layouts before v2 (same 70/46-byte frames, proven clean on a Roomba 550): the light-bump
/// signals come only in the secondary frame, so light_bumper_stop_signal is checked ~5 times a second.
pub const STREAM_PRIMARY_LEGACY: [u8; 6] = [6, 43, 44, 45, 54, 55];
pub const STREAM_SECONDARY_LEGACY: [u8; 14] = [7, 9, 10, 11, 12, 35, 43, 44, 106, 52, 53, 56, 57, 58];

/// (primary, secondary) stream layouts for the driver's stream_layout parameter.
pub fn stream_layouts(name: &str) -> Result<(&'static [u8], &'static [u8]), String> {
    match name {
        "v2" => Ok((&STREAM_PRIMARY, &STREAM_SECONDARY)),
        "legacy" => Ok((&STREAM_PRIMARY_LEGACY, &STREAM_SECONDARY_LEGACY)),
        other => Err(format!("stream_layout must be v2 or legacy, not '{other}'")),
    }
}
/// Bytes that fit in 15 ms at 115200 baud.
pub const MAX_STREAM_FRAME: usize = 172;

/// Re-synchronising parser for opcode-148 frames: [19][n][id][data]...[checksum].
/// Accepts frames for any of the given id lists (the robot can be switched between them).
#[derive(Debug, Default)]
pub struct StreamParser {
    expected: Vec<usize>,
    pub bad_frames: u64,
    buf: Vec<u8>,
}

impl StreamParser {
    pub fn new(id_lists: &[&[u8]]) -> Result<Self, String> {
        let mut expected = Vec::new();
        for ids in id_lists {
            let n = stream_payload_length(ids)?;
            if !expected.contains(&n) {
                expected.push(n);
            }
        }
        Ok(Self { expected, bad_frames: 0, buf: Vec::new() })
    }

    /// Add received bytes; return the decoded frames.
    pub fn feed(&mut self, data: &[u8]) -> Vec<SensorValues> {
        self.feed_frames(data).into_iter().map(|(_, v)| v).collect()
    }

    /// Like feed(), but returns (n, values) so callers can tell which layout arrived.
    pub fn feed_frames(&mut self, data: &[u8]) -> Vec<(usize, SensorValues)> {
        self.buf.extend_from_slice(data);
        let mut frames = Vec::new();
        let mut pos = 0; // consumed bytes, drained once at the end
        loop {
            let Some(start) = self.buf[pos..].iter().position(|&b| b == STREAM_HEADER) else {
                pos = self.buf.len();
                break;
            };
            pos += start;
            let buf = &self.buf[pos..];
            if buf.len() < 2 {
                break;
            }
            let n = buf[1] as usize;
            if !self.expected.contains(&n) {
                pos += 1;
                continue;
            }
            if buf.len() < n + 3 {
                break;
            }
            let frame = &buf[..n + 3];
            if frame.iter().fold(0u8, |a, &b| a.wrapping_add(b)) == 0 {
                if let Ok(values) = decode_stream_payload(&frame[2..n + 2]) {
                    frames.push((n, values));
                    pos += n + 3;
                    continue;
                }
            }
            self.bad_frames += 1;
            pos += 1;
        }
        self.buf.drain(..pos);
        frames
    }
}

// ---------------------------------------------------------------- command encoding
/// Python's int(round(x)) (round half to even) clamped to [lo, hi].
pub fn clamp(value: f64, lo: i64, hi: i64) -> i64 {
    (value.round_ties_even() as i64).clamp(lo, hi)
}

fn check(name: &str, value: i64, lo: i64, hi: i64) -> Result<u8, String> {
    if !(lo..=hi).contains(&value) {
        return Err(format!("{name} must be {lo}-{hi}, got {value}"));
    }
    Ok(value as u8)
}

fn be16(v: i64) -> [u8; 2] {
    (v as i16).to_be_bytes()
}

pub fn baud(rate: u32) -> Result<Vec<u8>, String> {
    let code = match rate {
        300 => 0,
        600 => 1,
        1200 => 2,
        2400 => 3,
        4800 => 4,
        9600 => 5,
        14400 => 6,
        19200 => 7,
        28800 => 8,
        38400 => 9,
        57600 => 10,
        115200 => 11,
        _ => return Err(format!("unsupported baud rate {rate}")),
    };
    Ok(vec![BAUD, code])
}

/// Opcode 137. Velocity is clamped to +-500 mm/s, radius to +-2000 mm (special values kept).
pub fn drive(velocity_mm_s: f64, radius_mm: i64) -> Vec<u8> {
    let v = clamp(velocity_mm_s, -MAX_WHEEL_SPEED_MM_S, MAX_WHEEL_SPEED_MM_S);
    let r = match radius_mm {
        RADIUS_STRAIGHT | 32768 | -32768 => RADIUS_STRAIGHT,
        RADIUS_TURN_CW | RADIUS_TURN_CCW => radius_mm,
        r => r.clamp(-MAX_RADIUS_MM, MAX_RADIUS_MM),
    };
    let (v, r) = (be16(v), be16(r));
    vec![DRIVE, v[0], v[1], r[0], r[1]]
}

/// Opcode 145. The robot expects the RIGHT wheel first.
pub fn drive_direct(left_mm_s: f64, right_mm_s: f64) -> Vec<u8> {
    let l = be16(clamp(left_mm_s, -MAX_WHEEL_SPEED_MM_S, MAX_WHEEL_SPEED_MM_S));
    let r = be16(clamp(right_mm_s, -MAX_WHEEL_SPEED_MM_S, MAX_WHEEL_SPEED_MM_S));
    vec![DRIVE_DIRECT, r[0], r[1], l[0], l[1]]
}

/// Opcode 146. Right wheel first, -255..255.
pub fn drive_pwm(left_pwm: f64, right_pwm: f64) -> Vec<u8> {
    let l = be16(clamp(left_pwm, -MAX_DRIVE_PWM, MAX_DRIVE_PWM));
    let r = be16(clamp(right_pwm, -MAX_DRIVE_PWM, MAX_DRIVE_PWM));
    vec![DRIVE_PWM, r[0], r[1], l[0], l[1]]
}

/// Opcode 138: cleaning motors on/off at full speed.
pub fn motors(side_brush: bool, vacuum: bool, main_brush: bool, side_brush_clockwise: bool, main_brush_outward: bool) -> Vec<u8> {
    let bits = side_brush as u8
        | (vacuum as u8) << 1
        | (main_brush as u8) << 2
        | (side_brush_clockwise as u8) << 3
        | (main_brush_outward as u8) << 4;
    vec![MOTORS, bits]
}

/// Opcode 144: cleaning motor duty cycles.
pub fn pwm_motors(main_brush: f64, side_brush: f64, vacuum: f64) -> Vec<u8> {
    vec![
        PWM_MOTORS,
        clamp(main_brush, -MAX_BRUSH_PWM, MAX_BRUSH_PWM) as i8 as u8,
        clamp(side_brush, -MAX_BRUSH_PWM, MAX_BRUSH_PWM) as i8 as u8,
        clamp(vacuum, 0, MAX_VACUUM_PWM) as u8,
    ]
}

/// Opcode 139. power_color 0 = green ... 255 = red.
pub fn leds(debris: bool, spot: bool, dock: bool, check_robot: bool, power_color: f64, power_intensity: f64) -> Vec<u8> {
    let bits = debris as u8 | (spot as u8) << 1 | (dock as u8) << 2 | (check_robot as u8) << 3;
    vec![LEDS, bits, clamp(power_color, 0, 255) as u8, clamp(power_intensity, 0, 255) as u8]
}

/// Opcode 162. weekdays: bit 0 = Sunday ... bit 6 = Saturday.
pub fn scheduling_leds(weekdays: i64, schedule: bool, clock: bool, am: bool, pm: bool, colon: bool) -> Vec<u8> {
    let bits = colon as u8 | (pm as u8) << 1 | (am as u8) << 2 | (clock as u8) << 3 | (schedule as u8) << 4;
    vec![SCHEDULING_LEDS, (weekdays & 0x7F) as u8, bits]
}

/// Opcode 163. Four segment bitmasks (bit 0 = A ... bit 6 = G), leftmost digit first.
pub fn digit_leds_raw(segments: &[i64]) -> Result<Vec<u8>, String> {
    if segments.len() != 4 {
        return Err("need exactly 4 digits".into());
    }
    let mut out = vec![DIGIT_LEDS_RAW];
    out.extend(segments.iter().map(|&s| (s & 0x7F) as u8));
    Ok(out)
}

/// Opcode 164. First four characters, padded with spaces; unprintable characters become spaces.
pub fn digit_leds_ascii(text: &str) -> Vec<u8> {
    let mut out = vec![DIGIT_LEDS_ASCII];
    out.extend(text.chars().take(4).map(|c| if (' '..='~').contains(&c) { c as u8 } else { b' ' }));
    out.resize(5, b' ');
    out
}

/// Opcode 165. bit 0 Clean, 1 Spot, 2 Dock, 3 Minute, 4 Hour, 5 Day, 6 Schedule, 7 Clock.
pub fn buttons(mask: i64) -> Result<Vec<u8>, String> {
    Ok(vec![BUTTONS, check("buttons", mask, 0, 255)?])
}

/// Opcode 140. notes: (midi_note, duration_64ths) pairs.
pub fn song(number: i64, notes: &[(i64, i64)]) -> Result<Vec<u8>, String> {
    check("song number", number, 0, NUM_SONGS - 1)?;
    if notes.is_empty() || notes.len() > MAX_SONG_NOTES {
        return Err(format!("a song needs 1-{MAX_SONG_NOTES} notes, got {}", notes.len()));
    }
    let mut out = vec![SONG, number as u8, notes.len() as u8];
    for &(note, duration) in notes {
        out.push(check("note", note, 0, 255)?);
        out.push(check("duration", duration, 0, 255)?);
    }
    Ok(out)
}

pub fn play(number: i64) -> Result<Vec<u8>, String> {
    Ok(vec![PLAY, check("song number", number, 0, NUM_SONGS - 1)?])
}

pub fn sensors(packet_id: u8) -> Result<Vec<u8>, String> {
    packet_range(packet_id)?;
    Ok(vec![SENSORS, packet_id])
}

pub fn query_list(ids: &[u8]) -> Result<Vec<u8>, String> {
    for &pid in ids {
        packet_range(pid)?;
    }
    let mut out = vec![QUERY_LIST, ids.len() as u8];
    out.extend_from_slice(ids);
    Ok(out)
}

pub fn stream(ids: &[u8]) -> Result<Vec<u8>, String> {
    for &pid in ids {
        packet_range(pid)?;
    }
    let mut out = vec![STREAM, ids.len() as u8];
    out.extend_from_slice(ids);
    Ok(out)
}

pub fn pause_resume_stream(resume: bool) -> Vec<u8> {
    vec![PAUSE_RESUME_STREAM, resume as u8]
}

/// Opcode 167. days bitmask (bit 0 = Sunday); times: 7 (hour, minute) pairs, Sunday first.
pub fn schedule(days: i64, times: &[(i64, i64)]) -> Result<Vec<u8>, String> {
    if times.len() != 7 {
        return Err("need 7 (hour, minute) pairs, Sunday first".into());
    }
    let mut data = Vec::with_capacity(14);
    for &(hour, minute) in times {
        data.push(check("hour", hour, 0, 23)?);
        data.push(check("minute", minute, 0, 59)?);
    }
    let mut out = vec![SCHEDULE, check("days", days, 0, 127)?];
    out.extend(data);
    Ok(out)
}

/// Opcode 168. day 0 = Sunday ... 6 = Saturday.
pub fn set_day_time(day: i64, hour: i64, minute: i64) -> Result<Vec<u8>, String> {
    Ok(vec![SET_DAY_TIME, check("day", day, 0, 6)?, check("hour", hour, 0, 23)?, check("minute", minute, 0, 59)?])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    fn be(vals: &[i16]) -> Vec<u8> {
        vals.iter().flat_map(|v| v.to_be_bytes()).collect()
    }

    #[test]
    fn group_sizes_match_spec() {
        let sizes: Vec<_> = [0, 1, 2, 3, 4, 5, 6, 100, 101, 106, 107].iter().map(|&g| packet_size(g).unwrap()).collect();
        assert_eq!(sizes, [26, 10, 6, 10, 14, 12, 52, 80, 28, 12, 9]);
    }

    #[test]
    fn decode_battery_packet_captured_from_robot() {
        let v = decode(3, &hex("043967018e12089a0a88")).unwrap();
        let got: Vec<_> = v.iter().map(|(f, x)| (f.name(), x)).collect();
        assert_eq!(
            got,
            [("charging_state", 4), ("voltage_mv", 14695), ("current_ma", 398), ("temperature_c", 18),
             ("charge_mah", 2202), ("capacity_mah", 2696)]
        );
    }

    #[test]
    fn decode_bumps_and_cliffs_captured_while_lifted() {
        let v = decode(1, &hex("0d000101010100800000")).unwrap();
        assert!(v.flag(F::WheelDropLeft) && v.flag(F::WheelDropRight) && v.flag(F::BumpRight));
        assert!(!v.flag(F::BumpLeft) && !v.flag(F::Wall) && !v.flag(F::VirtualWall));
        assert!(v.flag(F::CliffLeft) && v.flag(F::CliffFrontLeft) && v.flag(F::CliffFrontRight) && v.flag(F::CliffRight));
        for f in [F::OvercurrentLeftWheel, F::OvercurrentRightWheel, F::OvercurrentMainBrush, F::OvercurrentSideBrush] {
            assert!(v.has(f) && !v.flag(f));
        }
    }

    #[test]
    fn decode_group_100_has_every_sensor_key() {
        let v = decode(100, &[0; 80]).unwrap();
        for f in [F::LeftEncoderCounts, F::OiMode, F::Stasis, F::LightBumpRightSignal, F::ButtonClock,
                  F::HomeBaseAvailable, F::SideBrushCurrentMa] {
            assert!(v.has(f));
        }
        assert_eq!(v.len(), 68);
    }

    #[test]
    fn decode_signed_values() {
        let v = decode(3, &hex("0439678000f412089a0a")).unwrap();
        assert_eq!(v.get(F::CurrentMa), Some(-32768));
        assert_eq!(v.get(F::TemperatureC), Some(-12));
        assert_eq!(decode(20, &[0xff, 0xfe]).unwrap().get(F::AngleDeg), Some(-2));
    }

    #[test]
    fn decode_rejects_wrong_length() {
        assert_eq!(decode(3, &[0; 9]).unwrap_err(), "packet 3: expected 10 bytes, got 9");
    }

    #[test]
    fn stasis_uses_bit_zero_only() {
        assert_eq!(decode(58, &[1]).unwrap().get(F::Stasis), Some(1));
        assert_eq!(decode(58, &[2]).unwrap().get(F::Stasis), Some(0));
    }

    #[test]
    fn stream_parser_resyncs_and_splits() {
        let frame = hex("130807012bf9fe2c022568");
        let mut parser = StreamParser::new(&[&[7, 43, 44]]).unwrap();
        let mut first = vec![0x00, 0x13, 0xff];
        first.extend_from_slice(&frame[..5]);
        assert!(parser.feed(&first).is_empty());
        let mut rest = frame[5..].to_vec();
        rest.extend_from_slice(&frame);
        let frames = parser.feed(&rest);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].get(F::LeftEncoderCounts), Some(63998));
        assert_eq!(frames[0].get(F::RightEncoderCounts), Some(549));
        assert!(frames[0].flag(F::BumpRight) && !frames[0].flag(F::BumpLeft));
    }

    #[test]
    fn stream_parser_rejects_bad_checksum() {
        let mut frame = hex("130807012bf9fe2c022568");
        *frame.last_mut().unwrap() ^= 0x01;
        let mut parser = StreamParser::new(&[&[7, 43, 44]]).unwrap();
        assert!(parser.feed(&frame).is_empty());
        assert_eq!(parser.bad_frames, 1);
    }

    fn frame(payload: &[u8]) -> Vec<u8> {
        let mut body = vec![STREAM_HEADER, payload.len() as u8];
        body.extend_from_slice(payload);
        let sum = body.iter().fold(0u8, |a, &b| a.wrapping_add(b));
        body.push(sum.wrapping_neg());
        body
    }

    #[test]
    fn stream_parser_accepts_alternating_layouts() {
        let a = hex("130807012bf9fe2c022568");
        let b = frame(&[35, 2, 58, 1]);
        let mut parser = StreamParser::new(&[&[7, 43, 44], &[35, 58]]).unwrap();
        let data = [a.clone(), b, a].concat();
        let frames = parser.feed_frames(&data);
        assert_eq!(frames.iter().map(|f| f.0).collect::<Vec<_>>(), [8, 4, 8]);
        let got: Vec<_> = frames[1].1.iter().collect();
        assert_eq!(got, [(F::OiMode, 2), (F::Stasis, 1)]);
    }

    #[test]
    fn stream_parser_skips_frames_with_unknown_packets() {
        // Right length and checksum but an id that doesn't exist: counted bad, then resynced.
        let bad = frame(&[99, 0, 0, 0, 0, 0, 0, 0]);
        let good = hex("130807012bf9fe2c022568");
        let mut parser = StreamParser::new(&[&[7, 43, 44]]).unwrap();
        assert_eq!(parser.feed(&[bad, good].concat()).len(), 1);
        assert_eq!(parser.bad_frames, 1);
    }

    #[test]
    fn driver_stream_layouts_cover_every_sensor_and_fit() {
        let ids_of = |layout: &[u8]| -> std::collections::BTreeSet<u8> { layout.iter().flat_map(|&p| packet_ids(p).unwrap()).collect() };
        for name in ["v2", "legacy"] {
            let (primary, secondary) = stream_layouts(name).unwrap();
            let mut covered = std::collections::BTreeSet::new();
            for layout in [primary, secondary] {
                assert!(stream_payload_length(layout).unwrap() + 3 <= 70); // largest frames proven clean in Safe mode
                let ids = ids_of(layout);
                for p in [7, 9, 10, 11, 12, 35, 43, 44] {
                    assert!(ids.contains(&p));
                }
                covered.extend(ids);
            }
            assert_eq!(covered, packet_ids(100).unwrap().into_iter().collect());
            assert_ne!(stream_payload_length(primary), stream_payload_length(secondary));
        }
        // v2: the light-bumper stop (flags, front signals) is checked on every primary frame, not ~5 times a second.
        assert!([45, 47, 48, 49, 50].iter().all(|p| ids_of(&STREAM_PRIMARY).contains(p)));
    }

    #[test]
    fn legacy_stream_layouts_are_the_proven_ones_byte_for_byte() {
        let (primary, secondary) = stream_layouts("legacy").unwrap();
        assert_eq!(stream(primary).unwrap(), [148, 6, 6, 43, 44, 45, 54, 55]);
        assert_eq!(stream(secondary).unwrap(), [148, 14, 7, 9, 10, 11, 12, 35, 43, 44, 106, 52, 53, 56, 57, 58]);
        assert_eq!(stream_layouts("v2").unwrap(), (&STREAM_PRIMARY[..], &STREAM_SECONDARY[..]));
        assert_eq!(stream_layouts("v3").unwrap_err(), "stream_layout must be v2 or legacy, not 'v3'");
    }

    #[test]
    fn drive_direct_sends_right_wheel_first_and_clamps() {
        assert_eq!(drive_direct(-100.0, 200.0), [vec![145], be(&[200, -100])].concat());
        assert_eq!(drive_direct(900.0, -900.0), [vec![145], be(&[-500, 500])].concat());
    }

    #[test]
    fn drive_special_radii_and_clamping() {
        assert_eq!(drive(100.0, RADIUS_STRAIGHT), [vec![137], be(&[100, 32767])].concat());
        assert_eq!(drive(100.0, 32768), [vec![137], be(&[100, 32767])].concat());
        assert_eq!(drive(-900.0, -5000), [vec![137], be(&[-500, -2000])].concat());
        assert_eq!(drive(50.0, RADIUS_TURN_CW), [vec![137], be(&[50, -1])].concat());
    }

    #[test]
    fn rounding_is_half_to_even_like_python() {
        assert_eq!(drive_direct(116.5, 117.5), [vec![145], be(&[118, 116])].concat());
        assert_eq!(clamp(-0.5, -10, 10), 0);
    }

    #[test]
    fn drive_pwm_and_cleaning_motors() {
        assert_eq!(drive_pwm(-300.0, 100.0), [vec![146], be(&[100, -255])].concat());
        assert_eq!(motors(true, true, true, true, true), [138, 0b11111]);
        assert_eq!(motors(false, true, false, false, false), [138, 0b00010]);
        assert_eq!(pwm_motors(-200.0, 50.0, 300.0), [144, (-127i8) as u8, 50, 127]);
    }

    #[test]
    fn leds_and_scheduling_leds() {
        assert_eq!(leds(true, false, false, true, 255.0, 128.0), [139, 0b1001, 255, 128]);
        assert_eq!(scheduling_leds(0b1000001, true, false, false, false, true), [162, 0b1000001, 0b10001]);
    }

    #[test]
    fn display() {
        assert_eq!(digit_leds_ascii("HELO"), b"\xa4HELO");
        assert_eq!(digit_leds_ascii("Hi"), b"\xa4Hi  ");
        assert_eq!(digit_leds_ascii("\x01ABCDE"), b"\xa4 ABC");
        assert_eq!(digit_leds_ascii("\u{e9}t\u{e9}"), b"\xa4 t  ");
        assert_eq!(digit_leds_raw(&[0x7F, 0, 1, 0xFF]).unwrap(), [163, 0x7F, 0, 1, 0x7F]);
        assert!(digit_leds_raw(&[1, 2, 3]).is_err());
    }

    #[test]
    fn song_and_play_validation() {
        assert_eq!(song(2, &[(60, 16), (64, 32)]).unwrap(), [140, 2, 2, 60, 16, 64, 32]);
        assert_eq!(play(4).unwrap(), [141, 4]);
        assert_eq!(song(5, &[(60, 16)]).unwrap_err(), "song number must be 0-4, got 5");
        assert_eq!(song(0, &[]).unwrap_err(), "a song needs 1-16 notes, got 0");
        assert!(song(0, &[(60, 1); 17]).is_err());
        assert!(play(5).is_err());
    }

    #[test]
    fn schedule_clock_buttons_baud() {
        let mut want = vec![167, 0b0111110];
        for _ in 0..7 {
            want.extend([9, 30]);
        }
        assert_eq!(schedule(0b0111110, &[(9, 30); 7]).unwrap(), want);
        assert_eq!(set_day_time(3, 14, 5).unwrap(), [168, 3, 14, 5]);
        assert_eq!(buttons(0b101).unwrap(), [165, 5]);
        assert_eq!(baud(115200).unwrap(), [129, 11]);
        assert_eq!(schedule(1, &[(24, 0); 7]).unwrap_err(), "hour must be 0-23, got 24");
        // Like oi.py, the times are checked before the days.
        assert_eq!(schedule(200, &[(24, 0); 7]).unwrap_err(), "hour must be 0-23, got 24");
        assert_eq!(set_day_time(7, 0, 0).unwrap_err(), "day must be 0-6, got 7");
        assert!(buttons(256).is_err());
        assert!(baud(1000).is_err());
    }

    #[test]
    fn stream_and_query_requests() {
        assert_eq!(stream(&[100]).unwrap(), [148, 1, 100]);
        assert_eq!(query_list(&[43, 44]).unwrap(), [149, 2, 43, 44]);
        assert_eq!(pause_resume_stream(false), [150, 0]);
        assert_eq!(stream_payload_length(&[100]).unwrap(), 81);
        assert_eq!(stream(&[99]).unwrap_err(), "unknown sensor packet id 99");
    }

    #[test]
    fn mode_strings() {
        assert_eq!(mode_str(Some(2)), "safe");
        assert_eq!(mode_str(Some(7)), "7");
        assert_eq!(mode_str(None), "None");
        assert_eq!(Field::from_name("bump_right"), Some(F::BumpRight));
        assert_eq!(F::Stasis.name(), "stasis");
    }

    #[test]
    fn encode_is_the_inverse_of_decode() {
        assert_eq!(encode(3, &|pid| match pid {
            21 => 4,
            22 => 14695,
            23 => 398,
            24 => 18,
            25 => 2202,
            _ => 2696,
        })
        .unwrap(), hex("043967018e12089a0a88"));
        let raw = |pid: u8| match pid {
            7 => 0b0110,
            23 => -1500,
            24 => -12,
            43 => 64000,
            47 => 250,
            _ => pid as i32,
        };
        let frame = encode_stream_frame(&STREAM_PRIMARY, &raw).unwrap();
        assert_eq!(frame.len(), 3 + stream_payload_length(&STREAM_PRIMARY).unwrap());
        let mut parser = StreamParser::new(&[&STREAM_PRIMARY]).unwrap();
        let v = parser.feed(&frame).pop().unwrap();
        assert!(v.flag(F::BumpLeft) && v.flag(F::WheelDropRight) && !v.flag(F::BumpRight));
        assert_eq!(v.get(F::CurrentMa), Some(-1500));
        assert_eq!(v.get(F::TemperatureC), Some(-12));
        assert_eq!(v.get(F::LeftEncoderCounts), Some(64000));
        assert_eq!(v.get(F::LightBumpFrontLeftSignal), Some(250));
        assert_eq!(v.get(F::OiMode), Some(35));
    }
}
