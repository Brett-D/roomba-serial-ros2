"""iRobot Roomba 500-series Open Interface (OI): opcodes, sensor packets and command encoding.

Pure protocol code with no I/O so it can be unit tested without a robot.
Multi-byte values are big-endian. Reference: iRobot Roomba 500 Open Interface Specification.
"""
import struct

# ---------------------------------------------------------------- opcodes
RESET = 7
START = 128
BAUD = 129
CONTROL = 130           # legacy SCI name for passive -> safe
SAFE = 131
FULL = 132
POWER = 133
SPOT = 134
CLEAN = 135
MAX = 136
DRIVE = 137
MOTORS = 138
LEDS = 139
SONG = 140
PLAY = 141
SENSORS = 142
SEEK_DOCK = 143
PWM_MOTORS = 144
DRIVE_DIRECT = 145
DRIVE_PWM = 146
STREAM = 148
QUERY_LIST = 149
PAUSE_RESUME_STREAM = 150
SCHEDULING_LEDS = 162
DIGIT_LEDS_RAW = 163
DIGIT_LEDS_ASCII = 164
BUTTONS = 165
SCHEDULE = 167
SET_DAY_TIME = 168
STOP = 173

MODE_OFF, MODE_PASSIVE, MODE_SAFE, MODE_FULL = 0, 1, 2, 3
MODE_NAMES = {MODE_OFF: 'off', MODE_PASSIVE: 'passive', MODE_SAFE: 'safe', MODE_FULL: 'full'}

BAUD_CODES = {300: 0, 600: 1, 1200: 2, 2400: 3, 4800: 4, 9600: 5, 14400: 6, 19200: 7,
              28800: 8, 38400: 9, 57600: 10, 115200: 11}

MAX_WHEEL_SPEED_MM_S = 500
MAX_RADIUS_MM = 2000
MAX_DRIVE_PWM = 255
MAX_BRUSH_PWM = 127
MAX_VACUUM_PWM = 127
RADIUS_STRAIGHT = 32767
RADIUS_TURN_CW = -1
RADIUS_TURN_CCW = 1
NUM_SONGS = 5
MAX_SONG_NOTES = 16

STREAM_HEADER = 19
STREAM_PERIOD_S = 0.015

CHARGING_STATES = {0: 'not charging', 1: 'reconditioning', 2: 'full charging',
                   3: 'trickle charging', 4: 'waiting', 5: 'charging fault'}

# ---------------------------------------------------------------- sensor packets
# id: (key, struct format). Bit-field and boolean packets are expanded by _BITS / _BOOLS.
PACKETS = {
    7: ('bumps_wheel_drops', 'B'),
    8: ('wall', 'B'),
    9: ('cliff_left', 'B'),
    10: ('cliff_front_left', 'B'),
    11: ('cliff_front_right', 'B'),
    12: ('cliff_right', 'B'),
    13: ('virtual_wall', 'B'),
    14: ('overcurrents', 'B'),
    15: ('dirt_detect', 'B'),
    16: (None, 'B'),
    17: ('ir_omni', 'B'),
    18: ('buttons', 'B'),
    19: ('distance_mm', 'h'),
    20: ('angle_deg', 'h'),
    21: ('charging_state', 'B'),
    22: ('voltage_mv', 'H'),
    23: ('current_ma', 'h'),
    24: ('temperature_c', 'b'),
    25: ('charge_mah', 'H'),
    26: ('capacity_mah', 'H'),
    27: ('wall_signal', 'H'),
    28: ('cliff_left_signal', 'H'),
    29: ('cliff_front_left_signal', 'H'),
    30: ('cliff_front_right_signal', 'H'),
    31: ('cliff_right_signal', 'H'),
    32: (None, 'B'),
    33: (None, 'H'),
    34: ('charging_sources', 'B'),
    35: ('oi_mode', 'B'),
    36: ('song_number', 'B'),
    37: ('song_playing', 'B'),
    38: ('stream_packet_count', 'B'),
    39: ('requested_velocity_mm_s', 'h'),
    40: ('requested_radius_mm', 'h'),
    41: ('requested_right_velocity_mm_s', 'h'),
    42: ('requested_left_velocity_mm_s', 'h'),
    43: ('left_encoder_counts', 'H'),
    44: ('right_encoder_counts', 'H'),
    45: ('light_bumper', 'B'),
    46: ('light_bump_left_signal', 'H'),
    47: ('light_bump_front_left_signal', 'H'),
    48: ('light_bump_center_left_signal', 'H'),
    49: ('light_bump_center_right_signal', 'H'),
    50: ('light_bump_front_right_signal', 'H'),
    51: ('light_bump_right_signal', 'H'),
    52: ('ir_left', 'B'),
    53: ('ir_right', 'B'),
    54: ('left_motor_current_ma', 'h'),
    55: ('right_motor_current_ma', 'h'),
    56: ('main_brush_current_ma', 'h'),
    57: ('side_brush_current_ma', 'h'),
    58: ('stasis', 'B'),
}

_BITS = {
    7: ((0, 'bump_right'), (1, 'bump_left'), (2, 'wheel_drop_right'), (3, 'wheel_drop_left')),
    14: ((0, 'overcurrent_side_brush'), (2, 'overcurrent_main_brush'),
         (3, 'overcurrent_right_wheel'), (4, 'overcurrent_left_wheel')),
    18: ((0, 'button_clean'), (1, 'button_spot'), (2, 'button_dock'), (3, 'button_minute'),
         (4, 'button_hour'), (5, 'button_day'), (6, 'button_schedule'), (7, 'button_clock')),
    34: ((0, 'internal_charger_available'), (1, 'home_base_available')),
    45: ((0, 'light_bump_left'), (1, 'light_bump_front_left'), (2, 'light_bump_center_left'),
         (3, 'light_bump_center_right'), (4, 'light_bump_front_right'), (5, 'light_bump_right')),
}
_BOOLS = {8, 9, 10, 11, 12, 13, 37}
_MASKED = {58: 0x01}   # stasis: bit 0 = caster toggling; bit 1 = "stasis disabled" on some firmware

GROUPS = {
    0: range(7, 27),
    1: range(7, 17),
    2: range(17, 21),
    3: range(21, 27),
    4: range(27, 35),
    5: range(35, 43),
    6: range(7, 43),
    100: range(7, 59),
    101: range(43, 59),
    106: range(46, 52),
    107: range(54, 59),
}


def packet_ids(packet_id):
    """Individual packet ids contained in a packet or group id."""
    if packet_id in GROUPS:
        return list(GROUPS[packet_id])
    if packet_id in PACKETS:
        return [packet_id]
    raise ValueError(f'unknown sensor packet id {packet_id}')


def packet_size(packet_id):
    """Number of data bytes the robot returns for a packet or group id."""
    return sum(struct.calcsize('>' + PACKETS[i][1]) for i in packet_ids(packet_id))


def decode(packet_id, data):
    """Decode the data bytes of one packet or group into a dict of named values."""
    if len(data) != packet_size(packet_id):
        raise ValueError(f'packet {packet_id}: expected {packet_size(packet_id)} bytes, got {len(data)}')
    out = {}
    offset = 0
    for pid in packet_ids(packet_id):
        key, fmt = PACKETS[pid]
        (value,) = struct.unpack_from('>' + fmt, data, offset)
        offset += struct.calcsize('>' + fmt)
        if pid in _BITS:
            for bit, name in _BITS[pid]:
                out[name] = bool(value >> bit & 1)
        elif pid in _BOOLS:
            out[key] = bool(value)
        elif pid in _MASKED:
            out[key] = bool(value & _MASKED[pid])
        elif key is not None:
            out[key] = value
    return out


def decode_stream_payload(payload):
    """Decode the [id][data][id][data]... body of a stream frame."""
    out = {}
    i = 0
    while i < len(payload):
        pid = payload[i]
        size = packet_size(pid)
        if i + 1 + size > len(payload):
            raise ValueError('truncated stream payload')
        out.update(decode(pid, payload[i + 1:i + 1 + size]))
        i += 1 + size
    return out


def stream_payload_length(ids):
    """Value of the stream frame's byte-count field for a list of requested ids."""
    return sum(1 + packet_size(pid) for pid in ids)


# Streaming every sensor in one frame (group 100, 84 bytes) makes the robot send frames with bad
# checksums in Safe/Full mode, so the driver streams a ~70-byte primary set every 15 ms and swaps
# in the remaining sensors (with the safety and odometry packets) a few times a second.
STREAM_PRIMARY = [6, 43, 44, 45, 54, 55]
STREAM_SECONDARY = [7, 9, 10, 11, 12, 35, 43, 44, 106, 52, 53, 56, 57, 58]
MAX_STREAM_FRAME = 172   # bytes that fit in 15 ms at 115200 baud


class StreamParser:
    """Re-synchronising parser for opcode-148 frames: [19][n][id][data]...[checksum].

    Accepts frames for any of the given id lists (the robot can be switched between them).
    """

    def __init__(self, *id_lists):
        self.expected = {stream_payload_length(ids) for ids in id_lists}
        self.bad_frames = 0
        self._buf = bytearray()

    def feed(self, data):
        """Add received bytes; return a list of decoded frames (dicts)."""
        return [values for _n, values in self.feed_frames(data)]

    def feed_frames(self, data):
        """Like feed(), but returns (n, values) so callers can tell which layout arrived."""
        self._buf += data
        frames = []
        while True:
            start = self._buf.find(STREAM_HEADER)
            if start < 0:
                self._buf.clear()
                break
            if start:
                del self._buf[:start]
            if len(self._buf) < 2:
                break
            n = self._buf[1]
            if n not in self.expected:
                del self._buf[:1]
                continue
            if len(self._buf) < n + 3:
                break
            frame = bytes(self._buf[:n + 3])
            if sum(frame) & 0xFF == 0:
                try:
                    frames.append((n, decode_stream_payload(frame[2:-1])))
                    del self._buf[:n + 3]
                    continue
                except ValueError:
                    pass
            self.bad_frames += 1
            del self._buf[:1]
        return frames


# ---------------------------------------------------------------- command encoding
def _clamp(value, lo, hi):
    return max(lo, min(hi, int(round(value))))


def _check(name, value, lo, hi):
    if not lo <= int(value) <= hi:
        raise ValueError(f'{name} must be {lo}-{hi}, got {value}')
    return int(value)


def cmd(opcode, *data):
    return bytes([opcode, *data])


def baud(rate):
    if rate not in BAUD_CODES:
        raise ValueError(f'unsupported baud rate {rate}')
    return cmd(BAUD, BAUD_CODES[rate])


def drive(velocity_mm_s, radius_mm):
    """Opcode 137. Velocity is clamped to +-500 mm/s, radius to +-2000 mm (special values kept)."""
    v = _clamp(velocity_mm_s, -MAX_WHEEL_SPEED_MM_S, MAX_WHEEL_SPEED_MM_S)
    r = int(radius_mm)
    if r in (RADIUS_STRAIGHT, 32768, -32768):
        r = RADIUS_STRAIGHT
    elif r not in (RADIUS_TURN_CW, RADIUS_TURN_CCW):
        r = _clamp(r, -MAX_RADIUS_MM, MAX_RADIUS_MM)
    return bytes([DRIVE]) + struct.pack('>hh', v, r)


def drive_direct(left_mm_s, right_mm_s):
    """Opcode 145. The robot expects the RIGHT wheel first."""
    left = _clamp(left_mm_s, -MAX_WHEEL_SPEED_MM_S, MAX_WHEEL_SPEED_MM_S)
    right = _clamp(right_mm_s, -MAX_WHEEL_SPEED_MM_S, MAX_WHEEL_SPEED_MM_S)
    return bytes([DRIVE_DIRECT]) + struct.pack('>hh', right, left)


def drive_pwm(left_pwm, right_pwm):
    """Opcode 146. Right wheel first, -255..255."""
    left = _clamp(left_pwm, -MAX_DRIVE_PWM, MAX_DRIVE_PWM)
    right = _clamp(right_pwm, -MAX_DRIVE_PWM, MAX_DRIVE_PWM)
    return bytes([DRIVE_PWM]) + struct.pack('>hh', right, left)


def motors(side_brush=False, vacuum=False, main_brush=False,
           side_brush_clockwise=False, main_brush_outward=False):
    """Opcode 138: cleaning motors on/off at full speed."""
    bits = (side_brush << 0 | vacuum << 1 | main_brush << 2
            | side_brush_clockwise << 3 | main_brush_outward << 4)
    return cmd(MOTORS, bits)


def pwm_motors(main_brush=0, side_brush=0, vacuum=0):
    """Opcode 144: cleaning motor duty cycles."""
    return bytes([PWM_MOTORS]) + struct.pack(
        '>bbB',
        _clamp(main_brush, -MAX_BRUSH_PWM, MAX_BRUSH_PWM),
        _clamp(side_brush, -MAX_BRUSH_PWM, MAX_BRUSH_PWM),
        _clamp(vacuum, 0, MAX_VACUUM_PWM))


def leds(debris=False, spot=False, dock=False, check_robot=False, power_color=0, power_intensity=0):
    """Opcode 139. power_color 0 = green ... 255 = red."""
    bits = debris << 0 | spot << 1 | dock << 2 | check_robot << 3
    return cmd(LEDS, bits, _clamp(power_color, 0, 255), _clamp(power_intensity, 0, 255))


def scheduling_leds(weekdays=0, schedule=False, clock=False, am=False, pm=False, colon=False):
    """Opcode 162. weekdays: bit 0 = Sunday ... bit 6 = Saturday."""
    bits = colon << 0 | pm << 1 | am << 2 | clock << 3 | schedule << 4
    return cmd(SCHEDULING_LEDS, int(weekdays) & 0x7F, bits)


def digit_leds_raw(segments):
    """Opcode 163. Four segment bitmasks (bit 0 = A ... bit 6 = G), leftmost digit first."""
    segs = list(segments)
    if len(segs) != 4:
        raise ValueError('need exactly 4 digits')
    return cmd(DIGIT_LEDS_RAW, *(int(s) & 0x7F for s in segs))


def digit_leds_ascii(text):
    """Opcode 164. First four characters, padded with spaces; unprintable characters become spaces."""
    chars = [c if 32 <= ord(c) <= 126 else ' ' for c in str(text)[:4]]
    return bytes([DIGIT_LEDS_ASCII]) + ''.join(chars).ljust(4).encode('ascii')


def buttons(mask):
    """Opcode 165. bit 0 Clean, 1 Spot, 2 Dock, 3 Minute, 4 Hour, 5 Day, 6 Schedule, 7 Clock."""
    return cmd(BUTTONS, _check('buttons', mask, 0, 255))


def song(number, notes):
    """Opcode 140. notes: sequence of (midi_note, duration_64ths)."""
    _check('song number', number, 0, NUM_SONGS - 1)
    notes = list(notes)
    if not 1 <= len(notes) <= MAX_SONG_NOTES:
        raise ValueError(f'a song needs 1-{MAX_SONG_NOTES} notes, got {len(notes)}')
    data = []
    for note, duration in notes:
        data += [_check('note', note, 0, 255), _check('duration', duration, 0, 255)]
    return cmd(SONG, int(number), len(notes), *data)


def play(number):
    return cmd(PLAY, _check('song number', number, 0, NUM_SONGS - 1))


def sensors(packet_id):
    packet_ids(packet_id)
    return cmd(SENSORS, packet_id)


def query_list(ids):
    ids = list(ids)
    for pid in ids:
        packet_ids(pid)
    return cmd(QUERY_LIST, len(ids), *ids)


def stream(ids):
    ids = list(ids)
    for pid in ids:
        packet_ids(pid)
    return cmd(STREAM, len(ids), *ids)


def pause_resume_stream(resume):
    return cmd(PAUSE_RESUME_STREAM, 1 if resume else 0)


def schedule(days, times):
    """Opcode 167. days bitmask (bit 0 = Sunday); times: 7 (hour, minute) pairs, Sunday first."""
    times = list(times)
    if len(times) != 7:
        raise ValueError('need 7 (hour, minute) pairs, Sunday first')
    data = []
    for hour, minute in times:
        data += [_check('hour', hour, 0, 23), _check('minute', minute, 0, 59)]
    return cmd(SCHEDULE, _check('days', days, 0, 127), *data)


def set_day_time(day, hour, minute):
    """Opcode 168. day 0 = Sunday ... 6 = Saturday."""
    return cmd(SET_DAY_TIME, _check('day', day, 0, 6), _check('hour', hour, 0, 23),
               _check('minute', minute, 0, 59))
