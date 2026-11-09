# Roomba 500-series Open Interface: quick reference

A summary of the serial protocol as implemented in
[`roomba_driver/oi.py`](../ros2/roomba_driver/roomba_driver/oi.py) and checked against a Roomba 550.
For the full definitions, see iRobot's *Roomba 500 Open Interface Specification*, linked in the
[README](../README.md#references).

## Serial settings

| Setting | Value |
|---|---|
| Baud | **115200** (500 series default). The older SCI manual says 57600; that is for pre-500 robots. |
| Format | 8 data bits, no parity, 1 stop bit, no flow control |
| Levels | 0–5 V TTL. Not RS-232. |
| Byte order | Multi-byte values are big-endian (high byte first) |

## Modes

| Mode | Enter with | Notes |
|---|---|---|
| Off | power-up, or 173 Stop | Listens only for Start. |
| Passive | 128 Start (or any cleaning command) | Read sensors, run cleaning/dock behaviours, define songs. No actuator control. **Sleeps after 5 min idle**, unless a 1 s low pulse on BRC arrives every minute. |
| Safe | 131 Safe | Full control. Drops to passive on a cliff (moving forward), wheel drop, or charger. **Refused while the wheels are lifted.** Doesn't sleep. |
| Full | 132 Full | Full control with no cliff, wheel-drop or charger protection. |

130 (Control, from the older SCI spec) also takes the robot from passive to safe: a song played
after 128 → 130, and playing needs Safe mode. Packet 35 reports the current mode.

## Commands

| Opcode | Name | Data bytes | Mode |
|---|---|---|---|
| 7 | Reset | – | any |
| 128 | Start | – | any |
| 129 | Baud | code 0–11 (11 = 115200). Wait 100 ms afterwards. | P/S/F |
| 130 / 131 / 132 | Control / Safe / Full | – | P/S/F |
| 133 | Power (sleep) | – | P/S/F |
| 134 / 135 / 136 | Spot / Clean / Max | – | P/S/F, robot goes to passive |
| 143 | Seek Dock | – | P/S/F |
| 137 | Drive | velocity (−500…500 mm/s), radius (−2000…2000 mm; 32767 = straight, −1/1 = spin cw/ccw), int16 each | S/F |
| 145 | Drive Direct | **right** velocity, **left** velocity, int16 mm/s each | S/F |
| 146 | Drive PWM | **right** PWM, **left** PWM, int16 −255…255 each | S/F |
| 138 | Motors | bits: 0 side brush, 1 vacuum, 2 main brush, 3 side brush clockwise, 4 main brush outward | S/F |
| 144 | PWM Motors | main brush (−127…127), side brush (−127…127), vacuum (0…127) | S/F |
| 139 | LEDs | bits (0 debris, 1 spot, 2 dock, 3 check robot), power colour (0 green…255 red), power intensity | S/F |
| 162 | Scheduling LEDs | weekday bits (0 = Sunday), bits (0 colon, 1 PM, 2 AM, 3 clock, 4 schedule) | S/F |
| 163 | Digit LEDs Raw | 4 segment masks, leftmost digit first (bit 0 = A … 6 = G) | S/F |
| 164 | Digit LEDs ASCII | 4 ASCII characters (32–126), leftmost first. **Works on the 550.** | S/F |
| 165 | Buttons | bits: 0 clean, 1 spot, 2 dock, 3 minute, 4 hour, 5 day, 6 schedule, 7 clock | P/S/F |
| 140 | Song | song number (0–4), length (1–16), then note (MIDI 31–127) and duration (1/64 s) pairs | P/S/F |
| 141 | Play | song number | S/F |
| 142 | Sensors | packet id | P/S/F |
| 149 | Query List | count, ids… | P/S/F |
| 148 | Stream | count, ids…; the robot then sends a frame every 15 ms | P/S/F |
| 150 | Pause/Resume Stream | 0 / 1 | P/S/F |
| 167 | Schedule | day bits, then 7 × (hour, minute), Sunday first | P/S/F |
| 168 | Set Day/Time | day (0 = Sunday), hour, minute | P/S/F |
| 173 | Stop | – (not on every 500 firmware) | any |

## Sensor packets

| Id | Name | Bytes | Notes |
|---|---|---|---|
| 7 | Bumps & wheel drops | 1 | bits: 0 bump right, 1 bump left, 2 wheel drop right, 3 wheel drop left |
| 8–13 | Wall, cliff L / FL / FR / R, virtual wall | 1 each | 0/1 |
| 14 | Overcurrents | 1 | bits: 0 side brush, 2 main brush, 3 right wheel, 4 left wheel |
| 15 | Dirt detect | 1 | 0–255 |
| 17 / 52 / 53 | IR character omni / left / right | 1 | dock beams 160–173, virtual wall 162 |
| 18 | Buttons | 1 | same bits as opcode 165 |
| 19 / 20 | Distance (mm) / angle (deg) since last read | 2 signed | **Unreliable on some 500 firmware**: use the encoders |
| 21 | Charging state | 1 | 0 none, 1 reconditioning, 2 full, 3 trickle, 4 waiting, 5 fault |
| 22 / 23 | Voltage (mV) / current (mA, negative = discharging) | 2 | |
| 24 | Battery temperature (°C) | 1 signed | |
| 25 / 26 | Charge / capacity (mAh) | 2 | |
| 27–31 | Wall and cliff signal strengths | 2 each | 0–4095 |
| 34 | Charging sources | 1 | bit 0 internal charger, bit 1 home base |
| 35 | OI mode | 1 | 0 off, 1 passive, 2 safe, 3 full |
| 36 / 37 / 38 | Song number / song playing / stream packet count | 1 | |
| 39–42 | Requested velocity, radius, right velocity, left velocity | 2 signed | what the robot is currently executing |
| 43 / 44 | Left / right encoder counts | 2 | wraps at 65536. **Counts down in reverse.** 508.8 counts per wheel turn, 72 mm wheels (0.445 mm per count) |
| 45 | Light bumper | 1 | bits 0–5: left, front-left, centre-left, centre-right, front-right, right |
| 46–51 | Light bumper signals | 2 each | 0–4095 |
| 54–57 | Left wheel, right wheel, main brush, side brush current (mA) | 2 signed | |
| 58 | Stasis | 1 | bit 0: the front caster is turning (robot really is moving) |

Groups: 0 = 7–26, 1 = 7–16, 2 = 17–20, 3 = 21–26, 4 = 27–34, 5 = 35–42, 6 = 7–42,
100 = 7–58 (80 bytes), 101 = 43–58, 106 = 46–51, 107 = 54–58.

## Stream frames

`[19][n][id₁][data₁]…[idₖ][dataₖ][checksum]`, where every byte including the checksum sums to 0 mod 256.

At 115200 baud, a frame must fit in 15 ms (≈172 bytes). **On the Roomba 550, streaming group 100
(84-byte frames) in Safe mode produced about 10% bad checksums; frames of 70 bytes or less were
clean.** The driver therefore streams `[6, 43, 44, 45, 54, 55]` (70 bytes) every 15 ms and swaps in
`[7, 9–12, 35, 43, 44, 106, 52, 53, 56, 57, 58]` (46 bytes) about 5 times a second.

## Robot geometry (500 series)

| | |
|---|---|
| Wheel base | 235 mm |
| Wheel diameter | 72 mm |
| Encoder | 508.8 counts per wheel revolution |
| Max wheel speed | 500 mm/s |
