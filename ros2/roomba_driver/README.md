# roomba_driver

ROS 2 (Jazzy) driver for iRobot Roomba 500-series robots over the Open Interface (OI),
using a 5 V USB-TTL adapter on the robot's 7-pin Mini-DIN port. Interfaces are in `roomba_msgs`.

```bash
colcon build --packages-select roomba_msgs roomba_driver
ros2 launch roomba_driver roomba_driver.launch.py
```

## Wiring

| Adapter (jumper on **5V**) | Roomba Mini-DIN |
|---|---|
| TXD | pin 3 (RXD) |
| RXD | pin 4 (TXD) |
| GND | pin 6 or 7 |
| DTR (optional, enables `wake_with_dtr`) | pin 5 (BRC) |
| VCC | not connected |

Never connect pins 1-2 (raw battery, 14-17 V).

## OI modes

| Mode | Behaviour |
|---|---|
| passive | Sensors only. The robot's buttons work. Cleaning, dock and button services work. **The robot sleeps after 5 minutes idle** unless it's docked or `wake_with_dtr` is on; while it's asleep, `roomba/mode` reads `unknown`. |
| safe | Full control. The robot drops back to passive on a cliff, wheel drop, or when on the charger. Safe can't be entered while the wheels are lifted. |
| full | Full control with **no** cliff or wheel-drop protection. |

The driver enters `initial_mode` (default `safe`) at startup. Drive, LED, display and song
commands are ignored with a warning unless the robot is in safe or full mode. If the robot drops
to passive *without* a safety reason (no cliff, wheel drop or charger; e.g. the robot reset), the
driver re-enters the requested mode by itself. After a safety drop it stays in passive until
`roomba/set_mode` is called (or `auto_reenter_mode` is set). Each drive input
stops the wheels if no new command arrives within `cmd_vel_timeout_s`. On shutdown the driver
stops the motors and returns the robot to passive.

## Published topics

| Topic | Type | Notes |
|---|---|---|
| `wheel/odom` | nav_msgs/Odometry | From wheel encoders (the firmware's distance/angle packets are unreliable). No TF unless `publish_tf`. |
| `joint_states` | sensor_msgs/JointState | Wheel angles (rad) and velocities. |
| `battery_state` | sensor_msgs/BatteryState | Voltage, current, charge, capacity, temperature, charging status. |
| `roomba/sensors` | roomba_msgs/SensorState | Every sensor: bumps, wheel drops, cliffs and signals, walls, light bumper, IR characters, buttons, dirt, motor currents, overcurrents, encoders, stasis, mode, song status, requested velocities. |
| `roomba/mode` | std_msgs/String (latched) | `off`, `passive`, `safe`, `full`, or `unknown` (no sensor data for 1 s), on change. |

## Subscribed topics

Every topic in this table needs safe or full mode.

| Topic | Type | OI opcode |
|---|---|---|
| `cmd_vel` | geometry_msgs/Twist (or TwistStamped) | 145 Drive Direct |
| `roomba/drive` | roomba_msgs/Drive | 137 Drive (velocity + turn radius) |
| `roomba/drive_direct` | roomba_msgs/DriveDirect | 145 Drive Direct (wheel speeds) |
| `roomba/drive_pwm` | roomba_msgs/DrivePwm | 146 Drive PWM |
| `roomba/cleaning_motors` | roomba_msgs/CleaningMotors | 138 Motors / 144 PWM Motors |
| `roomba/leds` | roomba_msgs/Leds | 139 LEDs |
| `roomba/scheduling_leds` | roomba_msgs/SchedulingLeds | 162 Scheduling LEDs |
| `roomba/display` | std_msgs/String | 164 Digit LEDs ASCII (text longer than 4 characters scrolls) |
| `roomba/display_raw` | std_msgs/UInt8MultiArray | 163 Digit LEDs Raw (4 segment bitmasks, leftmost first) |

Drive speeds are limited to `max_wheel_speed_m_s`.

## Services

| Service | Type | OI opcode |
|---|---|---|
| `roomba/set_mode` | roomba_msgs/SetMode | 128 / 131 / 132 / 173. Reports the resulting mode, or why it was refused. |
| `roomba/clean`, `roomba/spot`, `roomba/max_clean` | std_srvs/Trigger | 135 / 134 / 136 (robot switches to passive) |
| `roomba/seek_dock` | std_srvs/Trigger | 143 Seek Dock |
| `roomba/power_off` | std_srvs/Trigger | 133 Power |
| `roomba/reset` | std_srvs/Trigger | 7 Reset (reboots the robot) |
| `roomba/wake` | std_srvs/Trigger | Pulses BRC through DTR (needs the DTR wire) |
| `roomba/define_song` | roomba_msgs/DefineSong | 140 Song (+141 if `play`) |
| `roomba/play_song` | roomba_msgs/PlaySong | 141 Play |
| `roomba/press_buttons` | roomba_msgs/PressButtons | 165 Buttons |
| `roomba/set_schedule` | roomba_msgs/SetSchedule | 167 Schedule |
| `roomba/set_day_time` | roomba_msgs/SetDayTime | 168 Set Day/Time |

The sensor stream (148/150), sensor polling (142/149) and the baud rate (129) are handled by the
driver. Polling and baud changes are also available in the Python library. Streaming every sensor
in one 84-byte frame produces bad checksums in Safe mode, so the driver streams a 70-byte primary
set every 15 ms and swaps in the remaining sensors about 5 times a second. Every frame includes
bumps, cliffs, mode and encoders.

## Examples

```bash
ros2 service call /roomba/set_mode roomba_msgs/srv/SetMode "{mode: 2}"
ros2 topic pub --once /roomba/display std_msgs/msg/String "{data: 'HELLO'}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"     # Ctrl-C stops it
ros2 service call /roomba/define_song roomba_msgs/srv/DefineSong \
  "{song_number: 0, notes: [72, 76, 79, 84], durations: [12, 12, 12, 24], play: true}"
ros2 topic pub --once /roomba/cleaning_motors roomba_msgs/msg/CleaningMotors \
  "{use_pwm: true, vacuum_pwm: 60}"
ros2 service call /roomba/seek_dock std_srvs/srv/Trigger
ros2 topic echo /roomba/sensors --field bump_left
```

## Python library (no ROS)

```python
from roomba_driver.roomba import Roomba
with Roomba('/dev/ttyUSB0') as robot:
    robot.start()
    print(robot.sensors(3))                       # battery packet
    robot.set_mode(2)                             # safe
    robot.display_text('HI')
    robot.start_stream(callback=lambda values, t: print(values['left_encoder_counts']))
```

## Parameters

See `config/roomba_driver.yaml`. Important ones:

| Parameter | Default | Meaning |
|---|---|---|
| `port` | /dev/ttyUSB0 | Serial device. Use the `/dev/serial/by-id/...` path. |
| `initial_mode` | safe | Mode entered at startup. |
| `auto_reenter_mode` | false | Re-enter `initial_mode` automatically when a motion command arrives after the robot dropped to passive. |
| `publish_tf` | false | Broadcast `odom_frame -> base_frame`. Leave off while VSLAM owns the `odom` TF. |
| `max_wheel_speed_m_s` | 0.3 | Per-wheel speed limit (hardware maximum 0.5). |
| `cmd_vel_timeout_s` | 0.5 | Stop the wheels if no drive command arrives for this long. |
| `bump_stop` | true | Stop when a bumper is pressed while driving forward, and refuse forward commands until it's released. Reversing and turning in place still work, so Nav2's back-up recovery can get free. |
| `wheel_base_m`, `wheel_diameter_m`, `encoder_counts_per_rev` | 0.235, 0.072, 508.8 | Odometry geometry. |
| `wake_with_dtr`, `keep_awake_period_s` | false, 60 | Wake the robot, and keep it awake in passive, with 1 s low pulses on BRC through DTR. Pulses of 50–500 ms would change the robot's baud rate. |

## Tests

```bash
cd ros2/roomba_driver && python3 -m pytest test
```
