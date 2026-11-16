# roomba_driver_rs

The Roomba driver in Rust ([rclrs](https://github.com/ros2-rust/ros2_rust) 0.8): a drop-in
replacement for the Python [`roomba_driver`](../roomba_driver) node. The node name (`roomba_driver`),
topics, QoS, services, parameters and their defaults, log lines, reconnect behaviour and shutdown
sequence are the same, and so are the bytes it writes to the serial port. The topics, services,
modes and parameters are documented in the [Python driver's README](../roomba_driver/README.md);
`ros2/roomba_driver/config/roomba_driver.yaml` works for both.

The Python driver stays: it is the reference implementation and the fallback.

## Build

Needs a Rust toolchain ([rustup](https://rustup.rs)), `libclang-dev` (rclrs generates its rcl
bindings with bindgen) and, on aarch64 (e.g. a Jetson), the mold linker: `.cargo/config.toml`
links with mold there because GNU ld takes minutes to link against the ROS libraries. cargo
fetches rclrs; the message crates (`roomba_msgs` included) come from the `share/<package>/rust`
directories on `AMENT_PREFIX_PATH`, so build `roomba_msgs` in the same workspace.

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
sudo apt install -y libclang-dev mold && source ~/.cargo/env
cd ~/ros2_ws && colcon build --packages-select roomba_msgs roomba_driver_rs
source install/setup.bash
```

The package is ament_cmake driving `cargo build --release --locked` (no colcon-ros-cargo needed).
`Cargo.lock` is committed.

## Run

```bash
ros2 run roomba_driver_rs roomba_driver --ros-args \
  --params-file $(ros2 pkg prefix roomba_driver)/share/roomba_driver/config/roomba_driver.yaml
```

or from a launch file: `Node(package='roomba_driver_rs', executable='roomba_driver', name='roomba_driver', parameters=[params_file])`.

Put the port and every other setting in the parameter file: with rclrs, an entry for the node in a
parameter file wins over a `-p` override on the command line.

SIGINT and SIGTERM stop the wheels and the cleaning motors, clear the display, stop the sensor
stream and hand the robot back in Passive mode before the node exits (status 0), as the Python node
does on Ctrl-C.

## Code

| File | Contents |
|---|---|
| `src/oi.rs` | Open Interface: opcodes, sensor packets, stream layouts, command encoding (port of `oi.py`) |
| `src/roomba.rs` | The serial port (termios), the stream reader thread with the layout rotation, stall restarts and reconnects (port of `roomba.py`) |
| `src/driver.rs` | The node's logic without ROS: odometry, mode tracking, bump stop and reflex, supervisor, drive and actuator inputs, service results (port of `node.py`) |
| `src/kinematics.rs` | Differential-drive kinematics and encoder odometry (port of `kinematics.py`) |
| `src/battery.rs` | `battery_state`: the Roomba's own counter or a Li-ion voltage curve (port of `battery.py`) |
| `src/bin/roomba_driver.rs` | The ROS node: parameters, publishers, subscriptions, services, timers, signals |

The library modules (`roomba_driver_rs::{oi, roomba, driver, kinematics, battery}`) use no ROS
types (only the binary does), so other crates can reuse them: the robot stack's 2D digital twin
([roomba-jetson-stack](https://github.com/Brett-D/roomba-jetson-stack), `roomba_rs` twin2d)
emulates the Roomba's Open Interface with `oi` and its battery with `battery`.

## Tests

```bash
colcon test --packages-select roomba_driver_rs && colcon test-result --verbose
# or, with the workspace sourced:
cd ros2/roomba_driver_rs && cargo test --release --locked --lib
```

[`tools/`](../../tools) checks the two drivers against each other on a simulated Roomba
(`tools/fake_roomba.py`, a pseudo-terminal that behaves like the robot's Open Interface and logs
every byte it receives):

```bash
tools/ab_driver.sh                               # serial bytes, topics, services, logs, shutdown bytes
CONFIG=/path/to/roomba_driver_robot.yaml tools/ab_driver.sh   # with a robot's parameters
SCENARIO=replug tools/ab_driver.sh               # the port appears late, then is unplugged (reconnects)
LAYOUT=legacy tools/ab_driver.sh                 # stream_layout: legacy
SCENARIO=lightbump tools/ab_driver.sh            # light-bumper stop latency
tools/ab_driver_cpu.sh                           # CPU and memory of both
```

The scripts take the drivers from the colcon workspace this repository is in (`WS=` to override)
and write under `OUT` (default `/data/rs_lanes/driver/ab`); the drivers only ever get the pty as
their port.
