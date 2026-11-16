#!/usr/bin/env bash
# CPU and memory of the Rust roomba_driver (roomba_driver_rs) against the Python one, each for DUR s on a
# simulated Roomba streaming at the real rate (tools/fake_roomba.py --scenario steady: a 15 ms frame
# stream with the layout rotation, wheels moving), with cmd_vel at 20 Hz (as while Nav2 drives) and
# subscribers on every output. The parameters of CONFIG (a driver parameter file, e.g. a robot's
# roomba_driver_robot.yaml; default: the driver package's config/roomba_driver.yaml) with the port set
# to the pty. Reports % of one core (utime + stime from /proc/<pid>/stat) and peak RSS (VmHWM).
#   tools/ab_driver_cpu.sh            (DUR=300, IMPLS="py rs", RS=Rust binary, OUT=/data/rs_lanes/driver/cpu,
#                                      WS=install space as for tools/ab_driver.sh)
HERE=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$HERE/../../../install}
source /opt/ros/jazzy/setup.bash; [ -f "$WS/setup.bash" ] && source "$WS/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-61}
DUR=${DUR:-300}; OUT=${OUT:-/data/rs_lanes/driver/cpu}; IMPLS=${IMPLS:-py rs}
RS=${RS:-$(ros2 pkg prefix roomba_driver_rs)/lib/roomba_driver_rs/roomba_driver}
PY=$(ros2 pkg prefix roomba_driver)/lib/roomba_driver/roomba_driver
SRC=${CONFIG:-$(ros2 pkg prefix roomba_driver)/share/roomba_driver/config/roomba_driver.yaml}
HZ=$(getconf CLK_TCK)
cpu() { awk '{print $14 + $15}' /proc/$1/stat; }

for impl in $IMPLS; do
  D=$OUT/$impl; rm -rf "$D"; mkdir -p "$D"
  BIN=$([ "$impl" = rs ] && echo "$RS" || echo "$PY")
  python3 "$HERE/fake_roomba.py" --link "$D/tty" --log /dev/null --t0-file "$D/t0.json" --scenario steady &
  FAKE=$!
  for _ in $(seq 50); do [ -e "$D/tty" ] && break; sleep 0.1; done
  python3 - "$SRC" "$D/tty" "$D/params.yaml" <<'EOF' || { kill $FAKE; exit 1; }
import sys, yaml
src, port, out = sys.argv[1:4]
d = yaml.safe_load(open(src))
d['roomba_driver']['ros__parameters']['port'] = port
assert not port.startswith('/dev/') and port.endswith('/tty'), port
yaml.safe_dump(d, open(out, 'w'))
EOF
  grep -q "port: $D/tty" "$D/params.yaml" || { echo "params.yaml has the wrong port"; kill $FAKE; exit 1; }
  # Load: cmd_vel at 20 Hz and a subscriber on every output (like the EKF, watchdog and web app).
  python3 - "$((DUR + 20))" > "$D/load.log" 2>&1 <<'EOF' &
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import String
from roomba_msgs.msg import SensorState
rclpy.init()
n = Node('ab_driver_cpu_load')
counts = {}
def count(k):
    return lambda m: counts.__setitem__(k, counts.get(k, 0) + 1)
for t, k in ((Odometry, 'wheel/odom'), (JointState, 'joint_states'), (BatteryState, 'battery_state'),
             (SensorState, 'roomba/sensors')):
    n.create_subscription(t, k, count(k), 10)
n.create_subscription(String, 'roomba/mode', count('roomba/mode'), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub = n.create_publisher(Twist, 'cmd_vel', 1)
i = [0]
def tick():
    m = Twist()
    m.linear.x = 0.15 if i[0] // 100 % 2 == 0 else 0.0
    m.angular.z = 0.0 if i[0] // 100 % 2 == 0 else 0.6
    i[0] += 1
    pub.publish(m)
n.create_timer(0.05, tick)
end = time.time() + float(sys.argv[1])
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
print(counts)
EOF
  LOAD=$!
  sleep 3
  "$BIN" --ros-args --params-file "$D/params.yaml" > "$D/driver.log" 2>&1 &
  DRV=$!
  for _ in $(seq 50); do grep -q "starting on" "$D/driver.log" && break; sleep 0.1; done
  if ! grep -q "starting on $D/tty " "$D/driver.log"; then
    kill -KILL $DRV; kill $FAKE $LOAD; echo "driver did not use the pty:"; cat "$D/driver.log"; exit 1
  fi
  sleep 10   # settle (safe mode, discovery)
  C0=$(cpu $DRV); T0=$(date +%s.%N)
  sleep "$DUR"
  C1=$(cpu $DRV); T1=$(date +%s.%N)
  PCT=$(echo "($C1-$C0)/$HZ/($T1-$T0)*100" | bc -l | cut -c1-5)
  RSS=$(awk '/VmHWM/ {printf "%.1f", $2/1024}' /proc/$DRV/status)
  THREADS=$(awk '/Threads/ {print $2}' /proc/$DRV/status)
  kill -TERM $DRV; wait $DRV
  wait $LOAD
  kill -TERM $FAKE; wait $FAKE
  echo "$impl: CPU $PCT % of one core over $DUR s, peak RSS $RSS MB, $THREADS threads; received $(tail -1 "$D/load.log")"
  grep -c WARN "$D/driver.log" | xargs echo "  warnings in the driver log:"
done
