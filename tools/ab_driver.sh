#!/usr/bin/env bash
# A/B of the Rust roomba_driver (roomba_driver_rs) against the Python one (roomba_driver) on a simulated
# Roomba: for each driver, tools/fake_roomba.py opens a pty and plays the scripted sensor scenario,
# tools/ab_driver_harness.py sends the same topics and service calls at the same times after the
# first stream request and records the outputs, then sends SIGTERM to check the shutdown bytes.
# tools/ab_driver_compare.py then compares the two runs. Never touches the real robot's port: the
# driver gets a generated parameter file whose port is the pty.
#   tools/ab_driver.sh                    CONFIG=default: the driver package's defaults, plus publish_tf: true
#   CONFIG=<file.yaml> tools/ab_driver.sh a driver parameter file, e.g. a robot's (roomba-jetson-stack:
#                                         roomba_bringup/config/roomba_driver_robot.yaml); output in
#                                         <scenario>_<file name>
#   SCENARIO=replug tools/ab_driver.sh    the port appears late, then is unplugged for 2.5 s (reconnects)
#   SCENARIO=lightbump tools/ab_driver.sh 16 light-bumper hits while driving at 0.25 m/s; the compare
#                                         reports each stop's latency and the distance driven meanwhile
#   LAYOUT=legacy tools/ab_driver.sh      with stream_layout: legacy (output in <scenario>_<config>_legacy)
# env: OUT (/data/rs_lanes/driver/ab), RS (Rust binary), IMPLS ("py rs"), ROS_DOMAIN_ID (61),
#      WS (install space with the drivers; default: that of the colcon workspace this repository is in,
#      <ws>/src/roomba-serial-ros2; without one the environment must already have both drivers)
HERE=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$HERE/../../../install}
source /opt/ros/jazzy/setup.bash; [ -f "$WS/setup.bash" ] && source "$WS/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-61}
CONFIG=${CONFIG:-default}; OUT=${OUT:-/data/rs_lanes/driver/ab}; IMPLS=${IMPLS:-py rs}; SCENARIO=${SCENARIO:-ab}
case $SCENARIO in replug) END=20 ;; lightbump) END=42.5 ;; *) END=70.5 ;; esac
RS=${RS:-$(ros2 pkg prefix roomba_driver_rs)/lib/roomba_driver_rs/roomba_driver}
PY=$(ros2 pkg prefix roomba_driver)/lib/roomba_driver/roomba_driver
case $CONFIG in
  default) SRC=$(ros2 pkg prefix roomba_driver)/share/roomba_driver/config/roomba_driver.yaml; EXTRA=publish_tf=true; NAME=default ;;
  # e.g. the robot's: its bump reflex reverses 0.08 m at 0.1 m/s = 0.8 s, which ends exactly on a 50 ms
  # supervisor tick. Both drivers count the reflex steps in supervisor ticks, so timer jitter can't add a
  # reverse command.
  *) [ -f "$CONFIG" ] || { echo "CONFIG must be default or a driver parameter file"; exit 1; }
     SRC=$(realpath "$CONFIG"); EXTRA=; NAME=$(basename "$CONFIG" .yaml) ;;
esac
[ -n "$LAYOUT" ] && EXTRA="${EXTRA:+$EXTRA,}stream_layout=$LAYOUT"
RUN=$OUT/${SCENARIO}_$NAME${LAYOUT:+_$LAYOUT}

for impl in $IMPLS; do
  D=$RUN/$impl; rm -rf "$D"; mkdir -p "$D"
  BIN=$([ "$impl" = rs ] && echo "$RS" || echo "$PY")
  python3 "$HERE/fake_roomba.py" --link "$D/tty" --log "$D/serial.jsonl" --t0-file "$D/t0.json" --scenario "$SCENARIO" &
  FAKE=$!
  # (replug: the port appears only after 6 s, so the driver starts before it exists)
  [ "$SCENARIO" = replug ] || for _ in $(seq 50); do [ -e "$D/tty" ] && break; sleep 0.1; done
  # The robot's YAML names the real adapter: write a copy whose port is the pty.
  python3 - "$SRC" "$D/tty" "$D/params.yaml" "$EXTRA" <<'EOF' || { kill $FAKE; exit 1; }
import sys, yaml
src, port, out, extra = sys.argv[1:5]
d = yaml.safe_load(open(src))
p = d['roomba_driver']['ros__parameters']
p['port'] = port
for kv in filter(None, extra.split(',')):
    k, v = kv.split('=')
    p[k] = yaml.safe_load(v)
assert not port.startswith('/dev/') and port.endswith('/tty'), port
yaml.safe_dump(d, open(out, 'w'))
EOF
  grep -q "port: $D/tty" "$D/params.yaml" || { echo "params.yaml has the wrong port"; kill $FAKE; exit 1; }
  python3 "$HERE/ab_driver_harness.py" --t0-file "$D/t0.json" --out "$D/harness.json" --driver-pid-file "$D/driver.pid" \
    --scenario "$SCENARIO" --end "$END" &
  HARNESS=$!
  sleep 3   # harness subscribed before the driver publishes its first mode
  "$BIN" --ros-args --params-file "$D/params.yaml" > "$D/driver.log" 2>&1 &
  DRV=$!; echo $DRV > "$D/driver.pid"
  for _ in $(seq 50); do grep -q "starting on" "$D/driver.log" && break; sleep 0.1; done
  if ! grep -q "starting on $D/tty " "$D/driver.log"; then
    kill -KILL $DRV; kill $FAKE $HARNESS; echo "driver did not use the pty:"; cat "$D/driver.log"; exit 1
  fi
  echo "$impl: running $BIN (pid $DRV) against $(readlink "$D/tty")"
  wait $HARNESS || { echo "$impl: harness failed"; kill -TERM $DRV; }
  for _ in $(seq 50); do kill -0 $DRV 2>/dev/null || break; sleep 0.1; done
  kill -0 $DRV 2>/dev/null && { echo "$impl: driver still running 5 s after SIGTERM; killing"; kill -KILL $DRV; }
  wait $DRV; echo "$impl: driver exit code $?"
  sleep 0.5; kill -TERM $FAKE; wait $FAKE
done
python3 "$HERE/ab_driver_compare.py" "$RUN/py" "$RUN/rs"
