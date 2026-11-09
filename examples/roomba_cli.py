#!/usr/bin/env python3
"""Talk to a Roomba 500-series over serial without ROS (needs only pyserial).

    python3 roomba_cli.py sensors                 # battery, mode, bumpers, cliffs, encoders...
    python3 roomba_cli.py beep
    python3 roomba_cli.py display HELLO           # scrolls text longer than 4 characters
    python3 roomba_cli.py drive 100 0.5           # 100 mm/s for 0.5 s, then report encoder distance
    python3 roomba_cli.py stop                    # wheels + cleaning motors off, back to passive
    python3 roomba_cli.py --port /dev/ttyUSB1 sensors

The robot must be awake (press Clean if its lights are off). Commands that act on the robot need
Safe mode, which the robot refuses while lifted (wheel drop), over a cliff or on its charger.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros2', 'roomba_driver'))

from roomba_driver import oi  # noqa: E402
from roomba_driver.kinematics import wrap_delta  # noqa: E402
from roomba_driver.roomba import Roomba, RoombaError  # noqa: E402

MM_PER_COUNT = math.pi * 72.0 / 508.8


def enter_safe(robot):
    mode = robot.set_mode(oi.MODE_SAFE)
    if mode != oi.MODE_SAFE:
        v = robot.sensors(100)
        why = [k for k in ('wheel_drop_left', 'wheel_drop_right', 'cliff_left', 'cliff_front_left',
                           'cliff_front_right', 'cliff_right', 'home_base_available',
                           'internal_charger_available') if v[k]]
        sys.exit(f"Roomba refused Safe mode (reports {oi.MODE_NAMES.get(mode, mode)}); "
                 f"active: {', '.join(why) or 'nothing reported'}")


def cmd_sensors(robot, _args):
    v = robot.sensors(100)
    pct = 100.0 * v['charge_mah'] / v['capacity_mah'] if v['capacity_mah'] else float('nan')
    print(f"mode           {oi.MODE_NAMES.get(v['oi_mode'], v['oi_mode'])}")
    print(f"battery        {v['voltage_mv'] / 1000:.2f} V  {v['current_ma']} mA  {v['temperature_c']} C  "
          f"{v['charge_mah']}/{v['capacity_mah']} mAh ({pct:.0f}%)  "
          f"{oi.CHARGING_STATES.get(v['charging_state'], v['charging_state'])}")
    print(f"bumpers        left={v['bump_left']} right={v['bump_right']}")
    print(f"wheel drops    left={v['wheel_drop_left']} right={v['wheel_drop_right']}")
    print(f"cliffs         L={v['cliff_left']} FL={v['cliff_front_left']} "
          f"FR={v['cliff_front_right']} R={v['cliff_right']}")
    lb = ('left', 'front_left', 'center_left', 'center_right', 'front_right', 'right')
    print('light bumper   ' + '  '.join(f"{n}={v['light_bump_' + n + '_signal']}" for n in lb))
    print(f"encoders       left={v['left_encoder_counts']} right={v['right_encoder_counts']}")
    print(f"dock / charger home_base={v['home_base_available']} internal={v['internal_charger_available']}")


def cmd_beep(robot, _args):
    enter_safe(robot)
    robot.song(0, [(72, 12), (76, 12), (79, 12), (84, 24)])
    robot.play(0)
    time.sleep(1.0)


def cmd_display(robot, args):
    enter_safe(robot)
    text = ' '.join(args.text)
    if len(text) <= 4:
        robot.display_text(text)
        time.sleep(args.hold)
    else:
        padded = '    ' + text + '    '
        for i in range(len(padded) - 3):
            robot.display_text(padded[i:i + 4])
            time.sleep(0.35)
    robot.display_text('    ')


def cmd_drive(robot, args):
    speed = max(-300, min(300, args.speed))
    duration = max(0.0, min(3.0, args.seconds))
    enter_safe(robot)
    before = robot.query_list([43, 44])
    try:
        robot.drive_direct(speed, speed)
        time.sleep(duration)
    finally:
        for _ in range(3):
            robot.stop_drive()
            time.sleep(0.03)
    time.sleep(0.3)
    after = robot.query_list([43, 44])
    dl = wrap_delta(after['left_encoder_counts'], before['left_encoder_counts'])
    dr = wrap_delta(after['right_encoder_counts'], before['right_encoder_counts'])
    print(f"commanded {speed} mm/s for {duration} s (~{speed * duration:.0f} mm)")
    print(f"left  {dl:+d} counts = {dl * MM_PER_COUNT:+.1f} mm")
    print(f"right {dr:+d} counts = {dr * MM_PER_COUNT:+.1f} mm")


def cmd_stop(robot, _args):
    robot.stop_drive()
    robot.pwm_motors(0, 0, 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('sensors').set_defaults(func=cmd_sensors)
    sub.add_parser('beep').set_defaults(func=cmd_beep)
    p = sub.add_parser('display')
    p.add_argument('text', nargs='+')
    p.add_argument('--hold', type=float, default=3.0, help='seconds to show text of 4 characters or fewer')
    p.set_defaults(func=cmd_display)
    p = sub.add_parser('drive', help='drive straight (robot must be on the floor with space ahead)')
    p.add_argument('speed', type=int, help='mm/s, -300..300 (negative = reverse)')
    p.add_argument('seconds', type=float, help='0-3 s')
    p.set_defaults(func=cmd_drive)
    sub.add_parser('stop').set_defaults(func=cmd_stop)
    args = parser.parse_args()

    try:
        with Roomba(args.port, args.baud) as robot:
            robot.start()
            time.sleep(0.1)
            try:
                args.func(robot, args)
            finally:
                robot.start()   # back to passive so the robot's buttons work again
    except RoombaError as e:
        sys.exit(f'error: {e}')


if __name__ == '__main__':
    main()
