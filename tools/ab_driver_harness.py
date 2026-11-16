#!/usr/bin/env python3
"""Drives one roomba_driver through the scenario of tools/fake_roomba.py --scenario ab and records
everything it publishes (tools/ab_driver.sh runs it for the Python and the Rust driver). With
--scenario lightbump it only sends cmd_vel (forward at 0.25 m/s); with --scenario replug it sends
nothing and only records (the fake Roomba's port comes and goes).

Waits for the fake Roomba's t0 file (first stream request), then at fixed times after t0 publishes
cmd_vel and the roomba/* actuator topics and calls every roomba/* service, recording the
responses; records wheel/odom, joint_states, battery_state, roomba/sensors, roomba/mode and /tf
with their CLOCK_MONOTONIC receive times. At --end s after t0 it sends SIGTERM to the driver and
writes everything to --out (JSON).
"""
import argparse
import json
import math
import os
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import String, UInt8MultiArray
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from roomba_msgs.msg import CleaningMotors, Drive, DriveDirect, DrivePwm, Leds, SchedulingLeds, SensorState
from roomba_msgs.srv import DefineSong, PlaySong, PressButtons, SetDayTime, SetMode, SetSchedule

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_roomba import LIGHT_HITS  # noqa: E402


def every(start, count, dt, action):
    return [(round(start + i * dt, 3), action) for i in range(count)]


def twist(x, z):
    m = Twist()
    m.linear.x, m.angular.z = float(x), float(z)
    return ('pub', 'cmd_vel', m)


def msg(cls, **fields):
    m = cls()
    for k, v in fields.items():
        setattr(m, k, v)
    return m


def srv(name, cls, **fields):
    return ('srv', name, cls, fields)


def lightbump_schedule():
    """Forward at 0.25 m/s (Nav2's speed) all along. No command in the 0.7 s after each hit's onset
    (until the light bumper reads clear again, also with stream_layout legacy) or around the end of
    its bump reflex (~1.3 s after the hit), so whether a command lands before the stop, while the
    bumper reads pressed or after the reflex doesn't depend on timing. The robot is stopped during
    the first gap and the second is under the 0.5 s command timeout. Hits within 3 s of the last
    reflex only stop."""
    hits = [a for a, _, _ in LIGHT_HITS['lightbump']]
    times = [round(1.0 + i * 0.1, 3) for i in range(410)]
    return [(t, twist(0.25, 0.0)) for t in times
            if not any(a <= t < a + 0.7 or a + 1.25 <= t < a + 1.6 for a in hits)]


def schedule():
    s = []
    s += every(2.0, 30, 0.1, twist(0.2, 0.0))           # straight; stops on the 0.5 s timeout
    s += every(6.0, 20, 0.1, twist(0.0, 1.0))           # spin in place
    # Light-bumper hit at 10.0 while driving. Its signal is in every primary stream frame, so the stop
    # follows within ~30 ms; no command is sent while the hit (10.0-10.3) or the end of the backup
    # (1.3 s after the first supervisor tick after the hit: 11.3-11.4) is detected.
    s += every(9.0, 10, 0.1, twist(0.15, 0.0))
    s += every(10.3, 10, 0.1, twist(0.15, 0.0))         # ignored during the backup
    s += every(11.6, 9, 0.1, twist(0.15, 0.0))
    s += every(13.7, 10, 0.1, twist(0.1, 0.0))          # forward while the left bumper is pressed
    s += every(18.5, 5, 0.1, twist(0.1, 0.0))           # refused: passive after the wheel drop
    s += [(19.5, srv('set_mode', SetMode, mode=2)),
          (22.5, srv('set_mode', SetMode, mode=3)),
          (23.5, srv('set_mode', SetMode, mode=2)),
          (24.0, srv('set_mode', SetMode, mode=9)),
          (41.0, srv('set_mode', SetMode, mode=2)),
          (45.0, srv('play_song', PlaySong, song_number=1)),
          (45.3, srv('define_song', DefineSong, song_number=0, notes=[72, 76, 79], durations=[12, 12, 24], play=True)),
          (45.6, srv('define_song', DefineSong, song_number=1, notes=[60, 62], durations=[16], play=False)),
          (45.9, srv('define_song', DefineSong, song_number=5, notes=[60], durations=[16], play=False)),
          (46.2, srv('set_schedule', SetSchedule, days=0b0111110, hours=[9] * 7, minutes=[30] * 7)),
          (46.5, srv('set_schedule', SetSchedule, days=1, hours=[24] * 7, minutes=[0] * 7)),
          (46.8, srv('set_day_time', SetDayTime, use_system_time=False, day=3, hour=14, minute=5)),
          (47.1, srv('set_day_time', SetDayTime, use_system_time=False, day=7, hour=0, minute=0)),
          (47.4, srv('set_day_time', SetDayTime, use_system_time=True)),
          (47.7, srv('press_buttons', PressButtons, buttons=5)),
          (48.0, srv('wake', Trigger)),
          (48.5, ('pub', 'roomba/leds', msg(Leds, debris=True, check_robot=True, power_color=255, power_intensity=128))),
          (48.7, ('pub', 'roomba/scheduling_leds', msg(SchedulingLeds, weekdays=0b1000001, schedule=True, colon=True))),
          (48.9, ('pub', 'roomba/display', msg(String, data='HI'))),
          (49.1, ('pub', 'roomba/display', msg(String, data='HELLO WORLD'))),
          (53.5, ('pub', 'roomba/display_raw', msg(UInt8MultiArray, data=[1, 2, 3]))),
          (53.7, ('pub', 'roomba/display_raw', msg(UInt8MultiArray, data=[0x7F, 0, 1, 0xFF]))),
          (54.0, ('pub', 'roomba/cleaning_motors', msg(CleaningMotors, use_pwm=True, main_brush_pwm=50,
                                                       side_brush_pwm=-30, vacuum_pwm=100))),
          (54.3, ('pub', 'roomba/cleaning_motors', msg(CleaningMotors, side_brush=True, vacuum=True, main_brush=True))),
          (54.6, ('pub', 'roomba/cleaning_motors', msg(CleaningMotors, use_pwm=True)))]
    # 0.4 s gaps between the drive inputs: shorter than the 0.5 s command timeout, so no race with it
    s += every(55.0, 5, 0.1, ('pub', 'roomba/drive', msg(Drive, velocity_mm_s=100, radius_mm=500)))
    s += every(55.8, 3, 0.1, ('pub', 'roomba/drive', msg(Drive, velocity_mm_s=80, radius_mm=-1)))
    s += every(56.4, 5, 0.1, ('pub', 'roomba/drive_direct', msg(DriveDirect, left_mm_s=120, right_mm_s=-80)))
    s += every(57.2, 3, 0.1, ('pub', 'roomba/drive_pwm', msg(DrivePwm, left_pwm=60, right_pwm=60)))
    s += [(58.5, srv('spot', Trigger)),
          (59.5, srv('set_mode', SetMode, mode=2)),
          (62.5, srv('reset', Trigger)),
          (66.0, srv('set_mode', SetMode, mode=2))]
    s += every(67.0, 5, 0.1, twist(0.1, 0.0))
    s += [(68.5, srv('set_mode', SetMode, mode=0)),
          (69.0, srv('set_mode', SetMode, mode=1)),
          (69.5, srv('set_mode', SetMode, mode=2))]
    return sorted(s, key=lambda a: a[0])


def fields(m, names):
    return {k: getattr(m, k) for k in names}


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def num(x):
    return None if isinstance(x, float) and math.isnan(x) else x


class Harness(Node):

    def __init__(self, args):
        super().__init__('ab_driver_harness')
        self.args = args
        self.rec = {k: [] for k in ('odom', 'joints', 'battery', 'sensors', 'mode', 'tf', 'services', 'sent')}
        self.t0 = None
        ns = args.ns.rstrip('/')
        self.ns = ns
        sub = self.create_subscription
        sub(Odometry, f'{ns}/wheel/odom', self.on_odom, 100)
        sub(JointState, f'{ns}/joint_states', self.on_joints, 100)
        sub(BatteryState, f'{ns}/battery_state', self.on_battery, 100)
        sub(SensorState, f'{ns}/roomba/sensors', self.on_sensors, 100)
        sub(String, f'{ns}/roomba/mode', self.on_mode, QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        sub(TFMessage, '/tf', self.on_tf, 100)
        self.sensor_fields = [k for k in SensorState.get_fields_and_field_types() if k != 'header']
        self.pubs = {}
        self.srv_clients = {}
        self.plan = {'ab': schedule, 'lightbump': lightbump_schedule}.get(args.scenario, list)()
        for _, action in self.plan:
            if action[0] == 'pub' and action[1] not in self.pubs:
                self.pubs[action[1]] = self.create_publisher(type(action[2]), f'{ns}/{action[1]}', 10)
            if action[0] == 'srv' and action[1] not in self.srv_clients:
                self.srv_clients[action[1]] = self.create_client(action[2], f'{ns}/roomba/{action[1]}')

    def now(self):
        return time.monotonic() - self.t0 if self.t0 is not None else None

    def on_odom(self, m):
        p, q, t = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        self.rec['odom'].append({'t': self.now(), 'stamp': stamp(m.header), 'frame': m.header.frame_id,
                                 'child': m.child_frame_id, 'x': p.x, 'y': p.y, 'qz': q.z, 'qw': q.w,
                                 'vx': t.linear.x, 'wz': t.angular.z, 'pcov': list(m.pose.covariance),
                                 'tcov': list(m.twist.covariance)})

    def on_joints(self, m):
        self.rec['joints'].append({'t': self.now(), 'stamp': stamp(m.header), 'name': list(m.name),
                                   'position': list(m.position), 'velocity': list(m.velocity)})

    def on_battery(self, m):
        d = fields(m, ('voltage', 'temperature', 'current', 'charge', 'capacity', 'design_capacity', 'percentage',
                       'power_supply_status', 'power_supply_health', 'power_supply_technology', 'present'))
        self.rec['battery'].append({'t': self.now(), 'stamp': stamp(m.header), 'frame': m.header.frame_id,
                                    **{k: num(v) for k, v in d.items()}})

    def on_sensors(self, m):
        self.rec['sensors'].append({'t': self.now(), 'stamp': stamp(m.header), 'frame': m.header.frame_id,
                                    **fields(m, self.sensor_fields)})

    def on_mode(self, m):
        self.rec['mode'].append({'t': self.now(), 'data': m.data})

    def on_tf(self, m):
        for tf in m.transforms:
            tr, r = tf.transform.translation, tf.transform.rotation
            self.rec['tf'].append({'t': self.now(), 'stamp': stamp(tf.header), 'parent': tf.header.frame_id,
                                   'child': tf.child_frame_id, 'x': tr.x, 'y': tr.y, 'z': tr.z, 'qz': r.z, 'qw': r.w})

    def act(self, at, action):
        if action[0] == 'pub':
            self.pubs[action[1]].publish(action[2])
            self.rec['sent'].append({'at': at, 't': self.now(), 'topic': action[1]})
            return
        _, name, cls, kw = action
        req = cls.Request()
        for k, v in kw.items():
            setattr(req, k, v)
        entry = {'at': at, 't': self.now(), 'service': name, 'request': {k: v for k, v in kw.items()}}
        self.rec['services'].append(entry)
        future = self.srv_clients[name].call_async(req)

        def done(f, entry=entry):
            r = f.result()
            entry['t_done'] = self.now()
            entry['response'] = {k: getattr(r, k) for k in r.get_fields_and_field_types()}
        future.add_done_callback(done)

    def run(self):
        while self.t0 is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                with open(self.args.t0_file) as f:
                    self.t0 = json.load(f)['t0']
            except (OSError, ValueError, KeyError):
                pass
        i = 0
        while self.now() < self.args.end:
            while i < len(self.plan) and self.now() >= self.plan[i][0]:
                self.act(*self.plan[i])
                i += 1
            rclpy.spin_once(self, timeout_sec=0.002)
        killed = exited = None
        pid = int(open(self.args.driver_pid_file).read()) if self.args.driver_pid_file else 0
        if pid:
            killed = self.now()
            os.kill(pid, signal.SIGTERM)
        end = time.monotonic() + 4.0
        while time.monotonic() < end:     # collect the last messages; wait for the driver to exit
            rclpy.spin_once(self, timeout_sec=0.01)
            if pid and exited is None and not os.path.exists(f'/proc/{pid}'):
                exited = self.now()
                end = min(end, time.monotonic() + 0.5)
        out = dict(self.rec, t0=self.t0, sigterm=killed, exited=exited)
        with open(self.args.out, 'w') as f:
            json.dump(out, f)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--t0-file', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--end', type=float, default=70.5)
    ap.add_argument('--scenario', default='ab', choices=('ab', 'lightbump', 'replug'))
    ap.add_argument('--ns', default='', help='namespace of the driver')
    ap.add_argument('--driver-pid-file', default='', help='file with the driver pid, sent SIGTERM at --end')
    args = ap.parse_args()
    rclpy.init()
    h = Harness(args)
    try:
        h.run()
    finally:
        h.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
