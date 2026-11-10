#!/usr/bin/env python3
"""ROS 2 driver for iRobot Roomba 500-series robots over the Open Interface.

Topics, services and parameters are listed in README.md.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import String, UInt8MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from roomba_msgs.msg import (CleaningMotors, Drive, DriveDirect, DrivePwm, Leds, SchedulingLeds,
                             SensorState)
from roomba_msgs.srv import DefineSong, PlaySong, PressButtons, SetDayTime, SetMode, SetSchedule

from roomba_driver import oi
from roomba_driver.kinematics import DiffDriveOdometry, twist_to_wheel_speeds
from roomba_driver.roomba import Roomba, RoombaError

MODES = {'off': oi.MODE_OFF, 'passive': oi.MODE_PASSIVE, 'safe': oi.MODE_SAFE, 'full': oi.MODE_FULL}
CONTROL_MODES = (oi.MODE_SAFE, oi.MODE_FULL)
STALE_AFTER_S = 1.0  # sensor data older than this means contact with the robot is lost
BUMP_FIELDS = ('bump_left', 'bump_right')
LIGHT_BUMP_FIELDS = ('light_bump_front_left', 'light_bump_center_left', 'light_bump_center_right',
                     'light_bump_front_right')


def _yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class RoombaDriver(Node):

    def __init__(self):
        super().__init__('roomba_driver')
        p = self.declare_parameter
        self.port = p('port', '/dev/ttyUSB0').value
        baud = int(p('baud', 115200).value)
        initial_mode = str(p('initial_mode', 'safe').value).lower()
        if initial_mode not in ('passive', 'safe', 'full'):
            raise ValueError(f"initial_mode must be passive, safe or full, not '{initial_mode}'")
        self.auto_reenter_mode = bool(p('auto_reenter_mode', False).value)
        self.base_frame = p('base_frame', 'base_link').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.publish_tf = bool(p('publish_tf', False).value)
        self.wheel_base = float(p('wheel_base_m', 0.235).value)
        wheel_diameter = float(p('wheel_diameter_m', 0.072).value)
        counts_per_rev = float(p('encoder_counts_per_rev', 508.8).value)
        self.max_wheel_speed = min(float(p('max_wheel_speed_m_s', 0.3).value),
                                   oi.MAX_WHEEL_SPEED_MM_S / 1000.0)
        self.cmd_timeout = float(p('cmd_vel_timeout_s', 0.5).value)
        stamped = bool(p('use_stamped_cmd_vel', False).value)
        odom_rate = float(p('odom_rate_hz', 50.0).value)
        sensor_rate = float(p('sensor_rate_hz', 20.0).value)
        battery_rate = float(p('battery_rate_hz', 1.0).value)
        self.joint_names = [p('left_wheel_joint', 'left_wheel_joint').value,
                            p('right_wheel_joint', 'right_wheel_joint').value]
        wake_with_dtr = bool(p('wake_with_dtr', False).value)
        keep_awake_s = float(p('keep_awake_period_s', 60.0).value)
        scroll_period = float(p('display_scroll_period_s', 0.35).value)
        # Safe mode doesn't stop on bumps; the camera can't see the last ~30 cm, so the bumper
        # is the last line of defence. Forward motion is refused while a bumper is pressed.
        self.bump_stop = bool(p('bump_stop', True).value)
        # Also treat the front light-bumper (IR proximity) detections as bumps.
        self.bump_stop_light = bool(p('bump_stop_light_bumper', False).value)
        # 0: use the Roomba's own light-bumper flags. >0: a light bumper counts only when its
        # signal (0-4095) reaches this, e.g. so deep carpet pile (~100-200) doesn't block driving.
        self.light_stop_signal = int(p('light_bumper_stop_signal', 0).value)
        # Faulty sensors to report as off everywhere (topics and safety logic), e.g. ['bump_right'].
        self.ignore_sensors = [s for s in p('ignore_sensors', ['']).value if s]
        unknown = set(self.ignore_sensors) - set(SensorState.get_fields_and_field_types())
        if unknown:
            raise ValueError(f'ignore_sensors: unknown sensor names {sorted(unknown)}')

        self.robot = Roomba(self.port, baud, logger=self.get_logger())
        self.robot.wake_with_dtr = wake_with_dtr
        self.robot.add_frame_callback(self._on_frame)

        self.odom = DiffDriveOdometry(self.wheel_base, wheel_diameter, counts_per_rev,
                                      max_wheel_speed=oi.MAX_WHEEL_SPEED_MM_S / 1000.0 * 1.5)
        self._lock = threading.Lock()
        self._latest = None              # (merged sensor values, rclpy Time)
        self._values = {}
        self._odom_stamp = None
        self._mode = None
        self._mode_event = None          # (previous, new, values) awaiting a log/publish
        self._desired_mode = MODES[initial_mode]
        self._apply_initial_mode = True
        self._check_mode_at = None
        self._last_frame_mono = 0.0
        self._contact_lost = False
        self._driving = False
        self._driving_forward = False
        self._bumped = False
        self._last_drive_time = 0.0
        self._last_reenter = 0.0
        self._scroll = None              # [padded text, index]
        self._sensor_fields = set(SensorState.get_fields_and_field_types()) - {'header'}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_odom = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_battery = self.create_publisher(BatteryState, 'battery_state', 10)
        self.pub_sensors = self.create_publisher(SensorState, 'roomba/sensors', 10)
        self.pub_mode = self.create_publisher(String, 'roomba/mode', latched)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # Depth 1: a backlog of old motion commands must never be replayed.
        if stamped:
            self.create_subscription(TwistStamped, 'cmd_vel', lambda m: self._on_cmd_vel(m.twist), 1)
        else:
            self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 1)
        self.create_subscription(Drive, 'roomba/drive', self._on_drive, 1)
        self.create_subscription(DriveDirect, 'roomba/drive_direct', self._on_drive_direct, 1)
        self.create_subscription(DrivePwm, 'roomba/drive_pwm', self._on_drive_pwm, 1)
        self.create_subscription(CleaningMotors, 'roomba/cleaning_motors', self._on_cleaning_motors, 10)
        self.create_subscription(Leds, 'roomba/leds', self._on_leds, 10)
        self.create_subscription(SchedulingLeds, 'roomba/scheduling_leds', self._on_scheduling_leds, 10)
        self.create_subscription(String, 'roomba/display', self._on_display, 10)
        self.create_subscription(UInt8MultiArray, 'roomba/display_raw', self._on_display_raw, 10)

        # Services may block (mode confirmation, wake pulse); run them one at a time in their own
        # group so they can never occupy the threads the watchdog and odometry timers need.
        services = MutuallyExclusiveCallbackGroup()
        self.create_service(SetMode, 'roomba/set_mode', self._srv_set_mode, callback_group=services)
        for name, action in (('clean', self.robot.clean), ('spot', self.robot.spot),
                             ('max_clean', self.robot.max_clean), ('seek_dock', self.robot.seek_dock),
                             ('power_off', self.robot.power_off)):
            self.create_service(Trigger, f'roomba/{name}', self._behaviour_service(name, action),
                                callback_group=services)
        self.create_service(Trigger, 'roomba/reset', self._srv_reset, callback_group=services)
        self.create_service(Trigger, 'roomba/wake', self._srv_wake, callback_group=services)
        self.create_service(DefineSong, 'roomba/define_song', self._srv_define_song, callback_group=services)
        self.create_service(PlaySong, 'roomba/play_song', self._srv_play_song, callback_group=services)
        self.create_service(PressButtons, 'roomba/press_buttons', self._srv_press_buttons,
                            callback_group=services)
        self.create_service(SetSchedule, 'roomba/set_schedule', self._srv_set_schedule,
                            callback_group=services)
        self.create_service(SetDayTime, 'roomba/set_day_time', self._srv_set_day_time,
                            callback_group=services)

        self._connect_timer = self.create_timer(2.0, self._connect,
                                                callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / odom_rate, self._publish_odometry)
        self.create_timer(1.0 / sensor_rate, self._publish_sensors)
        self.create_timer(1.0 / battery_rate, self._publish_battery)
        self.create_timer(0.05, self._supervise)
        self.create_timer(scroll_period, self._scroll_display)
        if wake_with_dtr and keep_awake_s > 0:
            self.create_timer(keep_awake_s, self._keep_awake, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(f'Roomba driver starting on {self.port} at {baud} baud '
                               f'(initial mode {initial_mode})')
        self._connect()

    # ------------------------------------------------------------ connection / stream
    def _connect(self):
        if self.robot.streaming:
            self._connect_timer.cancel()
            return
        try:
            self.robot.open()
            self.robot.start()
            time.sleep(0.05)
            self.robot.start_stream()   # primary + secondary layouts, every sensor
        except (OSError, RoombaError) as e:
            self.get_logger().warning(f'cannot open {self.port}: {e}; retrying',
                                      throttle_duration_sec=30.0)
            self.robot.close()
            return
        self._connect_timer.cancel()
        if self.robot.wait_for_frame(2.0):
            values, _ = self.robot.latest()
            self.get_logger().info(
                f"connected: battery {values['voltage_mv'] / 1000:.2f} V, "
                f"{values['charge_mah']}/{values['capacity_mah']} mAh, "
                f"mode {oi.MODE_NAMES.get(values['oi_mode'], values['oi_mode'])}")
        else:
            self.get_logger().warning('port open but no sensor data yet; is the Roomba awake? '
                                      'Will keep retrying.')

    def _on_frame(self, values, monotonic):
        """Reader-thread callback for every 15 ms sensor frame."""
        stamp = self.get_clock().now()
        with self._lock:
            self._values.update(values)   # frames alternate between two layouts; keep the union
            for name in self.ignore_sensors:
                if name in self._values:
                    self._values[name] = type(self._values[name])()   # False / 0
            self._latest = (self._values, stamp)
            self._last_frame_mono = monotonic
            if self.odom.update(values['left_encoder_counts'], values['right_encoder_counts'],
                                stamp.nanoseconds * 1e-9):
                self._odom_stamp = stamp
            mode = values['oi_mode']
            if mode != self._mode:
                self._mode_event = (self._mode, mode, dict(self._values))
                self._mode = mode
            bumped = any(self._values.get(k) for k in BUMP_FIELDS) or (
                self.bump_stop_light and any(self._light_bumped(k) for k in LIGHT_BUMP_FIELDS))
            newly_bumped = bumped and not self._bumped
            self._bumped = bumped
        if newly_bumped and self.bump_stop and self._driving_forward:
            # Stop straight from the reader thread: waiting for the next timer adds latency.
            if self._try(self.robot.stop_drive, 'stop on bump'):
                self._driving = self._driving_forward = False
            self.get_logger().warning('bumper hit: stopped (reverse or turn to get free)')

    def _light_bumped(self, field):
        """A light bumper counts as a bump (its flag, or its signal when a threshold is set)."""
        if self.light_stop_signal > 0:
            return self._values.get(field + '_signal', 0) >= self.light_stop_signal
        return bool(self._values.get(field))

    def _supervise(self):
        """Contact loss, mode changes, initial mode, and the drive command watchdog."""
        now = time.monotonic()
        with self._lock:
            if self._mode is not None and now - self._last_frame_mono > STALE_AFTER_S:
                self._mode = None
                self._mode_event = None
                lost = True
            else:
                lost = False
            event, self._mode_event = self._mode_event, None
            have_data = self._mode is not None
        if lost:
            if self._try(self.robot.stop_drive, 'stop after losing contact'):
                self._driving = self._driving_forward = False
            self._contact_lost = True
            self.pub_mode.publish(String(data='unknown'))
            self.get_logger().warning(
                'lost contact with the Roomba (asleep? In passive mode it sleeps after 5 minutes idle; '
                'press Clean to wake it, or wire DTR to pin 5 and set wake_with_dtr)')
        if event:
            self._report_mode_change(*event)
        if have_data and self._apply_initial_mode:
            self._apply_initial_mode = False
            if self._desired_mode in CONTROL_MODES:
                if self._try(lambda: self.robot.request_mode(self._desired_mode), 'set initial mode'):
                    self._check_mode_at = now + 1.0
        if self._check_mode_at and now > self._check_mode_at:
            self._check_mode_at = None
            if self._desired_mode in CONTROL_MODES and self._mode != self._desired_mode:
                values, _ = self.robot.latest()
                self.get_logger().warning(
                    f'robot did not enter {oi.MODE_NAMES[self._desired_mode]} mode '
                    f'({self._blockers(values) or "no reason reported"}); '
                    f'call roomba/set_mode once it is on the floor and off the dock')
        if self._driving and now - self._last_drive_time > self.cmd_timeout:
            # Keep retrying every cycle until a stop actually reaches the robot.
            if self._try(self.robot.stop_drive, 'stop after command timeout'):
                self._driving = self._driving_forward = False
                self.get_logger().info('no drive command for %.1f s; stopped' % self.cmd_timeout)

    def _report_mode_change(self, previous, mode, values):
        name = oi.MODE_NAMES.get(mode, str(mode))
        self.pub_mode.publish(String(data=name))
        if previous is None:
            if self._contact_lost:
                self._contact_lost = False
                self.get_logger().info(f'contact with the Roomba restored (mode {name})')
                self._apply_initial_mode = True
            if mode == oi.MODE_PASSIVE and not self.robot.wake_with_dtr and self._desired_mode not in CONTROL_MODES:
                self.get_logger().info('in passive mode the Roomba sleeps after 5 minutes idle')
            return
        reason = ''
        if previous in CONTROL_MODES and mode == oi.MODE_PASSIVE:
            blockers = self._blockers(values)
            reason = f' ({blockers})' if blockers else ''
            self._driving = self._driving_forward = False
            if not blockers and self._desired_mode in CONTROL_MODES:
                # Nothing unsafe happened (no cliff, wheel drop or charger): the OI was restarted
                # (e.g. a stream restart or a robot reset). Go back to the requested mode.
                reason = ' (no safety reason; re-entering ' + oi.MODE_NAMES[self._desired_mode] + ')'
                self._try(lambda: self.robot.request_mode(self._desired_mode), 're-enter mode')
        self.get_logger().info(f'OI mode {oi.MODE_NAMES.get(previous, previous)} -> {name}{reason}')

    @staticmethod
    def _blockers(values):
        """Conditions that keep the robot out of (or drop it from) Safe mode."""
        return ', '.join(k for k in ('wheel_drop_left', 'wheel_drop_right', 'cliff_left', 'cliff_front_left',
                                     'cliff_front_right', 'cliff_right', 'home_base_available',
                                     'internal_charger_available') if values.get(k))

    def _keep_awake(self):
        if self._mode == oi.MODE_PASSIVE:
            self._try(self.robot.wake, 'keep-awake pulse')

    # ------------------------------------------------------------ helpers
    def _try(self, action, what):
        try:
            action()
            return True
        except (RoombaError, ValueError) as e:
            self.get_logger().warning(f'{what} failed: {e}', throttle_duration_sec=2.0)
            return False

    def _in_control(self, what):
        """True if the robot is in Safe/Full mode; otherwise warns (and may re-enter the mode)."""
        mode = self._mode
        if mode in CONTROL_MODES:
            return True
        if mode is None:
            self.get_logger().warning(f'ignoring {what}: no sensor data from the Roomba yet',
                                      throttle_duration_sec=5.0)
            return False
        now = time.monotonic()
        if self.auto_reenter_mode and self._desired_mode in CONTROL_MODES and now - self._last_reenter > 1.0:
            self._last_reenter = now
            self.get_logger().warning(f're-entering {oi.MODE_NAMES[self._desired_mode]} mode for {what}')
            self._try(lambda: self.robot.request_mode(self._desired_mode), 're-enter mode')
        elif not self.auto_reenter_mode:
            self.get_logger().warning(
                f'ignoring {what}: Roomba is in {oi.MODE_NAMES.get(mode, mode)} mode. '
                f'Call roomba/set_mode with mode 2 (safe) or 3 (full).', throttle_duration_sec=5.0)
        return False

    def _drive(self, action, moving, what, forward=False):
        if not moving:
            # Stopping is always allowed, even if the mode is unknown.
            if self.robot.is_open and self._try(action, what):
                self._driving = self._driving_forward = False
            return
        if forward and self.bump_stop and self._bumped:
            self.get_logger().warning(f'ignoring forward {what}: bumper pressed (reverse or turn to get free)',
                                      throttle_duration_sec=2.0)
            if self._driving_forward and self._try(self.robot.stop_drive, 'stop on bump'):
                self._driving = self._driving_forward = False
            return
        if not self._in_control(what):
            return
        if self._try(action, what):
            self._driving = True
            self._driving_forward = forward
            self._last_drive_time = time.monotonic()

    def _service_result(self, response, action, what, need_control=False):
        if need_control and self._mode not in CONTROL_MODES:
            response.success = False
            response.message = (f'{what} needs Safe or Full mode; robot is in '
                                f'{oi.MODE_NAMES.get(self._mode, self._mode)} mode')
            return response
        try:
            action()
            response.success, response.message = True, f'{what} sent'
        except (RoombaError, ValueError) as e:
            response.success, response.message = False, str(e)
        return response

    # ------------------------------------------------------------ drive topics
    def _on_cmd_vel(self, msg):
        left, right = twist_to_wheel_speeds(msg.linear.x, msg.angular.z, self.wheel_base,
                                            self.max_wheel_speed)
        left_mm, right_mm = round(left * 1000), round(right * 1000)
        self._drive(lambda: self.robot.drive_direct(left_mm, right_mm), bool(left_mm or right_mm), 'cmd_vel',
                    forward=left_mm + right_mm > 0)

    def _on_drive(self, msg):
        limit = self.max_wheel_speed * 1000
        velocity = max(-limit, min(limit, msg.velocity_mm_s))
        spin = msg.radius_mm in (oi.RADIUS_TURN_CW, oi.RADIUS_TURN_CCW)
        self._drive(lambda: self.robot.drive(velocity, msg.radius_mm), bool(velocity), 'roomba/drive',
                    forward=velocity > 0 and not spin)

    def _on_drive_direct(self, msg):
        limit = self.max_wheel_speed * 1000
        left = max(-limit, min(limit, msg.left_mm_s))
        right = max(-limit, min(limit, msg.right_mm_s))
        self._drive(lambda: self.robot.drive_direct(left, right), bool(left or right), 'roomba/drive_direct',
                    forward=left + right > 0)

    def _on_drive_pwm(self, msg):
        self._drive(lambda: self.robot.drive_pwm(msg.left_pwm, msg.right_pwm),
                    bool(msg.left_pwm or msg.right_pwm), 'roomba/drive_pwm',
                    forward=msg.left_pwm + msg.right_pwm > 0)

    # ------------------------------------------------------------ other actuator topics
    def _on_cleaning_motors(self, msg):
        if not self._in_control('roomba/cleaning_motors'):
            return
        if msg.use_pwm:
            self._try(lambda: self.robot.pwm_motors(msg.main_brush_pwm, msg.side_brush_pwm, msg.vacuum_pwm),
                      'cleaning motors')
        else:
            self._try(lambda: self.robot.motors(msg.side_brush, msg.vacuum, msg.main_brush,
                                                msg.side_brush_clockwise, msg.main_brush_outward),
                      'cleaning motors')

    def _on_leds(self, msg):
        if self._in_control('roomba/leds'):
            self._try(lambda: self.robot.leds(msg.debris, msg.spot, msg.dock, msg.check_robot,
                                              msg.power_color, msg.power_intensity), 'LEDs')

    def _on_scheduling_leds(self, msg):
        if self._in_control('roomba/scheduling_leds'):
            self._try(lambda: self.robot.scheduling_leds(msg.weekdays, msg.schedule, msg.clock,
                                                         msg.am, msg.pm, msg.colon), 'scheduling LEDs')

    def _on_display(self, msg):
        if not self._in_control('roomba/display'):
            return
        text = msg.data
        if len(text) <= 4:
            self._scroll = None
            self._try(lambda: self.robot.display_text(text), 'display')
        else:
            self._scroll = ['    ' + text + '    ', 0]

    def _scroll_display(self):
        scroll = self._scroll
        if scroll is None or self._mode not in CONTROL_MODES:
            return
        text, i = scroll
        self._try(lambda: self.robot.display_text(text[i:i + 4]), 'display')
        scroll[1] = (i + 1) % (len(text) - 3)

    def _on_display_raw(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warning('roomba/display_raw needs exactly 4 bytes (leftmost digit first)')
            return
        if self._in_control('roomba/display_raw'):
            self._scroll = None
            self._try(lambda: self.robot.digit_leds_raw(list(msg.data)), 'display')

    # ------------------------------------------------------------ services
    def _srv_set_mode(self, request, response):
        mode = request.mode
        if mode not in oi.MODE_NAMES:
            response.success, response.message = False, f'unknown mode {mode}'
            response.mode = self._mode if self._mode is not None else 0
            return response
        if self._mode is None and mode != oi.MODE_OFF:
            if self.robot.streaming and self.robot.wake_with_dtr:
                self._try(self.robot.wake, 'wake')
            if not self.robot.wait_for_frame(2.0):
                response.success, response.mode = False, 0
                response.message = 'no sensor data from the Roomba (asleep? press Clean to wake it)'
                return response
        self._desired_mode = mode if mode in CONTROL_MODES else None
        try:
            reported = self.robot.set_mode(mode, timeout=1.0)
        except RoombaError as e:
            response.success, response.message, response.mode = False, str(e), 0
            return response
        reported = reported if reported is not None else 0
        response.mode = reported
        response.success = reported == mode
        want, got = oi.MODE_NAMES[mode], oi.MODE_NAMES.get(reported, reported)
        if response.success:
            response.message = f'now in {got} mode'
        else:
            values, _ = self.robot.latest()
            blockers = self._blockers(values)
            hint = f'; active: {blockers}' if blockers else ''
            response.message = f'asked for {want}, robot reports {got}{hint}'
        return response

    def _behaviour_service(self, name, action):
        def handler(_request, response):
            self._desired_mode = None
            self._scroll = None
            return self._service_result(response, action, name)
        return handler

    def _srv_reset(self, _request, response):
        self._desired_mode = None
        return self._service_result(response, self.robot.reset, 'reset (robot is rebooting)')

    def _srv_wake(self, _request, response):
        return self._service_result(response, self.robot.wake, 'wake pulse on DTR (needs DTR wired to pin 5)')

    def _srv_define_song(self, request, response):
        if len(request.notes) != len(request.durations):
            response.success, response.message = False, 'notes and durations must be the same length'
            return response
        notes = list(zip(request.notes, request.durations))
        self._service_result(response, lambda: self.robot.song(request.song_number, notes),
                             f'song {request.song_number}')
        if response.success and request.play:
            self._service_result(response, lambda: self.robot.play(request.song_number),
                                 f'song {request.song_number} defined and played', need_control=True)
        return response

    def _srv_play_song(self, request, response):
        return self._service_result(response, lambda: self.robot.play(request.song_number),
                                    f'play song {request.song_number}', need_control=True)

    def _srv_press_buttons(self, request, response):
        return self._service_result(response, lambda: self.robot.buttons(request.buttons), 'button press')

    def _srv_set_schedule(self, request, response):
        times = list(zip(request.hours, request.minutes))
        return self._service_result(response, lambda: self.robot.schedule(request.days, times), 'schedule')

    def _srv_set_day_time(self, request, response):
        if request.use_system_time:
            now = time.localtime()
            day, hour, minute = (now.tm_wday + 1) % 7, now.tm_hour, now.tm_min
        else:
            day, hour, minute = request.day, request.hour, request.minute
        return self._service_result(response, lambda: self.robot.set_day_time(day, hour, minute),
                                    f'clock set to day {day} {hour:02d}:{minute:02d}')

    # ------------------------------------------------------------ publishers
    def _publish_odometry(self):
        with self._lock:
            stamp, self._odom_stamp = self._odom_stamp, None
            if stamp is None:
                return
            o = self.odom
            x, y, theta, v, w = o.x, o.y, o.theta, o.linear, o.angular
            positions = [o.left_pos, o.right_pos]
            velocities = [o.left_vel, o.right_vel]
        qx, qy, qz, qw = _yaw_to_quaternion(theta)
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = qx, qy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        big = 1e6
        msg.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                               0, 0.01, 0, 0, 0, 0,
                               0, 0, big, 0, 0, 0,
                               0, 0, 0, big, 0, 0,
                               0, 0, 0, 0, big, 0,
                               0, 0, 0, 0, 0, 0.05]
        msg.twist.covariance = [0.002, 0, 0, 0, 0, 0,
                                0, big, 0, 0, 0, 0,
                                0, 0, big, 0, 0, 0,
                                0, 0, 0, big, 0, 0,
                                0, 0, 0, 0, big, 0,
                                0, 0, 0, 0, 0, 0.01]
        self.pub_odom.publish(msg)

        joints = JointState()
        joints.header.stamp = msg.header.stamp
        joints.name = self.joint_names
        joints.position = positions
        joints.velocity = velocities
        self.pub_joints.publish(joints)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def _publish_sensors(self):
        with self._lock:
            if self._latest is None or self._mode is None:   # never publish stale data
                return
            values, stamp = dict(self._latest[0]), self._latest[1]
        msg = SensorState()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.base_frame
        for key in self._sensor_fields.intersection(values):
            setattr(msg, key, values[key])
        self.pub_sensors.publish(msg)

    def _publish_battery(self):
        with self._lock:
            if self._latest is None or self._mode is None:
                return
            v, stamp = dict(self._latest[0]), self._latest[1]
        msg = BatteryState()
        msg.header.stamp = stamp.to_msg()
        msg.voltage = v['voltage_mv'] / 1000.0
        msg.current = v['current_ma'] / 1000.0
        msg.temperature = float(v['temperature_c'])
        msg.charge = v['charge_mah'] / 1000.0
        msg.capacity = v['capacity_mah'] / 1000.0
        msg.design_capacity = float('nan')
        msg.percentage = (min(1.0, v['charge_mah'] / v['capacity_mah']) if v['capacity_mah']
                          else float('nan'))
        state = v['charging_state']
        on_charger = v['home_base_available'] or v['internal_charger_available']
        if state in (1, 2, 3):
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif on_charger and msg.percentage >= 0.99:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif on_charger:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = (BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE if state == 5
                                   else BatteryState.POWER_SUPPLY_HEALTH_GOOD)
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_NIMH
        msg.present = True
        self.pub_battery.publish(msg)

    # ------------------------------------------------------------ shutdown
    def shutdown(self):
        """Stop everything and hand the robot back in Passive mode so its buttons work."""
        if self.robot.is_open:
            for action in (self.robot.stop_drive, lambda: self.robot.pwm_motors(0, 0, 0),
                           lambda: self.robot.display_text('    ')):
                try:
                    action()
                except RoombaError:
                    pass
            self.robot.stop_stream()
            try:
                self.robot.start()
            except RoombaError:
                pass
        self.robot.close()


def main(args=None):
    rclpy.init(args=args)
    node = RoombaDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
