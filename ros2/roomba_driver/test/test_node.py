"""node.py's driver logic on a fake robot and clock (no serial port, nothing spinning): the
light-bumper stop on primary frames, a drop to Passive first seen in a secondary frame, and the
end of the bump reflex."""
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('roomba_msgs')

from geometry_msgs.msg import Twist  # noqa: E402

from roomba_driver import node as driver_node, oi  # noqa: E402
from roomba_driver.roomba import Roomba, RoombaError  # noqa: E402

STOP = oi.drive_direct(0, 0)
# The robot's settings (roomba_bringup/config/roomba_driver_robot.yaml) that matter here.
ROBOT = {'max_wheel_speed_m_s': 0.4, 'bump_stop_light_bumper': 'true', 'bump_backup': 'true',
         'light_bumper_stop_signal': 200}


class FakeRobot(Roomba):
    """Records the bytes and mode requests the node sends; never opens a port."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.writes = []
        self.modes = []

    def open(self):
        raise RoombaError('no port in the tests')

    @property
    def is_open(self):
        return True

    def write(self, data):
        self.writes.append(bytes(data))

    def request_mode(self, mode):
        self.modes.append(mode)


class Log:
    def __init__(self):
        self.lines = []

    def info(self, msg, **_kwargs):
        self.lines.append(msg)

    warning = error = info

    def take(self):
        lines, self.lines = self.lines, []
        return lines


def frame(layout, mode, **values):
    """A frame of one of the driver's stream layouts: zeros, the mode, and values it carries."""
    v = {}
    for pid in layout:
        v.update(oi.decode(pid, bytes(oi.packet_size(pid))))
    missing = set(values) - set(v)
    assert not missing, f'not in this layout: {sorted(missing)}'
    v.update(values, oi_mode=mode)
    return v


def primary(mode, **values):
    return frame(oi.STREAM_PRIMARY, mode, **values)


def secondary(mode, **values):
    return frame(oi.STREAM_SECONDARY, mode, **values)


def twist(x, z=0.0):
    m = Twist()
    m.linear.x, m.angular.z = float(x), float(z)
    return m


class Driver:
    """A RoombaDriver on a FakeRobot and a fake clock, started up and in Safe mode at t = 101.1."""

    def __init__(self, monkeypatch, params):
        self.t = 100.0
        monkeypatch.setattr(driver_node, 'Roomba', FakeRobot)
        monkeypatch.setattr(driver_node.time, 'monotonic', lambda: self.t)
        args = ['--ros-args', '-p', 'port:=/nonexistent/roomba-test']
        for k, v in params.items():
            args += ['-p', f'{k}:={v}']
        rclpy.init(args=args)
        self.node = driver_node.RoombaDriver()
        self.robot = self.node.robot
        self.log = Log()
        monkeypatch.setattr(self.node, 'get_logger', lambda: self.log)
        self.node._on_frame(primary(oi.MODE_PASSIVE), 100.0)
        self.supervise(100.01)
        assert self.robot.modes == [oi.MODE_SAFE]       # initial mode requested
        self.tick(100.03)
        self.tick(101.1)                                 # safe mode confirmed: no warning
        assert self.log.take() == ['OI mode passive -> safe']
        self.robot.modes.clear()
        self.robot.writes.clear()

    def frame(self, values, t):
        self.node._on_frame(values, t)

    def supervise(self, t):
        self.t = t
        self.node._supervise()

    def tick(self, t):
        """A safe-mode primary frame just before a supervisor cycle, so contact isn't lost."""
        self.frame(primary(oi.MODE_SAFE), t - 0.001)
        self.supervise(t)

    def close(self):
        self.node.destroy_node()
        rclpy.try_shutdown()


@pytest.fixture
def robot(monkeypatch):
    d = Driver(monkeypatch, ROBOT)
    yield d
    d.close()


@pytest.fixture
def default(monkeypatch):
    d = Driver(monkeypatch, {})
    yield d
    d.close()


def test_light_bumper_stop_is_checked_on_every_primary_frame(robot):
    robot.node._on_cmd_vel(twist(0.25))
    robot.frame(primary(oi.MODE_SAFE, light_bump_front_left_signal=230), 101.115)
    assert robot.robot.writes == [oi.drive_direct(250, 250), STOP]
    assert robot.log.take() == ['bumper hit: backing up']


def test_drop_to_passive_first_seen_in_a_secondary_frame_waits_for_the_charger(default):
    d = default
    # Docked: the drop and the charger come together, but the first frame showing the drop is a
    # secondary one, without the charger packet.
    d.frame(secondary(oi.MODE_PASSIVE), 102.0)
    d.supervise(102.01)
    assert d.log.take() == []
    d.frame(primary(oi.MODE_PASSIVE, home_base_available=True), 102.015)
    d.supervise(102.06)
    assert d.log.take() == ['OI mode safe -> passive (home_base_available)']
    assert d.robot.modes == []                           # safe is not asked for on the charger
    # A drop without a reason (the OI restarted) is still undone once a primary frame confirms it.
    d.tick(102.1)
    d.frame(secondary(oi.MODE_PASSIVE), 102.2)
    d.supervise(102.21)
    assert d.log.take() == ['OI mode passive -> safe']
    d.frame(primary(oi.MODE_PASSIVE), 102.215)
    d.supervise(102.26)
    assert d.log.take() == ['OI mode safe -> passive (no safety reason; re-entering safe)']
    assert d.robot.modes == [oi.MODE_SAFE]
    # Without any primary frame the drop is judged on what is known after CHARGER_WAIT_S.
    d.frame(secondary(oi.MODE_SAFE), 102.3)
    d.supervise(102.31)
    d.frame(secondary(oi.MODE_PASSIVE), 102.4)
    d.supervise(102.41)
    d.frame(secondary(oi.MODE_PASSIVE), 103.0)
    d.supervise(103.39)
    assert d.log.take() == ['OI mode passive -> safe']
    d.supervise(103.41)
    assert d.log.take() == ['OI mode safe -> passive (no safety reason; re-entering safe)']


def test_bump_reflex_steps_end_on_supervisor_ticks_whatever_the_timer_jitter(robot):
    robot.node._on_cmd_vel(twist(0.2))
    robot.frame(primary(oi.MODE_SAFE, bump_left=True), 101.2)
    assert robot.log.take() == ['bumper hit: backing up']
    robot.robot.writes.clear()
    # The reflex reverses 0.8 s = exactly 16 ticks. Its first tick ran 3 ms late, so by the clock
    # the tick 0.8 s later comes 3 ms early and would reverse once more.
    robot.tick(101.253)
    for k in range(1, 27):
        robot.tick(101.25 + k * 0.05)
    writes = robot.robot.writes
    assert writes.count(oi.drive_direct(-100, -100)) == 16
    assert writes.count(oi.drive_direct(100, -100)) == 10
    assert writes[-1] == STOP
    assert robot.log.take() == ['bump backup done']
