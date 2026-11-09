import math

import pytest

from roomba_driver.kinematics import DiffDriveOdometry, twist_to_wheel_speeds, wrap_delta

BASE, DIAMETER, CPR = 0.235, 0.072, 508.8
M_PER_COUNT = math.pi * DIAMETER / CPR


def test_wrap_delta_handles_rollover():
    assert wrap_delta(5, 65530) == 11
    assert wrap_delta(65530, 5) == -11
    assert wrap_delta(1000, 900) == 100


def test_twist_to_wheel_speeds():
    assert twist_to_wheel_speeds(0.2, 0.0, BASE, 0.3) == (0.2, 0.2)
    left, right = twist_to_wheel_speeds(0.0, 1.0, BASE, 0.5)
    assert left == pytest.approx(-BASE / 2) and right == pytest.approx(BASE / 2)


def test_twist_scaling_keeps_turn_radius():
    left, right = twist_to_wheel_speeds(0.5, 2.0, BASE, 0.3)
    assert max(abs(left), abs(right)) == pytest.approx(0.3)
    assert left / right == pytest.approx((0.5 - BASE) / (0.5 + BASE))


def test_straight_drive_matches_hardware_test():
    # Measured on the robot: 0.5 s at 100 mm/s -> left +119, right +117 counts.
    odom = DiffDriveOdometry(BASE, DIAMETER, CPR)
    odom.update(63877, 430, 0.0)
    assert odom.update(63996, 547, 0.5)
    assert odom.x == pytest.approx(118 * M_PER_COUNT, rel=1e-3)
    assert abs(odom.y) < 0.001
    assert odom.linear == pytest.approx(118 * M_PER_COUNT / 0.5, rel=1e-3)


def test_spin_in_place_and_rollover():
    odom = DiffDriveOdometry(BASE, DIAMETER, CPR)
    odom.update(10, 65530, 0.0)
    assert odom.update(65446, 94, 1.0)            # left -100, right +100 counts across rollover
    assert odom.x == pytest.approx(0.0, abs=1e-9)
    assert odom.theta == pytest.approx(200 * M_PER_COUNT / BASE)
    assert odom.left_pos == pytest.approx(-100 * M_PER_COUNT / (DIAMETER / 2))


def test_implausible_jump_is_rejected():
    odom = DiffDriveOdometry(BASE, DIAMETER, CPR)
    odom.update(0, 0, 0.0)
    assert not odom.update(5000, 5000, 0.015)
    assert odom.rejected == 1 and odom.x == 0.0
    assert odom.update(5010, 5010, 0.030)


def test_first_sample_only_initialises():
    odom = DiffDriveOdometry(BASE, DIAMETER, CPR)
    assert not odom.update(123, 456, 0.0)
    assert odom.x == 0.0
