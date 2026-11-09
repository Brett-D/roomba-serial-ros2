"""Differential-drive kinematics and encoder odometry for the Roomba."""
import math

ENCODER_MODULO = 65536


def wrap_delta(new, old):
    """Signed difference between two 16-bit encoder readings, handling rollover."""
    return (int(new) - int(old) + ENCODER_MODULO // 2) % ENCODER_MODULO - ENCODER_MODULO // 2


def twist_to_wheel_speeds(linear, angular, wheel_base, max_wheel_speed):
    """(v m/s, w rad/s) -> (left, right) m/s, scaled down together to keep the turn radius."""
    left = linear - angular * wheel_base / 2.0
    right = linear + angular * wheel_base / 2.0
    peak = max(abs(left), abs(right))
    if peak > max_wheel_speed > 0:
        scale = max_wheel_speed / peak
        left *= scale
        right *= scale
    return left, right


class DiffDriveOdometry:
    """Integrates wheel encoder counts into a planar pose and velocity."""

    def __init__(self, wheel_base, wheel_diameter, counts_per_rev, max_wheel_speed=0.75):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_diameter / 2.0
        self.meters_per_count = math.pi * wheel_diameter / counts_per_rev
        self.max_wheel_speed = max_wheel_speed
        self.x = self.y = self.theta = 0.0
        self.linear = self.angular = 0.0
        self.left_pos = self.right_pos = 0.0     # wheel angles, radians
        self.left_vel = self.right_vel = 0.0     # rad/s
        self.rejected = 0
        self._prev = None

    def reset(self):
        self.x = self.y = self.theta = 0.0
        self.linear = self.angular = 0.0

    def update(self, left_counts, right_counts, stamp_s):
        """Feed one encoder sample. Returns True if the pose was updated."""
        if self._prev is None:
            self._prev = (left_counts, right_counts, stamp_s)
            return False
        pl, pr, pt = self._prev
        dt = stamp_s - pt
        if dt <= 0:
            return False
        self._prev = (left_counts, right_counts, stamp_s)
        dl = wrap_delta(left_counts, pl) * self.meters_per_count
        dr = wrap_delta(right_counts, pr) * self.meters_per_count
        limit = self.max_wheel_speed * dt + 0.01
        if abs(dl) > limit or abs(dr) > limit:
            self.rejected += 1
            return False

        d = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.wheel_base
        self.x += d * math.cos(self.theta + dtheta / 2.0)
        self.y += d * math.sin(self.theta + dtheta / 2.0)
        self.theta = math.atan2(math.sin(self.theta + dtheta), math.cos(self.theta + dtheta))
        self.linear = d / dt
        self.angular = dtheta / dt

        self.left_pos += dl / self.wheel_radius
        self.right_pos += dr / self.wheel_radius
        self.left_vel = dl / self.wheel_radius / dt
        self.right_vel = dr / self.wheel_radius / dt
        return True
