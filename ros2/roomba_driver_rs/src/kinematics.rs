//! Differential-drive kinematics and encoder odometry for the Roomba (port of
//! roomba_driver/kinematics.py, same arithmetic so the poses match bit for bit).
use std::f64::consts::PI;

pub const ENCODER_MODULO: i64 = 65536;

/// Signed difference between two 16-bit encoder readings, handling rollover.
pub fn wrap_delta(new: i64, old: i64) -> i64 {
    (new - old + ENCODER_MODULO / 2).rem_euclid(ENCODER_MODULO) - ENCODER_MODULO / 2
}

/// (v m/s, w rad/s) -> (left, right) m/s, scaled down together to keep the turn radius.
pub fn twist_to_wheel_speeds(linear: f64, angular: f64, wheel_base: f64, max_wheel_speed: f64) -> (f64, f64) {
    let mut left = linear - angular * wheel_base / 2.0;
    let mut right = linear + angular * wheel_base / 2.0;
    let peak = left.abs().max(right.abs());
    if peak > max_wheel_speed && max_wheel_speed > 0.0 {
        let scale = max_wheel_speed / peak;
        left *= scale;
        right *= scale;
    }
    (left, right)
}

/// Integrates wheel encoder counts into a planar pose and velocity.
#[derive(Clone, Debug)]
pub struct DiffDriveOdometry {
    pub wheel_base: f64,
    pub wheel_radius: f64,
    pub meters_per_count: f64,
    pub max_wheel_speed: f64,
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub linear: f64,
    pub angular: f64,
    /// Wheel angles, radians.
    pub left_pos: f64,
    pub right_pos: f64,
    /// rad/s
    pub left_vel: f64,
    pub right_vel: f64,
    pub rejected: u64,
    prev: Option<(i64, i64, f64)>,
}

impl DiffDriveOdometry {
    pub fn new(wheel_base: f64, wheel_diameter: f64, counts_per_rev: f64, max_wheel_speed: f64) -> Self {
        Self {
            wheel_base,
            wheel_radius: wheel_diameter / 2.0,
            meters_per_count: PI * wheel_diameter / counts_per_rev,
            max_wheel_speed,
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear: 0.0,
            angular: 0.0,
            left_pos: 0.0,
            right_pos: 0.0,
            left_vel: 0.0,
            right_vel: 0.0,
            rejected: 0,
            prev: None,
        }
    }

    pub fn reset(&mut self) {
        self.x = 0.0;
        self.y = 0.0;
        self.theta = 0.0;
        self.linear = 0.0;
        self.angular = 0.0;
    }

    /// Feed one encoder sample. Returns true if the pose was updated.
    pub fn update(&mut self, left_counts: i64, right_counts: i64, stamp_s: f64) -> bool {
        let Some((pl, pr, pt)) = self.prev else {
            self.prev = Some((left_counts, right_counts, stamp_s));
            return false;
        };
        let dt = stamp_s - pt;
        if dt <= 0.0 {
            return false;
        }
        self.prev = Some((left_counts, right_counts, stamp_s));
        let dl = wrap_delta(left_counts, pl) as f64 * self.meters_per_count;
        let dr = wrap_delta(right_counts, pr) as f64 * self.meters_per_count;
        let limit = self.max_wheel_speed * dt + 0.01;
        if dl.abs() > limit || dr.abs() > limit {
            self.rejected += 1;
            return false;
        }

        let d = (dl + dr) / 2.0;
        let dtheta = (dr - dl) / self.wheel_base;
        self.x += d * (self.theta + dtheta / 2.0).cos();
        self.y += d * (self.theta + dtheta / 2.0).sin();
        self.theta = (self.theta + dtheta).sin().atan2((self.theta + dtheta).cos());
        self.linear = d / dt;
        self.angular = dtheta / dt;

        self.left_pos += dl / self.wheel_radius;
        self.right_pos += dr / self.wheel_radius;
        self.left_vel = dl / self.wheel_radius / dt;
        self.right_vel = dr / self.wheel_radius / dt;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BASE: f64 = 0.235;
    const DIAMETER: f64 = 0.072;
    const CPR: f64 = 508.8;

    fn m_per_count() -> f64 {
        PI * DIAMETER / CPR
    }

    fn approx(a: f64, b: f64, rel: f64) -> bool {
        (a - b).abs() <= rel * b.abs().max(1e-12)
    }

    #[test]
    fn wrap_delta_handles_rollover() {
        assert_eq!(wrap_delta(5, 65530), 11);
        assert_eq!(wrap_delta(65530, 5), -11);
        assert_eq!(wrap_delta(1000, 900), 100);
    }

    #[test]
    fn twist_to_wheel_speeds_basic() {
        assert_eq!(twist_to_wheel_speeds(0.2, 0.0, BASE, 0.3), (0.2, 0.2));
        let (l, r) = twist_to_wheel_speeds(0.0, 1.0, BASE, 0.5);
        assert!(approx(l, -BASE / 2.0, 1e-6) && approx(r, BASE / 2.0, 1e-6));
    }

    #[test]
    fn twist_scaling_keeps_turn_radius() {
        let (l, r) = twist_to_wheel_speeds(0.5, 2.0, BASE, 0.3);
        assert!(approx(l.abs().max(r.abs()), 0.3, 1e-6));
        assert!(approx(l / r, (0.5 - BASE) / (0.5 + BASE), 1e-6));
    }

    #[test]
    fn straight_drive_matches_hardware_test() {
        // Measured on the robot: 0.5 s at 100 mm/s -> left +119, right +117 counts.
        let mut o = DiffDriveOdometry::new(BASE, DIAMETER, CPR, 0.75);
        o.update(63877, 430, 0.0);
        assert!(o.update(63996, 547, 0.5));
        assert!(approx(o.x, 118.0 * m_per_count(), 1e-3));
        assert!(o.y.abs() < 0.001);
        assert!(approx(o.linear, 118.0 * m_per_count() / 0.5, 1e-3));
    }

    #[test]
    fn spin_in_place_and_rollover() {
        let mut o = DiffDriveOdometry::new(BASE, DIAMETER, CPR, 0.75);
        o.update(10, 65530, 0.0);
        assert!(o.update(65446, 94, 1.0)); // left -100, right +100 counts across rollover
        assert!(o.x.abs() < 1e-9);
        assert!(approx(o.theta, 200.0 * m_per_count() / BASE, 1e-6));
        assert!(approx(o.left_pos, -100.0 * m_per_count() / (DIAMETER / 2.0), 1e-6));
    }

    #[test]
    fn implausible_jump_is_rejected() {
        let mut o = DiffDriveOdometry::new(BASE, DIAMETER, CPR, 0.75);
        o.update(0, 0, 0.0);
        assert!(!o.update(5000, 5000, 0.015));
        assert!(o.rejected == 1 && o.x == 0.0);
        assert!(o.update(5010, 5010, 0.030));
    }

    #[test]
    fn first_sample_only_initialises() {
        let mut o = DiffDriveOdometry::new(BASE, DIAMETER, CPR, 0.75);
        assert!(!o.update(123, 456, 0.0));
        assert_eq!(o.x, 0.0);
    }

    #[test]
    fn non_increasing_stamp_is_ignored() {
        let mut o = DiffDriveOdometry::new(BASE, DIAMETER, CPR, 0.75);
        o.update(0, 0, 1.0);
        assert!(!o.update(10, 10, 1.0));
        assert!(o.update(10, 10, 1.1));
        assert!(o.x > 0.0);
    }
}
