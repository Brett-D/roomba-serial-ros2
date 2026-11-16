//! Battery state: the Roomba's own coulomb counter, or the state of charge from the pack voltage for
//! batteries the Roomba can't gauge itself (port of roomba_driver/battery.py and the
//! _publish_battery arithmetic of node.py).

/// Resting voltage of one Li-ion (NMC) cell -> state of charge, typical of 18650 power-tool cells.
/// Good to roughly +-10 % with the reading smoothed and corrected for sag under load.
pub const LI_ION_CURVE: [(f64, f64); 12] = [
    (3.00, 0.00), (3.45, 0.05), (3.60, 0.10), (3.67, 0.20), (3.71, 0.30), (3.75, 0.40),
    (3.79, 0.50), (3.85, 0.60), (3.92, 0.70), (4.00, 0.80), (4.10, 0.90), (4.20, 1.00),
];

/// State of charge (0-1) of a Li-ion cell at this resting voltage, linearly interpolated.
pub fn li_ion_fraction(cell_voltage: f64) -> f64 {
    if cell_voltage.is_nan() {
        return f64::NAN;
    }
    if cell_voltage <= LI_ION_CURVE[0].0 {
        return 0.0;
    }
    for w in LI_ION_CURVE.windows(2) {
        let ((v0, f0), (v1, f1)) = (w[0], w[1]);
        if cell_voltage <= v1 {
            return f0 + (f1 - f0) * (cell_voltage - v0) / (v1 - v0);
        }
    }
    1.0
}

// sensor_msgs/BatteryState constants
pub const STATUS_CHARGING: u8 = 1;
pub const STATUS_DISCHARGING: u8 = 2;
pub const STATUS_NOT_CHARGING: u8 = 3;
pub const STATUS_FULL: u8 = 4;
pub const HEALTH_GOOD: u8 = 1;
pub const HEALTH_UNSPEC_FAILURE: u8 = 5;
pub const TECHNOLOGY_NIMH: u8 = 1;
pub const TECHNOLOGY_LION: u8 = 2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Gauge {
    /// The robot's own coulomb counter (stock battery).
    Roomba,
    /// The pack voltage through a Li-ion discharge curve, for another pack wired to the battery
    /// contacts (e.g. a power-tool battery), where the robot's mAh and temperature are made up.
    Voltage { cells: i64, capacity_ah: f64, resistance_ohm: f64, alpha: f64 },
}

impl Gauge {
    /// battery_gauge 'voltage' parameters; alpha is the per-sample weight of the smoothing filter.
    pub fn voltage(cells: i64, capacity_ah: f64, resistance_ohm: f64, rate_hz: f64, smoothing_s: f64) -> Self {
        let alpha = 1.0 - (-1.0 / (rate_hz * smoothing_s).max(1e-3)).exp();
        Gauge::Voltage { cells, capacity_ah, resistance_ohm, alpha }
    }
}

/// The battery packets of a sensor frame.
#[derive(Clone, Copy, Debug, Default)]
pub struct Reading {
    pub voltage_mv: i32,
    pub current_ma: i32,
    pub temperature_c: i32,
    pub charge_mah: i32,
    pub capacity_mah: i32,
    pub charging_state: i32,
    pub on_charger: bool,
}

/// BatteryState fields, in double precision until they are written into the float32 message.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BatteryFields {
    pub voltage: f64,
    pub current: f64,
    pub temperature: f64,
    pub charge: f64,
    pub capacity: f64,
    pub design_capacity: f64,
    pub percentage: f64,
    pub status: u8,
    pub health: u8,
    pub technology: u8,
}

/// Turns readings into BatteryState fields; keeps the voltage gauge's smoothed open-circuit voltage.
#[derive(Clone, Debug)]
pub struct BatteryGauge {
    pub gauge: Gauge,
    ocv: Option<f64>,
}

impl BatteryGauge {
    pub fn new(gauge: Gauge) -> Self {
        Self { gauge, ocv: None }
    }

    pub fn update(&mut self, r: &Reading) -> BatteryFields {
        let voltage = r.voltage_mv as f64 / 1000.0;
        let current = r.current_ma as f64 / 1000.0;
        let (temperature, percentage, capacity, design_capacity, charge, technology);
        match self.gauge {
            Gauge::Voltage { cells, capacity_ah, resistance_ohm, alpha } => {
                // current is negative while discharging: add back the sag across the pack
                // resistance, then smooth so motor load doesn't swing the gauge.
                let ocv = voltage - current * resistance_ohm;
                let smoothed = match self.ocv {
                    None => ocv,
                    Some(prev) => prev + alpha * (ocv - prev),
                };
                self.ocv = Some(smoothed);
                percentage = li_ion_fraction(smoothed / cells as f64);
                temperature = f64::NAN;
                capacity = if capacity_ah > 0.0 { capacity_ah } else { f64::NAN };
                design_capacity = capacity;
                charge = percentage * capacity;
                technology = TECHNOLOGY_LION;
            }
            Gauge::Roomba => {
                temperature = r.temperature_c as f64;
                charge = r.charge_mah as f64 / 1000.0;
                capacity = r.capacity_mah as f64 / 1000.0;
                design_capacity = f64::NAN;
                percentage = if r.capacity_mah != 0 {
                    (r.charge_mah as f64 / r.capacity_mah as f64).min(1.0)
                } else {
                    f64::NAN
                };
                technology = TECHNOLOGY_NIMH;
            }
        }
        let status = if (1..=3).contains(&r.charging_state) {
            STATUS_CHARGING
        } else if r.on_charger && percentage >= 0.99 {
            STATUS_FULL
        } else if r.on_charger {
            STATUS_NOT_CHARGING
        } else {
            STATUS_DISCHARGING
        };
        let health = if r.charging_state == 5 { HEALTH_UNSPEC_FAILURE } else { HEALTH_GOOD };
        BatteryFields { voltage, current, temperature, charge, capacity, design_capacity, percentage, status, health, technology }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn limits() {
        assert_eq!(li_ion_fraction(2.5), 0.0);
        assert_eq!(li_ion_fraction(3.0), 0.0);
        assert!(approx(li_ion_fraction(4.2), 1.0));
        assert_eq!(li_ion_fraction(4.35), 1.0);
    }

    #[test]
    fn interpolates_and_never_decreases() {
        assert!(approx(li_ion_fraction(3.79), 0.5));
        assert!(approx(li_ion_fraction(3.77), 0.45));
        let values: Vec<f64> = (0..300).map(|i| li_ion_fraction(2.9 + i as f64 * 0.005)).collect();
        assert!(values.windows(2).all(|w| w[1] >= w[0]));
    }

    #[test]
    fn five_cell_pack() {
        // An 18 V (5S) power-tool pack read 20.26 V at rest shortly after charging.
        let f = li_ion_fraction(20.26 / 5.0);
        assert!(0.8 < f && f < 0.9);
        assert_eq!(li_ion_fraction(15.0 / 5.0), 0.0);
    }

    #[test]
    fn nan() {
        assert!(li_ion_fraction(f64::NAN).is_nan());
    }

    #[test]
    fn voltage_gauge_corrects_sag_and_smooths() {
        let mut g = BatteryGauge::new(Gauge::voltage(5, 0.0, 0.1, 1.0, 30.0));
        let rest = Reading { voltage_mv: 19500, current_ma: -500, ..Default::default() };
        let first = g.update(&rest);
        // 19.5 V + 0.5 A * 0.1 ohm = 19.55 V open-circuit, 3.91 V per cell
        assert!(approx(first.percentage, li_ion_fraction(19.55 / 5.0)));
        assert!(first.temperature.is_nan() && first.capacity.is_nan() && first.charge.is_nan());
        assert_eq!((first.status, first.health, first.technology), (STATUS_DISCHARGING, HEALTH_GOOD, TECHNOLOGY_LION));
        // A load step moves the estimate by alpha of the way only.
        let loaded = g.update(&Reading { voltage_mv: 18500, current_ma: -2500, ..Default::default() });
        let alpha = 1.0 - (-1.0f64 / 30.0).exp();
        let ocv = 19.55 + alpha * ((18.5 + 0.25) - 19.55);
        assert!(approx(loaded.percentage, li_ion_fraction(ocv / 5.0)));
    }

    #[test]
    fn voltage_gauge_with_capacity() {
        let mut g = BatteryGauge::new(Gauge::voltage(5, 4.0, 0.1, 1.0, 30.0));
        let f = g.update(&Reading { voltage_mv: 21500, current_ma: 0, on_charger: true, ..Default::default() });
        assert_eq!((f.percentage, f.capacity, f.design_capacity, f.charge), (1.0, 4.0, 4.0, 4.0));
        assert_eq!(f.status, STATUS_FULL);
    }

    #[test]
    fn roomba_gauge_and_charging_states() {
        let mut g = BatteryGauge::new(Gauge::Roomba);
        let r = Reading { voltage_mv: 14695, current_ma: 398, temperature_c: 18, charge_mah: 2202, capacity_mah: 2696,
                          charging_state: 2, on_charger: true };
        let f = g.update(&r);
        assert!(approx(f.percentage, 2202.0 / 2696.0));
        assert_eq!((f.temperature, f.charge, f.capacity), (18.0, 2.202, 2.696));
        assert!(f.design_capacity.is_nan());
        assert_eq!((f.status, f.technology), (STATUS_CHARGING, TECHNOLOGY_NIMH));
        let f = g.update(&Reading { charging_state: 0, charge_mah: 2696, ..r });
        assert_eq!(f.status, STATUS_FULL);
        let f = g.update(&Reading { charging_state: 0, ..r });
        assert_eq!(f.status, STATUS_NOT_CHARGING);
        let f = g.update(&Reading { charging_state: 5, on_charger: false, capacity_mah: 0, ..r });
        assert!(f.percentage.is_nan());
        assert_eq!((f.status, f.health), (STATUS_DISCHARGING, HEALTH_UNSPEC_FAILURE));
    }
}
