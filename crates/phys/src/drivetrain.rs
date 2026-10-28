//! Engine, friction clutch, automatic gearbox and limited-slip differential.
//!
//! The engine is a spinning inertia driven by a torque curve. A friction clutch couples it to the
//! gearbox input; the clutch torque is limited, so launching from a standstill and a lugging engine
//! both slip instead of stalling. Wheel speeds feed back into the gearbox input speed.
use crate::math::*;

#[derive(Clone, Debug)]
pub struct DrivetrainParams {
    /// (rpm, N·m at full throttle), ascending rpm.
    pub torque_curve: Vec<(f64, f64)>,
    pub idle_rpm: f64,
    pub max_rpm: f64,
    pub engine_inertia: f64,
    /// Forward ratios, 1st first.
    pub gears: Vec<f64>,
    pub reverse: f64,
    pub final_drive: f64,
    pub efficiency: f64,
    /// Clutch torque per rad/s of slip, and its capacity at full engagement (N·m).
    pub clutch_k: f64,
    pub clutch_max: f64,
    /// Limited-slip differential: torque per rad/s of wheel speed difference, and its cap (N·m).
    pub lsd_k: f64,
    pub lsd_max: f64,
    pub shift_time: f64,
}

impl Default for DrivetrainParams {
    fn default() -> Self {
        DrivetrainParams {
            torque_curve: vec![(800.0, 110.0), (1500.0, 160.0), (2500.0, 205.0), (3500.0, 225.0), (4500.0, 230.0), (5500.0, 215.0), (6500.0, 185.0)],
            idle_rpm: 850.0,
            max_rpm: 6700.0,
            engine_inertia: 0.30,
            gears: vec![3.62, 2.19, 1.54, 1.14, 0.92, 0.76],
            reverse: 3.40,
            final_drive: 3.90,
            efficiency: 0.90,
            clutch_k: 60.0,
            clutch_max: 520.0,
            lsd_k: 300.0,
            lsd_max: 500.0,
            shift_time: 0.30,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Drivetrain {
    /// Engine speed, rad/s.
    pub w_e: f64,
    /// -1 reverse, 1.. forward.
    pub gear: i32,
    pub shift_t: f64,
    pub dwell: f64,
    pub clutch: f64,
    pub engine_torque: f64,
    pub clutch_torque: f64,
}

pub struct DriveOut {
    pub torque_l: f64,
    pub torque_r: f64,
    /// d(wheel torque)/d(wheel speed) magnitude from the clutch coupling, for implicit integration.
    pub stiffness: f64,
}

const RPM: f64 = 60.0 / (2.0 * std::f64::consts::PI);

pub fn rpm_of(w: f64) -> f64 {
    w * RPM
}

impl Drivetrain {
    pub fn new(p: &DrivetrainParams) -> Self {
        Drivetrain { w_e: p.idle_rpm / RPM, gear: 1, shift_t: 0.0, dwell: 0.0, clutch: 0.0, engine_torque: 0.0, clutch_torque: 0.0 }
    }

    pub fn rpm(&self) -> f64 {
        self.w_e * RPM
    }

    pub fn ratio(&self, p: &DrivetrainParams) -> f64 {
        match self.gear {
            g if g < 0 => -p.reverse * p.final_drive,
            g => p.gears[(g as usize - 1).min(p.gears.len() - 1)] * p.final_drive,
        }
    }

    fn curve(p: &DrivetrainParams, rpm: f64) -> f64 {
        let c = &p.torque_curve;
        if rpm <= c[0].0 {
            return c[0].1;
        }
        for i in 1..c.len() {
            if rpm <= c[i].0 {
                let t = (rpm - c[i - 1].0) / (c[i].0 - c[i - 1].0);
                return c[i - 1].1 + t * (c[i].1 - c[i - 1].1);
            }
        }
        c[c.len() - 1].1
    }

    pub fn step(&mut self, dt: f64, p: &DrivetrainParams, throttle: f64, reverse: bool, w_l: f64, w_r: f64) -> DriveOut {
        // --- gear selection ---
        self.dwell += dt;
        if self.shift_t > 0.0 {
            self.shift_t = (self.shift_t - dt).max(0.0);
        }
        if reverse {
            if self.gear != -1 {
                self.gear = -1;
                self.shift_t = p.shift_time;
            }
        } else {
            if self.gear < 1 {
                self.gear = 1;
                self.shift_t = p.shift_time;
            }
            let rpm = self.rpm();
            let up = 2200.0 + 3600.0 * throttle;
            let down = 1500.0 + 1500.0 * throttle;
            if self.shift_t == 0.0 && self.dwell > 1.0 {
                if rpm > up && (self.gear as usize) < p.gears.len() {
                    self.gear += 1;
                    self.shift_t = p.shift_time;
                    self.dwell = 0.0;
                } else if rpm < down && self.gear > 1 {
                    self.gear -= 1;
                    self.shift_t = p.shift_time;
                    self.dwell = 0.0;
                }
            }
        }

        // --- engine torque ---
        let rpm = self.rpm();
        let rev_cut = if rpm > p.max_rpm { 0.0 } else { 1.0 };
        let thr = throttle * rev_cut;
        let mut t_eng = thr * Self::curve(p, rpm) - (1.0 - thr) * (25.0 + 0.012 * rpm);
        if rpm < p.idle_rpm {
            t_eng += ((p.idle_rpm - rpm) * 1.0).clamp(0.0, 150.0);
        }
        self.engine_torque = t_eng;

        // --- clutch ---
        let ratio = self.ratio(p);
        let w_in = ratio * 0.5 * (w_l + w_r);
        let shifting = self.shift_t > 0.0;
        let engage = if shifting { 0.0 } else { smoothstep(1000.0, 1700.0, rpm) };
        self.clutch = engage;
        let cap = p.clutch_max * engage;
        let raw = p.clutch_k * (self.w_e - w_in);
        let t_c = raw.clamp(-cap, cap);
        self.clutch_torque = t_c;
        self.w_e += dt * (t_eng - t_c) / p.engine_inertia;
        self.w_e = self.w_e.max(0.4 * p.idle_rpm / RPM);

        // --- differential ---
        let total = t_c * ratio * p.efficiency;
        let lsd = (p.lsd_k * (w_l - w_r)).clamp(-p.lsd_max, p.lsd_max) * if engage > 0.0 { 1.0 } else { 0.0 };
        let stiffness = if raw.abs() < cap { p.clutch_k * ratio * ratio * p.efficiency * 0.25 } else { 0.0 };
        DriveOut { torque_l: total * 0.5 - lsd, torque_r: total * 0.5 + lsd, stiffness }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idles_without_stalling_in_neutral_like_load() {
        let p = DrivetrainParams::default();
        let mut d = Drivetrain::new(&p);
        for _ in 0..5000 {
            d.step(0.001, &p, 0.0, false, 0.0, 0.0);
        }
        assert!(d.rpm() > 700.0 && d.rpm() < 1100.0, "idle rpm {}", d.rpm());
    }

    #[test]
    fn revs_up_on_throttle_and_hits_limiter() {
        let p = DrivetrainParams::default();
        let mut d = Drivetrain::new(&p);
        d.shift_t = 10.0; // clutch open
        for _ in 0..3000 {
            d.step(0.001, &p, 1.0, false, 0.0, 0.0);
        }
        assert!(d.rpm() > 6000.0 && d.rpm() < 7200.0, "rpm {}", d.rpm());
    }

    #[test]
    fn clutch_torque_is_capped_and_sign_follows_slip() {
        let p = DrivetrainParams::default();
        let mut d = Drivetrain::new(&p);
        d.w_e = 3000.0 / RPM;
        let o = d.step(0.001, &p, 1.0, false, 0.0, 0.0);
        assert!(d.clutch_torque <= p.clutch_max + 1e-9 && d.clutch_torque > 0.0);
        assert!(o.torque_l > 0.0 && o.torque_r > 0.0);
        // Overrun (wheels faster than engine) gives engine braking: negative torque at the wheels.
        let mut d = Drivetrain::new(&p);
        d.w_e = 1500.0 / RPM;
        d.gear = 3;
        d.dwell = 0.0;
        let w = 200.0;
        let o = d.step(0.001, &p, 0.0, false, w, w);
        assert!(o.torque_l < 0.0 && o.torque_r < 0.0);
    }

    #[test]
    fn reverse_gear_pushes_backwards() {
        let p = DrivetrainParams::default();
        let mut d = Drivetrain::new(&p);
        d.w_e = 2500.0 / RPM;
        for _ in 0..400 {
            d.step(0.001, &p, 1.0, true, 0.0, 0.0);
        }
        let o = d.step(0.001, &p, 1.0, true, 0.0, 0.0);
        assert_eq!(d.gear, -1);
        assert!(o.torque_l < 0.0 && o.torque_r < 0.0, "{} {}", o.torque_l, o.torque_r);
    }
}
