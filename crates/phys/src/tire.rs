//! Tire force model: combined longitudinal / lateral slip with a Pacejka-style shape.
//!
//! Slip is normalised by the slip at peak grip, the two components are combined into one slip
//! magnitude, and the resulting force is distributed along the slip direction (friction-ellipse
//! behaviour). Below a reference speed `v0` slip is regularised so a stopped car does not jitter.

#[derive(Clone, Copy, Debug)]
pub struct TireParams {
    /// Peak friction coefficient on a reference (dry asphalt) surface.
    pub mu: f64,
    /// Slip ratio at peak longitudinal force.
    pub kappa_peak: f64,
    /// tan(slip angle) at peak lateral force.
    pub tan_alpha_peak: f64,
    /// Fraction of mu lost per unit of (Fz/Fz0 - 1).
    pub load_sens: f64,
    /// Reference vertical load (N).
    pub fz0: f64,
    /// Low-speed regularisation speed (m/s).
    pub v0: f64,
}

impl Default for TireParams {
    fn default() -> Self {
        TireParams { mu: 1.1, kappa_peak: 0.11, tan_alpha_peak: 0.14, load_sens: 0.10, fz0: 3800.0, v0: 3.0 }
    }
}

const SHAPE_C: f64 = 1.5;

/// Normalised friction curve: 0 at s=0, 1 at s=1 (peak), falling to ~0.71 at large slip.
pub fn shape(s: f64) -> f64 {
    let b = (std::f64::consts::PI / (2.0 * SHAPE_C)).tan();
    (SHAPE_C * (b * s).atan()).sin()
}

#[derive(Clone, Copy, Debug, Default)]
pub struct TireOut {
    /// Force along the wheel heading (N), positive = pushes the car forward.
    pub fx: f64,
    /// Force along the wheel's left axis (N).
    pub fy: f64,
    pub kappa: f64,
    pub tan_alpha: f64,
    /// Friction limit mu*Fz used for this evaluation (N).
    pub limit: f64,
}

/// `v_long`, `v_lat`: velocity of the contact patch along / across the wheel heading (m/s).
/// `omega`: wheel spin (rad/s, positive = rolling forward). `surface`: surface grip multiplier.
pub fn tire_force(p: &TireParams, fz: f64, surface: f64, v_long: f64, v_lat: f64, omega: f64, radius: f64) -> TireOut {
    if fz <= 0.0 {
        return TireOut::default();
    }
    let den = v_long.abs().max(p.v0);
    let kappa = (omega * radius - v_long) / den;
    let tan_a = v_lat / den;
    let sx = kappa / p.kappa_peak;
    let sy = tan_a / p.tan_alpha_peak;
    let s = (sx * sx + sy * sy).sqrt();
    let load = (1.0 - p.load_sens * (fz / p.fz0 - 1.0)).clamp(0.6, 1.3);
    let limit = p.mu * surface * load * fz;
    if s < 1e-9 {
        return TireOut { fx: 0.0, fy: 0.0, kappa, tan_alpha: tan_a, limit };
    }
    let f = limit * shape(s);
    TireOut { fx: f * sx / s, fy: -f * sy / s, kappa, tan_alpha: tan_a, limit }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shape_peaks_at_one_and_falls_off() {
        assert!((shape(1.0) - 1.0).abs() < 1e-9);
        assert!(shape(0.5) < 1.0 && shape(0.0).abs() < 1e-12);
        assert!(shape(5.0) < 0.85 && shape(5.0) > 0.70 && shape(1e4) < 0.72);
    }

    #[test]
    fn force_never_exceeds_friction_limit() {
        let p = TireParams::default();
        for &(vl, vt, om) in &[(10.0, 3.0, 40.0), (0.0, 0.0, 0.0), (30.0, -9.0, 0.0), (-5.0, 2.0, -50.0), (0.3, 0.2, 5.0)] {
            let o = tire_force(&p, 4000.0, 1.0, vl, vt, om, 0.33);
            assert!((o.fx * o.fx + o.fy * o.fy).sqrt() <= o.limit + 1e-6);
        }
    }

    #[test]
    fn forces_oppose_slip() {
        let p = TireParams::default();
        let o = tire_force(&p, 4000.0, 1.0, 15.0, 1.0, 15.0 / 0.33, 0.33);
        assert!(o.fy < 0.0, "sliding left must push right");
        let o = tire_force(&p, 4000.0, 1.0, 15.0, 0.0, 17.0 / 0.33, 0.33);
        assert!(o.fx > 0.0, "wheel faster than ground must push the car forward");
        let o = tire_force(&p, 4000.0, 1.0, 15.0, 0.0, 12.0 / 0.33, 0.33);
        assert!(o.fx < 0.0, "wheel slower than ground must brake the car");
    }
}
