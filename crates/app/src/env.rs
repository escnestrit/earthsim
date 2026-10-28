//! Sun, sky and ambient light as a function of the hour of day (ported from the WebGL prototype).
use crate::m4::{norm, V3f};

fn smooth(e0: f32, e1: f32, x: f32) -> f32 {
    let t = ((x - e0) / (e1 - e0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
fn lerp3(a: V3f, b: V3f, t: f32) -> V3f {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]
}

#[derive(Clone, Copy, Debug)]
pub struct Lighting {
    /// Unit vector towards the dominant light (sun by day, "moon" by night).
    pub light_dir: V3f,
    pub sun_dir: V3f,
    pub day: f32,
    pub elevation: f32,
    pub sun_col: V3f,
    pub sun_i: f32,
    pub zenith: V3f,
    pub horizon: V3f,
    pub sky_amb: V3f,
    pub ground_amb: V3f,
}

pub fn lighting(hour: f32) -> Lighting {
    let a = (hour - 6.0) / 12.0 * std::f32::consts::PI;
    let el = a.sin();
    let sun_dir = norm([a.cos() * 0.85, -0.45, el]);
    let day = smooth(-0.08, 0.2, el);
    let light = if el >= 0.0 { sun_dir } else { [-sun_dir[0], -sun_dir[1], -sun_dir[2]] };
    let low = smooth(0.45, 0.0, el);
    let sun_col = [1.0, 0.95 - 0.4 * low, 0.85 - 0.55 * low];
    let zen = lerp3([0.0025, 0.004, 0.012], [0.07, 0.18, 0.52], day);
    let hor = lerp3([0.012, 0.016, 0.035], [0.45, 0.58, 0.80], day);
    let tw = (-(el / 0.18).powi(2)).exp();
    let hor_t = [hor[0] + 0.45 * tw, hor[1] + 0.18 * tw, hor[2] + 0.02 * tw];
    let k = 0.65 + 0.15 * el.max(0.0);
    let sa = lerp3([0.012, 0.018, 0.04], [0.55, 0.68, 0.95], day);
    Lighting {
        light_dir: light,
        sun_dir,
        day,
        elevation: el,
        sun_col: if day > 0.01 { sun_col } else { [0.4, 0.5, 0.9] },
        sun_i: if el >= 0.0 { 2.5 * day } else { 0.12 * (1.0 - day) },
        zenith: zen,
        horizon: hor_t,
        sky_amb: [sa[0] * k, sa[1] * k, sa[2] * k],
        ground_amb: lerp3([0.006, 0.007, 0.012], [0.20, 0.17, 0.13], day),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn noon_is_bright_and_midnight_is_dark() {
        let n = lighting(12.0);
        let m = lighting(0.0);
        assert!(n.day > 0.99 && n.sun_i > 2.4 && n.light_dir[2] > 0.9);
        assert!(m.day < 0.01 && m.sun_i < 0.2);
        assert!(m.light_dir[2] > 0.0, "night light must still come from above");
        assert!(n.horizon[2] > m.horizon[2] * 5.0);
    }

    #[test]
    fn all_outputs_are_finite_for_every_hour() {
        for h in 0..96 {
            let l = lighting(h as f32 * 0.25);
            for v in l.sun_col.iter().chain(&l.zenith).chain(&l.horizon).chain(&l.sky_amb).chain(&l.ground_amb).chain(&l.light_dir) {
                assert!(v.is_finite() && *v >= -1.0001 && *v <= 3.0);
            }
        }
    }

    #[test]
    fn sunrise_is_in_the_east_afternoon_in_the_west() {
        // x axis = east: sun moves from +x (morning) to -x (afternoon)
        assert!(lighting(8.0).sun_dir[0] > 0.0);
        assert!(lighting(16.0).sun_dir[0] < 0.0);
    }
}
