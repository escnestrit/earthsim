//! WGS84, ECEF, local East-North-Up frames and UTM (Krüger series). float64 throughout.
//! A `LocalFrame` is the "floating origin": when the car gets far from it, make a new one and re-base.

pub const A: f64 = 6378137.0;
pub const F: f64 = 1.0 / 298.257223563;
pub const E2: f64 = F * (2.0 - F);

pub fn geodetic_to_ecef(lat: f64, lon: f64, h: f64) -> [f64; 3] {
    let (p, l) = (lat.to_radians(), lon.to_radians());
    let n = A / (1.0 - E2 * p.sin().powi(2)).sqrt();
    [(n + h) * p.cos() * l.cos(), (n + h) * p.cos() * l.sin(), (n * (1.0 - E2) + h) * p.sin()]
}

pub fn ecef_to_geodetic(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let lon = y.atan2(x);
    let r = x.hypot(y);
    let mut p = z.atan2(r * (1.0 - E2));
    let mut h = 0.0;
    for _ in 0..10 {
        let n = A / (1.0 - E2 * p.sin().powi(2)).sqrt();
        h = r / p.cos() - n;
        p = z.atan2(r * (1.0 - E2 * n / (n + h)));
    }
    (p.to_degrees(), lon.to_degrees(), h)
}

#[derive(Clone, Copy, Debug)]
pub struct LocalFrame {
    pub lat: f64,
    pub lon: f64,
    pub h: f64,
    o: [f64; 3],
    e: [f64; 3],
    n: [f64; 3],
    u: [f64; 3],
}

impl LocalFrame {
    pub fn new(lat: f64, lon: f64, h: f64) -> LocalFrame {
        let o = geodetic_to_ecef(lat, lon, h);
        let (p, l) = (lat.to_radians(), lon.to_radians());
        let (sp, cp, sl, cl) = (p.sin(), p.cos(), l.sin(), l.cos());
        LocalFrame { lat, lon, h, o, e: [-sl, cl, 0.0], n: [-sp * cl, -sp * sl, cp], u: [cp * cl, cp * sl, sp] }
    }

    /// (east, north, up) in metres of a geodetic point.
    pub fn to_local(&self, lat: f64, lon: f64, h: f64) -> [f64; 3] {
        let c = geodetic_to_ecef(lat, lon, h);
        let d = [c[0] - self.o[0], c[1] - self.o[1], c[2] - self.o[2]];
        let dot = |v: &[f64; 3]| d[0] * v[0] + d[1] * v[1] + d[2] * v[2];
        [dot(&self.e), dot(&self.n), dot(&self.u)]
    }

    /// (lat, lon, h) of a local point.
    pub fn from_local(&self, e: f64, n: f64, u: f64) -> (f64, f64, f64) {
        let o = self.o;
        let x = o[0] + e * self.e[0] + n * self.n[0] + u * self.u[0];
        let y = o[1] + e * self.e[1] + n * self.n[1] + u * self.u[1];
        let z = o[2] + e * self.e[2] + n * self.n[2] + u * self.u[2];
        ecef_to_geodetic(x, y, z)
    }
}

// ---- UTM (Krüger n-series, 3 terms) ----
const K0: f64 = 0.9996;
const E0: f64 = 500000.0;

struct Utm {
    n: f64,
    aa: f64,
    al: [f64; 3],
}

fn utm_consts() -> Utm {
    let n = F / (2.0 - F);
    Utm {
        n,
        aa: A / (1.0 + n) * (1.0 + n * n / 4.0 + n.powi(4) / 64.0),
        al: [n / 2.0 - 2.0 * n * n / 3.0 + 5.0 * n.powi(3) / 16.0, 13.0 * n * n / 48.0 - 3.0 * n.powi(3) / 5.0, 61.0 * n.powi(3) / 240.0],
    }
}

pub fn utm_zone(lon: f64) -> i32 {
    (((lon + 180.0) / 6.0).floor() as i32).rem_euclid(60) + 1
}

/// (zone, easting, northing).
pub fn utm_forward(lat: f64, lon: f64) -> (i32, f64, f64) {
    let u = utm_consts();
    let zone = utm_zone(lon);
    let lam = (lon - ((zone - 1) as f64 * 6.0 - 180.0 + 3.0)).to_radians();
    let phi = lat.to_radians();
    let c = 2.0 * u.n.sqrt() / (1.0 + u.n);
    let t = ((phi.sin()).atanh() - c * (c * phi.sin()).atanh()).sinh();
    let xi_ = t.atan2(lam.cos());
    let eta_ = (lam.sin() / (1.0 + t * t).sqrt()).atanh();
    let mut xi = xi_;
    let mut eta = eta_;
    for j in 1..=3 {
        let jf = 2.0 * j as f64;
        xi += u.al[j - 1] * (jf * xi_).sin() * (jf * eta_).cosh();
        eta += u.al[j - 1] * (jf * xi_).cos() * (jf * eta_).sinh();
    }
    let n0 = if lat >= 0.0 { 0.0 } else { 1e7 };
    (zone, E0 + K0 * u.aa * eta, n0 + K0 * u.aa * xi)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MINSK: (f64, f64) = (53.9045, 27.5615);

    #[test]
    fn ecef_matches_python_reference() {
        let e = geodetic_to_ecef(MINSK.0, MINSK.1, 0.0);
        assert!((e[0] - 3338445.3099130616).abs() < 1e-6);
        assert!((e[1] - 1742441.6575398142).abs() < 1e-6);
        assert!((e[2] - 5130488.779093844).abs() < 1e-6);
    }

    #[test]
    fn geodetic_roundtrip_everywhere() {
        for &(la, lo, h) in &[(0.0, 0.0, 0.0), (89.9, 10.0, 100.0), (-89.9, -170.0, 0.0), (53.9, 27.56, 250.0), (-33.86, 151.2, 5.0), (35.68, 139.7, 40.0), (0.001, 179.999, 0.0)] {
            let c = geodetic_to_ecef(la, lo, h);
            let (a, b, hh) = ecef_to_geodetic(c[0], c[1], c[2]);
            assert!((a - la).abs() < 1e-9 && (b - lo).abs() < 1e-9 && (hh - h).abs() < 1e-4, "{la} {lo} {h} -> {a} {b} {hh}");
        }
    }

    #[test]
    fn local_frame_matches_python_and_roundtrips() {
        let fr = LocalFrame::new(MINSK.0, MINSK.1, 0.0);
        let p = fr.to_local(53.9145, 27.5715, 0.0);
        assert!((p[0] - 657.1007374146391).abs() < 1e-6 && (p[1] - 1113.079487325863).abs() < 1e-6, "{p:?}");
        let (la, lo, _) = fr.from_local(p[0], p[1], p[2]);
        assert!((la - 53.9145).abs() < 1e-9 && (lo - 27.5715).abs() < 1e-9);
        let q = fr.to_local(0.0, 0.0, 0.0);
        assert!(q.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn one_degree_of_latitude_is_about_111_km() {
        let fr = LocalFrame::new(10.0, 20.0, 0.0);
        let p = fr.to_local(11.0, 20.0, 0.0);
        assert!((p[1] - 110_600.0).abs() < 1_500.0 && p[0].abs() < 1.0, "{p:?}");
    }

    #[test]
    fn utm_matches_python_reference() {
        let (z, e, n) = utm_forward(MINSK.0, MINSK.1);
        assert_eq!(z, 35);
        assert!((e - 536890.078875614).abs() < 1e-6 && (n - 5973042.43506374).abs() < 1e-6, "{e} {n}");
        assert_eq!(utm_zone(-180.0), 1);
        assert_eq!(utm_zone(179.999), 60);
    }
}
