//! Towers, domes and spires of places of worship (`Building::landmark`). Typical shapes placed by rule on
//! the footprint: an Orthodox church gets a bell tower at its west end and a drum with an onion dome over the
//! body, a Western church a tower with a pointed spire, a mosque a dome and a minaret. Not surveyed: the
//! real building may differ in every detail.
use crate::mesh::{mat, Mb, FLOATS_PER_VERTEX};
use crate::model::{Building, Landmark};

type V = [f64; 3];

const GOLD: [f32; 3] = [1.35, 1.02, 0.42];

fn push_col(m: &Mb, col: &mut Vec<f32>, c: [f32; 3]) {
    let n = m.a.len() / FLOATS_PER_VERTEX;
    while col.len() / 3 < n {
        col.extend_from_slice(&c);
    }
}

fn h01(seed: u64, k: u64) -> f64 {
    let mut h = seed ^ k.wrapping_mul(0x9E37_79B9_7F4A_7C15);
    h ^= h >> 31;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 27;
    (h % 10_000) as f64 / 10_000.0
}

/// Surface of revolution about a vertical axis: `prof` is (radius, height above z0) from the bottom up;
/// a last radius of 0 closes it at the top.
pub fn revolve(m: &mut Mb, cx: f64, cy: f64, z0: f64, prof: &[(f64, f64)], seg: usize, mt: f32) {
    let tau = std::f64::consts::TAU;
    for w in prof.windows(2) {
        let ((r0, z_0), (r1, z_1)) = (w[0], w[1]);
        // outward normal of the profile segment
        let (dr, dz) = (r1 - r0, z_1 - z_0);
        let l = dr.hypot(dz).max(1e-9);
        let (nr, nz) = (dz / l, -dr / l);
        for i in 0..seg {
            let (a0, a1) = (i as f64 / seg as f64 * tau, (i + 1) as f64 / seg as f64 * tau);
            let p = |a: f64, r: f64, z: f64| -> V { [cx + a.cos() * r, cy + a.sin() * r, z0 + z] };
            let n = |a: f64| -> V { [a.cos() * nr, a.sin() * nr, nz] };
            let (q0, q1, q2, q3) = (p(a0, r0, z_0), p(a1, r0, z_0), p(a1, r1, z_1), p(a0, r1, z_1));
            if r0 > 1e-6 {
                m.v(q0, n(a0), 0.0, 0.0, mt);
                m.v(q1, n(a1), 0.0, 0.0, mt);
                m.v(q2, n(a1), 0.0, 0.0, mt);
            }
            if r1 > 1e-6 {
                m.v(q0, n(a0), 0.0, 0.0, mt);
                m.v(q2, n(a1), 0.0, 0.0, mt);
                m.v(q3, n(a0), 0.0, 0.0, mt);
            }
        }
    }
}

/// Square or polygonal prism with facade UVs (u = metres along the wall, v = height in storey units) so
/// the facade texture puts windows on it.
#[allow(clippy::too_many_arguments)]
fn prism(m: &mut Mb, c: [f64; 2], r: f64, sides: usize, rot: f64, z0: f64, z1: f64, mt: f32, vscale: f64, cap: Option<f32>) {
    let tau = std::f64::consts::TAU;
    let pt = |k: usize| {
        let a = rot + k as f64 / sides as f64 * tau;
        [c[0] + a.cos() * r, c[1] + a.sin() * r]
    };
    let side = 2.0 * r * (std::f64::consts::PI / sides as f64).sin();
    // whole 3 m bays per side, as on the main walls
    let bays = (side / 3.0).round().max(1.0);
    let mut u = 0.0;
    for k in 0..sides {
        let (a, b) = (pt(k), pt(k + 1));
        let am = rot + (k as f64 + 0.5) / sides as f64 * tau;
        let n = [am.cos(), am.sin(), 0.0];
        let (u0, u1) = (u, u + bays * 3.0);
        m.v([a[0], a[1], z0], n, u0, z0 * vscale, mt);
        m.v([b[0], b[1], z0], n, u1, z0 * vscale, mt);
        m.v([b[0], b[1], z1], n, u1, z1 * vscale, mt);
        m.v([a[0], a[1], z0], n, u0, z0 * vscale, mt);
        m.v([b[0], b[1], z1], n, u1, z1 * vscale, mt);
        m.v([a[0], a[1], z1], n, u0, z1 * vscale, mt);
        u = u1;
    }
    if let Some(cm) = cap {
        for k in 0..sides {
            let (a, b) = (pt(k), pt(k + 1));
            m.v([a[0], a[1], z1], [0.0, 0.0, 1.0], a[0], a[1], cm);
            m.v([b[0], b[1], z1], [0.0, 0.0, 1.0], b[0], b[1], cm);
            m.v([c[0], c[1], z1], [0.0, 0.0, 1.0], c[0], c[1], cm);
        }
    }
}

/// Pointed roof (pyramid / cone) over a regular polygon.
fn spire(m: &mut Mb, c: [f64; 2], r: f64, sides: usize, rot: f64, z0: f64, h: f64, mt: f32) {
    let tau = std::f64::consts::TAU;
    let apex = [c[0], c[1], z0 + h];
    for k in 0..sides {
        let (a0, a1) = (rot + k as f64 / sides as f64 * tau, rot + (k + 1) as f64 / sides as f64 * tau);
        let am = (a0 + a1) / 2.0;
        let n = {
            let (x, y, z) = (am.cos() * h, am.sin() * h, r * (std::f64::consts::PI / sides as f64).cos());
            let l = (x * x + y * y + z * z).sqrt();
            [x / l, y / l, z / l]
        };
        m.tri([c[0] + a0.cos() * r, c[1] + a0.sin() * r, z0], [c[0] + a1.cos() * r, c[1] + a1.sin() * r, z0], apex, n, mt, 0.0);
    }
}

/// Orthodox cross (or a plain one) of thin boxes standing at `p`.
fn cross(m: &mut Mb, p: V, size: f64, along: [f64; 2], orthodox: bool) {
    let t = size * 0.06;
    m.box_(p[0] - t, p[1] - t, p[2], p[0] + t, p[1] + t, p[2] + size, &[mat::RIM]);
    let bar = |m: &mut Mb, z: f64, half: f64, tilt: f64| {
        let (ax, ay) = (along[0] * half, along[1] * half);
        let c = [p[0], p[1], p[2] + z];
        m.obox(c[0], c[1], ay.atan2(ax), half * 2.0, t * 2.0, c[2] - t + tilt * 0.0, c[2] + t, mat::RIM);
    };
    bar(m, size * 0.68, size * 0.32, 0.0);
    if orthodox {
        bar(m, size * 0.86, size * 0.16, 0.0);
        bar(m, size * 0.3, size * 0.2, 0.0);
    }
}

fn onion_profile(r: f64, h: f64) -> Vec<(f64, f64)> {
    [(0.82, 0.0), (1.0, 0.12), (1.08, 0.28), (0.98, 0.45), (0.7, 0.62), (0.38, 0.78), (0.14, 0.92), (0.0, 1.0)].iter().map(|(a, b)| (a * r, b * h)).collect()
}

fn dome_profile(r: f64, h: f64) -> Vec<(f64, f64)> {
    (0..=8).map(|k| {
        let a = k as f64 / 8.0 * std::f64::consts::FRAC_PI_2;
        (a.cos() * r, a.sin() * h)
    }).collect()
}

/// Footprint frame: centre, unit axis along the longest edge, extents along and across it.
fn frame(f: &[[f64; 2]]) -> ([f64; 2], [f64; 2], (f64, f64), (f64, f64)) {
    let n = f.len();
    let mut best = (0.0, [1.0, 0.0]);
    for i in 0..n {
        let (a, b) = (f[i], f[(i + 1) % n]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l = dx.hypot(dy);
        if l > best.0 {
            best = (l, [dx / l, dy / l]);
        }
    }
    let d = best.1;
    let c0 = [f.iter().map(|p| p[0]).sum::<f64>() / n as f64, f.iter().map(|p| p[1]).sum::<f64>() / n as f64];
    let (mut s0, mut s1, mut t0, mut t1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in f {
        let (x, y) = (p[0] - c0[0], p[1] - c0[1]);
        let (s, t) = (x * d[0] + y * d[1], -x * d[1] + y * d[0]);
        s0 = s0.min(s);
        s1 = s1.max(s);
        t0 = t0.min(t);
        t1 = t1.max(t);
    }
    // centre of the bounding rectangle
    let (sm, tm) = ((s0 + s1) / 2.0, (t0 + t1) / 2.0);
    let c = [c0[0] + d[0] * sm - d[1] * tm, c0[1] + d[1] * sm + d[0] * tm];
    (c, d, (s1 - s0, t1 - t0), (sm, tm))
}

/// Adds the landmark of building `b` to `m` (vertex colours in `col`).
pub fn add(m: &mut Mb, col: &mut Vec<f32>, b: &Building) {
    if b.landmark == Landmark::None || b.footprint.len() < 3 {
        return;
    }
    let (c, d, (len, wid), _) = frame(&b.footprint);
    if len < 4.0 || wid < 3.0 {
        return;
    }
    let h = b.height as f64;
    let wall = b.facade.colour;
    let vscale = 3.0 / b.facade.floor_height.max(2.0) as f64;
    let fm = b.facade.style;
    let seed = b.seed;
    let at = |s: f64, t: f64| [c[0] + d[0] * s - d[1] * t, c[1] + d[1] * s + d[0] * t];
    // the west end (churches face east); for a north-south axis, the south end
    let west = if d[0].abs() > 0.3 { -d[0].signum() } else { -d[1].signum() };
    let rot = d[1].atan2(d[0]) + std::f64::consts::FRAC_PI_4;
    let roof = b.facade.roof_colour;
    // dome finish: gold, or painted blue / green / silver (the cross is always gold)
    let (dome_m, dome_c) = match (seed >> 11) % 5 {
        0 | 1 => (mat::RIM, GOLD),
        2 => (mat::ROOF, [0.35, 0.55, 1.3]),
        3 => (mat::ROOF, [0.55, 1.15, 0.8]),
        _ => (mat::RIM, [1.1, 1.1, 1.1]),
    };
    match b.landmark {
        Landmark::Orthodox | Landmark::Spire => {
            let orthodox = b.landmark == Landmark::Orthodox;
            // tower over the west end, standing 0.4 m proud of the end wall
            let a = (wid * if orthodox { 0.5 } else { 0.55 }).clamp(4.0, 10.0);
            let s = west * (len / 2.0 - a / 2.0 + 0.4);
            let tc = at(s, 0.0);
            let r = a / std::f64::consts::SQRT_2;
            let body_top = h + (2.0 + h01(seed, 1) * 3.0);
            prism(m, tc, r, 4, rot, 0.0, body_top, fm, vscale, Some(mat::SIDEWALK));
            push_col(m, col, wall);
            // cornice band
            prism(m, tc, r + 0.2, 4, rot, body_top - 0.35, body_top, mat::SIDEWALK, 0.0, None);
            push_col(m, col, [wall[0] * 0.85, wall[1] * 0.85, wall[2] * 0.85]);
            if orthodox {
                // belfry: an open-looking narrower octagon, then a drum with an onion and a cross
                let r2 = a * 0.42;
                let bel = body_top + a * 0.9 + 1.0;
                prism(m, tc, r2, 8, rot, body_top, bel, fm, vscale, Some(mat::SIDEWALK));
                push_col(m, col, wall);
                spire(m, tc, r2 + 0.2, 8, rot, bel, a * 0.35, mat::ROOF);
                push_col(m, col, [roof[0] * 1.3, roof[1] * 1.3, roof[2] * 1.3]);
                let dz = bel + a * 0.3;
                revolve(m, tc[0], tc[1], dz, &[(a * 0.14, 0.0), (a * 0.14, a * 0.35)], 10, mat::SIDEWALK);
                push_col(m, col, wall);
                let oh = a * 0.55;
                revolve(m, tc[0], tc[1], dz + a * 0.35, &onion_profile(a * 0.2, oh), 12, dome_m);
                push_col(m, col, dome_c);
                cross(m, [tc[0], tc[1], dz + a * 0.35 + oh], a * 0.35, [-d[1], d[0]], true);
                push_col(m, col, GOLD);
            } else {
                // tall pointed spire with a small cross or a ball
                let sh = a * (2.2 + h01(seed, 2) * 1.2);
                spire(m, tc, r + 0.25, 4, rot, body_top, sh, mat::ROOF);
                push_col(m, col, [roof[0] * 1.2, roof[1] * 1.2, roof[2] * 1.2]);
                cross(m, [tc[0], tc[1], body_top + sh - 0.2], (a * 0.3).min(2.5), [-d[1], d[0]], false);
                push_col(m, col, GOLD);
            }
            if orthodox {
                // drum and onion over the middle of the rest of the body; four small ones on big churches
                let sc = -west * a / 2.0;
                let dc = at(sc, 0.0);
                let rd = (wid * 0.2).clamp(1.8, 6.0);
                let dz = h - 0.5;
                let dh = rd * 1.3;
                prism(m, dc, rd, 8, rot, dz, dz + dh, fm, vscale, None);
                push_col(m, col, wall);
                revolve(m, dc[0], dc[1], dz + dh, &onion_profile(rd * 1.08, rd * 2.1), 14, dome_m);
                push_col(m, col, dome_c);
                cross(m, [dc[0], dc[1], dz + dh + rd * 2.1], rd * 0.9, [-d[1], d[0]], true);
                push_col(m, col, GOLD);
                if wid > 14.0 && len > 22.0 {
                    let off = rd * 1.6;
                    for (ks, kt) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                        let sc2 = at(sc + ks * off, kt * off);
                        let r3 = rd * 0.45;
                        prism(m, sc2, r3, 8, rot, dz, dz + r3 * 1.6, fm, vscale, None);
                        push_col(m, col, wall);
                        revolve(m, sc2[0], sc2[1], dz + r3 * 1.6, &onion_profile(r3 * 1.1, r3 * 2.1), 10, dome_m);
                        push_col(m, col, dome_c);
                        cross(m, [sc2[0], sc2[1], dz + r3 * 1.6 + r3 * 2.1], r3 * 0.9, [-d[1], d[0]], true);
                        push_col(m, col, GOLD);
                    }
                }
            }
        }
        Landmark::Chapel => {
            let rd = (wid * 0.18).clamp(0.8, 2.5);
            let z = h - 0.3;
            prism(m, c, rd, 8, rot, z, z + rd, fm, vscale, None);
            push_col(m, col, wall);
            revolve(m, c[0], c[1], z + rd, &onion_profile(rd * 1.1, rd * 2.0), 12, dome_m);
            push_col(m, col, dome_c);
            cross(m, [c[0], c[1], z + rd * 3.0], rd * 1.0, [-d[1], d[0]], true);
            push_col(m, col, GOLD);
        }
        Landmark::Mosque => {
            let rd = (wid.min(len) * 0.32).clamp(2.0, 14.0);
            let z = h;
            prism(m, c, rd, 16, 0.0, z, z + rd * 0.35, mat::SIDEWALK, 0.0, None);
            push_col(m, col, wall);
            let (dm, dc) = if (seed >> 7) % 2 == 0 { (mat::ROOF, [0.45, 1.05, 0.9]) } else { (mat::RIM, [0.95, 0.97, 1.0]) };
            revolve(m, c[0], c[1], z + rd * 0.35, &dome_profile(rd * 1.02, rd * 0.95), 20, dm);
            push_col(m, col, dc);
            // minaret at a corner of the bounding rectangle, just outside it
            let mc = at(west * (len / 2.0 + 1.6), wid / 2.0 + 1.6);
            let mh = (h * 2.2).max(h + 14.0);
            let mr = 1.3;
            prism(m, mc, mr, 8, 0.0, 0.0, mh, mat::SIDEWALK, 0.0, None);
            push_col(m, col, wall);
            // balcony
            prism(m, mc, mr + 0.6, 8, 0.0, mh * 0.78, mh * 0.78 + 0.35, mat::SIDEWALK, 0.0, Some(mat::SIDEWALK));
            push_col(m, col, [wall[0] * 0.9, wall[1] * 0.9, wall[2] * 0.9]);
            spire(m, mc, mr + 0.15, 8, 0.0, mh, 4.5, dm);
            push_col(m, col, dc);
        }
        Landmark::None => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Facade;

    fn church(l: Landmark) -> Building {
        Building {
            id: "c".into(),
            status: String::new(),
            footprint: vec![[0.0, 0.0], [30.0, 0.0], [30.0, 14.0], [0.0, 14.0]],
            height: 14.0,
            height_origin: "tag".into(),
            verts: Vec::new(),
            indices: Vec::new(),
            facade: Facade::default(),
            ring: 4,
            kind: "church".into(),
            seed: 42,
            design: None,
            landmark: l,
        }
    }

    #[test]
    fn churches_get_a_tower_at_the_west_end_and_domes_above_the_roof() {
        for l in [Landmark::Orthodox, Landmark::Spire, Landmark::Chapel, Landmark::Mosque] {
            let (mut m, mut col) = (Mb::default(), Vec::new());
            add(&mut m, &mut col, &church(l));
            let n = m.a.len() / FLOATS_PER_VERTEX;
            assert!(n > 100, "{l:?}: {n}");
            assert_eq!(col.len() / 3, n);
            assert!(m.a.iter().all(|v| v.is_finite()));
            let top = m.a.chunks(FLOATS_PER_VERTEX).map(|v| v[2]).fold(f32::MIN, f32::max);
            assert!(top > 18.0, "{l:?} rises above the 14 m body: {top}");
            if matches!(l, Landmark::Orthodox | Landmark::Spire) {
                // tower in the western half (x < 15), tallest point there
                let x_top = m.a.chunks(FLOATS_PER_VERTEX).max_by(|a, b| a[2].total_cmp(&b[2])).unwrap()[0];
                assert!(x_top < 10.0, "{l:?} tower at the west end: {x_top}");
            }
        }
        let (mut m, mut col) = (Mb::default(), Vec::new());
        add(&mut m, &mut col, &church(Landmark::None));
        assert!(m.a.is_empty());
    }
}
