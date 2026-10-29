//! Bridges: roads tagged as bridges are drawn as raised decks with walkways, concrete parapets, a deck
//! edge, piers and lamp posts, and the physics drives on the same deck.
//!
//! Height: the deck runs straight between the ground heights at its two ends (so it spans a valley instead
//! of following it down) plus a rise in the middle: up to 5 m over a road it crosses, 2.5 m over water,
//! 2 m otherwise, limited so the ramps stay under about 10 %. A short bridge over a road on flat ground is
//! therefore lower than a real overpass, whose ramps lie on the neighbouring (untagged) road ways.
use crate::mesh::{mat, MeshData, Mb, FLOATS_PER_VERTEX};
use crate::model::{AreaKind, Road, Scene};

type V = [f64; 3];

const WALK: f64 = 1.5;
const PARAPET: f64 = 0.25;
const DECK_T: f64 = 0.9;

/// Where segment a-b meets segment c-d (touching counts): the point.
fn seg_cross(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> Option<[f64; 2]> {
    let (r, q) = ([b[0] - a[0], b[1] - a[1]], [d[0] - c[0], d[1] - c[1]]);
    let den = r[0] * q[1] - r[1] * q[0];
    if den.abs() < 1e-12 {
        return None;
    }
    let (ex, ey) = (c[0] - a[0], c[1] - a[1]);
    let t = (ex * q[1] - ey * q[0]) / den;
    let u = (ex * r[1] - ey * r[0]) / den;
    ((-1e-9..=1.0 + 1e-9).contains(&t) && (-1e-9..=1.0 + 1e-9).contains(&u)).then(|| [a[0] + r[0] * t, a[1] + r[1] * t])
}

fn in_tri(p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
    let o = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    let (d1, d2, d3) = (o(a, b, p), o(b, c, p), o(c, a, p));
    !((d1 < 0.0 || d2 < 0.0 || d3 < 0.0) && (d1 > 0.0 || d2 > 0.0 || d3 > 0.0))
}

/// What the bridge passes over: (a road, water).
fn crossings(sc: &Scene, r: &Road) -> (bool, bool) {
    // a road passing under it, not one joining it at an end
    let (e0, e1) = (r.pts[0], *r.pts.last().unwrap());
    let clear = |p: [f64; 2]| (p[0] - e0[0]).hypot(p[1] - e0[1]) > 3.0 && (p[0] - e1[0]).hypot(p[1] - e1[1]) > 3.0;
    let road = sc.roads.iter().filter(|o| !o.bridge).any(|o| {
        r.pts.windows(2).any(|s| o.pts.windows(2).any(|t| seg_cross(s[0], s[1], t[0], t[1]).is_some_and(clear)))
    });
    let water = sc.areas.iter().filter(|a| a.kind == AreaKind::Water).any(|a| {
        r.pts.windows(2).any(|s| {
            let m = [(s[0][0] + s[1][0]) / 2.0, (s[0][1] + s[1][1]) / 2.0];
            a.tris.iter().any(|t| in_tri(m, a.ring[t[0] as usize], a.ring[t[1] as usize], a.ring[t[2] as usize]))
        })
    });
    (road, water)
}

/// Distance along the road at each point.
fn chainage(pts: &[[f64; 2]]) -> Vec<f64> {
    let mut s = vec![0.0];
    for w in pts.windows(2) {
        s.push(s.last().unwrap() + (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]));
    }
    s
}

/// Rise of the deck above the straight line between its ends, per road point (0 at both ends).
pub fn rise(sc: &Scene, r: &Road) -> Vec<f64> {
    if !r.bridge || r.pts.len() < 2 {
        return vec![0.0; r.pts.len()];
    }
    let s = chainage(&r.pts);
    let len = *s.last().unwrap();
    let (over_road, over_water) = crossings(sc, r);
    let target: f64 = if over_road { 5.0 } else if over_water { 2.5 } else { 2.0 };
    // smoothstep ramps: steepest slope is 1.5 * rise / ramp; keep it near 10 %
    let ramp = (len / 2.0).min(target * 1.5 / 0.10).max(1e-3);
    let h = target.min(ramp * 0.10 / 1.5);
    s.iter()
        .map(|&d| {
            let x = (d.min(len - d) / ramp).clamp(0.0, 1.0);
            h * x * x * (3.0 - 2.0 * x)
        })
        .collect()
}

/// End points of a bridge mesh from its name (`bridge|ax|ay|bx|by`): the mesh is draped by the ground
/// heights at these two points (straight between them), not by the ground under each vertex.
pub fn anchors(name: &str) -> Option<([f64; 2], [f64; 2])> {
    let mut it = name.strip_prefix("bridge|")?.split('|').map(|v| v.parse::<f64>().ok());
    Some(([it.next()??, it.next()??], [it.next()??, it.next()??]))
}

/// Height along the chord a -> b at (x, y), for draping a bridge mesh.
pub fn chord_t(a: [f64; 2], b: [f64; 2], x: f64, y: f64) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    if l2 < 1e-9 {
        return 0.0;
    }
    (((x - a[0]) * dx + (y - a[1]) * dy) / l2).clamp(0.0, 1.0)
}

/// Physics decks of a scene's bridges; `ground(x, y)` is the terrain height.
pub fn deck_segs(sc: &Scene, ground: &dyn Fn(f64, f64) -> f64) -> Vec<phys::DeckSeg> {
    let mut out = Vec::new();
    for r in sc.roads.iter().filter(|r| r.bridge && r.pts.len() >= 2) {
        let rs = rise(sc, r);
        let (a, b) = (r.pts[0], *r.pts.last().unwrap());
        let (ga, gb) = (ground(a[0], a[1]), ground(b[0], b[1]));
        let z = |i: usize| {
            let p = r.pts[i];
            let t = chord_t(a, b, p[0], p[1]);
            ga + (gb - ga) * t + rs[i] + 0.02
        };
        for i in 0..r.pts.len() - 1 {
            out.push(phys::DeckSeg { a: r.pts[i], b: r.pts[i + 1], za: z(i), zb: z(i + 1), hw: r.width / 2.0 + WALK });
        }
    }
    out
}

/// Left-hand normals (mitred) of a polyline: (unit normal, mitre scale) per point.
fn normals(pts: &[[f64; 2]]) -> Vec<([f64; 2], f64)> {
    let n = pts.len();
    let seg: Vec<[f64; 2]> = pts.windows(2).map(|w| {
        let (dx, dy) = (w[1][0] - w[0][0], w[1][1] - w[0][1]);
        let l = dx.hypot(dy).max(1e-9);
        [-dy / l, dx / l]
    }).collect();
    (0..n)
        .map(|i| {
            let a = seg[i.saturating_sub(1).min(seg.len() - 1)];
            let b = seg[i.min(seg.len() - 1)];
            let (mut mx, mut my) = (a[0] + b[0], a[1] + b[1]);
            let ml = mx.hypot(my);
            if ml < 1e-9 {
                mx = a[0];
                my = a[1];
            } else {
                mx /= ml;
                my /= ml;
            }
            ([mx, my], 1.0 / (mx * a[0] + my * a[1]).max(0.45))
        })
        .collect()
}

/// Quad strip between two polylines; `away` is a direction the faces should point along.
fn strip<F: Fn(usize) -> V>(m: &mut Mb, a: &[V], b: &[V], away: F, mt: f32) {
    for i in 0..a.len().saturating_sub(1) {
        let q = [a[i], a[i + 1], b[i + 1], b[i]];
        let mut c = [0.0; 3];
        for p in &q {
            for k in 0..3 {
                c[k] += p[k] / 4.0;
            }
        }
        let w = away(i);
        let inner = [c[0] - w[0], c[1] - w[1], c[2] - w[2]];
        let area = ((q[1][0] - q[0][0]).hypot(q[1][1] - q[0][1]) + (q[1][2] - q[0][2]).abs()) * ((q[3][0] - q[0][0]).hypot(q[3][1] - q[0][1]) + (q[3][2] - q[0][2]).abs());
        if area > 1e-8 {
            m.face(&q, inner, mt);
        }
    }
}

/// One mesh per bridge (named with its anchors, see `anchors`), vertex-coloured.
pub fn meshes(sc: &Scene) -> Vec<MeshData> {
    let mut out = Vec::new();
    for r in sc.roads.iter().filter(|r| r.bridge && r.pts.len() >= 2) {
        let pts = &r.pts;
        let n = pts.len();
        let rs = rise(sc, r);
        let nm = normals(pts);
        let hw = r.width / 2.0;
        let mut m = Mb::default();
        let mut col: Vec<f32> = Vec::new();
        let fill = |m: &Mb, col: &mut Vec<f32>, c: [f32; 3]| {
            while col.len() / 3 < m.a.len() / FLOATS_PER_VERTEX {
                col.extend_from_slice(&c);
            }
        };
        // point at lateral offset d (left +) and height dz above the deck line
        let p = |i: usize, d: f64, dz: f64| -> V { [pts[i][0] + nm[i].0[0] * d * nm[i].1, pts[i][1] + nm[i].0[1] * d * nm[i].1, rs[i] + dz] };
        let line = |d: f64, dz: f64| -> Vec<V> { (0..n).map(|i| p(i, d, dz)).collect() };
        let up = |_: usize| [0.0, 0.0, 1.0];
        let down = |_: usize| [0.0, 0.0, -1.0];
        let nmr = &nm;
        let side = |s: f64| move |i: usize| [nmr[i].0[0] * s, nmr[i].0[1] * s, 0.0];
        // carriageway
        strip(&mut m, &line(hw, 0.02), &line(-hw, 0.02), up, mat::ASPHALT);
        fill(&m, &mut col, [1.0, 1.0, 1.0]);
        let e = hw + WALK;
        for s in [1.0, -1.0] {
            // walkway on a kerb
            strip(&mut m, &line(s * e, 0.17), &line(s * hw, 0.17), up, mat::SIDEWALK);
            fill(&m, &mut col, [1.0, 1.0, 1.0]);
            strip(&mut m, &line(s * hw, 0.02), &line(s * hw, 0.17), side(-s), mat::MARKING);
            fill(&m, &mut col, [0.62, 0.62, 0.6]);
            // concrete parapet; its outer face runs down as the deck edge
            strip(&mut m, &line(s * e, 0.17), &line(s * e, 1.1), side(-s), mat::MARKING);
            strip(&mut m, &line(s * e, 1.1), &line(s * (e + PARAPET), 1.1), up, mat::MARKING);
            fill(&m, &mut col, [0.78, 0.78, 0.76]);
            strip(&mut m, &line(s * (e + PARAPET), -DECK_T), &line(s * (e + PARAPET), 1.1), side(s), mat::MARKING);
            fill(&m, &mut col, [0.7, 0.7, 0.68]);
            // metal handrail on top
            strip(&mut m, &line(s * (e + 0.05), 1.1), &line(s * (e + 0.05), 1.3), side(-s), mat::RIM);
            strip(&mut m, &line(s * (e + 0.05), 1.3), &line(s * (e + 0.2), 1.3), up, mat::RIM);
            fill(&m, &mut col, [0.55, 0.6, 0.62]);
        }
        // underside
        strip(&mut m, &line(-(e + PARAPET), -DECK_T), &line(e + PARAPET, -DECK_T), down, mat::MARKING);
        fill(&m, &mut col, [0.45, 0.45, 0.44]);
        // piers where the deck stands clear of the ground line, about every 24 m; lamp posts every 30 m
        let ch = chainage(pts);
        let len = *ch.last().unwrap();
        let mut next_pier = 12.0;
        let mut next_lamp = 10.0;
        let mut lamp_side = 1.0;
        for i in 0..n {
            let (dx, dy) = if i + 1 < n { (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) } else { (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) };
            let yaw = dy.atan2(dx);
            if ch[i] >= next_pier && ch[i] < len - 8.0 {
                next_pier = ch[i] + 24.0;
                if rs[i] - DECK_T > 0.6 {
                    let c = pts[i];
                    // a wall pier across the deck, under it
                    m.obox(c[0], c[1], yaw + std::f64::consts::FRAC_PI_2, 2.0 * hw + 1.0, 1.1, -10.0, rs[i] - DECK_T, mat::MARKING);
                    // cap beam
                    m.obox(c[0], c[1], yaw + std::f64::consts::FRAC_PI_2, 2.0 * e, 1.6, rs[i] - DECK_T - 0.6, rs[i] - DECK_T, mat::MARKING);
                    fill(&m, &mut col, [0.66, 0.66, 0.64]);
                }
            }
            if ch[i] >= next_lamp && ch[i] < len - 5.0 {
                next_lamp = ch[i] + 30.0;
                let b = p(i, lamp_side * (e + 0.12), 1.1);
                let a = p(i, lamp_side * (e - 1.2), 0.0);
                m.box_(b[0] - 0.08, b[1] - 0.08, b[2], b[0] + 0.08, b[1] + 0.08, rs[i] + 8.0, &[mat::RIM]);
                fill(&m, &mut col, [0.5, 0.52, 0.55]);
                m.box_(a[0] - 0.25, a[1] - 0.25, rs[i] + 7.75, a[0] + 0.25, a[1] + 0.25, rs[i] + 7.9, &[mat::HEADLIGHT]);
                m.box_(a[0].min(b[0]) - 0.04, a[1].min(b[1]) - 0.04, rs[i] + 7.92, a[0].max(b[0]) + 0.04, a[1].max(b[1]) + 0.04, rs[i] + 8.0, &[mat::RIM]);
                fill(&m, &mut col, [0.5, 0.52, 0.55]);
                lamp_side = -lamp_side;
            }
        }
        let (a, b) = (pts[0], pts[n - 1]);
        out.push(MeshData { name: format!("bridge|{}|{}|{}|{}", a[0], a[1], b[0], b[1]), verts: m.a, polygon_offset: -3.0, casts_shadow: true, colors: col });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Area;

    fn road(pts: Vec<[f64; 2]>, bridge: bool) -> Road {
        let n = pts.len();
        Road { cls: "primary".into(), width: 7.0, pts, junction: vec![false; n], bridge }
    }

    #[test]
    fn decks_rise_over_roads_and_water_and_meet_the_ground_at_the_ends() {
        let mut sc = Scene::default();
        let pts: Vec<[f64; 2]> = (0..=20).map(|i| [i as f64 * 10.0, 0.0]).collect();
        sc.roads.push(road(pts.clone(), true));
        // nothing below: 2 m
        let r = rise(&sc, &sc.roads[0]);
        assert_eq!(r[0], 0.0);
        assert_eq!(*r.last().unwrap(), 0.0);
        assert!((r[10] - 2.0).abs() < 1e-9, "{}", r[10]);
        // a road below: 5 m
        sc.roads.push(road(vec![[100.0, -50.0], [100.0, 50.0]], false));
        assert!((rise(&sc, &sc.roads[0])[10] - 5.0).abs() < 1e-9);
        // short bridge: limited by the ramps
        let mut sc2 = Scene::default();
        sc2.roads.push(road(vec![[0.0, 0.0], [10.0, 0.0], [20.0, 0.0], [30.0, 0.0], [40.0, 0.0]], true));
        sc2.areas.push(Area { kind: AreaKind::Water, ring: vec![[5.0, -20.0], [35.0, -20.0], [35.0, 20.0], [5.0, 20.0]], tris: vec![[0, 1, 2], [0, 2, 3]] });
        let r2 = rise(&sc2, &sc2.roads[0]);
        assert!(r2[2] > 1.0 && r2[2] < 1.4, "{}", r2[2]);
        // meshes: one per bridge, finite, coloured, named with the end points
        let ms = meshes(&sc);
        assert_eq!(ms.len(), 1);
        assert_eq!(anchors(&ms[0].name), Some(([0.0, 0.0], [200.0, 0.0])));
        assert_eq!(ms[0].colors.len() / 3, ms[0].vertex_count());
        assert!(ms[0].verts.iter().all(|v| v.is_finite()));
        // physics: 20 segments, deck on a slope between the end heights
        let segs = deck_segs(&sc, &|x, _| x * 0.01);
        assert_eq!(segs.len(), 20);
        assert!((segs[0].za - 0.02).abs() < 1e-9 && (segs[19].zb - 2.02).abs() < 1e-9);
        assert!((segs[9].zb - (1.0 + 5.0 + 0.02)).abs() < 1e-9, "{}", segs[9].zb);
    }
}
