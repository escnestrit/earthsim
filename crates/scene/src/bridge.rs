//! Bridges: roads tagged as bridges are drawn as raised decks with walkways, concrete parapets, a deck
//! edge, piers and lamp posts, and the physics drives on the same deck.
//!
//! Height: the deck runs straight between the ground heights at its two ends (so it spans a valley instead
//! of following it down) plus a rise: 6 m over a road it crosses, 2.5 m over water, 2 m otherwise. The
//! climb happens on the approach roads (the road continuing straight from each end): they become ramps
//! between retaining walls, at most ~9 % steep and 130 m long, so a short approach gives a lower bridge.
//! Where no approach continues (a junction right at the end), the deck itself humps up and down.
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpanKind {
    Deck,
    Ramp,
}

/// A raised piece of road: the bridge deck itself or an approach ramp. `rise` is per point, above the
/// straight line between the ground at `a` and at `b` (the anchors the mesh is draped by).
#[derive(Clone, Debug)]
pub struct Span {
    pub kind: SpanKind,
    pub pts: Vec<[f64; 2]>,
    pub rise: Vec<f64>,
    pub width: f64,
    pub a: [f64; 2],
    pub b: [f64; 2],
}

const RAMP_MAX: f64 = 130.0;
/// ramp length per metre of height (smoothstep: steepest slope 1.5 / 16.7 = 9 %)
const RAMP_PER_M: f64 = 16.7;
/// height where two bridge ways meet
const JOINT_H: f64 = 4.0;

fn smooth(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

fn near(a: [f64; 2], b: [f64; 2]) -> bool {
    (a[0] - b[0]).hypot(a[1] - b[1]) < 0.5
}

/// The road continuing straight on from a bridge end (`end` = 0 first point, 1 last): its points from the
/// bridge end outwards, cut at RAMP_MAX metres, and its width.
fn approach(sc: &Scene, r: &Road, end: usize) -> Option<(Vec<[f64; 2]>, f64)> {
    let n = r.pts.len();
    let (e, inner) = if end == 0 { (r.pts[0], r.pts[1]) } else { (r.pts[n - 1], r.pts[n - 2]) };
    let out = [e[0] - inner[0], e[1] - inner[1]];
    let lo = out[0].hypot(out[1]).max(1e-9);
    let mut best: Option<(f64, Vec<[f64; 2]>, f64)> = None;
    for o in sc.roads.iter().filter(|o| !o.bridge && o.pts.len() >= 2) {
        let pts: Vec<[f64; 2]> = if near(o.pts[0], e) {
            o.pts.clone()
        } else if near(*o.pts.last().unwrap(), e) {
            o.pts.iter().rev().copied().collect()
        } else {
            continue;
        };
        let d = [pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]];
        let c = (d[0] * out[0] + d[1] * out[1]) / (lo * d[0].hypot(d[1]).max(1e-9));
        if c > 0.8 && best.as_ref().map_or(true, |b| c > b.0) {
            best = Some((c, pts, o.width));
        }
    }
    let (_, pts, w) = best?;
    // cut at RAMP_MAX
    let mut cut = vec![pts[0]];
    let mut len = 0.0;
    for wnd in pts.windows(2) {
        let l = (wnd[1][0] - wnd[0][0]).hypot(wnd[1][1] - wnd[0][1]);
        if len + l >= RAMP_MAX {
            let t = (RAMP_MAX - len) / l.max(1e-9);
            cut.push([wnd[0][0] + (wnd[1][0] - wnd[0][0]) * t, wnd[0][1] + (wnd[1][1] - wnd[0][1]) * t]);
            break;
        }
        len += l;
        cut.push(wnd[1]);
    }
    Some((cut, w))
}

/// Bridge `r` and its approach ramps.
pub fn spans_of(sc: &Scene, r: &Road) -> Vec<Span> {
    if !r.bridge || r.pts.len() < 2 {
        return Vec::new();
    }
    let n = r.pts.len();
    let s = chainage(&r.pts);
    let len = s[n - 1];
    let (over_road, over_water) = crossings(sc, r);
    let target: f64 = if over_road { 6.0 } else if over_water { 2.5 } else { 2.0 };
    // what continues from each end: an approach road (a ramp), another bridge, or nothing
    let mut appr: [Option<(Vec<[f64; 2]>, f64)>; 2] = [None, None];
    let mut end_h = [0.0f64; 2];
    for end in 0..2 {
        let e = if end == 0 { r.pts[0] } else { r.pts[n - 1] };
        if let Some((pts, w)) = approach(sc, r, end) {
            let al = *chainage(&pts).last().unwrap();
            end_h[end] = target.min(al / RAMP_PER_M);
            appr[end] = Some((pts, w));
        } else if sc.roads.iter().any(|o| o.bridge && !std::ptr::eq(o, r) && (near(o.pts[0], e) || near(*o.pts.last().unwrap(), e))) {
            end_h[end] = JOINT_H;
        }
        if end_h[end] < 0.3 {
            end_h[end] = 0.0;
        }
    }
    // the deck: a hump where both ends stay low; otherwise straight between the end heights, a low end
    // limiting the other so the deck itself can come down at about 10 %
    let hump_ramp = (len / 2.0).min(target * 1.5 / 0.10).max(1e-3);
    let hump_h = target.min(hump_ramp * 0.10 / 1.5);
    if (end_h[0] == 0.0) != (end_h[1] == 0.0) {
        let k = if end_h[0] == 0.0 { 1 } else { 0 };
        end_h[k] = end_h[k].min(len * 0.10 / 1.5);
    }
    let mut out = Vec::new();
    for end in 0..2 {
        let h = end_h[end];
        if let (Some((pts, w)), true) = (appr[end].take(), h > 0.0) {
            let ramp = h * RAMP_PER_M;
            let rise: Vec<f64> = chainage(&pts).iter().map(|d| h * smooth(1.0 - d / ramp)).collect();
            let e = pts[0];
            let far = *pts.last().unwrap();
            out.push(Span { kind: SpanKind::Ramp, a: e, b: far, pts, rise, width: w });
        }
    }
    let rise: Vec<f64> = s
        .iter()
        .map(|&d| {
            let t = d / len.max(1e-9);
            match (end_h[0] > 0.0, end_h[1] > 0.0) {
                (false, false) => hump_h * smooth(d.min(len - d) / hump_ramp),
                (true, true) => end_h[0] + (end_h[1] - end_h[0]) * t,
                (false, true) => end_h[1] * smooth(t),
                (true, false) => end_h[0] * smooth(1.0 - t),
            }
        })
        .collect();
    out.insert(0, Span { kind: SpanKind::Deck, a: r.pts[0], b: r.pts[n - 1], pts: r.pts.clone(), rise, width: r.width });
    out
}

/// All raised pieces of a scene: bridge decks first, then the ramps.
pub fn spans(sc: &Scene) -> Vec<Span> {
    let mut all: Vec<Span> = sc.roads.iter().flat_map(|r| spans_of(sc, r)).collect();
    all.sort_by_key(|s| s.kind == SpanKind::Ramp);
    all
}

/// Rise of a bridge's deck above the straight line between its ends, per road point.
pub fn rise(sc: &Scene, r: &Road) -> Vec<f64> {
    spans_of(sc, r).into_iter().find(|s| s.kind == SpanKind::Deck).map_or_else(|| vec![0.0; r.pts.len()], |s| s.rise)
}

/// Is (x, y) on a ramp raised more than 0.3 m (within `margin` of its edge)? Lamp posts and parked cars
/// of the road below stay out of these.
pub fn on_ramp(spans: &[Span], x: f64, y: f64, margin: f64) -> bool {
    spans.iter().filter(|s| s.kind == SpanKind::Ramp).any(|s| {
        let hw = s.width / 2.0 + WALK + PARAPET + margin;
        s.pts.windows(2).zip(s.rise.windows(2)).any(|(w, r)| {
            if r[0].max(r[1]) < 0.3 {
                return false;
            }
            let (dx, dy) = (w[1][0] - w[0][0], w[1][1] - w[0][1]);
            let l2 = (dx * dx + dy * dy).max(1e-9);
            let t = (((x - w[0][0]) * dx + (y - w[0][1]) * dy) / l2).clamp(0.0, 1.0);
            (w[0][0] + dx * t - x).hypot(w[0][1] + dy * t - y) < hw
        })
    })
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

/// Physics decks of a scene's bridges and ramps; `ground(x, y)` is the terrain height.
pub fn deck_segs(sc: &Scene, ground: &dyn Fn(f64, f64) -> f64) -> Vec<phys::DeckSeg> {
    let mut out = Vec::new();
    for sp in spans(sc) {
        let (ga, gb) = (ground(sp.a[0], sp.a[1]), ground(sp.b[0], sp.b[1]));
        let z = |i: usize| {
            let p = sp.pts[i];
            let t = chord_t(sp.a, sp.b, p[0], p[1]);
            ga + (gb - ga) * t + sp.rise[i] + 0.02
        };
        for i in 0..sp.pts.len() - 1 {
            // ramps only where they stand clear of the road below
            if sp.kind == SpanKind::Ramp && sp.rise[i].max(sp.rise[i + 1]) < 0.05 {
                continue;
            }
            out.push(phys::DeckSeg { a: sp.pts[i], b: sp.pts[i + 1], za: z(i), zb: z(i + 1), hw: sp.width / 2.0 + WALK });
        }
    }
    out
}

/// Lamp posts of a bridge: (point index, side +1 left / -1 right), every 30 m, alternating.
fn lamp_spots(pts: &[[f64; 2]]) -> Vec<(usize, f64)> {
    let ch = chainage(pts);
    let len = *ch.last().unwrap();
    let (mut next, mut side, mut out) = (10.0, 1.0, Vec::new());
    for i in 0..pts.len() {
        if ch[i] >= next && ch[i] < len - 5.0 {
            next = ch[i] + 30.0;
            out.push((i, side));
            side = -side;
        }
    }
    out
}

/// Lamp heads of the scene's bridges for lighting: (x, y, height above the deck line's base) and the
/// bridge's end points (the base is the ground height interpolated between them, as for the mesh).
pub fn lamps(sc: &Scene) -> Vec<([f64; 3], [f64; 2], [f64; 2])> {
    let mut out = Vec::new();
    for sp in spans(sc) {
        let nm = normals(&sp.pts);
        let e = sp.width / 2.0 + WALK;
        for (i, side) in lamp_spots(&sp.pts) {
            let d = side * (e - 1.2) * nm[i].1;
            out.push(([sp.pts[i][0] + nm[i].0[0] * d, sp.pts[i][1] + nm[i].0[1] * d, sp.rise[i] + 7.7], sp.a, sp.b));
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
    for sp in spans(sc) {
        let ramp = sp.kind == SpanKind::Ramp;
        let pts = &sp.pts;
        let n = pts.len();
        let rs = &sp.rise;
        let nm = normals(pts);
        let hw = sp.width / 2.0;
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
            // deck edge, or on a ramp a retaining wall down into the ground
            if ramp {
                let foot: Vec<V> = (0..n).map(|i| { let q = p(i, s * (e + PARAPET), 0.0); [q[0], q[1], -2.0] }).collect();
                strip(&mut m, &foot, &line(s * (e + PARAPET), 1.1), side(s), mat::MARKING);
            } else {
                strip(&mut m, &line(s * (e + PARAPET), -DECK_T), &line(s * (e + PARAPET), 1.1), side(s), mat::MARKING);
            }
            fill(&m, &mut col, [0.7, 0.7, 0.68]);
            // metal handrail on top
            strip(&mut m, &line(s * (e + 0.05), 1.1), &line(s * (e + 0.05), 1.3), side(-s), mat::RIM);
            strip(&mut m, &line(s * (e + 0.05), 1.3), &line(s * (e + 0.2), 1.3), up, mat::RIM);
            fill(&m, &mut col, [0.55, 0.6, 0.62]);
        }
        // underside
        if !ramp {
            strip(&mut m, &line(-(e + PARAPET), -DECK_T), &line(e + PARAPET, -DECK_T), down, mat::MARKING);
            fill(&m, &mut col, [0.45, 0.45, 0.44]);
        }
        // piers where the deck stands clear of the ground line, about every 24 m; lamp posts every 30 m
        let ch = chainage(pts);
        let len = *ch.last().unwrap();
        let mut next_pier = 12.0;
        let spots = lamp_spots(pts);
        for i in 0..n {
            let (dx, dy) = if i + 1 < n { (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) } else { (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) };
            let yaw = dy.atan2(dx);
            if ch[i] >= next_pier && ch[i] < len - 8.0 {
                next_pier = ch[i] + 24.0;
                if !ramp && rs[i] - DECK_T > 0.6 {
                    let c = pts[i];
                    // a wall pier across the deck, under it
                    m.obox(c[0], c[1], yaw + std::f64::consts::FRAC_PI_2, 2.0 * hw + 1.0, 1.1, -10.0, rs[i] - DECK_T, mat::MARKING);
                    // cap beam
                    m.obox(c[0], c[1], yaw + std::f64::consts::FRAC_PI_2, 2.0 * e, 1.6, rs[i] - DECK_T - 0.6, rs[i] - DECK_T, mat::MARKING);
                    fill(&m, &mut col, [0.66, 0.66, 0.64]);
                }
            }
            if let Some(&(_, lamp_side)) = spots.iter().find(|s| s.0 == i) {
                let b = p(i, lamp_side * (e + 0.12), 1.1);
                let a = p(i, lamp_side * (e - 1.2), 0.0);
                m.box_(b[0] - 0.08, b[1] - 0.08, b[2], b[0] + 0.08, b[1] + 0.08, rs[i] + 8.0, &[mat::RIM]);
                fill(&m, &mut col, [0.5, 0.52, 0.55]);
                m.box_(a[0] - 0.25, a[1] - 0.25, rs[i] + 7.75, a[0] + 0.25, a[1] + 0.25, rs[i] + 7.9, &[mat::HEADLIGHT]);
                m.box_(a[0].min(b[0]) - 0.04, a[1].min(b[1]) - 0.04, rs[i] + 7.92, a[0].max(b[0]) + 0.04, a[1].max(b[1]) + 0.04, rs[i] + 8.0, &[mat::RIM]);
                fill(&m, &mut col, [0.5, 0.52, 0.55]);
            }
        }
        let (a, b) = (sp.a, sp.b);
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
        Road { cls: "primary".into(), width: 7.0, pts, junction: vec![false; n], bridge, oneway: 0 }
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
        // a road below: 6 m
        sc.roads.push(road(vec![[100.0, -50.0], [100.0, 50.0]], false));
        assert!((rise(&sc, &sc.roads[0])[10] - 6.0).abs() < 1e-9);
        // approach roads continuing from both ends: ramps carry it up, the deck stays at 6 m
        sc.roads.push(road((0..=15).map(|i| [-(i as f64) * 10.0, 0.0]).collect(), false));
        sc.roads.push(road((0..=15).map(|i| [200.0 + i as f64 * 10.0, 0.0]).collect(), false));
        let sp = spans_of(&sc, &sc.roads[0]);
        assert_eq!(sp.len(), 3);
        assert!(sp[0].rise.iter().all(|r| (r - 6.0).abs() < 1e-9), "{:?}", sp[0].rise);
        for ramp in &sp[1..] {
            assert_eq!(ramp.kind, SpanKind::Ramp);
            assert!((ramp.rise[0] - 6.0).abs() < 1e-9 && *ramp.rise.last().unwrap() < 0.05, "{:?}", ramp.rise);
            // no steeper than ~9 %
            for (w, r) in ramp.pts.windows(2).zip(ramp.rise.windows(2)) {
                let l = (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]);
                assert!((r[0] - r[1]).abs() / l < 0.095);
            }
        }
        assert!(on_ramp(&spans(&sc), -20.0, 3.0, 0.0) && !on_ramp(&spans(&sc), -149.0, 0.0, 0.0));
        sc.roads.truncate(2);
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
        assert!((segs[9].zb - (1.0 + 6.0 + 0.02)).abs() < 1e-9, "{}", segs[9].zb);
    }
}
