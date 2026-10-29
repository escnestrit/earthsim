//! Ground cover from map data: water, grass, parks, forest, sand. Polygons (with holes) are clipped to their
//! tile, simplified, triangulated in local metres, and forests/parks get trees scattered on a grid that is
//! fixed in latitude/longitude (so a tree stays put across re-bases and tile reloads).
use crate::geo::LocalFrame;
use crate::stage_a::{self, Tags};
use crate::tiles::Bounds;
use scene::model::{Area, AreaKind};

#[derive(Clone, Debug)]
pub struct RawArea {
    pub kind: AreaKind,
    /// (lat, lon) rings, first outer then holes, clipped to the tile, not closed (first != last)
    pub rings: Vec<Vec<(f64, f64)>>,
}

/// OSM-style tags -> ground cover kind.
pub fn area_kind(tags: &Tags) -> Option<AreaKind> {
    let t = |k: &str| stage_a::tag(tags, k);
    if t("building").is_some() || t("area:highway").is_some() {
        return None;
    }
    // surface car parks only: multi-storey and underground ones are buildings or invisible
    if t("amenity") == Some("parking") && !matches!(t("parking"), Some("multi-storey" | "underground" | "rooftop")) {
        return Some(AreaKind::Parking);
    }
    if t("natural") == Some("water") || t("waterway") == Some("riverbank") || matches!(t("landuse"), Some("reservoir" | "basin")) {
        return Some(AreaKind::Water);
    }
    if t("natural") == Some("wood") || t("landuse") == Some("forest") {
        return Some(AreaKind::Forest);
    }
    if matches!(t("natural"), Some("sand" | "beach")) {
        return Some(AreaKind::Sand);
    }
    if matches!(t("leisure"), Some("park" | "garden" | "playground" | "pitch" | "golf_course" | "dog_park")) {
        return Some(AreaKind::Park);
    }
    if matches!(t("natural"), Some("scrub" | "heath" | "grassland" | "wetland"))
        || matches!(t("landuse"), Some("grass" | "meadow" | "village_green" | "recreation_ground" | "cemetery" | "allotments" | "orchard"))
    {
        return Some(AreaKind::Grass);
    }
    None
}

/// Sutherland-Hodgman clip of a (lat, lon) ring to the tile box. Returns an open ring.
/// A line (lat, lon) widened to `width` metres, as short overlapping strips (each a simple polygon of up to
/// 8 segments, so sharp bends never make a self-crossing ring).
pub fn line_strips(pts: &[(f64, f64)], width: f64) -> Vec<Vec<(f64, f64)>> {
    if pts.len() < 2 {
        return Vec::new();
    }
    let lat0 = pts[0].0;
    let (mla, mlo) = (111_320.0, 111_320.0 * lat0.to_radians().cos().max(0.05));
    let hw = width / 2.0;
    let mut out = Vec::new();
    let mut i = 0;
    while i + 1 < pts.len() {
        let j = (i + 8).min(pts.len() - 1);
        let run = &pts[i..=j];
        let n = run.len();
        let mut left = Vec::with_capacity(n);
        let mut right = Vec::with_capacity(n);
        for k in 0..n {
            let a = run[k.saturating_sub(1)];
            let c = run[(k + 1).min(n - 1)];
            let (dy, dx) = ((c.0 - a.0) * mla, (c.1 - a.1) * mlo);
            let l = dx.hypot(dy).max(1e-9);
            let (nx, ny) = (-dy / l * hw, dx / l * hw);
            left.push((run[k].0 + ny / mla, run[k].1 + nx / mlo));
            right.push((run[k].0 - ny / mla, run[k].1 - nx / mlo));
        }
        right.reverse();
        left.extend(right);
        out.push(left);
        i = j;
    }
    out
}

pub fn clip_to_bounds(ring: &[(f64, f64)], b: &Bounds) -> Vec<(f64, f64)> {
    let mut pts: Vec<(f64, f64)> = ring.to_vec();
    if pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    // (lat or lon axis, keep >= or <=, value)
    let planes = [(0, true, b.south), (0, false, b.north), (1, true, b.west), (1, false, b.east)];
    for (axis, ge, v) in planes {
        if pts.is_empty() {
            break;
        }
        let c = |p: (f64, f64)| if axis == 0 { p.0 } else { p.1 };
        let inside = |p: (f64, f64)| if ge { c(p) >= v } else { c(p) <= v };
        let mut out = Vec::with_capacity(pts.len() + 4);
        for i in 0..pts.len() {
            let (a, d) = (pts[i], pts[(i + 1) % pts.len()]);
            let (ia, id) = (inside(a), inside(d));
            if ia {
                out.push(a);
            }
            if ia != id {
                let t = (v - c(a)) / (c(d) - c(a));
                out.push((a.0 + (d.0 - a.0) * t, a.1 + (d.1 - a.1) * t));
            }
        }
        pts = out;
    }
    pts
}

fn signed_area(r: &[(f64, f64)]) -> f64 {
    stage_a::signed_area(r)
}

/// Douglas-Peucker on a closed ring (metres), tolerance raised until at most `max_pts` remain.
pub fn simplify_ring(r: &[(f64, f64)], mut tol: f64, max_pts: usize) -> Vec<(f64, f64)> {
    fn dp(p: &[(f64, f64)], tol: f64, keep: &mut [bool], i0: usize, i1: usize) {
        if i1 <= i0 + 1 {
            return;
        }
        let (a, b) = (p[i0], p[i1]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l = (dx * dx + dy * dy).sqrt().max(1e-12);
        let (mut best, mut bi) = (0.0, i0);
        for (i, q) in p.iter().enumerate().take(i1).skip(i0 + 1) {
            let d = ((q.0 - a.0) * dy - (q.1 - a.1) * dx).abs() / l;
            if d > best {
                best = d;
                bi = i;
            }
        }
        if best > tol {
            keep[bi] = true;
            dp(p, tol, keep, i0, bi);
            dp(p, tol, keep, bi, i1);
        }
    }
    if r.len() <= 4 {
        return r.to_vec();
    }
    // split at the vertex farthest from the first one so both halves are open chains
    let far = (1..r.len()).max_by(|&i, &j| {
        let d = |k: usize| (r[k].0 - r[0].0).powi(2) + (r[k].1 - r[0].1).powi(2);
        d(i).total_cmp(&d(j))
    }).unwrap();
    loop {
        let mut closed = r.to_vec();
        closed.push(r[0]);
        let mut keep = vec![false; closed.len()];
        keep[0] = true;
        keep[far] = true;
        keep[closed.len() - 1] = true;
        dp(&closed, tol, &mut keep, 0, far);
        dp(&closed, tol, &mut keep, far, closed.len() - 1);
        let out: Vec<(f64, f64)> = closed[..closed.len() - 1].iter().zip(&keep).filter(|(_, k)| **k).map(|(p, _)| *p).collect();
        if out.len() <= max_pts || tol > 1e4 {
            return out;
        }
        tol *= 1.6;
    }
}

fn seg_cross(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> bool {
    let o = |p: (f64, f64), q: (f64, f64), r: (f64, f64)| (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0);
    let (d1, d2, d3, d4) = (o(c, d, a), o(c, d, b), o(a, b, c), o(a, b, d));
    ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0)) && d1 != 0.0 && d2 != 0.0 && d3 != 0.0 && d4 != 0.0
}

/// Join the holes into the outer ring with bridge edges (outer CCW, holes CW), giving one simple ring.
pub fn bridge_holes(outer: &[(f64, f64)], holes: &[Vec<(f64, f64)>]) -> Vec<(f64, f64)> {
    let mut ring = outer.to_vec();
    let mut hs: Vec<&Vec<(f64, f64)>> = holes.iter().filter(|h| h.len() >= 3).collect();
    // rightmost holes first (standard earcut order)
    hs.sort_by(|a, b| b.iter().map(|p| p.0).fold(f64::MIN, f64::max).total_cmp(&a.iter().map(|p| p.0).fold(f64::MIN, f64::max)));
    for h in hs {
        let (mi, m) = h.iter().copied().enumerate().max_by(|a, b| a.1 .0.total_cmp(&b.1 .0)).unwrap();
        // nearest ring vertex reachable from m without crossing any edge of the ring or the other holes
        let mut cands: Vec<(f64, usize)> = ring.iter().enumerate().map(|(i, p)| ((p.0 - m.0).powi(2) + (p.1 - m.1).powi(2), i)).collect();
        cands.sort_by(|a, b| a.0.total_cmp(&b.0));
        let n = ring.len();
        let ok = |pi: usize| {
            let p = ring[pi];
            (0..n).all(|k| {
                let (a, b) = (ring[k], ring[(k + 1) % n]);
                k == pi || (k + 1) % n == pi || !seg_cross(m, p, a, b)
            }) && (0..h.len()).all(|k| {
                let (a, b) = (h[k], h[(k + 1) % h.len()]);
                k == mi || (k + 1) % h.len() == mi || !seg_cross(m, p, a, b)
            })
        };
        let Some(&(_, pi)) = cands.iter().take(64).find(|(_, i)| ok(*i)) else { continue };
        // ring[..=pi] + hole from mi around to mi + back to ring[pi] + rest
        let mut out = ring[..=pi].to_vec();
        for k in 0..=h.len() {
            out.push(h[(mi + k) % h.len()]);
        }
        out.push(ring[pi]);
        out.extend_from_slice(&ring[pi + 1..]);
        ring = out;
    }
    ring
}

/// Ear clipping for a CCW ring that may contain bridge edges (repeated points): vertices coinciding with an
/// ear's corners do not block it.
pub fn triangulate_bridged(ring: &[(f64, f64)]) -> Option<Vec<[usize; 3]>> {
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0);
    let same = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).abs() + (a.1 - b.1).abs() < 1e-9;
    let mut idx: Vec<usize> = (0..ring.len()).collect();
    idx.dedup_by(|a, b| same(ring[*a], ring[*b]));
    while idx.len() > 1 && same(ring[idx[0]], ring[*idx.last().unwrap()]) {
        idx.pop();
    }
    let mut out = Vec::new();
    let mut guard = 0usize;
    while idx.len() > 3 {
        guard += 1;
        if guard > 4 * ring.len() + 16 {
            return None;
        }
        let n = idx.len();
        let mut clipped = false;
        for k in 0..n {
            let (i0, i1, i2) = (idx[(k + n - 1) % n], idx[k], idx[(k + 1) % n]);
            let (a, b, c) = (ring[i0], ring[i1], ring[i2]);
            let cr = cross(a, b, c);
            if cr.abs() < 1e-12 {
                // collinear or a zero-width spike: drop the middle vertex
                idx.remove(k);
                clipped = true;
                break;
            }
            if cr < 0.0 {
                continue;
            }
            let blocked = idx.iter().any(|&j| {
                let p = ring[j];
                !(same(p, a) || same(p, b) || same(p, c)) && cross(a, b, p) >= 0.0 && cross(b, c, p) >= 0.0 && cross(c, a, p) >= 0.0
            });
            if blocked {
                continue;
            }
            out.push([i0, i1, i2]);
            idx.remove(k);
            clipped = true;
            break;
        }
        if !clipped {
            return None;
        }
    }
    if idx.len() == 3 && cross(ring[idx[0]], ring[idx[1]], ring[idx[2]]).abs() > 1e-12 {
        out.push([idx[0], idx[1], idx[2]]);
    }
    Some(out)
}

/// Point in one (lat, lon) ring.
pub fn point_in(p: (f64, f64), r: &[(f64, f64)]) -> bool {
    point_in_rings(p, std::slice::from_ref(&r.to_vec()))
}

fn point_in_rings(p: (f64, f64), rings: &[Vec<(f64, f64)>]) -> bool {
    let mut inside = false;
    for r in rings {
        let n = r.len();
        for i in 0..n {
            let (a, b) = (r[i], r[(i + 1) % n]);
            if (a.1 > p.1) != (b.1 > p.1) && p.0 < a.0 + (p.1 - a.1) / (b.1 - a.1) * (b.0 - a.0) {
                inside = !inside;
            }
        }
    }
    inside
}

fn hash2(i: i64, j: i64) -> u64 {
    let mut h = (i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ (j as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F);
    h ^= h >> 31;
    h = h.wrapping_mul(0xFF51_AFD7_ED55_8CCD);
    h ^ (h >> 29)
}

/// Tree spacing (m) for a kind; None = no trees.
fn tree_spacing(k: AreaKind) -> Option<f64> {
    match k {
        AreaKind::Forest => Some(9.0),
        AreaKind::Park => Some(24.0),
        _ => None,
    }
}

/// Most trees scattered per tile (keeps forests affordable to draw); set from the graphics quality.
pub static MAX_TREES_PER_TILE: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(800);

/// Areas in local metres plus scattered trees. `avoid(x, y)` = true where no tree may stand (roads).
pub fn build_areas(raw: &[RawArea], frame: &LocalFrame, avoid: &dyn Fn(f64, f64) -> bool) -> (Vec<Area>, Vec<[f64; 2]>) {
    let mut areas = Vec::new();
    let mut trees = Vec::new();
    for ra in raw {
        let loc: Vec<Vec<(f64, f64)>> = ra
            .rings
            .iter()
            .map(|r| {
                r.iter().map(|&(la, lo)| {
                    let p = frame.to_local(la, lo, 0.0);
                    (p[0], p[1])
                }).collect()
            })
            .collect();
        let Some(outer) = loc.first() else { continue };
        let mut outer = simplify_ring(outer, 0.8, 300);
        if outer.len() < 3 || signed_area(&outer).abs() < 4.0 {
            continue;
        }
        if signed_area(&outer) < 0.0 {
            outer.reverse();
        }
        let mut holes: Vec<Vec<(f64, f64)>> = Vec::new();
        for h in &loc[1..] {
            let mut h = simplify_ring(h, 0.8, 120);
            if h.len() >= 3 && signed_area(&h).abs() > 4.0 {
                if signed_area(&h) > 0.0 {
                    h.reverse();
                }
                holes.push(h);
            }
        }
        let ring = if holes.is_empty() { outer.clone() } else { bridge_holes(&outer, &holes) };
        let tris = triangulate_bridged(&ring).or_else(|| {
            holes.clear();
            triangulate_bridged(&outer)
        });
        let ring = if holes.is_empty() { outer.clone() } else { ring };
        let Some(tris) = tris else { continue };
        // trees on a lat/lon grid
        if let Some(sp) = tree_spacing(ra.kind) {
            let lat0 = ra.rings[0].iter().map(|p| p.0).sum::<f64>() / ra.rings[0].len() as f64;
            let (dla, dlo) = (sp / 111_320.0, sp / (111_320.0 * lat0.to_radians().cos().max(0.05)));
            let (mut s, mut w, mut n, mut e) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
            for p in &ra.rings[0] {
                s = s.min(p.0);
                n = n.max(p.0);
                w = w.min(p.1);
                e = e.max(p.1);
            }
            let rings_ll = &ra.rings;
            'grid: for i in (s / dla).floor() as i64..=(n / dla).ceil() as i64 {
                for j in (w / dlo).floor() as i64..=(e / dlo).ceil() as i64 {
                    if trees.len() >= MAX_TREES_PER_TILE.load(std::sync::atomic::Ordering::Relaxed) {
                        break 'grid;
                    }
                    let h = hash2(i, j);
                    let (jx, jy) = ((h & 0xffff) as f64 / 65535.0, ((h >> 16) & 0xffff) as f64 / 65535.0);
                    let (la, lo) = ((i as f64 + 0.15 + 0.7 * jx) * dla, (j as f64 + 0.15 + 0.7 * jy) * dlo);
                    if !point_in_rings((la, lo), rings_ll) {
                        continue;
                    }
                    let p = frame.to_local(la, lo, 0.0);
                    if !avoid(p[0], p[1]) {
                        trees.push([(p[0] * 100.0).round() / 100.0, (p[1] * 100.0).round() / 100.0]);
                    }
                }
            }
        }
        areas.push(Area { kind: ra.kind, ring: ring.iter().map(|p| [p.0, p.1]).collect(), tris: tris.iter().map(|t| t.map(|i| i as u32)).collect() });
    }
    (areas, trees)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tags(p: &[(&str, &str)]) -> Vec<(String, String)> {
        p.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect()
    }

    fn tri_area(r: &[[f64; 2]], t: &[[u32; 3]]) -> f64 {
        t.iter()
            .map(|t| {
                let (a, b, c) = (r[t[0] as usize], r[t[1] as usize], r[t[2] as usize]);
                ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs() / 2.0
            })
            .sum()
    }

    #[test]
    fn kinds_from_tags() {
        assert_eq!(area_kind(&tags(&[("natural", "water")])), Some(AreaKind::Water));
        assert_eq!(area_kind(&tags(&[("landuse", "forest")])), Some(AreaKind::Forest));
        assert_eq!(area_kind(&tags(&[("leisure", "park")])), Some(AreaKind::Park));
        assert_eq!(area_kind(&tags(&[("landuse", "meadow")])), Some(AreaKind::Grass));
        assert_eq!(area_kind(&tags(&[("natural", "beach")])), Some(AreaKind::Sand));
        assert_eq!(area_kind(&tags(&[("leisure", "park"), ("building", "yes")])), None);
        assert_eq!(area_kind(&tags(&[("landuse", "residential")])), None);
        assert_eq!(area_kind(&tags(&[("amenity", "parking")])), Some(AreaKind::Parking));
        assert_eq!(area_kind(&tags(&[("amenity", "parking"), ("parking", "surface")])), Some(AreaKind::Parking));
        assert_eq!(area_kind(&tags(&[("amenity", "parking"), ("parking", "underground")])), None);
    }

    #[test]
    fn clips_rings_to_the_tile() {
        let b = Bounds { south: 0.0, west: 0.0, north: 1.0, east: 1.0 };
        let r = clip_to_bounds(&[(-0.2, 0.5), (0.5, 1.2), (1.2, 0.5), (0.5, -0.2), (-0.2, 0.5)], &b);
        assert_eq!(r.len(), 8, "diamond clipped by a square: octagon {r:?}");
        assert!(r.iter().all(|p| (0.0..=1.0).contains(&p.0) && (0.0..=1.0).contains(&p.1)));
        assert!(clip_to_bounds(&[(5.0, 5.0), (6.0, 5.0), (6.0, 6.0)], &b).is_empty());
    }

    #[test]
    fn simplifies_dense_rings_under_a_cap() {
        let circle: Vec<(f64, f64)> = (0..2000).map(|i| {
            let a = i as f64 / 2000.0 * std::f64::consts::TAU;
            (100.0 * a.cos(), 100.0 * a.sin())
        }).collect();
        let s = simplify_ring(&circle, 0.8, 300);
        assert!(s.len() <= 300 && s.len() > 20, "{} points", s.len());
        let ratio = signed_area(&s) / signed_area(&circle);
        assert!(ratio > 0.98, "area kept {ratio}");
    }

    #[test]
    fn lake_with_an_island_triangulates_to_the_right_area() {
        let f = LocalFrame::new(0.0, 0.0, 0.0);
        let m = 1.0 / 111_320.0;
        let sq = |c: f64, h: f64| vec![(c - h * m, c - h * m), (c - h * m, c + h * m), (c + h * m, c + h * m), (c + h * m, c - h * m)];
        let raw = RawArea { kind: AreaKind::Water, rings: vec![sq(0.0, 100.0), sq(0.0, 20.0)] };
        let (areas, trees) = build_areas(&[raw], &f, &|_, _| false);
        assert!(trees.is_empty());
        let a = &areas[0];
        let got = tri_area(&a.ring, &a.tris);
        // 1e-5 degrees of latitude are ~0.995 m at the equator: compare with the outer square's own area
        let outer = 2.0 * 99.99954 * 2.0 * 99.33011; // the outer square in local metres
        assert!((got - outer * (1.0 - 0.04)).abs() < 0.002 * outer, "area {got} vs outer {outer}");
    }

    #[test]
    fn forest_gets_trees_away_from_roads_and_stable_positions() {
        let f = LocalFrame::new(10.0, 20.0, 0.0);
        let m = 1.0 / 111_320.0;
        let ring = vec![(10.0, 20.0), (10.0 + 200.0 * m, 20.0), (10.0 + 200.0 * m, 20.0 + 200.0 * m), (10.0, 20.0 + 200.0 * m)];
        let raw = RawArea { kind: AreaKind::Forest, rings: vec![ring] };
        let (_, t1) = build_areas(std::slice::from_ref(&raw), &f, &|_, y| y.abs() < 1e9 && (y - 100.0).abs() < 5.0);
        assert!(t1.len() > 300 && t1.len() < 700, "{} trees in 4 ha", t1.len());
        assert!(t1.iter().all(|p| (p[1] - 100.0).abs() >= 5.0), "no tree on the road");
        // a different frame (re-base) gives the same trees on the ground
        let f2 = LocalFrame::new(10.001, 20.001, 0.0);
        let (_, t2) = build_areas(&[raw], &f2, &|_, _| false);
        let g = |fr: &LocalFrame, p: [f64; 2]| fr.from_local(p[0], p[1], 0.0);
        let first = g(&f2, t2[0]);
        assert!(t1.iter().map(|p| g(&f, *p)).any(|q| (q.0 - first.0).abs() < 1e-6 && (q.1 - first.1).abs() < 1e-6));
    }
}
