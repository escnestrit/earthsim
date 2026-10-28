//! Building Stage A (port of building_pipeline/stage_a.py): footprint extrusion with a flat, ear-clipped roof.
//! This is a procedural approximation: facade style/colour come from tags when present, never reconstructed.
use crate::geo::LocalFrame;
use scene::Building;

pub const LEVEL_H: f64 = 3.0;
pub const ALGO: &str = "stageA-0.2";

pub type Tags = [(String, String)];

pub fn tag<'a>(tags: &'a Tags, key: &str) -> Option<&'a str> {
    tags.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
}

/// (height in metres, origin: "tag" | "estimated" | "default").
pub fn estimate_height(tags: &Tags) -> (f64, &'static str) {
    if let Some(h) = tag(tags, "height") {
        let first = h.split_whitespace().next().unwrap_or("").replace(',', ".");
        if let Ok(v) = first.parse::<f64>() {
            if v.is_finite() && v > 0.0 && v < 1000.0 {
                return (v, "tag");
            }
        }
    }
    if let Some(l) = tag(tags, "building:levels") {
        if let Ok(v) = l.trim().replace(',', ".").parse::<f64>() {
            if v.is_finite() && v > 0.0 && v < 300.0 {
                return (v * LEVEL_H, "estimated");
            }
        }
    }
    let n = match tag(tags, "building") {
        Some("house") | Some("garage") | Some("shed") | Some("detached") | Some("hut") => 2.0,
        _ => 5.0,
    };
    (n * LEVEL_H, "default")
}

/// Parse an OSM/Overture colour: `#rgb`, `#rrggbb` or a common name. Returns sRGB 0..1.
pub fn parse_colour(s: &str) -> Option<[f32; 3]> {
    let s = s.trim().to_ascii_lowercase();
    if let Some(h) = s.strip_prefix('#') {
        let hex = |a: &str| u8::from_str_radix(a, 16).ok().map(|v| v as f32 / 255.0);
        return match h.len() {
            6 => Some([hex(&h[0..2])?, hex(&h[2..4])?, hex(&h[4..6])?]),
            3 => Some([hex(&h[0..1].repeat(2))?, hex(&h[1..2].repeat(2))?, hex(&h[2..3].repeat(2))?]),
            _ => None,
        };
    }
    let c = match s.as_str() {
        "white" => [0.93, 0.93, 0.91],
        "black" => [0.10, 0.10, 0.10],
        "grey" | "gray" => [0.55, 0.55, 0.55],
        "lightgrey" | "lightgray" | "silver" => [0.75, 0.75, 0.75],
        "darkgrey" | "darkgray" => [0.35, 0.35, 0.35],
        "red" => [0.62, 0.20, 0.16],
        "maroon" | "darkred" => [0.45, 0.14, 0.12],
        "brown" => [0.45, 0.30, 0.20],
        "tan" => [0.82, 0.71, 0.55],
        "beige" => [0.88, 0.82, 0.68],
        "cream" | "ivory" => [0.94, 0.90, 0.80],
        "yellow" => [0.92, 0.82, 0.45],
        "orange" => [0.88, 0.55, 0.25],
        "pink" => [0.90, 0.68, 0.68],
        "green" => [0.40, 0.55, 0.38],
        "lightgreen" => [0.62, 0.78, 0.58],
        "blue" => [0.35, 0.48, 0.70],
        "lightblue" => [0.62, 0.74, 0.86],
        "darkblue" | "navy" => [0.18, 0.24, 0.42],
        "teal" => [0.25, 0.50, 0.50],
        "purple" => [0.48, 0.35, 0.55],
        _ => return None,
    };
    Some(c)
}

/// Facade style/colour from tags. Explicit `building:material` / `building:colour` (Overture: facade_material,
/// facade_color) win; otherwise the building class and height choose a plausible default. Not a reconstruction.
pub fn facade_from_tags(tags: &Tags, height: f64) -> scene::Facade {
    use scene::mesh::mat;
    let mut f = scene::Facade::default();
    let material = tag(tags, "building:material").map(|m| m.to_ascii_lowercase());
    let class = tag(tags, "building").unwrap_or("yes");
    let levels = tag(tags, "building:levels").and_then(|l| l.trim().parse::<f64>().ok()).filter(|l| *l >= 1.0 && *l < 300.0);
    f.style = match material.as_deref() {
        Some("brick") => mat::WALL_BRICK,
        Some("glass") | Some("mirror") => mat::WALL_GLASS,
        Some("concrete") | Some("panel") | Some("panels") | Some("reinforced_concrete") | Some("cement_block") => mat::WALL_PANEL,
        Some("wood") | Some("timber") | Some("timber_framing") | Some("log") => mat::WALL_WOOD,
        Some("metal") | Some("steel") | Some("metal_plates") | Some("aluminium") => mat::WALL_METAL,
        Some(_) => mat::WALL_PLASTER,
        None => match class {
            "office" | "commercial" | "retail" if height > 24.0 => mat::WALL_GLASS,
            "industrial" | "warehouse" | "hangar" | "service" | "manufacture" => mat::WALL_METAL,
            "apartments" | "residential" | "dormitory" if height >= 15.0 => mat::WALL_PANEL,
            "garage" | "garages" | "shed" => mat::WALL_BRICK,
            _ => mat::WALL_PLASTER,
        },
    };
    let default_colour = match f.style {
        s if s == mat::WALL_BRICK => [0.58, 0.30, 0.22],
        s if s == mat::WALL_GLASS => [0.55, 0.62, 0.68],
        s if s == mat::WALL_PANEL => [0.78, 0.77, 0.74],
        s if s == mat::WALL_WOOD => [0.55, 0.40, 0.26],
        s if s == mat::WALL_METAL => [0.62, 0.64, 0.66],
        _ => [0.86, 0.80, 0.68],
    };
    let colour = tag(tags, "building:colour").and_then(parse_colour);
    f.colour = colour.unwrap_or(default_colour);
    if let Some(rc) = tag(tags, "roof:colour").and_then(parse_colour) {
        f.roof_colour = rc;
    }
    if let Some(l) = levels {
        f.floor_height = ((height / l) as f32).clamp(2.4, 6.0);
    } else if f.style == mat::WALL_METAL {
        f.floor_height = 6.0;
    }
    f.from_data = material.is_some() || colour.is_some();
    f
}

pub fn signed_area(r: &[(f64, f64)]) -> f64 {
    let n = r.len();
    0.5 * (0..n).map(|i| {
        let (a, b) = (r[i], r[(i + 1) % n]);
        a.0 * b.1 - b.0 * a.1
    }).sum::<f64>()
}

fn cross(o: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
}

/// Ear clipping of a counter-clockwise simple polygon; returns index triples into the ORIGINAL ring.
/// Duplicate and collinear vertices are skipped for the roof only. `None` if the polygon is degenerate or not simple.
pub fn triangulate(ring: &[(f64, f64)]) -> Option<Vec<[usize; 3]>> {
    let near = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).abs() + (a.1 - b.1).abs() < 1e-6;
    let mut idx: Vec<usize> = Vec::new();
    for i in 0..ring.len() {
        if let Some(&l) = idx.last() {
            if near(ring[i], ring[l]) {
                continue;
            }
        }
        idx.push(i);
    }
    while idx.len() > 1 && near(ring[idx[0]], ring[*idx.last().unwrap()]) {
        idx.pop();
    }
    loop {
        if idx.len() < 3 {
            return None;
        }
        let n = idx.len();
        let mut removed = false;
        for k in 0..n {
            if cross(ring[idx[(k + n - 1) % n]], ring[idx[k]], ring[idx[(k + 1) % n]]).abs() < 1e-6 {
                idx.remove(k);
                removed = true;
                break;
            }
        }
        if !removed {
            break;
        }
    }
    let inside = |p, a, b, c| cross(a, b, p) >= 0.0 && cross(b, c, p) >= 0.0 && cross(c, a, p) >= 0.0;
    let mut out = Vec::new();
    while idx.len() > 3 {
        let n = idx.len();
        let mut clipped = false;
        for k in 0..n {
            let (i0, i1, i2) = (idx[(k + n - 1) % n], idx[k], idx[(k + 1) % n]);
            let (a, b, c) = (ring[i0], ring[i1], ring[i2]);
            if cross(a, b, c) <= 0.0 {
                continue;
            }
            if idx.iter().any(|&j| j != i0 && j != i1 && j != i2 && inside(ring[j], a, b, c)) {
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
    out.push([idx[0], idx[1], idx[2]]);
    Some(out)
}

pub struct BuiltBuilding {
    pub building: Building,
    pub ok_roof: bool,
}

/// Build a mesh from a lat/lon ring. Returns `None` for rings that are unusable (too few points, ~zero area).
pub fn build(id: &str, latlon_ring: &[(f64, f64)], tags: &Tags, frame: &LocalFrame) -> Option<BuiltBuilding> {
    let mut ring: Vec<(f64, f64)> = latlon_ring.iter().map(|&(la, lo)| {
        let p = frame.to_local(la, lo, 0.0);
        (p[0], p[1])
    }).collect();
    if ring.len() > 1 && ring.first() == ring.last() {
        ring.pop();
    }
    if ring.len() < 3 {
        return None;
    }
    let area = signed_area(&ring);
    if area.abs() < 2.0 {
        return None;
    }
    if area < 0.0 {
        ring.reverse();
    }
    let (h, origin) = estimate_height(tags);
    let tris = triangulate(&ring);
    let n = ring.len();
    let mut verts: Vec<f32> = Vec::with_capacity(n * 6);
    for p in &ring {
        verts.extend_from_slice(&[p.0 as f32, p.1 as f32, 0.0]);
    }
    for p in &ring {
        verts.extend_from_slice(&[p.0 as f32, p.1 as f32, h as f32]);
    }
    let mut indices: Vec<u32> = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        indices.extend_from_slice(&[i as u32, j as u32, (n + j) as u32, i as u32, (n + j) as u32, (n + i) as u32]);
    }
    let ok_roof = tris.is_some();
    if let Some(t) = &tris {
        for tri in t {
            indices.extend_from_slice(&[(n + tri[0]) as u32, (n + tri[1]) as u32, (n + tri[2]) as u32]);
        }
    }
    Some(BuiltBuilding {
        building: Building {
            id: id.to_string(),
            status: if ok_roof { "PROCEDURAL_APPROXIMATION" } else { "LOW_DETAIL_FALLBACK" }.to_string(),
            footprint: ring.iter().map(|p| [p.0, p.1]).collect(),
            height: h as f32,
            height_origin: origin.to_string(),
            verts,
            indices,
            facade: facade_from_tags(tags, h),
        },
        ok_roof,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(pairs: &[(&str, &str)]) -> Vec<(String, String)> {
        pairs.iter().map(|(a, b)| (a.to_string(), b.to_string())).collect()
    }

    #[test]
    fn facade_from_overture_style_tags() {
        use scene::mesh::mat;
        let f = facade_from_tags(&t(&[("building", "apartments"), ("building:material", "brick"), ("building:colour", "#c0a080"), ("building:levels", "5")]), 16.0);
        assert_eq!(f.style, mat::WALL_BRICK);
        assert!((f.colour[0] - 192.0 / 255.0).abs() < 1e-6);
        assert!((f.floor_height - 3.2).abs() < 1e-6);
        assert!(f.from_data);
        let f = facade_from_tags(&t(&[("building", "office")]), 60.0);
        assert_eq!(f.style, mat::WALL_GLASS);
        assert!(!f.from_data);
        assert_eq!(parse_colour("Beige"), Some([0.88, 0.82, 0.68]));
        assert_eq!(parse_colour("#fff"), Some([1.0, 1.0, 1.0]));
        assert_eq!(parse_colour("#12"), None);
    }

    #[test]
    fn height_rules_match_python() {
        assert_eq!(estimate_height(&t(&[("height", "12.5 m")])), (12.5, "tag"));
        assert_eq!(estimate_height(&t(&[("height", "12,5")])), (12.5, "tag"));
        assert_eq!(estimate_height(&t(&[("building:levels", "4")])), (12.0, "estimated"));
        assert_eq!(estimate_height(&t(&[("building", "house")])), (6.0, "default"));
        assert_eq!(estimate_height(&t(&[("building", "yes")])), (15.0, "default"));
        // garbage falls through to the next rule instead of failing
        assert_eq!(estimate_height(&t(&[("height", "tall"), ("building:levels", "2")])), (6.0, "estimated"));
        assert_eq!(estimate_height(&t(&[("height", "-5"), ("building", "yes")])).1, "default");
        assert_eq!(estimate_height(&t(&[("height", "1e9")])).1, "default");
        assert_eq!(estimate_height(&t(&[("building:levels", "nan")])).1, "default");
    }

    fn tri_area(p: &[(f64, f64)], t: &[[usize; 3]]) -> f64 {
        t.iter().map(|k| 0.5 * cross(p[k[0]], p[k[1]], p[k[2]])).sum()
    }

    #[test]
    fn square_and_l_shape_triangulate_to_the_exact_area() {
        let sq = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let t1 = triangulate(&sq).unwrap();
        assert_eq!(t1.len(), 2);
        assert!((tri_area(&sq, &t1) - 100.0).abs() < 1e-9);
        let l = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (5.0, 5.0), (5.0, 10.0), (0.0, 10.0)];
        let t2 = triangulate(&l).unwrap();
        assert!((tri_area(&l, &t2) - signed_area(&l)).abs() < 1e-9);
    }

    #[test]
    fn duplicate_and_collinear_vertices_are_tolerated() {
        let r = [(0.0, 0.0), (5.0, 0.0), (5.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 5.0)];
        let t = triangulate(&r).unwrap();
        assert!((tri_area(&r, &t) - 100.0).abs() < 1e-9);
        assert!(t.iter().flatten().all(|&i| i < r.len()));
    }

    #[test]
    fn degenerate_input_returns_none() {
        assert!(triangulate(&[(0.0, 0.0), (1.0, 0.0)]).is_none());
        assert!(triangulate(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]).is_none());
        assert!(triangulate(&[]).is_none());
    }

    #[test]
    fn fuzz_star_polygons_triangulate_to_the_exact_area() {
        let mut s: u64 = 0x9E3779B97F4A7C15;
        let mut rnd = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 11) as f64 / (1u64 << 53) as f64
        };
        for _ in 0..500 {
            let n = 3 + (rnd() * 20.0) as usize;
            // angles strictly increasing with gaps < pi => star-shaped, simple
            let mut ang: Vec<f64> = (0..n).map(|k| (k as f64 + 0.15 + 0.7 * rnd()) / n as f64 * std::f64::consts::TAU).collect();
            ang.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let p: Vec<(f64, f64)> = ang.iter().map(|&a| {
                let r = 5.0 + 20.0 * rnd();
                (r * a.cos(), r * a.sin())
            }).collect();
            if signed_area(&p) <= 0.0 {
                continue;
            }
            if let Some(t) = triangulate(&p) {
                assert!((tri_area(&p, &t) - signed_area(&p)).abs() < 1e-6 * signed_area(&p).max(1.0));
                assert_eq!(t.len(), n - 2);
            }
        }
    }

    #[test]
    fn build_makes_walls_and_roof_in_local_metres() {
        let fr = LocalFrame::new(53.9045, 27.5615, 0.0);
        // ~20 m x 10 m rectangle given clockwise: must be re-wound
        let o = fr.from_local(0.0, 0.0, 0.0);
        let a = fr.from_local(20.0, 0.0, 0.0);
        let b = fr.from_local(20.0, 10.0, 0.0);
        let c = fr.from_local(0.0, 10.0, 0.0);
        let ring = [(o.0, o.1), (c.0, c.1), (b.0, b.1), (a.0, a.1), (o.0, o.1)];
        let bb = build("osm:way/1", &ring, &t(&[("building", "yes"), ("building:levels", "3")]), &fr).unwrap();
        assert_eq!(bb.building.status, "PROCEDURAL_APPROXIMATION");
        assert_eq!(bb.building.height_origin, "estimated");
        assert!((bb.building.height - 9.0).abs() < 1e-6);
        assert_eq!(bb.building.verts.len(), 8 * 3);
        assert_eq!(bb.building.indices.len(), (4 * 2 + 2) * 3);
        assert!(bb.building.indices.iter().all(|&i| (i as usize) * 3 + 2 < bb.building.verts.len()));
        let area = signed_area(&bb.building.footprint.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>());
        assert!((area - 200.0).abs() < 0.01, "{area}");
    }

    #[test]
    fn tiny_or_collapsed_rings_are_rejected() {
        let fr = LocalFrame::new(53.9, 27.5, 0.0);
        assert!(build("x", &[(53.9, 27.5), (53.9, 27.5)], &[], &fr).is_none());
        assert!(build("x", &[(53.9, 27.5), (53.90001, 27.5), (53.9, 27.5)], &[], &fr).is_none());
    }
}
