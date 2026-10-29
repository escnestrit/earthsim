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
        Some("garage") | Some("garages") | Some("shed") | Some("kiosk") | Some("carport") | Some("toilets") => 1.0,
        Some("monument") => 4.0,
        Some("house") | Some("detached") | Some("hut") | Some("chapel") => 2.0,
        Some("retail") | Some("supermarket") => 1.5,
        Some("church") | Some("mosque") | Some("synagogue") | Some("temple") => 4.0,
        Some("school") | Some("kindergarten") => if tag(tags, "building") == Some("kindergarten") { 2.0 } else { 4.0 },
        _ => 5.0,
    };
    (n * LEVEL_H, "default")
}

/// What a building is used for, from its class, `amenity` and `building:use`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Use {
    Religious,
    Education,
    Medical,
    Office,
    Retail,
    Garage,
    Industrial,
    Civic,
    /// monuments, obelisks, memorials: stone, no windows
    Monument,
    /// chimneys, towers, silos, tanks: plain walls
    Structure,
    Other,
}

pub fn use_of(tags: &Tags) -> Use {
    let class = tag(tags, "building").unwrap_or("yes");
    let amenity = tag(tags, "amenity").unwrap_or("");
    let bu = tag(tags, "building:use").unwrap_or("");
    let any = |v: &[&str]| v.contains(&class) || v.contains(&amenity) || v.contains(&bu);
    if any(&["monument", "memorial", "obelisk"]) {
        return Use::Monument;
    }
    if any(&["chimney", "tower", "water_tower", "silo", "storage_tank", "transformer_tower", "bunker"]) {
        return Use::Structure;
    }
    if any(&["church", "cathedral", "chapel", "mosque", "synagogue", "temple", "shrine", "religious", "place_of_worship", "monastery"]) {
        Use::Religious
    } else if any(&["school", "university", "college", "kindergarten", "education", "library"]) {
        Use::Education
    } else if any(&["hospital", "clinic", "doctors", "medical"]) {
        Use::Medical
    } else if any(&["garage", "garages", "carport"]) {
        Use::Garage
    } else if any(&["office", "bank", "hotel"]) || tag(tags, "office").is_some() {
        Use::Office
    } else if any(&["retail", "supermarket", "commercial", "kiosk", "marketplace", "mall"]) || tag(tags, "shop").is_some() {
        Use::Retail
    } else if any(&["industrial", "warehouse", "manufacture", "factory", "hangar", "service"]) {
        Use::Industrial
    } else if any(&["civic", "public", "government", "townhall", "train_station", "transportation", "courthouse", "theatre", "museum", "fire_station", "police"]) {
        Use::Civic
    } else {
        Use::Other
    }
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
    facade_for(tags, height, 0)
}

/// Stable seed for a building id (per-building variety that survives reloads).
pub fn seed_of(id: &str) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in id.bytes() {
        h = (h ^ b as u64).wrapping_mul(0x100000001b3);
    }
    h ^ (h >> 29)
}

/// Facade from map attributes; where the data says nothing, `seed` picks a variant (window layout) and a
/// colour from a palette typical for the style, so neighbouring buildings differ.
pub fn facade_for(tags: &Tags, height: f64, seed: u64) -> scene::Facade {
    use scene::mesh::mat;
    let mut f = scene::Facade::default();
    let material = tag(tags, "building:material").map(|m| m.to_ascii_lowercase());
    let class = tag(tags, "building").unwrap_or("yes");
    let levels = tag(tags, "building:levels").and_then(|l| l.trim().parse::<f64>().ok()).filter(|l| *l >= 1.0 && *l < 300.0);
    let usage = use_of(tags);
    f.style = match material.as_deref() {
        Some("brick") => mat::WALL_BRICK,
        Some("glass") | Some("mirror") => mat::WALL_GLASS,
        Some("concrete") | Some("panel") | Some("panels") | Some("reinforced_concrete") | Some("cement_block") => mat::WALL_PANEL,
        Some("wood") | Some("timber") | Some("timber_framing") | Some("log") => mat::WALL_WOOD,
        Some("metal") | Some("steel") | Some("metal_plates") | Some("aluminium") => mat::WALL_METAL,
        Some(_) => mat::WALL_PLASTER,
        None => match class {
            _ if usage == Use::Garage => mat::WALL_BRICK,
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
    // palettes when the data has no colour (Minsk-like: pale plasters, grey/cream panels, red/yellow brick)
    let pick = |p: &[[f32; 3]]| p[(seed >> 8) as usize % p.len()];
    let palette_colour = if seed == 0 {
        default_colour
    } else {
        match f.style {
            s if s == mat::WALL_BRICK => pick(&[[0.58, 0.30, 0.22], [0.66, 0.38, 0.28], [0.80, 0.68, 0.48], [0.50, 0.26, 0.20], [0.72, 0.62, 0.52]]),
            s if s == mat::WALL_PANEL => pick(&[[0.78, 0.77, 0.74], [0.86, 0.84, 0.78], [0.70, 0.72, 0.74], [0.82, 0.78, 0.68], [0.74, 0.70, 0.66], [0.80, 0.82, 0.84]]),
            s if s == mat::WALL_GLASS => pick(&[[0.55, 0.62, 0.68], [0.45, 0.55, 0.60], [0.60, 0.60, 0.58], [0.40, 0.48, 0.55]]),
            s if s == mat::WALL_METAL => pick(&[[0.62, 0.64, 0.66], [0.55, 0.60, 0.68], [0.70, 0.68, 0.62], [0.50, 0.56, 0.52]]),
            s if s == mat::WALL_WOOD => pick(&[[0.55, 0.40, 0.26], [0.45, 0.32, 0.22], [0.62, 0.50, 0.36]]),
            _ => pick(&[[0.86, 0.80, 0.68], [0.92, 0.88, 0.78], [0.86, 0.74, 0.62], [0.80, 0.84, 0.82], [0.90, 0.82, 0.62], [0.84, 0.70, 0.66], [0.94, 0.92, 0.88], [0.78, 0.80, 0.72]]),
        }
    };
    f.colour = colour.unwrap_or(palette_colour);
    // window layout variant (0..3) of the style
    let style_index = [mat::WALL_PLASTER, mat::WALL_BRICK, mat::WALL_PANEL, mat::WALL_GLASS, mat::WALL_WOOD, mat::WALL_METAL].iter().position(|s| *s == f.style).unwrap_or(0);
    f.style = mat::facade(style_index, if seed == 0 { 0 } else { (seed % 3) as usize });
    // buildings by their use: arched windows for churches and civic buildings, wide glazing for schools
    // and hospitals, shop windows, garage doors, office cladding (unless the data names the material)
    let var = |n: u64| if seed == 0 { 0 } else { ((seed >> 3) % n) as usize };
    let brick = material.as_deref() == Some("brick");
    let special = match usage {
        Use::Religious => Some(mat::facade(8, if brick { 1 } else { [0, 0, 1][var(3)] })),
        Use::Civic if material.is_none() || brick => Some(mat::facade(8, if brick { 1 } else { [0, 2][var(2)] })),
        Use::Education | Use::Medical if material.is_none() || brick || material.as_deref() == Some("plaster") => Some(mat::facade(9, if brick { 1 } else if usage == Use::Medical { 0 } else { [0, 0, 2][var(3)] })),
        Use::Retail if material.is_none() && height <= 9.0 => Some(mat::facade(10, 0)),
        Use::Garage if material.is_none() || brick => Some(mat::facade(10, 2)),
        Use::Office if material.is_none() && height <= 24.0 => Some(mat::facade(11, var(3))),
        Use::Office if material.is_none() => Some(mat::facade(3, var(3))),
        Use::Monument => Some(mat::facade(12, 0)),
        Use::Structure => Some(mat::facade(12, if brick { 2 } else { 1 })),
        _ => None,
    };
    if let Some(sp) = special {
        f.style = sp;
        if colour.is_none() && seed != 0 {
            f.colour = match usage {
                Use::Religious | Use::Civic => pick(&[[0.95, 0.94, 0.90], [0.94, 0.88, 0.70], [0.93, 0.86, 0.76], [0.80, 0.86, 0.90], [0.92, 0.80, 0.78], [0.96, 0.95, 0.93]]),
                Use::Education | Use::Medical => pick(&[[0.94, 0.92, 0.86], [0.92, 0.84, 0.66], [0.84, 0.88, 0.84], [0.96, 0.95, 0.93], [0.88, 0.80, 0.70]]),
                Use::Office => pick(&[[0.85, 0.86, 0.88], [0.72, 0.74, 0.78], [0.88, 0.84, 0.76], [0.62, 0.64, 0.66]]),
                Use::Retail => pick(&[[0.85, 0.83, 0.80], [0.70, 0.70, 0.72], [0.90, 0.86, 0.78]]),
                _ => f.colour,
            };
        }
        if usage == Use::Religious && tag(tags, "roof:colour").is_none() {
            f.roof_colour = pick(&[[0.30, 0.48, 0.38], [0.35, 0.38, 0.42], [0.55, 0.30, 0.22]]);
        }
    }
    if let Some(rc) = tag(tags, "roof:colour").and_then(parse_colour) {
        f.roof_colour = rc;
    }
    if let Some(l) = levels {
        f.floor_height = ((height / l) as f32).clamp(2.4, 6.0);
    } else if usage == Use::Religious {
        f.floor_height = 5.5;
    } else if matches!(usage, Use::Education | Use::Medical | Use::Civic) {
        f.floor_height = 3.4 + if usage == Use::Civic { 0.6 } else { 0.0 };
    } else if usage == Use::Garage {
        f.floor_height = 3.0;
    } else if usage == Use::Retail && height <= 9.0 {
        f.floor_height = (height as f32).clamp(3.0, 4.5);
    } else if mat::facade_parts(f.style).0 == 5 {
        f.floor_height = 6.0;
    } else if seed != 0 {
        f.floor_height = 2.8 + ((seed >> 20) % 5) as f32 * 0.1;
    }
    f.from_data = material.is_some() || colour.is_some();
    f
}

pub struct RoofPlan {
    /// simplified 4-corner footprint (counter-clockwise)
    pub corners: Vec<(f64, f64)>,
    /// ridge end points (gabled/hipped) or the apex (pyramidal)
    pub top: Vec<(f64, f64)>,
    /// index of a long edge (the ridge runs parallel to it)
    pub long_edge: usize,
    pub roof_h: f64,
    /// dome/onion: intermediate rings (scale about the apex, fraction of roof height) between eaves and apex
    pub profile: Vec<(f64, f64)>,
}

/// Drop vertices where the outline barely turns (< ~8 degrees) or that are closer than 0.3 m.
pub fn simplify(ring: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut r = ring.to_vec();
    loop {
        let n = r.len();
        if n <= 3 {
            return r;
        }
        let mut removed = false;
        for i in 0..n {
            let (a, b, c) = (r[(i + n - 1) % n], r[i], r[(i + 1) % n]);
            let (u, v) = ((b.0 - a.0, b.1 - a.1), (c.0 - b.0, c.1 - b.1));
            let (lu, lv) = ((u.0 * u.0 + u.1 * u.1).sqrt(), (v.0 * v.0 + v.1 * v.1).sqrt());
            let straight = lu > 1e-9 && lv > 1e-9 && ((u.0 * v.1 - u.1 * v.0) / (lu * lv)).abs() < 0.14 && (u.0 * v.0 + u.1 * v.1) > 0.0;
            if lu < 0.3 || straight {
                r.remove(i);
                removed = true;
                break;
            }
        }
        if !removed {
            return r;
        }
    }
}

/// Pitched roof for simple (4-corner, convex) footprints: `roof:shape` from the data (Overture roof_shape)
/// or, for small houses without it, a gabled roof as a plausible default. Everything else stays flat.
pub fn roof_plan(tags: &Tags, ring: &[(f64, f64)], h: f64) -> Option<RoofPlan> {
    let class = tag(tags, "building").unwrap_or("yes");
    let shape = match tag(tags, "roof:shape") {
        Some("gabled") | Some("gambrel") | Some("saltbox") => "gabled",
        Some("hipped") | Some("half-hipped") | Some("mansard") => "hipped",
        Some("pyramidal") | Some("cone") | Some("conical") => "pyramidal",
        Some("dome") => "dome",
        Some("onion") => "onion",
        Some(_) => return None,
        None => {
            let small_house = matches!(class, "house" | "detached" | "semidetached_house" | "bungalow" | "cabin" | "farm" | "hut" | "dacha");
            let church = use_of(tags) == Use::Religious && !matches!(class, "mosque") && tag(tags, "religion") != Some("muslim");
            // a tall thin monument: an obelisk's pointed top
            let obelisk = use_of(tags) == Use::Monument && signed_area(ring).abs() < 150.0 && h > 2.5 * signed_area(ring).abs().sqrt();
            if obelisk {
                "pyramidal"
            } else if small_house && h <= 12.0 || church && h <= 30.0 {
                "gabled"
            } else {
                return None;
            }
        }
    };
    let c = simplify(ring);
    // pointed and round roofs on any convex outline (towers, church domes, kiosks)
    if matches!(shape, "pyramidal" | "dome" | "onion") && c.len() != 4 {
        return apex_roof(tags, &c, h, shape);
    }
    if c.len() != 4 {
        return None;
    }
    // convex check
    for i in 0..4 {
        let (a, b, d) = (c[i], c[(i + 1) % 4], c[(i + 2) % 4]);
        if (b.0 - a.0) * (d.1 - b.1) - (b.1 - a.1) * (d.0 - b.0) <= 0.0 {
            return None;
        }
    }
    let len = |i: usize| ((c[(i + 1) % 4].0 - c[i].0).powi(2) + (c[(i + 1) % 4].1 - c[i].1).powi(2)).sqrt();
    let a = if len(0) + len(2) >= len(1) + len(3) { 0 } else { 1 };
    let width = (len(a + 1) + len((a + 3) % 4)) / 2.0;
    let length = (len(a) + len(a + 2)) / 2.0;
    let tagged_h = tag(tags, "roof:height").and_then(|v| v.trim().parse::<f64>().ok()).filter(|v| *v > 0.0);
    let roof_h = tagged_h.unwrap_or(match shape {
        "pyramidal" => width.min(length) * 0.35,
        _ => width * 0.5 * 30f64.to_radians().tan(),
    });
    let roof_h = roof_h.min(h * 0.6);
    if h - roof_h < 2.2 || roof_h < 0.5 {
        return None;
    }
    let mid = |i: usize| ((c[i % 4].0 + c[(i + 1) % 4].0) / 2.0, (c[i % 4].1 + c[(i + 1) % 4].1) / 2.0);
    let top = match shape {
        "pyramidal" => vec![((c[0].0 + c[1].0 + c[2].0 + c[3].0) / 4.0, (c[0].1 + c[1].1 + c[2].1 + c[3].1) / 4.0)],
        _ => {
            let (m1, m3) = (mid(a + 1), mid(a + 3));
            if shape == "hipped" {
                let (dx, dy) = (m3.0 - m1.0, m3.1 - m1.1);
                let l = (dx * dx + dy * dy).sqrt();
                let inset = width / 2.0;
                if l <= 2.0 * inset + 0.2 {
                    vec![((m1.0 + m3.0) / 2.0, (m1.1 + m3.1) / 2.0)]
                } else {
                    let (ux, uy) = (dx / l, dy / l);
                    vec![(m1.0 + ux * inset, m1.1 + uy * inset), (m3.0 - ux * inset, m3.1 - uy * inset)]
                }
            } else {
                vec![m1, m3]
            }
        }
    };
    let profile = match shape {
        "dome" | "onion" => return apex_roof(tags, &c, h, shape),
        _ => Vec::new(),
    };
    Some(RoofPlan { corners: c, top, long_edge: a, roof_h, profile })
}

fn is_convex(c: &[(f64, f64)]) -> bool {
    let n = c.len();
    n >= 3 && (0..n).all(|i| {
        let (a, b, d) = (c[i], c[(i + 1) % n], c[(i + 2) % n]);
        (b.0 - a.0) * (d.1 - b.1) - (b.1 - a.1) * (d.0 - b.0) >= -1e-9
    })
}

/// Roof rising to one apex over the centroid: pyramid/cone, or a dome/onion through profile rings.
fn apex_roof(tags: &Tags, c: &[(f64, f64)], h: f64, shape: &str) -> Option<RoofPlan> {
    if c.len() > 48 || !is_convex(c) {
        return None;
    }
    let n = c.len() as f64;
    let ctr = (c.iter().map(|p| p.0).sum::<f64>() / n, c.iter().map(|p| p.1).sum::<f64>() / n);
    let rad = c.iter().map(|p| (p.0 - ctr.0).hypot(p.1 - ctr.1)).sum::<f64>() / n;
    let tagged_h = tag(tags, "roof:height").and_then(|v| v.trim().parse::<f64>().ok()).filter(|v| *v > 0.0);
    let (roof_h, profile) = match shape {
        "dome" => (tagged_h.unwrap_or(rad), vec![(0.95, 0.3), (0.82, 0.58), (0.6, 0.8), (0.32, 0.95)]),
        "onion" => (tagged_h.unwrap_or(rad * 2.2), vec![(1.12, 0.15), (1.15, 0.3), (0.95, 0.48), (0.55, 0.66), (0.22, 0.82), (0.06, 0.93)]),
        _ => (tagged_h.unwrap_or(rad * 0.9), Vec::new()),
    };
    let roof_h = roof_h.min(h * 0.75);
    if roof_h < 0.3 || h - roof_h < 0.0 {
        return None;
    }
    Some(RoofPlan { corners: c.to_vec(), top: vec![ctr], long_edge: 0, roof_h, profile })
}

/// Depth of a building's corpus (m): the short side of the rectangle with the same area and perimeter.
/// Exact for rectangles, and for L/U-shaped blocks it gives the wing depth, not the whole block's.
pub fn corpus_depth(ring: &[(f64, f64)]) -> f64 {
    let n = ring.len();
    if n < 3 {
        return 0.0;
    }
    let area = signed_area(ring).abs();
    let per: f64 = (0..n).map(|i| (ring[(i + 1) % n].0 - ring[i].0).hypot(ring[(i + 1) % n].1 - ring[i].1)).sum();
    let q = per / 4.0;
    q - (q * q - area).max(0.0).sqrt()
}

#[cfg(test)]
mod depth_tests {
    #[test]
    fn corpus_depth_of_bars_and_l_shapes() {
        let rect = [(0.0, 0.0), (60.0, 0.0), (60.0, 12.0), (0.0, 12.0)];
        assert!((super::corpus_depth(&rect) - 12.0).abs() < 1e-6);
        let l = [(0.0, 0.0), (60.0, 0.0), (60.0, 12.0), (12.0, 12.0), (12.0, 40.0), (0.0, 40.0)];
        assert!((super::corpus_depth(&l) - 12.0).abs() < 0.5, "{}", super::corpus_depth(&l));
    }
}

/// Move a counter-clockwise ring outward by `d` (mitred, limited at sharp corners).
pub fn offset_ring(r: &[(f64, f64)], d: f64) -> Vec<(f64, f64)> {
    let n = r.len();
    (0..n)
        .map(|i| {
            let (p, c, q) = (r[(i + n - 1) % n], r[i], r[(i + 1) % n]);
            let nrm = |a: (f64, f64), b: (f64, f64)| {
                let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                let l = dx.hypot(dy).max(1e-9);
                (dy / l, -dx / l)
            };
            let (n1, n2) = (nrm(p, c), nrm(c, q));
            let (mx, my) = (n1.0 + n2.0, n1.1 + n2.1);
            let ml = mx.hypot(my);
            if ml < 1e-9 {
                return c;
            }
            let (mx, my) = (mx / ml, my / ml);
            let k = d / (mx * n1.0 + my * n1.1).max(0.3);
            (c.0 + mx * k, c.1 + my * k)
        })
        .collect()
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

/// The building class to work with: the mapped `building=*` when it says something, otherwise a guess from
/// the other tags (amenity, shop, office, tourism, historic, Overture subtype), the name, and the shape.
/// Only a slab-shaped block (8.5-14.5 m deep, long, 3+ storeys) with nothing else known is taken for an
/// apartment block; anything else unknown stays a plain "yes" building (no balconies, no stairwells).
pub fn infer_class(tags: &Tags, ring: &[(f64, f64)], area: f64) -> String {
    let class = tag(tags, "building").unwrap_or("yes");
    if !matches!(class, "yes" | "building" | "true" | "1") {
        return class.to_string();
    }
    let t = |k: &str| tag(tags, k).unwrap_or("");
    match t("amenity") {
        "place_of_worship" => return "religious".into(),
        a @ ("school" | "kindergarten" | "university" | "college") => return a.into(),
        "hospital" | "clinic" | "doctors" => return "hospital".into(),
        "bank" | "bureau_de_change" => return "office".into(),
        "townhall" | "courthouse" | "police" | "fire_station" | "library" | "theatre" | "cinema" | "arts_centre" | "community_centre" | "post_office" => return "civic".into(),
        "fuel" | "restaurant" | "cafe" | "fast_food" | "bar" | "pub" | "marketplace" | "pharmacy" => return if area < 40.0 { "kiosk".into() } else { "retail".into() },
        _ => {}
    }
    if !t("shop").is_empty() {
        return if area < 40.0 { "kiosk".into() } else { "retail".into() };
    }
    if !t("office").is_empty() {
        return "office".into();
    }
    if matches!(t("tourism"), "hotel" | "hostel" | "motel" | "guest_house") {
        return "hotel".into();
    }
    if matches!(t("tourism"), "museum" | "gallery") {
        return "civic".into();
    }
    if matches!(t("historic"), "memorial" | "monument") || matches!(t("man_made"), "obelisk" | "monument") {
        return "monument".into();
    }
    if matches!(t("man_made"), "tower" | "chimney" | "water_tower" | "silo" | "storage_tank") {
        return t("man_made").into();
    }
    match t("building:use") {
        "residential" => return "residential".into(),
        "commercial" => return "commercial".into(),
        "industrial" => return "industrial".into(),
        "education" => return "school".into(),
        "religious" => return "religious".into(),
        "medical" => return "hospital".into(),
        "civic" | "entertainment" => return "civic".into(),
        "transportation" => return "transportation".into(),
        "outbuilding" | "agricultural" => return "shed".into(),
        "service" => return "service".into(),
        _ => {}
    }
    // the name usually says what a named building is
    let name = t("name").to_lowercase();
    if !name.is_empty() {
        let has = |ws: &[&str]| ws.iter().any(|w| name.contains(w));
        if has(&["жк ", "жилой", "residen", "apartment"]) {
            return "residential".into();
        }
        if has(&["бизнес", "офис", "business", "office", "банк", "bank", "tower", "плаза", "plaza"]) {
            return "office".into();
        }
        if has(&["гостиниц", "отель", "hotel", "хостел", "hostel"]) {
            return "hotel".into();
        }
        if has(&["торгов", "трц", "тц ", "универмаг", "универсам", "магазин", "рынок", "mall", "market", "shop", "store", "гипермаркет", "супермаркет"]) {
            return "retail".into();
        }
        if has(&["школ", "гимназ", "лицей", "school"]) {
            return "school".into();
        }
        if has(&["детский сад", "ясли", "kindergarten"]) {
            return "kindergarten".into();
        }
        if has(&["университет", "институт", "академи", "колледж", "university", "college"]) {
            return "university".into();
        }
        if has(&["больниц", "поликлин", "госпитал", "hospital", "clinic", "медицин"]) {
            return "hospital".into();
        }
        if has(&["церк", "храм", "собор", "костёл", "костел", "часовн", "church", "cathedral", "chapel"]) {
            return "religious".into();
        }
        if has(&["театр", "музей", "дворец", "библиотек", "кинотеатр", "вокзал", "филармон", "цирк", "theatre", "museum", "station", "library", "palace"]) {
            return "civic".into();
        }
        if has(&["памятник", "монумент", "обелиск", "мемориал", "monument", "memorial", "obelisk"]) {
            return "monument".into();
        }
        if has(&["завод", "фабрик", "комбинат", "factory", "plant", "склад", "warehouse"]) {
            return "industrial".into();
        }
        if has(&["гараж", "garage"]) {
            return "garages".into();
        }
        if has(&["стадион", "арена", "спорт", "бассейн", "stadium", "arena", "sport"]) {
            return "civic".into();
        }
    }
    // shapes
    let height = tag(tags, "height").and_then(|v| v.split_whitespace().next()?.replace(',', ".").parse::<f64>().ok());
    if let Some(h) = height {
        if area < 150.0 && h > 3.0 * area.sqrt() {
            return "monument".into();
        }
    }
    if area < 22.0 {
        return "kiosk".into();
    }
    let depth = corpus_depth(ring);
    let n = ring.len();
    let per: f64 = (0..n).map(|i| (ring[(i + 1) % n].0 - ring[i].0).hypot(ring[(i + 1) % n].1 - ring[i].1)).sum();
    let length = per / 2.0 - depth;
    let storeys = tag(tags, "building:levels").and_then(|l| l.trim().parse::<f64>().ok()).or(height.map(|h| h / 3.0));
    let slab = (8.5..=14.5).contains(&depth) && length >= 24.0 && length / depth >= 2.2;
    if slab && storeys.is_none_or(|s| (3.0..=26.0).contains(&s)) && !matches!(tag(tags, "building:material"), Some("glass") | Some("metal")) {
        return "residential".into();
    }
    "yes".into()
}

/// Tower, dome or spire for a place of worship: from `religion` / `denomination` when mapped, otherwise the
/// tradition most common in the region. Detailed OSM churches (domes as building parts, a dome or onion
/// `roof:shape`) already carry their own shape and get nothing added.
pub fn landmark_for(tags: &Tags, at: (f64, f64), area: f64, base: f64) -> scene::model::Landmark {
    use scene::model::Landmark;
    if use_of(tags) != Use::Religious || base > 0.1 || area < 40.0 || tag(tags, "building:part").is_some() && tag(tags, "building").is_none() {
        return Landmark::None;
    }
    if matches!(tag(tags, "roof:shape"), Some("dome") | Some("onion") | Some("pyramidal") | Some("cone") | Some("conical")) {
        return Landmark::None;
    }
    let class = tag(tags, "building").unwrap_or("yes");
    let religion = tag(tags, "religion").unwrap_or("");
    let denom = tag(tags, "denomination").unwrap_or("").to_ascii_lowercase();
    if class == "mosque" || religion == "muslim" {
        return Landmark::Mosque;
    }
    if matches!(class, "synagogue" | "temple" | "shrine") || !religion.is_empty() && religion != "christian" {
        return Landmark::None;
    }
    let orthodox = if denom.contains("orthodox") {
        true
    } else if denom.contains("catholic") || denom.contains("lutheran") || denom.contains("protestant") || denom.contains("anglican") || denom.contains("baptist") || denom.contains("evangelical") {
        false
    } else {
        let r = crate::series::regions_at(at.0, at.1);
        r.contains(&crate::series::Region::ExUssr) || r.contains(&crate::series::Region::Belarus)
    };
    if class == "chapel" || area < 120.0 {
        if orthodox { Landmark::Chapel } else { Landmark::Spire }
    } else if orthodox {
        Landmark::Orthodox
    } else {
        Landmark::Spire
    }
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
    // a part's walls stand 6 cm proud of the outline it lies in, so coinciding walls never flicker
    if tag(tags, "building:part").is_some() && tag(tags, "building").is_none() {
        ring = offset_ring(&ring, 0.06);
    }
    // what the building is: unknown classes are guessed from the other tags, the name and the shape
    let owned: Vec<(String, String)>;
    let tags: &Tags = if tag(tags, "building:part").is_some() && tag(tags, "building").is_none() {
        tags
    } else {
        let class = infer_class(tags, &ring, area.abs());
        owned = tags.iter().filter(|(k, _)| k != "building").cloned().chain(std::iter::once(("building".to_string(), class))).collect();
        &owned
    };
    let (mut h, mut origin) = estimate_height(tags);
    // typical design (series / project) from the data and the outline
    let seed = seed_of(id).max(1);
    let design = {
        let (la, lo) = latlon_ring[0];
        let levels = tag(tags, "building:levels").and_then(|l| l.trim().parse::<f64>().ok()).filter(|l| *l >= 1.0).map(|l| l.round() as u32);
        let year = tag(tags, "start_date").and_then(|d| d.trim().get(..4)).and_then(|y| y.parse::<i32>().ok());
        let class = tag(tags, "building").unwrap_or(if tag(tags, "building:part").is_some() { "part" } else { "yes" });
        let depth = corpus_depth(&ring);
        let lv = levels.or_else(|| (origin != "default").then(|| (h / LEVEL_H).round().max(1.0) as u32));
        crate::series::match_design(tag(tags, "building:series"), class, la, lo, lv, depth, year, seed)
    };
    if let Some(d) = design {
        // no height in the data: the design's typical storeys
        if origin == "default" {
            h = d.typical_levels(seed) as f64 * d.floor_h as f64 + d.parapet_h as f64 * 0.0;
            origin = "series";
        }
    }
    // parts that start above the ground (overhangs, upper storeys, arches): min_height or building:min_level
    let base = tag(tags, "min_height")
        .and_then(|v| v.split_whitespace().next().and_then(|x| x.replace(',', ".").parse::<f64>().ok()))
        .or_else(|| tag(tags, "building:min_level").and_then(|v| v.trim().parse::<f64>().ok()).map(|l| l * LEVEL_H))
        .filter(|b| b.is_finite() && *b > 0.0 && *b < h - 0.5)
        .unwrap_or(0.0);
    let shaped = roof_plan(tags, &ring, h - base).map(|mut r| {
        r.roof_h = r.roof_h.min(h - base);
        r
    });
    if let Some(sh) = &shaped {
        ring = sh.corners.clone();
    }
    let wall_h = shaped.as_ref().map_or(h, |s| h - s.roof_h);
    let tris = triangulate(&ring);
    let n = ring.len();
    let mut verts: Vec<f32> = Vec::with_capacity(n * 6 + 6);
    for p in &ring {
        verts.extend_from_slice(&[p.0 as f32, p.1 as f32, base as f32]);
    }
    for p in &ring {
        verts.extend_from_slice(&[p.0 as f32, p.1 as f32, wall_h as f32]);
    }
    let mut indices: Vec<u32> = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        indices.extend_from_slice(&[i as u32, j as u32, (n + j) as u32, i as u32, (n + j) as u32, (n + i) as u32]);
    }
    let ok_roof = tris.is_some();
    match &shaped {
        Some(sh) => {
            let t = |k: usize| (n + k % n) as u32;
            if !sh.profile.is_empty() {
                // dome / onion: rings scaled about the apex, then the apex
                let apex = sh.top[0];
                let mut prev: Vec<u32> = (0..n).map(t).collect();
                for &(sc, fz) in &sh.profile {
                    let first = (verts.len() / 3) as u32;
                    for p in &ring {
                        let q = (apex.0 + (p.0 - apex.0) * sc, apex.1 + (p.1 - apex.1) * sc);
                        verts.extend_from_slice(&[q.0 as f32, q.1 as f32, (wall_h + sh.roof_h * fz) as f32]);
                    }
                    let cur: Vec<u32> = (0..n as u32).map(|k| first + k).collect();
                    for k in 0..n {
                        let k1 = (k + 1) % n;
                        indices.extend_from_slice(&[prev[k], prev[k1], cur[k1], prev[k], cur[k1], cur[k]]);
                    }
                    prev = cur;
                }
                let ai = (verts.len() / 3) as u32;
                verts.extend_from_slice(&[apex.0 as f32, apex.1 as f32, h as f32]);
                for k in 0..n {
                    indices.extend_from_slice(&[prev[k], prev[(k + 1) % n], ai]);
                }
            } else {
            let base = (verts.len() / 3) as u32;
            for p in &sh.top {
                verts.extend_from_slice(&[p.0 as f32, p.1 as f32, h as f32]);
            }
            if sh.top.len() == 1 {
                for k in 0..n {
                    indices.extend_from_slice(&[t(k), t(k + 1), base]);
                }
            } else {
                // ridge R1 near short edge a+1, R3 near short edge a+3
                let a = sh.long_edge;
                let (r1, r3) = (base, base + 1);
                indices.extend_from_slice(&[t(a), t(a + 1), r1, t(a), r1, r3]);
                indices.extend_from_slice(&[t(a + 2), t(a + 3), r3, t(a + 2), r3, r1]);
                indices.extend_from_slice(&[t(a + 1), t(a + 2), r1]);
                indices.extend_from_slice(&[t(a + 3), t(a), r3]);
            }
            }
        }
        None => {
            if let Some(tr) = &tris {
                for tri in tr {
                    indices.extend_from_slice(&[(n + tri[0]) as u32, (n + tri[1]) as u32, (n + tri[2]) as u32]);
                }
            }
        }
    }
    // a part floating above the ground shows its underside
    if base > 0.0 {
        if let Some(tr) = &tris {
            for tri in tr {
                indices.extend_from_slice(&[tri[0] as u32, tri[2] as u32, tri[1] as u32]);
            }
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
            facade: {
                let mut f = facade_for(tags, h, seed);
                if let Some(d) = design {
                    // the design sets material, window layout, colour and storey height unless the data does
                    if tag(tags, "building:material").is_none() {
                        f.style = scene::mesh::mat::facade(d.style, d.variant);
                    }
                    if tag(tags, "building:colour").is_none() {
                        f.colour = d.palette[(seed >> 8) as usize % d.palette.len()];
                    }
                    if tag(tags, "building:levels").is_none() {
                        f.floor_height = d.floor_h;
                    }
                }
                // big-box stores: sandwich panels rather than shop windows all round
                if use_of(tags) == Use::Retail && f.style == scene::mesh::mat::facade(10, 0) && area.abs() > 1500.0 {
                    f.style = scene::mesh::mat::facade(10, 1);
                }
                f
            },
            landmark: landmark_for(tags, latlon_ring[0], area.abs(), base),
            design: design.map(|d| d.design()),
            kind: if tag(tags, "building:part").is_some() && tag(tags, "building").is_none() { "part".into() } else { tag(tags, "building").unwrap_or("yes").to_string() },
            seed,
            ring: n as u32,
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
    fn unknown_buildings_are_not_taken_for_apartment_blocks() {
        let rect = |w: f64, d: f64| vec![(0.0, 0.0), (w, 0.0), (w, d), (0.0, d)];
        // a business centre with a name, a deep block, a kiosk, an obelisk, a slab of flats
        assert_eq!(infer_class(&t(&[("building", "yes"), ("name", "Бизнес-центр Royal Plaza")]), &rect(40.0, 18.0), 720.0), "office");
        assert_eq!(infer_class(&t(&[("building", "yes"), ("building:levels", "6")]), &rect(40.0, 24.0), 960.0), "yes");
        assert_eq!(infer_class(&t(&[("building", "yes")]), &rect(4.0, 3.0), 12.0), "kiosk");
        assert_eq!(infer_class(&t(&[("building", "yes"), ("shop", "tobacco")]), &rect(5.0, 4.0), 20.0), "kiosk");
        assert_eq!(infer_class(&t(&[("building", "yes"), ("height", "38")]), &rect(8.0, 8.0), 64.0), "monument");
        assert_eq!(infer_class(&t(&[("building", "yes"), ("historic", "memorial")]), &rect(20.0, 20.0), 400.0), "monument");
        assert_eq!(infer_class(&t(&[("building", "yes"), ("building:levels", "9")]), &rect(60.0, 12.0), 720.0), "residential");
        assert_eq!(infer_class(&t(&[("building", "yes"), ("building:use", "commercial")]), &rect(60.0, 12.0), 720.0), "commercial");
        assert_eq!(infer_class(&t(&[("building", "apartments")]), &rect(40.0, 24.0), 960.0), "apartments");
        // what they become: no housing series for "yes"; the obelisk is blank stone with a point
        let fr = crate::geo::LocalFrame::new(53.9, 27.56, 0.0);
        let ll = |r: Vec<(f64, f64)>| -> Vec<(f64, f64)> { r.iter().map(|&(x, y)| { let (a, b, _) = fr.from_local(x, y, 0.0); (a, b) }).collect() };
        let b = build("bc", &ll(rect(40.0, 18.0)), &t(&[("building", "yes"), ("building:levels", "9")]), &fr).unwrap().building;
        assert!(b.design.is_none(), "a deep 9-storey block is not matched to a panel series");
        let o = build("ob", &ll(rect(8.0, 8.0)), &t(&[("building", "yes"), ("height", "38")]), &fr).unwrap().building;
        assert_eq!(o.kind, "monument");
        assert_eq!(scene::mesh::mat::facade_parts(o.facade.style).0, 12);
        assert!(o.verts.len() / 3 > 8, "pointed top");
    }

    #[test]
    fn buildings_by_use_get_their_facade_roof_and_landmark() {
        use scene::mesh::mat;
        use scene::model::Landmark;
        let minsk = (53.9, 27.56);
        let rome = (41.9, 12.5);
        let church = t(&[("building", "church"), ("amenity", "place_of_worship"), ("religion", "christian")]);
        assert_eq!(use_of(&church), Use::Religious);
        assert_eq!(landmark_for(&church, minsk, 400.0, 0.0), Landmark::Orthodox);
        assert_eq!(landmark_for(&church, rome, 400.0, 0.0), Landmark::Spire);
        let cath = t(&[("building", "church"), ("denomination", "roman_catholic")]);
        assert_eq!(landmark_for(&cath, minsk, 400.0, 0.0), Landmark::Spire);
        assert_eq!(landmark_for(&t(&[("building", "chapel")]), minsk, 50.0, 0.0), Landmark::Chapel);
        assert_eq!(landmark_for(&t(&[("building", "mosque")]), minsk, 400.0, 0.0), Landmark::Mosque);
        assert_eq!(landmark_for(&t(&[("building", "church"), ("roof:shape", "onion")]), minsk, 400.0, 0.0), Landmark::None);
        assert_eq!(landmark_for(&t(&[("building", "school")]), minsk, 400.0, 0.0), Landmark::None);
        assert_eq!(mat::facade_parts(facade_for(&church, 12.0, 99).style).0, 8);
        assert_eq!(mat::facade_parts(facade_for(&t(&[("building", "school")]), 12.0, 99).style).0, 9);
        assert_eq!(mat::facade_parts(facade_for(&t(&[("building", "yes"), ("amenity", "school")]), 12.0, 99).style).0, 9);
        assert_eq!(facade_for(&t(&[("building", "garages")]), 3.0, 99).style, mat::facade(10, 2));
        assert_eq!(mat::facade_parts(facade_for(&t(&[("building", "office")]), 15.0, 99).style).0, 11);
        assert_eq!(facade_for(&t(&[("building", "office")]), 60.0, 99).style, mat::facade(3, (99 % 3) as usize));
        // a church gets a gabled roof; a garage row is one storey
        let fr = crate::geo::LocalFrame::new(53.9, 27.56, 0.0);
        let ring: Vec<(f64, f64)> = [(0.0, 0.0), (30.0, 0.0), (30.0, 14.0), (0.0, 14.0)].iter().map(|&(x, y)| {
            let (la, lo, _) = fr.from_local(x, y, 0.0);
            (la, lo)
        }).collect();
        let b = build("c", &ring, &church, &fr).unwrap().building;
        assert!(b.verts.len() / 3 > 8, "ridge vertices");
        assert_eq!(b.landmark, Landmark::Orthodox);
        assert_eq!(estimate_height(&t(&[("building", "garages")])).0, 3.0);
        // every facade id maps back to its style and variant
        for st in 0..mat::FACADE_STYLES {
            for v in 0..3 {
                assert_eq!(mat::facade_parts(mat::facade(st, v)), (st, v));
                assert!(mat::is_facade(mat::facade(st, v)));
            }
        }
    }

    #[test]
    fn pitched_roofs_for_simple_footprints() {
        let fr = crate::geo::LocalFrame::new(53.9, 27.56, 0.0);
        // ~ 12 m x 8 m rectangle, with an extra collinear vertex that must be simplified away
        let (dla, dlo) = (8.0 / 111_320.0, 12.0 / (111_320.0 * 53.9f64.to_radians().cos()));
        let ring = [(53.9, 27.56), (53.9, 27.56 + dlo / 2.0), (53.9, 27.56 + dlo), (53.9 + dla, 27.56 + dlo), (53.9 + dla, 27.56), (53.9, 27.56)];
        for (shape, tops) in [("gabled", 2usize), ("hipped", 2), ("pyramidal", 1)] {
            let b = build("t", &ring, &t(&[("building", "house"), ("roof:shape", shape), ("height", "8")]), &fr).unwrap().building;
            assert_eq!(b.ring, 4);
            assert_eq!(b.verts.len() / 3, 8 + tops, "{shape}");
            let zs: Vec<f32> = b.verts.chunks(3).map(|v| v[2]).collect();
            assert!((zs.iter().cloned().fold(0.0, f32::max) - 8.0).abs() < 1e-4);
            assert!(zs[4] < 8.0 && zs[4] > 2.2, "eaves {}", zs[4]);
            // every index valid, every triangle non-degenerate
            assert!(b.indices.iter().all(|&k| (k as usize) < b.verts.len() / 3));
            // every face points outwards (back faces are culled by the renderer)
            let v = |k: u32| { let k = k as usize * 3; [b.verts[k] as f64, b.verts[k + 1] as f64, b.verts[k + 2] as f64] };
            let nv = b.verts.len() / 3;
            let c: Vec<f64> = (0..3).map(|a| (0..nv).map(|k| v(k as u32)[a]).sum::<f64>() / nv as f64).collect();
            for t in b.indices.chunks(3) {
                let (p, q, r) = (v(t[0]), v(t[1]), v(t[2]));
                let e1 = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
                let e2 = [r[0] - p[0], r[1] - p[1], r[2] - p[2]];
                let n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
                let m = [(p[0] + q[0] + r[0]) / 3.0 - c[0], (p[1] + q[1] + r[1]) / 3.0 - c[1], (p[2] + q[2] + r[2]) / 3.0 - c[2]];
                assert!(n[0] * m[0] + n[1] * m[1] + n[2] * m[2] > 0.0, "{shape}: inward face {t:?}");
            }
        }
        // flat roof tag and big buildings stay flat
        let b = build("t", &ring, &t(&[("building", "house"), ("roof:shape", "flat"), ("height", "8")]), &fr).unwrap().building;
        assert_eq!(b.verts.len() / 3, 2 * b.ring as usize);
        let b = build("t", &ring, &t(&[("building", "apartments"), ("height", "30")]), &fr).unwrap().building;
        assert_eq!(b.verts.len() / 3, 2 * b.ring as usize);
        // default gabled for a small house without roof:shape
        let b = build("t", &ring, &t(&[("building", "house")]), &fr).unwrap().building;
        assert_eq!(b.verts.len() / 3, 10);
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

    #[test]
    fn parts_start_at_min_height_and_domes_rise_round() {
        let f = LocalFrame::new(53.9, 27.56, 0.0);
        let m = 1.0 / 111_320.0;
        let k = 1.0 / (111_320.0 * 53.9f64.to_radians().cos());
        let sq: Vec<(f64, f64)> = vec![(53.9, 27.56), (53.9, 27.56 + 10.0 * k), (53.9 + 10.0 * m, 27.56 + 10.0 * k), (53.9 + 10.0 * m, 27.56), (53.9, 27.56)];
        let b = build("p", &sq, &t(&[("building:part", "yes"), ("height", "30"), ("min_height", "12")]), &f).unwrap().building;
        let zs: Vec<f32> = b.verts.chunks(3).map(|v| v[2]).collect();
        assert!((zs.iter().cloned().fold(f32::MAX, f32::min) - 12.0).abs() < 1e-4, "starts at 12 m");
        // octagonal tower with an onion dome: round profile above the walls, apex at the top
        let oct: Vec<(f64, f64)> = (0..=8).map(|i| {
            let a = (i % 8) as f64 / 8.0 * std::f64::consts::TAU;
            (53.9 + 4.0 * a.sin() * m, 27.56 + 4.0 * a.cos() * k)
        }).collect();
        let b = build("d", &oct, &t(&[("building", "church"), ("height", "20"), ("roof:shape", "onion")]), &f).unwrap().building;
        let zmax = b.verts.chunks(3).map(|v| v[2]).fold(f32::MIN, f32::max);
        assert!((zmax - 20.0).abs() < 1e-3);
        let above_eaves = b.verts.chunks(3).filter(|v| v[2] > 12.0 && v[2] < 19.9).count();
        assert!(above_eaves >= 8 * 5, "onion profile rings: {above_eaves}");
    }
}