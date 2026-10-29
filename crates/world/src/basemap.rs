//! Tiles for the in-game map: Overture vector tiles (base: land, water, land cover/use; transportation: roads;
//! buildings) rasterised on the CPU into 256x256 RGBA images. Zoom levels past an archive's maximum are drawn
//! from the deepest tile available (over-zoom), so the map can zoom into streets.
//!
//! Data: © OpenStreetMap contributors, © Overture Maps Foundation.

use crate::mvt::{self, Feature, GeomType, Layer};
use crate::pmtiles::{PmTiles, RangeSource};

pub const SIZE: usize = 256;
pub type Rgba = [u8; 4];

const LAND: Rgba = [238, 234, 226, 255];
const WATER: Rgba = [164, 198, 230, 255];
const FOREST: Rgba = [196, 220, 184, 255];
const GRASS: Rgba = [214, 232, 196, 255];
const PARK: Rgba = [200, 230, 186, 255];
const URBAN: Rgba = [228, 224, 218, 255];
const SAND: Rgba = [240, 230, 196, 255];
const ICE: Rgba = [246, 250, 252, 255];
const BUILDING: Rgba = [208, 200, 190, 255];
const BUILDING_EDGE: Rgba = [180, 170, 160, 255];

/// An RGBA image.
#[derive(Clone)]
pub struct Raster {
    pub px: Vec<u8>,
}

impl Raster {
    pub fn filled(c: Rgba) -> Raster {
        Raster { px: c.iter().copied().cycle().take(SIZE * SIZE * 4).collect() }
    }

    pub fn get(&self, x: usize, y: usize) -> Rgba {
        let i = (y * SIZE + x) * 4;
        [self.px[i], self.px[i + 1], self.px[i + 2], self.px[i + 3]]
    }

    fn blend(&mut self, x: usize, y: usize, c: Rgba, cover: f32) {
        let a = (c[3] as f32 / 255.0) * cover.clamp(0.0, 1.0);
        if a <= 0.0 {
            return;
        }
        let i = (y * SIZE + x) * 4;
        for k in 0..3 {
            self.px[i + k] = (self.px[i + k] as f32 * (1.0 - a) + c[k] as f32 * a).round() as u8;
        }
        self.px[i + 3] = 255;
    }

    /// Fill polygons (all rings together, even-odd: holes come out right) with 4x vertical supersampling.
    pub fn fill(&mut self, rings: &[Vec<(f32, f32)>], c: Rgba) {
        const SUB: usize = 4;
        let mut cover = vec![0f32; SIZE];
        let (mut ymin, mut ymax) = (f32::MAX, f32::MIN);
        for r in rings {
            for p in r {
                ymin = ymin.min(p.1);
                ymax = ymax.max(p.1);
            }
        }
        if ymax < 0.0 || ymin > SIZE as f32 {
            return;
        }
        let y0 = ymin.max(0.0) as usize;
        let y1 = (ymax.ceil() as usize).min(SIZE);
        let mut xs: Vec<f32> = Vec::new();
        for y in y0..y1 {
            cover.iter_mut().for_each(|c| *c = 0.0);
            let mut any = false;
            for s in 0..SUB {
                let sy = y as f32 + (s as f32 + 0.5) / SUB as f32;
                xs.clear();
                for r in rings {
                    let n = r.len();
                    for i in 0..n {
                        let (a, b) = (r[i], r[(i + 1) % n]);
                        if (a.1 <= sy) != (b.1 <= sy) {
                            xs.push(a.0 + (sy - a.1) / (b.1 - a.1) * (b.0 - a.0));
                        }
                    }
                }
                xs.sort_by(|a, b| a.total_cmp(b));
                for pair in xs.chunks(2) {
                    if pair.len() < 2 {
                        break;
                    }
                    let (l, r) = (pair[0].max(0.0), pair[1].min(SIZE as f32));
                    if r <= l {
                        continue;
                    }
                    any = true;
                    let (li, ri) = (l as usize, (r.ceil() as usize).min(SIZE));
                    for x in li..ri {
                        // horizontal coverage of pixel x by [l, r]
                        let c = (r.min(x as f32 + 1.0) - l.max(x as f32)).max(0.0);
                        cover[x] += c / SUB as f32;
                    }
                }
            }
            if any {
                for x in 0..SIZE {
                    if cover[x] > 0.0 {
                        self.blend(x, y, c, cover[x]);
                    }
                }
            }
        }
    }

    /// Thick polyline (width in pixels), drawn as one quad per segment plus a square joint.
    pub fn stroke(&mut self, pts: &[(f32, f32)], width: f32, c: Rgba) {
        let h = width / 2.0;
        let mut quads: Vec<Vec<(f32, f32)>> = Vec::new();
        for w in pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let l = (dx * dx + dy * dy).sqrt();
            if l < 1e-4 {
                continue;
            }
            let (nx, ny) = (-dy / l * h, dx / l * h);
            // extend by half a width so joints close
            let (ex, ey) = (dx / l * h * 0.9, dy / l * h * 0.9);
            quads.push(vec![(a.0 - ex + nx, a.1 - ey + ny), (b.0 + ex + nx, b.1 + ey + ny), (b.0 + ex - nx, b.1 + ey - ny), (a.0 - ex - nx, a.1 - ey - ny)]);
        }
        // each quad separately: overlapping quads would cancel under even-odd
        for q in quads {
            self.fill(&[q], c);
        }
    }
}

/// Where a target tile sits inside a (possibly shallower) source tile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Window {
    pub sz: u8,
    pub sx: u32,
    pub sy: u32,
    /// target tiles per source tile along an axis (2^(z - sz))
    pub scale: f32,
    pub ox: f32,
    pub oy: f32,
}

pub fn window(z: u8, x: u32, y: u32, max_zoom: u8) -> Window {
    let sz = z.min(max_zoom);
    let d = z - sz;
    let (sx, sy) = (x >> d, y >> d);
    Window { sz, sx, sy, scale: (1u32 << d) as f32, ox: (x - (sx << d)) as f32, oy: (y - (sy << d)) as f32 }
}

impl Window {
    fn px(&self, extent: u32, p: (i32, i32)) -> (f32, f32) {
        let e = extent as f32;
        (((p.0 as f32 / e) * self.scale - self.ox) * SIZE as f32, ((p.1 as f32 / e) * self.scale - self.oy) * SIZE as f32)
    }
    fn ring(&self, extent: u32, r: &[(i32, i32)]) -> Vec<(f32, f32)> {
        r.iter().map(|&p| self.px(extent, p)).collect()
    }
}

fn prop<'a>(f: &'a Feature, k: &str) -> &'a str {
    f.prop(k).and_then(|v| v.as_str()).unwrap_or("")
}

/// Feature kind in the Protomaps basemap schema (also read, so any Protomaps/OSM archive works as a map).
fn pkind(f: &Feature) -> &str {
    let k = prop(f, "pmap:kind");
    if k.is_empty() {
        prop(f, "kind")
    } else {
        k
    }
}

fn polys(r: &mut Raster, w: &Window, l: &Layer, f: &Feature, c: Rgba) {
    if f.kind == GeomType::Polygon {
        let rings: Vec<Vec<(f32, f32)>> = f.parts.iter().map(|p| w.ring(l.extent, p)).collect();
        r.fill(&rings, c);
    }
}

fn cover_colour(f: &Feature) -> Option<Rgba> {
    Some(match prop(f, "subtype") {
        "forest" | "wetland" | "mangrove" => FOREST,
        "grass" | "shrub" | "crop" | "moss" => GRASS,
        "urban" => URBAN,
        "barren" => SAND,
        "snow" => ICE,
        _ => return None,
    })
}

fn use_colour(f: &Feature) -> Option<Rgba> {
    let s = prop(f, "subtype");
    let c = match prop(f, "class") {
        "" => pkind(f),
        c => c,
    };
    Some(match (s, c) {
        (_, "park" | "garden" | "playground" | "recreation_ground" | "village_green" | "dog_park") => PARK,
        (_, "forest" | "wood" | "scrub" | "wetland") => FOREST,
        (_, "pitch") => [190, 224, 190, 255],
        (_, "residential" | "school" | "university" | "college" | "hospital" | "industrial" | "commercial" | "retail" | "military" | "pedestrian") => URBAN,
        (_, "grass" | "meadow" | "greenfield" | "farmland" | "orchard" | "vineyard" | "allotments" | "cemetery" | "golf_course") => GRASS,
        (_, "beach" | "sand") => SAND,
        ("park" | "recreation" | "golf" | "horticulture", _) => PARK,
        ("agriculture", _) => GRASS,
        ("residential" | "developed" | "education" | "medical" | "transportation" | "entertainment", _) => URBAN,
        _ => return None,
    })
}

/// (width at zoom 16 in px, colour, minimum zoom shown)
fn road_style(class: &str) -> Option<(f32, Rgba, u8)> {
    Some(match class {
        "motorway" => (9.0, [233, 144, 120, 255], 5),
        "trunk" => (8.0, [242, 178, 128, 255], 6),
        "primary" => (7.5, [246, 206, 140, 255], 8),
        "secondary" => (7.0, [247, 235, 170, 255], 10),
        "tertiary" => (6.5, [255, 255, 255, 255], 11),
        "residential" | "unclassified" | "living_street" => (5.5, [255, 255, 255, 255], 13),
        "service" => (3.5, [255, 255, 255, 255], 15),
        "track" | "unknown" => (2.5, [214, 200, 172, 255], 14),
        _ => return None,
    })
}

/// Draw one decoded base-theme tile. `None` = the archive has no tile there (open ocean at low zoom).
pub fn draw_base(r: &mut Raster, w: &Window, layers: Option<&[Layer]>) {
    let Some(layers) = layers else {
        *r = Raster::filled(WATER);
        return;
    };
    let get = |n: &str| layers.iter().find(|l| l.name == n).or_else(|| match n {
        "land" => layers.iter().find(|l| l.name == "earth"),
        "land_use" => layers.iter().find(|l| l.name == "landuse"),
        _ => None,
    });
    // the archive has land polygons: start from water and lay the land on it; otherwise from land
    let has_land = get("land").is_some_and(|l| l.features.iter().any(|f| f.kind == GeomType::Polygon));
    let has_ocean = get("water").is_some_and(|l| l.features.iter().any(|f| prop(f, "subtype") == "ocean"));
    if has_land && !has_ocean {
        *r = Raster::filled(WATER);
    }
    if let Some(l) = get("land") {
        for f in &l.features {
            let c = match prop(f, "subtype") {
                "glacier" => ICE,
                "sand" => SAND,
                "forest" => FOREST,
                "grass" | "shrub" | "wetland" => GRASS,
                _ => LAND,
            };
            polys(r, w, l, f, c);
        }
    }
    if let Some(l) = get("land_cover") {
        for f in &l.features {
            if let Some(c) = cover_colour(f) {
                polys(r, w, l, f, c);
            }
        }
    }
    for l in get("land_use").into_iter().chain(layers.iter().filter(|l| l.name == "natural")) {
        for f in &l.features {
            if let Some(c) = use_colour(f) {
                polys(r, w, l, f, c);
            }
        }
    }
    for l in layers.iter().filter(|l| l.name == "physical_line") {
        for f in &l.features {
            if f.kind == GeomType::Line && matches!(pkind(f), "river" | "canal" | "stream") {
                let wd = if pkind(f) == "stream" { 1.5 } else { 3.0 };
                for p in &f.parts {
                    r.stroke(&w.ring(l.extent, p), wd * w.scale.sqrt(), WATER);
                }
            }
        }
    }
    if let Some(l) = get("water") {
        for f in &l.features {
            match f.kind {
                GeomType::Polygon => polys(r, w, l, f, WATER),
                GeomType::Line => {
                    let wd = if matches!(prop(f, "subtype"), "river" | "canal") { 3.0 } else { 1.5 };
                    for p in &f.parts {
                        r.stroke(&w.ring(l.extent, p), wd * w.scale.sqrt(), WATER);
                    }
                }
                _ => {}
            }
        }
    }
}

/// Roads from a transportation-theme tile; widths scale with the zoom shown.
pub fn draw_roads(r: &mut Raster, w: &Window, z: u8, layers: &[Layer]) {
    let Some(l) = layers.iter().find(|l| l.name == "segment" || l.name == "roads") else { return };
    let zoom_k = 2f32.powf(z as f32 - 16.0);
    let mut segs: Vec<(usize, &Feature, (f32, Rgba, u8))> = Vec::new();
    for f in &l.features {
        if f.kind != GeomType::Line {
            continue;
        }
        let class = if prop(f, "subtype") == "road" {
            prop(f, "class")
        } else {
            match pkind(f) {
                "highway" => "motorway",
                "major_road" => "primary",
                "medium_road" => "tertiary",
                "minor_road" => "residential",
                "other" => "service",
                _ => continue,
            }
        };
        if let Some(st) = road_style(class) {
            if z >= st.2 {
                segs.push((rank(class), f, st));
            }
        }
    }
    // minor roads first, casings under fills
    segs.sort_by_key(|s| s.0);
    for (_, f, (wd, _, _)) in &segs {
        let wpx = (wd * zoom_k).max(1.2) + 1.5;
        for p in &f.parts {
            r.stroke(&w.ring(l.extent, p), wpx, [170, 160, 150, 255]);
        }
    }
    for (_, f, (wd, c, _)) in &segs {
        let wpx = (wd * zoom_k).max(1.6);
        for p in &f.parts {
            r.stroke(&w.ring(l.extent, p), wpx, *c);
        }
    }
}

fn rank(class: &str) -> usize {
    ["track", "unknown", "service", "unclassified", "living_street", "residential", "tertiary", "secondary", "primary", "trunk", "motorway"].iter().position(|c| *c == class).unwrap_or(0)
}

pub fn draw_buildings(r: &mut Raster, w: &Window, layers: &[Layer]) {
    for l in layers.iter().filter(|l| l.name == "building" || l.name == "buildings") {
        for f in &l.features {
            if f.kind == GeomType::Polygon {
                let rings: Vec<Vec<(f32, f32)>> = f.parts.iter().map(|p| w.ring(l.extent, p)).collect();
                r.fill(&rings, BUILDING_EDGE);
                // a slightly inset lighter fill reads as an outline
                r.fill(&rings, [BUILDING[0], BUILDING[1], BUILDING[2], 220]);
            }
        }
    }
}

/// The three Overture archives the map reads (any may be missing).
pub struct MapSource<S: RangeSource> {
    pub base: Option<PmTiles<S>>,
    pub transport: Option<PmTiles<S>>,
    pub buildings: Option<PmTiles<S>>,
}

/// Deepest zoom the map offers (over-zoomed from the archives' own maximum).
pub const MAX_ZOOM: u8 = 18;

impl<S: RangeSource> MapSource<S> {
    fn decoded(p: &PmTiles<S>, w: &Window) -> Result<Option<Vec<Layer>>, String> {
        match p.get(w.sz, w.sx, w.sy)? {
            Some(b) => mvt::decode(&b).map(Some),
            None => Ok(None),
        }
    }

    /// Render map tile z/x/y (Web Mercator, XYZ scheme).
    pub fn render(&self, z: u8, x: u32, y: u32) -> Result<Raster, String> {
        let mut r = Raster::filled(LAND);
        match &self.base {
            Some(b) => {
                let w = window(z, x, y, b.header.max_zoom);
                let layers = Self::decoded(b, &w)?;
                draw_base(&mut r, &w, layers.as_deref());
            }
            None => {}
        }
        if z >= 13 {
            if let Some(b) = &self.buildings {
                let w = window(z, x, y, b.header.max_zoom);
                if let Some(layers) = Self::decoded(b, &w)? {
                    draw_buildings(&mut r, &w, &layers);
                }
            }
        }
        if z >= 5 {
            if let Some(t) = &self.transport {
                if z >= t.header.min_zoom {
                    let w = window(z, x, y, t.header.max_zoom);
                    if let Some(layers) = Self::decoded(t, &w)? {
                        draw_roads(&mut r, &w, z, &layers);
                    }
                }
            }
        }
        Ok(r)
    }
}

/// Web Mercator: (lat, lon) -> world coordinates in [0, 1) (x east, y south).
pub fn to_world(lat: f64, lon: f64) -> (f64, f64) {
    let lat = lat.clamp(-85.05, 85.05).to_radians();
    ((lon + 180.0) / 360.0, (1.0 - (lat.tan() + 1.0 / lat.cos()).ln() / std::f64::consts::PI) / 2.0)
}

pub fn from_world(x: f64, y: f64) -> (f64, f64) {
    let lon = x * 360.0 - 180.0;
    let lat = (std::f64::consts::PI * (1.0 - 2.0 * y)).sinh().atan().to_degrees();
    (lat, lon)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mvt::{GeomType, Value};

    fn feat(kind: GeomType, props: &[(&str, &str)], parts: Vec<Vec<(i32, i32)>>) -> Feature {
        Feature { id: 0, kind, props: props.iter().map(|(k, v)| (k.to_string(), Value::Str(v.to_string()))).collect(), parts }
    }

    #[test]
    fn fills_squares_with_holes_and_antialiased_edges() {
        let mut r = Raster::filled([0, 0, 0, 255]);
        let outer = vec![(10.0, 10.0), (110.0, 10.0), (110.0, 110.0), (10.0, 110.0)];
        let hole = vec![(40.0, 40.0), (80.0, 40.0), (80.0, 80.0), (40.0, 80.0)];
        r.fill(&[outer, hole], [255, 255, 255, 255]);
        assert_eq!(r.get(20, 20), [255, 255, 255, 255]);
        assert_eq!(r.get(60, 60), [0, 0, 0, 255], "hole stays empty");
        assert_eq!(r.get(5, 5), [0, 0, 0, 255]);
        // half-covered pixel at a fractional edge
        let mut r2 = Raster::filled([0, 0, 0, 255]);
        r2.fill(&[vec![(0.0, 0.0), (20.5, 0.0), (20.5, 10.0), (0.0, 10.0)]], [200, 200, 200, 255]);
        assert_eq!(r2.get(20, 5)[0], 100);
    }

    #[test]
    fn strokes_lines() {
        let mut r = Raster::filled([0, 0, 0, 255]);
        r.stroke(&[(10.0, 100.0), (200.0, 100.0), (200.0, 10.0)], 6.0, [255, 0, 0, 255]);
        assert_eq!(r.get(100, 100), [255, 0, 0, 255]);
        assert_eq!(r.get(200, 50), [255, 0, 0, 255]);
        assert_eq!(r.get(100, 110), [0, 0, 0, 255]);
    }

    #[test]
    fn overzoom_windows() {
        let w = window(16, 3 * 4 + 1, 5 * 4 + 2, 14);
        assert_eq!((w.sz, w.sx, w.sy, w.scale, w.ox, w.oy), (14, 3, 5, 4.0, 1.0, 2.0));
        assert_eq!(window(10, 7, 9, 14), Window { sz: 10, sx: 7, sy: 9, scale: 1.0, ox: 0.0, oy: 0.0 });
        // the second quarter of the source tile fills the target tile
        assert_eq!(w.px(4096, (1024, 2048)), (0.0, 0.0));
        assert_eq!(w.px(4096, (2048, 3072)), (256.0, 256.0));
    }

    #[test]
    fn base_tile_land_water_and_parks() {
        let e = 4096;
        let land = Layer { name: "land".into(), extent: e, features: vec![feat(GeomType::Polygon, &[("subtype", "land")], vec![vec![(0, 0), (2048, 0), (2048, 4096), (0, 4096)]])] };
        let water = Layer { name: "water".into(), extent: e, features: vec![feat(GeomType::Polygon, &[("subtype", "lake")], vec![vec![(200, 200), (800, 200), (800, 800), (200, 800)]])] };
        let lu = Layer { name: "land_use".into(), extent: e, features: vec![feat(GeomType::Polygon, &[("subtype", "park"), ("class", "park")], vec![vec![(200, 2000), (1500, 2000), (1500, 3000), (200, 3000)]])] };
        let w = window(10, 0, 0, 14);
        let mut r = Raster::filled(LAND);
        draw_base(&mut r, &w, Some(&[land, water, lu]));
        assert_eq!(r.get(200, 128), WATER, "no land polygon there: sea");
        assert_eq!(r.get(100, 200), LAND);
        assert_eq!(r.get(30, 30), WATER, "lake");
        assert_eq!(r.get(50, 160), PARK);
        let mut r = Raster::filled(LAND);
        draw_base(&mut r, &w, None);
        assert_eq!(r.get(10, 10), WATER, "missing tile = open sea");
    }

    #[test]
    fn roads_by_class_and_zoom() {
        let seg = |class: &str, y: i32| feat(GeomType::Line, &[("subtype", "road"), ("class", class)], vec![vec![(0, y), (4096, y)]]);
        let l = Layer { name: "segment".into(), extent: 4096, features: vec![seg("motorway", 1032), seg("residential", 3072)] };
        let w = window(12, 0, 0, 14);
        let mut r = Raster::filled(LAND);
        draw_roads(&mut r, &w, 12, std::slice::from_ref(&l));
        assert_eq!(r.get(128, 64), [233, 144, 120, 255]);
        assert_eq!(r.get(128, 192), LAND, "residential hidden at z12");
        let w = window(15, 0, 0, 14);
        let mut r = Raster::filled(LAND);
        draw_roads(&mut r, &w, 15, &[l]);
        assert_eq!(r.get(128, 384 / 2 - 0), LAND);
        // z15 shows the left-top quarter of the z14 tile: residential (y = 3072/4096) is outside, motorway at y=128
        assert_eq!(r.get(128, 128), [233, 144, 120, 255]);
    }

    #[test]
    fn mercator_roundtrip() {
        let (x, y) = to_world(53.9, 27.56);
        let (la, lo) = from_world(x, y);
        assert!((la - 53.9).abs() < 1e-9 && (lo - 27.56).abs() < 1e-9);
        assert_eq!(to_world(0.0, 0.0), (0.5, 0.5));
    }
}
