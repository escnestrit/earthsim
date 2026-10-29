//! Overture Maps as a map source. Overture publishes PMTiles archives (vector tiles, all attributes from
//! zoom 14) on S3; a client reads only the byte ranges it needs. Each z15 streaming tile is answered from its
//! z14 parent and converted to the same Overpass-style JSON the rest of the pipeline (cache, parser, scene
//! builder) already understands, with Overture attributes mapped to OSM tag names.
//!
//! Data: © OpenStreetMap contributors, © Overture Maps Foundation (ODbL / CDLA-Permissive per theme).

use crate::fetch::{FetchError, Fetcher};
use crate::mvt::{self, Feature, GeomType, Layer, Value};
use crate::pmtiles::{PmTiles, RangeSource};
use crate::tiles::TileId;
use scene::json::Json;
use std::sync::{Arc, Mutex};

pub const BUCKET: &str = "https://overturemaps-extras-us-west-2.s3.us-west-2.amazonaws.com";
/// Used when the release list cannot be read.
pub const FALLBACK_RELEASE: &str = "2026-10-21.0";
pub const SOURCE_ZOOM: u8 = 14;

pub fn theme_url(release: &str, theme: &str) -> String {
    format!("{BUCKET}/tiles/{release}/{theme}.pmtiles")
}

/// Newest release from an S3 ListObjectsV2 response (`?list-type=2&prefix=tiles/&delimiter=/`).
pub fn newest_release(xml: &str) -> Option<String> {
    xml.split("<Prefix>tiles/")
        .skip(1)
        .filter_map(|s| s.split('/').next())
        .filter(|r| r.len() >= 10 && r.as_bytes()[0].is_ascii_digit())
        .max()
        .map(String::from)
}

fn tile_to_latlon(z: u8, tx: u32, ty: u32, extent: u32, px: i32, py: i32) -> (f64, f64) {
    let n = (1u64 << z) as f64;
    let x = (tx as f64 + px as f64 / extent as f64) / n;
    let y = (ty as f64 + py as f64 / extent as f64) / n;
    let lon = x * 360.0 - 180.0;
    let lat = (std::f64::consts::PI * (1.0 - 2.0 * y)).sinh().atan().to_degrees();
    (lat, lon)
}

fn num(v: Option<&Value>) -> Option<f64> {
    v.and_then(|v| match v {
        Value::Str(s) => s.trim().parse().ok(),
        o => o.as_f64(),
    })
}

fn s(v: Option<&Value>) -> Option<String> {
    v.and_then(|v| match v {
        Value::Str(s) if !s.is_empty() => Some(s.clone()),
        Value::Num(n) => Some(format!("{n}")),
        Value::Int(i) => Some(format!("{i}")),
        _ => None,
    })
}

/// Overture building properties -> OSM-style tags.
pub fn building_tags(f: &Feature) -> Vec<(String, String)> {
    let mut t: Vec<(String, String)> = Vec::new();
    let class = s(f.prop("class")).unwrap_or_else(|| "yes".into());
    t.push(("building".into(), class));
    let mut put = |k: &str, v: Option<String>| {
        if let Some(v) = v {
            t.push((k.into(), v));
        }
    };
    put("height", num(f.prop("height")).filter(|h| *h > 0.0).map(|h| format!("{h:.2}")));
    put("min_height", num(f.prop("min_height")).filter(|h| *h > 0.0).map(|h| format!("{h:.2}")));
    put("building:levels", num(f.prop("num_floors")).filter(|n| *n > 0.0).map(|n| format!("{}", n.round() as i64)));
    put("roof:shape", s(f.prop("roof_shape")));
    put("roof:height", num(f.prop("roof_height")).map(|h| format!("{h:.2}")));
    put("roof:colour", s(f.prop("roof_color")));
    put("roof:material", s(f.prop("roof_material")));
    put("building:colour", s(f.prop("facade_color")));
    put("building:material", s(f.prop("facade_material")));
    put("name", s(f.prop("@name")));
    t
}

/// Overture segment -> (highway class, extra tags), or None if not a drivable road.
pub fn road_tags(f: &Feature) -> Option<Vec<(String, String)>> {
    if s(f.prop("subtype")).as_deref() != Some("road") {
        return None;
    }
    let class = s(f.prop("class"))?;
    let link = s(f.prop("subclass")).as_deref() == Some("link");
    let hw = if link && matches!(class.as_str(), "motorway" | "trunk" | "primary" | "secondary" | "tertiary") { format!("{class}_link") } else { class };
    let mut t = vec![("highway".to_string(), hw)];
    let flags = s(f.prop("road_flags")).unwrap_or_default();
    if flags.contains("is_tunnel") {
        t.push(("tunnel".into(), "yes".into()));
    }
    if flags.contains("is_bridge") {
        t.push(("bridge".into(), "yes".into()));
    }
    if let Some(n) = s(f.prop("@name")) {
        t.push(("name".into(), n));
    }
    Some(t)
}

fn tags_json(t: &[(String, String)]) -> Json {
    Json::Obj(t.iter().map(|(k, v)| (k.clone(), Json::Str(v.clone()))).collect())
}

fn geom_json(pts: &[(f64, f64)]) -> Json {
    Json::Arr(pts.iter().map(|(la, lo)| Json::Obj(vec![("lat".into(), Json::Num(*la)), ("lon".into(), Json::Num(*lo))])).collect())
}

/// Clip a closed ring to the tile square [0, extent]^2 (Sutherland-Hodgman). Vector tiles carry a buffer
/// around each tile; without this a building crossing a tile edge would appear twice, overlapping.
pub fn clip_ring(ring: &[(i32, i32)], extent: i32) -> Vec<(i32, i32)> {
    let mut pts: Vec<(f64, f64)> = ring.iter().map(|&(x, y)| (x as f64, y as f64)).collect();
    if pts.len() > 1 && pts.first() == pts.last() {
        pts.pop();
    }
    let e = extent as f64;
    // (axis, keep >= value?, value)
    for (axis, ge, v) in [(0, true, 0.0), (0, false, e), (1, true, 0.0), (1, false, e)] {
        if pts.is_empty() {
            break;
        }
        let inside = |p: &(f64, f64)| {
            let c = if axis == 0 { p.0 } else { p.1 };
            if ge { c >= v } else { c <= v }
        };
        let mut out = Vec::with_capacity(pts.len() + 4);
        for i in 0..pts.len() {
            let a = pts[i];
            let b = pts[(i + 1) % pts.len()];
            let (ia, ib) = (inside(&a), inside(&b));
            if ia {
                out.push(a);
            }
            if ia != ib {
                let (ca, cb) = if axis == 0 { (a.0, b.0) } else { (a.1, b.1) };
                let t = (v - ca) / (cb - ca);
                out.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
            }
        }
        pts = out;
    }
    let mut r: Vec<(i32, i32)> = pts.iter().map(|p| (p.0.round() as i32, p.1.round() as i32)).collect();
    r.dedup();
    if r.len() >= 3 {
        r.push(r[0]);
        r
    } else {
        Vec::new()
    }
}

/// Endpoint ids so that segments meeting at the same spot count as a junction (~0.3 m grid).
fn point_id(p: (f64, f64)) -> i64 {
    let a = (p.0 * 3e5).round() as i64;
    let b = (p.1 * 3e5).round() as i64;
    (a.wrapping_mul(1_000_003) ^ b).abs()
}

/// Overture base-theme feature -> OSM-style tags understood by `areas::area_kind`.
pub fn area_tags(layer: &str, f: &Feature) -> Option<Vec<(String, String)>> {
    let st = s(f.prop("subtype")).unwrap_or_default();
    let class = s(f.prop("class")).unwrap_or_default();
    let kv = |k: &str, v: &str| Some(vec![(k.to_string(), v.to_string())]);
    match layer {
        "water" => kv("natural", "water"),
        "land" => match st.as_str() {
            "forest" => kv("natural", "wood"),
            "grass" | "shrub" | "wetland" => kv("natural", "grassland"),
            "sand" => kv("natural", "sand"),
            _ => None,
        },
        "land_use" => match (st.as_str(), class.as_str()) {
            (_, "forest" | "wood") => kv("landuse", "forest"),
            (_, "park" | "garden" | "playground" | "dog_park") | ("park", _) => kv("leisure", "park"),
            (_, "pitch" | "golf_course") => kv("leisure", "pitch"),
            (_, "grass" | "meadow" | "village_green" | "recreation_ground" | "cemetery" | "allotments" | "orchard") => kv("landuse", "grass"),
            (_, "beach") => kv("natural", "beach"),
            _ => None,
        },
        _ => None,
    }
}

/// Convert the z14 layers to Overpass-style JSON. The z15 parser clips to the requested tile itself.
pub fn to_overpass(z: u8, x: u32, y: u32, buildings: &[Layer], transport: &[Layer]) -> String {
    to_overpass_with_base(z, x, y, buildings, transport, &[])
}

/// As `to_overpass`, plus ground cover from the base theme as `earthsim_area` elements (rings with holes).
pub fn to_overpass_with_base(z: u8, x: u32, y: u32, buildings: &[Layer], transport: &[Layer], base: &[Layer]) -> String {
    let mut els: Vec<Json> = Vec::new();
    let mut next_id: i64 = 1;
    for l in base.iter().filter(|l| matches!(l.name.as_str(), "water" | "land" | "land_use")) {
        for f in l.features.iter().filter(|f| f.kind == GeomType::Polygon) {
            let Some(tags) = area_tags(&l.name, f) else { continue };
            // MVT polygons: an outer ring (positive area) followed by its holes
            let mut polys: Vec<Vec<&Vec<(i32, i32)>>> = Vec::new();
            for r in f.parts.iter().filter(|r| r.len() >= 4) {
                if mvt::ring_area(r) > 0.0 {
                    polys.push(vec![r]);
                } else if let Some(p) = polys.last_mut() {
                    p.push(r);
                }
            }
            for p in polys {
                let rings: Vec<Json> = p
                    .iter()
                    .map(|r| geom_json(&r.iter().map(|&(px, py)| tile_to_latlon(z, x, y, l.extent, px, py)).collect::<Vec<_>>()))
                    .collect();
                els.push(Json::Obj(vec![
                    ("type".into(), Json::Str("earthsim_area".into())),
                    ("id".into(), Json::Num(next_id as f64)),
                    ("tags".into(), tags_json(&tags)),
                    ("rings".into(), Json::Arr(rings)),
                ]));
                next_id += 1;
            }
        }
    }
    for l in buildings.iter().filter(|l| l.name == "building") {
        for f in l.features.iter().filter(|f| f.kind == GeomType::Polygon) {
            if matches!(f.prop("is_underground"), Some(Value::Bool(true))) {
                continue;
            }
            let tags = building_tags(f);
            for ring in f.parts.iter().filter(|r| mvt::ring_area(r) > 0.0 && r.len() >= 4) {
                let ring = clip_ring(ring, l.extent as i32);
                if ring.len() < 4 || mvt::ring_area(&ring) < 1.0 {
                    continue;
                }
                let pts: Vec<(f64, f64)> = ring.iter().map(|&(px, py)| tile_to_latlon(z, x, y, l.extent, px, py)).collect();
                els.push(Json::Obj(vec![
                    ("type".into(), Json::Str("way".into())),
                    ("id".into(), Json::Num(next_id as f64)),
                    ("tags".into(), tags_json(&tags)),
                    ("geometry".into(), geom_json(&pts)),
                ]));
                next_id += 1;
            }
        }
    }
    for l in transport.iter().filter(|l| l.name == "segment") {
        for f in l.features.iter().filter(|f| f.kind == GeomType::Line) {
            let Some(tags) = road_tags(f) else { continue };
            for part in f.parts.iter().filter(|p| p.len() >= 2) {
                let pts: Vec<(f64, f64)> = part.iter().map(|&(px, py)| tile_to_latlon(z, x, y, l.extent, px, py)).collect();
                let nodes = Json::Arr(pts.iter().map(|p| Json::Num(point_id(*p) as f64)).collect());
                els.push(Json::Obj(vec![
                    ("type".into(), Json::Str("way".into())),
                    ("id".into(), Json::Num(next_id as f64)),
                    ("tags".into(), tags_json(&tags)),
                    ("nodes".into(), nodes),
                    ("geometry".into(), geom_json(&pts)),
                ]));
                next_id += 1;
            }
        }
    }
    Json::Obj(vec![("source".into(), Json::Str("overture".into())), ("elements".into(), Json::Arr(els))]).dump()
}

pub struct OvertureFetcher<S: RangeSource> {
    buildings: PmTiles<S>,
    transport: PmTiles<S>,
    /// land / water / land use (optional: without it there is no ground cover)
    base: Option<PmTiles<S>>,
    release: String,
    /// decoded z14 parents shared by their four z15 children
    memo: Mutex<Vec<((u32, u32), Arc<String>)>>,
}

impl<S: RangeSource> OvertureFetcher<S> {
    pub fn new(buildings: S, transport: S, release: &str) -> Result<Self, String> {
        Ok(OvertureFetcher { buildings: PmTiles::open(buildings)?, transport: PmTiles::open(transport)?, base: None, release: release.to_string(), memo: Mutex::new(Vec::new()) })
    }

    /// Also read ground cover from the base theme.
    pub fn with_base(mut self, base: S) -> Self {
        match PmTiles::open(base) {
            Ok(p) => self.base = Some(p),
            Err(e) => crate::log::log(&format!("Overture base theme not available ({e}): no water/parks/forests")),
        }
        self
    }

    pub fn parent_json(&self, px: u32, py: u32) -> Result<Arc<String>, String> {
        if let Some((_, j)) = self.memo.lock().unwrap().iter().find(|(k, _)| *k == (px, py)) {
            return Ok(j.clone());
        }
        let z = SOURCE_ZOOM;
        let dec = |pm: &PmTiles<S>| -> Result<Vec<Layer>, String> {
            match pm.get(z, px, py)? {
                Some(b) => mvt::decode(&b),
                None => Ok(Vec::new()),
            }
        };
        let b = dec(&self.buildings)?;
        let t = dec(&self.transport)?;
        // ground cover is optional: a failed base read must not lose the roads and buildings
        let g = match &self.base {
            Some(p) => dec(p).unwrap_or_else(|e| {
                crate::log::log(&format!("Overture base {px}/{py}: {e}"));
                Vec::new()
            }),
            None => Vec::new(),
        };
        let j = Arc::new(to_overpass_with_base(z, px, py, &b, &t, &g));
        let mut m = self.memo.lock().unwrap();
        if m.len() >= 8 {
            m.remove(0);
        }
        m.push(((px, py), j.clone()));
        Ok(j)
    }
}

impl<S: RangeSource> Fetcher for OvertureFetcher<S> {
    fn fetch(&self, tile: TileId, _query: &str) -> Result<String, FetchError> {
        let shift = tile.z.saturating_sub(SOURCE_ZOOM);
        self.parent_json(tile.x >> shift, tile.y >> shift).map(|j| (*j).clone()).map_err(FetchError::Transient)
    }
    fn describe(&self) -> String {
        format!("Overture {} ({})", self.release, self.buildings.src.describe())
    }
}

/// Lets tests and offline use count / share one archive.
impl<S: RangeSource> RangeSource for Arc<S> {
    fn read(&self, offset: u64, len: u64) -> Result<Vec<u8>, String> {
        (**self).read(offset, len)
    }
    fn describe(&self) -> String {
        (**self).describe()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mvt::tests::{field_bytes, packed_u32, vint, zz};
    use crate::osm::parse_overpass;

    fn str_val(s: &str) -> Vec<u8> {
        let mut v = Vec::new();
        field_bytes(&mut v, 1, s.as_bytes());
        v
    }
    fn dbl_val(x: f64) -> Vec<u8> {
        let mut v = Vec::new();
        vint(&mut v, 3 << 3 | 1);
        v.extend_from_slice(&x.to_le_bytes());
        v
    }

    fn layer(name: &str, kind: u64, geom: &[u32], keys: &[&str], vals: &[Vec<u8>]) -> Vec<u8> {
        let mut feat = Vec::new();
        let tags: Vec<u32> = (0..keys.len() as u32).flat_map(|i| [i, i]).collect();
        field_bytes(&mut feat, 2, &packed_u32(&tags));
        vint(&mut feat, 3 << 3);
        vint(&mut feat, kind);
        field_bytes(&mut feat, 4, &packed_u32(geom));
        let mut l = Vec::new();
        field_bytes(&mut l, 1, name.as_bytes());
        field_bytes(&mut l, 2, &feat);
        for k in keys {
            field_bytes(&mut l, 3, k.as_bytes());
        }
        for v in vals {
            field_bytes(&mut l, 4, v);
        }
        vint(&mut l, 5 << 3);
        vint(&mut l, 4096);
        let mut t = Vec::new();
        field_bytes(&mut t, 3, &l);
        t
    }

    #[test]
    fn rings_are_clipped_to_the_tile() {
        let sq = [(-100, -100), (200, -100), (200, 200), (-100, 200), (-100, -100)];
        let c = clip_ring(&sq, 4096);
        assert_eq!(c.first(), c.last());
        assert!(c.iter().all(|p| p.0 >= 0 && p.1 >= 0));
        assert!((mvt::ring_area(&c) - 200.0 * 200.0).abs() < 1.0);
        assert!(clip_ring(&[(-50, -50), (-10, -50), (-10, -10), (-50, -50)], 4096).is_empty());
    }

    #[test]
    fn release_from_listing() {
        let xml = "<ListBucketResult><CommonPrefixes><Prefix>tiles/2026-09-17.0/</Prefix></CommonPrefixes><CommonPrefixes><Prefix>tiles/2026-10-21.0/</Prefix></CommonPrefixes><CommonPrefixes><Prefix>tiles/latest/</Prefix></CommonPrefixes></ListBucketResult>";
        assert_eq!(newest_release(xml).as_deref(), Some("2026-10-21.0"));
        assert_eq!(newest_release("<x/>"), None);
    }

    #[test]
    fn overture_tile_becomes_parseable_overpass_json() {
        // a z14 tile over Minsk; a building in its NW quarter and a road crossing it
        let t14 = TileId::from_lat_lon(53.9045, 27.5615, 14);
        let child = TileId { z: 15, x: t14.x * 2, y: t14.y * 2 };
        let bgeom = [9, zz(400), zz(400), 3 << 3 | 2, zz(300), zz(0), zz(0), zz(300), zz(-300), zz(0), 15];
        let b = layer("building", 3, &bgeom, &["height", "num_floors", "facade_material", "roof_shape"], &[dbl_val(24.0), dbl_val(8.0), str_val("brick"), str_val("flat")]);
        let rgeom = [9, zz(0), zz(1000), 2 << 3 | 2, zz(1500), zz(0), zz(1500), zz(0)];
        let r = layer("segment", 2, &rgeom, &["subtype", "class"], &[str_val("road"), str_val("primary")]);
        let json = to_overpass(14, t14.x, t14.y, &mvt::decode(&b).unwrap(), &mvt::decode(&r).unwrap());
        let raw = parse_overpass(&json, child).unwrap();
        assert_eq!(raw.buildings.len(), 1);
        let tg = |k: &str| crate::stage_a::tag(&raw.buildings[0].tags, k).map(String::from);
        assert_eq!(tg("height").as_deref(), Some("24.00"));
        assert_eq!(tg("building:levels").as_deref(), Some("8"));
        assert_eq!(tg("building:material").as_deref(), Some("brick"));
        assert_eq!(tg("roof:shape").as_deref(), Some("flat"));
        assert_eq!(raw.roads.len(), 1);
        assert_eq!(raw.roads[0].cls, "primary");
        // the building's first vertex maps back inside the child tile
        let p = raw.buildings[0].ring[0];
        assert_eq!(TileId::from_lat_lon(p.0, p.1, 15), child);
    }

    #[test]
    fn base_theme_water_with_an_island_becomes_a_scene_area() {
        let t14 = TileId::from_lat_lon(53.9045, 27.5615, 14);
        let child = TileId { z: 15, x: t14.x * 2, y: t14.y * 2 };
        // outer ring (100,100)-(1900,1900), hole (800,800)-(1200,1200) wound the other way
        let g = [9, zz(100), zz(100), 3 << 3 | 2, zz(1800), zz(0), zz(0), zz(1800), zz(-1800), zz(0), 15, 9, zz(700), zz(-1100), 3 << 3 | 2, zz(0), zz(400), zz(400), zz(0), zz(0), zz(-400), 15];
        let w = layer("water", 3, &g, &["subtype"], &[str_val("lake")]);
        let park = layer("land_use", 3, &[9, zz(2200), zz(2200), 3 << 3 | 2, zz(500), zz(0), zz(0), zz(500), zz(-500), zz(0), 15], &["subtype", "class"], &[str_val("park"), str_val("park")]);
        let base: Vec<Layer> = [w, park].iter().flat_map(|l| mvt::decode(l).unwrap()).collect();
        let json = to_overpass_with_base(14, t14.x, t14.y, &[], &[], &base);
        let raw = parse_overpass(&json, child).unwrap();
        assert_eq!(raw.areas.len(), 1, "the park lies in another child tile");
        assert_eq!(raw.areas[0].kind, scene::model::AreaKind::Water);
        assert_eq!(raw.areas[0].rings.len(), 2);
        let frame = crate::geo::LocalFrame::new(53.9045, 27.5615, 0.0);
        let sc = crate::osm::build_scene(&raw, &frame);
        assert_eq!(sc.areas.len(), 1);
        let a = &sc.areas[0];
        let area: f64 = a.tris.iter().map(|t| {
            let (p, q, r) = (a.ring[t[0] as usize], a.ring[t[1] as usize], a.ring[t[2] as usize]);
            ((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])).abs() / 2.0
        }).sum();
        // z14 tile ~1440 m wide at this latitude: outer 1800/4096 of it squared minus the island
        let unit = 40_075_016.7 * 53.9045f64.to_radians().cos() / 16384.0 / 4096.0;
        let expect = ((1800.0 * unit).powi(2) - (400.0 * unit).powi(2)) as f64;
        assert!((area - expect).abs() < 0.03 * expect, "area {area:.0} m2, expected {expect:.0}");
        // the other child gets the park, with trees
        let raw2 = parse_overpass(&json, TileId { z: 15, x: t14.x * 2 + 1, y: t14.y * 2 + 1 }).unwrap();
        let sc2 = crate::osm::build_scene(&raw2, &frame);
        assert_eq!(sc2.areas.len(), 1);
        assert_eq!(sc2.areas[0].kind, scene::model::AreaKind::Park);
        assert!(!sc2.trees.is_empty());
    }

    #[test]
    fn non_roads_and_underground_are_dropped() {
        let rgeom = [9, zz(0), zz(10), 2 << 3 | 2, zz(100), zz(0), zz(100), zz(0)];
        let rail = layer("segment", 2, &rgeom, &["subtype", "class"], &[str_val("rail"), str_val("standard_gauge")]);
        let j = to_overpass(14, 9446, 5267, &[], &mvt::decode(&rail).unwrap());
        assert!(!j.contains("highway"));
    }
}
