//! Overture footprints + OpenStreetMap heights. Overture's building shapes are good, but its storey counts lag
//! OSM by a release (or are missing for footprints that came from machine-learning datasets), so for every
//! tile a small Overpass query fetches just the tags of OSM buildings that state a height or a level count,
//! with one centre point each (`out tags center`), and those values are copied onto the Overture building
//! that contains the point. OSM wins for height and levels; other attributes only fill gaps.
//! Data © OpenStreetMap contributors (ODbL), © Overture Maps Foundation.
use crate::fetch::{FetchError, Fetcher};
use crate::tiles::{Bounds, TileId};
use scene::json::Json;
use std::sync::Arc;

/// Marker on tiles whose OSM height lookup failed: the cache treats them as stale and refetches when it can.
pub const HEIGHTS_FAILED: &str = "\"earthsim_osm_heights\":\"failed\"";

/// Tags taken from OSM even when Overture has its own value.
const OVERRIDE: &[&str] = &["height", "building:levels", "min_height", "building:min_level", "roof:levels"];
/// Tags taken from OSM only when Overture has none.
const FILL: &[&str] = &["roof:shape", "roof:height", "building:colour", "roof:colour", "building:material", "roof:material", "building:series", "start_date", "building:architecture", "amenity", "religion", "denomination", "shop", "office", "tower:type", "tourism", "historic", "man_made", "name", "building:use", "leisure"];

pub fn heights_query(b: &Bounds) -> String {
    let bb = format!("{:.6},{:.6},{:.6},{:.6}", b.south, b.west, b.north, b.east);
    format!(
        "[out:json][timeout:25];(way[\"building\"][\"building:levels\"]({bb});way[\"building\"][\"height\"]({bb});\
         relation[\"building\"][\"building:levels\"]({bb});relation[\"building\"][\"height\"]({bb});\
         way[\"building\"][~\"^(amenity|shop|office|tourism|historic|man_made|name|building:use)$\"~\".\"]({bb});way[\"building\"~\"^(church|cathedral|chapel|mosque|temple|synagogue|school|university|college|kindergarten|hospital|office|retail|commercial|supermarket|garages|garage|civic|public|government|train_station)$\"]({bb}););out tags center;\
         node[\"entrance\"]({bb});out body;"
    )
}

fn ring_of(el: &Json) -> Vec<(f64, f64)> {
    el.get("geometry").and_then(|g| g.as_arr()).map(|a| a.iter().filter_map(|p| Some((p.get("lat")?.as_f64()?, p.get("lon")?.as_f64()?))).collect()).unwrap_or_default()
}

fn contains(r: &[(f64, f64)], p: (f64, f64)) -> bool {
    let mut inside = false;
    let n = r.len();
    for i in 0..n {
        let (a, b) = (r[i], r[(i + 1) % n]);
        if (a.0 > p.0) != (b.0 > p.0) && p.1 < a.1 + (p.0 - a.0) / (b.0 - a.0) * (b.1 - a.1) {
            inside = !inside;
        }
    }
    inside
}

fn area(r: &[(f64, f64)]) -> f64 {
    let n = r.len();
    (0..n).map(|i| r[i].1 * r[(i + 1) % n].0 - r[(i + 1) % n].1 * r[i].0).sum::<f64>().abs() / 2.0
}

/// Copy OSM height/level tags onto the Overture buildings in `overture_json`. Returns (json, buildings changed).
pub fn merge_osm_heights(overture_json: &str, osm_json: &str) -> Result<(String, usize), String> {
    let mut j = Json::parse(overture_json).map_err(|e| e.to_string())?;
    let osm = Json::parse(osm_json).map_err(|e| e.to_string())?;
    let osm_els = osm.get("elements").and_then(|e| e.as_arr()).ok_or("OSM response has no elements")?;
    // OSM centre points with their tags
    let pts: Vec<((f64, f64), Vec<(String, String)>)> = osm_els
        .iter()
        .filter_map(|e| {
            let c = e.get("center")?;
            let p = (c.get("lat")?.as_f64()?, c.get("lon")?.as_f64()?);
            let tags = match e.get("tags") {
                Some(Json::Obj(o)) => o.iter().filter_map(|(k, v)| v.as_str().map(|s| (k.clone(), s.to_string()))).collect(),
                _ => Vec::new(),
            };
            Some((p, tags))
        })
        .collect();
    let Json::Obj(top) = &mut j else { return Err("not an object".into()) };
    let Some((_, Json::Arr(els))) = top.iter_mut().find(|(k, _)| k == "elements") else { return Err("no elements".into()) };
    // Overture buildings: index, ring, bbox
    let rings: Vec<(usize, Vec<(f64, f64)>, [f64; 4])> = els
        .iter()
        .enumerate()
        .filter(|(_, e)| e.get("tags").and_then(|t| t.get("building")).is_some())
        .map(|(i, e)| {
            let r = ring_of(e);
            let bb = r.iter().fold([f64::MAX, f64::MAX, f64::MIN, f64::MIN], |b, p| [b[0].min(p.0), b[1].min(p.1), b[2].max(p.0), b[3].max(p.1)]);
            (i, r, bb)
        })
        .filter(|(_, r, _)| r.len() >= 3)
        .collect();
    let mut changed = 0;
    for (p, tags) in &pts {
        // the smallest Overture outline around the OSM centre
        let best = rings
            .iter()
            .filter(|(_, r, bb)| p.0 >= bb[0] && p.0 <= bb[2] && p.1 >= bb[1] && p.1 <= bb[3] && contains(r, *p))
            .min_by(|a, b| area(&a.1).total_cmp(&area(&b.1)));
        let Some((i, _, _)) = best else { continue };
        let Some(Json::Obj(t)) = (match &mut els[*i] {
            Json::Obj(o) => o.iter_mut().find(|(k, _)| k == "tags").map(|(_, v)| v),
            _ => None,
        }) else {
            continue;
        };
        let mut did = false;
        // the OSM building class when Overture only knows "a building"
        if let Some((_, b)) = tags.iter().find(|(k, _)| k == "building") {
            if b != "yes" {
                if let Some((_, old)) = t.iter_mut().find(|(k, _)| k == "building") {
                    if old.as_str() == Some("yes") {
                        *old = Json::Str(b.clone());
                        did = true;
                    }
                }
            }
        }
        for (k, v) in tags {
            let over = OVERRIDE.contains(&k.as_str());
            if !over && !FILL.contains(&k.as_str()) {
                continue;
            }
            match t.iter_mut().find(|(kk, _)| kk == k) {
                Some((_, old)) if over => {
                    if old.as_str() != Some(v.as_str()) {
                        *old = Json::Str(v.clone());
                        did = true;
                    }
                }
                Some(_) => {}
                None => {
                    t.push((k.clone(), Json::Str(v.clone())));
                    did = true;
                }
            }
        }
        // an OSM level count beats an Overture height that may have been estimated from the footprint
        if tags.iter().any(|(k, _)| k == "building:levels") && !tags.iter().any(|(k, _)| k == "height") {
            t.retain(|(k, _)| k != "height");
        }
        if did {
            changed += 1;
        }
    }
    // entrances (doors) exist only in OSM: add them as nodes
    for e in osm_els.iter().filter(|e| e.get("type").and_then(|t| t.as_str()) == Some("node") && e.get("tags").and_then(|t| t.get("entrance")).is_some()) {
        els.push(e.clone());
    }
    top.insert(0, ("earthsim_osm_heights".into(), Json::Num(changed as f64)));
    Ok((j.dump(), changed))
}

/// Overture (or whatever `base` is) with OSM heights on top, via `osm` (an Overpass fetcher).
pub struct EnrichFetcher {
    pub base: Arc<dyn Fetcher>,
    pub osm: Arc<dyn Fetcher>,
}

impl Fetcher for EnrichFetcher {
    fn fetch(&self, tile: TileId, query: &str) -> Result<String, FetchError> {
        let text = self.base.fetch(tile, query)?;
        // only Overture output needs it (a plain Overpass answer already is OSM)
        if !text.get(..200).unwrap_or(&text).contains("\"source\":\"overture\"") {
            return Ok(text);
        }
        let failed = |why: String| {
            crate::log::log(&format!("  OSM heights for {}/{}/{}: {why}", tile.z, tile.x, tile.y));
            match text.trim_start().strip_prefix('{') {
                Some(rest) => format!("{{{HEIGHTS_FAILED},{rest}"),
                None => text.clone(),
            }
        };
        match self.osm.fetch(tile, &heights_query(&tile.bounds())) {
            Ok(o) => match merge_osm_heights(&text, &o) {
                Ok((j, n)) => {
                    if n > 0 {
                        crate::log::log(&format!("  OSM heights: {n} buildings corrected in {}/{}/{}", tile.z, tile.x, tile.y));
                    }
                    Ok(j)
                }
                Err(e) => Ok(failed(e)),
            },
            Err(e) => Ok(failed(e.to_string())),
        }
    }
    fn describe(&self) -> String {
        format!("{} + OSM heights", self.base.describe())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OVERTURE: &str = r#"{"source":"overture","elements":[
        {"type":"way","id":1,"tags":{"building":"yes","building:levels":"5","height":"15.00"},
         "geometry":[{"lat":0.0,"lon":0.0},{"lat":0.0,"lon":0.001},{"lat":0.001,"lon":0.001},{"lat":0.001,"lon":0.0},{"lat":0.0,"lon":0.0}]},
        {"type":"way","id":2,"tags":{"building":"yes"},
         "geometry":[{"lat":0.002,"lon":0.0},{"lat":0.002,"lon":0.001},{"lat":0.003,"lon":0.001},{"lat":0.003,"lon":0.0},{"lat":0.002,"lon":0.0}]},
        {"type":"way","id":3,"tags":{"highway":"primary"},"geometry":[{"lat":0.0,"lon":0.0},{"lat":0.003,"lon":0.0}]}]}"#;

    #[test]
    fn osm_levels_override_and_fill() {
        let osm = r#"{"elements":[
            {"type":"way","id":10,"center":{"lat":0.0005,"lon":0.0005},"tags":{"building":"yes","building:levels":"9","roof:shape":"flat"}},
            {"type":"way","id":11,"center":{"lat":0.0025,"lon":0.0005},"tags":{"building":"yes","height":"40","building:levels":"12"}},
            {"type":"way","id":12,"center":{"lat":0.5,"lon":0.5},"tags":{"building":"yes","building:levels":"3"}},
            {"type":"node","id":20,"lat":0.0001,"lon":0.0005,"tags":{"entrance":"staircase","ref":"1"}}]}"#;
        let (j, n) = merge_osm_heights(OVERTURE, osm).unwrap();
        assert_eq!(n, 2);
        let raw = crate::osm::parse_overpass(&j, TileId::from_lat_lon(0.0005, 0.0005, 15)).unwrap();
        let tag = |i: usize, k: &str| crate::stage_a::tag(&raw.buildings[i].tags, k).map(String::from);
        // building 1: OSM says 9 levels; Overture's 15 m height (5 storeys) is dropped so the levels decide
        assert_eq!(tag(0, "building:levels").as_deref(), Some("9"));
        assert_eq!(tag(0, "height"), None);
        assert_eq!(tag(0, "roof:shape").as_deref(), Some("flat"));
        assert_eq!(raw.entrances.len(), 1, "entrance nodes from OSM reach the tile");
        let (h, _) = crate::stage_a::estimate_height(&raw.buildings[0].tags);
        assert!(h > 25.0, "9 storeys, got {h} m");
    }

    #[test]
    fn non_overture_text_and_failures() {
        struct Fixed(Result<String, FetchError>);
        impl Fetcher for Fixed {
            fn fetch(&self, _: TileId, _: &str) -> Result<String, FetchError> {
                match &self.0 {
                    Ok(s) => Ok(s.clone()),
                    Err(FetchError::Transient(e)) => Err(FetchError::Transient(e.clone())),
                    Err(FetchError::Permanent(e)) => Err(FetchError::Permanent(e.clone())),
                }
            }
            fn describe(&self) -> String {
                "fixed".into()
            }
        }
        let t = TileId::from_lat_lon(0.0005, 0.0005, 15);
        let down = Arc::new(Fixed(Err(FetchError::Transient("timeout".into()))));
        let f = EnrichFetcher { base: Arc::new(Fixed(Ok(OVERTURE.into()))), osm: down.clone() };
        let out = f.fetch(t, "").unwrap();
        assert!(out.starts_with(&format!("{{{HEIGHTS_FAILED}")), "{}", &out[..80]);
        assert!(crate::osm::parse_overpass(&out, t).is_ok());
        // plain Overpass answers pass through untouched
        let plain = r#"{"elements":[]}"#;
        let f = EnrichFetcher { base: Arc::new(Fixed(Ok(plain.into()))), osm: down };
        assert_eq!(f.fetch(t, "").unwrap(), plain);
        let q = heights_query(&t.bounds());
        // ways: tags + a centre point; entrance nodes need their coordinates ("out tags" would drop them)
        assert!(q.contains("out tags center") && q.contains("node[\"entrance\"]") && q.ends_with("out body;"), "{q}");
    }
}
