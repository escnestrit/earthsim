//! OpenStreetMap (Overpass `out geom` JSON) -> per-tile raw data -> `scene::Scene` in a local frame.
//! Data © OpenStreetMap contributors, ODbL. Not handled yet: multipolygon relations, building:part,
//! bridge/tunnel layers (tunnels are skipped, bridges are drawn at ground level), elevation.
use crate::geo::LocalFrame;
use crate::stage_a;
use crate::tiles::{Bounds, TileId};
use scene::json::Json;
use scene::model::Meta;
use scene::{Road, Scene};
use std::collections::HashMap;

/// Drivable highway classes and their default carriageway width in metres (an assumption, `lanes` overrides it).
pub const DRIVABLE: &[(&str, f64)] = &[
    ("motorway", 14.0), ("trunk", 12.0), ("primary", 11.0), ("secondary", 9.0), ("tertiary", 8.0),
    ("unclassified", 6.5), ("residential", 6.5), ("living_street", 5.0), ("service", 4.0),
    ("motorway_link", 7.0), ("trunk_link", 7.0), ("primary_link", 7.0), ("secondary_link", 6.5), ("tertiary_link", 6.0),
];

/// Extra margin (m) around the tile kept when clipping road polylines.
const CLIP_MARGIN_M: f64 = 80.0;

#[derive(Clone, Debug)]
pub struct RawRoad {
    pub cls: String,
    pub width: f64,
    pub width_from_tag: bool,
    pub name: String,
    /// (lat, lon)
    pub pts: Vec<(f64, f64)>,
    pub junction: Vec<bool>,
}

#[derive(Clone, Debug)]
pub struct RawBuilding {
    pub id: i64,
    pub tags: Vec<(String, String)>,
    /// (lat, lon), closed ring (first == last)
    pub ring: Vec<(f64, f64)>,
}

#[derive(Clone, Debug)]
pub struct TileRaw {
    pub tile: TileId,
    pub roads: Vec<RawRoad>,
    pub buildings: Vec<RawBuilding>,
    pub trees: Vec<(f64, f64)>,
    /// ground cover clipped to the tile
    pub areas: Vec<crate::areas::RawArea>,
    pub elements: usize,
}

pub fn overpass_query(b: &Bounds) -> String {
    let hw: Vec<&str> = DRIVABLE.iter().map(|d| d.0).collect();
    let bb = format!("{:.6},{:.6},{:.6},{:.6}", b.south, b.west, b.north, b.east);
    format!(
        "[out:json][timeout:30];(way[\"highway\"~\"^({})$\"]({bb});way[\"building\"]({bb});node[\"natural\"=\"tree\"]({bb});\
         way[\"natural\"~\"^(water|wood|scrub|heath|grassland|wetland|sand|beach)$\"]({bb});\
         way[\"landuse\"~\"^(forest|grass|meadow|village_green|recreation_ground|cemetery|reservoir|basin|allotments|orchard)$\"]({bb});\
         way[\"leisure\"~\"^(park|garden|pitch|playground|golf_course|dog_park)$\"]({bb});way[\"waterway\"=\"riverbank\"]({bb});way[\"amenity\"=\"parking\"]({bb}););out geom;",
        hw.join("|")
    )
}

fn tags_of(j: &Json) -> Vec<(String, String)> {
    match j.get("tags") {
        Some(Json::Obj(o)) => o.iter().filter_map(|(k, v)| v.as_str().map(|s| (k.clone(), s.to_string()))).collect(),
        _ => Vec::new(),
    }
}

fn latlon_list(j: &Json) -> Vec<(f64, f64)> {
    j.get("geometry")
        .and_then(|g| g.as_arr())
        .map(|a| a.iter().filter_map(|p| Some((p.get("lat")?.as_f64()?, p.get("lon")?.as_f64()?))).collect())
        .unwrap_or_default()
}

fn valid_latlon(p: (f64, f64)) -> bool {
    p.0.is_finite() && p.1.is_finite() && p.0.abs() <= 90.0 && p.1.abs() <= 180.0
}

/// Runs of consecutive segments that touch `b` (inclusive point ranges).
fn clip_runs(pts: &[(f64, f64)], b: &Bounds) -> Vec<(usize, usize)> {
    let mut runs: Vec<(usize, usize)> = Vec::new();
    for i in 0..pts.len().saturating_sub(1) {
        let (a, c) = (pts[i], pts[i + 1]);
        let hit = b.intersects_box(a.0.min(c.0), a.1.min(c.1), a.0.max(c.0), a.1.max(c.1));
        if hit {
            match runs.last_mut() {
                Some(r) if r.1 == i => r.1 = i + 1,
                _ => runs.push((i, i + 1)),
            }
        }
    }
    runs
}

/// Parse an Overpass response for `tile`. Buildings are kept only if their centroid lies in the tile (so each
/// building belongs to exactly one tile); trees likewise; road polylines are clipped to the tile plus a margin.
pub fn parse_overpass(text: &str, tile: TileId) -> Result<TileRaw, String> {
    let j = Json::parse(text).map_err(|e| e.to_string())?;
    if let Some(r) = j.get("remark").and_then(|r| r.as_str()) {
        let low = r.to_lowercase();
        if low.contains("error") || low.contains("timed out") || low.contains("out of memory") {
            return Err(format!("overpass remark: {r}"));
        }
    }
    let elements = j.get("elements").and_then(|e| e.as_arr()).ok_or("response has no 'elements'")?;
    let b = tile.bounds();
    let dlat = CLIP_MARGIN_M / 111_320.0;
    let dlon = CLIP_MARGIN_M / (111_320.0 * ((b.south + b.north) / 2.0).to_radians().cos().max(0.01));
    let eb = Bounds { south: b.south - dlat, west: b.west - dlon, north: b.north + dlat, east: b.east + dlon };

    struct Pending {
        cls: String,
        width: f64,
        from_tag: bool,
        name: String,
        pts: Vec<(f64, f64)>,
        nodes: Vec<i64>,
    }
    let mut pend: Vec<Pending> = Vec::new();
    let mut node_use: HashMap<i64, u32> = HashMap::new();
    let mut out = TileRaw { tile, roads: Vec::new(), buildings: Vec::new(), trees: Vec::new(), areas: Vec::new(), elements: elements.len() };

    for el in elements {
        let ty = el.get("type").and_then(|t| t.as_str()).unwrap_or("");
        let tags = tags_of(el);
        let tg = |k: &str| stage_a::tag(&tags, k);
        match ty {
            "node" => {
                if tg("natural") == Some("tree") {
                    if let (Some(la), Some(lo)) = (el.get("lat").and_then(|v| v.as_f64()), el.get("lon").and_then(|v| v.as_f64())) {
                        if valid_latlon((la, lo)) && TileId::from_lat_lon(la, lo, tile.z) == tile {
                            out.trees.push((la, lo));
                        }
                    }
                }
            }
            "way" => {
                let pts = latlon_list(el);
                if pts.iter().any(|p| !valid_latlon(*p)) {
                    continue;
                }
                if let Some(hw) = tg("highway") {
                    let Some(&(_, dw)) = DRIVABLE.iter().find(|d| d.0 == hw) else { continue };
                    if matches!(tg("tunnel"), Some("yes") | Some("building_passage") | Some("culvert")) || tg("area") == Some("yes") || pts.len() < 2 {
                        continue;
                    }
                    let (width, from_tag) = match tg("lanes").and_then(|l| l.trim().parse::<u32>().ok()) {
                        Some(n) if (1..=12).contains(&n) => (n as f64 * 3.3, true),
                        _ => (dw, false),
                    };
                    let nodes: Vec<i64> = el.get("nodes").and_then(|n| n.as_arr()).map(|a| a.iter().filter_map(|v| v.as_f64().map(|f| f as i64)).collect()).unwrap_or_default();
                    for n in &nodes {
                        *node_use.entry(*n).or_default() += 1;
                    }
                    pend.push(Pending { cls: hw.to_string(), width, from_tag, name: tg("name").unwrap_or("").to_string(), pts, nodes });
                } else if let Some(kind) = crate::areas::area_kind(&tags) {
                    // closed ways only (multipolygon relations come through Overture, not Overpass)
                    if pts.len() >= 4 && pts.first() == pts.last() {
                        let r = crate::areas::clip_to_bounds(&pts, &b);
                        if r.len() >= 3 {
                            out.areas.push(crate::areas::RawArea { kind, rings: vec![r] });
                        }
                    }
                } else if tg("building").is_some() {
                    if pts.len() < 4 || pts.first() != pts.last() {
                        continue;
                    }
                    let m = pts.len() - 1;
                    let (cla, clo) = (pts[..m].iter().map(|p| p.0).sum::<f64>() / m as f64, pts[..m].iter().map(|p| p.1).sum::<f64>() / m as f64);
                    if TileId::from_lat_lon(cla, clo, tile.z) != tile {
                        continue;
                    }
                    let id = el.get("id").and_then(|v| v.as_f64()).unwrap_or(0.0) as i64;
                    out.buildings.push(RawBuilding { id, tags, ring: pts });
                }
            }
            // Overture polygons with holes (see overture::to_overpass)
            "earthsim_area" => {
                let Some(kind) = crate::areas::area_kind(&tags) else { continue };
                let rings: Vec<Vec<(f64, f64)>> = el
                    .get("rings")
                    .and_then(|r| r.as_arr())
                    .unwrap_or(&[])
                    .iter()
                    .map(|r| r.as_arr().unwrap_or(&[]).iter().filter_map(|p| Some((p.get("lat")?.as_f64()?, p.get("lon")?.as_f64()?))).collect::<Vec<_>>())
                    .map(|r: Vec<(f64, f64)>| crate::areas::clip_to_bounds(&r, &b))
                    .collect();
                if rings.first().is_some_and(|r| r.len() >= 3) {
                    out.areas.push(crate::areas::RawArea { kind, rings: rings.into_iter().filter(|r| r.len() >= 3).collect() });
                }
            }
            _ => {}
        }
    }

    for p in pend {
        let junction_full: Vec<bool> = if p.nodes.len() == p.pts.len() {
            p.nodes.iter().map(|n| node_use.get(n).copied().unwrap_or(0) >= 2).collect()
        } else {
            vec![false; p.pts.len()]
        };
        for (s, e) in clip_runs(&p.pts, &eb) {
            out.roads.push(RawRoad {
                cls: p.cls.clone(),
                width: p.width,
                width_from_tag: p.from_tag,
                name: p.name.clone(),
                pts: p.pts[s..=e].to_vec(),
                junction: junction_full[s..=e].to_vec(),
            });
        }
    }
    Ok(out)
}

fn seg_dist(x: f64, y: f64, a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 { (((x - a[0]) * dx + (y - a[1]) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    (x - a[0] - t * dx).hypot(y - a[1] - t * dy)
}

/// Convert raw tile data to a scene in `frame` (east/north metres). Cheap enough to redo after a re-base.
pub fn build_scene(raw: &TileRaw, frame: &LocalFrame) -> Scene {
    let mut sc = Scene::default();
    let loc = |la: f64, lo: f64| {
        let p = frame.to_local(la, lo, 0.0);
        [(p[0] * 100.0).round() / 100.0, (p[1] * 100.0).round() / 100.0]
    };
    for r in &raw.roads {
        sc.roads.push(Road { cls: r.cls.clone(), width: r.width, pts: r.pts.iter().map(|p| loc(p.0, p.1)).collect(), junction: r.junction.clone() });
    }
    let mut status: HashMap<String, usize> = HashMap::new();
    let mut origins: HashMap<String, usize> = HashMap::new();
    for b in &raw.buildings {
        if let Some(bb) = stage_a::build(&format!("osm:way/{}", b.id), &b.ring, &b.tags, frame) {
            *status.entry(bb.building.status.clone()).or_default() += 1;
            *origins.entry(bb.building.height_origin.clone()).or_default() += 1;
            sc.buildings.push(bb.building);
        }
    }
    for t in &raw.trees {
        sc.trees.push(loc(t.0, t.1));
    }
    // ground cover, and trees in forests and parks kept off the roads
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, r) in sc.roads.iter().enumerate() {
        for p in &r.pts {
            grid.entry(((p[0] / 25.0).floor() as i64, (p[1] / 25.0).floor() as i64)).or_default().push(i);
        }
    }
    let roads = &sc.roads;
    let on_road = |x: f64, y: f64| {
        let (ci, cj) = ((x / 25.0).floor() as i64, (y / 25.0).floor() as i64);
        (ci - 1..=ci + 1).any(|i| {
            (cj - 1..=cj + 1).any(|j| {
                grid.get(&(i, j)).is_some_and(|v| {
                    v.iter().any(|&k| {
                        let r = &roads[k];
                        r.pts.windows(2).any(|w| seg_dist(x, y, w[0], w[1]) < r.width / 2.0 + 2.5)
                    })
                })
            })
        })
    };
    let (areas, area_trees) = crate::areas::build_areas(&raw.areas, frame, &on_road);
    sc.areas = areas;
    sc.trees.extend(area_trees);
    let mut sv: Vec<_> = status.into_iter().collect();
    sv.sort();
    let mut ov: Vec<_> = origins.into_iter().collect();
    ov.sort();
    sc.meta = Meta {
        source: Some("OpenStreetMap via Overpass (ODbL)".into()),
        terrain: "FLAT (no DEM yet)".into(),
        n_roads: sc.roads.len(),
        n_buildings: sc.buildings.len(),
        n_trees: sc.trees.len(),
        status_counts: sv,
        height_origin_counts: ov,
    };
    sc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::LocalFrame;

    pub const DEMO: &str = include_str!("../../../data/demo_overpass.json");
    const MINSK: (f64, f64) = (53.9045, 27.5615);

    fn demo_tiles() -> Vec<TileId> {
        TileId::around(MINSK.0, MINSK.1, 1100.0, 15)
    }

    #[test]
    fn every_building_and_tree_belongs_to_exactly_one_tile() {
        let (mut nb, mut nt, mut nr) = (0, 0, 0);
        for t in demo_tiles() {
            let raw = parse_overpass(DEMO, t).unwrap();
            nb += raw.buildings.len();
            nt += raw.trees.len();
            nr += raw.roads.len();
        }
        assert_eq!(nb, 363, "buildings");
        assert_eq!(nt, 1215, "trees");
        assert!(nr >= 23, "roads {nr}");
    }

    #[test]
    fn built_scene_matches_python_converter_counts_and_sizes() {
        let fr = LocalFrame::new(MINSK.0, MINSK.1, 0.0);
        let mut total = (0, 0, 0);
        let mut sample_h = None;
        for t in demo_tiles() {
            let raw = parse_overpass(DEMO, t).unwrap();
            let sc = build_scene(&raw, &fr);
            total.0 += sc.buildings.len();
            total.1 += sc.trees.len();
            total.2 += sc.roads.len();
            assert_eq!(sc.meta.n_buildings, sc.buildings.len());
            for b in &sc.buildings {
                assert!(b.height > 2.0 && b.height < 60.0);
                assert!(b.indices.iter().all(|&i| (i as usize) * 3 + 2 < b.verts.len()));
                sample_h.get_or_insert(b.height);
            }
        }
        assert_eq!((total.0, total.1), (363, 1215));
        assert!(sample_h.is_some());
    }

    #[test]
    fn remark_errors_and_malformed_json_are_errors_not_panics() {
        let t = TileId::from_lat_lon(MINSK.0, MINSK.1, 15);
        assert!(parse_overpass(r#"{"elements":[],"remark":"runtime error: Query timed out in \"query\" at line 1"}"#, t).is_err());
        assert!(parse_overpass("<html>Too Many Requests</html>", t).is_err());
        assert!(parse_overpass("", t).is_err());
        assert!(parse_overpass(r#"{"version":0.6}"#, t).is_err());
        let ok = parse_overpass(r#"{"elements":[]}"#, t).unwrap();
        assert_eq!((ok.roads.len(), ok.buildings.len(), ok.trees.len()), (0, 0, 0));
    }

    #[test]
    fn junk_elements_are_skipped() {
        let t = TileId::from_lat_lon(MINSK.0, MINSK.1, 15);
        let (la, lo) = t.center();
        let js = format!(
            r#"{{"elements":[
            {{"type":"way","id":1,"tags":{{"highway":"residential"}}}},
            {{"type":"way","id":2,"tags":{{"highway":"footway"}},"geometry":[{{"lat":{la},"lon":{lo}}},{{"lat":{la},"lon":{lo}}}]}},
            {{"type":"way","id":3,"tags":{{"building":"yes"}},"geometry":[{{"lat":{la},"lon":{lo}}},{{"lat":{la},"lon":{lo}}},{{"lat":{la},"lon":{lo}}}]}},
            {{"type":"way","id":4,"tags":{{"highway":"primary"}},"geometry":[{{"lat":999,"lon":{lo}}},{{"lat":{la},"lon":{lo}}}]}},
            {{"type":"node","id":5,"lat":"x","lon":1,"tags":{{"natural":"tree"}}}},
            {{"type":"relation","id":6}},
            {{"nothing":true}}]}}"#
        );
        let raw = parse_overpass(&js, t).unwrap();
        assert_eq!((raw.roads.len(), raw.buildings.len(), raw.trees.len()), (0, 0, 0));
    }

    #[test]
    fn tunnels_are_skipped_lanes_set_width_and_junctions_are_found() {
        let t = TileId::from_lat_lon(MINSK.0, MINSK.1, 15);
        let (la, lo) = t.center();
        let p = |dx: f64, dy: f64| format!(r#"{{"lat":{},"lon":{}}}"#, la + dy / 111320.0, lo + dx / 66000.0);
        let js = format!(
            r#"{{"elements":[
            {{"type":"way","id":1,"nodes":[1,2,3],"tags":{{"highway":"primary","lanes":"4","name":"A"}},"geometry":[{},{},{}]}},
            {{"type":"way","id":2,"nodes":[3,4],"tags":{{"highway":"residential"}},"geometry":[{},{}]}},
            {{"type":"way","id":3,"nodes":[5,6],"tags":{{"highway":"primary","tunnel":"yes"}},"geometry":[{},{}]}}]}}"#,
            p(-100.0, 0.0), p(0.0, 0.0), p(100.0, 0.0), p(100.0, 0.0), p(100.0, 100.0), p(0.0, 50.0), p(10.0, 50.0)
        );
        let raw = parse_overpass(&js, t).unwrap();
        assert_eq!(raw.roads.len(), 2, "tunnel must be skipped");
        let a = &raw.roads[0];
        assert!((a.width - 13.2).abs() < 1e-9 && a.width_from_tag && a.name == "A");
        assert_eq!(a.junction, vec![false, false, true], "node 3 is shared by two ways");
        assert!(!raw.roads[1].width_from_tag);
    }

    #[test]
    fn long_roads_are_clipped_to_the_tile() {
        let t = TileId::from_lat_lon(MINSK.0, MINSK.1, 15);
        let (la, lo) = t.center();
        // a 20 km straight road with a vertex every 500 m passing through the tile
        let pts: Vec<String> = (-20..=20).map(|i| format!(r#"{{"lat":{la},"lon":{}}}"#, lo + i as f64 * 500.0 / 66000.0)).collect();
        let js = format!(r#"{{"elements":[{{"type":"way","id":1,"tags":{{"highway":"trunk"}},"geometry":[{}]}}]}}"#, pts.join(","));
        let raw = parse_overpass(&js, t).unwrap();
        assert_eq!(raw.roads.len(), 1);
        let n = raw.roads[0].pts.len();
        assert!(n >= 2 && n <= 6, "clipped road should keep only nearby vertices, kept {n}");
    }

    #[test]
    fn query_mentions_bbox_and_classes() {
        let b = TileId::from_lat_lon(MINSK.0, MINSK.1, 15).bounds();
        let q = overpass_query(&b);
        assert!(q.contains("out geom") && q.contains("residential") && q.contains("natural") && q.contains("building"));
        assert!(q.contains(&format!("{:.6}", b.south)));
    }
}
