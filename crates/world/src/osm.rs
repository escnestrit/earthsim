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
    /// a `building:part` (drawn instead of the outline it lies in)
    pub part: bool,
}

/// An OSM entrance node (`entrance=*`): a door of a building.
#[derive(Clone, Debug)]
pub struct RawEntrance {
    pub lat: f64,
    pub lon: f64,
    pub kind: String,
}

#[derive(Clone, Debug)]
pub struct TileRaw {
    pub tile: TileId,
    pub roads: Vec<RawRoad>,
    pub buildings: Vec<RawBuilding>,
    pub trees: Vec<(f64, f64)>,
    /// ground cover clipped to the tile
    pub areas: Vec<crate::areas::RawArea>,
    pub entrances: Vec<RawEntrance>,
    pub elements: usize,
}

pub fn overpass_query(b: &Bounds) -> String {
    let hw: Vec<&str> = DRIVABLE.iter().map(|d| d.0).collect();
    let bb = format!("{:.6},{:.6},{:.6},{:.6}", b.south, b.west, b.north, b.east);
    format!(
        "[out:json][timeout:30];(way[\"highway\"~\"^({})$\"]({bb});way[\"building\"]({bb});node[\"natural\"=\"tree\"]({bb});\
         way[\"natural\"~\"^(water|wood|scrub|heath|grassland|wetland|sand|beach)$\"]({bb});\
         way[\"landuse\"~\"^(forest|grass|meadow|village_green|recreation_ground|cemetery|reservoir|basin|allotments|orchard)$\"]({bb});\
         way[\"leisure\"~\"^(park|garden|pitch|playground|golf_course|dog_park)$\"]({bb});way[\"waterway\"=\"riverbank\"]({bb});way[\"amenity\"=\"parking\"]({bb});way[\"building:part\"]({bb});node[\"entrance\"]({bb}););out geom;",
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
    let mut out = TileRaw { tile, roads: Vec::new(), buildings: Vec::new(), trees: Vec::new(), areas: Vec::new(), entrances: Vec::new(), elements: elements.len() };

    for el in elements {
        let ty = el.get("type").and_then(|t| t.as_str()).unwrap_or("");
        let tags = tags_of(el);
        let tg = |k: &str| stage_a::tag(&tags, k);
        match ty {
            "node" => {
                if let Some(kind) = tg("entrance") {
                    if let (Some(la), Some(lo)) = (el.get("lat").and_then(|v| v.as_f64()), el.get("lon").and_then(|v| v.as_f64())) {
                        if valid_latlon((la, lo)) && TileId::from_lat_lon(la, lo, tile.z) == tile {
                            out.entrances.push(RawEntrance { lat: la, lon: lo, kind: kind.to_string() });
                        }
                    }
                }
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
                } else if tg("building").is_some_and(|b| b != "no") || tg("building:part").is_some_and(|b| b != "no") {
                    if pts.len() < 4 || pts.first() != pts.last() {
                        continue;
                    }
                    let m = pts.len() - 1;
                    let (cla, clo) = (pts[..m].iter().map(|p| p.0).sum::<f64>() / m as f64, pts[..m].iter().map(|p| p.1).sum::<f64>() / m as f64);
                    if TileId::from_lat_lon(cla, clo, tile.z) != tile {
                        continue;
                    }
                    let id = el.get("id").and_then(|v| v.as_f64()).unwrap_or(0.0) as i64;
                    let part = tg("building:part").is_some_and(|b| b != "no");
                    out.buildings.push(RawBuilding { id, tags, ring: pts, part });
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

    // outlines whose parts cover most of them are drawn as the parts (OSM Simple 3D Buildings)
    let ll_area = |r: &[(f64, f64)]| stage_a::signed_area(r).abs();
    let centroid = |r: &[(f64, f64)]| {
        let m = (r.len() - 1).max(1) as f64;
        (r[..r.len() - 1].iter().map(|p| p.0).sum::<f64>() / m, r[..r.len() - 1].iter().map(|p| p.1).sum::<f64>() / m)
    };
    let parts: Vec<((f64, f64), f64)> = out.buildings.iter().filter(|b| b.part).map(|b| (centroid(&b.ring), ll_area(&b.ring))).collect();
    if !parts.is_empty() {
        out.buildings.retain(|b| {
            if b.part {
                return true;
            }
            let covered: f64 = parts.iter().filter(|(c, _)| crate::areas::point_in(*c, &b.ring)).map(|(_, a)| *a).sum();
            covered < 0.6 * ll_area(&b.ring)
        });
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

/// Entrances where the map has none: apartment blocks (3+ storeys, 20 m+) get a staircase door every ~18 m
/// along their longest wall on the side away from the nearest road (the courtyard), other buildings over
/// 60 m² one door on the wall nearest a road. Doors already mapped near a wall suppress the estimate there.
pub fn estimate_entrances(sc: &mut Scene) {
    let mapped: Vec<[f64; 2]> = sc.entrances.iter().map(|e| e.p).collect();
    let road_pts: Vec<[f64; 2]> = sc.roads.iter().flat_map(|r| r.pts.iter().copied()).collect();
    let mut grid: HashMap<(i64, i64), Vec<[f64; 2]>> = HashMap::new();
    for p in &road_pts {
        grid.entry(((p[0] / 50.0).floor() as i64, (p[1] / 50.0).floor() as i64)).or_default().push(*p);
    }
    let road_dist = |x: f64, y: f64| {
        let (ci, cj) = ((x / 50.0).floor() as i64, (y / 50.0).floor() as i64);
        let mut best = f64::MAX;
        for i in ci - 2..=ci + 2 {
            for j in cj - 2..=cj + 2 {
                for p in grid.get(&(i, j)).into_iter().flatten() {
                    best = best.min((p[0] - x).hypot(p[1] - y));
                }
            }
        }
        best
    };
    let mut out = Vec::new();
    for b in &sc.buildings {
        let f = &b.footprint;
        let n = f.len();
        if n < 3 || b.kind == "part" || matches!(b.kind.as_str(), "garage" | "garages" | "shed" | "roof" | "hut" | "kiosk" | "transformer_tower" | "service") {
            continue;
        }
        let zmin = b.verts.chunks(3).map(|v| v[2]).fold(f32::MAX, f32::min);
        if zmin > 0.5 {
            continue;
        }
        let area: f64 = (0..n).map(|i| f[i][0] * f[(i + 1) % n][1] - f[(i + 1) % n][0] * f[i][1]).sum::<f64>().abs() / 2.0;
        if area < 60.0 {
            continue;
        }
        let cx = f.iter().map(|p| p[0]).sum::<f64>() / n as f64;
        let cy = f.iter().map(|p| p[1]).sum::<f64>() / n as f64;
        // already has a mapped door within the outline's reach
        let reach = f.iter().map(|p| (p[0] - cx).hypot(p[1] - cy)).fold(0.0, f64::max) + 2.0;
        if mapped.iter().any(|m| (m[0] - cx).hypot(m[1] - cy) < reach) {
            continue;
        }
        let edges: Vec<([f64; 2], [f64; 2], f64, [f64; 2])> = (0..n)
            .map(|i| {
                let (a, c) = (f[i], f[(i + 1) % n]);
                let l = (c[0] - a[0]).hypot(c[1] - a[1]);
                (a, c, l, if l > 1e-9 { [(c[1] - a[1]) / l, -(c[0] - a[0]) / l] } else { [0.0, 0.0] })
            })
            .collect();
        let mid = |e: &([f64; 2], [f64; 2], f64, [f64; 2])| [(e.0[0] + e.1[0]) / 2.0, (e.0[1] + e.1[1]) / 2.0];
        let floors = b.height as f64 / b.facade.floor_height.max(2.4) as f64;
        let long = edges.iter().cloned().fold(None::<([f64; 2], [f64; 2], f64, [f64; 2])>, |acc, e| if acc.is_none_or(|a| e.2 > a.2) { Some(e) } else { acc }).unwrap();
        let apartments = matches!(b.kind.as_str(), "apartments" | "residential" | "dormitory" | "yes") && floors >= 3.0 && long.2 >= 20.0;
        if apartments {
            // the long walls facing each other: pick the one farther from roads (courtyard side)
            let opposite = edges.iter().filter(|e| e.2 > long.2 * 0.7 && (e.3[0] * long.3[0] + e.3[1] * long.3[1]) < -0.8).cloned().next();
            let side = match opposite {
                Some(o) => {
                    let (m1, m2) = (mid(&long), mid(&o));
                    if road_dist(m1[0] + long.3[0] * 8.0, m1[1] + long.3[1] * 8.0) > road_dist(m2[0] + o.3[0] * 8.0, m2[1] + o.3[1] * 8.0) { long } else { o }
                }
                None => long,
            };
            let count = ((side.2 / 18.0).round() as usize).max(1);
            let dir = [(side.1[0] - side.0[0]) / side.2, (side.1[1] - side.0[1]) / side.2];
            for k in 0..count {
                let s = side.2 * (k as f64 + 0.5) / count as f64;
                out.push(scene::model::Entrance { p: [side.0[0] + dir[0] * s, side.0[1] + dir[1] * s], out: side.3, kind: "staircase".into() });
            }
        } else {
            // one door, on the longer of the walls closest to a road
            let e = edges.iter().filter(|e| e.2 >= 4.0).min_by(|a, b| {
                let (ma, mb) = (mid(a), mid(b));
                road_dist(ma[0] + a.3[0] * 3.0, ma[1] + a.3[1] * 3.0).total_cmp(&road_dist(mb[0] + b.3[0] * 3.0, mb[1] + b.3[1] * 3.0))
            });
            if let Some(e) = e {
                let m = mid(e);
                let kind = if matches!(b.kind.as_str(), "retail" | "commercial" | "office" | "supermarket") { "main" } else { "yes" };
                out.push(scene::model::Entrance { p: m, out: e.3, kind: kind.into() });
            }
        }
    }
    sc.entrances.extend(out);
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
    // roads: points at most 12 m apart so they follow the terrain between map vertices
    for r in &raw.roads {
        let (mut pts, mut jn): (Vec<[f64; 2]>, Vec<bool>) = (Vec::new(), Vec::new());
        let src: Vec<[f64; 2]> = r.pts.iter().map(|p| loc(p.0, p.1)).collect();
        for (i, p) in src.iter().enumerate() {
            if i > 0 {
                let q = src[i - 1];
                let d = (p[0] - q[0]).hypot(p[1] - q[1]);
                let k = (d / 12.0).floor() as usize;
                for s in 1..=k {
                    let t = s as f64 / (k + 1) as f64;
                    pts.push([q[0] + (p[0] - q[0]) * t, q[1] + (p[1] - q[1]) * t]);
                    jn.push(false);
                }
            }
            pts.push(*p);
            jn.push(r.junction.get(i).copied().unwrap_or(false));
        }
        sc.roads.push(Road { cls: r.cls.clone(), width: r.width, pts, junction: jn });
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
    // entrances: onto the nearest wall within 3 m, facing out of the building
    let mut edges: HashMap<(i64, i64), Vec<([f64; 2], [f64; 2])>> = HashMap::new();
    for b in &sc.buildings {
        let f = &b.footprint;
        for i in 0..f.len() {
            let (a, c) = (f[i], f[(i + 1) % f.len()]);
            let m = [(a[0] + c[0]) / 2.0, (a[1] + c[1]) / 2.0];
            let len = (c[0] - a[0]).hypot(c[1] - a[1]);
            let r = (len / 2.0 / 20.0).ceil() as i64;
            let (ci, cj) = ((m[0] / 20.0).floor() as i64, (m[1] / 20.0).floor() as i64);
            for di in -r..=r {
                for dj in -r..=r {
                    edges.entry((ci + di, cj + dj)).or_default().push((a, c));
                }
            }
        }
    }
    for e in &raw.entrances {
        let p = loc(e.lat, e.lon);
        let (ci, cj) = ((p[0] / 20.0).floor() as i64, (p[1] / 20.0).floor() as i64);
        let mut best: Option<(f64, [f64; 2], [f64; 2])> = None;
        for (a, c) in edges.get(&(ci, cj)).into_iter().flatten() {
            let (dx, dy) = (c[0] - a[0], c[1] - a[1]);
            let l2 = dx * dx + dy * dy;
            if l2 < 1.0 {
                continue;
            }
            let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.1, 0.9);
            let q = [a[0] + dx * t, a[1] + dy * t];
            let d = (p[0] - q[0]).hypot(p[1] - q[1]);
            let l = l2.sqrt();
            // footprints are counter-clockwise: outside is to the right of the edge
            if d < 3.0 && best.is_none_or(|b| d < b.0) {
                best = Some((d, q, [dy / l, -dx / l]));
            }
        }
        if let Some((_, q, out)) = best {
            sc.entrances.push(scene::model::Entrance { p: q, out, kind: e.kind.clone() });
        }
    }

    let (areas, area_trees) = crate::areas::build_areas(&raw.areas, frame, &on_road);
    sc.areas = areas;
    sc.trees.extend(area_trees);
    // buildings without mapped entrances get typical ones (estimated, see README)
    estimate_entrances(&mut sc);
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

    #[test]
    fn building_parts_replace_their_outline_and_entrances_sit_on_walls() {
        let (la, lo) = MINSK;
        let m = 1.0 / 111_320.0;
        let k = 1.0 / (111_320.0 * la.to_radians().cos());
        let sq = |x0: f64, y0: f64, x1: f64, y1: f64| format!(
            r#"[{{"lat":{a},"lon":{b}}},{{"lat":{a},"lon":{d}}},{{"lat":{c},"lon":{d}}},{{"lat":{c},"lon":{b}}},{{"lat":{a},"lon":{b}}}]"#,
            a = la + y0 * m, b = lo + x0 * k, c = la + y1 * m, d = lo + x1 * k
        );
        let json = format!(
            r#"{{"elements":[
              {{"type":"way","id":1,"tags":{{"building":"apartments","building:levels":"9"}},"geometry":{}}},
              {{"type":"way","id":2,"tags":{{"building:part":"yes","building:levels":"9"}},"geometry":{}}},
              {{"type":"way","id":3,"tags":{{"building:part":"yes","building:levels":"16","building:min_level":"9"}},"geometry":{}}},
              {{"type":"way","id":4,"tags":{{"building":"yes","building:levels":"2"}},"geometry":{}}},
              {{"type":"node","id":5,"lat":{},"lon":{},"tags":{{"entrance":"staircase","ref":"1"}}}}]}}"#,
            sq(0.0, 0.0, 40.0, 12.0), sq(0.0, 0.0, 40.0, 12.0), sq(10.0, 0.0, 20.0, 12.0), sq(60.0, 0.0, 70.0, 10.0), la - 0.5 * m, lo + 30.0 * k
        );
        let tile = TileId::from_lat_lon(la + 6.0 * m, lo + 20.0 * k, 15);
        let raw = parse_overpass(&json, tile).unwrap();
        // the outline is covered by its parts and dropped; the other house stays
        assert_eq!(raw.buildings.len(), 3, "{:?}", raw.buildings.iter().map(|b| b.id).collect::<Vec<_>>());
        assert!(!raw.buildings.iter().any(|b| b.id == 1));
        assert_eq!(raw.entrances.len(), 1);
        let f = LocalFrame::new(la, lo, 0.0);
        let sc = build_scene(&raw, &f);
        let tower = sc.buildings.iter().find(|b| b.id.ends_with("/3")).unwrap();
        let zmin = tower.verts.chunks(3).map(|v| v[2]).fold(f32::MAX, f32::min);
        assert!((zmin - 27.0).abs() < 0.01, "upper part starts at storey 9: {zmin}");
        // the mapped door, plus an estimated one for the small house that has none in the data
        assert_eq!(sc.entrances.len(), 2);
        let est = &sc.entrances[1];
        assert!(est.p[0] > 59.0 && est.p[0] < 71.0, "estimated door on the house: {:?}", est.p);
        let e = &sc.entrances[0];
        assert!(e.p[1].abs() < 0.1 && (e.p[0] - 30.0).abs() < 0.2, "on the south wall: {:?}", e.p);
        assert!(e.out[1] < -0.99, "facing south, out of the building: {:?}", e.out);
        assert!(scene::mesh::scene_meshes_opts(&sc, false).iter().any(|m| m.name == "entrances" && !m.verts.is_empty()));
    }
}