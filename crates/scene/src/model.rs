//! Scene data model and loader for scene.json (see tools/osm_to_scene.py).
use crate::json::Json;
use phys::StaticWorld;

#[derive(Clone, Debug)]
pub struct Road {
    pub cls: String,
    pub width: f64,
    pub pts: Vec<[f64; 2]>,
    pub junction: Vec<bool>,
}

#[derive(Clone, Debug)]
pub struct Building {
    pub id: String,
    pub status: String,
    pub footprint: Vec<[f64; 2]>,
    /// Building height in metres and where it came from ("tag", "estimated" or "default").
    pub height: f32,
    pub height_origin: String,
    /// xyz triples of the Stage A mesh.
    pub verts: Vec<f32>,
    pub indices: Vec<u32>,
    pub facade: Facade,
    /// Footprint vertex count when `verts` starts with the bottom ring then the eave ring (Stage A layout);
    /// further vertices (ridge, apex) follow. 0 = derive from `verts` (bottom + top ring only).
    pub ring: u32,
}

/// How a building's walls and roof look, from map attributes (Overture/OSM) or from heuristics.
/// This is an approximation of the real facade, not a reconstruction of it.
#[derive(Clone, Debug, PartialEq)]
pub struct Facade {
    /// one of `mesh::mat::WALL_*`
    pub style: f32,
    /// linear-ish sRGB 0..1
    pub colour: [f32; 3],
    pub roof_colour: [f32; 3],
    /// storey height used to align window rows (m)
    pub floor_height: f32,
    /// true when style/colour came from the data rather than defaults
    pub from_data: bool,
}

impl Default for Facade {
    fn default() -> Self {
        Facade { style: crate::mesh::mat::WALL_PLASTER, colour: [0.80, 0.76, 0.68], roof_colour: [0.38, 0.33, 0.30], floor_height: 3.0, from_data: false }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Meta {
    pub source: Option<String>,
    pub terrain: String,
    pub n_roads: usize,
    pub n_buildings: usize,
    pub n_trees: usize,
    pub status_counts: Vec<(String, usize)>,
    pub height_origin_counts: Vec<(String, usize)>,
}

#[derive(Clone, Debug, Default)]
pub struct Scene {
    pub meta: Meta,
    pub roads: Vec<Road>,
    pub buildings: Vec<Building>,
    pub trees: Vec<[f64; 2]>,
    /// ground cover: water, grass, parks, forest floor, sand (flat, drawn under the roads)
    pub areas: Vec<Area>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AreaKind {
    Water,
    Grass,
    Park,
    Forest,
    Sand,
    /// surface car parks (multi-storey ones are buildings)
    Parking,
}

impl AreaKind {
    pub fn material(self) -> f32 {
        use crate::mesh::mat;
        match self {
            AreaKind::Water => mat::WATER,
            AreaKind::Grass | AreaKind::Park => mat::GRASS,
            AreaKind::Forest => mat::FOREST_FLOOR,
            AreaKind::Sand => mat::SAND,
            AreaKind::Parking => mat::PARKING,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            AreaKind::Water => "water",
            AreaKind::Grass => "grass",
            AreaKind::Park => "park",
            AreaKind::Forest => "forest",
            AreaKind::Sand => "sand",
            AreaKind::Parking => "parking",
        }
    }
    pub fn parse(s: &str) -> Option<AreaKind> {
        [AreaKind::Water, AreaKind::Grass, AreaKind::Park, AreaKind::Forest, AreaKind::Sand, AreaKind::Parking].into_iter().find(|k| k.name() == s)
    }
}

/// A flat ground-cover polygon in local metres, already triangulated.
#[derive(Clone, Debug)]
pub struct Area {
    pub kind: AreaKind,
    pub ring: Vec<[f64; 2]>,
    pub tris: Vec<[u32; 3]>,
}

#[derive(Clone, Copy, Debug)]
pub struct Spawn {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

fn num(j: &Json) -> Result<f64, String> {
    j.as_f64().ok_or_else(|| "expected number".to_string())
}

fn pts2(j: &Json) -> Result<Vec<[f64; 2]>, String> {
    j.as_arr()
        .ok_or("expected array of points")?
        .iter()
        .map(|p| {
            let a = p.as_arr().ok_or("expected [x,y]")?;
            if a.len() < 2 {
                return Err("point needs 2 numbers".to_string());
            }
            Ok([num(&a[0])?, num(&a[1])?])
        })
        .collect()
}

fn counts(j: Option<&Json>) -> Vec<(String, usize)> {
    match j {
        Some(Json::Obj(o)) => o.iter().filter_map(|(k, v)| v.as_f64().map(|n| (k.clone(), n as usize))).collect(),
        _ => Vec::new(),
    }
}

impl Scene {
    pub fn from_json_str(s: &str) -> Result<Scene, String> {
        let j = Json::parse(s).map_err(|e| e.to_string())?;
        Self::from_json(&j)
    }

    pub fn from_json(j: &Json) -> Result<Scene, String> {
        let mut sc = Scene::default();
        if let Some(m) = j.get("meta") {
            sc.meta.source = m.get("source").and_then(|v| v.as_str()).map(String::from);
            sc.meta.terrain = m.get("terrain").and_then(|v| v.as_str()).unwrap_or("FLAT").to_string();
            sc.meta.status_counts = counts(m.get("status_counts"));
            sc.meta.height_origin_counts = counts(m.get("height_origin_counts"));
        }
        for r in j.get("roads").and_then(|v| v.as_arr()).unwrap_or(&[]) {
            let pts = pts2(r.get("pts").ok_or("road without pts")?)?;
            let mut junction: Vec<bool> = r
                .get("junction")
                .and_then(|v| v.as_arr())
                .map(|a| a.iter().map(|x| x.as_bool().unwrap_or(false)).collect())
                .unwrap_or_default();
            junction.resize(pts.len(), false);
            sc.roads.push(Road {
                cls: r.get("cls").and_then(|v| v.as_str()).unwrap_or("residential").to_string(),
                width: r.get("w").and_then(|v| v.as_f64()).unwrap_or(6.5).max(2.0),
                pts,
                junction,
            });
        }
        for b in j.get("buildings").and_then(|v| v.as_arr()).unwrap_or(&[]) {
            let footprint = pts2(b.get("fp").ok_or("building without fp")?)?;
            let verts: Vec<f32> = b
                .get("v")
                .and_then(|v| v.as_arr())
                .ok_or("building without v")?
                .iter()
                .map(|x| x.as_f64().unwrap_or(0.0) as f32)
                .collect();
            let indices: Vec<u32> = b
                .get("i")
                .and_then(|v| v.as_arr())
                .ok_or("building without i")?
                .iter()
                .map(|x| x.as_f64().unwrap_or(0.0) as u32)
                .collect();
            // Reject out-of-range indices instead of crashing later in mesh generation.
            if verts.len() % 3 != 0 || indices.len() % 3 != 0 || indices.iter().any(|&k| (k as usize) * 3 + 2 >= verts.len()) {
                return Err(format!("building {} has an inconsistent mesh", b.get("id").and_then(|v| v.as_str()).unwrap_or("?")));
            }
            sc.buildings.push(Building {
                facade: Facade::default(),
                ring: 0,
                id: b.get("id").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                status: b.get("status").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                footprint,
                height: b.get("h").and_then(|v| v.as_f64()).unwrap_or(0.0) as f32,
                height_origin: b.get("h_src").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                verts,
                indices,
            });
        }
        for t in j.get("trees").and_then(|v| v.as_arr()).unwrap_or(&[]) {
            let a = t.as_arr().ok_or("tree must be [x,y]")?;
            if a.len() >= 2 {
                sc.trees.push([num(&a[0])?, num(&a[1])?]);
            }
        }
        for a in j.get("areas").and_then(|v| v.as_arr()).unwrap_or(&[]) {
            let Some(kind) = a.get("kind").and_then(|v| v.as_str()).and_then(AreaKind::parse) else { continue };
            let ring = pts2(a.get("ring").ok_or("area without ring")?)?;
            let tris: Vec<[u32; 3]> = a.get("tris").and_then(|v| v.as_arr()).unwrap_or(&[]).chunks_exact(3).map(|c| [0, 1, 2].map(|k| c[k].as_f64().unwrap_or(0.0) as u32)).collect();
            if tris.iter().flatten().all(|&i| (i as usize) < ring.len()) {
                sc.areas.push(Area { kind, ring, tris });
            }
        }
        sc.meta.n_roads = sc.roads.len();
        sc.meta.n_buildings = sc.buildings.len();
        sc.meta.n_trees = sc.trees.len();
        Ok(sc)
    }

    /// Half-size of the area covered by roads and buildings (m), at least 100.
    pub fn extent(&self) -> f64 {
        let mut m: f64 = 100.0;
        for r in &self.roads {
            for p in &r.pts {
                m = m.max(p[0].abs()).max(p[1].abs());
            }
        }
        for b in &self.buildings {
            for p in &b.footprint {
                m = m.max(p[0].abs()).max(p[1].abs());
            }
        }
        m
    }

    /// Starting pose: the point on a real road (width >= 5 m) closest to the origin, in the right-hand lane.
    pub fn spawn(&self) -> Spawn {
        let mut best = f64::MAX;
        let mut sp = Spawn { x: 0.0, y: 0.0, yaw: 0.0 };
        for r in &self.roads {
            if r.width < 5.0 {
                continue;
            }
            for w in r.pts.windows(2) {
                let (a, b) = (w[0], w[1]);
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                let l2 = dx * dx + dy * dy;
                if l2 < 1e-9 {
                    continue;
                }
                let t = ((-a[0]) * dx + (-a[1]) * dy) / l2;
                let t = t.clamp(0.0, 1.0);
                let (px, py) = (a[0] + dx * t, a[1] + dy * t);
                let d = px.hypot(py);
                if d < best {
                    best = d;
                    let l = l2.sqrt();
                    sp = Spawn { x: px + dy / l * r.width / 4.0, y: py - dx / l * r.width / 4.0, yaw: dy.atan2(dx) };
                }
            }
        }
        sp
    }

    /// Collision world: building footprints as polygons, tree trunks as discs.
    pub fn static_world(&self) -> StaticWorld {
        let mut w = StaticWorld::new(25.0);
        for b in &self.buildings {
            w.add_poly(b.footprint.iter().map(|p| (p[0], p[1])).collect());
        }
        for t in &self.trees {
            w.add_circle(t[0], t[1], 0.35);
        }
        w
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    pub const SAMPLE: &str = r#"{"meta":{"terrain":"FLAT","status_counts":{"PROCEDURAL_APPROXIMATION":1}},
      "roads":[{"cls":"primary","w":11,"pts":[[-100,0],[0,0],[100,0]],"junction":[false,true,false]},
               {"cls":"service","w":4,"pts":[[0,-50],[0,50]],"junction":[false,false]}],
      "buildings":[{"id":"osm:way/1","status":"PROCEDURAL_APPROXIMATION","fp":[[20,10],[40,10],[40,30],[20,30]],
        "v":[20,10,0,40,10,0,40,30,0,20,30,0,20,10,9,40,10,9,40,30,9,20,30,9],
        "i":[0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7]}],
      "trees":[[5,8],[7,8]]}"#;

    #[test]
    fn loads_sample_scene() {
        let s = Scene::from_json_str(SAMPLE).unwrap();
        assert_eq!((s.roads.len(), s.buildings.len(), s.trees.len()), (2, 1, 2));
        assert_eq!(s.roads[0].junction, vec![false, true, false]);
        assert_eq!(s.meta.status_counts[0].0, "PROCEDURAL_APPROXIMATION");
        assert!(s.extent() >= 100.0);
    }

    #[test]
    fn spawn_is_on_the_wide_road_in_the_right_lane() {
        let s = Scene::from_json_str(SAMPLE).unwrap();
        let sp = s.spawn();
        // closest wide road passes through the origin heading +X; right lane is y = -w/4
        assert!((sp.yaw - 0.0).abs() < 1e-9 && (sp.y + 11.0 / 4.0).abs() < 1e-9, "{sp:?}");
    }

    #[test]
    fn rejects_bad_indices() {
        let bad = SAMPLE.replace("\"i\":[0,2,1", "\"i\":[0,2,99");
        assert!(Scene::from_json_str(&bad).is_err());
    }

    #[test]
    fn static_world_has_buildings_and_trees() {
        let s = Scene::from_json_str(SAMPLE).unwrap();
        let w = s.static_world();
        assert_eq!((w.poly_count(), w.circle_count()), (1, 2));
    }
}
