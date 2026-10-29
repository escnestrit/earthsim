//! Game state: fixed-step physics, camera, input mapping and HUD data. Platform independent.
use crate::m4::{self, V3f};
use crate::render::{FrameParams, HudData};
use phys::{Driver, Keys, SoftVehicle, StaticWorld, Vehicle, VehicleParams};
use scene::{car_meshes, scene_meshes, MeshData, Scene};
use std::collections::HashMap;
use std::sync::Arc;
use world::streamer::{StreamEvent, Streamer};
use world::tiles::{TileId, ZOOM};

/// Changes to the GPU side that the game queues for the renderer.
pub enum RenderOp {
    AddTile(u64, Arc<Vec<MeshData>>),
    RemoveTile(u64),
    FollowGround(bool),
}

/// Places the F2 key jumps to.
pub const PRESETS: &[(&str, f64, f64)] = &[
    ("MINSK", 53.9045, 27.5615),
    ("PARIS", 48.8584, 2.2945),
    ("NEW YORK", 40.7580, -73.9855),
    ("TOKYO", 35.6595, 139.7005),
    ("LONDON", 51.5074, -0.1278),
    ("BERLIN", 52.5200, 13.4050),
];

struct Stream {
    streamer: Streamer,
    scenes: HashMap<u64, Arc<Scene>>,
    home: (String, f64, f64),
    preset: usize,
    place: String,
    /// Car is held still until the map around it has arrived.
    waiting: bool,
    wait_since: std::time::Instant,
    origin_latlon: (f64, f64),
}

pub const PHYS_DT: f64 = 0.001;

/// Terrain field: half-size and cell (m).
const TERRAIN_HALF: f64 = 2500.0;
const TERRAIN_CELL: f64 = 10.0;

#[derive(Default)]
struct Terrain {
    src: Option<Arc<dyn world::dem::DemSource>>,
    datum: Option<f64>,
    center: (f64, f64),
    job: Option<std::sync::mpsc::Receiver<(phys::HeightField, f64, Vec<String>)>>,
    generation: u64,
    /// flat meshes of the loaded tiles (re-draped when the field changes)
    flat: HashMap<u64, Arc<Vec<MeshData>>>,
    redrape: Vec<u64>,
}

/// Put a tile's flat meshes on the terrain: every vertex rises by the ground height under it; building
/// feet go a little into the ground so slopes leave no gaps under walls.
pub fn drape(ms: &[MeshData], f: &phys::HeightField) -> Vec<MeshData> {
    if f.is_empty() {
        return ms.to_vec();
    }
    ms.iter()
        .map(|m| {
            let sink = matches!(m.name.as_str(), "buildings" | "building_details" | "entrances");
            let mut m = m.clone();
            // a bridge spans straight between the ground at its ends
            if let Some((a, b)) = scene::bridge::anchors(&m.name) {
                let (ha, hb) = (f.height(a[0], a[1]), f.height(b[0], b[1]));
                for v in m.verts.chunks_exact_mut(scene::mesh::FLOATS_PER_VERTEX) {
                    let t = scene::bridge::chord_t(a, b, v[0] as f64, v[1] as f64);
                    v[2] += (ha + (hb - ha) * t) as f32;
                }
                return m;
            }
            for v in m.verts.chunks_exact_mut(scene::mesh::FLOATS_PER_VERTEX) {
                let h = f.height(v[0] as f64, v[1] as f64) as f32;
                v[2] += h - if sink && v[2] < 0.05 { 1.0 } else { 0.0 };
            }
            m
        })
        .collect()
}

/// State shared between the game and the physics thread of a JBeam car.
pub struct JShared {
    pub v: jbeam::vehicle::JbeamVehicle,
    pub keys: Keys,
    pub driver: Driver,
    pub world: Arc<StaticWorld>,
    /// terrain under the car (flat where no elevation data)
    pub ground: phys::TerrainGround,
    /// held still (map not loaded yet)
    pub frozen: bool,
    pub steps: u64,
}

impl JShared {
    fn step_n(&mut self, n: usize) {
        if self.frozen {
            return;
        }
        let JShared { v, keys, driver, world, steps, ground, .. } = self;
        let ground = &*ground;
        for _ in 0..n {
            let fwd_speed = v.vel.dot(v.forward());
            let inp = driver.update(PHYS_DT, keys, fwd_speed);
            v.step(PHYS_DT, &inp, ground, &**world);
            *steps += 1;
        }
    }
}

/// A JBeam car simulated on its own thread in real time (or stepped by the game when `threaded` is false,
/// for deterministic tests), so a heavy car never holds up rendering.
pub struct JCar {
    shared: Arc<std::sync::Mutex<JShared>>,
    stop: Arc<std::sync::atomic::AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl JCar {
    pub fn new(v: jbeam::vehicle::JbeamVehicle, driver: Driver, world: Arc<StaticWorld>, threaded: bool) -> JCar {
        let shared = Arc::new(std::sync::Mutex::new(JShared { v, keys: Keys::default(), driver, world, ground: phys::TerrainGround::default(), frozen: false, steps: 0 }));
        let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let mut c = JCar { shared, stop, thread: None };
        if threaded {
            c.spawn();
        }
        c
    }

    /// Start the physics thread (no-op when already running).
    pub fn spawn(&mut self) {
        if self.thread.is_some() {
            return;
        }
        let shared = self.shared.clone();
        let stop = self.stop.clone();
        self.thread = Some({
            let (sh, st) = (shared.clone(), stop.clone());
            std::thread::Builder::new()
                .name("physics".into())
                .spawn(move || {
                    let mut last = std::time::Instant::now();
                    let mut acc = 0.0;
                    while !st.load(std::sync::atomic::Ordering::Relaxed) {
                        let now = std::time::Instant::now();
                        // a stall (debugger, slow PC) is not made up for: the car slows down instead of exploding
                        acc = (acc + (now - last).as_secs_f64()).min(0.05);
                        last = now;
                        let n = ((acc / PHYS_DT) as usize).min(8);
                        if n == 0 {
                            std::thread::sleep(std::time::Duration::from_micros(300));
                            continue;
                        }
                        acc -= n as f64 * PHYS_DT;
                        // short critical sections so the renderer can read the state between them
                        sh.lock().unwrap_or_else(|e| e.into_inner()).step_n(n);
                    }
                })
                .expect("physics thread")
        });

    }

    pub fn lock(&self) -> std::sync::MutexGuard<'_, JShared> {
        self.shared.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn is_threaded(&self) -> bool {
        self.thread.is_some()
    }
}

impl Drop for JCar {
    fn drop(&mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}
/// Rest height of the centre of gravity above the ground; the car mesh origin sits on the ground.
pub const CG_HEIGHT: f64 = 0.581;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Key {
    Up,
    Down,
    Left,
    Right,
    Handbrake,
    Camera,
    HourUp,
    HourDown,
    Lights,
    Respawn,
    Recover,
    Hud,
    Tcs,
    Abs,
    Help,
    Teleport,
    SoftBody,
}

pub struct Game {
    pub scene: Scene,
    pub meshes: Vec<MeshData>,
    pub scene_dirty: bool,
    pub world: StaticWorld,
    pub car: Vehicle,
    pub driver: Driver,
    pub keys: Keys,
    ground: phys::TerrainGround,
    terrain: Terrain,
    spawn: scene::model::Spawn,
    acc: f64,
    pub hour: f32,
    pub cam_mode: u8,
    cam_yaw: f64,
    pub headlights: Option<bool>,
    pub show_hud: bool,
    pub show_help: bool,
    pub fps: f32,
    fps_acc: f32,
    fps_n: u32,
    pub draws: u32,
    pub triangles: u64,
    pub sim_steps_last_frame: u32,
    pub renderer_info: String,
    pub car_meshes: (MeshData, MeshData),
    pub render_ops: Vec<RenderOp>,
    pub soft: SoftVehicle,
    pub soft_on: bool,
    skin: crate::softmesh::SkinnedBody,
    /// A BeamNG vehicle mod driven with the soft-body solver (replaces the built-in soft car while F3 is on).
    pub jcar: Option<JCar>,
    /// run the JBeam car on its own thread (false: stepped inside `update`, for tests)
    pub threaded_physics: bool,
    /// name + loading notes shown in the HUD
    pub jcar_info: Vec<String>,
    /// jcar wheel index for FL, FR, RL, RR
    jcar_map: [Option<usize>; 4],
    /// the mod's meshes/materials/textures, for the renderer to take once
    pub jcar_visual: Option<jbeam::visual::CarVisual>,
    /// the mod car came with meshes (stays true after the front end takes `jcar_visual`)
    pub jcar_has_visual: bool,
    /// street lamp heads per tile (u64::MAX: the static scene)
    lamps: HashMap<u64, Vec<[f64; 3]>>,
    /// cars driving around the player
    pub traffic: crate::traffic::Traffic,
    traffic_last: Option<(f64, f64)>,
    pub deformed: Vec<f32>,
    /// Fixed camera (eye, target, fov) used for screenshots and tests instead of the follow cameras.
    pub cam_override: Option<(V3f, V3f, f32)>,
    stream: Option<Box<Stream>>,
}

impl Game {
    pub fn new(scene: Scene) -> Game {
        let ground = phys::TerrainGround::default();
        let spawn = scene.spawn();
        let car = Vehicle::new(VehicleParams::sedan(), spawn.x, spawn.y, spawn.yaw, &ground);
        let driver = Driver::new(car.p.steer_max);
        let world = scene.static_world();
        let meshes = scene_meshes(&scene);
        let soft = SoftVehicle::new(VehicleParams::sedan(), spawn.x, spawn.y, spawn.yaw, &ground);
        let car_meshes = car_meshes();
        let skin = crate::softmesh::SkinnedBody::new(&car_meshes.0, &soft);
        let mut g = Game {
            scene,
            meshes,
            scene_dirty: true,
            world,
            car,
            driver,
            keys: Keys::default(),
            ground,
            terrain: Terrain::default(),
            spawn,
            acc: 0.0,
            hour: 15.0,
            cam_mode: 0,
            cam_yaw: spawn.yaw,
            headlights: None,
            show_hud: true,
            show_help: true,
            fps: 0.0,
            fps_acc: 0.0,
            fps_n: 0,
            draws: 0,
            triangles: 0,
            sim_steps_last_frame: 0,
            renderer_info: String::new(),
            car_meshes,
            render_ops: Vec::new(),
            soft,
            soft_on: true,
            skin,
            jcar: None,
            threaded_physics: false,
            jcar_info: Vec::new(),
            jcar_map: [None; 4],
            jcar_visual: None,
            jcar_has_visual: false,
            lamps: HashMap::new(),
            traffic: crate::traffic::Traffic::default(),
            traffic_last: None,
            deformed: Vec::new(),
            cam_override: None,
            stream: None,
        };
        g.rebuild_decks();
        g.rebuild_traffic();
        g
    }

    /// Streaming game: the world is loaded around the car while it drives. `home` is where it starts and where
    /// R brings it back to.
    pub fn new_streaming(streamer: Streamer, name: &str, lat: f64, lon: f64) -> Game {
        let mut g = Game::new(Scene::default());
        g.meshes.clear();
        g.world = StaticWorld::new(25.0);
        g.scene.meta.source = Some("(c) OPENSTREETMAP CONTRIBUTORS (ODBL)".into());
        g.scene.meta.terrain = String::new();
        g.render_ops.push(RenderOp::FollowGround(true));
        g.stream = Some(Box::new(Stream {
            streamer,
            scenes: HashMap::new(),
            home: (name.to_string(), lat, lon),
            preset: usize::MAX,
            place: name.to_string(),
            waiting: true,
            wait_since: std::time::Instant::now(),
            origin_latlon: (lat, lon),
        }));
        g
    }

    /// Put the car (rigid and soft model alike) on the ground, repaired, stopped or rolling at `speed`.
    fn place_car(&mut self, x: f64, y: f64, yaw: f64, speed: f64) {
        self.car.place(x, y, yaw, speed, &self.ground);
        self.soft.place(x, y, yaw, speed, &self.ground);
        if let Some(j) = self.jcar.as_ref() {
            let mut s = j.lock();
            s.v.place(x, y, yaw, speed, &self.ground);
            s.driver = Driver::new(self.car.p.steer_max);
        }
        self.sync_from_soft_if_on();
    }

    /// Crash test: a 3 m wide, 3 m high concrete block `gap` metres ahead of the bumper, the car facing it (+X)
    /// at `speed` m/s.
    pub fn crash_test(&mut self, speed: f64, gap: f64) {
        let (x, y) = (self.car.pos.x, self.car.pos.y);
        let front = match self.jcar.as_ref() {
            Some(j) => {
                let s = j.lock();
                let rc = s.v.rest_com();
                s.v.body.nodes.iter().map(|n| n.rest.x - rc.x).fold(f64::MIN, f64::max)
            }
            None => 2.3,
        };
        let wx = x + front + gap;
        self.world.add_poly(vec![(wx, y - 1.5), (wx + 1.0, y - 1.5), (wx + 1.0, y + 1.5), (wx, y + 1.5)]);
        let mut mb = scene::mesh::Mb::default();
        mb.box_(wx, y - 1.5, 0.0, wx + 1.0, y + 1.5, 3.0, &[scene::mesh::mat::SIDEWALK]);
        let mesh = MeshData { name: "crash_block".into(), verts: mb.a, polygon_offset: 0.0, casts_shadow: true, colors: Vec::new() };
        self.render_ops.push(RenderOp::AddTile(u64::MAX - 1, Arc::new(vec![mesh])));
        self.publish_world();
        self.place_car(x, y, 0.0, speed);
        self.driver = Driver::new(self.car.p.steer_max);
    }

    /// Drive a BeamNG vehicle mod instead of the built-in car.
    pub fn set_jbeam_car(&mut self, mut lv: jbeam::LoadedVehicle, name: &str) {
        self.jcar_visual = lv.visual.take();
        self.jcar_has_visual = self.jcar_visual.is_some();
        let mut v = lv.vehicle;
        let f = self.car.forward();
        v.place(self.car.pos.x, self.car.pos.y, f.y.atan2(f.x), 0.0, &self.ground);
        // FL, FR, RL, RR by position in the car frame
        let mut map = [None; 4];
        for (i, w) in v.wheels.iter().enumerate() {
            let left = (v.body.nodes[w.n1].rest.y + v.body.nodes[w.n2].rest.y) > 0.0;
            let slot = match (w.front, left) {
                (true, true) => 0,
                (true, false) => 1,
                (false, true) => 2,
                (false, false) => 3,
            };
            if map[slot].is_none() {
                map[slot] = Some(i);
            }
        }
        self.jcar_map = map;
        let threaded = self.threaded_physics;
        self.jcar_info = vec![format!("CAR: {} FROM MOD ({} NODES, {} BEAMS, MAIN PART {})", name.to_uppercase(), lv.nodes, lv.beams, lv.main_part.to_uppercase())];
        self.jcar_info.extend(lv.notes.iter().filter(|n| n.starts_with("meshes") || n.contains("driven") || n.contains("engine")).map(|n| format!("  {}", n.to_uppercase())));
        self.jcar = Some(JCar::new(v, Driver::new(self.car.p.steer_max), Arc::new(self.world.clone()), threaded));
        self.soft_on = true;
        self.sync_from_soft_if_on();
    }

    /// Run the JBeam car on its own thread from now on (the real game; tests keep stepping it in `update`).
    pub fn set_threaded_physics(&mut self, on: bool) {
        self.threaded_physics = on;
        if on {
            if let Some(j) = self.jcar.as_mut() {
                j.spawn();
            }
        }
    }

    /// Road network for the traffic from the loaded tiles.
    fn rebuild_traffic(&mut self) {
        let mut ev = crate::traffic::Events::default();
        match self.stream.as_ref() {
            Some(st) => self.traffic.rebuild(st.scenes.values().map(|s| &**s), &mut ev),
            None => self.traffic.rebuild(std::iter::once(&self.scene), &mut ev),
        }
        self.apply_traffic_events(ev);
    }

    fn apply_traffic_events(&mut self, ev: crate::traffic::Events) {
        if ev.wrecked.is_empty() && ev.cleared.is_empty() {
            return;
        }
        for id in ev.cleared {
            self.world.remove_tag(crate::traffic::TAG_BASE + id);
        }
        for (id, o) in ev.wrecked {
            self.world.add_poly_tagged(crate::traffic::TAG_BASE + id, o);
        }
        self.publish_world();
    }

    /// Traffic cars to draw: (id, seed, position, heading).
    pub fn traffic_cars(&self) -> Vec<(u64, u64, [f32; 3], f32)> {
        self.traffic.agents.iter().map(|a| (a.id, a.seed, [a.x as f32, a.y as f32, a.z as f32], a.yaw as f32)).collect()
    }

    fn traffic_tick(&mut self, dt: f64) {
        if self.traffic.target == 0 && self.traffic.agents.is_empty() {
            return;
        }
        let (x, y) = (self.car.pos.x, self.car.pos.y);
        let speed = self.traffic_last.map_or(0.0, |(lx, ly)| (x - lx).hypot(y - ly) / dt.max(1e-3));
        self.traffic_last = Some((x, y));
        let f = self.car.forward();
        let ground = self.ground.clone();
        let h = move |x: f64, y: f64, z: f64| {
            use phys::Ground;
            ground.sample_at(x, y, z).height
        };
        let ev = self.traffic.step(dt, (x, y, f.y.atan2(f.x), speed), &h);
        self.apply_traffic_events(ev);
    }

    /// Bridge decks of the loaded tiles on the current ground, for the physics.
    fn rebuild_decks(&mut self) {
        let f = self.ground.field.clone();
        let h = move |x: f64, y: f64| f.height(x, y);
        let mut segs = Vec::new();
        match self.stream.as_ref() {
            Some(st) => {
                for sc in st.scenes.values() {
                    segs.extend(scene::bridge::deck_segs(sc, &h));
                }
            }
            None => segs = scene::bridge::deck_segs(&self.scene, &h),
        }
        self.ground.decks = Arc::new(phys::Decks::new(segs));
    }

    /// Give the physics thread the current static world (after tiles were added or removed).
    fn publish_world(&mut self) {
        if let Some(j) = self.jcar.as_ref() {
            let mut s = j.lock();
            s.world = Arc::new(self.world.clone());
            s.ground = self.ground.clone();
        }
    }

    /// Elevation data for terrain (none: flat world).
    pub fn set_terrain_source(&mut self, src: Arc<dyn world::dem::DemSource>) {
        self.terrain.src = Some(src);
        self.terrain.datum = None;
        self.request_terrain(0.0, 0.0);
    }

    /// Terrain heights (for the renderer's ground mesh) and a counter that changes with them.
    pub fn terrain_field(&self) -> (Arc<phys::HeightField>, u64) {
        (self.ground.field.clone(), self.terrain.generation)
    }

    pub fn ground_height(&self, x: f64, y: f64) -> f64 {
        self.ground.field.height(x, y)
    }

    fn request_terrain(&mut self, cx: f64, cy: f64) {
        let (Some(src), Some(st)) = (self.terrain.src.clone(), self.stream.as_ref()) else { return };
        let frame = st.streamer.frame();
        let datum = self.terrain.datum;
        let (tx, rx) = std::sync::mpsc::channel();
        self.terrain.job = Some(rx);
        self.terrain.center = (cx, cy);
        std::thread::Builder::new()
            .name("terrain".into())
            .spawn(move || {
                // the datum is the ground at the first place: local heights stay near zero
                let datum = datum.unwrap_or_else(|| {
                    let (la, lo, _) = frame.from_local(0.0, 0.0, 0.0);
                    world::dem::DemCache::new(&*src).height(la, lo).unwrap_or(0.0)
                });
                let (f, errs) = world::dem::build_field(&*src, &frame, cx, cy, TERRAIN_HALF, TERRAIN_CELL, datum);
                let _ = tx.send((f, datum, errs));
            })
            .ok();
    }

    /// Pick up a finished terrain field; re-drape the loaded tiles; keep the car on the new ground.
    fn terrain_tick(&mut self) {
        if let Some(rx) = &self.terrain.job {
            if let Ok((f, datum, errs)) = rx.try_recv() {
                self.terrain.job = None;
                for e in &errs {
                    world::log::log(&format!("terrain: {e}"));
                }
                let (x, y) = (self.car.pos.x, self.car.pos.y);
                let before = self.ground.field.height(x, y);
                self.terrain.datum = Some(datum);
                self.ground = phys::TerrainGround { field: Arc::new(f), decks: Default::default() };
                self.rebuild_decks();
                self.terrain.generation += 1;
                let dz = self.ground.field.height(x, y) - before;
                if dz.abs() > 1e-3 {
                    self.shift_car(phys::v3(0.0, 0.0, dz));
                }
                world::log::log(&format!("terrain: {}x{} heights around ({x:.0}, {y:.0}), datum {datum:.1} m", self.ground.field.nx, self.ground.field.ny));
                self.publish_world();
                self.terrain.redrape = self.terrain.flat.keys().copied().collect();
                self.scene.meta.terrain = if errs.is_empty() { "ELEVATION: TERRAIN TILES (SRTM ETC., MAPZEN/AWS)".into() } else { "ELEVATION: PARTLY MISSING (SEE LOG)".into() };
            }
        }
        // the car drove far from the field's centre: a new field around it
        if self.terrain.src.is_some() && self.terrain.job.is_none() {
            let (cx, cy) = self.terrain.center;
            if (self.car.pos.x - cx).hypot(self.car.pos.y - cy) > TERRAIN_HALF * 0.6 {
                self.request_terrain(self.car.pos.x, self.car.pos.y);
            }
        }
        // one tile per frame onto the new ground
        if let Some(k) = self.terrain.redrape.pop() {
            if let Some(ms) = self.terrain.flat.get(&k).cloned() {
                self.render_ops.push(RenderOp::AddTile(k, Arc::new(drape(&ms, &self.ground.field))));
            }
        }
    }

    fn shift_car(&mut self, d: phys::V3) {
        self.car.pos += d;
        self.soft.pos += d;
        for n in self.soft.body.nodes.iter_mut() {
            n.p += d;
        }
        if let Some(j) = self.jcar.as_ref() {
            j.lock().v.shift(d);
        }
    }

    fn sync_from_jcar(&mut self) {
        let Some(jc) = self.jcar.as_ref() else { return };
        let guard = jc.lock();
        let j = &guard.v;
        self.driver = guard.driver.clone();
        self.car.pos = j.pos;
        self.car.q = j.q;
        self.car.vel = j.vel;
        self.car.omega = phys::V3::ZERO;
        for (slot, m) in self.jcar_map.iter().enumerate() {
            if let Some(i) = m {
                self.car.wheels[slot] = j.wheels[*i].state.clone();
            }
        }
        self.car.drive = j.drive.clone();
        self.car.impact = j.impact;
        self.car.accel = j.accel;
        // body surface from the mod's triangles, both sides (their winding is not consistent)
        let d = &mut self.deformed;
        d.clear();
        for t in &j.triangles {
            let [a, b, c] = t.map(|i| j.body.nodes[i].p);
            let n = (b - a).cross(c - a);
            let l = n.len();
            if l < 1e-9 {
                continue;
            }
            let n = n / l;
            for (pts, nn) in [([a, b, c], n), ([a, c, b], n * -1.0)] {
                for p in pts {
                    d.extend_from_slice(&[p.x as f32, p.y as f32, p.z as f32, nn.x as f32, nn.y as f32, nn.z as f32, 0.0, 0.0, scene::mesh::mat::PAINT]);
                }
            }
        }
    }

    fn sync_from_soft_if_on(&mut self) {
        if !self.soft_on {
            return;
        }
        if self.jcar.is_some() {
            self.sync_from_jcar();
            return;
        }
        let s = &self.soft;
        self.car.pos = s.pos;
        self.car.q = s.q;
        self.car.vel = s.vel;
        self.car.omega = phys::V3::ZERO;
        self.car.wheels = s.wheels.clone();
        self.car.drive = s.drive.clone();
        self.car.impact = s.impact;
        self.car.accel = s.accel;
        self.skin.deform(&self.soft, &mut self.deformed);
    }

    /// The deformed body mesh in world coordinates when the soft-body model is active.
    pub fn deformed_body(&self) -> Option<&[f32]> {
        (self.soft_on && !self.deformed.is_empty()).then_some(self.deformed.as_slice())
    }

    /// Where the car is on Earth (streaming worlds only): (lat, lon, heading in degrees clockwise from north).
    pub fn car_geo(&self) -> Option<(f64, f64, f64)> {
        let st = self.stream.as_ref()?;
        let (lat, lon, _) = st.streamer.frame().from_local(self.car.pos.x, self.car.pos.y, 0.0);
        let f = self.car.forward();
        Some((lat, lon, f.x.atan2(f.y).to_degrees().rem_euclid(360.0)))
    }

    /// The `n` street lamps nearest to (x, y) (lamp head positions).
    pub fn nearest_lamps(&mut self, x: f64, y: f64, n: usize) -> Vec<[f64; 3]> {
        if self.lamps.is_empty() && !self.scene.roads.is_empty() && self.stream.is_none() {
            self.lamps.insert(u64::MAX, scene::mesh::street_lamps(&self.scene));
        }
        let mut all: Vec<(f64, [f64; 3])> = self.lamps.values().flatten().map(|p| ((p[0] - x).hypot(p[1] - y), *p)).filter(|(d, _)| *d < 250.0).collect();
        all.sort_by(|a, b| a.0.total_cmp(&b.0));
        let f = &self.ground.field;
        all.into_iter().take(n).map(|(_, p)| [p[0], p[1], p[2] + f.height(p[0], p[1])]).collect()
    }

    /// Radius of the loaded world around the car (m).
    pub fn set_view_radius(&mut self, m: f64) {
        if let Some(st) = self.stream.as_mut() {
            st.streamer.set_radius(m);
        }
    }

    /// Name of the current place (preset or teleport target).
    pub fn place_name(&self) -> Option<&str> {
        self.stream.as_ref().map(|s| s.place.as_str())
    }

    pub fn is_streaming(&self) -> bool {
        self.stream.is_some()
    }

    pub fn take_render_ops(&mut self) -> Vec<RenderOp> {
        std::mem::take(&mut self.render_ops)
    }

    /// Jump to a new place: everything loaded is dropped, the car waits for the new map.
    pub fn teleport(&mut self, name: &str, lat: f64, lon: f64) {
        let Some(st) = self.stream.as_mut() else { return };
        let mut ev = Vec::new();
        st.streamer.set_origin(lat, lon, &mut ev);
        st.place = name.to_string();
        st.waiting = true;
        st.wait_since = std::time::Instant::now();
        st.origin_latlon = (lat, lon);
        self.apply_stream_events(ev, false);
        // new place, new ground: flat until its elevation arrives
        self.ground = phys::TerrainGround::default();
        self.terrain.generation += 1;
        self.terrain.datum = None;
        if self.terrain.src.is_some() {
            self.request_terrain(0.0, 0.0);
        }
        self.place_car(0.0, 0.0, 0.0, 0.0);
        self.driver = Driver::new(self.car.p.steer_max);
        self.cam_yaw = 0.0;
    }

    fn apply_stream_events(&mut self, events: Vec<StreamEvent>, shift_car: bool) {
        let Some(st) = self.stream.as_mut() else { return };
        let mut changed = false;
        let mut rebased = false;
        for e in events {
            match e {
                StreamEvent::Add { tile, scene, meshes } => {
                    let key = tile.key();
                    for b in &scene.buildings {
                        self.world.add_poly_tagged(key, b.footprint.iter().map(|p| (p[0], p[1])).collect());
                    }
                    for t in &scene.trees {
                        self.world.add_circle_tagged(key, t[0], t[1], 0.35);
                    }
                    for c in &scene.parked {
                        self.world.add_poly_tagged(key, c.outline());
                    }
                    self.lamps.insert(key, scene::mesh::street_lamps(&scene));
                    st.scenes.insert(key, scene);
                    let draped = if self.ground.field.is_empty() { meshes.clone() } else { Arc::new(drape(&meshes, &self.ground.field)) };
                    self.terrain.flat.insert(key, meshes);
                    self.render_ops.push(RenderOp::AddTile(key, draped));
                    changed = true;
                }
                StreamEvent::Remove { tile } => {
                    let key = tile.key();
                    self.world.remove_tag(key);
                    st.scenes.remove(&key);
                    self.lamps.remove(&key);
                    self.terrain.flat.remove(&key);
                    self.render_ops.push(RenderOp::RemoveTile(key));
                    changed = true;
                }
                StreamEvent::Rebased { old, new } => {
                    // the terrain moves with the frame until a new field is computed
                    {
                        let (la, lo, _) = old.from_local(0.0, 0.0, 0.0);
                        let p = new.to_local(la, lo, 0.0);
                        self.traffic.shift(p[0], p[1]);
                        self.traffic_last = None;
                        if !self.ground.field.is_empty() {
                            self.ground = phys::TerrainGround { field: Arc::new(self.ground.field.shifted(p[0], p[1])), decks: Default::default() };
                            self.terrain.generation += 1;
                        }
                        rebased = true;
                    }
                    if shift_car {
                        let (la, lo, _) = old.from_local(self.car.pos.x, self.car.pos.y, 0.0);
                        let p = new.to_local(la, lo, 0.0);
                        let (dx, dy) = (p[0] - self.car.pos.x, p[1] - self.car.pos.y);
                        self.car.pos.x += dx;
                        self.car.pos.y += dy;
                        self.soft.pos.x += dx;
                        self.soft.pos.y += dy;
                        if let Some(j) = self.jcar.as_ref() {
                            j.lock().v.shift(phys::v3(dx, dy, 0.0));
                        }
                        for n in self.soft.body.nodes.iter_mut() {
                            n.p.x += dx;
                            n.p.y += dy;
                        }
                    }
                }
            }
        }
        if rebased && self.terrain.src.is_some() {
            let (x, y) = (self.car.pos.x, self.car.pos.y);
            self.request_terrain(x, y);
        }
        if changed || rebased {
            self.rebuild_decks();
            self.rebuild_traffic();
        }
        let Some(st) = self.stream.as_mut() else { return };
        if changed || rebased {
            if let Some(j) = self.jcar.as_ref() {
                let mut s = j.lock();
                s.world = Arc::new(self.world.clone());
                s.ground = self.ground.clone();
            }
            let mut meta = std::mem::take(&mut self.scene.meta);
            meta.n_roads = st.scenes.values().map(|s| s.roads.len()).sum();
            meta.n_buildings = st.scenes.values().map(|s| s.buildings.len()).sum();
            meta.n_trees = st.scenes.values().map(|s| s.trees.len()).sum();
            let mut counts: std::collections::BTreeMap<String, usize> = Default::default();
            for sc in st.scenes.values() {
                for b in &sc.buildings {
                    *counts.entry(b.status.clone()).or_default() += 1;
                }
            }
            meta.status_counts = counts.into_iter().collect();
            self.scene.meta = meta;
        }
    }

    /// Nearest wide road among the loaded tiles, as a start pose in the current frame.
    fn find_spawn(&self) -> Option<scene::model::Spawn> {
        let st = self.stream.as_ref()?;
        let mut best: Option<(f64, scene::model::Spawn)> = None;
        for sc in st.scenes.values() {
            if !sc.roads.iter().any(|r| r.width >= 5.0) {
                continue;
            }
            let sp = sc.spawn();
            let d = sp.x.hypot(sp.y);
            if best.as_ref().map_or(true, |b| d < b.0) {
                best = Some((d, sp));
            }
        }
        best.map(|b| b.1)
    }

    fn stream_tick(&mut self) {
        let (cx, cy) = (self.car.pos.x, self.car.pos.y);
        let Some(st) = self.stream.as_mut() else { return };
        let ev = st.streamer.update(cx, cy);
        self.apply_stream_events(ev, true);
        self.terrain_tick();
        let st = self.stream.as_mut().unwrap();
        if st.waiting {
            let center = TileId::from_lat_lon(st.origin_latlon.0, st.origin_latlon.1, ZOOM).key();
            let stats = st.streamer.stats();
            let have_center = st.scenes.contains_key(&center);
            let idle = stats.fetching == 0 && stats.building == 0;
            let gave_up = stats.loaded == 0 && stats.errors > 0 && st.wait_since.elapsed().as_secs() >= 3;
            if have_center || gave_up {
                if let Some(sp) = self.find_spawn() {
                    self.spawn = sp;
                } else if have_center && !idle {
                    return; // neighbouring tiles may still bring a road
                } else {
                    self.spawn = scene::model::Spawn { x: 0.0, y: 0.0, yaw: 0.0 };
                }
                self.place_car(self.spawn.x, self.spawn.y, self.spawn.yaw, 0.0);
                self.driver = Driver::new(self.car.p.steer_max);
                self.cam_yaw = self.spawn.yaw;
                self.stream.as_mut().unwrap().waiting = false;
            }
        }
    }

    pub fn load_scene(&mut self, scene: Scene) {
        self.spawn = scene.spawn();
        self.world = scene.static_world();
        self.meshes = scene_meshes(&scene);
        self.scene = scene;
        self.scene_dirty = true;
        self.respawn();
    }

    pub fn respawn(&mut self) {
        if let Some(st) = &self.stream {
            let (n, la, lo) = st.home.clone();
            self.teleport(&n, la, lo);
            return;
        }
        self.place_car(self.spawn.x, self.spawn.y, self.spawn.yaw, 0.0);
        self.driver = Driver::new(self.car.p.steer_max);
        self.cam_yaw = self.spawn.yaw;
    }

    /// Put the car back on its wheels where it is, pointing the same way, stopped.
    pub fn recover(&mut self) {
        let f = self.car.forward();
        let yaw = f.y.atan2(f.x);
        let (x, y) = (self.car.pos.x, self.car.pos.y);
        self.place_car(x, y, yaw, 0.0);
        self.driver = Driver::new(self.car.p.steer_max);
    }

    /// Held-key state. Returns true if the key was handled.
    pub fn key_state(&mut self, k: Key, down: bool) {
        match k {
            Key::Up => self.keys.up = down,
            Key::Down => self.keys.down = down,
            Key::Left => self.keys.left = down,
            Key::Right => self.keys.right = down,
            Key::Handbrake => self.keys.handbrake = down,
            _ if !down => {}
            Key::Camera => self.cam_mode = (self.cam_mode + 1) % 3,
            Key::HourUp => self.hour = (self.hour + 1.0) % 24.0,
            Key::HourDown => self.hour = (self.hour + 23.0) % 24.0,
            Key::Lights => {
                self.headlights = match self.headlights {
                    None => Some(true),
                    Some(true) => Some(false),
                    Some(false) => None,
                }
            }
            Key::SoftBody => {
                let f = self.car.forward();
                let yaw = f.y.atan2(f.x);
                let speed = self.car.telemetry().forward_speed;
                let (x, y) = (self.car.pos.x, self.car.pos.y);
                self.soft_on = !self.soft_on;
                self.place_car(x, y, yaw, speed);
                self.driver = Driver::new(self.car.p.steer_max);
            }
            Key::Respawn => self.respawn(),
            Key::Teleport => {
                if let Some(st) = self.stream.as_mut() {
                    st.preset = st.preset.wrapping_add(1) % PRESETS.len();
                    let (n, la, lo) = PRESETS[st.preset];
                    self.teleport(n, la, lo);
                }
            }
            Key::Recover => self.recover(),
            Key::Hud => self.show_hud = !self.show_hud,
            Key::Tcs => self.driver.tcs = !self.driver.tcs,
            Key::Abs => self.driver.abs = !self.driver.abs,
            Key::Help => self.show_help = !self.show_help,
        }
    }

    pub fn release_all(&mut self) {
        self.keys = Keys::default();
    }

    /// Advance the simulation by `dt` real seconds using fixed 1 ms physics steps.
    pub fn update(&mut self, dt: f64) {
        let dt = dt.clamp(0.0, 0.05);
        self.stream_tick();
        let frozen = self.stream.as_ref().map_or(false, |s| s.waiting);
        if frozen {
            self.acc = 0.0;
        } else {
            self.acc += dt;
        }
        let mut n = 0;
        if let (true, Some(j)) = (self.soft_on, self.jcar.as_ref()) {
            // the JBeam car runs on its own thread: hand it the controls, or step it here when not threaded
            let mut s = j.lock();
            s.keys = self.keys;
            s.frozen = frozen;
            if !j.is_threaded() {
                let steps = (self.acc / PHYS_DT) as usize;
                s.step_n(steps);
                self.acc -= steps as f64 * PHYS_DT;
                n = steps as u32;
            } else {
                self.acc = 0.0;
                n = 1;
            }
        }
        while self.acc >= PHYS_DT {
            if self.soft_on {
                let fwd_speed = self.soft.vel.dot(self.soft.forward());
                let inp = self.driver.update(PHYS_DT, &self.keys, fwd_speed);
                self.soft.step(PHYS_DT, &inp, &self.ground, &self.world);
            } else {
                let fwd_speed = self.car.telemetry().forward_speed;
                let inp = self.driver.update(PHYS_DT, &self.keys, fwd_speed);
                self.car.step(PHYS_DT, &inp, &self.ground, &self.world);
            }
            self.acc -= PHYS_DT;
            n += 1;
        }
        if n > 0 {
            self.sync_from_soft_if_on();
        }
        self.sim_steps_last_frame = n;
        if !frozen && dt > 0.0 {
            self.traffic_tick(dt);
        }
        // camera follows the heading with a lag
        let f = self.car.forward();
        let yaw = f.y.atan2(f.x);
        let mut d = yaw - self.cam_yaw;
        d = d.sin().atan2(d.cos());
        self.cam_yaw += d * (dt * 3.2).min(1.0);
        // fps
        self.fps_acc += dt as f32;
        self.fps_n += 1;
        if self.fps_acc > 0.5 {
            self.fps = self.fps_n as f32 / self.fps_acc;
            self.fps_acc = 0.0;
            self.fps_n = 0;
        }
    }

    fn headlights_on(&self, day: f32) -> bool {
        self.headlights.unwrap_or(day < 0.6)
    }

    pub fn frame_params(&self) -> FrameParams {
        let car = &self.car;
        let t = car.telemetry();
        let spd = t.speed;
        let cp = car.pos;
        let f = car.forward();
        let yaw = f.y.atan2(f.x);
        let (cy, sy) = (yaw.cos(), yaw.sin());
        // camera heights are above the ground under the car
        let gz = self.ground.field.height(cp.x, cp.y);
        let (eye, target, fov): (V3f, V3f, f32) = match self.cam_mode {
            0 => {
                let d = 7.5 + spd * 0.06;
                (
                    [(cp.x - self.cam_yaw.cos() * d) as f32, (cp.y - self.cam_yaw.sin() * d) as f32, (gz + 2.6 + spd * 0.012) as f32],
                    [(cp.x + cy * 3.0) as f32, (cp.y + sy * 3.0) as f32, (gz + 1.2) as f32],
                    (62.0 + spd * 0.25) as f32,
                )
            }
            1 => {
                let e = [(cp.x + cy * 0.25 - sy * 0.37) as f32, (cp.y + sy * 0.25 + cy * 0.37) as f32, (gz + 1.2) as f32];
                ([e[0], e[1], e[2]], [e[0] + cy as f32 * 20.0, e[1] + sy as f32 * 20.0, e[2] - 0.1], 75.0)
            }
            _ => (
                [(cp.x - self.cam_yaw.cos() * 25.0) as f32, (cp.y - self.cam_yaw.sin() * 25.0) as f32, (gz + 60.0) as f32],
                [cp.x as f32, cp.y as f32, gz as f32],
                55.0,
            ),
        };
        let mut up: V3f = [0.0, 0.0, 1.0];
        let mut interior = false;
        let (mut eye, mut target, mut fov) = (eye, target, fov);
        // interior view of a mod car: the driver's eye from the mod, moving and tilting with the body
        if let (1, true, Some(jc)) = (self.cam_mode, self.soft_on, self.jcar.as_ref()) {
            let (e, r, f) = jc.lock().v.eye_pose();
            let fwd = r.rotate(phys::math::v3(1.0, 0.0, 0.0));
            let u = r.rotate(phys::math::v3(0.0, 0.0, 1.0));
            // look slightly down at the road like a seated driver
            let dir = (fwd - u * 0.06).norm();
            eye = [e.x as f32, e.y as f32, e.z as f32];
            target = [(e.x + dir.x * 10.0) as f32, (e.y + dir.y * 10.0) as f32, (e.z + dir.z * 10.0) as f32];
            up = [u.x as f32, u.y as f32, u.z as f32];
            fov = (f as f32 + spd as f32 * 0.08).min(f as f32 + 10.0);
            interior = true;
        }
        let (eye, target, fov) = self.cam_override.unwrap_or((eye, target, fov));
        let interior = interior && self.cam_override.is_none();
        let body = m4::mul(&m4::from_quat([car.q.w, car.q.x, car.q.y, car.q.z], [cp.x, cp.y, cp.z]), &m4::translate(0.0, 0.0, -CG_HEIGHT as f32));
        let mut wheels = [m4::id(); 4];
        if let (true, Some(jc)) = (self.soft_on, self.jcar.as_ref()) {
            let guard = jc.lock();
            let j = &guard.v;
            let (_, _, up) = j.axes();
            for (slot, w) in wheels.iter_mut().enumerate() {
                let Some(i) = self.jcar_map[slot] else { continue };
                let c = j.wheel_center(i);
                let y = j.wheel_axis(i);
                let x = y.cross(up).norm();
                let z = x.cross(y);
                let basis: m4::M4 = [
                    x.x as f32, x.y as f32, x.z as f32, 0.0, y.x as f32, y.y as f32, y.z as f32, 0.0, z.x as f32, z.y as f32, z.z as f32, 0.0, c.x as f32, c.y as f32, c.z as f32, 1.0,
                ];
                *w = m4::mul(&basis, &m4::rot_y(j.wheels[i].state.spin as f32));
            }
        } else {
        for (i, w) in wheels.iter_mut().enumerate() {
            let c = car.wheel_center(i);
            let wl = &car.wheels[i];
            let r = m4::from_quat([car.q.w, car.q.x, car.q.y, car.q.z], [c.x, c.y, c.z]);
            *w = m4::mul(&m4::mul(&r, &m4::rot_z(wl.steer as f32)), &m4::rot_y(wl.spin as f32));
        }
        }
        let lg = crate::env::lighting(self.hour);
        let braking = self.driver.brake > 0.2 || self.driver.handbrake > 0.0;
        FrameParams {
            eye,
            target,
            fov_deg: fov,
            up: if interior { up } else { [0.0, 0.0, 1.0] },
            interior,
            body,
            wheels,
            // first person: the textured mod car is drawn (dashboard, pillars); the plain car is hidden
            draw_car: self.cam_mode != 1 || (interior && self.jcar_has_visual),
            car_pos: [cp.x as f32, cp.y as f32, cp.z as f32],
            head_pos: [(cp.x + f.x * 2.3) as f32, (cp.y + f.y * 2.3) as f32, (cp.z + f.z * 2.3 + 0.2) as f32],
            head_dir: [f.x as f32, f.y as f32, f.z as f32 - 0.05],
            headlights: self.headlights_on(lg.day),
            braking,
            hour: self.hour,
            paint: [0.42, 0.04, 0.05],
        }
    }

    pub fn hud(&self) -> HudData {
        let t = self.car.telemetry();
        let m = &self.scene.meta;
        let mut lines = vec![
            if self.draws > 0 {
                format!("FPS {:.0}   DRAWS {}   TRIS {}K   TIME {:02}:00", self.fps, self.draws, self.triangles / 1000, self.hour as i32)
            } else {
                format!("FPS {:.0}   {}   TIME {:02}:00", self.fps, self.renderer_info, self.hour as i32)
            },
            format!("TCS {}   ABS {}   SIDESLIP {:.0} DEG   LAT {:.2} G", on_off(self.driver.tcs), on_off(self.driver.abs), t.side_slip.to_degrees().abs(), t.lat_g.abs()),
            format!("ROADS {}   BUILDINGS {}   TREES {}", m.n_roads, m.n_buildings, m.n_trees),
        ];
        let st: Vec<String> = m.status_counts.iter().map(|(k, v)| format!("{k} X{v}")).collect();
        if !st.is_empty() {
            lines.push(format!("BUILDINGS: {}", st.join(", ")));
        }
        lines.push(format!("TERRAIN: {}   FACADES: PROCEDURAL WINDOWS (NOT RECONSTRUCTION)", if m.terrain.is_empty() { "FLAT" } else { &m.terrain }));
        if let Some(st) = &self.stream {
            let ss = st.streamer.stats();
            let (la, lo, _) = st.streamer.frame().from_local(self.car.pos.x, self.car.pos.y, 0.0);
            lines.push(format!("{}   LAT {:.5}  LON {:.5}", st.place, la, lo));
            lines.push(format!(
                "TILES LOADED {}  FETCHING {}  BUILDING {}  ERRORS {}   NET {}  CACHE {}  {} KB",
                ss.loaded, ss.fetching, ss.building, ss.errors, ss.net_requests, ss.cache_hits, ss.bytes / 1024
            ));
            if st.waiting {
                lines.push(format!("LOADING THE MAP AROUND {} - CAR HELD UNTIL IT ARRIVES", st.place));
            }
            if let Some(e) = &ss.last_error {
                let e: String = e.chars().take(110).collect();
                lines.push(format!("LAST ERROR: {}", e.to_uppercase()));
            }
        }
        if let (true, Some(jc)) = (self.soft_on, self.jcar.as_ref()) {
            lines.extend(self.jcar_info.iter().cloned());
            let (dmg, broken, loose, groups) = {
                let j = jc.lock();
                let (d, b) = j.v.damage();
                (d, b, j.v.detached_nodes(), j.v.groups.triggered.iter().filter(|t| **t).count() + j.v.groups.broken_group_count())
            };
            lines.push(format!("PHYSICS: SOFT-BODY FROM JBEAM   BODY DAMAGE {:.0} CM   BROKEN BEAMS {}   (F3 = BUILT-IN RIGID CAR)", dmg * 100.0, broken));
            if loose > 0 || groups > 0 {
                lines.push(format!("DAMAGE: {groups} PARTS BROKEN/CRACKED, {loose} NODES TORN OFF"));
            }
        } else if self.soft_on {
            let (dmg, broken) = self.soft.damage();
            lines.push(format!("PHYSICS: SOFT-BODY NODE-BEAM ({} NODES, {} BEAMS)   BODY DAMAGE {:.0} CM   BROKEN BEAMS {}", self.soft.body.nodes.len(), self.soft.body.beams.len(), dmg * 100.0, broken));
        } else {
            lines.push("PHYSICS: RIGID BODY (F3 = SOFT-BODY)".to_string());
        }
        let locked: Vec<&str> = ["FL", "FR", "RL", "RR"].iter().enumerate().filter(|(i, _)| self.car.wheels[*i].locked).map(|(_, n)| *n).collect();
        if !locked.is_empty() {
            lines.push(format!("WHEELS LOCKED: {}", locked.join(" ")));
        }
        if self.keys.handbrake {
            lines.push("HANDBRAKE".to_string());
        }
        if self.car.impact > 500.0 {
            lines.push("COLLISION".to_string());
        }
        let help = if self.show_help {
            "W/S THROTTLE + BRAKE/REVERSE   A/D STEER   SPACE HANDBRAKE\nC CAMERA (CHASE / DRIVER / TOP)   M MAP + TELEPORT   [ ] TIME OF DAY   L LIGHTS   R RESPAWN+REPAIR   U RECOVER+REPAIR\nT TCS   B ABS   H HUD   F1 HELP   F2 TELEPORT   F3 SOFT/RIGID   F4 CONTROLLERS   F5 GRAPHICS".to_string()
        } else {
            String::new()
        };
        HudData {
            speed_kmh: (t.forward_speed.abs() * 3.6) as f32,
            gear: t.gear,
            rpm: t.rpm as f32,
            rpm_max: self.car.p.drive.max_rpm as f32,
            lines,
            warn: match &m.source {
                Some(s) => format!("DATA: {s}"),
                None => "SYNTHETIC DEMO CITY - NOT REAL DATA".to_string(),
            },
            warn_is_demo: m.source.is_none() && self.stream.is_none(),
            help,
            show: self.show_hud,
        }
    }
}

/// Build the game for the chosen mode. `live` is the network client (only the Windows front end has one).
pub fn build_game(o: &crate::cli::Options, live: Option<Arc<dyn world::fetch::Fetcher>>, cache_dir: std::path::PathBuf) -> Result<Game, String> {
    use crate::cli::Mode;
    use world::cache::DiskCache;
    use world::fetch::VirtualOsm;
    use world::streamer::StreamConfig;
    const DEMO_SCENE: &str = include_str!("../../../data/demo_scene.json");
    const DEMO_OVERPASS: &str = include_str!("../../../data/demo_overpass.json");
    let name = if (o.lat, o.lon) == crate::cli::MINSK { "MINSK" } else { "CUSTOM PLACE" };
    let cache = || Some(DiskCache::new(&cache_dir, std::time::Duration::from_secs(30 * 24 * 3600)));
    match &o.mode {
        Mode::Demo => Ok(Game::new(Scene::from_json_str(DEMO_SCENE)?)),
        Mode::SceneFile(p) => {
            let text = std::fs::read_to_string(p).map_err(|e| format!("cannot read '{p}': {e}"))?;
            Ok(Game::new(Scene::from_json_str(&text).map_err(|e| format!("'{p}': {e}"))?))
        }
        Mode::Offline => {
            // EARTHSIM_OFFLINE_OSM=<overpass json> replaces the bundled demo data (tests, own extracts)
            let text = std::env::var("EARTHSIM_OFFLINE_OSM").ok().and_then(|p| std::fs::read_to_string(p).ok());
            let v = Arc::new(VirtualOsm::from_overpass_json(text.as_deref().unwrap_or(DEMO_OVERPASS))?);
            let mut cfg = StreamConfig::default();
            cfg.min_request_interval = std::time::Duration::ZERO;
            let mut g = Game::new_streaming(Streamer::new(cfg, v, None, o.lat, o.lon), name, o.lat, o.lon);
            // EARTHSIM_DEM_DIR=<folder of z/x/y.png terrarium tiles>, or EARTHSIM_DEM_SLOPE=<m per pixel> (synthetic)
            if let Ok(d) = std::env::var("EARTHSIM_DEM_DIR") {
                g.set_terrain_source(Arc::new(world::dem::DirDem::<world::dem::NoDem> { dir: d.into(), inner: None }));
            } else if let Some(sl) = std::env::var("EARTHSIM_DEM_SLOPE").ok().and_then(|v| v.parse().ok()) {
                g.set_terrain_source(Arc::new(world::dem::SyntheticDem { base: 200.0, slope: sl }));
            }
            Ok(g)
        }
        Mode::Live => {
            let f = live.ok_or("live streaming needs the Windows network client; use --offline or --demo here")?;
            #[allow(unused_mut)]
            let mut g = Game::new_streaming(Streamer::new(StreamConfig::default(), f, cache(), o.lat, o.lon), name, o.lat, o.lon);
            // terrain: EARTHSIM_TERRAIN=0 keeps the world flat
            #[cfg(any(windows, earthsim_check_win))]
            if std::env::var("EARTHSIM_TERRAIN").map_or(true, |v| v != "0") {
                g.set_terrain_source(world::fetch::live_dem(&cache_dir.join("dem")));
            }
            Ok(g)
        }
    }
    .map(|mut g| {
        if let Some(path) = &o.car {
            let p = std::path::Path::new(path);
            match jbeam::load_vehicle(p, o.car_part.as_deref()) {
                Ok(lv) => {
                    let name = p.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
                    world::log::log(&format!("car mod {path}: main part {}, {} nodes, {} beams", lv.main_part, lv.nodes, lv.beams));
                    for n in &lv.notes {
                        world::log::log(&format!("  {n}"));
                    }
                    g.set_jbeam_car(lv, &name);
                }
                Err(e) => {
                    world::log::log(&format!("car mod {path} could not be loaded: {e}"));
                    g.jcar_info = vec![format!("CAR MOD NOT LOADED: {}", e.to_uppercase())];
                }
            }
        }
        g
    })
}

fn on_off(b: bool) -> &'static str {
    if b { "ON" } else { "OFF" }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn game() -> Game {
        let s = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/demo_scene.json")).unwrap();
        Game::new(Scene::from_json_str(&s).unwrap())
    }

    #[test]
    fn throttle_moves_the_car_and_the_hud_reports_it() {
        let mut g = game();
        g.key_state(Key::Up, true);
        for _ in 0..300 {
            g.update(1.0 / 60.0);
        }
        let h = g.hud();
        assert!(h.speed_kmh > 20.0, "speed {}", h.speed_kmh);
        assert!(h.gear >= 1 && h.rpm > 1000.0);
        assert!(h.warn.contains("SYNTHETIC"));
        assert!(g.sim_steps_last_frame >= 16);
    }

    #[test]
    fn a_long_frame_does_not_stall_the_game_or_explode() {
        let mut g = game();
        g.key_state(Key::Up, true);
        g.update(5.0); // clamped to 50 ms
        assert!(g.sim_steps_last_frame <= 51);
    }

    #[test]
    fn a_jbeam_car_drives_in_the_game_and_shows_its_body() {
        let mut g = game();
        let mut lib = jbeam::assemble::Library::default();
        lib.add_file("car.jbeam", &jbeam::vehicle::synthetic_jbeam());
        let lv = jbeam::load_from_library(&lib, None).unwrap();
        g.set_jbeam_car(lv, "test");
        assert!(g.jcar.is_some() && g.jcar_map.iter().all(|m| m.is_some()));
        for _ in 0..120 {
            g.update(1.0 / 60.0);
        }
        g.key_state(Key::Up, true);
        for _ in 0..300 {
            g.update(1.0 / 60.0);
        }
        let h = g.hud();
        assert!(h.speed_kmh > 20.0, "speed {}", h.speed_kmh);
        assert!(h.lines.iter().any(|l| l.contains("FROM MOD")));
        let body = g.deformed_body().expect("mod body mesh");
        assert_eq!(body.len() % scene::mesh::FLOATS_PER_VERTEX, 0);
        // wheels are drawn where the mod's axles are
        let fp = g.frame_params();
        let c = g.jcar.as_ref().unwrap().lock().v.wheel_center(g.jcar_map[0].unwrap());
        assert!((fp.wheels[0][12] as f64 - c.x).abs() < 1e-3 && (fp.wheels[0][14] as f64 - c.z).abs() < 1e-3);
        // interior camera: at the mod car's eye, looking forward along the body
        g.cam_mode = 1;
        let fp = g.frame_params();
        assert!(fp.interior);
        let (e, r, _) = g.jcar.as_ref().unwrap().lock().v.eye_pose();
        assert!((fp.eye[0] as f64 - e.x).abs() < 1e-3 && (fp.eye[2] as f64 - e.z).abs() < 1e-3);
        let f = r.rotate(phys::v3(1.0, 0.0, 0.0));
        let d = [fp.target[0] - fp.eye[0], fp.target[1] - fp.eye[1]];
        assert!((d[0] as f64 * f.x + d[1] as f64 * f.y) > 9.0, "interior camera looks forward");
        assert!(fp.up[2] > 0.95);
        g.cam_mode = 0;
        // F3 switches to the built-in car and back
        g.key_state(Key::SoftBody, true);
        assert!(!g.soft_on && g.deformed_body().is_none());
    }

    #[test]
    fn a_threaded_jbeam_car_moves_in_real_time() {
        let mut g = game();
        let mut lib = jbeam::assemble::Library::default();
        lib.add_file("car.jbeam", &jbeam::vehicle::synthetic_jbeam());
        g.set_jbeam_car(jbeam::load_from_library(&lib, None).unwrap(), "test");
        g.set_threaded_physics(true);
        assert!(g.jcar.as_ref().unwrap().is_threaded());
        let s0 = g.jcar.as_ref().unwrap().lock().steps;
        std::thread::sleep(std::time::Duration::from_millis(300));
        g.update(0.3);
        let s1 = g.jcar.as_ref().unwrap().lock().steps;
        // about 300 physics steps of 1 ms in 0.3 s of wall time (allow a slow test machine)
        assert!(s1 - s0 > 100 && s1 - s0 < 400, "{} steps", s1 - s0);
        g.key_state(Key::Up, true);
        g.update(0.0);
        std::thread::sleep(std::time::Duration::from_millis(1500));
        g.update(0.016);
        assert!(g.hud().speed_kmh > 5.0, "speed {}", g.hud().speed_kmh);
    }

    #[test]
    fn respawn_and_recover() {
        let mut g = game();
        g.key_state(Key::Up, true);
        for _ in 0..200 {
            g.update(1.0 / 60.0);
        }
        g.key_state(Key::Up, false);
        let moved = g.car.pos.x;
        g.key_state(Key::Respawn, true);
        assert!(g.car.telemetry().speed < 1e-6);
        let _ = moved;
        // flip the car over, then recover
        g.car.q = phys::Quat::axis_angle(phys::V3::X, 3.0);
        g.key_state(Key::Recover, true);
        assert!(g.car.q.rotate(phys::V3::Z).z > 0.99);
    }

    #[test]
    fn cycling_options() {
        let mut g = game();
        for _ in 0..3 {
            g.key_state(Key::Camera, true);
        }
        assert_eq!(g.cam_mode, 0);
        g.key_state(Key::HourDown, true);
        assert_eq!(g.hour, 14.0);
        g.hour = 0.0;
        g.key_state(Key::HourDown, true);
        assert_eq!(g.hour, 23.0);
        g.key_state(Key::Lights, true);
        assert_eq!(g.headlights, Some(true));
        g.key_state(Key::Lights, true);
        g.key_state(Key::Lights, true);
        assert_eq!(g.headlights, None);
    }

    #[test]
    fn frame_params_are_finite_in_every_camera() {
        let mut g = game();
        g.key_state(Key::Up, true);
        for _ in 0..120 {
            g.update(1.0 / 60.0);
        }
        for mode in 0..3 {
            g.cam_mode = mode;
            let f = g.frame_params();
            let all: Vec<f32> = f.eye.iter().chain(&f.target).chain(&f.body).chain(f.wheels.iter().flatten()).copied().collect();
            assert!(all.iter().all(|v| v.is_finite()));
        }
        assert!(!g.frame_params().draw_car || g.cam_mode != 1);
    }

    #[test]
    fn handbrake_drift_does_not_bring_the_car_to_a_stop() {
        // The reported bug: holding the handbrake eventually stopped the car even with power applied.
        // Open ground (no buildings) so the test measures the physics, not a collision.
        let mut g = Game::new(Scene::default());
        g.key_state(Key::Up, true);
        for _ in 0..360 {
            g.update(1.0 / 60.0);
        }
        g.key_state(Key::Handbrake, true);
        g.key_state(Key::Left, true);
        for _ in 0..90 {
            g.update(1.0 / 60.0); // kick the tail out
        }
        g.key_state(Key::Left, false);
        for _ in 0..(60 * 15) {
            g.update(1.0 / 60.0); // keep throttle and handbrake held
        }
        assert!(g.car.telemetry().speed > 3.0, "stalled at {} m/s", g.car.telemetry().speed);
    }

    #[test]
    fn the_car_drives_up_onto_a_bridge_deck_and_the_deck_mesh_matches_it() {
        use phys::Ground;
        use scene::model::{Area, AreaKind, Road};
        let road = |pts: Vec<[f64; 2]>, bridge: bool| Road { cls: "primary".into(), width: 8.0, junction: vec![false; pts.len()], pts, bridge };
        let mut sc = Scene::default();
        sc.roads.push(road((0..=20).map(|i| [-200.0 + i as f64 * 10.0, 0.0]).collect(), false));
        sc.roads.push(road((0..=20).map(|i| [i as f64 * 10.0, 0.0]).collect(), true));
        sc.roads.push(road((0..=20).map(|i| [200.0 + i as f64 * 10.0, 0.0]).collect(), false));
        sc.areas.push(Area { kind: AreaKind::Water, ring: vec![[40.0, -50.0], [160.0, -50.0], [160.0, 50.0], [40.0, 50.0]], tris: vec![[0, 1, 2], [0, 2, 3]] });
        let mut g = Game::new(sc);
        let top = scene::bridge::rise(&g.scene, &g.scene.roads[1])[10] + 0.02;
        assert!((top - 2.52).abs() < 1e-6, "{top}");
        // physics deck
        assert!((g.ground.sample_at(100.0, 1.0, 5.0).height - top).abs() < 1e-6);
        assert!(g.ground.sample_at(100.0, 1.0, 0.3).height.abs() < 1e-9, "under the deck: the ground");
        // the drawn deck: carriageway vertices near the middle at the same height
        let m = g.meshes.iter().find(|m| m.name.starts_with("bridge|")).expect("bridge mesh");
        let z_mid = m.verts.chunks(scene::mesh::FLOATS_PER_VERTEX).filter(|v| v[8] == scene::mesh::mat::ASPHALT && (v[0] - 100.0).abs() < 0.5).map(|v| v[2]).fold(f32::MIN, f32::max);
        assert!((z_mid as f64 - top).abs() < 1e-3, "mesh {z_mid} vs deck {top}");
        // drive over it
        g.place_car(-60.0, 0.0, 0.0, 12.0);
        g.key_state(Key::Up, true);
        let mut peak: f64 = 0.0;
        for _ in 0..(60 * 20) {
            g.update(1.0 / 60.0);
            let p = if g.soft_on { g.soft.pos } else { g.car.pos };
            if (80.0..120.0).contains(&p.x) {
                peak = peak.max(p.z);
            }
            if p.x > 260.0 {
                break;
            }
        }
        let p = if g.soft_on { g.soft.pos } else { g.car.pos };
        assert!(p.x > 200.0, "crossed the bridge: x {}", p.x);
        assert!(peak > top + 0.2, "rode on the deck: peak z {peak}, deck {top}");
    }

    // ---------- streaming ----------
    fn offline_game() -> Game {
        let o = crate::cli::Options { mode: crate::cli::Mode::Offline, lat: crate::cli::MINSK.0, lon: crate::cli::MINSK.1, cache: None, car: None, car_part: None };
        build_game(&o, None, std::env::temp_dir()).unwrap()
    }

    fn run_until(g: &mut Game, secs: f64, mut f: impl FnMut(&Game) -> bool) -> bool {
        let t0 = std::time::Instant::now();
        while t0.elapsed().as_secs_f64() < secs {
            g.update(1.0 / 60.0);
            if f(g) {
                return true;
            }
            std::thread::sleep(std::time::Duration::from_millis(3));
        }
        false
    }

    fn settled(g: &Game) -> bool {
        let st = g.stream.as_ref().unwrap();
        let ss = st.streamer.stats();
        !st.waiting && ss.loaded >= 9 && ss.fetching == 0 && ss.building == 0
    }

    #[test]
    fn streaming_game_holds_the_car_then_places_it_on_a_road() {
        let mut g = offline_game();
        assert!(g.is_streaming());
        assert!(g.hud().lines.iter().any(|l| l.contains("LOADING")), "{:?}", g.hud().lines);
        assert!(run_until(&mut g, 15.0, |g| !g.hud().lines.iter().any(|l| l.contains("LOADING"))), "never left the loading state");
        let ops = g.take_render_ops();
        let adds = ops.iter().filter(|o| matches!(o, RenderOp::AddTile(..))).count();
        assert!(adds >= 1 && ops.iter().any(|o| matches!(o, RenderOp::FollowGround(true))));
        assert!(g.world.poly_count() > 0 && g.scene.meta.n_buildings > 0, "collision world should hold the loaded buildings");
        // the car sits on a wide road and can drive
        g.key_state(Key::Up, true);
        for _ in 0..180 {
            g.update(1.0 / 60.0);
        }
        assert!(g.car.telemetry().speed > 3.0);
        let h = g.hud();
        assert!(h.warn.contains("OPENSTREETMAP") && !h.warn_is_demo);
        assert!(h.lines.iter().any(|l| l.starts_with("TILES LOADED")));
    }

    #[test]
    fn teleport_swaps_the_world_and_freezes_the_car_until_it_is_ready() {
        let mut g = offline_game();
        assert!(run_until(&mut g, 15.0, settled));
        g.take_render_ops();
        let polys = g.world.poly_count();
        assert!(polys > 0);
        g.key_state(Key::Teleport, true); // first preset (Minsk) again: same data, must reload from scratch
        let ops = g.take_render_ops();
        assert!(ops.iter().any(|o| matches!(o, RenderOp::RemoveTile(_))));
        assert_eq!(g.world.poly_count(), 0, "old collision data must be gone immediately");
        let before = g.car.pos;
        g.update(0.016);
        assert!((g.car.pos - before).len() < 1e-9, "car must not move while waiting");
        assert!(run_until(&mut g, 15.0, settled));
        assert_eq!(g.world.poly_count(), polys, "same place must give the same collision world");
        let (la, lo, _) = g.car_geo().unwrap();
        assert!((la - crate::cli::MINSK.0).abs() < 0.02 && (lo - crate::cli::MINSK.1).abs() < 0.02, "car at {la},{lo}");
        // teleport to map coordinates (as the map screen does)
        g.teleport("53.95000, 27.70000", 53.95, 27.70);
        let (la, lo, _) = g.car_geo().unwrap();
        assert!((la - 53.95).abs() < 1e-4 && (lo - 27.70).abs() < 1e-4, "car at {la},{lo} after teleport");
        assert_eq!(g.place_name(), Some("53.95000, 27.70000"));
    }

    #[test]
    fn terrain_lifts_ground_car_and_tiles() {
        let mut g = offline_game();
        g.set_terrain_source(Arc::new(world::dem::SyntheticDem { base: 200.0, slope: 0.5 }));
        assert!(run_until(&mut g, 20.0, |g| g.terrain.generation > 0 && !g.hud().lines.iter().any(|l| l.contains("LOADING"))), "terrain never arrived");
        let f = g.ground.field.clone();
        assert!(!f.is_empty());
        // datum = ground at the origin; the synthetic slope rises eastwards
        assert!(f.height(0.0, 0.0).abs() < 1.0 && f.height(500.0, 0.0) > f.height(-500.0, 0.0) + 5.0);
        // the car stands on the terrain
        for _ in 0..120 {
            g.update(1.0 / 60.0);
        }
        let ground = f.height(g.car.pos.x, g.car.pos.y);
        assert!((g.car.pos.z - ground) > 0.1 && (g.car.pos.z - ground) < 1.5, "car z {} over ground {}", g.car.pos.z, ground);
        // tiles are re-draped onto it: a road vertex sits at the ground height under it
        let mut ok = false;
        for _ in 0..200 {
            g.update(1.0 / 60.0);
            for op in g.take_render_ops() {
                if let RenderOp::AddTile(_, ms) = op {
                    if let Some(m) = ms.iter().find(|m| m.name.starts_with("road") && !m.verts.is_empty()) {
                        let v = &m.verts[..3];
                        ok |= (v[2] as f64 - f.height(v[0] as f64, v[1] as f64) - 0.02).abs() < 0.05;
                    }
                }
            }
        }
        assert!(ok, "no draped road seen");
    }

    #[test]
    fn driving_far_rebases_without_teleporting_the_car() {
        let mut g = offline_game();
        assert!(run_until(&mut g, 15.0, |g| !g.hud().lines.iter().any(|l| l.contains("LOADING"))));
        let (la0, lo0, _) = g.stream.as_ref().unwrap().streamer.frame().from_local(g.car.pos.x, g.car.pos.y, 0.0);
        // push the car 4.5 km east (beyond the rebase distance) as a physics-free shift
        let fr = g.stream.as_ref().unwrap().streamer.frame();
        let p = fr.to_local(la0, lo0, 0.0);
        let _ = p;
        g.car.pos.x += 4500.0;
        g.soft.pos.x += 4500.0;
        for n in g.soft.body.nodes.iter_mut() {
            n.p.x += 4500.0;
        }
        let (la1, lo1, _) = fr.from_local(g.car.pos.x, g.car.pos.y, 0.0);
        g.update(0.016);
        let (la2, lo2, _) = g.stream.as_ref().unwrap().streamer.frame().from_local(g.car.pos.x, g.car.pos.y, 0.0);
        assert!((la1 - la2).abs() < 1e-7 && (lo1 - lo2).abs() < 1e-7, "geographic position must be preserved");
        assert!(g.car.pos.x.abs() < 50.0, "local coordinates restart near zero, got {}", g.car.pos.x);
    }

    // ---------- soft-body in the game ----------
    #[test]
    fn soft_body_is_the_default_and_f3_switches_models_keeping_pose_and_speed() {
        let mut g = Game::new(Scene::default());
        assert!(g.soft_on && g.deformed_body().is_none() == g.deformed.is_empty());
        g.key_state(Key::Up, true);
        for _ in 0..240 {
            g.update(1.0 / 60.0);
        }
        assert!(g.deformed_body().is_some());
        let (p, v) = (g.car.pos, g.car.telemetry().forward_speed);
        assert!(v > 5.0);
        g.key_state(Key::SoftBody, true);
        assert!(!g.soft_on && g.deformed_body().is_none());
        assert!((g.car.pos - p).len() < 0.3, "pose jumped {:?}", (g.car.pos - p).len());
        assert!((g.car.telemetry().forward_speed - v).abs() < 0.5);
        assert!(g.hud().lines.iter().any(|l| l.contains("RIGID")));
        for _ in 0..120 {
            g.update(1.0 / 60.0);
        }
        g.key_state(Key::SoftBody, true);
        assert!(g.soft_on && g.hud().lines.iter().any(|l| l.contains("SOFT-BODY")));
        assert!(g.car.telemetry().speed > 5.0, "should still be rolling");
    }

    #[test]
    fn driving_into_a_building_deforms_the_car_and_the_hud_reports_it_until_repaired() {
        let mut g = Game::new(Scene::default());
        g.world.add_poly(vec![(120.0, -20.0), (130.0, -20.0), (130.0, 20.0), (120.0, 20.0)]);
        g.key_state(Key::Up, true);
        let mut hit = false;
        for _ in 0..(60 * 40) {
            g.update(1.0 / 60.0);
            if g.car.impact > 500.0 {
                hit = true;
                break;
            }
        }
        assert!(hit, "never reached the wall, x = {}", g.car.pos.x);
        // hold throttle a moment longer so the nose really loads
        for _ in 0..30 {
            g.update(1.0 / 60.0);
        }
        let (dmg, _) = g.soft.damage();
        let hud = g.hud();
        assert!(dmg > 0.05 || g.soft.body.beams.iter().any(|b| b.broken), "no damage after a crash? {dmg}");
        assert!(hud.lines.iter().any(|l| l.contains("BODY DAMAGE")));
        assert!(g.deformed_body().unwrap().iter().all(|v| v.is_finite()));
        g.key_state(Key::Respawn, true);
        assert!(g.soft.damage().0 < 1e-6, "respawn must repair the car");
    }
}
