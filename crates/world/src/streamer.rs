//! Tile streamer: keeps the tiles around the car loaded, in the background.
//!
//! Per-tile state machine (spec s.cells): `Unrequested -> Fetching -> Building -> Ready`, with `Error` (retry with
//! exponential backoff). A network thread fetches/parses (disk cache first, at most one request at a time with a
//! minimum gap), a build thread turns raw data into meshes in the current local frame, and the main thread only
//! receives finished tiles. The local frame is a floating origin: when the car is far from it the frame is re-based
//! and tiles are rebuilt from the in-memory raw data (no new downloads).
use crate::cache::DiskCache;
use crate::fetch::{FetchError, Fetcher};
use crate::geo::LocalFrame;
use crate::osm::{build_scene, overpass_query, parse_overpass, TileRaw};
use crate::tiles::{TileId, ZOOM};
use scene::{scene_meshes_opts, MeshData, Scene};
use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
pub struct StreamConfig {
    pub zoom: u8,
    /// Tiles overlapping this radius around the car are loaded.
    pub load_radius_m: f64,
    /// Tiles whose centre is further than this are unloaded (hysteresis against flicker at the border).
    pub unload_radius_m: f64,
    /// Re-base the local frame when the car is this far from its origin.
    pub rebase_dist_m: f64,
    /// Minimum gap between two network requests.
    pub min_request_interval: Duration,
    /// How many tile requests may be queued at the network thread.
    pub max_net_queue: usize,
    pub max_build_queue: usize,
    pub backoff_base: Duration,
    pub backoff_max: Duration,
    /// Finished tiles handed to the game per `update` (spreads GPU uploads over frames).
    pub max_adds_per_update: usize,
    /// Extra pause of the network thread after a transient error.
    pub cooldown_after_error: Duration,
}

impl Default for StreamConfig {
    fn default() -> Self {
        StreamConfig {
            zoom: ZOOM,
            load_radius_m: 1500.0,
            unload_radius_m: 2400.0,
            rebase_dist_m: 4000.0,
            min_request_interval: Duration::from_millis(1500),
            max_net_queue: 3,
            max_build_queue: 3,
            backoff_base: Duration::from_secs(5),
            backoff_max: Duration::from_secs(300),
            max_adds_per_update: 2,
            cooldown_after_error: Duration::from_secs(6),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TileState {
    Unrequested,
    Fetching,
    Building,
    Ready,
    Error { attempts: u32, retry_at: Instant },
}

#[derive(Clone)]
pub enum StreamEvent {
    Add { tile: TileId, scene: Arc<Scene>, meshes: Arc<Vec<MeshData>> },
    Remove { tile: TileId },
    /// The local frame changed. Convert any local position with `old.from_local` -> `new.to_local`.
    Rebased { old: LocalFrame, new: LocalFrame },
}

#[derive(Clone, Debug, Default)]
pub struct StreamStats {
    pub loaded: usize,
    pub fetching: usize,
    pub building: usize,
    pub errors: usize,
    pub net_requests: u64,
    pub cache_hits: u64,
    pub bytes: u64,
    pub generation: u64,
    pub last_error: Option<String>,
}

struct NetJob {
    tile: TileId,
}

struct NetResult {
    tile: TileId,
    outcome: Result<Arc<TileRaw>, (String, bool)>, // error message, transient?
    from_cache: bool,
    bytes: usize,
    skipped: bool,
}

struct BuildJob {
    tile: TileId,
    raw: Arc<TileRaw>,
    frame: LocalFrame,
    generation: u64,
}

struct Built {
    tile: TileId,
    generation: u64,
    scene: Arc<Scene>,
    meshes: Arc<Vec<MeshData>>,
}

pub struct Streamer {
    cfg: StreamConfig,
    frame: LocalFrame,
    generation: u64,
    states: HashMap<TileId, TileState>,
    attempts: HashMap<TileId, u32>,
    raw: HashMap<TileId, Arc<TileRaw>>,
    wanted: Arc<Mutex<HashSet<TileId>>>,
    net_tx: Option<Sender<NetJob>>,
    net_rx: Receiver<NetResult>,
    build_tx: Option<Sender<BuildJob>>,
    build_rx: Receiver<Built>,
    in_net: usize,
    in_build: usize,
    ready: VecDeque<Built>,
    loaded: HashSet<TileId>,
    stats: StreamStats,
    shutdown: Arc<AtomicU64>,
}

impl Streamer {
    pub fn new(cfg: StreamConfig, fetcher: Arc<dyn Fetcher>, cache: Option<DiskCache>, lat: f64, lon: f64) -> Streamer {
        let (net_tx, net_job_rx) = channel::<NetJob>();
        let (net_res_tx, net_rx) = channel::<NetResult>();
        let (build_tx, build_job_rx) = channel::<BuildJob>();
        let (build_res_tx, build_rx) = channel::<Built>();
        let wanted = Arc::new(Mutex::new(HashSet::new()));
        let shutdown = Arc::new(AtomicU64::new(0));
        {
            let (wanted, fetcher, cache, cfg, shutdown) = (wanted.clone(), fetcher, cache.map(Arc::new), cfg.clone(), shutdown.clone());
            thread::Builder::new().name("earthsim-net".into()).spawn(move || net_worker(net_job_rx, net_res_tx, fetcher, cache, wanted, cfg, shutdown)).expect("spawn net thread");
        }
        thread::Builder::new().name("earthsim-build".into()).spawn(move || {
            while let Ok(job) = build_job_rx.recv() {
                let scene = build_scene(&job.raw, &job.frame);
                let meshes = scene_meshes_opts(&scene, false);
                if build_res_tx.send(Built { tile: job.tile, generation: job.generation, scene: Arc::new(scene), meshes: Arc::new(meshes) }).is_err() {
                    break;
                }
            }
        }).expect("spawn build thread");
        Streamer {
            cfg,
            frame: LocalFrame::new(lat, lon, 0.0),
            generation: 1,
            states: HashMap::new(),
            attempts: HashMap::new(),
            raw: HashMap::new(),
            wanted,
            net_tx: Some(net_tx),
            net_rx,
            build_tx: Some(build_tx),
            build_rx,
            in_net: 0,
            in_build: 0,
            ready: VecDeque::new(),
            loaded: HashSet::new(),
            stats: StreamStats { generation: 1, ..Default::default() },
            shutdown,
        }
    }

    /// How far around the car the world is kept loaded (graphics quality).
    pub fn set_radius(&mut self, load_m: f64) {
        self.cfg.load_radius_m = load_m.clamp(300.0, 3000.0);
        self.cfg.unload_radius_m = self.cfg.load_radius_m * 1.6;
    }

    pub fn frame(&self) -> LocalFrame {
        self.frame
    }

    pub fn state_of(&self, t: TileId) -> TileState {
        self.states.get(&t).copied().unwrap_or(TileState::Unrequested)
    }

    pub fn loaded_tiles(&self) -> Vec<TileId> {
        let mut v: Vec<_> = self.loaded.iter().copied().collect();
        v.sort();
        v
    }

    pub fn stats(&self) -> StreamStats {
        let mut s = self.stats.clone();
        s.loaded = self.loaded.len();
        s.fetching = self.states.values().filter(|x| matches!(x, TileState::Fetching)).count();
        s.building = self.states.values().filter(|x| matches!(x, TileState::Building)).count();
        s.errors = self.states.values().filter(|x| matches!(x, TileState::Error { .. })).count();
        s.generation = self.generation;
        s
    }

    fn tile_distance(&self, t: TileId, x: f64, y: f64) -> f64 {
        let (la, lo) = t.center();
        let p = self.frame.to_local(la, lo, 0.0);
        (p[0] - x).hypot(p[1] - y)
    }

    /// Move the origin to a new place (teleport) or to the car's current place (re-base). Everything loaded is
    /// removed; raw data stays cached in memory so a re-base needs no downloads.
    pub fn set_origin(&mut self, lat: f64, lon: f64, events: &mut Vec<StreamEvent>) {
        let old = self.frame;
        let new = LocalFrame::new(lat, lon, 0.0);
        for t in self.loaded.drain() {
            events.push(StreamEvent::Remove { tile: t });
        }
        self.ready.clear();
        self.generation += 1;
        self.frame = new;
        for st in self.states.values_mut() {
            if matches!(st, TileState::Ready | TileState::Building) {
                *st = TileState::Unrequested;
            }
        }
        self.in_build = 0; // results of the old generation are discarded when they arrive
        events.push(StreamEvent::Rebased { old, new });
    }

    /// Call every frame with the car position in the current local frame.
    pub fn update(&mut self, car_x: f64, car_y: f64) -> Vec<StreamEvent> {
        let now = Instant::now();
        let mut events = Vec::new();
        let (lat, lon, _) = self.frame.from_local(car_x, car_y, 0.0);

        if car_x.hypot(car_y) > self.cfg.rebase_dist_m {
            self.set_origin(lat, lon, &mut events);
        }
        let (car_x, car_y) = if matches!(events.last(), Some(StreamEvent::Rebased { .. })) { (0.0, 0.0) } else { (car_x, car_y) };

        // ---- collect network results ----
        while let Ok(r) = self.net_rx.try_recv() {
            self.in_net = self.in_net.saturating_sub(1);
            if r.skipped {
                self.states.insert(r.tile, TileState::Unrequested);
                continue;
            }
            if r.from_cache {
                self.stats.cache_hits += 1;
            } else {
                self.stats.net_requests += 1;
            }
            self.stats.bytes += r.bytes as u64;
            match r.outcome {
                Ok(raw) => {
                    self.attempts.remove(&r.tile);
                    self.raw.insert(r.tile, raw);
                    self.states.insert(r.tile, TileState::Unrequested); // picked up for building below
                }
                Err((msg, transient)) => {
                    let n = self.attempts.entry(r.tile).or_default();
                    *n += 1;
                    let backoff = if transient { self.cfg.backoff_base.saturating_mul(1 << (*n - 1).min(10)).min(self.cfg.backoff_max) } else { self.cfg.backoff_max };
                    self.stats.last_error = Some(format!("{}: {}", r.tile.path(), msg));
                    self.states.insert(r.tile, TileState::Error { attempts: *n, retry_at: now + backoff });
                }
            }
        }
        // ---- collect finished builds ----
        while let Ok(b) = self.build_rx.try_recv() {
            self.in_build = self.in_build.saturating_sub(1);
            if b.generation == self.generation {
                self.ready.push_back(b);
            }
        }

        // ---- decide what we want ----
        let desired = TileId::around(lat_of(&self.frame, car_x, car_y), lon_of(&self.frame, car_x, car_y), self.cfg.load_radius_m, self.cfg.zoom);
        {
            let mut w = self.wanted.lock().unwrap();
            w.clear();
            w.extend(desired.iter().copied());
        }
        for t in &desired {
            let st = self.state_of(*t);
            let st = match st {
                TileState::Error { retry_at, .. } if retry_at <= now => {
                    self.states.insert(*t, TileState::Unrequested);
                    TileState::Unrequested
                }
                s => s,
            };
            if st != TileState::Unrequested {
                continue;
            }
            if let Some(raw) = self.raw.get(t).cloned() {
                if self.in_build < self.cfg.max_build_queue {
                    if let Some(tx) = &self.build_tx {
                        if tx.send(BuildJob { tile: *t, raw, frame: self.frame, generation: self.generation }).is_ok() {
                            self.in_build += 1;
                            self.states.insert(*t, TileState::Building);
                        }
                    }
                }
            } else if self.in_net < self.cfg.max_net_queue {
                if let Some(tx) = &self.net_tx {
                    if tx.send(NetJob { tile: *t }).is_ok() {
                        self.in_net += 1;
                        self.states.insert(*t, TileState::Fetching);
                    }
                }
            }
        }

        // ---- hand finished tiles to the game ----
        for _ in 0..self.cfg.max_adds_per_update {
            let Some(b) = self.ready.pop_front() else { break };
            if self.loaded.contains(&b.tile) {
                continue;
            }
            // skip tiles the car has already left again
            if self.tile_distance(b.tile, car_x, car_y) > self.cfg.unload_radius_m {
                self.states.insert(b.tile, TileState::Unrequested);
                continue;
            }
            self.loaded.insert(b.tile);
            self.states.insert(b.tile, TileState::Ready);
            events.push(StreamEvent::Add { tile: b.tile, scene: b.scene, meshes: b.meshes });
        }

        // ---- unload what is far away ----
        let far: Vec<TileId> = self.loaded.iter().copied().filter(|t| self.tile_distance(*t, car_x, car_y) > self.cfg.unload_radius_m).collect();
        for t in far {
            self.loaded.remove(&t);
            self.states.insert(t, TileState::Unrequested);
            events.push(StreamEvent::Remove { tile: t });
        }
        // forget raw data and idle state far beyond the unload radius
        let drop_raw: Vec<TileId> = self.raw.keys().copied().filter(|t| self.tile_distance(*t, car_x, car_y) > self.cfg.unload_radius_m * 1.8).collect();
        for t in drop_raw {
            self.raw.remove(&t);
            self.states.remove(&t);
            self.attempts.remove(&t);
        }
        events
    }
}

fn lat_of(f: &LocalFrame, x: f64, y: f64) -> f64 {
    f.from_local(x, y, 0.0).0
}
fn lon_of(f: &LocalFrame, x: f64, y: f64) -> f64 {
    f.from_local(x, y, 0.0).1
}

impl Drop for Streamer {
    fn drop(&mut self) {
        self.shutdown.store(1, Ordering::SeqCst);
        self.net_tx = None; // closing the channels ends the worker threads
        self.build_tx = None;
    }
}

fn net_worker(rx: Receiver<NetJob>, tx: Sender<NetResult>, fetcher: Arc<dyn Fetcher>, cache: Option<Arc<DiskCache>>, wanted: Arc<Mutex<HashSet<TileId>>>, cfg: StreamConfig, shutdown: Arc<AtomicU64>) {
    let mut last_request: Option<Instant> = None;
    while let Ok(job) = rx.recv() {
        if shutdown.load(Ordering::SeqCst) != 0 {
            break;
        }
        let tile = job.tile;
        let skipped = || NetResult { tile, outcome: Err((String::new(), true)), from_cache: false, bytes: 0, skipped: true };
        if !wanted.lock().unwrap().contains(&tile) {
            let _ = tx.send(skipped());
            continue;
        }
        // 1. disk cache
        let mut stale_text: Option<String> = None;
        if let Some(c) = &cache {
            if let Some((text, stale)) = c.get(tile) {
                match parse_overpass(&text, tile) {
                    Ok(raw) if !stale => {
                        let _ = tx.send(NetResult { tile, outcome: Ok(Arc::new(raw)), from_cache: true, bytes: 0, skipped: false });
                        continue;
                    }
                    Ok(_) => stale_text = Some(text),
                    Err(_) => c.remove(tile), // corrupt entry
                }
            }
        }
        // 2. network, politely
        if let Some(l) = last_request {
            let el = l.elapsed();
            if el < cfg.min_request_interval {
                thread::sleep(cfg.min_request_interval - el);
            }
        }
        if shutdown.load(Ordering::SeqCst) != 0 {
            break;
        }
        last_request = Some(Instant::now());
        let q = overpass_query(&tile.bounds());
        crate::log::log(&format!("tile {}: request via {}", tile.path(), fetcher.describe()));
        let t_req = Instant::now();
        let res = fetcher.fetch(tile, &q);
        match &res {
            Ok(text) => crate::log::log(&format!("tile {}: HTTP ok, {} bytes in {:.1}s", tile.path(), text.len(), t_req.elapsed().as_secs_f64())),
            Err(e) => crate::log::log(&format!("tile {}: FAILED after {:.1}s: {}", tile.path(), t_req.elapsed().as_secs_f64(), e)),
        }
        let outcome = match res {
            Ok(text) => match parse_overpass(&text, tile) {
                Ok(raw) => {
                    if let Some(c) = &cache {
                        let _ = c.put(tile, &text);
                    }
                    Ok((Arc::new(raw), text.len()))
                }
                Err(e) => {
                    let head: String = text.chars().take(160).collect();
                    crate::log::log(&format!("tile {}: response could not be parsed ({e}); starts with: {head}", tile.path()));
                    Err((format!("bad response: {e}"), true))
                }
            },
            Err(FetchError::Transient(m)) => Err((m, true)),
            Err(FetchError::Permanent(m)) => Err((m, false)),
        };
        match outcome {
            Ok((raw, bytes)) => {
                let _ = tx.send(NetResult { tile, outcome: Ok(raw), from_cache: false, bytes, skipped: false });
            }
            Err((msg, transient)) => {
                // network failed: an old cached copy is better than nothing
                if let Some(text) = stale_text {
                    if let Ok(raw) = parse_overpass(&text, tile) {
                        let _ = tx.send(NetResult { tile, outcome: Ok(Arc::new(raw)), from_cache: true, bytes: 0, skipped: false });
                        continue;
                    }
                }
                let _ = tx.send(NetResult { tile, outcome: Err((msg, transient)), from_cache: false, bytes: 0, skipped: false });
                if transient {
                    thread::sleep(cfg.cooldown_after_error);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fetch::VirtualOsm;
    use std::sync::atomic::AtomicUsize;

    const DEMO: &str = include_str!("../../../data/demo_overpass.json");
    const MINSK: (f64, f64) = (53.9045, 27.5615);

    fn fast_cfg() -> StreamConfig {
        StreamConfig {
            min_request_interval: Duration::ZERO,
            backoff_base: Duration::from_millis(30),
            backoff_max: Duration::from_millis(200),
            cooldown_after_error: Duration::ZERO,
            max_net_queue: 8,
            max_build_queue: 8,
            max_adds_per_update: 8,
            ..Default::default()
        }
    }

    fn tmpdir(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("earthsim-st-{}-{}-{}", name, std::process::id(), std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos()))
    }

    /// Pump `update` until `done` or timeout. Collects all events.
    fn pump(s: &mut Streamer, x: f64, y: f64, secs: f64, mut done: impl FnMut(&Streamer, &[StreamEvent]) -> bool) -> Vec<StreamEvent> {
        let mut all = Vec::new();
        let t0 = Instant::now();
        while t0.elapsed().as_secs_f64() < secs {
            let ev = s.update(x, y);
            all.extend(ev);
            if done(s, &all) {
                break;
            }
            thread::sleep(Duration::from_millis(2));
        }
        all
    }

    fn adds(ev: &[StreamEvent]) -> usize {
        ev.iter().filter(|e| matches!(e, StreamEvent::Add { .. })).count()
    }

    #[test]
    fn loads_the_tiles_around_the_car_and_builds_real_meshes() {
        let v = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap());
        let mut s = Streamer::new(fast_cfg(), v.clone(), None, MINSK.0, MINSK.1);
        let ev = pump(&mut s, 0.0, 0.0, 10.0, |s, _| s.stats().loaded >= 9 && s.stats().fetching == 0 && s.stats().building == 0);
        let st = s.stats();
        assert!(st.loaded >= 9, "loaded {} tiles; stats {:?}", st.loaded, st);
        let mut buildings = 0;
        let mut tris = 0;
        for e in &ev {
            if let StreamEvent::Add { scene, meshes, .. } = e {
                buildings += scene.buildings.len();
                tris += meshes.iter().map(|m| m.vertex_count() / 3).sum::<usize>();
                assert!(meshes.iter().all(|m| m.name != "ground"), "ground is not per tile");
            }
        }
        assert_eq!(buildings, 363, "all demo buildings should be loaded exactly once");
        assert!(tris > 10_000);
        assert_eq!(st.net_requests as usize, v.request_count());
        assert!(st.bytes > 0 && st.last_error.is_none());
    }

    #[test]
    fn second_run_uses_the_disk_cache_and_makes_no_requests() {
        let dir = tmpdir("cache");
        let v1 = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap());
        {
            let mut s = Streamer::new(fast_cfg(), v1.clone(), Some(DiskCache::new(&dir, Duration::from_secs(3600))), MINSK.0, MINSK.1);
            pump(&mut s, 0.0, 0.0, 10.0, |s, _| s.stats().loaded >= 9 && s.stats().fetching == 0 && s.stats().building == 0);
        }
        assert!(v1.request_count() >= 9);
        let v2 = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap().always_permanent_error());
        let mut s = Streamer::new(fast_cfg(), v2.clone(), Some(DiskCache::new(&dir, Duration::from_secs(3600))), MINSK.0, MINSK.1);
        pump(&mut s, 0.0, 0.0, 10.0, |s, _| s.stats().loaded >= 9 && s.stats().fetching == 0 && s.stats().building == 0);
        assert!(s.stats().loaded >= 9);
        assert_eq!(v2.request_count(), 0, "everything should come from the cache");
        assert!(s.stats().cache_hits >= 9);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn stale_cache_is_used_when_the_network_is_down() {
        let dir = tmpdir("stale");
        let v1 = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap());
        {
            let mut s = Streamer::new(fast_cfg(), v1, Some(DiskCache::new(&dir, Duration::ZERO)), MINSK.0, MINSK.1);
            pump(&mut s, 0.0, 0.0, 10.0, |s, _| s.stats().loaded >= 9 && s.stats().fetching == 0);
        }
        thread::sleep(Duration::from_millis(30));
        let down = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap().always_permanent_error());
        let mut s = Streamer::new(fast_cfg(), down.clone(), Some(DiskCache::new(&dir, Duration::ZERO)), MINSK.0, MINSK.1);
        pump(&mut s, 0.0, 0.0, 10.0, |s, _| s.stats().loaded >= 9 && s.stats().fetching == 0);
        assert!(s.stats().loaded >= 9, "{:?}", s.stats());
        assert!(down.request_count() >= 9, "stale tiles should have been re-requested first");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn transient_errors_back_off_then_recover() {
        let v = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap().failing_first(3));
        let mut s = Streamer::new(fast_cfg(), v.clone(), None, MINSK.0, MINSK.1);
        let mut saw_error = false;
        pump(&mut s, 0.0, 0.0, 15.0, |s, _| {
            saw_error |= s.stats().errors > 0;
            s.stats().loaded >= 9 && s.stats().fetching == 0 && s.stats().errors == 0
        });
        assert!(saw_error, "tiles should have entered the Error state");
        assert!(s.stats().loaded >= 9, "should recover after backoff: {:?}", s.stats());
        assert!(v.request_count() >= 12);
    }

    #[test]
    fn permanent_errors_do_not_spam_the_server() {
        let v = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap().always_permanent_error());
        let mut cfg = fast_cfg();
        cfg.backoff_max = Duration::from_secs(60); // permanent errors wait the maximum backoff
        let mut s = Streamer::new(cfg, v.clone(), None, MINSK.0, MINSK.1);
        pump(&mut s, 0.0, 0.0, 1.0, |_, _| false);
        let n = v.request_count();
        let desired = TileId::around(MINSK.0, MINSK.1, 1500.0, 15).len();
        assert_eq!(n, desired, "each tile asked exactly once, then parked in Error");
        assert_eq!(s.stats().loaded, 0);
        assert_eq!(s.stats().errors, desired);
        assert!(s.stats().last_error.as_ref().unwrap().contains("403"));
    }

    #[test]
    fn requests_are_rate_limited() {
        let v = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap());
        let mut cfg = fast_cfg();
        cfg.min_request_interval = Duration::from_millis(80);
        let mut s = Streamer::new(cfg, v.clone(), None, MINSK.0, MINSK.1);
        let t0 = Instant::now();
        pump(&mut s, 0.0, 0.0, 20.0, |s, _| s.stats().loaded >= 9 && s.stats().fetching == 0);
        let n = v.request_count();
        let min = Duration::from_millis(80) * (n as u32 - 1);
        assert!(t0.elapsed() >= min, "{n} requests finished in {:?}, expected at least {min:?}", t0.elapsed());
    }

    #[test]
    fn driving_away_unloads_tiles_and_the_event_stream_is_consistent() {
        let v = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap());
        let mut s = Streamer::new(fast_cfg(), v, None, MINSK.0, MINSK.1);
        let mut loaded: HashSet<TileId> = HashSet::new();
        let check = |ev: &[StreamEvent], loaded: &mut HashSet<TileId>| {
            for e in ev {
                match e {
                    StreamEvent::Add { tile, .. } => assert!(loaded.insert(*tile), "double Add of {tile:?}"),
                    StreamEvent::Remove { tile } => assert!(loaded.remove(tile), "Remove of a tile that was not loaded {tile:?}"),
                    StreamEvent::Rebased { .. } => loaded.clear(),
                }
            }
        };
        let ev = pump(&mut s, 0.0, 0.0, 10.0, |s, _| s.stats().loaded >= 9 && s.stats().fetching == 0);
        check(&ev, &mut loaded);
        let n0 = loaded.len();
        // drive east in steps; the car never exceeds the rebase distance here
        let mut x = 0.0;
        while x < 3500.0 {
            x += 100.0;
            let ev = pump(&mut s, x, 0.0, 0.05, |_, _| false);
            check(&ev, &mut loaded);
            assert!(loaded.len() <= 40, "tile count must stay bounded, got {}", loaded.len());
        }
        let ev = pump(&mut s, x, 0.0, 3.0, |_, _| false);
        check(&ev, &mut loaded);
        assert_eq!(loaded.len(), s.stats().loaded);
        assert!(n0 >= 9);
        let (la, lo, _) = s.frame().from_local(x, 0.0, 0.0);
        let _ = (la, lo);
        // the original centre tile is far behind now and must have been unloaded
        let first = TileId::from_lat_lon(MINSK.0, MINSK.1, 15);
        assert!(!loaded.contains(&first), "start tile should have been unloaded");
    }

    #[test]
    fn rebase_rebuilds_from_memory_without_new_downloads() {
        let v = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap());
        let mut cfg = fast_cfg();
        cfg.rebase_dist_m = 800.0;
        cfg.load_radius_m = 1500.0;
        let mut s = Streamer::new(cfg, v.clone(), None, MINSK.0, MINSK.1);
        pump(&mut s, 0.0, 0.0, 10.0, |s, _| s.stats().loaded >= 9 && s.stats().fetching == 0);
        let before = v.request_count();
        let old_frame = s.frame();
        let gen0 = s.stats().generation;
        // car is now 900 m east of the origin: triggers a re-base
        let ev = pump(&mut s, 900.0, 0.0, 10.0, |_, e| e.iter().any(|x| matches!(x, StreamEvent::Rebased { .. })));
        let rebased = ev.iter().find_map(|e| if let StreamEvent::Rebased { old, new } = e { Some((*old, *new)) } else { None }).expect("rebased event");
        assert_eq!(s.stats().generation, gen0 + 1);
        // new origin is where the car was
        let p = old_frame.to_local(rebased.1.lat, rebased.1.lon, 0.0);
        assert!((p[0] - 900.0).abs() < 0.01 && p[1].abs() < 0.01, "{p:?}");
        // car coordinates convert to ~(0,0) in the new frame
        let (la, lo, _) = rebased.0.from_local(900.0, 0.0, 0.0);
        let q = rebased.1.to_local(la, lo, 0.0);
        assert!(q[0].abs() < 1e-3 && q[1].abs() < 1e-3);
        // re-add tiles for the new origin, from memory. The car is now at (0,0) in the new frame.
        pump(&mut s, 0.0, 0.0, 10.0, |s, _| s.stats().loaded >= 6 && s.stats().building == 0);
        assert!(s.stats().loaded >= 6, "{:?}", s.stats());
        let new_requests = v.request_count() - before;
        // only tiles that were never seen before may be requested (the area to the east), never the old ones again
        assert!(new_requests <= 12, "{new_requests} new requests");
    }

    #[test]
    fn teleport_clears_everything_and_loads_the_new_place() {
        let v = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap());
        let mut s = Streamer::new(fast_cfg(), v, None, 48.8584, 2.2945); // Paris: no demo data
        pump(&mut s, 0.0, 0.0, 5.0, |s, _| s.stats().loaded >= 9);
        let mut ev = Vec::new();
        let n_before = s.stats().loaded;
        s.set_origin(MINSK.0, MINSK.1, &mut ev);
        let removed = ev.iter().filter(|e| matches!(e, StreamEvent::Remove { .. })).count();
        assert_eq!(removed, n_before);
        assert!(matches!(ev.last(), Some(StreamEvent::Rebased { .. })));
        let ev = pump(&mut s, 0.0, 0.0, 10.0, |s, e| s.stats().loaded >= 9 && adds(e) >= 9);
        let b: usize = ev.iter().map(|e| if let StreamEvent::Add { scene, .. } = e { scene.buildings.len() } else { 0 }).sum();
        assert_eq!(b, 363);
    }

    #[test]
    fn dropping_the_streamer_stops_its_threads() {
        let cnt = Arc::new(AtomicUsize::new(0));
        let v = Arc::new(VirtualOsm::from_overpass_json(DEMO).unwrap().with_delay_ms(5));
        let c2 = cnt.clone();
        {
            let mut s = Streamer::new(fast_cfg(), v.clone(), None, MINSK.0, MINSK.1);
            pump(&mut s, 0.0, 0.0, 0.05, |_, _| false);
            c2.store(v.request_count(), Ordering::SeqCst);
        }
        thread::sleep(Duration::from_millis(300));
        // after drop at most the in-flight request completes; the queue is not drained
        assert!(v.request_count() <= cnt.load(Ordering::SeqCst) + 2);
    }
}
