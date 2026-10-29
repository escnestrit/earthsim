//! World map (M): Overture map tiles drawn on the CPU by `world::basemap`, the car's position and heading,
//! mouse wheel / +/- to zoom, drag to pan, click (or Enter on the centre) to teleport there.
use crate::Sim;
use bevy::asset::RenderAssetUsages;
use bevy::image::{ImageSampler, ImageSamplerDescriptor};
use bevy::input::mouse::{AccumulatedMouseScroll, MouseScrollUnit};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::window::PrimaryWindow;
use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Condvar, Mutex};
use world::basemap::{self, MapSource, MAX_ZOOM, SIZE};
use world::pmtiles::RangeSource;

type Key = (u8, u32, u32);
type Source = MapSource<Box<dyn RangeSource>>;

pub struct MapPlugin;

impl Plugin for MapPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MapView::new(open_source()))
            .add_systems(Startup, setup)
            .add_systems(Update, (toggle, interact, receive, layout).chain().before(crate::input::read_input));
    }
}

/// Where map tiles come from: Overture over HTTP on Windows; `EARTHSIM_MAP_PMTILES=<file>` (any vector
/// PMTiles archive, e.g. a Protomaps extract) everywhere, for offline use and tests.
/// Archives are opened by the first tile request (it needs the network), and again after a failure.
fn open_source() -> Option<Arc<LazySource>> {
    use world::pmtiles::{FileSource, PmTiles};
    if let Ok(f) = std::env::var("EARTHSIM_MAP_PMTILES") {
        return Some(LazySource::new(Box::new(move || {
            let open = || PmTiles::open(Box::new(FileSource(f.clone().into())) as Box<dyn RangeSource>).ok();
            MapSource { base: open(), transport: open(), buildings: open() }
        })));
    }
    #[cfg(windows)]
    {
        let exe_dir = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.to_path_buf())).unwrap_or_else(|| ".".into());
        let cache = std::env::var("EARTHSIM_CACHE").map(std::path::PathBuf::from).unwrap_or_else(|_| exe_dir.join("cache")).join("map");
        return Some(LazySource::new(Box::new(move || world::fetch::live_map_source(&cache))));
    }
    #[allow(unreachable_code)]
    None
}

struct LazySource {
    open: Box<dyn Fn() -> Source + Send + Sync>,
    src: Mutex<(Option<Arc<Source>>, Option<std::time::Instant>)>,
}

impl LazySource {
    fn new(open: Box<dyn Fn() -> Source + Send + Sync>) -> Arc<LazySource> {
        Arc::new(LazySource { open, src: Mutex::new((None, None)) })
    }

    fn get(&self) -> Result<Arc<Source>, String> {
        let mut s = self.src.lock().unwrap();
        if let Some(src) = &s.0 {
            return Ok(src.clone());
        }
        if s.1.is_some_and(|t| t.elapsed().as_secs() < 30) {
            return Err("map data not reachable (retrying shortly)".into());
        }
        s.1 = Some(std::time::Instant::now());
        let src = (self.open)();
        if src.base.is_none() && src.transport.is_none() && src.buildings.is_none() {
            return Err("map data not reachable".into());
        }
        let src = Arc::new(src);
        s.0 = Some(src.clone());
        Ok(src)
    }
}

/// Shared between the UI and the tile workers: wanted tiles (newest first) and finished ones.
struct Queue {
    want: Mutex<(Vec<Key>, HashSet<Key>)>,
    cv: Condvar,
    done: Mutex<Vec<(Key, Result<Vec<u8>, String>)>>,
}

#[derive(Resource)]
pub struct MapView {
    pub open: bool,
    /// view centre in Web Mercator world units [0, 1)
    cx: f64,
    cy: f64,
    zoom: f64,
    tiles: HashMap<Key, Handle<Image>>,
    /// tiles that failed, retried after a while
    failed: HashMap<Key, std::time::Instant>,
    last_error: String,
    q: Option<Arc<Queue>>,
    drag: Option<(Vec2, f64, f64, bool)>,
    size: Vec2,
    pub status: String,
    /// recentre on the car next time the map opens
    follow: bool,
}

impl MapView {
    fn new(src: Option<Arc<LazySource>>) -> MapView {
        let q = src.map(|src| {
            let q = Arc::new(Queue { want: Mutex::new((Vec::new(), HashSet::new())), cv: Condvar::new(), done: Mutex::new(Vec::new()) });
            for _ in 0..4 {
                let (q, src) = (q.clone(), src.clone());
                std::thread::spawn(move || loop {
                    let key = {
                        let mut w = q.want.lock().unwrap();
                        loop {
                            if let Some(k) = w.0.pop() {
                                w.1.insert(k);
                                break k;
                            }
                            w = q.cv.wait(w).unwrap();
                        }
                    };
                    let r = src.get().and_then(|s| s.render(key.0, key.1, key.2).map(|r| r.px));
                    q.want.lock().unwrap().1.remove(&key);
                    q.done.lock().unwrap().push((key, r));
                });
            }
            q
        });
        let status = if q.is_some() { String::new() } else { "MAP DATA NOT AVAILABLE IN THIS BUILD (SET EARTHSIM_MAP_PMTILES)".into() };
        MapView { open: false, cx: 0.5, cy: 0.5, zoom: 14.0, tiles: HashMap::new(), failed: HashMap::new(), last_error: String::new(), q, drag: None, size: Vec2::new(1280.0, 720.0), status, follow: true }
    }

    /// Screen position (logical px) of a world point.
    fn to_screen(&self, wx: f64, wy: f64) -> Vec2 {
        let s = SIZE as f64 * 2f64.powf(self.zoom);
        Vec2::new(((wx - self.cx) * s) as f32 + self.size.x / 2.0, ((wy - self.cy) * s) as f32 + self.size.y / 2.0)
    }

    fn to_world(&self, p: Vec2) -> (f64, f64) {
        let s = SIZE as f64 * 2f64.powf(self.zoom);
        (self.cx + (p.x - self.size.x / 2.0) as f64 / s, (self.cy + (p.y - self.size.y / 2.0) as f64 / s).clamp(0.0, 0.9999))
    }
}

#[derive(Component)]
struct MapRoot;
#[derive(Component)]
struct MapTile(Key);
#[derive(Component)]
struct CarMarker;
#[derive(Component)]
struct MapText;

fn setup(mut commands: Commands) {
    commands
        .spawn((
            MapRoot,
            Node { position_type: PositionType::Absolute, left: Val::Px(0.0), top: Val::Px(0.0), width: Val::Percent(100.0), height: Val::Percent(100.0), overflow: Overflow::clip(), ..default() },
            BackgroundColor(Color::srgb(0.64, 0.77, 0.9)),
            GlobalZIndex(50),
            Visibility::Hidden,
        ))
        .with_children(|p| {
            p.spawn((
                CarMarker,
                Node { position_type: PositionType::Absolute, width: Val::Px(16.0), height: Val::Px(16.0), border: UiRect::all(Val::Px(3.0)), border_radius: BorderRadius::MAX, ..default() },
                BackgroundColor(Color::srgb(0.95, 0.15, 0.1)),
                BorderColor::all(Color::WHITE),
                ZIndex(10),
            ));
            p.spawn((
                MapText,
                Text::new(""),
                TextFont { font_size: FontSize::Px(15.0), ..default() },
                TextColor(Color::srgb(0.1, 0.1, 0.12)),
                Node { position_type: PositionType::Absolute, left: Val::Px(12.0), bottom: Val::Px(10.0), padding: UiRect::all(Val::Px(6.0)), ..default() },
                BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.85)),
                ZIndex(11),
            ));
        });
}

fn toggle(keys: Res<ButtonInput<KeyCode>>, mut view: ResMut<MapView>, sim: Res<Sim>) {
    let forced = std::env::var("EARTHSIM_SHOT_MAP").is_ok();
    if keys.just_pressed(KeyCode::KeyM) || (forced && !view.open) || (view.open && keys.just_pressed(KeyCode::Escape)) {
        view.open = !view.open || forced;
        if view.open {
            if let Some((lat, lon, _)) = sim.g().car_geo() {
                if view.follow {
                    let (x, y) = basemap::to_world(lat, lon);
                    view.cx = x;
                    view.cy = y;
                }
            }
            if let Ok(z) = std::env::var("EARTHSIM_SHOT_MAP").map(|v| v.parse::<f64>().unwrap_or(15.0)) {
                view.zoom = z;
            }
        }
    }
}

fn interact(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    scroll: Res<AccumulatedMouseScroll>,
    windows: Query<&Window, With<PrimaryWindow>>,
    root: Query<&ComputedNode, With<MapRoot>>,
    mut view: ResMut<MapView>,
    sim: Res<Sim>,
) {
    if let Ok(c) = root.single() {
        let s = c.size() * c.inverse_scale_factor();
        if s.x > 1.0 && s.y > 1.0 {
            view.size = s;
        }
    }
    if !view.open {
        return;
    }
    let cursor = windows.single().ok().and_then(|w| w.cursor_position());
    // zoom about the cursor
    let mut dz = match scroll.unit {
        MouseScrollUnit::Line => scroll.delta.y as f64 * 0.5,
        MouseScrollUnit::Pixel => scroll.delta.y as f64 / 120.0,
    };
    if keys.just_pressed(KeyCode::Equal) || keys.just_pressed(KeyCode::NumpadAdd) {
        dz += 1.0;
    }
    if keys.just_pressed(KeyCode::Minus) || keys.just_pressed(KeyCode::NumpadSubtract) {
        dz -= 1.0;
    }
    if dz != 0.0 {
        let anchor = cursor.unwrap_or(view.size / 2.0);
        let before = view.to_world(anchor);
        view.zoom = (view.zoom + dz).clamp(2.0, MAX_ZOOM as f64);
        let after = view.to_world(anchor);
        view.cx += before.0 - after.0;
        view.cy = (view.cy + before.1 - after.1).clamp(0.0, 0.9999);
    }
    // pan with the arrow keys
    let step = 0.25 * view.size.y as f64 / (SIZE as f64 * 2f64.powf(view.zoom));
    for (k, dx, dy) in [(KeyCode::ArrowLeft, -1.0, 0.0), (KeyCode::ArrowRight, 1.0, 0.0), (KeyCode::ArrowUp, 0.0, -1.0), (KeyCode::ArrowDown, 0.0, 1.0)] {
        if keys.just_pressed(k) {
            view.cx += dx * step;
            view.cy = (view.cy + dy * step).clamp(0.0, 0.9999);
            view.follow = false;
        }
    }
    view.cx = view.cx.rem_euclid(1.0);
    // drag to pan; a press-release without moving teleports
    if let Some(p) = cursor {
        if mouse.just_pressed(MouseButton::Left) {
            view.drag = Some((p, view.cx, view.cy, false));
        }
        if let Some((start, cx, cy, moved)) = view.drag {
            let d = p - start;
            let moved = moved || d.length() > 4.0;
            if moved {
                let s = SIZE as f64 * 2f64.powf(view.zoom);
                view.cx = (cx - d.x as f64 / s).rem_euclid(1.0);
                view.cy = (cy - d.y as f64 / s).clamp(0.0, 0.9999);
                view.follow = false;
            }
            view.drag = Some((start, cx, cy, moved));
            if mouse.just_released(MouseButton::Left) {
                view.drag = None;
                if !moved {
                    let (wx, wy) = view.to_world(p);
                    teleport(&mut view, &sim, wx, wy);
                }
            }
        }
    }
    if keys.just_pressed(KeyCode::Enter) || keys.just_pressed(KeyCode::NumpadEnter) {
        let (wx, wy) = (view.cx, view.cy);
        teleport(&mut view, &sim, wx, wy);
    }
    if keys.just_pressed(KeyCode::KeyF) {
        view.follow = true;
        if let Some((lat, lon, _)) = sim.g().car_geo() {
            let (x, y) = basemap::to_world(lat, lon);
            view.cx = x;
            view.cy = y;
        }
    }
}

fn teleport(view: &mut MapView, sim: &Sim, wx: f64, wy: f64) {
    let (lat, lon) = basemap::from_world(wx.rem_euclid(1.0), wy);
    let mut g = sim.g();
    if !g.is_streaming() {
        view.status = "TELEPORT NEEDS THE STREAMED WORLD (NOT THE DEMO SCENE)".into();
        return;
    }
    world::log::log(&format!("map: teleport to {lat:.5}, {lon:.5}"));
    g.teleport(&format!("{lat:.5}, {lon:.5}"), lat, lon);
    view.open = false;
    view.follow = true;
    view.status.clear();
}

fn receive(view: ResMut<MapView>, mut images: ResMut<Assets<Image>>) {
    let view = view.into_inner();
    let Some(q) = view.q.clone() else { return };
    let done: Vec<_> = std::mem::take(&mut *q.done.lock().unwrap());
    for (key, r) in done {
        match r {
            Ok(px) => {
                let mut img = Image::new(
                    Extent3d { width: SIZE as u32, height: SIZE as u32, depth_or_array_layers: 1 },
                    TextureDimension::D2,
                    px,
                    TextureFormat::Rgba8UnormSrgb,
                    RenderAssetUsages::RENDER_WORLD,
                );
                img.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor::linear());
                view.tiles.insert(key, images.add(img));
            }
            Err(e) => {
                if view.last_error != e {
                    world::log::log(&format!("map tile {}/{}/{}: {e}", key.0, key.1, key.2));
                }
                view.last_error = e;
                view.failed.insert(key, std::time::Instant::now());
            }
        }
    }
    // keep memory bounded: forget tiles far from the view
    if view.tiles.len() > 700 {
        let z = view.zoom.floor() as i32;
        let (cx, cy) = (view.cx, view.cy);
        view.tiles.retain(|k, h| {
            let n = (1u64 << k.0) as f64;
            let near = (k.0 as i32 - z).abs() <= 1 && ((k.1 as f64 + 0.5) / n - cx).abs() < 0.02 * 2f64.powi(12 - k.0 as i32).max(1.0) && ((k.2 as f64 + 0.5) / n - cy).abs() < 0.02 * 2f64.powi(12 - k.0 as i32).max(1.0);
            if !near {
                images.remove(h.id());
            }
            near
        });
    }
}

#[allow(clippy::too_many_arguments)]
fn layout(
    mut commands: Commands,
    view: ResMut<MapView>,
    sim: Res<Sim>,
    mut root: Query<(Entity, &mut Visibility), With<MapRoot>>,
    mut tiles: Query<(Entity, &MapTile, &mut Node), (Without<CarMarker>, Without<MapText>)>,
    mut marker: Query<(&mut Node, &mut Visibility), (With<CarMarker>, Without<MapTile>, Without<MapRoot>)>,
    mut text: Query<&mut Text, With<MapText>>,
) {
    let view = view.into_inner();
    let Ok((root_e, mut vis)) = root.single_mut() else { return };
    *vis = if view.open { Visibility::Inherited } else { Visibility::Hidden };
    if !view.open {
        return;
    }
    let z = (view.zoom.round() as i64).clamp(0, MAX_ZOOM as i64) as u8;
    let n = 1i64 << z;
    let scale = 2f64.powf(view.zoom - z as f64);
    let tile_px = SIZE as f64 * scale;
    // visible tiles at zoom z
    let half = (view.size.as_dvec2() / 2.0) / tile_px;
    let (tcx, tcy) = (view.cx * n as f64, view.cy * n as f64);
    let (x0, x1) = ((tcx - half.x).floor() as i64, (tcx + half.x).floor() as i64);
    let (y0, y1) = ((tcy - half.y).floor().max(0.0) as i64, (tcy + half.y).floor().min(n as f64 - 1.0) as i64);
    let mut visible: Vec<(Key, Vec2, i64)> = Vec::new();
    for ty in y0..=y1 {
        for tx in x0..=x1 {
            let key = (z, tx.rem_euclid(n) as u32, ty as u32);
            let pos = Vec2::new(((tx as f64 - tcx) * tile_px) as f32 + view.size.x / 2.0, ((ty as f64 - tcy) * tile_px) as f32 + view.size.y / 2.0);
            let dist = (tx as f64 + 0.5 - tcx).abs().max((ty as f64 + 0.5 - tcy).abs()) as i64;
            visible.push((key, pos, dist));
        }
    }
    // request missing tiles, nearest the centre first (workers pop from the end)
    if let Some(q) = &view.q {
        view.failed.retain(|_, t| t.elapsed().as_secs() < 20);
        let mut want: Vec<(i64, Key)> = visible.iter().filter(|(k, _, _)| !view.tiles.contains_key(k) && !view.failed.contains_key(k)).map(|(k, _, d)| (*d, *k)).collect();
        want.sort_by(|a, b| b.0.cmp(&a.0));
        let mut w = q.want.lock().unwrap();
        let in_flight = w.1.clone();
        w.0 = want.into_iter().map(|x| x.1).filter(|k| !in_flight.contains(k)).collect();
        if !w.0.is_empty() {
            q.cv.notify_all();
        }
    }
    // place tile nodes (a tile not ready yet shows its parent, scaled, when that is loaded)
    let mut shown: HashMap<Key, (Handle<Image>, Vec2, f32)> = HashMap::new();
    for (key, pos, _) in &visible {
        if let Some(h) = view.tiles.get(key) {
            shown.insert(*key, (h.clone(), *pos, tile_px as f32));
        } else if key.0 > 0 {
            let pk = (key.0 - 1, key.1 / 2, key.2 / 2);
            if let Some(h) = view.tiles.get(&pk) {
                let off = Vec2::new((key.1 % 2) as f32, (key.2 % 2) as f32) * tile_px as f32;
                shown.entry(pk).or_insert((h.clone(), *pos - off, 2.0 * tile_px as f32));
            }
        }
    }
    let mut existing: HashSet<Key> = HashSet::new();
    for (e, t, mut node) in &mut tiles {
        match shown.get(&t.0) {
            Some((_, pos, size)) => {
                existing.insert(t.0);
                node.left = Val::Px(pos.x.floor());
                node.top = Val::Px(pos.y.floor());
                node.width = Val::Px(size.ceil() + 1.0);
                node.height = Val::Px(size.ceil() + 1.0);
            }
            None => commands.entity(e).despawn(),
        }
    }
    for (key, (h, pos, size)) in shown {
        if existing.contains(&key) {
            continue;
        }
        let child = commands
            .spawn((
                MapTile(key),
                ImageNode::new(h),
                Node { position_type: PositionType::Absolute, left: Val::Px(pos.x.floor()), top: Val::Px(pos.y.floor()), width: Val::Px(size.ceil() + 1.0), height: Val::Px(size.ceil() + 1.0), ..default() },
                ZIndex(if key.0 == z { 1 } else { 0 }),
            ))
            .id();
        commands.entity(root_e).add_child(child);
    }
    // the car
    let g = sim.g();
    let geo = g.car_geo();
    let place = g.place_name().unwrap_or("").to_string();
    drop(g);
    if let Ok((mut node, mut mv)) = marker.single_mut() {
        match geo {
            Some((lat, lon, _)) => {
                let (wx, wy) = basemap::to_world(lat, lon);
                // nearest copy of the car across the antimeridian
                let wx = wx + (view.cx - wx).round();
                let p = view.to_screen(wx, wy);
                node.left = Val::Px(p.x - 8.0);
                node.top = Val::Px(p.y - 8.0);
                *mv = Visibility::Inherited;
            }
            None => *mv = Visibility::Hidden,
        }
    }
    let (clat, clon) = basemap::from_world(view.cx, view.cy);
    let loading = view.q.as_ref().map(|q| { let w = q.want.lock().unwrap(); w.0.len() + w.1.len() }).unwrap_or(0);
    if let Ok(mut t) = text.single_mut() {
        let mut s = format!("MAP   CENTRE {clat:.5}, {clon:.5}   ZOOM {:.1}", view.zoom);
        if let Some((lat, lon, hdg)) = geo {
            s.push_str(&format!("\nCAR {lat:.5}, {lon:.5}  HEADING {hdg:.0}   {place}"));
        }
        if loading > 0 {
            s.push_str(&format!("\nLOADING {loading} TILES"));
        }
        if !view.status.is_empty() {
            s.push('\n');
            s.push_str(&view.status);
        }
        if !view.failed.is_empty() && !view.last_error.is_empty() {
            s.push_str(&format!("\n{} TILES FAILED: {}", view.failed.len(), view.last_error.to_uppercase()));
        }
        s.push_str("\nCLICK: TELEPORT THERE   DRAG / ARROWS: MOVE   WHEEL / + -: ZOOM   F: FIND CAR   M / ESC: CLOSE\n(C) OPENSTREETMAP CONTRIBUTORS, (C) OVERTURE MAPS FOUNDATION");
        t.0 = s;
    }
}
