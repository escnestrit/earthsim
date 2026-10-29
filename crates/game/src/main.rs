//! earthsim on Bevy: the simulation (streaming world, soft-body car, driver model) is the engine-independent
//! `earthsim::game::Game`; Bevy renders it, reads keyboard / gamepad / steering wheel and shows the HUD.
//!
//! World axes: the simulation is Z-up (x east, y north); Bevy is Y-up. All world geometry hangs under one
//! root entity rotated -90 degrees about X, and the camera is converted with `zup`.

mod car_visual;
mod facade;
mod hud;
mod input;
mod lights;
mod map;
mod settings;
mod terrain;
mod view;

use bevy::prelude::*;
use earthsim::game::Game;

/// The simulation behind a mutex: its streamer holds channel receivers, which are `Send` but not `Sync`.
#[derive(Resource)]
pub struct Sim(pub std::sync::Mutex<Game>);

impl Sim {
    pub fn g(&self) -> std::sync::MutexGuard<'_, Game> {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// Headless capture for automated checks (`EARTHSIM_SHOT=out.png`): render N frames, save, exit.
#[derive(Resource, Clone)]
pub struct Shot {
    pub path: String,
    pub frames: u32,
    pub warmup_sim_s: f64,
}

fn build_sim() -> Result<Game, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut opts = earthsim::cli::parse(args)?;
    let exe_dir = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.to_path_buf())).unwrap_or_else(|| ".".into());
    let cache_dir = std::env::var("EARTHSIM_CACHE").ok().map(std::path::PathBuf::from).or_else(|| opts.cache.clone().map(Into::into)).unwrap_or_else(|| exe_dir.join("cache"));
    world::log::init(&exe_dir.join("earthsim.log"));
    world::log::log(&format!("earthsim (Bevy) mode {:?}, cache {}", opts.mode, cache_dir.display()));
    // a BeamNG vehicle zip in the "mods" folder next to the exe is driven when no --car is given
    if opts.car.is_none() {
        let mut zips: Vec<_> = std::fs::read_dir(exe_dir.join("mods")).into_iter().flatten().flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("zip"))).collect();
        zips.sort();
        if let Some(z) = zips.first() {
            world::log::log(&format!("using car mod {}", z.display()));
            opts.car = Some(z.to_string_lossy().into_owned());
        }
    }
    #[cfg(windows)]
    let live = Some(world::fetch::default_live_fetcher());
    #[cfg(not(windows))]
    let live: Option<std::sync::Arc<dyn world::fetch::Fetcher>> = None;
    earthsim::game::build_game(&opts, live, cache_dir)
}

fn main() {
    let game = match build_sim() {
        Ok(mut g) => {
            // physics of a mod car on its own thread, except in deterministic test captures
            if std::env::var("EARTHSIM_SHOT").is_err() {
                g.set_threaded_physics(true);
            }
            g
        }
        Err(e) => {
            eprintln!("earthsim: {e}");
            world::log::log(&format!("startup failed: {e}"));
            std::process::exit(2);
        }
    };
    let shot = std::env::var("EARTHSIM_SHOT").ok().map(|path| Shot {
        path,
        frames: std::env::var("EARTHSIM_SHOT_FRAMES").ok().and_then(|v| v.parse().ok()).unwrap_or(40),
        warmup_sim_s: std::env::var("EARTHSIM_SHOT_SIM").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0),
    });

    let mut app = App::new();
    if let Some(s) = &shot {
        use bevy::app::ScheduleRunnerPlugin;
        use bevy::window::ExitCondition;
        app.add_plugins(
            DefaultPlugins
                .set(WindowPlugin { primary_window: None, exit_condition: ExitCondition::DontExit, ..default() })
                .disable::<bevy::winit::WinitPlugin>(),
        )
        .add_plugins(ScheduleRunnerPlugin::run_loop(std::time::Duration::from_millis(1)))
        .insert_resource(s.clone())
        // The dev sandbox renders with software OpenGL, where one optional compute shader (SSAO depth
        // pre-pass) fails to compile; ignore that instead of quitting. Not used for the real window.
        .insert_resource(bevy::render::error_handler::RenderErrorHandler(|e, _, _| {
            eprintln!("earthsim: ignoring render error in capture mode: {:?}", e.ty);
            bevy::render::error_handler::RenderErrorPolicy::Ignore
        }));
    } else {
        app.add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title: "BeamNG.Earth prototype (earthsim)".into(), resolution: (1600u32, 900u32).into(), ..default() }),
            ..default()
        }));
    }
    app.insert_resource(Sim(std::sync::Mutex::new(game)))
        .add_plugins((view::ViewPlugin, input::InputPlugin, hud::HudPlugin, car_visual::CarVisualPlugin, map::MapPlugin, settings::SettingsPlugin, lights::LightsPlugin, terrain::TerrainPlugin))
        .add_systems(Update, step_sim.after(input::read_input).before(view::sync_world));
    app.run();
}

/// Advance the simulation by the frame time (the game runs its own fixed 1 ms physics steps).
fn step_sim(sim: ResMut<Sim>, time: Res<Time>, shot: Option<Res<Shot>>, mut warm: Local<bool>) {
    if let Some(s) = shot {
        // deterministic captures: fixed 1/60 s per frame, plus an optional warm-up
        if !*warm {
            *warm = true;
            let mut t = 0.0;
            while t < s.warmup_sim_s {
                sim.g().update(0.02);
                t += 0.02;
            }
            // EARTHSIM_SHOT_CRASH=<km/h>: drive into a concrete block, let it settle, then capture
            if let Some(kmh) = std::env::var("EARTHSIM_SHOT_CRASH").ok().and_then(|v| v.parse::<f64>().ok()) {
                let mut g = sim.g();
                g.crash_test(kmh / 3.6, 2.0);
                let mut t = 0.0;
                while t < 2.5 {
                    g.update(0.02);
                    t += 0.02;
                }
            }
        }
        let mut g = sim.g();
        if let Some(h) = std::env::var("EARTHSIM_SHOT_HOUR").ok().and_then(|v| v.parse::<f32>().ok()) {
            g.hour = h;
        }
        g.update(1.0 / 60.0);
        // EARTHSIM_SHOT_CAM=side|front: fixed camera beside / in front of the car (for checking car models)
        // EARTHSIM_SHOT_CAM=interior: the driver's view
        let view = std::env::var("EARTHSIM_SHOT_CAM").ok();
        // EARTHSIM_SHOT_EYE="x,y,z,tx,ty,tz": free camera in local metres (checking generated buildings)
        let free: Option<Vec<f32>> = std::env::var("EARTHSIM_SHOT_EYE").ok().map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect());
        if let Some(e) = free.filter(|e| e.len() == 6) {
            g.cam_override = Some(([e[0], e[1], e[2]], [e[3], e[4], e[5]], 60.0));
        } else if view.as_deref() == Some("interior") {
            g.cam_mode = 1;
        } else if let Some(view) = view {
            let (p, f) = (g.car.pos, g.car.forward());
            let left = phys::v3(-f.y, f.x, 0.0);
            let eye = match view.as_str() {
                "front" => p + f * 6.0 + left * 2.5 + phys::v3(0.0, 0.0, 0.8),
                "crash" => p + f * 2.2 + left * 4.2 + phys::v3(0.0, 0.0, 1.3),
                "crashfront" => p + f * 4.6 + left * 3.4 + phys::v3(0.0, 0.0, 1.5),
                _ => p + left * 5.5 + f * 1.5 + phys::v3(0.0, 0.0, 0.6),
            };
            let tgt = match view.as_str() {
                "crash" => p + f * 1.2,
                "crashfront" => p + f * 1.9,
                _ => p,
            };
            g.cam_override = Some(([eye.x as f32, eye.y as f32, eye.z as f32], [tgt.x as f32, tgt.y as f32, (tgt.z - 0.1) as f32], 50.0));
        }
    } else {
        sim.g().update(time.delta_secs_f64());
    }
}
