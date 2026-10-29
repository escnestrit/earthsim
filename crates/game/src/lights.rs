//! Night lighting: exposure that adapts from daylight to street lighting, actual light from the street lamps
//! nearest the camera (a pool of spot lights moved to them), and the car's headlights.
use crate::view::{MainCamera, WorldRoot};
use crate::Sim;
use bevy::prelude::*;

pub struct LightsPlugin;

impl Plugin for LightsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup.after(crate::view::setup_marker)).add_systems(Update, update.after(crate::view::sync_car));
    }
}

/// Spot lights moved to the nearest street lamps at night (no shadows: cheap).
const POOL: usize = 24;

#[derive(Component)]
struct Lamp;
#[derive(Component)]
struct Headlight(f32);

fn setup(mut commands: Commands, root: Query<Entity, With<WorldRoot>>) {
    let Ok(root) = root.single() else { return };
    for _ in 0..POOL {
        commands.spawn((
            Lamp,
            SpotLight { intensity: 0.0, range: 30.0, radius: 0.2, outer_angle: 1.15, inner_angle: 0.5, shadow_maps_enabled: false, color: Color::srgb(1.0, 0.82, 0.6), ..default() },
            Transform::default(),
            Visibility::Hidden,
            ChildOf(root),
        ));
    }
    for side in [-0.65f32, 0.65] {
        commands.spawn((
            Headlight(side),
            SpotLight { intensity: 0.0, range: 70.0, radius: 0.1, outer_angle: 0.42, inner_angle: 0.2, shadow_maps_enabled: false, color: Color::srgb(1.0, 0.96, 0.9), ..default() },
            Transform::default(),
            Visibility::Hidden,
            ChildOf(root),
        ));
    }
}

#[allow(clippy::type_complexity)]
fn update(
    sim: Res<Sim>,
    mut lamps: Query<(&mut Transform, &mut SpotLight, &mut Visibility), (With<Lamp>, Without<Headlight>)>,
    mut heads: Query<(&Headlight, &mut Transform, &mut SpotLight, &mut Visibility), Without<Lamp>>,
    mut cam: Query<&mut bevy::camera::Exposure, With<MainCamera>>,
    mut frame: Local<u32>,
) {
    *frame += 1;
    let mut g = sim.g();
    let fp = g.frame_params();
    let lg = earthsim::env::lighting(fp.hour);
    // 0 by day, 1 at night
    let night = ((0.35 - lg.day) / 0.3).clamp(0.0, 1.0);
    // exposure: sunlight (EV100 15) to a street-lit night (EV100 ~7)
    for mut e in &mut cam {
        e.ev100 = 15.0 - 8.0 * night;
    }
    // street lamps: every few frames pick the ones nearest the camera
    if night > 0.01 {
        if *frame % 6 == 0 {
            let near = g.nearest_lamps(fp.eye[0] as f64, fp.eye[1] as f64, POOL);
            if *frame % 120 == 0 && std::env::var("EARTHSIM_DEBUG_LAMPS").is_ok() {
                world::log::log(&format!("lamps near camera: {} (first {:?})", near.len(), near.first()));
            }
            for (i, (mut t, mut s, mut v)) in lamps.iter_mut().enumerate() {
                match near.get(i) {
                    Some(p) => {
                        let pos = Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32);
                        // spot lights shine along their local -Z; the world root is Z-up
                        *t = Transform::from_translation(pos).looking_at(pos - Vec3::Z, Vec3::X);
                        s.intensity = 3_000_000.0 * night;
                        *v = Visibility::Inherited;
                    }
                    None => *v = Visibility::Hidden,
                }
            }
        }
    } else {
        for (_, _, mut v) in &mut lamps {
            *v = Visibility::Hidden;
        }
    }
    // headlights from the car's front, aimed slightly down
    let on = fp.headlights;
    let hp = Vec3::from(fp.head_pos);
    let hd = Vec3::from(fp.head_dir).normalize_or(Vec3::X);
    let side = Vec3::new(-hd.y, hd.x, 0.0).normalize_or(Vec3::Y);
    for (h, mut t, mut s, mut v) in &mut heads {
        if !on {
            *v = Visibility::Hidden;
            continue;
        }
        let p = hp + side * h.0 - hd * 0.3;
        *t = Transform::from_translation(p).looking_at(p + hd * 10.0 - Vec3::Z * 0.6, Vec3::Z);
        s.intensity = 3_500_000.0;
        *v = Visibility::Inherited;
    }
    drop(g);
}
