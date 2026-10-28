use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, shot)
        .run();
}

fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
    commands.spawn((Mesh3d(meshes.add(Plane3d::default().mesh().size(5.0, 5.0))), MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3)))));
    commands.spawn((Mesh3d(meshes.add(Cuboid::default())), MeshMaterial3d(materials.add(Color::srgb(0.8, 0.7, 0.6))), Transform::from_xyz(0.0, 0.5, 0.0)));
    commands.spawn((DirectionalLight { shadow_maps_enabled: true, ..default() }, Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y)));
    commands.spawn((Camera3d::default(), Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y)));
}

fn shot(mut commands: Commands, mut frame: Local<u32>, mut exit: MessageWriter<AppExit>) {
    *frame += 1;
    if let Ok(p) = std::env::var("EARTHSIM_SHOT") {
        if *frame == 30 {
            commands.spawn(Screenshot::primary_window()).observe(save_to_disk(p));
        }
        if *frame == 60 {
            exit.write(AppExit::Success);
        }
    }
}
