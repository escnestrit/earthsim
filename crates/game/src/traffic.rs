//! Draws the traffic cars of the simulation (see earthsim::traffic): one entity per car under the world
//! root, sharing a few prebuilt low-detail car meshes.
use crate::view::WorldRoot;
use crate::Sim;
use bevy::prelude::*;
use std::collections::HashMap;

const VARIANTS: u64 = 16;

pub struct TrafficPlugin;

impl Plugin for TrafficPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TrafficView>().add_systems(Update, sync.after(crate::view::sync_camera_and_light));
    }
}

type Parts = Vec<(Handle<Mesh>, Handle<StandardMaterial>)>;

#[derive(Resource, Default)]
struct TrafficView {
    /// per variant: (mesh, material) parts
    parts: Vec<Parts>,
    cars: HashMap<u64, Entity>,
    /// people: per look, per pose (standing, stride, other stride)
    people_parts: Vec<[Parts; 3]>,
    /// person -> (entity, pose shown)
    people: HashMap<u64, (Entity, u8)>,
}

const LOOKS: u64 = 12;

fn variant_seed(k: u64) -> u64 {
    (k.wrapping_add(1)).wrapping_mul(0x2545_F491_4F6C_DD1D) >> 7
}

fn sync(mut commands: Commands, sim: Res<Sim>, mut tv: ResMut<TrafficView>, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>, mut tiles: ResMut<crate::view::Tiles>, root: Query<Entity, With<WorldRoot>>, mut tf: Query<&mut Transform>) {
    let Ok(root) = root.single() else { return };
    let cars = sim.g().traffic_cars();
    if tv.parts.is_empty() {
        for k in 0..VARIANTS {
            let (mut m, mut col) = (scene::mesh::Mb::default(), Vec::new());
            scene::mesh::simple_car(&mut m, &mut col, &scene::model::ParkedCar { x: 0.0, y: 0.0, yaw: 0.0, seed: variant_seed(k) }, true);
            let md = scene::mesh::MeshData { name: "traffic_car".into(), verts: m.a, polygon_offset: 0.0, casts_shadow: true, colors: col };
            let parts = crate::view::to_bevy_meshes(&md)
                .into_iter()
                .map(|(id, mesh)| (meshes.add(mesh), crate::view::mat_handle(&mut tiles, &mut materials, id)))
                .collect();
            tv.parts.push(parts);
        }
    }
    let people = sim.g().people_list();
    if tv.people_parts.is_empty() {
        for k in 0..LOOKS {
            let mk = |stride: f64, meshes: &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>, tiles: &mut crate::view::Tiles| -> Parts {
                let (mut m, mut col) = (scene::mesh::Mb::default(), Vec::new());
                scene::mesh::person(&mut m, &mut col, variant_seed(k + 100), stride);
                let md = scene::mesh::MeshData { name: "person".into(), verts: m.a, polygon_offset: 0.0, casts_shadow: true, colors: col };
                crate::view::to_bevy_meshes(&md).into_iter().map(|(id, mesh)| (meshes.add(mesh), crate::view::mat_handle(tiles, materials, id))).collect()
            };
            let a = mk(0.0, &mut meshes, &mut materials, &mut tiles);
            let b = mk(1.0, &mut meshes, &mut materials, &mut tiles);
            let c = mk(-1.0, &mut meshes, &mut materials, &mut tiles);
            tv.people_parts.push([a, b, c]);
        }
    }
    let mut seen_p = Vec::with_capacity(people.len());
    for (id, seed, p, yaw, pose) in people {
        seen_p.push(id);
        let t = Transform::from_translation(Vec3::new(p[0], p[1], p[2])).with_rotation(Quat::from_rotation_z(yaw));
        let look = (seed % LOOKS) as usize;
        match tv.people.get(&id).copied() {
            Some((e, shown)) if shown == pose => {
                if let Ok(mut cur) = tf.get_mut(e) {
                    *cur = t;
                }
            }
            other => {
                // new person, or a new walking pose: rebuild its parts
                if let Some((e, _)) = other {
                    commands.entity(e).despawn();
                }
                let e = commands.spawn((t, Visibility::default(), ChildOf(root))).id();
                for (mh, mat) in tv.people_parts[look][pose as usize % 3].clone() {
                    commands.spawn((Mesh3d(mh), MeshMaterial3d(mat), Transform::default(), ChildOf(e)));
                }
                tv.people.insert(id, (e, pose));
            }
        }
    }
    let gone: Vec<u64> = tv.people.keys().copied().filter(|id| !seen_p.contains(id)).collect();
    for id in gone {
        if let Some((e, _)) = tv.people.remove(&id) {
            commands.entity(e).despawn();
        }
    }
    let mut seen = Vec::with_capacity(cars.len());
    for (id, seed, p, yaw) in cars {
        seen.push(id);
        let t = Transform::from_translation(Vec3::new(p[0], p[1], p[2])).with_rotation(Quat::from_rotation_z(yaw));
        if let Some(&e) = tv.cars.get(&id) {
            if let Ok(mut cur) = tf.get_mut(e) {
                *cur = t;
            }
            continue;
        }
        // the car's own seed picks one of the prebuilt variants
        let v = (seed % VARIANTS) as usize;
        let e = commands.spawn((t, Visibility::default(), ChildOf(root))).id();
        for (mh, mat) in tv.parts[v].clone() {
            commands.spawn((Mesh3d(mh), MeshMaterial3d(mat), Transform::default(), ChildOf(e)));
        }
        tv.cars.insert(id, e);
    }
    let gone: Vec<u64> = tv.cars.keys().copied().filter(|id| !seen.contains(id)).collect();
    for id in gone {
        if let Some(e) = tv.cars.remove(&id) {
            commands.entity(e).despawn();
        }
    }
}
