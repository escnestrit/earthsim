//! Terrain surface: a grid mesh over the simulation's height field (20 m cells), rebuilt when the field
//! changes; the flat ground plane drops below it so it only shows beyond the terrain's edge.
use crate::view::{GroundPlane, WorldRoot};
use crate::Sim;
use bevy::asset::RenderAssetUsages;
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;
use scene::mesh::mat;

pub struct TerrainPlugin;

impl Plugin for TerrainPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<TerrainState>().add_systems(Update, update.after(crate::view::sync_camera_and_light));
    }
}

#[derive(Resource, Default)]
struct TerrainState {
    generation: u64,
    entity: Option<Entity>,
    plane_y: Option<f32>,
}

/// Grid mesh (sim coordinates, Z up) with `sub` vertices per field cell, at the physics' bilinear heights,
/// so the drawn ground and the ground the car stands on agree to a few centimetres.
pub fn terrain_mesh(f: &phys::HeightField, sub: usize) -> Mesh {
    let nx = (f.nx - 1) * sub + 1;
    let ny = (f.ny - 1) * sub + 1;
    let step = 1.0 / sub as f64;
    let mut pos = Vec::with_capacity(nx * ny);
    let mut nrm = Vec::with_capacity(nx * ny);
    let mut uv = Vec::with_capacity(nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let (x, y) = (f.x0 + i as f64 * step * f.cell, f.y0 + j as f64 * step * f.cell);
            // a little under the draped roads and ground cover
            pos.push([x as f32, y as f32, f.height(x, y) as f32 - 0.02]);
            let n = f.normal(x, y);
            nrm.push([n.x as f32, n.y as f32, n.z as f32]);
            uv.push([x as f32, y as f32]);
        }
    }
    let mut idx = Vec::with_capacity((nx - 1) * (ny - 1) * 6);
    for j in 0..ny - 1 {
        for i in 0..nx - 1 {
            let a = (j * nx + i) as u32;
            let b = a + 1;
            let c = a + nx as u32;
            let d = c + 1;
            idx.extend_from_slice(&[a, b, d, a, d, c]);
        }
    }
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, pos)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, nrm)
        .with_inserted_attribute(Mesh::ATTRIBUTE_TANGENT, vec![[1.0f32, 0.0, 0.0, 1.0]; nx * ny])
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uv)
        .with_inserted_indices(Indices::U32(idx))
}

fn update(
    mut commands: Commands,
    sim: Res<Sim>,
    mut st: ResMut<TerrainState>,
    mut meshes: ResMut<Assets<Mesh>>,
    tiles: Res<crate::view::Tiles>,
    root: Query<Entity, With<WorldRoot>>,
    mut plane: Query<&mut Transform, With<GroundPlane>>,
) {
    let (field, generation) = sim.g().terrain_field();
    if generation != st.generation {
        st.generation = generation;
        if let Some(e) = st.entity.take() {
            commands.entity(e).despawn();
        }
        st.plane_y = None;
        if !field.is_empty() {
            if let Ok(root) = root.single() {
                let m = meshes.add(terrain_mesh(&field, 1));
                let gm = tiles.materials[&(mat::GROUND as u32)].clone();
                let e = commands.spawn((Mesh3d(m), MeshMaterial3d(gm), Transform::default(), ChildOf(root))).id();
                st.entity = Some(e);
            }
            st.plane_y = Some(field.h.iter().cloned().fold(f32::MAX, f32::min) - 2.0);
        }
    }
    // the plane keeps following the car (view.rs); only its height is set here
    if let Some(y) = st.plane_y {
        for mut t in &mut plane {
            t.translation.y = y;
        }
    }
}
