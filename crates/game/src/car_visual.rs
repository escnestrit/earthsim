//! Draws a BeamNG vehicle mod with its own meshes and materials: flexbodies skinned to the soft-body nodes,
//! wheels as rigid meshes spun at the axle. The whole car moves by one transform per frame; a part's vertices
//! are recomputed only when its nodes moved relative to the body (suspension travel, crash damage).
use crate::view::{CarBody, CarWheel, SoftBody, WorldRoot};
use crate::Sim;
use bevy::asset::RenderAssetUsages;
use bevy::image::{CompressedImageFormats, ImageSampler, ImageType};
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;
use bevy::render::renderer::RenderDevice;
use jbeam::flex::{self, FlexPart};
use jbeam::materials::MatDef;
use phys::math::V3;
use std::collections::HashMap;

pub struct CarVisualPlugin;

impl Plugin for CarVisualPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup.after(crate::view::setup_marker))
            .add_systems(Update, update.after(crate::view::sync_car));
    }
}

/// re-skin a part when one of its nodes moved this far relative to the body since the last time (m)
const RESKIN_EPS: f64 = 0.008;
/// cap on vertices recomputed per frame (the rest waits a frame)
const RESKIN_BUDGET: usize = 120_000;

struct PartVis {
    part: FlexPart,
    meshes: Vec<Option<Handle<Mesh>>>,
    /// entity per submesh (None: never drawn)
    ents: Vec<Option<Entity>>,
    snap: Vec<V3>,
}

/// A drawn submesh of the car (material/visibility change with damage).
#[derive(Component)]
struct CarSubmesh;

#[derive(Resource)]
struct CarVis {
    root: Entity,
    parts: Vec<PartVis>,
    /// (entity, wheel index)
    wheels: Vec<(Entity, usize)>,
    rest_com: V3,
    cursor: usize,
    pos: Vec<[f32; 3]>,
    nrm: Vec<[f32; 3]>,
    mats: HashMap<String, Handle<StandardMaterial>>,
    /// damage-group generation the materials reflect
    groups_gen: u32,
    /// (entity, prop) for the steering wheel and other node-carried meshes
    props: Vec<(Entity, jbeam::props::Prop)>,
}

/// A node-carried rigid car mesh (steering wheel).
#[derive(Component)]
struct CarProp;

fn quat(q: phys::math::Quat) -> Quat {
    Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.w as f32)
}
fn vec3(v: V3) -> Vec3 {
    Vec3::new(v.x as f32, v.y as f32, v.z as f32)
}

fn material(md: Option<&MatDef>, tex: &HashMap<String, Handle<Image>>) -> StandardMaterial {
    let Some(md) = md else {
        return StandardMaterial { base_color: Color::srgb(0.35, 0.35, 0.36), perceptual_roughness: 0.7, ..default() };
    };
    let c = md.base_color;
    StandardMaterial {
        base_color: Color::srgba(c[0], c[1], c[2], if md.translucent { c[3].min(0.45) } else { 1.0 }),
        base_color_texture: md.base_map.as_ref().and_then(|t| tex.get(t).cloned()),
        perceptual_roughness: md.roughness.clamp(0.05, 1.0),
        metallic: md.metallic.clamp(0.0, 1.0),
        alpha_mode: if md.translucent { AlphaMode::Blend } else { AlphaMode::Opaque },
        double_sided: md.translucent,
        cull_mode: if md.translucent { None } else { Some(bevy::render::render_resource::Face::Back) },
        ..default()
    }
}

fn make_mesh(pos: Vec<[f32; 3]>, nrm: Vec<[f32; 3]>, uv: Vec<[f32; 2]>, idx: Vec<u32>) -> Mesh {
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, pos)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, nrm)
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uv)
        .with_inserted_indices(Indices::U32(idx))
}

fn setup(
    mut commands: Commands,
    sim: Res<Sim>,
    root: Query<Entity, With<WorldRoot>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    device: Option<Res<RenderDevice>>,
) {
    let mut g = sim.g();
    let Some(vis) = g.jcar_visual.take() else { return };
    let Some(jc) = g.jcar.as_ref() else { return };
    let guard = jc.lock();
    let j = &guard.v;
    let Ok(world_root) = root.single() else { return };
    let formats = device.map_or(CompressedImageFormats::NONE, |d| CompressedImageFormats::from_features(d.features()));
    // textures
    let mut tex: HashMap<String, Handle<Image>> = HashMap::new();
    let mut failed = 0;
    // EARTHSIM_CAR_TEXTURES=0 skips textures (used for software-rendered test captures)
    let skip_tex = std::env::var("EARTHSIM_CAR_TEXTURES").is_ok_and(|v| v == "0");
    for (name, bytes) in vis.textures.iter().filter(|_| !skip_tex) {
        let ext = name.rsplit('.').next().unwrap_or("").to_ascii_lowercase();
        let srgb = !name.to_lowercase().contains(".normal");
        match Image::from_buffer(bytes, ImageType::Extension(&ext), formats, srgb, ImageSampler::linear(), RenderAssetUsages::RENDER_WORLD) {
            Ok(img) => {
                tex.insert(name.clone(), images.add(img));
            }
            Err(e) => {
                failed += 1;
                world::log::log(&format!("texture {name}: {e}"));
            }
        }
    }
    let mut mat_handles: HashMap<String, Handle<StandardMaterial>> = HashMap::new();
    let car_root = commands.spawn((Transform::default(), Visibility::default(), ChildOf(world_root))).id();
    let mut parts = Vec::new();
    let mut wheels = Vec::new();
    for part in vis.parts {
        let parent = match part.wheel {
            Some(wi) => {
                let e = commands.spawn((Transform::default(), Visibility::default(), ChildOf(world_root))).id();
                wheels.push((e, wi));
                e
            }
            None => car_root,
        };
        // wheel meshes are re-expressed in the wheel's own rest frame
        let wheel_frame = part.wheel.map(|wi| {
            let w = &j.wheels[wi];
            let (a, b) = (j.body.nodes[w.n1].rest, j.body.nodes[w.n2].rest);
            let c = (a + b) * 0.5;
            let mut y = (a - b).norm();
            if y.y < 0.0 {
                y = y * -1.0;
            }
            let x = y.cross(phys::math::V3::Z).norm();
            let z = x.cross(y);
            (c, x, y, z)
        });
        let mut handles = Vec::new();
        let mut ents = Vec::new();
        // the damaged material of a swap is made up front (textures are only at hand here)
        if let Some(w) = part.swap.as_ref().filter(|w| !flex::is_invisible_material(&w.damaged)) {
            mat_handles.entry(w.damaged.clone()).or_insert_with(|| materials.add(material(vis.materials.get(&w.damaged), &tex)));
        }
        for sm in &part.submeshes {
            let invisible = flex::is_invisible_material(&sm.material);
            let swaps = part.swap.as_ref().is_some_and(|w| w.base == sm.material);
            if invisible && !swaps {
                handles.push(None);
                ents.push(None);
                continue;
            }
            // an invisible base (crack overlay) still needs a material until it is swapped in
            let key = if invisible { part.swap.as_ref().map(|w| w.damaged.clone()).unwrap_or_default() } else { sm.material.clone() };
            let mh = mat_handles.entry(key.clone()).or_insert_with(|| materials.add(material(vis.materials.get(&key), &tex))).clone();
            let (pos, nrm) = match wheel_frame {
                Some((c, x, y, z)) => {
                    let to = |p: &[f32; 3]| {
                        let d = phys::math::v3(p[0] as f64, p[1] as f64, p[2] as f64) - c;
                        [d.dot(x) as f32, d.dot(y) as f32, d.dot(z) as f32]
                    };
                    let tn = |n: &[f32; 3]| {
                        let d = phys::math::v3(n[0] as f64, n[1] as f64, n[2] as f64);
                        [d.dot(x) as f32, d.dot(y) as f32, d.dot(z) as f32]
                    };
                    (sm.positions.iter().map(to).collect(), sm.normals.iter().map(tn).collect())
                }
                None => (sm.positions.clone(), sm.normals.clone()),
            };
            let h = meshes.add(make_mesh(pos, nrm, sm.uvs.clone(), sm.indices.clone()));
            let e = commands
                .spawn((CarSubmesh, Mesh3d(h.clone()), MeshMaterial3d(mh), Transform::default(), if invisible { Visibility::Hidden } else { Visibility::Inherited }, ChildOf(parent)))
                .id();
            handles.push(Some(h));
            ents.push(Some(e));
        }
        let snap = part.nodes.iter().map(|&n| j.body.nodes[n as usize].rest).collect();
        parts.push(PartVis { part, meshes: handles, ents, snap });
    }
    world::log::log(&format!("car meshes: {} parts, {} textures ({failed} failed), {} materials", parts.len(), tex.len(), mat_handles.len()));
    let rest_com = j.rest_com();
    let mut props = Vec::new();
    for pr in vis.props {
        let e = commands.spawn((CarProp, Transform::default(), Visibility::default(), ChildOf(car_root))).id();
        for sm in &pr.submeshes {
            if flex::is_invisible_material(&sm.material) {
                continue;
            }
            let mh = mat_handles.entry(sm.material.clone()).or_insert_with(|| materials.add(material(vis.materials.get(&sm.material), &tex))).clone();
            let h = meshes.add(make_mesh(sm.positions.clone(), sm.normals.clone(), sm.uvs.clone(), sm.indices.clone()));
            commands.spawn((Mesh3d(h), MeshMaterial3d(mh), Transform::default(), ChildOf(e)));
        }
        props.push((e, pr));
    }
    let groups_gen = j.groups.generation;
    commands.insert_resource(CarVis { root: car_root, parts, wheels, rest_com, cursor: 0, pos: Vec::new(), nrm: Vec::new(), mats: mat_handles, groups_gen, props });
}

#[allow(clippy::type_complexity)]
fn update(
    sim: Res<Sim>,
    vis: Option<ResMut<CarVis>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut tf: Query<(&mut Transform, &mut Visibility), (Without<CarBody>, Without<SoftBody>, Without<CarWheel>, Without<CarSubmesh>, Without<CarProp>)>,
    mut hide: Query<&mut Visibility, (Or<(With<SoftBody>, With<CarWheel>, With<CarBody>)>, Without<CarSubmesh>)>,
    mut subs: Query<(&mut MeshMaterial3d<StandardMaterial>, &mut Visibility), (With<CarSubmesh>, Without<CarBody>, Without<SoftBody>, Without<CarWheel>)>,
    mut prop_tf: Query<&mut Transform, (With<CarProp>, Without<CarBody>, Without<SoftBody>, Without<CarWheel>, Without<CarSubmesh>)>,
) {
    let Some(mut vis) = vis else { return };
    let g = sim.g();
    let active = g.soft_on && g.jcar.is_some() && g.frame_params().draw_car;
    let show = if active { Visibility::Inherited } else { Visibility::Hidden };
    let wheel_ents: Vec<(Entity, usize)> = vis.wheels.clone();
    if let Ok((_, mut v)) = tf.get_mut(vis.root) {
        *v = show;
    }
    for (e, _) in &wheel_ents {
        if let Ok((_, mut v)) = tf.get_mut(*e) {
            *v = show;
        }
    }
    if !active {
        return;
    }
    // the mod's own meshes replace the placeholder body and wheels
    for mut v in &mut hide {
        *v = Visibility::Hidden;
    }
    // copy what is needed and let the physics thread go on
    let rc = vis.rest_com;
    let (r, pos, nodes, wheel_tf, groups, steer) = {
        let guard = g.jcar.as_ref().unwrap().lock();
        let j = &guard.v;
        let steer = j.steer_cmd;
        let groups = (j.groups.generation != vis.groups_gen).then(|| (j.groups.generation, j.groups.triggered_names()));
        let (_, _, up) = j.axes();
        let wheel_tf: Vec<(Vec3, Mat3, f32)> = wheel_ents
            .iter()
            .map(|&(_, wi)| {
                let y = j.wheel_axis(wi);
                let x = y.cross(up).norm();
                let z = x.cross(y);
                (vec3(j.wheel_center(wi)), Mat3::from_cols(vec3(x), vec3(y), vec3(z)), j.wheels[wi].state.spin as f32)
            })
            .collect();
        (j.body_rotation(), j.pos, j.body.nodes.iter().map(|n| n.p).collect::<Vec<V3>>(), wheel_tf, groups, steer)
    };
    drop(g);
    // damage groups changed (glass cracked, lights broken, or everything repaired): swap materials
    if let Some((gen, triggered)) = groups {
        vis.groups_gen = gen;
        for pv in &vis.parts {
            let Some(w) = &pv.part.swap else { continue };
            let hit = triggered.iter().any(|t| *t == w.group);
            let name = if hit { &w.damaged } else { &w.base };
            for (sm, e) in pv.part.submeshes.iter().zip(&pv.ents) {
                let (Some(e), true) = (e, sm.material == w.base) else { continue };
                if let Ok((mut m, mut v)) = subs.get_mut(*e) {
                    let hidden = flex::is_invisible_material(name);
                    *v = if hidden { Visibility::Hidden } else { Visibility::Inherited };
                    if let Some(h) = vis.mats.get(name) {
                        m.0 = h.clone();
                    }
                }
            }
        }
    }
    if let Ok((mut t, _)) = tf.get_mut(vis.root) {
        *t = Transform { translation: vec3(pos - r.rotate(rc)), rotation: quat(r), scale: Vec3::ONE };
    }
    for ((e, _), (c, basis, spin)) in wheel_ents.iter().zip(wheel_tf) {
        if let Ok((mut t, _)) = tf.get_mut(*e) {
            *t = Transform { translation: c, rotation: Quat::from_mat3(&basis) * Quat::from_rotation_y(spin), scale: Vec3::ONE };
        }
    }
    // node positions in the rest frame
    let inv = r.conj();
    let local: Vec<V3> = nodes.iter().map(|p| inv.rotate(*p - pos) + rc).collect();
    // steering wheel and other props follow their three nodes (rest-frame coordinates, like the car root)
    if !vis.props.is_empty() {
        let lp: Vec<[f64; 3]> = local.iter().map(|p| [p.x, p.y, p.z]).collect();
        for (e, pr) in &vis.props {
            let (o, m) = pr.pose(&lp, steer);
            if let Ok(mut t) = prop_tf.get_mut(*e) {
                let cols = Mat3::from_cols(
                    Vec3::new(m[0][0] as f32, m[1][0] as f32, m[2][0] as f32),
                    Vec3::new(m[0][1] as f32, m[1][1] as f32, m[2][1] as f32),
                    Vec3::new(m[0][2] as f32, m[1][2] as f32, m[2][2] as f32),
                );
                *t = Transform { translation: Vec3::new(o[0] as f32, o[1] as f32, o[2] as f32), rotation: Quat::from_mat3(&cols), scale: Vec3::ONE };
                if std::env::var("EARTHSIM_DEBUG_PROPS").is_ok() {
                    let wp = pos + r.rotate(phys::math::v3(o[0], o[1], o[2]) - rc);
                    world::log::log(&format!("prop {} at {:?} rot {:?}; world (sim) {:.2} {:.2} {:.2}", pr.mesh, t.translation, t.rotation, wp.x, wp.y, wp.z));
                }
            }
        }
    }
    let n = vis.parts.len();
    let mut budget = RESKIN_BUDGET;
    let start = vis.cursor;
    let CarVis { parts, pos, nrm, cursor, .. } = &mut *vis;
    for k in 0..n {
        let pi = (start + k) % n;
        let pv = &mut parts[pi];
        if pv.part.wheel.is_some() {
            continue;
        }
        let moved = pv.part.nodes.iter().zip(&pv.snap).any(|(&ni, s)| (local[ni as usize] - *s).len2() > RESKIN_EPS * RESKIN_EPS);
        if !moved {
            continue;
        }
        let verts: usize = pv.part.binds.iter().map(|b| b.len()).sum();
        if verts > budget && budget < RESKIN_BUDGET {
            *cursor = pi; // continue here next frame
            break;
        }
        budget = budget.saturating_sub(verts);
        for (b, h) in pv.part.binds.iter().zip(&pv.meshes) {
            let Some(h) = h else { continue };
            flex::deform(b, &local, pos, nrm);
            if let Some(mut m) = meshes.get_mut(h) {
                m.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos.clone());
                m.insert_attribute(Mesh::ATTRIBUTE_NORMAL, nrm.clone());
            }
        }
        pv.snap = pv.part.nodes.iter().map(|&ni| local[ni as usize]).collect();
    }
}
