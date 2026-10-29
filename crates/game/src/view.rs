//! Scene graph: streamed map tiles, the car (rigid meshes or the deformed soft body), ground, sun, sky, camera.
use crate::{Shot, Sim};
use bevy::asset::RenderAssetUsages;
use bevy::camera::RenderTarget;
use bevy::light::CascadeShadowConfigBuilder;
use bevy::prelude::*;
use bevy::render::render_resource::{PrimitiveTopology, TextureFormat, TextureUsages};
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use earthsim::game::RenderOp;
use scene::mesh::{mat, FLOATS_PER_VERTEX};
use scene::MeshData;
use std::collections::HashMap;

pub struct ViewPlugin;

impl Plugin for ViewPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Tiles>()
            .init_resource::<PendingOps>()
            .add_systems(Update, hitch_log)
            .add_systems(Startup, setup.in_set(SetupMarker))
            .add_systems(Update, (sync_world, sync_car, sync_camera_and_light, capture).chain());
    }
}

/// Systems that need the world root run after this set.
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct SetupMarker;
#[allow(non_upper_case_globals)]
pub const setup_marker: SetupMarker = SetupMarker;

/// Simulation (Z-up) to Bevy (Y-up).
pub fn zup(v: [f32; 3]) -> Vec3 {
    Vec3::new(v[0], v[2], -v[1])
}

#[derive(Component)]
pub struct WorldRoot;
#[derive(Component)]
pub struct GroundPlane;
#[derive(Component)]
pub struct CarBody;
#[derive(Component)]
pub struct CarWheel(pub usize);
#[derive(Component)]
pub struct SoftBody;
#[derive(Component)]
pub struct MainCamera;
#[derive(Component)]
pub struct Sun;

#[derive(Resource, Default)]
pub struct Tiles {
    map: HashMap<u64, Vec<Entity>>,
    pub materials: HashMap<u32, Handle<StandardMaterial>>,
    root: Option<Entity>,
    soft_mesh: Option<Handle<Mesh>>,
    pub facades: Vec<Handle<StandardMaterial>>,
    pub facade_depth: Vec<Handle<Image>>,
}

/// PBR parameters per material id of the shared mesh format (colour, roughness, metallic, depth bias).
pub fn material_for(id: u32) -> StandardMaterial {
    let (c, rough, metal, bias, emissive) = match id as f32 {
        x if x == mat::GROUND => ([0.30, 0.36, 0.24], 0.95, 0.0, 0.0, false),
        x if x == mat::SIDEWALK => ([0.55, 0.54, 0.52], 0.9, 0.0, 1.0, false),
        x if x == mat::ASPHALT => ([0.11, 0.11, 0.12], 0.85, 0.0, 2.0, false),
        x if x == mat::MARKING => ([0.85, 0.85, 0.82], 0.7, 0.0, 3.0, false),
        x if x == mat::WALL => ([0.72, 0.66, 0.58], 0.85, 0.0, 0.0, false),
        x if mat::is_facade(x) => ([0.85, 0.80, 0.70], 0.85, 0.0, 0.0, false),
        x if x == mat::ROOF => ([0.36, 0.30, 0.28], 0.8, 0.0, 0.0, false),
        x if x == mat::PAINT => ([0.42, 0.04, 0.05], 0.35, 0.3, 0.0, false),
        x if x == mat::GLASS => ([0.15, 0.19, 0.24], 0.08, 0.0, 0.0, false),
        x if x == mat::TIRE => ([0.04, 0.04, 0.04], 0.9, 0.0, 0.0, false),
        x if x == mat::HEADLIGHT => ([0.95, 0.95, 0.9], 0.2, 0.0, 0.0, true),
        x if x == mat::FOLIAGE => ([0.18, 0.32, 0.12], 0.9, 0.0, 0.0, false),
        x if x == mat::TRUNK => ([0.30, 0.22, 0.15], 0.9, 0.0, 0.0, false),
        x if x == mat::TAILLIGHT => ([0.6, 0.02, 0.02], 0.3, 0.0, 0.0, true),
        x if x == mat::RIM => ([0.7, 0.7, 0.72], 0.3, 0.9, 0.0, false),
        x if x == mat::GRASS => ([0.26, 0.40, 0.17], 0.95, 0.0, 0.5, false),
        x if x == mat::FOREST_FLOOR => ([0.17, 0.25, 0.11], 1.0, 0.0, 0.5, false),
        x if x == mat::SAND => ([0.74, 0.68, 0.50], 0.95, 0.0, 0.5, false),
        x if x == mat::PARKING => ([0.16, 0.16, 0.17], 0.85, 0.0, 0.7, false),
        x if x == mat::WATER => ([0.09, 0.19, 0.25], 0.06, 0.0, 0.8, false),
        _ => ([0.6, 0.6, 0.6], 0.8, 0.0, 0.0, false),
    };
    let col = Color::srgb(c[0], c[1], c[2]);
    StandardMaterial {
        base_color: col,
        perceptual_roughness: rough,
        metallic: metal,
        // water: strong sky reflection at grazing angles
        reflectance: if id as f32 == mat::WATER { 0.9 } else { 0.5 },
        depth_bias: bias,
        emissive: if emissive { LinearRgba::from(col) * 2.0 } else { LinearRgba::BLACK },
        ..default()
    }
}

/// Split one interleaved mesh (position, normal, uv, material id) into one Bevy mesh per material.
/// Per-triangle tangents (xyz + handedness) for a non-indexed triangle list; `flip_v` when the material
/// samples the texture with v negated.
pub fn tangents(p: &[[f32; 3]], n: &[[f32; 3]], uv: &[[f32; 2]], flip_v: bool) -> Vec<[f32; 4]> {
    let mut out = vec![[1.0, 0.0, 0.0, 1.0]; p.len()];
    let sv = if flip_v { -1.0 } else { 1.0 };
    for t in (0..p.len() / 3).map(|k| k * 3) {
        let (p0, p1, p2) = (Vec3::from(p[t]), Vec3::from(p[t + 1]), Vec3::from(p[t + 2]));
        let (u0, u1, u2) = (Vec2::new(uv[t][0], uv[t][1] * sv), Vec2::new(uv[t + 1][0], uv[t + 1][1] * sv), Vec2::new(uv[t + 2][0], uv[t + 2][1] * sv));
        let (e1, e2) = (p1 - p0, p2 - p0);
        let (d1, d2) = (u1 - u0, u2 - u0);
        let r = d1.x * d2.y - d2.x * d1.y;
        if r.abs() < 1e-12 {
            continue;
        }
        let tan = (e1 * d2.y - e2 * d1.y) / r;
        let bit = (e2 * d1.x - e1 * d2.x) / r;
        for k in 0..3 {
            let nn = Vec3::from(n[t + k]);
            let tt = (tan - nn * nn.dot(tan)).normalize_or(Vec3::X);
            let w = if nn.cross(tt).dot(bit) < 0.0 { -1.0 } else { 1.0 };
            out[t + k] = [tt.x, tt.y, tt.z, w];
        }
    }
    out
}

pub fn to_bevy_meshes(m: &MeshData) -> Vec<(u32, Mesh)> {
    type Parts = (Vec<[f32; 3]>, Vec<[f32; 3]>, Vec<[f32; 2]>, Vec<[f32; 4]>);
    let mut by_mat: HashMap<u32, Parts> = HashMap::new();
    let has_col = m.colors.len() * FLOATS_PER_VERTEX == m.verts.len() * 3;
    for (i, v) in m.verts.chunks_exact(FLOATS_PER_VERTEX).enumerate() {
        let id = v[8].round() as u32;
        let e = by_mat.entry(id).or_default();
        e.0.push([v[0], v[1], v[2]]);
        e.1.push([v[3], v[4], v[5]]);
        // textured ground is mapped by world position (metres), so ribbons and polygons do not stretch it
        e.2.push(if crate::ground::is_textured(id) { [v[0], v[1]] } else { [v[6], v[7]] });
        if has_col {
            // colours are sRGB; vertex colours are linear
            let c = LinearRgba::from(Color::srgb(m.colors[i * 3], m.colors[i * 3 + 1], m.colors[i * 3 + 2]));
            e.3.push([c.red, c.green, c.blue, 1.0]);
        }
    }
    let mut out: Vec<(u32, Mesh)> = by_mat
        .into_iter()
        .filter(|(_, (p, _, _, _))| p.len() >= 3)
        .map(|(id, (p, n, uv, col))| {
            let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::RENDER_WORLD)
                .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, p)
                .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, n)
                .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uv);
            if !col.is_empty() {
                mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, col);
            }
            // relief-mapped materials need tangents (facades sample v flipped, see the uv transform)
            if mat::is_facade(id as f32) || crate::ground::is_textured(id) {
                let flip = mat::is_facade(id as f32);
                let (Some(bevy::mesh::VertexAttributeValues::Float32x3(p)), Some(bevy::mesh::VertexAttributeValues::Float32x3(n)), Some(bevy::mesh::VertexAttributeValues::Float32x2(uv))) =
                    (mesh.attribute(Mesh::ATTRIBUTE_POSITION), mesh.attribute(Mesh::ATTRIBUTE_NORMAL), mesh.attribute(Mesh::ATTRIBUTE_UV_0))
                else {
                    return (id, mesh);
                };
                let t = tangents(p, n, uv, flip);
                mesh.insert_attribute(Mesh::ATTRIBUTE_TANGENT, t);
            }
            (id, mesh)
        })
        .collect();
    out.sort_by_key(|(id, _)| *id);
    out
}

pub fn mat_handle(t: &mut Tiles, materials: &mut Assets<StandardMaterial>, id: u32) -> Handle<StandardMaterial> {
    t.materials.entry(id).or_insert_with(|| materials.add(material_for(id))).clone()
}

fn spawn_meshdata(commands: &mut Commands, t: &mut Tiles, meshes: &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>, parent: Entity, m: &MeshData) -> Vec<Entity> {
    let mut ents = Vec::new();
    for (id, mesh) in to_bevy_meshes(m) {
        let h = mat_handle(t, materials, id);
        let mut e = commands.spawn((Mesh3d(meshes.add(mesh)), MeshMaterial3d(h), Transform::default(), ChildOf(parent)));
        if !m.casts_shadow {
            e.insert(bevy::light::NotShadowCaster);
        }
        ents.push(e.id());
    }
    ents
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut tiles: ResMut<Tiles>,
    sim: ResMut<Sim>,
    shot: Option<Res<Shot>>,
) {
    // facade materials (textures generated once at startup)
    let ids = crate::facade::styles();
    let generated: Vec<[Vec<u8>; 4]> = std::thread::scope(|sc| {
        let hs: Vec<_> = ids.iter().map(|&id| sc.spawn(move || crate::facade::generate(id))).collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for (st, [a, m, e, d]) in ids.into_iter().zip(generated) {
        let (a, m, e) = (images.add(crate::facade::image(a, true)), images.add(crate::facade::image(m, false)), images.add(crate::facade::image(e, true)));
        let d = images.add(crate::facade::image_fmt(d, TextureFormat::R8Unorm));
        let k = 1.0 / crate::facade::REPEAT_M;
        let h = materials.add(StandardMaterial {
            base_color: Color::WHITE,
            base_color_texture: Some(a),
            metallic_roughness_texture: Some(m),
            perceptual_roughness: 1.0,
            metallic: 1.0,
            emissive_texture: Some(e),
            emissive: LinearRgba::BLACK,
            uv_transform: bevy::math::Affine2::from_scale(Vec2::new(k, -k)),
            // recessed windows, reveals, sills, joints: parallax relief (quality settings tune or disable it)
            depth_map: Some(d.clone()),
            parallax_depth_scale: 0.022,
            max_parallax_layer_count: 16.0,
            parallax_mapping_method: bevy::pbr::ParallaxMappingMethod::Occlusion,
            ..default()
        });
        tiles.facade_depth.push(d);
        tiles.materials.insert(st as u32, h.clone());
        tiles.facades.push(h);
    }
    // textured ground materials (asphalt, pavers, grass...), generated in parallel
    let gids: Vec<u32> = [mat::ASPHALT, mat::SIDEWALK, mat::GRASS, mat::FOREST_FLOOR, mat::SAND, mat::PARKING, mat::GROUND].iter().map(|m| *m as u32).collect();
    let gtex: Vec<[Vec<u8>; 3]> = std::thread::scope(|sc| {
        let hs: Vec<_> = gids.iter().map(|&id| sc.spawn(move || crate::ground::generate(id))).collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for (id, [a, m, n]) in gids.iter().zip(gtex) {
        let img = |d: Vec<u8>, f: TextureFormat| crate::facade::image_fmt(d, f);
        let k = 1.0 / crate::ground::repeat_m(*id);
        let mut gm = StandardMaterial {
            base_color: Color::WHITE,
            base_color_texture: Some(images.add(img(a, TextureFormat::Rgba8UnormSrgb))),
            metallic_roughness_texture: Some(images.add(img(m, TextureFormat::Rgba8Unorm))),
            perceptual_roughness: 1.0,
            metallic: 1.0,
            normal_map_texture: Some(images.add(img(n, TextureFormat::Rgba8Unorm))),
            uv_transform: bevy::math::Affine2::from_scale(Vec2::splat(k)),
            depth_bias: material_for(*id).depth_bias,
            reflectance: 0.35,
            ..default()
        };
        if *id == mat::GROUND as u32 {
            gm.depth_bias = -0.5;
        }
        let h = materials.add(gm);
        tiles.materials.insert(*id, h);
    }
    let root = commands.spawn((WorldRoot, Transform::from_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)), Visibility::default())).id();
    tiles.root = Some(root);
    // ground: a large plane that follows the car (beyond the terrain mesh); grass every 10 m (the plane
    // snaps in 100 m steps, a whole number of repeats, so the texture does not jump)
    let mut gp = materials.get(&tiles.materials[&(mat::GROUND as u32)]).cloned().unwrap_or_else(|| material_for(mat::GROUND as u32));
    gp.uv_transform = bevy::math::Affine2::from_scale(Vec2::splat(8000.0 / 10.0));
    gp.depth_bias = 0.0;
    let ground = materials.add(gp);
    let mut plane = Plane3d::new(Vec3::Y, Vec2::splat(4000.0)).mesh().build();
    plane.insert_attribute(Mesh::ATTRIBUTE_TANGENT, vec![[1.0f32, 0.0, 0.0, 1.0]; plane.count_vertices()]);
    commands.spawn((GroundPlane, Mesh3d(meshes.add(plane)), MeshMaterial3d(ground), Transform::from_xyz(0.0, -0.02, 0.0)));
    // whole non-streamed scene (demo / scene file)
    if !sim.g().is_streaming() {
        let ms = sim.g().meshes.clone();
        let mut ents = Vec::new();
        for m in &ms {
            ents.extend(spawn_meshdata(&mut commands, &mut tiles, &mut meshes, &mut materials, root, m));
        }
        tiles.map.insert(u64::MAX, ents);
        sim.g().scene_dirty = false;
    }
    // car: body + 4 wheels (rigid view), deformed soft body (dynamic mesh)
    let (body, wheel) = sim.g().car_meshes.clone();
    let car = commands.spawn((CarBody, Transform::default(), Visibility::default(), ChildOf(root))).id();
    spawn_meshdata(&mut commands, &mut tiles, &mut meshes, &mut materials, car, &body);
    for i in 0..4 {
        let w = commands.spawn((CarWheel(i), Transform::default(), Visibility::default(), ChildOf(root))).id();
        spawn_meshdata(&mut commands, &mut tiles, &mut meshes, &mut materials, w, &wheel);
    }
    let soft = meshes.add(Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vec![[0.0f32; 3]; 3])
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0f32, 0.0, 1.0]; 3]));
    tiles.soft_mesh = Some(soft.clone());
    let paint = mat_handle(&mut tiles, &mut materials, mat::PAINT as u32);
    commands.spawn((SoftBody, Mesh3d(soft), MeshMaterial3d(paint), Transform::default(), Visibility::Hidden, ChildOf(root)));

    // sun with cascaded shadows sized for driving
    commands.spawn((
        Sun,
        DirectionalLight { shadow_maps_enabled: true, illuminance: 20_000.0, ..default() },
        CascadeShadowConfigBuilder { num_cascades: 3, first_cascade_far_bound: 25.0, maximum_distance: 400.0, ..default() }.build(),
        Transform::default(),
    ));
    commands.insert_resource(GlobalAmbientLight { color: Color::srgb(0.75, 0.82, 1.0), brightness: 900.0, ..default() });

    let fog = DistanceFog { color: Color::srgb(0.70, 0.78, 0.88), falloff: FogFalloff::Linear { start: 600.0, end: 2600.0 }, ..default() };
    let mut cam = commands.spawn((MainCamera, Camera3d::default(), bevy::camera::Exposure::SUNLIGHT, IsDefaultUiCamera, Projection::Perspective(PerspectiveProjection { far: 6000.0, ..default() }), fog, Transform::default()));
    if let Some(s) = shot {
        let mut img = Image::new_target_texture(1280, 720, TextureFormat::Rgba8UnormSrgb, None);
        img.texture_descriptor.usage |= TextureUsages::COPY_SRC;
        let h = images.add(img);
        cam.insert(RenderTarget::Image(h.clone().into()));
        commands.insert_resource(ShotTarget(h, s.frames));
    }
    commands.insert_resource(ClearColor(Color::srgb(0.55, 0.70, 0.90)));
}

#[derive(Resource)]
struct ShotTarget(Handle<Image>, u32);

/// Render ops waiting for their frame: adding a tile (mesh conversion, GPU upload) is spread over frames.
#[derive(Resource, Default)]
pub struct PendingOps {
    pub ops: std::collections::VecDeque<RenderOp>,
    /// what the last frame did (for the hitch log)
    pub last: String,
}

pub fn sync_world(mut commands: Commands, sim: ResMut<Sim>, mut tiles: ResMut<Tiles>, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>, mut pending: ResMut<PendingOps>) {
    let Some(root) = tiles.root else { return };
    pending.ops.extend(sim.g().take_render_ops());
    let t0 = std::time::Instant::now();
    let (mut added, mut removed) = (0, 0);
    // at most ~4 ms of tile additions per frame (always at least one); removals are cheap
    while let Some(op) = pending.ops.front() {
        if matches!(op, RenderOp::AddTile(..)) && added > 0 && t0.elapsed().as_secs_f64() > 0.004 {
            break;
        }
        let op = pending.ops.pop_front().unwrap();
        match op {
            RenderOp::AddTile(id, ms) => {
                if let Some(old) = tiles.map.remove(&id) {
                    for e in old {
                        commands.entity(e).despawn();
                    }
                }
                let mut ents = Vec::new();
                for m in ms.iter() {
                    ents.extend(spawn_meshdata(&mut commands, &mut tiles, &mut meshes, &mut materials, root, m));
                }
                tiles.map.insert(id, ents);
                added += 1;
            }
            RenderOp::RemoveTile(id) => {
                for e in tiles.map.remove(&id).unwrap_or_default() {
                    commands.entity(e).despawn();
                }
                removed += 1;
            }
            RenderOp::FollowGround(_) => {}
        }
    }
    pending.last = format!("tiles +{added} -{removed} in {:.1} ms, {} waiting", t0.elapsed().as_secs_f64() * 1000.0, pending.ops.len());
}

/// Logs frames that took long (stutter), with what the world sync did, so a player's log shows the cause.
pub fn hitch_log(time: Res<Time>, pending: Res<PendingOps>, mut n: Local<u32>) {
    let dt = time.delta_secs_f64();
    if dt > 0.12 && *n < 200 {
        *n += 1;
        world::log::log(&format!("slow frame {:.0} ms ({})", dt * 1000.0, pending.last));
    }
}

fn m4_to_transform(m: &[f32; 16]) -> Transform {
    Transform::from_matrix(Mat4::from_cols_array(m))
}

pub fn sync_car(
    sim: Res<Sim>,
    tiles: Res<Tiles>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut body: Query<(&mut Transform, &mut Visibility), (With<CarBody>, Without<CarWheel>, Without<SoftBody>)>,
    mut wheels: Query<(&CarWheel, &mut Transform, &mut Visibility), (Without<CarBody>, Without<SoftBody>)>,
    mut soft: Query<&mut Visibility, (With<SoftBody>, Without<CarBody>, Without<CarWheel>)>,
) {
    let g = sim.g();
    let fp = g.frame_params();
    let deformed = g.deformed_body();
    let show = |b: bool| if b { Visibility::Inherited } else { Visibility::Hidden };
    for (mut t, mut v) in &mut body {
        *t = m4_to_transform(&fp.body);
        *v = show(fp.draw_car && deformed.is_none());
    }
    for (w, mut t, mut v) in &mut wheels {
        *t = m4_to_transform(&fp.wheels[w.0]);
        *v = show(fp.draw_car);
    }
    for mut v in &mut soft {
        *v = show(fp.draw_car && deformed.is_some());
    }
    if let (Some(d), Some(h)) = (deformed, &tiles.soft_mesh) {
        if let Some(mut mesh) = meshes.get_mut(h) {
            let mut p = Vec::with_capacity(d.len() / FLOATS_PER_VERTEX);
            let mut n = Vec::with_capacity(p.capacity());
            for v in d.chunks_exact(FLOATS_PER_VERTEX) {
                p.push([v[0], v[1], v[2]]);
                n.push([v[3], v[4], v[5]]);
            }
            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, p);
            mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, n);
        }
    }
}

pub fn sync_camera_and_light(
    sim: Res<Sim>,
    mut cam: Query<(&mut Transform, &mut Projection, &mut DistanceFog), (With<MainCamera>, Without<Sun>, Without<GroundPlane>)>,
    mut sun: Query<(&mut Transform, &mut DirectionalLight), (With<Sun>, Without<MainCamera>, Without<GroundPlane>)>,
    mut ground: Query<&mut Transform, (With<GroundPlane>, Without<MainCamera>, Without<Sun>)>,
    mut clear: ResMut<ClearColor>,
    mut amb: ResMut<GlobalAmbientLight>,
    tiles: Res<Tiles>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut last_night: Local<f32>,
) {
    let fp = sim.g().frame_params();
    let lg = earthsim::env::lighting(fp.hour);
    let eye = zup(fp.eye);
    let target = zup(fp.target);
    let sky = Color::srgb(lg.horizon[0], lg.horizon[1], lg.horizon[2]);
    for (mut t, mut p, mut fog) in &mut cam {
        *t = Transform::from_translation(eye).looking_at(target, zup(fp.up).normalize_or(Vec3::Y));
        if let Projection::Perspective(pp) = p.as_mut() {
            pp.fov = fp.fov_deg.to_radians();
            pp.near = if fp.interior { 0.03 } else { 0.1 };
        }
        fog.color = sky;
    }
    clear.0 = sky;
    let to_light = zup(lg.light_dir).normalize_or(Vec3::Y);
    for (mut t, mut l) in &mut sun {
        *t = Transform::from_translation(zup(fp.car_pos) + to_light * 100.0).looking_at(zup(fp.car_pos), Vec3::Y);
        l.color = Color::srgb(lg.sun_col[0], lg.sun_col[1], lg.sun_col[2]);
        // moonlight-dark at night (exposure adapts in lights.rs), so lamps and headlights carry the scene
        let night = ((0.35 - lg.day) / 0.3).clamp(0.0, 1.0);
        l.illuminance = (400.0 * (1.0 - night) + 0.3 * night) + 100_000.0 * lg.sun_i.clamp(0.0, 1.5) * lg.day.clamp(0.0, 1.0);
    }
    // lit windows fade in at dusk (only touch the materials when the level changes)
    let night = ((0.35 - lg.day) / 0.3).clamp(0.0, 1.0);
    if (night - *last_night).abs() > 0.01 {
        *last_night = night;
        for h in &tiles.facades {
            if let Some(mut m) = materials.get_mut(h) {
                m.emissive = LinearRgba::rgb(1.0, 1.0, 1.0) * (night * 2.5);
            }
        }
    }
    amb.color = Color::srgb(lg.sky_amb[0], lg.sky_amb[1], lg.sky_amb[2]);
    let night = ((0.35 - lg.day) / 0.3).clamp(0.0, 1.0);
    amb.brightness = (150.0 + 6_000.0 * lg.day) * (1.0 - night) + 8.0 * night;
    // keep the ground plane under the car, snapped so its texture would not swim
    let c = zup(fp.car_pos);
    for mut g in &mut ground {
        g.translation = Vec3::new((c.x / 100.0).round() * 100.0, -0.02, (c.z / 100.0).round() * 100.0);
    }
}

fn capture(mut commands: Commands, target: Option<Res<ShotTarget>>, shot: Option<Res<Shot>>, mut frame: Local<u32>, mut exit: MessageWriter<AppExit>) {
    let (Some(t), Some(s)) = (target, shot) else { return };
    *frame += 1;
    if *frame == t.1 {
        eprintln!("earthsim: capturing frame {} -> {}", *frame, s.path);
        commands.spawn(Screenshot::image(t.0.clone())).observe(save_to_disk(s.path.clone()));
    }
    if *frame == t.1 + 120 {
        exit.write(AppExit::Success);
    }
}
