//! Integration: load the exported demo scene, build meshes, and drive the physics car through it.
use phys::*;
use scene::{scene_meshes, Scene};

fn load() -> Scene {
    let s = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/demo_scene.json")).expect("demo scene present");
    Scene::from_json_str(&s).expect("demo scene parses")
}

#[test]
fn demo_scene_loads_and_meshes() {
    let sc = load();
    assert!(sc.roads.len() > 10 && sc.buildings.len() > 50, "{} roads {} buildings", sc.roads.len(), sc.buildings.len());
    let ms = scene_meshes(&sc);
    let tris: usize = ms.iter().map(|m| m.vertex_count() / 3).sum();
    assert!(tris > 10_000, "only {tris} triangles");
    for m in &ms {
        assert!(m.verts.iter().all(|v| v.is_finite()), "{}", m.name);
    }
}

#[test]
fn car_drives_down_the_spawn_road_without_hitting_anything_for_a_while() {
    let sc = load();
    let world = sc.static_world();
    let g = FlatGround { grip: 1.0 };
    let sp = sc.spawn();
    let mut car = Vehicle::new(VehicleParams::sedan(), sp.x, sp.y, sp.yaw, &g);
    let mut drv = Driver::new(car.p.steer_max);
    let keys = Keys { up: true, ..Default::default() };
    let mut moved = 0.0;
    for _ in 0..8000 {
        let t = car.telemetry();
        let inp = drv.update(0.001, &keys, t.forward_speed);
        car.step(0.001, &inp, &g, &world);
        assert!(car.pos.is_finite());
    }
    moved += (car.pos.x - sp.x).hypot(car.pos.y - sp.y);
    assert!(moved > 20.0, "car only moved {moved} m in 8 s");
}

#[test]
fn spawn_point_is_not_inside_any_obstacle() {
    let sc = load();
    let world = sc.static_world();
    let sp = sc.spawn();
    let mut out = Vec::new();
    world.query(sp.x, sp.y, 1.0, &mut out);
    assert!(out.is_empty(), "spawn overlaps {} obstacles", out.len());
}
