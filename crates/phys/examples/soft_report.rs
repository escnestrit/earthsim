use phys::*;
use phys::vehicle::{Input, G};
fn main() {
    let g = FlatGround { grip: 1.0 };
    let w = StaticWorld::new(10.0);
    let mut c = SoftVehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
    println!("mass {:.0} nodes {} beams {}", c.body.total_mass(), c.body.nodes.len(), c.body.beams.len());
    let none = Input { tcs: true, abs: true, ..Default::default() };
    let t0 = std::time::Instant::now();
    for i in 0..4000 {
        c.step(0.001, &none, &g, &w);
        if i % 500 == 499 {
            let (dmg, br) = c.damage();
            println!("t={:.1} cg z {:.3} vel {:.4} fz {:?} dmg {:.4} broken {} diverged {}", c.time, c.pos.z, c.vel.len(),
                c.wheels.iter().map(|w| w.fz as i32).collect::<Vec<_>>(), dmg, br, c.body.diverged);
        }
    }
    println!("4 s sim took {:?}", t0.elapsed());
    let load: f64 = c.wheels.iter().map(|w| w.fz).sum();
    println!("total load {:.0} expected {:.0}", load, 1450.0 * G);
    let up = c.q.rotate(V3::Z);
    println!("up.z {:.4}", up.z);

    // --- acceleration, rigid vs soft ---
    let run0_100 = |soft: bool| -> f64 {
        let inp = Input { throttle: 1.0, tcs: true, abs: true, ..Default::default() };
        if soft {
            let mut c = SoftVehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
            let mut t = 0.0;
            while c.vel.dot(c.forward()) < 27.78 && t < 30.0 { c.step(0.001, &inp, &g, &w); t += 0.001; }
            t
        } else {
            let mut c = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
            let mut t = 0.0;
            while c.telemetry().forward_speed < 27.78 && t < 30.0 { c.step(0.001, &inp, &g, &w); t += 0.001; }
            t
        }
    };
    println!("0-100: rigid {:.2}s  soft {:.2}s", run0_100(false), run0_100(true));

    // --- braking from 100 km/h ---
    let brake = |soft: bool| -> (f64, f64) {
        let inp = Input { brake: 1.0, tcs: true, abs: true, ..Default::default() };
        if soft {
            let mut c = SoftVehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
            c.place(0.0, 0.0, 0.0, 27.78, &g);
            for _ in 0..1500 { c.step(0.001, &Input::default(), &g, &w); }
            c.place(0.0, 0.0, 0.0, 27.78, &g);
            let mut maxp: f64 = 0.0;
            for _ in 0..8000 { c.step(0.001, &inp, &g, &w); maxp = maxp.max(c.forward().z.abs()); if c.vel.len() < 0.1 { break; } }
            (c.pos.x, maxp.asin().to_degrees())
        } else {
            let mut c = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
            c.place(0.0, 0.0, 0.0, 27.78, &g);
            let mut maxp: f64 = 0.0;
            for _ in 0..8000 { c.step(0.001, &inp, &g, &w); maxp = maxp.max(c.forward().z.abs()); if c.vel.len() < 0.1 { break; } }
            (c.pos.x, maxp.asin().to_degrees())
        }
    };
    println!("brake 100->0: rigid (dist, max pitch) {:?} soft {:?}", brake(false), brake(true));

    // --- steady cornering: 40 km/h constant circle-ish with steer 0.12 ---
    let mut c = SoftVehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
    c.place(0.0, 0.0, 0.0, 15.0, &g);
    let mut r = Vehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
    r.place(0.0, 0.0, 0.0, 15.0, &g);
    let inp = Input { throttle: 0.15, steer_angle: 0.08, tcs: true, abs: true, ..Default::default() };
    for _ in 0..4000 { c.step(0.001, &inp, &g, &w); r.step(0.001, &inp, &g, &w); }
    let roll = |q: Quat| q.rotate(V3::Y).z.asin().to_degrees();
    println!("cornering: soft lat_g {:.2} roll {:.1} deg speed {:.1} | rigid lat_g {:.2} roll {:.1} speed {:.1}",
        c.accel.y / G, roll(c.q), c.vel.len(), r.accel.y / G, roll(r.q), r.vel.len());

    // --- 50 km/h into a wall ---
    let mut wall = StaticWorld::new(10.0);
    wall.add_poly(vec![(30.0, -10.0), (32.0, -10.0), (32.0, 10.0), (30.0, 10.0)]);
    let mut c = SoftVehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
    c.place(0.0, 0.0, 0.0, 13.9, &g);
    let none = Input { tcs: true, abs: true, ..Default::default() };
    let e0 = 0.5 * 1450.0 * 13.9 * 13.9;
    let mut peak_g: f64 = 0.0;
    let mut last_v = c.vel.x;
    let mut front_max: f64 = 0.0;
    for i in 0..3000 {
        c.step(0.001, &none, &g, &wall);
        if i % 5 == 4 { peak_g = peak_g.max(((c.vel.x - last_v) / 0.005).abs() / G); last_v = c.vel.x; }
        for &k in &c.cage { front_max = front_max.max(c.body.nodes[k].p.x); }
        if i % 250 == 249 {
            let (d, br) = c.damage();
            println!("crash t={:.2} x={:.2} v={:.2} dmg {:.2} broken {} plastic {:.0} J  KE {:.0} J front_max {:.2}", c.time, c.pos.x, c.vel.x, d, br, c.body.plastic_work, c.body.kinetic_energy(), front_max);
        }
    }
    println!("crash: e0 {:.0} J, peak decel {:.1} g, diverged {}", e0, peak_g, c.body.diverged);
}

#[allow(dead_code)]
fn unused() {}
