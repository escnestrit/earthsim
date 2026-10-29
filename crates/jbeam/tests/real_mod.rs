//! Drives a real BeamNG mod when `JBEAM_MOD=<zip>` (and optionally `JBEAM_MAIN=<part>`) is set; skipped
//! otherwise (mods are not part of the repository).
use jbeam::{assemble::*, model, vehicle::JbeamVehicle, zip::Zip};
use phys::vehicle::Input;
use phys::world::{FlatGround, StaticWorld};
use std::collections::HashMap;

#[test]
fn real_mod_stands_accelerates_turns_and_brakes() {
    let Ok(path) = std::env::var("JBEAM_MOD") else {
        eprintln!("JBEAM_MOD not set: skipped");
        return;
    };
    let main = std::env::var("JBEAM_MAIN").unwrap_or_else(|_| "ravxa20".into());
    let mut z = Zip::open(std::path::Path::new(&path)).unwrap();
    let lib = Library::from_zip(&mut z);
    let m = model::build(assemble(&lib, &main, &HashMap::new()));
    let mut v = JbeamVehicle::from_model(&m).unwrap();
    let g = FlatGround { grip: 1.0 };
    let w = StaticWorld::new(8.0);
    v.place(0.0, 0.0, 0.0, 0.0, &g);
    let idle = Input { throttle: 0.0, brake: 0.0, steer_angle: 0.0, handbrake: 0.0, reverse: false, tcs: true, abs: true };
    let run = |v: &mut JbeamVehicle, secs: f64, inp: &Input| {
        for _ in 0..(secs * 1000.0) as usize {
            v.step(0.001, inp, &g, &w);
        }
    };
    run(&mut v, 3.0, &idle);
    let weight = v.total_mass() * 9.81;
    // average over a second: the body keeps some small vibration
    let mut fz = 0.0;
    let mut min_fz = f64::MAX;
    let mut trace = Vec::new();
    for k in 0..1000 {
        v.step(0.001, &idle, &g, &w);
        let s: f64 = v.wheels.iter().map(|w| w.state.fz).sum();
        fz += s / 1000.0;
        min_fz = min_fz.min(v.wheels.iter().map(|w| w.state.fz).fold(f64::MAX, f64::min));
        if k % 100 == 0 {
            trace.push(format!("{s:.0}"));
        }
    }
    eprintln!("sum fz trace {:?}, min wheel {min_fz:.0}", trace);
    eprintln!("mass {:.0} kg, tyres carry {:.0} N of {:.0}", v.total_mass(), fz, weight);
    assert!((fz - weight).abs() < 0.1 * weight);
    assert!(min_fz > 0.1 * weight / 4.0, "a wheel left the ground while standing");
    // 0-100 km/h
    let thr = Input { throttle: 1.0, ..idle };
    let mut t = 0.0;
    while v.vel.dot(v.forward()) < 27.78 && t < 30.0 {
        run(&mut v, 0.1, &thr);
        t += 0.1;
    }
    eprintln!("0-100 km/h in {t:.1} s");
    assert!(t > 5.0 && t < 20.0);
    // left turn at moderate speed
    let yaw0 = v.forward().y.atan2(v.forward().x);
    run(&mut v, 3.0, &Input { throttle: 0.2, steer_angle: 0.08, ..idle });
    let dyaw = (v.forward().y.atan2(v.forward().x) - yaw0).to_degrees();
    eprintln!("yaw change in 3 s of left steer: {dyaw:.0} deg");
    assert!(dyaw > 20.0);
    // brake to a stop
    let v0 = v.vel.len();
    let p0 = v.pos;
    run(&mut v, 8.0, &Input { brake: 1.0, ..idle });
    let dist = (v.pos - p0).len();
    eprintln!("braking from {:.0} km/h: {dist:.0} m", v0 * 3.6);
    assert!(v.vel.len() < 0.5 && !v.body.diverged);
    let (_, broken) = v.damage();
    eprintln!("broken beams {broken}");
}

#[test]
fn real_mod_frontal_crash_at_50_kmh() {
    let Ok(path) = std::env::var("JBEAM_MOD") else {
        return;
    };
    let main = std::env::var("JBEAM_MAIN").ok();
    let lv = jbeam::load_vehicle(std::path::Path::new(&path), main.as_deref()).unwrap();
    let mut v = lv.vehicle;
    let g = FlatGround { grip: 1.0 };
    let mut wall = StaticWorld::new(10.0);
    let front = v.body.nodes.iter().map(|n| n.rest.x).fold(f64::MIN, f64::max);
    let cx = {
        let (mut s, mut m) = (0.0, 0.0);
        for n in &v.body.nodes {
            s += n.rest.x * n.m;
            m += n.m;
        }
        s / m
    };
    let wall_x = (front - cx) + 1.0; // 1 m ahead of the bumper
    wall.add_poly(vec![(wall_x, -10.0), (wall_x + 2.0, -10.0), (wall_x + 2.0, 10.0), (wall_x, 10.0)]);
    v.place(0.0, 0.0, 0.0, 50.0 / 3.6, &g);
    let none = Input { tcs: true, abs: true, ..Default::default() };
    let mut max_x: f64 = f64::MIN;
    for _ in 0..1500 {
        v.step(0.001, &none, &g, &wall);
        max_x = max_x.max(v.body.nodes.iter().map(|n| n.p.x).fold(f64::MIN, f64::max));
    }
    let (dmg, broken) = v.damage();
    eprintln!("crash: max deformation {:.2} m, broken beams {broken}, plastic work {:.0} kJ, speed after {:.1} km/h, deepest node x {:.2} (wall at {:.2})",
        dmg, v.body.plastic_work / 1000.0, v.vel.len() * 3.6, max_x, wall_x);
    assert!(!v.body.diverged);
    assert!(dmg > 0.15, "a 50 km/h crash must deform the front");
    // stopped by the wall; some elastic rebound is normal (real cars come back at a few km/h)
    assert!(v.vel.x < 0.5 && v.vel.x > -20.0 / 3.6, "after the crash vx = {:.1} m/s", v.vel.x);
    assert!(max_x < wall_x + 0.1, "nodes went through the wall");
}
