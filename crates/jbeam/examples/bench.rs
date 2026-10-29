use phys::world::{FlatGround, StaticWorld};
use std::time::Instant;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let lv = jbeam::load_vehicle(std::path::Path::new(&a[1]), a.get(2).map(String::as_str)).unwrap();
    let mut v = lv.vehicle;
    let g = FlatGround { grip: 1.0 };
    let w = StaticWorld::new(8.0);
    v.place(0.0, 0.0, 0.0, 0.0, &g);
    let n = 4000;
    let mut b = v.body.clone();
    let t = Instant::now();
    for _ in 0..n {
        b.clear_forces();
        b.beam_forces(0.00025);
    }
    let tb = t.elapsed().as_secs_f64();
    let t = Instant::now();
    for _ in 0..n {
        b.slide_forces();
    }
    let ts = t.elapsed().as_secs_f64();
    let t = Instant::now();
    for _ in 0..n {
        b.contacts(&g, &w, 0.00025);
    }
    let tc = t.elapsed().as_secs_f64();
    let t = Instant::now();
    for _ in 0..n {
        b.integrate(0.00025);
        b.resolve_world(&w, 0.00025);
    }
    let ti = t.elapsed().as_secs_f64();
    let idle = phys::vehicle::Input { throttle: 0.0, brake: 0.0, steer_angle: 0.0, handbrake: 0.0, reverse: false, tcs: true, abs: true };
    let t = Instant::now();
    for _ in 0..1000 {
        v.step(0.001, &idle, &g, &w);
    }
    let tf = t.elapsed().as_secs_f64();
    println!("per simulated second: beams {:.0} ms, slides {:.1} ms, contacts {:.1} ms, integrate {:.1} ms; full vehicle {:.0} ms ({} beams, {} nodes)", tb * 1000.0, ts * 1000.0, tc * 1000.0, ti * 1000.0, tf * 1000.0, b.beams.len(), b.nodes.len());
}
