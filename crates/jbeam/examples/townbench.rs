//! townbench <mod.zip> [main]: physics cost driving along a street lined with many-sided buildings.
use phys::vehicle::Input;
use phys::world::{FlatGround, StaticWorld};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let lv = jbeam::load_vehicle_with(std::path::Path::new(&a[1]), a.get(2).map(String::as_str), None).unwrap();
    let mut v = lv.vehicle;
    let g = FlatGround { grip: 1.0 };
    let mut w = StaticWorld::new(25.0);
    // two rows of 40-sided "buildings" 1.6 m from the car's sides, every 25 m, for 2 km
    for k in 0..80 {
        for side in [-1.0, 1.0] {
            let (cx, cy) = (k as f64 * 25.0, side * (1.0 + 1.6 + 10.0));
            w.add_poly((0..40).map(|i| {
                let t = i as f64 / 40.0 * std::f64::consts::TAU;
                (cx + 11.0 * t.cos(), cy + 10.0 * t.sin())
            }).collect());
        }
    }
    v.place(0.0, 0.0, 0.0, 15.0, &g);
    let inp = Input { throttle: 0.3, tcs: true, abs: true, ..Default::default() };
    let t = std::time::Instant::now();
    for _ in 0..3000 {
        v.step(0.001, &inp, &g, &w);
    }
    let s = t.elapsed().as_secs_f64();
    println!("3 s simulated in {s:.2} s ({:.0}% of real time), x = {:.0} m, damage {:.2}", s / 3.0 * 100.0, v.pos.x, v.damage().0);
}
