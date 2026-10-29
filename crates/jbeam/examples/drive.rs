//! Load a BeamNG vehicle mod and test-drive it on flat ground: settle, accelerate, steer, brake.
use jbeam::{assemble::*, model, vehicle::JbeamVehicle, zip::Zip};
use phys::vehicle::Input;
use phys::world::{FlatGround, StaticWorld};
use std::collections::HashMap;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut z = Zip::open(std::path::Path::new(&a[1])).unwrap();
    let main = a.get(2).map(String::as_str).unwrap_or("ravxa20");
    let lib = Library::from_zip(&mut z);
    let m = model::build(assemble(&lib, main, &HashMap::new()));
    let mut v = JbeamVehicle::from_model(&m).unwrap();
    for n in &v.notes {
        println!("note: {n}");
    }
    let g = FlatGround { grip: 1.0 };
    let w = StaticWorld::new(8.0);
    v.place(0.0, 0.0, 0.0, 0.0, &g);
    println!("mass {:.0} kg, nodes {}, beams {}, wheels {}", v.total_mass(), v.body.nodes.len(), v.body.beams.len(), v.wheels.len());
    let t0 = std::time::Instant::now();
    let mut t = 0.0;
    let dt = 0.001;
    let run = |v: &mut JbeamVehicle, secs: f64, inp: Input, label: &str, t: &mut f64| {
        let steps = (secs / dt) as usize;
        for k in 0..steps {
            v.step(dt, &inp, &g, &w);
            *t += dt;
            if (k + 1) % (steps / 4).max(1) == 0 {
                let (dmg, br) = v.damage();
                let fz: Vec<String> = v.wheels.iter().map(|w| format!("{:.0}", w.state.fz)).collect();
                println!(
                    "{label:10} t={:5.2} pos=({:6.2},{:6.2},{:5.2}) speed={:5.1} km/h fwd=({:.2},{:.2},{:.2}) rpm={:4.0} gear={} fz=[{}] dmg={:.3} broken={} diverged={}",
                    *t, v.pos.x, v.pos.y, v.pos.z, v.vel.len() * 3.6, v.forward().x, v.forward().y, v.forward().z, v.drive.rpm(), v.drive.gear, fz.join(","), dmg, br, v.body.diverged
                );
            }
        }
    };
    let idle = Input { throttle: 0.0, brake: 0.0, steer_angle: 0.0, handbrake: 0.0, reverse: false, tcs: true, abs: true };
    run(&mut v, 3.0, idle, "settle", &mut t);
    if std::env::var("DIAG").is_ok() {
        for (i, wh) in v.wheels.iter().enumerate() {
            let c = v.wheel_center(i);
            println!("wheel {} centre z={:.3} r={:.2} axis={:?}", wh.name, c.z, wh.radius, v.wheel_axis(i));
        }
        let mut low: Vec<(f64, &str)> = v.body.nodes.iter().zip(&v.names).filter(|(n, _)| n.p.z < 0.08 && n.hit_ground).map(|(n, s)| (n.p.z, s.as_str())).collect();
        low.sort_by(|a, b| a.0.total_cmp(&b.0));
        println!("nodes touching ground: {:?}", &low[..low.len().min(30)]);
        let mut moved: Vec<(f64, &str)> = v.body.nodes.iter().zip(&v.names).map(|(n, s)| ((n.p - n.rest).len(), s.as_str())).collect();
        let fr = ["fh4r", "rf3", "f1r", "r1r"];
        for nm in fr {
            if let Some(i) = v.names.iter().position(|x| x == nm) {
                println!("{nm}: p={:?} rest={:?}", v.body.nodes[i].p, v.body.nodes[i].rest);
            }
        }
        let dev = v.deviations();
        let mut d: Vec<(f64, &str)> = dev.iter().zip(&v.names).map(|(a, b)| (*a, b.as_str())).collect();
        d.sort_by(|a, b| b.0.total_cmp(&a.0));
        println!("largest deviations: {:?}", &d[..15]);
        moved.sort_by(|a, b| b.0.total_cmp(&a.0));
        println!("most displaced from rest: {:?}", &moved[..12]);
    }
    run(&mut v, 10.0, Input { throttle: 1.0, ..idle }, "throttle", &mut t);
    run(&mut v, 4.0, Input { throttle: 0.3, steer_angle: 0.25, ..idle }, "steer", &mut t);
    run(&mut v, 5.0, Input { brake: 1.0, ..idle }, "brake", &mut t);
    let wall = t0.elapsed().as_secs_f64();
    println!("cpu: {:.2} s for {:.1} s simulated ({:.0}% of one core)", wall, t, wall / t * 100.0);
}
