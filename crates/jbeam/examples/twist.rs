use jbeam::{assemble::*, model, vehicle::JbeamVehicle, zip::Zip};
use phys::vehicle::Input;
use phys::world::{FlatGround, StaticWorld};
use std::collections::HashMap;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut z = Zip::open(std::path::Path::new(&a[1])).unwrap();
    let lib = Library::from_zip(&mut z);
    let m = model::build(assemble(&lib, "ravxa20", &HashMap::new()));
    let mut v = JbeamVehicle::from_model(&m).unwrap();
    let g = FlatGround { grip: 1.0 };
    let w = StaticWorld::new(8.0);
    v.place(0.0, 0.0, 0.0, 0.0, &g);
    let idle = Input { throttle: 0.0, brake: 0.0, steer_angle: 0.0, handbrake: 0.0, reverse: false, tcs: true, abs: true };
    let watch: Vec<usize> = a[2..].iter().map(|n| m.node_index[n.as_str()]).collect();
    if std::env::var("HEAVE").is_ok() {
        for k in 0..6000 {
            v.step(0.001, &idle, &g, &w);
            if k >= 4000 && k % 10 == 0 {
                let fz: Vec<String> = v.wheels.iter().map(|w| format!("{:5.0}", w.state.fz)).collect();
                let c: Vec<String> = (0..4).map(|i| format!("{:.4}", v.wheel_center(i).z)).collect();
                println!("t={:.2} z={:.4} pitch={:.4} fz=[{}] wz=[{}]", k as f64 / 1000.0, v.pos.z, v.forward().z, fz.join(","), c.join(","));
            }
        }
        return;
    }
    for k in 0..=400 {
        if k % 20 == 0 || k < 5 {
            let fr = v.wheels.iter().position(|w| w.name == "FR").unwrap();
            let ax = v.wheel_axis(fr);
            let pts: Vec<String> = watch.iter().map(|&i| { let p = v.body.nodes[i].p; format!("{}=({:.3},{:.3},{:.3})", m.nodes[i].name, p.x, p.y, p.z) }).collect();
            let broken: Vec<String> = v.body.beams.iter().enumerate().filter(|(_, b)| b.broken).take(5).map(|(i, b)| format!("{}-{}", m.nodes[b.a].name, if i < m.beams.len() { m.nodes[b.b].name.clone() } else { "hydro".into() })).collect();
            println!("t={:.3} FR axis=({:.2},{:.2},{:.2}) {} broken={:?}", k as f64 * 0.001, ax.x, ax.y, ax.z, pts.join(" "), broken);
        }
        v.step(0.001, &idle, &g, &w);
    }
}
