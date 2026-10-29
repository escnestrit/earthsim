//! crashviz <mod.zip> [main] [kmh]: frontal crash into a wall; where did the deformation go (nodes and meshes)?
use phys::vehicle::Input;
use phys::world::{FlatGround, StaticWorld};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let kmh: f64 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(50.0);
    let lv = jbeam::load_vehicle_with(std::path::Path::new(&a[1]), a.get(2).map(String::as_str).filter(|s| *s != "-"), None).unwrap();
    let mut v = lv.vehicle;
    let vis = lv.visual.expect("meshes");
    let g = FlatGround { grip: 1.0 };
    let mut wall = StaticWorld::new(10.0);
    let rc = v.rest_com();
    let front = v.body.nodes.iter().map(|n| n.rest.x).fold(f64::MIN, f64::max) - rc.x;
    let wall_x = front + 0.3;
    wall.add_poly(vec![(wall_x, -10.0), (wall_x + 2.0, -10.0), (wall_x + 2.0, 10.0), (wall_x, 10.0)]);
    let none = Input { tcs: true, abs: true, ..Default::default() };
    // standing still first: nothing may break or trigger from settling
    v.place(-50.0, 0.0, 0.0, 0.0, &g);
    for _ in 0..2000 {
        v.step(0.001, &none, &g, &wall);
    }
    println!("settled 2 s: broken groups {}, triggered {:?}, broken beams {}", v.groups.broken_group_count(), v.groups.triggered_names(), v.damage().1);
    v.place(0.0, 0.0, 0.0, kmh / 3.6, &g);
    let t = std::time::Instant::now();
    for _ in 0..1500 {
        v.step(0.001, &none, &g, &wall);
    }
    println!("simulated 1.5 s in {:.2} s", t.elapsed().as_secs_f64());
    let (dmg, broken) = v.damage();
    let dev = v.deviations();
    let hist = |lim: f64| dev.iter().zip(&v.weak).filter(|(d, w)| !**w && **d > lim).count();
    println!("max {:.2} m, broken {broken}, plastic {:.0} kJ; nodes moved >2cm {} >5cm {} >15cm {} >30cm {} (of {})", dmg, v.body.plastic_work / 1000.0, hist(0.02), hist(0.05), hist(0.15), hist(0.3), dev.len());
    println!("after crash: broken groups {:?}, triggered {:?}", v.groups.broken_group_names(), v.groups.triggered_names());
    let yielded = v.body.beams.iter().filter(|b| (b.rest - b.rest0).abs() > 0.01).count();
    println!("beams with permanent set >1cm: {yielded} of {}", v.body.beams.len());
    // body-frame node positions now (undo the car's motion) -> how far each mesh vertex moved
    let r = v.body_rotation();
    let inv = r.conj();
    let now: Vec<phys::math::V3> = v.body.nodes.iter().map(|n| inv.rotate(n.p - v.pos) + rc).collect();
    let rest: Vec<phys::math::V3> = v.body.nodes.iter().map(|n| n.rest).collect();
    let mut rows = Vec::new();
    let (mut p0, mut n0, mut p1, mut n1) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for part in vis.parts.iter().filter(|p| p.wheel.is_none()) {
        let mut mx: f32 = 0.0;
        for b in &part.binds {
            jbeam::flex::deform(b, &rest, &mut p0, &mut n0);
            jbeam::flex::deform(b, &now, &mut p1, &mut n1);
            for (x, y) in p0.iter().zip(&p1) {
                mx = mx.max(((x[0] - y[0]).powi(2) + (x[1] - y[1]).powi(2) + (x[2] - y[2]).powi(2)).sqrt());
            }
        }
        rows.push((mx, part.mesh.clone()));
    }
    rows.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (m, n) in rows.iter().take(14) {
        println!("  mesh {n}: moved up to {:.2} m", m);
    }
    // where are the most-moved nodes (rest position, body frame)
    let mut order: Vec<usize> = (0..dev.len()).filter(|&i| !v.weak[i]).collect();
    order.sort_by(|&i, &j| dev[j].total_cmp(&dev[i]));
    for &i in order.iter().take(8) {
        let n = &v.body.nodes[i];
        println!("  node {} rest ({:.2},{:.2},{:.2}) moved {:.2}", v.names[i], n.rest.x, n.rest.y, n.rest.z, dev[i]);
    }
}
