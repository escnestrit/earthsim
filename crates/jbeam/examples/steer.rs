use jbeam::{assemble::*, model, vehicle::JbeamVehicle, zip::Zip};
use phys::vehicle::Input;
use phys::world::{FlatGround, StaticWorld};
use std::collections::HashMap;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut z = Zip::open(std::path::Path::new(&a[1])).unwrap();
    let lib = Library::from_zip(&mut z);
    let m = model::build(assemble(&lib, "ravxa20", &HashMap::new()));
    // connectivity
    let n = m.nodes.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(p: &mut Vec<usize>, x: usize) -> usize { let mut x = x; while p[x] != x { p[x] = p[p[x]]; x = p[x]; } x }
    for b in &m.beams { let (ra, rb) = (find(&mut parent, b.a), find(&mut parent, b.b)); parent[ra] = rb; }
    let mut comp: HashMap<usize, Vec<usize>> = HashMap::new();
    for i in 0..n { let r = find(&mut parent, i); comp.entry(r).or_default().push(i); }
    let mut sizes: Vec<(usize, String)> = comp.values().map(|v| (v.len(), v.iter().take(6).map(|&i| m.nodes[i].name.clone()).collect::<Vec<_>>().join(","))).collect();
    sizes.sort_by(|a, b| b.0.cmp(&a.0));
    println!("components: {:?}", &sizes[..sizes.len().min(10)]);
    let mut v = JbeamVehicle::from_model(&m).unwrap();
    let g = FlatGround { grip: 1.0 };
    let w = StaticWorld::new(8.0);
    v.place(0.0, 0.0, 0.0, 0.0, &g);
    let idle = Input { throttle: 0.0, brake: 0.0, steer_angle: 0.0, handbrake: 0.0, reverse: false, tcs: true, abs: true };
    for _ in 0..2000 { v.step(0.001, &idle, &g, &w); }
    for s in [0.0, 0.25, -0.25, 0.55] {
        let inp = Input { steer_angle: s, throttle: 0.15, ..idle };
        for _ in 0..1500 { v.step(0.001, &inp, &g, &w); }
        let st: Vec<String> = v.wheels.iter().map(|w| format!("{}={:.1}°", w.name, w.state.steer.to_degrees())).collect();
        let hy: Vec<String> = v.hydros.iter().map(|h| { let b = &v.body.beams[h.beam]; let len = (v.body.nodes[b.b].p - v.body.nodes[b.a].p).len(); format!("rest {:.3} len {:.3} base {:.3} broken {}", b.rest, len, h.base, b.broken) }).collect();
        println!("steer_in={s:+.2} wheels: {} hydros: {:?} speed {:.1} yaw {:.1}°", st.join(" "), hy, v.vel.len() * 3.6, v.forward().y.atan2(v.forward().x).to_degrees());
    }
    let broken: Vec<String> = v.body.beams.iter().filter(|b| b.broken).map(|b| format!("{}-{}", m.nodes[b.a].name, m.nodes[b.b].name)).collect();
    println!("broken: {broken:?}");
}
