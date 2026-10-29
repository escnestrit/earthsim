fn main() {
    let a: Vec<String> = std::env::args().collect();
    let lv = jbeam::load_vehicle_with(std::path::Path::new(&a[1]), a.get(2).map(String::as_str), None).unwrap();
    let v = &lv.vehicle;
    let nodes: Vec<[f64; 3]> = v.body.nodes.iter().map(|n| [n.rest.x, n.rest.y, n.rest.z]).collect();
    println!("eye {:?}", v.eye.rest);
    for p in &lv.visual.as_ref().unwrap().props {
        for sm in &p.submeshes { println!("  sub {} verts {} tris {}", sm.material, sm.positions.len(), sm.indices.len() / 3); }
        let (mut rl, mut rh) = ([f32::MAX; 3], [f32::MIN; 3]);
        for sm in &p.submeshes { for q in &sm.positions { for i in 0..3 { rl[i] = rl[i].min(q[i]); rh[i] = rh[i].max(q[i]); } } }
        println!("  raw mesh bbox {:?}..{:?} base {:?} trans {:?} axis {:?}", rl, rh, p.base_rot_deg, p.base_trans, p.axis);
        let (o, m) = p.pose(&nodes, 0.0);
        let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
        for sm in &p.submeshes {
            for q in &sm.positions {
                let q = [q[0] as f64, q[1] as f64, q[2] as f64];
                for i in 0..3 {
                    let w = o[i] + m[i][0] * q[0] + m[i][1] * q[1] + m[i][2] * q[2];
                    lo[i] = lo[i].min(w);
                    hi[i] = hi[i].max(w);
                }
            }
        }
        println!("{} refs {:?} origin {:?} bbox {:?}..{:?} axis(mesh z) {:?}", p.mesh, p.refs.map(|i| nodes[i]), o, lo, hi, [m[0][2], m[1][2], m[2][2]]);
    }
}
// raw mesh bbox printed separately
