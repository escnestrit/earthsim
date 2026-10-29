fn main() {
    let a: Vec<String> = std::env::args().collect();
    let t = std::time::Instant::now();
    let lv = jbeam::load_vehicle(std::path::Path::new(&a[1]), None).unwrap();
    println!("loaded in {:.1} s", t.elapsed().as_secs_f64());
    for n in &lv.notes {
        println!("note: {n}");
    }
    let v = lv.visual.as_ref().unwrap();
    println!("textures {} ({} MB)", v.textures.len(), v.textures.values().map(|b| b.len()).sum::<usize>() / 1_000_000);
    for p in v.parts.iter().filter(|p| p.wheel.is_some()) {
        let (mut c, mut n) = ([0.0f64; 3], 0.0);
        for s in &p.submeshes {
            for q in &s.positions {
                for k in 0..3 {
                    c[k] += q[k] as f64;
                }
                n += 1.0;
            }
        }
        let wi = p.wheel.unwrap();
        let w = &lv.vehicle.wheels[wi];
        let wc = (lv.vehicle.body.nodes[w.n1].rest + lv.vehicle.body.nodes[w.n2].rest) * 0.5;
        println!("wheel mesh {} -> {}: centroid ({:.2},{:.2},{:.2}) wheel centre ({:.2},{:.2},{:.2})", p.mesh, w.name, c[0] / n, c[1] / n, c[2] / n, wc.x, wc.y, wc.z);
    }
    let mut used: Vec<(&str, bool)> = v.parts.iter().flat_map(|p| p.submeshes.iter().map(|s| (s.material.as_str(), v.materials.get(&s.material).is_some_and(|m| m.base_map.is_some())))).collect();
    used.sort();
    used.dedup();
    println!("materials with texture: {:?}", used.iter().filter(|x| x.1).map(|x| x.0).collect::<Vec<_>>());
    println!("without: {:?}", used.iter().filter(|x| !x.1).map(|x| x.0).collect::<Vec<_>>());
}
