use jbeam::shape::{self, Mp, Val};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let f = std::fs::read(&a[1]).unwrap();
    let t = std::time::Instant::now();
    let (_h, body) = shape::split(&f).unwrap();
    println!("body {} bytes, decompressed in {:.2}s", body.len(), t.elapsed().as_secs_f64());
    let mut m = Mp::new(&body);
    let mut head = Vec::new();
    for _ in 0..6 {
        head.push(format!("{:?}", m.val().unwrap()));
    }
    println!("shape info: {}", head.join(" "));
    for n in ["nodes", "objects", "ssFirstNode", "ssFirstObj", "ssNumNodes", "ssNumObj", "defRot", "defTrans", "nodeRot", "nodeTrans", "uScale", "aScale", "arbScaleF", "arbScaleR", "groundT", "groundR", "objStates", "triggers", "details"] {
        let (c, es, _) = m.vector().unwrap();
        println!("  {n}: {c} x {es}");
    }
    let nn = m.int().unwrap();
    let mut names = Vec::new();
    for _ in 0..nn {
        if let Val::Str(s) = m.val().unwrap() {
            names.push(s);
        }
    }
    println!("names {nn}: {:?}", &names[..names.len().min(5)]);
    let mc = m.int().unwrap();
    println!("meshes {mc}");
    for i in 0..mc.min(3) {
        let ty = m.int().unwrap();
        println!(" mesh {i} type {ty}");
        if ty == 4 { continue; }
        let hdr: Vec<String> = (0..6).map(|_| format!("{:?}", m.val().unwrap())).collect();
        println!("  hdr {}", hdr.join(" "));
        for n in ["verts", "tverts", "tverts2", "colors", "norms", "encNorms", "prims", "indices", "tangents"] {
            let (c, es, b) = m.vector().unwrap();
            println!("   {n}: {c} x {es}  first {:?}", &b[..b.len().min(16)]);
        }
        println!("  vpf {:?} flags {:?}", m.val().unwrap(), m.val().unwrap());
    }
    let t = std::time::Instant::now();
    match shape::parse(&f) {
        Ok(s) => {
            let mut k: Vec<_> = s.objects.keys().cloned().collect();
            k.sort();
            winding(&s);
            println!("parsed {} objects, {} materials in {:.2}s; e.g. {:?}; mats {:?}", s.objects.len(), s.materials.len(), t.elapsed().as_secs_f64(), &k[..k.len().min(6)], &s.materials[..s.materials.len().min(8)]);
        }
        Err(e) => println!("parse error: {e}"),
    }
}

#[allow(dead_code)]
fn winding(s: &jbeam::shape::Shape) {
    let (mut agree, mut total) = (0usize, 0usize);
    for subs in s.objects.values() {
        for sm in subs {
            for t in sm.indices.chunks_exact(3) {
                let [a, b, c] = [t[0], t[1], t[2]].map(|i| sm.positions[i as usize]);
                let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                let n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
                let vn = sm.normals[t[0] as usize];
                let d = n[0] * vn[0] + n[1] * vn[1] + n[2] * vn[2];
                if d.abs() > 1e-12 {
                    total += 1;
                    if d > 0.0 {
                        agree += 1;
                    }
                }
            }
        }
    }
    println!("winding: {agree} of {total} triangles agree with their vertex normals");
}
