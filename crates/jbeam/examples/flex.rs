use jbeam::{assemble::*, dae::Dae, model, zip::Zip};
use std::collections::HashMap;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut z = Zip::open(std::path::Path::new(&a[1])).unwrap();
    let lib = Library::from_zip(&mut z);
    let m = model::build(assemble(&lib, "ravxa20", &HashMap::new()));
    let t0 = std::time::Instant::now();
    let names: Vec<String> = z.entries.iter().filter(|e| e.name.to_lowercase().ends_with(".dae")).map(|e| e.name.clone()).collect();
    let mut daes: Vec<Dae> = Vec::new();
    for n in &names {
        let bytes = z.read_name(n).unwrap();
        daes.push(Dae::parse(String::from_utf8_lossy(&bytes).into_owned()).unwrap());
    }
    println!("{} dae files, {} scene nodes, read+index {:.2} s", daes.len(), daes.iter().map(|d| d.nodes.len()).sum::<usize>(), t0.elapsed().as_secs_f64());
    let t1 = std::time::Instant::now();
    let (mut found, mut missing, mut tris, mut verts) = (0, Vec::new(), 0usize, 0usize);
    let mut mats: HashMap<String, usize> = HashMap::new();
    for (_, r) in m.table("flexbodies") {
        let Some(mesh) = r.str("mesh") else { continue };
        match daes.iter().find(|d| d.node(mesh).is_some()) {
            Some(d) => {
                found += 1;
                for sm in d.mesh(mesh).unwrap() {
                    tris += sm.indices.len() / 3;
                    verts += sm.positions.len();
                    *mats.entry(sm.material).or_default() += 1;
                }
            }
            None => missing.push(mesh.to_string()),
        }
    }
    println!("flexbodies: {found} found, {} missing {:?}", missing.len(), &missing[..missing.len().min(15)]);
    println!("triangles {tris}, vertices {verts}, parse {:.2} s", t1.elapsed().as_secs_f64());
    let veh = jbeam::vehicle::JbeamVehicle::from_model(&m).unwrap();
    let t2 = std::time::Instant::now();
    let files: Vec<jbeam::flex::MeshFile> = daes.into_iter().map(jbeam::flex::MeshFile::Dae).collect();
    let (parts, notes) = jbeam::flex::build(&m, &veh, &files);
    println!("bound {} parts ({} rigid wheels) in {:.2} s; notes {:?}", parts.len(), parts.iter().filter(|p| p.wheel.is_some()).count(), t2.elapsed().as_secs_f64(), notes);
    // skinning cost at rest
    let rest: Vec<phys::math::V3> = veh.body.nodes.iter().map(|n| n.rest).collect();
    let t3 = std::time::Instant::now();
    let (mut p, mut n) = (Vec::new(), Vec::new());
    let mut nv = 0;
    let mut err: f32 = 0.0;
    for part in &parts {
        for (sm, b) in part.submeshes.iter().zip(&part.binds) {
            jbeam::flex::deform(b, &rest, &mut p, &mut n);
            nv += p.len();
            for (a, bb) in p.iter().zip(&sm.positions) {
                err = err.max(((a[0] - bb[0]).powi(2) + (a[1] - bb[1]).powi(2) + (a[2] - bb[2]).powi(2)).sqrt());
            }
        }
    }
    println!("skinned {nv} vertices in {:.1} ms, max rest error {err:.6} m", t3.elapsed().as_secs_f64() * 1000.0);
    let mut mv: Vec<_> = mats.into_iter().collect();
    mv.sort();
    println!("materials ({}): {:?}", mv.len(), mv.iter().map(|x| x.0.as_str()).collect::<Vec<_>>());
}
