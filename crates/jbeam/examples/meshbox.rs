//! meshbox <zip> <mesh> [node...]: bounding box of a mesh as loaded (jbeam axes) and positions of nodes
use jbeam::{assemble::*, dae::Dae, model, zip::Zip};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut z = Zip::open(std::path::Path::new(&a[1])).unwrap();
    let names: Vec<String> = z.entries.iter().filter(|e| e.name.to_lowercase().ends_with(".dae")).map(|e| e.name.clone()).collect();
    for n in names {
        let d = Dae::parse(String::from_utf8_lossy(&z.read_name(&n).unwrap()).into_owned()).unwrap();
        if d.node(&a[2]).is_some() {
            let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
            for sm in d.mesh(&a[2]).unwrap() {
                for p in &sm.positions {
                    for i in 0..3 { lo[i] = lo[i].min(p[i]); hi[i] = hi[i].max(p[i]); }
                }
            }
            println!("{n}: {:?} .. {:?}", lo, hi);
            break;
        }
    }
    let lib = Library::from_zip(&mut z);
    let m = model::build(assemble(&lib, "ravxa20", &Default::default()));
    for id in &a[3..] {
        if let Some(&i) = m.node_index.get(id) { println!("node {id}: {:?}", m.nodes[i].pos); }
    }
}
