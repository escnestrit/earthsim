use jbeam::{assemble::*, model, zip::Zip};
use std::collections::{BTreeMap, HashMap};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut z = Zip::open(std::path::Path::new(&a[1])).unwrap();
    let lib = Library::from_zip(&mut z);
    let asm = assemble(&lib, "ravxa20", &HashMap::new());
    let m = model::build(asm);
    match a[2].as_str() {
        "skips" => {
            let mut c: BTreeMap<String, usize> = BTreeMap::new();
            for (pi, p) in m.asm.parts.iter().enumerate() {
                let _ = pi;
                let Some(sec) = p.body.get("beams") else { continue };
                for r in jbeam::table::rows(sec) {
                    let (Some(x), Some(y)) = (r.str("id1"), r.str("id2")) else { continue };
                    for n in [x, y] { if !m.node_index.contains_key(n) && !r.flag("optional") { *c.entry(format!("{} <- {}", n, p.name)).or_default() += 1; } }
                }
            }
            let mut by_part: BTreeMap<String, usize> = BTreeMap::new();
            for (k, v) in &c { *by_part.entry(k.split(" <- ").nth(1).unwrap().to_string()).or_default() += v; }
            println!("{by_part:?}");
            for (k, v) in c.iter().take(25) { println!("{k} x{v}"); }
        }
        s => {
            for (pi, r) in m.table(s) {
                println!("[{}] {:?}", m.asm.parts[pi].name, r.vals.iter().map(|(k, v)| format!("{k}={}", v.dump())).collect::<Vec<_>>().join(" "));
            }
        }
    }
}
