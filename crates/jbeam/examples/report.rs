use jbeam::{assemble::*, model, zip::Zip};
use std::collections::{BTreeMap, HashMap};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut z = Zip::open(std::path::Path::new(&a[1])).unwrap();
    let main = a.get(2).map(String::as_str).unwrap_or("ravxa20");
    let lib = Library::from_zip(&mut z);
    println!("library: {} parts, {} slot types, {} library warnings", lib.parts.len(), lib.by_type.len(), lib.warnings.len());
    for w in &lib.warnings { println!("  lib: {w}"); }
    let asm = assemble(&lib, main, &HashMap::new());
    println!("assembled {} parts; missing defaults: {}", asm.parts.len(), asm.missing.len());
    for (t, p) in &asm.missing { println!("  missing: slot {t} wants {p}"); }
    let m = model::build(asm);
    println!("nodes {}  beams {}  skipped beams {}", m.nodes.len(), m.beams.len(), m.skipped_beams);
    let mut kinds: BTreeMap<&str, usize> = BTreeMap::new();
    for b in &m.beams { *kinds.entry(&b.kind).or_default() += 1; }
    println!("beam kinds: {kinds:?}");
    let (lo, hi) = m.bounds();
    println!("mass {:.0} kg; bounds x {:.2}..{:.2} (len {:.2}) y {:.2}..{:.2} (w {:.2}) z {:.2}..{:.2} (h {:.2})", m.total_mass(), lo[0], hi[0], hi[0]-lo[0], lo[1], hi[1], hi[1]-lo[1], lo[2], hi[2], hi[2]-lo[2]);
    let mut secs: BTreeMap<String, usize> = BTreeMap::new();
    for p in &m.asm.parts { if let scene::json::Json::Obj(o) = &p.body { for (k, _) in o { *secs.entry(k.clone()).or_default() += 1; } } }
    println!("sections: {secs:?}");
    for s in ["pressureWheels", "hubWheels", "wheels", "hydros", "torsionbars", "triangles", "flexbodies"] {
        println!("  table {s}: {} rows", m.table(s).len());
    }
    for w in m.warnings.iter().chain(m.asm.warnings.iter()).take(40) { println!("  warn: {w}"); }
}
