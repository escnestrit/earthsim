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
        "nodes" => {
            for n in &m.nodes {
                if a.len() < 4 || n.name.starts_with(a[3].as_str()) {
                    println!("{:8} {:?} m={} part={} grp={:?}", n.name, n.pos, n.mass, m.asm.parts[n.part].name, n.group);
                }
            }
        }
        "bounded" => {
            for b in m.beams.iter().filter(|b| b.kind == a[3]).take(12) {
                let keys = ["beamSpring","beamDamp","beamDampRebound","beamLongBound","beamShortBound","beamLimitSpring","beamLimitDamp","beamLimitDampRebound","beamPrecompression","beamDeform","beamStrength","precompressionRange","beamDampFast","beamDampVelocitySplit"];
                let v: Vec<String> = keys.iter().filter_map(|k| b.row.get(k).map(|x| format!("{k}={}", x.dump()))).collect();
                println!("{}-{} [{}] {}", m.nodes[b.a].name, m.nodes[b.b].name, m.asm.parts[b.part].name, v.join(" "));
            }
        }
        "soft" => {
            for b in m.beams.iter().filter(|b| b.spring() < a[3].parse::<f64>().unwrap() && m.asm.parts[b.part].name.contains(a.get(4).map(String::as_str).unwrap_or(""))) {
                let v: Vec<String> = b.row.vals.iter().filter(|(k, _)| k.starts_with("beam") || k.starts_with("precomp")).map(|(k, x)| format!("{k}={}", x.dump())).collect();
                println!("{}-{} [{}] {} {}", m.nodes[b.a].name, m.nodes[b.b].name, m.asm.parts[b.part].name, b.kind, v.join(" "));
            }
        }
        "touch" => {
            let i = m.node_index[a[3].as_str()];
            for b in m.beams.iter().filter(|b| b.a == i || b.b == i) {
                let o = if b.a == i { b.b } else { b.a };
                let v: Vec<String> = ["beamSpring","beamDamp","beamDeform","beamStrength","beamPrecompression","precompressionRange","beamLongBound","beamShortBound","beamLimitSpring"].iter().filter_map(|k| b.row.get(k).map(|x| format!("{k}={}", x.dump()))).collect();
                println!("-> {:8} [{}] {} {}", m.nodes[o].name, m.asm.parts[b.part].name, b.kind, v.join(" "));
            }
        }
        "section" => {
            for p in &m.asm.parts {
                if let Some(v) = p.body.get(a[3].as_str()) {
                    println!("[{}] {}", p.name, &v.dump()[..v.dump().len().min(3000)]);
                }
            }
        }
        s => {
            for (pi, r) in m.table(s) {
                println!("[{}] {:?}", m.asm.parts[pi].name, r.vals.iter().map(|(k, v)| format!("{k}={}", v.dump())).collect::<Vec<_>>().join(" "));
            }
        }
    }
}
