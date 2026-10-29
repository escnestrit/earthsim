//! fallbackcmp <mod_subset.zip> <full.zip> <main>: assembling the subset with the full zip as fallback must give the full model.
use jbeam::{assemble::*, model, zip::Zip};
use std::collections::HashMap;
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut sub = Zip::open(std::path::Path::new(&a[1])).unwrap();
    let mut full = Zip::open(std::path::Path::new(&a[2])).unwrap();
    let mut l1 = Library::from_zip(&mut sub);
    let added = l1.add_zip_fallback(&mut full);
    let l2 = Library::from_zip(&mut full);
    let m1 = model::build(assemble(&l1, &a[3], &HashMap::new()));
    let m2 = model::build(assemble(&l2, &a[3], &HashMap::new()));
    println!("fallback added {added} parts");
    println!("subset+fallback: {} parts, {} nodes, {} beams, missing {}", m1.asm.parts.len(), m1.nodes.len(), m1.beams.len(), m1.asm.missing.len());
    println!("full:            {} parts, {} nodes, {} beams, missing {}", m2.asm.parts.len(), m2.nodes.len(), m2.beams.len(), m2.asm.missing.len());
    let mut p1: Vec<_> = m1.asm.parts.iter().map(|p| p.name.clone()).collect();
    let mut p2: Vec<_> = m2.asm.parts.iter().map(|p| p.name.clone()).collect();
    p1.sort();
    p2.sort();
    println!("same parts: {}", p1 == p2);
}
