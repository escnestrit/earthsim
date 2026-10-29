fn main() {
    let a: Vec<String> = std::env::args().collect();
    let mut z = jbeam::zip::Zip::open(std::path::Path::new(&a[1])).unwrap();
    let es: Vec<_> = z.entries.iter().filter(|e| e.name.to_lowercase().ends_with(".cdae")).cloned().collect();
    let t = std::time::Instant::now();
    let mut total = 0;
    for e in &es {
        let head = z.read_prefix(e, 8).unwrap();
        let hs = jbeam::shape::header_len(&head).unwrap();
        let names = jbeam::shape::object_names(&z.read_prefix(e, 8 + hs as u64).unwrap()).unwrap();
        total += names.len();
        println!("{} ({} MB): {} objects, e.g. {:?}", e.name, e.size / 1_000_000, names.len(), &names[..names.len().min(4)]);
        if let Some(n) = a.get(2) {
            if names.contains(n) {
                let s = jbeam::shape::parse(&z.read(e).unwrap()).unwrap();
                println!("  {n}: found in index and parsed: {}", s.has(n));
            }
        }
    }
    println!("{total} objects indexed in {:.3} s", t.elapsed().as_secs_f64());
}
