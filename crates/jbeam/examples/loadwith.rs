//! Load a mod as the game does, with an explicit BeamNG vehicles dir: loadwith <zip> <main> [vehicles_dir|-]
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let dir = a.get(3).filter(|d| *d != "-").map(std::path::PathBuf::from);
    let t = std::time::Instant::now();
    match jbeam::load_vehicle_with(std::path::Path::new(&a[1]), a.get(2).map(String::as_str), dir.as_deref()) {
        Ok(lv) => {
            println!("ok in {:.1}s: {} nodes, {} beams, {} wheels, eye from mod {}", t.elapsed().as_secs_f64(), lv.nodes, lv.beams, lv.vehicle.wheels.len(), lv.vehicle.eye.from_mod);
            for n in &lv.notes { println!("  {n}"); }
        }
        Err(e) => println!("error: {e}"),
    }
}
