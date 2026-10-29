fn main() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/demo_overpass.json")).unwrap();
    let (la, lo) = (53.9045, 27.5615);
    let f = world::geo::LocalFrame::new(la, lo, 0.0);
    let t = world::tiles::TileId::from_lat_lon(la, lo, 15);
    let raw = world::osm::parse_overpass(&text, t).unwrap();
    let sc = world::osm::build_scene(&raw, &f);
    println!("{} buildings, {} entrances ({} mapped in OSM)", sc.buildings.len(), sc.entrances.len(), raw.entrances.len());
    for e in sc.entrances.iter().filter(|e| e.p[0].hypot(e.p[1]) < 120.0).take(5) {
        println!("  {:?} out {:?} {}", e.p, e.out, e.kind);
    }
}
