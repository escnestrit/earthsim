fn main() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../data/demo_overpass.json")).unwrap();
    let (la, lo) = (53.9045, 27.5615);
    let f = world::geo::LocalFrame::new(la, lo, 0.0);
    let t = world::tiles::TileId::from_lat_lon(la, lo, 15);
    let raw = world::osm::parse_overpass(&text, t).unwrap();
    let sc = world::osm::build_scene(&raw, &f);
    println!("{} buildings, {} entrances ({} mapped in OSM)", sc.buildings.len(), sc.entrances.len(), raw.entrances.len());
    let mut counts: std::collections::BTreeMap<String, usize> = Default::default();
    for b in &sc.buildings {
        *counts.entry(b.design.as_ref().map_or("(none)".to_string(), |d| d.id.clone())).or_default() += 1;
    }
    println!("designs: {counts:?}");
    for b in raw.buildings.iter().take(40) {
        let t = |k: &str| world::stage_a::tag(&b.tags, k).unwrap_or("-").to_string();
        let ring: Vec<(f64, f64)> = b.ring.iter().map(|&(la, lo)| { let p = f.to_local(la, lo, 0.0); (p[0], p[1]) }).collect();
        println!("  class {} levels {} start {} series {} depth {:.1}", t("building"), t("building:levels"), t("start_date"), t("building:series"), world::stage_a::corpus_depth(&ring));
    }
    for e in sc.entrances.iter().filter(|e| e.p[0].hypot(e.p[1]) < 120.0).take(5) {
        println!("  {:?} out {:?} {}", e.p, e.out, e.kind);
    }
}
