//! maptile <archive.pmtiles> <z> <x> <y> <out.rgba>: list the tile's layers and render it as the in-game map does.
use world::basemap::MapSource;
use world::pmtiles::{FileSource, PmTiles, RangeSource};
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let open = || -> PmTiles<Box<dyn RangeSource>> { PmTiles::open(Box::new(FileSource(a[1].clone().into())) as Box<dyn RangeSource>).unwrap() };
    let (z, x, y): (u8, u32, u32) = (a[2].parse().unwrap(), a[3].parse().unwrap(), a[4].parse().unwrap());
    let p = open();
    println!("zoom {}..{}, bounds {:.3},{:.3} .. {:.3},{:.3}", p.header.min_zoom, p.header.max_zoom, p.header.min_lat, p.header.min_lon, p.header.max_lat, p.header.max_lon);
    let w = world::basemap::window(z, x, y, p.header.max_zoom);
    if let Some(b) = p.get(w.sz, w.sx, w.sy).unwrap() {
        for l in world::mvt::decode(&b).unwrap() {
            let mut kinds: std::collections::BTreeMap<String, usize> = Default::default();
            for f in &l.features {
                let k = ["kind", "pmap:kind", "subtype", "class"].iter().find_map(|k| f.prop(k).and_then(|v| v.as_str()).map(|s| format!("{k}={s}"))).unwrap_or_default();
                *kinds.entry(k).or_default() += 1;
            }
            println!("layer {} ({} features): {:?}", l.name, l.features.len(), kinds);
        }
    }
    let src = MapSource { base: Some(open()), transport: Some(open()), buildings: Some(open()) };
    let r = src.render(z, x, y).unwrap();
    std::fs::write(&a[5], &r.px).unwrap();
}
