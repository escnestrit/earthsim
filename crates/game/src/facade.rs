//! Procedural facade textures. One texture covers 4 x 4 bays of 3 m x 3 m (12 m square) so lit windows at
//! night vary between neighbours. Walls carry u = metres along the footprint and v = height scaled to 3 m
//! storeys (see scene::mesh), so the texture repeats every 12 m in both directions.
//!
//! These are generic styles chosen from map attributes (material, colour, class, storeys); they are NOT the
//! real facade of the building.
use bevy::asset::RenderAssetUsages;
use bevy::image::{ImageAddressMode, ImageSampler, ImageSamplerDescriptor};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use scene::mesh::mat;

pub const TEX: usize = 512;
pub const BAYS: usize = 4;
const BAY: usize = TEX / BAYS; // pixels per 3 m
pub const REPEAT_M: f32 = 12.0;

/// Per pixel: albedo (sRGB), roughness, metallic, lit-window emission (0..1).
#[derive(Clone, Copy)]
struct Px {
    c: [f32; 3],
    rough: f32,
    metal: f32,
    lit: f32,
    /// relief for parallax mapping: 0 = proud (sills, pilasters), 0.1 = wall plane, up to 1 = deepest (glass)
    depth: f32,
}

const WALL_D: f32 = 0.1;

fn hash(a: u32, b: u32) -> f32 {
    let mut h = a.wrapping_mul(0x9E37_79B1) ^ b.wrapping_mul(0x85EB_CA77);
    h ^= h >> 15;
    h = h.wrapping_mul(0x2C1B_3C6D);
    h ^= h >> 12;
    (h & 0xFFFF) as f32 / 65535.0
}

struct Style {
    wall: fn(x: f32, y: f32, bx: usize, by: usize) -> Px,
    /// window rectangles inside a bay, in metres from the bay's bottom-left: (x0, x1, y0, y1)
    windows: Vec<(f32, f32, f32, f32)>,
    frame: [f32; 3],
    /// darker recessed band (loggia) under the windows: (y0, y1)
    loggia: Option<(f32, f32)>,
    /// vertical pilaster at the bay edge (width m)
    pilaster: f32,
    /// round-headed windows (churches, civic buildings)
    arch: bool,
    /// glazing divided into this many panes across (2 = one central mullion)
    panes: usize,
    /// openings are roller/garage doors, not glass
    doors: bool,
    /// no sill under the openings (shop windows, doors)
    no_sill: bool,
}

fn plaster(x: f32, y: f32, _: usize, _: usize) -> Px {
    let n = hash((x * 40.0) as u32, (y * 40.0) as u32) * 0.04;
    // cornice line at each floor slab
    let band = if y < 0.12 { 0.88 } else { 0.97 };
    Px { c: [band - n, band - n, band - n], rough: 0.9, metal: 0.0, lit: 0.0, depth: WALL_D }
}

fn brick(x: f32, y: f32, _: usize, _: usize) -> Px {
    let course = (y / 0.075) as u32;
    let off = if course % 2 == 0 { 0.0 } else { 0.125 };
    let bx = ((x + off) / 0.25).fract();
    let by = (y / 0.075).fract();
    let mortar = bx < 0.05 || by < 0.13;
    let v = hash(((x + off) / 0.25) as u32, course) * 0.18;
    if mortar {
        Px { c: [1.25, 1.2, 1.1], rough: 0.95, metal: 0.0, lit: 0.0, depth: 0.22 }
    } else {
        Px { c: [0.92 - v, 0.9 - v, 0.9 - v], rough: 0.85, metal: 0.0, lit: 0.0, depth: WALL_D }
    }
}

fn panel(x: f32, y: f32, bx: usize, by: usize) -> Px {
    // large precast panels: seams every bay horizontally and every storey vertically
    let seam = x < 0.03 || y < 0.03;
    let n = hash(bx as u32 * 7 + (x * 8.0) as u32, by as u32 * 13 + (y * 8.0) as u32) * 0.06;
    let v = if seam { 0.55 } else { 0.95 - n };
    Px { c: [v, v, v], rough: 0.92, metal: 0.0, lit: 0.0, depth: if seam { 0.35 } else { WALL_D } }
}

fn glass(x: f32, y: f32, _: usize, _: usize) -> Px {
    // curtain wall: mullions every 1.5 m, spandrel band at each slab
    let mull = (x / 1.5).fract() < 0.03 || y < 0.04;
    let spandrel = y < 0.7;
    if mull {
        Px { c: [0.55, 0.57, 0.6], rough: 0.4, metal: 0.8, lit: 0.0, depth: WALL_D }
    } else if spandrel {
        Px { c: [0.35, 0.38, 0.42], rough: 0.3, metal: 0.2, lit: 0.0, depth: WALL_D }
    } else {
        Px { c: [0.27, 0.35, 0.43], rough: 0.08, metal: 0.0, lit: 1.0, depth: WALL_D }
    }
}

fn wood(_x: f32, y: f32, _: usize, by: usize) -> Px {
    let board = (y / 0.18).fract();
    let v = 0.9 - hash((y / 0.18) as u32, by as u32) * 0.15 - if board < 0.08 { 0.3 } else { 0.0 };
    Px { c: [v, v, v], rough: 0.85, metal: 0.0, lit: 0.0, depth: WALL_D }
}

fn metal(x: f32, y: f32, _: usize, _: usize) -> Px {
    let rib = ((x / 0.2).fract() - 0.5).abs() * 2.0;
    let v = 0.82 + rib * 0.15 - if y < 0.05 { 0.2 } else { 0.0 };
    Px { c: [v, v, v], rough: 0.45, metal: 0.6, lit: 0.0, depth: WALL_D }
}

fn tile_panel(x: f32, y: f32, bx: usize, by: usize) -> Px {
    // precast panels faced with small ceramic tiles (P-44, II-49): grout grid, panel seams every bay/storey
    let seam = x < 0.03 || y < 0.03;
    let (tx, ty) = ((x / 0.12).fract(), (y / 0.12).fract());
    let grout = tx < 0.12 || ty < 0.12;
    let n = hash((x / 0.12) as u32 + bx as u32 * 97, (y / 0.12) as u32 + by as u32 * 89) * 0.08;
    let v = if seam { 0.5 } else if grout { 0.8 } else { 0.97 - n };
    Px { c: [v, v, v], rough: if grout { 0.95 } else { 0.35 }, metal: 0.0, lit: 0.0, depth: if seam { 0.35 } else if grout { 0.16 } else { WALL_D } }
}

fn silicate(x: f32, y: f32, bx: usize, by: usize) -> Px {
    // white sand-lime brick: smaller courses, pale mortar, a little colour noise
    let course = (y / 0.088) as u32;
    let off = if course % 2 == 0 { 0.0 } else { 0.125 };
    let fx = ((x + off) / 0.25).fract();
    let fy = (y / 0.088).fract();
    let mortar = fx < 0.04 || fy < 0.11;
    let v = hash(((x + off) / 0.25) as u32 + bx as u32 * 7, course + by as u32 * 31) * 0.07;
    if mortar {
        Px { c: [0.8, 0.79, 0.76], rough: 0.95, metal: 0.0, lit: 0.0, depth: 0.2 }
    } else {
        Px { c: [0.97 - v, 0.96 - v, 0.93 - v], rough: 0.85, metal: 0.0, lit: 0.0, depth: WALL_D }
    }
}

fn glass_ribbon(x: f32, y: f32, bx: usize, by: usize) -> Px {
    // ribbon windows: a solid band at each slab, continuous glazing above
    if y < 1.0 {
        let v = 0.9 - hash(bx as u32, by as u32) * 0.03;
        Px { c: [v, v, v], rough: 0.6, metal: 0.1, lit: 0.0, depth: WALL_D }
    } else {
        glass(x, y, bx, by)
    }
}

fn rustic(x: f32, y: f32, bx: usize, by: usize) -> Px {
    // rusticated stone / render: deep horizontal joints every 0.375 m, staggered vertical joints
    let course = (y / 0.375) as u32;
    let off = if course % 2 == 0 { 0.0 } else { 0.6 };
    let fx = ((x + off) / 1.2).fract();
    let fy = (y / 0.375).fract();
    let joint = fy < 0.07 || fx < 0.025;
    let v = 0.95 - hash(((x + off) / 1.2) as u32 + bx as u32 * 13, course + by as u32 * 7) * 0.05;
    if joint {
        Px { c: [0.7, 0.7, 0.7], rough: 0.95, metal: 0.0, lit: 0.0, depth: 0.3 }
    } else {
        Px { c: [v, v, v], rough: 0.85, metal: 0.0, lit: 0.0, depth: WALL_D }
    }
}

fn sandwich(x: f32, y: f32, _: usize, _: usize) -> Px {
    // horizontal sandwich panels of big-box stores and warehouses: 1 m high, shallow ribs
    let fy = y.fract();
    let rib = (fy * 4.0).fract() < 0.08;
    let seam = fy < 0.02 || (x / 6.0).fract() < 0.004;
    let v = if seam { 0.7 } else if rib { 0.88 } else { 0.97 };
    Px { c: [v, v, v], rough: 0.5, metal: 0.3, lit: 0.0, depth: if seam { 0.3 } else if rib { 0.15 } else { WALL_D } }
}

fn alu_panel(x: f32, y: f32, bx: usize, by: usize) -> Px {
    // composite aluminium cladding: 1.5 m x 1 m cassettes, thin dark joints
    let joint = (x / 1.5).fract() < 0.012 || (y / 1.0).fract() < 0.018;
    let v = 0.93 - hash(bx as u32 * 3 + (x / 1.5) as u32, by as u32 * 5 + y as u32) * 0.025;
    if joint {
        Px { c: [0.35, 0.36, 0.38], rough: 0.6, metal: 0.2, lit: 0.0, depth: 0.25 }
    } else {
        Px { c: [v, v, v * 1.01], rough: 0.35, metal: 0.55, lit: 0.0, depth: WALL_D }
    }
}

fn stone_slab(x: f32, y: f32, bx: usize, by: usize) -> Px {
    // large stone/porcelain slabs 1.5 m x 0.75 m
    let joint = (x / 1.5).fract() < 0.01 || (y / 0.75).fract() < 0.02;
    let v = 0.92 - hash(bx as u32 * 7 + (x / 1.5) as u32, by as u32 * 11 + (y / 0.75) as u32) * 0.08 + hash((x * 30.0) as u32, (y * 30.0) as u32) * 0.02;
    Px { c: if joint { [0.6, 0.6, 0.6] } else { [v, v * 0.98, v * 0.95] }, rough: 0.5, metal: 0.0, lit: 0.0, depth: if joint { 0.2 } else { WALL_D } }
}

/// Style index (plaster, brick, panel, glass, wood, metal) and variant (window layout) of a facade material.
fn style(id: f32) -> Style {
    let dark = [0.25, 0.25, 0.25];
    let white = [0.95, 0.95, 0.95];
    let (si, v) = mat::facade_parts(id);
    let st = |wall, windows: Vec<(f32, f32, f32, f32)>, frame| Style { wall, windows, frame, loggia: None, pilaster: 0.0, arch: false, panes: 2, doors: false, no_sill: false };
    let grey = [0.32, 0.33, 0.35];
    match (si, v) {
        // blank walls: monuments (stone), towers and chimneys (concrete, brick)
        (12, 0) => st(stone_slab, vec![], dark),
        (12, 1) => st(panel, vec![], dark),
        (12, _) => st(brick, vec![], dark),
        // arched: churches, theatres, old civic buildings
        (8, 0) => Style { pilaster: 0.35, arch: true, ..st(plaster, vec![(1.0, 2.0, 0.6, 2.65)], white) },
        (8, 1) => Style { arch: true, ..st(plaster, vec![(0.55, 1.25, 0.8, 2.55), (1.75, 2.45, 0.8, 2.55)], white) },
        (8, _) => Style { arch: true, ..st(rustic, vec![(0.95, 2.05, 0.7, 2.6)], [0.9, 0.9, 0.88]) },
        // schools, universities, hospitals: wide three-pane windows
        (9, 0) => Style { panes: 3, ..st(plaster, vec![(0.2, 2.8, 0.85, 2.6)], white) },
        (9, 1) => Style { panes: 3, ..st(brick, vec![(0.25, 2.75, 0.85, 2.6)], white) },
        (9, _) => Style { panes: 3, ..st(silicate, vec![(0.2, 2.8, 0.85, 2.6)], white) },
        // shop windows; big-box sandwich panels; garage doors
        (10, 0) => Style { panes: 1, no_sill: true, ..st(stone_slab, vec![(0.12, 2.88, 0.12, 2.45)], grey) },
        (10, 1) => st(sandwich, vec![], dark),
        (10, _) => Style { doors: true, no_sill: true, ..st(brick, vec![(0.3, 2.7, 0.0, 2.25)], [0.6, 0.6, 0.6]) },
        // offices: aluminium cladding with strip windows, glazing between fins, stone slabs
        (11, 0) => Style { panes: 3, ..st(alu_panel, vec![(0.15, 2.85, 0.95, 2.7)], grey) },
        (11, 1) => Style { pilaster: 0.4, panes: 2, no_sill: true, ..st(alu_panel, vec![(0.3, 2.7, 0.25, 2.8)], grey) },
        (11, _) => Style { panes: 2, ..st(stone_slab, vec![(0.45, 2.55, 0.85, 2.6)], grey) },
        (1, 0) => st(brick, vec![(0.85, 2.15, 0.85, 2.45)], [0.92, 0.92, 0.9]),
        (1, 1) => Style { pilaster: 0.25, ..st(brick, vec![(0.95, 2.05, 0.7, 2.55)], white) },
        (1, _) => st(brick, vec![(0.55, 1.3, 0.9, 2.3), (1.7, 2.45, 0.9, 2.3)], [0.85, 0.85, 0.82]),
        (2, 0) => st(panel, vec![(0.75, 2.25, 0.9, 2.3)], [0.9, 0.9, 0.88]),
        (2, 1) => Style { loggia: Some((0.05, 0.95)), ..st(panel, vec![(0.6, 2.4, 1.0, 2.4)], [0.92, 0.92, 0.9]) },
        (2, _) => st(panel, vec![(0.45, 1.35, 0.9, 2.35), (1.75, 2.55, 0.9, 2.35)], [0.88, 0.88, 0.86]),
        (3, 0) => st(glass, vec![], dark),
        (3, 1) => st(glass_ribbon, vec![], dark),
        (3, _) => st(glass, vec![], [0.7, 0.7, 0.72]),
        (4, 0) => st(wood, vec![(1.0, 2.0, 1.0, 2.2)], white),
        (4, _) => st(wood, vec![(0.6, 1.4, 0.9, 2.2), (1.8, 2.4, 0.9, 2.2)], white),
        (6, 0) => st(tile_panel, vec![(0.6, 2.4, 0.9, 2.35)], white),
        (6, 1) => Style { loggia: Some((0.05, 0.9)), ..st(tile_panel, vec![(0.45, 1.3, 0.9, 2.35), (1.7, 2.55, 0.9, 2.35)], white) },
        (6, _) => st(tile_panel, vec![(0.75, 2.25, 0.95, 2.3)], [0.88, 0.88, 0.86]),
        (7, 0) => st(silicate, vec![(0.8, 2.2, 0.9, 2.35)], white),
        (7, _) => st(silicate, vec![(0.55, 1.3, 0.9, 2.3), (1.7, 2.45, 0.9, 2.3)], white),
        (5, 0) => st(metal, vec![], dark),
        (5, 1) => st(metal, vec![(0.3, 2.7, 2.0, 2.6)], dark),
        (5, _) => st(metal, vec![(0.6, 2.4, 1.6, 2.5)], dark),
        (_, 0) => st(plaster, vec![(0.8, 2.2, 0.9, 2.4)], white),
        (_, 1) => Style { pilaster: 0.3, ..st(plaster, vec![(1.0, 2.0, 0.75, 2.55)], white) },
        _ => st(plaster, vec![(0.5, 1.35, 0.95, 2.35), (1.65, 2.5, 0.95, 2.35)], [0.9, 0.88, 0.84]),
    }
}

fn pixel(st: &Style, px: usize, py_from_bottom: usize) -> Px {
    let (bx, by) = (px / BAY, py_from_bottom / BAY);
    let m = 3.0 / BAY as f32;
    let x = (px % BAY) as f32 * m + m * 0.5;
    let y = (py_from_bottom % BAY) as f32 * m + m * 0.5;
    // pilasters at the bay edges and recessed loggia bands
    if st.pilaster > 0.0 && (x < st.pilaster / 2.0 || x > 3.0 - st.pilaster / 2.0) {
        let mut p = (st.wall)(x, y, bx, by);
        p.c = p.c.map(|v| v * 0.9);
        p.depth = 0.0;
        return p;
    }
    if let Some((l0, l1)) = st.loggia {
        if y > l0 && y < l1 && x > 0.3 && x < 2.7 {
            let mut p = (st.wall)(x, y, bx, by);
            p.c = p.c.map(|v| v * 0.62);
            p.depth = 0.75;
            return p;
        }
    }
    for &(x0, x1, y0, y1) in &st.windows {
        let f = 0.07;
        // round head: above the spring line the opening is a half circle (an ellipse when stretched)
        let r = (x1 - x0) / 2.0;
        let cx = (x0 + x1) / 2.0;
        let in_arch = |pad: f32| {
            let yc = y1 - r;
            y <= yc || ((x - cx) / (r + pad)).powi(2) + ((y - yc) / (r + pad)).powi(2) <= 1.0
        };
        if x >= x0 - f && x <= x1 + f && y >= y0 - f && y <= y1 + f && (!st.arch || in_arch(f)) {
            let inside = x >= x0 && x <= x1 && y >= y0 && y <= y1 && (!st.arch || in_arch(0.0));
            if st.doors && inside {
                // roller door: horizontal slats, painted metal
                let slat = (y / 0.1).fract() < 0.15;
                let v = if slat { 0.62 } else { 0.78 } - hash(bx as u32 * 5, by as u32 * 3 + 1) * 0.1;
                return Px { c: [v, v, v * 1.02], rough: 0.5, metal: 0.4, lit: 0.0, depth: if slat { 0.45 } else { 0.38 } };
            }
            let pane_w = (x1 - x0) / st.panes as f32;
            let mull = st.panes > 1 && (1..st.panes).any(|k| (x - (x0 + pane_w * k as f32)).abs() < 0.03);
            let transom = st.arch && (y - (y1 - r)).abs() < 0.03;
            if !inside || mull || transom {
                // frame set back into the opening; the mullion a little deeper
                return Px { c: st.frame, rough: 0.5, metal: 0.0, lit: 0.0, depth: if inside { 0.55 } else { 0.4 } };
            }
            // glass deep in the reveal: ambient occlusion near the edges, occasional curtains, reflections
            let curtain = hash(bx as u32 * 31 + 1, by as u32 * 17 + 3) > 0.7 && y > y1 - (y1 - y0) * 0.35;
            let g = if curtain { [0.55, 0.52, 0.48] } else { [0.13, 0.17, 0.22] };
            let edge = (x - x0).min(x1 - x).min(y - y0).min(y1 - y);
            let ao = 0.55 + 0.45 * (edge / 0.25).clamp(0.0, 1.0);
            let sky = if curtain { 1.0 } else { 1.0 + 0.6 * ((y - y0) / (y1 - y0)) };
            let k = ao * sky;
            return Px { c: [g[0] * k, g[1] * k, g[2] * k], rough: 0.04, metal: 0.0, lit: 1.0, depth: 1.0 };
        }
        if st.no_sill {
            continue;
        }
        // sill: a proud ledge under the window, its shadow below
        if x >= x0 - 0.1 && x <= x1 + 0.1 && y < y0 - f && y > y0 - f - 0.06 {
            let mut p = (st.wall)(x, y, bx, by);
            p.c = p.c.map(|v| v * 1.05);
            p.depth = 0.0;
            return p;
        }
        if x >= x0 && x <= x1 && y < y0 - f - 0.06 && y > y0 - f - 0.16 {
            let mut p = (st.wall)(x, y, bx, by);
            p.c = p.c.map(|v| v * 0.75);
            return p;
        }
        // weathering: faint streaks running down from the sill corners
        if ((x - x0).abs() < 0.08 || (x - x1).abs() < 0.08) && y < y0 - 0.16 && y > y0 - 1.4 {
            let mut p = (st.wall)(x, y, bx, by);
            let k = 0.93 + 0.07 * ((y0 - y) / 1.4) + hash((x * 50.0) as u32, (y * 30.0) as u32) * 0.02;
            p.c = p.c.map(|v| v * k);
            return p;
        }
    }
    (st.wall)(x, y, bx, by)
}

/// RGBA8 images (base level + full mip chain): albedo, metallic-roughness, emissive.
/// Facade images (base level + full mip chain): albedo, metallic-roughness, emissive (RGBA8), relief (R8).
pub fn generate(id: f32) -> [Vec<u8>; 4] {
    let st = style(id);
    let mut alb = vec![0u8; TEX * TEX * 4];
    let mut mr = vec![0u8; TEX * TEX * 4];
    let mut em = vec![0u8; TEX * TEX * 4];
    let mut dep = vec![0u8; TEX * TEX];
    for row in 0..TEX {
        let py = TEX - 1 - row; // image rows go down, metres go up
        for px in 0..TEX {
            let p = pixel(&st, px, py);
            dep[row * TEX + px] = (p.depth.clamp(0.0, 1.0) * 255.0) as u8;
            let i = (row * TEX + px) * 4;
            for k in 0..3 {
                alb[i + k] = (p.c[k].clamp(0.0, 1.0) * 255.0) as u8;
            }
            alb[i + 3] = 255;
            mr[i] = 255;
            mr[i + 1] = (p.rough.clamp(0.0, 1.0) * 255.0) as u8;
            mr[i + 2] = (p.metal.clamp(0.0, 1.0) * 255.0) as u8;
            mr[i + 3] = 255;
            // about a third of the windows are lit at night, warm or cool white
            let (bx, by) = (px / BAY, py / BAY);
            let on = p.lit > 0.0 && hash(bx as u32 + 101, by as u32 + 211) < 0.35;
            let warm = hash(bx as u32 + 7, by as u32 + 5) < 0.7;
            let e = if on { if warm { [1.0, 0.78, 0.45] } else { [0.85, 0.9, 1.0] } } else { [0.0; 3] };
            for k in 0..3 {
                em[i + k] = (e[k] * 255.0) as u8;
            }
            em[i + 3] = 255;
        }
    }
    [with_mips(alb, 4), with_mips(mr, 4), with_mips(em, 4), with_mips(dep, 1)]
}

pub fn mip_count() -> u32 {
    TEX.trailing_zeros() + 1
}

/// Box-filtered mip chain appended after the base level (wgpu's layer-major order for a single layer).
fn with_mips(base: Vec<u8>, ch: usize) -> Vec<u8> {
    let mut out = base.clone();
    let mut cur = base;
    let mut size = TEX;
    while size > 1 {
        let ns = size / 2;
        let mut next = vec![0u8; ns * ns * ch];
        for y in 0..ns {
            for x in 0..ns {
                for k in 0..ch {
                    let s = |dx: usize, dy: usize| cur[((y * 2 + dy) * size + x * 2 + dx) * ch + k] as u32;
                    next[(y * ns + x) * ch + k] = ((s(0, 0) + s(1, 0) + s(0, 1) + s(1, 1) + 2) / 4) as u8;
                }
            }
        }
        out.extend_from_slice(&next);
        cur = next;
        size = ns;
    }
    out
}

pub fn image(data: Vec<u8>, srgb: bool) -> Image {
    image_fmt(data, if srgb { TextureFormat::Rgba8UnormSrgb } else { TextureFormat::Rgba8Unorm })
}

pub fn image_fmt(data: Vec<u8>, fmt: TextureFormat) -> Image {
    let mut img = Image::new_uninit(Extent3d { width: TEX as u32, height: TEX as u32, depth_or_array_layers: 1 }, TextureDimension::D2, fmt, RenderAssetUsages::RENDER_WORLD);
    img.data = Some(data);
    img.texture_descriptor.mip_level_count = mip_count();
    img.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::Repeat,
        address_mode_v: ImageAddressMode::Repeat,
        anisotropy_clamp: 8,
        ..ImageSamplerDescriptor::linear()
    });
    img
}

/// All facade materials: 8 styles x 3 window layouts.
pub fn styles() -> Vec<f32> {
    (0..mat::FACADE_STYLES).flat_map(|s| (0..3).map(move |v| mat::facade(s, v))).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn textures_have_full_mip_chains_and_windows() {
        for s in styles() {
            let [a, m, e, d] = generate(s);
            let expect: usize = (0..mip_count()).map(|l| (TEX >> l).max(1).pow(2) * 4).sum();
            assert_eq!((a.len(), m.len(), e.len(), d.len() * 4), (expect, expect, expect, expect));
        }
        // plaster: some windows lit (emissive), most of the wall not; glass is the deepest relief
        let [_, _, e, d] = generate(mat::WALL_PLASTER);
        let deep = d[..TEX * TEX].iter().filter(|v| **v == 255).count();
        assert!(deep > 1000 && deep < TEX * TEX / 3, "{deep}");
        let lit = e[..TEX * TEX * 4].chunks_exact(4).filter(|p| p[0] > 0).count();
        assert!(lit > 1000 && lit < TEX * TEX / 4, "{lit}");
    }
}
