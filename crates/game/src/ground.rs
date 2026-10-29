//! Procedural ground textures: asphalt (aggregate, stains, cracks), concrete pavers, grass, forest floor,
//! sand, car-park asphalt. Each has albedo, metallic-roughness and a normal map, tiles seamlessly, and is
//! mapped by world position (u = x, v = y in metres) so long road ribbons do not stretch it.
use bevy::prelude::*;
use scene::mesh::mat;

pub const SIZE: usize = 512;

/// Materials drawn with these textures (their meshes get world-planar UVs and tangents).
pub fn is_textured(id: u32) -> bool {
    [mat::ASPHALT, mat::SIDEWALK, mat::GRASS, mat::FOREST_FLOOR, mat::SAND, mat::PARKING, mat::GROUND].iter().any(|m| *m as u32 == id)
}

/// Metres covered by one texture repeat.
pub fn repeat_m(id: u32) -> f32 {
    match id as f32 {
        x if x == mat::SIDEWALK => 2.4,
        x if x == mat::ASPHALT || x == mat::PARKING => 12.0,
        x if x == mat::SAND => 4.0,
        _ => 10.0,
    }
}

fn hash(x: i32, y: i32, s: u32) -> f32 {
    let mut h = (x as u32).wrapping_mul(0x8DA6_B343) ^ (y as u32).wrapping_mul(0xD816_3841) ^ s.wrapping_mul(0xCB1A_B31F);
    h ^= h >> 13;
    h = h.wrapping_mul(0x5bd1_e995);
    h ^= h >> 15;
    (h & 0xFFFF) as f32 / 65535.0
}

/// Tileable value noise: `cells` lattice cells across the texture.
fn vnoise(u: f32, v: f32, cells: i32, s: u32) -> f32 {
    let (x, y) = (u * cells as f32, v * cells as f32);
    let (xi, yi) = (x.floor() as i32, y.floor() as i32);
    let (fx, fy) = (x - xi as f32, y - yi as f32);
    let sm = |t: f32| t * t * (3.0 - 2.0 * t);
    let (sx, sy) = (sm(fx), sm(fy));
    let w = |i: i32| i.rem_euclid(cells);
    let a = hash(w(xi), w(yi), s);
    let b = hash(w(xi + 1), w(yi), s);
    let c = hash(w(xi), w(yi + 1), s);
    let d = hash(w(xi + 1), w(yi + 1), s);
    (a * (1.0 - sx) + b * sx) * (1.0 - sy) + (c * (1.0 - sx) + d * sx) * sy
}

fn fbm(u: f32, v: f32, base: i32, oct: u32, s: u32) -> f32 {
    let (mut sum, mut amp, mut norm) = (0.0, 1.0, 0.0);
    for o in 0..oct {
        sum += vnoise(u, v, base << o, s + o * 17) * amp;
        norm += amp;
        amp *= 0.5;
    }
    sum / norm
}

struct G {
    c: [f32; 3],
    rough: f32,
    h: f32,
}

fn asphalt(u: f32, v: f32, park: bool) -> G {
    let base = fbm(u, v, 6, 5, 1);
    let stain = fbm(u, v, 3, 3, 9);
    let speck = hash((u * SIZE as f32) as i32, (v * SIZE as f32) as i32, 3);
    // fine cracks, only in worn patches
    let worn = fbm(u, v, 2, 2, 27) > 0.62;
    let crack_n = (fbm(u + 0.08 * vnoise(u, v, 5, 5), v, 5, 4, 21) - 0.5).abs();
    let crack = worn && crack_n < 0.0045;
    // repair patches: slightly darker, smoother rectangles
    // repair patches: few, of different sizes, with ragged edges
    let pu = (u * 4.0).floor();
    let pv = (v * 4.0).floor();
    let ph = hash(pu as i32, pv as i32, 61);
    let (fu, fv) = ((u * 4.0).fract(), (v * 4.0).fract());
    let rag = (vnoise(u, v, 64, 63) - 0.5) * 0.06;
    let (w, h) = (0.3 + ph * 0.5, 0.2 + hash(pu as i32, pv as i32, 67) * 0.5);
    let patch = ph > 0.88 && fu > 0.1 + rag && fu < 0.1 + w + rag && fv > 0.15 - rag && fv < 0.15 + h - rag;
    let mut g = if park { 0.105 } else { 0.085 } + (base - 0.5) * 0.03 - (stain - 0.55).max(0.0) * 0.05;
    if patch {
        g *= 0.82;
    }
    if speck > 0.975 {
        g += 0.05;
    } else if speck < 0.03 {
        g -= 0.02;
    }
    if crack {
        g *= 0.55;
    }
    let rough = if crack { 0.95 } else if patch { 0.7 } else { 0.8 + base * 0.12 };
    G { c: [g, g, g * 1.03], rough, h: base * 0.3 + speck * 0.35 - crack as i32 as f32 * 0.5 }
}

fn pavers(u: f32, v: f32) -> G {
    // 0.3 m square slabs, every other row offset by half; 2.4 m per repeat = 8 slabs
    let n = 8.0;
    let row = (v * n).floor();
    let off = if row as i32 % 2 == 0 { 0.0 } else { 0.5 };
    let (fx, fy) = ((u * n + off).fract(), (v * n).fract());
    let joint = fx < 0.04 || fy < 0.04 || fx > 0.96 || fy > 0.96;
    let tile = hash((u * n + off).floor() as i32, row as i32, 7);
    let n2 = fbm(u, v, 8, 3, 2);
    let g = if joint { 0.30 } else { 0.50 + tile * 0.08 + (n2 - 0.5) * 0.06 };
    G { c: [g, g * 0.99, g * 0.96], rough: if joint { 0.95 } else { 0.8 }, h: if joint { 0.0 } else { 0.8 + n2 * 0.2 } }
}

fn grass(u: f32, v: f32, forest: bool) -> G {
    let a = fbm(u, v, 4, 5, 31);
    let b = fbm(u, v, 16, 3, 37);
    let blade = hash((u * SIZE as f32) as i32, (v * SIZE as f32) as i32, 41);
    let dry = (fbm(u, v, 3, 3, 43) - 0.62).max(0.0) * 3.0;
    let mut c = if forest { [0.20, 0.17, 0.10] } else { [0.20, 0.33, 0.11] };
    let k = 0.75 + a * 0.4 + (b - 0.5) * 0.2 + (blade - 0.5) * 0.12;
    c = c.map(|x| x * k);
    if forest {
        // leaf litter and moss
        let moss = (fbm(u, v, 6, 3, 47) - 0.55).max(0.0) * 2.5;
        c = [c[0] * (1.0 - moss) + 0.16 * moss, c[1] * (1.0 - moss) + 0.26 * moss, c[2] * (1.0 - moss) + 0.08 * moss];
    } else {
        c = [c[0] + dry * 0.18, c[1] + dry * 0.10, c[2] + dry * 0.02];
    }
    G { c, rough: 0.95, h: a * 0.5 + blade * 0.5 }
}

fn sand(u: f32, v: f32) -> G {
    let a = fbm(u, v, 6, 5, 51);
    let grain = hash((u * SIZE as f32) as i32, (v * SIZE as f32) as i32, 53);
    let ripple = ((v * 24.0 + a * 3.0) * std::f32::consts::TAU).sin() * 0.5 + 0.5;
    let g = 0.66 + (a - 0.5) * 0.08 + (grain - 0.5) * 0.06;
    G { c: [g, g * 0.92, g * 0.72], rough: 0.95, h: ripple * 0.5 + grain * 0.2 }
}

fn sample(id: u32, u: f32, v: f32) -> G {
    match id as f32 {
        x if x == mat::ASPHALT => asphalt(u, v, false),
        x if x == mat::PARKING => asphalt(u, v, true),
        x if x == mat::SIDEWALK => pavers(u, v),
        x if x == mat::FOREST_FLOOR => grass(u, v, true),
        x if x == mat::SAND => sand(u, v),
        _ => grass(u, v, false),
    }
}

/// (albedo sRGB RGBA, metallic-roughness RGBA, normal RGBA), each with a full mip chain.
pub fn generate(id: u32) -> [Vec<u8>; 3] {
    let n = SIZE;
    let mut gs: Vec<G> = Vec::with_capacity(n * n);
    for y in 0..n {
        for x in 0..n {
            gs.push(sample(id, (x as f32 + 0.5) / n as f32, (y as f32 + 0.5) / n as f32));
        }
    }
    let (mut alb, mut mr, mut nm) = (vec![0u8; n * n * 4], vec![0u8; n * n * 4], vec![0u8; n * n * 4]);
    let strength = match id as f32 {
        x if x == mat::SIDEWALK => 3.0,
        x if x == mat::ASPHALT || x == mat::PARKING => 1.5,
        _ => 2.0,
    };
    for y in 0..n {
        for x in 0..n {
            let i = y * n + x;
            let g = &gs[i];
            let srgb = |c: f32| (c.clamp(0.0, 1.0).powf(1.0 / 2.2) * 255.0) as u8;
            alb[i * 4..i * 4 + 4].copy_from_slice(&[srgb(g.c[0]), srgb(g.c[1]), srgb(g.c[2]), 255]);
            mr[i * 4..i * 4 + 4].copy_from_slice(&[255, (g.rough.clamp(0.0, 1.0) * 255.0) as u8, 0, 255]);
            let hx = gs[y * n + (x + 1) % n].h - gs[y * n + (x + n - 1) % n].h;
            let hy = gs[((y + 1) % n) * n + x].h - gs[((y + n - 1) % n) * n + x].h;
            let v = Vec3::new(-hx * strength, -hy * strength, 1.0).normalize();
            nm[i * 4..i * 4 + 4].copy_from_slice(&[((v.x * 0.5 + 0.5) * 255.0) as u8, ((v.y * 0.5 + 0.5) * 255.0) as u8, ((v.z * 0.5 + 0.5) * 255.0) as u8, 255]);
        }
    }
    [mips(alb), mips(mr), mips(nm)]
}

fn mips(base: Vec<u8>) -> Vec<u8> {
    let mut out = base.clone();
    let mut cur = base;
    let mut size = SIZE;
    while size > 1 {
        let ns = size / 2;
        let mut next = vec![0u8; ns * ns * 4];
        for y in 0..ns {
            for x in 0..ns {
                for k in 0..4 {
                    let s = |dx: usize, dy: usize| cur[((y * 2 + dy) * size + x * 2 + dx) * 4 + k] as u32;
                    next[(y * ns + x) * 4 + k] = ((s(0, 0) + s(1, 0) + s(0, 1) + s(1, 1) + 2) / 4) as u8;
                }
            }
        }
        out.extend_from_slice(&next);
        cur = next;
        size = ns;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn ground_textures_tile_and_differ() {
        let [a, _, n] = generate(mat::ASPHALT as u32);
        assert_eq!(a.len(), n.len());
        // seamless: the left and right columns are close (lattice noise wraps)
        let px = |x: usize, y: usize| a[(y * SIZE + x) * 4] as i32;
        let diff: i32 = (0..SIZE).map(|y| (px(0, y) - px(SIZE - 1, y)).abs()).sum::<i32>() / SIZE as i32;
        assert!(diff < 25, "seam {diff}");
        let [g, _, _] = generate(mat::GRASS as u32);
        assert!(g[1] > g[0] && g[1] > g[2], "grass is green");
    }
}
