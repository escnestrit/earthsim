//! Terrain: elevation tiles in the "terrarium" PNG encoding (AWS Terrain Tiles / Mapzen: SRTM, GMTED,
//! NED, ETOPO1 and other public sources; attribution required), decoded and resampled into a local height
//! grid around the car. Heights are relative to a datum (the ground at the place the player started), so
//! local z stays near zero.
use crate::geo::LocalFrame;
use phys::HeightField;
use std::collections::HashMap;

pub const TERRARIUM_URL: &str = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium";
/// ~19 m per pixel at the equator, ~11 m at 55°N.
pub const DEM_ZOOM: u8 = 13;

/// Minimal PNG decoder: 8-bit RGB/RGBA (or grey), non-interlaced. Returns (width, height, RGB bytes).
pub fn decode_png(b: &[u8]) -> Result<(usize, usize, Vec<u8>), String> {
    if b.len() < 8 || &b[..8] != b"\x89PNG\r\n\x1a\n" {
        return Err("not a PNG".into());
    }
    let (mut w, mut h, mut ct, mut idat) = (0usize, 0usize, 0u8, Vec::new());
    let mut p = 8;
    while p + 8 <= b.len() {
        let len = u32::from_be_bytes(b[p..p + 4].try_into().unwrap()) as usize;
        let ty = &b[p + 4..p + 8];
        let data = b.get(p + 8..p + 8 + len).ok_or("PNG truncated")?;
        match ty {
            b"IHDR" => {
                w = u32::from_be_bytes(data[0..4].try_into().unwrap()) as usize;
                h = u32::from_be_bytes(data[4..8].try_into().unwrap()) as usize;
                let (depth, interlace) = (data[8], data[12]);
                ct = data[9];
                if depth != 8 || interlace != 0 || !matches!(ct, 0 | 2 | 6) {
                    return Err(format!("PNG format not supported (depth {depth}, colour {ct}, interlace {interlace})"));
                }
            }
            b"IDAT" => idat.extend_from_slice(data),
            b"IEND" => break,
            _ => {}
        }
        p += 12 + len;
    }
    if w == 0 || idat.len() < 2 {
        return Err("PNG without image data".into());
    }
    let bpp = match ct {
        0 => 1,
        2 => 3,
        _ => 4,
    };
    let raw = scene::inflate::inflate(&idat[2..], h * (w * bpp + 1)).map_err(|e| format!("PNG inflate: {e:?}"))?;
    let stride = w * bpp;
    if raw.len() < h * (stride + 1) {
        return Err("PNG data short".into());
    }
    let mut img = vec![0u8; h * stride];
    for y in 0..h {
        let f = raw[y * (stride + 1)];
        let src = &raw[y * (stride + 1) + 1..(y + 1) * (stride + 1)];
        for x in 0..stride {
            let a = if x >= bpp { img[y * stride + x - bpp] as i32 } else { 0 };
            let up = if y > 0 { img[(y - 1) * stride + x] as i32 } else { 0 };
            let c = if x >= bpp && y > 0 { img[(y - 1) * stride + x - bpp] as i32 } else { 0 };
            let pred = match f {
                0 => 0,
                1 => a,
                2 => up,
                3 => (a + up) / 2,
                4 => {
                    let pp = a + up - c;
                    let (pa, pb, pc) = ((pp - a).abs(), (pp - up).abs(), (pp - c).abs());
                    if pa <= pb && pa <= pc {
                        a
                    } else if pb <= pc {
                        up
                    } else {
                        c
                    }
                }
                _ => return Err(format!("PNG filter {f}")),
            };
            img[y * stride + x] = (src[x] as i32 + pred) as u8;
        }
    }
    let rgb = match bpp {
        3 => img,
        1 => img.iter().flat_map(|&g| [g, g, g]).collect(),
        _ => img.chunks_exact(4).flat_map(|p| [p[0], p[1], p[2]]).collect(),
    };
    Ok((w, h, rgb))
}

/// Terrarium encoding: metres = R * 256 + G + B / 256 - 32768.
pub fn terrarium_heights(png: &[u8]) -> Result<(usize, Vec<f32>), String> {
    let (w, h, rgb) = decode_png(png)?;
    if w != h {
        return Err("DEM tile is not square".into());
    }
    Ok((w, rgb.chunks_exact(3).map(|p| p[0] as f32 * 256.0 + p[1] as f32 + p[2] as f32 / 256.0 - 32768.0).collect()))
}

/// Where elevation tiles come from (network, disk, tests).
pub trait DemSource: Send + Sync {
    /// PNG bytes of terrarium tile z/x/y.
    fn tile(&self, z: u8, x: u32, y: u32) -> Result<Vec<u8>, String>;
}

/// Decoded DEM tiles with bilinear lookup by latitude/longitude.
pub struct DemCache<'a> {
    src: &'a dyn DemSource,
    tiles: HashMap<(u32, u32), Option<(usize, Vec<f32>)>>,
    pub errors: Vec<String>,
}

impl<'a> DemCache<'a> {
    pub fn new(src: &'a dyn DemSource) -> Self {
        DemCache { src, tiles: HashMap::new(), errors: Vec::new() }
    }

    /// Elevation (m) or None where no tile could be read.
    pub fn height(&mut self, lat: f64, lon: f64) -> Option<f64> {
        let n = (1u64 << DEM_ZOOM) as f64;
        let wx = (lon + 180.0) / 360.0 * n;
        let lr = lat.clamp(-85.0, 85.0).to_radians();
        let wy = (1.0 - (lr.tan() + 1.0 / lr.cos()).ln() / std::f64::consts::PI) / 2.0 * n;
        let (tx, ty) = (wx.floor() as u32, wy.floor() as u32);
        let src = self.src;
        let errors = &mut self.errors;
        let t = self.tiles.entry((tx, ty)).or_insert_with(|| match src.tile(DEM_ZOOM, tx, ty).and_then(|b| terrarium_heights(&b)) {
            Ok(t) => Some(t),
            Err(e) => {
                errors.push(format!("DEM {DEM_ZOOM}/{tx}/{ty}: {e}"));
                None
            }
        });
        let (size, h) = t.as_ref()?;
        let s = *size as f64;
        // pixel centres
        let px = (wx - tx as f64) * s - 0.5;
        let py = (wy - ty as f64) * s - 0.5;
        let (i, j) = (px.floor(), py.floor());
        let (fx, fy) = (px - i, py - j);
        let at = |i: f64, j: f64| {
            let (i, j) = ((i.max(0.0) as usize).min(size - 1), (j.max(0.0) as usize).min(size - 1));
            h[j * size + i] as f64
        };
        let a = at(i, j) * (1.0 - fx) + at(i + 1.0, j) * fx;
        let b = at(i, j + 1.0) * (1.0 - fx) + at(i + 1.0, j + 1.0) * fx;
        Some(a * (1.0 - fy) + b * fy)
    }
}

/// Height grid around local point (cx, cy) in `frame`, relative to `datum` metres. Cells without data are 0.
pub fn build_field(src: &dyn DemSource, frame: &LocalFrame, cx: f64, cy: f64, half: f64, cell: f64, datum: f64) -> (HeightField, Vec<String>) {
    let n = ((2.0 * half / cell).ceil() as usize + 1).max(2);
    let (x0, y0) = (cx - half, cy - half);
    let mut dem = DemCache::new(src);
    let mut h = vec![0f32; n * n];
    for j in 0..n {
        for i in 0..n {
            let (la, lo, _) = frame.from_local(x0 + i as f64 * cell, y0 + j as f64 * cell, 0.0);
            if let Some(v) = dem.height(la, lo) {
                h[j * n + i] = (v - datum) as f32;
            }
        }
    }
    let mut errs = dem.errors;
    errs.truncate(5);
    (HeightField { x0, y0, cell, nx: n, ny: n, h }, errs)
}

/// Terrarium PNGs from a folder `<dir>/<z>/<x>/<y>.png` (offline use, tests), optionally falling back to
/// another source and saving what it returns there (a disk cache).
pub struct DirDem<S: DemSource> {
    pub dir: std::path::PathBuf,
    pub inner: Option<S>,
}

impl<S: DemSource> DemSource for DirDem<S> {
    fn tile(&self, z: u8, x: u32, y: u32) -> Result<Vec<u8>, String> {
        let p = self.dir.join(z.to_string()).join(x.to_string()).join(format!("{y}.png"));
        if let Ok(b) = std::fs::read(&p) {
            return Ok(b);
        }
        let Some(inner) = &self.inner else { return Err(format!("{} missing", p.display())) };
        let b = inner.tile(z, x, y)?;
        if let Some(d) = p.parent() {
            let _ = std::fs::create_dir_all(d);
            let _ = std::fs::write(&p, &b);
        }
        Ok(b)
    }
}

/// Never answers (no terrain: flat world).
pub struct NoDem;
impl DemSource for NoDem {
    fn tile(&self, _: u8, _: u32, _: u32) -> Result<Vec<u8>, String> {
        Err("no elevation source".into())
    }
}

    /// Encode a terrarium PNG of a plane rising `slope` metres per pixel eastwards from `base` (filter type 1).
pub fn encode_terrarium(size: usize, base: f64, slope: f64) -> Vec<u8> {
    fn crc(d: &[u8]) -> u32 {
        let mut c = 0xffff_ffffu32;
        for &b in d {
            c ^= b as u32;
            for _ in 0..8 {
                c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
            }
        }
        !c
    }
    fn chunk(out: &mut Vec<u8>, ty: &[u8], data: &[u8]) {
        out.extend_from_slice(&(data.len() as u32).to_be_bytes());
        let mut c = ty.to_vec();
        c.extend_from_slice(data);
        out.extend_from_slice(&c);
        out.extend_from_slice(&crc(&c).to_be_bytes());
    }
    let mut raw = Vec::new();
    for _y in 0..size {
        raw.push(1u8); // Sub filter
        let mut prev = [0u8; 3];
        for x in 0..size {
            let v = base + slope * x as f64 + 32768.0;
            let r = (v / 256.0).floor();
            let g = (v - r * 256.0).floor();
            let b = ((v - r * 256.0 - g) * 256.0).round();
            let px = [r as u8, g as u8, b as u8];
            for k in 0..3 {
                raw.push(px[k].wrapping_sub(prev[k]));
            }
            prev = px;
        }
    }
    // zlib stream of stored blocks
    let mut z = vec![0x78, 0x01];
    for (i, blk) in raw.chunks(65535).enumerate() {
        let last = (i + 1) * 65535 >= raw.len();
        z.push(last as u8);
        z.extend_from_slice(&(blk.len() as u16).to_le_bytes());
        z.extend_from_slice(&(!(blk.len() as u16)).to_le_bytes());
        z.extend_from_slice(blk);
    }
    z.extend_from_slice(&[0, 0, 0, 0]);
    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut ihdr = (size as u32).to_be_bytes().to_vec();
    ihdr.extend_from_slice(&(size as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &z);
    chunk(&mut out, b"IEND", &[]);
    out
}


/// Synthetic elevation (tests, demos): every tile a plane rising `slope` m per pixel eastwards from `base`.
pub struct SyntheticDem {
    pub base: f64,
    pub slope: f64,
}
impl DemSource for SyntheticDem {
    fn tile(&self, _z: u8, _x: u32, _y: u32) -> Result<Vec<u8>, String> {
        Ok(encode_terrarium(256, self.base, self.slope))
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    pub struct Ramp;
    impl DemSource for Ramp {
        fn tile(&self, _z: u8, _x: u32, _y: u32) -> Result<Vec<u8>, String> {
            Ok(encode_terrarium(256, 200.0, 0.5))
        }
    }

    #[test]
    fn decodes_terrarium_png() {
        let (n, h) = terrarium_heights(&encode_terrarium(8, 210.25, 1.0)).unwrap();
        assert_eq!(n, 8);
        assert!((h[0] - 210.25).abs() < 0.01 && (h[7] - 217.25).abs() < 0.01, "{:?}", &h[..8]);
        assert!(decode_png(b"nope").is_err());
    }

    #[test]
    fn field_follows_the_dem() {
        let f = LocalFrame::new(53.9, 27.56, 0.0);
        let (field, errs) = build_field(&Ramp, &f, 0.0, 0.0, 200.0, 20.0, 200.0);
        assert!(errs.is_empty());
        // terrain rises eastwards: 0.5 m per pixel, a z13 pixel is ~11.4 m wide here
        let (w, e) = (field.height(-150.0, 0.0), field.height(150.0, 0.0));
        assert!(e > w, "{w} {e}");
        let per_m = (e - w) / 300.0;
        assert!((per_m - 0.5 / 11.4).abs() < 0.01, "slope {per_m}");
        let (flat, errs) = build_field(&NoDem, &f, 0.0, 0.0, 100.0, 50.0, 0.0);
        assert!(flat.h.iter().all(|v| *v == 0.0) && !errs.is_empty());
    }
}
