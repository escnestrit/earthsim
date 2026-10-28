//! PMTiles v3 reader (single-file tile archive read with HTTP range requests or from a local file).
//! Spec: https://github.com/protomaps/PMTiles/blob/main/spec/v3/spec.md

use scene::inflate::gunzip;
use std::collections::HashMap;
use std::sync::Mutex;

/// Random access to the archive bytes.
pub trait RangeSource: Send + Sync {
    fn read(&self, offset: u64, len: u64) -> Result<Vec<u8>, String>;
    fn describe(&self) -> String;
}

pub struct FileSource(pub std::path::PathBuf);

impl RangeSource for FileSource {
    fn read(&self, offset: u64, len: u64) -> Result<Vec<u8>, String> {
        use std::io::{Read, Seek, SeekFrom};
        let mut f = std::fs::File::open(&self.0).map_err(|e| e.to_string())?;
        f.seek(SeekFrom::Start(offset)).map_err(|e| e.to_string())?;
        let mut buf = vec![0u8; len as usize];
        f.read_exact(&mut buf).map_err(|e| e.to_string())?;
        Ok(buf)
    }
    fn describe(&self) -> String {
        self.0.display().to_string()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Header {
    pub root_offset: u64,
    pub root_len: u64,
    pub metadata_offset: u64,
    pub metadata_len: u64,
    pub leaf_offset: u64,
    pub leaf_len: u64,
    pub data_offset: u64,
    pub data_len: u64,
    pub internal_compression: u8,
    pub tile_compression: u8,
    pub tile_type: u8,
    pub min_zoom: u8,
    pub max_zoom: u8,
    /// lon/lat in degrees
    pub min_lon: f64,
    pub min_lat: f64,
    pub max_lon: f64,
    pub max_lat: f64,
}

fn u64le(b: &[u8], o: usize) -> u64 {
    let mut a = [0u8; 8];
    a.copy_from_slice(&b[o..o + 8]);
    u64::from_le_bytes(a)
}
fn i32le(b: &[u8], o: usize) -> i32 {
    i32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

pub fn parse_header(b: &[u8]) -> Result<Header, String> {
    if b.len() < 127 || &b[0..7] != b"PMTiles" {
        return Err("not a PMTiles archive".into());
    }
    if b[7] != 3 {
        return Err(format!("unsupported PMTiles version {}", b[7]));
    }
    Ok(Header {
        root_offset: u64le(b, 8),
        root_len: u64le(b, 16),
        metadata_offset: u64le(b, 24),
        metadata_len: u64le(b, 32),
        leaf_offset: u64le(b, 40),
        leaf_len: u64le(b, 48),
        data_offset: u64le(b, 56),
        data_len: u64le(b, 64),
        internal_compression: b[97],
        tile_compression: b[98],
        tile_type: b[99],
        min_zoom: b[100],
        max_zoom: b[101],
        min_lon: i32le(b, 102) as f64 / 1e7,
        min_lat: i32le(b, 106) as f64 / 1e7,
        max_lon: i32le(b, 110) as f64 / 1e7,
        max_lat: i32le(b, 114) as f64 / 1e7,
    })
}

/// Hilbert-curve tile id (spec section "Tile IDs").
pub fn tile_id(z: u8, x: u32, y: u32) -> u64 {
    let acc: u64 = ((1u64 << (2 * z as u64)) - 1) / 3;
    let n: u64 = 1 << z;
    let (mut tx, mut ty) = (x as u64, y as u64);
    let mut d: u64 = 0;
    let mut s = n / 2;
    while s > 0 {
        let rx = u64::from(tx & s > 0);
        let ry = u64::from(ty & s > 0);
        d += s * s * ((3 * rx) ^ ry);
        if ry == 0 {
            if rx == 1 {
                tx = n - 1 - tx;
                ty = n - 1 - ty;
            }
            std::mem::swap(&mut tx, &mut ty);
        }
        s /= 2;
    }
    acc + d
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Entry {
    pub tile_id: u64,
    pub offset: u64,
    pub length: u64,
    pub run_length: u64,
}

fn varint(b: &[u8], p: &mut usize) -> Result<u64, String> {
    let mut v: u64 = 0;
    let mut shift = 0;
    loop {
        let byte = *b.get(*p).ok_or("truncated directory")?;
        *p += 1;
        v |= ((byte & 0x7f) as u64) << shift;
        if byte & 0x80 == 0 {
            return Ok(v);
        }
        shift += 7;
        if shift > 63 {
            return Err("varint too long".into());
        }
    }
}

pub fn parse_directory(b: &[u8]) -> Result<Vec<Entry>, String> {
    let mut p = 0;
    let n = varint(b, &mut p)? as usize;
    if n > 10_000_000 {
        return Err("directory too large".into());
    }
    let mut e = vec![Entry { tile_id: 0, offset: 0, length: 0, run_length: 0 }; n];
    let mut last = 0u64;
    for x in e.iter_mut() {
        last += varint(b, &mut p)?;
        x.tile_id = last;
    }
    for x in e.iter_mut() {
        x.run_length = varint(b, &mut p)?;
    }
    for x in e.iter_mut() {
        x.length = varint(b, &mut p)?;
    }
    for i in 0..n {
        let v = varint(b, &mut p)?;
        e[i].offset = if v == 0 && i > 0 { e[i - 1].offset + e[i - 1].length } else { v.wrapping_sub(1) };
    }
    Ok(e)
}

fn find(entries: &[Entry], id: u64) -> Option<Entry> {
    // last entry with tile_id <= id
    let i = entries.partition_point(|e| e.tile_id <= id);
    if i == 0 {
        return None;
    }
    let e = entries[i - 1];
    if e.run_length == 0 || id < e.tile_id + e.run_length {
        Some(e)
    } else {
        None
    }
}

fn decompress(kind: u8, data: Vec<u8>) -> Result<Vec<u8>, String> {
    match kind {
        0 | 1 => Ok(data),
        2 => gunzip(&data).map_err(|e| e.to_string()),
        3 => Err("brotli-compressed archive is not supported".into()),
        4 => Err("zstd-compressed archive is not supported".into()),
        k => Err(format!("unknown compression {k}")),
    }
}

pub struct PmTiles<S: RangeSource> {
    pub src: S,
    pub header: Header,
    root: Vec<Entry>,
    leaves: Mutex<HashMap<u64, std::sync::Arc<Vec<Entry>>>>,
}

impl<S: RangeSource> PmTiles<S> {
    pub fn open(src: S) -> Result<Self, String> {
        // header + root directory are guaranteed to fit in the first 16 KiB
        let first = src.read(0, 16384).or_else(|_| src.read(0, 127))?;
        let header = parse_header(&first)?;
        let root_raw = if header.root_offset + header.root_len <= first.len() as u64 {
            first[header.root_offset as usize..(header.root_offset + header.root_len) as usize].to_vec()
        } else {
            src.read(header.root_offset, header.root_len)?
        };
        let root = parse_directory(&decompress(header.internal_compression, root_raw)?)?;
        Ok(PmTiles { src, header, root, leaves: Mutex::new(HashMap::new()) })
    }

    /// Tile bytes, decompressed. `Ok(None)` = tile not in the archive (empty area).
    pub fn get(&self, z: u8, x: u32, y: u32) -> Result<Option<Vec<u8>>, String> {
        let id = tile_id(z, x, y);
        let mut dir: std::sync::Arc<Vec<Entry>> = std::sync::Arc::new(self.root.clone());
        for _depth in 0..4 {
            let Some(e) = find(&dir, id) else { return Ok(None) };
            if e.run_length > 0 {
                let raw = self.src.read(self.header.data_offset + e.offset, e.length)?;
                return decompress(self.header.tile_compression, raw).map(Some);
            }
            let key = e.offset;
            let cached = self.leaves.lock().unwrap().get(&key).cloned();
            dir = match cached {
                Some(d) => d,
                None => {
                    let raw = self.src.read(self.header.leaf_offset + e.offset, e.length)?;
                    let d = std::sync::Arc::new(parse_directory(&decompress(self.header.internal_compression, raw)?)?);
                    let mut l = self.leaves.lock().unwrap();
                    if l.len() > 256 {
                        l.clear();
                    }
                    l.insert(key, d.clone());
                    d
                }
            };
        }
        Err("directory nesting too deep".into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tile_ids_match_spec_table() {
        assert_eq!(tile_id(0, 0, 0), 0);
        assert_eq!(tile_id(1, 0, 0), 1);
        assert_eq!(tile_id(1, 0, 1), 2);
        assert_eq!(tile_id(1, 1, 1), 3);
        assert_eq!(tile_id(1, 1, 0), 4);
        assert_eq!(tile_id(2, 0, 0), 5);
        assert_eq!(tile_id(12, 3423, 1763), 19_078_479);
    }

    #[test]
    fn directory_roundtrip() {
        // entries: id 5 off 0 len 10 run 1; id 6 off 10 len 5 run 2 (contiguous -> offset 0); id 9 leaf off 100 len 7 run 0
        let mut b = vec![3u8, 5, 1, 3, 1, 2, 0, 10, 5, 7, 1, 0, 101];
        assert_eq!(parse_directory(&b).unwrap(), vec![
            Entry { tile_id: 5, offset: 0, length: 10, run_length: 1 },
            Entry { tile_id: 6, offset: 10, length: 5, run_length: 2 },
            Entry { tile_id: 9, offset: 100, length: 7, run_length: 0 },
        ]);
        let d = parse_directory(&b).unwrap();
        assert_eq!(find(&d, 7).unwrap().tile_id, 6);
        assert!(find(&d, 4).is_none());
        assert_eq!(find(&d, 50).unwrap().run_length, 0); // falls into the leaf
        b.truncate(5);
        assert!(parse_directory(&b).is_err());
    }
}

#[cfg(test)]
mod fixture_tests {
    use super::*;

    /// Real archive from the PMTiles repository (Florence, vector, gzip). Skipped when not present.
    fn fixture() -> Option<std::path::PathBuf> {
        let p = std::path::PathBuf::from(std::env::var("PMTILES_FIXTURE").ok()?);
        p.exists().then_some(p)
    }

    #[test]
    fn reads_real_vector_archive() {
        let Some(p) = fixture() else { return };
        let pm = PmTiles::open(FileSource(p)).unwrap();
        let h = &pm.header;
        assert_eq!((h.tile_type, h.tile_compression), (1, 2));
        // the tile containing the centre of Florence at the max zoom
        let (lat, lon) = ((h.min_lat + h.max_lat) / 2.0, (h.min_lon + h.max_lon) / 2.0);
        let z = h.max_zoom;
        let n = (1u64 << z) as f64;
        let x = ((lon + 180.0) / 360.0 * n) as u32;
        let y = ((1.0 - (lat.to_radians().tan() + 1.0 / lat.to_radians().cos()).ln() / std::f64::consts::PI) / 2.0 * n) as u32;
        let t = pm.get(z, x, y).unwrap().expect("centre tile exists");
        let layers = crate::mvt::decode(&t).unwrap();
        let names: Vec<_> = layers.iter().map(|l| l.name.as_str()).collect();
        eprintln!("z{z} {x} {y}: {} bytes, layers {names:?}", t.len());
        assert!(layers.iter().map(|l| l.features.len()).sum::<usize>() > 10);
        assert!(pm.get(z, 0, 0).unwrap().is_none());
    }
}
