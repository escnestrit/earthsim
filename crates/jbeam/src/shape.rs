//! BeamNG binary shapes (`.cdae` and `.dts` version 31): an int32 version, an int32 header size, a MessagePack
//! header map (`compression`, `bodysize`, `objectNames`), then the body — Zstandard-compressed when
//! `compression` is true — which is a sequence of MessagePack values in Torque3D `TSShape` order.
//! Spec: https://documentation.beamng.com/modding/file_formats/cdae/
//! Only what flexbodies need is decoded: objects (named meshes), node transforms, the highest-detail mesh of
//! every object (positions, normals, first UV set, triangles per material) and the material names.

use crate::dae::SubMesh;
use std::collections::HashMap;

/// Minimal MessagePack reader over a byte slice.
pub struct Mp<'a> {
    pub b: &'a [u8],
    pub p: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Val {
    Nil,
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
    Bin(usize, usize), // offset, length into the buffer
    Arr(Vec<Val>),
    Map(Vec<(Val, Val)>),
}

impl Val {
    pub fn as_i64(&self) -> Option<i64> {
        match self {
            Val::Int(i) => Some(*i),
            Val::Float(f) => Some(*f as i64),
            _ => None,
        }
    }
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Val::Int(i) => Some(*i as f64),
            Val::Float(f) => Some(*f),
            _ => None,
        }
    }
}

impl<'a> Mp<'a> {
    pub fn new(b: &'a [u8]) -> Mp<'a> {
        Mp { b, p: 0 }
    }
    fn take(&mut self, n: usize) -> Result<&'a [u8], String> {
        let e = self.p.checked_add(n).filter(|e| *e <= self.b.len()).ok_or("msgpack: truncated")?;
        let s = &self.b[self.p..e];
        self.p = e;
        Ok(s)
    }
    fn be(&mut self, n: usize) -> Result<u64, String> {
        Ok(self.take(n)?.iter().fold(0u64, |a, &x| (a << 8) | x as u64))
    }
    pub fn val(&mut self) -> Result<Val, String> {
        let t = *self.take(1)?.first().unwrap();
        Ok(match t {
            0x00..=0x7f => Val::Int(t as i64),
            0x80..=0x8f => self.map((t & 15) as usize)?,
            0x90..=0x9f => self.arr((t & 15) as usize)?,
            0xa0..=0xbf => self.str((t & 31) as usize)?,
            0xc0 => Val::Nil,
            0xc2 => Val::Bool(false),
            0xc3 => Val::Bool(true),
            0xc4 | 0xc5 | 0xc6 => {
                let n = self.be([1, 2, 4][(t - 0xc4) as usize])? as usize;
                let s = self.p;
                self.take(n)?;
                Val::Bin(s, n)
            }
            0xca => Val::Float(f32::from_be_bytes(self.take(4)?.try_into().unwrap()) as f64),
            0xcb => Val::Float(f64::from_be_bytes(self.take(8)?.try_into().unwrap())),
            0xcc..=0xcf => Val::Int(self.be(1 << (t - 0xcc))? as i64),
            0xd0 => Val::Int(self.take(1)?[0] as i8 as i64),
            0xd1 => Val::Int(i16::from_be_bytes(self.take(2)?.try_into().unwrap()) as i64),
            0xd2 => Val::Int(i32::from_be_bytes(self.take(4)?.try_into().unwrap()) as i64),
            0xd3 => Val::Int(i64::from_be_bytes(self.take(8)?.try_into().unwrap())),
            0xd9 | 0xda | 0xdb => {
                let n = self.be([1, 2, 4][(t - 0xd9) as usize])? as usize;
                self.str(n)?
            }
            0xdc | 0xdd => {
                let n = self.be(if t == 0xdc { 2 } else { 4 })? as usize;
                self.arr(n)?
            }
            0xde | 0xdf => {
                let n = self.be(if t == 0xde { 2 } else { 4 })? as usize;
                self.map(n)?
            }
            0xe0..=0xff => Val::Int(t as i8 as i64),
            _ => return Err(format!("msgpack: unsupported type 0x{t:02x} at {}", self.p - 1)),
        })
    }
    fn str(&mut self, n: usize) -> Result<Val, String> {
        Ok(Val::Str(String::from_utf8_lossy(self.take(n)?).into_owned()))
    }
    fn arr(&mut self, n: usize) -> Result<Val, String> {
        let mut a = Vec::with_capacity(n.min(1 << 16));
        for _ in 0..n {
            a.push(self.val()?);
        }
        Ok(Val::Arr(a))
    }
    fn map(&mut self, n: usize) -> Result<Val, String> {
        let mut a = Vec::with_capacity(n.min(1 << 16));
        for _ in 0..n {
            let k = self.val()?;
            let v = self.val()?;
            a.push((k, v));
        }
        Ok(Val::Map(a))
    }
    pub fn int(&mut self) -> Result<i64, String> {
        self.val()?.as_i64().ok_or_else(|| format!("msgpack: expected int at {}", self.p))
    }
    pub fn float(&mut self) -> Result<f64, String> {
        self.val()?.as_f64().ok_or_else(|| format!("msgpack: expected number at {}", self.p))
    }
    /// BeamNG `pack_vector`: element count, element size, then a bin of count*size bytes.
    pub fn vector(&mut self) -> Result<(usize, usize, &'a [u8]), String> {
        let n = self.int()? as usize;
        let es = self.int()? as usize;
        match self.val()? {
            Val::Bin(o, l) => {
                if n * es != l && n != 0 {
                    return Err(format!("pack_vector: {n} x {es} != {l} bytes"));
                }
                Ok((n, es, &self.b[o..o + l]))
            }
            Val::Nil if n == 0 => Ok((0, es, &[])),
            v => Err(format!("pack_vector: expected bin, got {v:?}")),
        }
    }
}

fn i32s(b: &[u8]) -> Vec<i32> {
    b.chunks_exact(4).map(|c| i32::from_le_bytes(c.try_into().unwrap())).collect()
}
fn f32s(b: &[u8]) -> Vec<f32> {
    b.chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect()
}

/// A decoded shape: object (mesh) names -> submeshes in shape space.
pub struct Shape {
    pub objects: HashMap<String, Vec<SubMesh>>,
    pub materials: Vec<String>,
}

impl Shape {
    pub fn has(&self, name: &str) -> bool {
        self.objects.contains_key(name)
    }
}

/// Split a shape file into its header map and (decompressed) body.
pub fn split(file: &[u8]) -> Result<(Val, Vec<u8>), String> {
    if file.len() < 8 {
        return Err("shape: file too short".into());
    }
    let ver = u32::from_le_bytes(file[0..4].try_into().unwrap());
    if ver & 0xff != 31 {
        return Err(format!("shape: version {} is not supported (31 is)", ver & 0xff));
    }
    let hs = u32::from_le_bytes(file[4..8].try_into().unwrap()) as usize;
    let hend = 8usize.checked_add(hs).filter(|e| *e <= file.len()).ok_or("shape: bad header size")?;
    let header = Mp::new(&file[8..hend]).val()?;
    let compressed = matches!(header_get(&header, "compression"), Some(Val::Bool(true)));
    let body = &file[hend..];
    let body = if compressed { zstd(body)? } else { body.to_vec() };
    Ok((header, body))
}

pub fn header_get<'v>(h: &'v Val, key: &str) -> Option<&'v Val> {
    match h {
        Val::Map(m) => m.iter().find(|(k, _)| matches!(k, Val::Str(s) if s == key)).map(|(_, v)| v),
        _ => None,
    }
}

fn zstd(b: &[u8]) -> Result<Vec<u8>, String> {
    use std::io::Read;
    let mut dec = ruzstd::decoding::StreamingDecoder::new(b).map_err(|e| format!("zstd: {e}"))?;
    let mut out = Vec::new();
    dec.read_to_end(&mut out).map_err(|e| format!("zstd: {e}"))?;
    Ok(out)
}

#[derive(Clone, Copy, Default)]
struct Node {
    parent: i32,
}

/// Quat16 (Torque): 4 x i16 scaled by 32767.
fn quat16(b: &[u8]) -> [f64; 4] {
    let q: Vec<f64> = b.chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]]) as f64 / 32767.0).collect();
    [q[0], q[1], q[2], q[3]]
}

fn qrot(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    // Torque stores (x, y, z, w) with the conjugate convention: rotate by the inverse
    let (x, y, z, w) = (-q[0], -q[1], -q[2], q[3]);
    let (vx, vy, vz) = (v[0], v[1], v[2]);
    let (tx, ty, tz) = (2.0 * (y * vz - z * vy), 2.0 * (z * vx - x * vz), 2.0 * (x * vy - y * vx));
    [vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)]
}

pub fn parse(file: &[u8]) -> Result<Shape, String> {
    let (header, body) = split(file)?;
    let object_names: Vec<String> = match header_get(&header, "objectNames") {
        Some(Val::Arr(a)) => a.iter().map(|v| if let Val::Str(s) = v { s.clone() } else { String::new() }).collect(),
        _ => Vec::new(),
    };
    let mut m = Mp::new(&body);
    // shape info
    let _smallest_size = m.float()?;
    let _smallest_dl = m.int()?;
    let _radius = m.float()?;
    let _tube = m.float()?;
    // center (Point3F) and bounds (Box3F): arrays of floats, or loose floats in other writers
    for n in [3, 6] {
        if let Val::Float(_) | Val::Int(_) = m.val()? {
            for _ in 1..n {
                m.float()?;
            }
        }
    }
    let names_order = [
        "nodes", "objects", "subShapeFirstNode", "subShapeFirstObject", "subShapeNumNodes", "subShapeNumObjects", "defaultRotations", "defaultTranslations", "nodeRotations", "nodeTranslations",
        "nodeUniformScales", "nodeAlignedScales", "nodeArbitraryScaleFactors", "nodeArbitraryScaleRots", "groundTranslations", "groundRotations", "objectStates", "triggers", "details",
    ];
    let mut vecs: HashMap<&str, (usize, usize, &[u8])> = HashMap::new();
    for n in names_order {
        vecs.insert(n, m.vector()?);
    }
    let names_n = m.int()? as usize;
    let mut names = Vec::with_capacity(names_n);
    for _ in 0..names_n {
        names.push(match m.val()? {
            Val::Str(s) => s,
            v => format!("{v:?}"),
        });
    }
    // nodes: name, parent, firstObject, firstChild, nextSibling
    let (nn, nes, nb) = vecs["nodes"];
    let nodes: Vec<Node> = (0..nn).map(|i| {
        let v = i32s(&nb[i * nes..i * nes + nes.min(20)]);
        Node { parent: v[1] }
    }).collect();
    let (_, res, rb) = vecs["defaultRotations"];
    let (_, tes, tb) = vecs["defaultTranslations"];
    // world transform of each node (rotations/translations are relative to the parent)
    let mut world: Vec<Option<([f64; 4], [f64; 3])>> = vec![None; nodes.len()];
    fn resolve(i: usize, nodes: &[Node], rb: &[u8], res: usize, tb: &[u8], tes: usize, world: &mut Vec<Option<([f64; 4], [f64; 3])>>) -> ([f64; 4], [f64; 3]) {
        if let Some(w) = world[i] {
            return w;
        }
        let q = if res >= 8 && rb.len() >= (i + 1) * res { quat16(&rb[i * res..i * res + 8]) } else { [0.0, 0.0, 0.0, 1.0] };
        let t = if tes >= 12 && tb.len() >= (i + 1) * tes {
            let f = f32s(&tb[i * tes..i * tes + 12]);
            [f[0] as f64, f[1] as f64, f[2] as f64]
        } else {
            [0.0; 3]
        };
        let w = match nodes[i].parent {
            p if p >= 0 && (p as usize) < nodes.len() && p as usize != i => {
                let (pq, pt) = resolve(p as usize, nodes, rb, res, tb, tes, world);
                // compose: world = parent * local; rotations are kept as a list applied in order
                let r = qrot(pq, t);
                let qm = qmul(pq, q);
                (qm, [pt[0] + r[0], pt[1] + r[1], pt[2] + r[2]])
            }
            _ => (q, t),
        };
        world[i] = Some(w);
        w
    }
    for i in 0..nodes.len() {
        resolve(i, &nodes, rb, res, tb, tes, &mut world);
    }
    // objects: name, numMeshes, startMeshIndex, nodeIndex, nextSibling, firstDecal
    let (on, oes, ob) = vecs["objects"];
    let objects: Vec<[i32; 4]> = (0..on).map(|i| {
        let v = i32s(&ob[i * oes..i * oes + oes.min(24)]);
        [v[0], v[1], v[2], v[3]]
    }).collect();
    // meshes
    let mesh_count = m.int()? as usize;
    struct RawMesh {
        verts: Vec<f32>,
        tverts: Vec<f32>,
        norms: Vec<f32>,
        prims: Vec<i32>,
        prim_es: usize,
        indices: Vec<u32>,
        parent: i64,
    }
    let mut meshes: Vec<Option<RawMesh>> = Vec::with_capacity(mesh_count);
    for _ in 0..mesh_count {
        let ty = m.int()?;
        if ty == 4 {
            meshes.push(None);
            continue;
        }
        let _frames = m.int()?;
        let _matframes = m.int()?;
        let parent = m.int()?;
        // mBounds, mCenter (arrays or loose floats), mRadius
        for n in [6, 3] {
            if let Val::Float(_) | Val::Int(_) = m.val()? {
                for _ in 1..n {
                    m.float()?;
                }
            }
        }
        m.float()?;
        let mut v = Vec::new();
        for _ in 0..9 {
            v.push(m.vector()?);
        }
        let _verts_per_frame = m.int()?;
        let _flags = m.int()?;
        if ty == 1 {
            // skin mesh extras
            if parent < 0 {
                m.vector()?;
                m.vector()?;
            }
            m.vector()?;
            if parent < 0 {
                for _ in 0..4 {
                    m.vector()?;
                }
            }
        }
        let (_, _, vb) = v[0];
        let (_, _, tvb) = v[1];
        let (_, _, nb) = v[4];
        let (_, pes, pb) = v[6];
        let (_, ies, ib) = v[7];
        let indices: Vec<u32> = match ies {
            2 => ib.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]]) as u32).collect(),
            _ => ib.chunks_exact(4).map(|c| u32::from_le_bytes(c.try_into().unwrap())).collect(),
        };
        let prims = match pes {
            // older layout: u16 start, u16 count, u32 material
            8 => pb.chunks_exact(8).flat_map(|c| [u16::from_le_bytes([c[0], c[1]]) as i32, u16::from_le_bytes([c[2], c[3]]) as i32, i32::from_le_bytes(c[4..8].try_into().unwrap())]).collect(),
            _ => i32s(pb),
        };
        meshes.push(Some(RawMesh { verts: f32s(vb), tverts: f32s(tvb), norms: f32s(nb), prims, prim_es: 12, indices, parent }));
    }
    // sequences (skipped) then materials
    let mut materials = Vec::new();
    if let Ok(nseq) = m.int() {
        let mut ok = nseq == 0;
        if nseq > 0 {
            // sequences carry integer sets of unknown encoding; without them the material list is unreachable
            ok = false;
        }
        if ok {
            if let Ok(nmat) = m.int() {
                for _ in 0..nmat {
                    match m.val() {
                        Ok(Val::Str(s)) => materials.push(s),
                        _ => break,
                    }
                    for _ in 0..4 {
                        let _ = m.int();
                    }
                    let _ = m.float();
                    let _ = m.float();
                }
            }
        }
    }
    let mut out: HashMap<String, Vec<SubMesh>> = HashMap::new();
    for (oi, o) in objects.iter().enumerate() {
        let name = names.get(o[0] as usize).cloned().or_else(|| object_names.get(oi).cloned()).unwrap_or_default();
        let (num, start, node) = (o[1], o[2], o[3]);
        if num <= 0 || start < 0 {
            continue;
        }
        let Some(Some(raw)) = meshes.get(start as usize) else { continue };
        // meshes sharing data point at a parent mesh
        let src = if raw.parent >= 0 { meshes.get(raw.parent as usize).and_then(|x| x.as_ref()).unwrap_or(raw) } else { raw };
        let (q, t) = if node >= 0 && (node as usize) < world.len() { world[node as usize].unwrap_or(([0.0, 0.0, 0.0, 1.0], [0.0; 3])) } else { ([0.0, 0.0, 0.0, 1.0], [0.0; 3]) };
        let nv = src.verts.len() / 3;
        let positions: Vec<[f32; 3]> = (0..nv).map(|i| {
            let p = qrot(q, [src.verts[i * 3] as f64, src.verts[i * 3 + 1] as f64, src.verts[i * 3 + 2] as f64]);
            [(p[0] + t[0]) as f32, (p[1] + t[1]) as f32, (p[2] + t[2]) as f32]
        }).collect();
        let normals: Vec<[f32; 3]> = (0..nv).map(|i| {
            if src.norms.len() >= (i + 1) * 3 {
                let n = qrot(q, [src.norms[i * 3] as f64, src.norms[i * 3 + 1] as f64, src.norms[i * 3 + 2] as f64]);
                [n[0] as f32, n[1] as f32, n[2] as f32]
            } else {
                [0.0, 0.0, 1.0]
            }
        }).collect();
        let uvs: Vec<[f32; 2]> = (0..nv).map(|i| if src.tverts.len() >= (i + 1) * 2 { [src.tverts[i * 2], src.tverts[i * 2 + 1]] } else { [0.0, 0.0] }).collect();
        // primitives: start, count, material|flags
        let mut by_mat: HashMap<usize, Vec<u32>> = HashMap::new();
        for p in src.prims.chunks_exact(src.prim_es / 4) {
            let (st, cnt, mi) = (p[0].max(0) as usize, p[1].max(0) as usize, p[2] as u32);
            let mat = (mi & 0x0fff_ffff) as usize;
            let kind = mi & 0xc000_0000;
            let idx = &src.indices[st.min(src.indices.len())..(st + cnt).min(src.indices.len())];
            let e = by_mat.entry(mat).or_default();
            match kind {
                0x4000_0000 => {
                    // strip
                    for k in 2..idx.len() {
                        let (a, b, c) = if k % 2 == 0 { (idx[k - 2], idx[k - 1], idx[k]) } else { (idx[k - 1], idx[k - 2], idx[k]) };
                        if a != b && b != c && a != c {
                            e.extend_from_slice(&[a, b, c]);
                        }
                    }
                }
                0x8000_0000 => {
                    for k in 2..idx.len() {
                        e.extend_from_slice(&[idx[0], idx[k - 1], idx[k]]);
                    }
                }
                _ => e.extend_from_slice(&idx[..idx.len() / 3 * 3]),
            }
        }
        let mut subs = Vec::new();
        for (mat, mut ind) in by_mat {
            if ind.iter().any(|&i| i as usize >= nv) {
                continue;
            }
            // Torque winds triangles clockwise; flip to counter-clockwise like Collada
            for t in ind.chunks_exact_mut(3) {
                t.swap(1, 2);
            }
            subs.push(SubMesh { material: materials.get(mat).cloned().unwrap_or_else(|| format!("material{mat}")), positions: positions.clone(), normals: normals.clone(), uvs: uvs.clone(), indices: ind });
        }
        if !subs.is_empty() {
            out.insert(name, subs);
        }
    }
    Ok(Shape { objects: out, materials })
}

fn qmul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    // Torque quaternions combine in reverse order (row vectors)
    let (ax, ay, az, aw) = (a[0], a[1], a[2], a[3]);
    let (bx, by, bz, bw) = (b[0], b[1], b[2], b[3]);
    [bw * ax + bx * aw + by * az - bz * ay, bw * ay - bx * az + by * aw + bz * ax, bw * az + bx * ay - by * ax + bz * aw, bw * aw - bx * ax - by * ay - bz * az]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn msgpack_values() {
        let b = [0x93, 0x01, 0xa2, b'h', b'i', 0x81, 0xc3, 0xcb, 0x3f, 0xf8, 0, 0, 0, 0, 0, 0, 0xc4, 0x02, 9, 8, 0xd2, 0xff, 0xff, 0xff, 0xfe];
        let mut m = Mp::new(&b);
        match m.val().unwrap() {
            Val::Arr(a) => {
                assert_eq!(a[0], Val::Int(1));
                assert_eq!(a[1], Val::Str("hi".into()));
                assert_eq!(a[2], Val::Map(vec![(Val::Bool(true), Val::Float(1.5))]));
            }
            v => panic!("{v:?}"),
        }
        assert_eq!(m.val().unwrap(), Val::Bin(18, 2));
        assert_eq!(m.int().unwrap(), -2);
        assert!(m.val().is_err());
    }

    #[test]
    fn rejects_other_versions() {
        let mut f = vec![30u8, 0, 0, 0, 1, 0, 0, 0, 0x80];
        assert!(split(&f).is_err());
        f[0] = 31;
        let (h, body) = split(&f).unwrap();
        assert_eq!(h, Val::Map(vec![]));
        assert!(body.is_empty());
    }
}
