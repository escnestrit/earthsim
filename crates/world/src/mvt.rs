//! Mapbox Vector Tile (protobuf) decoder: layers, features, properties, geometry in tile units.

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Str(String),
    Num(f64),
    Int(i64),
    Bool(bool),
}

impl Value {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Num(n) => Some(*n),
            Value::Int(i) => Some(*i as f64),
            _ => None,
        }
    }
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(s) => Some(s),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GeomType {
    Unknown,
    Point,
    Line,
    Polygon,
}

#[derive(Clone, Debug)]
pub struct Feature {
    pub id: u64,
    pub kind: GeomType,
    pub props: Vec<(String, Value)>,
    /// MoveTo starts a new part. Points: each part is one point. Polygons: rings (outer rings have positive
    /// shoelace area in tile coordinates, y pointing down).
    pub parts: Vec<Vec<(i32, i32)>>,
}

impl Feature {
    pub fn prop(&self, k: &str) -> Option<&Value> {
        self.props.iter().find(|(kk, _)| kk == k).map(|(_, v)| v)
    }
}

#[derive(Clone, Debug)]
pub struct Layer {
    pub name: String,
    pub extent: u32,
    pub features: Vec<Feature>,
}

struct Pb<'a> {
    b: &'a [u8],
    p: usize,
}

impl<'a> Pb<'a> {
    fn varint(&mut self) -> Result<u64, String> {
        let mut v = 0u64;
        let mut s = 0;
        loop {
            let c = *self.b.get(self.p).ok_or("truncated protobuf")?;
            self.p += 1;
            v |= ((c & 0x7f) as u64) << s;
            if c & 0x80 == 0 {
                return Ok(v);
            }
            s += 7;
            if s > 63 {
                return Err("bad varint".into());
            }
        }
    }
    fn bytes(&mut self) -> Result<&'a [u8], String> {
        let n = self.varint()? as usize;
        let e = self.p.checked_add(n).filter(|e| *e <= self.b.len()).ok_or("truncated field")?;
        let s = &self.b[self.p..e];
        self.p = e;
        Ok(s)
    }
    fn fixed(&mut self, n: usize) -> Result<&'a [u8], String> {
        let e = self.p + n;
        if e > self.b.len() {
            return Err("truncated fixed field".into());
        }
        let s = &self.b[self.p..e];
        self.p = e;
        Ok(s)
    }
    /// Returns (field, wire type) or None at end.
    fn key(&mut self) -> Result<Option<(u64, u8)>, String> {
        if self.p >= self.b.len() {
            return Ok(None);
        }
        let k = self.varint()?;
        Ok(Some((k >> 3, (k & 7) as u8)))
    }
    fn skip(&mut self, wt: u8) -> Result<(), String> {
        match wt {
            0 => {
                self.varint()?;
            }
            1 => {
                self.fixed(8)?;
            }
            2 => {
                self.bytes()?;
            }
            5 => {
                self.fixed(4)?;
            }
            _ => return Err(format!("unsupported wire type {wt}")),
        }
        Ok(())
    }
}

fn packed(b: &[u8]) -> Result<Vec<u32>, String> {
    let mut p = Pb { b, p: 0 };
    let mut v = Vec::new();
    while p.p < b.len() {
        v.push(p.varint()? as u32);
    }
    Ok(v)
}

fn value(b: &[u8]) -> Result<Value, String> {
    let mut p = Pb { b, p: 0 };
    let mut out = Value::Bool(false);
    while let Some((f, wt)) = p.key()? {
        out = match (f, wt) {
            (1, 2) => Value::Str(String::from_utf8_lossy(p.bytes()?).into_owned()),
            (2, 5) => Value::Num(f32::from_le_bytes(p.fixed(4)?.try_into().unwrap()) as f64),
            (3, 1) => Value::Num(f64::from_le_bytes(p.fixed(8)?.try_into().unwrap())),
            (4, 0) | (5, 0) => Value::Int(p.varint()? as i64),
            (6, 0) => {
                let u = p.varint()?;
                Value::Int(((u >> 1) as i64) ^ -((u & 1) as i64))
            }
            (7, 0) => Value::Bool(p.varint()? != 0),
            (_, wt) => {
                p.skip(wt)?;
                continue;
            }
        };
    }
    Ok(out)
}

fn geometry(cmds: &[u32]) -> Vec<Vec<(i32, i32)>> {
    let mut parts: Vec<Vec<(i32, i32)>> = Vec::new();
    let (mut x, mut y) = (0i32, 0i32);
    let mut i = 0;
    let zz = |u: u32| ((u >> 1) as i32) ^ -((u & 1) as i32);
    while i < cmds.len() {
        let id = cmds[i] & 7;
        let count = (cmds[i] >> 3) as usize;
        i += 1;
        match id {
            1 | 2 => {
                for _ in 0..count {
                    if i + 1 >= cmds.len() {
                        return parts;
                    }
                    x = x.wrapping_add(zz(cmds[i]));
                    y = y.wrapping_add(zz(cmds[i + 1]));
                    i += 2;
                    if id == 1 {
                        parts.push(vec![(x, y)]);
                    } else if let Some(last) = parts.last_mut() {
                        last.push((x, y));
                    }
                }
            }
            7 => {
                if let Some(last) = parts.last_mut() {
                    if let Some(&f) = last.first() {
                        last.push(f);
                    }
                }
            }
            _ => return parts,
        }
    }
    parts
}

fn feature(b: &[u8], keys: &[String], vals: &[Value]) -> Result<Feature, String> {
    let mut p = Pb { b, p: 0 };
    let mut f = Feature { id: 0, kind: GeomType::Unknown, props: Vec::new(), parts: Vec::new() };
    let mut tags = Vec::new();
    while let Some((field, wt)) = p.key()? {
        match (field, wt) {
            (1, 0) => f.id = p.varint()?,
            (2, 2) => tags = packed(p.bytes()?)?,
            (3, 0) => {
                f.kind = match p.varint()? {
                    1 => GeomType::Point,
                    2 => GeomType::Line,
                    3 => GeomType::Polygon,
                    _ => GeomType::Unknown,
                }
            }
            (4, 2) => f.parts = geometry(&packed(p.bytes()?)?),
            (_, wt) => p.skip(wt)?,
        }
    }
    for kv in tags.chunks_exact(2) {
        if let (Some(k), Some(v)) = (keys.get(kv[0] as usize), vals.get(kv[1] as usize)) {
            f.props.push((k.clone(), v.clone()));
        }
    }
    Ok(f)
}

fn layer(b: &[u8]) -> Result<Layer, String> {
    let mut p = Pb { b, p: 0 };
    let mut name = String::new();
    let mut extent = 4096;
    let mut raw_features = Vec::new();
    let mut keys = Vec::new();
    let mut vals = Vec::new();
    while let Some((field, wt)) = p.key()? {
        match (field, wt) {
            (1, 2) => name = String::from_utf8_lossy(p.bytes()?).into_owned(),
            (2, 2) => raw_features.push(p.bytes()?),
            (3, 2) => keys.push(String::from_utf8_lossy(p.bytes()?).into_owned()),
            (4, 2) => vals.push(value(p.bytes()?)?),
            (5, 0) => extent = p.varint()? as u32,
            (_, wt) => p.skip(wt)?,
        }
    }
    let features = raw_features.into_iter().map(|f| feature(f, &keys, &vals)).collect::<Result<Vec<_>, _>>()?;
    Ok(Layer { name, extent, features })
}

pub fn decode(tile: &[u8]) -> Result<Vec<Layer>, String> {
    let mut p = Pb { b: tile, p: 0 };
    let mut out = Vec::new();
    while let Some((field, wt)) = p.key()? {
        if field == 3 && wt == 2 {
            out.push(layer(p.bytes()?)?);
        } else {
            p.skip(wt)?;
        }
    }
    Ok(out)
}

/// Shoelace area in tile units (positive = outer ring in MVT convention).
pub fn ring_area(r: &[(i32, i32)]) -> f64 {
    let mut a = 0.0;
    for w in r.windows(2) {
        a += w[0].0 as f64 * w[1].1 as f64 - w[1].0 as f64 * w[0].1 as f64;
    }
    a / 2.0
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    // minimal protobuf writer for tests
    pub fn vint(out: &mut Vec<u8>, mut v: u64) {
        loop {
            let b = (v & 0x7f) as u8;
            v >>= 7;
            if v == 0 {
                out.push(b);
                return;
            }
            out.push(b | 0x80);
        }
    }
    pub fn field_bytes(out: &mut Vec<u8>, f: u64, b: &[u8]) {
        vint(out, f << 3 | 2);
        vint(out, b.len() as u64);
        out.extend_from_slice(b);
    }
    pub fn zz(v: i32) -> u32 {
        ((v << 1) ^ (v >> 31)) as u32
    }
    pub fn packed_u32(v: &[u32]) -> Vec<u8> {
        let mut o = Vec::new();
        for &x in v {
            vint(&mut o, x as u64);
        }
        o
    }

    pub fn test_tile() -> Vec<u8> {
        // layer "building": one square polygon (0,0)-(10,10) with height=12.5 and class="house"
        let mut val_h = Vec::new();
        vint(&mut val_h, 3 << 3 | 1);
        val_h.extend_from_slice(&12.5f64.to_le_bytes());
        let mut val_c = Vec::new();
        field_bytes(&mut val_c, 1, b"house");
        let geom = [1 << 3 | 1, zz(0), zz(0), 3 << 3 | 2, zz(10), zz(0), zz(0), zz(10), zz(-10), zz(0), 1 << 3 | 7];
        let mut feat = Vec::new();
        vint(&mut feat, 1 << 3);
        vint(&mut feat, 42);
        field_bytes(&mut feat, 2, &packed_u32(&[0, 0, 1, 1]));
        vint(&mut feat, 3 << 3);
        vint(&mut feat, 3);
        field_bytes(&mut feat, 4, &packed_u32(&geom));
        let mut layer = Vec::new();
        field_bytes(&mut layer, 1, b"building");
        field_bytes(&mut layer, 2, &feat);
        field_bytes(&mut layer, 3, b"height");
        field_bytes(&mut layer, 3, b"class");
        field_bytes(&mut layer, 4, &val_h);
        field_bytes(&mut layer, 4, &val_c);
        vint(&mut layer, 5 << 3);
        vint(&mut layer, 4096);
        let mut tile = Vec::new();
        field_bytes(&mut tile, 3, &layer);
        tile
    }

    #[test]
    fn decodes_layer_feature_props_geometry() {
        let l = decode(&test_tile()).unwrap();
        assert_eq!(l.len(), 1);
        assert_eq!(l[0].name, "building");
        let f = &l[0].features[0];
        assert_eq!(f.id, 42);
        assert_eq!(f.kind, GeomType::Polygon);
        assert_eq!(f.prop("height").unwrap().as_f64(), Some(12.5));
        assert_eq!(f.prop("class").unwrap().as_str(), Some("house"));
        assert_eq!(f.parts[0], vec![(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]);
    }

    #[test]
    fn truncated_input_errors() {
        let t = test_tile();
        assert!(decode(&t[..t.len() - 3]).is_err());
    }
}
