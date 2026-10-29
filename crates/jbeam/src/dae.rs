//! Collada (.dae) reader for BeamNG vehicle meshes: just what flexbodies need — the scene's named nodes with
//! their transform and geometry, and that geometry's triangles (positions, normals, first UV set) per material.
//! The file is scanned as text; only geometries that are asked for are parsed (a car .dae is ~100 MB).

use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct SubMesh {
    /// material symbol with the "-material" suffix removed (BeamNG `mapTo` name)
    pub material: String,
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub uvs: Vec<[f32; 2]>,
    pub indices: Vec<u32>,
}

#[derive(Clone, Debug)]
pub struct DaeNode {
    pub name: String,
    pub geometry: String,
    /// row-major 4x4 (Collada order), parent transforms already applied
    pub matrix: [f64; 16],
    /// material symbol -> target id
    pub materials: Vec<(String, String)>,
}

pub struct Dae {
    text: String,
    pub nodes: Vec<DaeNode>,
    geoms: HashMap<String, (usize, usize)>,
}

fn attr<'a>(tag: &'a str, name: &str) -> Option<&'a str> {
    let pat = format!(" {name}=\"");
    let i = tag.find(&pat)? + pat.len();
    let j = tag[i..].find('"')? + i;
    Some(&tag[i..j])
}

const IDENTITY: [f64; 16] = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.];

fn mat_mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let mut r = [0.0; 16];
    for i in 0..4 {
        for j in 0..4 {
            r[i * 4 + j] = (0..4).map(|k| a[i * 4 + k] * b[k * 4 + j]).sum();
        }
    }
    r
}

/// The opening tag text starting at `i` (which points at '<'), and the index after '>'.
fn tag_at(t: &str, i: usize) -> (&str, usize) {
    let e = t[i..].find('>').map_or(t.len(), |e| i + e + 1);
    (&t[i..e], e)
}

impl Dae {
    pub fn parse(text: String) -> Result<Dae, String> {
        if !text.contains("<COLLADA") {
            return Err("not a Collada file".into());
        }
        // geometry index
        let mut geoms = HashMap::new();
        let mut pos = 0;
        while let Some(i) = text[pos..].find("<geometry ") {
            let s = pos + i;
            let (tag, after) = tag_at(&text, s);
            let e = text[after..].find("</geometry>").map_or(text.len(), |e| after + e);
            if let Some(id) = attr(tag, "id") {
                geoms.insert(id.to_string(), (s, e));
            }
            pos = e;
        }
        // scene nodes (nested nodes multiply their transforms)
        let mut nodes = Vec::new();
        if let Some(vs) = text.find("<library_visual_scenes") {
            let ve = text[vs..].find("</library_visual_scenes>").map_or(text.len(), |e| vs + e);
            let seg = &text[vs..ve];
            let mut stack: Vec<(String, [f64; 16], bool)> = Vec::new(); // name, world matrix, matrix seen
            let mut p = 0;
            while let Some(i) = seg[p..].find('<') {
                let s = p + i;
                let (tag, after) = tag_at(seg, s);
                p = after;
                if tag.starts_with("<node") {
                    let parent = stack.last().map_or(IDENTITY, |n| n.1);
                    let name = attr(tag, "name").or(attr(tag, "id")).unwrap_or("").to_string();
                    let self_closing = tag.ends_with("/>");
                    if !self_closing {
                        stack.push((name, parent, false));
                    }
                } else if tag.starts_with("</node") {
                    stack.pop();
                } else if tag.starts_with("<matrix") {
                    let e = seg[after..].find("</matrix>").map_or(after, |e| after + e);
                    let v: Vec<f64> = seg[after..e].split_ascii_whitespace().filter_map(|x| x.parse().ok()).collect();
                    if let (Some(top), true) = (stack.last_mut(), v.len() == 16) {
                        if !top.2 {
                            let mut m = [0.0; 16];
                            m.copy_from_slice(&v);
                            top.1 = mat_mul(&top.1, &m);
                            top.2 = true;
                        }
                    }
                    p = e;
                } else if tag.starts_with("<instance_geometry") {
                    let url = attr(tag, "url").unwrap_or("").trim_start_matches('#').to_string();
                    let end = seg[after..].find("</instance_geometry>").map_or(after, |e| after + e);
                    let mut materials = Vec::new();
                    let mut q = after;
                    while let Some(k) = seg[q..end].find("<instance_material") {
                        let (mt, a2) = tag_at(seg, q + k);
                        if let (Some(sym), Some(tg)) = (attr(mt, "symbol"), attr(mt, "target")) {
                            materials.push((sym.to_string(), tg.trim_start_matches('#').to_string()));
                        }
                        q = a2;
                    }
                    if let Some(top) = stack.last() {
                        nodes.push(DaeNode { name: top.0.clone(), geometry: url, matrix: top.1, materials });
                    }
                    p = end;
                }
            }
        }
        Ok(Dae { text, nodes, geoms })
    }

    pub fn node(&self, name: &str) -> Option<&DaeNode> {
        self.nodes.iter().find(|n| n.name == name)
    }

    /// Triangles of a scene node, transformed by its matrix, one submesh per material.
    pub fn mesh(&self, node_name: &str) -> Result<Vec<SubMesh>, String> {
        let node = self.node(node_name).ok_or_else(|| format!("mesh '{node_name}' not in the .dae"))?;
        let &(s, e) = self.geoms.get(&node.geometry).ok_or_else(|| format!("geometry '{}' missing", node.geometry))?;
        let g = &self.text[s..e];
        // sources
        let mut sources: HashMap<&str, (Vec<f32>, usize)> = HashMap::new();
        let mut p = 0;
        while let Some(i) = g[p..].find("<source ") {
            let st = p + i;
            let (tag, after) = tag_at(g, st);
            let end = g[after..].find("</source>").map_or(g.len(), |e| after + e);
            let body = &g[after..end];
            if let (Some(id), Some(fa)) = (attr(tag, "id"), body.find("<float_array")) {
                let (_, fa_after) = tag_at(body, fa);
                let fe = body[fa_after..].find("</float_array>").map_or(body.len(), |e| fa_after + e);
                let vals: Vec<f32> = body[fa_after..fe].split_ascii_whitespace().filter_map(|x| x.parse().ok()).collect();
                let stride = body.find("<accessor").map(|a| tag_at(body, a).0).and_then(|t| attr(t, "stride")).and_then(|x| x.parse().ok()).unwrap_or(3);
                sources.insert(id, (vals, stride));
            }
            p = end;
        }
        // <vertices id> -> POSITION source
        let mut vert_alias: HashMap<&str, &str> = HashMap::new();
        if let Some(vi) = g.find("<vertices ") {
            let (tag, after) = tag_at(g, vi);
            let ve = g[after..].find("</vertices>").map_or(g.len(), |e| after + e);
            if let Some(ii) = g[after..ve].find("semantic=\"POSITION\"") {
                let ts = g[..after + ii].rfind('<').unwrap_or(after);
                let (itag, _) = tag_at(g, ts);
                if let (Some(id), Some(src)) = (attr(tag, "id"), attr(itag, "source")) {
                    vert_alias.insert(id, src.trim_start_matches('#'));
                }
            }
        }
        let m = node.matrix;
        let xf = |v: &[f32]| -> [f32; 3] {
            let (x, y, z) = (v[0] as f64, v[1] as f64, v[2] as f64);
            [(m[0] * x + m[1] * y + m[2] * z + m[3]) as f32, (m[4] * x + m[5] * y + m[6] * z + m[7]) as f32, (m[8] * x + m[9] * y + m[10] * z + m[11]) as f32]
        };
        let xn = |v: &[f32]| -> [f32; 3] {
            let (x, y, z) = (v[0] as f64, v[1] as f64, v[2] as f64);
            let r = [m[0] * x + m[1] * y + m[2] * z, m[4] * x + m[5] * y + m[6] * z, m[8] * x + m[9] * y + m[10] * z];
            let l = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt().max(1e-12);
            [(r[0] / l) as f32, (r[1] / l) as f32, (r[2] / l) as f32]
        };
        let mut out: Vec<SubMesh> = Vec::new();
        let mut p = 0;
        loop {
            let tri = g[p..].find("<triangles ");
            let poly = g[p..].find("<polylist ");
            let (i, is_poly) = match (tri, poly) {
                (Some(a), Some(b)) => if a < b { (a, false) } else { (b, true) },
                (Some(a), None) => (a, false),
                (None, Some(b)) => (b, true),
                (None, None) => break,
            };
            let st = p + i;
            let (tag, after) = tag_at(g, st);
            let close = if is_poly { "</polylist>" } else { "</triangles>" };
            let end = g[after..].find(close).map_or(g.len(), |e| after + e);
            p = end;
            let body = &g[after..end];
            let material = attr(tag, "material").unwrap_or("").trim_end_matches("-material").to_string();
            // inputs
            let (mut off_v, mut off_n, mut off_t) = (None, None, None);
            let (mut src_v, mut src_n, mut src_t) = ("", "", "");
            let mut stride = 0usize;
            let mut q = 0;
            while let Some(k) = body[q..].find("<input ") {
                let (it, a2) = tag_at(body, q + k);
                q = a2;
                let off: usize = attr(it, "offset").and_then(|x| x.parse().ok()).unwrap_or(0);
                stride = stride.max(off + 1);
                let src = attr(it, "source").unwrap_or("").trim_start_matches('#');
                match attr(it, "semantic") {
                    Some("VERTEX") => {
                        off_v = Some(off);
                        src_v = vert_alias.get(src).copied().unwrap_or(src);
                    }
                    Some("NORMAL") => {
                        off_n = Some(off);
                        src_n = src;
                    }
                    Some("TEXCOORD") if off_t.is_none() || attr(it, "set") == Some("0") => {
                        off_t = Some(off);
                        src_t = src;
                    }
                    _ => {}
                }
            }
            let Some(ov) = off_v else { continue };
            let Some((pos, ps)) = sources.get(src_v) else { continue };
            let nrm = off_n.and_then(|_| sources.get(src_n));
            let uv = off_t.and_then(|_| sources.get(src_t));
            let pi = match body.find("<p>") {
                Some(k) => {
                    let e = body[k..].find("</p>").map_or(body.len(), |e| k + e);
                    body[k + 3..e].split_ascii_whitespace().filter_map(|x| x.parse::<u32>().ok()).collect::<Vec<u32>>()
                }
                None => continue,
            };
            let vcount: Option<Vec<usize>> = if is_poly {
                body.find("<vcount>").map(|k| {
                    let e = body[k..].find("</vcount>").map_or(body.len(), |e| k + e);
                    body[k + 8..e].split_ascii_whitespace().filter_map(|x| x.parse().ok()).collect()
                })
            } else {
                None
            };
            let sm_idx = match out.iter().position(|s| s.material == material) {
                Some(k) => k,
                None => {
                    out.push(SubMesh { material: material.clone(), positions: Vec::new(), normals: Vec::new(), uvs: Vec::new(), indices: Vec::new() });
                    out.len() - 1
                }
            };
            let sm = &mut out[sm_idx];
            let mut dedup: HashMap<(u32, u32, u32), u32> = HashMap::new();
            let ncorners = pi.len() / stride.max(1);
            let mut corner = |c: usize, sm: &mut SubMesh| -> Option<u32> {
                let base = c * stride;
                let vi = *pi.get(base + ov)?;
                let ni = off_n.map_or(u32::MAX, |o| pi.get(base + o).copied().unwrap_or(u32::MAX));
                let ti = off_t.map_or(u32::MAX, |o| pi.get(base + o).copied().unwrap_or(u32::MAX));
                if let Some(&k) = dedup.get(&(vi, ni, ti)) {
                    return Some(k);
                }
                let pv = pos.get(vi as usize * ps..vi as usize * ps + 3)?;
                sm.positions.push(xf(pv));
                sm.normals.push(match nrm {
                    Some((n, nstr)) => n.get(ni as usize * nstr..ni as usize * nstr + 3).map(xn).unwrap_or([0.0, 0.0, 1.0]),
                    None => [0.0, 0.0, 1.0],
                });
                sm.uvs.push(match uv {
                    Some((t, tst)) => t.get(ti as usize * tst..ti as usize * tst + 2).map(|u| [u[0], 1.0 - u[1]]).unwrap_or([0.0, 0.0]),
                    None => [0.0, 0.0],
                });
                let k = (sm.positions.len() - 1) as u32;
                dedup.insert((vi, ni, ti), k);
                Some(k)
            };
            match vcount {
                Some(vc) => {
                    let mut c = 0;
                    for n in vc {
                        let ids: Vec<u32> = (0..n).filter_map(|k| corner(c + k, sm)).collect();
                        for k in 1..ids.len().saturating_sub(1) {
                            sm.indices.extend_from_slice(&[ids[0], ids[k], ids[k + 1]]);
                        }
                        c += n;
                    }
                }
                None => {
                    for t in 0..ncorners / 3 {
                        let ids: Vec<u32> = (0..3).filter_map(|k| corner(t * 3 + k, sm)).collect();
                        if ids.len() == 3 {
                            sm.indices.extend_from_slice(&ids);
                        }
                    }
                }
            }
        }
        out.retain(|s| !s.indices.is_empty());
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r##"<?xml version="1.0"?>
<COLLADA version="1.4.1"><library_geometries>
<geometry id="quad-mesh" name="quad"><mesh>
<source id="quad-pos"><float_array id="a" count="12">0 0 0 1 0 0 1 1 0 0 1 0</float_array><technique_common><accessor source="#a" count="4" stride="3"/></technique_common></source>
<source id="quad-nrm"><float_array id="b" count="3">0 0 1</float_array><technique_common><accessor source="#b" count="1" stride="3"/></technique_common></source>
<source id="quad-uv"><float_array id="c" count="8">0 0 1 0 1 1 0 1</float_array><technique_common><accessor source="#c" count="4" stride="2"/></technique_common></source>
<vertices id="quad-verts"><input semantic="POSITION" source="#quad-pos"/></vertices>
<triangles material="paint-material" count="2"><input semantic="VERTEX" source="#quad-verts" offset="0"/><input semantic="NORMAL" source="#quad-nrm" offset="1"/><input semantic="TEXCOORD" source="#quad-uv" offset="2" set="0"/>
<p>0 0 0 1 0 1 2 0 2 0 0 0 2 0 2 3 0 3</p></triangles>
<polylist material="glass-material" count="1"><input semantic="VERTEX" source="#quad-verts" offset="0"/><vcount>4</vcount><p>0 1 2 3</p></polylist>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene">
<node id="parent" name="parent"><matrix sid="transform">1 0 0 10 0 1 0 0 0 0 1 0 0 0 0 1</matrix>
  <node id="body" name="body_mesh"><matrix sid="transform">1 0 0 0 0 1 0 0 0 0 1 5 0 0 0 1</matrix>
    <instance_geometry url="#quad-mesh" name="body_mesh"><bind_material><technique_common>
      <instance_material symbol="paint-material" target="#paint-material"/></technique_common></bind_material></instance_geometry>
  </node>
</node>
</visual_scene></library_visual_scenes></COLLADA>"##;

    #[test]
    fn reads_nodes_transforms_and_triangles() {
        let d = Dae::parse(SAMPLE.to_string()).unwrap();
        assert_eq!(d.nodes.len(), 1);
        let n = d.node("body_mesh").unwrap();
        assert_eq!(n.materials[0].0, "paint-material");
        let m = d.mesh("body_mesh").unwrap();
        assert_eq!(m.len(), 2);
        let paint = &m[0];
        assert_eq!(paint.material, "paint");
        assert_eq!(paint.indices.len(), 6);
        assert_eq!(paint.positions.len(), 4); // shared corners de-duplicated
        // parent (x+10) and child (z+5) transforms both applied
        assert_eq!(paint.positions[0], [10.0, 0.0, 5.0]);
        assert_eq!(paint.normals[2], [0.0, 0.0, 1.0]);
        assert_eq!(paint.uvs[2], [1.0, 0.0]); // v flipped to top-left origin
        assert_eq!(m[1].material, "glass");
        assert_eq!(m[1].indices.len(), 6); // quad polygon fanned into two triangles
        assert!(d.mesh("nope").is_err());
    }
}
