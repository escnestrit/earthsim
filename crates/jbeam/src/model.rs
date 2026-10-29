//! Flat node/beam model of an assembled vehicle (still in JBeam units/axes; use `to_ours` for the engine frame).

use crate::assemble::Assembly;
use crate::table::{rows_merged, Row};
use scene::json::Json;
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct Node {
    pub name: String,
    pub part: usize,
    /// JBeam coordinates: +X left, +Y backwards, +Z up (metres), slot offsets applied
    pub pos: [f64; 3],
    pub mass: f64,
    pub friction: f64,
    pub collision: bool,
    pub self_collision: bool,
    pub fixed: bool,
    pub group: Vec<String>,
    pub material: String,
}

#[derive(Clone, Debug)]
pub struct Beam {
    pub a: usize,
    pub b: usize,
    pub part: usize,
    /// NORMAL, HYDRO, BOUNDED, LBEAM, PRESSURED, SUPPORT, ANISOTROPIC
    pub kind: String,
    pub row: Row,
}

impl Beam {
    pub fn spring(&self) -> f64 {
        self.row.num_or("beamSpring", 4_300_000.0)
    }
    pub fn damp(&self) -> f64 {
        self.row.num_or("beamDamp", 580.0)
    }
    /// Force at which the beam yields (N); `None` = effectively never (FLT_MAX).
    pub fn deform(&self) -> Option<f64> {
        limit(&self.row, "beamDeform", 220_000.0)
    }
    pub fn strength(&self) -> Option<f64> {
        limit(&self.row, "beamStrength", f64::MAX)
    }
}

fn limit(r: &Row, k: &str, d: f64) -> Option<f64> {
    match r.get(k) {
        Some(Json::Num(n)) => Some(*n).filter(|v| *v < 1e30),
        Some(Json::Str(s)) if s.contains("FLT_MAX") => None,
        _ => Some(d).filter(|v| *v < 1e30),
    }
}

/// JBeam (+X left, +Y back, +Z up) to the engine frame (+X forward, +Y left, +Z up).
pub fn to_ours(p: [f64; 3]) -> [f64; 3] {
    [-p[1], p[0], p[2]]
}

pub struct Model {
    pub asm: Assembly,
    pub nodes: Vec<Node>,
    pub beams: Vec<Beam>,
    pub node_index: HashMap<String, usize>,
    pub warnings: Vec<String>,
    pub skipped_beams: usize,
}

fn offset_of(r: &Row, key: &str) -> [f64; 3] {
    match r.get(key) {
        Some(j @ Json::Obj(_)) => {
            let g = |k: &str| j.get(k).and_then(|v| v.as_f64()).unwrap_or(0.0);
            [g("x"), g("y"), g("z")]
        }
        _ => [0.0; 3],
    }
}

pub fn build(asm: Assembly) -> Model {
    let mut nodes: Vec<Node> = Vec::new();
    let mut index: HashMap<String, usize> = HashMap::new();
    let mut warnings = Vec::new();
    let mut dup_nodes = 0usize;
    let node_rows = rows_merged(asm.parts.iter().enumerate().filter_map(|(i, p)| p.body.get("nodes").map(|s| (i, s))));
    for (pi, r) in node_rows {
        let part = &asm.parts[pi];
        {
            let Some(id) = r.str("id") else { continue };
            let (mv, off) = (offset_of(&r, "nodeMove"), offset_of(&r, "nodeOffset"));
            let raw = [r.num_or("posX", 0.0), r.num_or("posY", 0.0), r.num_or("posZ", 0.0)];
            let mirror = if raw[0] < 0.0 { -1.0 } else { 1.0 };
            let pos = [0, 1, 2].map(|i| {
                let o = if i == 0 { (part.offset[0] + off[0]) * mirror } else { part.offset[i] + off[i] };
                raw[i] + o + part.node_move[i] + mv[i]
            });
            if index.contains_key(id) {
                dup_nodes += 1; // shared nodes (glass/panels re-declaring body nodes) are normal in BeamNG mods
                continue;
            }
            index.insert(id.to_string(), nodes.len());
            nodes.push(Node {
                name: id.to_string(),
                part: pi,
                pos,
                mass: r.num_or("nodeWeight", 25.0),
                friction: r.num_or("frictionCoef", 1.0),
                collision: !matches!(r.get("collision"), Some(Json::Bool(false))),
                self_collision: r.flag("selfCollision"),
                fixed: r.flag("fixed"),
                group: r.strings("group"),
                material: r.str("nodeMaterial").unwrap_or("").to_string(),
            });
        }
    }
    let mut beams = Vec::new();
    let mut skipped = 0;
    let mut first_skips: Vec<String> = Vec::new();
    let beam_rows = rows_merged(asm.parts.iter().enumerate().filter_map(|(i, p)| p.body.get("beams").map(|s| (i, s))));
    for (pi, r) in beam_rows {
        let part = &asm.parts[pi];
        {
            let (Some(a), Some(b)) = (r.str("id1"), r.str("id2")) else { continue };
            match (index.get(a), index.get(b)) {
                (Some(&ia), Some(&ib)) if ia != ib => {
                    let kind = r.str("beamType").unwrap_or("|NORMAL").trim_start_matches('|').to_string();
                    beams.push(Beam { a: ia, b: ib, part: pi, kind, row: r });
                }
                _ => {
                    if !r.flag("optional") {
                        skipped += 1;
                        if first_skips.len() < 12 {
                            first_skips.push(format!("{}: beam {a}-{b} references a missing node", part.name));
                        }
                    }
                }
            }
        }
    }
    warnings.extend(first_skips);
    if dup_nodes > 0 {
        warnings.push(format!("{dup_nodes} node ids were declared by more than one part (first definition kept)"));
    }
    Model { asm, nodes, beams, node_index: index, warnings, skipped_beams: skipped }
}

impl Model {
    /// All rows of a table section across parts, with the index of the part they came from.
    pub fn table(&self, section: &str) -> Vec<(usize, Row)> {
        rows_merged(self.asm.parts.iter().enumerate().filter_map(|(i, p)| p.body.get(section).map(|s| (i, s))))
    }

    pub fn total_mass(&self) -> f64 {
        self.nodes.iter().map(|n| n.mass).sum()
    }

    /// Bounding box in the engine frame: (min, max).
    pub fn bounds(&self) -> ([f64; 3], [f64; 3]) {
        let mut lo = [f64::MAX; 3];
        let mut hi = [f64::MIN; 3];
        for n in &self.nodes {
            let p = to_ours(n.pos);
            for i in 0..3 {
                lo[i] = lo[i].min(p[i]);
                hi[i] = hi[i].max(p[i]);
            }
        }
        (lo, hi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assemble::{assemble, Library};

    #[test]
    fn nodes_beams_and_modifiers() {
        let mut l = Library::default();
        l.add_file(
            "t.jbeam",
            r#"{"car":{"slotType":"main","slots":[["type","default"],["w","wheel","",{"nodeOffset":{"x":1,"y":0,"z":0}}]],
              "nodes":[["id","posX","posY","posZ"],{"nodeWeight":10},["a",0,-1,0.5],["b",0,1,0.5,{"nodeWeight":3}],{"fixed":true},["c",1,0,0]],
              "beams":[["id1:","id2:"],{"beamSpring":1000,"beamDeform":"FLT_MAX"},["a","b"],{"beamType":"|HYDRO"},["b","c",{"beamSpring":5}],["a","zzz"],["a","zzz",{"optional":true}]]},
              "wheel":{"slotType":"w","nodes":[["id","posX","posY","posZ"],["w1",0,0,0.3],["w2",-0.5,0,0.3]]}}"#,
        );
        let m = build(assemble(&l, "car", &HashMap::new()));
        assert_eq!(m.nodes.len(), 5);
        assert_eq!(m.nodes[0].mass, 10.0);
        assert_eq!(m.nodes[1].mass, 3.0);
        assert!(m.nodes[2].fixed && !m.nodes[0].fixed);
        assert_eq!(m.nodes[3].pos, [1.0, 0.0, 0.3]);
        // nodeOffset x is mirrored for nodes on the right (negative x)
        assert_eq!(m.nodes[4].pos, [-1.5, 0.0, 0.3]);
        assert_eq!(m.beams.len(), 2);
        assert_eq!(m.beams[0].spring(), 1000.0);
        assert_eq!(m.beams[0].deform(), None);
        assert_eq!(m.beams[1].kind, "HYDRO");
        assert_eq!(m.beams[1].spring(), 5.0);
        assert_eq!(m.skipped_beams, 1);
        // forward = -jbeam Y: node a (y=-1) is in front
        assert!(to_ours(m.nodes[0].pos)[0] > 0.0);
    }
}
