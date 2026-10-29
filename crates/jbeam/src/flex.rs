//! Flexbodies: the mod's visual meshes glued to its node/beam skeleton. Each vertex is expressed in a small
//! frame spanned by three nearby nodes of the flexbody's node groups (like BeamNG), so the mesh follows the
//! nodes exactly under rigid motion and bends with them when they deform. Wheel meshes, whose only nodes are
//! a wheel's two axle nodes, are kept rigid and spun by the renderer instead.

use crate::dae::{Dae, SubMesh};
use crate::model::{to_ours, Model};
use crate::vehicle::JbeamVehicle;
use phys::math::{v3, V3};
use std::collections::HashSet;

#[derive(Clone, Copy, Debug)]
pub struct Bind {
    pub a: u32,
    pub b: u32,
    pub c: u32,
    /// position = a + p0*e1 + p1*e2 + p2*n
    pub p: [f32; 3],
    /// normal direction = n0*e1 + n1*e2 + n2*n
    pub n: [f32; 3],
}

pub struct FlexPart {
    pub mesh: String,
    pub submeshes: Vec<SubMesh>,
    /// one binding per vertex of each submesh (empty for rigid wheels)
    pub binds: Vec<Vec<Bind>>,
    /// index of the vehicle wheel this mesh belongs to (rendered rigid, in the wheel's frame)
    pub wheel: Option<usize>,
    /// nodes this part depends on (for "has it changed" checks)
    pub nodes: Vec<u32>,
}

/// frame vector n for an (e1, e2) pair: unit normal scaled to the edge size, so it stretches with the part
fn frame_n(e1: V3, e2: V3) -> V3 {
    let c = e1.cross(e2);
    let l = c.len();
    if l < 1e-12 {
        return V3::ZERO;
    }
    c * ((e1.len() + e2.len()) * 0.5 / l)
}

fn solve3(c0: V3, c1: V3, c2: V3, r: V3) -> Option<[f64; 3]> {
    let det = c0.dot(c1.cross(c2));
    if det.abs() < 1e-12 {
        return None;
    }
    Some([r.dot(c1.cross(c2)) / det, c0.dot(r.cross(c2)) / det, c0.dot(c1.cross(r)) / det])
}

/// Choose three nodes around `v` that span a usable frame and bind `v` (and its normal) to them.
pub fn bind_vertex(v: V3, nrm: V3, cand: &[(u32, V3)]) -> Option<Bind> {
    if cand.len() < 3 {
        return None;
    }
    let mut near: Vec<(f64, usize)> = cand.iter().enumerate().map(|(i, (_, p))| ((*p - v).len2(), i)).collect();
    let k = near.len().min(10);
    near.select_nth_unstable_by(k - 1, |a, b| a.0.total_cmp(&b.0));
    near.truncate(k);
    near.sort_by(|a, b| a.0.total_cmp(&b.0));
    let a = near[0].1;
    let pa = cand[a].1;
    let mut best: Option<(f64, usize, usize)> = None;
    for i in 1..near.len() {
        for j in i + 1..near.len() {
            let (pb, pc) = (cand[near[i].1].1, cand[near[j].1].1);
            let (e1, e2) = (pb - pa, pc - pa);
            let area = e1.cross(e2).len();
            let quality = area / (e1.len2() + e2.len2()).max(1e-9); // well-shaped, not degenerate
            // prefer good shape, then closeness
            let score = quality - 0.05 * (near[i].0.sqrt() + near[j].0.sqrt());
            if quality > 0.05 && best.is_none_or(|b| score > b.0) {
                best = Some((score, near[i].1, near[j].1));
            }
        }
    }
    let (_, b, c) = best?;
    let (pb, pc) = (cand[b].1, cand[c].1);
    let (e1, e2) = (pb - pa, pc - pa);
    let n = frame_n(e1, e2);
    let p = solve3(e1, e2, n, v - pa)?;
    let nn = solve3(e1, e2, n, nrm)?;
    Some(Bind { a: cand[a].0, b: cand[b].0, c: cand[c].0, p: p.map(|x| x as f32), n: nn.map(|x| x as f32) })
}

/// Rebuild positions/normals of one submesh from node positions (any frame: rest, local or world).
pub fn deform(binds: &[Bind], nodes: &[V3], pos: &mut Vec<[f32; 3]>, nrm: &mut Vec<[f32; 3]>) {
    pos.clear();
    nrm.clear();
    for b in binds {
        let pa = nodes[b.a as usize];
        let e1 = nodes[b.b as usize] - pa;
        let e2 = nodes[b.c as usize] - pa;
        let n = frame_n(e1, e2);
        let p = pa + e1 * b.p[0] as f64 + e2 * b.p[1] as f64 + n * b.p[2] as f64;
        let d = (e1 * b.n[0] as f64 + e2 * b.n[1] as f64 + n * b.n[2] as f64).norm();
        pos.push([p.x as f32, p.y as f32, p.z as f32]);
        nrm.push([d.x as f32, d.y as f32, d.z as f32]);
    }
}

fn to_ours_f(p: [f32; 3]) -> V3 {
    let o = to_ours([p[0] as f64, p[1] as f64, p[2] as f64]);
    v3(o[0], o[1], o[2])
}

/// Build all flexbodies of an assembled vehicle from its .dae files. Positions are converted to the engine frame
/// and to the vehicle's rest shape (same offsets as `JbeamVehicle` node rest positions).
pub fn build(m: &Model, v: &JbeamVehicle, daes: &[Dae]) -> (Vec<FlexPart>, Vec<String>) {
    let mut notes = Vec::new();
    // rest positions of the vehicle nodes (engine frame, ground at z = 0 under the wheels)
    let rest: Vec<V3> = v.body.nodes.iter().map(|n| n.rest).collect();
    // the vehicle shifted its nodes vertically when it put the wheels on the ground; meshes need the same shift
    let shift = if m.nodes.is_empty() { V3::ZERO } else { rest[0] - to_ours_f(m.nodes[0].pos.map(|x| x as f32)) };
    let wheel_nodes: Vec<HashSet<u32>> = v.wheels.iter().map(|w| [w.n1 as u32, w.n2 as u32].into_iter().collect()).collect();
    let mut parts = Vec::new();
    let mut missing = 0;
    for (pi, r) in m.table("flexbodies") {
        let Some(mesh) = r.str("mesh") else { continue };
        let part = &m.asm.parts[pi];
        let groups = r.strings("[group]");
        let Some(dae) = daes.iter().find(|d| d.node(mesh).is_some()) else {
            missing += 1;
            continue;
        };
        let Ok(mut subs) = dae.mesh(mesh) else { continue };
        // flexbody placement options (jbeam axes): scale, rotate (degrees, x then y then z), translate; then the
        // part's slot offset (x mirrored to the mesh's side, like nodes) and move
        let g3 = |k: &str, d: f64| -> [f64; 3] {
            let o = r.get(k);
            let c = |a: &str| o.and_then(|o| o.get(a)).and_then(|v| v.as_f64()).unwrap_or(d);
            [c("x"), c("y"), c("z")]
        };
        let (fpos, frot, fscale) = (g3("pos", 0.0), g3("rot", 0.0), g3("scale", 1.0));
        let (sx, cx) = frot[0].to_radians().sin_cos();
        let (sy, cy) = frot[1].to_radians().sin_cos();
        let (sz, cz) = frot[2].to_radians().sin_cos();
        let rot = |v: [f64; 3]| -> [f64; 3] {
            let (x, y, z) = (v[0], v[1] * cx - v[2] * sx, v[1] * sx + v[2] * cx);
            let (x, z) = (x * cy + z * sy, -x * sy + z * cy);
            [x * cz - y * sz, x * sz + y * cz, z]
        };
        let mut cxs = 0.0;
        let mut ncount = 0usize;
        for s in subs.iter() {
            for p in &s.positions {
                cxs += p[0] as f64 * fscale[0];
                ncount += 1;
            }
        }
        let side = if fpos[0] + cxs / ncount.max(1) as f64 >= 0.0 { 1.0 } else { -1.0 };
        let place = |p: [f32; 3]| -> [f32; 3] {
            let v = rot([p[0] as f64 * fscale[0], p[1] as f64 * fscale[1], p[2] as f64 * fscale[2]]);
            let v = [v[0] + fpos[0] + part.offset[0] * side + part.node_move[0], v[1] + fpos[1] + part.offset[1] + part.node_move[1], v[2] + fpos[2] + part.offset[2] + part.node_move[2]];
            [v[0] as f32, v[1] as f32, v[2] as f32]
        };
        for s in subs.iter_mut() {
            for p in s.positions.iter_mut() {
                let q = to_ours_f(place(*p)) + shift;
                *p = [q.x as f32, q.y as f32, q.z as f32];
            }
            for n in s.normals.iter_mut() {
                let v = rot([n[0] as f64, n[1] as f64, n[2] as f64]);
                let q = to_ours_f([v[0] as f32, v[1] as f32, v[2] as f32]).norm();
                *n = [q.x as f32, q.y as f32, q.z as f32];
            }
        }
        let gset: HashSet<&str> = groups.iter().map(String::as_str).collect();
        let mut cand: Vec<(u32, V3)> = m.nodes.iter().enumerate().filter(|(_, n)| n.group.iter().any(|g| gset.contains(g.as_str()))).map(|(i, _)| (i as u32, rest[i])).collect();
        let node_ids: HashSet<u32> = cand.iter().map(|c| c.0).collect();
        let wheel = wheel_nodes.iter().position(|w| !node_ids.is_empty() && node_ids.is_subset(w));
        if let Some(wi) = wheel {
            parts.push(FlexPart { mesh: mesh.to_string(), submeshes: subs, binds: Vec::new(), wheel: Some(wi), nodes: node_ids.into_iter().collect() });
            continue;
        }
        if cand.len() < 3 {
            // too few nodes in its groups (often a group from a missing part): glue it to the whole body
            cand = rest.iter().enumerate().map(|(i, p)| (i as u32, *p)).collect();
        }
        let mut binds = Vec::new();
        let mut used: HashSet<u32> = HashSet::new();
        let mut unbound = 0;
        for s in &subs {
            let mut bs = Vec::with_capacity(s.positions.len());
            for (p, n) in s.positions.iter().zip(&s.normals) {
                let pv = v3(p[0] as f64, p[1] as f64, p[2] as f64);
                let nv = v3(n[0] as f64, n[1] as f64, n[2] as f64);
                match bind_vertex(pv, nv, &cand) {
                    Some(b) => {
                        used.extend([b.a, b.b, b.c]);
                        bs.push(b);
                    }
                    None => {
                        unbound += 1;
                        // degenerate neighbourhood: pin to the nearest node with a plain offset
                        let (i, _) = cand.iter().min_by(|x, y| (x.1 - pv).len2().total_cmp(&(y.1 - pv).len2())).copied().unwrap();
                        bs.push(Bind { a: i, b: i, c: i, p: [0.0; 3], n: [0.0, 0.0, 1.0] });
                    }
                }
            }
            binds.push(bs);
        }
        if unbound > 0 {
            notes.push(format!("{mesh}: {unbound} vertices without a node frame"));
        }
        parts.push(FlexPart { mesh: mesh.to_string(), submeshes: subs, binds, wheel: None, nodes: used.into_iter().collect() });
    }
    if missing > 0 {
        notes.push(format!("{missing} flexbody meshes are not in the mod's .dae files (BeamNG common parts)"));
    }
    (parts, notes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use phys::math::Quat;

    fn cloud() -> Vec<(u32, V3)> {
        let mut c = Vec::new();
        let mut k = 0;
        for x in [-1.0, 0.0, 1.0] {
            for y in [-0.5, 0.5] {
                for z in [0.2, 1.0] {
                    c.push((k, v3(x, y, z)));
                    k += 1;
                }
            }
        }
        c
    }

    #[test]
    fn rigid_motion_moves_the_mesh_exactly() {
        let c = cloud();
        let verts = [v3(0.3, 0.1, 0.6), v3(-0.8, 0.45, 0.25), v3(0.95, -0.4, 1.1)];
        let nrm = v3(0.0, 0.6, 0.8);
        let binds: Vec<Bind> = verts.iter().map(|v| bind_vertex(*v, nrm, &c).unwrap()).collect();
        let q = Quat::from_yaw(0.7);
        let t = v3(5.0, -2.0, 0.3);
        let moved: Vec<V3> = c.iter().map(|(_, p)| q.rotate(*p) + t).collect();
        let (mut pos, mut nn) = (Vec::new(), Vec::new());
        deform(&binds, &moved, &mut pos, &mut nn);
        for (i, v) in verts.iter().enumerate() {
            let want = q.rotate(*v) + t;
            let got = v3(pos[i][0] as f64, pos[i][1] as f64, pos[i][2] as f64);
            assert!((got - want).len() < 1e-5, "{got:?} vs {want:?}");
            let wn = q.rotate(nrm);
            let gn = v3(nn[i][0] as f64, nn[i][1] as f64, nn[i][2] as f64);
            assert!((gn - wn).len() < 1e-5);
        }
        // and at rest it reproduces the input
        let rest: Vec<V3> = c.iter().map(|x| x.1).collect();
        deform(&binds, &rest, &mut pos, &mut nn);
        assert!((pos[0][0] - 0.3).abs() < 1e-6 && (pos[0][2] - 0.6).abs() < 1e-6);
    }

    #[test]
    fn deformation_follows_the_nodes() {
        let c = cloud();
        let v = v3(0.9, 0.0, 0.6);
        let b = bind_vertex(v, v3(1.0, 0.0, 0.0), &c).unwrap();
        // push the +x end of the cloud in by 20 cm
        let moved: Vec<V3> = c.iter().map(|(_, p)| if p.x > 0.5 { *p - v3(0.2, 0.0, 0.0) } else { *p }).collect();
        let (mut pos, mut nn) = (Vec::new(), Vec::new());
        deform(&[b], &moved, &mut pos, &mut nn);
        assert!(pos[0][0] < 0.85 && pos[0][0] > 0.6, "vertex x {}", pos[0][0]);
    }
}
