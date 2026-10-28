//! Skins the car body mesh onto the soft-body lattice so crash damage is visible: every vertex follows the four
//! nearest lattice nodes (inverse-distance weights), carrying its rest offset in the chassis frame.
use phys::softcar::{cage_index, STATIONS, YS, ZS};
use phys::{SoftVehicle, V3, v3};
use scene::mesh::FLOATS_PER_VERTEX;
use scene::MeshData;

const STRIDE: usize = FLOATS_PER_VERTEX;

pub struct SkinnedBody {
    rest: Vec<f32>,
    /// per vertex: 4 (node index, weight)
    bind: Vec<[(u16, f32); 4]>,
    /// per triangle: +1 when the rest normal agrees with (p1-p0)x(p2-p0), else -1
    flip: Vec<f32>,
}

fn p3(v: &[f32], i: usize) -> V3 {
    v3(v[i * STRIDE] as f64, v[i * STRIDE + 1] as f64, v[i * STRIDE + 2] as f64)
}

impl SkinnedBody {
    pub fn new(mesh: &MeshData, car: &SoftVehicle) -> SkinnedBody {
        let lattice: Vec<usize> = (0..STATIONS.len()).flat_map(|s| (0..YS.len()).flat_map(move |y| (0..ZS.len()).map(move |z| cage_index(s, y, z)))).collect();
        let n = mesh.verts.len() / STRIDE;
        let mut bind = Vec::with_capacity(n);
        for vi in 0..n {
            let v = p3(&mesh.verts, vi);
            let mut d: Vec<(f64, usize)> = lattice.iter().map(|&k| ((car.body.nodes[k].rest - v).len(), k)).collect();
            d.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
            let mut w = [(0u16, 0f32); 4];
            let mut sum = 0.0;
            for (j, &(dist, k)) in d.iter().take(4).enumerate() {
                let wt = 1.0 / (dist * dist + 0.01).powi(2);
                w[j] = (k as u16, wt as f32);
                sum += wt;
            }
            for e in w.iter_mut() {
                e.1 /= sum as f32;
            }
            bind.push(w);
        }
        let mut flip = Vec::with_capacity(n / 3);
        for t in 0..n / 3 {
            let (a, b, c) = (p3(&mesh.verts, t * 3), p3(&mesh.verts, t * 3 + 1), p3(&mesh.verts, t * 3 + 2));
            let nr = (b - a).cross(c - a);
            let on = v3(mesh.verts[t * 3 * STRIDE + 3] as f64, mesh.verts[t * 3 * STRIDE + 4] as f64, mesh.verts[t * 3 * STRIDE + 5] as f64);
            flip.push(if nr.dot(on) >= 0.0 { 1.0 } else { -1.0 });
        }
        SkinnedBody { rest: mesh.verts.clone(), bind, flip }
    }

    /// World-space vertices (same layout as the mesh) for the car's current state; normals recomputed per triangle.
    pub fn deform(&self, car: &SoftVehicle, out: &mut Vec<f32>) {
        out.clear();
        out.extend_from_slice(&self.rest);
        let q = car.q;
        let n = self.bind.len();
        for vi in 0..n {
            let v = p3(&self.rest, vi);
            let mut p = V3::ZERO;
            for &(k, w) in &self.bind[vi] {
                let node = &car.body.nodes[k as usize];
                p += (node.p + q.rotate(v - node.rest)) * w as f64;
            }
            out[vi * STRIDE] = p.x as f32;
            out[vi * STRIDE + 1] = p.y as f32;
            out[vi * STRIDE + 2] = p.z as f32;
        }
        for t in 0..n / 3 {
            let (a, b, c) = (p3(out, t * 3), p3(out, t * 3 + 1), p3(out, t * 3 + 2));
            let mut nr = (b - a).cross(c - a);
            let l = nr.len();
            nr = if l > 1e-12 { nr * (self.flip[t] as f64 / l) } else { V3::Z };
            for j in 0..3 {
                let o = (t * 3 + j) * STRIDE;
                out[o + 3] = nr.x as f32;
                out[o + 4] = nr.y as f32;
                out[o + 5] = nr.z as f32;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phys::{FlatGround, StaticWorld, VehicleParams};

    #[test]
    fn undamaged_car_is_the_rest_mesh_moved_rigidly() {
        let g = FlatGround { grip: 1.0 };
        let (body, _) = scene::car_meshes();
        let mut car = SoftVehicle::new(VehicleParams::sedan(), 10.0, 5.0, 0.7, &g);
        let skin = SkinnedBody::new(&body, &car);
        let w = StaticWorld::new(10.0);
        for _ in 0..3000 {
            car.step(0.001, &Default::default(), &g, &w);
        }
        let mut out = Vec::new();
        skin.deform(&car, &mut out);
        assert_eq!(out.len(), body.verts.len());
        // wheels' lowest point sits on the ground, the roof ~1.46 m above it, within a few cm of settling
        let zs: Vec<f32> = (0..out.len() / STRIDE).map(|i| out[i * STRIDE + 2]).collect();
        let (lo, hi) = (zs.iter().cloned().fold(f32::MAX, f32::min), zs.iter().cloned().fold(f32::MIN, f32::max));
        assert!(lo > 0.1 && lo < 0.3 && hi > 1.40 && hi < 1.55, "z range {lo}..{hi}");
        // distances between vertices are preserved (rigid motion) to a few mm
        let mut worst: f64 = 0.0;
        for (i, j) in [(0usize, 100usize), (7, 200), (30, 150), (50, 240)] {
            let (a0, b0) = (p3(&body.verts, i), p3(&body.verts, j));
            let (a1, b1) = (p3(&out, i), p3(&out, j));
            worst = worst.max(((a1 - b1).len() - (a0 - b0).len()).abs());
        }
        assert!(worst < 0.01, "distance error {worst}");
        // normals are unit length and agree with the original orientation after the 0.7 rad yaw
        for t in (0..out.len() / STRIDE / 3).step_by(37) {
            let o = t * 3 * STRIDE;
            let n = v3(out[o + 3] as f64, out[o + 4] as f64, out[o + 5] as f64);
            let n0 = car.q.rotate(v3(body.verts[o + 3] as f64, body.verts[o + 4] as f64, body.verts[o + 5] as f64));
            assert!((n.len() - 1.0).abs() < 1e-3 && n.dot(n0) > 0.98, "tri {t}: {n:?} vs {n0:?}");
        }
    }

    #[test]
    fn a_crash_moves_the_nose_back_but_not_the_roof_rear() {
        let g = FlatGround { grip: 1.0 };
        let (body, _) = scene::car_meshes();
        let mut car = SoftVehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
        let skin = SkinnedBody::new(&body, &car);
        let mut wall = StaticWorld::new(10.0);
        wall.add_poly(vec![(30.0, -10.0), (32.0, -10.0), (32.0, 10.0), (30.0, 10.0)]);
        car.place(0.0, 0.0, 0.0, 13.9, &g);
        let mut before = Vec::new();
        skin.deform(&car, &mut before);
        for _ in 0..3500 {
            car.step(0.001, &Default::default(), &g, &wall);
        }
        let mut after = Vec::new();
        skin.deform(&car, &mut after);
        let n = out_len(&after);
        // express both in the car frame to compare shapes
        let local = |v: &Vec<f32>, car: &SoftVehicle, i: usize| car.q.inv_rotate(p3(v, i) - car.pos);
        let mut nose_shift: f64 = 0.0;
        let mut tail_shift: f64 = 0.0;
        let ref_car = SoftVehicle::new(VehicleParams::sedan(), 0.0, 0.0, 0.0, &g);
        for i in 0..n {
            let rest = local(&before, &ref_car, i);
            let now = local(&after, &car, i);
            if rest.x > 2.0 {
                nose_shift = nose_shift.max(rest.x - now.x);
            }
            if rest.x < -1.8 {
                tail_shift = tail_shift.max((rest.x - now.x).abs());
            }
        }
        assert!(nose_shift > 0.25, "nose moved only {nose_shift} m");
        assert!(tail_shift < 0.15, "tail moved {tail_shift} m");
        assert!(after.iter().all(|v| v.is_finite()));
    }

    fn out_len(v: &[f32]) -> usize {
        v.len() / STRIDE
    }
}
