//! BeamNG `props`: rigid meshes carried by a frame of three nodes and turned by a vehicle signal.
//! Only the steering wheel is handled so far: its mesh is modelled around its own origin and placed at the
//! `idRef` node in the frame (x towards `idX`, y towards `idY`, z = x × y), with `baseRotation` (degrees,
//! x then y then z) and `baseTranslation`, then turned about `rotation` by the steering wheel angle.
//! (Gauge needles and pedals use a different placement convention that is not handled yet.)
use crate::dae::SubMesh;
use crate::flex::MeshFile;
use crate::model::Model;

#[derive(Clone, Debug)]
pub struct Prop {
    pub func: String,
    pub mesh: String,
    pub submeshes: Vec<SubMesh>,
    /// idRef, idX, idY (node indices)
    pub refs: [usize; 3],
    pub base_rot_deg: [f64; 3],
    pub base_trans: [f64; 3],
    /// rotation axis (mesh space), degrees per unit of the signal
    pub axis: [f64; 3],
    pub multiplier: f64,
    pub offset: f64,
    pub min: f64,
    pub max: f64,
    /// steering wheel turn at full lock (degrees)
    pub lock_deg: f64,
}

fn idx(m: &Model, n: Option<&str>) -> Option<usize> {
    n.and_then(|n| m.node_index.get(n).copied())
}

fn v3(r: &crate::table::Row, k: &str, d: f64) -> [f64; 3] {
    let o = r.get(k);
    let c = |a: &str| o.and_then(|o| o.get(a)).and_then(|v| v.as_f64()).unwrap_or(d);
    [c("x"), c("y"), c("z")]
}

pub fn build(m: &Model, files: &[MeshFile]) -> (Vec<Prop>, Vec<String>) {
    let lock = m.table("hydros").iter().find_map(|(_, r)| r.num("steeringWheelLock")).unwrap_or(450.0);
    let mut out = Vec::new();
    let mut notes = Vec::new();
    for (_, r) in m.table("props") {
        let Some(func) = r.str("func") else { continue };
        if func != "steering" {
            continue;
        }
        let Some(mesh) = r.str("mesh") else { continue };
        let (Some(a), Some(b), Some(c)) = (idx(m, r.str("idRef")), idx(m, r.str("idX")), idx(m, r.str("idY"))) else { continue };
        let Some(f) = files.iter().find(|f| f.has(mesh)) else {
            notes.push(format!("prop mesh {mesh} not found"));
            continue;
        };
        let Ok(subs) = f.mesh(mesh) else { continue };
        out.push(Prop {
            func: func.to_string(),
            mesh: mesh.to_string(),
            submeshes: subs,
            refs: [a, b, c],
            base_rot_deg: v3(&r, "baseRotation", 0.0),
            base_trans: v3(&r, "baseTranslation", 0.0),
            axis: v3(&r, "rotation", 0.0),
            multiplier: r.num_or("multiplier", 1.0),
            offset: r.num_or("offset", 0.0),
            min: r.num_or("min", -1e9),
            max: r.num_or("max", 1e9),
            lock_deg: lock,
        });
    }
    if !out.is_empty() {
        notes.push(format!("steering wheel: {} prop(s), lock {lock:.0} deg", out.len()));
    }
    (out, notes)
}

type M3 = [[f64; 3]; 3];

fn mul(a: &M3, b: &M3) -> M3 {
    let mut o = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            o[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    o
}

fn rot_axis(axis: [f64; 3], deg: f64) -> M3 {
    let l = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if l < 1e-12 || deg == 0.0 {
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    }
    let (x, y, z) = (axis[0] / l, axis[1] / l, axis[2] / l);
    let (s, c) = deg.to_radians().sin_cos();
    let t = 1.0 - c;
    [[t * x * x + c, t * x * y - s * z, t * x * z + s * y], [t * x * y + s * z, t * y * y + c, t * y * z - s * x], [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]
}

impl Prop {
    /// Signal -> angle (degrees) about `axis`. For the steering wheel the signal is BeamNG's steering input
    /// (-1..1, positive to the right) times the lock.
    pub fn angle(&self, signal: f64) -> f64 {
        let v = match self.func.as_str() {
            "steering" => signal * self.lock_deg,
            _ => signal,
        };
        (v * self.multiplier + self.offset).clamp(self.min, self.max)
    }

    /// Pose in the frame of the given node positions: (origin, 3x3 rotation as columns) mapping mesh space
    /// into the nodes' space. `nodes` may be in any frame (rest, local, world).
    pub fn pose(&self, nodes: &[[f64; 3]], signal: f64) -> ([f64; 3], M3) {
        let p = |i: usize| nodes[self.refs[i]];
        let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
        let norm = |a: [f64; 3]| {
            let l = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt().max(1e-12);
            [a[0] / l, a[1] / l, a[2] / l]
        };
        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let cross = |a: [f64; 3], b: [f64; 3]| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
        let o = p(0);
        let x = norm(sub(p(1), o));
        let y0 = sub(p(2), o);
        let y = norm(sub(y0, x.map(|c| c * dot(y0, x))));
        let z = cross(x, y);
        // frame as columns
        let f: M3 = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]];
        let [bx, by, bz] = self.base_rot_deg;
        let base = mul(&rot_axis([0.0, 0.0, 1.0], bz), &mul(&rot_axis([0.0, 1.0, 0.0], by), &rot_axis([1.0, 0.0, 0.0], bx)));
        let r = mul(&f, &mul(&base, &rot_axis(self.axis, self.angle(signal))));
        let t = self.base_trans;
        let off = [0, 1, 2].map(|i| f[i][0] * t[0] + f[i][1] * t[1] + f[i][2] * t[2]);
        ([o[0] + off[0], o[1] + off[1], o[2] + off[2]], r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_base_rotation_and_turning() {
        let p = Prop {
            func: "steering".into(),
            mesh: "w".into(),
            submeshes: Vec::new(),
            refs: [0, 1, 2],
            base_rot_deg: [0.0, 90.0, 0.0],
            base_trans: [0.1, 0.0, 0.0],
            axis: [0.0, 0.0, 1.0],
            multiplier: 1.0,
            offset: 0.0,
            min: -1000.0,
            max: 1000.0,
            lock_deg: 450.0,
        };
        let nodes = [[1.0, 2.0, 3.0], [1.0, 2.0, 4.0], [1.0, 5.0, 3.0]]; // x up (world z), y towards world y
        let (o, r) = p.pose(&nodes, 0.0);
        assert!((o[2] - 3.1).abs() < 1e-9, "base translation along the frame x: {o:?}");
        // mesh z (wheel axis) after base rotation y 90 -> frame -x... check it is along world z (the column)
        let mz = [r[0][2], r[1][2], r[2][2]];
        assert!(mz[2].abs() > 0.999, "{mz:?}");
        // a quarter of full lock turns the rim by 112.5 degrees about the wheel axis
        let (_, r2) = p.pose(&nodes, 0.25);
        let mx0 = [r[0][0], r[1][0], r[2][0]];
        let mx1 = [r2[0][0], r2[1][0], r2[2][0]];
        let c = mx0[0] * mx1[0] + mx0[1] * mx1[1] + mx0[2] * mx1[2];
        assert!((c - 112.5f64.to_radians().cos()).abs() < 1e-9);
        assert_eq!(p.angle(10.0), 1000.0);
    }
}
