//! 4x4 column-major f32 matrices and small vector helpers (right-handed, Z-up world, OpenGL clip space).
pub type M4 = [f32; 16];
pub type V3f = [f32; 3];

pub fn id() -> M4 {
    [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.]
}
pub fn mul(a: &M4, b: &M4) -> M4 {
    let mut o = [0.0f32; 16];
    for c in 0..4 {
        for r in 0..4 {
            let mut s = 0.0;
            for k in 0..4 {
                s += a[k * 4 + r] * b[c * 4 + k];
            }
            o[c * 4 + r] = s;
        }
    }
    o
}
pub fn translate(x: f32, y: f32, z: f32) -> M4 {
    let mut m = id();
    m[12] = x;
    m[13] = y;
    m[14] = z;
    m
}
pub fn rot_z(a: f32) -> M4 {
    let (s, c) = a.sin_cos();
    let mut m = id();
    m[0] = c;
    m[1] = s;
    m[4] = -s;
    m[5] = c;
    m
}
pub fn rot_y(a: f32) -> M4 {
    let (s, c) = a.sin_cos();
    let mut m = id();
    m[0] = c;
    m[2] = -s;
    m[8] = s;
    m[10] = c;
    m
}
/// Rotation + translation from a unit quaternion (w, x, y, z) and a position.
pub fn from_quat(q: [f64; 4], p: [f64; 3]) -> M4 {
    let (w, x, y, z) = (q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32);
    let mut m = id();
    m[0] = 1.0 - 2.0 * (y * y + z * z);
    m[1] = 2.0 * (x * y + w * z);
    m[2] = 2.0 * (x * z - w * y);
    m[4] = 2.0 * (x * y - w * z);
    m[5] = 1.0 - 2.0 * (x * x + z * z);
    m[6] = 2.0 * (y * z + w * x);
    m[8] = 2.0 * (x * z + w * y);
    m[9] = 2.0 * (y * z - w * x);
    m[10] = 1.0 - 2.0 * (x * x + y * y);
    m[12] = p[0] as f32;
    m[13] = p[1] as f32;
    m[14] = p[2] as f32;
    m
}
pub fn perspective(fov_y: f32, aspect: f32, n: f32, f: f32) -> M4 {
    let t = 1.0 / (fov_y / 2.0).tan();
    let mut m = [0.0f32; 16];
    m[0] = t / aspect;
    m[5] = t;
    m[10] = (f + n) / (n - f);
    m[11] = -1.0;
    m[14] = 2.0 * f * n / (n - f);
    m
}
pub fn ortho(l: f32, r: f32, b: f32, t: f32, n: f32, f: f32) -> M4 {
    let mut m = [0.0f32; 16];
    m[0] = 2.0 / (r - l);
    m[5] = 2.0 / (t - b);
    m[10] = -2.0 / (f - n);
    m[12] = -(r + l) / (r - l);
    m[13] = -(t + b) / (t - b);
    m[14] = -(f + n) / (f - n);
    m[15] = 1.0;
    m
}
pub fn look_at(e: V3f, c: V3f, u: V3f) -> M4 {
    let f = norm(sub(c, e));
    let s = norm(cross(f, u));
    let w = cross(s, f);
    let mut m = [0.0f32; 16];
    m[0] = s[0];
    m[4] = s[1];
    m[8] = s[2];
    m[1] = w[0];
    m[5] = w[1];
    m[9] = w[2];
    m[2] = -f[0];
    m[6] = -f[1];
    m[10] = -f[2];
    m[12] = -dot(s, e);
    m[13] = -dot(w, e);
    m[14] = dot(f, e);
    m[15] = 1.0;
    m
}
pub fn sub(a: V3f, b: V3f) -> V3f {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn add(a: V3f, b: V3f) -> V3f {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn scale(a: V3f, s: f32) -> V3f {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub fn dot(a: V3f, b: V3f) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: V3f, b: V3f) -> V3f {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn norm(a: V3f) -> V3f {
    let l = dot(a, a).sqrt();
    if l < 1e-12 { [0.0, 0.0, 1.0] } else { scale(a, 1.0 / l) }
}
pub fn transform_point(m: &M4, p: V3f) -> [f32; 4] {
    let mut o = [0.0; 4];
    for r in 0..4 {
        o[r] = m[r] * p[0] + m[4 + r] * p[1] + m[8 + r] * p[2] + m[12 + r];
    }
    o
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: &[f32], b: &[f32]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-5)
    }

    #[test]
    fn identity_and_translation_compose() {
        let t = translate(1.0, 2.0, 3.0);
        assert!(close(&mul(&id(), &t), &t));
        let p = transform_point(&mul(&t, &translate(1.0, 0.0, 0.0)), [0.0, 0.0, 0.0]);
        assert!(close(&p, &[2.0, 2.0, 3.0, 1.0]));
    }

    #[test]
    fn rot_y_positive_moves_top_forward() {
        // wheel top (+Z) must go toward +X for forward rolling
        let p = transform_point(&rot_y(0.3), [0.0, 0.0, 1.0]);
        assert!(p[0] > 0.0);
        let p = transform_point(&rot_z(std::f32::consts::FRAC_PI_2), [1.0, 0.0, 0.0]);
        assert!(close(&p, &[0.0, 1.0, 0.0, 1.0]));
    }

    #[test]
    fn quaternion_matrix_matches_phys_rotation() {
        let q = phys::Quat::axis_angle(phys::v3(0.3, -0.5, 0.8).norm(), 0.9);
        let m = from_quat([q.w, q.x, q.y, q.z], [0.0, 0.0, 0.0]);
        let v = phys::v3(1.0, 2.0, 3.0);
        let r = q.rotate(v);
        let p = transform_point(&m, [1.0, 2.0, 3.0]);
        assert!(close(&p, &[r.x as f32, r.y as f32, r.z as f32, 1.0]));
    }

    #[test]
    fn look_at_puts_target_in_front_and_projects_to_centre() {
        let v = look_at([0.0, -10.0, 2.0], [0.0, 0.0, 2.0], [0.0, 0.0, 1.0]);
        let p = transform_point(&v, [0.0, 0.0, 2.0]);
        assert!(close(&p[..3], &[0.0, 0.0, -10.0]));
        let clip = transform_point(&mul(&perspective(1.0, 1.5, 0.5, 1000.0), &v), [0.0, 0.0, 2.0]);
        assert!((clip[0] / clip[3]).abs() < 1e-5 && (clip[1] / clip[3]).abs() < 1e-5);
        let d = clip[2] / clip[3];
        assert!(d > -1.0 && d < 1.0);
    }
}
