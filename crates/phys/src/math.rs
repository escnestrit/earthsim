//! Minimal f64 vector / quaternion math. No dependencies.
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct V3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub const fn v3(x: f64, y: f64, z: f64) -> V3 {
    V3 { x, y, z }
}

impl V3 {
    pub fn splat(a: f64) -> V3 {
        V3 { x: a, y: a, z: a }
    }
    pub fn min(self, o: V3) -> V3 {
        V3 { x: self.x.min(o.x), y: self.y.min(o.y), z: self.z.min(o.z) }
    }
    pub fn max(self, o: V3) -> V3 {
        V3 { x: self.x.max(o.x), y: self.y.max(o.y), z: self.z.max(o.z) }
    }
    pub const ZERO: V3 = v3(0.0, 0.0, 0.0);
    pub const X: V3 = v3(1.0, 0.0, 0.0);
    pub const Y: V3 = v3(0.0, 1.0, 0.0);
    pub const Z: V3 = v3(0.0, 0.0, 1.0);

    #[inline]
    pub fn dot(self, o: V3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    #[inline]
    pub fn cross(self, o: V3) -> V3 {
        v3(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    #[inline]
    pub fn len2(self) -> f64 {
        self.dot(self)
    }
    #[inline]
    pub fn len(self) -> f64 {
        self.len2().sqrt()
    }
    /// Unit vector, or zero if the length is (nearly) zero.
    pub fn norm(self) -> V3 {
        let l = self.len();
        if l < 1e-12 {
            V3::ZERO
        } else {
            self / l
        }
    }
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// Component-wise multiply.
    pub fn mulv(self, o: V3) -> V3 {
        v3(self.x * o.x, self.y * o.y, self.z * o.z)
    }
}

impl Add for V3 {
    type Output = V3;
    #[inline]
    fn add(self, o: V3) -> V3 {
        v3(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for V3 {
    type Output = V3;
    #[inline]
    fn sub(self, o: V3) -> V3 {
        v3(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Neg for V3 {
    type Output = V3;
    #[inline]
    fn neg(self) -> V3 {
        v3(-self.x, -self.y, -self.z)
    }
}
impl Mul<f64> for V3 {
    type Output = V3;
    #[inline]
    fn mul(self, s: f64) -> V3 {
        v3(self.x * s, self.y * s, self.z * s)
    }
}
impl Mul<V3> for f64 {
    type Output = V3;
    #[inline]
    fn mul(self, v: V3) -> V3 {
        v * self
    }
}
impl Div<f64> for V3 {
    type Output = V3;
    #[inline]
    fn div(self, s: f64) -> V3 {
        v3(self.x / s, self.y / s, self.z / s)
    }
}
impl AddAssign for V3 {
    #[inline]
    fn add_assign(&mut self, o: V3) {
        *self = *self + o;
    }
}
impl SubAssign for V3 {
    #[inline]
    fn sub_assign(&mut self, o: V3) {
        *self = *self - o;
    }
}

/// Unit quaternion, body -> world.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quat {
    pub const IDENT: Quat = Quat { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    pub fn axis_angle(axis: V3, ang: f64) -> Quat {
        let a = axis.norm();
        let (s, c) = (ang * 0.5).sin_cos();
        Quat { w: c, x: a.x * s, y: a.y * s, z: a.z * s }
    }
    pub fn mul(self, o: Quat) -> Quat {
        Quat {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
    }
    pub fn normalize(self) -> Quat {
        let n = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if n < 1e-12 {
            Quat::IDENT
        } else {
            Quat { w: self.w / n, x: self.x / n, y: self.y / n, z: self.z / n }
        }
    }
    pub fn conj(self) -> Quat {
        Quat { w: self.w, x: -self.x, y: -self.y, z: -self.z }
    }
    /// Rotate a body-frame vector into the world frame.
    #[inline]
    pub fn rotate(self, v: V3) -> V3 {
        let q = v3(self.x, self.y, self.z);
        let t = 2.0 * q.cross(v);
        v + self.w * t + q.cross(t)
    }
    /// Rotate a world-frame vector into the body frame.
    #[inline]
    pub fn inv_rotate(self, v: V3) -> V3 {
        self.conj().rotate(v)
    }
    /// Integrate an angular velocity given in the world frame.
    pub fn integrate(self, w_world: V3, dt: f64) -> Quat {
        let wq = Quat { w: 0.0, x: w_world.x, y: w_world.y, z: w_world.z };
        let d = wq.mul(self);
        Quat {
            w: self.w + 0.5 * dt * d.w,
            x: self.x + 0.5 * dt * d.x,
            y: self.y + 0.5 * dt * d.y,
            z: self.z + 0.5 * dt * d.z,
        }
        .normalize()
    }
    /// Heading of the body +X axis in the XY plane.
    pub fn yaw(self) -> f64 {
        let f = self.rotate(V3::X);
        f.y.atan2(f.x)
    }
    pub fn from_yaw(yaw: f64) -> Quat {
        Quat::axis_angle(V3::Z, yaw)
    }
}

pub fn clamp(x: f64, a: f64, b: f64) -> f64 {
    x.max(a).min(b)
}

pub fn smoothstep(a: f64, b: f64, x: f64) -> f64 {
    let t = clamp((x - a) / (b - a), 0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quat_rotates_like_axis_angle() {
        let q = Quat::axis_angle(V3::Z, std::f64::consts::FRAC_PI_2);
        let r = q.rotate(V3::X);
        assert!((r - V3::Y).len() < 1e-12);
        assert!((q.inv_rotate(r) - V3::X).len() < 1e-12);
    }

    #[test]
    fn quat_integration_matches_constant_rotation() {
        let mut q = Quat::IDENT;
        let w = v3(0.0, 0.0, 1.0); // 1 rad/s about Z
        for _ in 0..10_000 {
            q = q.integrate(w, 1e-4);
        }
        assert!((q.yaw() - 1.0).abs() < 1e-3, "yaw {}", q.yaw());
    }

    #[test]
    fn cross_dot_identities() {
        let a = v3(1.0, 2.0, 3.0);
        let b = v3(-4.0, 0.5, 2.0);
        let c = a.cross(b);
        assert!(c.dot(a).abs() < 1e-12 && c.dot(b).abs() < 1e-12);
    }
}
