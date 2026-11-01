// Minimal vector / quaternion maths for the simulation core (no engine dependency).
// Frame: x right, y forward, z up (converted at the engine boundary).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace tt {

constexpr double TT_PI = 3.14159265358979323846;

struct V3 {
    double x = 0, y = 0, z = 0;
    V3() = default;
    constexpr V3(double a, double b, double c) : x(a), y(b), z(c) {}
    V3 operator+(const V3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(double k) const { return {x * k, y * k, z * k}; }
    V3 operator/(double k) const { return {x / k, y / k, z / k}; }
    V3 operator-() const { return {-x, -y, -z}; }
    V3 &operator+=(const V3 &o) { x += o.x, y += o.y, z += o.z; return *this; }
    V3 &operator-=(const V3 &o) { x -= o.x, y -= o.y, z -= o.z; return *this; }
    V3 &operator*=(double k) { x *= k, y *= k, z *= k; return *this; }
};
inline double dot(const V3 &a, const V3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(const V3 &a, const V3 &b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double len(const V3 &a) { return std::sqrt(dot(a, a)); }
inline double len2(const V3 &a) { return dot(a, a); }
inline V3 norm(const V3 &a) {
    double l = len(a);
    return l > 1e-12 ? a / l : V3{0, 0, 1};
}
inline double clampd(double v, double a, double b) { return std::min(std::max(v, a), b); }
inline double lerpd(double a, double b, double t) { return a + (b - a) * t; }

struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
    static Quat axis_angle(const V3 &ax, double a) {
        V3 n = norm(ax);
        double s = std::sin(a * 0.5);
        return {std::cos(a * 0.5), n.x * s, n.y * s, n.z * s};
    }
    Quat operator*(const Quat &q) const {
        return {w * q.w - x * q.x - y * q.y - z * q.z, w * q.x + x * q.w + y * q.z - z * q.y, w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w};
    }
    Quat conj() const { return {w, -x, -y, -z}; }
    V3 rotate(const V3 &v) const {
        V3 u{x, y, z};
        V3 t = cross(u, v) * 2.0;
        return v + t * w + cross(u, t);
    }
    Quat normalized() const {
        double l = std::sqrt(w * w + x * x + y * y + z * z);
        return {w / l, x / l, y / l, z / l};
    }
};

// orthonormal frame -> quaternion (columns: right, forward, up)
inline Quat quat_from_basis(const V3 &r, const V3 &f, const V3 &u) {
    double m00 = r.x, m01 = f.x, m02 = u.x, m10 = r.y, m11 = f.y, m12 = u.y, m20 = r.z, m21 = f.z, m22 = u.z;
    double tr = m00 + m11 + m22;
    Quat q;
    if (tr > 0) {
        double s = std::sqrt(tr + 1.0) * 2;
        q = {0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s};
    } else if (m00 > m11 && m00 > m22) {
        double s = std::sqrt(1.0 + m00 - m11 - m22) * 2;
        q = {(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s};
    } else if (m11 > m22) {
        double s = std::sqrt(1.0 + m11 - m00 - m22) * 2;
        q = {(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s};
    } else {
        double s = std::sqrt(1.0 + m22 - m00 - m11) * 2;
        q = {(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s};
    }
    return q.normalized();
}

inline uint32_t hash32(uint64_t x) {
    x ^= x >> 33, x *= 0xff51afd7ed558ccdULL, x ^= x >> 33, x *= 0xc4ceb9fe1a85ec53ULL, x ^= x >> 33;
    return uint32_t(x);
}

}  // namespace tt
