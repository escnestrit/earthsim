// Basic math and geodesy: 2D/3D vectors, Web Mercator tiles, the local metric frame.
// Local frame: x east, y north, z up (metres). Godot frame: (x, z, -y).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace es {

constexpr double PI = 3.14159265358979323846;
constexpr double EARTH_CIRC = 40075016.686;  // equatorial circumference, m

struct V2 {
    double x = 0, y = 0;
    V2() = default;
    constexpr V2(double x_, double y_) : x(x_), y(y_) {}
    V2 operator+(V2 o) const { return {x + o.x, y + o.y}; }
    V2 operator-(V2 o) const { return {x - o.x, y - o.y}; }
    V2 operator*(double s) const { return {x * s, y * s}; }
    V2 operator/(double s) const { return {x / s, y / s}; }
    V2 &operator+=(V2 o) { x += o.x, y += o.y; return *this; }
    V2 &operator-=(V2 o) { x -= o.x, y -= o.y; return *this; }
    bool operator==(V2 o) const { return x == o.x && y == o.y; }
};
inline double dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(V2 a, V2 b) { return a.x * b.y - a.y * b.x; }
inline double len(V2 a) { return std::sqrt(dot(a, a)); }
inline double dist(V2 a, V2 b) { return len(a - b); }
inline V2 norm(V2 a) {
    double l = len(a);
    return l > 1e-12 ? a / l : V2{0, 0};
}
inline V2 perp(V2 a) { return {-a.y, a.x}; }  // rotate +90 (to the left)
inline V2 lerp(V2 a, V2 b, double t) { return a + (b - a) * t; }

struct V3 {
    double x = 0, y = 0, z = 0;
    V3() = default;
    constexpr V3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    V3(V2 p, double z_) : x(p.x), y(p.y), z(z_) {}
    V3 operator+(V3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(V3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(double s) const { return {x * s, y * s, z * s}; }
    V2 xy() const { return {x, y}; }
};
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 norm(V3 a) {
    double l = std::sqrt(dot(a, a));
    return l > 1e-12 ? a * (1.0 / l) : V3{0, 0, 1};
}

inline double clampd(double v, double a, double b) { return v < a ? a : v > b ? b : v; }
inline double smoothstep(double e0, double e1, double x) {
    double t = clampd((x - e0) / (e1 - e0), 0, 1);
    return t * t * (3 - 2 * t);
}
inline double lerpd(double a, double b, double t) { return a + (b - a) * t; }

// Web Mercator in unit square: mx in [0,1) west->east, my in [0,1) north->south.
inline double merc_x(double lon) { return (lon + 180.0) / 360.0; }
inline double merc_y(double lat) {
    double s = std::sin(lat * PI / 180.0);
    return 0.5 - std::log((1 + s) / (1 - s)) / (4 * PI);
}
inline double merc_lat(double my) { return std::atan(std::sinh(PI * (1 - 2 * my))) * 180.0 / PI; }
inline double merc_lon(double mx) { return mx * 360.0 - 180.0; }

struct TileId {
    int z = 0, x = 0, y = 0;
    bool operator==(const TileId &o) const { return z == o.z && x == o.x && y == o.y; }
    bool operator<(const TileId &o) const { return z != o.z ? z < o.z : x != o.x ? x < o.x : y < o.y; }
};

// Local metric frame around (lat0, lon0). Mercator is conformal, so near the origin it is a plain
// scaled copy of the ground; scale error at 5 km is ~0.1%.
struct Frame {
    double lat0 = 0, lon0 = 0, mx0 = 0, my0 = 0, m_per_unit = 1;
    Frame() = default;
    Frame(double lat, double lon) : lat0(lat), lon0(lon), mx0(merc_x(lon)), my0(merc_y(lat)), m_per_unit(EARTH_CIRC * std::cos(lat * PI / 180.0)) {}
    V2 from_merc(double mx, double my) const { return {(mx - mx0) * m_per_unit, -(my - my0) * m_per_unit}; }
    V2 from_ll(double lat, double lon) const { return from_merc(merc_x(lon), merc_y(lat)); }
    void to_merc(V2 p, double &mx, double &my) const { mx = mx0 + p.x / m_per_unit, my = my0 - p.y / m_per_unit; }
    void to_ll(V2 p, double &lat, double &lon) const {
        double mx, my;
        to_merc(p, mx, my);
        lat = merc_lat(my), lon = merc_lon(mx);
    }
    // tile-local integer coordinates (extent) -> local metres
    V2 from_tile(const TileId &t, double ext, double px, double py) const {
        double n = double(1 << t.z);
        return from_merc((t.x + px / ext) / n, (t.y + py / ext) / n);
    }
    // local rectangle of a tile: min (west, south), max (east, north)
    void tile_rect(const TileId &t, V2 &mn, V2 &mx) const {
        V2 a = from_tile(t, 1, 0, 0), b = from_tile(t, 1, 1, 1);
        mn = {a.x, b.y}, mx = {b.x, a.y};
    }
};

struct Rect {
    V2 mn{1e300, 1e300}, mx{-1e300, -1e300};
    void add(V2 p) { mn.x = std::min(mn.x, p.x), mn.y = std::min(mn.y, p.y), mx.x = std::max(mx.x, p.x), mx.y = std::max(mx.y, p.y); }
    bool contains(V2 p) const { return p.x >= mn.x && p.x <= mx.x && p.y >= mn.y && p.y <= mx.y; }
    bool overlaps(const Rect &o) const { return !(o.mn.x > mx.x || o.mx.x < mn.x || o.mn.y > mx.y || o.mx.y < mn.y); }
    Rect grown(double d) const { return {{mn.x - d, mn.y - d}, {mx.x + d, mx.y + d}}; }
    bool valid() const { return mn.x <= mx.x; }
};

using Poly = std::vector<V2>;  // open ring

double poly_area(const Poly &p);  // signed, CCW positive (y north)
bool point_in_poly(const Poly &p, V2 q);
V2 poly_centroid(const Poly &p);
// distance from q to segment ab and the parameter t of the closest point
double seg_dist(V2 q, V2 a, V2 b, double *t = nullptr);
// clip a polygon to an axis-aligned rectangle (Sutherland-Hodgman)
Poly clip_poly(const Poly &p, const Rect &r);
// clip a polyline to a rectangle; may return several pieces
std::vector<std::vector<V2>> clip_line(const std::vector<V2> &l, const Rect &r);
// segment intersection: returns true and parameters if segments ab, cd cross
bool seg_intersect(V2 a, V2 b, V2 c, V2 d, double &t, double &u);
// intersection of infinite lines p+t*r and q+u*s
bool line_intersect(V2 p, V2 r, V2 q, V2 s, double &t, double &u);
double polyline_length(const std::vector<V2> &l);
// point at arc length s along polyline; dir receives the unit tangent
V2 polyline_at(const std::vector<V2> &l, double s, V2 *dir = nullptr);
// 32-bit hash for deterministic variety
inline uint32_t hash32(uint64_t x) {
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33;
    return uint32_t(x);
}
inline double hash01(uint64_t x) { return (hash32(x) & 0xffffff) / double(0x1000000); }
uint64_t hash_str(const char *s, size_t n);

}  // namespace es
