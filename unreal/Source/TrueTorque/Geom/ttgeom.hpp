// Car body geometry and visual damage, plain C++ (no engine types) so it is tested and previewed
// outside Unreal. Coordinates are Unreal's car space: x forward, y right, z up, centimetres, z = 0 on
// the ground. A triangle (a, b, c) faces the viewer when cross(c - a, b - a) points at the viewer
// (Unreal's clockwise front faces); tri() orients every triangle from an "outward" hint.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ttg {

struct V {
    double x = 0, y = 0, z = 0;
    V() = default;
    V(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
    V operator+(const V &o) const { return {x + o.x, y + o.y, z + o.z}; }
    V operator-(const V &o) const { return {x - o.x, y - o.y, z - o.z}; }
    V operator*(double s) const { return {x * s, y * s, z * s}; }
    V operator-() const { return {-x, -y, -z}; }
    V &operator+=(const V &o) { x += o.x, y += o.y, z += o.z; return *this; }
};
inline double dot(const V &a, const V &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V cross(const V &a, const V &b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double len(const V &a) { return std::sqrt(dot(a, a)); }
inline V norm(const V &a) {
    double l = len(a);
    return l > 1e-12 ? a * (1.0 / l) : V{0, 0, 0};
}
inline V lerp(const V &a, const V &b, double t) { return a + (b - a) * t; }

enum Slot : uint8_t { PAINT = 0, GLASS = 1, TRIM = 2, HEAD = 3, TAIL = 4, NUM_SLOTS = 5 };
enum WheelSlot : uint8_t { TYRE = 0, RIM = 1 };

// smooth 3D value noise in [-1, 1]
double noise3(const V &p);

struct Tri {
    int a, b, c;
    uint8_t slot;
    bool smooth;
};

struct Mesh {
    std::vector<V> base, off;      // rest position and accumulated dent
    std::vector<float> scrape;     // 0 paint .. 1 bare metal / cracked glass
    std::vector<Tri> tris;
    // imported meshes keep their own normals and UVs; points at the same position form a weld group
    // (UV seams and hard edges split vertices, dents must still move them together)
    std::vector<V> nrm0;
    std::vector<float> u0, v0;
    std::vector<int> group;
    int add_vertex(const V &p, const V &n, float u, float v) {
        nrm0.push_back(n), u0.push_back(u), v0.push_back(v);
        return add(p);
    }
    void build_groups();  // call once after all vertices are added (imported meshes)

    int add(const V &p) {
        base.push_back(p);
        off.push_back({0, 0, 0});
        scrape.push_back(0.f);
        return int(base.size()) - 1;
    }
    V p(int i) const { return base[i] + off[i]; }
    void tri(int a, int b, int c, uint8_t slot, bool smooth, const V &outward);
    void quad(int a, int b, int c, int d, uint8_t slot, bool smooth, const V &outward) {
        tri(a, b, c, slot, smooth, outward);
        tri(a, c, d, slot, smooth, outward);
    }
    // push the metal near `centre` along `dir` (unit); depth/radius in cm. Returns total movement (cm).
    double dent(const V &centre, const V &dir, double depth, double radius, double max_offset, double seed);
    // paint scraped off near `centre` (sliding contact)
    void scratch(const V &centre, double radius, double amount);
    void repair(double fraction);  // 1 = as new
    void point_normals(std::vector<V> &out) const;  // area-weighted, smooth triangles only
    V centre() const;
    void bounds(V &lo, V &hi) const;
};

struct CarGeom {
    Mesh body, hood, trunk, bumper_f, bumper_r;
};

// dimensions shared with the physics
constexpr double WHEELBASE_HALF = 130, TRACK_HALF = 77, WHEEL_R = 33, WHEEL_W = 22;

void build_car(CarGeom &g);
void build_wheel(Mesh &m, double r = WHEEL_R, double width = WHEEL_W);  // axle along y, outer face +y

}  // namespace ttg
