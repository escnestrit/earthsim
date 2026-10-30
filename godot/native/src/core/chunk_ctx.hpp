// Internal state shared by the parts of the chunk builder.
#pragma once
#include <unordered_map>
#include "chunk.hpp"

namespace ec {

enum Occ : uint8_t { O_ROAD = 1, O_BUILDING = 2, O_WATER = 4, O_PATH = 8, O_WALK = 16, O_PARKING = 32 };

// Segments of lines bucketed on a coarse grid, for "how far is the nearest road" questions.
struct SegIndex {
    struct Seg {
        V2 a, b;
        double hw;
        int id;
    };
    double cell = 16;
    std::unordered_map<uint64_t, std::vector<int>> b;
    std::vector<Seg> segs;
    static uint64_t key(int i, int j) { return ((uint64_t)(uint32_t)i << 32) | (uint32_t)j; }
    void add(V2 a, V2 c, double hw, int id) {
        int s = (int)segs.size();
        segs.push_back({a, c, hw, id});
        double r = hw + 2;
        int i0 = (int)std::floor((std::min(a.x, c.x) - r) / cell), i1 = (int)std::floor((std::max(a.x, c.x) + r) / cell);
        int j0 = (int)std::floor((std::min(a.y, c.y) - r) / cell), j1 = (int)std::floor((std::max(a.y, c.y) + r) / cell);
        for (int i = i0; i <= i1; i++)
            for (int j = j0; j <= j1; j++) b[key(i, j)].push_back(s);
    }
    // smallest (distance - half width); large when nothing is near
    double clearance(V2 p, int *id = nullptr) const {
        auto it = b.find(key((int)std::floor(p.x / cell), (int)std::floor(p.y / cell)));
        double best = 1e9;
        if (it == b.end()) return best;
        for (int s : it->second) {
            double d = seg_dist(p, segs[s].a, segs[s].b) - segs[s].hw;
            if (d < best) {
                best = d;
                if (id) *id = segs[s].id;
            }
        }
        return best;
    }
};

struct Ctx {
    int cx, cy;
    double x0, y0, x1, y1;
    const Dem &dem;
    const LocalFrame &fr;
    ChunkOut &out;
    std::vector<uint8_t> layer;  // SPLAT_N^2 ground-cover layer per texel
    std::vector<uint8_t> occ;    // SPLAT_N^2 Occ bits
    SegIndex roads;              // carriageways (hw = carriageway half width + sidewalk)
    SegIndex paths;
    std::vector<Ring> building_rings;  // outer rings of all buildings near the chunk (local)

    Ctx(int cx_, int cy_, const Dem &d, const LocalFrame &f, ChunkOut &o) : cx(cx_), cy(cy_), dem(d), fr(f), out(o) {
        x0 = cx * CHUNK, y0 = cy * CHUNK, x1 = x0 + CHUNK, y1 = y0 + CHUNK;
        layer.assign(SPLAT_N * SPLAT_N, L_GRASS);
        occ.assign(SPLAT_N * SPLAT_N, 0);
    }
    // natural ground (DEM) at a local point
    double G(V2 p) const {
        double la, lo;
        fr.to_latlon(p, la, lo);
        return dem.height(la, lo, 0);
    }
    // ground low-passed over ~35 m: what roads are laid on. The DEM (about 15-20 m cells, with buildings and
    // trees baked into it in towns) is bumpy at road scale, and roads following it raw drive like waves. Only
    // a function of position, so neighbouring chunks agree where a road crosses between them.
    double Gs(V2 p, double scale = 1) const {
        const double R1 = 9 * scale, R2 = 18 * scale;
        double sum = G(p) * 4, w = 4;
        for (int k = 0; k < 6; k++) {
            double a = k * PI / 3;
            sum += G(p + V2(std::cos(a), std::sin(a)) * R1) * 2, w += 2;
        }
        for (int k = 0; k < 8; k++) {
            double a = k * PI / 4 + PI / 8;
            sum += G(p + V2(std::cos(a), std::sin(a)) * R2), w += 1;
        }
        return sum / w;
    }
    // a solid box (yawed l x w footprint) in the static collision
    void coll_prism(V2 p, double yaw, double l, double w, double z0, double z1) {
        double cs = std::cos(yaw), sn = std::sin(yaw);
        V3 q[4];
        for (int k = 0; k < 4; k++) {
            double a = (k == 0 || k == 3) ? -l / 2 : l / 2, b = k < 2 ? -w / 2 : w / 2;
            q[k] = V3(p.x + a * cs - b * sn, p.y + a * sn + b * cs, 0);
        }
        for (int k = 0; k < 4; k++) {
            V3 a = q[k], b = q[(k + 1) % 4];
            V3 a0(a.x, a.y, z0), b0(b.x, b.y, z0), a1(a.x, a.y, z1), b1(b.x, b.y, z1);
            tri_coll(a0, b0, b1);
            tri_coll(a0, b1, a1);
        }
        V3 t0(q[0].x, q[0].y, z1), t1(q[1].x, q[1].y, z1), t2(q[2].x, q[2].y, z1), t3(q[3].x, q[3].y, z1);
        tri_coll(t0, t1, t2);
        tri_coll(t0, t2, t3);
    }
    bool own(V2 p) const { return p.x >= x0 && p.x < x1 && p.y >= y0 && p.y < y1; }
    bool near(V2 lo, V2 hi, double m) const { return !(hi.x < x0 - m || lo.x > x1 + m || hi.y < y0 - m || lo.y > y1 + m); }
    // ground after flattening and carving
    double ground(V2 p) const { return out.grid.sample(p.x, p.y); }
    int texel(V2 p) const {
        int i = (int)std::floor(p.x - x0), j = (int)std::floor(p.y - y0);
        if (i < 0 || j < 0 || i >= SPLAT_N || j >= SPLAT_N) return -1;
        return j * SPLAT_N + i;
    }
    uint8_t occ_at(V2 p) const {
        int t = texel(p);
        return t < 0 ? 0 : occ[t];
    }

    // even-odd fill of a polygon with holes at texel centres
    template <class F>
    void fill(const std::vector<Ring> &rings, F f) {
        double ylo = 1e300, yhi = -1e300;
        for (auto &r : rings)
            for (auto &p : r) ylo = std::min(ylo, p.y), yhi = std::max(yhi, p.y);
        int j0 = std::max(0, (int)std::floor(ylo - y0 - 0.5)), j1 = std::min(SPLAT_N - 1, (int)std::ceil(yhi - y0));
        std::vector<double> xs;
        for (int j = j0; j <= j1; j++) {
            double y = y0 + j + 0.5;
            xs.clear();
            for (auto &r : rings)
                for (size_t i = 0, n = r.size(); i < n; i++) {
                    V2 a = r[i], b = r[(i + 1) % n];
                    if ((a.y > y) != (b.y > y)) xs.push_back(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
                }
            std::sort(xs.begin(), xs.end());
            for (size_t k = 0; k + 1 < xs.size(); k += 2) {
                int i0 = std::max(0, (int)std::ceil(xs[k] - x0 - 0.5)), i1 = std::min(SPLAT_N - 1, (int)std::floor(xs[k + 1] - x0 - 0.5));
                for (int i = i0; i <= i1; i++) f(j * SPLAT_N + i);
            }
        }
    }
    // texels within `hw` of segment a-b
    template <class F>
    void stroke(V2 a, V2 b, double hw, F f) {
        int i0 = std::max(0, (int)std::floor(std::min(a.x, b.x) - hw - x0)), i1 = std::min(SPLAT_N - 1, (int)std::ceil(std::max(a.x, b.x) + hw - x0));
        int j0 = std::max(0, (int)std::floor(std::min(a.y, b.y) - hw - y0)), j1 = std::min(SPLAT_N - 1, (int)std::ceil(std::max(a.y, b.y) + hw - y0));
        for (int j = j0; j <= j1; j++)
            for (int i = i0; i <= i1; i++)
                if (seg_dist({x0 + i + 0.5, y0 + j + 0.5}, a, b) <= hw) f(j * SPLAT_N + i);
    }
    void tri_coll(V3 a, V3 b, V3 c) {
        out.coll.insert(out.coll.end(), {(float)a.x, (float)a.y, (float)a.z, (float)b.x, (float)b.y, (float)b.z, (float)c.x, (float)c.y, (float)c.z});
    }
};

// roads.cpp
void build_roads(Ctx &c, const std::vector<std::shared_ptr<const RawTile>> &tiles);

}  // namespace ec
