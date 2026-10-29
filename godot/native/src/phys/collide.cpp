#include "collide.hpp"
#include <algorithm>
#include <mutex>
#include "../core/chunk.hpp"

namespace ec {

namespace {
constexpr double CELL = 4.0;

V3 closest_on_tri(V3 p, V3 a, V3 b, V3 c) {
    // Ericson, Real-Time Collision Detection 5.1.5
    V3 ab = b - a, ac = c - a, ap = p - a;
    double d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0) return a;
    V3 bp = p - b;
    double d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3) return b;
    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    V3 cp = p - c;
    double d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6) return c;
    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    double den = 1 / (va + vb + vc);
    return a + ab * (vb * den) + ac * (vc * den);
}
}  // namespace

double layer_grip(int l) {
    switch (l) {
        case L_ASPHALT: return 1.0;
        case L_PAVERS: return 0.95;
        case L_GRASS: return 0.62;
        case L_FOREST: return 0.6;
        case L_SAND: return 0.5;
        case L_FARM: return 0.55;
        case L_DIRT: return 0.65;
        case L_PITCH: return 0.65;
        default: return 0.8;
    }
}

void ColChunk::index() {
    x0 = cx * CHUNK, y0 = cy * CHUNK;
    ncell = (int)std::ceil(CHUNK / CELL);
    cells.assign((size_t)ncell * ncell, {});
    for (uint32_t t = 0; t * 9 + 8 < tris.size(); t++) {
        const float *f = &tris[t * 9];
        double lx = std::min({f[0], f[3], f[6]}), hx = std::max({f[0], f[3], f[6]});
        double ly = std::min({f[1], f[4], f[7]}), hy = std::max({f[1], f[4], f[7]});
        int i0 = std::clamp((int)std::floor((lx - x0) / CELL), 0, ncell - 1), i1 = std::clamp((int)std::floor((hx - x0) / CELL), 0, ncell - 1);
        int j0 = std::clamp((int)std::floor((ly - y0) / CELL), 0, ncell - 1), j1 = std::clamp((int)std::floor((hy - y0) / CELL), 0, ncell - 1);
        for (int j = j0; j <= j1; j++)
            for (int i = i0; i <= i1; i++) cells[(size_t)j * ncell + i].push_back(t);
    }
}

void CollisionWorld::add(std::shared_ptr<ColChunk> c) {
    if (c->cells.empty()) c->index();
    std::unique_lock<std::shared_mutex> l(mu_);
    chunks_[{c->cx, c->cy}] = std::move(c);
}
void CollisionWorld::remove(int cx, int cy) {
    std::unique_lock<std::shared_mutex> l(mu_);
    chunks_.erase({cx, cy});
}
bool CollisionWorld::has(int cx, int cy) const {
    std::shared_lock<std::shared_mutex> l(mu_);
    return chunks_.count({cx, cy}) > 0;
}

const ColChunk *CollisionWorld::chunk_at(double x, double y) const {
    auto it = chunks_.find({(int)std::floor(x / CHUNK), (int)std::floor(y / CHUNK)});
    return it == chunks_.end() ? nullptr : it->second.get();
}

Surface CollisionWorld::surface(double x, double y, double z, double step) const {
    std::shared_lock<std::shared_mutex> l(mu_);
    Surface s;
    const ColChunk *c = chunk_at(x, y);
    if (!c) return s;
    s.height = c->grid.sample(x, y);
    s.normal = c->grid.normal(x, y);
    int ti = (int)std::floor(x - c->x0), tj = (int)std::floor(y - c->y0);
    if (ti >= 0 && tj >= 0 && ti < SPLAT_N && tj < SPLAT_N && !c->layer.empty()) s.grip = layer_grip(c->layer[(size_t)tj * SPLAT_N + ti]);
    s.found = true;
    int i = std::clamp((int)std::floor((x - c->x0) / CELL), 0, c->ncell - 1), j = std::clamp((int)std::floor((y - c->y0) / CELL), 0, c->ncell - 1);
    for (uint32_t t : c->cells[(size_t)j * c->ncell + i]) {
        const float *f = &c->tris[t * 9];
        V3 a(f[0], f[1], f[2]), b(f[3], f[4], f[5]), d(f[6], f[7], f[8]);
        V3 n = (b - a).cross(d - a);
        double nl = n.len();
        if (nl < 1e-9) continue;
        n = n * (1 / nl);
        if (n.z < 0) n = n * -1;
        if (n.z < 0.6) continue;
        // point in triangle (xy) by barycentrics
        double den = (b.y - d.y) * (a.x - d.x) + (d.x - b.x) * (a.y - d.y);
        if (std::abs(den) < 1e-12) continue;
        double w0 = ((b.y - d.y) * (x - d.x) + (d.x - b.x) * (y - d.y)) / den;
        double w1 = ((d.y - a.y) * (x - d.x) + (a.x - d.x) * (y - d.y)) / den;
        double w2 = 1 - w0 - w1;
        if (w0 < -1e-6 || w1 < -1e-6 || w2 < -1e-6) continue;
        double h = a.z * w0 + b.z * w1 + d.z * w2;
        if (h > s.height && h <= z + step) {
            s.height = h;
            s.normal = n;
            s.grip = 1.0;
        }
    }
    return s;
}

void CollisionWorld::sphere(V3 p, double r, std::vector<Hit> &out, bool terrain) const {
    out.clear();
    std::shared_lock<std::shared_mutex> l(mu_);
    // chunks touched by the sphere's box
    int ci0 = (int)std::floor((p.x - r) / CHUNK), ci1 = (int)std::floor((p.x + r) / CHUNK);
    int cj0 = (int)std::floor((p.y - r) / CHUNK), cj1 = (int)std::floor((p.y + r) / CHUNK);
    for (int cj = cj0; cj <= cj1; cj++)
        for (int ci = ci0; ci <= ci1; ci++) {
            auto it = chunks_.find({ci, cj});
            if (it == chunks_.end()) continue;
            const ColChunk *c = it->second.get();
            if (terrain && ci == (int)std::floor(p.x / CHUNK) && cj == (int)std::floor(p.y / CHUNK)) {
                double h = c->grid.sample(p.x, p.y);
                V3 n = c->grid.normal(p.x, p.y);
                double depth = r - (p.z - h) * n.z;
                if (depth > 0) {
                    int ti = (int)std::floor(p.x - c->x0), tj = (int)std::floor(p.y - c->y0);
                    double g = (ti >= 0 && tj >= 0 && ti < SPLAT_N && tj < SPLAT_N && !c->layer.empty()) ? layer_grip(c->layer[(size_t)tj * SPLAT_N + ti]) : 0.8;
                    out.push_back({n, depth, g});
                }
            }
            int i0 = std::clamp((int)std::floor((p.x - r - c->x0) / CELL), 0, c->ncell - 1), i1 = std::clamp((int)std::floor((p.x + r - c->x0) / CELL), 0, c->ncell - 1);
            int j0 = std::clamp((int)std::floor((p.y - r - c->y0) / CELL), 0, c->ncell - 1), j1 = std::clamp((int)std::floor((p.y + r - c->y0) / CELL), 0, c->ncell - 1);
            for (int j = j0; j <= j1; j++)
                for (int i = i0; i <= i1; i++)
                    for (uint32_t t : c->cells[(size_t)j * c->ncell + i]) {
                        const float *f = &c->tris[t * 9];
                        double lz = std::min({f[2], f[5], f[8]}), hz = std::max({f[2], f[5], f[8]});
                        if (p.z + r < lz || p.z - r > hz) continue;
                        V3 a(f[0], f[1], f[2]), b(f[3], f[4], f[5]), d(f[6], f[7], f[8]);
                        V3 q = closest_on_tri(p, a, b, d);
                        V3 dv = p - q;
                        double dist = dv.len();
                        if (dist >= r) continue;
                        V3 n;
                        if (dist > 1e-6) n = dv * (1 / dist);
                        else {
                            n = (b - a).cross(d - a).norm();
                        }
                        // the same triangle can sit in several buckets: keep one hit per triangle
                        bool dup = false;
                        for (auto &h : out)
                            if (std::abs(h.depth - (r - dist)) < 1e-9 && (h.n - n).len() < 1e-9) dup = true;
                        if (!dup) out.push_back({n, r - dist, 1.0});
                    }
        }
}

bool CollisionWorld::any_near(V3 lo, V3 hi) const {
    std::shared_lock<std::shared_mutex> l(mu_);
    int ci0 = (int)std::floor(lo.x / CHUNK), ci1 = (int)std::floor(hi.x / CHUNK);
    int cj0 = (int)std::floor(lo.y / CHUNK), cj1 = (int)std::floor(hi.y / CHUNK);
    for (int cj = cj0; cj <= cj1; cj++)
        for (int ci = ci0; ci <= ci1; ci++) {
            auto it = chunks_.find({ci, cj});
            if (it == chunks_.end()) continue;
            const ColChunk *c = it->second.get();
            int i0 = std::clamp((int)std::floor((lo.x - c->x0) / CELL), 0, c->ncell - 1), i1 = std::clamp((int)std::floor((hi.x - c->x0) / CELL), 0, c->ncell - 1);
            int j0 = std::clamp((int)std::floor((lo.y - c->y0) / CELL), 0, c->ncell - 1), j1 = std::clamp((int)std::floor((hi.y - c->y0) / CELL), 0, c->ncell - 1);
            for (int j = j0; j <= j1; j++)
                for (int i = i0; i <= i1; i++)
                    for (uint32_t t : c->cells[(size_t)j * c->ncell + i]) {
                        const float *f = &c->tris[t * 9];
                        double lz = std::min({f[2], f[5], f[8]}), hz = std::max({f[2], f[5], f[8]});
                        if (hz >= lo.z && lz <= hi.z) return true;
                    }
        }
    return false;
}

}  // namespace ec
