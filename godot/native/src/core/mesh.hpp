// Mesh buffers per material, in the local frame (x east, y north, z up). The engine glue converts to its
// own axes. Vertex colour multiplies the material's albedo (tint).
#pragma once
#include <map>
#include <vector>
#include "util.hpp"

namespace ec {

enum Mat : int {
    M_TERRAIN = 0,
    M_ASPHALT = 1,
    M_PAVERS = 2,
    M_CURB = 3,
    M_MARKING = 4,
    M_GRASS = 5,
    M_FOREST = 6,
    M_SAND = 7,
    M_WATER = 8,
    M_PARKING = 9,
    M_GRAVEL = 10,
    M_FARM = 11,
    M_PITCH = 12,
    M_ROOF_FLAT = 20,
    M_ROOF_TILE = 21,
    M_ROOF_METAL = 22,
    M_CONCRETE = 23,
    M_PAINT = 24,
    M_GLASS = 25,
    M_METAL = 26,
    M_GOLD = 27,
    M_DARK = 28,
    M_LAMP = 29,
    M_BARK = 30,
    M_LEAVES = 31,
    M_CONIFER = 32,
    M_CARPAINT = 33,
    M_TIRE = 34,
    M_SIGNAL = 35,
    M_SCREEN = 36,  // LED advertising screens (animated in the shader, uv 0..1 over the screen)  // traffic light lamps (animated in the shader)
    M_FACADE = 100,  // + style * 3 + variant
};

constexpr int FACADE_STYLES = 13;
inline int facade_mat(int style, int variant) { return M_FACADE + style * 3 + variant; }

struct Col {
    float r = 1, g = 1, b = 1, a = 1;
};

struct MeshBuf {
    std::vector<float> pos, nrm, uv, col;  // 3, 3, 2, 4 per vertex
    std::vector<int> idx;
    size_t verts() const { return pos.size() / 3; }
    int v(V3 p, V3 n, double u, double w, Col c) {
        pos.insert(pos.end(), {(float)p.x, (float)p.y, (float)p.z});
        nrm.insert(nrm.end(), {(float)n.x, (float)n.y, (float)n.z});
        uv.insert(uv.end(), {(float)u, (float)w});
        col.insert(col.end(), {c.r, c.g, c.b, c.a});
        return (int)verts() - 1;
    }
    void tri(int a, int b, int c) { idx.insert(idx.end(), {a, b, c}); }
    // quad p0..p3 counter-clockwise seen from the normal side, with uvs
    void quad(V3 p0, V3 p1, V3 p2, V3 p3, V3 n, Col c, double u0 = 0, double v0 = 0, double u1 = 1, double v1 = 1) {
        int a = v(p0, n, u0, v0, c), b = v(p1, n, u1, v0, c), d = v(p2, n, u1, v1, c), e = v(p3, n, u0, v1, c);
        tri(a, b, d);
        tri(a, d, e);
    }
    // quad with the normal computed from the corners
    void quadn(V3 p0, V3 p1, V3 p2, V3 p3, Col c, double u0 = 0, double v0 = 0, double u1 = 1, double v1 = 1) {
        V3 n = (p1 - p0).cross(p3 - p0).norm();
        quad(p0, p1, p2, p3, n, c, u0, v0, u1, v1);
    }
    void box(V3 lo, V3 hi, Col c) { obox({(lo.x + hi.x) / 2, (lo.y + hi.y) / 2}, 0, hi.x - lo.x, hi.y - lo.y, lo.z, hi.z, c); }
    // box turned by yaw about z
    void obox(V2 ctr, double yaw, double l, double w, double z0, double z1, Col c, bool bottom = false) {
        double cs = std::cos(yaw), sn = std::sin(yaw);
        auto P = [&](double a, double b, double z) { return V3(ctr.x + a * cs - b * sn, ctr.y + a * sn + b * cs, z); };
        double hl = l / 2, hw = w / 2;
        V3 c0 = P(-hl, -hw, z0), c1 = P(hl, -hw, z0), c2 = P(hl, hw, z0), c3 = P(-hl, hw, z0);
        V3 d0 = P(-hl, -hw, z1), d1 = P(hl, -hw, z1), d2 = P(hl, hw, z1), d3 = P(-hl, hw, z1);
        quadn(c0, c1, d1, d0, c, 0, 0, l, z1 - z0);
        quadn(c1, c2, d2, d1, c, 0, 0, w, z1 - z0);
        quadn(c2, c3, d3, d2, c, 0, 0, l, z1 - z0);
        quadn(c3, c0, d0, d3, c, 0, 0, w, z1 - z0);
        quadn(d0, d1, d2, d3, c, 0, 0, l, w);
        if (bottom) quadn(c3, c2, c1, c0, c);
    }
    // vertical cylinder / cone frustum
    void cyl(V2 ctr, double r0, double r1, double z0, double z1, int seg, Col c, bool cap = true) {
        int base = (int)verts();
        double slope = (r0 - r1) / std::max(1e-6, z1 - z0);
        for (int i = 0; i <= seg; i++) {
            double a = 2 * PI * i / seg;
            V3 n = V3(std::cos(a), std::sin(a), slope).norm();
            v({ctr.x + std::cos(a) * r0, ctr.y + std::sin(a) * r0, z0}, n, (double)i / seg, 0, c);
            v({ctr.x + std::cos(a) * r1, ctr.y + std::sin(a) * r1, z1}, n, (double)i / seg, 1, c);
        }
        for (int i = 0; i < seg; i++) {
            int a = base + i * 2;
            tri(a, a + 2, a + 3);
            tri(a, a + 3, a + 1);
        }
        if (cap && r1 > 1e-4) {
            int ci = v({ctr.x, ctr.y, z1}, {0, 0, 1}, 0.5, 0.5, c);
            for (int i = 0; i < seg; i++) {
                double a0 = 2 * PI * i / seg, a1 = 2 * PI * (i + 1) / seg;
                int p = v({ctr.x + std::cos(a0) * r1, ctr.y + std::sin(a0) * r1, z1}, {0, 0, 1}, 0, 0, c);
                int q = v({ctr.x + std::cos(a1) * r1, ctr.y + std::sin(a1) * r1, z1}, {0, 0, 1}, 0, 0, c);
                tri(ci, p, q);
            }
        }
    }
    // surface of revolution: profile (radius, height) bottom to top
    void revolve(V2 ctr, double z0, const std::vector<std::pair<double, double>> &prof, int seg, Col c) {
        for (size_t k = 0; k + 1 < prof.size(); k++) {
            auto [r0, h0] = prof[k];
            auto [r1, h1] = prof[k + 1];
            double dr = r1 - r0, dz = h1 - h0;
            double l = std::max(1e-9, std::sqrt(dr * dr + dz * dz));
            double nr = dz / l, nz = -dr / l;
            int base = (int)verts();
            for (int i = 0; i <= seg; i++) {
                double a = 2 * PI * i / seg;
                V3 n(std::cos(a) * nr, std::sin(a) * nr, nz);
                v({ctr.x + std::cos(a) * r0, ctr.y + std::sin(a) * r0, z0 + h0}, n, (double)i / seg, h0, c);
                v({ctr.x + std::cos(a) * r1, ctr.y + std::sin(a) * r1, z0 + h1}, n, (double)i / seg, h1, c);
            }
            for (int i = 0; i < seg; i++) {
                int a = base + i * 2;
                tri(a, a + 2, a + 3);
                tri(a, a + 3, a + 1);
            }
        }
    }
    void append(const MeshBuf &o) {
        int base = (int)verts();
        pos.insert(pos.end(), o.pos.begin(), o.pos.end());
        nrm.insert(nrm.end(), o.nrm.begin(), o.nrm.end());
        uv.insert(uv.end(), o.uv.begin(), o.uv.end());
        col.insert(col.end(), o.col.begin(), o.col.end());
        for (int i : o.idx) idx.push_back(base + i);
    }
};

// a text sign placed in the world: outward normal angle in the local x/y plane, max width in metres
struct Label {
    V3 p;
    double ang = 0, max_w = 3;
    std::string text;
    Col c;
};

struct MeshSet {
    std::map<int, MeshBuf> m;
    MeshBuf &operator[](int mat) { return m[mat]; }
};

}  // namespace ec
