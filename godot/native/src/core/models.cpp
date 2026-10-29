#include "models.hpp"
#include <algorithm>
#include <functional>
#include "chunk.hpp"

namespace ec {

namespace {

float h01(uint64_t s, uint64_t k) { return (float)hash01(s, k); }

// tapered tube along a polyline of (point, radius)
void tube(MeshBuf &m, const std::vector<std::pair<V3, double>> &path, int seg, Col c) {
    if (path.size() < 2) return;
    int base = (int)m.verts();
    double vacc = 0;
    for (size_t k = 0; k < path.size(); k++) {
        V3 p = path[k].first;
        V3 d = (k + 1 < path.size() ? path[k + 1].first - p : p - path[k - 1].first).norm();
        if (k > 0) vacc += (p - path[k - 1].first).len();
        V3 a = std::abs(d.z) < 0.9 ? V3(0, 0, 1).cross(d).norm() : V3(1, 0, 0).cross(d).norm();
        V3 b = d.cross(a);
        for (int i = 0; i <= seg; i++) {
            double t = 2 * PI * i / seg;
            V3 n = a * std::cos(t) + b * std::sin(t);
            m.v(p + n * path[k].second, n, (double)i / seg, vacc / 1.5, c);
        }
    }
    for (size_t k = 0; k + 1 < path.size(); k++)
        for (int i = 0; i < seg; i++) {
            int a0 = base + (int)k * (seg + 1) + i, a1 = a0 + 1, b0 = a0 + seg + 1, b1 = b0 + 1;
            m.tri(a0, a1, b1);
            m.tri(a0, b1, b0);
        }
}

// a leaf card: square of `size` centred at c facing `n` (both sides drawn), normals bent outward from the crown
void card(MeshBuf &m, V3 c, V3 u, V3 w, double size, V3 crown, Col col) {
    V3 p0 = c - u * (size / 2) - w * (size / 2), p1 = c + u * (size / 2) - w * (size / 2), p2 = c + u * (size / 2) + w * (size / 2), p3 = c - u * (size / 2) + w * (size / 2);
    V3 ps[4] = {p0, p1, p2, p3};
    double uv[4][2] = {{0, 1}, {1, 1}, {1, 0}, {0, 0}};
    int ia[4], ib[4];
    for (int k = 0; k < 4; k++) {
        V3 n = (ps[k] - crown).norm();
        n = (n + V3(0, 0, 0.35)).norm();
        ia[k] = m.v(ps[k], n, uv[k][0], uv[k][1], col);
        ib[k] = m.v(ps[k], n, uv[k][0], uv[k][1], col);
    }
    m.tri(ia[0], ia[1], ia[2]);
    m.tri(ia[0], ia[2], ia[3]);
    m.tri(ib[0], ib[2], ib[1]);
    m.tri(ib[0], ib[3], ib[2]);
}

void cluster(MeshBuf &m, V3 c, double size, V3 crown, uint64_t seed, Col col) {
    // three crossed cards at random orientations
    for (int k = 0; k < 3; k++) {
        double yaw = h01(seed, k * 3) * 2 * PI, pitch = (h01(seed, k * 3 + 1) - 0.5) * 1.2;
        V3 u(std::cos(yaw), std::sin(yaw), 0);
        V3 w = V3(-std::sin(yaw) * std::cos(pitch), std::cos(yaw) * std::cos(pitch), std::sin(pitch));
        if (k == 2) w = V3(0, 0, 1);
        card(m, c, u, w, size * (0.8 + 0.4 * h01(seed, k * 3 + 2)), crown, col);
    }
}

MeshSet broadleaf(int v) {
    MeshSet ms;
    MeshBuf &B = ms[M_BARK], &L = ms[M_LEAVES];
    uint64_t seed = 1000 + v * 77;
    double H = 9 + v * 2.2, trunk_h = H * (0.32 + 0.08 * h01(seed, 1)), r0 = 0.18 + v * 0.04;
    Col bark{1, 1, 1, 1};
    Col leaf{1, 1, 1, 1};
    // slight lean and bend in the trunk
    V3 lean(h01(seed, 2) * 0.4 - 0.2, h01(seed, 3) * 0.4 - 0.2, 0);
    std::vector<std::pair<V3, double>> tr;
    for (int k = 0; k <= 6; k++) {
        double t = k / 6.0;
        tr.push_back({V3(lean.x * t * t * H * 0.1, lean.y * t * t * H * 0.1, t * H * 0.85), r0 * (1 - 0.75 * t) * (k == 0 ? 1.35 : 1)});
    }
    tube(B, tr, 8, bark);
    V3 crown(lean.x * 0.08 * H, lean.y * 0.08 * H, trunk_h + (H - trunk_h) * 0.5);
    double crown_r = (H - trunk_h) * 0.55;
    // main limbs from the upper trunk, then twigs ending in leaf clusters
    int limbs = 5 + v;
    for (int i = 0; i < limbs; i++) {
        double t0 = trunk_h / H * 0.85 + (0.8 - trunk_h / H * 0.85) * i / limbs;
        V3 start = tr[std::min<size_t>(6, (size_t)std::lround(t0 * 6))].first;
        start.z = t0 * H * 0.85;
        double yaw = i * 2.4 + h01(seed, 10 + i) * 0.8;
        double up = 0.45 + h01(seed, 20 + i) * 0.5;
        V3 dir = V3(std::cos(yaw), std::sin(yaw), up).norm();
        double len = crown_r * (0.9 + 0.3 * h01(seed, 30 + i));
        V3 mid = start + dir * (len * 0.5) + V3(0, 0, len * 0.1), end = start + dir * len + V3(0, 0, len * 0.15);
        tube(B, {{start, r0 * 0.45}, {mid, r0 * 0.28}, {end, r0 * 0.12}}, 5, bark);
        int twigs = 3 + (int)(h01(seed, 40 + i) * 3);
        for (int k = 0; k < twigs; k++) {
            double s = 0.35 + 0.65 * (k + 1) / twigs;
            V3 p = start + (end - start) * s;
            double ty = yaw + (h01(seed, 50 + i * 7 + k) - 0.5) * 2.2;
            V3 td = V3(std::cos(ty), std::sin(ty), 0.3 + h01(seed, 60 + i * 7 + k)).norm();
            V3 q = p + td * (crown_r * 0.35);
            tube(B, {{p, r0 * 0.08}, {q, r0 * 0.03}}, 3, bark);
            cluster(L, q, crown_r * 0.75, crown, seed * 31 + i * 7 + k, leaf);
            cluster(L, (p + q) * 0.5, crown_r * 0.6, crown, seed * 37 + i * 7 + k, leaf);
        }
    }
    // fill the crown's core so it doesn't look hollow
    for (int k = 0; k < 6; k++) {
        double a = k * 1.047 + h01(seed, 90 + k);
        V3 p = crown + V3(std::cos(a) * crown_r * 0.4, std::sin(a) * crown_r * 0.4, (h01(seed, 95 + k) - 0.3) * crown_r * 0.6);
        cluster(L, p, crown_r * 0.8, crown, seed * 41 + k, leaf);
    }
    cluster(L, crown + V3(0, 0, crown_r * 0.7), crown_r * 0.9, crown, seed * 43, leaf);
    return ms;
}

MeshSet conifer(int v) {
    MeshSet ms;
    MeshBuf &B = ms[M_BARK], &L = ms[M_CONIFER];
    uint64_t seed = 2000 + v * 91;
    double H = 12 + v * 4, r0 = 0.2 + v * 0.05;
    tube(B, {{V3(0, 0, 0), r0 * 1.3}, {V3(0, 0, H * 0.3), r0}, {V3(0, 0, H), 0.03}}, 7, {1, 1, 1, 1});
    int whorls = 9 + v * 3;
    double base = H * (0.12 + 0.05 * v);
    for (int k = 0; k < whorls; k++) {
        double t = double(k) / whorls;
        double z = base + (H - base) * t;
        double rad = (H - base) * 0.32 * (1 - t) + 0.4;
        int arms = std::max(3, (int)std::lround(7 - t * 3));
        for (int a = 0; a < arms; a++) {
            double yaw = a * 2 * PI / arms + k * 0.7 + h01(seed, k * 13 + a) * 0.5;
            V3 d(std::cos(yaw), std::sin(yaw), 0);
            V3 p = V3(0, 0, z) + d * (rad * 0.5) + V3(0, 0, -rad * 0.12);
            V3 side(-d.y, d.x, 0);
            // a drooping card along the branch
            V3 droop = (d + V3(0, 0, -0.35)).norm();
            V3 crown(0, 0, z + rad * 0.3);
            double s = rad * 1.05;
            V3 p0 = V3(0, 0, z) - side * (s * 0.45), p1 = V3(0, 0, z) + side * (s * 0.45);
            V3 p2 = p1 + droop * s, p3 = p0 + droop * s;
            V3 pts[4] = {p0, p1, p2, p3};
            double uv[4][2] = {{0, 0.5}, {0, 0.5}, {1, 1}, {1, 0}};
            uv[0][0] = 0.0, uv[0][1] = 0.0;
            uv[1][0] = 0.0, uv[1][1] = 1.0;
            int ia[4], ib[4];
            for (int q = 0; q < 4; q++) {
                V3 n = ((pts[q] - crown).norm() + V3(0, 0, 0.6)).norm();
                ia[q] = L.v(pts[q], n, uv[q][0] * 0.5 + 0.25, uv[q][1], {1, 1, 1, 1});
                ib[q] = L.v(pts[q], n, uv[q][0] * 0.5 + 0.25, uv[q][1], {1, 1, 1, 1});
            }
            L.tri(ia[0], ia[1], ia[2]);
            L.tri(ia[0], ia[2], ia[3]);
            L.tri(ib[0], ib[2], ib[1]);
            L.tri(ib[0], ib[3], ib[2]);
            (void)p;
        }
    }
    // top tuft
    cluster(L, V3(0, 0, H - 0.6), 1.4, V3(0, 0, H - 1.5), seed + 5, {1, 1, 1, 1});
    return ms;
}

MeshSet bush(int v) {
    MeshSet ms;
    MeshBuf &L = ms[M_LEAVES];
    uint64_t seed = 3000 + v * 13;
    double r = 0.9 + v * 0.4;
    V3 crown(0, 0, r * 0.7);
    for (int k = 0; k < 7; k++) {
        double a = k * 0.9 + h01(seed, k);
        V3 p = crown + V3(std::cos(a) * r * 0.45, std::sin(a) * r * 0.45, (h01(seed, 10 + k) - 0.4) * r * 0.5);
        cluster(L, p, r * 1.1, crown, seed * 7 + k, {1, 1, 1, 1});
    }
    return ms;
}

MeshSet lamp() {
    MeshSet ms;
    MeshBuf &M = ms[M_METAL];
    Col g{0.32f, 0.34f, 0.36f, 1};
    M.cyl({0, 0}, 0.13, 0.13, 0, 0.6, 10, g, false);
    M.cyl({0, 0}, 0.085, 0.06, 0.6, 8.0, 10, g, true);
    // curved arm toward the road (+x) and the head
    MeshBuf &A = ms[M_METAL];
    std::vector<std::pair<V3, double>> arm;
    for (int k = 0; k <= 6; k++) {
        double t = k / 6.0;
        arm.push_back({V3(t * 1.6, 0, 7.6 + std::sin(t * PI * 0.6) * 0.5), 0.045});
    }
    tube(A, arm, 6, g);
    M.obox({1.75, 0}, 0, 0.7, 0.32, 7.85, 8.05, g, true);
    ms[M_LAMP].quad({1.42, -0.13, 7.84}, {1.42, 0.13, 7.84}, {2.08, 0.13, 7.84}, {2.08, -0.13, 7.84}, {0, 0, -1}, {1, 1, 1, 1});
    return ms;
}

MeshSet car(bool with_wheels) {
    MeshSet ms;
    MeshBuf &P = ms[M_CARPAINT], &G = ms[M_GLASS], &T = ms[M_TIRE], &D = ms[M_DARK], &Lm = ms[M_LAMP];
    Col w{1, 1, 1, 1};
    // side profile (x forward, z up) of a compact hatchback, extruded across the width with rounded sides
    std::vector<V2> lower = {{-2.15, 0.32}, {2.15, 0.32}, {2.25, 0.55}, {2.2, 0.82}, {1.2, 0.98}, {-1.6, 1.0}, {-2.15, 0.95}, {-2.25, 0.6}};
    double hw = 0.88;
    auto extrude = [&](MeshBuf &m, const std::vector<V2> &prof, double half, double inset_top, Col c) {
        size_t n = prof.size();
        // sides
        for (int s : {-1, 1}) {
            int base = (int)m.verts();
            double y = s * half;
            for (auto &p : prof) {
                double ins = std::max(0.0, (p.y - 0.8)) * inset_top;
                m.v({p.x, y - s * ins, p.y}, {0, (double)s, 0}, p.x, p.y, c);
            }
            for (size_t i = 1; i + 1 < n; i++) {
                if (s > 0) m.tri(base, base + (int)i + 1, base + (int)i);
                else m.tri(base, base + (int)i, base + (int)i + 1);
            }
        }
        // around the profile
        for (size_t i = 0; i < n; i++) {
            V2 a = prof[i], b = prof[(i + 1) % n];
            double ia = std::max(0.0, (a.y - 0.8)) * inset_top, ib = std::max(0.0, (b.y - 0.8)) * inset_top;
            V3 p0(a.x, -half + ia, a.y), p1(b.x, -half + ib, b.y), p2(b.x, half - ib, b.y), p3(a.x, half - ia, a.y);
            V3 nrm = V3(b.y - a.y, 0, -(b.x - a.x)).norm();
            m.quad(p0, p3, p2, p1, nrm, c);
        }
    };
    extrude(P, lower, hw, 0.0, w);
    std::vector<V2> cabin = {{-1.55, 0.98}, {1.15, 0.97}, {0.35, 1.42}, {-1.35, 1.45}, {-1.75, 1.2}};
    extrude(G, cabin, hw - 0.08, 0.35, w);
    // roof cap over the glass
    P.obox({-0.5, 0}, 0, 1.6, (hw - 0.08 - 0.22) * 2, 1.44, 1.47, w);
    // wheels
    for (double x : {-1.35, 1.35})
        for (double y : {-0.78, 0.78}) {
            if (!with_wheels) continue;
            int base = (int)T.verts();
            int seg = 14;
            double r = 0.31, half = 0.1;
            for (int i = 0; i <= seg; i++) {
                double a = 2 * PI * i / seg;
                V3 n(std::cos(a), 0, std::sin(a));
                T.v({x + std::cos(a) * r, y - half, 0.31 + std::sin(a) * r}, n, 0, 0, w);
                T.v({x + std::cos(a) * r, y + half, 0.31 + std::sin(a) * r}, n, 0, 0, w);
            }
            for (int i = 0; i < seg; i++) {
                int a0 = base + i * 2;
                T.tri(a0, a0 + 3, a0 + 2);
                T.tri(a0, a0 + 1, a0 + 3);
            }
            double ys = y > 0 ? y + half : y - half;
            int ci = D.v({x, ys, 0.31}, {0, y > 0 ? 1.0 : -1.0, 0}, 0, 0, Col{0.6f, 0.6f, 0.62f, 1});
            for (int i = 0; i < seg; i++) {
                double a0 = 2 * PI * i / seg, a1 = 2 * PI * (i + 1) / seg;
                int p = D.v({x + std::cos(a0) * r * 0.65, ys, 0.31 + std::sin(a0) * r * 0.65}, {0, y > 0 ? 1.0 : -1.0, 0}, 0, 0, Col{0.6f, 0.6f, 0.62f, 1});
                int q = D.v({x + std::cos(a1) * r * 0.65, ys, 0.31 + std::sin(a1) * r * 0.65}, {0, y > 0 ? 1.0 : -1.0, 0}, 0, 0, Col{0.6f, 0.6f, 0.62f, 1});
                if (y > 0) D.tri(ci, q, p);
                else D.tri(ci, p, q);
            }
        }
    // lights and grille
    for (double y : {-0.6, 0.6}) {
        Lm.quad({2.24, y - 0.18, 0.62}, {2.24, y + 0.18, 0.62}, {2.21, y + 0.18, 0.76}, {2.21, y - 0.18, 0.76}, {1, 0, 0}, Col{1, 1, 1, 1});
        D.quad({-2.24, y + 0.16, 0.7}, {-2.24, y - 0.16, 0.7}, {-2.21, y - 0.16, 0.86}, {-2.21, y + 0.16, 0.86}, {-1, 0, 0}, Col{0.7f, 0.05f, 0.04f, 1});
    }
    D.quad({2.255, -0.4, 0.4}, {2.255, 0.4, 0.4}, {2.255, 0.4, 0.55}, {2.255, -0.4, 0.55}, {1, 0, 0}, Col{0.15f, 0.15f, 0.16f, 1});
    return ms;
}

}  // namespace

MeshSet model_mesh(int kind, int v) {
    switch (kind) {
        case I_TREE: return broadleaf(v % TREE_VARIANTS);
        case I_CONIFER: return conifer(v % CONIFER_VARIANTS);
        case I_BUSH: return bush(v % BUSH_VARIANTS);
        case I_LAMP: return lamp();
        case I_CAR: return car(v == 0);
        default: return MeshSet();
    }
}

}  // namespace ec
