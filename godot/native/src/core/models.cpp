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

// A compact crossover body lofted from smooth cross-sections (superellipses) along its length, with a
// tumblehome greenhouse, wheel arches, glazing, lights and trim picked per face.
MeshSet car(bool with_wheels) {
    MeshSet ms;
    MeshBuf &P = ms[M_CARPAINT], &G = ms[M_GLASS], &T = ms[M_TIRE], &D = ms[M_DARK], &Lm = ms[M_LAMP];
    Col w{1, 1, 1, 1};
    const double X0 = -2.25, X1 = 2.25;
    const double WX[2] = {1.35, -1.35};
    auto smooth = [](double e0, double e1, double x) {
        double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
        return t * t * (3 - 2 * t);
    };
    auto half_w = [&](double x) { return 0.91 - 0.13 * std::pow(std::abs(x) / 2.25, 5); };
    auto bottom = [&](double x) { return 0.34 + std::max(0.0, x - 1.85) * 0.55 + std::max(0.0, -1.95 - x) * 0.7; };
    auto top = [&](double x) {
        // tail, rear window, roof, windscreen, hood, nose
        if (x < -2.1) return 0.96 + 0.16 * smooth(-2.25, -2.1, x);
        if (x < -1.5) return 1.12 + 0.46 * smooth(-2.1, -1.5, x);
        if (x < 0.3) return 1.58 + 0.03 * smooth(-1.5, -0.6, x) - 0.01 * smooth(-0.6, 0.3, x);
        if (x < 1.15) return 1.6 - 0.56 * smooth(0.3, 1.15, x);
        if (x < 2.05) return 1.04 - 0.08 * smooth(1.15, 2.05, x);
        return 0.96 - 0.22 * smooth(2.05, 2.25, x);
    };
    auto belt = [&](double x) { return 1.02 + 0.05 * smooth(0.5, -1.8, x); };
    auto arch = [&](double x) {
        double z = 0;
        for (double wx : WX) {
            double dx = x - wx, r = 0.43;
            if (std::abs(dx) < r) z = std::max(z, 0.33 + std::sqrt(r * r - dx * dx));
        }
        return z;
    };
    const int NX = 48, NS = 40;
    std::vector<V3> pts((NX + 1) * NS);
    for (int i = 0; i <= NX; i++) {
        double t = double(i) / NX;
        double x = X0 + (X1 - X0) * (0.5 - 0.5 * std::cos(t * PI));  // denser at the ends
        double zb = bottom(x), zt = top(x), zbelt = std::min(belt(x), zt - 0.02), hwid = half_w(x);
        double zc = (zb + zt) / 2, hz = (zt - zb) / 2;
        for (int k = 0; k < NS; k++) {
            double a = 2 * PI * k / NS;
            double c = std::cos(a), sn = std::sin(a);
            double n = 5.0;
            double y = hwid * (c < 0 ? -1 : 1) * std::pow(std::abs(c), 2 / n);
            double z = zc + hz * (sn < 0 ? -1 : 1) * std::pow(std::abs(sn), 2 / n);
            if (z > zbelt) {
                double f = (z - zbelt) / std::max(0.05, zt - zbelt);
                y *= 1 - 0.2 * f;  // tumblehome
            }
            if (z < zc && std::abs(y) > 0.62 * hwid) z = std::max(z, arch(x));
            pts[i * NS + k] = V3(x, y, z);
        }
    }
    auto P_ = [&](int i, int k) { return pts[std::clamp(i, 0, NX) * NS + ((k % NS) + NS) % NS]; };
    auto nrm = [&](int i, int k) {
        V3 dx = P_(i + 1, k) - P_(i - 1, k), dk = P_(i, k + 1) - P_(i, k - 1);
        V3 n = dk.cross(dx);
        if (n.len() < 1e-9) return V3(i == 0 ? -1 : 1, 0, 0);
        return n.norm();
    };
    for (int i = 0; i < NX; i++)
        for (int k = 0; k < NS; k++) {
            V3 a = P_(i, k), b = P_(i + 1, k), c = P_(i + 1, k + 1), d = P_(i, k + 1);
            V3 m = (a + b + c + d) * 0.25;
            double x = m.x, zb = bottom(x), zt = top(x), zbelt = belt(x);
            V3 fn = nrm(i, k) + nrm(i + 1, k + 1);
            fn = fn.norm();
            MeshBuf *mb = &P;
            Col col = w;
            bool upper = m.z > zbelt + 0.03 && m.z < zt - 0.04;
            bool cabin = x > -2.1 && x < 1.12;
            bool pillar = std::abs(x - 1.1) < 0.06 || std::abs(x + 0.35) < 0.05 || std::abs(x + 1.55) < 0.12;
            if (upper && cabin && !(std::abs(fn.z) > 0.85 && x < 0.3 && x > -1.5) && !(pillar && std::abs(fn.y) > 0.5)) mb = &G;
            else if (m.z < zb + 0.12 || (x > 2.12 && m.z < 0.55) || (x < -2.15 && m.z < 0.6)) mb = &D, col = {0.35f, 0.35f, 0.37f, 1};
            else if (x > 2.08 && m.z > 0.66 && m.z < 0.84 && std::abs(m.y) > 0.42) mb = &Lm;
            else if (x < -2.12 && m.z > 0.86 && m.z < 1.05 && std::abs(m.y) > 0.45) mb = &D, col = {0.75f, 0.06f, 0.05f, 1};
            else if (x > 2.15 && m.z > 0.5 && m.z < 0.66 && std::abs(m.y) < 0.45) mb = &D, col = {0.12f, 0.12f, 0.13f, 1};
            (void)zt;
            int ia = mb->v(a, nrm(i, k), a.x / 2, a.z / 2, col), ib = mb->v(b, nrm(i + 1, k), b.x / 2, b.z / 2, col);
            int ic = mb->v(c, nrm(i + 1, k + 1), c.x / 2, c.z / 2, col), id = mb->v(d, nrm(i, k + 1), d.x / 2, d.z / 2, col);
            // outward facing: section points run counter-clockwise seen from +x
            mb->tri(ia, id, ic);
            mb->tri(ia, ic, ib);
        }
    // end caps
    for (int e = 0; e < 2; e++) {
        int i = e == 0 ? 0 : NX;
        V3 ctr;
        for (int k = 0; k < NS; k++) ctr = ctr + P_(i, k);
        ctr = ctr * (1.0 / NS);
        V3 n(e == 0 ? -1 : 1, 0, 0);
        int ci = D.v(ctr, n, 0, 0, Col{0.3f, 0.3f, 0.32f, 1});
        for (int k = 0; k < NS; k++) {
            int a = D.v(P_(i, k), n, 0, 0, Col{0.3f, 0.3f, 0.32f, 1}), b = D.v(P_(i, k + 1), n, 0, 0, Col{0.3f, 0.3f, 0.32f, 1});
            if (e == 0) D.tri(ci, a, b);
            else D.tri(ci, b, a);
        }
    }
    // mirrors
    for (double y : {-1.0, 1.0}) P.obox({0.95, y * 0.97}, 0, 0.2, 0.14, 1.02, 1.13, w);
    // roof rails
    for (double y : {-0.58, 0.58}) D.obox({-0.55, y}, 0, 2.0, 0.05, 1.6, 1.64, Col{0.2f, 0.2f, 0.21f, 1});
    // wheels: tyre, sidewall and rim
    for (double x : WX)
        for (double y : {-0.78, 0.78}) {
            if (!with_wheels) continue;
            int base = (int)T.verts();
            int seg = 20;
            double r = 0.33, half = 0.115;
            for (int i = 0; i <= seg; i++) {
                double a = 2 * PI * i / seg;
                V3 n(std::cos(a), 0, std::sin(a));
                T.v({x + std::cos(a) * r, y - half, 0.33 + std::sin(a) * r}, n, 0, 0, w);
                T.v({x + std::cos(a) * r, y + half, 0.33 + std::sin(a) * r}, n, 0, 0, w);
            }
            for (int i = 0; i < seg; i++) {
                int a0 = base + i * 2;
                T.tri(a0, a0 + 3, a0 + 2);
                T.tri(a0, a0 + 1, a0 + 3);
            }
            double ys = y > 0 ? y + half : y - half;
            double sgn = y > 0 ? 1.0 : -1.0;
            Col rim{0.72f, 0.73f, 0.75f, 1};
            int ci = D.v({x, ys, 0.33}, {0, sgn, 0}, 0, 0, rim);
            for (int i = 0; i < seg; i++) {
                double a0 = 2 * PI * i / seg, a1 = 2 * PI * (i + 1) / seg;
                int p = D.v({x + std::cos(a0) * r * 0.7, ys, 0.33 + std::sin(a0) * r * 0.7}, {0, sgn, 0}, 0, 0, rim);
                int q = D.v({x + std::cos(a1) * r * 0.7, ys, 0.33 + std::sin(a1) * r * 0.7}, {0, sgn, 0}, 0, 0, rim);
                if (y > 0) D.tri(ci, q, p);
                else D.tri(ci, p, q);
            }
        }
    return ms;
}

// cabin seen from the driver's seat: dashboard, instrument binnacle, centre console, seats, door cards
MeshSet car_interior() {
    MeshSet ms;
    MeshBuf &D = ms[M_DARK];
    Col dash{0.16f, 0.16f, 0.17f, 1}, seat{0.12f, 0.12f, 0.13f, 1}, trim{0.35f, 0.33f, 0.31f, 1};
    // dashboard: a slab under the windscreen, sloping toward the glass
    D.obox({0.95, 0}, 0, 0.55, 1.62, 0.78, 1.02, dash, true);
    D.obox({0.62, 0}, 0, 0.2, 1.62, 0.7, 0.98, dash, true);
    // binnacle over the instruments (driver on the left: +y)
    D.obox({0.62, 0.37}, 0, 0.22, 0.42, 0.98, 1.06, dash, true);
    // centre console
    D.obox({0.15, 0}, 0, 0.9, 0.24, 0.42, 0.72, dash, true);
    // seats: cushion and back
    for (double y : {0.37, -0.37}) {
        D.obox({-0.15, y}, 0, 0.5, 0.5, 0.42, 0.58, seat, true);
        D.obox({-0.45, y}, 0, 0.12, 0.5, 0.58, 1.25, seat, true);
        D.obox({-0.47, y}, 0, 0.1, 0.26, 1.25, 1.42, seat, true);
    }
    D.obox({-1.05, 0}, 0, 0.5, 1.4, 0.42, 1.05, seat, true);  // rear bench
    // door cards
    for (double y : {0.82, -0.82}) D.obox({0.0, y}, 0, 2.1, 0.06, 0.45, 1.0, trim, true);
    // instrument faces: two dials
    MeshBuf &L = ms[M_LAMP];
    for (double dy : {0.27, 0.47}) {
        V2 c(0.53, dy);
        int ci = L.v({c.x, c.y, 0.97}, {-1, 0, 0}, 0.5, 0.5, Col{0.3f, 0.35f, 0.4f, 1});
        for (int k = 0; k < 16; k++) {
            double a0 = 2 * PI * k / 16, a1 = 2 * PI * (k + 1) / 16;
            int p = L.v({c.x, c.y + std::cos(a0) * 0.07, 0.97 + std::sin(a0) * 0.07}, {-1, 0, 0}, 0, 0, Col{0.3f, 0.35f, 0.4f, 1});
            int q = L.v({c.x, c.y + std::cos(a1) * 0.07, 0.97 + std::sin(a1) * 0.07}, {-1, 0, 0}, 0, 0, Col{0.3f, 0.35f, 0.4f, 1});
            L.tri(ci, q, p);
        }
    }
    return ms;
}

// steering wheel about its own axis (x along the column toward the driver), centre at the origin
MeshSet steering_wheel() {
    MeshSet ms;
    MeshBuf &D = ms[M_DARK];
    Col c{0.1f, 0.1f, 0.11f, 1};
    const double R = 0.19, r = 0.018;
    int seg = 28, sides = 8;
    int base = (int)D.verts();
    for (int i = 0; i <= seg; i++) {
        double a = 2 * PI * i / seg;
        V3 ctr(0, std::cos(a) * R, std::sin(a) * R), out(0, std::cos(a), std::sin(a));
        for (int k = 0; k <= sides; k++) {
            double b = 2 * PI * k / sides;
            V3 n = out * std::cos(b) + V3(1, 0, 0) * std::sin(b);
            D.v(ctr + n * r, n, 0, 0, c);
        }
    }
    for (int i = 0; i < seg; i++)
        for (int k = 0; k < sides; k++) {
            int a0 = base + i * (sides + 1) + k, a1 = a0 + 1, b0 = a0 + sides + 1, b1 = b0 + 1;
            D.tri(a0, b0, b1);
            D.tri(a0, b1, a1);
        }
    // hub and three spokes (the bottom spoke marks the straight-ahead position)
    D.obox({0.0, 0}, 0, 0.05, 0.12, -0.05, 0.05, c, true);
    D.obox({0.0, 0.1}, 0, 0.025, 0.18, -0.015, 0.015, c, true);
    D.obox({0.0, -0.1}, 0, 0.025, 0.18, -0.015, 0.015, c, true);
    D.obox({0.0, 0}, 0, 0.025, 0.04, -0.19, -0.04, c, true);
    return ms;
}


// a pedestrian, 1.75 m: uv.x tags the swinging limbs (1/2 legs, 3/4 arms) for the walk shader; uv.y is a material
// code (the shirt, built with colour alpha 0, takes the instance tint)
MeshSet person() {
    MeshSet ms;
    MeshBuf &D = ms[M_PAINT];
    Col skin{0.78f, 0.6f, 0.48f, 1}, pants{0.16f, 0.17f, 0.22f, 1}, shirt{1, 1, 1, 0}, hair{0.12f, 0.09f, 0.07f, 1}, shoe{0.08f, 0.08f, 0.08f, 1};
    auto tag = [&](size_t from, float part) {
        for (size_t i = from; i < D.verts(); i++) {
            const float *c = &D.col[i * 4];
            // material code for the shader: 0 skin, 1 shirt, 2 trousers, 3 shoes, 4 hair
            float code = c[3] < 0.5f ? 1 : c[0] == pants.r ? 2 : c[0] == shoe.r ? 3 : c[0] == hair.r ? 4 : 0;
            D.uv[i * 2] = part, D.uv[i * 2 + 1] = code;
        }
    };
    for (int s = 0; s < 2; s++) {
        double y = s ? -0.1 : 0.1;
        size_t f = D.verts();
        D.obox({0.0, y}, 0, 0.15, 0.13, 0.08, 0.92, pants, true);
        D.obox({0.04, y}, 0, 0.26, 0.11, 0.0, 0.08, shoe, true);
        tag(f, s ? 2.0f : 1.0f);
        f = D.verts();
        double ya = s ? -0.255 : 0.255;
        D.obox({0.0, ya}, 0, 0.1, 0.09, 1.02, 1.45, shirt, true);
        D.obox({0.0, ya}, 0, 0.08, 0.07, 0.8, 1.02, skin, true);
        tag(f, s ? 4.0f : 3.0f);
    }
    size_t f = D.verts();
    D.obox({0.0, 0}, 0, 0.22, 0.4, 0.9, 1.47, shirt, true);
    D.cyl({0.0, 0}, 0.05, 0.05, 1.47, 1.54, 8, skin, false);
    D.obox({0.01, 0}, 0, 0.2, 0.17, 1.54, 1.76, skin, true);
    D.obox({-0.01, 0}, 0, 0.21, 0.18, 1.72, 1.79, hair, false);
    tag(f, 0.0f);
    return ms;
}


// ---- street signs, traffic lights, bench, bin, bus shelter (model frame: x toward the traffic, z up) ----

// flat polygon facing +x at plane x, centred (cy, cz)
void fan_x(MeshBuf &m, double x, double cy, double cz, double r, int sides, double rot, Col c, bool front = true, double sy = 1.0) {
    V3 n(front ? 1 : -1, 0, 0);
    int ci = m.v({x, cy, cz}, n, 0.5, 0.5, c);
    std::vector<int> ring;
    for (int i = 0; i < sides; i++) {
        double a = rot + 2 * PI * i / sides;
        ring.push_back(m.v({x, cy + std::cos(a) * r * sy, cz + std::sin(a) * r}, n, 0.5 + std::cos(a) * 0.5, 0.5 + std::sin(a) * 0.5, c));
    }
    for (int i = 0; i < sides; i++) {
        if (front) m.tri(ci, ring[i], ring[(i + 1) % sides]);
        else m.tri(ci, ring[(i + 1) % sides], ring[i]);
    }
}
void rect_x(MeshBuf &m, double x, double y0, double z0, double y1, double z1, Col c) { m.quad({x, y0, z0}, {x, y1, z0}, {x, y1, z1}, {x, y0, z1}, {1, 0, 0}, c); }

void digit_x(MeshBuf &m, double x, double cy, double cz, double h, int d, Col c) {
    // 7 segments a(top) b(upper right) c(lower right) d(bottom) e(lower left) f(upper left) g(middle)
    static const int SEG[10] = {0b0111111, 0b0000110, 0b1011011, 0b1001111, 0b1100110, 0b1101101, 0b1111101, 0b0000111, 0b1111111, 0b1101111};
    double w = h * 0.5, t = h * 0.14;
    int mask = SEG[d];
    auto hbar = [&](double z) { rect_x(m, x, cy - w / 2, z - t / 2, cy + w / 2, z + t / 2, c); };
    auto vbar = [&](double y, double z0, double z1) { rect_x(m, x, y - t / 2, z0, y + t / 2, z1, c); };
    if (mask & 1) hbar(cz + h / 2 - t / 2);
    if (mask & 2) vbar(cy + w / 2 - t / 2, cz, cz + h / 2);
    if (mask & 4) vbar(cy + w / 2 - t / 2, cz - h / 2, cz);
    if (mask & 8) hbar(cz - h / 2 + t / 2);
    if (mask & 16) vbar(cy - w / 2 + t / 2, cz - h / 2, cz);
    if (mask & 32) vbar(cy - w / 2 + t / 2, cz, cz + h / 2);
    if (mask & 64) hbar(cz);
}

MeshSet sign_post(int kind) {
    MeshSet ms;
    MeshBuf &M = ms[M_METAL], &P = ms[M_PAINT];
    Col pole{0.5f, 0.52f, 0.55f, 1}, back{0.55f, 0.57f, 0.6f, 1};
    Col red{0.78f, 0.06f, 0.06f, 1}, white{0.96f, 0.96f, 0.94f, 1}, blue{0.05f, 0.25f, 0.62f, 1}, yellow{0.98f, 0.8f, 0.05f, 1}, black{0.05f, 0.05f, 0.06f, 1};
    double H = 2.75, cz = 2.3;  // plate centre height
    if (kind == SG_BUS) cz = 2.5;
    M.cyl({0, 0}, 0.04, 0.04, 0, H, 8, pole, true);
    const double X = 0.05;
    switch (kind) {
        case SG_STOP: {
            fan_x(M, X - 0.012, 0, cz, 0.37, 8, PI / 8, back, false);
            fan_x(P, X, 0, cz, 0.36, 8, PI / 8, white);
            fan_x(P, X + 0.001, 0, cz, 0.34, 8, PI / 8, red);
            rect_x(P, X + 0.002, -0.27, cz - 0.045, 0.27, cz + 0.045, white);
            break;
        }
        case SG_YIELD: {
            // triangle pointing down
            for (int pass = 0; pass < 3; pass++) {
                double r = pass == 0 ? 0.5 : pass == 1 ? 0.47 : 0.28;
                Col c = pass == 1 ? red : pass == 0 ? white : white;
                double x = pass == 0 ? X : X + 0.001 * pass;
                if (pass == 0) {
                    // back plate
                    V3 n(-1, 0, 0);
                    int ci = M.v({X - 0.012, 0, cz - 0.04}, n, 0.5, 0.5, back);
                    int a = M.v({X - 0.012, 0, cz - 0.04 - r * 0.9}, n, 0, 0, back), b = M.v({X - 0.012, -r * 0.78, cz - 0.04 + r * 0.45}, n, 0, 0, back), cc = M.v({X - 0.012, r * 0.78, cz - 0.04 + r * 0.45}, n, 0, 0, back);
                    M.tri(ci, a, b), M.tri(ci, b, cc), M.tri(ci, cc, a);
                }
                V3 n(1, 0, 0);
                double zc = cz - 0.04 + (pass == 2 ? 0.02 : 0);
                int ci = P.v({x, 0, zc}, n, 0.5, 0.5, c);
                int a = P.v({x, 0, zc - r * 0.9}, n, 0, 0, c), b = P.v({x, r * 0.78, zc + r * 0.45}, n, 0, 0, c), cc = P.v({x, -r * 0.78, zc + r * 0.45}, n, 0, 0, c);
                P.tri(ci, a, b), P.tri(ci, b, cc), P.tri(ci, cc, a);
            }
            break;
        }
        case SG_PRIORITY: {
            fan_x(M, X - 0.012, 0, cz, 0.42, 4, 0, back, false);
            fan_x(P, X, 0, cz, 0.41, 4, 0, black);
            fan_x(P, X + 0.001, 0, cz, 0.385, 4, 0, white);
            fan_x(P, X + 0.002, 0, cz, 0.30, 4, 0, yellow);
            break;
        }
        case SG_CROSSING: {
            rect_x(M, X - 0.012, -0.32, cz - 0.32, 0.32, cz + 0.32, back);
            rect_x(P, X, -0.31, cz - 0.31, 0.31, cz + 0.31, white);
            rect_x(P, X + 0.001, -0.29, cz - 0.29, 0.29, cz + 0.29, blue);
            // white triangle with a walker
            {
                V3 n(1, 0, 0);
                double x = X + 0.002;
                int a = P.v({x, -0.22, cz - 0.2}, n, 0, 0, white), b = P.v({x, 0.22, cz - 0.2}, n, 0, 0, white), c2 = P.v({x, 0, cz + 0.22}, n, 0, 0, white);
                P.tri(a, b, c2);
                fan_x(P, X + 0.003, 0.0, cz + 0.05, 0.03, 8, 0, black);
                rect_x(P, X + 0.003, -0.02, cz - 0.1, 0.02, cz + 0.02, black);
                rect_x(P, X + 0.003, -0.07, cz - 0.17, -0.02, cz - 0.1, black);
                rect_x(P, X + 0.003, 0.02, cz - 0.17, 0.07, cz - 0.1, black);
            }
            break;
        }
        case SG_SPEED30:
        case SG_SPEED40:
        case SG_SPEED50:
        case SG_SPEED60: {
            int tens = kind - SG_SPEED30 + 3;
            fan_x(M, X - 0.012, 0, cz, 0.37, 24, 0, back, false);
            fan_x(P, X, 0, cz, 0.36, 24, 0, red);
            fan_x(P, X + 0.001, 0, cz, 0.27, 24, 0, white);
            digit_x(P, X + 0.002, -0.07, cz, 0.24, tens, black);
            digit_x(P, X + 0.002, 0.07, cz, 0.24, 0, black);
            break;
        }
        case SG_NOENTRY: {
            fan_x(M, X - 0.012, 0, cz, 0.37, 24, 0, back, false);
            fan_x(P, X, 0, cz, 0.36, 24, 0, red);
            rect_x(P, X + 0.001, -0.27, cz - 0.05, 0.27, cz + 0.05, white);
            break;
        }
        case SG_PARKING: {
            rect_x(M, X - 0.012, -0.32, cz - 0.32, 0.32, cz + 0.32, back);
            rect_x(P, X, -0.31, cz - 0.31, 0.31, cz + 0.31, white);
            rect_x(P, X + 0.001, -0.29, cz - 0.29, 0.29, cz + 0.29, blue);
            rect_x(P, X + 0.002, -0.13, cz - 0.2, -0.04, cz + 0.2, white);
            rect_x(P, X + 0.002, -0.04, cz + 0.11, 0.12, cz + 0.2, white);
            rect_x(P, X + 0.002, -0.04, cz - 0.0, 0.12, cz + 0.09, white);
            rect_x(P, X + 0.002, 0.08, cz - 0.0, 0.14, cz + 0.2, white);
            break;
        }
        case SG_BUS: {
            rect_x(M, X - 0.012, -0.32, cz - 0.32, 0.32, cz + 0.32, back);
            rect_x(P, X, -0.31, cz - 0.31, 0.31, cz + 0.31, white);
            rect_x(P, X + 0.001, -0.29, cz - 0.29, 0.29, cz + 0.29, blue);
            rect_x(P, X + 0.002, -0.2, cz - 0.1, 0.2, cz + 0.17, white);
            rect_x(P, X + 0.003, -0.16, cz + 0.02, 0.16, cz + 0.12, blue);
            rect_x(P, X + 0.003, -0.15, cz - 0.17, -0.07, cz - 0.1, white);
            rect_x(P, X + 0.003, 0.07, cz - 0.17, 0.15, cz - 0.1, white);
            break;
        }
    }
    return ms;
}

// lamp disc facing +x (code in vertex colour: r 1 red, 0.5 amber, 0 green; g 1 = pedestrian lamp)
MeshSet signal_pole() {
    MeshSet ms;
    MeshBuf &M = ms[M_METAL], &D = ms[M_DARK], &L = ms[M_SIGNAL];
    Col pole{0.2f, 0.21f, 0.23f, 1}, head{0.08f, 0.08f, 0.09f, 1};
    M.cyl({0, 0}, 0.07, 0.07, 0, 3.7, 10, pole, true);
    // vehicle head: box with three lamps and visors
    D.box({-0.12, -0.17, 2.75}, {0.2, 0.17, 3.65}, head);
    const double zs[3] = {3.4, 3.15, 2.9}, cd[3] = {1.0, 0.5, 0.0};
    for (int i = 0; i < 3; i++) {
        fan_x(L, 0.201, 0, zs[i], 0.1, 14, 0, Col{cd[i] == 1.0 ? 1.0f : (cd[i] == 0.5 ? 0.5f : 0.0f), 0, 0, 1});
        // visor
        D.box({0.2, -0.13, zs[i] + 0.09}, {0.34, 0.13, zs[i] + 0.12}, head);
    }
    // pedestrian head toward the carriageway (-y)
    D.box({-0.13, -0.2, 2.0}, {0.13, -0.05, 2.6}, head);
    for (int i = 0; i < 2; i++) {
        double z = i == 0 ? 2.42 : 2.15;
        V3 n(0, -1, 0);
        double y = -0.2005;
        Col cc{i == 0 ? 1.0f : 0.0f, 1, 0, 1};
        int ci = L.v({0, y, z}, n, 0.5, 0.5, cc);
        std::vector<int> ring;
        for (int k = 0; k < 12; k++) {
            double a = 2 * PI * k / 12;
            ring.push_back(L.v({std::cos(a) * 0.075, y, z + std::sin(a) * 0.075}, n, 0, 0, cc));
        }
        for (int k = 0; k < 12; k++) L.tri(ci, ring[k], ring[(k + 1) % 12]);
    }
    return ms;
}

MeshSet bench() {
    MeshSet ms;
    MeshBuf &W = ms[M_PAINT], &M = ms[M_METAL];
    Col wood{0.45f, 0.3f, 0.16f, 1}, iron{0.15f, 0.16f, 0.17f, 1};
    for (int i = 0; i < 4; i++) W.box({-0.25 + i * 0.13, -0.8, 0.44}, {-0.14 + i * 0.13, 0.8, 0.48}, wood);
    for (int i = 0; i < 3; i++) W.box({-0.3, -0.8, 0.62 + i * 0.12}, {-0.26, 0.8, 0.7 + i * 0.12}, wood);
    for (double y : {-0.7, 0.7}) {
        M.box({-0.28, y - 0.025, 0}, {-0.24, y + 0.025, 0.95}, iron);
        M.box({-0.28, y - 0.025, 0.4}, {0.27, y + 0.025, 0.44}, iron);
        M.box({0.2, y - 0.025, 0}, {0.26, y + 0.025, 0.44}, iron);
    }
    return ms;
}

MeshSet bin() {
    MeshSet ms;
    ms[M_METAL].cyl({0, 0}, 0.2, 0.2, 0.0, 0.8, 12, Col{0.18f, 0.25f, 0.2f, 1}, true);
    ms[M_DARK].cyl({0, 0}, 0.21, 0.21, 0.8, 0.86, 12, Col{0.06f, 0.06f, 0.07f, 1}, true);
    return ms;
}

MeshSet shelter() {
    MeshSet ms;
    MeshBuf &M = ms[M_METAL], &G = ms[M_GLASS], &P = ms[M_PAINT];
    Col fr{0.25f, 0.27f, 0.3f, 1}, roof{0.2f, 0.3f, 0.5f, 1};
    const double W = 1.55, Dp = 0.7;
    for (double y : {-W, W}) {
        M.box({-Dp - 0.04, y - 0.04, 0}, {-Dp + 0.04, y + 0.04, 2.5}, fr);
        M.box({Dp - 0.04, y - 0.04, 0}, {Dp + 0.04, y + 0.04, 2.5}, fr);
        G.quad({-Dp, y, 0.3}, {Dp, y, 0.3}, {Dp, y, 2.3}, {-Dp, y, 2.3}, {0, y > 0 ? 1.0 : -1.0, 0}, Col{1, 1, 1, 1});
        G.quad({Dp, y, 0.3}, {-Dp, y, 0.3}, {-Dp, y, 2.3}, {Dp, y, 2.3}, {0, y > 0 ? -1.0 : 1.0, 0}, Col{1, 1, 1, 1});
    }
    G.quad({-Dp, -W, 0.3}, {-Dp, W, 0.3}, {-Dp, W, 2.3}, {-Dp, -W, 2.3}, {1, 0, 0}, Col{1, 1, 1, 1});
    G.quad({-Dp, W, 0.3}, {-Dp, -W, 0.3}, {-Dp, -W, 2.3}, {-Dp, W, 2.3}, {-1, 0, 0}, Col{1, 1, 1, 1});
    P.box({-Dp - 0.2, -W - 0.2, 2.5}, {Dp + 0.2, W + 0.2, 2.58}, roof);
    // bench inside
    P.box({-0.55, -1.1, 0.45}, {-0.15, 1.1, 0.5}, Col{0.45f, 0.3f, 0.16f, 1});
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
        case I_CAR_INTERIOR: return car_interior();
        case I_STEERING_WHEEL: return steering_wheel();
        case I_PERSON: return person();
        case I_SIGNAL: return signal_pole();
        case I_SIGN: return sign_post(std::clamp(v, 0, (int)SG_COUNT - 1));
        case I_BENCH: return bench();
        case I_BIN: return bin();
        case I_SHELTER: return shelter();
        default: return MeshSet();
    }
}

}  // namespace ec
