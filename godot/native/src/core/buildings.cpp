#include "buildings.hpp"
#include "signage.hpp"
#include <algorithm>

namespace ec {

namespace {

constexpr double TEX_M = 12.0;  // a facade texture covers 4 x 4 bays of 3 m

Col mul(Col c, float k) { return {c.r * k, c.g * k, c.b * k, c.a}; }

void push_tri(std::vector<float> &coll, V3 a, V3 b, V3 c) {
    coll.insert(coll.end(), {(float)a.x, (float)a.y, (float)a.z, (float)b.x, (float)b.y, (float)b.z, (float)c.x, (float)c.y, (float)c.z});
}

// Walls of all rings between z0 and z1 with facade texture coordinates.
void walls(const std::vector<Ring> &rings, double z0, double z1, double zref, int mat, Col col, double floor_h, uint64_t seed, MeshSet &ms, std::vector<float> &coll) {
    MeshBuf &m = ms[mat];
    double vs = 3.0 / std::max(2.0, floor_h);
    double uoff = double(seed % 4) * 3.0;
    for (auto &r : rings) {
        size_t n = r.size();
        double ustart = 0;
        for (size_t i = 0; i < n; i++) {
            V2 a = r[i], b = r[(i + 1) % n];
            V2 d = b - a;
            double l = d.len();
            if (l < 0.05) continue;
            double bays = std::round(l / 3.0);
            // whole bays per wall, so windows never wrap round a corner; very short walls stay blank
            double u0, u1;
            if (bays >= 1) {
                u0 = uoff + ustart;
                u1 = u0 + bays * 3.0;
                ustart += bays * 3.0;
            } else {
                u0 = u1 = uoff + ustart + 0.1;  // the plain strip at the bay edge
            }
            V3 nrm = V3(d.y, -d.x, 0).norm();
            V3 p0(a.x, a.y, z0), p1(b.x, b.y, z0), p2(b.x, b.y, z1), p3(a.x, a.y, z1);
            m.quad(p0, p1, p2, p3, nrm, col, u0 / TEX_M, -(z0 - zref) * vs / TEX_M, u1 / TEX_M, -(z1 - zref) * vs / TEX_M);
            push_tri(coll, p0, p1, p2);
            push_tri(coll, p0, p2, p3);
        }
    }
}

void flat_cap(const std::vector<Ring> &rings, double z, bool up, int mat, Col col, MeshSet &ms, std::vector<float> *coll) {
    std::vector<int> tri = triangulate(rings);
    std::vector<V2> pts;
    for (auto &r : rings) pts.insert(pts.end(), r.begin(), r.end());
    MeshBuf &m = ms[mat];
    V3 n = up ? V3(0, 0, 1) : V3(0, 0, -1);
    for (size_t k = 0; k + 2 < tri.size(); k += 3) {
        V2 a = pts[tri[k]], b = pts[tri[k + 1]], c = pts[tri[k + 2]];
        bool ccw = (b - a).cross(c - a) > 0;
        if (ccw != up) std::swap(b, c);
        int i0 = m.v({a.x, a.y, z}, n, a.x / 4, a.y / 4, col), i1 = m.v({b.x, b.y, z}, n, b.x / 4, b.y / 4, col), i2 = m.v({c.x, c.y, z}, n, c.x / 4, c.y / 4, col);
        m.tri(i0, i1, i2);
        if (coll) push_tri(*coll, {a.x, a.y, z}, {b.x, b.y, z}, {c.x, c.y, z});
    }
}

bool is_convex(const Ring &c) {
    size_t n = c.size();
    if (n < 3) return false;
    for (size_t i = 0; i < n; i++)
        if ((c[(i + 1) % n] - c[i]).cross(c[(i + 2) % n] - c[(i + 1) % n]) < -1e-9) return false;
    return true;
}

// Pitched roof over a simplified 4-corner convex outline (gabled / hipped / pyramidal) or an apex roof
// (pyramid / dome / onion) over any convex outline. Returns false when the outline does not suit.
bool pitched_roof(const Ring &outer, const std::string &shape, double wall_top, double &roof_h, int mat, Col col, MeshSet &ms, std::vector<float> &coll, Ring &used) {
    Ring c = simplify(outer, 0.5, 8);
    make_ccw(c);
    if (!is_convex(c)) return false;
    MeshBuf &m = ms[mat];
    auto add_tri = [&](V3 a, V3 b, V3 d) {
        V3 n = (b - a).cross(d - a).norm();
        if (n.z < 0) {
            std::swap(b, d);
            n = n * -1;
        }
        // texture rows run along the eaves, columns up the slope
        V3 t = V3(n.y, -n.x, 0);
        if (t.len() < 1e-6) t = V3(1, 0, 0);
        t = t.norm();
        V3 bt = n.cross(t);
        auto U = [&](V3 p) { return p.dot(t) / 3; };
        auto W = [&](V3 p) { return -p.dot(bt) / 3; };
        int i0 = m.v(a, n, U(a), W(a), col), i1 = m.v(b, n, U(b), W(b), col), i2 = m.v(d, n, U(d), W(d), col);
        m.tri(i0, i1, i2);
        push_tri(coll, a, b, d);
    };
    size_t n = c.size();
    if (shape == "dome" || shape == "onion" || (shape == "pyramidal" && n != 4)) {
        if (n > 48) return false;
        V2 ctr;
        for (auto &p : c) ctr = ctr + p;
        ctr = ctr * (1.0 / n);
        double rad = 0;
        for (auto &p : c) rad += (p - ctr).len();
        rad /= n;
        std::vector<std::pair<double, double>> prof;
        if (shape == "dome") prof = {{1, 0}, {0.95, 0.3}, {0.82, 0.58}, {0.6, 0.8}, {0.32, 0.95}, {0, 1}}, roof_h = roof_h > 0 ? roof_h : rad;
        else if (shape == "onion") prof = {{1, 0}, {1.12, 0.15}, {1.15, 0.3}, {0.95, 0.48}, {0.55, 0.66}, {0.22, 0.82}, {0.06, 0.93}, {0, 1}}, roof_h = roof_h > 0 ? roof_h : rad * 2.2;
        else prof = {{1, 0}, {0, 1}}, roof_h = roof_h > 0 ? roof_h : rad * 0.9;
        // rings scaled about the centre, following the outline
        for (size_t k = 0; k + 1 < prof.size(); k++) {
            for (size_t i = 0; i < n; i++) {
                V2 a = c[i], b = c[(i + 1) % n];
                auto P = [&](V2 q, size_t kk) { V2 s = ctr + (q - ctr) * prof[kk].first; return V3(s.x, s.y, wall_top + roof_h * prof[kk].second); };
                V3 a0 = P(a, k), b0 = P(b, k), a1 = P(a, k + 1), b1 = P(b, k + 1);
                add_tri(a0, b0, b1);
                if (prof[k + 1].first > 1e-6) add_tri(a0, b1, a1);
            }
        }
        used = c;
        return true;
    }
    if (n != 4) return false;
    auto len = [&](size_t i) { return (c[(i + 1) % 4] - c[i % 4]).len(); };
    size_t a = len(0) + len(2) >= len(1) + len(3) ? 0 : 1;
    double width = (len(a + 1) + len(a + 3)) / 2, length = (len(a) + len(a + 2)) / 2;
    if (roof_h <= 0) roof_h = shape == "pyramidal" ? std::min(width, length) * 0.35 : width * 0.5 * std::tan(30 * PI / 180);
    roof_h = std::min(roof_h, 12.0);
    auto mid = [&](size_t i) { return (c[i % 4] + c[(i + 1) % 4]) * 0.5; };
    auto T = [&](size_t k) { V2 p = c[k % 4]; return V3(p.x, p.y, wall_top); };
    double top = wall_top + roof_h;
    if (shape == "pyramidal") {
        V2 ap = (c[0] + c[1] + c[2] + c[3]) * 0.25;
        V3 A(ap.x, ap.y, top);
        for (size_t k = 0; k < 4; k++) add_tri(T(k), T(k + 1), A);
    } else {
        V2 m1 = mid(a + 1), m3 = mid(a + 3);
        V2 r1 = m1, r3 = m3;
        bool hip = shape == "hipped";
        if (hip) {
            V2 d = m3 - m1;
            double l = d.len(), inset = width / 2;
            if (l <= 2 * inset + 0.2) r1 = r3 = (m1 + m3) * 0.5;
            else {
                V2 u = d * (1 / l);
                r1 = m1 + u * inset;
                r3 = m3 - u * inset;
            }
        }
        V3 R1(r1.x, r1.y, top), R3(r3.x, r3.y, top);
        add_tri(T(a), T(a + 1), R1);
        add_tri(T(a), R1, R3);
        add_tri(T(a + 2), T(a + 3), R3);
        add_tri(T(a + 2), R3, R1);
        if (hip) {
            add_tri(T(a + 1), T(a + 2), R1);
            add_tri(T(a + 3), T(a), R3);
        } else {
            // gable ends: wall-coloured triangles are added by the caller as walls; close here with the roof
            // material's underside colour (seen as eaves)
            add_tri(T(a + 1), T(a + 2), R1);
            add_tri(T(a + 3), T(a), R3);
        }
    }
    used = c;
    return true;
}

double ring_area(const Ring &r) { return std::abs(signed_area(r)); }

// ---- landmarks: towers, domes, spires (typical shapes placed by rule) ----

std::vector<std::pair<double, double>> onion(double r, double h) {
    std::vector<std::pair<double, double>> p = {{0.82, 0}, {1.0, 0.12}, {1.08, 0.28}, {0.98, 0.45}, {0.7, 0.62}, {0.38, 0.78}, {0.14, 0.92}, {0, 1}};
    for (auto &q : p) q = {q.first * r, q.second * h};
    return p;
}
std::vector<std::pair<double, double>> dome(double r, double h) {
    std::vector<std::pair<double, double>> p;
    for (int k = 0; k <= 8; k++) {
        double a = k / 8.0 * PI / 2;
        p.push_back({std::cos(a) * r, std::sin(a) * h});
    }
    return p;
}

void prism(MeshBuf &m, V2 c, double r, int sides, double rot, double z0, double z1, Col col, double vs, bool cap) {
    double side = 2 * r * std::sin(PI / sides);
    double bays = std::max(1.0, std::round(side / 3));
    double u = 0;
    for (int k = 0; k < sides; k++) {
        double a0 = rot + 2 * PI * k / sides, a1 = rot + 2 * PI * (k + 1) / sides, am = (a0 + a1) / 2;
        V2 p(c.x + std::cos(a0) * r, c.y + std::sin(a0) * r), q(c.x + std::cos(a1) * r, c.y + std::sin(a1) * r);
        m.quad({p.x, p.y, z0}, {q.x, q.y, z0}, {q.x, q.y, z1}, {p.x, p.y, z1}, {std::cos(am), std::sin(am), 0}, col, u / TEX_M, -z0 * vs / TEX_M, (u + bays * 3) / TEX_M, -z1 * vs / TEX_M);
        u += bays * 3;
    }
    if (cap) {
        int ci = m.v({c.x, c.y, z1}, {0, 0, 1}, 0, 0, col);
        for (int k = 0; k < sides; k++) {
            double a0 = rot + 2 * PI * k / sides, a1 = rot + 2 * PI * (k + 1) / sides;
            int p = m.v({c.x + std::cos(a0) * r, c.y + std::sin(a0) * r, z1}, {0, 0, 1}, 0, 0, col);
            int q = m.v({c.x + std::cos(a1) * r, c.y + std::sin(a1) * r, z1}, {0, 0, 1}, 0, 0, col);
            m.tri(ci, p, q);
        }
    }
}

void spire(MeshBuf &m, V2 c, double r, int sides, double rot, double z0, double h, Col col) {
    V3 ap(c.x, c.y, z0 + h);
    for (int k = 0; k < sides; k++) {
        double a0 = rot + 2 * PI * k / sides, a1 = rot + 2 * PI * (k + 1) / sides, am = (a0 + a1) / 2;
        V3 n = V3(std::cos(am) * h, std::sin(am) * h, r * std::cos(PI / sides)).norm();
        int i0 = m.v({c.x + std::cos(a0) * r, c.y + std::sin(a0) * r, z0}, n, 0, 0, col);
        int i1 = m.v({c.x + std::cos(a1) * r, c.y + std::sin(a1) * r, z0}, n, 1, 0, col);
        int i2 = m.v(ap, n, 0.5, 1, col);
        m.tri(i0, i1, i2);
    }
}

void cross(MeshSet &ms, V3 p, double size, V2 along, bool orthodox) {
    MeshBuf &g = ms[M_GOLD];
    Col c{1, 1, 1, 1};
    double t = size * 0.06;
    g.box({p.x - t, p.y - t, p.z}, {p.x + t, p.y + t, p.z + size}, c);
    double yaw = std::atan2(along.y, along.x);
    auto bar = [&](double z, double half) { g.obox({p.x, p.y}, yaw, half * 2, t * 2, p.z + z - t, p.z + z + t, c, true); };
    bar(size * 0.68, size * 0.32);
    if (orthodox) {
        bar(size * 0.86, size * 0.16);
        bar(size * 0.3, size * 0.2);
    }
}

void landmark(const BuildingGeom &b, double base, double h, MeshSet &ms) {
    const auto &bi = b.info;
    if (bi.landmark == Landmark::None) return;
    V2 c, d;
    double len, wid;
    footprint_frame(b.rings[0], c, d, len, wid);
    if (len < 4 || wid < 3) return;
    int fm = facade_mat(bi.d.style, bi.d.variant);
    Col wall = bi.d.colour;
    double vs = 3.0 / std::max(2.0f, bi.d.floor_h);
    uint64_t seed = bi.seed;
    auto at = [&](double s, double t) { return c + d * s + d.perp() * t; };
    double west = std::abs(d.x) > 0.3 ? (d.x > 0 ? -1 : 1) : (d.y > 0 ? -1 : 1);
    double rot = std::atan2(d.y, d.x) + PI / 4;
    Col roof = bi.roof_colour;
    int dome_mat = M_GOLD;
    Col dome_c{1, 1, 1, 1};
    switch ((seed >> 11) % 5) {
        case 2: dome_mat = M_ROOF_METAL, dome_c = {0.25f, 0.4f, 0.85f, 1}; break;
        case 3: dome_mat = M_ROOF_METAL, dome_c = {0.35f, 0.7f, 0.5f, 1}; break;
        case 4: dome_mat = M_ROOF_METAL, dome_c = {0.8f, 0.82f, 0.85f, 1}; break;
        default: break;
    }
    double zb = base;  // ground
    double top = base + h;
    MeshBuf &W = ms[fm];
    if (bi.landmark == Landmark::Orthodox || bi.landmark == Landmark::Spire) {
        bool orth = bi.landmark == Landmark::Orthodox;
        double a = std::clamp(wid * (orth ? 0.5 : 0.55), 4.0, 10.0);
        V2 tc = at(west * (len / 2 - a / 2 + 0.4), 0);
        double r = a / std::sqrt(2.0);
        double body_top = top + 2 + hash01(seed, 1) * 3;
        prism(W, tc, r, 4, rot, zb - 0.5, body_top, wall, vs, false);
        prism(ms[M_CONCRETE], tc, r + 0.2, 4, rot, body_top - 0.35, body_top, mul(wall, 0.85f), vs, true);
        if (orth) {
            double r2 = a * 0.42, bel = body_top + a * 0.9 + 1;
            prism(W, tc, r2, 8, rot, body_top, bel, wall, vs, false);
            spire(ms[M_ROOF_METAL], tc, r2 + 0.2, 8, rot, bel, a * 0.35, roof);
            double dz = bel + a * 0.3;
            ms[M_CONCRETE].cyl(tc, a * 0.14, a * 0.14, dz - a * 0.3, dz + a * 0.35, 10, wall, false);
            double oh = a * 0.55;
            ms[dome_mat].revolve(tc, dz + a * 0.35, onion(a * 0.2, oh), 16, dome_c);
            cross(ms, {tc.x, tc.y, dz + a * 0.35 + oh}, a * 0.35, d.perp(), true);
            // drum and onion over the body; four small ones on big churches
            V2 dc = at(-west * a / 2, 0);
            double rd = std::clamp(wid * 0.2, 1.8, 6.0), dz2 = top - 0.5, dh = rd * 1.3;
            prism(W, dc, rd, 8, rot, dz2, dz2 + dh, wall, vs, false);
            ms[dome_mat].revolve(dc, dz2 + dh, onion(rd * 1.08, rd * 2.1), 18, dome_c);
            cross(ms, {dc.x, dc.y, dz2 + dh + rd * 2.1}, rd * 0.9, d.perp(), true);
            if (wid > 14 && len > 22) {
                double off = rd * 1.6;
                for (int ks = -1; ks <= 1; ks += 2)
                    for (int kt = -1; kt <= 1; kt += 2) {
                        V2 sc2 = at(-west * a / 2 + ks * off, kt * off);
                        double r3 = rd * 0.45;
                        prism(W, sc2, r3, 8, rot, dz2, dz2 + r3 * 1.6, wall, vs, false);
                        ms[dome_mat].revolve(sc2, dz2 + r3 * 1.6, onion(r3 * 1.1, r3 * 2.1), 12, dome_c);
                        cross(ms, {sc2.x, sc2.y, dz2 + r3 * 1.6 + r3 * 2.1}, r3 * 0.9, d.perp(), true);
                    }
            }
        } else {
            double sh = a * (2.2 + hash01(seed, 2) * 1.2);
            spire(ms[M_ROOF_METAL], tc, r + 0.25, 4, rot, body_top, sh, mul(roof, 1.2f));
            cross(ms, {tc.x, tc.y, body_top + sh - 0.2}, std::min(a * 0.3, 2.5), d.perp(), false);
        }
    } else if (bi.landmark == Landmark::Chapel) {
        double rd = std::clamp(wid * 0.18, 0.8, 2.5), z = top - 0.3;
        prism(W, c, rd, 8, rot, z, z + rd, wall, vs, false);
        ms[dome_mat].revolve(c, z + rd, onion(rd * 1.1, rd * 2), 14, dome_c);
        cross(ms, {c.x, c.y, z + rd * 3}, rd, d.perp(), true);
    } else if (bi.landmark == Landmark::Mosque) {
        double rd = std::clamp(std::min(wid, len) * 0.32, 2.0, 14.0);
        ms[M_CONCRETE].cyl(c, rd, rd, top, top + rd * 0.35, 20, wall, false);
        ms[M_ROOF_METAL].revolve(c, top + rd * 0.35, dome(rd * 1.02, rd * 0.95), 24, (seed >> 7) % 2 ? Col{0.85f, 0.88f, 0.92f, 1} : Col{0.3f, 0.6f, 0.5f, 1});
        V2 mc = at(west * (len / 2 + 1.6), wid / 2 + 1.6);
        double mh = std::max(h * 2.2, h + 14), mr = 1.3;
        ms[M_CONCRETE].cyl(mc, mr, mr * 0.9, zb - 0.5, zb + mh, 12, wall, false);
        ms[M_CONCRETE].cyl(mc, mr + 0.6, mr + 0.6, zb + mh * 0.78, zb + mh * 0.78 + 0.35, 12, mul(wall, 0.9f), true);
        ms[M_ROOF_METAL].cyl(mc, mr + 0.1, 0.0, zb + mh, zb + mh + 4.5, 12, Col{0.3f, 0.6f, 0.5f, 1}, false);
    }
}

}  // namespace


// A standing figure in bronze (or stone), height h, facing `yaw`, feet at z: a sculptor's mannequin rather than
// a portrait, but read from the street as a statue on its pedestal.
void statue(MeshSet &ms, V2 c, double yaw, double z, double h, Col col, uint64_t seed) {
    MeshBuf &M = ms[M_METAL];
    double k = h / 1.8;
    double cs = std::cos(yaw), sn = std::sin(yaw);
    auto L = [&](double fwd, double side) { return V2(c.x + fwd * cs - side * sn, c.y + fwd * sn + side * cs); };
    double pose = hash01(seed, 91);  // 0..1: arm raised, coat, hands down
    // legs (or a long coat), torso, shoulders, head
    if (pose < 0.4) {
        M.cyl(L(0, 0), 0.26 * k, 0.2 * k, z, z + 1.0 * k, 10, col, false);  // long coat
    } else {
        M.obox(L(0, 0.1 * k), yaw, 0.16 * k, 0.13 * k, z, z + 0.92 * k, col, true);
        M.obox(L(0.05 * k, -0.11 * k), yaw, 0.16 * k, 0.13 * k, z, z + 0.92 * k, col, true);
    }
    M.cyl(L(0, 0), 0.2 * k, 0.24 * k, z + 0.9 * k, z + 1.45 * k, 10, col, false);
    M.obox(L(0, 0), yaw, 0.22 * k, 0.5 * k, z + 1.38 * k, z + 1.5 * k, col, true);
    M.cyl(L(0, 0), 0.05 * k, 0.05 * k, z + 1.5 * k, z + 1.56 * k, 6, col, false);
    M.cyl(L(0.01 * k, 0), 0.1 * k, 0.09 * k, z + 1.56 * k, z + 1.8 * k, 10, col, true);
    // arms: one raised forward in the bigger poses, the other along the body
    M.obox(L(0, 0.29 * k), yaw, 0.11 * k, 0.1 * k, z + 0.85 * k, z + 1.46 * k, col, true);
    if (pose > 0.7) {
        V2 a0 = L(0.05 * k, -0.29 * k), a1 = L(0.45 * k, -0.35 * k);
        for (int i = 0; i < 4; i++) {
            double t0 = i / 4.0, t1 = (i + 1) / 4.0;
            V2 p0 = a0 + (a1 - a0) * t0, p1 = a0 + (a1 - a0) * t1;
            M.obox((p0 + p1) * 0.5, yaw, 0.12 * k, 0.1 * k, z + (1.42 + 0.35 * t0) * k, z + (1.48 + 0.35 * t1) * k, col, true);
        }
    } else
        M.obox(L(0, -0.29 * k), yaw, 0.11 * k, 0.1 * k, z + 0.85 * k, z + 1.46 * k, col, true);
}

// Monuments and memorials: stepped granite base following the footprint, a pedestal, and on it a statue, an
// obelisk or a stele, chosen by shape and height (not a building with windows)
static void monument(const BuildingGeom &b, MeshSet &ms, std::vector<float> &coll) {
    const BuildingInfo &bi = b.info;
    const Ring &outer = b.rings[0];
    double area = std::abs(signed_area(outer));
    double base = b.base, h = std::max(1.5, bi.height);
    uint64_t seed = bi.seed;
    Col granite = hash01(seed, 7) < 0.5 ? Col{0.42f, 0.36f, 0.34f, 1} : Col{0.62f, 0.6f, 0.58f, 1};
    Col bronze = hash01(seed, 8) < 0.75 ? Col{0.32f, 0.28f, 0.2f, 1} : Col{0.3f, 0.42f, 0.36f, 1};  // or with verdigris
    V2 ctr = centroid(outer);
    // the long axis
    double best = 0, yaw = 0;
    for (size_t i = 0; i < outer.size(); i++) {
        V2 d = outer[(i + 1) % outer.size()] - outer[i];
        if (d.len() > best) best = d.len(), yaw = std::atan2(d.y, d.x);
    }
    double rad = std::sqrt(area / PI);
    bool slender = h > 2.5 * std::sqrt(std::max(area, 1.0));
    // steps: two rings each 0.25 m in, 0.2 m high
    std::vector<Ring> steps = {outer};
    for (int sidx = 0; sidx < 2 && rad > 1.6; sidx++) {
        Ring r = offset_ring(steps.back(), -0.35);
        if (r.size() < 3 || std::abs(signed_area(r)) < 1.0) break;
        steps.push_back(r);
    }
    double z = base - 0.3;
    for (size_t sidx = 0; sidx < steps.size(); sidx++) {
        double z1 = base + 0.2 * (sidx + 1);
        walls({steps[sidx]}, z, z1, base, M_CONCRETE, granite, 3, seed, ms, coll);
        flat_cap({steps[sidx]}, z1, true, M_CONCRETE, granite, ms, &coll);
        z = z1;
    }
    double top_steps = z;
    if (slender) {
        // obelisk / column: a tapering shaft with a pyramid tip
        double r0 = std::max(0.3, std::min(rad * 0.45, 2.0)), r1 = r0 * 0.7;
        double shaft = h - 0.2 * steps.size() - r1 * 1.2;
        ms[M_CONCRETE].cyl(ctr, r0, r1, top_steps, top_steps + shaft, 4, granite, false);
        ms[M_CONCRETE].cyl(ctr, r1, 0.01, top_steps + shaft, top_steps + shaft + r1 * 1.2, 4, granite, false);
        for (int i = 0; i < 4; i++) {
            double a = i * PI / 2 + PI / 4;
            V2 p(ctr.x + std::cos(a) * r0 * 0.7, ctr.y + std::sin(a) * r0 * 0.7);
            (void)p;
        }
        V3 lo(ctr.x - r0 * 0.7, ctr.y - r0 * 0.7, top_steps), hi(ctr.x + r0 * 0.7, ctr.y + r0 * 0.7, top_steps + shaft);
        push_tri(coll, V3(lo.x, lo.y, lo.z), V3(hi.x, lo.y, lo.z), V3(hi.x, lo.y, hi.z));
        push_tri(coll, V3(lo.x, lo.y, lo.z), V3(hi.x, lo.y, hi.z), V3(lo.x, lo.y, hi.z));
        push_tri(coll, V3(hi.x, hi.y, lo.z), V3(lo.x, hi.y, lo.z), V3(lo.x, hi.y, hi.z));
        push_tri(coll, V3(hi.x, hi.y, lo.z), V3(lo.x, hi.y, hi.z), V3(hi.x, hi.y, hi.z));
        return;
    }
    if (area > 250 && h < 6) {
        // a memorial site: a long stele and a flame bowl in front
        double sl = std::min(best * 0.5, 12.0), sh = std::max(2.5, h);
        ms[M_CONCRETE].obox(ctr, yaw, sl, 0.8, top_steps, top_steps + sh, granite, false);
        V2 fp(ctr.x - std::sin(yaw) * 3, ctr.y + std::cos(yaw) * 3);
        ms[M_METAL].cyl(fp, 0.8, 0.5, top_steps, top_steps + 0.35, 12, bronze, true);
        Col fl{1.0f, 0.55f, 0.15f, 1};
        ms[M_LAMP].cyl(fp, 0.25, 0.02, top_steps + 0.35, top_steps + 0.9, 8, fl, false);
        return;
    }
    // pedestal and statue
    double ped_w = std::max(0.8, std::min(rad * 0.9, 3.0));
    double statue_h = std::clamp((h - (top_steps - base)) * 0.45, 1.8, 9.0);
    double ped_h = std::max(1.2, h - (top_steps - base) - statue_h);
    ms[M_CONCRETE].obox(ctr, yaw, ped_w, ped_w * 0.8, top_steps, top_steps + ped_h, granite, false);
    ms[M_CONCRETE].obox(ctr, yaw, ped_w * 1.12, ped_w * 0.92, top_steps + ped_h - 0.15, top_steps + ped_h, mul(granite, 1.1f), false);
    statue(ms, ctr, yaw + PI / 2, top_steps + ped_h, statue_h, bronze, seed);
    V3 a(ctr.x - ped_w / 2, ctr.y - ped_w / 2, top_steps), c(ctr.x + ped_w / 2, ctr.y + ped_w / 2, top_steps + ped_h + statue_h * 0.8);
    push_tri(coll, V3(a.x, a.y, a.z), V3(c.x, a.y, a.z), V3(c.x, a.y, c.z));
    push_tri(coll, V3(a.x, a.y, a.z), V3(c.x, a.y, c.z), V3(a.x, a.y, c.z));
    push_tri(coll, V3(c.x, c.y, a.z), V3(a.x, c.y, a.z), V3(a.x, c.y, c.z));
    push_tri(coll, V3(c.x, c.y, a.z), V3(a.x, c.y, c.z), V3(c.x, c.y, c.z));
}

void build_building(const BuildingGeom &b, MeshSet &ms, std::vector<float> &coll, std::vector<Label> *labels) {
    const BuildingInfo &bi = b.info;
    if (b.rings.empty() || b.rings[0].size() < 3) return;
    if (bi.use == Use::Monument && !bi.part) {
        monument(b, ms, coll);
        return;
    }
    if (bi.use == Use::Monument && bi.part) {
        // a piece of a monument: dressed stone, and a slender top piece ends in a pyramidion
        const Ring &o = b.rings[0];
        double ar = std::abs(signed_area(o));
        double z0 = bi.min_height > 0.1 ? b.base + bi.min_height : b.base - 0.5, z1 = b.base + bi.height;
        Col stone = hash01(bi.seed, 7) < 0.6 ? Col{0.78f, 0.76f, 0.72f, 1} : Col{0.45f, 0.4f, 0.38f, 1};
        walls(b.rings, z0, z1, b.base, M_CONCRETE, stone, 3, bi.seed, ms, coll);
        bool slender = (z1 - z0) > 3 * std::sqrt(std::max(ar, 0.5));
        if (slender && o.size() >= 3) {
            V2 cc = centroid(o);
            double tip = std::sqrt(ar) * 0.9;
            for (size_t i = 0; i < o.size(); i++) {
                V2 a = o[i], e = o[(i + 1) % o.size()];
                ms[M_CONCRETE].quadn(V3(a.x, a.y, z1), V3(e.x, e.y, z1), V3(cc.x, cc.y, z1 + tip), V3(cc.x, cc.y, z1 + tip), stone);
            }
        } else
            flat_cap(b.rings, z1, true, M_CONCRETE, stone, ms, &coll);
        if (bi.min_height > 0.1) flat_cap(b.rings, z0, false, M_CONCRETE, mul(stone, 0.8f), ms, nullptr);
        return;
    }
    const Ring &outer = b.rings[0];
    double area = ring_area(outer);
    double base = b.base;
    double h = bi.height;
    double z_bot = bi.min_height > 0.1 ? base + bi.min_height : base - 1.5;  // into the ground on slopes
    double top = base + h;
    int fm = facade_mat(bi.d.style, bi.d.variant);
    Col wall = bi.d.colour;
    double floor_h = bi.d.floor_h;
    // roof shape (only for whole buildings without courtyards)
    std::string shape = bi.roof_shape;
    bool pitched = shape != "flat" && b.rings.size() == 1;
    double roof_h = bi.roof_height;
    double wall_top = top;
    Ring used;
    if (pitched) {
        double rh = roof_h > 0 ? roof_h : 0;
        // estimate the roof height to know where walls end
        Ring c = simplify(outer, 0.5, 8);
        double est;
        if (shape == "dome" || shape == "onion" || shape == "pyramidal") {
            V2 ctr = centroid(c);
            double rad = 0;
            for (auto &p : c) rad += (p - ctr).len();
            rad /= std::max<size_t>(1, c.size());
            est = rh > 0 ? rh : shape == "dome" ? rad : shape == "onion" ? rad * 2.2 : (c.size() == 4 ? 0 : rad * 0.9);
            if (shape == "pyramidal" && c.size() == 4) {
                double l0 = (c[1] - c[0]).len(), l1 = (c[2] - c[1]).len();
                est = rh > 0 ? rh : std::min(l0, l1) * 0.35;
            }
        } else {
            V2 cc;
            double len, wid;
            footprint_frame(c, cc, cc, len, wid);
            est = rh > 0 ? rh : wid * 0.5 * std::tan(30 * PI / 180);
        }
        est = std::min({est, (h - bi.min_height) * 0.75, 12.0});
        if (h - bi.min_height - est < 2.2 && shape != "dome" && shape != "onion" && shape != "pyramidal") pitched = false;
        else {
            roof_h = est;
            wall_top = top - roof_h;
            int rm = (shape == "dome" || shape == "onion") ? M_ROOF_METAL : M_ROOF_TILE;
            if (!pitched_roof(outer, shape, wall_top, roof_h, rm, bi.roof_colour, ms, coll, used)) {
                pitched = false;
                wall_top = top;
            }
        }
    }
    // walls: pitched roofs sit on the simplified outline they were built on
    std::vector<Ring> wall_rings = pitched && !used.empty() ? std::vector<Ring>{used} : b.rings;
    walls(wall_rings, z_bot, wall_top, base, fm, wall, floor_h, bi.seed, ms, coll);
    if (pitched && !used.empty() && shape == "gabled") {
        // gable triangles in the wall material
        Ring c = used;
        auto len = [&](size_t i) { return (c[(i + 1) % 4] - c[i % 4]).len(); };
        size_t a = len(0) + len(2) >= len(1) + len(3) ? 0 : 1;
        for (size_t e : {a + 1, a + 3}) {
            V2 p = c[e % 4], q = c[(e + 1) % 4];
            V2 m = (p + q) * 0.5;
            V3 nrm = V3((q - p).y, -(q - p).x, 0).norm();
            MeshBuf &W = ms[fm];
            double vs = 3.0 / std::max(2.0, floor_h);
            int i0 = W.v({p.x, p.y, wall_top}, nrm, 0, -(wall_top - base) * vs / TEX_M, wall);
            int i1 = W.v({q.x, q.y, wall_top}, nrm, (q - p).len() / TEX_M, -(wall_top - base) * vs / TEX_M, wall);
            int i2 = W.v({m.x, m.y, top}, nrm, (q - p).len() / 2 / TEX_M, -(top - base) * vs / TEX_M, wall);
            W.tri(i0, i1, i2);
        }
    }
    if (!pitched) flat_cap(b.rings, top, true, M_ROOF_FLAT, bi.roof_colour, ms, &coll);
    if (bi.min_height > 0.1) flat_cap(b.rings, z_bot, false, M_CONCRETE, mul(wall, 0.7f), ms, nullptr);

    const Design &d = bi.d;
    int floors = (int)std::floor((h - bi.min_height) / std::max(2.4f, d.floor_h));
    bool ground = bi.min_height < 0.1;
    bool flat = !pitched;
    uint64_t seed = bi.seed;
    MeshBuf &C = ms[M_CONCRETE];
    // plinth: a darker band round the foot
    if (ground && h > 4 && d.plinth_h > 0.05) {
        for (auto &r : b.rings)
            for (size_t i = 0; i < r.size(); i++) {
                V2 a = r[i], c = r[(i + 1) % r.size()];
                V2 dd = c - a;
                if (dd.len() < 0.5) continue;
                V2 o = V2(dd.y, -dd.x).norm() * 0.05;
                V3 nrm(o.x * 20, o.y * 20, 0);
                C.quad({a.x + o.x, a.y + o.y, base - 1.5}, {c.x + o.x, c.y + o.y, base - 1.5}, {c.x + o.x, c.y + o.y, base + d.plinth_h}, {a.x + o.x, a.y + o.y, base + d.plinth_h}, nrm, mul(wall, 0.55f), 0, 0, dd.len() / 3, (d.plinth_h + 1.5) / 3);
            }
    }
    // parapet with a cap round flat roofs
    if (flat && d.parapet_h > 0.05 && h > 5 && !(bi.part && h < 8)) {
        double ph = d.parapet_h, t = 0.22;
        for (auto &r : b.rings)
            for (size_t i = 0; i < r.size(); i++) {
                V2 a = r[i], c = r[(i + 1) % r.size()];
                V2 dd = c - a;
                if (dd.len() < 0.3) continue;
                V2 o = V2(dd.y, -dd.x).norm();
                V2 ai = a - o * t, ci = c - o * t;
                Col pc = mul(wall, 0.85f);
                C.quad({a.x, a.y, top}, {c.x, c.y, top}, {c.x, c.y, top + ph}, {a.x, a.y, top + ph}, {o.x, o.y, 0}, pc, 0, 0, dd.len() / 3, ph / 3);
                C.quad({ci.x, ci.y, top}, {ai.x, ai.y, top}, {ai.x, ai.y, top + ph}, {ci.x, ci.y, top + ph}, {-o.x, -o.y, 0}, pc, 0, 0, dd.len() / 3, ph / 3);
                C.quad({a.x, a.y, top + ph}, {c.x, c.y, top + ph}, {ci.x, ci.y, top + ph}, {ai.x, ai.y, top + ph}, {0, 0, 1}, mul(pc, 1.05f));
            }
    }
    // rooftop: lift machine rooms / stair heads over each staircase, ventilation boxes
    if (flat && h >= 12 && area > 150 && bi.landmark == Landmark::None) {
        std::vector<V2> spots;
        for (auto &dr : b.doors)
            if (dr.kind == "staircase") spots.push_back(dr.p - dr.out * 3.5);
        if (spots.empty()) spots.push_back(centroid(outer));
        for (size_t k = 0; k < spots.size(); k++) {
            double w = 2.6 + hash01(seed, 4 + k) * 1.2, dd = 3 + hash01(seed, 9) * 1.5, bh = 2.4 + hash01(seed, 14) * 0.8;
            V2 s = spots[k];
            if (point_in_poly(s + V2(-w / 2, -dd / 2), b.rings) && point_in_poly(s + V2(w / 2, dd / 2), b.rings) && point_in_poly(s + V2(-w / 2, dd / 2), b.rings) &&
                point_in_poly(s + V2(w / 2, -dd / 2), b.rings))
                C.box({s.x - w / 2, s.y - dd / 2, top}, {s.x + w / 2, s.y + dd / 2, top + bh}, mul(wall, 0.78f));
        }
        // a few vents / AC boxes
        V2 cc, ax;
        double L, Wd;
        footprint_frame(outer, cc, ax, L, Wd);
        int nv = (int)std::min(6.0, area / 300);
        for (int k = 0; k < nv; k++) {
            V2 p = cc + ax * ((hash01(seed, 40 + k) - 0.5) * L * 0.7) + ax.perp() * ((hash01(seed, 60 + k) - 0.5) * Wd * 0.6);
            if (!point_in_poly(p, b.rings)) continue;
            double s = 0.6 + hash01(seed, 80 + k) * 0.8;
            ms[M_METAL].box({p.x - s / 2, p.y - s / 2, top}, {p.x + s / 2, p.y + s / 2, top + s * 0.8}, Col{0.75f, 0.76f, 0.78f, 1});
        }
    }
    // doors on each wall
    auto doors_on = [&](V2 a, V2 c) {
        std::vector<double> out;
        V2 dd = c - a;
        double l = dd.len();
        if (l < 1e-6) return out;
        V2 dir = dd * (1 / l), o = V2(dd.y, -dd.x) * (1 / l);
        for (auto &e : b.doors) {
            if (e.out.dot(o) < 0.95) continue;
            double s = (e.p - a).dot(dir), off = (e.p - a).dot(o);
            if (s > -0.5 && s < l + 0.5 && std::abs(off) < 0.5) out.push_back(s);
        }
        return out;
    };
    // a wall-coloured panel over the facade hides the texture's windows behind added elements
    auto cover = [&](V2 a, V2 dir, V2 o, double s0, double s1, double z0, double z1, Col cl) {
        auto P = [&](double s, double z) { V2 q = a + dir * s + o * 0.025; return V3(q.x, q.y, z); };
        C.quad(P(s0, z0), P(s1, z0), P(s1, z1), P(s0, z1), {o.x, o.y, 0}, cl, 0, 0, (s1 - s0) / 3, (z1 - z0) / 3);
    };
    // mapped shops: a storefront at each place's position, with its name on the sign board
    if (ground && !b.pois.empty()) {
        bool cyr = cyrillic_region(b.lat, b.lon);
        std::vector<std::vector<std::pair<double, double>>> used(outer.size());
        int made = 0;
        for (auto &poi : b.pois) {
            if (made >= 8) break;
            int wi = -1;
            double bestd = 25;
            for (size_t i = 0; i < outer.size(); i++) {
                double dd = seg_dist(poi.p, outer[i], outer[(i + 1) % outer.size()]);
                double l0 = (outer[(i + 1) % outer.size()] - outer[i]).len();
                if (l0 >= 4.5 && dd < bestd) bestd = dd, wi = (int)i;
            }
            if (wi < 0) continue;
            V2 a = outer[wi], cc = outer[(wi + 1) % outer.size()];
            V2 dd = cc - a;
            double l = dd.len();
            V2 dir = dd * (1 / l), o = V2(dd.y, -dd.x) * (1 / l);
            double wd = std::clamp(std::min(l - 1.2, 6.5), 3.0, 8.0);
            double s = std::clamp((poi.p - a).dot(dir), 0.6 + wd / 2, std::max(0.6 + wd / 2, l - 0.6 - wd / 2));
            double s0 = s - wd / 2, s1 = s + wd / 2;
            bool clash = false;
            for (auto &iv : used[wi])
                if (s0 < iv.second + 0.2 && s1 > iv.first - 0.2) clash = true;
            if (clash) {
                // slide along the wall to the nearest free stretch
                bool ok = false;
                for (double sh = wd + 0.3; sh < l && !ok; sh += wd + 0.3)
                    for (double sg : {1.0, -1.0}) {
                        double t0 = s0 + sg * sh, t1 = s1 + sg * sh;
                        if (t0 < 0.5 || t1 > l - 0.5) continue;
                        bool c2 = false;
                        for (auto &iv : used[wi])
                            if (t0 < iv.second + 0.2 && t1 > iv.first - 0.2) c2 = true;
                        if (!c2) {
                            s0 = t0, s1 = t1, ok = true;
                            break;
                        }
                    }
                if (!ok) continue;
            }
            used[wi].push_back({s0, s1});
            made++;
            ShopStyle st = shop_style(poi.key, poi.value, poi.name, poi.brand, cyr);
            auto P = [&](double ss, double z, double off = 0.07) { V2 q = a + dir * ss + o * off; return V3(q.x, q.y, z); };
            V3 nrm(o.x, o.y, 0);
            double topz = base + std::min<double>(floor_h, 4.5);
            double z0 = base + std::max(0.35f, d.plinth_h);
            // wall panel behind, glazing, door at one end, mullions
            cover(a, dir, o, s0 - 0.1, s1 + 0.1, z0 - 0.3, topz, mul(wall, 0.9f));
            double dw = 1.1, ds0 = ((int)(hash01(seed, 300 + made) * 2) ? s1 - dw - 0.15 : s0 + 0.15), ds1 = ds0 + dw;
            ms[M_GLASS].quad(P(s0 + 0.1, z0), P(s1 - 0.1, z0), P(s1 - 0.1, topz - 0.7), P(s0 + 0.1, topz - 0.7), nrm, Col{1, 1, 1, 1}, 0, 0, (s1 - s0) / 3, 1);
            ms[M_DARK].quad(P(ds0, z0, 0.09), P(ds1, z0, 0.09), P(ds1, z0 + 2.15, 0.09), P(ds0, z0 + 2.15, 0.09), nrm, Col{1, 1, 1, 1});
            Col fr{0.22f, 0.23f, 0.25f, 1};
            for (double ss : {s0 + 0.1, s1 - 0.1, ds0 - 0.04, ds1})
                ms[M_METAL].quad(P(ss, z0, 0.1), P(ss + 0.05, z0, 0.1), P(ss + 0.05, topz - 0.7, 0.1), P(ss, topz - 0.7, 0.1), nrm, fr);
            ms[M_METAL].quad(P(s0 + 0.1, topz - 0.75, 0.1), P(s1 - 0.1, topz - 0.75, 0.1), P(s1 - 0.1, topz - 0.7, 0.1), P(s0 + 0.1, topz - 0.7, 0.1), nrm, fr);
            // sign board with a thin frame
            double bz0 = topz - 0.68, bz1 = topz - 0.05;
            ms[M_METAL].quad(P(s0 - 0.05, bz0 - 0.04, 0.085), P(s1 + 0.05, bz0 - 0.04, 0.085), P(s1 + 0.05, bz1 + 0.04, 0.085), P(s0 - 0.05, bz1 + 0.04, 0.085), nrm, mul(st.board, 0.55f));
            ms[M_PAINT].quad(P(s0, bz0, 0.1), P(s1, bz0, 0.1), P(s1, bz1, 0.1), P(s0, bz1, 0.1), nrm, st.board);
            double tx0 = s0;
            if (st.icon == 1) {  // cross
                double cx = s0 + 0.5, cz = (bz0 + bz1) / 2;
                ms[M_PAINT].quad(P(cx - 0.22, cz - 0.07, 0.11), P(cx + 0.22, cz - 0.07, 0.11), P(cx + 0.22, cz + 0.07, 0.11), P(cx - 0.22, cz + 0.07, 0.11), nrm, Col{1, 1, 1, 1});
                ms[M_PAINT].quad(P(cx - 0.07, cz - 0.22, 0.11), P(cx + 0.07, cz - 0.22, 0.11), P(cx + 0.07, cz + 0.22, 0.11), P(cx - 0.07, cz + 0.22, 0.11), nrm, Col{1, 1, 1, 1});
                tx0 += 0.9;
            } else if (st.icon == 2) {  // cup
                double cx = s0 + 0.5, cz = (bz0 + bz1) / 2;
                ms[M_PAINT].quad(P(cx - 0.14, cz - 0.15, 0.11), P(cx + 0.1, cz - 0.15, 0.11), P(cx + 0.1, cz + 0.12, 0.11), P(cx - 0.14, cz + 0.12, 0.11), nrm, st.text);
                ms[M_PAINT].quad(P(cx + 0.1, cz - 0.07, 0.11), P(cx + 0.2, cz - 0.07, 0.11), P(cx + 0.2, cz + 0.05, 0.11), P(cx + 0.1, cz + 0.05, 0.11), nrm, st.text);
                tx0 += 0.9;
            }
            // awning over the window
            if (st.awning_on && wd >= 3.5) {
                double az = topz - 0.72;
                V3 p0 = P(s0, az, 0.07), p1 = P(s1, az, 0.07), p2 = P(s1, az - 0.45, 1.0), p3 = P(s0, az - 0.45, 1.0);
                ms[M_PAINT].quadn(p0, p3, p2, p1, st.awning);
                ms[M_PAINT].quadn(p0, p1, p2, p3, mul(st.awning, 0.75f));
            }
            if (labels) {
                std::string text = poi.name.empty() ? st.word : poi.name;
                V2 q = a + dir * ((tx0 + s1) / 2) + o * 0.12;
                labels->push_back({V3(q.x, q.y, (bz0 + bz1) / 2), std::atan2(o.y, o.x), (s1 - tx0) - 0.3, text, st.text, (bz1 - bz0) - 0.12});
            }
        }
    }
    // shop fronts on the ground floor (not on walls with staircase doors: those face the yard)
    if (ground && d.shops && b.pois.empty()) {
        for (size_t i = 0; i < outer.size(); i++) {
            V2 a = outer[i], c = outer[(i + 1) % outer.size()];
            V2 dd = c - a;
            double l = dd.len();
            if (l < 6 || (!doors_on(a, c).empty() && d.matched)) continue;
            V2 dir = dd * (1 / l), o = V2(dd.y, -dd.x) * (1 / l);
            auto P = [&](double s, double z) { V2 q = a + dir * s + o * 0.06; return V3(q.x, q.y, z); };
            double topz = base + std::min<double>(floor_h, 4.5);
            double z0 = base + std::max(0.4f, d.plinth_h);
            ms[M_GLASS].quad(P(1, z0), P(l - 1, z0), P(l - 1, topz - 0.6), P(1, topz - 0.6), {o.x, o.y, 0}, Col{1, 1, 1, 1}, 0, 0, (l - 2) / 3, 1);
            static const Col SIGN[] = {{0.75f, 0.12f, 0.1f}, {0.1f, 0.3f, 0.65f}, {0.15f, 0.5f, 0.25f}, {0.85f, 0.65f, 0.1f}, {0.2f, 0.2f, 0.22f}};
            Col sg = SIGN[((seed >> 5) + i) % 5];
            ms[M_PAINT].quad(P(1, topz - 0.55), P(l - 1, topz - 0.55), P(l - 1, topz - 0.1), P(1, topz - 0.1), {o.x, o.y, 0}, sg);
            // mullions
            for (double s = 1 + 2.5; s < l - 1.2; s += 2.5) {
                V2 q = a + dir * s + o * 0.08;
                ms[M_METAL].box({q.x - 0.04, q.y - 0.04, z0}, {q.x + 0.04, q.y + 0.04, topz - 0.6}, Col{0.25f, 0.26f, 0.28f, 1});
            }
        }
    }
    // stairwell windows above staircase doors
    if (d.stair != StairWin::None && floors >= 2 && ground) {
        for (size_t i = 0; i < outer.size(); i++) {
            V2 a = outer[i], c = outer[(i + 1) % outer.size()];
            V2 dd = c - a;
            double l = dd.len();
            if (l < 3) continue;
            V2 dir = dd * (1 / l), o = V2(dd.y, -dd.x) * (1 / l);
            for (double s : doors_on(a, c)) {
                double bay = std::max(2.4f, d.bay);
                double s0 = std::max(0.1, s - bay / 2), s1 = std::min(l - 0.1, s + bay / 2);
                cover(a, dir, o, s0, s1, base + 2.9, top - 0.15, mul(wall, 0.97f));
                auto P = [&](double ss, double z) { V2 q = a + dir * ss + o * 0.05; return V3(q.x, q.y, z); };
                double hw = 0.6;
                if (d.stair == StairWin::Band) {
                    ms[M_GLASS].quad(P(s - hw, base + 3.2), P(s + hw, base + 3.2), P(s + hw, top - 0.6), P(s - hw, top - 0.6), {o.x, o.y, 0}, Col{1, 1, 1, 1});
                    for (int fl = 1; fl < floors; fl++) {
                        double z = base + fl * floor_h + floor_h * 0.5;
                        ms[M_PAINT].quad(P(s - hw, z), P(s + hw, z), P(s + hw, z + 0.08), P(s - hw, z + 0.08), {o.x, o.y, 0}, Col{0.95f, 0.95f, 0.95f, 1});
                    }
                } else {
                    for (int fl = 1; fl < std::max(1, floors); fl++) {
                        double z0 = base + fl * floor_h + floor_h * 0.5 - 0.15;
                        if (z0 + 1.1 > top - 0.3) break;
                        ms[M_GLASS].quad(P(s - hw, z0), P(s + hw, z0), P(s + hw, z0 + 1.1), P(s - hw, z0 + 1.1), {o.x, o.y, 0}, Col{1, 1, 1, 1});
                        Col fr{0.95f, 0.95f, 0.95f, 1};
                        ms[M_PAINT].quad(P(s - hw - 0.08, z0 - 0.08), P(s + hw + 0.08, z0 - 0.08), P(s + hw + 0.08, z0), P(s - hw - 0.08, z0), {o.x, o.y, 0}, fr);
                        ms[M_PAINT].quad(P(s - hw - 0.08, z0 + 1.1), P(s + hw + 0.08, z0 + 1.1), P(s + hw + 0.08, z0 + 1.18), P(s - hw - 0.08, z0 + 1.18), {o.x, o.y, 0}, fr);
                    }
                }
            }
        }
    }
    // a low technical top floor: plain band with small vents
    if (d.tech_floor && floors >= 6 && flat) {
        double tz = top - 1.8;
        for (size_t i = 0; i < outer.size(); i++) {
            V2 a = outer[i], c = outer[(i + 1) % outer.size()];
            V2 dd = c - a;
            double l = dd.len();
            if (l < 2) continue;
            V2 dir = dd * (1 / l), o = V2(dd.y, -dd.x) * (1 / l);
            cover(a, dir, o, 0, l, tz, top, mul(wall, 0.93f));
            auto P = [&](double ss, double z) { V2 q = a + dir * ss + o * 0.05; return V3(q.x, q.y, z); };
            for (double s = 1.5; s + 0.6 < l - 0.5; s += 3) ms[M_DARK].quad(P(s, tz + 0.6), P(s + 0.6, tz + 0.6), P(s + 0.6, tz + 1.0), P(s, tz + 1.0), {o.x, o.y, 0}, Col{1, 1, 1, 1});
        }
    }
    // balconies / loggias in vertical stacks, clear of the stairwell column
    if (d.balconies != Balconies::None && floors >= 2 && ground) {
        double bay = std::max(2.4f, d.bay);
        double depth = d.balconies == Balconies::Railing ? 0.25 : 0.9 + hash01(seed, 22) * 0.4;
        int stacks = 0;
        for (size_t i = 0; i < outer.size(); i++) {
            V2 a = outer[i], c = outer[(i + 1) % outer.size()];
            V2 dd = c - a;
            double l = dd.len();
            if (l < 9 || hash01(seed, 30 + i) >= d.balcony_sides) continue;
            V2 dir = dd * (1 / l), o = V2(dd.y, -dd.x) * (1 / l);
            auto ds = doors_on(a, c);
            int bays = (int)std::floor(l / bay);
            for (int k = 1; k < bays - 1; k++) {
                if (k % std::max(1, d.balcony_every)) continue;
                double s0 = (l - bays * bay) / 2 + k * bay + 0.3, s1 = s0 + bay - 0.6;
                bool clash = false;
                for (double s : ds)
                    if (s > s0 - 1.2 && s < s1 + 1.2) clash = true;
                if (clash) continue;
                if (++stacks > 60) break;
                for (int fl = 1; fl < floors; fl++) {
                    double z = base + fl * floor_h;
                    auto P = [&](double s, double dpt, double zz) { V2 q = a + dir * s + o * dpt; return V3(q.x, q.y, zz); };
                    Col sc = mul(wall, 0.92f);
                    uint64_t hb = seed ^ ((uint64_t)(i * 131 + k) * 2654435761ull) ^ ((uint64_t)fl * 40503ull);
                    double u = hash01(hb, 1);
                    // vertical bars between a bottom and a top rail
                    auto railing = [&](double s0r, double s1r, double dpt, double h, Col rc) {
                        ms[M_METAL].quad(P(s0r, dpt, z + h - 0.05), P(s1r, dpt, z + h - 0.05), P(s1r, dpt, z + h), P(s0r, dpt, z + h), {o.x, o.y, 0}, rc);
                        ms[M_METAL].quad(P(s0r, dpt, z + 0.1), P(s1r, dpt, z + 0.1), P(s1r, dpt, z + 0.14), P(s0r, dpt, z + 0.14), {o.x, o.y, 0}, rc);
                        int nb = std::max(2, (int)((s1r - s0r) / 0.14));
                        for (int q = 0; q <= nb; q++) {
                            double sb = s0r + (s1r - s0r) * q / nb;
                            ms[M_METAL].quad(P(sb - 0.012, dpt, z + 0.14), P(sb + 0.012, dpt, z + 0.14), P(sb + 0.012, dpt, z + h - 0.05), P(sb - 0.012, dpt, z + h - 0.05), {o.x, o.y, 0}, rc);
                        }
                    };
                    if (d.balconies == Balconies::Railing) {
                        // French balcony: a shallow ledge with wrought-iron bars
                        C.quad(P(s0 + 0.2, 0, z + 0.05), P(s1 - 0.2, 0, z + 0.05), P(s1 - 0.2, depth, z + 0.05), P(s0 + 0.2, depth, z + 0.05), {0, 0, 1}, sc);
                        railing(s0 + 0.2, s1 - 0.2, depth, 1.0, Col{0.12f, 0.12f, 0.13f, 1});
                        continue;
                    }
                    // slab
                    C.quad(P(s0, 0, z), P(s1, 0, z), P(s1, depth, z), P(s0, depth, z), {0, 0, 1}, sc);
                    C.quad(P(s1, 0, z - 0.15), P(s0, 0, z - 0.15), P(s0, depth, z - 0.15), P(s1, depth, z - 0.15), {0, 0, -1}, mul(sc, 0.8f));
                    C.quad(P(s0, depth, z - 0.15), P(s1, depth, z - 0.15), P(s1, depth, z), P(s0, depth, z), {o.x, o.y, 0}, mul(sc, 0.85f));
                    // the parapet: the building's own panel, or what the owner put up (corrugated sheet in some
                    // colour), or just a railing
                    bool glazed = d.balconies == Balconies::Glazed ? u < 0.72 : u < 0.22;  // residents glaze their own
                    double ph = 1.0;
                    int ptype = (int)(hash01(hb, 2) * 3);
                    if (!glazed && ptype == 2) {
                        railing(s0, s1, depth, ph, Col{0.2f, 0.2f, 0.21f, 1});
                    } else {
                        Col pc = sc;
                        if (glazed && ptype != 0) {
                            static const Col SHEET[] = {{0.93f, 0.92f, 0.88f, 1}, {0.85f, 0.78f, 0.62f, 1}, {0.55f, 0.36f, 0.26f, 1}, {0.55f, 0.62f, 0.72f, 1}, {0.75f, 0.76f, 0.74f, 1}};
                            pc = SHEET[(int)(hash01(hb, 3) * 5) % 5];
                        }
                        C.quad(P(s0, depth, z), P(s1, depth, z), P(s1, depth, z + ph), P(s0, depth, z + ph), {o.x, o.y, 0}, pc, 0, 0, (s1 - s0) / 3, 0.4);
                    }
                    C.quad(P(s0, 0, z - 0.15), P(s0, depth, z - 0.15), P(s0, depth, z + ph), P(s0, 0, z + ph), {-dir.x, -dir.y, 0}, sc);
                    C.quad(P(s1, depth, z - 0.15), P(s1, 0, z - 0.15), P(s1, 0, z + ph), P(s1, depth, z + ph), {dir.x, dir.y, 0}, sc);
                    if (glazed) {
                        double top = z + floor_h - 0.2;
                        ms[M_GLASS].quad(P(s0, depth, z + ph), P(s1, depth, z + ph), P(s1, depth, top), P(s0, depth, top), {o.x, o.y, 0}, Col{1, 1, 1, 1});
                        // frames: white PVC or brown, a mullion every ~0.8 m, a transom rail
                        Col fc = hash01(hb, 4) < 0.7 ? Col{0.92f, 0.92f, 0.9f, 1} : Col{0.35f, 0.25f, 0.18f, 1};
                        int nm = std::max(2, (int)std::round((s1 - s0) / 0.8));
                        for (int q = 0; q <= nm; q++) {
                            double sm = s0 + (s1 - s0) * q / nm;
                            ms[M_PAINT].quad(P(sm - 0.03, depth + 0.01, z + ph), P(sm + 0.03, depth + 0.01, z + ph), P(sm + 0.03, depth + 0.01, top), P(sm - 0.03, depth + 0.01, top), {o.x, o.y, 0}, fc);
                        }
                        for (double zz : {z + ph, top - 0.05})
                            ms[M_PAINT].quad(P(s0, depth + 0.01, zz), P(s1, depth + 0.01, zz), P(s1, depth + 0.01, zz + 0.06), P(s0, depth + 0.01, zz + 0.06), {o.x, o.y, 0}, fc);
                        // a roof over the top floor's glazing
                        if (fl == floors - 1) C.quad(P(s0, 0, top + 0.05), P(s1, 0, top + 0.05), P(s1, depth + 0.1, top), P(s0, depth + 0.1, top), {0, 0, 1}, Col{0.45f, 0.45f, 0.46f, 1});
                    } else if (hash01(hb, 5) < 0.12) {
                        // an air-conditioner on the balcony
                        V2 ac = a + dir * (s0 + 0.5) + o * (depth - 0.2);
                        ms[M_PAINT].obox(ac, std::atan2(o.y, o.x), 0.3, 0.75, z + 0.05, z + 0.6, Col{0.9f, 0.9f, 0.88f, 1});
                    }
                }
            }
        }
    }
    // entrances: door, frame, porch slab with a step, canopy
    for (auto &e : b.doors) {
        V2 o = e.out, t = o.perp();
        double w = e.kind == "garage" ? 2.6 : (e.kind == "service" || e.kind == "emergency" || e.kind == "exit") ? 1.0 : e.kind == "main" ? 2.0 : 1.4;
        bool canopy = !(e.kind == "garage" || e.kind == "service" || e.kind == "emergency") && d.canopy != Canopy::None;
        double porch = canopy ? 0.3 : 0.0;
        double gz = base;
        auto at = [&](double s, double dd, double z) { V2 q = e.p + t * s + o * dd; return V3(q.x, q.y, gz + z); };
        // door leaf (dark glass), frame
        ms[M_DARK].quad(at(-w / 2, 0.04, porch), at(w / 2, 0.04, porch), at(w / 2, 0.04, porch + 2.2), at(-w / 2, 0.04, porch + 2.2), {o.x, o.y, 0}, Col{1, 1, 1, 1});
        Col fr{0.6f, 0.62f, 0.64f, 1};
        ms[M_METAL].quad(at(-w / 2 - 0.12, 0.06, porch), at(-w / 2, 0.06, porch), at(-w / 2, 0.06, porch + 2.32), at(-w / 2 - 0.12, 0.06, porch + 2.32), {o.x, o.y, 0}, fr);
        ms[M_METAL].quad(at(w / 2, 0.06, porch), at(w / 2 + 0.12, 0.06, porch), at(w / 2 + 0.12, 0.06, porch + 2.32), at(w / 2, 0.06, porch + 2.32), {o.x, o.y, 0}, fr);
        ms[M_METAL].quad(at(-w / 2 - 0.12, 0.06, porch + 2.2), at(w / 2 + 0.12, 0.06, porch + 2.2), at(w / 2 + 0.12, 0.06, porch + 2.32), at(-w / 2 - 0.12, 0.06, porch + 2.32), {o.x, o.y, 0}, fr);
        if (!canopy) continue;
        V2 pc = e.p + o * 0.8;
        double yaw = std::atan2(o.y, o.x);
        C.obox(pc, yaw, 1.6, w + 1.2, gz - 0.5, gz + porch, Col{0.62f, 0.61f, 0.6f, 1});
        if (d.canopy == Canopy::Gable) {
            double hw = w / 2 + 0.4, z0 = porch + 2.5, rise = 0.6;
            V3 r0 = at(0, 0, z0 + rise), r1 = at(0, 1.0, z0 + rise);
            ms[M_ROOF_TILE].quadn(at(-hw, 0, z0), at(-hw, 1.0, z0), r1, r0, bi.roof_colour);
            ms[M_ROOF_TILE].quadn(at(hw, 1.0, z0), at(hw, 0, z0), r0, r1, bi.roof_colour);
        } else {
            V2 cc = e.p + o * 0.65;
            C.obox(cc, yaw, 1.3, w + 1.0, gz + porch + 2.55, gz + porch + 2.7, Col{0.7f, 0.7f, 0.68f, 1}, true);
        }
    }
    // shopping centres and big stores: LED screens and the name in large letters on the main facade
    bool big_shop = bi.use == Use::Retail && area >= 1200 && bi.min_height < 0.1;
    bool venue = (b.amenity == "cinema" || b.amenity == "theatre") && area >= 400;
    if ((big_shop || venue) && h >= 6) {
        // the facade: the wall with the main door, else the longest
        int wi = -1;
        double wl = 0;
        for (size_t i = 0; i < outer.size(); i++) {
            V2 a = outer[i], c2 = outer[(i + 1) % outer.size()];
            double l = (c2 - a).len();
            bool door = false;
            for (auto &dr : b.doors)
                if (dr.kind == "main" && seg_dist(dr.p, a, c2) < 0.5) door = true;
            double score = l + (door ? 1000 : 0);
            if (l >= 12 && score > wl) wl = score, wi = (int)i;
        }
        if (wi >= 0) {
            V2 a = outer[wi], c2 = outer[(wi + 1) % outer.size()];
            double l = (c2 - a).len();
            V2 dir = (c2 - a) * (1 / l), o(dir.y, -dir.x);
            V3 nrm(o.x, o.y, 0);
            auto P = [&](double s, double z, double off) { V2 q = a + dir * s + o * off; return V3(q.x, q.y, z); };
            double wt = std::min(wall_top, top) - 0.6;
            uint64_t sd = bi.seed;
            sd ^= (uint64_t)std::llround(a.x * 10) * 2654435761ull ^ (uint64_t)std::llround(a.y * 10);
            // one or two screens, to the side of the centre where the name goes
            double sw = std::min({l * 0.32, 14.0, (wt - base - 3.5) * 1.8});
            double sh = std::min(sw * 0.56, wt - base - 3.5);
            if (sw >= 3 && sh >= 1.8) {
                int count = l > 50 ? 2 : 1;
                for (int k = 0; k < count; k++) {
                    double sc = count == 1 ? l * 0.75 : (k == 0 ? l * 0.2 : l * 0.8);
                    double s0 = std::clamp(sc - sw / 2, 0.5, l - sw - 0.5), s1 = s0 + sw;
                    double z1 = wt - 0.3, z0 = z1 - sh;
                    ms[M_DARK].quad(P(s0 - 0.25, z0 - 0.25, 0.12), P(s1 + 0.25, z0 - 0.25, 0.12), P(s1 + 0.25, z1 + 0.25, 0.12), P(s0 - 0.25, z1 + 0.25, 0.12), nrm, Col{1, 1, 1, 1});
                    // the screen: uv 0..1, colour carries a per-screen seed for the shader's programme
                    float seed = (float)((sd >> (k * 8)) % 97) / 97.0f;
                    MeshBuf &S = ms[M_SCREEN];
                    int i0 = S.v(P(s0, z0, 0.16), nrm, 0, 1, Col{seed, (float)(sw / sh) / 4, 0, 1}), i1 = S.v(P(s1, z0, 0.16), nrm, 1, 1, Col{seed, (float)(sw / sh) / 4, 0, 1});
                    int i2 = S.v(P(s1, z1, 0.16), nrm, 1, 0, Col{seed, (float)(sw / sh) / 4, 0, 1}), i3 = S.v(P(s0, z1, 0.16), nrm, 0, 0, Col{seed, (float)(sw / sh) / 4, 0, 1});
                    S.tri(i0, i1, i2);
                    S.tri(i0, i2, i3);
                    // depth of the box
                    ms[M_DARK].quad(P(s0 - 0.25, z1 + 0.25, 0.12), P(s1 + 0.25, z1 + 0.25, 0.12), P(s1 + 0.25, z1 + 0.25, 0.0), P(s0 - 0.25, z1 + 0.25, 0.0), V3(0, 0, 1), Col{1, 1, 1, 1});
                }
            }
            // name in big letters above the entrance
            std::string nm = b.name;
            if (labels && !nm.empty() && utf8_len(nm) <= 28) {
                double lw = std::min(l * 0.45, 3.0 + 1.1 * utf8_len(nm));
                double lz = std::min(wt - 0.4, base + std::max(5.0, (wt - base) * 0.7));
                V2 q = a + dir * (l * (big_shop && l > 50 ? 0.5 : 0.3)) + o * 0.2;
                if (l <= 50 && sw >= 3) q = a + dir * std::max(lw / 2 + 0.5, l * 0.75 - sw / 2 - lw / 2 - 1.0) + o * 0.2;
                Col lc = (sd % 3 == 0) ? Col{0.95f, 0.15f, 0.15f, 1} : (sd % 3 == 1) ? Col{1, 1, 1, 1} : Col{1.0f, 0.75f, 0.1f, 1};
                labels->push_back({V3(q.x, q.y, lz), std::atan2(o.y, o.x), lw, nm, lc, std::max(0.8, lw * 0.22)});
            }
        }
    }
    landmark(b, base, h, ms);
}

}  // namespace ec
