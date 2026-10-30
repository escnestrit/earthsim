#include "chunk.hpp"
#include "infra.hpp"
#include <algorithm>
#include <cstring>
#include "chunk_ctx.hpp"

namespace ec {

Bounds chunk_bounds(int cx, int cy, const LocalFrame &fr, double m) {
    double la0, lo0, la1, lo1;
    fr.to_latlon({cx * CHUNK - m, cy * CHUNK - m}, la0, lo0);
    fr.to_latlon({(cx + 1) * CHUNK + m, (cy + 1) * CHUNK + m}, la1, lo1);
    return {la0, lo0, la1, lo1};
}

namespace {

uint8_t layer_of(AreaKind k) {
    switch (k) {
        case AreaKind::Forest: return L_FOREST;
        case AreaKind::Sand: return L_SAND;
        case AreaKind::Parking: return L_ASPHALT;
        case AreaKind::Pitch: return L_PITCH;
        case AreaKind::Farmland: return L_FARM;
        case AreaKind::Water: return L_SAND;
        default: return L_GRASS;
    }
}

// Sutherland-Hodgman against an axis box
Ring clip_box(const Ring &r, double x0, double y0, double x1, double y1) {
    Ring cur = r;
    for (int e = 0; e < 4 && !cur.empty(); e++) {
        Ring nxt;
        auto in = [&](V2 p) { return e == 0 ? p.x >= x0 : e == 1 ? p.x <= x1 : e == 2 ? p.y >= y0 : p.y <= y1; };
        auto isect = [&](V2 a, V2 b) {
            double t = e == 0 ? (x0 - a.x) / (b.x - a.x) : e == 1 ? (x1 - a.x) / (b.x - a.x) : e == 2 ? (y0 - a.y) / (b.y - a.y) : (y1 - a.y) / (b.y - a.y);
            return a + (b - a) * t;
        };
        for (size_t i = 0, n = cur.size(); i < n; i++) {
            V2 a = cur[i], b = cur[(i + 1) % n];
            bool ia = in(a), ib = in(b);
            if (ia) nxt.push_back(a);
            if (ia != ib) nxt.push_back(isect(a, b));
        }
        cur.swap(nxt);
    }
    return cur;
}

std::vector<Ring> to_local(const LocalFrame &fr, const std::vector<std::vector<LL>> &rings, V2 &lo, V2 &hi) {
    std::vector<Ring> out;
    lo = V2(1e300, 1e300), hi = V2(-1e300, -1e300);
    for (auto &r : rings) {
        Ring q;
        for (auto &p : r) {
            V2 v = fr.to_local(p.lat, p.lon);
            if (!q.empty() && (v - q.back()).len() < 0.05) continue;
            q.push_back(v);
            lo = V2(std::min(lo.x, v.x), std::min(lo.y, v.y)), hi = V2(std::max(hi.x, v.x), std::max(hi.y, v.y));
        }
        if (q.size() >= 3 && (q.front() - q.back()).len() < 0.05) q.pop_back();
        if (q.size() >= 3) out.push_back(std::move(q));
    }
    if (!out.empty()) {
        make_ccw(out[0]);
        for (size_t k = 1; k < out.size(); k++) make_cw(out[k]);
    }
    return out;
}

// grid samples inside a polygon (even-odd), calling f(index)
template <class F>
void grid_fill(const HeightGrid &g, const std::vector<Ring> &rings, F f) {
    int N = g.n + 1;
    std::vector<double> xs;
    double ylo = 1e300, yhi = -1e300;
    for (auto &r : rings)
        for (auto &p : r) ylo = std::min(ylo, p.y), yhi = std::max(yhi, p.y);
    int j0 = std::max(0, (int)std::floor((ylo - g.y0) / g.cell)), j1 = std::min(N - 1, (int)std::ceil((yhi - g.y0) / g.cell));
    for (int j = j0; j <= j1; j++) {
        double y = g.y0 + j * g.cell + 1e-7;
        xs.clear();
        for (auto &r : rings)
            for (size_t i = 0, n = r.size(); i < n; i++) {
                V2 a = r[i], b = r[(i + 1) % n];
                if ((a.y > y) != (b.y > y)) xs.push_back(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
            }
        std::sort(xs.begin(), xs.end());
        for (size_t k = 0; k + 1 < xs.size(); k += 2) {
            int i0 = std::max(0, (int)std::ceil((xs[k] - g.x0) / g.cell)), i1 = std::min(N - 1, (int)std::floor((xs[k + 1] - g.x0) / g.cell));
            for (int i = i0; i <= i1; i++) f(j * N + i);
        }
    }
}

struct WaterLine {
    std::vector<V2> p;
    std::vector<double> lvl;
    double hw;
};

double line_level(const std::vector<WaterLine> &lines, V2 q, double maxd, bool &found) {
    double best = 1e300, lv = 0;
    for (auto &l : lines)
        for (size_t i = 0; i + 1 < l.p.size(); i++) {
            V2 a = l.p[i], b = l.p[i + 1], d = b - a;
            double l2 = d.dot(d);
            double t = l2 > 0 ? std::clamp((q - a).dot(d) / l2, 0.0, 1.0) : 0;
            double dist = (a + d * t - q).len();
            if (dist < best) best = dist, lv = l.lvl[i] + (l.lvl[i + 1] - l.lvl[i]) * t;
        }
    found = best <= maxd;
    return lv;
}

void water(Ctx &c, const std::vector<std::shared_ptr<const RawTile>> &tiles) {
    HeightGrid &g = c.out.grid;
    int N = g.n + 1;
    // centre lines: rivers, streams, canals
    std::vector<WaterLine> lines;
    for (auto &t : tiles)
        for (auto &[pts, w] : t->waterlines) {
            WaterLine wl;
            wl.hw = std::clamp(w, 1.0, 200.0) / 2;
            V2 lo(1e300, 1e300), hi(-1e300, -1e300);
            for (auto &q : pts) {
                V2 v = c.fr.to_local(q.lat, q.lon);
                wl.p.push_back(v);
                lo = V2(std::min(lo.x, v.x), std::min(lo.y, v.y)), hi = V2(std::max(hi.x, v.x), std::max(hi.y, v.y));
            }
            if (wl.p.size() < 2 || !c.near(lo, hi, FEATURE_MARGIN + wl.hw)) continue;
            wl.p = densify(wl.p, 4);
            for (auto &p : wl.p) wl.lvl.push_back(c.G(p) - 0.35);
            lines.push_back(std::move(wl));
        }
    // polygons
    struct Poly {
        std::vector<Ring> rings;
        double level;
        bool lined;
    };
    std::vector<Poly> polys;
    for (auto &t : tiles)
        for (auto &a : t->areas) {
            if (a.kind != AreaKind::Water) continue;
            V2 lo, hi;
            auto rings = to_local(c.fr, a.rings, lo, hi);
            if (rings.empty() || !c.near(lo, hi, 4)) continue;
            std::vector<double> zs;
            for (auto &q : densify(rings[0], 12)) zs.push_back(c.G(q));
            std::sort(zs.begin(), zs.end());
            double lvl = zs[zs.size() / 20] - 0.25;
            bool lined = false;
            for (auto &l : lines)
                for (size_t i = 0; i + 1 < l.p.size() && !lined; i += 3)
                    if (point_in_poly((l.p[i] + l.p[i + 1]) * 0.5, rings)) lined = true;
            polys.push_back({std::move(rings), lvl, lined});
        }
    auto level_at = [&](const Poly &p, V2 q) {
        if (!p.lined) return p.level;
        bool f;
        double lv = line_level(lines, q, 1e9, f);
        return f ? lv : p.level;
    };
    // carve: mask of grid samples under water and their distance to the shore
    std::vector<float> lvl(N * N, 1e30f);
    std::vector<uint8_t> mask(N * N, 0);
    for (auto &p : polys)
        grid_fill(g, p.rings, [&](int q) {
            mask[q] = 1;
            V2 pt(g.x0 + (q % N) * g.cell, g.y0 + (q / N) * g.cell);
            lvl[q] = std::min(lvl[q], (float)level_at(p, pt));
        });
    for (auto &l : lines)
        for (size_t i = 0; i + 1 < l.p.size(); i++) {
            V2 a = l.p[i], b = l.p[i + 1];
            double R = l.hw;
            int i0 = std::max(0, (int)std::floor((std::min(a.x, b.x) - R - g.x0) / g.cell)), i1 = std::min(N - 1, (int)std::ceil((std::max(a.x, b.x) + R - g.x0) / g.cell));
            int j0 = std::max(0, (int)std::floor((std::min(a.y, b.y) - R - g.y0) / g.cell)), j1 = std::min(N - 1, (int)std::ceil((std::max(a.y, b.y) + R - g.y0) / g.cell));
            V2 d = b - a;
            double l2 = std::max(1e-9, d.dot(d));
            for (int j = j0; j <= j1; j++)
                for (int ii = i0; ii <= i1; ii++) {
                    V2 p(g.x0 + ii * g.cell, g.y0 + j * g.cell);
                    double t = std::clamp((p - a).dot(d) / l2, 0.0, 1.0);
                    if ((a + d * t - p).len() > R) continue;
                    int q = j * N + ii;
                    mask[q] = 1;
                    lvl[q] = std::min(lvl[q], (float)(l.lvl[i] + (l.lvl[i + 1] - l.lvl[i]) * t));
                }
        }
    // chamfer distance to the nearest dry sample
    std::vector<float> dist(N * N);
    for (int q = 0; q < N * N; q++) dist[q] = mask[q] ? 1e9f : 0;
    const float D1 = 1, D2 = 1.4142f;
    for (int j = 0; j < N; j++)
        for (int i = 0; i < N; i++) {
            float &d = dist[j * N + i];
            if (i > 0) d = std::min(d, dist[j * N + i - 1] + D1);
            if (j > 0) {
                d = std::min(d, dist[(j - 1) * N + i] + D1);
                if (i > 0) d = std::min(d, dist[(j - 1) * N + i - 1] + D2);
                if (i + 1 < N) d = std::min(d, dist[(j - 1) * N + i + 1] + D2);
            }
        }
    for (int j = N - 1; j >= 0; j--)
        for (int i = N - 1; i >= 0; i--) {
            float &d = dist[j * N + i];
            if (i + 1 < N) d = std::min(d, dist[j * N + i + 1] + D1);
            if (j + 1 < N) {
                d = std::min(d, dist[(j + 1) * N + i] + D1);
                if (i + 1 < N) d = std::min(d, dist[(j + 1) * N + i + 1] + D2);
                if (i > 0) d = std::min(d, dist[(j + 1) * N + i - 1] + D2);
            }
        }
    for (int q = 0; q < N * N; q++) {
        if (!mask[q]) continue;
        double depth = std::min(3.0, 0.35 + dist[q] * g.cell * 0.3);
        g.h[q] = (float)std::min<double>(g.h[q], lvl[q] - depth);
    }
    // ground cover under and round the water
    for (auto &p : polys) c.fill(p.rings, [&](int t) { c.layer[t] = L_SAND, c.occ[t] |= O_WATER; });
    for (auto &l : lines)
        for (size_t i = 0; i + 1 < l.p.size(); i++) c.stroke(l.p[i], l.p[i + 1], l.hw + 0.5, [&](int t) { c.layer[t] = L_SAND, c.occ[t] |= O_WATER; });
    // surfaces
    MeshBuf &W = c.out.ms[M_WATER];
    Col wc{1, 1, 1, 1};
    for (auto &p : polys) {
        std::vector<Ring> cl;
        for (size_t k = 0; k < p.rings.size(); k++) {
            Ring r = clip_box(p.rings[k], c.x0, c.y0, c.x1, c.y1);
            if (r.size() < 3 || std::abs(signed_area(r)) < 0.5) {
                if (k == 0) break;
                continue;
            }
            cl.push_back(std::move(r));
        }
        if (cl.empty()) continue;
        std::vector<int> tri = triangulate(cl);
        std::vector<V2> pts;
        for (auto &r : cl) pts.insert(pts.end(), r.begin(), r.end());
        std::vector<std::array<V3, 3>> parts;
        for (size_t k = 0; k + 2 < tri.size(); k += 3) {
            V2 a = pts[tri[k]], b = pts[tri[k + 1]], d = pts[tri[k + 2]];
            if ((b - a).cross(d - a) < 0) std::swap(b, d);
            if (p.lined) split_tri({a.x, a.y, 0}, {b.x, b.y, 0}, {d.x, d.y, 0}, 8, 12, parts);
            else parts.push_back({V3(a.x, a.y, 0), V3(b.x, b.y, 0), V3(d.x, d.y, 0)});
        }
        for (auto &t : parts) {
            int i[3];
            for (int k = 0; k < 3; k++) {
                double z = level_at(p, {t[k].x, t[k].y});
                i[k] = W.v({t[k].x, t[k].y, z}, {0, 0, 1}, t[k].x / 8, t[k].y / 8, wc);
            }
            W.tri(i[0], i[1], i[2]);
        }
    }
    for (auto &l : lines) {
        std::vector<V2> n;
        std::vector<double> sc;
        normals(l.p, n, sc);
        for (size_t i = 0; i + 1 < l.p.size(); i++) {
            V2 mid = (l.p[i] + l.p[i + 1]) * 0.5;
            if (!c.own(mid)) continue;
            bool in_poly = false;
            for (auto &p : polys)
                if (point_in_poly(mid, p.rings)) {
                    in_poly = true;
                    break;
                }
            if (in_poly) continue;
            auto P = [&](size_t k, double s) { V2 q = l.p[k] + n[k] * (s * l.hw * sc[k]); return V3(q.x, q.y, l.lvl[k]); };
            V3 a = P(i, -1), b = P(i + 1, -1), d = P(i + 1, 1), e = P(i, 1);
            int ia = W.v(a, {0, 0, 1}, a.x / 8, a.y / 8, wc), ib = W.v(b, {0, 0, 1}, b.x / 8, b.y / 8, wc), id = W.v(d, {0, 0, 1}, d.x / 8, d.y / 8, wc), ie = W.v(e, {0, 0, 1}, e.x / 8, e.y / 8, wc);
            W.tri(ia, ib, id);
            W.tri(ia, id, ie);
        }
    }
}

void areas(Ctx &c, const std::vector<std::shared_ptr<const RawTile>> &tiles) {
    // big, coarse covers first so the specific ones win
    std::vector<std::pair<int, const RawArea *>> order;
    for (auto &t : tiles)
        for (auto &a : t->areas) {
            if (a.kind == AreaKind::Water) continue;
            int rank = a.kind == AreaKind::Farmland ? 0 : a.kind == AreaKind::Forest ? 1 : a.kind == AreaKind::Grass || a.kind == AreaKind::Park ? 2 : 3;
            order.push_back({rank, &a});
        }
    std::stable_sort(order.begin(), order.end(), [](auto &a, auto &b) { return a.first < b.first; });
    for (auto &[rank, a] : order) {
        V2 lo, hi;
        auto rings = to_local(c.fr, a->rings, lo, hi);
        if (rings.empty() || !c.near(lo, hi, 2)) continue;
        uint8_t L = layer_of(a->kind);
        bool park = a->kind == AreaKind::Parking;
        c.fill(rings, [&](int t) {
            c.layer[t] = L;
            if (park) c.occ[t] |= O_PARKING;
        });
    }
}

void paths(Ctx &c, const std::vector<std::shared_ptr<const RawTile>> &tiles) {
    MeshBuf &PV = c.out.ms[M_PAVERS], &GR = c.out.ms[M_GRAVEL];
    for (auto &t : tiles)
        for (auto &rp : t->paths) {
            std::vector<V2> p;
            V2 lo(1e300, 1e300), hi(-1e300, -1e300);
            for (auto &q : rp.pts) {
                V2 v = c.fr.to_local(q.lat, q.lon);
                if (!p.empty() && (v - p.back()).len() < 0.1) continue;
                p.push_back(v);
                lo = V2(std::min(lo.x, v.x), std::min(lo.y, v.y)), hi = V2(std::max(hi.x, v.x), std::max(hi.y, v.y));
            }
            if (p.size() < 2 || !c.near(lo, hi, 4)) continue;
            p = densify(p, 2.0);
            double hw = std::clamp(rp.width, 1.0, 6.0) / 2;
            std::vector<V2> n;
            std::vector<double> sc;
            normals(p, n, sc);
            std::vector<double> z(p.size());
            double L = 0;
            std::vector<double> s(p.size(), 0);
            for (size_t i = 1; i < p.size(); i++) s[i] = s[i - 1] + (p[i] - p[i - 1]).len();
            L = s.back();
            for (size_t i = 0; i < p.size(); i++) {
                if (rp.bridge) {
                    double za = c.G(p.front()), zb = c.G(p.back());
                    auto sm = [](double x) { x = std::clamp(x, 0.0, 1.0); return x * x * (3 - 2 * x); };
                    double R = std::min(L / 2, 15.0);
                    z[i] = za + (zb - za) * (L > 0 ? s[i] / L : 0) + 1.5 * (R > 0 ? sm(s[i] / R) * sm((L - s[i]) / R) : 0) + 0.05;
                } else
                    z[i] = std::max(c.ground(p[i]), c.ground(p[i] + n[i] * hw) - 0.02) + 0.05;
                z[i] = std::max(z[i], std::max(c.ground(p[i] - n[i] * hw), c.ground(p[i] + n[i] * hw)) + 0.035);
            }
            MeshBuf &M = rp.paved ? PV : GR;
            Col col = rp.paved ? Col{0.92f, 0.9f, 0.88f, 1} : Col{1, 1, 1, 1};
            for (size_t i = 0; i + 1 < p.size(); i++) {
                V2 mid = (p[i] + p[i + 1]) * 0.5;
                if (!rp.bridge) c.stroke(p[i], p[i + 1], hw, [&](int t) {
                    if (!(c.occ[t] & (O_ROAD | O_WALK | O_WATER))) c.layer[t] = rp.paved ? L_PAVERS : L_DIRT, c.occ[t] |= O_PATH;
                });
                if (!c.own(mid)) continue;
                if (!rp.bridge && (c.roads.clearance(mid) < 0.6 || (c.occ_at(mid) & O_WATER))) continue;
                bool inb = false;
                for (auto &br : c.building_rings)
                    if (point_in_ring(mid, br)) {
                        inb = true;
                        break;
                    }
                if (inb) continue;
                V3 a(p[i].x - n[i].x * hw * sc[i], p[i].y - n[i].y * hw * sc[i], z[i]), b(p[i + 1].x - n[i + 1].x * hw * sc[i + 1], p[i + 1].y - n[i + 1].y * hw * sc[i + 1], z[i + 1]);
                V3 d(p[i + 1].x + n[i + 1].x * hw * sc[i + 1], p[i + 1].y + n[i + 1].y * hw * sc[i + 1], z[i + 1]), e(p[i].x + n[i].x * hw * sc[i], p[i].y + n[i].y * hw * sc[i], z[i]);
                M.quad(a, b, d, e, {0, 0, 1}, col, -hw / 2, s[i] / 2, hw / 2, s[i + 1] / 2);
                c.out.walks.push_back({V3(p[i].x, p[i].y, z[i]), V3(p[i + 1].x, p[i + 1].y, z[i + 1])});
                c.tri_coll(a, b, d);
                c.tri_coll(a, d, e);
                if (rp.bridge) {
                    MeshBuf &CO = c.out.ms[M_METAL];
                    Col rc{0.35f, 0.37f, 0.4f, 1};
                    for (int sd : {-1, 1}) {
                        V3 q0 = sd < 0 ? a : e, q1 = sd < 0 ? b : d;
                        V3 u0 = q0 + V3(0, 0, 1.0), u1 = q1 + V3(0, 0, 1.0);
                        CO.quadn(q0, q1, u1, u0, rc);
                        CO.quadn(q1, q0, u0, u1, rc);
                    }
                    V3 a2 = a - V3(0, 0, 0.4), b2 = b - V3(0, 0, 0.4), d2 = d - V3(0, 0, 0.4), e2 = e - V3(0, 0, 0.4);
                    c.out.ms[M_CONCRETE].quadn(e2, d2, b2, a2, {0.6f, 0.6f, 0.6f, 1});
                }
            }
            if (c.paths.segs.size() < 200000)
                for (size_t i = 0; i + 1 < p.size(); i++) c.paths.add(p[i], p[i + 1], hw, 0);
        }
}

struct BRef {
    const RawBuilding *b;
    std::vector<Ring> rings;
    V2 ctr;
    double area;
};

void buildings(Ctx &c, const std::vector<std::shared_ptr<const RawTile>> &tiles, std::vector<BRef> &all) {
    // entrances
    std::vector<std::pair<V2, std::string>> ents;
    for (auto &t : tiles)
        for (auto &p : t->points)
            if (p.kind.rfind("entrance:", 0) == 0) {
                V2 v = c.fr.to_local(p.p.lat, p.p.lon);
                if (v.x > c.x0 - FEATURE_MARGIN && v.x < c.x1 + FEATURE_MARGIN && v.y > c.y0 - FEATURE_MARGIN && v.y < c.y1 + FEATURE_MARGIN) ents.push_back({v, p.kind.substr(9)});
            }
    // mapped shops and amenities: "poi:<key>=<value>|<name>|<brand>"
    std::vector<ShopPoi> pois;
    for (auto &t : tiles)
        for (auto &p : t->points)
            if (p.kind.rfind("poi:", 0) == 0) {
                V2 v = c.fr.to_local(p.p.lat, p.p.lon);
                if (v.x < c.x0 - FEATURE_MARGIN || v.x > c.x1 + FEATURE_MARGIN || v.y < c.y0 - FEATURE_MARGIN || v.y > c.y1 + FEATURE_MARGIN) continue;
                std::string k = p.kind.substr(4);
                size_t eq = k.find('='), b1 = k.find('|'), b2 = k.find('|', b1 == std::string::npos ? 0 : b1 + 1);
                if (eq == std::string::npos || b1 == std::string::npos) continue;
                ShopPoi sp;
                sp.p = v, sp.key = k.substr(0, eq), sp.value = k.substr(eq + 1, b1 - eq - 1);
                sp.name = b2 == std::string::npos ? k.substr(b1 + 1) : k.substr(b1 + 1, b2 - b1 - 1);
                sp.brand = b2 == std::string::npos ? "" : k.substr(b2 + 1);
                pois.push_back(sp);
            }
    // which outlines are represented by their parts
    std::vector<const BRef *> parts;
    for (auto &b : all)
        if (b.b->part) parts.push_back(&b);
    for (auto &b : all) {
        if (b.b->part || !c.own(b.ctr)) continue;
        double covered = 0, part_hmin = 1e9;
        for (auto *p : parts)
            if (point_in_poly(p->ctr, b.rings)) {
                covered += p->area;
                double hv = 0;
                if (const std::string *ht = tag(p->b->tags, "height"); ht && parse_num(*ht, hv)) part_hmin = std::min(part_hmin, hv);
                else if (const std::string *lv = tag(p->b->tags, "building:levels"); lv && parse_num(*lv, hv)) part_hmin = std::min(part_hmin, hv * 3.0);
            }
        // OSM convention (Overture keeps it as has_parts): a building drawn by its parts is not drawn itself.
        // When the parts leave much of the outline uncovered (only a tower or a wing mapped as a part), the
        // outline is still drawn, no taller than the lowest part, so the building has no holes.
        bool has_parts = tagv(b.b->tags, "earthsim:has_parts") == "yes";
        if (covered >= 0.75 * b.area) continue;
        Tags capped;
        bool cap = (has_parts || covered >= 0.4 * b.area) && covered > 0.02 * b.area;
        if (cap && part_hmin < 1e8) {
            capped = b.b->tags;
            set_tag(capped, "height", std::to_string(std::max(3.0, part_hmin)));
            for (auto &kv : capped)
                if (kv.first == "building:levels") kv.second = std::to_string(std::max(1, (int)(part_hmin / 3)));
        }
        BuildingGeom g;
        g.rings = b.rings;
        double la, lo;
        c.fr.to_latlon(b.ctr, la, lo);
        for (auto &sp : pois)
            if (point_in_poly(sp.p, b.rings) || ring_dist(sp.p, b.rings[0]) < 4.0) g.pois.push_back(sp);
        // what the building is from what is inside it, when its own tags only say "a building": shops make it
        // retail (a small one) or keep it housing with shop fronts, offices an office building, a school...
        Tags tg = capped.empty() ? b.b->tags : capped;
        {
            std::string bc = tagv(tg, "building", "yes");
            if ((bc == "yes" || bc.empty()) && !g.pois.empty() && !tag(tg, "shop") && !tag(tg, "amenity") && !tag(tg, "office")) {
                int shops = 0, offices = 0, food = 0;
                std::string first_amenity;
                for (auto &sp : g.pois) {
                    if (sp.key == "shop") shops++;
                    else if (sp.key == "office") offices++;
                    else if (sp.key == "amenity") {
                        if (sp.value == "cafe" || sp.value == "restaurant" || sp.value == "fast_food" || sp.value == "bar" || sp.value == "pub" || sp.value == "bank" || sp.value == "pharmacy") food++;
                        else if (first_amenity.empty()) first_amenity = sp.value;
                    }
                }
                double lv = 0;
                bool tall = tag(tg, "building:levels") && parse_num(*tag(tg, "building:levels"), lv) && lv > 5;
                if (!first_amenity.empty() && (first_amenity == "school" || first_amenity == "kindergarten" || first_amenity == "hospital" || first_amenity == "clinic" || first_amenity == "university" || first_amenity == "college" || first_amenity == "place_of_worship" || first_amenity == "cinema" || first_amenity == "theatre"))
                    set_tag(tg, "amenity", first_amenity);
                else if (offices >= 2 && offices > shops + food) set_tag(tg, "building", "office");
                else if (shops + food >= 3 && b.area > 600 && !tall) set_tag(tg, "building", "retail");
                else if (shops + food >= 1 && b.area < 400 && !tall) set_tag(tg, "building", "retail");
            }
        }
        g.info = classify(b.b->id, tg, false, b.rings[0], la, lo);
        double gmin = 1e300, gmax = -1e300;
        for (auto &p : b.rings[0]) {
            double z = c.ground(p);
            gmin = std::min(gmin, z), gmax = std::max(gmax, z);
        }
        g.base = gmin;
        g.ground_max = gmax;
        g.lat = la, g.lon = lo;
        g.name = tagv(b.b->tags, "name");
        g.amenity = tagv(tg, "amenity");
        // entrances from the data: on the nearest wall within 3 m
        const Ring &o = b.rings[0];
        for (auto &[ep, kind] : ents) {
            double best = 3;
            int bi = -1;
            for (size_t i = 0; i < o.size(); i++) {
                double d = seg_dist(ep, o[i], o[(i + 1) % o.size()]);
                if (d < best) best = d, bi = (int)i;
            }
            if (bi < 0) continue;
            V2 a = o[bi], d = o[(bi + 1) % o.size()] - a;
            double l = d.len();
            if (l < 1.5) continue;
            V2 dir = d * (1 / l);
            double s = std::clamp((ep - a).dot(dir), 0.8, l - 0.8);
            g.doors.push_back({a + dir * s, V2(dir.y, -dir.x), kind == "yes" ? (g.info.use == Use::Housing ? "staircase" : "main") : kind});
        }
        // no data: apartment blocks get staircase doors on the side away from the street, shops/offices a main door to it
        if (g.doors.empty() && o.size() >= 4) {
            double hgt = g.info.height;
            int best = -1;
            double bl = 0;
            for (size_t i = 0; i < o.size(); i++) {
                double l = (o[(i + 1) % o.size()] - o[i]).len();
                if (l > bl) bl = l, best = (int)i;
            }
            // the long walls: the longest and the most anti-parallel to it
            V2 da = (o[(best + 1) % o.size()] - o[best]).norm();
            int opp = -1;
            double ol = 0;
            for (size_t i = 0; i < o.size(); i++) {
                V2 d = o[(i + 1) % o.size()] - o[i];
                if (d.norm().dot(da) < -0.9 && d.len() > ol) ol = d.len(), opp = (int)i;
            }
            auto mid = [&](int i) { return (o[i] + o[(i + 1) % o.size()]) * 0.5; };
            auto road_d = [&](int i) { return c.roads.clearance(mid(i)); };
            bool housing = g.info.use == Use::Housing && hgt >= 8 && bl >= 16;
            int wall = best;
            if (opp >= 0) {
                double d0 = road_d(best), d1 = road_d(opp);
                if (housing) wall = d0 > d1 ? best : opp;  // yard side
                else wall = d0 < d1 ? best : opp;          // street side
            }
            V2 a = o[wall], d = o[(wall + 1) % o.size()] - a;
            double l = d.len();
            if (l >= 4) {
                V2 dir = d * (1 / l), out(dir.y, -dir.x);
                if (housing) {
                    int sec = std::max(1, (int)std::lround(l / std::max(10.0f, g.info.d.section_len)));
                    for (int k = 0; k < sec; k++) g.doors.push_back({a + dir * (l * (k + 0.5) / sec), out, "staircase"});
                } else if (g.info.use != Use::Garage && g.info.use != Use::Structure && g.info.use != Use::Monument && b.area > 30) {
                    g.doors.push_back({a + dir * (l / 2), out, g.info.use == Use::House ? "yes" : "main"});
                }
            }
        }
        // house number plate (street name above the number) at the street end of the wall nearest a road
        if (const std::string *hn = tag(b.b->tags, "addr:housenumber"); hn && !hn->empty() && hn->size() < 12 && b.area > 25 && o.size() >= 3) {
            int wi = -1;
            double wd = 1e9;
            for (size_t i = 0; i < o.size(); i++) {
                V2 a = o[i], e = o[(i + 1) % o.size()];
                if ((e - a).len() < 3) continue;
                double d = c.roads.clearance((a + e) * 0.5);
                if (d < wd) wd = d, wi = (int)i;
            }
            if (wi >= 0) {
                V2 a = o[wi], e = o[(wi + 1) % o.size()], dd = e - a;
                double l = dd.len();
                V2 dir = dd * (1 / l), out(dir.y, -dir.x);
                // nearer the end that is closer to a road (corner plates read from the junction)
                double sp = c.roads.clearance(a + dir * 1.2) <= c.roads.clearance(e - dir * 1.2) ? 1.0 : l - 1.0;
                V2 m = a + dir * sp + out * 0.06;
                double z = std::min(g.base + 2.7, g.base + g.info.height - 0.6);
                const std::string *st = tag(b.b->tags, "addr:street");
                double w = st ? 0.75 : 0.42, hh = st ? 0.42 : 0.32;
                MeshBuf &PM = c.out.ms[M_PAINT];
                V3 nrm(out.x, out.y, 0);
                auto Q = [&](double u, double v, double off) { V2 q = m + dir * u + out * off; return V3(q.x, q.y, z + v); };
                PM.quad(Q(-w / 2 - 0.02, -hh / 2 - 0.02, 0), Q(w / 2 + 0.02, -hh / 2 - 0.02, 0), Q(w / 2 + 0.02, hh / 2 + 0.02, 0), Q(-w / 2 - 0.02, hh / 2 + 0.02, 0), nrm, Col{0.92f, 0.92f, 0.92f, 1});
                PM.quad(Q(-w / 2, -hh / 2, 0.01), Q(w / 2, -hh / 2, 0.01), Q(w / 2, hh / 2, 0.01), Q(-w / 2, hh / 2, 0.01), nrm, Col{0.08f, 0.22f, 0.5f, 1});
                std::string text = *hn;
                if (st) {
                    std::string sn = *st;
                    // shorten the usual Russian street words
                    for (auto [from, to] : std::initializer_list<std::pair<const char *, const char *>>{{"улица ", "ул. "}, {" улица", ""}, {"проспект ", "пр. "}, {" проспект", " пр."}, {"переулок ", "пер. "}, {" переулок", " пер."}, {"бульвар ", "б-р "}, {"площадь ", "пл. "}, {"вуліца ", "вул. "}, {" вуліца", ""}, {"праспект ", "пр. "}, {"завулак ", "зав. "}, {"плошча ", "пл. "}}) {
                        size_t k = sn.find(from);
                        if (k != std::string::npos) sn.replace(k, strlen(from), to);
                    }
                    text = sn + "\n" + *hn;
                }
                V2 lq = m + out * 0.03;
                c.out.labels.push_back({V3(lq.x, lq.y, z), std::atan2(out.y, out.x), w - 0.08, text, Col{1, 1, 1, 1}});
            }
        }
        build_building(g, c.out.ms, c.out.coll, &c.out.labels);
        c.out.buildings++;
    }
    for (auto *p : parts) {
        if (!c.own(p->ctr)) continue;
        BuildingGeom g;
        g.rings = p->rings;
        double la, lo;
        c.fr.to_latlon(p->ctr, la, lo);
        g.info = classify(p->b->id, p->b->tags, true, p->rings[0], la, lo);
        // parts stand on the lowest ground under the building they belong to
        double gmin = 1e300;
        const BRef *host = nullptr;
        for (auto &b : all)
            if (!b.b->part && point_in_poly(p->ctr, b.rings) && (!host || b.area < host->area)) host = &b;
        for (auto &q : (host ? host->rings[0] : p->rings[0])) gmin = std::min(gmin, c.ground(q));
        g.base = gmin;
        build_building(g, c.out.ms, c.out.coll);
        c.out.buildings++;
    }
}

const Col CAR_COLS[] = {{0.85f, 0.85f, 0.86f, 1}, {0.08f, 0.08f, 0.09f, 1}, {0.45f, 0.47f, 0.5f, 1}, {0.7f, 0.72f, 0.75f, 1}, {0.55f, 0.08f, 0.07f, 1},
                        {0.1f, 0.2f, 0.45f, 1},    {0.95f, 0.95f, 0.95f, 1}, {0.2f, 0.32f, 0.22f, 1}, {0.6f, 0.5f, 0.35f, 1}, {0.3f, 0.3f, 0.32f, 1}};

void objects(Ctx &c, const std::vector<std::shared_ptr<const RawTile>> &tiles) {
    auto blocked = [&](V2 p, double r) {
        uint8_t o = c.occ_at(p);
        if (o & (O_ROAD | O_WALK | O_WATER | O_PATH | O_BUILDING | O_PARKING)) return true;
        if (c.roads.clearance(p) < r) return true;
        return false;
    };
    // mapped trees and lamps
    for (auto &t : tiles)
        for (auto &pt : t->points) {
            if (pt.kind != "tree" && pt.kind != "lamp") continue;
            V2 v = c.fr.to_local(pt.p.lat, pt.p.lon);
            if (!c.own(v)) continue;
            uint64_t h = hash_ll(pt.p.lat, pt.p.lon, 77);
            if (pt.kind == "tree") {
                if (c.occ_at(v) & (O_ROAD | O_BUILDING | O_WATER)) continue;
                double s = 0.75 + hash01(h, 1) * 0.6;
                c.out.inst.push_back({hash01(h, 2) < 0.15 ? I_CONIFER : I_TREE, {v.x, v.y, c.ground(v)}, hash01(h, 3) * 2 * PI, s, {1, 1, 1, 1}});
            } else {
                if (c.occ_at(v) & (O_ROAD | O_BUILDING)) continue;
                int rid = -1;
                c.roads.clearance(v, &rid);
                double yaw = 0;
                if (rid >= 0) {
                    // face the nearest carriageway
                    double best = 1e9;
                    V2 near;
                    for (auto &sg : c.roads.segs)
                        if (sg.id == rid) {
                            V2 d = sg.b - sg.a;
                            double l2 = std::max(1e-9, d.dot(d));
                            double tt = std::clamp((v - sg.a).dot(d) / l2, 0.0, 1.0);
                            V2 q = sg.a + d * tt;
                            if ((q - v).len() < best) best = (q - v).len(), near = q;
                        }
                    yaw = std::atan2(near.y - v.y, near.x - v.x);
                }
                c.out.inst.push_back({I_LAMP, {v.x, v.y, c.ground(v)}, yaw, 1, {1, 1, 1, 1}});
            }
        }
    // grass tufts on lawns, pitches and in forests, up to ~1 per m2
    c.out.grass.assign(16, {});
    for (int j = 0; j < SPLAT_N; j++)
        for (int i = 0; i < SPLAT_N; i++) {
            int t = j * SPLAT_N + i;
            uint8_t L = c.layer[t];
            double dens = L == L_GRASS ? 1.1 : L == L_PITCH ? 0.6 : L == L_FOREST ? 0.35 : L == L_FARM ? 0.2 : 0;
            if (dens <= 0 || (c.occ[t] & (O_ROAD | O_WALK | O_WATER | O_PATH | O_BUILDING | O_PARKING))) continue;
            uint64_t h = mix64(((uint64_t)(uint32_t)(c.cx * SPLAT_N + i) << 32) ^ (uint32_t)(c.cy * SPLAT_N + j) ^ 0x6a55);
            int n = (int)std::floor(dens + hash01(h, 9));
            for (int k = 0; k < n; k++) {
                V2 p(c.x0 + i + hash01(h, 1 + k * 4), c.y0 + j + hash01(h, 2 + k * 4));
                if (c.roads.clearance(p) < 0.6) continue;
                double z = c.ground(p);
                double yaw = hash01(h, 3 + k * 4) * 2 * PI, sc = 0.6 + 0.8 * hash01(h, 4 + k * 4);
                double cs = std::cos(yaw) * sc, sn = std::sin(yaw) * sc;
                // Godot: x, y up, z = -north; rotation about y
                float gx = (float)p.x, gy = (float)z, gz = (float)-p.y;
                float hue = (float)hash01(h, 30 + k);
                float dry = L == L_FOREST ? 0.2f : 0.0f;
                std::vector<float> &b = c.out.grass[(j / 64) * 4 + i / 64];
                b.insert(b.end(), {(float)cs, 0, (float)sn, gx, 0, (float)sc, 0, gy, (float)-sn, 0, (float)cs, gz, 0.85f + 0.3f * hue + dry, 0.9f + 0.2f * hue, 0.8f + 0.2f * hue - dry, 1});
            }
        }

    // scattered trees and bushes on a jittered global grid
    const double CELL = 4.5;
    int i0 = (int)std::floor(c.x0 / CELL), i1 = (int)std::floor(c.x1 / CELL), j0 = (int)std::floor(c.y0 / CELL), j1 = (int)std::floor(c.y1 / CELL);
    int urban_grass = 0, mapped_trees = 0;
    for (auto &t : tiles) {
        urban_grass += t->buildings.size() > 60;
        for (auto &pt : t->points) mapped_trees += pt.kind == "tree";
    }
    for (int j = j0; j <= j1; j++)
        for (int i = i0; i <= i1; i++) {
            uint64_t h = mix64(((uint64_t)(uint32_t)i << 32) ^ (uint32_t)j ^ 0x7ee5);
            V2 p((i + 0.15 + 0.7 * hash01(h, 1)) * CELL, (j + 0.15 + 0.7 * hash01(h, 2)) * CELL);
            if (!c.own(p)) continue;
            int tx = c.texel(p);
            if (tx < 0) continue;
            uint8_t L = c.layer[tx];
            double prob = 0, conifer = 0.15, bush = 0;
            if (L == L_FOREST) prob = 0.75, conifer = 0.45;
            else if (L == L_GRASS) prob = urban_grass ? (mapped_trees > 40 ? 0.008 : 0.025) : 0.012, bush = 0.45;  // where trees are mapped, few made up
            else if (L == L_FARM) prob = 0.002;
            if (prob <= 0 || hash01(h, 3) >= prob) continue;
            if (blocked(p, 2.0)) continue;
            bool near_b = false;
            for (auto &br : c.building_rings)
                if (ring_dist(p, br) < 3.5 || point_in_ring(p, br)) {
                    near_b = true;
                    break;
                }
            if (near_b) continue;
            int kind = hash01(h, 4) < bush ? I_BUSH : hash01(h, 5) < conifer ? I_CONIFER : I_TREE;
            double s = kind == I_BUSH ? 0.6 + hash01(h, 6) * 0.6 : 0.7 + hash01(h, 6) * 0.7;
            c.out.inst.push_back({kind, {p.x, p.y, c.ground(p)}, hash01(h, 7) * 2 * PI, s, {1, 1, 1, 1}});
        }
    // parked cars in car parks, in rows of perpendicular bays
    for (auto &t : tiles)
        for (auto &a : t->areas) {
            if (a.kind != AreaKind::Parking) continue;
            V2 lo, hi;
            auto rings = to_local(c.fr, a.rings, lo, hi);
            if (rings.empty() || !c.near(lo, hi, 2)) continue;
            double area = std::abs(signed_area(rings[0]));
            if (area < 120) continue;
            V2 ctr, ax;
            double L, Wd;
            footprint_frame(rings[0], ctr, ax, L, Wd);
            V2 pe = ax.perp();
            uint64_t seed = hash_str(std::to_string((long long)std::llround(ctr.x)) + "," + std::to_string((long long)std::llround(ctr.y)));
            // row pattern across the width: bay (5 m), bay (5 m), aisle (6.5 m)
            double t0 = -Wd / 2 + 2.6;
            int row = 0;
            for (double tt = t0; tt < Wd / 2 - 2.4; row++) {
                bool facing = row % 2 == 0;
                for (double ss = -L / 2 + 1.4; ss < L / 2 - 1.3; ss += 2.6) {
                    V2 p = ctr + ax * ss + pe * tt;
                    if (!c.own(p)) continue;
                    uint64_t h = mix64(seed ^ ((uint64_t)row << 20) ^ (uint64_t)(int64_t)std::llround(ss * 10));
                    if (hash01(h, 1) > 0.62) continue;
                    bool ok = point_in_poly(p, rings) && point_in_poly(p + pe * 2.4, rings) && point_in_poly(p - pe * 2.4, rings);
                    if (!ok || c.roads.clearance(p) < 1.2 || (c.occ_at(p) & (O_BUILDING | O_WATER | O_WALK))) continue;
                    bool inb = false;
                    for (auto &br : c.building_rings)
                        if (ring_dist(p, br) < 2.5 || point_in_ring(p, br)) {
                            inb = true;
                            break;
                        }
                    if (inb) continue;
                    double yaw = std::atan2(pe.y, pe.x) + (facing ? 0 : PI) + (hash01(h, 2) - 0.5) * 0.06;
                    c.out.inst.push_back({I_CAR, {p.x, p.y, c.ground(p)}, yaw, 1, CAR_COLS[(size_t)(hash01(h, 3) * 10) % 10]});
                }
                tt += row % 2 == 0 ? 5.0 : 5.0 + 6.5;
            }
        }
}

void splat_images(Ctx &c) {
    c.out.splat0.assign(SPLAT_N * SPLAT_N * 4, 0);
    c.out.splat1.assign(SPLAT_N * SPLAT_N * 4, 0);
    for (int j = 0; j < SPLAT_N; j++)
        for (int i = 0; i < SPLAT_N; i++) {
            int w[8] = {0};
            int tot = 0;
            for (int dj = -1; dj <= 1; dj++)
                for (int di = -1; di <= 1; di++) {
                    int ii = std::clamp(i + di, 0, SPLAT_N - 1), jj = std::clamp(j + dj, 0, SPLAT_N - 1);
                    int k = (di == 0 && dj == 0) ? 4 : 1;
                    w[c.layer[jj * SPLAT_N + ii]] += k;
                    tot += k;
                }
            size_t q = ((size_t)j * SPLAT_N + i) * 4;
            for (int l = 0; l < 4; l++) c.out.splat0[q + l] = (uint8_t)(w[l] * 255 / tot);
            for (int l = 0; l < 4; l++) c.out.splat1[q + l] = (uint8_t)(w[4 + l] * 255 / tot);
        }
}

}  // namespace

ChunkOut build_chunk(int cx, int cy, const std::vector<std::shared_ptr<const RawTile>> &tiles, const Dem &dem, const LocalFrame &fr) {
    ChunkOut out;
    out.cx = cx, out.cy = cy;
    Ctx c(cx, cy, dem, fr, out);
    out.x0 = c.x0, out.y0 = c.y0;
    // extended grid: one sample of border each side for normals
    HeightGrid &g = out.grid;
    g.cell = CHUNK / GRID_N;
    g.n = GRID_N + 2;
    g.x0 = c.x0 - g.cell, g.y0 = c.y0 - g.cell;
    g.h.resize((size_t)(g.n + 1) * (g.n + 1));
    for (int j = 0; j <= g.n; j++)
        for (int i = 0; i <= g.n; i++) g.at(i, j) = (float)c.G({g.x0 + i * g.cell, g.y0 + j * g.cell});
    // buildings near the chunk (outlines for clearance tests)
    std::vector<BRef> all;
    for (auto &t : tiles)
        for (auto &b : t->buildings) {
            V2 lo, hi;
            auto rings = to_local(fr, b.rings, lo, hi);
            if (rings.empty() || !c.near(lo, hi, FEATURE_MARGIN)) continue;
            double area = std::abs(signed_area(rings[0]));
            if (area < 4) continue;
            V2 ctr = centroid(rings[0]);
            all.push_back({&b, rings, ctr, area});
            if (!b.part) c.building_rings.push_back(rings[0]);
        }
    areas(c, tiles);
    water(c, tiles);
    build_roads(c, tiles);
    paths(c, tiles);
    build_infra(c, tiles);
    // footprints: aprons of paving round urban buildings, occupancy
    for (auto &b : all) {
        if (b.b->part) continue;
        Ring o = offset_ring(b.rings[0], 1.2);
        c.fill({o}, [&](int t) {
            if (!(c.occ[t] & (O_ROAD | O_WATER))) c.layer[t] = L_PAVERS;
            c.occ[t] |= O_BUILDING;
        });
    }
    buildings(c, tiles, all);
    objects(c, tiles);
    splat_images(c);
    out.layer = c.layer;
    // collision of placed objects: poles, trunks, parked cars (vertical prisms)
    for (auto &in : out.inst) {
        auto prism = [&](V2 ctr, double yaw, double l, double w, double z0, double z1, int sides) {
            std::vector<V2> pts;
            if (sides == 4) {
                double cs = std::cos(yaw), sn = std::sin(yaw);
                for (auto [a, b] : {std::pair{-l / 2, -w / 2}, {l / 2, -w / 2}, {l / 2, w / 2}, {-l / 2, w / 2}}) pts.push_back({ctr.x + a * cs - b * sn, ctr.y + a * sn + b * cs});
            } else
                for (int k = 0; k < sides; k++) pts.push_back({ctr.x + std::cos(2 * PI * k / sides) * l, ctr.y + std::sin(2 * PI * k / sides) * l});
            for (size_t k = 0; k < pts.size(); k++) {
                V2 a = pts[k], b = pts[(k + 1) % pts.size()];
                c.tri_coll({a.x, a.y, z0}, {b.x, b.y, z0}, {b.x, b.y, z1});
                c.tri_coll({a.x, a.y, z0}, {b.x, b.y, z1}, {a.x, a.y, z1});
            }
            for (size_t k = 1; k + 1 < pts.size(); k++) c.tri_coll({pts[0].x, pts[0].y, z1}, {pts[k].x, pts[k].y, z1}, {pts[k + 1].x, pts[k + 1].y, z1});
        };
        V2 p(in.p.x, in.p.y);
        if (in.kind == I_LAMP) prism(p, 0, 0.12, 0, in.p.z - 0.5, in.p.z + 8, 6);
        else if (in.kind == I_TREE || in.kind == I_CONIFER) prism(p, 0, 0.22 * in.scale, 0, in.p.z - 0.5, in.p.z + 4, 6);
        else if (in.kind == I_CAR) prism(p, in.yaw, 4.4, 1.78, in.p.z + 0.15, in.p.z + 1.35, 4);
        else if (in.kind == I_SIGNAL || in.kind == I_SIGN) prism(p, 0, 0.07, 0, in.p.z - 0.2, in.p.z + 2.7, 5);
        else if (in.kind == I_BIN) prism(p, 0, 0.2, 0, in.p.z, in.p.z + 0.85, 6);
        else if (in.kind == I_SHELTER) prism(p, in.yaw, 1.5, 3.2, in.p.z, in.p.z + 2.5, 4);
    }

    // terrain meshes from the extended grid, then crop it to the chunk
    auto H = [&](int i, int j) { return (double)g.at(i + 1, j + 1); };  // chunk sample coordinates
    auto nrm = [&](int i, int j) {
        double dx = g.at(i + 2, j + 1) - g.at(i, j + 1), dy = g.at(i + 1, j + 2) - g.at(i + 1, j);
        return V3(-dx / (2 * g.cell), -dy / (2 * g.cell), 1).norm();
    };
    for (int lod = 0; lod < 2; lod++) {
        int step = lod == 0 ? 2 : 8;
        MeshBuf &m = lod == 0 ? out.terrain_hi : out.terrain_lo;
        int n = GRID_N / step;
        for (int j = 0; j <= n; j++)
            for (int i = 0; i <= n; i++) {
                int gi = i * step, gj = j * step;
                V3 p(c.x0 + gi * g.cell, c.y0 + gj * g.cell, H(gi, gj));
                m.v(p, nrm(gi, gj), double(gi) / GRID_N, double(gj) / GRID_N, {1, 1, 1, 1});
            }
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++) {
                int a = j * (n + 1) + i, b = a + 1, d = a + (n + 1) + 1, e = a + (n + 1);
                m.tri(a, b, d);
                m.tri(a, d, e);
            }
        if (lod == 1) {
            // skirts hide cracks against finer neighbours
            auto skirt = [&](int gi, int gj, int gi2, int gj2) {
                V3 p(c.x0 + gi * g.cell, c.y0 + gj * g.cell, H(gi, gj)), q(c.x0 + gi2 * g.cell, c.y0 + gj2 * g.cell, H(gi2, gj2));
                V3 nn = nrm(gi, gj);
                int a = m.v(p, nn, double(gi) / GRID_N, double(gj) / GRID_N, {1, 1, 1, 1}), b = m.v(q, nn, double(gi2) / GRID_N, double(gj2) / GRID_N, {1, 1, 1, 1});
                int d = m.v(q - V3(0, 0, 3), nn, double(gi2) / GRID_N, double(gj2) / GRID_N, {1, 1, 1, 1}), e = m.v(p - V3(0, 0, 3), nn, double(gi) / GRID_N, double(gj) / GRID_N, {1, 1, 1, 1});
                m.tri(a, b, d);
                m.tri(a, d, e);
                m.tri(a, d, b);
                m.tri(a, e, d);
            };
            for (int k = 0; k < n; k++) {
                skirt(k * step, 0, (k + 1) * step, 0);
                skirt((k + 1) * step, GRID_N, k * step, GRID_N);
                skirt(0, (k + 1) * step, 0, k * step);
                skirt(GRID_N, k * step, GRID_N, (k + 1) * step);
            }
        }
    }
    HeightGrid crop;
    crop.cell = g.cell, crop.n = GRID_N, crop.x0 = c.x0, crop.y0 = c.y0;
    crop.h.resize((size_t)(GRID_N + 1) * (GRID_N + 1));
    for (int j = 0; j <= GRID_N; j++)
        for (int i = 0; i <= GRID_N; i++) crop.at(i, j) = g.at(i + 1, j + 1);
    out.grid = std::move(crop);
    return out;
}

}  // namespace ec
