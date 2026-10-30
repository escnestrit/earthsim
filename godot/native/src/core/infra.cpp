// Railways (ballast, sleepers, rails; tram tracks laid in the street with overhead wires), level crossings
// with barriers, power lines on towers and poles, fences, walls and hedges, metro entrance signs: all from the
// Overture transportation (rail) and base infrastructure layers, so the same everywhere in the world.
#include <cmath>
#include "chunk_ctx.hpp"
#include "infra.hpp"
#include "signage.hpp"
#include "buildings.hpp"
#include "util.hpp"

namespace ec {

namespace {

std::vector<V2> local_line(Ctx &c, const std::vector<LL> &pts) {
    std::vector<V2> p;
    for (auto &q : pts) {
        V2 v = c.fr.to_local(q.lat, q.lon);
        if (!p.empty() && (v - p.back()).len() < 0.1) continue;
        p.push_back(v);
    }
    return p;
}

// a box between two points (width w, from z0 to z1 above the line points' heights)
void beam(MeshBuf &m, V3 a, V3 b, double w, double h, Col col) {
    V2 d = V2(b.x - a.x, b.y - a.y);
    double l = d.len();
    if (l < 1e-6) return;
    d = d * (1 / l);
    V2 n = d.perp() * (w / 2);
    V3 p0(a.x + n.x, a.y + n.y, a.z), p1(b.x + n.x, b.y + n.y, b.z), p2(b.x - n.x, b.y - n.y, b.z), p3(a.x - n.x, a.y - n.y, a.z);
    V3 up(0, 0, h);
    m.quadn(p0, p1, p1 + up, p0 + up, col);
    m.quadn(p2, p3, p3 + up, p2 + up, col);
    m.quadn(p0 + up, p1 + up, p2 + up, p3 + up, col);
}

// a thin wire between two points, sagging; drawn as a narrow vertical ribbon (visible from the side and below)
void wire(MeshBuf &m, V3 a, V3 b, double sag, double th, Col col) {
    const int N = 8;
    V3 prev = a;
    for (int k = 1; k <= N; k++) {
        double t = (double)k / N;
        V3 p = a + (b - a) * t;
        p.z -= sag * 4 * t * (1 - t);
        V3 q0 = prev, q1 = p;
        m.quadn(q0, q1, q1 + V3(0, 0, th), q0 + V3(0, 0, th), col);
        m.quadn(q1, q0, q0 + V3(0, 0, th), q1 + V3(0, 0, th), col);
        prev = p;
    }
}

bool seg_cross(V2 a, V2 b, V2 c, V2 d, V2 &x) {
    V2 r = b - a, s = d - c;
    double den = r.cross(s);
    if (std::abs(den) < 1e-9) return false;
    double t = (c - a).cross(s) / den, u = (c - a).cross(r) / den;
    if (t < 0 || t > 1 || u < 0 || u > 1) return false;
    x = a + r * t;
    return true;
}

}  // namespace

void build_infra(Ctx &c, const std::vector<std::shared_ptr<const RawTile>> &tiles) {
    MeshBuf &MT = c.out.ms[M_METAL], &CO = c.out.ms[M_CONCRETE], &DK = c.out.ms[M_DARK], &PM = c.out.ms[M_PAINT], &GV = c.out.ms[M_GRAVEL];
    const Col rail_c{0.42f, 0.4f, 0.38f, 1}, sleeper_c{0.45f, 0.42f, 0.38f, 1}, wire_c{0.12f, 0.12f, 0.13f, 1};
    std::vector<std::pair<V2, V2>> rail_segs;  // for level crossings
    for (auto &t : tiles)
        for (auto &ln : t->lines) {
            if (ln.kind.rfind("rail:", 0) != 0) continue;
            std::string cls = ln.kind.substr(5);
            bool tram = cls == "tram" || cls == "light_rail";
            auto p0 = local_line(c, ln.pts);
            if (p0.size() < 2) continue;
            auto p = densify(p0, 2.0);
            std::vector<double> s(p.size(), 0);
            for (size_t i = 1; i < p.size(); i++) s[i] = s[i - 1] + (p[i] - p[i - 1]).len();
            const double G = cls == "narrow_gauge" ? 0.75 : 1.52;  // 1520 mm (ex-USSR) is close enough to 1435 for looks
            for (size_t i = 0; i + 1 < p.size(); i++) {
                V2 a = p[i], b = p[i + 1];
                V2 mid = (a + b) * 0.5;
                if (!c.own(mid)) continue;
                V2 d = (b - a).norm(), n = d.perp();
                double on_road = c.roads.clearance(mid);
                bool in_street = on_road < 0.5;
                double za, zb;
                if (ln.bridge) {
                    double zA = c.Gs(p.front()), zB = c.Gs(p.back()), L = s.back();
                    za = zA + (zB - zA) * s[i] / L + 0.4, zb = zA + (zB - zA) * s[i + 1] / L + 0.4;
                } else if (in_street) {
                    za = c.ground(a) + 0.07, zb = c.ground(b) + 0.07;  // flush with the carriageway
                } else {
                    za = std::max(c.ground(a), c.Gs(a)) + 0.35, zb = std::max(c.ground(b), c.Gs(b)) + 0.35;
                }
                // level crossings only where a real railway (not a tram, not on a bridge) meets the road
                if (!tram && !ln.bridge) rail_segs.push_back({a, b});
                if (!in_street && !ln.bridge) {
                    // ballast bed: top 3.2 m, slopes down 0.6 m
                    double top = tram ? 1.4 : 1.7, foot = top + 0.9;
                    V3 l0(a.x + n.x * top, a.y + n.y * top, za), l1(b.x + n.x * top, b.y + n.y * top, zb), r0(a.x - n.x * top, a.y - n.y * top, za), r1(b.x - n.x * top, b.y - n.y * top, zb);
                    V3 fl0(a.x + n.x * foot, a.y + n.y * foot, c.ground(a + n * foot) - 0.1), fl1(b.x + n.x * foot, b.y + n.y * foot, c.ground(b + n * foot) - 0.1);
                    V3 fr0(a.x - n.x * foot, a.y - n.y * foot, c.ground(a - n * foot) - 0.1), fr1(b.x - n.x * foot, b.y - n.y * foot, c.ground(b - n * foot) - 0.1);
                    Col bc{0.62f, 0.58f, 0.54f, 1};
                    GV.quadn(r0, r1, l1, l0, bc, -top, s[i], top, s[i + 1]);
                    GV.quadn(l0, l1, fl1, fl0, bc);
                    GV.quadn(fr0, fr1, r1, r0, bc);
                    c.tri_coll(r0, r1, l1);
                    c.tri_coll(r0, l1, l0);
                    c.tri_coll(l0, l1, fl1);
                    c.tri_coll(l0, fl1, fl0);
                    c.tri_coll(fr0, fr1, r1);
                    c.tri_coll(fr0, r1, r0);
                    // sleepers every 0.6 m
                    for (double ss = std::ceil(s[i] / 0.6) * 0.6; ss < s[i + 1]; ss += 0.6) {
                        double tt = (ss - s[i]) / std::max(1e-6, s[i + 1] - s[i]);
                        V2 q = a + (b - a) * tt;
                        double z = za + (zb - za) * tt;
                        DK.obox(q, std::atan2(n.y, n.x), G + 1.0, 0.25, z, z + 0.08, sleeper_c);
                    }
                }
                if (ln.bridge) {
                    V3 A(a.x, a.y, za - 0.3), B(b.x, b.y, zb - 0.3);
                    beam(CO, A - V3(0, 0, 1.0), B - V3(0, 0, 1.0), G + 2.6, 1.0, Col{0.6f, 0.6f, 0.58f, 1});
                    beam(MT, A + V3(n.x * (G / 2 + 1.3), n.y * (G / 2 + 1.3), 0), B + V3(n.x * (G / 2 + 1.3), n.y * (G / 2 + 1.3), 0), 0.08, 1.1, Col{0.3f, 0.32f, 0.35f, 1});
                    beam(MT, A - V3(n.x * (G / 2 + 1.3), n.y * (G / 2 + 1.3), 0), B - V3(n.x * (G / 2 + 1.3), n.y * (G / 2 + 1.3), 0), 0.08, 1.1, Col{0.3f, 0.32f, 0.35f, 1});
                }
                // the two rails
                double rz = in_street ? 0.0 : 0.08;
                for (int sd : {-1, 1}) {
                    V2 o = n * (sd * G / 2);
                    V3 A(a.x + o.x, a.y + o.y, za + rz), B(b.x + o.x, b.y + o.y, zb + rz);
                    beam(MT, A, B, 0.075, in_street ? 0.006 : 0.15, rail_c);
                    if (in_street) beam(DK, A + V3(n.x * 0.06 * sd, n.y * 0.06 * sd, 0), B + V3(n.x * 0.06 * sd, n.y * 0.06 * sd, 0), 0.05, 0.004, Col{0.05f, 0.05f, 0.05f, 1});
                }
                // trams and electrified lines: the contact wire
                if (tram) {
                    V3 A(a.x, a.y, za + 5.9), B(b.x, b.y, zb + 5.9);
                    wire(DK, A, B, 0.02, 0.025, wire_c);
                }
            }
            // catenary poles along tram lines every 30 m when the data has none nearby
            if (tram) {
                for (double ss = 15; ss < s.back(); ss += 32) {
                    size_t i = 0;
                    while (i + 1 < s.size() && s[i + 1] < ss) i++;
                    V2 d = (p[std::min(i + 1, p.size() - 1)] - p[i]).norm();
                    V2 q = p[i] + d * (ss - s[i]);
                    V2 side = d.perp();
                    double out = c.roads.clearance(q) < 0.5 ? -c.roads.clearance(q) + 1.2 : 3.0;
                    V2 pp = q + side * out;
                    if (!c.own(pp)) continue;
                    bool blocked = false;
                    for (auto &br : c.building_rings)
                        if (point_in_ring(pp, br)) blocked = true;
                    if (blocked || c.roads.clearance(pp) < 0.2) continue;
                    double z = c.ground(pp);
                    MT.cyl(pp, 0.12, 0.09, z - 0.2, z + 7.0, 8, Col{0.35f, 0.4f, 0.38f, 1});
                    V3 top(pp.x, pp.y, z + 6.4), tip(q.x - side.x * 0.3, q.y - side.y * 0.3, z + 6.4);
                    beam(MT, top, tip, 0.06, 0.08, Col{0.3f, 0.32f, 0.33f, 1});
                    c.coll_prism(pp, 0, 0.26, 0.26, z - 0.2, z + 6.8);
                }
            }
        }
    // level crossings: rails over roads (not trams in the street, not bridges): barriers and warning signs
    {
        for (auto &[ra, rb] : rail_segs) {
            for (auto &sg : c.roads.segs) {
                V2 x;
                if (!seg_cross(ra, rb, sg.a, sg.b, x) || !c.own(x)) continue;
                if (sg.hw < 3.0) continue;  // not parking aisles and yard lanes
                V2 rd = (rb - ra).norm(), ud = (sg.b - sg.a).norm();
                if (std::abs(rd.cross(ud)) < 0.4) continue;  // tram running along the street
                // one crossing per spot
                bool seen = false;
                for (auto &in : c.out.inst)
                    if (in.kind == I_SIGN && std::hypot(in.p.x - x.x, in.p.y - x.y) < 12 && in.c.r == (float)SG_RAILWAY) seen = true;
                if (seen) continue;
                for (int sd : {-1, 1}) {
                    // the road side half a road width out, a few metres before the rails, for each direction
                    V2 back = ud * (sd * 6.0);
                    V2 right = (ud * sd).perp() * -1;
                    V2 bp = x - back + right * (sg.hw + 0.8);
                    double z = c.ground(bp);
                    c.out.inst.push_back({I_SIGN, {bp.x, bp.y, z}, std::atan2(-(ud * sd).y, -(ud * sd).x), 1, {(float)SG_RAILWAY, 0, 0, 1}});
                    // barrier: post and a red-and-white arm over the incoming half
                    V2 post = x - ud * (sd * 4.0) + right * (sg.hw + 0.4);
                    double pz = c.ground(post);
                    MT.obox(post, 0, 0.25, 0.25, pz, pz + 1.1, Col{0.85f, 0.85f, 0.85f, 1});
                    c.coll_prism(post, 0, 0.3, 0.3, pz, pz + 1.1);
                    V2 dir = right * -1;
                    double len = sg.hw + 0.2;
                    for (int k = 0; k < 6; k++) {
                        V2 q0 = post + dir * (len * k / 6), q1 = post + dir * (len * (k + 1) / 6);
                        beam(PM, V3(q0.x, q0.y, pz + 1.0), V3(q1.x, q1.y, pz + 1.0), 0.08, 0.08, k % 2 ? Col{0.95f, 0.95f, 0.95f, 1} : Col{0.85f, 0.1f, 0.08f, 1});
                    }
                }
            }
        }
    }
    // power lines: towers at the line's vertices, three sagging conductors; minor lines on poles
    for (auto &t : tiles)
        for (auto &ln : t->lines) {
            if (ln.kind.rfind("power:", 0) != 0) continue;
            bool major = ln.kind == "power:power_line";
            auto p = local_line(c, ln.pts);
            if (p.size() < 2) continue;
            double H = major ? 26 : 9.5;
            std::vector<double> z(p.size());
            for (size_t i = 0; i < p.size(); i++) z[i] = c.G(p[i]);
            for (size_t i = 0; i < p.size(); i++) {
                V2 dprev = i > 0 ? (p[i] - p[i - 1]).norm() : (p[1] - p[0]).norm();
                V2 dnext = i + 1 < p.size() ? (p[i + 1] - p[i]).norm() : dprev;
                V2 d = (dprev + dnext).norm(), n = d.perp();
                if (c.own(p[i]) && c.roads.clearance(p[i]) > 0.5) {
                    if (major) {
                        // a lattice tower, simplified: four tapering legs, cross-arms
                        for (int k = 0; k < 4; k++) {
                            double sx = (k & 1) ? 1 : -1, sy = (k & 2) ? 1 : -1;
                            V3 bot(p[i].x + (d.x * sx + n.x * sy) * 2.5, p[i].y + (d.y * sx + n.y * sy) * 2.5, z[i] - 0.3);
                            V3 top(p[i].x + (d.x * sx + n.x * sy) * 0.6, p[i].y + (d.y * sx + n.y * sy) * 0.6, z[i] + H);
                            V2 dd = V2(top.x - bot.x, top.y - bot.y);
                            (void)dd;
                            int segs = 6;
                            for (int q = 0; q < segs; q++) {
                                V3 u0 = bot + (top - bot) * ((double)q / segs), u1 = bot + (top - bot) * ((double)(q + 1) / segs);
                                MT.obox({(u0.x + u1.x) / 2, (u0.y + u1.y) / 2}, 0, 0.18, 0.18, u0.z, u1.z, Col{0.5f, 0.52f, 0.54f, 1});
                            }
                        }
                        for (double ah : {H * 0.72, H * 0.86}) {
                            V3 a0(p[i].x + n.x * 7, p[i].y + n.y * 7, z[i] + ah), a1(p[i].x - n.x * 7, p[i].y - n.y * 7, z[i] + ah);
                            beam(MT, a0, a1, 0.3, 0.4, Col{0.5f, 0.52f, 0.54f, 1});
                        }
                        c.coll_prism(p[i], std::atan2(d.y, d.x), 5.4, 5.4, z[i] - 0.3, z[i] + 6);
                    } else {
                        CO.cyl(p[i], 0.16, 0.11, z[i] - 0.3, z[i] + H, 8, Col{0.72f, 0.71f, 0.68f, 1});
                        V3 a0(p[i].x + n.x * 0.9, p[i].y + n.y * 0.9, z[i] + H - 0.5), a1(p[i].x - n.x * 0.9, p[i].y - n.y * 0.9, z[i] + H - 0.5);
                        beam(MT, a0, a1, 0.1, 0.1, Col{0.35f, 0.35f, 0.36f, 1});
                        c.coll_prism(p[i], 0, 0.32, 0.32, z[i] - 0.3, z[i] + 6);
                    }
                }
                if (i + 1 < p.size() && c.own((p[i] + p[i + 1]) * 0.5)) {
                    V2 n2 = (p[i + 1] - p[i]).norm().perp();
                    double span = (p[i + 1] - p[i]).len();
                    std::vector<double> offs = major ? std::vector<double>{-6.2, 0.0, 6.2} : std::vector<double>{-0.75, 0.75};
                    for (double o : offs) {
                        double h = major ? (o == 0.0 ? H * 0.86 : H * 0.72) - 0.6 : H - 0.55;
                        V3 a(p[i].x + n2.x * o, p[i].y + n2.y * o, z[i] + h), b(p[i + 1].x + n2.x * o, p[i + 1].y + n2.y * o, z[i + 1] + h);
                        wire(DK, a, b, span * (major ? 0.025 : 0.015), major ? 0.05 : 0.03, wire_c);
                    }
                }
            }
        }
    // poles from points (catenary masts for trams and trolleybuses, wooden or concrete power poles)
    for (auto &t : tiles)
        for (auto &pt : t->points) {
            if (pt.kind.rfind("pole:", 0) != 0) continue;
            V2 v = c.fr.to_local(pt.p.lat, pt.p.lon);
            if (!c.own(v) || c.roads.clearance(v) < 0.3) continue;
            double z = c.ground(v);
            bool cat = pt.kind == "pole:catenary_mast";
            Col pc = cat ? Col{0.35f, 0.4f, 0.38f, 1} : Col{0.72f, 0.71f, 0.68f, 1};
            (cat ? MT : CO).cyl(v, cat ? 0.12 : 0.16, cat ? 0.09 : 0.11, z - 0.2, z + (cat ? 7.5 : 9.5), 8, pc);
            c.coll_prism(v, 0, 0.3, 0.3, z - 0.2, z + 6.5);
        }
    // fences, walls, hedges, guard rails
    for (auto &t : tiles)
        for (auto &ln : t->lines) {
            if (ln.kind.rfind("barrier:", 0) != 0) continue;
            std::string k = ln.kind.substr(8);
            auto p = densify(local_line(c, ln.pts), 3.0);
            if (p.size() < 2) continue;
            double h = k == "wall" || k == "city_wall" ? 2.0 : k == "hedge" ? 1.3 : k == "retaining_wall" ? 1.0 : k == "guard_rail" ? 0.8 : 1.7;
            double w = k == "hedge" ? 0.8 : k == "fence" ? 0.05 : k == "guard_rail" ? 0.1 : 0.3;
            MeshBuf &M = k == "hedge" ? c.out.ms[M_GRASS] : k == "fence" || k == "guard_rail" ? MT : CO;
            Col col = k == "hedge" ? Col{0.3f, 0.45f, 0.2f, 1} : k == "fence" ? Col{0.35f, 0.38f, 0.36f, 1} : Col{0.72f, 0.7f, 0.66f, 1};
            for (size_t i = 0; i + 1 < p.size(); i++) {
                V2 mid = (p[i] + p[i + 1]) * 0.5;
                if (!c.own(mid)) continue;
                // not across carriageways (gates and gaps are not in the data)
                if (c.roads.clearance(mid) < 0.5) continue;
                V3 a(p[i].x, p[i].y, c.ground(p[i]) - 0.2), b(p[i + 1].x, p[i + 1].y, c.ground(p[i + 1]) - 0.2);
                beam(M, a, b, w, h + 0.2, col);
                if (k == "fence") {
                    // posts
                    MT.obox(p[i], 0, 0.06, 0.06, a.z, a.z + h + 0.25, Col{0.25f, 0.27f, 0.26f, 1});
                }
                V2 d = V2(b.x - a.x, b.y - a.y);
                double l = d.len();
                c.coll_prism(mid, std::atan2(d.y, d.x), l, std::max(0.15, w), std::min(a.z, b.z), std::max(a.z, b.z) + h + 0.2);
            }
        }
    // public art (statues and sculptures mapped as points) and small fountains
    for (auto &t : tiles)
        for (auto &pt : t->points) {
            bool art = pt.kind.rfind("artwork", 0) == 0, fountain = pt.kind == "fountain";
            if (!art && !fountain) continue;
            V2 v = c.fr.to_local(pt.p.lat, pt.p.lon);
            if (!c.own(v) || c.roads.clearance(v) < 1.0) continue;
            bool inside = false;
            for (auto &br : c.building_rings)
                if (point_in_ring(v, br)) inside = true;
            if (inside) continue;
            uint64_t h = hash_ll(pt.p.lat, pt.p.lon, 501);
            double z = c.ground(v), yaw = hash01(h, 1) * 2 * PI;
            if (fountain) {
                Col st{0.66f, 0.64f, 0.6f, 1};
                CO.cyl(v, 2.2, 2.2, z - 0.2, z + 0.5, 20, st, false);
                c.out.ms[M_WATER].cyl(v, 2.0, 2.0, z + 0.38, z + 0.4, 20, Col{1, 1, 1, 1}, true);
                CO.cyl(v, 0.35, 0.25, z, z + 1.6, 10, st, true);
                c.coll_prism(v, 0, 4.2, 4.2, z - 0.2, z + 0.5);
                continue;
            }
            std::string type = pt.kind.size() > 8 ? pt.kind.substr(8) : "";
            Col bronze{0.32f, 0.28f, 0.2f, 1};
            if (type == "statue" || (type.empty() && hash01(h, 2) < 0.5)) {
                CO.obox(v, yaw, 1.0, 1.0, z - 0.1, z + 1.4, Col{0.45f, 0.4f, 0.38f, 1});
                statue(c.out.ms, v, yaw, z + 1.4, 2.0, bronze, h);
                c.coll_prism(v, yaw, 1.0, 1.0, z - 0.1, z + 3.2);
            } else {
                // an abstract piece: turned stacked blocks in steel or painted metal
                Col pc = hash01(h, 3) < 0.5 ? Col{0.7f, 0.72f, 0.75f, 1} : Col{0.75f, 0.2f, 0.12f, 1};
                CO.obox(v, yaw, 1.4, 1.4, z - 0.1, z + 0.4, Col{0.6f, 0.6f, 0.58f, 1});
                double zz = z + 0.4;
                for (int k = 0; k < 4; k++) {
                    double sz = 1.1 - k * 0.2, hh = 0.5 + hash01(h, 10 + k) * 0.6;
                    MT.obox(v, yaw + k * 0.35, sz, sz * 0.35, zz, zz + hh, pc, true);
                    zz += hh;
                }
                c.coll_prism(v, yaw, 1.4, 1.4, z - 0.1, zz);
            }
        }
    // metro entrances / stations: a post with the "М" (CIS) / "M" sign
    for (auto &t : tiles)
        for (auto &pt : t->points) {
            if (pt.kind.rfind("metro|", 0) != 0) continue;
            V2 v = c.fr.to_local(pt.p.lat, pt.p.lon);
            if (!c.own(v)) continue;
            double la, lo;
            c.fr.to_latlon(v, la, lo);
            const char *letter = cyrillic_region(la, lo) ? "М" : "M";
            // stand it on the pavement, not in the road or a building
            if (c.roads.clearance(v) < 0.5) continue;
            double z = c.ground(v);
            MT.cyl(v, 0.07, 0.07, z - 0.1, z + 3.2, 8, Col{0.3f, 0.3f, 0.32f, 1});
            Col bc{0.1f, 0.3f, 0.75f, 1};
            for (int k = 0; k < 4; k++) {
                double a = k * PI / 2;
                V2 dv(std::cos(a), std::sin(a));
                V2 q = v + dv * 0.42;
                V3 nrm(dv.x, dv.y, 0);
                V2 side = dv.perp() * 0.4;
                PM.quad(V3(q.x - side.x, q.y - side.y, z + 2.9), V3(q.x + side.x, q.y + side.y, z + 2.9), V3(q.x + side.x, q.y + side.y, z + 3.7), V3(q.x - side.x, q.y - side.y, z + 3.7), nrm, bc);
                c.out.labels.push_back({V3(q.x + dv.x * 0.02, q.y + dv.y * 0.02, z + 3.3), a, 0.6, letter, Col{1, 1, 1, 1}, 0.62});
            }
            PM.obox(v, 0, 0.84, 0.84, z + 3.7, z + 3.72, bc);
            c.coll_prism(v, 0, 0.2, 0.2, z, z + 3);
        }
}

}  // namespace ec
