#include <cmath>
#include "roadgeom.hpp"

namespace es {

namespace {

V3 v3(V2 p, double z, V2 o) { return {p.x - o.x, p.y - o.y, z}; }

// Generic strip between polylines A and B (same count); front face is cross(B-A, along).
void strip(Mesh &m, const std::vector<V3> &A, const std::vector<V3> &B, const std::vector<V2> &uvA, const std::vector<V2> &uvB, V2 uv2 = {0, 0},
           float r = 1, float g = 1, float b = 1, float a = 1) {
    size_t n = A.size();
    if (n < 2 || B.size() != n) return;
    std::vector<int> ia(n), ib(n);
    for (size_t k = 0; k < n; k++) {
        size_t k0 = k == 0 ? 0 : k - 1, k1 = k + 1 < n ? k + 1 : n - 1;
        V3 along = (A[k1] - A[k0]) + (B[k1] - B[k0]);
        V3 nrm = norm(cross(B[k] - A[k], along));
        ia[k] = m.vert(A[k], nrm, uvA[k], uv2, r, g, b, a);
        ib[k] = m.vert(B[k], nrm, uvB[k], uv2, r, g, b, a);
    }
    for (size_t k = 0; k + 1 < n; k++) m.quad(ia[k], ib[k], ib[k + 1], ia[k + 1]);
}

std::vector<V2> world_uv(const std::vector<V3> &P, V2 o) {
    std::vector<V2> u;
    for (auto &p : P) u.push_back({p.x + o.x, p.y + o.y});
    return u;
}

struct Sample {
    double s;
    V2 p, t;  // point, tangent (a->b)
    double miter;
};

// Samples along [s0, s1] of an edge: original vertices plus subdivision to <= step.
std::vector<Sample> sample_edge(const RoadEdge &e, double s0, double s1, double step) {
    std::vector<Sample> out;
    if (s1 - s0 < 1e-3) return out;
    std::vector<double> ss;
    ss.push_back(s0);
    double acc = 0;
    for (size_t k = 0; k + 1 < e.pts.size(); k++) {
        acc += dist(e.pts[k], e.pts[k + 1]);
        if (acc > s0 + 0.05 && acc < s1 - 0.05) ss.push_back(acc);
    }
    ss.push_back(s1);
    std::vector<double> fine;
    for (size_t i = 0; i + 1 < ss.size(); i++) {
        int n = std::max(1, int(std::ceil((ss[i + 1] - ss[i]) / step)));
        for (int j = 0; j < n; j++) fine.push_back(ss[i] + (ss[i + 1] - ss[i]) * j / n);
    }
    fine.push_back(s1);
    for (size_t i = 0; i < fine.size(); i++) {
        double s = fine[i];
        Sample x;
        x.s = s;
        V2 d0, d1;
        x.p = e.at(s, &d0);
        // tangent: average of the segment before and after s (miter); ends use the end segment
        double eps = 0.02;
        V2 pa = e.at(std::max(0.0, s - eps)), pb = e.at(std::min(e.len, s + eps));
        V2 tb = norm(pb - x.p), ta = norm(x.p - pa);
        if (i == 0) ta = tb;
        if (i + 1 == fine.size()) tb = ta;
        if (len(ta) < 0.5) ta = tb;
        if (len(tb) < 0.5) tb = ta;
        V2 t = norm(ta + tb);
        if (len(t) < 0.5) t = tb;
        x.t = t;
        x.miter = 1.0 / std::max(0.5, dot(perp(t), perp(tb)));
        out.push_back(x);
    }
    return out;
}

V2 bez(V2 a, V2 c, V2 b, double t) { return a * ((1 - t) * (1 - t)) + c * (2 * (1 - t) * t) + b * (t * t); }

// corner curve from a (leaving along -da, i.e. towards the junction) to b; c = intersection of the two
// boundary lines or midpoint if they do not meet sensibly
std::vector<V2> corner_curve(V2 a, V2 da, V2 b, V2 db, int nseg, bool &curved) {
    double t, u;
    curved = false;
    std::vector<V2> out;
    V2 c = (a + b) * 0.5;
    if (line_intersect(a, da, b, db, t, u) && t < 0.05 && u < 0.05 && t > -60 && u > -60) c = a + da * t, curved = true;
    for (int i = 0; i <= nseg; i++) out.push_back(curved ? bez(a, c, b, double(i) / nseg) : lerp(a, b, double(i) / nseg));
    return out;
}

double fillet_radius(int prio_a, int prio_b) {
    int p = std::max(prio_a, prio_b);
    return p >= 7 ? 9.0 : p >= 4 ? 5.0 : 2.5;
}

}  // namespace

void RoadGeom::build(RoadNet &N) {
    net = &N;
    junctions.assign(N.nodes.size(), {});
    for (auto &e : N.edges) e.trim_a = e.trim_b = 0;
    // pass 1: required trims from corner geometry
    for (size_t n = 0; n < N.nodes.size(); n++) {
        auto &node = N.nodes[n];
        if (node.edges.size() < 2) continue;
        Junction &J = junctions[n];
        J.node = int(n);
        for (int ei : node.edges) {
            auto &e = N.edges[ei];
            for (int end = 0; end < 2; end++) {
                bool at_a = end == 0;
                if ((at_a ? e.a : e.b) != int(n)) continue;
                JArm A;
                A.edge = ei, A.at_a = at_a;
                A.h = e.half();
                A.swl = at_a ? e.sw(1) : e.sw(-1);
                A.swr = at_a ? e.sw(-1) : e.sw(1);
                double s = std::min(e.len, 10.0);
                V2 p = e.at(at_a ? s : e.len - s);
                A.D = norm(p - node.p);
                if (len(A.D) < 0.5) A.D = {1, 0};
                A.ang = std::atan2(A.D.y, A.D.x);
                J.arms.push_back(A);
            }
        }
        std::sort(J.arms.begin(), J.arms.end(), [](const JArm &a, const JArm &b) { return a.ang < b.ang; });
        size_t k = J.arms.size();
        std::vector<double> need(k, k >= 3 ? 1.0 : 0.0);
        for (size_t i = 0; i < k; i++) {
            auto &A = J.arms[i];
            auto &B = J.arms[(i + 1) % k];
            double th = B.ang - A.ang;
            if (th <= 0) th += 2 * PI;
            if (th >= PI - 0.08) continue;  // straight or reflex: no corner
            double t, u;
            // A's left line and B's right line
            if (!line_intersect(node.p + perp(A.D) * A.h, A.D, node.p - perp(B.D) * B.h, B.D, t, u)) continue;
            double r = fillet_radius(N.edges[A.edge].prio, N.edges[B.edge].prio);
            double tl = std::min(25.0, r / std::tan(th / 2));
            need[i] = std::max(need[i], t + tl);
            need[(i + 1) % k] = std::max(need[(i + 1) % k], u + tl);
        }
        for (size_t i = 0; i < k; i++) J.arms[i].trim = need[i];
    }
    // pass 2: clamp trims so both ends fit on every edge
    for (auto &J : junctions)
        for (auto &A : J.arms) {
            auto &e = N.edges[A.edge];
            (A.at_a ? e.trim_a : e.trim_b) = std::min(A.trim, 45.0);
        }
    for (auto &e : N.edges) {
        double tot = e.trim_a + e.trim_b, mx = e.len * 0.9;
        if (tot > mx && tot > 0) e.trim_a *= mx / tot, e.trim_b *= mx / tot;
    }
    // pass 3: corners and outlines
    for (size_t n = 0; n < N.nodes.size(); n++) {
        Junction &J = junctions[n];
        if (J.arms.empty()) continue;
        auto &node = N.nodes[n];
        size_t k = J.arms.size();
        double maxtrim = 0;
        for (auto &A : J.arms) {
            auto &e = N.edges[A.edge];
            A.trim = A.at_a ? e.trim_a : e.trim_b;
            maxtrim = std::max(maxtrim, A.trim);
            double s = A.at_a ? A.trim : e.len - A.trim;
            V2 t;
            A.P = e.at(s, &t);
            // tangent of the segment at s, pointing away from the junction
            V2 d = A.at_a ? t : t * -1.0;
            if (!A.at_a) {
                V2 q = e.at(std::max(0.0, s - 0.01));
                d = norm(q - A.P);
                if (len(d) < 0.5) d = t * -1.0;
            }
            A.D = d;
            double z = e.z_at(s);
            A.L = V3(A.P + perp(d) * A.h, z);
            A.R = V3(A.P - perp(d) * A.h, z);
        }
        J.centre = V3(node.p, node.z);
        J.flat = k == 2 && maxtrim < 0.05 && std::fabs(J.arms[0].h - J.arms[1].h) < 0.05;
        J.corners.assign(k, {});
        J.outline.clear();
        for (size_t i = 0; i < k; i++) {
            auto &A = J.arms[i];
            auto &B = J.arms[(i + 1) % k];
            JCorner &C = J.corners[i];
            bool curved;
            auto kc = corner_curve(A.L.xy(), A.D, B.R.xy(), B.D, 8, curved);
            for (size_t q = 0; q < kc.size(); q++) C.kerb.push_back(V3(kc[q], lerpd(A.L.z, B.R.z, double(q) / (kc.size() - 1))));
            if (A.swl > 0 && B.swr > 0 && k >= 2) {
                V2 ao = A.L.xy() + perp(A.D) * A.swl, bo = B.R.xy() - perp(B.D) * B.swr;
                bool c2;
                auto oc = corner_curve(ao, A.D, bo, B.D, 8, c2);
                for (size_t q = 0; q < oc.size(); q++) C.outer.push_back(V3(oc[q], lerpd(A.L.z, B.R.z, double(q) / (oc.size() - 1)) + KERB_H));
            }
            J.outline.push_back(A.R);
            J.outline.push_back(A.L);
            for (size_t q = 1; q + 1 < C.kerb.size(); q++) J.outline.push_back(C.kerb[q]);
        }
        J.box = Rect();
        for (auto &p : J.outline) J.box.add(p.xy());
        for (auto &C : J.corners)
            for (auto &p : C.outer) J.box.add(p.xy());
        J.box.add(node.p);
    }
}

// ------------------------------------------------------------------ emission

void RoadGeom::emit_junction(int n, V2 o, Mesh *mats) const {
    const Junction &J = junctions[n];
    if (J.arms.empty()) return;
    const RoadNode &node = net->nodes[n];
    const auto &edges = net->edges;
    bool bridge = false, tunnel = false;
    for (auto &A : J.arms) bridge |= edges[A.edge].bridge || edges[A.edge].level > 0, tunnel |= edges[A.edge].tunnel || edges[A.edge].level < 0;
    if (!J.flat && J.outline.size() >= 3) {
        Mesh &m = mats[M_JUNCTION];
        int c = m.vert(v3(J.centre.xy(), J.centre.z, o), {0, 0, 1}, {J.centre.x, J.centre.y});
        std::vector<int> ids;
        for (auto &p : J.outline) ids.push_back(m.vert(v3(p.xy(), p.z, o), {0, 0, 1}, {p.x, p.y}));
        for (size_t i = 0; i < ids.size(); i++) m.tri(c, ids[i], ids[(i + 1) % ids.size()]);
    }
    // corners: kerb faces and pavement
    for (size_t i = 0; i < J.corners.size(); i++) {
        const JCorner &C = J.corners[i];
        if (C.outer.empty()) {
            // no pavement: low skirt so the carriageway edge never floats
            if (!bridge && !tunnel) {
                std::vector<V3> top, bot;
                for (size_t q = 0; q < C.kerb.size(); q++) {
                    V3 p = C.kerb[q];
                    // outward = away from the junction centre
                    V2 out = norm(p.xy() - J.centre.xy());
                    top.push_back(v3(p.xy(), p.z, o)), bot.push_back(v3(p.xy() + out * 1.2, p.z - 0.6, o));
                }
                std::vector<V2> u1, u2;
                double acc = 0;
                for (size_t q = 0; q < top.size(); q++) {
                    if (q) acc += dist(top[q].xy(), top[q - 1].xy());
                    u1.push_back({acc, 0}), u2.push_back({acc, 0.6});
                }
                strip(mats[M_KERB], top, bot, u1, u2);  // faces outwards
            }
            continue;
        }
        std::vector<V3> kb, kt, ot, ob;
        for (size_t q = 0; q < C.kerb.size(); q++) {
            V3 a = C.kerb[q], b = C.outer[q];
            kb.push_back(v3(a.xy(), a.z, o));
            kt.push_back(v3(a.xy(), a.z + KERB_H, o));
            ot.push_back(v3(b.xy(), b.z, o));
            ob.push_back(v3(b.xy(), b.z - KERB_H - 0.45, o));
        }
        std::vector<V2> ua, ub;
        double acc = 0;
        for (size_t q = 0; q < kb.size(); q++) {
            if (q) acc += dist(kb[q].xy(), kb[q - 1].xy());
            ua.push_back({acc, 0}), ub.push_back({acc, KERB_H});
        }
        strip(mats[M_KERB], kb, kt, ua, ub);                             // kerb face towards the carriageway
        strip(mats[M_PAVEMENT], kt, ot, world_uv(kt, o), world_uv(ot, o));  // pavement top
        strip(mats[M_KERB], ot, ob, ua, ub);                             // outer edge
    }
    (void)node;
}

void RoadGeom::emit_edge(int ei, V2 o, Mesh *mats) const {
    const RoadEdge &e = net->edges[ei];
    double s0 = e.trim_a, s1 = e.len - e.trim_b;
    auto S = sample_edge(e, s0, s1, 4.0);
    if (S.size() < 2) return;
    bool bridge = e.bridge || e.level > 0, tunnel = e.tunnel || e.level < 0;
    double h = e.half();
    size_t n = S.size();
    std::vector<V3> L(n), R(n);
    std::vector<V2> uL(n), uR(n);
    std::vector<double> z(n);
    for (size_t k = 0; k < n; k++) {
        z[k] = e.z_at(S[k].s);
        V2 nl = perp(S[k].t) * (h * S[k].miter);
        L[k] = v3(S[k].p + nl, z[k], o);
        R[k] = v3(S[k].p - nl, z[k], o);
        uL[k] = {0, S[k].s};
        uR[k] = {e.width, S[k].s};
    }
    // exact ends: match the junction corners
    auto fix_end = [&](size_t k, bool at_a) {
        const Junction &J = junctions[at_a ? e.a : e.b];
        for (auto &A : J.arms)
            if (A.edge == ei && A.at_a == at_a) {
                // arm left/right are relative to looking away from the junction
                V3 l = at_a ? A.L : A.R, r = at_a ? A.R : A.L;
                L[k] = v3(l.xy(), l.z, o), R[k] = v3(r.xy(), r.z, o);
            }
    };
    if (e.trim_a >= 0) fix_end(0, true);
    fix_end(n - 1, false);
    float flags = float((e.oneway ? 1 : 0) | (e.prio >= 4 ? 2 : 0) | (e.prio >= 7 && !e.oneway ? 4 : 0) | (e.surface == "paving_stones" || e.surface == "sett" ? 8 : 0));
    strip(mats[M_ASPHALT], L, R, uL, uR, {double(e.lanes) + 16.0 * flags, e.width});

    // sides: pavement (urban), skirt (rural), deck (bridge), walls (tunnel)
    std::vector<V3> outerL(n), outerR(n);  // outermost top lines (for bridges)
    for (int side = 0; side < 2; side++) {
        bool left = side == 0;
        double sw = e.sw(left ? 1 : -1);
        bool has_sw = sw > 0;
        std::vector<V3> &edge = left ? L : R;
        std::vector<V3> kb(n), kt(n), ot(n), ob(n), sk(n);
        std::vector<V2> ua(n), ub(n);
        for (size_t k = 0; k < n; k++) {
            V3 p = edge[k];
            V2 out = perp(S[k].t) * (left ? 1.0 : -1.0);
            kb[k] = p;
            kt[k] = {p.x, p.y, p.z + KERB_H};
            V2 q = p.xy() + out * (sw * S[k].miter);
            ot[k] = {q.x, q.y, p.z + KERB_H};
            ob[k] = {q.x, q.y, p.z - (bridge ? 1.4 : 0.45)};
            // bridges: vertical deck side; roads: a sloped shoulder down to the ground (drivable)
            V2 sq = bridge ? p.xy() : p.xy() + out * (1.2 * S[k].miter);
            sk[k] = {sq.x, sq.y, p.z - (bridge ? 1.4 : 0.6)};
            ua[k] = {S[k].s, 0}, ub[k] = {S[k].s, KERB_H};
        }
        if (tunnel) {
            // tunnel: walls 1 m outside the carriageway, 5 m high, facing in
            std::vector<V3> wb(n), wt(n);
            std::vector<V2> wu(n), wv(n);
            for (size_t k = 0; k < n; k++) {
                V2 out = perp(S[k].t) * (left ? 1.0 : -1.0);
                V2 q = edge[k].xy() + out * 1.0;
                wb[k] = {q.x, q.y, edge[k].z - 0.1}, wt[k] = {q.x, q.y, edge[k].z + 5.0};
                wu[k] = {S[k].s, 0}, wv[k] = {S[k].s, 5};
            }
            if (left) strip(mats[M_CONCRETE], wt, wb, wv, wu);
            else strip(mats[M_CONCRETE], wb, wt, wu, wv);
            (left ? outerL : outerR) = wt;
            // floor between carriageway and wall
            std::vector<V3> fl(n);
            for (size_t k = 0; k < n; k++) fl[k] = {wb[k].x, wb[k].y, edge[k].z};
            if (left) strip(mats[M_KERB], fl, edge, world_uv(fl, o), world_uv(edge, o));
            else strip(mats[M_KERB], edge, fl, world_uv(edge, o), world_uv(fl, o));
            continue;
        }
        if (has_sw) {
            if (left) {
                strip(mats[M_KERB], kt, kb, ub, ua);                                      // face right (inwards)
                strip(mats[M_PAVEMENT], ot, kt, world_uv(ot, o), world_uv(kt, o));        // top
                strip(mats[M_KERB], ob, ot, ua, ub);                                      // outer, facing left
            } else {
                strip(mats[M_KERB], kb, kt, ua, ub);
                strip(mats[M_PAVEMENT], kt, ot, world_uv(kt, o), world_uv(ot, o));
                strip(mats[M_KERB], ot, ob, ub, ua);
            }
            (left ? outerL : outerR) = ot;
        } else {
            if (left) strip(bridge ? mats[M_CONCRETE] : mats[M_KERB], sk, kb, ua, ub);
            else strip(bridge ? mats[M_CONCRETE] : mats[M_KERB], kb, sk, ub, ua);
            (left ? outerL : outerR) = kb;
        }
    }
    if (tunnel) {
        // ceiling (facing down) and roof top (facing up, seen where the ground opens near portals)
        std::vector<V3> cl(n), cr(n), tl(n), tr(n);
        for (size_t k = 0; k < n; k++) {
            cl[k] = outerL[k], cr[k] = outerR[k];
            tl[k] = {cl[k].x, cl[k].y, cl[k].z + 0.6}, tr[k] = {cr[k].x, cr[k].y, cr[k].z + 0.6};
        }
        strip(mats[M_CONCRETE], cr, cl, world_uv(cr, o), world_uv(cl, o));
        strip(mats[M_CONCRETE], tl, tr, world_uv(tl, o), world_uv(tr, o));
        return;
    }
    if (bridge) {
        // underside and parapets
        std::vector<V3> bl(n), br(n);
        for (size_t k = 0; k < n; k++) bl[k] = {outerL[k].x, outerL[k].y, z[k] - 1.4}, br[k] = {outerR[k].x, outerR[k].y, z[k] - 1.4};
        strip(mats[M_CONCRETE], br, bl, world_uv(br, o), world_uv(bl, o));
        for (int side = 0; side < 2; side++) {
            bool left = side == 0;
            auto &ol = left ? outerL : outerR;
            if (e.sw(left ? 1 : -1) > 0) {  // outer side face of the deck (without pavement the skirt already is one)
                std::vector<V3> b(n);
                std::vector<V2> u0(n), u1(n);
                for (size_t k = 0; k < n; k++) b[k] = {ol[k].x, ol[k].y, z[k] - 1.4}, u0[k] = {S[k].s, 0}, u1[k] = {S[k].s, 1.4};
                if (left) strip(mats[M_CONCRETE], b, ol, u0, u1);
                else strip(mats[M_CONCRETE], ol, b, u1, u0);
            }
            // parapet: 0.3 m wide, 1.0 m high, standing on the outer edge
            std::vector<V3> pi(n), pit(n), pot(n), po(n);
            std::vector<V2> u0(n), u1(n);
            for (size_t k = 0; k < n; k++) {
                V2 out = perp(S[k].t) * (left ? 1.0 : -1.0);
                V2 a = ol[k].xy() - out * 0.3, b = ol[k].xy();
                pi[k] = {a.x, a.y, ol[k].z}, pit[k] = {a.x, a.y, ol[k].z + 1.0}, pot[k] = {b.x, b.y, ol[k].z + 1.0}, po[k] = {b.x, b.y, ol[k].z};
                u0[k] = {S[k].s, 0}, u1[k] = {S[k].s, 1};
            }
            if (left) {
                strip(mats[M_CONCRETE], pit, pi, u1, u0);
                strip(mats[M_CONCRETE], pot, pit, u1, u0);
                strip(mats[M_CONCRETE], po, pot, u0, u1);
            } else {
                strip(mats[M_CONCRETE], pi, pit, u0, u1);
                strip(mats[M_CONCRETE], pit, pot, u0, u1);
                strip(mats[M_CONCRETE], pot, po, u1, u0);
            }
        }
        // piers every ~28 m where the deck is high above the ground
        // (ground height is passed through z of the edge profile minus the structure offset is unknown here;
        //  the tile builder adds piers because it knows the terrain)
    }
}

// ------------------------------------------------------------------ queries

static const double BUCKET = 16.0;

void RoadGeom::Cache::bucket_range(const Rect &q, int &x0, int &y0, int &x1, int &y1) const {
    x0 = std::max(0, int(std::floor((q.mn.x - rect.mn.x) / BUCKET)));
    y0 = std::max(0, int(std::floor((q.mn.y - rect.mn.y) / BUCKET)));
    x1 = std::min(bw - 1, int(std::floor((q.mx.x - rect.mn.x) / BUCKET)));
    y1 = std::min(bh - 1, int(std::floor((q.mx.y - rect.mn.y) / BUCKET)));
}

void RoadGeom::prefetch(const Rect &r, Cache &c) const {
    net->query_rect(r.grown(60), c.edges);
    c.nodes.clear();
    for (int ei : c.edges) c.nodes.push_back(net->edges[ei].a), c.nodes.push_back(net->edges[ei].b);
    std::sort(c.nodes.begin(), c.nodes.end());
    c.nodes.erase(std::unique(c.nodes.begin(), c.nodes.end()), c.nodes.end());
    // bucket everything by its padded extent so a query only visits nearby items
    c.rect = r;
    c.bw = std::max(1, int(std::ceil((r.mx.x - r.mn.x) / BUCKET)));
    c.bh = std::max(1, int(std::ceil((r.mx.y - r.mn.y) / BUCKET)));
    c.be.assign(size_t(c.bw) * c.bh, {});
    c.bn.assign(size_t(c.bw) * c.bh, {});
    const double PAD = 32.0;  // max reach used by queries
    for (size_t i = 0; i < c.edges.size(); i++) {
        auto &e = net->edges[c.edges[i]];
        double ext = e.half() + e.sw_width + PAD;
        for (size_t k = 0; k + 1 < e.pts.size(); k++) {
            Rect sb;
            sb.add(e.pts[k]), sb.add(e.pts[k + 1]);
            int x0, y0, x1, y1;
            c.bucket_range(sb.grown(ext), x0, y0, x1, y1);
            for (int y = y0; y <= y1; y++)
                for (int x = x0; x <= x1; x++) {
                    auto &v = c.be[size_t(y) * c.bw + x];
                    if (v.empty() || v.back() != int(i)) v.push_back(int(i));
                }
        }
    }
    for (size_t i = 0; i < c.nodes.size(); i++) {
        auto &J = junctions[c.nodes[i]];
        if (J.arms.empty()) continue;
        int x0, y0, x1, y1;
        c.bucket_range(J.box.grown(PAD), x0, y0, x1, y1);
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++) c.bn[size_t(y) * c.bw + x].push_back(int(i));
    }
}

SurfaceHit RoadGeom::query(V2 p, const Cache &c, double reach) const {
    SurfaceHit best;
    double best_road_d = 1e9;  // for choosing among overlapping carriageways
    double best_pave_d = 1e9;  // overlapping pavements: the nearest kerb wins
    auto consider_out = [&](double d, double z, bool br) {
        if (best.kind == 0 && d < best.dist) best.dist = d, best.znear = z, best.bridge = br;
    };
    static const std::vector<int> empty;
    const std::vector<int> *ev = nullptr, *nv = nullptr;
    bool bucketed = c.bw > 0 && c.rect.contains(p);
    if (bucketed) {
        int bx = std::min(c.bw - 1, int((p.x - c.rect.mn.x) / BUCKET)), by = std::min(c.bh - 1, int((p.y - c.rect.mn.y) / BUCKET));
        ev = &c.be[size_t(by) * c.bw + bx], nv = &c.bn[size_t(by) * c.bw + bx];
    }
    size_t ne = bucketed ? ev->size() : c.edges.size();
    for (size_t ii = 0; ii < ne; ii++) {
        int ei = bucketed ? c.edges[(*ev)[ii]] : c.edges[ii];
        const RoadEdge &e = net->edges[ei];
        bool br = e.bridge || e.level > 0, tu = e.tunnel || e.level < 0;
        // quick reject with the edge's bounding box
        double h = e.half(), ext = h + e.sw_width + reach;
        double acc = 0, bd = 1e18, bs = 0, bside = 1;
        for (size_t k = 0; k + 1 < e.pts.size(); k++) {
            V2 a = e.pts[k], b = e.pts[k + 1];
            double sl = dist(a, b);
            if (std::min(a.x, b.x) - ext > p.x || std::max(a.x, b.x) + ext < p.x || std::min(a.y, b.y) - ext > p.y || std::max(a.y, b.y) + ext < p.y) {
                acc += sl;
                continue;
            }
            double t;
            double d = seg_dist(p, a, b, &t);
            if (d < bd) bd = d, bs = acc + t * sl, bside = cross(b - a, p - a) >= 0 ? 1 : -1;
            acc += sl;
        }
        if (bd > ext) continue;
        double sw = e.sw(bside);
        double z = e.z_at(bs);
        bool inside_span = bs >= e.trim_a - 0.01 && bs <= e.len - e.trim_b + 0.01;
        if (tu) {
            if (bd < h + 1.5 && inside_span) {
                best.tunnel = true;
                best.tunnel_z = std::max(best.tunnel_z, z);
            }
            continue;
        }
        if (!inside_span) {
            // beyond the trimmed span the junction polygon decides; still a "near surface" for blending
            consider_out(std::max(0.0, bd - h - sw), z + (sw > 0 ? KERB_H : 0), br);
            continue;
        }
        if (bd < h + 1.0) {
            if (best.kind != 1 || (best.bridge && !br) || (!br && z < best.z)) {
                best.kind = 1, best.z = z, best_road_d = bd, best.bridge = br;
                best.margin = bd >= h && sw <= 0;
            }
        } else if (sw > 0 && bd < h + sw + 0.3) {
            if (best.kind == 0 || (best.kind == 2 && bd - h < best_pave_d)) best.kind = 2, best.z = z + KERB_H, best.bridge = br, best_pave_d = bd - h;
        } else
            consider_out(bd - h - sw, z + (sw > 0 ? KERB_H : 0), br);
    }
    size_t nn = bucketed ? nv->size() : c.nodes.size();
    for (size_t ii = 0; ii < nn; ii++) {
        int ni = bucketed ? c.nodes[(*nv)[ii]] : c.nodes[ii];
        const Junction &J = junctions[ni];
        if (J.arms.empty() || J.flat) continue;
        bool br = false, tu = false;
        for (auto &A : J.arms) br |= net->edges[A.edge].bridge || net->edges[A.edge].level > 0, tu |= net->edges[A.edge].tunnel || net->edges[A.edge].level < 0;
        if (tu) continue;
        if (!J.box.grown(reach + 1).contains(p)) continue;
        Poly ol;
        for (auto &q : J.outline) ol.push_back(q.xy());
        // distance to outline
        double d = 1e18;
        for (size_t i = 0; i < ol.size(); i++) d = std::min(d, seg_dist(p, ol[i], ol[(i + 1) % ol.size()]));
        bool in = point_in_poly(ol, p);
        if (in || d < 1.0) {
            if (best.kind != 1 || (best.bridge && !br)) {
                // height: fan triangle containing p, else nearest outline vertex
                double z = J.centre.z;
                for (size_t i = 0; i < J.outline.size(); i++) {
                    V3 a = J.centre, b = J.outline[i], c3 = J.outline[(i + 1) % J.outline.size()];
                    double den = cross(b.xy() - a.xy(), c3.xy() - a.xy());
                    if (std::fabs(den) < 1e-9) continue;
                    double u = cross(p - a.xy(), c3.xy() - a.xy()) / den, v = cross(b.xy() - a.xy(), p - a.xy()) / den;
                    if (u >= -0.02 && v >= -0.02 && u + v <= 1.02) {
                        z = a.z + (b.z - a.z) * u + (c3.z - a.z) * v;
                        break;
                    }
                }
                if (!in) {
                    // within the margin: use the nearest outline point height
                    double bdd = 1e18;
                    for (auto &q : J.outline)
                        if (dist(q.xy(), p) < bdd) bdd = dist(q.xy(), p), z = q.z;
                }
                best.kind = 1, best.z = z, best.bridge = br;
                best.margin = !in;
                best_road_d = 0;
            }
            continue;
        }
        // corner pavements
        bool done = false;
        for (auto &C : J.corners) {
            if (C.outer.empty()) continue;
            Poly pv;
            for (auto &q : C.kerb) pv.push_back(q.xy());
            for (size_t q = C.outer.size(); q-- > 0;) pv.push_back(C.outer[q].xy());
            double dd = 1e18;
            for (size_t i = 0; i < pv.size(); i++) dd = std::min(dd, seg_dist(p, pv[i], pv[(i + 1) % pv.size()]));
            if (point_in_poly(pv, p) || dd < 0.3) {
                if (best.kind == 0) best.kind = 2, best.z = C.kerb[C.kerb.size() / 2].z + KERB_H, best.bridge = br;
                done = true;
                break;
            }
            consider_out(dd, C.outer[C.outer.size() / 2].z, br);
        }
        if (!done) consider_out(d, J.centre.z, br);
    }
    return best;
}

double RoadGeom::surface_z(V2 p) const {
    Cache c;
    prefetch(Rect{p - V2{40, 40}, p + V2{40, 40}}, c);
    SurfaceHit h = query(p, c, 0);
    return h.kind ? h.z : NAN;
}

}  // namespace es
