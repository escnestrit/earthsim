// Road network of a chunk: pieces joined into chains, junctions with rounded kerbs, carriageways that sit on
// terrain flattened under them, raised sidewalks with kerbs, markings and zebra crossings, bridges with
// parapets and piers, street lamps.
#include <algorithm>
#include <map>
#include <set>
#include "chunk_ctx.hpp"

namespace ec {

namespace {

struct RPt {
    V2 p;
    double z = 0;
    bool dash = false;  // the segment starting here carries a dash of a dashed line
};

struct Road {
    std::vector<RPt> pts;
    std::string cls;
    double hw = 3;   // carriageway half width
    double sw = 0;   // sidewalk width (0: none)
    bool bridge = false, urban = false;
    int oneway = 0, layer = 0, prio = 0;
    int lanes = 2;
    double trim[2] = {0, 0};
    int node[2] = {-1, -1};
    bool alive = true;
};

struct End {
    int road;
    bool start;
    V2 u;  // unit direction away from the node
    double ang;
};

struct Node {
    V2 p;
    std::vector<End> ends;
    double z = 0;
};

int prio_of(const std::string &c) {
    static const std::pair<const char *, int> P[] = {{"motorway", 9}, {"trunk", 8}, {"primary", 7}, {"secondary", 6}, {"tertiary", 5}, {"motorway_link", 4}, {"trunk_link", 4},
                                                     {"primary_link", 4}, {"secondary_link", 4}, {"tertiary_link", 4}, {"unclassified", 3}, {"residential", 3}, {"road", 3},
                                                     {"living_street", 2}, {"service", 1}};
    for (auto &p : P)
        if (c == p.first) return p.second;
    return 0;
}

bool wants_sidewalk(const std::string &c) {
    return c == "primary" || c == "secondary" || c == "tertiary" || c == "unclassified" || c == "residential" || c == "living_street";
}

uint64_t qkey(V2 p) {
    int64_t i = (int64_t)std::llround(p.x * 2), j = (int64_t)std::llround(p.y * 2);
    return ((uint64_t)(uint32_t)i << 32) | (uint32_t)j;
}

double chain_len(const std::vector<RPt> &p) {
    double s = 0;
    for (size_t i = 1; i < p.size(); i++) s += (p[i].p - p[i - 1].p).len();
    return s;
}

// the polyline between chainages s0 and s1
std::vector<RPt> cut(const std::vector<RPt> &p, double s0, double s1) {
    std::vector<RPt> o;
    double s = 0;
    for (size_t i = 0; i + 1 < p.size(); i++) {
        double l = (p[i + 1].p - p[i].p).len();
        double a = s, b = s + l;
        if (b > s0 && a < s1 && l > 1e-9) {
            auto at = [&](double t) {
                double k = std::clamp((t - a) / l, 0.0, 1.0);
                RPt r;
                r.p = p[i].p + (p[i + 1].p - p[i].p) * k;
                r.z = p[i].z + (p[i + 1].z - p[i].z) * k;
                r.dash = p[i].dash;
                return r;
            };
            if (o.empty()) o.push_back(at(std::max(a, s0)));
            RPt e = at(std::min(b, s1));
            if ((e.p - o.back().p).len() > 0.05) o.push_back(e);
            else o.back().dash = p[i].dash;
        }
        s = b;
    }
    return o;
}

// point at chainage s from one end
RPt at_chainage(const std::vector<RPt> &p, double s, bool from_start) {
    std::vector<RPt> c = from_start ? p : std::vector<RPt>(p.rbegin(), p.rend());
    double acc = 0;
    for (size_t i = 0; i + 1 < c.size(); i++) {
        double l = (c[i + 1].p - c[i].p).len();
        if (acc + l >= s && l > 1e-9) {
            double k = (s - acc) / l;
            RPt r;
            r.p = c[i].p + (c[i + 1].p - c[i].p) * k;
            r.z = c[i].z + (c[i + 1].z - c[i].z) * k;
            return r;
        }
        acc += l;
    }
    return c.back();
}

bool line_x(V2 p, V2 d, V2 q, V2 e, V2 &x) {
    double den = d.cross(e);
    if (std::abs(den) < 1e-6) return false;
    double t = (q - p).cross(e) / den;
    x = p + d * t;
    return true;
}

std::vector<V2> bezier(V2 a, V2 c, V2 b, int n) {
    std::vector<V2> o;
    for (int i = 0; i <= n; i++) {
        double t = double(i) / n, u = 1 - t;
        o.push_back(a * (u * u) + c * (2 * u * t) + b * (t * t));
    }
    return o;
}

Col asphalt_col(const Road &r) {
    // older, greyer surfaces on minor roads
    float k = r.prio >= 5 ? 0.92f : r.prio >= 3 ? 1.0f : 1.06f;
    return {k, k, k, 1};
}

}  // namespace

void build_roads(Ctx &c, const std::vector<std::shared_ptr<const RawTile>> &tiles) {
    struct Piece {
        Road r;
        std::vector<size_t> orig;  // indices of the data's own vertices among the densified points
    };
    std::vector<Piece> pieces;
    std::vector<Road> roads;
    // ---- pieces, densified with dash flags and natural-ground heights
    for (auto &t : tiles) {
        bool urban = t->buildings.size() > 60;
        for (auto &rr : t->roads) {
            if (rr.pts.size() < 2) continue;
            std::vector<V2> p;
            V2 lo(1e300, 1e300), hi(-1e300, -1e300);
            for (auto &q : rr.pts) {
                V2 v = c.fr.to_local(q.lat, q.lon);
                if (!p.empty() && (v - p.back()).len() < 0.1) continue;
                p.push_back(v);
                lo = V2(std::min(lo.x, v.x), std::min(lo.y, v.y)), hi = V2(std::max(hi.x, v.x), std::max(hi.y, v.y));
            }
            if (p.size() < 2 || !c.near(lo, hi, FEATURE_MARGIN)) continue;
            Road r;
            r.cls = rr.cls;
            r.hw = std::clamp(rr.width, 3.0, 40.0) / 2;
            r.bridge = rr.bridge;
            r.oneway = rr.oneway;
            r.layer = rr.layer;
            r.prio = prio_of(rr.cls);
            r.urban = urban;
            r.lanes = std::max(1, (int)std::lround(r.hw * 2 / 3.3));
            if (urban && wants_sidewalk(rr.cls) && !rr.bridge) r.sw = r.prio >= 6 ? 3.5 : 2.5;
            // densify at <= 3 m per piece; dashes on alternate sub-segments
            std::vector<size_t> orig;
            for (size_t i = 0; i < p.size(); i++) {
                if (i > 0) {
                    V2 a = p[i - 1];
                    double l = (p[i] - a).len();
                    int k = std::max(1, (int)std::ceil(l / 3.0));
                    for (int s = 1; s < k; s++) r.pts.push_back({a + (p[i] - a) * (double(s) / k), 0, false});
                }
                orig.push_back(r.pts.size());
                r.pts.push_back({p[i], 0, false});
            }
            for (size_t i = 0; i < r.pts.size(); i++) r.pts[i].dash = i % 2 == 0;
            pieces.push_back({std::move(r), std::move(orig)});
        }
    }
    // ---- split pieces where another road shares one of their inner vertices (junctions inside a segment)
    {
        std::unordered_map<uint64_t, int> users;
        for (size_t k = 0; k < pieces.size(); k++) {
            std::set<uint64_t> mine;
            for (size_t oi : pieces[k].orig) mine.insert(qkey(pieces[k].r.pts[oi].p));
            for (uint64_t q : mine) users[q]++;
        }
        for (auto &pc : pieces) {
            size_t from = 0;
            for (size_t n = 1; n + 1 < pc.orig.size(); n++) {
                size_t oi = pc.orig[n];
                if (users[qkey(pc.r.pts[oi].p)] < 2) continue;
                Road part = pc.r;
                part.pts.assign(pc.r.pts.begin() + from, pc.r.pts.begin() + oi + 1);
                roads.push_back(std::move(part));
                from = oi;
            }
            Road last = pc.r;
            last.pts.assign(pc.r.pts.begin() + from, pc.r.pts.end());
            roads.push_back(std::move(last));
        }
    }
    // ---- chains: join pieces meeting end to end when nothing else meets there and they look alike
    {
        std::unordered_map<uint64_t, std::vector<std::pair<int, bool>>> ends;
        for (int i = 0; i < (int)roads.size(); i++) {
            ends[qkey(roads[i].pts.front().p)].push_back({i, true});
            ends[qkey(roads[i].pts.back().p)].push_back({i, false});
        }
        bool merged = true;
        while (merged) {
            merged = false;
            for (auto &[k, v] : ends) {
                if (v.size() != 2) continue;
                auto [a, as] = v[0];
                auto [b, bs] = v[1];
                if (a == b || !roads[a].alive || !roads[b].alive) continue;
                Road &A = roads[a], &B = roads[b];
                if (A.cls != B.cls || std::abs(A.hw - B.hw) > 0.3 || A.bridge != B.bridge || A.sw != B.sw || (A.oneway != 0) != (B.oneway != 0)) continue;
                // orient A to end here and B to start here
                if (as) {
                    std::reverse(A.pts.begin(), A.pts.end());
                    A.oneway = -A.oneway;
                }
                if (!bs) {
                    std::reverse(B.pts.begin(), B.pts.end());
                    B.oneway = -B.oneway;
                }
                if (A.oneway != 0 && A.oneway != B.oneway) {
                    // opposite one-way directions: keep apart (undo nothing harmful: orientation is free)
                    continue;
                }
                A.pts.insert(A.pts.end(), B.pts.begin() + 1, B.pts.end());
                B.alive = false;
                // re-point B's far end to A
                uint64_t fk = qkey(A.pts.back().p);
                for (auto &e : ends[fk])
                    if (e.first == b) e = {a, false};
                uint64_t sk = qkey(A.pts.front().p);
                for (auto &e : ends[sk])
                    if (e.first == a) e.second = true;
                v.clear();
                merged = true;
            }
        }
        std::vector<Road> keep;
        for (auto &r : roads)
            if (r.alive) keep.push_back(std::move(r));
        roads.swap(keep);
    }
    // ---- heights: natural ground; bridges arch between their ends
    for (auto &r : roads) {
        for (auto &q : r.pts) q.z = c.G(q.p);
        if (r.bridge) {
            double L = chain_len(r.pts), za = r.pts.front().z, zb = r.pts.back().z;
            // how high it must rise: over roads a lorry's clearance, over water a little, otherwise it follows
            // the ground between its ends (viaducts over valleys)
            auto seg_x = [](V2 a, V2 b, V2 c, V2 d) {
                double d1 = (b - a).cross(c - a), d2 = (b - a).cross(d - a), d3 = (d - c).cross(a - c), d4 = (d - c).cross(b - c);
                return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
            };
            bool over_road = false, over_water = false;
            for (auto &o : roads) {
                if (&o == &r || o.bridge || over_road) continue;
                for (size_t i = 0; i + 1 < r.pts.size() && !over_road; i += 2)
                    for (size_t k = 0; k + 1 < o.pts.size(); k++)
                        if (seg_x(r.pts[i].p, r.pts[std::min(i + 2, r.pts.size() - 1)].p, o.pts[k].p, o.pts[k + 1].p)) {
                            over_road = true;
                            break;
                        }
            }
            (void)over_water;
            double amp = over_road ? 6.0 * std::max(1, r.layer) : std::min(2.0, 0.02 * L);
            double R = std::min(L / 2, 45.0);
            double s = 0;
            for (size_t i = 0; i < r.pts.size(); i++) {
                if (i) s += (r.pts[i].p - r.pts[i - 1].p).len();
                auto sm = [](double x) { x = std::clamp(x, 0.0, 1.0); return x * x * (3 - 2 * x); };
                double ramp = R > 1e-6 ? sm(s / R) * sm((L - s) / R) : 0;
                double lin = za + (zb - za) * (L > 0 ? s / L : 0);
                // never below the ground under it plus clearance where it should pass over something
                r.pts[i].z = lin + amp * ramp;
            }
        }
    }
    // ---- nodes and junction geometry
    std::vector<Node> nodes;
    {
        std::unordered_map<uint64_t, int> idx;
        for (int i = 0; i < (int)roads.size(); i++)
            for (int e = 0; e < 2; e++) {
                Road &r = roads[i];
                if (r.pts.size() < 2) continue;
                V2 p = e == 0 ? r.pts.front().p : r.pts.back().p;
                uint64_t k = qkey(p);
                auto it = idx.find(k);
                int n;
                if (it == idx.end()) {
                    n = (int)nodes.size();
                    idx[k] = n;
                    nodes.push_back({p, {}, 0});
                } else n = it->second;
                V2 q = e == 0 ? r.pts[1].p : r.pts[r.pts.size() - 2].p;
                V2 u = (q - p).norm();
                nodes[n].ends.push_back({i, e == 0, u, std::atan2(u.y, u.x)});
                r.node[e] = n;
            }
    }
    struct Corner {
        std::vector<V2> inner, outer;  // kerb curve and sidewalk back curve
        std::vector<double> zi;
        bool walk = false;
    };
    std::vector<std::vector<Corner>> corners(nodes.size());
    for (size_t ni = 0; ni < nodes.size(); ni++) {
        Node &n = nodes[ni];
        if (n.ends.size() < 2) continue;
        std::sort(n.ends.begin(), n.ends.end(), [](const End &a, const End &b) { return a.ang < b.ang; });
        size_t k = n.ends.size();
        std::vector<double> trim(k, 0);
        for (size_t i = 0; i < k; i++) {
            const End &a = n.ends[i], &b = n.ends[(i + 1) % k];
            const Road &A = roads[a.road], &B = roads[b.road];
            double gap = b.ang - a.ang + (i + 1 == k ? 2 * PI : 0);
            double base_t = std::max(A.hw, B.hw) * 0.5 + 0.5;
            V2 la = n.p + a.u.perp() * A.hw, rb = n.p - b.u.perp() * B.hw;
            V2 x;
            if (gap < PI * 0.92 && line_x(la, a.u, rb, b.u, x)) {
                double r = A.urban ? 4.5 : 7;
                double ta = (x - n.p).dot(a.u) + r, tb = (x - n.p).dot(b.u) + r;
                trim[i] = std::max(trim[i], std::clamp(ta, base_t, 30.0));
                trim[(i + 1) % k] = std::max(trim[(i + 1) % k], std::clamp(tb, base_t, 30.0));
            } else {
                trim[i] = std::max(trim[i], base_t);
                trim[(i + 1) % k] = std::max(trim[(i + 1) % k], base_t);
            }
        }
        for (size_t i = 0; i < k; i++) {
            Road &r = roads[n.ends[i].road];
            double L = chain_len(r.pts);
            trim[i] = std::min(trim[i], L * 0.45);
            r.trim[n.ends[i].start ? 0 : 1] = trim[i];
        }
    }
    // node heights: mean of the road ends' heights at their trims
    for (auto &n : nodes) {
        double z = 0;
        for (auto &e : n.ends) {
            Road &r = roads[e.road];
            z += at_chainage(r.pts, r.trim[e.start ? 0 : 1], e.start).z;
        }
        n.z = n.ends.empty() ? c.G(n.p) : z / n.ends.size();
    }
    auto zoff = [](const Road &r) { return 0.03 + r.prio * 0.004; };

    // ---- flatten the terrain under carriageways and sidewalks (not under bridges)
    {
        HeightGrid &g = c.out.grid;
        int N = g.n + 1;
        std::vector<float> bw(N * N, 0), bt(N * N, 0), bd(N * N, 1e9);
        auto put = [&](int i, int j, double w, double t, double d) {
            size_t q = (size_t)j * N + i;
            if (w > bw[q] + 1e-6 || (std::abs(w - bw[q]) <= 1e-6 && d < bd[q])) bw[q] = (float)w, bt[q] = (float)t, bd[q] = (float)d;
        };
        const double FALL = 7;
        for (auto &r : roads) {
            if (r.bridge) continue;
            double core = r.hw + r.sw + 0.8;
            for (size_t i = 0; i + 1 < r.pts.size(); i++) {
                V2 a = r.pts[i].p, b = r.pts[i + 1].p;
                double R = core + FALL;
                int i0 = std::max(0, (int)std::floor((std::min(a.x, b.x) - R - g.x0) / g.cell)), i1 = std::min(N - 1, (int)std::ceil((std::max(a.x, b.x) + R - g.x0) / g.cell));
                int j0 = std::max(0, (int)std::floor((std::min(a.y, b.y) - R - g.y0) / g.cell)), j1 = std::min(N - 1, (int)std::ceil((std::max(a.y, b.y) + R - g.y0) / g.cell));
                V2 d = b - a;
                double l2 = std::max(1e-9, d.dot(d));
                for (int jj = j0; jj <= j1; jj++)
                    for (int ii = i0; ii <= i1; ii++) {
                        V2 p(g.x0 + ii * g.cell, g.y0 + jj * g.cell);
                        double t = std::clamp((p - a).dot(d) / l2, 0.0, 1.0);
                        double dist = (a + d * t - p).len();
                        if (dist > R) continue;
                        double w = dist <= core ? 1 : 1 - (dist - core) / FALL;
                        w = w * w * (3 - 2 * w);
                        put(ii, jj, w, r.pts[i].z + (r.pts[i + 1].z - r.pts[i].z) * t, dist);
                    }
            }
        }
        // junction discs
        for (auto &n : nodes) {
            if (n.ends.size() < 2) continue;
            bool any = false;
            double rad = 0;
            for (auto &e : n.ends) {
                const Road &r = roads[e.road];
                if (r.bridge) continue;
                any = true;
                rad = std::max(rad, r.trim[e.start ? 0 : 1] + r.hw);
            }
            if (!any) continue;
            double R = rad + FALL;
            int i0 = std::max(0, (int)std::floor((n.p.x - R - g.x0) / g.cell)), i1 = std::min(N - 1, (int)std::ceil((n.p.x + R - g.x0) / g.cell));
            int j0 = std::max(0, (int)std::floor((n.p.y - R - g.y0) / g.cell)), j1 = std::min(N - 1, (int)std::ceil((n.p.y + R - g.y0) / g.cell));
            for (int jj = j0; jj <= j1; jj++)
                for (int ii = i0; ii <= i1; ii++) {
                    V2 p(g.x0 + ii * g.cell, g.y0 + jj * g.cell);
                    double dist = (p - n.p).len();
                    if (dist > R) continue;
                    double w = dist <= rad ? 1 : 1 - (dist - rad) / FALL;
                    w = w * w * (3 - 2 * w);
                    // inside the junction the nearest road end decides; the disc only helps where roads leave gaps
                    put(ii, jj, w * 0.999, n.z, dist + 1000);
                }
        }
        for (int q = 0; q < N * N; q++)
            if (bw[q] > 0) g.h[q] = (float)(g.h[q] + (bt[q] - g.h[q]) * bw[q]);
    }

    // ---- index, splat, occupancy
    for (int i = 0; i < (int)roads.size(); i++) {
        Road &r = roads[i];
        for (size_t k = 0; k + 1 < r.pts.size(); k++) {
            V2 a = r.pts[k].p, b = r.pts[k + 1].p;
            c.roads.add(a, b, r.hw, i);
            if (r.bridge) continue;
            c.stroke(a, b, r.hw + 0.4, [&](int t) { c.layer[t] = L_ASPHALT, c.occ[t] |= O_ROAD; });
            if (r.sw > 0) c.stroke(a, b, r.hw + r.sw, [&](int t) {
                if (!(c.occ[t] & O_ROAD)) c.layer[t] = L_PAVERS, c.occ[t] |= O_WALK;
            });
        }
    }
    for (auto &n : nodes)
        if (n.ends.size() >= 2) {
            double rad = 0;
            for (auto &e : n.ends) rad = std::max(rad, roads[e.road].hw);
            c.stroke(n.p, n.p, rad + 1, [&](int t) { c.layer[t] = L_ASPHALT, c.occ[t] |= O_ROAD; });
        }

    // ---- meshes
    MeshBuf &AS = c.out.ms[M_ASPHALT], &MK = c.out.ms[M_MARKING], &CU = c.out.ms[M_CURB], &PV = c.out.ms[M_PAVERS], &CO = c.out.ms[M_CONCRETE];
    for (auto &r0 : roads) {
        double L = chain_len(r0.pts);
        if (L - r0.trim[0] - r0.trim[1] < 0.2) continue;
        std::vector<RPt> P = cut(r0.pts, r0.trim[0], L - r0.trim[1]);
        if (P.size() < 2) continue;
        c.out.roads++;
        // centre line through the junctions (untrimmed), so pieces meet at shared nodes
        for (size_t i = 0; i + 1 < r0.pts.size(); i++) {
            const RPt &a = r0.pts[i], &b = r0.pts[i + 1];
            if (!c.own((a.p + b.p) * 0.5)) continue;
            double zo2 = 0.03 + r0.prio * 0.004;
            c.out.lanes.push_back({V3(a.p.x, a.p.y, a.z + zo2), V3(b.p.x, b.p.y, b.z + zo2), r0.hw, r0.oneway, r0.prio, r0.lanes});
        }
        std::vector<V2> pp(P.size()), nn;
        std::vector<double> sc, s(P.size(), 0);
        for (size_t i = 0; i < P.size(); i++) pp[i] = P[i].p;
        normals(pp, nn, sc);
        for (size_t i = 1; i < P.size(); i++) s[i] = s[i - 1] + (pp[i] - pp[i - 1]).len();
        double zo = zoff(r0);
        Col ac = asphalt_col(r0);
        auto Q = [&](size_t i, double off, double dz) {
            V2 q = pp[i] + nn[i] * (off * sc[i]);
            return V3(q.x, q.y, P[i].z + dz);
        };
        // strip between offsets o0 < o1 (left positive) at heights dz0/dz1, for owned segments
        auto strip = [&](MeshBuf &m, double o0, double o1, double dz0, double dz1, Col col, bool coll, double uscale, bool only_dash = false) {
            for (size_t i = 0; i + 1 < P.size(); i++) {
                if (!c.own((pp[i] + pp[i + 1]) * 0.5)) continue;
                if (only_dash && !P[i].dash) continue;
                V3 a = Q(i, o0, dz0), b = Q(i + 1, o0, dz0), d = Q(i + 1, o1, dz1), e = Q(i, o1, dz1);
                V3 n = (b - a).cross(e - a).norm();
                if (n.z < -0.5 || (std::abs(n.z) < 0.5 && false)) n = n * -1;
                int ia = m.v(a, n, o0 / uscale, s[i] / uscale, col), ib = m.v(b, n, o0 / uscale, s[i + 1] / uscale, col);
                int id = m.v(d, n, o1 / uscale, s[i + 1] / uscale, col), ie = m.v(e, n, o1 / uscale, s[i] / uscale, col);
                m.tri(ia, ib, id);
                m.tri(ia, id, ie);
                if (coll) {
                    c.tri_coll(a, b, d);
                    c.tri_coll(a, d, e);
                }
            }
        };
        // vertical face along offset o from dz0 up to dz1, facing +side (1 = left, -1 = right)
        auto face = [&](MeshBuf &m, double o, double dz0, double dz1, int side, Col col, bool coll) {
            for (size_t i = 0; i + 1 < P.size(); i++) {
                if (!c.own((pp[i] + pp[i + 1]) * 0.5)) continue;
                V3 a = Q(i, o, dz0), b = Q(i + 1, o, dz0), d = Q(i + 1, o, dz1), e = Q(i, o, dz1);
                // (a, b, d, e) faces right; mirror for a left-facing face
                if (side > 0) std::swap(a, b), std::swap(d, e);
                double s0 = side > 0 ? s[i + 1] : s[i], s1 = side > 0 ? s[i] : s[i + 1];
                m.quadn(a, b, d, e, col, s0 / 2, 0, s1 / 2, dz1 - dz0);
                if (coll) {
                    c.tri_coll(a, b, d);
                    c.tri_coll(a, d, e);
                }
            }
        };
        if (!r0.bridge) {
            strip(AS, -r0.hw, r0.hw, zo, zo, ac, true, 4);
        } else {
            // deck: wider by the parapets, slab under it, parapet walls
            double dw = r0.hw + 0.9;
            strip(AS, -r0.hw, r0.hw, zo, zo, ac, true, 4);
            strip(CO, r0.hw, dw, zo + 0.15, zo + 0.15, Col{0.75f, 0.74f, 0.72f, 1}, true, 2);
            strip(CO, -dw, -r0.hw, zo + 0.15, zo + 0.15, Col{0.75f, 0.74f, 0.72f, 1}, true, 2);
            face(CU, r0.hw, zo, zo + 0.15, -1, Col{0.8f, 0.8f, 0.78f, 1}, true);
            face(CU, -r0.hw, zo, zo + 0.15, 1, Col{0.8f, 0.8f, 0.78f, 1}, true);
            // parapets
            face(CO, dw - 0.25, zo + 0.15, zo + 1.1, -1, Col{0.78f, 0.77f, 0.75f, 1}, true);
            face(CO, dw, zo - 1.2, zo + 1.1, 1, Col{0.7f, 0.69f, 0.67f, 1}, true);
            face(CO, -dw + 0.25, zo + 0.15, zo + 1.1, 1, Col{0.78f, 0.77f, 0.75f, 1}, true);
            face(CO, -dw, zo - 1.2, zo + 1.1, -1, Col{0.7f, 0.69f, 0.67f, 1}, true);
            strip(CO, dw - 0.25, dw, zo + 1.1, zo + 1.1, Col{0.82f, 0.81f, 0.8f, 1}, false, 2);
            strip(CO, -dw, -dw + 0.25, zo + 1.1, zo + 1.1, Col{0.82f, 0.81f, 0.8f, 1}, false, 2);
            // underside
            for (size_t i = 0; i + 1 < P.size(); i++) {
                if (!c.own((pp[i] + pp[i + 1]) * 0.5)) continue;
                V3 a = Q(i, dw, -1.2), b = Q(i + 1, dw, -1.2), d = Q(i + 1, -dw, -1.2), e = Q(i, -dw, -1.2);
                CO.quadn(a, b, d, e, Col{0.6f, 0.6f, 0.58f, 1});
            }
            // piers where the deck is high above the ground
            double next = 12;
            for (size_t i = 0; i + 1 < P.size(); i++) {
                if (s[i] < next) continue;
                next = s[i] + 24;
                if (!c.own(pp[i])) continue;
                double gz = c.G(pp[i]);
                if (P[i].z - gz < 3.5) continue;
                double yaw = std::atan2(nn[i].y, nn[i].x);
                CO.obox(pp[i], yaw, dw * 1.4, 1.2, gz - 3, P[i].z - 1.2, Col{0.66f, 0.65f, 0.63f, 1});
                V2 ctr = pp[i];
                double cs = std::cos(yaw), sn = std::sin(yaw), hl = dw * 0.7, hw2 = 0.6;
                V3 q[4];
                for (int k = 0; k < 4; k++) {
                    double a = (k == 0 || k == 3) ? -hl : hl, b = k < 2 ? -hw2 : hw2;
                    q[k] = V3(ctr.x + a * cs - b * sn, ctr.y + a * sn + b * cs, 0);
                }
                for (int k = 0; k < 4; k++) {
                    V3 a = q[k], b = q[(k + 1) % 4];
                    V3 a0(a.x, a.y, gz - 3), b0(b.x, b.y, gz - 3), a1(a.x, a.y, P[i].z - 1.2), b1(b.x, b.y, P[i].z - 1.2);
                    c.tri_coll(a0, b0, b1);
                    c.tri_coll(a0, b1, a1);
                }
            }
        }
        // sidewalks: kerb face, kerb stone, paving, back face down into the ground
        if (r0.sw > 0) {
            Col kc{0.78f, 0.78f, 0.76f, 1}, pc{1, 1, 1, 1};
            for (int side : {1, -1}) {
                double o0 = side * r0.hw, o1 = side * (r0.hw + 0.18), o2 = side * (r0.hw + r0.sw);
                face(CU, o0, zo - 0.02, 0.15 + zo, side > 0 ? -1 : 1, kc, true);
                if (side > 0) {
                    strip(CU, o0, o1, 0.15 + zo, 0.15 + zo, kc, true, 1);
                    strip(PV, o1, o2, 0.15 + zo, 0.15 + zo, pc, true, 3);
                } else {
                    strip(CU, o1, o0, 0.15 + zo, 0.15 + zo, kc, true, 1);
                    strip(PV, o2, o1, 0.15 + zo, 0.15 + zo, pc, true, 3);
                }
                face(CU, o2, -0.4, 0.15 + zo, side > 0 ? 1 : -1, kc, false);
            }
        }
        // markings
        if (!(r0.cls == "service" && r0.hw < 3)) {
            Col w{0.95f, 0.95f, 0.93f, 1};
            double mz = zo + 0.008;
            bool two_way = r0.oneway == 0;
            bool major = r0.prio >= 5;
            if (two_way && r0.hw * 2 >= 5.4) {
                if (r0.lanes >= 4) {
                    strip(MK, -0.2, -0.08, mz, mz, w, false, 1);
                    strip(MK, 0.08, 0.2, mz, mz, w, false, 1);
                } else
                    strip(MK, -0.06, 0.06, mz, mz, w, false, 1, true);
                // lane lines on wide roads
                int per = r0.lanes / 2;
                for (int k = 1; k < per; k++) {
                    double o = r0.hw * k / per;
                    strip(MK, o - 0.06, o + 0.06, mz, mz, w, false, 1, true);
                    strip(MK, -o - 0.06, -o + 0.06, mz, mz, w, false, 1, true);
                }
            } else if (!two_way && r0.lanes >= 2) {
                for (int k = 1; k < r0.lanes; k++) {
                    double o = -r0.hw + 2 * r0.hw * k / r0.lanes;
                    strip(MK, o - 0.06, o + 0.06, mz, mz, w, false, 1, true);
                }
            }
            if (major || r0.prio == 4) {
                strip(MK, r0.hw - 0.4, r0.hw - 0.25, mz, mz, w, false, 1);
                strip(MK, -r0.hw + 0.25, -r0.hw + 0.4, mz, mz, w, false, 1);
            }
        }
        // zebra crossings at urban junction ends
        if (r0.urban && r0.prio >= 2 && r0.prio <= 7 && !r0.bridge) {
            for (int e = 0; e < 2; e++) {
                int n = r0.node[e];
                if (n < 0 || nodes[n].ends.size() < 3) continue;
                size_t i = e == 0 ? 0 : P.size() - 1;
                if (!c.own(pp[i])) continue;
                V2 dir = e == 0 ? (pp[1] - pp[0]).norm() : (pp[P.size() - 2] - pp[P.size() - 1]).norm();
                V2 side = dir.perp();
                double z = P[i].z + zo + 0.008;
                for (double o = -r0.hw + 0.5; o + 0.5 <= r0.hw - 0.3; o += 1.0) {
                    V2 a = pp[i] + dir * 0.6 + side * o;
                    V3 p0(a.x, a.y, z), p1 = V3(a.x + side.x * 0.5, a.y + side.y * 0.5, z);
                    V3 p2 = p1 + V3(dir.x * 3, dir.y * 3, 0), p3 = p0 + V3(dir.x * 3, dir.y * 3, 0);
                    MK.quadn(p0, p1, p2, p3, Col{0.95f, 0.95f, 0.93f, 1});
                }
            }
        }
        // street lamps along urban roads with sidewalks, every 32 m, alternate sides on wide roads
        if (r0.sw > 0 && r0.prio >= 2) {
            double first = std::fmod(std::abs(pp[0].x * 0.37 + pp[0].y * 0.61), 32.0);
            int k = 0;
            for (double sp = first + 8; sp < s.back() - 4; sp += 32, k++) {
                size_t i = 0;
                while (i + 1 < s.size() && s[i + 1] < sp) i++;
                double t = (sp - s[i]) / std::max(1e-6, s[i + 1] - s[i]);
                V2 cp = pp[i] + (pp[i + 1] - pp[i]) * t;
                V2 nrm = (nn[i] + (nn[i + 1] - nn[i]) * t).norm();
                int side = (r0.hw > 5 && k % 2) ? -1 : 1;
                V2 lp = cp + nrm * (side * (r0.hw + r0.sw - 0.6));
                if (!c.own(lp)) continue;
                if (c.roads.clearance(lp) < r0.sw - 1.0) continue;  // another carriageway
                bool inside = false;
                for (auto &br : c.building_rings)
                    if (ring_dist(lp, br) < 1.0 || point_in_ring(lp, br)) {
                        inside = true;
                        break;
                    }
                if (inside) continue;
                double z = P[i].z + (P[i + 1].z - P[i].z) * t + zo + 0.15;
                V2 toward = nrm * (-side);
                c.out.inst.push_back({I_LAMP, {lp.x, lp.y, z}, std::atan2(toward.y, toward.x), 1, {1, 1, 1, 1}});
            }
        }
    }
    // junction surfaces with rounded kerbs, sidewalk corners
    for (size_t ni = 0; ni < nodes.size(); ni++) {
        Node &n = nodes[ni];
        size_t k = n.ends.size();
        if (k < 2 || !c.own(n.p)) continue;
        bool bridge = false;
        int pr = 0;
        for (auto &e : n.ends) bridge |= roads[e.road].bridge, pr = std::max(pr, roads[e.road].prio);
        std::vector<V2> poly;
        std::vector<double> pz;
        struct EP {
            V2 c, l, r;
            double z, hw, sw;
            V2 u;
        };
        std::vector<EP> ep;
        for (auto &e : n.ends) {
            Road &r = roads[e.road];
            double t = r.trim[e.start ? 0 : 1];
            RPt q = at_chainage(r.pts, t, e.start);
            RPt q2 = at_chainage(r.pts, t + 0.5, e.start);
            V2 u = (q2.p - q.p).norm();
            if (u.len() < 0.5) u = e.u;
            ep.push_back({q.p, q.p + u.perp() * r.hw, q.p - u.perp() * r.hw, q.z, r.hw, r.sw, u});
        }
        double zo = 0.03 + pr * 0.004 + 0.002;
        for (size_t i = 0; i < k; i++) {
            const EP &a = ep[i], &b = ep[(i + 1) % k];
            poly.push_back(a.r), pz.push_back(a.z);
            poly.push_back(a.l), pz.push_back(a.z);
            double gap = n.ends[(i + 1) % k].ang - n.ends[i].ang + (i + 1 == k ? 2 * PI : 0);
            V2 x;
            std::vector<V2> curve;
            if (gap < PI * 0.92 && line_x(a.l, a.u, b.r, b.u, x) && (x - n.p).len() < 40) curve = bezier(a.l, x, b.r, 8);
            else curve = {a.l, b.r};
            for (size_t j = 1; j + 1 < curve.size(); j++) {
                poly.push_back(curve[j]);
                pz.push_back(a.z + (b.z - a.z) * double(j) / (curve.size() - 1));
            }
            // sidewalk corner between the two kerbs
            if (a.sw > 0 && b.sw > 0 && curve.size() > 2) {
                V2 ao = a.l + a.u.perp() * a.sw, bo = b.r - b.u.perp() * b.sw, xo;
                if (line_x(ao, a.u, bo, b.u, xo) && (xo - n.p).len() < 50) {
                    std::vector<V2> outer = bezier(ao, xo, bo, 8);
                    std::vector<V2> inner = bezier(a.l, x, b.r, 8);
                    Col kc{0.78f, 0.78f, 0.76f, 1};
                    for (size_t j = 0; j + 1 < inner.size(); j++) {
                        double z0 = a.z + (b.z - a.z) * double(j) / 8 + zo + 0.15, z1 = a.z + (b.z - a.z) * double(j + 1) / 8 + zo + 0.15;
                        V3 i0(inner[j].x, inner[j].y, z0), i1(inner[j + 1].x, inner[j + 1].y, z1), o0(outer[j].x, outer[j].y, z0), o1(outer[j + 1].x, outer[j + 1].y, z1);
                        // paving (inner->outer is to the outside of the junction)
                        V3 nrm(0, 0, 1);
                        int a0 = PV.v(i0, nrm, i0.x / 3, i0.y / 3, {1, 1, 1, 1}), a1 = PV.v(i1, nrm, i1.x / 3, i1.y / 3, {1, 1, 1, 1});
                        int b1 = PV.v(o1, nrm, o1.x / 3, o1.y / 3, {1, 1, 1, 1}), b0 = PV.v(o0, nrm, o0.x / 3, o0.y / 3, {1, 1, 1, 1});
                        if ((i1 - i0).cross(o0 - i0).z > 0) {
                            PV.tri(a0, a1, b1);
                            PV.tri(a0, b1, b0);
                        } else {
                            PV.tri(a0, b1, a1);
                            PV.tri(a0, b0, b1);
                        }
                        c.tri_coll(i0, i1, o1);
                        c.tri_coll(i0, o1, o0);
                        // kerb face toward the junction
                        V3 d0(i0.x, i0.y, z0 - 0.15), d1(i1.x, i1.y, z1 - 0.15);
                        CU.quadn(d1, d0, i0, i1, kc);
                        c.tri_coll(d0, d1, i1);
                        c.tri_coll(d0, i1, i0);
                        // back face
                        V3 e0(o0.x, o0.y, z0 - 0.55), e1(o1.x, o1.y, z1 - 0.55);
                        CU.quadn(e0, e1, o1, o0, kc);
                    }
                }
            }
        }
        // fan from the centre
        double zc = 0;
        for (auto &e : ep) zc += e.z;
        zc /= ep.size();
        V3 C(n.p.x, n.p.y, zc + zo);
        int ci = AS.v(C, {0, 0, 1}, C.x / 4, C.y / 4, {0.95f, 0.95f, 0.95f, 1});
        std::vector<int> vi;
        for (size_t i = 0; i < poly.size(); i++) vi.push_back(AS.v({poly[i].x, poly[i].y, pz[i] + zo}, {0, 0, 1}, poly[i].x / 4, poly[i].y / 4, {0.95f, 0.95f, 0.95f, 1}));
        for (size_t i = 0; i < poly.size(); i++) {
            size_t j = (i + 1) % poly.size();
            V2 a = poly[i] - n.p, b = poly[j] - n.p;
            if (a.cross(b) <= 0) continue;
            AS.tri(ci, vi[i], vi[j]);
            c.tri_coll(C, {poly[i].x, poly[i].y, pz[i] + zo}, {poly[j].x, poly[j].y, pz[j] + zo});
        }
        (void)bridge;
    }
}

}  // namespace ec
