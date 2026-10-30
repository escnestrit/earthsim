// Road network of a chunk: pieces joined into chains, junctions with rounded kerbs, carriageways that sit on
// terrain flattened under them, raised sidewalks with kerbs, markings and zebra crossings, bridges with
// parapets and piers, street lamps.
#include <array>
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
    double sws[2] = {0, 0};  // per side (left, right): out to the facades where buildings line the street
    bool bridge = false, urban = false, tunnel = false;
    int oneway = 0, layer = 0, prio = 0;
    int lanes = 2;
    std::string name;
    int speed = 0;
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

Col mul_col(Col c, float k) { return {c.r * k, c.g * k, c.b * k, c.a}; }

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
            r.tunnel = rr.tunnel && !rr.bridge;
            r.oneway = rr.oneway;
            r.name = rr.name;
            r.speed = rr.speed;
            r.layer = rr.layer;
            r.prio = prio_of(rr.cls);
            r.urban = urban;
            r.lanes = std::max(1, (int)std::lround(r.hw * 2 / 3.3));
            if (urban && wants_sidewalk(rr.cls) && !rr.bridge && !rr.tunnel) r.sw = r.prio >= 6 ? 4.5 : r.prio >= 5 ? 3.5 : 3.0;
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
        // main roads are graded flatter (they were built that way, and drive better for it)
        for (auto &q : r.pts) q.z = c.Gs(q.p, r.prio >= 6 ? 1.7 : 1.0);
        if (r.bridge) {
            double L = chain_len(r.pts), za = r.pts.front().z, zb = r.pts.back().z;
            // how high it must rise: over roads a lorry's clearance, over water a little, otherwise it follows
            // the ground between its ends (viaducts over valleys)
            auto seg_x = [](V2 a, V2 b, V2 c, V2 d) {
                double d1 = (b - a).cross(c - a), d2 = (b - a).cross(d - a), d3 = (d - c).cross(a - c), d4 = (d - c).cross(b - c);
                return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
            };
            // where it passes over another road it must clear a lorry (5 m over that road's surface): the rise
            // is just enough for that, not a fixed hump
            double R = std::min(L / 2, 45.0);
            auto sm = [](double x) { x = std::clamp(x, 0.0, 1.0); return x * x * (3 - 2 * x); };
            std::vector<double> cs(r.pts.size(), 0);
            for (size_t i = 1; i < r.pts.size(); i++) cs[i] = cs[i - 1] + (r.pts[i].p - r.pts[i - 1].p).len();
            double amp = std::min(1.0, 0.01 * L);  // otherwise (water, valleys): a gentle camber
            for (auto &o : roads) {
                if (&o == &r || o.bridge) continue;
                for (size_t i = 0; i + 1 < r.pts.size(); i++)
                    for (size_t k = 0; k + 1 < o.pts.size(); k++)
                        if (seg_x(r.pts[i].p, r.pts[i + 1].p, o.pts[k].p, o.pts[k + 1].p)) {
                            double sx = (cs[i] + cs[i + 1]) / 2;
                            double ramp = R > 1e-6 ? sm(sx / R) * sm((L - sx) / R) : 0;
                            double lin = za + (zb - za) * (L > 0 ? sx / L : 0);
                            double need = c.Gs((r.pts[i].p + r.pts[i + 1].p) * 0.5) + 5.3 - lin;
                            if (ramp > 0.15 && need > 0) amp = std::max(amp, std::min(9.0, need / ramp));
                        }
            }
            double s = 0;
            for (size_t i = 0; i < r.pts.size(); i++) {
                if (i) s += (r.pts[i].p - r.pts[i - 1].p).len();
                double ramp = R > 1e-6 ? sm(s / R) * sm((L - s) / R) : 0;
                double lin = za + (zb - za) * (L > 0 ? s / L : 0);
                // never below the ground under it plus clearance where it should pass over something
                r.pts[i].z = lin + amp * ramp;
            }
        }
        if (r.tunnel) {
            // down under whatever is above: up to 6.5 m below the line between the portals, eased in over 50 m
            double L = chain_len(r.pts), za = r.pts.front().z, zb = r.pts.back().z;
            double R = std::min(L / 2, 50.0), s = 0;
            // short "tunnels" are passages through buildings and the like: they stay level
            double depth = L < 70 ? 0.0 : std::min(6.5, 3.0 + (L - 70) * 0.1);
            auto sm = [](double x) { x = std::clamp(x, 0.0, 1.0); return x * x * (3 - 2 * x); };
            for (size_t i = 0; i < r.pts.size(); i++) {
                if (i) s += (r.pts[i].p - r.pts[i - 1].p).len();
                double ramp = R > 1e-6 ? sm(s / R) * sm((L - s) / R) : 0;
                r.pts[i].z = za + (zb - za) * (L > 0 ? s / L : 0) - depth * ramp;
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
    // node heights: the road ends' heights at their trims, the bigger roads weighing most; then each road is
    // eased to its junctions' heights over its last 30 m, so junction surfaces are flat and meet the arms
    for (auto &n : nodes) {
        double z = 0, wsum = 0;
        for (auto &e : n.ends) {
            Road &r = roads[e.road];
            double w = 1 + r.prio * r.prio;
            z += w * at_chainage(r.pts, r.trim[e.start ? 0 : 1], e.start).z, wsum += w;
        }
        n.z = n.ends.empty() ? c.G(n.p) : z / wsum;
    }
    for (auto &r : roads) {
        if (r.pts.size() < 2) continue;
        double L = chain_len(r.pts);
        std::vector<double> s(r.pts.size(), 0);
        for (size_t i = 1; i < r.pts.size(); i++) s[i] = s[i - 1] + (r.pts[i].p - r.pts[i - 1].p).len();
        double d[2] = {0, 0};
        for (int e = 0; e < 2; e++) {
            int nd = r.node[e];
            if (nd < 0 || nodes[nd].ends.size() < 2) continue;
            d[e] = nodes[nd].z - at_chainage(r.pts, r.trim[e], e == 0).z;
        }
        if (d[0] == 0 && d[1] == 0) continue;
        double R = std::min(30.0, L * 0.5);
        for (size_t i = 0; i < r.pts.size(); i++) {
            auto sm = [](double x) { x = std::clamp(x, 0.0, 1.0); return 1 - x * x * (3 - 2 * x); };
            double w0 = sm((s[i] - r.trim[0]) / R), w1 = sm((L - r.trim[1] - s[i]) / R);
            if (s[i] < r.trim[0]) w0 = 1;
            if (L - s[i] < r.trim[1]) w1 = 1;
            r.pts[i].z += d[0] * w0 + d[1] * w1;
        }
    }
    auto zoff = [](const Road &r) { return 0.03 + r.prio * 0.004; };

    // ---- sidewalks out to the building line: in town the pavement usually runs from the kerb to the facades
    {
        struct RB {
            const Ring *r;
            V2 lo, hi;
        };
        std::vector<RB> rbs;
        for (auto &br : c.building_rings) {
            V2 lo(1e300, 1e300), hi(-1e300, -1e300);
            for (auto &p : br) lo = V2(std::min(lo.x, p.x), std::min(lo.y, p.y)), hi = V2(std::max(hi.x, p.x), std::max(hi.y, p.y));
            rbs.push_back({&br, lo, hi});
        }
        for (auto &r : roads) {
            r.sws[0] = r.sws[1] = r.sw;
            if (r.sw <= 0 || r.bridge || r.pts.size() < 2) continue;
            for (int sd = 0; sd < 2; sd++) {
                double side = sd == 0 ? 1 : -1;
                std::vector<double> fac;
                int n = 0;
                for (size_t i = 0; i + 1 < r.pts.size(); i += 2) {
                    V2 a = r.pts[i].p, b = r.pts[i + 1].p, d = (b - a).norm(), nrm = d.perp() * side;
                    V2 q = (a + b) * 0.5;
                    n++;
                    // walk out along the normal to the first wall within 18 m of the kerb
                    double best = 1e9;
                    for (auto &rb : rbs) {
                        double reach = r.hw + 18;
                        if (q.x + reach < rb.lo.x && q.x + nrm.x * reach < rb.lo.x) continue;
                        if (q.x - reach > rb.hi.x && q.x + nrm.x * reach > rb.hi.x) continue;
                        if (q.y + reach < rb.lo.y && q.y + nrm.y * reach < rb.lo.y) continue;
                        if (q.y - reach > rb.hi.y && q.y + nrm.y * reach > rb.hi.y) continue;
                        const Ring &ring = *rb.r;
                        for (size_t k = 0; k < ring.size(); k++) {
                            V2 x;
                            if (line_x(q, nrm, ring[k], ring[(k + 1) % ring.size()] - ring[k], x)) {
                                double t = (x - q).dot(nrm);
                                V2 e = ring[(k + 1) % ring.size()] - ring[k];
                                double u = (x - ring[k]).dot(e) / std::max(1e-9, e.dot(e));
                                if (t > r.hw && t < best && u >= 0 && u <= 1) best = t;
                            }
                        }
                    }
                    if (best < r.hw + 18) fac.push_back(best - r.hw);
                }
                // the street is lined with buildings on this side: pavement to the median facade line
                if (n > 0 && (int)fac.size() * 2 >= n) {
                    std::nth_element(fac.begin(), fac.begin() + fac.size() / 3, fac.end());
                    double w = fac[fac.size() / 3] - 0.15;  // the nearer third: most facades are reached
                    if (w > r.sw) r.sws[sd] = std::min(w, 16.0);
                    else if (w > 1.5) r.sws[sd] = w;  // a narrow street: as wide as there is room
                }
            }
            r.sw = std::max(r.sws[0], r.sws[1]);
        }
    }
    // ---- flatten the terrain under carriageways and sidewalks (not under bridges)
    {
        HeightGrid &g = c.out.grid;
        int N = g.n + 1;
        std::vector<float> bw(N * N, 0), bt(N * N, 0), bd(N * N, 1e9);
        auto put = [&](int i, int j, double w, double t, double d) {
            size_t q = (size_t)j * N + i;
            if (w > bw[q] + 1e-6 || (std::abs(w - bw[q]) <= 1e-6 && d < bd[q])) bw[q] = (float)w, bt[q] = (float)t, bd[q] = (float)d;
        };
        const double FALL_ROAD = 7, FALL = FALL_ROAD;
        for (auto &r : roads) {
            if (r.bridge) continue;
            double core = r.tunnel ? r.hw + 0.4 : r.hw + r.sw + 0.8;
            const double FALL = r.tunnel ? 0.6 : FALL_ROAD;
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

    auto free_spot = [&](V2 p, double clear) {
        for (auto &br : c.building_rings)
            if (ring_dist(p, br) < clear || point_in_ring(p, br)) return false;
        return true;
    };
    // ---- dual carriageways: a side of a one-way road that runs alongside another road a few metres away gets
    // a central reservation (kerbed strip, guard rail on fast roads) instead of a sidewalk
    std::vector<std::array<double, 2>> med(roads.size(), {-1.0, -1.0});
    {
        std::vector<std::pair<V2, V2>> bb(roads.size());
        for (size_t i = 0; i < roads.size(); i++) {
            V2 lo(1e300, 1e300), hi(-1e300, -1e300);
            for (auto &q : roads[i].pts) lo = V2(std::min(lo.x, q.p.x), std::min(lo.y, q.p.y)), hi = V2(std::max(hi.x, q.p.x), std::max(hi.y, q.p.y));
            bb[i] = {lo, hi};
        }
        for (size_t ri = 0; ri < roads.size(); ri++) {
            const Road &r = roads[ri];
            if (!r.alive || r.oneway == 0 || r.pts.size() < 2 || r.prio < 3) continue;
            for (int sd = 0; sd < 2; sd++) {
                double side = sd == 0 ? 1 : -1;
                int n = 0, hit = 0;
                std::vector<double> gaps;
                for (size_t i = 0; i + 1 < r.pts.size(); i += 2) {
                    V2 a = r.pts[i].p, b = r.pts[i + 1].p, d = (b - a).norm();
                    V2 q = (a + b) * 0.5 + d.perp() * (side * (r.hw + 0.2));
                    n++;
                    double best = 1e9;
                    for (size_t oi = 0; oi < roads.size(); oi++) {
                        if (oi == ri || !roads[oi].alive) continue;
                        const Road &o = roads[oi];
                        // only the other carriageway of the same road: one-way the other way, same class
                        if (o.oneway == 0 || std::abs(o.prio - r.prio) > 1) continue;
                        if (q.x < bb[oi].first.x - 20 || q.x > bb[oi].second.x + 20 || q.y < bb[oi].first.y - 20 || q.y > bb[oi].second.y + 20) continue;
                        for (size_t k = 0; k + 1 < o.pts.size(); k++) {
                            V2 e = o.pts[k + 1].p - o.pts[k].p;
                            double el = e.len();
                            if (el < 1e-6 || std::abs(d.cross(e * (1 / el))) > 0.35) continue;  // only parallel pieces
                            // travelling the opposite way
                            if (d.dot(e * (1 / el)) * r.oneway * o.oneway > -0.5) continue;
                            double dist = seg_dist(q, o.pts[k].p, o.pts[k + 1].p) - o.hw;
                            // on our side of the road
                            V2 mid = (o.pts[k].p + o.pts[k + 1].p) * 0.5;
                            if ((mid - a).dot(d.perp()) * side <= 0) continue;
                            best = std::min(best, dist);
                        }
                    }
                    if (best < 14) hit++, gaps.push_back(std::max(0.0, best));
                }
                if (n > 0 && hit * 2 > n) {
                    std::nth_element(gaps.begin(), gaps.begin() + gaps.size() / 2, gaps.end());
                    med[ri][sd] = gaps[gaps.size() / 2] + 0.2;
                }
            }
        }
    }

    // ---- meshes
    MeshBuf &AS = c.out.ms[M_ASPHALT], &MK = c.out.ms[M_MARKING], &CU = c.out.ms[M_CURB], &PV = c.out.ms[M_PAVERS], &CO = c.out.ms[M_CONCRETE];
    for (size_t r0i = 0; r0i < roads.size(); r0i++) {
        Road &r0 = roads[r0i];
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
        if (r0.tunnel) {
            strip(AS, -r0.hw, r0.hw, zo, zo, ac, true, 4);
            // retaining walls up to the natural ground; a concrete roof where it is deep enough to be covered
            Col wc{0.66f, 0.65f, 0.62f, 1}, rc{0.6f, 0.6f, 0.58f, 1};
            for (size_t i = 0; i + 1 < P.size(); i++) {
                if (!c.own((pp[i] + pp[i + 1]) * 0.5)) continue;
                double g0 = c.Gs(pp[i]), g1 = c.Gs(pp[i + 1]);
                double d0 = g0 - P[i].z, d1 = g1 - P[i + 1].z;
                if (d0 < 0.3 && d1 < 0.3) continue;
                for (int side : {1, -1}) {
                    double o = side * (r0.hw + 0.3);
                    V3 a = Q(i, o, 0), b = Q(i + 1, o, 0);
                    V3 a1(a.x, a.y, std::max(a.z, g0 + 0.3)), b1(b.x, b.y, std::max(b.z, g1 + 0.3));
                    if (side > 0) CO.quadn(b, a, a1, b1, wc), c.tri_coll(b, a, a1), c.tri_coll(b, a1, b1);
                    else CO.quadn(a, b, b1, a1, wc), c.tri_coll(a, b, b1), c.tri_coll(a, b1, a1);
                    // the wall's cap
                    V3 o0 = Q(i, o + side * 0.4, 0), o1 = Q(i + 1, o + side * 0.4, 0);
                    V3 c0(o0.x, o0.y, a1.z), c1(o1.x, o1.y, b1.z);
                    if (side > 0) CO.quadn(a1, b1, c1, c0, mul_col(wc, 1.05f));
                    else CO.quadn(b1, a1, c0, c1, mul_col(wc, 1.05f));
                }
                if (d0 > 5.0 && d1 > 5.0) {
                    double w = r0.hw + 0.7;
                    V3 l0 = Q(i, w, 0), l1 = Q(i + 1, w, 0), r0p = Q(i, -w, 0), r1 = Q(i + 1, -w, 0);
                    V3 t0(l0.x, l0.y, g0 + 0.3), t1(l1.x, l1.y, g1 + 0.3), u0(r0p.x, r0p.y, g0 + 0.3), u1(r1.x, r1.y, g1 + 0.3);
                    // top (walkable/drivable) and the dark ceiling below
                    CO.quadn(u0, u1, t1, t0, rc);
                    c.tri_coll(u0, u1, t1);
                    c.tri_coll(u0, t1, t0);
                    V3 dn(0, 0, -0.5);
                    c.out.ms[M_DARK].quadn(t0 + dn, t1 + dn, u1 + dn, u0 + dn, Col{1, 1, 1, 1});
                }
            }
        } else if (!r0.bridge) {
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
        // central reservation on median sides: kerb, a strip out to the middle of the gap, guard rail on fast roads
        if (!r0.bridge && !r0.tunnel)
            for (int side : {1, -1}) {
                double gap = med[r0i][side > 0 ? 0 : 1];
                if (gap < 0) continue;
                Col kc{0.78f, 0.78f, 0.76f, 1};
                double half = std::max(0.25, gap / 2 - 0.02);
                double o0 = side * r0.hw, o1 = side * (r0.hw + half);
                if ((r0.prio >= 7 && gap < 3.0) || gap < 1.2) {
                    // fast roads and tight gaps: a concrete barrier (New Jersey profile, roughly), solid in collision
                    double bw = std::min(half, 0.3);
                    double ob = side * (r0.hw + half - bw);
                    strip(CO, side > 0 ? o0 : ob, side > 0 ? ob : o0, zo, zo, Col{0.55f, 0.55f, 0.53f, 1}, true, 3);
                    face(CO, ob, zo - 0.05, zo + 0.25, side > 0 ? -1 : 1, Col{0.74f, 0.73f, 0.7f, 1}, true);
                    double ot = side * (r0.hw + half - bw * 0.35);
                    // sloped lower face up to the narrow top
                    for (size_t i = 0; i + 1 < P.size(); i++) {
                        if (!c.own((pp[i] + pp[i + 1]) * 0.5)) continue;
                        V3 a = Q(i, ob, zo + 0.25), b = Q(i + 1, ob, zo + 0.25), d = Q(i + 1, ot, zo + 0.85), e = Q(i, ot, zo + 0.85);
                        if (side < 0) std::swap(a, b), std::swap(d, e);
                        CO.quadn(a, b, d, e, Col{0.74f, 0.73f, 0.7f, 1});
                        c.tri_coll(a, b, d);
                        c.tri_coll(a, d, e);
                    }
                    strip(CO, side > 0 ? ot : o1, side > 0 ? o1 : ot, zo + 0.85, zo + 0.85, Col{0.78f, 0.77f, 0.74f, 1}, true, 2);
                } else {
                    // kerbed island: grass-coloured top on wide medians, paving on narrow ones
                    face(CU, o0, zo - 0.02, zo + 0.15, side > 0 ? -1 : 1, kc, true);
                    MeshBuf &top = gap > 3 ? c.out.ms[M_GRASS] : CU;
                    Col tc = gap > 3 ? Col{0.55f, 0.62f, 0.4f, 1} : kc;
                    strip(top, side > 0 ? o0 : o1, side > 0 ? o1 : o0, zo + 0.15, zo + 0.15, tc, true, 2);
                    // a lane-side rail on dual carriageways of fast roads with wide medians
                    if (r0.prio >= 6 && gap > 3) {
                        double orl = side * (r0.hw + 0.6);
                        face(c.out.ms[M_METAL], orl, zo + 0.45, zo + 0.8, side > 0 ? -1 : 1, Col{0.75f, 0.76f, 0.78f, 1}, true);
                        face(c.out.ms[M_METAL], orl + side * 0.02, zo + 0.45, zo + 0.8, side > 0 ? 1 : -1, Col{0.65f, 0.66f, 0.68f, 1}, false);
                        for (size_t i = 0; i < P.size(); i += 1) {
                            if (s[i] - std::floor(s[i] / 4) * 4 > 3.01 || !c.own(pp[i])) continue;
                            V3 b = Q(i, orl + side * 0.1, zo + 0.15);
                            c.out.ms[M_METAL].obox({b.x, b.y}, 0, 0.1, 0.1, b.z, P[i].z + zo + 0.8, Col{0.6f, 0.6f, 0.62f, 1});
                        }
                    }
                }
            }
        if (r0.sw > 0) {
            Col kc{0.78f, 0.78f, 0.76f, 1}, pc{1, 1, 1, 1};
            for (int side : {1, -1}) {
                if (med[r0i][side > 0 ? 0 : 1] >= 0) continue;
                double swd = r0.sws[side > 0 ? 0 : 1];
                if (swd <= 0) continue;
                double o0 = side * r0.hw, o1 = side * (r0.hw + 0.18), o2 = side * (r0.hw + swd);
                face(CU, o0, zo - 0.02, 0.15 + zo, side > 0 ? -1 : 1, kc, true);
                if (side > 0) {
                    strip(CU, o0, o1, 0.15 + zo, 0.15 + zo, kc, true, 1);
                    strip(PV, o1, o2, 0.15 + zo, 0.15 + zo, pc, true, 3);
                } else {
                    strip(CU, o1, o0, 0.15 + zo, 0.15 + zo, kc, true, 1);
                    strip(PV, o2, o1, 0.15 + zo, 0.15 + zo, pc, true, 3);
                }
                face(CU, o2, -0.4, 0.15 + zo, side > 0 ? 1 : -1, kc, false);
                // walking line in the middle of the sidewalk
                double om = side * (r0.hw + std::min(swd * 0.55, 4.0));
                for (size_t i = 0; i + 1 < P.size(); i++) {
                    if (!c.own((pp[i] + pp[i + 1]) * 0.5)) continue;
                    c.out.walks.push_back({Q(i, om, 0.15 + zo), Q(i + 1, om, 0.15 + zo)});
                }
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
        // speed limit signs where a road with a posted limit leaves a junction (the town default, 60, only on
        // main roads)
        if (!r0.bridge && r0.speed >= 30 && r0.speed <= 60 && r0.speed % 10 == 0 && (r0.speed != 60 || r0.prio >= 6) && s.back() > 30) {
            int sg = r0.speed == 30 ? SG_SPEED30 : r0.speed == 40 ? SG_SPEED40 : r0.speed == 50 ? SG_SPEED50 : SG_SPEED60;
            for (int e = 0; e < 2; e++) {
                if (e == 1 && r0.oneway != 0) continue;
                if (e == 0 && r0.oneway < 0) continue;
                int nd = r0.node[e];
                if (nd < 0 || nodes[nd].ends.size() < 3) continue;
                double sp = e == 0 ? 14 : s.back() - 14;
                size_t i = 0;
                while (i + 1 < s.size() && s[i + 1] < sp) i++;
                V2 dir = (pp[std::min(i + 1, pp.size() - 1)] - pp[i]).norm() * (e == 0 ? 1.0 : -1.0);
                double side = e == 0 ? -1.0 : 1.0;  // right of the traffic
                V2 q = pp[i] + nn[i] * (side * (r0.hw + (r0.sw > 0 ? 0.6 : 1.0)));
                if (!c.own(q) || !free_spot(q, 0.5)) continue;
                double z = P[i].z + zo + (r0.sw > 0 ? 0.15 : 0.0);
                c.out.inst.push_back({I_SIGN, {q.x, q.y, z}, std::atan2(-dir.y, -dir.x), 1, {(float)sg, 0, 0, 1}});
            }
        }
        // cars parked along the kerb of town streets wide enough for it (not near junctions or crossings)
        if (r0.urban && !r0.bridge && !r0.tunnel && r0.prio >= 2 && r0.prio <= 5 && r0.hw >= 5.5 && s.back() > 40) {
            static const Col PC[] = {{0.85f, 0.85f, 0.86f, 1}, {0.08f, 0.08f, 0.09f, 1}, {0.45f, 0.47f, 0.5f, 1}, {0.7f, 0.72f, 0.75f, 1}, {0.55f, 0.08f, 0.07f, 1},
                                     {0.1f, 0.2f, 0.45f, 1},    {0.95f, 0.95f, 0.95f, 1}, {0.2f, 0.32f, 0.22f, 1}, {0.6f, 0.5f, 0.35f, 1}, {0.3f, 0.3f, 0.32f, 1}};
            for (int side : {-1, 1}) {
                if (r0.oneway != 0 && side > 0 && r0.hw < 7) continue;  // one-way: one parking side unless wide
                for (double sp = 18; sp < s.back() - 18; sp += 6.2) {
                    size_t i = 0;
                    while (i + 1 < s.size() && s[i + 1] < sp) i++;
                    double t = (sp - s[i]) / std::max(1e-6, s[i + 1] - s[i]);
                    V2 cp = pp[i] + (pp[i + 1] - pp[i]) * t;
                    uint64_t h = (uint64_t)std::llround(cp.x * 5.3) * 73856093ull ^ (uint64_t)std::llround(cp.y * 4.7) * 19349663ull ^ (uint64_t)(side + 7);
                    h = h * 6364136223846793005ull + 1442695040888963407ull;
                    if (((h >> 20) % 100) >= 55) continue;
                    V2 nrm = (nn[i] + (nn[i + 1] - nn[i]) * t).norm();
                    V2 dir = (pp[i + 1] - pp[i]).norm();
                    V2 q = cp + nrm * (side * (r0.hw - 0.95));
                    if (!c.own(q)) continue;
                    double z = P[i].z + (P[i + 1].z - P[i].z) * t + zo;
                    // facing the traffic of that side (right-hand traffic: the right side faces along)
                    double yaw = std::atan2(dir.y, dir.x) + (side > 0 ? PI : 0) + (((h >> 33) % 100) / 100.0 - 0.5) * 0.06;
                    c.out.inst.push_back({I_CAR, {q.x, q.y, z}, yaw, 1, PC[(h >> 40) % 10]});
                }
            }
        }
        // advertising billboards beside main roads: 6 x 3 m boards on a post (half digital, half posters),
        // about every 220 m where there is room behind the pavement
        if (r0.prio >= 5 && !r0.bridge && !r0.tunnel && s.back() > 80) {
            uint64_t hs = (uint64_t)std::llround(pp[0].x * 3.1) * 2654435761ull ^ (uint64_t)std::llround(pp[0].y * 2.7) * 40503ull;
            double first = 60 + std::fmod((double)(hs % 1000), 160.0);
            for (double sp = first; sp < s.back() - 30; sp += 220) {
                size_t i = 0;
                while (i + 1 < s.size() && s[i + 1] < sp) i++;
                double t = (sp - s[i]) / std::max(1e-6, s[i + 1] - s[i]);
                V2 cp = pp[i] + (pp[i + 1] - pp[i]) * t;
                V2 nrm = (nn[i] + (nn[i + 1] - nn[i]) * t).norm();
                int side = ((hs >> 7) + (uint64_t)(sp / 220)) % 2 ? 1 : -1;
                double off = r0.hw + std::max(r0.sws[side > 0 ? 0 : 1], 1.0) + 2.5;
                V2 bp = cp + nrm * (side * off);
                if (!c.own(bp) || !free_spot(bp, 4.0) || c.roads.clearance(bp) < 2.0 || (c.occ_at(bp) & O_WATER)) continue;
                double z = c.ground(bp);
                V2 along = (pp[i + 1] - pp[i]).norm();
                // the board faces the oncoming traffic on its side, angled 25 degrees towards the road
                V2 face = (along * (side > 0 ? 1.0 : -1.0) + nrm * (-side * 0.45)).norm();
                V2 bx = face.perp();
                MeshBuf &MT2 = c.out.ms[M_METAL];
                MT2.cyl(bp, 0.22, 0.2, z - 0.3, z + 5.0, 10, Col{0.45f, 0.46f, 0.48f, 1});
                c.coll_prism(bp, 0, 0.5, 0.5, z - 0.3, z + 5.0);
                double W = 6.0, H = 3.0, z0 = z + 4.6;
                auto Pt = [&](double u, double v, double o) { V2 q = bp + bx * u + face * o; return V3(q.x, q.y, z0 + v); };
                V3 nf(face.x, face.y, 0);
                c.out.ms[M_DARK].quad(Pt(W / 2 + 0.15, -0.15, 0.25), Pt(-W / 2 - 0.15, -0.15, 0.25), Pt(-W / 2 - 0.15, H + 0.15, 0.25), Pt(W / 2 + 0.15, H + 0.15, 0.25), nf, Col{1, 1, 1, 1});
                c.out.ms[M_DARK].quad(Pt(-W / 2 - 0.15, -0.15, 0.2), Pt(W / 2 + 0.15, -0.15, 0.2), Pt(W / 2 + 0.15, H + 0.15, 0.2), Pt(-W / 2 - 0.15, H + 0.15, 0.2), nf * -1, Col{1, 1, 1, 1});
                bool digital = (hs >> 13) % 2;
                MeshBuf &S = c.out.ms[digital ? M_SCREEN : M_POSTER];
                float seed = (float)((hs >> 17) % 97) / 97.0f;
                Col sc{seed, (float)(W / H) / 4, 0, 1};
                int a0 = S.v(Pt(W / 2, 0, 0.27), nf, 0, 1, sc), a1 = S.v(Pt(-W / 2, 0, 0.27), nf, 1, 1, sc), a2 = S.v(Pt(-W / 2, H, 0.27), nf, 1, 0, sc), a3 = S.v(Pt(W / 2, H, 0.27), nf, 0, 0, sc);
                S.tri(a0, a1, a2);
                S.tri(a0, a2, a3);
                hs = hs * 6364136223846793005ull + 1442695040888963407ull;
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
                V2 lp = cp + nrm * (side * (r0.hw + std::min(r0.sws[side > 0 ? 0 : 1], 3.5) - 0.6));
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
    // mapped street furniture: signals, stop / give-way signs, crossings, bus stops, signs, benches, bins
    struct FPt {
        V2 p;
        std::string kind;
    };
    std::vector<FPt> fpts;
    bool all_osm = !tiles.empty();
    for (auto &t : tiles) {
        if (!t->has_osm && !t->has_furniture) all_osm = false;
        for (auto &p : t->points) {
            const std::string &k = p.kind;
            if (k == "signal" || k == "stop" || k == "giveway" || k == "bench" || k == "bin" || k.rfind("crossing:", 0) == 0 || k.rfind("busstop", 0) == 0 || k.rfind("sign:", 0) == 0) {
                // the same node from Overture's infrastructure layer and the OSM answer: keep one
                V2 v = c.fr.to_local(p.p.lat, p.p.lon);
                bool dup = false;
                std::string kb = k.substr(0, k.find_first_of(":|"));
                for (auto &f : fpts)
                    if ((f.p - v).len() < 2.0 && f.kind.substr(0, f.kind.find_first_of(":|")) == kb) {
                        dup = true;
                        break;
                    }
                if (!dup) fpts.push_back({v, k});
            }
        }
    }
    auto has_pt = [&](V2 p, const char *kind, double r) {
        for (auto &f : fpts)
            if (f.kind == kind && (f.p - p).len() < r) return true;
        return false;
    };
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
        // traffic lights, give-way / stop signs with their lines, priority and crossing signs
        if (k >= 3 && !bridge) {
            std::set<std::string> named;
            int top1 = 0, top2 = 0, nmajor = 0;
            for (auto &e : n.ends) {
                int pe = roads[e.road].prio;
                if (pe > top1) top2 = top1, top1 = pe;
                else if (pe > top2) top2 = pe;
            }
            bool minor_arm = false;
            for (auto &e : n.ends) minor_arm |= roads[e.road].prio <= top1 - 2;
            for (auto &e : n.ends) nmajor += roads[e.road].prio >= top1 && top1 >= 5;
            bool urban_n = false;
            for (auto &e : n.ends) urban_n |= roads[e.road].urban;
            bool signalised = has_pt(n.p, "signal", 30.0) || (!all_osm && urban_n && top1 >= 6 && top2 >= 5 && k >= 3);
            bool stop_n = has_pt(n.p, "stop", 25.0);
            uint64_t hseed = (uint64_t)std::llround(n.p.x * 7.0) * 73856093ull ^ (uint64_t)std::llround(n.p.y * 7.0) * 19349663ull;
            (void)hseed;
            for (size_t ai = 0; ai < k; ai++) {
                const EP &a = ep[ai];
                const Road &ar = roads[n.ends[ai].road];
                if (ar.hw < 2.4 || ar.prio < 2) continue;
                V2 rs = a.u.perp();  // right of the traffic coming in
                double edge = a.hw + (a.sw > 0 ? 0.6 : 1.2);
                auto on_arm = [&](double along, double lat) { return a.c + a.u * along + rs * lat; };
                double zs = a.z + zo + (a.sw > 0 ? 0.15 : 0.0);
                double yaw = std::atan2(a.u.y, a.u.x);
                bool zebra = ar.urban && ar.prio >= 2 && ar.prio <= 7;
                double zl = at_chainage(ar.pts, ar.trim[n.ends[ai].start ? 0 : 1] + 4.1, n.ends[ai].start).z + 0.03 + ar.prio * 0.004 + 0.008;
                if (signalised) {
                    V2 pp = on_arm(1.6, edge);
                    if (free_spot(pp, 0.6)) c.out.inst.push_back({I_SIGNAL, {pp.x, pp.y, zs}, yaw, 1, {1, 1, 1, 1}});
                } else if (minor_arm && ar.prio <= top1 - 2 && top1 >= 5) {
                    V2 pp = on_arm(4.6, edge);
                    if (free_spot(pp, 0.6)) c.out.inst.push_back({I_SIGN, {pp.x, pp.y, zs}, yaw, 1, {stop_n ? (float)SG_STOP : (float)SG_YIELD, 0, 0, 1}});
                } else if (minor_arm && ar.prio >= top1 && top1 >= 5) {
                    V2 pp = on_arm(2.4, edge);
                    if (free_spot(pp, 0.6)) c.out.inst.push_back({I_SIGN, {pp.x, pp.y, zs}, yaw, 1, {(float)SG_PRIORITY, 0, 0, 1}});
                }
                // stop line (solid for lights and stop, teeth for give way) across the incoming half
                bool line_solid = signalised || stop_n;
                bool line_yield = !signalised && !stop_n && minor_arm && ar.prio <= top1 - 2 && top1 >= 5;
                if ((line_solid || line_yield) && zebra) {
                    Col w{0.95f, 0.95f, 0.93f, 1};
                    if (line_solid) {
                        V2 q0 = on_arm(3.9, 0.15), q1 = on_arm(3.9, a.hw - 0.2), q2 = on_arm(4.35, a.hw - 0.2), q3 = on_arm(4.35, 0.15);
                        c.out.ms[M_MARKING].quadn(V3(q0.x, q0.y, zl), V3(q3.x, q3.y, zl), V3(q2.x, q2.y, zl), V3(q1.x, q1.y, zl), w);
                    } else {
                        for (double o = 0.3; o + 0.5 < a.hw - 0.2; o += 0.9) {
                            V2 q0 = on_arm(3.9, o), q1 = on_arm(3.9, o + 0.5), q2 = on_arm(4.4, o + 0.25);
                            c.out.ms[M_MARKING].quadn(V3(q0.x, q0.y, zl), V3(q2.x, q2.y, zl), V3(q2.x, q2.y, zl), V3(q1.x, q1.y, zl), w);
                        }
                    }
                }
                if (zebra && !signalised) {
                    V2 pp = on_arm(3.4, edge + 0.5);
                    if (free_spot(pp, 0.6)) c.out.inst.push_back({I_SIGN, {pp.x, pp.y, zs}, yaw, 1, {(float)SG_CROSSING, 0, 0, 1}});
                }
                // street name plate on the right corner of each named arm (two-sided, across the arm so the
                // crossing traffic reads it)
                if (!ar.name.empty() && urban_n && ar.prio >= 2 && !named.count(ar.name)) {
                    named.insert(ar.name);
                    V2 pp = on_arm(2.2, -(edge + 0.2));
                    if (c.own(pp) && free_spot(pp, 0.5)) {
                        MeshBuf &MT = c.out.ms[M_METAL], &PM = c.out.ms[M_PAINT];
                        MT.cyl(pp, 0.04, 0.04, zs - 0.1, zs + 2.75, 6, Col{0.45f, 0.46f, 0.48f, 1});
                        double w = std::clamp(0.12 * (double)utf8_len(ar.name) + 0.3, 0.9, 2.4);
                        double pz = zs + 2.45;
                        // plate along the arm direction
                        V2 d = a.u, nside = a.u.perp();
                        V2 c0 = pp + d * 0.05;
                        auto P = [&](double along, double up, double off) { V2 q = c0 + d * along + nside * off; return V3(q.x, q.y, pz + up); };
                        for (int sd : {1, -1}) {
                            V3 nv(nside.x * sd, nside.y * sd, 0);
                            double off = 0.03 * sd;
                            V3 q0 = P(0, -0.17, off), q1 = P(w, -0.17, off), q2 = P(w, 0.17, off), q3 = P(0, 0.17, off);
                            if (sd < 0) std::swap(q0, q1), std::swap(q2, q3);
                            PM.quad(q0, q1, q2, q3, nv, Col{0.07f, 0.22f, 0.52f, 1});
                            V3 m = P(w / 2, 0, off * 1.6);
                            c.out.labels.push_back({m, std::atan2(nside.y * sd, nside.x * sd), w - 0.15, ar.name, Col{1, 1, 1, 1}});
                        }
                        c.coll_prism(pp, 0, 0.09, 0.09, zs - 0.1, zs + 2.7);
                    }
                }
                // a direction panel some way before junctions of main roads: the crossing streets left and right
                if (ar.prio >= 6 && !ar.bridge && ar.oneway >= 0 && k >= 3) {
                    std::string left, right, ahead;
                    V2 travel = a.u * -1;
                    for (size_t bi = 0; bi < k; bi++) {
                        if (bi == ai) continue;
                        const Road &br = roads[n.ends[bi].road];
                        if (br.name.empty() || br.name == ar.name || br.prio < 3) continue;
                        V2 bu = ep[bi].u;
                        double cr = travel.cross(bu), dt = travel.dot(bu);
                        if (dt > 0.7) { if (ahead.empty()) ahead = br.name; }
                        else if (cr > 0) { if (left.empty()) left = br.name; }
                        else if (right.empty()) right = br.name;
                    }
                    if (!left.empty() || !right.empty()) {
                        double back = std::min(35.0, chain_len(ar.pts) - ar.trim[0] - ar.trim[1] - 5);
                        if (back > 12) {
                            RPt q = at_chainage(ar.pts, ar.trim[n.ends[ai].start ? 0 : 1] + back, n.ends[ai].start);
                            V2 pp = q.p + rs * (a.hw + (a.sw > 0 ? a.sw * 0.5 : 1.6));
                            if (c.own(pp) && free_spot(pp, 0.8)) {
                                double gz = q.z + zo + (a.sw > 0 ? 0.15 : 0);
                                MeshBuf &MT = c.out.ms[M_METAL], &PM = c.out.ms[M_PAINT];
                                double W = 2.6, H = 1.5, z0 = gz + 2.0;
                                V2 nf = a.u;  // the panel faces the approaching traffic
                                V2 side = nf.perp();
                                for (double o : {-W * 0.35, W * 0.35}) {
                                    V2 lp = pp + side * o;
                                    MT.cyl(lp, 0.06, 0.06, gz - 0.2, z0 + H * 0.9, 6, Col{0.5f, 0.51f, 0.53f, 1});
                                    c.coll_prism(lp, 0, 0.13, 0.13, gz - 0.2, z0 + H);
                                }
                                auto P = [&](double u, double v, double off) { V2 qq = pp + side * u + nf * off; return V3(qq.x, qq.y, z0 + v); };
                                V3 nv(nf.x, nf.y, 0);
                                PM.quad(P(W / 2, 0, 0.08), P(-W / 2, 0, 0.08), P(-W / 2, H, 0.08), P(W / 2, H, 0.08), nv, Col{0.92f, 0.92f, 0.92f, 1});
                                PM.quad(P(W / 2 - 0.05, 0.05, 0.09), P(-W / 2 + 0.05, 0.05, 0.09), P(-W / 2 + 0.05, H - 0.05, 0.09), P(W / 2 - 0.05, H - 0.05, 0.09), nv, Col{0.05f, 0.2f, 0.5f, 1});
                                PM.quad(P(-W / 2, 0, 0.06), P(W / 2, 0, 0.06), P(W / 2, H, 0.06), P(-W / 2, H, 0.06), nv * -1, Col{0.5f, 0.51f, 0.53f, 1});
                                std::string text;
                                if (!ahead.empty()) text += "↑ " + ahead + "\n";
                                if (!left.empty()) text += "← " + left + "\n";
                                if (!right.empty()) text += right + " →";
                                while (!text.empty() && text.back() == '\n') text.pop_back();
                                V3 m = P(0, H / 2, 0.11);
                                c.out.labels.push_back({m, std::atan2(nf.y, nf.x), W - 0.3, text, Col{1, 1, 1, 1}});
                            }
                        }
                    }
                }
            }
        }
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
    // ---- mapped points away from junctions: bus stops, benches, bins, standalone signs, mid-block crossings ----
    auto nearest_seg = [&](V2 p, double maxd, int &rid, V2 &q, V2 &tan) {
        double best = maxd;
        rid = -1;
        for (auto &sg : c.roads.segs) {
            V2 d = sg.b - sg.a;
            double l2 = std::max(1e-9, d.dot(d));
            double tt = std::clamp((p - sg.a).dot(d) / l2, 0.0, 1.0);
            V2 cp = sg.a + d * tt;
            double dist = (cp - p).len();
            if (dist < best) best = dist, rid = sg.id, q = cp, tan = d.norm();
        }
    };
    // which side of the carriageway faces which traffic: sign at `p` should face the drivers who have it on their right
    auto face_traffic = [&](V2 p, V2 q, V2 tan) {
        V2 v = p - q;
        V2 travel = tan.cross(v) < 0 ? tan : tan * -1.0;  // the sign is on the right of this direction
        V2 f = travel * -1.0;
        return std::atan2(f.y, f.x);
    };
    auto sign_of = [&](const std::string &k) -> int {
        std::string v = k.substr(5);
        if (v.rfind("maxspeed=", 0) == 0) {
            int sp = std::atoi(v.c_str() + 9);
            return sp == 30 ? SG_SPEED30 : sp == 40 ? SG_SPEED40 : sp == 50 ? SG_SPEED50 : sp == 60 ? SG_SPEED60 : -1;
        }
        auto has = [&](const char *t) { return v.find(t) != std::string::npos; };
        if (has("2.1") && !has("2.1.")) return SG_PRIORITY;
        if (has("2.4")) return SG_YIELD;
        if (has("2.5")) return SG_STOP;
        if (has("5.19")) return SG_CROSSING;
        if (has("3.1") && !has("3.1.") && !has("3.10")) return SG_NOENTRY;
        if (has("6.4")) return SG_PARKING;
        if (has("5.16")) return SG_BUS;
        return -1;
    };
    for (auto &f : fpts) {
        if (!c.own(f.p)) continue;
        int rid;
        V2 q, tan;
        const std::string &k = f.kind;
        if (k == "signal" || k == "crossing:traffic_signals") continue;  // junction logic handles lights; mid-block signalled crossings are zebras
        bool nearj = false;
        for (auto &n : nodes)
            if (n.ends.size() >= 3 && (n.p - f.p).len() < 25) nearj = true;
        nearest_seg(f.p, 30, rid, q, tan);
        if (rid < 0) continue;
        const Road &r = roads[(size_t)rid];
        if (r.bridge) continue;
        if (!free_spot(f.p, 0.8)) continue;
        double zg = c.ground(f.p);
        if (k.rfind("busstop", 0) == 0) {
            double sidew = (f.p - q).len();
            V2 away = (f.p - q).norm();
            double yaw = std::atan2(-away.y, -away.x);
            V2 sp = f.p + away * std::max(0.0, 1.4 - std::max(0.0, sidew - r.hw));
            if (free_spot(sp, 0.5) && c.roads.clearance(sp) > -0.5) c.out.inst.push_back({I_SHELTER, {sp.x, sp.y, c.ground(sp)}, yaw, 1, {1, 1, 1, 1}});
            c.out.inst.push_back({I_SIGN, {f.p.x, f.p.y, zg}, face_traffic(f.p, q, tan), 1, {(float)SG_BUS, 0, 0, 1}});
        } else if (k == "bench" || k == "bin") {
            V2 away = (f.p - q).norm();
            c.out.inst.push_back({k == "bench" ? I_BENCH : I_BIN, {f.p.x, f.p.y, zg}, std::atan2(-away.y, -away.x), 1, {1, 1, 1, 1}});
        } else if (k == "stop" || k == "giveway") {
            if (nearj) continue;
            c.out.inst.push_back({I_SIGN, {f.p.x, f.p.y, zg}, face_traffic(f.p, q, tan), 1, {(float)(k == "stop" ? SG_STOP : SG_YIELD), 0, 0, 1}});
        } else if (k.rfind("sign:", 0) == 0) {
            int v = sign_of(k);
            if (v < 0) continue;
            c.out.inst.push_back({I_SIGN, {f.p.x, f.p.y, zg}, face_traffic(f.p, q, tan), 1, {(float)v, 0, 0, 1}});
        } else if (k.rfind("crossing:", 0) == 0) {
            std::string kind = k.substr(9);
            if (nearj || kind == "no" || kind == "unmarked") continue;
            if (!r.urban && r.prio < 3) continue;
            // zebra stripes across the carriageway at the road's point nearest the node, plus a sign on each side
            V2 lat = tan.perp();
            double zl = c.ground(q) + 0.03 + r.prio * 0.004 + 0.01;
            for (double o = -r.hw + 0.5; o + 0.5 <= r.hw - 0.3; o += 1.0) {
                V2 a = q + lat * o - tan * 1.5;
                V3 p0(a.x, a.y, zl), p1(a.x + lat.x * 0.5, a.y + lat.y * 0.5, zl);
                V3 p2 = p1 + V3(tan.x * 3, tan.y * 3, 0), p3 = p0 + V3(tan.x * 3, tan.y * 3, 0);
                c.out.ms[M_MARKING].quadn(p0, p3, p2, p1, Col{0.95f, 0.95f, 0.93f, 1});
            }
            for (int sd : {-1, 1}) {
                V2 pp = q + lat * (sd * (r.hw + (r.sw > 0 ? 0.6 : 1.2)));
                if (!free_spot(pp, 0.6)) continue;
                c.out.inst.push_back({I_SIGN, {pp.x, pp.y, c.ground(pp)}, face_traffic(pp, q, tan), 1, {(float)SG_CROSSING, 0, 0, 1}});
            }
        }
    }
}

}  // namespace ec
