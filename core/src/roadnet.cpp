#include <functional>
#include <cmath>
#include "roadnet.hpp"
#include <map>
#include <queue>
#include <set>

namespace es {

bool road_drivable(const std::string &c) {
    return c == "motorway" || c == "trunk" || c == "primary" || c == "secondary" || c == "tertiary" || c == "unclassified" || c == "residential" ||
           c == "living_street" || c == "service" || c == "track" || c == "unknown";
}

int road_prio(const std::string &c) {
    if (c == "motorway") return 10;
    if (c == "trunk") return 9;
    if (c == "primary") return 8;
    if (c == "secondary") return 7;
    if (c == "tertiary") return 6;
    if (c == "unclassified") return 5;
    if (c == "residential") return 4;
    if (c == "living_street") return 3;
    if (c == "service" || c == "unknown") return 2;
    return 1;
}

double road_default_width(const std::string &c, const std::string &sub, int oneway) {
    bool ow = oneway != 0;
    if (sub == "link") return ow ? 5.0 : 8.0;
    if (c == "motorway") return ow ? 11.0 : 22.0;
    if (c == "trunk") return ow ? 10.5 : 15.0;
    if (c == "primary") return ow ? 10.5 : 14.0;
    if (c == "secondary") return ow ? 7.0 : 11.0;
    if (c == "tertiary") return ow ? 6.5 : 8.5;
    if (c == "unclassified" || c == "residential") return ow ? 4.5 : 6.5;
    if (c == "living_street") return 5.5;
    if (c == "service") {
        if (sub == "parking_aisle") return 6.0;
        if (sub == "driveway" || sub == "alley") return 3.5;
        return 4.5;
    }
    if (c == "track") return 3.5;
    return 4.5;
}

static int default_speed(const std::string &c) {
    if (c == "motorway") return 110;
    if (c == "trunk") return 90;
    if (c == "primary" || c == "secondary") return 60;
    if (c == "tertiary" || c == "unclassified") return 50;
    if (c == "residential") return 40;
    return 20;
}

double RoadEdge::z_at(double s) const {
    if (ps.empty()) return 0;
    if (s <= ps.front()) return pz.front();
    if (s >= ps.back()) return pz.back();
    size_t i = size_t(std::upper_bound(ps.begin(), ps.end(), s) - ps.begin());
    double t = (s - ps[i - 1]) / std::max(1e-9, ps[i] - ps[i - 1]);
    return pz[i - 1] + (pz[i] - pz[i - 1]) * t;
}

namespace {
struct Cut {
    double s;
    int prio;  // 2 connector, 1 endpoint, 0 range boundary
    std::string conn;
};
bool in_ranges(const std::vector<Range> &rs, double f) {
    for (auto &r : rs)
        if (f >= r.a && f <= r.b) return true;
    return false;
}
// project p onto polyline: arc length and distance
double project(const std::vector<V2> &l, V2 p, double &arc) {
    double best = 1e18, acc = 0;
    arc = 0;
    for (size_t k = 0; k + 1 < l.size(); k++) {
        double t, sl = dist(l[k], l[k + 1]);
        double d = seg_dist(p, l[k], l[k + 1], &t);
        if (d < best) best = d, arc = acc + t * sl;
        acc += sl;
    }
    return best;
}
std::vector<V2> sub_line(const std::vector<V2> &l, double s0, double s1) {
    std::vector<V2> out;
    out.push_back(polyline_at(l, s0));
    double acc = 0;
    for (size_t k = 0; k + 1 < l.size(); k++) {
        acc += dist(l[k], l[k + 1]);
        if (acc > s0 + 1e-6 && acc < s1 - 1e-6) out.push_back(l[k + 1]);
    }
    out.push_back(polyline_at(l, s1));
    return out;
}
}  // namespace

void RoadNet::build(const Region &R) {
    nodes.clear(), edges.clear(), grid.clear();
    std::unordered_map<std::string, int> conn_node;
    std::unordered_map<uint64_t, std::vector<int>> loose;  // endpoint nodes without connector, by 2 m cell
    auto loose_node = [&](V2 p) {
        int cx = int(std::floor(p.x / 2)), cy = int(std::floor(p.y / 2));
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++) {
                auto it = loose.find(key(cx + dx, cy + dy));
                if (it == loose.end()) continue;
                for (int n : it->second)
                    if (dist(nodes[n].p, p) < 0.8) return n;
            }
        nodes.push_back({});
        nodes.back().p = p;
        loose[key(cx, cy)].push_back(int(nodes.size() - 1));
        return int(nodes.size() - 1);
    };

    for (auto &r : R.roads) {
        if (r.kind != RoadKind::Road || !road_drivable(r.cls) || r.pts.size() < 2) continue;
        double L = polyline_length(r.pts);
        if (L < 0.5) continue;
        double fspan = r.frac1 - r.frac0;
        auto f2s = [&](double f) { return (f - r.frac0) / (fspan != 0 ? fspan : 1) * L; };
        auto s2f = [&](double s) { return r.frac0 + s / L * fspan; };
        std::vector<Cut> cuts;
        for (auto &c : r.connectors) {
            auto it = R.connectors.find(c.first);
            double s;
            if (it != R.connectors.end() && it->second.have) {
                if (project(r.pts, it->second.p, s) > 2.0) continue;
            } else {
                s = f2s(c.second);
                if (s < -0.5 || s > L + 0.5) continue;
                continue;  // connector outside the loaded data: do not create a node there
            }
            cuts.push_back({clampd(s, 0, L), 2, c.first});
        }
        auto add_range = [&](const Range &g) {
            for (double f : {g.a, g.b}) {
                double s = f2s(f);
                if (s > 0.5 && s < L - 0.5) cuts.push_back({s, 0, ""});
            }
        };
        for (auto &g : r.bridge) add_range(g);
        for (auto &g : r.tunnel) add_range(g);
        for (auto &g : r.level) add_range(g.first);
        cuts.push_back({0, 1, ""});
        cuts.push_back({L, 1, ""});
        std::sort(cuts.begin(), cuts.end(), [](const Cut &a, const Cut &b) { return a.s < b.s; });
        // merge cuts closer than 0.5 m, keeping the strongest
        std::vector<Cut> cs;
        for (auto &c : cuts) {
            if (!cs.empty() && c.s - cs.back().s < 0.5) {
                if (c.prio > cs.back().prio) {
                    double keep_s = (cs.back().s == 0 || cs.back().s == L) ? cs.back().s : c.s;
                    cs.back() = c;
                    cs.back().s = keep_s;
                }
                continue;
            }
            cs.push_back(c);
        }
        if (cs.back().s < L - 1e-6) cs.back().s = L;  // the last cut must sit at the end
        cs.front().s = 0;
        if (cs.size() < 2) continue;
        std::vector<int> cn;
        for (auto &c : cs) {
            V2 p = polyline_at(r.pts, c.s);
            int n;
            if (!c.conn.empty()) {
                auto it = conn_node.find(c.conn);
                if (it != conn_node.end()) n = it->second;
                else {
                    auto ci = R.connectors.find(c.conn);
                    nodes.push_back({});
                    nodes.back().p = (ci != R.connectors.end() && ci->second.have) ? ci->second.p : p;
                    n = int(nodes.size() - 1);
                    conn_node[c.conn] = n;
                }
            } else if (c.prio == 1)
                n = loose_node(p);
            else {
                nodes.push_back({});
                nodes.back().p = p;
                n = int(nodes.size() - 1);
            }
            cn.push_back(n);
        }
        for (size_t i = 0; i + 1 < cs.size(); i++) {
            if (cn[i] == cn[i + 1]) continue;
            RoadEdge e;
            e.a = cn[i], e.b = cn[i + 1];
            e.pts = sub_line(r.pts, cs[i].s, cs[i + 1].s);
            e.pts.front() = nodes[e.a].p, e.pts.back() = nodes[e.b].p;
            // drop vertices closer than 0.3 m
            std::vector<V2> cl;
            for (auto q : e.pts)
                if (cl.empty() || dist(cl.back(), q) > 0.3) cl.push_back(q);
            if (cl.size() < 2) cl = {nodes[e.a].p, nodes[e.b].p};
            cl.back() = nodes[e.b].p;
            e.pts = cl;
            e.len = polyline_length(e.pts);
            if (e.len < 0.2) continue;
            double fm = s2f((cs[i].s + cs[i + 1].s) / 2);
            e.cls = r.cls, e.name = r.name, e.surface = r.surface;
            e.prio = road_prio(r.cls);
            e.oneway = r.oneway;
            e.bridge = in_ranges(r.bridge, fm);
            e.tunnel = in_ranges(r.tunnel, fm);
            e.len = polyline_length(e.pts);
            // short tunnels on minor roads are building passages: they stay at grade
            if (e.tunnel && e.level == 0 && (road_prio(r.cls) < 4 || e.len < 40)) e.tunnel = false;
            for (auto &l : r.level)
                if (fm >= l.first.a && fm <= l.first.b) e.level = l.second;
            // ramps of minor roads going below ground lead into garages we do not model: drop them
            if ((e.level < 0 || e.tunnel) && road_prio(r.cls) <= 2) continue;
            e.width = r.width >= 2.5 ? r.width : road_default_width(r.cls, r.subclass, r.oneway);
            e.lanes = e.oneway ? std::max(1, int(e.width / 3.4 + 0.3)) : std::max(2, 2 * int(e.width / 6.8 + 0.3));
            if (e.width < 5.0 && !e.oneway) e.lanes = 1;  // too narrow for marked lanes
            e.speed = r.speed ? r.speed : default_speed(r.cls);
            bool urban = r.cls == "primary" || r.cls == "secondary" || r.cls == "tertiary" || r.cls == "residential" || r.cls == "unclassified" || r.cls == "living_street";
            e.sidewalk = urban && !e.tunnel && r.subclass != "link";
            e.sw_width = r.cls == "primary" ? 3.5 : r.cls == "secondary" ? 3.0 : 2.5;
            nodes[e.a].edges.push_back(int(edges.size()));
            nodes[e.b].edges.push_back(int(edges.size()));
            edges.push_back(std::move(e));
        }
    }

    // contract degree-2 nodes joining identical edges
    auto same = [](const RoadEdge &x, const RoadEdge &y) {
        return x.cls == y.cls && x.width == y.width && x.bridge == y.bridge && x.tunnel == y.tunnel && x.level == y.level && x.sidewalk == y.sidewalk && x.name == y.name;
    };
    std::vector<bool> dead(edges.size(), false);
    int merged = 0;
    for (size_t n = 0; n < nodes.size(); n++) {
        auto &N = nodes[n];
        if (N.edges.size() != 2) continue;
        int i = N.edges[0], j = N.edges[1];
        if (i == j || dead[i] || dead[j]) continue;
        RoadEdge &E1 = edges[i], &E2 = edges[j];
        if (!same(E1, E2)) continue;
        // orient E1 to end at n and E2 to start at n
        RoadEdge A = E1, B = E2;
        if (A.b != int(n)) std::reverse(A.pts.begin(), A.pts.end()), std::swap(A.a, A.b), A.oneway = -A.oneway;
        if (B.a != int(n)) std::reverse(B.pts.begin(), B.pts.end()), std::swap(B.a, B.b), B.oneway = -B.oneway;
        if (A.oneway != B.oneway || A.a == B.b) continue;
        RoadEdge M = A;
        M.b = B.b;
        M.pts.insert(M.pts.end(), B.pts.begin() + 1, B.pts.end());
        M.len = A.len + B.len;
        M.speed = std::max(A.speed, B.speed);
        edges[i] = M;
        dead[j] = true;
        auto &nb = nodes[M.b].edges;
        for (auto &x : nb)
            if (x == j) x = i;
        N.edges.clear();
        merged++;
    }
    // compact
    {
        std::vector<int> remap(edges.size(), -1);
        std::vector<RoadEdge> ne;
        for (size_t i = 0; i < edges.size(); i++)
            if (!dead[i]) remap[i] = int(ne.size()), ne.push_back(std::move(edges[i]));
        edges.swap(ne);
        for (auto &N : nodes) {
            std::vector<int> v;
            for (int x : N.edges)
                if (x >= 0 && remap[x] >= 0) v.push_back(remap[x]);
            std::sort(v.begin(), v.end());
            v.erase(std::unique(v.begin(), v.end()), v.end());
            N.edges = v;
        }
    }
    // signals: a traffic signal point within 22 m of a junction node
    {
        std::unordered_map<uint64_t, std::vector<V2>> sg;
        for (auto &p : R.points)
            if (p.kind == "traffic_signals" || p.kind == "crossing_signals") sg[key(int(std::floor(p.p.x / 25)), int(std::floor(p.p.y / 25)))].push_back(p.p);
        for (auto &N : nodes) {
            if (N.edges.size() < 3) continue;
            int cx = int(std::floor(N.p.x / 25)), cy = int(std::floor(N.p.y / 25));
            for (int dx = -1; dx <= 1 && !N.signals; dx++)
                for (int dy = -1; dy <= 1 && !N.signals; dy++) {
                    auto it = sg.find(key(cx + dx, cy + dy));
                    if (it != sg.end())
                        for (auto q : it->second)
                            if (dist(q, N.p) < 22) N.signals = true;
                }
        }
    }
    index();
    medians();
    profiles(R);
    int nb = 0, nt = 0;
    for (auto &e : edges) nb += e.bridge || e.level > 0, nt += e.tunnel || e.level < 0;
    log.push_back("roadnet: " + std::to_string(nodes.size()) + " nodes, " + std::to_string(edges.size()) + " edges (" + std::to_string(merged) + " merged), bridges " + std::to_string(nb) + ", tunnels " + std::to_string(nt));
}

// Dual carriageways: the opposite carriageway on one side means a median there, not a pavement.
void RoadNet::medians() {
    int cut = 0;
    std::vector<int> cand;
    for (size_t i = 0; i < edges.size(); i++) {
        auto &e = edges[i];
        e.sw_l = e.sw_r = e.sidewalk;
        if (!e.sidewalk || !e.oneway) continue;
        for (double side : {1.0, -1.0}) {
            int votes = 0;
            for (double t : {0.2, 0.5, 0.8}) {
                V2 d;
                V2 p = e.at(e.len * t, &d);
                V2 n = perp(d) * side;
                V2 travel = d * double(e.oneway);
                query(p, 40, cand);
                bool hit = false;
                for (int j : cand) {
                    if (j == int(i)) continue;
                    auto &f = edges[j];
                    if (!f.oneway || f.prio < 4) continue;
                    double best = 1e9, acc = 0, arc = 0;
                    V2 fd{1, 0};
                    for (size_t k = 0; k + 1 < f.pts.size(); k++) {
                        double u, sl = dist(f.pts[k], f.pts[k + 1]);
                        double dd = seg_dist(p, f.pts[k], f.pts[k + 1], &u);
                        if (dd < best) best = dd, arc = acc + u * sl, fd = norm(f.pts[k + 1] - f.pts[k]);
                        acc += sl;
                    }
                    V2 q = f.at(arc);
                    double lat = dot(q - p, n), lon = std::fabs(dot(q - p, d));
                    if (lat < e.half() + 0.5 || lat > e.half() + f.half() + 28 || lon > 12) continue;
                    if (dot(travel, fd * double(f.oneway)) > -0.6) continue;
                    hit = true;
                    break;
                }
                votes += hit;
            }
            if (votes >= 2) {
                (side > 0 ? e.sw_l : e.sw_r) = false;
                cut++;
            }
        }
    }
    log.push_back("medians: " + std::to_string(cut) + " pavement sides removed next to opposite carriageways");
}

void RoadNet::index() {
    grid.clear();
    for (size_t i = 0; i < edges.size(); i++) {
        auto &e = edges[i];
        std::set<uint64_t> cells;
        for (size_t k = 0; k + 1 < e.pts.size(); k++) {
            V2 a = e.pts[k], b = e.pts[k + 1];
            double l = dist(a, b);
            int n = std::max(1, int(l / (cell * 0.5)));
            for (int j = 0; j <= n; j++) {
                V2 p = lerp(a, b, double(j) / n);
                cells.insert(key(int(std::floor(p.x / cell)), int(std::floor(p.y / cell))));
            }
        }
        for (auto c : cells) grid[c].push_back(int(i));
    }
}

void RoadNet::query_rect(const Rect &r, std::vector<int> &out) const {
    out.clear();
    int x0 = int(std::floor(r.mn.x / cell)), x1 = int(std::floor(r.mx.x / cell));
    int y0 = int(std::floor(r.mn.y / cell)), y1 = int(std::floor(r.mx.y / cell));
    for (int x = x0; x <= x1; x++)
        for (int y = y0; y <= y1; y++) {
            auto it = grid.find(key(x, y));
            if (it != grid.end()) out.insert(out.end(), it->second.begin(), it->second.end());
        }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

void RoadNet::query(V2 p, double maxd, std::vector<int> &out) const {
    // cells are filled by sampling the centreline, so pad by one cell
    query_rect(Rect{p - V2{maxd + cell, maxd + cell}, p + V2{maxd + cell, maxd + cell}}, out);
}

RoadHit RoadNet::nearest(V2 p, double maxd) const {
    RoadHit h;
    std::vector<int> c;
    query(p, maxd, c);
    for (int i : c) {
        auto &e = edges[i];
        double acc = 0;
        for (size_t k = 0; k + 1 < e.pts.size(); k++) {
            double t, sl = dist(e.pts[k], e.pts[k + 1]);
            double d = seg_dist(p, e.pts[k], e.pts[k + 1], &t);
            if (d < h.d && d <= maxd) {
                h.d = d, h.edge = i, h.s = acc + t * sl;
                h.side = cross(e.pts[k + 1] - e.pts[k], p - e.pts[k]) >= 0 ? d : -d;
            }
            acc += sl;
        }
    }
    return h;
}

RoadHit RoadNet::nearest_at(V2 p, double z, double maxd, double dz) const {
    RoadHit h;
    std::vector<int> c;
    query(p, maxd, c);
    for (int i : c) {
        auto &e = edges[i];
        double acc = 0;
        for (size_t k = 0; k + 1 < e.pts.size(); k++) {
            double t, sl = dist(e.pts[k], e.pts[k + 1]);
            double d = seg_dist(p, e.pts[k], e.pts[k + 1], &t);
            if (d < h.d && d <= maxd && std::fabs(e.z_at(acc + t * sl) - z) <= dz) {
                h.d = d, h.edge = i, h.s = acc + t * sl;
                h.side = cross(e.pts[k + 1] - e.pts[k], p - e.pts[k]) >= 0 ? d : -d;
            }
            acc += sl;
        }
    }
    return h;
}

void RoadNet::profiles(const Region &R) {
    const double STEP_UP = 6.5, STEP_DOWN = 6.0;  // vertical offset per level, m
    size_t N = nodes.size();
    for (auto &n : nodes) n.ground = R.dem.sample(n.p);
    auto structure_off = [&](const RoadEdge &e) {
        if (e.level < 0 || (e.tunnel && e.level == 0)) return -STEP_DOWN * std::max(1, std::abs(e.level));
        if (e.level > 0) return STEP_UP * std::min(e.level, 3);
        return 0.0;
    };
    // targets: nodes touching structures want the structure's offset
    std::vector<double> tpos(N, 0), tneg(N, 0);
    for (auto &e : edges) {
        double o = structure_off(e);
        for (int n : {e.a, e.b}) {
            if (o > 0) tpos[n] = std::max(tpos[n], o);
            if (o < 0) tneg[n] = std::min(tneg[n], o);
        }
    }
    // junctions on ordinary roads are pinned to the ground: ramps go into the structure instead
    std::vector<bool> pin(N, false);
    for (size_t n = 0; n < N; n++) {
        if (nodes[n].edges.size() < 3) continue;
        for (int ei : nodes[n].edges)
            if (structure_off(edges[ei]) == 0) pin[n] = true;
    }
    // graph distance to the nearest pin (Dijkstra)
    std::vector<double> dpin(N, 1e18);
    {
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> q;
        for (size_t n = 0; n < N; n++)
            if (pin[n]) dpin[n] = 0, q.push({0, int(n)});
        while (!q.empty()) {
            auto [d, n] = q.top();
            q.pop();
            if (d > dpin[n]) continue;
            for (int ei : nodes[n].edges) {
                auto &e = edges[ei];
                int m = e.a == n ? e.b : e.a;
                if (d + e.len < dpin[m]) dpin[m] = d + e.len, q.push({d + e.len, m});
            }
        }
    }
    std::vector<double> pos, neg;
    auto relax = [&] {
        // field from structure targets, decaying with grade along roads; flat along structures
        auto spread = [&](const std::vector<double> &tgt, double sg, std::vector<double> &f) {
            f.assign(N, 0);
            std::priority_queue<std::pair<double, int>> q;
            for (size_t n = 0; n < N; n++)
                if (tgt[n] * sg > 0) f[n] = tgt[n], q.push({tgt[n] * sg, int(n)});
            while (!q.empty()) {
                auto [v, n] = q.top();
                q.pop();
                if (v < f[n] * sg - 1e-9) continue;
                for (int ei : nodes[n].edges) {
                    auto &e = edges[ei];
                    int m = e.a == n ? e.b : e.a;
                    double so = structure_off(e) * sg;
                    double cand = so > 0 ? std::max(v - grade * e.len, std::min(v, so)) : v - grade * e.len;
                    if (cand > f[m] * sg + 1e-6) f[m] = cand * sg, q.push({cand, m});
                }
            }
            // cap by distance to pinned junctions
            for (size_t n = 0; n < N; n++) f[n] = sg * std::min(f[n] * sg, grade * dpin[n]);
        };
        spread(tpos, 1, pos);
        spread(tneg, -1, neg);
        for (size_t n = 0; n < N; n++) {
            nodes[n].off = pos[n] + neg[n];
            nodes[n].z = nodes[n].ground + nodes[n].off;
        }
        for (auto &e : edges) {
            int ns = std::max(1, int(std::ceil(e.len / 4.0)));
            e.ps.resize(ns + 1), e.pz.resize(ns + 1);
            double so = structure_off(e);
            double zA = nodes[e.a].z, zB = nodes[e.b].z, L = e.len;
            for (int k = 0; k <= ns; k++) {
                double s = L * k / ns, t = double(k) / ns;
                double g = R.dem.sample(e.at(s));
                // positive part: wanted value, then the Lipschitz envelope through the end values
                double up = so > 0 ? so : std::max({0.0, pos[e.a] - grade * s, pos[e.b] - grade * (L - s)});
                double op = std::min({up, pos[e.a] + grade * s, pos[e.b] + grade * (L - s)});
                double dn = so < 0 ? so : std::min({0.0, neg[e.a] + grade * s, neg[e.b] + grade * (L - s)});
                double on = std::max({dn, neg[e.a] - grade * s, neg[e.b] - grade * (L - s)});
                double z = g + op + on;
                if (e.bridge || e.level > 0) z = std::max(z, lerpd(zA, zB, t));  // a deck does not sag into a valley
                if (e.tunnel || e.level < 0) z = std::min(z, lerpd(zA, zB, t));  // a tunnel does not climb under a hill
                e.ps[k] = s, e.pz[k] = z;
            }
            e.ps.back() = L;
            e.pz.front() = zA, e.pz.back() = zB;
            // smooth the profile (removes DEM wiggles), ends fixed; bridges keep their straight decks
            if (ns >= 3 && !(e.bridge || e.level > 0)) {
                std::vector<double> t2(e.pz);
                for (int it = 0; it < 3; it++) {
                    for (int k = 1; k < ns; k++) t2[k] = 0.25 * e.pz[k - 1] + 0.5 * e.pz[k] + 0.25 * e.pz[k + 1];
                    for (int k = 1; k < ns; k++) e.pz[k] = t2[k];
                }
            }
        }
    };
    relax();
    // clearance: a bridge must pass >= 5.2 m above whatever it crosses (roads not joined to it, railways)
    std::vector<std::vector<V2>> rails;
    for (auto &r : R.roads)
        if (r.kind == RoadKind::Rail && r.tunnel.empty() && r.cls != "subway") rails.push_back(r.pts);
    for (int iter = 0; iter < 3; iter++) {
        int raised = 0;
        for (size_t i = 0; i < edges.size(); i++) {
            auto &e = edges[i];
            if (!(e.bridge || e.level > 0)) continue;
            double need = 0;
            auto check = [&](const std::vector<V2> &l, const RoadEdge *o) {
                double acc_e = 0;
                for (size_t k = 0; k + 1 < e.pts.size(); k++) {
                    double sl = dist(e.pts[k], e.pts[k + 1]);
                    double acc_o = 0;
                    for (size_t j = 0; j + 1 < l.size(); j++) {
                        double so_ = dist(l[j], l[j + 1]);
                        double t, u;
                        if (seg_intersect(e.pts[k], e.pts[k + 1], l[j], l[j + 1], t, u)) {
                            double s = acc_e + t * sl;
                            double zu = o ? o->z_at(acc_o + u * so_) : R.dem.sample(lerp(l[j], l[j + 1], u));
                            double zd = e.z_at(s);
                            // only lift when the deck is meant to be above (it is the bridge)
                            if (zd - zu < 5.2 && zd - zu > -3.0) need = std::max(need, 5.2 - (zd - zu));
                        }
                        acc_o += so_;
                    }
                    acc_e += sl;
                }
            };
            std::vector<int> near;
            Rect bb;
            for (auto p : e.pts) bb.add(p);
            query_rect(bb, near);
            for (int j : near) {
                if (j == int(i)) continue;
                auto &o = edges[j];
                if (o.a == e.a || o.a == e.b || o.b == e.a || o.b == e.b) continue;
                if (o.bridge || o.level > 0) continue;
                check(o.pts, &o);
            }
            for (auto &rl : rails) {
                Rect rb;
                for (auto p : rl) rb.add(p);
                if (rb.overlaps(bb)) check(rl, nullptr);
            }
            if (need > 0.05 && iter == 2) log.push_back("  bridge at (" + std::to_string(int(e.pts[0].x)) + "," + std::to_string(int(e.pts[0].y)) + ") still needs " + std::to_string(need) + " m (pinned " + std::to_string(pin[e.a]) + std::to_string(pin[e.b]) + ")");
            if (need > 0.05) {
                for (int n : {e.a, e.b}) tpos[n] = std::max(tpos[n], pos[n] + need);
                raised++;
            }
        }
        if (!raised) break;
        log.push_back("clearance pass " + std::to_string(iter) + ": raised " + std::to_string(raised) + " bridges");
        relax();
    }
    merge_overlaps();
}

// Where a minor road's carriageway overlaps a more important one at about the same level (shallow
// merges, service roads running alongside a street), the minor road takes the major road's height
// and only climbs away once they separate. Ends stay at their node heights.
void RoadNet::merge_overlaps() {
    int changed = 0;
    std::vector<int> cand;
    for (size_t i = 0; i < edges.size(); i++) {
        RoadEdge &e = edges[i];
        if (e.bridge || e.tunnel || e.level) continue;
        size_t n = e.ps.size();
        if (n < 3) continue;
        std::vector<double> delta(n, NAN);
        bool any = false;
        for (size_t k = 1; k + 1 < n; k++) {
            V2 p = e.at(e.ps[k]);
            query(p, e.half() + 12, cand);
            double best = NAN, bd = 1e9;
            for (int j : cand) {
                if (j == int(i)) continue;
                const RoadEdge &f = edges[j];
                if (f.prio <= e.prio || f.bridge || f.tunnel || f.level) continue;
                double fd = 1e18, acc = 0, arc = 0;
                for (size_t q = 0; q + 1 < f.pts.size(); q++) {
                    double t, sl = dist(f.pts[q], f.pts[q + 1]);
                    double d = seg_dist(p, f.pts[q], f.pts[q + 1], &t);
                    if (d < fd) fd = d, arc = acc + t * sl;
                    acc += sl;
                }
                if (fd > e.half() + f.half() + 0.5) continue;
                double zf = f.z_at(arc);
                if (std::fabs(zf - e.pz[k]) > 2.5) continue;  // different levels: a crossing, not a merge
                if (fd < bd) bd = fd, best = zf;
            }
            if (!std::isnan(best)) delta[k] = best - e.pz[k], any = true;
        }
        if (!any) continue;
        // spread each constrained delta along the edge, fading at 6% grade; ends fixed
        std::vector<double> out(n, 0.0);
        for (size_t k = 1; k + 1 < n; k++) {
            double v = 0, wbest = 0;
            for (size_t c = 1; c + 1 < n; c++) {
                if (std::isnan(delta[c])) continue;
                double ds = std::fabs(e.ps[k] - e.ps[c]);
                double fade = std::fabs(delta[c]) / 0.06 + 4.0;
                double w = ds == 0 ? 1.0 : std::max(0.0, 1.0 - ds / fade);
                if (w > wbest) wbest = w, v = delta[c] * w;
            }
            out[k] = v;
        }
        // keep the ends at the nodes: blend the correction out over the first/last 6 m
        for (size_t k = 1; k + 1 < n; k++) {
            double de = std::min(e.ps[k], e.len - e.ps[k]);
            bool hard = !std::isnan(delta[k]);
            double fe = hard ? 1.0 : clampd(de / 6.0, 0.0, 1.0);
            e.pz[k] += out[k] * fe;
        }
        changed++;
    }
    log.push_back("merge overlaps: " + std::to_string(changed) + " minor roads levelled with the roads they overlap");
}

}  // namespace es

namespace es {
bool RoadNet::follow(V2 p, V2 h, double ahead, V2 &target, int *speed_limit, V2 *centre, double *width, double z) const {
    RoadHit hit = std::isnan(z) ? nearest(p, 25) : nearest_at(p, z, 25, 3.5);
    if (hit.edge < 0) return false;
    int ei = hit.edge;
    V2 d;
    edges[ei].at(hit.s, &d);
    int dir = dot(d, h) >= 0 ? 1 : -1;  // travelling a->b (+1) or b->a (-1)
    double s = hit.s, left = ahead;
    for (int guard = 0; guard < 16; guard++) {
        const RoadEdge &e = edges[ei];
        double room = dir > 0 ? e.len - s : s;
        if (left <= room) {
            double st = s + dir * left;
            V2 t;
            V2 c = e.at(st, &t);
            t = t * double(dir);
            // right-hand lane centre: a quarter of the width right of the centre on two-way roads,
            // on one-way roads the rightmost lane
            double off = e.oneway ? std::max(0.0, e.half() - 1.8) : std::min(e.half() * 0.5, 2.0);
            target = c - perp(t) * off;
            if (speed_limit) *speed_limit = e.speed;
            if (centre) *centre = c;
            if (width) *width = e.width;
            return true;
        }
        left -= room;
        int node = dir > 0 ? e.b : e.a;
        V2 tin;
        e.at(dir > 0 ? e.len : 0.0, &tin);
        tin = tin * double(dir);
        // straightest allowed continuation
        int best = -1, bdir = 1;
        double bd = -2;
        for (int nj : nodes[node].edges) {
            if (nj == ei) continue;
            const RoadEdge &f = edges[nj];
            int fd = f.a == node ? 1 : -1;
            if (f.oneway && f.oneway != fd) continue;
            if (f.prio < 2) continue;
            V2 q = f.at(fd > 0 ? std::min(f.len, 8.0) : std::max(0.0, f.len - 8.0));
            double dd = dot(norm(q - nodes[node].p), tin);
            // avoid dead ends (we cannot turn round) and minor service lanes when there is a choice
            int far = fd > 0 ? f.b : f.a;
            if (nodes[far].edges.size() <= 1) dd -= 2.5;
            if (f.prio <= 2 && edges[ei].prio > 2) dd -= 0.6;
            if (dd > bd) bd = dd, best = nj, bdir = fd;
        }
        if (best < 0) {
            // dead end: stop at the end
            target = nodes[node].p;
            if (speed_limit) *speed_limit = 0;
            if (centre) *centre = target;
            if (width) *width = 0;
            return true;
        }
        ei = best, dir = bdir;
        s = dir > 0 ? 0 : edges[ei].len;
    }
    return false;
}
}  // namespace es
