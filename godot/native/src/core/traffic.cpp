#include "traffic.hpp"
#include <algorithm>

namespace ec {

namespace {
uint64_t qk(V3 p) {
    int64_t i = (int64_t)std::llround(p.x * 2), j = (int64_t)std::llround(p.y * 2);
    return ((uint64_t)(uint32_t)i << 32) | (uint32_t)j;
}
double speed_for(int prio) {
    static const double K[10] = {12, 15, 20, 40, 45, 50, 55, 60, 80, 90};
    return K[std::clamp(prio, 0, 9)] / 3.6;
}
const Col COLS[] = {{0.85f, 0.85f, 0.86f, 1}, {0.08f, 0.08f, 0.09f, 1}, {0.45f, 0.47f, 0.5f, 1}, {0.7f, 0.72f, 0.75f, 1}, {0.55f, 0.08f, 0.07f, 1},
                    {0.1f, 0.2f, 0.45f, 1},    {0.95f, 0.95f, 0.95f, 1}, {0.2f, 0.32f, 0.22f, 1}, {0.6f, 0.5f, 0.35f, 1}, {0.3f, 0.3f, 0.32f, 1}};
}  // namespace

void Traffic::set_roads(const std::vector<LaneSeg> &segs) {
    // keep cars where they are: remember positions, rebuild, then snap each car to its old edge by geometry
    std::unordered_map<uint64_t, int> nodes;
    std::vector<Edge> E;
    std::vector<std::vector<int>> NE;
    auto node = [&](V3 p) {
        auto it = nodes.find(qk(p));
        if (it != nodes.end()) return it->second;
        int n = (int)NE.size();
        nodes[qk(p)] = n;
        NE.push_back({});
        return n;
    };
    for (auto &s : segs) {
        if (s.prio < 1) continue;
        double len = std::hypot(s.b.x - s.a.x, s.b.y - s.a.y);
        if (len < 0.05) continue;
        Edge e{node(s.a), node(s.b), s.a, s.b, len, s.hw, s.oneway, s.prio, s.lanes};
        if (e.a == e.b) continue;
        int id = (int)E.size();
        E.push_back(e);
        NE[e.a].push_back(id);
        NE[e.b].push_back(id);
    }
    // re-attach cars by their edge endpoints
    std::unordered_map<uint64_t, int> by_ends;
    for (int i = 0; i < (int)E.size(); i++) by_ends[qk(E[i].pa) * 31 + qk(E[i].pb)] = i;
    std::vector<TrafficCar> keep;
    for (auto &c : cars_) {
        if (c.edge < 0 || c.edge >= (int)edges_.size()) continue;
        const Edge &o = edges_[c.edge];
        auto it = by_ends.find(qk(o.pa) * 31 + qk(o.pb));
        if (it == by_ends.end()) continue;
        c.edge = it->second;
        c.prev_node = -1;
        keep.push_back(c);
    }
    edges_ = std::move(E);
    node_edges_ = std::move(NE);
    cars_ = std::move(keep);
}

V3 Traffic::dir_of(const TrafficCar &c) const {
    const Edge &e = edges_[c.edge];
    V3 d = e.pb - e.pa;
    d = d * (1 / std::max(1e-9, d.len()));
    return c.fwd ? d : d * -1;
}

void Traffic::place(TrafficCar &c) {
    const Edge &e = edges_[c.edge];
    double t = std::clamp(c.s / e.len, 0.0, 1.0);
    V3 a = c.fwd ? e.pa : e.pb, b = c.fwd ? e.pb : e.pa;
    V3 p = a + (b - a) * t;
    V2 d(b.x - a.x, b.y - a.y);
    d = d.norm();
    V2 right(d.y, -d.x);
    c.pos = V3(p.x + right.x * c.lane, p.y + right.y * c.lane, p.z);
    c.yaw = std::atan2(d.y, d.x);
    double target_pitch = std::atan2(b.z - a.z, e.len);
    c.pitch += (target_pitch - c.pitch) * 0.2;
}

int Traffic::next_edge(const TrafficCar &c, int node) {
    V3 d = dir_of(c);
    std::vector<std::pair<double, int>> opts;
    for (int id : node_edges_[node]) {
        if (id == c.edge) continue;
        const Edge &e = edges_[id];
        bool fwd = e.a == node;
        if ((fwd && e.oneway < 0) || (!fwd && e.oneway > 0)) continue;
        if (e.prio < 2 && edges_[c.edge].prio >= 2) continue;  // stay off service roads
        V3 nd = (fwd ? e.pb - e.pa : e.pa - e.pb);
        nd = nd * (1 / std::max(1e-9, nd.len()));
        double straight = d.x * nd.x + d.y * nd.y;
        if (straight < -0.3) continue;  // no sharp turn-backs
        double w = 0.2 + std::max(0.0, straight) * 2 + (e.prio >= edges_[c.edge].prio ? 0.5 : 0);
        opts.push_back({w, fwd ? id : -id - 1});
    }
    if (opts.empty()) return INT32_MIN;
    double tot = 0;
    for (auto &o : opts) tot += o.first;
    double r = rnd() * tot;
    for (auto &o : opts) {
        if ((r -= o.first) <= 0) return o.second;
    }
    return opts.back().second;
}

bool Traffic::spawn(V3 player) {
    if (edges_.empty()) return false;
    for (int attempt = 0; attempt < 30; attempt++) {
        int id = (int)(rnd() * edges_.size());
        const Edge &e = edges_[id];
        if (e.prio < 2) continue;
        V3 m = (e.pa + e.pb) * 0.5;
        double d = std::hypot(m.x - player.x, m.y - player.y);
        if (d < 120 || d > radius) continue;
        bool crowded = false;
        for (auto &c : cars_)
            if (std::hypot(c.pos.x - m.x, c.pos.y - m.y) < 15) crowded = true;
        if (crowded) continue;
        TrafficCar c;
        c.edge = id;
        c.fwd = e.oneway > 0 ? true : e.oneway < 0 ? false : rnd() < 0.5;
        c.s = rnd() * e.len;
        c.vmax = speed_for(e.prio) * (0.85 + 0.3 * rnd());
        c.v = c.vmax * 0.7;
        int lanes_dir = e.oneway ? std::max(1, e.lanes) : std::max(1, e.lanes / 2);
        double lw = e.oneway ? 2 * e.hw / lanes_dir : e.hw / lanes_dir;
        int lane = (int)(rnd() * lanes_dir);
        c.lane = e.oneway ? -e.hw + lw * (lane + 0.5) : lw * (lane + 0.5);
        c.col = COLS[(int)(rnd() * 10) % 10];
        place(c);
        c.pitch = 0;
        cars_.push_back(c);
        return true;
    }
    return false;
}

void Traffic::step(double dt, V3 player, V3 pvel) {
    // drop far cars, top up
    cars_.erase(std::remove_if(cars_.begin(), cars_.end(), [&](const TrafficCar &c) { return std::hypot(c.pos.x - player.x, c.pos.y - player.y) > radius + 80 || c.edge < 0; }), cars_.end());
    for (int k = 0; k < 3 && (int)cars_.size() < target; k++) spawn(player);
    for (size_t i = 0; i < cars_.size(); i++) {
        TrafficCar &c = cars_[i];
        if (c.edge < 0) continue;
        V3 d = dir_of(c);
        // gap to the nearest obstacle ahead in this lane: other cars and the player
        double gap = 1e9;
        auto check = [&](V3 p) {
            double dx = p.x - c.pos.x, dy = p.y - c.pos.y;
            double ahead = dx * d.x + dy * d.y, side = std::abs(-dx * d.y + dy * d.x);
            if (ahead > 0.5 && ahead < 40 && side < 2.2 && std::abs(p.z - c.pos.z) < 4) gap = std::min(gap, ahead - 4.6);
        };
        for (size_t j = 0; j < cars_.size(); j++)
            if (j != i) check(cars_[j].pos);
        check(player);
        // turns ahead: slow down near the end of the edge when the road bends
        const Edge &e = edges_[c.edge];
        double remain = e.len - c.s;
        double vt = c.vmax;
        if (remain < 25) {
            int node = c.fwd ? e.b : e.a;
            int sharp = 0;
            for (int id : node_edges_[node]) sharp += id != c.edge;
            if (sharp >= 2) vt = std::min(vt, 9.0 + remain * 0.4);  // junction
        }
        // keep a time gap
        double vsafe = std::max(0.0, (gap - 2.0) / 1.4);
        vt = std::min(vt, vsafe);
        double acc = vt > c.v ? 2.0 : -6.0;
        c.v = std::max(0.0, c.v + std::clamp(vt - c.v, -6.0 * dt, 2.0 * dt));
        (void)acc;
        c.s += c.v * dt;
        int guard = 0;
        while (c.edge >= 0 && c.s > edges_[c.edge].len && guard++ < 8) {
            const Edge &ce = edges_[c.edge];
            c.s -= ce.len;
            int node = c.fwd ? ce.b : ce.a;
            int nx = next_edge(c, node);
            if (nx == INT32_MIN) {
                // dead end: turn round
                c.fwd = !c.fwd;
                c.s = 0;
                continue;
            }
            bool nf = nx >= 0;
            int ne = nf ? nx : -nx - 1;
            const Edge &ee = edges_[ne];
            // keep the same lane side; adapt to the new road's lanes
            if (ee.oneway == 0 && c.lane < 0) c.lane = ee.hw * 0.5;
            if (ee.oneway == 0) c.lane = std::clamp(c.lane, 0.6, std::max(0.6, ee.hw - 0.8));
            else c.lane = std::clamp(c.lane, -ee.hw + 1.2, ee.hw - 1.2);
            c.edge = ne;
            c.fwd = nf;
            c.prev_node = node;
            c.vmax = speed_for(ee.prio) * (0.85 + 0.3 * rnd());
        }
        place(c);
    }
}

}  // namespace ec
