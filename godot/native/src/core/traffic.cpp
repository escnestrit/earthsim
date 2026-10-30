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
const double LAT_ACC = 2.3, BRAKE = 2.6, ACCEL = 1.8, WHEELBASE = 2.7;
}  // namespace

void Traffic::set_roads(const std::vector<LaneSeg> &segs) {
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
    // re-attach routes by edge endpoints; a route is cut at the first edge that no longer exists
    std::unordered_map<uint64_t, int> by_ends;
    for (int i = 0; i < (int)E.size(); i++) by_ends[qk(E[i].pa) * 31 + qk(E[i].pb)] = i;
    std::vector<TrafficCar> keep;
    for (auto &c : cars_) {
        std::deque<std::pair<int, bool>> r;
        for (auto &[id, f] : c.route) {
            if (id < 0 || id >= (int)edges_.size()) break;
            auto it = by_ends.find(qk(edges_[id].pa) * 31 + qk(edges_[id].pb));
            if (it == by_ends.end()) break;
            r.push_back({it->second, f});
        }
        if (r.empty()) continue;
        c.route = std::move(r);
        c.edge = c.route.front().first;
        keep.push_back(std::move(c));
    }
    edges_ = std::move(E);
    node_edges_ = std::move(NE);
    cars_ = std::move(keep);
}

double Traffic::lane_for(const Edge &e, int idx, int lanes_dir) const {
    double lw = e.oneway ? 2 * e.hw / lanes_dir : e.hw / lanes_dir;
    return e.oneway ? -e.hw + lw * (idx + 0.5) : lw * (idx + 0.5);
}

bool Traffic::extend(TrafficCar &c) {
    auto [cur, cf] = c.route.back();
    const Edge &ce = edges_[cur];
    int node = cf ? ce.b : ce.a;
    V3 d = cf ? ce.pb - ce.pa : ce.pa - ce.pb;
    d = d * (1 / std::max(1e-9, d.len()));
    std::vector<std::pair<double, std::pair<int, bool>>> opts;
    for (int id : node_edges_[node]) {
        if (id == cur) continue;
        const Edge &e = edges_[id];
        bool fwd = e.a == node;
        if ((fwd && e.oneway < 0) || (!fwd && e.oneway > 0)) continue;
        if (e.prio < 2 && ce.prio >= 2) continue;  // stay off service roads
        V3 nd = fwd ? e.pb - e.pa : e.pa - e.pb;
        nd = nd * (1 / std::max(1e-9, nd.len()));
        double straight = d.x * nd.x + d.y * nd.y;
        if (straight < -0.2) continue;
        double w = 0.15 + std::max(0.0, straight) * 2.5 + (e.prio >= ce.prio ? 0.6 : 0);
        opts.push_back({w, {id, fwd}});
    }
    if (opts.empty()) return false;
    double tot = 0;
    for (auto &o : opts) tot += o.first;
    double r = rnd() * tot;
    for (auto &o : opts)
        if ((r -= o.first) <= 0) {
            c.route.push_back(o.second);
            return true;
        }
    c.route.push_back(opts.back().second);
    return true;
}

V3 Traffic::route_point(const TrafficCar &c, double ahead, double lane, V2 *dir) const {
    double s = c.s + ahead;
    size_t k = 0;
    while (k + 1 < c.route.size() && s > edges_[c.route[k].first].len) s -= edges_[c.route[k].first].len, k++;
    const Edge &e = edges_[c.route[k].first];
    bool f = c.route[k].second;
    V3 a = f ? e.pa : e.pb, b = f ? e.pb : e.pa;
    double t = std::clamp(s / e.len, 0.0, 1.0);
    V3 p = a + (b - a) * t;
    V2 d = V2(b.x - a.x, b.y - a.y).norm();
    if (dir) *dir = d;
    return V3(p.x + d.y * lane, p.y - d.x * lane, p.z);
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
        bool f = e.oneway > 0 ? true : e.oneway < 0 ? false : rnd() < 0.5;
        c.route.push_back({id, f});
        c.edge = id;
        c.s = rnd() * e.len;
        c.aggr = 0.85 + 0.3 * rnd();
        c.vmax = speed_for(e.prio) * c.aggr;
        c.v = c.vmax * 0.7;
        int lanes_dir = e.oneway ? std::max(1, e.lanes) : std::max(1, e.lanes / 2);
        c.lane = c.lane_target = lane_for(e, (int)(rnd() * lanes_dir), lanes_dir);
        c.col = COLS[(int)(rnd() * 10) % 10];
        while (c.route.size() < 40 && extend(c)) {}
        V2 dir;
        c.pos = route_point(c, 0, c.lane, &dir);
        c.yaw = std::atan2(dir.y, dir.x);
        c.id = next_id_++;
        cars_.push_back(c);
        return true;
    }
    return false;
}

void Traffic::step(double dt, V3 player, V3 pvel) {
    (void)pvel;
    cars_.erase(std::remove_if(cars_.begin(), cars_.end(), [&](const TrafficCar &c) { return std::hypot(c.pos.x - player.x, c.pos.y - player.y) > radius + 80 || c.route.empty(); }), cars_.end());
    for (int k = 0; k < 3 && (int)cars_.size() < target; k++) spawn(player);
    for (size_t i = 0; i < cars_.size(); i++) {
        TrafficCar &c = cars_[i];
        // keep ~120 m of route committed ahead
        double ahead_len = -c.s;
        for (auto &r : c.route) ahead_len += edges_[r.first].len;
        bool dead_end = false;
        while (ahead_len < 120 && c.route.size() < 80) {
            if (!extend(c)) {
                dead_end = true;
                break;
            }
            ahead_len += edges_[c.route.back().first].len;
        }
        const Edge &e = edges_[c.route.front().first];
        // lane on this road: keep the index from the right, clamped to what this road has
        int lanes_dir = e.oneway ? std::max(1, e.lanes) : std::max(1, e.lanes / 2);
        double lt = c.lane_target;
        lt = std::clamp(lt, lane_for(e, 0, lanes_dir), lane_for(e, lanes_dir - 1, lanes_dir));
        c.lane_target = lt;
        c.lane += std::clamp(c.lane_target - c.lane, -0.9 * dt, 0.9 * dt);  // gradual lane change

        // speed plan: bends within the braking distance ahead (heading change over 10 m windows)
        double vt = speed_for(e.prio) * c.aggr;
        V2 d0;
        route_point(c, 0, 0, &d0);
        double prev_ang = std::atan2(d0.y, d0.x);
        for (double a = 5; a < std::min(ahead_len, 90.0); a += 5) {
            V2 d1;
            route_point(c, a, 0, &d1);
            V2 d2;
            route_point(c, a + 10, 0, &d2);
            double turn = std::abs(std::remainder(std::atan2(d2.y, d2.x) - std::atan2(d1.y, d1.x), 2 * PI));
            double kappa = turn / 10.0;
            double vb = kappa > 1e-4 ? std::sqrt(LAT_ACC * c.aggr / kappa) : 1e9;
            double allowed = std::sqrt(vb * vb + 2 * BRAKE * std::max(0.0, a - 2));
            vt = std::min(vt, allowed);
            (void)prev_ang;
        }
        if (dead_end) vt = std::min(vt, std::sqrt(2 * BRAKE * std::max(0.0, ahead_len - 4)));
        // gap to the nearest obstacle ahead along the heading
        V2 hd(std::cos(c.yaw), std::sin(c.yaw));
        double gap = 1e9;
        auto check = [&](V3 p, double their_v) {
            double dx = p.x - c.pos.x, dy = p.y - c.pos.y;
            double ahead = dx * hd.x + dy * hd.y, side = std::abs(-dx * hd.y + dy * hd.x);
            if (ahead > 0.5 && ahead < 50 && side < 2.0 && std::abs(p.z - c.pos.z) < 4) gap = std::min(gap, ahead - 4.7 + their_v * 0.6);
        };
        for (size_t j = 0; j < cars_.size(); j++)
            if (j != i) check(cars_[j].pos, cars_[j].v);
        check(player, 0);
        double headway = 1.3 / c.aggr;
        double vsafe = std::max(0.0, (gap - 2.0) / headway);
        vt = std::min(vt, vsafe);
        double dv = vt - c.v;
        // gentle throttle, firmer braking when something is close
        double brk = gap < 12 ? 6.0 : BRAKE * 1.4;
        c.v = std::max(0.0, c.v + std::clamp(dv, -brk * dt, ACCEL * dt));

        // steering: pure pursuit of a point ahead in the lane
        double look = std::clamp(c.v * 0.9, 4.0, 16.0);
        V3 tgt = route_point(c, look, c.lane);
        double want = std::atan2(tgt.y - c.pos.y, tgt.x - c.pos.x);
        double err = std::remainder(want - c.yaw, 2 * PI);
        double max_rate = std::min({1.1, std::max(0.4, c.v) / WHEELBASE * std::tan(0.6), 3.8 / std::max(0.5, c.v)});
        c.yaw += std::clamp(err * 2.5, -max_rate, max_rate) * dt;
        c.pos.x += std::cos(c.yaw) * c.v * dt;
        c.pos.y += std::sin(c.yaw) * c.v * dt;
        // progress along the route: the projection of the motion on the path, so the car and its path
        // parameter stay together
        V2 pd;
        V3 here = route_point(c, 0, c.lane, &pd);
        double along = (c.pos.x - here.x) * pd.x + (c.pos.y - here.y) * pd.y;
        c.s += std::max(0.0, along);
        // a car that drifted far off its path (rebuilt roads, a crash) is put back on it
        double off = std::hypot(c.pos.x - here.x, c.pos.y - here.y);
        if (off > 3.5) c.v *= std::pow(0.5, dt);  // off its line: slow down to get back on it
        if (off > 14) c.pos = here, c.yaw = std::atan2(pd.y, pd.x), c.resets++, total_resets++;
        // ground height and pitch from the path
        V3 h2 = route_point(c, 3, c.lane);
        c.pos.z = here.z;
        double tp = std::atan2(h2.z - here.z, 3.0);
        c.pitch += (tp - c.pitch) * std::min(1.0, dt * 5);
        // next edges
        while (c.route.size() > 1 && c.s > edges_[c.route.front().first].len) {
            c.s -= edges_[c.route.front().first].len;
            int prev_prio = edges_[c.route.front().first].prio;
            c.route.pop_front();
            const Edge &ne = edges_[c.route.front().first];
            if (ne.prio != prev_prio) c.vmax = speed_for(ne.prio) * c.aggr;
        }
        if (c.route.size() == 1 && c.s > edges_[c.route.front().first].len - 0.5 && c.v < 0.3) {
            // stopped at a dead end: turn round
            auto &f = c.route.front();
            f.second = !f.second;
            c.s = 0;
            c.yaw += PI;
            c.lane = c.lane_target = std::abs(c.lane);
        }
        c.edge = c.route.front().first;
    }
}

}  // namespace ec
