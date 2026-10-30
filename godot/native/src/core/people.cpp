#include "people.hpp"
#include <algorithm>
#include <unordered_map>

namespace ec {

namespace {
uint64_t qk(V3 p) {
    int64_t i = (int64_t)std::llround(p.x * 4), j = (int64_t)std::llround(p.y * 4);
    return ((uint64_t)(uint32_t)i << 32) | (uint32_t)j;
}
const Col SHIRTS[] = {{0.15f, 0.2f, 0.45f, 1}, {0.6f, 0.1f, 0.1f, 1}, {0.85f, 0.85f, 0.82f, 1}, {0.1f, 0.1f, 0.1f, 1}, {0.3f, 0.45f, 0.25f, 1}, {0.55f, 0.45f, 0.3f, 1}, {0.45f, 0.45f, 0.5f, 1}, {0.7f, 0.55f, 0.15f, 1}};
}  // namespace

void Crowd::set_walks(const std::vector<std::pair<V3, V3>> &segs) {
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
    for (auto &[a, b] : segs) {
        double l = std::hypot(b.x - a.x, b.y - a.y);
        if (l < 0.05) continue;
        Edge e{node(a), node(b), a, b, l};
        if (e.a == e.b) continue;
        int id = (int)E.size();
        E.push_back(e);
        NE[e.a].push_back(id);
        NE[e.b].push_back(id);
    }
    // keep walkers that can be re-attached by position
    std::unordered_map<uint64_t, int> by_ends;
    for (int i = 0; i < (int)E.size(); i++) by_ends[qk(E[i].pa) * 31 + qk(E[i].pb)] = i;
    std::vector<Walker> keep;
    for (auto &w : w_) {
        if (w.edge < 0 || w.edge >= (int)e_.size()) continue;
        auto it = by_ends.find(qk(e_[w.edge].pa) * 31 + qk(e_[w.edge].pb));
        if (it == by_ends.end()) continue;
        w.edge = it->second;
        keep.push_back(w);
    }
    e_ = std::move(E);
    ne_ = std::move(NE);
    w_ = std::move(keep);
}

void Crowd::place(Walker &w) {
    const Edge &e = e_[w.edge];
    double t = std::clamp(w.s / e.len, 0.0, 1.0);
    V3 a = w.fwd ? e.pa : e.pb, b = w.fwd ? e.pb : e.pa;
    V3 p = a + (b - a) * t;
    V2 d(b.x - a.x, b.y - a.y);
    d = d.norm();
    V2 r(d.y, -d.x);
    w.pos = V3(p.x + r.x * w.side, p.y + r.y * w.side, p.z);
    double yaw = std::atan2(d.y, d.x);
    double dy = std::remainder(yaw - w.yaw, 2 * PI);
    w.yaw += dy * 0.25;
}

void Crowd::step(double dt, V3 player) {
    w_.erase(std::remove_if(w_.begin(), w_.end(), [&](const Walker &w) { return std::hypot(w.pos.x - player.x, w.pos.y - player.y) > radius + 40; }), w_.end());
    for (int k = 0; k < 4 && (int)w_.size() < target && !e_.empty(); k++) {
        int id = (int)(rnd() * e_.size());
        V3 m = (e_[id].pa + e_[id].pb) * 0.5;
        double d = std::hypot(m.x - player.x, m.y - player.y);
        if (d > radius || d < 25) continue;
        Walker w;
        w.edge = id;
        w.fwd = rnd() < 0.5;
        w.s = rnd() * e_[id].len;
        w.v = 1.1 + rnd() * 0.5;
        w.side = (rnd() - 0.5) * 0.8;
        w.col = SHIRTS[(int)(rnd() * 8) % 8];
        w.phase = rnd() * 6.28;
        w.scale = 0.9 + rnd() * 0.17;
        place(w);
        w.yaw = std::atan2((w.fwd ? e_[id].pb : e_[id].pa).y - w.pos.y, (w.fwd ? e_[id].pb : e_[id].pa).x - w.pos.x);
        w_.push_back(w);
    }
    for (auto &w : w_) {
        // step aside / stop for the player's car
        double dx = player.x - w.pos.x, dy = player.y - w.pos.y;
        double v = std::hypot(dx, dy) < 3.0 ? 0 : w.v;
        w.s += v * dt;
        w.phase += v * dt * 3.2;
        int guard = 0;
        while (w.s > e_[w.edge].len && guard++ < 6) {
            w.s -= e_[w.edge].len;
            int node = w.fwd ? e_[w.edge].b : e_[w.edge].a;
            std::vector<int> opts;
            for (int id : ne_[node])
                if (id != w.edge) opts.push_back(id);
            if (opts.empty()) {
                w.fwd = !w.fwd;
                continue;
            }
            int ne = opts[(size_t)(rnd() * opts.size()) % opts.size()];
            w.fwd = e_[ne].a == node;
            w.edge = ne;
        }
        place(w);
    }
}

}  // namespace ec
