#include <chrono>
#include <cstdio>
#include <thread>
#include <map>
#include "../src/core/streamer.hpp"
using namespace ec;
int main() {
    StreamConfig cfg;
    cfg.cache_dir = "/opt/data/minsk";
    cfg.lat0 = 53.9045, cfg.lon0 = 27.5615;
    cfg.online = false, cfg.enrich = false;
    cfg.detail_radius = 700, cfg.terrain_radius = 900;
    Streamer s(cfg, [](const std::string &, const std::vector<std::string> &, const std::string *, std::string &, int &st, std::string &) { st = 0; return false; });
    s.update(0, 0);
    ChunkResult r;
    for (int i = 0; i < 500; i++) {
        while (s.poll(r)) {}
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (s.status().find("builds 0 (0 queued)") != std::string::npos && i > 50) break;
    }
    V3 pl(0, 0, 0);
    std::vector<double> py;
    double max_rate = 0, max_lat = 0, vsum = 0;
    int n = 0, jumps = 0;
    std::map<int, std::pair<V3, double>> prev;
    int resets = 0, turnarounds = 0;
    const double dt = 1.0 / 60;
    for (int k = 0; k < 60 * 90; k++) {
        auto &cars = s.traffic_step(dt, pl, V3(), 40);
        std::map<int, std::pair<V3, double>> cur;
        for (auto &c : cars) {
            cur[c.id] = {c.pos, c.yaw};
            auto it = prev.find(c.id);
            if (k < 600 || it == prev.end()) continue;
            double rate = std::abs(std::remainder(c.yaw - it->second.second, 2 * PI)) / dt;
            double step = std::hypot(c.pos.x - it->second.first.x, c.pos.y - it->second.first.y);
            if (step > 3) { jumps++; continue; }
            if (rate > 3) { turnarounds++; continue; }
            max_rate = std::max(max_rate, rate);
            max_lat = std::max(max_lat, rate * c.v);
            vsum += c.v, n++;
        }
        prev = cur;
        if (k == 60 * 90 - 1) for (auto &c : cars) resets += c.resets;
    }
    printf("resets %d turnarounds %d\n", s.traffic_resets(), turnarounds);
    printf("cars %zu, mean speed %.1f km/h, max yaw rate %.2f rad/s, max lateral acc %.2f m/s2, jumps %d\n", prev.size(), vsum / std::max(1, n) * 3.6, max_rate, max_lat, jumps);
}
