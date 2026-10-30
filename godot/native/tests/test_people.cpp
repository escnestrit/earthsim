#include <chrono>
#include <cstdio>
#include <thread>
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
    for (int k = 0; k < 600; k++) s.people_step(0.05, pl, 60);
    auto &w = s.people_step(0.05, pl, 60);
    printf("%zu walkers\n", w.size());
    for (size_t i = 0; i < w.size() && i < 6; i++) printf("  %.1f %.1f %.1f yaw %.2f\n", w[i].pos.x, w[i].pos.y, w[i].pos.z, w[i].yaw);
}
