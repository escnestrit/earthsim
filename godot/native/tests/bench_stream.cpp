#include <chrono>
#include <cstdio>
#include <thread>
#include "../src/core/streamer.hpp"
using namespace ec;
int main() {
    StreamConfig cfg;
    cfg.cache_dir = "/opt/data/minsk";
    cfg.lat0 = 53.9045, cfg.lon0 = 27.5615;
    cfg.online = false;
    cfg.enrich = false;
    cfg.detail_radius = 700, cfg.terrain_radius = 900;
    cfg.build_threads = 1;
    auto t0 = std::chrono::steady_clock::now();
    Streamer s(cfg, [](const std::string &, const std::vector<std::string> &, const std::string *, std::string &, int &st, std::string &e) { st = 0; return false; });
    s.update(0, 0);
    int n = 0, full = 0;
    double first = -1;
    for (int i = 0; i < 6000; i++) {
        ChunkResult r;
        while (s.poll(r)) {
            n++;
            full += !r.terrain_only;
            if (first < 0) first = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (s.status().find("builds 0 (0 queued)") != std::string::npos && n > 10 && s.status().find("net 0 (0") != std::string::npos) break;
    }
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("%d chunks (%d full) in %.1f s with 1 build thread, first after %.2f s\n%s\n", n, full, t, first, s.status().c_str());
}
