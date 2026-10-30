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
    double hi = 0, lo = 0, ms = 0, coll = 0, grass = 0, inst = 0;
    int n = 0;
    for (int i = 0; i < 500; i++) {
        while (s.poll(r)) {
            if (r.terrain_only || !r.chunk) continue;
            auto &o = *r.chunk;
            n++;
            auto mb = [](const MeshBuf &m) { return (m.pos.size() + m.nrm.size() + m.uv.size() + m.col.size()) * 4.0 + m.idx.size() * 4.0; };
            hi += mb(o.terrain_hi), lo += mb(o.terrain_lo);
            for (auto &[k, m] : o.ms.m) ms += mb(m);
            coll += o.coll.size() * 4.0;
            for (auto &g : o.grass) grass += g.size() * 4.0;
            inst += o.inst.size();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (s.status().find("builds 0 (0 queued)") != std::string::npos && i > 50) break;
    }
    printf("%d full chunks, per chunk MB: hi %.2f lo %.2f meshes %.2f coll %.2f grass %.2f, inst %.0f\n", n, hi / n / 1e6, lo / n / 1e6, ms / n / 1e6, coll / n / 1e6, grass / n / 1e6, inst / n);
}
