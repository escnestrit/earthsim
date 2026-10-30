// Builds chunks of a pre-fetched region (tools/fetch_region.py layout) and dumps facts / images.
#include <cstdio>
#include <fstream>
#include <thread>
#include "../src/core/streamer.hpp"
using namespace ec;

int main(int argc, char **argv) {
    StreamConfig cfg;
    cfg.cache_dir = argc > 1 ? argv[1] : "/opt/data/minsk";
    cfg.lat0 = 53.9045, cfg.lon0 = 27.5615;
    cfg.online = false;
    cfg.enrich = false;
    cfg.detail_radius = 200, cfg.terrain_radius = 300;
    int want_x = argc > 2 ? atoi(argv[2]) : 0, want_y = argc > 3 ? atoi(argv[3]) : 0;
    Streamer s(cfg, [](const std::string &, const std::vector<std::string> &, const std::string *, std::string &, int &st, std::string &e) { st = 0; e = "offline"; return false; });
    s.update(want_x * CHUNK + 128, want_y * CHUNK + 128);
    for (int i = 0; i < 600; i++) {
        ChunkResult r;
        while (s.poll(r)) {
            if (r.terrain_only || r.chunk->cx != want_x || r.chunk->cy != want_y) continue;
            auto &c = *r.chunk;
            int hist[8] = {0};
            for (auto l : c.layer) hist[l]++;
            printf("chunk %d,%d: buildings %d roads %d inst %zu lanes %zu coll %zu\n", c.cx, c.cy, c.buildings, c.roads, c.inst.size(), c.lanes.size(), c.coll.size() / 9);
            printf("layers:");
            for (int k = 0; k < 8; k++) printf(" %d", hist[k]);
            printf("\n");
            // splat as PPM: dominant layer colour, row 0 = north
            std::ofstream f("/tmp/layer.ppm", std::ios::binary);
            f << "P6 256 256 255\n";
            const unsigned char C[8][3] = {{60, 160, 60}, {20, 90, 20}, {220, 200, 140}, {40, 40, 40}, {200, 120, 120}, {150, 110, 60}, {120, 90, 60}, {80, 200, 80}};
            for (int j = 255; j >= 0; j--)
                for (int i = 0; i < 256; i++) f.write((const char *)C[c.layer[j * 256 + i]], 3);
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("timeout: %s\n", s.status().c_str());
}
