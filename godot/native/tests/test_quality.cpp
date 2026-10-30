// Quality gate over real data: builds the chunks round a point from the game's cache (offline) and checks the
// ground model. Exit code 1 when a check fails, so it can run before any "fixed" claim.
//   test_quality <cache dir> [lat lon] [radius in chunks]
#include <cstdio>
#include <cstdlib>
#include <set>
#include <thread>
#include "../src/core/streamer.hpp"
using namespace ec;

int main(int argc, char **argv) {
    StreamConfig cfg;
    cfg.cache_dir = argc > 1 ? argv[1] : "cache";
    cfg.lat0 = argc > 3 ? atof(argv[2]) : 53.90218, cfg.lon0 = argc > 3 ? atof(argv[3]) : 27.56212;
    int R = argc > 4 ? atoi(argv[4]) : 1;
    cfg.online = false, cfg.enrich = true;
    cfg.detail_radius = (R + 0.5) * CHUNK, cfg.terrain_radius = (R + 1) * CHUNK;
    Streamer s(cfg, [](const std::string &, const std::vector<std::string> &, const std::string *, std::string &, int &st, std::string &e) { st = 0; e = "offline"; return false; });
    s.update(0, 0);
    std::set<std::pair<int, int>> want, got;
    for (int x = -R; x <= R; x++)
        for (int y = -R; y <= R; y++) want.insert({x, y});
    ChunkOut::Qa t;
    for (int i = 0, idle = 0; i < 3000 && got.size() < want.size() && idle < 150; i++, idle++) {
        ChunkResult r;
        while (s.poll(r)) {
            idle = 0;
            if (r.terrain_only || !want.count({r.chunk->cx, r.chunk->cy}) || got.count({r.chunk->cx, r.chunk->cy})) continue;
            got.insert({r.chunk->cx, r.chunk->cy});
            auto &q = r.chunk->qa;
            t.bld += q.bld, t.bld_low += q.bld_low, t.bld_worst = std::max(t.bld_worst, q.bld_worst);
            t.pave += q.pave, t.pave_bad += q.pave_bad, t.pave_worst = std::max(t.pave_worst, q.pave_worst);
        }
        s.update(0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (got.size() * 5 < want.size() * 4) {  // chunks with no cached data are skipped, most must build
        printf("FAIL: built %zu of %zu chunks (%s)\n", got.size(), want.size(), s.status().c_str());
        return 1;
    }
    bool ok = true;
    auto check = [&](const char *name, int bad, int n, double worst, double max_share) {
        double share = n ? double(bad) / n : 0;
        bool pass = share <= max_share;
        ok = ok && pass;
        printf("%s %-34s %5d / %6d bad (%.2f%%), worst %.2f m\n", pass ? "ok  " : "FAIL", name, bad, n, share * 100, worst);
    };
    printf("%zu chunks round %.5f, %.5f\n", got.size(), cfg.lat0, cfg.lon0);
    check("floor below the pavement (>5 cm)", t.bld_low, t.bld, t.bld_worst, 0.0);
    check("pavement edge needs a retaining wall", t.pave_bad, t.pave, t.pave_worst, 0.04);
    return ok ? 0 : 1;
}
