// Dev tool: load a region and print a summary.
#include <chrono>
#include <cstdio>
#include <map>
#include "region.hpp"
#include "roadnet.hpp"

int main(int argc, char **argv) {
    auto t0 = std::chrono::steady_clock::now();
    es::Region R;
    if (!R.load(argc > 1 ? argv[1] : "data")) {
        for (auto &l : R.log) printf("%s\n", l.c_str());
        return 1;
    }
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    for (auto &l : R.log) printf("%s\n", l.c_str());
    printf("load %.0f ms, bounds %.0f..%.0f x %.0f..%.0f\n", ms, R.bounds.mn.x, R.bounds.mx.x, R.bounds.mn.y, R.bounds.mx.y);
    std::map<std::string, int> cls, pk, ak;
    for (auto &r : R.roads) cls[r.cls + (r.kind == es::RoadKind::Rail ? "(rail)" : "")]++;
    for (auto &c : cls) printf("  road %-20s %d\n", c.first.c_str(), c.second);
    for (auto &p : R.points) pk[p.kind]++;
    for (auto &c : pk)
        if (c.second > 20) printf("  point %-20s %d\n", c.first.c_str(), c.second);
    int ow = 0, br = 0, tu = 0, lv = 0;
    for (auto &r : R.roads) ow += r.oneway != 0, br += !r.bridge.empty(), tu += !r.tunnel.empty(), lv += !r.level.empty();
    printf("  oneway %d bridge %d tunnel %d level %d\n", ow, br, tu, lv);
    es::RoadNet N;
    t0 = std::chrono::steady_clock::now();
    N.build(R);
    ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    for (auto &l : N.log) printf("%s\n", l.c_str());
    printf("roadnet %.0f ms\n", ms);
    std::map<int, int> deg;
    double maxg = 0; int steep = 0, sig = 0;
    for (auto &n : N.nodes) deg[n.edges.size()]++, sig += n.signals;
    for (auto &d : deg) printf("  degree %d: %d\n", d.first, d.second);
    for (auto &e : N.edges) for (size_t k = 0; k + 1 < e.ps.size(); k++) { double g = std::fabs(e.pz[k+1]-e.pz[k]) / std::max(0.1, e.ps[k+1]-e.ps[k]); maxg = std::max(maxg, g); if (g > 0.12) { steep++; if (steep % 300 == 1) printf("   steep %.2f at (%.0f,%.0f) %s br%d tu%d lv%d s=%.0f/%.0f offA %.1f offB %.1f gnd %.1f\n", g, e.at(e.ps[k]).x, e.at(e.ps[k]).y, e.cls.c_str(), e.bridge, e.tunnel, e.level, e.ps[k], e.len, N.nodes[e.a].off, N.nodes[e.b].off, R.dem.sample(e.at(e.ps[k]))); } }
    printf("  signals %d, max grade %.3f, samples >12%%: %d\n", sig, maxg, steep);
    for (double y : {-2000.0, 0.0, 2000.0}) {
        for (double x : {-3000.0, -1000.0, 0.0, 1000.0, 3000.0}) printf(" %6.1f", R.dem.sample({x, y}));
        printf("\n");
    }
}
