// Dev tool: load the world, build chunks, print timings and stats; optionally write an OBJ of a chunk.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include "world.hpp"

using clk = std::chrono::steady_clock;
static double ms(clk::time_point a) { return std::chrono::duration<double, std::milli>(clk::now() - a).count(); }

int main(int argc, char **argv) {
    const char *root = argc > 1 ? argv[1] : "data";
    auto t0 = clk::now();
    es::World W;
    if (!W.load(root)) {
        for (auto &l : W.log) printf("%s\n", l.c_str());
        return 1;
    }
    for (auto &l : W.log) printf("%s\n", l.c_str());
    printf("world load %.0f ms\n", ms(t0));
    int x0, y0, x1, y1;
    W.chunk_range(x0, y0, x1, y1);
    printf("chunks %d..%d x %d..%d\n", x0, x1, y0, y1);
    int cx = argc > 3 ? atoi(argv[2]) : 0, cy = argc > 3 ? atoi(argv[3]) : 0;
    int rad = argc > 4 ? atoi(argv[4]) : 1;
    double tot = 0, worst = 0;
    size_t tris = 0, verts = 0;
    for (int y = cy - rad; y <= cy + rad; y++)
        for (int x = cx - rad; x <= cx + rad; x++) {
            auto t = clk::now();
            auto c = W.build_chunk(x, y);
            double m = ms(t);
            tot += m, worst = std::max(worst, m);
            size_t tr = 0, vv = 0;
            for (auto &mm : c->mats) tr += mm.idx.size() / 3, vv += mm.verts();
            tris += tr, verts += vv;
            printf("chunk %3d %3d: %6.0f ms  %7zu tris %6zu verts  bld %3d edges %3d trees %4zu lamps %3zu signals %2zu col r%zu g%zu s%zu\n", x, y, m, tr, vv, c->stats_buildings, c->stats_edges,
                   c->inst[es::I_TREE].size(), c->inst[es::I_LAMP].size(), c->inst[es::I_SIGNAL].size(), c->col_road.size() / 9, c->col_ground.size() / 9, c->col_solid.size() / 9);
            if (argc > 5 && x == cx && y == cy) {
                FILE *f = fopen(argv[5], "w");
                size_t base = 1;
                for (int k = 0; k < es::M_COUNT; k++) {
                    auto &mm = c->mats[k];
                    fprintf(f, "o mat%d\n", k);
                    for (size_t i = 0; i < mm.verts(); i++) fprintf(f, "v %.3f %.3f %.3f\n", mm.pos[i * 3], mm.pos[i * 3 + 2], -mm.pos[i * 3 + 1]);
                    for (size_t i = 0; i + 2 < mm.idx.size(); i += 3) fprintf(f, "f %zu %zu %zu\n", base + mm.idx[i], base + mm.idx[i + 1], base + mm.idx[i + 2]);
                    base += mm.verts();
                }
                fclose(f);
            }
        }
    printf("total %.0f ms, worst %.0f ms, %zu tris, %zu verts\n", tot, worst, tris, verts);
}
