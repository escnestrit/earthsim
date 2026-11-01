// Checks the procedural car: consistent triangle orientation, outward faces, sane size,
// dents bounded and local, scratches only change paint, repair restores the shape.
#include "ttgeom.hpp"
#include <cstdio>
#include <map>
#include <utility>
using namespace ttg;
static int fails = 0;
#define CHECK(c, ...) do { bool ok_ = (c); printf("[%s] ", ok_ ? "PASS" : "FAIL"); printf(__VA_ARGS__); printf("\n"); if (!ok_) fails++; } while (0)

static int bad_edges(const Mesh &m) {
    std::map<std::pair<int,int>, int> e;
    for (const Tri &t : m.tris) { e[{t.a,t.b}]++; e[{t.b,t.c}]++; e[{t.c,t.a}]++; }
    int bad = 0;
    for (auto &[k, n] : e) if (n > 1) bad++;  // same directed edge twice = flipped neighbour
    return bad;
}
static double outward_ratio(const Mesh &m, V ref) {
    int out = 0, n = 0;
    for (const Tri &t : m.tris) {
        V a = m.p(t.a), b = m.p(t.b), c = m.p(t.c), ce = (a + b + c) * (1.0 / 3);
        V nn = cross(c - a, b - a);
        V r{ce.x, ref.y, ref.z};
        if (len(nn) < 1e-9) continue;
        n++; if (dot(nn, ce - r) > 0) out++;
    }
    return double(out) / n;
}
int main() {
    CarGeom g; build_car(g); Mesh w; build_wheel(w);
    const char *names[] = {"body", "hood", "boot lid", "front bumper", "rear bumper", "wheel"};
    Mesh *all[] = {&g.body, &g.hood, &g.trunk, &g.bumper_f, &g.bumper_r, &w};
    for (int i = 0; i < 6; i++) CHECK(bad_edges(*all[i]) == 0 && !all[i]->tris.empty(), "%s: %zu triangles, orientation consistent (%d flipped edges)", names[i], all[i]->tris.size(), bad_edges(*all[i]));
    CHECK(outward_ratio(g.hood, V{0, 0, 40}) > 0.99, "hood faces up/out (%.3f)", outward_ratio(g.hood, V{0, 0, 40}));
    CHECK(outward_ratio(g.trunk, V{0, 0, 40}) > 0.99, "boot lid faces out (%.3f)", outward_ratio(g.trunk, V{0, 0, 40}));
    V lo, hi; g.body.bounds(lo, hi);
    V flo, fhi; g.bumper_f.bounds(flo, fhi); V rlo, rhi; g.bumper_r.bounds(rlo, rhi);
    CHECK(hi.z > 135 && hi.z < 150 && lo.z > 15 && hi.y < 106 && fhi.x > 220 && rlo.x < -220, "size: length %.0f, width %.0f (with mirrors), height %.0f cm", fhi.x - rlo.x, hi.y - lo.y, hi.z);
    // wheel fits inside its arch: arch top 73 cm over the axle, wheel top 33 + 33
    CHECK(WHEEL_R * 2 < 73, "wheel (top %.0f cm) clears the arch (73 cm)", WHEEL_R * 2);
    // a hard frontal hit
    Mesh body = g.bumper_f;
    double moved = body.dent(V{226, 0, 40}, V{-1, 0, 0}, 30, 80, 45, 1.0);
    double maxoff = 0, far = 0;
    for (size_t i = 0; i < body.base.size(); i++) {
        maxoff = std::max(maxoff, len(body.off[i]));
        if (len(body.base[i] - V{226, 0, 40}) > 80) far = std::max(far, len(body.off[i]));
    }
    CHECK(moved > 0 && maxoff <= 45.0 + 1e-6 && far == 0, "frontal dent: max %.1f cm (cap 45), nothing moves outside the radius", maxoff);
    int pushed_back = 0, n = 0;
    for (size_t i = 0; i < body.base.size(); i++) if (len(body.off[i]) > 1) { n++; if (body.off[i].x < 0) pushed_back++; }
    CHECK(n > 0 && pushed_back > n * 0.9, "metal moves into the car (%d of %d points)", pushed_back, n);
    // repeated hits stay bounded
    for (int k = 0; k < 20; k++) body.dent(V{226, 0, 40}, V{-1, 0, 0}, 38, 100, 45, k);
    double m2 = 0; for (auto &o : body.off) m2 = std::max(m2, len(o));
    CHECK(m2 <= 45.0 + 1e-6, "20 more hits: still capped (%.1f cm)", m2);
    body.repair(1.0);
    double rest = 0; for (auto &o : body.off) rest = std::max(rest, len(o));
    CHECK(rest == 0, "repair restores the shape");
    Mesh side = g.body;
    side.scratch(V{0, 92, 60}, 40, 1.0);
    double sc = 0, mv = 0; for (size_t i = 0; i < side.base.size(); i++) sc = std::max(sc, double(side.scrape[i])), mv = std::max(mv, len(side.off[i]));
    CHECK(sc > 0.3 && mv == 0, "scrape removes paint (%.2f) without moving metal", sc);
    printf(fails ? "FAILED: %d\n" : "OK: 0 failed\n", fails);
    return fails != 0;
}
