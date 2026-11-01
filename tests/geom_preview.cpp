// Builds the car geometry, applies a frontal crash dent and writes OBJ files for previewing.
#include "../unreal/Source/TrueTorque/Geom/ttgeom.hpp"
#include <cstdio>
using namespace ttg;
static void obj(const Mesh &m, FILE *f, int &base, const V &shift) {
    for (size_t i = 0; i < m.base.size(); i++) { V q = m.p(int(i)) + shift; fprintf(f, "v %f %f %f %f\n", q.x, q.y, q.z, double(m.tris.size() ? m.scrape[i] : 0)); }
    for (const Tri &t : m.tris) fprintf(f, "f %d %d %d %d\n", base + t.a + 1, base + t.c + 1, base + t.b + 1, int(t.slot));  // OBJ: ccw
    base += int(m.base.size());
}
static void write(const char *path, const CarGeom &g, const Mesh &w) {
    FILE *f = fopen(path, "w"); int b = 0;
    for (const Mesh *m : {&g.body, &g.hood, &g.trunk, &g.bumper_f, &g.bumper_r}) obj(*m, f, b, V{0,0,0});
    for (int i = 0; i < 4; i++) {
        Mesh m = w; double sy = (i & 1) ? 1 : -1;
        for (auto &p : m.base) { p.y *= sy; p = p + V{(i < 2 ? 1 : -1) * WHEELBASE_HALF, sy * TRACK_HALF, WHEEL_R}; }
        if (sy < 0) for (auto &t : m.tris) std::swap(t.b, t.c);
        for (auto &t : m.tris) t.slot = t.slot == RIM ? 10 : 11;
        obj(m, f, b, V{0,0,0});
    }
    fclose(f);
}
int main() {
    CarGeom g; build_car(g); Mesh w; build_wheel(w);
    size_t pts = 0, tris = 0;
    for (const Mesh *m : {&g.body, &g.hood, &g.trunk, &g.bumper_f, &g.bumper_r, &w}) pts += m->base.size(), tris += m->tris.size();
    printf("points %zu tris %zu (body %zu hood %zu trunk %zu bf %zu br %zu wheel %zu)\n", pts, tris, g.body.tris.size(), g.hood.tris.size(), g.trunk.tris.size(), g.bumper_f.tris.size(), g.bumper_r.tris.size(), w.tris.size());
    write("car.obj", g, w);
    // frontal-offset crash at ~50 km/h: depth from the game's formula
    double dv = 13.9, depth = std::min(38.0, (dv - 2.2) * 2.4), rad = std::min(110.0, 38 + dv * 3.2);
    double moved = 0;
    for (Mesh *m : {&g.body, &g.hood, &g.bumper_f}) moved += m->dent(V{226, 45, 45}, norm(V{-1, -0.15, 0}), depth, rad, 45, 3.7);
    // side scrape along the right doors
    for (Mesh *m : {&g.body}) m->scratch(V{-20, 92, 60}, 60, 0.9);
    printf("dent depth %.1f cm radius %.1f cm moved %.0f\n", depth, rad, moved);
    write("car_dent.obj", g, w);
}
