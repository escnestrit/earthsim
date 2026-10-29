#include <cassert>
#include <cstdio>
#include "../src/core/chunk.hpp"
#include "../src/phys/softcar.hpp"
using namespace ec;

std::shared_ptr<ColChunk> flat(int cx, int cy, double h, bool wall) {
    auto c = std::make_shared<ColChunk>();
    c->cx = cx, c->cy = cy;
    c->grid.cell = 1, c->grid.n = GRID_N, c->grid.x0 = cx * CHUNK, c->grid.y0 = cy * CHUNK;
    c->grid.h.assign((GRID_N + 1) * (GRID_N + 1), (float)h);
    c->layer.assign(SPLAT_N * SPLAT_N, L_ASPHALT);
    if (wall) {  // a wall across x = 120 (in chunk 0)
        float x = 120;
        float a = -200, b = -10; float t[] = {x, a, (float)h - 1, x, b, (float)h - 1, x, b, (float)h + 3, x, a, (float)h - 1, x, b, (float)h + 3, x, a, (float)h + 3};
        c->tris.assign(t, t + 18);
    }
    return c;
}

int main() {
    CollisionWorld w;
    for (int j = -1; j <= 0; j++)
        for (int i = -1; i <= 1; i++) w.add(flat(i, j, 10, i == 0 && j == -1));
    CarParams p;
    SoftCar car(p);
    car.place(w, -150, -100, 20, 0, 0);
    CarInput in;
    for (int i = 0; i < 2000; i++) car.step(0.001, in, w);  // settle 2 s
    printf("settled: z %.3f (ground 10) com %.3f up %.3f %.3f %.3f\n", car.body.nodes[car.hubs[0]].p.z, car.pos.z, car.up.x, car.up.y, car.up.z);
    in.throttle = 1;
    double t100 = -1;
    for (int i = 0; i < 12000; i++) {
        car.step(0.001, in, w);
        if (t100 < 0 && car.forward_speed() > 27.78) t100 = i * 0.001;
    }
    printf("after 12 s full throttle: %.1f km/h, 0-100 %.2f s, gear %d rpm %.0f, x %.1f diverged %d\n", car.forward_speed() * 3.6, t100, car.drive.gear, car.drive.rpm(), car.pos.x, car.body.diverged);
    int broken;
    double dmg = car.damage(&broken);
    printf("damage before the wall: %.3f m, %d broken\n", dmg, broken);
    // hit the wall at x=120 (car is at y=-100: chunk (0,-1) wall spans y -50..50 local?)
    printf("pos %.1f %.1f\n", car.pos.x, car.pos.y);
    in.throttle = 1;
    double maxx = -1e9;
    for (int i = 0; i < 6000; i++) {
        car.step(0.001, in, w);
        for (auto &n : car.body.nodes) maxx = std::max(maxx, n.p.x);
    }
    dmg = car.damage(&broken);
    printf("after the wall: max node x %.2f (wall 120), damage %.2f m, %d broken, speed %.1f, diverged %d\n", maxx, dmg, broken, car.forward_speed() * 3.6, car.body.diverged);
    return car.body.diverged ? 1 : 0;
}
