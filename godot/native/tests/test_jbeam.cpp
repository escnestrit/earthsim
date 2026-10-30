#include <chrono>
#include <cstdio>
#include <cstdlib>
#include "../src/core/chunk.hpp"
#include "../src/jbeam/jvisual.hpp"
using namespace ec;

int main(int argc, char **argv) {
    auto t0 = std::chrono::steady_clock::now();
    auto secs = [&] { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };
    Zip z;
    std::string err;
    if (!z.open(argv[1], err)) return printf("%s\n", err.c_str()), 1;
    Library lib;
    lib.add_zip(z, false);
    printf("parts %zu (%.1f s), warnings %zu\n", lib.parts.size(), secs(), lib.warnings.size());
    std::string main;
    auto it = lib.by_type.find("main");
    if (it != lib.by_type.end()) {
        auto v = it->second;
        std::sort(v.begin(), v.end());
        main = v[0];
    }
    printf("main part: %s (of %zu)\n", main.c_str(), it == lib.by_type.end() ? 0 : it->second.size());
    JModel m = build_model(assemble(lib, main, {}));
    printf("nodes %zu beams %zu skipped %d parts %zu missing %zu\n", m.nodes.size(), m.beams.size(), m.skipped_beams, m.asm_.parts.size(), m.asm_.missing.size());
    for (size_t i = 0; i < m.asm_.warnings.size() && i < 5; i++) printf("  w: %s\n", m.asm_.warnings[i].c_str());
    JVehicle v;
    if (!JVehicle::from_model(m, v, err)) return printf("vehicle: %s\n", err.c_str()), 1;
    for (auto &n : v.notes) printf("  note: %s\n", n.c_str());
    printf("mass %.0f kg, wheels %zu, hydros %zu, %.1f s\n", v.body.total_mass(), v.wheels.size(), v.hydros.size(), secs());
    CollisionWorld w;
    for (int j = -1; j <= 0; j++)
        for (int i = -1; i <= 1; i++) {
            auto c = std::make_shared<ColChunk>();
            c->cx = i, c->cy = j;
            c->grid.cell = 1, c->grid.n = GRID_N, c->grid.x0 = i * CHUNK, c->grid.y0 = j * CHUNK;
            c->grid.h.assign((GRID_N + 1) * (GRID_N + 1), 10.0f);
            c->layer.assign(SPLAT_N * SPLAT_N, L_ASPHALT);
            w.add(c);
        }
    if (getenv("SUBDT")) v.sub_dt = atof(getenv("SUBDT"));
    v.place(w, -150, -100, 10, 0, 0);
    CarInput in;
    double ts = secs();
    for (int i = 0; i < 120; i++) v.step(1.0 / 60, in, w);
    printf("settled 2 s (%.2f s cpu): z %.2f, diverged %d, damage %.3f\n", secs() - ts, v.pos.z, v.body.diverged, v.damage());
    in.throttle = 1;
    ts = secs();
    double t100 = -1;
    for (int i = 0; i < 600; i++) {
        v.step(1.0 / 60, in, w);
        if (t100 < 0 && v.forward_speed() > 27.78) t100 = i / 60.0;
    }
    printf("10 s throttle (%.2f s cpu): %.1f km/h, 0-100 %.1f s, gear %d, diverged %d, damage %.3f\n", secs() - ts, v.forward_speed() * 3.6, t100, v.drive.gear, v.body.diverged, v.damage());
    in.throttle = 0.3;
    in.steer = 0.4;
    for (int i = 0; i < 300; i++) v.step(1.0 / 60, in, w);
    printf("turning: speed %.1f, yaw fwd %.2f %.2f, diverged %d\n", v.forward_speed() * 3.6, v.fwd.x, v.fwd.y, v.body.diverged);
    CarVisual vis;
    Zip *common = nullptr;
    Zip cz;
    if (argc > 2 && cz.open(argv[2], err)) common = &cz;
    ts = secs();
    if (!load_visual(z, common, m, v, vis, err)) printf("visual: %s\n", err.c_str());
    size_t tris = 0;
    for (auto &p : vis.parts)
        for (auto &s : p.subs) tris += s.idx.size() / 3;
    printf("visual: %zu parts, %zu tris, %zu props, %zu materials, %zu textures (%.1f s)\n", vis.parts.size(), tris, vis.props.size(), vis.materials.size(), vis.textures.size(), secs() - ts);
    for (size_t i = 0; i < vis.notes.size() && i < 8; i++) printf("  vis: %s\n", vis.notes[i].c_str());
}
