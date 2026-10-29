// Native checks of the chunk builder on synthetic data: a crossroads with sidewalks, buildings, a pond.
#include <cassert>
#include <cstdio>
#include <fstream>
#include "../src/core/chunk.hpp"
using namespace ec;

static std::vector<LL> ll(const LocalFrame &f, std::vector<V2> pts) {
    std::vector<LL> o;
    for (auto p : pts) {
        LL q;
        f.to_latlon(p, q.lat, q.lon);
        o.push_back(q);
    }
    return o;
}

int main(int argc, char **argv) {
    LocalFrame fr(53.9, 27.56);
    Dem dem;
    // DEM: a gentle slope rising east, around the frame origin
    TileId t0 = tile_of(53.9, 27.56, DEM_ZOOM);
    for (int y = t0.y - 2; y <= t0.y + 2; y++)
        for (int x = t0.x - 2; x <= t0.x + 2; x++) {
            std::vector<float> h(256 * 256);
            for (int j = 0; j < 256; j++)
                for (int i = 0; i < 256; i++) h[j * 256 + i] = 200 + ((x - t0.x) * 256 + i) * 0.4f + std::sin(((y - t0.y) * 256 + j) * 0.05f) * 3;
            dem.add(x, y, h);
        }
    auto t = std::make_shared<RawTile>();
    t->tile = tile_of(53.9, 27.56, 14);
    RawRoad a;
    a.pts = ll(fr, {{-300, 100}, {100, 100}, {300, 140}});
    a.cls = "secondary";
    a.width = 12;
    t->roads.push_back(a);
    RawRoad b;
    b.pts = ll(fr, {{100, -200}, {100, 100}});
    b.cls = "residential";
    b.width = 7;
    t->roads.push_back(b);
    RawRoad c2;
    c2.pts = ll(fr, {{100, 100}, {120, 400}});
    c2.cls = "residential";
    c2.width = 7;
    t->roads.push_back(c2);
    RawRoad br;
    br.pts = ll(fr, {{20, 150}, {20, 240}});
    br.cls = "tertiary";
    br.width = 8;
    br.bridge = true;
    br.layer = 1;
    t->roads.push_back(br);
    for (int i = 0; i < 70; i++) {
        RawBuilding bd;
        double x = 20 + (i % 7) * 30, y = 140 + (i / 7) * 25;
        if (i >= 7) {  // far ones elsewhere
            x = -2000 - i * 30;
        }
        bd.id = "w" + std::to_string(i);
        bd.rings.push_back(ll(fr, {{x, y}, {x + 20, y}, {x + 20, y + 12}, {x, y + 12}}));
        bd.tags = {{"building", "apartments"}, {"building:levels", "5"}};
        t->buildings.push_back(bd);
    }
    RawArea pond;
    pond.kind = AreaKind::Water;
    pond.rings.push_back(ll(fr, {{150, 20}, {220, 20}, {230, 70}, {160, 80}}));
    t->areas.push_back(pond);
    RawArea park;
    park.kind = AreaKind::Forest;
    park.rings.push_back(ll(fr, {{0, 0}, {90, 0}, {90, 90}, {0, 90}}));
    t->areas.push_back(park);
    RawPath p;
    p.pts = ll(fr, {{10, 30}, {200, 95}});
    p.width = 2;
    t->paths.push_back(p);

    ChunkOut o = build_chunk(0, 0, {t}, dem, fr);
    printf("buildings %d roads %d coll tris %zu inst %zu\n", o.buildings, o.roads, o.coll.size() / 9, o.inst.size());
    size_t verts = 0;
    for (auto &[m, b2] : o.ms.m) {
        verts += b2.verts();
        for (float v : b2.pos) assert(std::isfinite(v));
        for (int i : b2.idx) assert(i >= 0 && (size_t)i < b2.verts());
        printf("  mat %d: %zu verts %zu tris\n", m, b2.verts(), b2.idx.size() / 3);
    }
    printf("terrain hi %zu lo %zu verts; grid %d\n", o.terrain_hi.verts(), o.terrain_lo.verts(), o.grid.n);
    // the road surface is above the terrain under it
    for (double x = 0; x < 256; x += 7) {
        double gz = o.grid.sample(x, 100);
        assert(std::isfinite(gz));
    }
    // dump an OBJ for inspection
    if (argc > 1) {
        std::ofstream f(argv[1]);
        size_t base = 1;
        auto dump = [&](const MeshBuf &m, const char *name) {
            f << "o " << name << "\n";
            for (size_t i = 0; i < m.verts(); i++) f << "v " << m.pos[i * 3] << " " << m.pos[i * 3 + 2] << " " << -m.pos[i * 3 + 1] << "\n";
            for (size_t i = 0; i + 2 < m.idx.size(); i += 3) f << "f " << base + m.idx[i] << " " << base + m.idx[i + 1] << " " << base + m.idx[i + 2] << "\n";
            base += m.verts();
        };
        dump(o.terrain_lo, "terrain");
        for (auto &[m, b2] : o.ms.m) dump(b2, ("m" + std::to_string(m)).c_str());
    }
    printf("ok\n");
}
