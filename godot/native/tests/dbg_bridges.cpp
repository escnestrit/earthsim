#include <cstdio>
#include <fstream>
#include <sstream>
#include "../src/core/overture.hpp"
using namespace ec;
int main() {
    LocalFrame fr(53.9045, 27.5615);
    TileId t = tile_of(53.9045 + 190 / 111320.0, 27.5615 - 140 / 65600.0, 14);
    std::vector<mvt::Layer> L[3];
    std::ifstream f("/opt/data/minsk/overture/2026-10-21.0/transportation/14/" + std::to_string(t.x) + "/" + std::to_string(t.y) + ".mvt", std::ios::binary);
    std::stringstream ss; ss << f.rdbuf();
    mvt::decode(ss.str(), L[1]);
    RawTile rt;
    overture_features(t, t, L[0], L[1], L[2], 0, rt);
    for (auto &r : rt.roads) if (r.bridge) {
        V2 a = fr.to_local(r.pts.front().lat, r.pts.front().lon), b = fr.to_local(r.pts.back().lat, r.pts.back().lon);
        if (std::hypot((a.x+b.x)/2+130,(a.y+b.y)/2-190) < 200) printf("road %s w%.1f layer %d (%.0f,%.0f)-(%.0f,%.0f)\n", r.cls.c_str(), r.width, r.layer, a.x,a.y,b.x,b.y);
    }
    for (auto &r : rt.paths) if (r.bridge) {
        V2 a = fr.to_local(r.pts.front().lat, r.pts.front().lon), b = fr.to_local(r.pts.back().lat, r.pts.back().lon);
        if (std::hypot((a.x+b.x)/2+130,(a.y+b.y)/2-190) < 200) printf("path w%.1f (%.0f,%.0f)-(%.0f,%.0f)\n", r.width, a.x,a.y,b.x,b.y);
    }
}
