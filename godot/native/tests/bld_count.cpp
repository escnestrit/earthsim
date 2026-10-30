#include <cstdio>
#include <fstream>
#include <sstream>
#include "../src/core/overture.hpp"
using namespace ec;
int main() {
    std::string root = "/opt/data/minsk/overture/2026-10-21.0/";
    int x = 9446, y = 5267;
    std::vector<mvt::Layer> L[3];
    const char *th[3] = {"buildings", "transportation", "base"};
    for (int k = 0; k < 3; k++) {
        int z = k == 2 ? 13 : 14, tx = k == 2 ? x >> 1 : x, ty = k == 2 ? y >> 1 : y;
        std::ifstream f(root + th[k] + "/" + std::to_string(z) + "/" + std::to_string(tx) + "/" + std::to_string(ty) + ".mvt", std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        if (!ss.str().empty()) mvt::decode(ss.str(), L[k]);
    }
    for (auto &l : L[0]) printf("layer %s: %zu features\n", l.name.c_str(), l.features.size());
    RawTile t;
    TileId base{13, x >> 1, y >> 1};
    overture_features({14, x, y}, {14, x, y}, L[0], L[1], L[2], 0, t, &base);
    int parts = 0, hp = 0, bridges = 0;
    for (auto &b : t.buildings) parts += b.part, hp += tagv(b.tags, "earthsim:has_parts") == "yes";
    for (auto &r : t.roads) bridges += r.bridge;
    printf("buildings %zu (parts %d, has_parts %d), roads %zu (bridges %d), paths %zu\n", t.buildings.size(), parts, hp, t.roads.size(), bridges, t.paths.size());
    int wa = 0; for (auto &a : t.areas) wa += a.kind == AreaKind::Water; printf("water areas %d, waterlines %zu\n", wa, t.waterlines.size());
    int tun = 0; for (auto &r : t.roads) { tun += r.tunnel; if (r.tunnel && r.pts.size() > 1) { double L = 0; for (size_t i = 1; i < r.pts.size(); i++) L += std::hypot((r.pts[i].lat - r.pts[i-1].lat) * 111320, (r.pts[i].lon - r.pts[i-1].lon) * 65600); printf("tunnel %s L %.0f at x %.0f z %.0f\n", r.cls.c_str(), L, (r.pts[0].lon - 27.5615) * 111320 * 0.5893, -(r.pts[0].lat - 53.9045) * 111320); } } printf("tunnels %d\n", tun);
    for (auto &r : t.roads) if (r.bridge) printf("bridge %s %s %zu pts at x %.0f z %.0f\n", r.cls.c_str(), r.name.c_str(), r.pts.size(), (r.pts[0].lon - 27.5615) * 111320 * 0.5893, -(r.pts[0].lat - 53.9045) * 111320);
    for (auto &n : t.notes) printf("note %s\n", n.c_str());
}
