// test only: writes a fake OSM enrich answer with shops on buildings, a bus stop, signs etc. near the start point
#include <cstdio>
#include <fstream>
#include <sstream>
#include "../src/core/overture.hpp"
using namespace ec;
int main(int argc, char **argv) {
    int x = atoi(argv[1]), y = atoi(argv[2]);
    std::string root = "/opt/data/minsk/overture/2026-10-21.0/";
    std::vector<mvt::Layer> L[3];
    const char *th[3] = {"buildings", "transportation", "base"};
    for (int k = 0; k < 3; k++) {
        int z = k == 2 ? 13 : 14, tx = k == 2 ? x >> 1 : x, ty = k == 2 ? y >> 1 : y;
        std::ifstream f(root + th[k] + "/" + std::to_string(z) + "/" + std::to_string(tx) + "/" + std::to_string(ty) + ".mvt", std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        if (!ss.str().empty()) mvt::decode(ss.str(), L[k]);
    }
    RawTile t;
    TileId base{13, x >> 1, y >> 1};
    overture_features({14, x, y}, {14, x, y}, L[0], L[1], L[2], 0, t, &base);
    static const char *KV[][2] = {{"shop", "supermarket"}, {"shop", "bakery"}, {"amenity", "pharmacy"}, {"amenity", "cafe"}, {"shop", "clothes"}, {"shop", "hairdresser"}, {"amenity", "bank"}, {"shop", "mobile_phone"}, {"amenity", "restaurant"}, {"shop", "florist"}};
    static const char *NM[] = {"Продукты №1", "Хлебный дом", "Аптека Здоровье", "Кофейня", "Мода", "Локон", "Банк", "МТС", "Ресторан Минск", "Цветы"};
    printf("{\"elements\":[\n");
    int n = 0;
    for (auto &b : t.buildings) {
        if (b.part || b.rings.empty() || b.rings[0].size() < 4) continue;
        double lat = 0, lon = 0;
        for (auto &p : b.rings[0]) lat += p.lat, lon += p.lon;
        lat /= b.rings[0].size(), lon /= b.rings[0].size();
        if (std::abs(lat - 53.9045) > 0.0025 || std::abs(lon - 27.5615) > 0.004) continue;
        // a point just inside the longest wall
        size_t best = 0;
        double bl = 0;
        auto &r = b.rings[0];
        for (size_t i = 0; i < r.size(); i++) {
            auto &q = r[(i + 1) % r.size()];
            double l = std::hypot((q.lat - r[i].lat) * 111320, (q.lon - r[i].lon) * 66000);
            if (l > bl) bl = l, best = i;
        }
        if (bl < 8) continue;
        auto &a = r[best], &q = r[(best + 1) % r.size()];
        double plat = (a.lat + q.lat) / 2 + (lat - (a.lat + q.lat) / 2) * 0.05, plon = (a.lon + q.lon) / 2 + (lon - (a.lon + q.lon) / 2) * 0.05;
        int k = n % 10;
        if (n++ % 3) continue;
        printf("%s{\"type\":\"node\",\"id\":%d,\"lat\":%.7f,\"lon\":%.7f,\"tags\":{\"%s\":\"%s\",\"name\":\"%s\"}}\n", n > 1 ? "," : "", 1000 + n, plat, plon, KV[k][0], KV[k][1], NM[k]);
    }
    printf("]}\n");
}
