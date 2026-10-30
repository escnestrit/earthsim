#include <cstdio>
#include <fstream>
#include <sstream>
#include "../src/core/mvt.hpp"
using namespace ec;
int main(int argc, char **argv) {
    std::ifstream f(argv[1], std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    std::vector<mvt::Layer> L;
    mvt::decode(ss.str(), L);
    for (auto &l : L) {
        if (l.name != "segment") continue;
        for (auto &ft : l.features) {
            std::string fl = ft.str("road_flags");
            if (fl.find("is_bridge") == std::string::npos || ft.str("subtype") != "road") continue;
            int mn = 1 << 30, mx = -(1 << 30);
            for (auto &p : ft.parts) for (auto &q : p) mn = std::min({mn, q.first, q.second}), mx = std::max({mx, q.first, q.second});
            printf("%s parts %zu range %d..%d ext %d %s\n", ft.str("class").c_str(), ft.parts.size(), mn, mx, l.extent, fl.c_str());
        }
    }
}
