#include <cstdio>
#include <fstream>
#include <sstream>
#include "../src/core/mvt.hpp"
using namespace ec;
int main(int argc, char **argv) {
    std::ifstream f(argv[1], std::ios::binary); std::stringstream ss; ss << f.rdbuf();
    std::vector<mvt::Layer> L; mvt::decode(ss.str(), L);
    int n = 0;
    for (auto &l : L) if (l.name == "segment") for (auto &ft : l.features) {
        std::string a = ft.str("level_rules"), b = ft.str("speed_limits"), c = ft.str("width_rules");
        if ((!a.empty() || !b.empty() || !c.empty()) && n++ < 14) printf("%s | L %s | S %s | W %s\n", ft.str("class").c_str(), a.c_str(), b.c_str(), c.c_str());
    }
}
