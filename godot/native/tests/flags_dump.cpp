#include <cstdio>
#include <fstream>
#include <sstream>
#include <set>
#include "../src/core/mvt.hpp"
using namespace ec;
int main(int argc, char **argv) {
    std::ifstream f(argv[1], std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    std::vector<mvt::Layer> L;
    mvt::decode(ss.str(), L);
    std::set<std::string> seen;
    for (auto &l : L) {
        if (l.name != "segment") continue;
        for (auto &ft : l.features) {
            std::string fl = ft.str("road_flags");
            if (fl.empty() || seen.count(fl)) continue;
            seen.insert(fl);
            printf("%s | %s\n", ft.str("class").c_str(), fl.c_str());
        }
    }
    for (auto &l : L) if (l.name == "segment" && !l.features.empty()) { for (auto &p : l.features[0].props) printf("key %s\n", p.first.c_str()); break; }
}
