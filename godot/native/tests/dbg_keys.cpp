#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include "../src/core/mvt.hpp"
using namespace ec;
int main(int, char **argv) {
    std::ifstream f(argv[1], std::ios::binary);
    std::stringstream ss; ss << f.rdbuf();
    std::vector<mvt::Layer> L;
    mvt::decode(ss.str(), L);
    for (auto &l : L) {
        std::set<std::string> keys;
        for (auto &ft : l.features) for (auto &[k, v] : ft.props) keys.insert(k);
        printf("%s (%zu):", l.name.c_str(), l.features.size());
        for (auto &k : keys) printf(" %s", k.c_str());
        printf("\n");
    }
}
