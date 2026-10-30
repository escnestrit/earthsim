#include <cstdio>
#include <fstream>
#include <sstream>
#include <map>
#include "../src/core/mvt.hpp"
using namespace ec;
int main(int argc, char **argv) {
    std::ifstream f(argv[1], std::ios::binary); std::stringstream ss; ss << f.rdbuf();
    std::vector<mvt::Layer> L; mvt::decode(ss.str(), L);
    for (auto &l : L) { std::map<std::string,int> k; for (auto &ft : l.features) for (auto &p : ft.props) k[p.first]++; printf("== %s\n", l.name.c_str()); for (auto &[a,b] : k) printf("  %s %d\n", a.c_str(), b); }
}
