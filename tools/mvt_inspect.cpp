// Dev tool: print layers, geometry kinds and the most common property values of MVT files.
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include "mvt.hpp"

int main(int argc, char **argv) {
    for (int a = 1; a < argc; a++) {
        std::ifstream f(argv[a], std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        std::string d = ss.str();
        std::vector<es::mvt::Layer> L;
        std::string err;
        if (!es::mvt::decode((const uint8_t *)d.data(), d.size(), L, &err)) printf("ERR %s\n", err.c_str());
        printf("== %s\n", argv[a]);
        for (auto &l : L) {
            std::map<int, int> kinds;
            std::map<std::string, std::map<std::string, int>> vals;
            std::map<std::string, int> keys;
            for (auto &ft : l.features) {
                kinds[int(ft.kind)]++;
                for (auto &p : ft.props) {
                    keys[p.first]++;
                    std::string v = p.second.kind == es::mvt::Value::Str ? p.second.s : p.second.kind == es::mvt::Value::Bool ? (p.second.b ? "T" : "F") : std::to_string(p.second.num());
                    if (v.size() > 70) v = v.substr(0, 70) + "...";
                    vals[p.first][v]++;
                }
            }
            printf(" layer %s: %zu feats extent %d kinds", l.name.c_str(), l.features.size(), l.extent);
            for (auto &k : kinds) printf(" %d:%d", k.first, k.second);
            printf("\n");
            for (auto &k : keys) {
                std::vector<std::pair<int, std::string>> top;
                for (auto &v : vals[k.first]) top.push_back({-v.second, v.first});
                std::sort(top.begin(), top.end());
                printf("   %-28s %6d  [%zu uniq] ", k.first.c_str(), k.second, top.size());
                for (size_t i = 0; i < top.size() && i < 7; i++) printf("%s:%d  ", top[i].second.c_str(), -top[i].first);
                printf("\n");
            }
        }
    }
}
