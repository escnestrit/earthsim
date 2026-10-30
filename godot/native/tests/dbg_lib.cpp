#include <cstdio>
#include "../src/jbeam/jmodel.hpp"
#include "../src/jbeam/jtable.hpp"
#include "../src/jbeam/beamng.hpp"
#include "../src/jbeam/zip.hpp"
using namespace ec;
int main(int argc, char **argv) {
    Zip z;
    std::string err;
    z.open(argv[1], err);
    Library lib;
    lib.add_zip(z, false);
    int n = 0;
    for (auto &[k, v] : lib.parts)
        if (k.find(argv[2]) != std::string::npos && n++ < 10) printf("part %s\n", k.c_str());
    for (auto &w : lib.warnings) printf("warn %s\n", w.c_str());
    const ZipEntry *pe = z.find("vehicles/pickup/d15_4wd_A.pc");
    std::string txt;
    z.read(*pe, txt, err);
    Json c;
    bool ok = jbeam_parse(txt, c, err);
    printf("pc parse %d err %s kind %d keys:", ok, err.c_str(), (int)c.kind);
    for (auto &[k, v] : c.o) printf(" %s(%d)", k.c_str(), (int)v.kind);
    printf("\n");
}
