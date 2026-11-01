// Writes the procedural sedan as a modular car kit (OBJ parts with tuning variants) to test the
// importer: <out>/TT Test Sedan/<Category>/<Variant>/<Variant>.obj (+ .mtl)
#include "ttgeom.hpp"
#include <cstdio>
#include <string>
#include <sys/stat.h>
using namespace ttg;
static void mk(const std::string &p) { std::string s; for (char c : p) { s += c; if (c == '/') mkdir(s.c_str(), 0755); } mkdir(p.c_str(), 0755); }
static const char *MAT[] = {"paint", "glass", "trim", "headlight", "taillight"};
static void obj(const Mesh &m, const std::string &dir, const std::string &name) {
    mk(dir);
    FILE *f = fopen((dir + "/" + name + ".mtl").c_str(), "w");
    const double kd[5][3] = {{0.5, 0.04, 0.03}, {0.02, 0.02, 0.025}, {0.03, 0.03, 0.03}, {0.95, 0.95, 0.9}, {0.8, 0.05, 0.03}};
    for (int s = 0; s < 5; s++) fprintf(f, "newmtl %s\nKd %f %f %f\nNs 200\n\n", MAT[s], kd[s][0], kd[s][1], kd[s][2]);
    fclose(f);
    f = fopen((dir + "/" + name + ".obj").c_str(), "w");
    fprintf(f, "mtllib %s.mtl\n", name.c_str());
    std::vector<V> pn; m.point_normals(pn);
    // OBJ is right-handed Y-up in metres: x forward -> x, y right -> -z, z up -> y
    for (auto &p : m.base) fprintf(f, "v %f %f %f\n", p.x / 100, p.z / 100, -p.y / 100);
    for (auto &n : pn) fprintf(f, "vn %f %f %f\n", n.x, n.z, -n.y);
    for (int s = 0; s < 5; s++) {
        bool any = false;
        for (auto &t : m.tris) if (t.slot == s) { if (!any) fprintf(f, "usemtl %s\n", MAT[s]), any = true;
            // mirrored handedness keeps counter-clockwise = front in the right-handed file
            fprintf(f, "f %d//%d %d//%d %d//%d\n", t.a + 1, t.a + 1, t.b + 1, t.b + 1, t.c + 1, t.c + 1); }
    }
    fclose(f);
}
int main(int argc, char **argv) {
    std::string out = std::string(argc > 1 ? argv[1] : "CarModels") + "/TT Test Sedan";
    CarGeom g; build_car(g);
    obj(g.body, out + "/Chassis/Chassis_stock", "Chassis_stock");
    obj(g.hood, out + "/Bonnet/Bonnet_stock", "Bonnet_stock");
    obj(g.trunk, out + "/Trunk/Trunk_stock", "Trunk_stock");
    obj(g.bumper_f, out + "/Front Bumper/BumperFront_stock", "BumperFront_stock");
    obj(g.bumper_r, out + "/Rear Bumper/BumperRear_stock", "BumperRear_stock");
    // tuning variants: a vented bonnet (raised centre) and a deep front splitter bumper
    Mesh h = g.hood; for (auto &p : h.base) if (std::fabs(p.y) < 35) p.z += 2.5 * (1 - std::fabs(p.y) / 35);
    obj(h, out + "/Bonnet/Bonnet_Sport", "Bonnet_Sport");
    Mesh b = g.bumper_f; for (auto &p : b.base) if (p.z < 30) p.z -= 6, p.x += (p.x > 210 ? 4 : 0);
    obj(b, out + "/Front Bumper/BumperFront_Sport", "BumperFront_Sport");
    printf("written %s\n", out.c_str());
}
