#include "beamng.hpp"
#include "jtable.hpp"
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace ec {

namespace sfs = std::filesystem;

std::vector<std::string> steam_libraries(const std::string &vdf) {
    std::vector<std::string> out;
    std::istringstream in(vdf);
    std::string line;
    while (std::getline(in, line)) {
        size_t a = line.find("\"path\"");
        if (a == std::string::npos) continue;
        size_t q1 = line.find('"', a + 6);
        size_t q2 = line.rfind('"');
        if (q1 == std::string::npos || q2 <= q1) continue;
        std::string p = line.substr(q1 + 1, q2 - q1 - 1);
        std::string r;
        for (size_t i = 0; i < p.size(); i++) {
            if (p[i] == '\\' && i + 1 < p.size() && p[i + 1] == '\\') i++;
            r += p[i];
        }
        out.push_back(r);
    }
    return out;
}

static std::string vehicles_in(const sfs::path &install) {
    std::error_code ec;
    sfs::path v = install / "content" / "vehicles";
    if (sfs::is_regular_file(v / "common.zip", ec)) return v.u8string();
    return "";
}

std::string find_beamng_vehicles() {
    std::error_code ec;
    if (const char *e = getenv("EARTHSIM_BEAMNG")) {
        sfs::path p(e);
        if (sfs::is_regular_file(p / "common.zip", ec)) return p.u8string();
        std::string v = vehicles_in(p);
        if (!v.empty()) return v;
    }
    std::vector<sfs::path> steams = {"C:\\Program Files (x86)\\Steam", "C:\\Program Files\\Steam"};
    for (char d : std::string("CDEFG")) {
        std::string D(1, d);
        for (const char *s : {":\\Steam", ":\\steam", ":\\SteamLibrary", ":\\Games\\Steam"}) steams.push_back(D + s);
    }
    if (const char *h = getenv("HOME")) steams.push_back(sfs::path(h) / ".steam/steam");
    std::vector<sfs::path> libs;
    for (auto &s : steams) {
        libs.push_back(s);
        std::ifstream f(s / "steamapps" / "libraryfolders.vdf");
        if (f) {
            std::stringstream ss;
            ss << f.rdbuf();
            for (auto &l : steam_libraries(ss.str())) libs.push_back(l);
        }
    }
    for (auto &l : libs) {
        std::string v = vehicles_in(l / "steamapps" / "common" / "BeamNG.drive");
        if (!v.empty()) return v;
    }
    return "";
}

}  // namespace ec

namespace ec {

bool default_config(Zip &z, VehicleConfig &out, std::string &err) {
    // vehicles/<model>/info.json
    const ZipEntry *info = nullptr;
    for (auto &e : z.entries) {
        if (e.name.size() > 10 && e.name.compare(e.name.size() - 10, 10, "/info.json") == 0 && std::count(e.name.begin(), e.name.end(), '/') == 2 && e.name.rfind("vehicles/", 0) == 0) {
            info = &e;
            break;
        }
    }
    if (!info) {
        err = "no vehicles/<model>/info.json";
        return false;
    }
    std::string dir = info->name.substr(0, info->name.size() - 9);  // vehicles/<model>/
    out.model = dir.substr(9, dir.size() - 10);
    std::string txt;
    Json j;
    if (!z.read(*info, txt, err) || !jbeam_parse(txt, j, err)) return false;
    std::string pc;
    if (const Json *d = j.get("default_pc"); d && d->kind == Json::Str) pc = d->s;
    if (const Json *n = j.get("Name"); n && n->kind == Json::Str) out.name = n->s;
    if (pc.empty()) {
        // no default named: the first .pc in the folder
        for (auto &e : z.entries)
            if (e.name.rfind(dir, 0) == 0 && e.name.size() > 3 && e.name.compare(e.name.size() - 3, 3, ".pc") == 0 && e.name.find('/', dir.size()) == std::string::npos) {
                pc = e.name.substr(dir.size(), e.name.size() - dir.size() - 3);
                break;
            }
        if (pc.empty()) {
            err = "no configuration (.pc) files";
            return false;
        }
    }
    const ZipEntry *pe = z.find(dir + pc + ".pc");
    if (!pe) {
        err = dir + pc + ".pc not in the archive";
        return false;
    }
    Json c;
    if (!z.read(*pe, txt, err) || !jbeam_parse(txt, c, err)) return false;
    if (const Json *m = c.get("mainPartName"); m && m->kind == Json::Str) out.main_part = m->s;
    if (const Json *p = c.get("parts"); p && p->kind == Json::Obj)
        for (auto &[k, v] : p->o)
            if (v.kind == Json::Str) out.parts[k] = v.s;
    if (const Json *v = c.get("vars"); v && v->kind == Json::Obj)
        for (auto &[k, x] : v->o) out.vars[k] = x;
    return true;
}

}  // namespace ec
