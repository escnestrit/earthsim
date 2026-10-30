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

bool default_config(Zip &z, VehicleConfig &out, std::string &err) { return load_config(z, "", out, err); }

bool load_config(Zip &z, const std::string &want_pc, VehicleConfig &out, std::string &err) {
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
    if (!z.read(*info, txt, err) || !Json::parse(jbeam_normalize(txt), j, &err)) return false;
    std::string pc = want_pc;
    if (const Json *d = j.get("default_pc"); pc.empty() && d && d->kind == Json::Str) pc = d->s;
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
    // a .pc is plain (lenient) JSON: {"format": 2, "model": ..., "parts": {slot: part}, "vars": {...}}
    Json c;
    if (!z.read(*pe, txt, err)) return false;
    if (!Json::parse(jbeam_normalize(txt), c, &err)) return false;
    if (const Json *m = c.get("mainPartName"); m && m->kind == Json::Str) out.main_part = m->s;
    if (const Json *p = c.get("parts"); p && p->kind == Json::Obj) {
        for (auto &[k, v] : p->o)
            if (v.kind == Json::Str) out.parts[k] = v.s;
    } else if (c.kind == Json::Obj) {
        // format 1: slots at the top level
        for (auto &[k, v] : c.o)
            if (v.kind == Json::Str && k != "model" && k != "mainPartName" && k != "licenseName") out.parts[k] = v.s;
    }
    if (const Json *v = c.get("vars"); v && v->kind == Json::Obj)
        for (auto &[k, x] : v->o) out.vars[k] = x;
    return true;
}

}  // namespace ec

namespace ec {

namespace sfs2 = std::filesystem;

std::vector<std::string> list_configs(Zip &z) {
    std::vector<std::string> out;
    VehicleConfig def;
    std::string err;
    std::string first;
    if (default_config(z, def, err)) {
        // the default's name: read info.json again for it
        for (auto &e : z.entries)
            if (e.name.size() > 10 && e.name.compare(e.name.size() - 10, 10, "/info.json") == 0 && std::count(e.name.begin(), e.name.end(), '/') == 2) {
                std::string txt;
                Json j;
                if (z.read(e, txt, err) && Json::parse(jbeam_normalize(txt), j, &err))
                    if (const Json *d = j.get("default_pc"); d && d->kind == Json::Str) first = d->s;
                break;
            }
    }
    if (!first.empty()) out.push_back(first);
    for (auto &e : z.entries) {
        if (e.name.size() < 4 || e.name.compare(e.name.size() - 3, 3, ".pc") != 0 || e.name.rfind("vehicles/", 0) != 0) continue;
        if (std::count(e.name.begin(), e.name.end(), '/') != 2) continue;
        std::string n = e.name.substr(e.name.rfind('/') + 1);
        n = n.substr(0, n.size() - 3);
        if (n != first) out.push_back(n);
    }
    std::sort(out.begin() + (first.empty() ? 0 : 1), out.end());
    return out;
}

std::vector<std::string> find_beamng_mod_dirs() {
    std::vector<sfs2::path> users;
    std::error_code ec;
    // a custom user folder named in the install's startup.ini
    std::string veh = find_beamng_vehicles();
    if (!veh.empty()) {
        sfs2::path install = sfs2::u8path(veh).parent_path().parent_path();
        std::ifstream f(install / "startup.ini");
        std::string line;
        while (f && std::getline(f, line)) {
            size_t k = line.find("UserPath");
            if (k == std::string::npos) continue;
            size_t eq = line.find('=', k);
            if (eq == std::string::npos) continue;
            std::string v = line.substr(eq + 1);
            while (!v.empty() && (v.front() == ' ' || v.front() == '"')) v.erase(v.begin());
            while (!v.empty() && (v.back() == ' ' || v.back() == '"' || v.back() == '\r')) v.pop_back();
            if (!v.empty()) users.push_back(sfs2::u8path(v));
        }
    }
    if (const char *e = getenv("EARTHSIM_BEAMNG_USER")) users.push_back(sfs2::u8path(e));
    if (const char *la = getenv("LOCALAPPDATA")) {
        users.push_back(sfs2::u8path(la) / "BeamNG" / "BeamNG.drive");
        users.push_back(sfs2::u8path(la) / "BeamNG.drive");
    }
    if (const char *up = getenv("USERPROFILE")) users.push_back(sfs2::u8path(up) / "Documents" / "BeamNG.drive");
    std::vector<std::string> out;
    auto add_mods = [&](const sfs2::path &u) {
        sfs2::path m = u / "mods";
        if (!sfs2::is_directory(m, ec)) return;
        out.push_back(m.u8string());
        if (sfs2::is_directory(m / "repo", ec)) out.push_back((m / "repo").u8string());
        if (sfs2::is_directory(m / "unpacked", ec)) out.push_back((m / "unpacked").u8string());
    };
    for (auto &u : users) {
        if (!sfs2::is_directory(u, ec)) continue;
        add_mods(u);
        // versioned layouts: <user>/current, <user>/0.3x
        for (auto &d : sfs2::directory_iterator(u, ec))
            if (d.is_directory(ec)) add_mods(d.path());
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

}  // namespace ec
