#include "beamng.hpp"
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
    if (sfs::is_regular_file(v / "common.zip", ec)) return v.string();
    return "";
}

std::string find_beamng_vehicles() {
    std::error_code ec;
    if (const char *e = getenv("EARTHSIM_BEAMNG")) {
        sfs::path p(e);
        if (sfs::is_regular_file(p / "common.zip", ec)) return p.string();
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
