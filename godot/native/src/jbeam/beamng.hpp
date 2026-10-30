// Where BeamNG.drive is installed (its content/vehicles folder with common.zip), for loading the user's own
// vehicles. Nothing from BeamNG ships with the game.
#pragma once
#include <string>
#include <map>
#include <vector>
#include "../core/json.hpp"
#include "zip.hpp"

namespace ec {
std::vector<std::string> steam_libraries(const std::string &vdf);
// content/vehicles folder of a BeamNG.drive installation, or "" (EARTHSIM_BEAMNG overrides)
std::string find_beamng_vehicles();

// Folders where the player's BeamNG mods live: the user folder's mods, mods/repo (from the in-game repository),
// and each mods/unpacked/<mod> folder. The user folder is found from the install's startup.ini (UserPath) and
// the usual places under %LOCALAPPDATA% and Documents.
std::vector<std::string> find_beamng_mod_dirs();

// The vehicle's default configuration (info.json "default_pc" -> <name>.pc): which part goes in each slot and
// the tuning variables, as BeamNG spawns it. `model` is the folder under vehicles/. False when there is none.
struct VehicleConfig {
    std::string model, name, main_part;
    std::map<std::string, std::string> parts;
    std::map<std::string, Json> vars;
};
bool default_config(Zip &z, VehicleConfig &out, std::string &err);
// a named configuration (<name>.pc in the vehicle's folder), or the default one when `pc` is empty
bool load_config(Zip &z, const std::string &pc, VehicleConfig &out, std::string &err);
// the configurations a vehicle archive offers (pc file names without extension), the default first
std::vector<std::string> list_configs(Zip &z);
}  // namespace ec
