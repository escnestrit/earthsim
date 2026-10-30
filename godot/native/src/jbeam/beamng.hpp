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

// The vehicle's default configuration (info.json "default_pc" -> <name>.pc): which part goes in each slot and
// the tuning variables, as BeamNG spawns it. `model` is the folder under vehicles/. False when there is none.
struct VehicleConfig {
    std::string model, name, main_part;
    std::map<std::string, std::string> parts;
    std::map<std::string, Json> vars;
};
bool default_config(Zip &z, VehicleConfig &out, std::string &err);
}  // namespace ec
