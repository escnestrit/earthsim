// Where BeamNG.drive is installed (its content/vehicles folder with common.zip), for loading the user's own
// vehicles. Nothing from BeamNG ships with the game.
#pragma once
#include <string>
#include <vector>

namespace ec {
std::vector<std::string> steam_libraries(const std::string &vdf);
// content/vehicles folder of a BeamNG.drive installation, or "" (EARTHSIM_BEAMNG overrides)
std::string find_beamng_vehicles();
}  // namespace ec
