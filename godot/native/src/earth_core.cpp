#include "earth_core.h"
using namespace godot;
void EarthCore::_bind_methods() {
    ClassDB::bind_static_method("EarthCore", D_METHOD("version"), &EarthCore::version);
}
String EarthCore::version() { return "earthcore 0.1"; }
