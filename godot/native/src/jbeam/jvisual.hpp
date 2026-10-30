// What a BeamNG vehicle looks like: its flexbody meshes placed in the rest shape and bound to the nodes of their
// groups (skinned on the GPU), wheel meshes (rigid, turned with the wheel), the steering-wheel prop, materials
// and textures from the mod (and BeamNG's common.zip when installed).
#pragma once
#include <algorithm>
#include <memory>
#include "dae.hpp"
#include "jvehicle.hpp"
#include "shape.hpp"

namespace ec {

struct MatDef {
    std::string name;
    float base_color[4] = {1, 1, 1, 1};
    std::string base_map, normal_map;
    float roughness = 0.6f, metallic = 0;
    bool translucent = false;
};

struct VisPart {
    std::string mesh;
    std::vector<SubMesh> subs;            // positions in the vehicle's rest frame (ours: x fwd, y left, z up)
    std::vector<std::vector<std::array<int, 4>>> bones;     // per submesh, per vertex: 4 node indices
    std::vector<std::vector<std::array<float, 4>>> weights;
    int wheel = -1;                       // rigid wheel part: positions in the wheel's frame (x fwd, y axle, z up)
    std::string swap_group, swap_base, swap_damaged;
};

struct Prop {
    std::vector<SubMesh> subs;
    int refs[3];
    double base_rot[3], base_trans[3], axis[3];
    double multiplier = 1, offset = 0, mn = -1e9, mx = 1e9, lock_deg = 450;
    double angle(double signal) const { return std::clamp(signal * lock_deg * multiplier + offset, mn, mx); }
    // pose from node positions (rest frame): origin and rotation columns
    void pose(const std::vector<V3> &nodes, double signal, V3 &origin, V3 cols[3]) const;
};

struct CarVisual {
    std::vector<VisPart> parts;
    std::vector<Prop> props;
    std::map<std::string, MatDef> materials;
    std::map<std::string, std::string> textures;  // path -> file bytes (dds/png/jpg)
    std::vector<std::string> notes;
};

bool is_invisible_material(const std::string &name);

// mod: the vehicle zip; common: BeamNG's content/vehicles/common.zip (may be null)
bool load_visual(Zip &mod, Zip *common, const JModel &m, const JVehicle &v, CarVisual &out, std::string &err);

}  // namespace ec
