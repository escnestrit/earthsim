// A drivable vehicle built from an assembled JBeam model: every node and beam goes into the soft-body solver;
// tyres act at the wheel axle node pairs through our tyre model; steering moves the hubs through the mod's
// hydros; engine, gearbox and brakes come from the mod where present. Not like BeamNG: tyres are not node
// meshes, no Lua controllers, no torsion bars, no self-collision.
#pragma once
#include "../phys/softcar.hpp"
#include "jmodel.hpp"

namespace ec {

struct JWheel {
    std::string name;
    int n1, n2, arm = -1;
    double radius = 0.33, width = 0.2;
    bool front = false, driven = false;
    double brake_torque = 1500, parking_torque = 0, inertia = 1.1;
    WheelState st;
};

struct Hydro {
    int beam;
    double base, factor, in_rate, out_rate, state = 0;
};

struct DamageGroups {
    std::vector<std::string> names;
    std::vector<bool> triggered;
    std::vector<std::tuple<int, int, float>> triggers;  // beam, group, ratio
    std::vector<std::pair<std::string, std::vector<int>>> break_groups;
    std::vector<bool> broken_groups;
    uint32_t generation = 0;
    bool is_triggered(const std::string &n) const;
    void reset();
    void update(SoftBody &b);
};

struct Eye {
    V3 rest;
    double fov = 60;
    std::vector<int> refs;
    bool from_mod = false;
};

class JVehicle {
public:
    SoftBody body;
    std::vector<std::string> names;
    std::vector<JWheel> wheels;
    std::vector<Hydro> hydros;
    Drivetrain drive;
    DriveParams dparams;
    TireParams tire;
    std::vector<std::array<int, 3>> triangles;
    V3 pos, vel, accel, fwd{1, 0, 0}, left{0, 1, 0}, up{0, 0, 1};
    double impact = 0, cd_a = 0.85, steer_cmd = 0;
    std::vector<std::string> notes;
    std::vector<bool> weak;
    Eye eye;
    DamageGroups groups;
    double sub_dt = 0.00025;

    static bool from_model(const JModel &m, JVehicle &out, std::string &err);
    void place(const CollisionWorld &w, double x, double y, double z_hint, double yaw, double speed);
    void step(double dt, const CarInput &in, const CollisionWorld &w);
    double forward_speed() const { return vel.dot(fwd); }
    // rotation rest -> now as the three axes (columns) and the rest-shape mass centre
    void rotation(V3 &f, V3 &l, V3 &u) const;
    V3 rest_com() const;
    V3 wheel_center(int i) const { return (body.nodes[wheels[i].n1].p + body.nodes[wheels[i].n2].p) * 0.5; }
    V3 wheel_axis(int i) const;
    void eye_pose(V3 &p) const;
    double damage(int *broken = nullptr) const;

private:
    int frame_[4];
    int driven_l_ = 0, driven_r_ = 1;
    V3 rf_, rl_, ru_;  // rest-shape axes
    void axes(V3 &f, V3 &l, V3 &u) const;
    void refresh();
    void substep(double dt, const CarInput &in, const CollisionWorld &w);
};

DriveParams drivetrain_params(const JModel &m, std::vector<std::string> &notes);

}  // namespace ec
