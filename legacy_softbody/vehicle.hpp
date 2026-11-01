// A car: soft-body structure + suspension corners + tyres + drivetrain + mechanical state that the
// player maintains (bolts, fluids, engine wear). Everything a garage can touch lives here.
#pragma once
#include <string>
#include <vector>
#include "softbody.hpp"

namespace tt {

struct Controls {
    double throttle = 0, brake = 0, steer = 0, handbrake = 0, clutch = 0;  // steer -1 left .. 1 right
    int shift = 0;                                                       // +1 / -1 requests
    bool ignition = true;
};

// A bolted joint the mechanic tightens with a torque wrench.
struct Bolt {
    std::string where;      // "wheel FL 1/5", "subframe front left"...
    double spec = 120;      // specified torque N m
    double torque = 120;    // current torque
    bool stripped = false;
    int part = -1;          // part whose attach beams depend on it (-1: wheel/hub, see wheel)
    int wheel = -1;
};

struct Wheel {
    int hub = -1;                 // hub node
    int mount_a = -1, mount_b = -1;  // chassis nodes the corner is attached to
    V3 rest_local;                // hub position in the chassis frame at ride height
    double radius = 0.31, width = 0.205, inertia = 1.2, mass = 22;
    double omega = 0;             // spin rad/s (positive = forward)
    double steer = 0;             // current steer angle rad (positive right)
    bool front = false, left = false, driven = false;
    // suspension
    double k = 32000, c_bump = 2600, c_reb = 4200, travel_up = 0.10, travel_down = 0.12;
    double k_lat = 1.6e6;         // stiffness of the links holding the hub sideways / along
    V3 bend;                      // permanent misalignment of the corner (crash damage), chassis frame
    double toe_bend = 0;          // rad
    // tyre state
    double load = 0, slip_ratio = 0, slip_angle = 0, wear = 0, pressure = 2.2;  // bar
    bool contact = false, detached = false, flat = false;
    int surface = 0;
    double brake_torque_max = 2400;
    double tyre_temp = 30;
};

struct Engine {
    double idle = 850, redline = 6800, inertia = 0.18;
    double rpm = 850;
    double peak_torque = 280;     // N m
    double wear = 0;              // 0..1, loses compression
    double damage = 0;            // 0..1 from impacts (block, mounts)
    double oil = 1.0, coolant = 1.0;  // fill levels
    double temp = 20;             // coolant temperature C
    bool running = true, seized = false;
    double knock = 0;             // 0..1 (sound cue)
    double torque_at(double r) const;
};

struct Gearbox {
    std::vector<double> ratios{-3.3, 0, 3.6, 2.1, 1.4, 1.05, 0.84, 0.7};  // index 1 = neutral
    int gear = 2;  // first
    double final_drive = 3.9;
    bool automatic = true;
    double shift_timer = 0;
    double ratio() const { return ratios[gear] * final_drive; }
    int display() const { return gear - 1; }  // -1 R, 0 N, 1..
};

// Visual skin: quads over the node lattice (the deformation shows exactly as simulated)
struct SkinQuad {
    int a, b, c, d;   // node indices, counter-clockwise seen from outside
    uint8_t mat;      // 0 paint, 1 glass, 2 dark (underbody, trim), 3 lights
    int part;
};

struct VehicleDesc {
    std::string name = "sedan";
    double length = 4.55, width = 1.80, height = 1.45, wheelbase = 2.70, track = 1.54;
    double mass = 1380;
    bool rwd = false;
    double engine_torque = 280;
};

class Vehicle {
  public:
    SoftBody body;
    std::vector<Wheel> wheels;  // FL FR RL RR
    std::vector<Bolt> bolts;
    std::vector<SkinQuad> skin;
    Engine engine;
    Gearbox gearbox;
    Controls in;
    VehicleDesc desc;
    int ref_c = 0, ref_f = 0, ref_l = 0;  // chassis frame reference nodes
    int engine_node = -1, radiator_node = -1;
    double odometer = 0, speed = 0;
    bool abs_on = true, tcs_on = true, esc_on = true;
    double substep = 1.0 / 2000.0;
    double vibration = 0;  // RMS chassis vibration (loosens bolts, sound)

    void build(const VehicleDesc &d);
    // advance by frame_dt using fixed substeps (returns the number of substeps run)
    int update(double frame_dt, const GroundFn &ground, std::vector<Vehicle *> *others = nullptr);
    void place(const V3 &pos, double yaw);  // teleport, resets velocities
    void frame(V3 &origin, V3 &right, V3 &fwd, V3 &up) const;
    // workshop
    bool tighten(int bolt, double torque);  // returns false if the thread strips
    void replace_part(int part);
    void refill();
    double part_health(const std::string &name) const;
    std::string damage_report() const;

  private:
    double acc_ = 0;
    V3 last_v_;
    void substep_(double dt, const GroundFn &ground);
    void suspension_(double dt, const V3 &o, const V3 &r, const V3 &f, const V3 &u);
    void tyres_and_drive_(double dt, const GroundFn &ground, const V3 &r, const V3 &f, const V3 &u);
    void mechanical_(double dt);
};

}  // namespace tt
