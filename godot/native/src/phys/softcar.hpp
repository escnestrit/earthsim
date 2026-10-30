// Soft-body car: a deformable chassis lattice (node-beam), four hub nodes on beam suspension, a combined-slip
// tyre model, brakes with ABS, engine + clutch + automatic gearbox + limited-slip differential.
// Frame: +X forward, +Y left, +Z up; the rest shape is in "model space" (z from the ground under the wheels).
#pragma once
#include <array>
#include "softbody.hpp"

namespace ec {

struct TireParams {
    double mu = 1.1, kappa_peak = 0.11, tan_alpha_peak = 0.14, load_sens = 0.10, fz0 = 3800, v0 = 3.0;
    // grip left when fully sliding, as a fraction of the peak (BeamNG's slidingFrictionCoef / frictionCoef)
    double sliding = 0.707;
    // BeamNG-style load sensitivity, used when bl_slope > 0: mu(fz) = mu * (full + (none - full) * exp(-slope * fz))
    double bl_none = 0, bl_full = 0, bl_slope = 0;
};
struct TireOut {
    double fx = 0, fy = 0, kappa = 0, tan_alpha = 0, limit = 0;
};
TireOut tire_force(const TireParams &p, double fz, double surface, double v_long, double v_lat, double omega, double radius);

struct DriveParams {
    std::vector<std::pair<double, double>> torque_curve = {{800, 110}, {1500, 160}, {2500, 205}, {3500, 225}, {4500, 230}, {5500, 215}, {6500, 185}};
    double idle_rpm = 850, max_rpm = 6700, engine_inertia = 0.3;
    std::vector<double> gears = {3.62, 2.19, 1.54, 1.14, 0.92, 0.76};
    double reverse = 3.4, final_drive = 3.9, efficiency = 0.9;
    double clutch_k = 60, clutch_max = 520, lsd_k = 300, lsd_max = 500, shift_time = 0.3;
};

struct Drivetrain {
    double w_e = 89, shift_t = 0, dwell = 0, clutch = 0, engine_torque = 0, clutch_torque = 0;
    int gear = 1;
    double rpm() const { return w_e * 60 / (2 * PI); }
    double ratio(const DriveParams &p) const;
    // returns torques at the two driven wheels and the coupling stiffness
    void step(double dt, const DriveParams &p, double throttle, bool reverse, double wl, double wr, double &tl, double &tr, double &stiff);
};

struct CarParams {
    double mass = 1450, wheel_radius = 0.33, wheel_inertia = 1.4;
    double spring[2] = {35000, 38000}, damper[2] = {3200, 3000}, arb[2] = {16000, 9000};
    double bump_k = 250000;
    TireParams tire;
    double brake_front = 1700, brake_rear = 1000, handbrake = 1500;
    double cd_a = 0.75, crr = 0.014;
    double wheelbase = 2.7, track = 1.56, steer_max = 0.55;
    DriveParams drive;
    bool awd = false;  // else front-wheel drive
};

struct CarInput {
    double throttle = 0, brake = 0, steer = 0, handbrake = 0;  // steer: road-wheel angle, + = left (rad)
    bool reverse = false, tcs = true, abs = true;
};

struct WheelState {
    double w = 0, spin = 0, steer = 0, fz = 0, kappa = 0, tan_alpha = 0, fx = 0, fy = 0, comp = 0;
    bool contact = false, locked = false;
    V3 pc;
};

class SoftCar {
public:
    CarParams p;
    SoftBody body;
    std::vector<int> cage;
    int hubs[4], towers[4], long_anchor[4], lat_anchor[4];
    WheelState wheels[4];
    Drivetrain drive;
    double time = 0, impact = 0;
    V3 pos, vel, fwd{1, 0, 0}, left{0, 1, 0}, up{0, 0, 1}, accel;
    double sub_dt = 0.00025;

    explicit SoftCar(const CarParams &p);
    // place the (repaired) car with its wheels on the surface at (x, y), heading yaw, rolling at speed
    void place(const CollisionWorld &w, double x, double y, double z_hint, double yaw, double speed);
    void step(double dt, const CarInput &in, const CollisionWorld &w);
    double forward_speed() const { return vel.dot(fwd); }
    // rest-to-world rigid frame of the chassis (rotation rows fwd/left/up, origin at the rest-frame origin)
    V3 to_world(V3 rest) const;
    V3 rest_com() const { return rest_com_; }
    // biggest displacement of a chassis node from the undamaged shape (m), broken beams
    double damage(int *broken = nullptr) const;

private:
    V3 rest_com_;
    double spring_rest_[2];
    std::vector<Hit> tmp_;
    void refresh_frame();
    void axes(V3 &f, V3 &l, V3 &u) const;
    void substep(double dt, const CarInput &in, const CollisionWorld &w);
};

// Smoothed keyboard / pad input -> car input (automatic reverse, speed-sensitive steering lock)
struct Driver {
    double throttle = 0, brake = 0, steer = 0, handbrake = 0;
    bool reverse = false;
    double steer_max = 0.55;
    CarInput update(double dt, bool up, bool down, bool left, bool right, bool hb, double fwd_speed);
    CarInput update_analog(double steer_axis, double thr, double brk, bool hb, double fwd_speed);
};

}  // namespace ec
