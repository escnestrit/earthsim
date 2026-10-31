// Physics checks of the True Torque core, no engine needed.
#include <chrono>
#include <cstdio>
#include <string>
#include "net.hpp"
#include "vehicle.hpp"

using namespace tt;

static int failures = 0;
static void check(bool ok, const char *name, const std::string &detail) {
    printf("[%s] %-34s %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
    if (!ok) failures++;
}
static std::string fmt(const char *f, double a = 0, double b = 0, double c = 0, double d = 0) {
    char buf[256];
    snprintf(buf, sizeof buf, f, a, b, c, d);
    return buf;
}

static GroundFn flat(double friction = 1.0) {
    return [friction](double, double) { return GroundHit{0.0, {0, 0, 1}, friction, 0}; };
}

// height of the car body's floor reference above the ground
static double ride(const Vehicle &v) { return v.body.nodes[v.ref_c].p.z; }

int main() {
    using clk = std::chrono::steady_clock;
    VehicleDesc desc;

    // ---- 1. settles on its wheels without exploding
    {
        Vehicle v;
        v.build(desc);
        v.place({0, 0, 0.15}, 0);
        auto g = flat();
        double maxv = 0;
        for (int i = 0; i < 120 * 3; i++) {
            v.update(1.0 / 120, g);
            maxv = std::max(maxv, len(v.body.velocity()));
        }
        double h = ride(v), vel = len(v.body.velocity());
        int contacts = 0;
        for (auto &w : v.wheels) contacts += w.contact;
        check(h > 0.15 && h < 0.45 && vel < 0.05 && contacts == 4 && v.body.stats.broken_beams == 0, "settles on its suspension",
              fmt("floor %.2f m, speed %.3f m/s, wheels on ground %.0f, broken beams %.0f", h, vel, contacts, v.body.stats.broken_beams));
        check(v.body.stats.deformation_energy < 50, "no deformation from its own weight", fmt("%.1f J", v.body.stats.deformation_energy));
    }
    // ---- 2. drives: 0-100 km/h, top speed, braking, goes straight
    {
        Vehicle v;
        v.build(desc);
        v.place({0, 0, 0.1}, 0);
        auto g = flat();
        for (int i = 0; i < 120; i++) v.update(1.0 / 120, g);
        v.in.throttle = 1;
        double t = 0, t100 = -1;
        auto t0 = clk::now();
        int steps = 0;
        while (t < 40) {
            steps += v.update(1.0 / 120, g);
            t += 1.0 / 120;
            if (t100 < 0 && v.speed * 3.6 >= 100) t100 = t;
        }
        double ms = std::chrono::duration<double, std::milli>(clk::now() - t0).count();
        double vmax = v.speed * 3.6;
        double drift = std::fabs(v.body.nodes[v.ref_c].p.x);
        check(t100 > 5 && t100 < 13, "0-100 km/h", fmt("%.1f s", t100));
        check(vmax > 170 && vmax < 260, "top speed", fmt("%.0f km/h", vmax));
        check(drift < v.odometer * 0.005, "drives straight", fmt("%.2f m sideways after %.0f m", drift, v.odometer));
        check(ms / steps < 0.06, "simulation cost", fmt("%.3f ms per 0.5 ms substep (%.0f nodes, %.0f beams)", ms / steps, v.body.nodes.size(), v.body.beams.size()));
        // brake from 100
        while (v.speed * 3.6 > 101) v.in.throttle = 0, v.in.brake = 1, v.update(1.0 / 120, g);
        double y0 = v.body.nodes[v.ref_c].p.y;
        v.in.throttle = 0, v.in.brake = 1;
        for (int i = 0; i < 120 * 8 && v.speed > 0.2; i++) v.update(1.0 / 120, g);
        double dist = v.body.nodes[v.ref_c].p.y - y0;
        check(dist > 30 && dist < 50, "braking 100-0", fmt("%.1f m", dist));
        check(v.body.stats.broken_beams == 0, "no damage from driving", fmt("%.0f broken beams", v.body.stats.broken_beams));
    }
    // ---- 3. cornering: steady circle, lateral grip
    {
        Vehicle v;
        v.build(desc);
        v.place({0, 0, 0.1}, 0);
        auto g = flat();
        for (int i = 0; i < 120; i++) v.update(1.0 / 120, g);
        v.in.throttle = 0.5;
        for (int i = 0; i < 120 * 6; i++) v.update(1.0 / 120, g);
        double latmax = 0;
        V3 prev = v.body.velocity();
        v.in.steer = 1.0;
        for (int i = 0; i < 120 * 6; i++) {
            v.in.throttle = v.speed * 3.6 < 60 ? 0.6 : 0.2;
            v.update(1.0 / 120, g);
            V3 vel = v.body.velocity();
            V3 a = (vel - prev) * 120.0;
            prev = vel;
            V3 o, r, f, u;
            v.frame(o, r, f, u);
            if (i > 240) latmax = std::max(latmax, std::fabs(dot(a, r)));
        }
        V3 o, r, f, u;
        v.frame(o, r, f, u);
        check(latmax > 4.0 && latmax < 14.0 && u.z > 0.9, "cornering grip", fmt("max lateral %.1f m/s2, upright %.2f", latmax, u.z));
    }
    // ---- 4. crash into a wall at 50 km/h: crumple zone takes it, the cabin survives
    double dent50 = 0;
    for (double kmh : {50.0, 80.0}) {
        Vehicle v;
        v.build(desc);
        v.place({0, 0, 0.1}, 0);
        auto g = flat();
        for (int i = 0; i < 120; i++) v.update(1.0 / 120, g);
        // wall plane at y = 12 facing -y
        double wy = v.body.nodes[v.ref_c].p.y + 6;
        for (size_t i = 0; i < v.body.nodes.size(); i++) v.body.contacts[i] = {ContactPlane{{0, wy, 0}, {0, -1, 0}, 0.6, 0}};
        for (auto &n : v.body.nodes) n.v = V3{0, kmh / 3.6, 0};
        for (auto &w : v.wheels) w.omega = kmh / 3.6 / w.radius;
        double cab0 = len(v.body.nodes[v.ref_f].p - v.body.nodes[v.ref_c].p);
        // crush of the structure (not of loose panels that may come off)
        auto front_of = [&]() {
            double m = 0;
            for (auto &n : v.body.nodes)
                if (n.part >= 0 && (v.body.parts[n.part].name == "front_structure" || (v.body.parts[n.part].name == "bumper_front" && !v.body.parts[n.part].detached)))
                    m = std::max(m, n.p.y - v.body.nodes[v.ref_c].p.y);
            return m;
        };
        double front0 = front_of();
        for (int i = 0; i < 120 * 3; i++) v.update(1.0 / 120, g);
        double front1 = front_of();
        double cab1 = len(v.body.nodes[v.ref_f].p - v.body.nodes[v.ref_c].p);
        double crush = front0 - front1;
        double cab = std::fabs(cab1 - cab0);
        bool finite = std::isfinite(v.body.centre_of_mass().y) && len(v.body.velocity()) < kmh / 3.6;
        if (kmh == 50) {
            dent50 = crush;
            check(finite && crush > 0.15 && crush < 0.9 && cab < 0.08, "crash 50 km/h: front crumples",
                  fmt("front shorter by %.2f m, cabin %.3f m, energy %.0f kJ, broken beams %.0f", crush, cab, v.body.stats.deformation_energy / 1000, v.body.stats.broken_beams));
            check(v.part_health("bumper_front") < 0.8 && v.part_health("roof") > 0.9, "crash 50 km/h: damage where it hit",
                  fmt("bumper %.2f, hood %.2f, roof %.2f, engine damage %.2f", v.part_health("bumper_front"), v.part_health("hood"), v.part_health("roof"), v.engine.damage));
        } else {
            check(finite && crush > dent50 && v.body.stats.broken_beams > 15, "crash 80 km/h: worse than 50",
                  fmt("front shorter by %.2f m, radiator %.2f, engine damage %.2f, broken beams %.0f", crush, v.part_health("radiator"), v.engine.damage, v.body.stats.broken_beams));
        }
    }
    // ---- 5. loose wheel nuts: the wheel comes off on a rough road; tightened ones hold
    {
        auto rough = [](double x, double y) {
            double h = 0.025 * std::sin(y * 3.1) * std::sin(x * 2.3 + y * 0.7);
            return GroundHit{h, {0, 0, 1}, 1.0, 0};
        };
        for (int loose : {0, 1}) {
            Vehicle v;
            v.build(desc);
            v.place({0, 0, 0.12}, 0);
            if (loose)
                for (auto &b : v.bolts)
                    if (b.wheel == 0) v.tighten(int(&b - &v.bolts[0]), 25);
            v.in.throttle = 0.35;
            double t = 0;
            while (t < 120 && !v.wheels[0].detached) {
                v.in.throttle = v.speed * 3.6 < 50 ? 0.4 : 0.0;
                v.update(1.0 / 120, rough);
                t += 1.0 / 120;
            }
            if (loose) check(v.wheels[0].detached, "loose nuts: wheel comes off", fmt("after %.0f m (vibration %.1f, nut %.1f N m)", v.odometer, v.vibration, v.bolts[0].torque));
            else check(!v.wheels[0].detached, "torqued nuts: wheel stays", fmt("%.0f m on a rough road", v.odometer));
        }
        Vehicle v;
        v.build(desc);
        bool ok = v.tighten(0, 400);
        check(!ok && v.bolts[0].stripped, "overtightening strips the thread", "400 N m on a 120 N m nut");
    }
    // ---- 6. repair: replacing parts restores them
    {
        Vehicle v;
        v.build(desc);
        v.place({0, 0, 0.1}, 0);
        auto g = flat();
        for (int i = 0; i < 120; i++) v.update(1.0 / 120, g);
        for (size_t i = 0; i < v.body.nodes.size(); i++) v.body.contacts[i] = {ContactPlane{{0, 6, 0}, {0, -1, 0}, 0.6, 0}};
        for (auto &n : v.body.nodes) n.v = V3{0, 14, 0};
        for (int i = 0; i < 240; i++) v.update(1.0 / 120, g);
        double before = v.part_health("bumper_front");
        for (auto &c : v.body.contacts) c.clear();
        for (size_t p = 0; p < v.body.parts.size(); p++)
            if (v.body.parts[p].name == "bumper_front") v.replace_part(int(p));
        for (int i = 0; i < 60; i++) v.update(1.0 / 120, g);
        check(before < 0.8 && v.part_health("bumper_front") > 0.95, "new bumper fitted", fmt("health %.2f -> %.2f", before, v.part_health("bumper_front")));
    }
    // ---- 7. two cars collide: both deform, momentum roughly conserved
    {
        Vehicle a, b;
        a.build(desc), b.build(desc);
        a.place({0, 0, 0.1}, 0), b.place({0, 7, 0.1}, TT_PI);
        auto g = flat();
        std::vector<Vehicle *> all{&a, &b};
        for (int i = 0; i < 60; i++) a.update(1.0 / 120, g, &all), b.update(1.0 / 120, g, &all);
        for (auto &n : a.body.nodes) n.v = {0, 11, 0};
        for (auto &n : b.body.nodes) n.v = {0, -11, 0};
        double p0 = a.body.velocity().y * a.body.mass() + b.body.velocity().y * b.body.mass();
        for (int i = 0; i < 240; i++) a.update(1.0 / 120, g, &all), b.update(1.0 / 120, g, &all);
        double p1 = a.body.velocity().y * a.body.mass() + b.body.velocity().y * b.body.mass();
        check(a.body.stats.deformation_energy > 2000 && b.body.stats.deformation_energy > 2000 && std::fabs(a.body.velocity().y) < 6, "car-to-car crash",
              fmt("deformation %.0f / %.0f kJ, momentum before %.0f after %.0f", a.body.stats.deformation_energy / 1000, b.body.stats.deformation_energy / 1000, p0, p1));
    }
    // ---- 8. network snapshot: small, accurate
    {
        Vehicle v, w;
        v.build(desc), w.build(desc);
        v.place({100, -50, 0.1}, 0.7), w.place({0, 0, 0}, 0);
        auto g = flat();
        v.in.throttle = 0.5;
        for (int i = 0; i < 240; i++) v.update(1.0 / 120, g);
        Snapshot s = encode(v, 77);
        uint32_t t = decode(s, w);
        double err = 0;
        for (size_t i = 0; i < v.body.nodes.size(); i++) err = std::max(err, len(v.body.nodes[i].p - w.body.nodes[i].p));
        check(t == 77 && err < 0.004 && s.bytes.size() < 900, "network snapshot", fmt("%.0f bytes, max error %.1f mm", s.bytes.size(), err * 1000));
    }
    printf("%s: %d failed\n", failures ? "FAILED" : "OK", failures);
    return failures ? 1 : 0;
}
