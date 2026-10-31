#include "vehicle.hpp"
#include <cstdio>
#include <map>
#include <tuple>

namespace tt {

double Engine::torque_at(double r) const {
    // broad torque plateau, falls off above 5000 rpm
    double t = r < 1000 ? 0.55 + 0.25 * (r - 600) / 400 : r < 3800 ? 0.8 + 0.2 * (r - 1000) / 2800 : r < 5200 ? 1.0 : std::max(0.45, 1.0 - (r - 5200) / 3500);
    double health = (1.0 - 0.6 * damage) * (1.0 - 0.35 * wear);
    return peak_torque * clampd(t, 0.0, 1.0) * health;
}

namespace {
double magic(double s, double b, double c) { return std::sin(c * std::atan(b * s)); }
double crit(double k, double m) { return 2.0 * std::sqrt(k * m); }
}  // namespace

// ------------------------------------------------------------------ construction

void Vehicle::build(const VehicleDesc &d) {
    desc = d;
    body = SoftBody();
    wheels.clear();
    bolts.clear();
    engine = Engine();
    engine.peak_torque = d.engine_torque;
    gearbox = Gearbox();
    double hw = d.width * 0.5 - 0.02, ax = d.wheelbase * 0.5;
    double yF = d.length * 0.5, yR = -d.length * 0.5;
    std::vector<double> st = {yR, yR + 0.45, -ax, -0.75, 0.0, 0.75, ax, yF - 0.40, yF};
    auto &B = body;
    auto part = [&](const char *n) {
        B.parts.push_back(Part{n});
        return int(B.parts.size() - 1);
    };
    int P_BF = part("bumper_front"), P_BR = part("bumper_rear"), P_HOOD = part("hood"), P_TRUNK = part("trunk"), P_DL = part("door_left"),
        P_DR = part("door_right"), P_RAD = part("radiator"), P_ENG = part("engine"), P_ROOF = part("roof"), P_FRONT = part("front_structure"),
        P_REAR = part("rear_structure"), P_CABIN = part("cabin");
    // ---- structure lattice
    struct K {
        int s, x, z;
        bool operator<(const K &o) const { return std::tie(s, x, z) < std::tie(o.s, o.x, o.z); }
    };
    std::map<K, int> id;
    double body_mass = d.mass - 160 - 4 * 22 - 70;  // engine, wheels, panels
    std::vector<V3> pts;
    std::vector<K> keys;
    for (int s = 0; s < int(st.size()); s++)
        for (int xi = -1; xi <= 1; xi++)
            for (int zi = 0; zi <= 2; zi++) {
                bool cab = s >= 3 && s <= 5;
                if (zi == 2 && (!cab || xi == 0)) continue;
                double x = xi * (zi == 2 ? hw - 0.16 : hw);
                double z = zi == 0 ? 0.30 : zi == 1 ? 0.74 : d.height - 0.05;
                if ((s == 0 || s == int(st.size()) - 1) && zi == 1) z = 0.62;
                pts.push_back({x, st[s], z});
                keys.push_back({s, xi, zi});
            }
    double nm = body_mass / pts.size();
    for (size_t i = 0; i < pts.size(); i++) {
        int s = keys[i].s;
        int pr = s == 0 ? P_BR : s == int(st.size()) - 1 ? P_BF : s >= 3 && s <= 5 ? (keys[i].z == 2 ? P_ROOF : P_CABIN) : s < 3 ? P_REAR : P_FRONT;
        id[keys[i]] = B.add_node(pts[i], nm, pr);
    }
    for (auto &n : B.nodes) n.radius = 0.1;
    auto zone = [&](const V3 &a, const V3 &b, double &k, double &yield, double &brk) {
        double ym = std::max(std::fabs(a.y), std::fabs(b.y));
        bool roof = a.z > 1.0 || b.z > 1.0;
        if (ym > yF - 0.5 || std::fabs(std::min(a.y, b.y) - yR) < 0.5) k = 5.0e5, yield = 0.02, brk = 0.7;  // crumple zones: fold at a steady force
        else if (roof) k = 3.5e6, yield = 0.04, brk = 0.5;
        else if (ym > 0.8) k = 3.0e6, yield = 0.035, brk = 0.55;
        else k = 7.0e6, yield = 0.06, brk = 0.65;  // safety cell
    };
    for (size_t i = 0; i < pts.size(); i++)
        for (size_t j = i + 1; j < pts.size(); j++) {
            double l = len(pts[j] - pts[i]);
            int ds = std::abs(keys[i].s - keys[j].s);
            if (l > 1.25 || ds > 1) continue;
            double k, y, b;
            zone(pts[i], pts[j], k, y, b);
            int pi = B.nodes[i].part, pj = B.nodes[j].part;
            bool att = (pi == P_BF || pi == P_BR) != (pj == P_BF || pj == P_BR);
            int pb = att ? (pi == P_BF || pj == P_BF ? P_BF : P_BR) : (pi == pj ? pi : P_CABIN);
            if (att) b = 0.22, k *= 0.6;
            B.add_beam(int(i), int(j), k, 0.12 * crit(k, nm * 0.5), y, b, pb, att ? BEAM_ATTACH : BEAM_NORMAL);
        }
    auto nearest = [&](V3 p, int skip_part, int count) {
        std::vector<std::pair<double, int>> c;
        for (size_t i = 0; i < pts.size(); i++)
            if (B.nodes[i].part != skip_part) c.push_back({len(B.nodes[i].p - p), int(i)});
        std::sort(c.begin(), c.end());
        std::vector<int> r;
        for (int i = 0; i < count && i < int(c.size()); i++) r.push_back(c[i].second);
        return r;
    };
    // ---- panels: hood, trunk lid, doors (light nodes on hinges/latches = attach beams)
    auto panel = [&](int pid, std::vector<V3> ps, double mass, int hinge_n) {
        std::vector<int> ns;
        for (auto &p : ps) ns.push_back(B.add_node(p, mass / ps.size(), pid)), B.nodes.back().radius = 0.06;
        for (size_t i = 0; i < ns.size(); i++)
            for (size_t j = i + 1; j < ns.size(); j++) B.add_beam(ns[i], ns[j], 4e5, crit(4e5, 2) * 0.15, 0.02, 0.6, pid);
        for (int n : ns)
            for (int s : nearest(B.nodes[n].p, pid, hinge_n)) B.add_beam(n, s, 6e5, crit(6e5, 2) * 0.15, 0.05, 0.25, pid, BEAM_ATTACH);
    };
    panel(P_HOOD, {{-hw + 0.1, ax - 0.1, 0.80}, {hw - 0.1, ax - 0.1, 0.80}, {-hw + 0.1, yF - 0.35, 0.70}, {hw - 0.1, yF - 0.35, 0.70}}, 16, 2);
    panel(P_TRUNK, {{-hw + 0.1, yR + 0.3, 0.80}, {hw - 0.1, yR + 0.3, 0.80}, {-hw + 0.1, -ax + 0.1, 0.84}, {hw - 0.1, -ax + 0.1, 0.84}}, 12, 2);
    for (int side : {-1, 1})
        panel(side < 0 ? P_DL : P_DR, {{side * (hw + 0.04), -0.65, 0.38}, {side * (hw + 0.04), 0.65, 0.38}, {side * (hw + 0.04), -0.65, 0.98}, {side * (hw + 0.04), 0.65, 0.98}}, 21, 2);
    // ---- engine block on mounts, radiator in front of it
    engine_node = B.add_node({0, ax + 0.15, 0.55}, 160, P_ENG);
    B.nodes[engine_node].radius = 0.25;
    for (int s : nearest(B.nodes[engine_node].p, P_ENG, 6)) B.add_beam(engine_node, s, 2.5e6, crit(2.5e6, 30) * 0.2, 0.08, 0.6, P_ENG, BEAM_ATTACH);
    radiator_node = B.add_node({0, yF - 0.32, 0.5}, 9, P_RAD);
    for (int s : nearest(B.nodes[radiator_node].p, P_RAD, 4)) B.add_beam(radiator_node, s, 3e5, crit(3e5, 4) * 0.2, 0.03, 0.3, P_RAD, BEAM_ATTACH);
    // ---- chassis frame references (floor nodes in the safety cell)
    ref_c = id[{4, 0, 0}], ref_f = id[{5, 0, 0}], ref_l = id[{4, -1, 0}];
    // ---- wheels
    for (int i = 0; i < 4; i++) {
        Wheel w;
        w.front = i < 2, w.left = (i % 2) == 0;
        w.driven = d.rwd ? !w.front : w.front;
        double x = (w.left ? -1 : 1) * d.track * 0.5, y = w.front ? ax : -ax;
        w.rest_local = {x, y, w.radius};
        w.hub = B.add_node({x, y, w.radius}, w.mass, -1);
        B.nodes[w.hub].collide = false;
        int s = w.front ? 6 : 2, xi = w.left ? -1 : 1;
        w.mount_a = id[{s, xi, 0}], w.mount_b = id[{s, xi, 1}];
        if (!w.front) w.k *= 0.9, w.brake_torque_max *= 0.55;
        wheels.push_back(w);
        for (int k = 0; k < 5; k++) bolts.push_back(Bolt{std::string(i == 0 ? "FL" : i == 1 ? "FR" : i == 2 ? "RL" : "RR") + " wheel nut " + std::to_string(k + 1), 120, 120, false, -1, i});
    }
    for (auto [nm2, p] : std::vector<std::pair<const char *, int>>{{"hood latch", P_HOOD}, {"front bumper", P_BF}, {"rear bumper", P_BR}, {"engine mount", P_ENG}})
        bolts.push_back(Bolt{nm2, p == P_ENG ? 90.0 : 25.0, p == P_ENG ? 90.0 : 25.0, false, p, -1});
    B.finalize();
    // ---- skin: outer quads of the lattice (orientation fixed below by facing away from the centre)
    skin.clear();
    auto q = [&](int a, int b, int c, int d, uint8_t mat) {
        if (a < 0 || b < 0 || c < 0 || d < 0) return;
        V3 ctr = (B.nodes[a].rest + B.nodes[b].rest + B.nodes[c].rest + B.nodes[d].rest) * 0.25;
        V3 n = cross(B.nodes[b].rest - B.nodes[a].rest, B.nodes[c].rest - B.nodes[a].rest);
        V3 out = ctr - V3{0, 0, 0.7};
        if (dot(n, out) < 0) std::swap(b, d);
        skin.push_back({a, b, c, d, mat, B.nodes[a].part});
    };
    auto at = [&](int s, int x, int z) {
        auto it = id.find({s, x, z});
        return it == id.end() ? -1 : it->second;
    };
    int ns = int(st.size());
    for (int s = 0; s + 1 < ns; s++) {
        bool cab = s >= 3 && s + 1 <= 5;
        for (int xi : {-1, 1}) {
            q(at(s, xi, 0), at(s + 1, xi, 0), at(s + 1, xi, 1), at(s, xi, 1), 0);            // side
            if (cab) q(at(s, xi, 1), at(s + 1, xi, 1), at(s + 1, xi, 2), at(s, xi, 2), 1);   // side windows
        }
        for (int xi : {-1, 0}) {
            q(at(s, xi, 0), at(s, xi + 1, 0), at(s + 1, xi + 1, 0), at(s + 1, xi, 0), 2);    // floor
            if (!cab) q(at(s, xi, 1), at(s + 1, xi, 1), at(s + 1, xi + 1, 1), at(s, xi + 1, 1), 0);  // bonnet / boot deck
        }
        if (cab) q(at(s, -1, 2), at(s + 1, -1, 2), at(s + 1, 1, 2), at(s, 1, 2), 0);          // roof
    }
    // windscreen and rear window, front and rear faces
    q(at(5, -1, 1), at(5, 1, 1), at(5, 1, 2), at(5, -1, 2), 1);
    q(at(3, -1, 1), at(3, 1, 1), at(3, 1, 2), at(3, -1, 2), 1);
    for (int s : {0, ns - 1})
        for (int xi : {-1, 0}) q(at(s, xi, 0), at(s, xi + 1, 0), at(s, xi + 1, 1), at(s, xi, 1), xi == -1 ? 3 : 3);
    // panels (hood, trunk, doors) are their own 4 nodes
    for (auto &p : B.parts)
        if (p.nodes.size() == 4 && (p.name == "hood" || p.name == "trunk" || p.name == "door_left" || p.name == "door_right"))
            q(p.nodes[0], p.nodes[1], p.nodes[3], p.nodes[2], 0);
}

void Vehicle::place(const V3 &pos, double yaw) {
    Quat q = Quat::axis_angle({0, 0, 1}, yaw);
    for (auto &n : body.nodes) {
        n.p = pos + q.rotate(n.rest);
        n.v = {0, 0, 0};
        n.f = {0, 0, 0};
    }
    for (auto &w : wheels) w.omega = 0;
    acc_ = 0;
}

void Vehicle::frame(V3 &o, V3 &r, V3 &f, V3 &u) const {
    o = body.nodes[ref_c].p;
    f = norm(body.nodes[ref_f].p - o);
    V3 right = norm(o - body.nodes[ref_l].p);  // the reference node is on the left
    u = norm(cross(right, f));                 // x right, y forward, z up
    r = norm(cross(f, u));
}

// ------------------------------------------------------------------ simulation

int Vehicle::update(double frame_dt, const GroundFn &ground, std::vector<Vehicle *> *others) {
    acc_ += std::min(frame_dt, 0.05);
    int n = 0;
    while (acc_ >= substep) {
        if (others)
            for (auto *o : *others)
                if (o != this && this < o) SoftBody::collide(body, o->body, substep);
        substep_(substep, ground);
        acc_ -= substep;
        n++;
    }
    body.update_parts();
    mechanical_(n * substep);
    return n;
}

void Vehicle::substep_(double dt, const GroundFn &ground) {
    V3 o, r, f, u;
    frame(o, r, f, u);
    suspension_(dt, o, r, f, u);
    tyres_and_drive_(dt, ground, r, f, u);
    // aerodynamic drag and a little downforce on the chassis frame nodes
    V3 v = body.nodes[ref_c].v;
    double sp = len(v);
    V3 drag = v * (-0.5 * 1.2 * 0.68 * sp);
    body.nodes[ref_c].f += drag * 0.5;
    body.nodes[ref_f].f += drag * 0.5;
    body.step(dt, ground);
    speed = dot(v, f);
    odometer += std::fabs(speed) * dt;
}

void Vehicle::suspension_(double dt, const V3 &o, const V3 &r, const V3 &f, const V3 &u) {
    double g_share = desc.mass * 9.81 * 0.25;
    for (auto &w : wheels) {
        if (w.detached) continue;
        Node &h = body.nodes[w.hub];
        Node &ma = body.nodes[w.mount_a];
        Node &mb = body.nodes[w.mount_b];
        // the corner follows its mounts: target position from the mount nodes (bent mounts bend the corner)
        V3 mount = (ma.p + mb.p) * 0.5;
        V3 mount_rest = (ma.rest + mb.rest) * 0.5;
        V3 off = w.rest_local + w.bend - mount_rest;
        V3 T = mount + r * off.x + f * off.y + u * off.z;
        V3 e = h.p - T;
        V3 vrel = h.v - (ma.v + mb.v) * 0.5;
        double ex = dot(e, r), ey = dot(e, f), ez = dot(e, u);
        double vx = dot(vrel, r), vy = dot(vrel, f), vz = dot(vrel, u);
        // links: stiff sideways / fore-aft
        double cl = crit(w.k_lat, w.mass) * 0.5;
        V3 F = r * (-w.k_lat * ex - cl * vx) + f * (-w.k_lat * ey - cl * vy);
        // spring + damper + preload (ride height), bump stop, droop limit
        double fz = -g_share - w.k * ez - (vz > 0 ? w.c_bump : w.c_reb) * vz;
        if (ez > w.travel_up) fz -= 2.5e5 * (ez - w.travel_up);
        if (ez < -w.travel_down) fz -= 3e5 * (ez + w.travel_down);
        F += u * fz;
        h.f += F;
        ma.f -= F * 0.5;
        mb.f -= F * 0.5;
        // hard sideways hits bend the corner for good (toe / camber out after a kerb strike)
        double side = std::fabs(ex) + std::fabs(ey);
        if (side > 0.035) {
            w.bend += (r * ex + f * ey) * 0.002;
            w.toe_bend += (w.left ? -1 : 1) * ey * 0.004;
        }
    }
}

void Vehicle::tyres_and_drive_(double dt, const GroundFn &ground, const V3 &r, const V3 &f, const V3 &u) {
    Controls c = in;
    // steering: speed sensitive rate for keyboard input
    double target = c.steer * 0.62 / (1.0 + std::fabs(speed) / 22.0);
    double ax = desc.wheelbase, tr = desc.track;
    for (auto &w : wheels) {
        double st = 0;
        if (w.front && std::fabs(target) > 1e-4) {
            // Ackermann
            double R = ax / std::tan(std::fabs(target));
            bool inner = (target > 0) != w.left;
            st = (target > 0 ? 1 : -1) * std::atan(ax / (R + (inner ? -tr * 0.5 : tr * 0.5)));
        }
        w.steer = st + w.toe_bend;
    }
    // ---- drivetrain (lumped): engine, clutch, gearbox, open differential
    Gearbox &gb = gearbox;
    if (gb.shift_timer > 0) gb.shift_timer -= dt;
    double ratio = gb.ratio();
    double wheel_omega = 0;
    int nd = 0;
    for (auto &w : wheels)
        if (w.driven && !w.detached) wheel_omega += w.omega, nd++;
    if (nd) wheel_omega /= nd;
    double throttle = c.throttle * (engine.running && !engine.seized ? 1 : 0);
    if (gb.shift_timer > 0) throttle = 0;
    double rpm_w = std::fabs(wheel_omega * ratio) * 60 / (2 * TT_PI);
    bool slip = gb.gear == 1 || ratio == 0 || (rpm_w < engine.idle + 250 && throttle > 0.02) || (rpm_w < engine.idle && throttle <= 0.02) || c.clutch > 0.5;
    double et;
    if (slip) {
        double free_rpm = engine.idle + throttle * 2800;
        engine.rpm = lerpd(engine.rpm, free_rpm, 0.004);
        et = throttle * engine.torque_at(engine.rpm) * (gb.gear == 1 ? 0 : 1) * clampd((engine.rpm - rpm_w) / 500, 0, 1);
    } else {
        engine.rpm = std::max(rpm_w, engine.idle);
        et = throttle * engine.torque_at(engine.rpm) - (throttle > 0.05 ? 0 : 20 + engine.rpm * 0.01);
        if (engine.rpm > engine.redline) et = std::min(et, -30.0);
    }
    if (engine.seized) engine.rpm = 0;
    // TCS
    if (tcs_on)
        for (auto &w : wheels)
            if (w.driven && w.contact && w.slip_ratio > 0.2 && std::fabs(speed) > 1) et *= clampd(1 - (w.slip_ratio - 0.2) * 4, 0.2, 1.0);
    double drive_each = nd ? et * ratio * 0.92 / nd : 0;
    // ---- tyres
    double esc_yaw = 0;
    for (size_t i = 0; i < wheels.size(); i++) {
        Wheel &w = wheels[i];
        if (w.detached) continue;
        Node &h = body.nodes[w.hub];
        double cs = std::cos(w.steer), sn = std::sin(w.steer);
        V3 fw = f * cs + r * sn, sw = r * cs - f * sn;
        GroundHit g = ground ? ground(h.p.x, h.p.y) : GroundHit{};
        double dz = dot(h.p - V3{h.p.x, h.p.y, g.h}, g.n);
        double R = w.radius * (w.flat ? 0.6 : 1.0);
        double pen = R - dz;
        w.contact = pen > 0;
        double t_drive = w.driven ? drive_each : 0;
        double bt = c.brake * w.brake_torque_max + (w.front ? 0 : c.handbrake * 3000);
        double fx = 0;
        if (w.contact) {
            V3 n = g.n;
            V3 fp = norm(fw - n * dot(fw, n)), sp = norm(sw - n * dot(sw, n));
            double vn = dot(h.v, n);
            double kt = 2.3e5 * (w.pressure / 2.2) * (w.flat ? 0.3 : 1.0);
            double Fn = std::max(0.0, kt * pen - 900 * vn);
            double vl = dot(h.v, fp), vs = dot(h.v, sp);
            double ref = std::max(std::fabs(vl), 2.5);
            double kappa = (w.omega * R - vl) / ref;
            double alpha = std::atan2(vs, std::max(std::fabs(vl), 1.0));
            double mu = g.friction * (1.0 - 0.35 * w.wear) * (w.flat ? 0.5 : 1.0);
            double dmax = mu * Fn;
            fx = dmax * magic(kappa, 10, 1.9);
            double fy = -dmax * magic(alpha, 8.5, 1.35);
            double e = std::sqrt(fx * fx + fy * fy) / std::max(dmax, 1.0);
            if (e > 1) fx /= e, fy /= e;
            double low = clampd(1 - std::fabs(vl) / 3, 0, 1);
            fy = lerpd(fy, clampd(-vs * Fn * 0.6, -dmax, dmax), low);
            if (abs_on && c.brake > 0 && kappa < -0.12 && std::fabs(vl) > 2) bt *= 0.25;
            h.f += n * Fn + fp * fx + sp * fy;
            w.load = Fn, w.slip_ratio = kappa, w.slip_angle = alpha, w.surface = g.surface;
            // wear and heat from sliding
            double sliding = std::fabs(kappa) * std::fabs(vl) + std::fabs(vs);
            w.wear = std::min(1.0, w.wear + sliding * Fn * dt * 2e-10);
            w.tyre_temp += (sliding * Fn * 2e-5 - (w.tyre_temp - 30) * 0.02) * dt;
            if (w.front && std::fabs(alpha) > 0.12) esc_yaw += alpha;
        } else {
            w.load = 0;
        }
        // wheel spin: drive, tyre reaction, brakes and rolling resistance (never reverse the spin)
        double inert = w.inertia + (w.driven && !slip ? engine.inertia * ratio * ratio / nd : 0);
        double om = w.omega + (t_drive - fx * R) / inert * dt;
        double stop = (bt + w.load * 0.012 * R) / inert * dt;
        om = std::fabs(om) <= stop ? 0 : om - (om > 0 ? 1 : -1) * stop;
        w.omega = om;
    }
    // ESC: brake the outer front wheel when the car slides sideways at speed
    if (esc_on && std::fabs(speed) > 8) {
        V3 v = body.nodes[ref_c].v;
        double beta = std::atan2(dot(v, r), std::max(std::fabs(dot(v, f)), 1.0));
        if (std::fabs(beta) > 0.12) {
            Wheel &w = wheels[beta > 0 ? 0 : 1];
            w.omega *= 0.995;
        }
    }
    // automatic gearbox
    if (gb.automatic && gb.shift_timer <= 0 && gb.gear >= 2) {
        if (engine.rpm > 6000 && gb.gear < int(gb.ratios.size()) - 1) gb.gear++, gb.shift_timer = 0.25;
        else if (engine.rpm < 2000 && gb.gear > 2) gb.gear--, gb.shift_timer = 0.25;
    }
    if (c.shift) {
        gb.gear = std::clamp(gb.gear + c.shift, 0, int(gb.ratios.size()) - 1);
        gb.shift_timer = 0.2;
        in.shift = 0;
    }
}

// ------------------------------------------------------------------ mechanical state (bolts, fluids, heat)

void Vehicle::mechanical_(double dt) {
    if (dt <= 0) return;
    // vibration: how much the chassis shakes (kerbs, broken parts, unbalanced wheels)
    V3 a = body.nodes[ref_c].v;
    double jerk = len(a - last_v_) / dt;
    last_v_ = a;
    vibration = lerpd(vibration, std::min(jerk, 400.0), clampd(dt * 3, 0, 1));
    // loose bolts work themselves looser; parts and wheels follow
    for (auto &b : bolts) {
        if (b.stripped) b.torque = 0;
        else if (b.torque < b.spec * 0.75) b.torque = std::max(0.0, b.torque - vibration * dt * 0.5 * (1.0 - b.torque / b.spec));
    }
    for (size_t i = 0; i < wheels.size(); i++) {
        double sum = 0;
        for (auto &b : bolts)
            if (b.wheel == int(i)) sum += b.torque / b.spec;
        if (sum < 0.25 && !wheels[i].detached) {
            wheels[i].detached = true;
            body.nodes[wheels[i].hub].collide = true;
            body.nodes[wheels[i].hub].radius = 0.12;
        }
    }
    for (auto &b : bolts)
        if (b.part >= 0) {
            double k = b.torque / b.spec;
            for (int bi : body.parts[b.part].attach) body.beams[bi].brk = std::min(body.beams[bi].brk, 0.05 + 0.3 * k);
        }
    // engine damage from impacts on the block / mounts, radiator leaks, heat
    Part &eng = body.parts[body.nodes[engine_node].part];
    Part &rad = body.parts[body.nodes[radiator_node].part];
    engine.damage = std::max(engine.damage, 1.0 - eng.health);
    if (rad.health < 0.7 || rad.detached) engine.coolant = std::max(0.0, engine.coolant - (1.0 - rad.health + (rad.detached ? 1 : 0)) * dt * 0.01);
    double heat = (engine.rpm / engine.redline) * (0.3 + in.throttle) * 18;
    double cool = engine.coolant > 0.3 && !rad.detached ? (engine.temp - 88) * 0.6 * engine.coolant : (engine.temp - 20) * 0.02;
    if (engine.running) engine.temp += (heat - std::max(0.0, cool) + (engine.temp < 88 ? 4 : 0)) * dt;
    else engine.temp += (20 - engine.temp) * 0.002 * dt;
    if (engine.temp > 120) engine.damage = std::min(1.0, engine.damage + (engine.temp - 120) * dt * 0.002);
    if (engine.oil < 0.2 && engine.running) engine.damage = std::min(1.0, engine.damage + dt * 0.01 * (engine.rpm / 3000));
    if (engine.damage > 0.97 || engine.temp > 145) engine.seized = true, engine.running = false;
    engine.knock = clampd(engine.damage * 1.3 - 0.2, 0, 1) * (engine.running ? 1 : 0);
    engine.wear = std::min(1.0, engine.wear + engine.rpm * dt * 1e-10);
}

// ------------------------------------------------------------------ workshop

bool Vehicle::tighten(int i, double torque) {
    if (i < 0 || i >= int(bolts.size())) return false;
    Bolt &b = bolts[i];
    if (b.stripped) return false;
    if (torque > b.spec * 1.45) {
        b.stripped = true;  // overtightened: the thread is gone
        b.torque = 0;
        return false;
    }
    b.torque = torque;
    if (b.wheel >= 0) {
        double sum = 0;
        for (auto &o : bolts)
            if (o.wheel == b.wheel) sum += o.torque / o.spec;
        if (sum > 2.5 && wheels[b.wheel].detached) {
            wheels[b.wheel].detached = false;
            body.nodes[wheels[b.wheel].hub].collide = false;
        }
    }
    return true;
}

void Vehicle::replace_part(int p) {
    if (p < 0 || p >= int(body.parts.size())) return;
    Part &pt = body.parts[p];
    for (int b : pt.beams) {
        auto &bm = body.beams[b];
        bm.L0 = bm.L0_build, bm.broken = false, bm.plastic = 0, bm.fatigue = 0;
    }
    for (int b : pt.attach) {
        auto &bm = body.beams[b];
        bm.L0 = bm.L0_build, bm.broken = false, bm.plastic = 0, bm.fatigue = 0;
    }
    // put the part's nodes back where they belong relative to the chassis frame
    V3 o, r, f, u;
    frame(o, r, f, u);
    V3 o0 = body.nodes[ref_c].rest;
    for (int n : pt.nodes) {
        V3 d = body.nodes[n].rest - o0;
        body.nodes[n].p = o + r * d.x + f * d.y + u * d.z;
        body.nodes[n].v = body.nodes[ref_c].v;
    }
    pt.health = 1, pt.detached = false;
    if (p == body.nodes[engine_node].part) engine.damage = 0, engine.seized = false, engine.running = true;
    if (p == body.nodes[radiator_node].part) engine.coolant = 1;
}

void Vehicle::refill() {
    engine.oil = 1, engine.coolant = 1;
}

double Vehicle::part_health(const std::string &name) const {
    for (auto &p : body.parts)
        if (p.name == name) return p.detached ? 0 : p.health;
    return 1;
}

std::string Vehicle::damage_report() const {
    std::string s;
    char b[160];
    for (auto &p : body.parts) {
        snprintf(b, sizeof b, "%s %.0f%%%s; ", p.name.c_str(), p.health * 100, p.detached ? " (off)" : "");
        s += b;
    }
    snprintf(b, sizeof b, "engine dmg %.0f%% temp %.0fC coolant %.0f%%", engine.damage * 100, engine.temp, engine.coolant * 100);
    return s + b;
}

}  // namespace tt
