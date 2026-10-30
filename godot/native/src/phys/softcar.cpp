#include "softcar.hpp"
#include <algorithm>
#include <functional>

namespace ec {

namespace {
const double STATIONS[6] = {-2.05, -1.45, -0.50, 0.45, 1.25, 2.30};
const double YS[3] = {-0.80, 0.0, 0.80};
const double ZS[2] = {0.30, 0.92};
const int NS = 6, FRONT_STATION = 4, REAR_STATION = 1;
const double TOWER_Z = 0.75, HUB_MASS = 28, TOWER_MASS = 5;
const double CHASSIS_EA = 2.0e6, LINK_EA = 6.0e6, CRUMPLE = 0.35;
const double TIRE_K = 2.0e5, TIRE_C = 1200, G = 9.81, HANDBRAKE_GRIP_LOSS = 0.45;

int cage_index(int si, int yi, int zi) { return si * 6 + yi * 2 + zi; }

double shape(double s, double sliding = 0.707) {
    // Pacejka-like rise to 1 at s = 1, then falling towards `sliding`
    const double C = 2 - 2 * std::asin(std::clamp(sliding, 0.3, 0.98)) / PI;
    double b = std::tan(PI / (2 * C));
    return std::sin(C * std::atan(b * s));
}

double apply_brake(double w, double tb, double dt, double I) {
    if (tb <= 0) return w;
    double dw = dt * tb / I;
    if (std::abs(w) <= dw) return 0;
    return w - (w > 0 ? dw : -dw);
}

double smoothstep(double a, double b, double x) {
    double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3 - 2 * t);
}
}  // namespace

TireOut tire_force(const TireParams &p, double fz, double surface, double vl, double vt, double omega, double radius) {
    TireOut o;
    if (fz <= 0) return o;
    double den = std::max(std::abs(vl), p.v0);
    double kappa = (omega * radius - vl) / den, ta = vt / den;
    double sx = kappa / p.kappa_peak, sy = ta / p.tan_alpha_peak;
    double s = std::sqrt(sx * sx + sy * sy);
    double load = p.bl_slope > 0 ? p.bl_full + (p.bl_none - p.bl_full) * std::exp(-p.bl_slope * fz) : std::clamp(1 - p.load_sens * (fz / p.fz0 - 1), 0.6, 1.3);
    double limit = p.mu * surface * load * fz;
    o.kappa = kappa, o.tan_alpha = ta, o.limit = limit;
    if (s < 1e-9) return o;
    double f = limit * shape(s, p.sliding);
    o.fx = f * sx / s;
    o.fy = -f * sy / s;
    return o;
}

double Drivetrain::ratio(const DriveParams &p) const {
    if (gear < 0) return -p.reverse * p.final_drive;
    return p.gears[std::min<size_t>(gear - 1, p.gears.size() - 1)] * p.final_drive;
}

void Drivetrain::step(double dt, const DriveParams &p, double throttle, bool reverse, double wl, double wr, double &tl, double &tr, double &stiff) {
    const double RPM = 60 / (2 * PI);
    dwell += dt;
    if (shift_t > 0) shift_t = std::max(0.0, shift_t - dt);
    if (reverse) {
        if (gear != -1) gear = -1, shift_t = p.shift_time;
    } else {
        if (gear < 1) gear = 1, shift_t = p.shift_time;
        double r = rpm();
        double upr = 2200 + 3600 * throttle, downr = 1500 + 1500 * throttle;
        if (shift_t == 0 && dwell > 1.0) {
            if (r > upr && gear < (int)p.gears.size()) gear++, shift_t = p.shift_time, dwell = 0;
            else if (r < downr && gear > 1) gear--, shift_t = p.shift_time, dwell = 0;
        }
    }
    double r = rpm();
    double thr = throttle * (r > p.max_rpm ? 0 : 1);
    auto curve = [&](double x) {
        auto &c = p.torque_curve;
        if (x <= c[0].first) return c[0].second;
        for (size_t i = 1; i < c.size(); i++)
            if (x <= c[i].first) return c[i - 1].second + (x - c[i - 1].first) / (c[i].first - c[i - 1].first) * (c[i].second - c[i - 1].second);
        return c.back().second;
    };
    double te = thr * curve(r) - (1 - thr) * (25 + 0.012 * r);
    if (r < p.idle_rpm) te += std::clamp(p.idle_rpm - r, 0.0, 150.0);
    engine_torque = te;
    double ra = ratio(p);
    double w_in = ra * 0.5 * (wl + wr);
    double engage = shift_t > 0 ? 0 : smoothstep(1000, 1700, r);
    clutch = engage;
    double cap = p.clutch_max * engage;
    double raw = p.clutch_k * (w_e - w_in);
    double tc = std::clamp(raw, -cap, cap);
    clutch_torque = tc;
    w_e += dt * (te - tc) / p.engine_inertia;
    w_e = std::max(w_e, 0.4 * p.idle_rpm / RPM);
    double total = tc * ra * p.efficiency;
    double lsd = std::clamp(p.lsd_k * (wl - wr), -p.lsd_max, p.lsd_max) * (engage > 0 ? 1 : 0);
    stiff = std::abs(raw) < cap ? p.clutch_k * ra * ra * p.efficiency * 0.25 : 0;
    tl = total * 0.5 - lsd;
    tr = total * 0.5 + lsd;
}

SoftCar::SoftCar(const CarParams &cp) : p(cp) {
    double rw = p.wheel_radius, cg_z = 0.58;
    double hub_x[4] = {STATIONS[FRONT_STATION], STATIONS[FRONT_STATION], STATIONS[REAR_STATION], STATIONS[REAR_STATION]};
    double hw = p.track * 0.5;
    double hub_y[4] = {hw, -hw, hw, -hw};
    int n_cage = NS * 3 * 2;
    double m_cage = p.mass - 4 * HUB_MASS - 4 * TOWER_MASS;
    std::vector<V3> pr;
    for (int s = 0; s < NS; s++)
        for (int y = 0; y < 3; y++)
            for (int z = 0; z < 2; z++) pr.push_back({STATIONS[s], YS[y], ZS[z]});
    double fixed_mx = 0, fixed_mz = 4 * HUB_MASS * rw + 4 * TOWER_MASS * TOWER_Z;
    for (int i = 0; i < 4; i++) fixed_mx += (HUB_MASS + TOWER_MASS) * hub_x[i];
    auto weights = [&](double b, double f) {
        std::vector<double> w;
        for (size_t i = 0; i < pr.size(); i++) {
            int zi = i % 2, yi = (i / 2) % 3;
            w.push_back((1 + b * pr[i].x) * (0.12 + std::exp(-std::pow(pr[i].x / 1.2, 2))) * (zi == 0 ? f : 1.0) * (yi == 1 ? 2.0 : 1.0));
        }
        return w;
    };
    auto moments = [&](double b, double f, double &mx, double &mz) {
        auto w = weights(b, f);
        double sw = 0;
        for (double x : w) sw += x;
        mx = mz = 0;
        for (size_t i = 0; i < w.size(); i++) mx += w[i] / sw * m_cage * pr[i].x, mz += w[i] / sw * m_cage * pr[i].z;
    };
    double b = 0, f = 1;
    for (int it = 0; it < 40; it++) {
        double lo = -0.3, hi = 0.3, mx, mz;
        for (int k = 0; k < 50; k++) {
            double mid = 0.5 * (lo + hi);
            moments(mid, f, mx, mz);
            if (mx + fixed_mx > 0) hi = mid;
            else lo = mid;
        }
        b = 0.5 * (lo + hi);
        lo = 0.2, hi = 6;
        for (int k = 0; k < 50; k++) {
            double mid = 0.5 * (lo + hi);
            moments(b, mid, mx, mz);
            if (mz + fixed_mz > p.mass * cg_z) lo = mid;
            else hi = mid;
        }
        f = 0.5 * (lo + hi);
    }
    auto w = weights(b, f);
    double sw = 0;
    for (double x : w) sw += x;
    for (int i = 0; i < n_cage; i++) {
        int id = body.add_node(pr[i], w[i] / sw * m_cage);
        body.nodes[id].radius = 0.1;
        body.nodes[id].hit_ground = body.nodes[id].hit_world = true;
        cage.push_back(id);
    }
    for (int si = 0; si < NS; si++)
        for (int yi = 0; yi < 3; yi++)
            for (int zi = 0; zi < 2; zi++) {
                int a = cage_index(si, yi, zi);
                for (int ds = -1; ds <= 1; ds++)
                    for (int dy = -1; dy <= 1; dy++)
                        for (int dz = -1; dz <= 1; dz++) {
                            int s2 = si + ds, y2 = yi + dy, z2 = zi + dz;
                            if (s2 < 0 || y2 < 0 || z2 < 0 || s2 >= NS || y2 >= 3 || z2 >= 2) continue;
                            int bn = cage_index(s2, y2, z2);
                            if (bn <= a) continue;
                            double len = (body.nodes[bn].p - body.nodes[a].p).len();
                            double k = CHASSIS_EA / len;
                            double me = body.nodes[a].m * body.nodes[bn].m / (body.nodes[a].m + body.nodes[bn].m);
                            int bi = body.add_beam(a, bn, k, 2 * 0.08 * std::sqrt(k * me));
                            V3 dir = (body.nodes[bn].p - body.nodes[a].p) * (1 / len);
                            double along = std::abs(dir.x);
                            auto zone = [](int s) { return (s >= 4 || s <= 1) ? CRUMPLE : 1.0; };
                            double fz = std::min(zone(si), zone(s2));
                            body.beams[bi].yield_c = (12000 + 14000 * along) * fz;
                            body.beams[bi].yield_t = (18000 + 14000 * along) * fz;
                            body.beams[bi].break_strain = 0.35;
                        }
            }
    for (int i = 0; i < 4; i++) {
        int ax = i / 2;
        bool front = ax == 0, lft = i % 2 == 0;
        int st = front ? FRONT_STATION : REAR_STATION;
        int yi = lft ? 2 : 0, yo = 2 - yi;
        int hub = body.add_node({hub_x[i], hub_y[i], rw - 0.017}, HUB_MASS);
        body.nodes[hub].radius = rw * 0.9;
        body.nodes[hub].hit_world = true;
        hubs[i] = hub;
        int tower = body.add_node({hub_x[i], hub_y[i] * 0.9, TOWER_Z}, TOWER_MASS);
        towers[i] = tower;
        cage.push_back(tower);
        std::vector<std::pair<double, int>> nearv;
        for (int c = 0; c < n_cage; c++) nearv.push_back({(body.nodes[c].p - body.nodes[tower].p).len(), c});
        std::sort(nearv.begin(), nearv.end());
        for (int k = 0; k < 6; k++) {
            double len = nearv[k].first;
            double kk = CHASSIS_EA * 2 / len;
            int bi = body.add_beam(tower, nearv[k].second, kk, 2 * 0.1 * std::sqrt(kk * TOWER_MASS));
            body.beams[bi].yield_t = body.beams[bi].yield_c = 40000;
            body.beams[bi].break_strain = 0.35;
        }
        int la = cage_index(front ? FRONT_STATION - 1 : REAR_STATION + 1, yi, 0), lt = cage_index(st, yo, 0);
        long_anchor[i] = la, lat_anchor[i] = lt;
        for (int anchor : {la, lt}) {
            double len = (body.nodes[anchor].p - body.nodes[hub].p).len();
            double kk = LINK_EA / len;
            double me = HUB_MASS * body.nodes[anchor].m / (HUB_MASS + body.nodes[anchor].m);
            int bi = body.add_beam(hub, anchor, kk, 2 * 0.1 * std::sqrt(kk * me));
            body.beams[bi].yield_t = body.beams[bi].yield_c = 45000;
            body.beams[bi].break_strain = 0.3;
        }
        double wb = STATIONS[FRONT_STATION] - STATIONS[REAR_STATION];
        double static_load = p.mass * G * (front ? (-STATIONS[REAR_STATION]) / wb : STATIONS[FRONT_STATION] / wb) / 2 - HUB_MASS * G;
        double len_now = TOWER_Z - (rw - 0.017);
        double rest = len_now + static_load / p.spring[ax];
        spring_rest_[ax] = rest;
        int bi = body.add_beam(hub, tower, p.spring[ax], p.damper[ax]);
        body.beams[bi].rest = body.beams[bi].rest0 = rest;
        body.beams[bi].has_stops = true;
        body.beams[bi].stops = {len_now - 0.09, rest + 0.01, p.bump_k, 2500};
    }
    p.wheelbase = STATIONS[FRONT_STATION] - STATIONS[REAR_STATION];
    double m = 0;
    for (int i : cage) rest_com_ = rest_com_ + body.nodes[i].rest * body.nodes[i].m, m += body.nodes[i].m;
    rest_com_ = rest_com_ * (1 / m);
    drive.w_e = p.drive.idle_rpm * 2 * PI / 60;
    refresh_frame();
}

void SoftCar::axes(V3 &f, V3 &l, V3 &u) const {
    V3 fr, re, le, ri;
    int nf = 0, nr = 0, nl = 0, nri = 0;
    for (int si = 0; si < NS; si++)
        for (int yi = 0; yi < 3; yi++)
            for (int zi = 0; zi < 2; zi++) {
                V3 q = body.nodes[cage_index(si, yi, zi)].p;
                if (si >= 4) fr = fr + q, nf++;
                if (si <= 1) re = re + q, nr++;
                if (yi == 2) le = le + q, nl++;
                if (yi == 0) ri = ri + q, nri++;
            }
    f = (fr * (1.0 / nf) - re * (1.0 / nr)).norm();
    V3 lv = (le * (1.0 / nl) - ri * (1.0 / nri)).norm();
    u = f.cross(lv).norm();
    l = u.cross(f).norm();
}

void SoftCar::refresh_frame() {
    axes(fwd, left, up);
    V3 c, v;
    double m = 0;
    for (int i : cage) {
        auto &n = body.nodes[i];
        c = c + n.p * n.m, v = v + n.v * n.m, m += n.m;
    }
    pos = c * (1 / m);
    vel = v * (1 / m);
}

V3 SoftCar::to_world(V3 r) const {
    V3 d = r - rest_com_;
    return pos + fwd * d.x + left * d.y + up * d.z;
}

double SoftCar::damage(int *broken) const {
    double worst = 0;
    for (int i : cage) worst = std::max(worst, (body.nodes[i].p - to_world(body.nodes[i].rest)).len());
    if (broken) {
        int b = 0;
        for (auto &be : body.beams) b += be.broken;
        *broken = b;
    }
    return worst;
}

void SoftCar::place(const CollisionWorld &w, double x, double y, double z_hint, double yaw, double speed) {
    Surface s = w.surface(x, y, z_hint, 50);
    double h = s.found ? s.height : z_hint;
    double c = std::cos(yaw), sn = std::sin(yaw);
    V3 f(c, sn, 0), l(-sn, c, 0);
    for (auto &n : body.nodes) {
        n.p = V3(x, y, h + 0.02) + f * n.rest.x + l * n.rest.y + V3(0, 0, n.rest.z);
        n.v = f * speed;
        n.f = V3();
    }
    for (auto &b : body.beams) b.broken = false, b.rest = b.rest0;
    body.plastic_work = 0;
    body.diverged = false;
    for (auto &wh : wheels) wh = WheelState(), wh.w = speed / p.wheel_radius;
    drive = Drivetrain();
    drive.w_e = p.drive.idle_rpm * 2 * PI / 60;
    time = 0, impact = 0;
    refresh_frame();
}

void SoftCar::step(double dt, const CarInput &in, const CollisionWorld &w) {
    int n = std::max(1, (int)std::lround(dt / sub_dt));
    double h = dt / n;
    double imp = 0;
    V3 v0 = vel;
    for (int i = 0; i < n; i++) {
        substep(h, in, w);
        imp = std::max(imp, body.peak_world);
    }
    impact = imp;
    refresh_frame();
    V3 a = (vel - v0) * (1 / dt) - V3(0, 0, -G);
    V3 spec(a.dot(fwd), a.dot(left), a.dot(up));
    double k = std::min(dt / 0.1, 1.0);
    if (std::isfinite(spec.x)) accel = accel * (1 - k) + spec * k;
    time += dt;
}

void SoftCar::substep(double dt, const CarInput &in, const CollisionWorld &w) {
    refresh_frame();
    double rw = p.wheel_radius;
    body.clear_forces();
    body.beam_forces(dt);
    body.slide_forces();
    double sp = vel.len();
    V3 drag = vel * (-0.5 * 1.2 * p.cd_a * sp);
    double mc = 0;
    for (int i : cage) mc += body.nodes[i].m;
    for (int i : cage) body.nodes[i].f = body.nodes[i].f + drag * (body.nodes[i].m / mc);
    // drivetrain: front-wheel drive (or all four when awd)
    int da = 0, db = 1;
    double thr = in.throttle;
    if (in.tcs && in.handbrake < 0.1 && (wheels[da].contact || wheels[db].contact)) {
        double k = std::max(wheels[da].kappa, wheels[db].kappa);
        thr *= std::clamp(1 - (k - 0.22) / 0.15, 0.0, 1.0);
    }
    double tl, tr, stiff;
    double wl = p.awd ? 0.5 * (wheels[0].w + wheels[2].w) : wheels[da].w, wr = p.awd ? 0.5 * (wheels[1].w + wheels[3].w) : wheels[db].w;
    drive.step(dt, p.drive, thr, in.reverse, wl, wr, tl, tr, stiff);
    double t = std::tan(in.steer), half = p.track * 0.5, L = p.wheelbase;
    double steer_l = std::atan2(L * t, L - half * t), steer_r = std::atan2(L * t, L + half * t);
    for (int i = 0; i < 4; i++) {
        bool front = i < 2;
        double delta = front ? (i == 0 ? steer_l : steer_r) : 0;
        wheels[i].steer = delta;
        int hub = hubs[i];
        V3 hp = body.nodes[hub].p, hv = body.nodes[hub].v;
        // circle-vs-ground: centre sample plus fore/aft samples so kerbs push the wheel up
        Surface s; V3 n(0, 0, 1); double pen = -1;
        for (int k = -2; k <= 2; k++) {
            V3 q = hp + fwd * (k * 0.35 * rw);
            Surface si = w.surface(q.x, q.y, hp.z - rw * 0.55, rw * 0.5);
            if (!si.found) continue;
            V3 gp(q.x, q.y, si.height);
            if (k == 0) {
                double pi = rw - (hp - gp).dot(si.normal);
                if (pi > pen) pen = pi, s = si, n = si.normal;
            } else {
                V3 d = hp - gp; double dl = d.len();
                if (dl < 1e-6 || d.z < 0.2 * dl) continue;
                double pi = rw - dl;
                if (pi > pen + 0.002) pen = pi, s = si, n = d * (1 / dl);
            }
        }
        bool contact = pen > 0;
        double t_drive = 0, st = 0;
        if (p.awd) {
            t_drive = (i % 2 == 0 ? tl : tr) * 0.5, st = stiff * 0.5;
        } else if (i == da) t_drive = tl, st = stiff;
        else if (i == db) t_drive = tr, st = stiff;
        double tb = in.brake * (front ? p.brake_front : p.brake_rear);
        if (in.abs && in.brake > 0.05 && wheels[i].contact) {
            double slip = -wheels[i].kappa;
            if (slip > 0.12) tb *= std::clamp(1 - (slip - 0.12) / 0.1, 0.05, 1.0);
        }
        if (!front) tb += in.handbrake * p.handbrake * (1 - 0.75 * thr);
        double w0 = wheels[i].w, wn, fz = 0;
        TireOut out;
        V3 pc = hp;
        if (contact) {
            double vn = hv.dot(n);
            fz = std::max(0.0, TIRE_K * pen - TIRE_C * vn);
            pc = hp - n * rw;
            V3 head = fwd * std::cos(delta) + left * std::sin(delta);
            V3 fp = (head - n * head.dot(n)).norm();
            V3 lat = n.cross(fp);
            double vl = hv.dot(fp), vt = hv.dot(lat);
            double grip = s.grip * (front ? 1.0 : 1.0 - HANDBRAKE_GRIP_LOSS * in.handbrake);
            TireParams tp = p.tire;
            if (tb > 0 && std::abs(w0) < 0.5 && sp < 1.0) tp.v0 = 0.4;
            TireOut o1 = tire_force(tp, fz, grip, vl, vt, w0, rw), o2 = tire_force(tp, fz, grip, vl, vt, w0 + 1, rw);
            double jx = std::max(0.0, o2.fx - o1.fx);
            double den = p.wheel_inertia + dt * (rw * jx + st);
            double pre = w0 + dt * (t_drive - rw * o1.fx) / den;
            wn = apply_brake(pre, tb, dt, p.wheel_inertia);
            out = tire_force(tp, fz, grip, vl, vt, wn, rw);
            double rr = -p.crr * fz * std::tanh(vl / 0.3);
            V3 ft = fp * (out.fx + rr) + lat * out.fy;
            body.nodes[hub].f = body.nodes[hub].f + n * fz + ft;
            V3 mcpl = (pc - hp).cross(ft);
            double ml = mcpl.dot(left), mf = mcpl.dot(fwd);
            int tw = towers[i], la = long_anchor[i], lt = lat_anchor[i];
            double dx = (body.nodes[tw].p - body.nodes[la].p).dot(fwd), dy = (body.nodes[tw].p - body.nodes[lt].p).dot(left);
            if (std::abs(dx) > 0.2) {
                V3 fv = up * (-ml / dx);
                body.nodes[tw].f = body.nodes[tw].f + fv;
                body.nodes[la].f = body.nodes[la].f - fv;
            }
            if (std::abs(dy) > 0.2) {
                V3 fv = up * (mf / dy);
                body.nodes[tw].f = body.nodes[tw].f + fv;
                body.nodes[lt].f = body.nodes[lt].f - fv;
            }
        } else {
            double pre = w0 + dt * t_drive / (p.wheel_inertia + dt * st);
            wn = apply_brake(pre, tb, dt, p.wheel_inertia);
        }
        if (!std::isfinite(wn)) wn = 0;
        WheelState &wh = wheels[i];
        wh.w = wn;
        wh.spin += wn * dt;
        wh.fz = fz;
        wh.contact = contact;
        wh.kappa = out.kappa;
        wh.tan_alpha = out.tan_alpha;
        wh.fx = out.fx, wh.fy = out.fy;
        wh.locked = tb > 0 && wn == 0;
        wh.pc = pc;
        // compression: hub height relative to the tower along body up
        double ax = (body.nodes[towers[i]].p - hp).dot(up);
        wh.comp = std::max(0.0, spring_rest_[i / 2] - ax);
    }
    for (int ax = 0; ax < 2; ax++) {
        int l = ax * 2, r = l + 1;
        double fa = p.arb[ax] * (wheels[l].comp - wheels[r].comp);
        V3 fu = up * fa;
        body.nodes[towers[l]].f = body.nodes[towers[l]].f + fu;
        body.nodes[hubs[l]].f = body.nodes[hubs[l]].f - fu;
        body.nodes[towers[r]].f = body.nodes[towers[r]].f - fu;
        body.nodes[hubs[r]].f = body.nodes[hubs[r]].f + fu;
    }
    body.contacts(w, dt);
    body.integrate(dt);
    body.resolve_world(w, dt);
}

CarInput Driver::update(double dt, bool upk, bool downk, bool lk, bool rk, bool hb, double v) {
    double thr = 0, brk = 0;
    if (upk && !downk) {
        if (v < -1) brk = 1;
        else thr = 1, reverse = false;
    } else if (downk && !upk) {
        if (v > 1) brk = 1;
        else thr = 1, reverse = true;
    }
    auto rise = [&](double cur, double tgt, double u, double d) { return tgt > cur ? std::min(cur + u * dt, tgt) : std::max(cur - d * dt, tgt); };
    throttle = rise(throttle, thr, 4, 8);
    brake = rise(brake, brk, 6, 10);
    handbrake = hb ? 1 : 0;
    double lock = steer_max / (1 + std::pow(std::abs(v) / 14, 2));
    double dir = (lk ? 1 : 0) - (rk ? 1 : 0);
    double target = dir * lock;
    double rate = dir == 0 ? 3.5 : 2.2;
    double stp = rate * dt * steer_max;
    steer += std::clamp(target - steer, -stp, stp);
    steer = std::clamp(steer, -steer_max, steer_max);
    if (std::abs(steer) > lock && dir != 0) steer = (steer > 0 ? 1 : -1) * lock;
    CarInput in;
    in.throttle = throttle, in.brake = brake, in.steer = steer, in.handbrake = handbrake, in.reverse = reverse;
    // hold the car when stopped with no pedal pressed (like an automatic's hill hold)
    if (thr == 0 && brk == 0 && std::abs(v) < 1.0) in.brake = std::max(in.brake, 0.35);
    return in;
}

CarInput Driver::update_analog(double axis, double thr, double brk, bool hb, double v) {
    thr = std::clamp(thr, 0.0, 1.0), brk = std::clamp(brk, 0.0, 1.0);
    if (thr > 0.05 && v > -1) reverse = false;
    if (brk > 0.5 && thr < 0.05 && std::abs(v) < 0.5) reverse = true;
    if (reverse) {
        throttle = v < 1 ? brk : 0;
        brake = std::max(thr, v > 1 ? brk : 0.0);
    } else
        throttle = thr, brake = brk;
    steer = std::clamp(axis, -1.0, 1.0) * steer_max;
    handbrake = hb ? 1 : 0;
    CarInput in;
    in.throttle = throttle, in.brake = brake, in.steer = steer, in.handbrake = handbrake, in.reverse = reverse;
    return in;
}

}  // namespace ec
