#include "jvehicle.hpp"
#include <algorithm>
#include <functional>
#include <unordered_map>

namespace ec {

namespace {
const double TIRE_K = 2.2e5, TIRE_C = 1500, WHEEL_EXTRA_MASS = 16;

V3 ours(const std::array<double, 3> &p) {
    auto q = to_ours(p);
    return V3(q[0], q[1], q[2]);
}
int idx(const JModel &m, const std::string &name) {
    if (name.empty()) return -1;
    auto it = m.node_index.find(name);
    return it == m.node_index.end() ? -1 : it->second;
}
double apply_brake(double w, double tb, double dt, double I) {
    if (tb <= 0) return w;
    double dw = dt * tb / I;
    if (std::abs(w) <= dw) return 0;
    return w - (w > 0 ? dw : -dw);
}
}  // namespace

bool DamageGroups::is_triggered(const std::string &n) const {
    for (size_t i = 0; i < names.size(); i++)
        if (names[i] == n) return triggered[i];
    return false;
}
void DamageGroups::reset() {
    std::fill(triggered.begin(), triggered.end(), false);
    std::fill(broken_groups.begin(), broken_groups.end(), false);
    generation++;
}
void DamageGroups::update(SoftBody &b) {
    for (size_t g = 0; g < break_groups.size(); g++) {
        if (broken_groups[g]) continue;
        bool any = false;
        for (int bi : break_groups[g].second) any |= b.beams[bi].broken;
        if (any) {
            for (int bi : break_groups[g].second) b.beams[bi].broken = true;
            broken_groups[g] = true;
            generation++;
        }
    }
    for (auto &[bi, g, ratio] : triggers) {
        if (triggered[g]) continue;
        auto &be = b.beams[bi];
        if (be.broken || std::abs(be.rest - be.rest0) > ratio * be.rest0) triggered[g] = true, generation++;
    }
}

DriveParams drivetrain_params(const JModel &m, std::vector<std::string> &notes) {
    DriveParams p;
    const Json *eng = nullptr, *gb = nullptr;
    for (auto &pt : m.asm_.parts) {
        const Json *e = pt.body.get("mainEngine");
        if (!eng && e && e->get("torque")) eng = e;
        const Json *g = pt.body.get("gearbox");
        if (!gb && g && g->get("gearRatios")) gb = g;
    }
    if (eng) {
        const Json *t = eng->get("torque");
        if (t && t->kind == Json::Arr) {
            std::vector<std::pair<double, double>> curve;
            for (auto &r : t->a)
                if (r.kind == Json::Arr && r.a.size() >= 2 && r.a[0].kind == Json::Num && r.a[1].kind == Json::Num && r.a[0].n > 0) curve.push_back({r.a[0].n, r.a[1].n});
            if (curve.size() >= 2) p.torque_curve = curve;
        }
        if (const Json *v = eng->get("idleRPM"); v && v->kind == Json::Num) p.idle_rpm = v->n;
        if (const Json *v = eng->get("maxRPM"); v && v->kind == Json::Num) p.max_rpm = v->n;
        if (const Json *v = eng->get("inertia"); v && v->kind == Json::Num) p.engine_inertia = std::max(0.15, v->n);
        double peak = 0;
        for (auto &c : p.torque_curve) peak = std::max(peak, c.second);
        notes.push_back("engine from mod: peak " + std::to_string((int)peak) + " Nm");
    } else
        notes.push_back("engine: none in mod, using default");
    if (gb) {
        std::vector<double> fwd;
        for (auto &v : gb->get("gearRatios")->a)
            if (v.kind == Json::Num) {
                if (v.n > 0) fwd.push_back(v.n);
                else if (v.n < 0 && p.reverse == 3.4) p.reverse = -v.n;
            }
        if (!fwd.empty()) p.gears = fwd;
    }
    double fd = -1;
    for (auto &[pi, r] : m.table("powertrain"))
        if (r.str("type") == "differential" && r.has_num("gearRatio")) {
            fd = r.num("gearRatio");
            break;
        }
    p.final_drive = fd > 0 ? fd : 4.1;
    if (fd <= 0) notes.push_back("final drive not in mod: 4.1 assumed");
    return p;
}

bool JVehicle::from_model(const JModel &m, JVehicle &v, std::string &err) {
    SoftBody &body = v.body;
    for (auto &n : m.nodes) {
        int i = body.add_node(ours(n.pos), std::max(0.05, n.mass));
        body.nodes[i].radius = 0.03;
        body.nodes[i].hit_ground = body.nodes[i].hit_world = n.collision;
    }
    for (auto &[pi, r] : m.table("nodes")) {
        const Json *sc = r.get("staticCollision");
        if (sc && sc->kind == Json::Bool && !sc->b) {
            int i = idx(m, r.str("id"));
            if (i >= 0) body.nodes[i].hit_ground = body.nodes[i].hit_world = false;
        }
    }
    std::unordered_map<std::string, int> bg_index;
    for (auto &b : m.beams) {
        int bi = body.add_beam(b.a, b.b, b.spring(), b.damp());
        for (auto &name : b.row.strings("breakGroup")) {
            auto it = bg_index.find(name);
            int gi;
            if (it == bg_index.end()) {
                gi = (int)v.groups.break_groups.size();
                bg_index[name] = gi;
                v.groups.break_groups.push_back({name, {}});
            } else
                gi = it->second;
            v.groups.break_groups[gi].second.push_back(bi);
        }
        if (b.row.has_num("deformationTriggerRatio")) {
            double ratio = std::max(0.0, b.row.num("deformationTriggerRatio"));
            for (auto &name : b.row.strings("deformGroup")) {
                auto it = std::find(v.groups.names.begin(), v.groups.names.end(), name);
                int gi = (int)(it - v.groups.names.begin());
                if (it == v.groups.names.end()) v.groups.names.push_back(name);
                v.groups.triggers.push_back({bi, gi, (float)ratio});
            }
        }
        SBeam &be = body.beams[bi];
        double pre = b.row.num("beamPrecompression", 1);
        if (pre > 0 && std::abs(pre - 1) > 1e-9) be.rest = be.rest0 = be.rest0 * pre;
        if (b.row.has_num("precompressionRange")) be.rest += b.row.num("precompressionRange"), be.rest0 = be.rest;
        double d = b.deform();
        if (d >= 0) be.yield_t = be.yield_c = d;
        double s = b.strength();
        if (s >= 0) be.break_force = s;
        if (b.row.has_num("beamDampRebound")) be.c_reb = b.row.num("beamDampRebound");
        be.break_strain = 1.5;
        if (b.kind == "SUPPORT") {
            be.support = true;
            be.break_strain = std::max(0.05, b.row.num("beamLongBound", 1.0));
        } else if (b.kind == "BOUNDED") {
            double lng = b.row.num("beamLongBound", 1), sh = b.row.num("beamShortBound", 1), lk = b.row.num("beamLimitSpring", 0), lc = b.row.num("beamLimitDamp", 0);
            if (lk > 0 || lc > 0) {
                be.has_stops = true;
                be.stops = {be.rest * std::max(0.0, 1 - sh), be.rest * (1 + lng), lk, lc};
            }
            be.break_strain = 1e300;
        }
    }
    // rails and slide nodes
    std::unordered_map<std::string, std::pair<std::vector<int>, bool>> rails;
    for (auto &p : m.asm_.parts) {
        const Json *rs = p.body.get("rails");
        if (!rs || rs->kind != Json::Obj) continue;
        for (auto &[name, r] : rs->o) {
            const Json *l = r.get("links:");
            if (!l) l = r.get("links");
            std::vector<int> links;
            if (l && l->kind == Json::Arr)
                for (auto &n : l->a)
                    if (n.kind == Json::Str && idx(m, n.s) >= 0) links.push_back(idx(m, n.s));
            const Json *cp = r.get("capped");
            bool capped = !(cp && cp->kind == Json::Bool && !cp->b);
            if (links.size() >= 2) rails[name] = {links, capped};
        }
    }
    for (auto &[pi, r] : m.table("slidenodes")) {
        int n = idx(m, r.str("id"));
        auto it = rails.find(r.str("railName"));
        if (n < 0 || it == rails.end()) continue;
        double k = r.num("spring", 1e7);
        body.slides.push_back({n, it->second.first, k, 2 * 0.2 * std::sqrt(k * body.nodes[n].m), it->second.second});
    }
    // couplers (doors, hood, tailgate): closed at spawn, open above their strength
    for (auto &p : m.asm_.parts) {
        if (p.body.kind != Json::Obj) continue;
        for (auto &[k, sec] : p.body.o) {
            const Json *cn = sec.get("couplerNodes");
            if (!cn) continue;
            for (auto &r : jbeam_rows(*cn)) {
                int a = idx(m, r.str("cid1")), b = idx(m, r.str("cid2"));
                if (a < 0 || b < 0) continue;
                double mm = std::min(body.nodes[a].m, body.nodes[b].m);
                double kk = std::min(0.5 * mm / (v.sub_dt * v.sub_dt) * 0.25, 2e6);
                body.pins.push_back({a, b, kk, 2 * std::sqrt(kk * mm) * 0.3, r.num("autoCouplingStrength", 30000)});
            }
        }
    }
    for (auto &[pi, r] : m.table("hydros")) {
        int a = idx(m, r.str("id1")), b = idx(m, r.str("id2"));
        if (a < 0 || b < 0) continue;
        double len = (body.nodes[b].p - body.nodes[a].p).len();
        int bi = body.add_beam(a, b, r.num("beamSpring", 4.3e6), r.num("beamDamp", 580));
        if (r.has_num("beamStrength")) body.beams[bi].break_force = r.num("beamStrength");
        v.hydros.push_back({bi, len, r.num("factor", 0), r.num("inRate", 2), r.num("outRate", 2), 0});
    }
    if (v.hydros.empty()) v.notes.push_back("no steering hydros: the front wheels will not steer");
    for (const char *sec : {"pressureWheels", "hubWheels", "wheels"})
        for (auto &[pi, r] : m.table(sec)) {
            int n1 = idx(m, r.str("node1")), n2 = idx(m, r.str("node2"));
            if (n1 < 0 || n2 < 0) {
                v.notes.push_back("wheel " + r.str("name", "?") + " skipped: axle nodes missing");
                continue;
            }
            JWheel w;
            w.name = r.str("name", "?");
            w.n1 = n1, w.n2 = n2;
            w.radius = r.has_num("radius") ? r.num("radius") : r.has_num("hubRadius") ? r.num("hubRadius") + 0.12 : 0.33;
            w.width = r.num("tireWidth", 0.2);
            body.nodes[n1].m += WHEEL_EXTRA_MASS / 2;
            body.nodes[n2].m += WHEEL_EXTRA_MASS / 2;
            w.front = ours(m.nodes[n1].pos).x > 0;
            w.arm = idx(m, r.str("nodeArm"));
            w.brake_torque = r.num("brakeTorque", 1500);
            w.parking_torque = r.num("parkingTorque", 0);
            v.wheels.push_back(w);
        }
    if (v.wheels.size() < 3) {
        err = "only " + std::to_string(v.wheels.size()) + " usable wheels found";
        return false;
    }
    auto pt = m.table("powertrain");
    auto input_of = [&](const std::string &name) -> std::string {
        for (auto &[pi, r] : pt)
            if (r.str("name") == name) return r.str("inputName");
        return "";
    };
    for (auto &w : v.wheels) {
        std::string cur;
        for (auto &[pi, r] : pt)
            if (r.str("connectedWheel") == w.name) {
                cur = r.str("name");
                break;
            }
        for (int k = 0; k < 12 && !cur.empty(); k++) {
            if (cur == "mainEngine") {
                w.driven = true;
                break;
            }
            cur = input_of(cur);
        }
    }
    bool any = false;
    for (auto &w : v.wheels) any |= w.driven;
    if (!any) {
        v.notes.push_back("no driven wheels in the powertrain: front wheels driven");
        for (auto &w : v.wheels) w.driven = w.front;
    }
    double ly = -1e9, ry = 1e9;
    for (size_t i = 0; i < v.wheels.size(); i++) {
        if (!v.wheels[i].driven) continue;
        double y = ours(m.nodes[v.wheels[i].n1].pos).y;
        if (y > ly) ly = y, v.driven_l_ = (int)i;
        if (y < ry) ry = y, v.driven_r_ = (int)i;
    }
    auto rn = m.table("refNodes");
    if (rn.empty()) {
        err = "refNodes (ref/back/left/up) missing";
        return false;
    }
    const Row &r0 = rn[0].second;
    v.frame_[0] = idx(m, r0.str("ref")), v.frame_[1] = idx(m, r0.str("back")), v.frame_[2] = idx(m, r0.str("left")), v.frame_[3] = idx(m, r0.str("up"));
    for (int f : v.frame_)
        if (f < 0) {
            err = "refNodes reference missing nodes";
            return false;
        }
    v.dparams = drivetrain_params(m, v.notes);
    v.drive = Drivetrain();
    v.drive.w_e = v.dparams.idle_rpm * 2 * PI / 60;
    for (auto &[pi, r] : m.table("triangles")) {
        int a = idx(m, r.str("id1")), b = idx(m, r.str("id2")), c = idx(m, r.str("id3"));
        if (a >= 0 && b >= 0 && c >= 0) v.triangles.push_back({a, b, c});
    }
    v.groups.triggered.assign(v.groups.names.size(), false);
    v.groups.broken_groups.assign(v.groups.break_groups.size(), false);
    std::vector<int> deg(body.nodes.size(), 0);
    for (auto &b : body.beams) deg[b.a]++, deg[b.b]++;
    v.weak.resize(body.nodes.size());
    for (size_t i = 0; i < deg.size(); i++) v.weak[i] = deg[i] < 3;
    // rest shape: lowest wheel bottom at z = 0
    double bottom = 1e300;
    for (auto &w : v.wheels) bottom = std::min(bottom, (body.nodes[w.n1].p.z + body.nodes[w.n2].p.z) / 2 - w.radius);
    for (auto &n : body.nodes) n.p.z -= bottom, n.rest = n.p;
    // driver's eye
    auto cams = m.table("camerasInternal");
    const Row *cr = nullptr;
    for (const char *want : {"driver", "dash"})
        for (auto &[pi, r] : cams)
            if (!cr && r.str("type") == want) cr = &r;
    for (auto &[pi, r] : cams)
        if (!cr && r.flag("isInside")) cr = &r;
    if (cr) {
        v.eye.rest = ours({cr->num("x"), cr->num("y"), cr->num("z", 1.2)});
        v.eye.rest.z -= bottom;
        v.eye.fov = std::clamp(cr->num("fov", 60), 30.0, 110.0);
        for (int i = 1; i <= 8; i++) {
            int n = idx(m, cr->str("id" + std::to_string(i)));
            if (n >= 0) v.eye.refs.push_back(n);
        }
        v.eye.from_mod = true;
    } else {
        V3 lo(1e300, 1e300, 1e300), hi(-1e300, -1e300, -1e300);
        for (auto &n : body.nodes) lo = V3(std::min(lo.x, n.p.x), std::min(lo.y, n.p.y), std::min(lo.z, n.p.z)), hi = V3(std::max(hi.x, n.p.x), std::max(hi.y, n.p.y), std::max(hi.z, n.p.z));
        v.eye.rest = V3(lo.x + 0.45 * (hi.x - lo.x), (lo.y + hi.y) / 2 + 0.37, std::max(0.8, std::min(hi.z - 0.35, 1.15)));
    }
    if (v.eye.refs.empty()) {
        std::vector<std::pair<double, int>> d;
        for (size_t i = 0; i < body.nodes.size(); i++)
            if (!v.weak[i]) d.push_back({(body.nodes[i].p - v.eye.rest).len(), (int)i});
        std::sort(d.begin(), d.end());
        for (size_t k = 0; k < d.size() && k < 6; k++) v.eye.refs.push_back(d[k].second);
    }
    for (auto &n : m.nodes) v.names.push_back(n.name);
    v.axes(v.rf_, v.rl_, v.ru_);
    v.refresh();
    return true;
}

void JVehicle::axes(V3 &f, V3 &l, V3 &u) const {
    V3 r = body.nodes[frame_[0]].p, b = body.nodes[frame_[1]].p, uu = body.nodes[frame_[3]].p;
    f = (r - b).norm();
    V3 up0 = (uu - r).norm();
    l = up0.cross(f).norm();
    u = f.cross(l).norm();
}

void JVehicle::rotation(V3 &cf, V3 &cl, V3 &cu) const {
    // columns of R = [f l u][rf rl ru]^T
    V3 f, l, u;
    axes(f, l, u);
    cf = f * rf_.x + l * rl_.x + u * ru_.x;
    cl = f * rf_.y + l * rl_.y + u * ru_.y;
    cu = f * rf_.z + l * rl_.z + u * ru_.z;
}

V3 JVehicle::rest_com() const {
    V3 s;
    double m = 0;
    for (auto &n : body.nodes) s = s + n.rest * n.m, m += n.m;
    return s * (1 / m);
}

V3 JVehicle::wheel_axis(int i) const {
    V3 a = (body.nodes[wheels[i].n1].p - body.nodes[wheels[i].n2].p).norm();
    return a.dot(left) < 0 ? a * -1 : a;
}

void JVehicle::eye_pose(V3 &p) const {
    V3 cf, cl, cu;
    rotation(cf, cl, cu);
    V3 now, rest;
    for (int i : eye.refs) now = now + body.nodes[i].p, rest = rest + body.nodes[i].rest;
    double k = 1.0 / std::max<size_t>(1, eye.refs.size());
    V3 d = eye.rest - rest * k;
    p = now * k + cf * d.x + cl * d.y + cu * d.z;
}

void JVehicle::refresh() {
    axes(fwd, left, up);
    V3 c, vv;
    double m = 0;
    for (auto &n : body.nodes) c = c + n.p * n.m, vv = vv + n.v * n.m, m += n.m;
    pos = c * (1 / m);
    vel = vv * (1 / m);
}

double JVehicle::damage(int *broken) const {
    V3 cf, cl, cu;
    rotation(cf, cl, cu);
    V3 rc = rest_com();
    double worst = 0;
    for (size_t i = 0; i < body.nodes.size(); i++) {
        if (weak[i]) continue;
        V3 d = body.nodes[i].rest - rc;
        V3 ideal = pos + cf * d.x + cl * d.y + cu * d.z;
        worst = std::max(worst, (body.nodes[i].p - ideal).len());
    }
    if (broken) {
        int b = 0;
        for (auto &be : body.beams) b += be.broken;
        *broken = b;
    }
    return worst;
}

void JVehicle::place(const CollisionWorld &w, double x, double y, double z_hint, double yaw, double speed) {
    Surface s = w.surface(x, y, z_hint, 2.0);
    double h = s.found ? s.height : z_hint;
    V3 c = rest_com();
    c.z = 0;
    double cs = std::cos(yaw), sn = std::sin(yaw);
    V3 f(cs, sn, 0);
    for (auto &n : body.nodes) {
        V3 d = n.rest - c;
        n.p = V3(x + d.x * cs - d.y * sn, y + d.x * sn + d.y * cs, h + d.z + 0.003);
        n.v = f * speed;
        n.f = V3();
    }
    for (auto &b : body.beams) b.broken = false, b.rest = b.rest0;
    groups.reset();
    for (auto &p : body.pins) p.broken = false;
    for (auto &hd : hydros) hd.state = 0, body.beams[hd.beam].rest = hd.base;
    body.plastic_work = 0;
    body.diverged = false;
    for (auto &wh : wheels) wh.st = WheelState(), wh.st.w = speed / wh.radius;
    drive = Drivetrain();
    drive.w_e = dparams.idle_rpm * 2 * PI / 60;
    impact = 0;
    refresh();
}

void JVehicle::step(double dt, const CarInput &in, const CollisionWorld &w) {
    int n = std::max(1, (int)std::lround(dt / sub_dt));
    double h = dt / n, imp = 0;
    V3 v0 = vel;
    for (int i = 0; i < n; i++) {
        substep(h, in, w);
        imp = std::max(imp, body.peak_world);
    }
    impact = imp;
    groups.update(body);
    refresh();
    V3 a = (vel - v0) * (1 / dt) - V3(0, 0, -9.81);
    V3 spec(a.dot(fwd), a.dot(left), a.dot(up));
    double k = std::min(dt / 0.1, 1.0);
    if (std::isfinite(spec.x)) accel = accel * (1 - k) + spec * k;
}

void JVehicle::substep(double dt, const CarInput &in, const CollisionWorld &w) {
    V3 f, l, u;
    axes(f, l, u);
    // BeamNG steering input is positive to the right; ours is a road-wheel angle, positive left
    double steer = -std::clamp(in.steer / 0.55, -1.0, 1.0);
    steer_cmd = steer;
    for (auto &hd : hydros) {
        bool away = (std::abs(steer) > std::abs(hd.state) && (steer > 0) == (hd.state > 0)) || hd.state == 0;
        double rate = (away ? hd.in_rate : hd.out_rate) * dt;
        hd.state += std::clamp(steer - hd.state, -rate, rate);
        body.beams[hd.beam].rest = hd.base * (1 + hd.factor * hd.state);
    }
    body.clear_forces();
    body.beam_forces(dt);
    body.slide_forces();
    V3 cv;
    double mt = 0;
    for (auto &n : body.nodes) cv = cv + n.v * n.m, mt += n.m;
    cv = cv * (1 / mt);
    double sp = cv.len();
    V3 drag = cv * (-0.5 * 1.2 * cd_a * sp);
    for (auto &n : body.nodes) n.f = n.f + drag * (n.m / mt);
    double thr = in.throttle;
    JWheel &wl = wheels[driven_l_], &wr = wheels[driven_r_];
    if (in.tcs && in.handbrake < 0.1 && (wl.st.contact || wr.st.contact)) {
        double k = std::max(wl.st.kappa, wr.st.kappa);
        thr *= std::clamp(1 - (k - 0.22) / 0.15, 0.0, 1.0);
    }
    double tl, tr, stiff;
    drive.step(dt, dparams, thr, in.reverse, wl.st.w, wr.st.w, tl, tr, stiff);
    int nd = 0;
    for (auto &wh : wheels) nd += wh.driven;
    nd = std::max(nd, 2);
    for (size_t i = 0; i < wheels.size(); i++) {
        JWheel &wh = wheels[i];
        SNode &a1 = body.nodes[wh.n1], &a2 = body.nodes[wh.n2];
        V3 c = (a1.p + a2.p) * 0.5, cvel = (a1.v + a2.v) * 0.5;
        V3 lat = (a1.p - a2.p).norm();
        if (lat.dot(l) < 0) lat = lat * -1;
        double t_drive = 0, st = 0;
        if ((int)i == driven_l_) t_drive = tl, st = stiff;
        else if ((int)i == driven_r_) t_drive = tr, st = stiff;
        else if (wh.driven) t_drive = (tl + tr) / nd;
        double tb = in.brake * wh.brake_torque;
        if (in.abs && in.brake > 0.05 && wh.st.contact && -wh.st.kappa > 0.12) tb *= std::clamp(1 - (-wh.st.kappa - 0.12) / 0.1, 0.05, 1.0);
        tb += in.handbrake * wh.parking_torque * (1 - 0.75 * thr);
        double r = wh.radius;
        Surface s = w.surface(c.x, c.y, c.z - r, 0.12);
        V3 nrm = s.normal;
        double pen = s.found ? r - (c - V3(c.x, c.y, s.height)).dot(nrm) : -1;
        double w0 = wh.st.w, wn, fz = 0;
        TireOut out;
        V3 pc = c - nrm * r;
        if (pen > 0) {
            double vn = cvel.dot(nrm);
            fz = std::max(0.0, TIRE_K * pen - TIRE_C * vn);
            V3 head = lat.cross(nrm).norm(), side = nrm.cross(head);
            double vl = cvel.dot(head), vt = cvel.dot(side);
            double grip = s.grip * (!wh.front ? 1 - 0.45 * in.handbrake : 1.0);
            TireParams tp = tire;
            if (tb > 0 && std::abs(w0) < 0.5 && sp < 1) tp.v0 = 0.4;
            TireOut o1 = tire_force(tp, fz, grip, vl, vt, w0, r), o2 = tire_force(tp, fz, grip, vl, vt, w0 + 1, r);
            double jx = std::max(0.0, o2.fx - o1.fx);
            double den = wh.inertia + dt * (r * jx + st);
            wn = apply_brake(w0 + dt * (t_drive - r * o1.fx) / den, tb, dt, wh.inertia);
            out = tire_force(tp, fz, grip, vl, vt, wn, r);
            double rr = -0.012 * fz * std::tanh(vl / 0.05);
            V3 ft = head * (out.fx + rr) + side * out.fy;
            V3 force = nrm * fz + ft;
            a1.f = a1.f + force * 0.5;
            a2.f = a2.f + force * 0.5;
            if (wh.arm >= 0) {
                V3 mom = (pc - c).cross(force);
                V3 m_lat = lat * mom.dot(lat);
                V3 ra = body.nodes[wh.arm].p - c;
                ra = ra - lat * ra.dot(lat);
                double rr2 = ra.dot(ra);
                if (rr2 > 1e-4) {
                    V3 fa = m_lat.cross(ra) * (1 / rr2);
                    body.nodes[wh.arm].f = body.nodes[wh.arm].f + fa;
                    a1.f = a1.f - fa * 0.5;
                    a2.f = a2.f - fa * 0.5;
                }
            }
        } else
            wn = apply_brake(w0 + dt * t_drive / (wh.inertia + dt * st), tb, dt, wh.inertia);
        if (!std::isfinite(wn)) wn = 0;
        wh.st.w = wn;
        wh.st.spin += wn * dt;
        wh.st.fz = fz;
        wh.st.contact = pen > 0;
        wh.st.kappa = out.kappa, wh.st.tan_alpha = out.tan_alpha, wh.st.fx = out.fx, wh.st.fy = out.fy;
        wh.st.locked = tb > 0 && wn == 0;
        wh.st.pc = pc;
        V3 head = lat.cross(u);
        wh.st.steer = std::atan2(head.dot(l), head.dot(f));
    }
    body.contacts(w, dt);
    body.integrate(dt);
    body.resolve_world(w, dt);
}

}  // namespace ec
