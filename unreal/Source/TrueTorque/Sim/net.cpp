#include "net.hpp"
#include <cstring>

namespace tt {

namespace {
template <class T> void put(std::vector<uint8_t> &b, T v) {
    uint8_t t[sizeof(T)];
    memcpy(t, &v, sizeof(T));
    b.insert(b.end(), t, t + sizeof(T));
}
template <class T> bool get(const std::vector<uint8_t> &b, size_t &o, T &v) {
    if (o + sizeof(T) > b.size()) return false;
    memcpy(&v, b.data() + o, sizeof(T));
    o += sizeof(T);
    return true;
}
const double Q = 0.002;  // 2 mm
}  // namespace

Snapshot encode(const Vehicle &v, uint32_t tick) {
    Snapshot s;
    auto &b = s.bytes;
    V3 o, r, f, u;
    v.frame(o, r, f, u);
    Quat q = quat_from_basis(r, f, u);
    put<uint32_t>(b, tick);
    put<float>(b, float(o.x)), put<float>(b, float(o.y)), put<float>(b, float(o.z));
    for (double c : {q.w, q.x, q.y, q.z}) put<int16_t>(b, int16_t(std::lround(c * 32767)));
    V3 vel = v.body.nodes[v.ref_c].v;
    put<int16_t>(b, int16_t(clampd(vel.x * 100, -32767, 32767))), put<int16_t>(b, int16_t(clampd(vel.y * 100, -32767, 32767))), put<int16_t>(b, int16_t(clampd(vel.z * 100, -32767, 32767)));
    put<uint16_t>(b, uint16_t(v.body.nodes.size()));
    Quat qi = q.conj();
    for (auto &n : v.body.nodes) {
        V3 l = qi.rotate(n.p - o);
        for (double c : {l.x, l.y, l.z}) put<int16_t>(b, int16_t(clampd(std::lround(c / Q), -32767, 32767)));
    }
    put<uint8_t>(b, uint8_t(v.wheels.size()));
    for (auto &w : v.wheels) {
        put<int16_t>(b, int16_t(clampd(w.omega * 100, -32767, 32767)));
        put<int16_t>(b, int16_t(clampd(w.steer * 10000, -32767, 32767)));
        put<uint8_t>(b, uint8_t((w.detached ? 1 : 0) | (w.flat ? 2 : 0) | (w.contact ? 4 : 0)));
    }
    put<uint16_t>(b, uint16_t(clampd(v.engine.rpm, 0, 65535)));
    put<int8_t>(b, int8_t(v.gearbox.gear));
    put<uint8_t>(b, uint8_t(clampd(v.in.throttle * 255, 0, 255)));
    put<uint8_t>(b, uint8_t(clampd(v.in.brake * 255, 0, 255)));
    // broken beams as a bit field (deformation itself is carried by the node positions)
    std::vector<uint8_t> bits((v.body.beams.size() + 7) / 8, 0);
    for (size_t i = 0; i < v.body.beams.size(); i++)
        if (v.body.beams[i].broken) bits[i / 8] |= uint8_t(1 << (i % 8));
    put<uint16_t>(b, uint16_t(bits.size()));
    b.insert(b.end(), bits.begin(), bits.end());
    return s;
}

uint32_t decode(const Snapshot &s, Vehicle &v) {
    auto &b = s.bytes;
    size_t o = 0;
    uint32_t tick;
    float px, py, pz;
    int16_t qw, qx, qy, qz, vx, vy, vz;
    uint16_t nn;
    if (!get(b, o, tick) || !get(b, o, px) || !get(b, o, py) || !get(b, o, pz) || !get(b, o, qw) || !get(b, o, qx) || !get(b, o, qy) || !get(b, o, qz) ||
        !get(b, o, vx) || !get(b, o, vy) || !get(b, o, vz) || !get(b, o, nn))
        return 0;
    if (nn != v.body.nodes.size()) return 0;
    Quat q = Quat{qw / 32767.0, qx / 32767.0, qy / 32767.0, qz / 32767.0}.normalized();
    V3 org{px, py, pz}, vel{vx / 100.0, vy / 100.0, vz / 100.0};
    for (auto &n : v.body.nodes) {
        int16_t a, c, d;
        if (!get(b, o, a) || !get(b, o, c) || !get(b, o, d)) return 0;
        n.p = org + q.rotate(V3{a * Q, c * Q, d * Q});
        n.v = vel;
    }
    uint8_t nw;
    if (!get(b, o, nw) || nw != v.wheels.size()) return 0;
    for (auto &w : v.wheels) {
        int16_t om, st;
        uint8_t fl;
        if (!get(b, o, om) || !get(b, o, st) || !get(b, o, fl)) return 0;
        w.omega = om / 100.0, w.steer = st / 10000.0, w.detached = fl & 1, w.flat = fl & 2, w.contact = fl & 4;
    }
    uint16_t rpm;
    int8_t gear;
    uint8_t th, br;
    uint16_t nb;
    if (!get(b, o, rpm) || !get(b, o, gear) || !get(b, o, th) || !get(b, o, br) || !get(b, o, nb)) return 0;
    v.engine.rpm = rpm, v.gearbox.gear = gear, v.in.throttle = th / 255.0, v.in.brake = br / 255.0;
    for (size_t i = 0; i < v.body.beams.size() && i / 8 < nb; i++) v.body.beams[i].broken = (b[o + i / 8] >> (i % 8)) & 1;
    return tick;
}

}  // namespace tt
