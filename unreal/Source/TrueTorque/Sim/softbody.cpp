#include "softbody.hpp"

namespace tt {

int SoftBody::add_node(V3 p, double mass, int part) {
    Node n;
    n.p = p;
    n.m = mass;
    n.inv_m = mass > 0 ? 1.0 / mass : 0;
    n.part = part;
    nodes.push_back(n);
    return int(nodes.size() - 1);
}

int SoftBody::add_beam(int a, int b, double k, double c, double yield, double brk, int part, BeamKind kind) {
    Beam bm;
    bm.a = a, bm.b = b;
    bm.L0 = bm.L0_build = len(nodes[b].p - nodes[a].p);
    bm.k = k, bm.c = c, bm.yield = yield, bm.brk = brk, bm.part = part, bm.kind = kind;
    beams.push_back(bm);
    int i = int(beams.size() - 1);
    if (part >= 0 && part < int(parts.size())) (kind == BEAM_ATTACH ? parts[part].attach : parts[part].beams).push_back(i);
    return i;
}

void SoftBody::finalize() {
    for (auto &n : nodes) n.rest = n.p;
    contacts.assign(nodes.size(), {});
    for (size_t i = 0; i < nodes.size(); i++)
        if (nodes[i].part >= 0) parts[nodes[i].part].nodes.push_back(int(i));
}

void SoftBody::beam_forces(double dt) {
    for (auto &b : beams) {
        if (b.broken) continue;
        Node &A = nodes[b.a], &B = nodes[b.b];
        V3 d = B.p - A.p;
        double L = len(d);
        if (L < 1e-6) continue;
        V3 u = d / L;
        double strain = (L - b.L0) / b.L0_build;
        b.strain = float(strain);
        if (b.kind == BEAM_SUPPORT && strain > 0) continue;  // only pushes
        double as = std::fabs(strain);
        // plastic flow: the rest length follows the stretch beyond the yield strain
        if (as > b.yield) {
            double excess = (as - b.yield) * (strain > 0 ? 1 : -1);
            double dl = excess * b.L0_build;
            stats.deformation_energy += std::fabs(b.k * b.yield * b.L0_build * dl);
            b.L0 += dl;
            b.plastic += std::fabs(excess);
            strain = (L - b.L0) / b.L0_build;
        }
        // fatigue: cycles above ~half the yield strain slowly use up the beam's strength
        if (as > b.yield * 0.55) b.fatigue = std::min(1.0, b.fatigue + (as - b.yield * 0.55) / b.yield * dt * 0.02);
        double total = std::fabs(b.L0 - b.L0_build) / b.L0_build + std::fabs(strain);
        if (total > b.brk * (1.0 - 0.6 * b.fatigue)) {
            b.broken = true;
            stats.broken_beams++;
            continue;
        }
        double vrel = dot(B.v - A.v, u);
        double f = b.k * (L - b.L0) + b.c * vrel;
        V3 F = u * f;
        A.f += F;
        B.f -= F;
    }
}

void SoftBody::contact_forces(double dt, const GroundFn &ground) {
    double peak = 0;
    for (size_t i = 0; i < nodes.size(); i++) {
        Node &n = nodes[i];
        if (!n.collide) continue;
        auto apply = [&](const V3 &q, const V3 &nrm, double mu) {
            double pen = n.radius - dot(n.p - q, nrm);
            if (pen <= 0) return;
            double vn = dot(n.v, nrm);
            double fn = contact_k * pen * std::sqrt(n.m / 10.0) - contact_c * std::sqrt(n.m / 10.0) * vn;
            if (fn <= 0) return;
            V3 vt = n.v - nrm * vn;
            double vtl = len(vt);
            V3 ft{0, 0, 0};
            if (vtl > 1e-5) {
                // regularised Coulomb friction (no sticking jitter at rest)
                double fmax = mu * n.friction * fn;
                double fv = std::min(fmax, vtl * n.m / dt * 0.5);
                ft = vt * (-fv / vtl);
            }
            n.f += nrm * fn + ft;
            peak = std::max(peak, fn * dt);
        };
        if (ground) {
            GroundHit g = ground(n.p.x, n.p.y);
            if (g.h > -1e8) apply(V3{n.p.x, n.p.y, g.h}, g.n, g.friction);
        }
        for (auto &c : contacts[i]) apply(c.p, c.n, c.friction);
    }
    stats.impact_peak = std::max(stats.impact_peak, peak);
}

void SoftBody::step(double dt, const GroundFn &ground) {
    beam_forces(dt);
    contact_forces(dt, ground);
    for (auto &n : nodes) {
        if (n.inv_m == 0) {
            n.f = {0, 0, 0};
            continue;
        }
        n.v += (n.f * n.inv_m + gravity) * dt;
        n.p += n.v * dt;
        n.f = {0, 0, 0};
    }
}

void SoftBody::collide(SoftBody &A, SoftBody &B, double dt) {
    V3 amn, amx, bmn, bmx;
    A.bounds(amn, amx);
    B.bounds(bmn, bmx);
    if (amx.x < bmn.x || bmx.x < amn.x || amx.y < bmn.y || bmx.y < amn.y || amx.z < bmn.z || bmx.z < amn.z) return;
    for (auto &a : A.nodes) {
        if (!a.collide || a.p.x < bmn.x - 0.3 || a.p.x > bmx.x + 0.3 || a.p.y < bmn.y - 0.3 || a.p.y > bmx.y + 0.3) continue;
        for (auto &b : B.nodes) {
            if (!b.collide) continue;
            V3 d = b.p - a.p;
            double r = a.radius + b.radius + 0.06;
            double l2 = len2(d);
            if (l2 >= r * r || l2 < 1e-10) continue;
            double l = std::sqrt(l2);
            V3 u = d / l;
            double pen = r - l;
            double mu = std::sqrt(a.m * b.m / (a.m + b.m) / 10.0);
            double vn = dot(b.v - a.v, u);
            double f = 4e5 * pen * mu - 3e3 * mu * vn;
            if (f <= 0) continue;
            a.f -= u * f;
            b.f += u * f;
            A.stats.impact_peak = std::max(A.stats.impact_peak, f * dt);
            B.stats.impact_peak = std::max(B.stats.impact_peak, f * dt);
        }
    }
}

V3 SoftBody::centre_of_mass() const {
    V3 s{0, 0, 0};
    double m = 0;
    for (auto &n : nodes) s += n.p * n.m, m += n.m;
    return m > 0 ? s / m : s;
}

V3 SoftBody::velocity() const {
    V3 s{0, 0, 0};
    double m = 0;
    for (auto &n : nodes) s += n.v * n.m, m += n.m;
    return m > 0 ? s / m : s;
}

double SoftBody::mass() const {
    double m = 0;
    for (auto &n : nodes) m += n.m;
    return m;
}

void SoftBody::translate(const V3 &d) {
    for (auto &n : nodes) n.p += d;
}

void SoftBody::bounds(V3 &mn, V3 &mx) const {
    mn = {1e18, 1e18, 1e18}, mx = {-1e18, -1e18, -1e18};
    for (auto &n : nodes) {
        mn.x = std::min(mn.x, n.p.x), mn.y = std::min(mn.y, n.p.y), mn.z = std::min(mn.z, n.p.z);
        mx.x = std::max(mx.x, n.p.x), mx.y = std::max(mx.y, n.p.y), mx.z = std::max(mx.z, n.p.z);
    }
}

void SoftBody::update_parts() {
    for (auto &p : parts) {
        double pl = 0;
        int nb = 0, broken = 0;
        for (int i : p.beams) pl += beams[i].plastic, nb++, broken += beams[i].broken;
        p.health = nb ? clampd(1.0 - pl / nb * 8.0 - double(broken) / nb * 2.0, 0.0, 1.0) : 1.0;
        if (!p.detached && !p.attach.empty()) {
            int left = 0;
            for (int i : p.attach) left += !beams[i].broken;
            if (left == 0) p.detached = true;
        }
    }
}

void SoftBody::repair_all() {
    for (auto &b : beams) b.L0 = b.L0_build, b.broken = false, b.plastic = 0, b.fatigue = 0, b.strain = 0;
    for (auto &p : parts) p.health = 1, p.detached = false;
    stats = {};
}

}  // namespace tt
