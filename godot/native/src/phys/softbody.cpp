#include "softbody.hpp"
#include <algorithm>

namespace ec {

int SoftBody::add_node(V3 p, double m) {
    SNode n;
    n.p = p, n.rest = p, n.m = m;
    nodes.push_back(n);
    return (int)nodes.size() - 1;
}

int SoftBody::add_beam(int a, int b, double k, double c) {
    double len = (nodes[b].p - nodes[a].p).len();
    SBeam be;
    be.a = a, be.b = b, be.rest = len, be.rest0 = len, be.k = k, be.c = c, be.c_reb = c;
    beams.push_back(be);
    return (int)beams.size() - 1;
}

double SoftBody::total_mass() const {
    double m = 0;
    for (auto &n : nodes) m += n.m;
    return m;
}
V3 SoftBody::centre_of_mass() const {
    V3 c;
    for (auto &n : nodes) c = c + n.p * n.m;
    return c * (1 / total_mass());
}

void SoftBody::clear_forces() {
    for (auto &n : nodes) n.f = gravity * n.m;
}

WorkPool::WorkPool(int workers) {
    for (int i = 0; i < workers; i++) th_.emplace_back([this, i] { loop(i + 1); });
}

WorkPool::~WorkPool() {
    stop_ = true;
    {
        std::lock_guard<std::mutex> l(m_);
        gen_++;
    }
    cv_.notify_all();
    for (auto &t : th_) t.join();
}

void WorkPool::loop(int part) {
    uint64_t seen = 0;
    while (true) {
        // spin a little (the next physics step comes within microseconds), then sleep
        int spins = 0;
        while (gen_.load(std::memory_order_acquire) == seen && !stop_) {
            if (++spins < 4000) continue;
            if (spins < 4200) {
                std::this_thread::yield();
                continue;
            }
            std::unique_lock<std::mutex> l(m_);
            cv_.wait_for(l, std::chrono::milliseconds(2), [&] { return gen_.load() != seen || stop_; });
        }
        if (stop_) return;
        seen = gen_.load(std::memory_order_acquire);
        (*fn_)(part);
        left_.fetch_sub(1, std::memory_order_acq_rel);
    }
}

void WorkPool::run(const std::function<void(int)> &fn) {
    fn_ = &fn;
    left_.store((int)th_.size(), std::memory_order_release);
    {
        std::lock_guard<std::mutex> l(m_);
        gen_.fetch_add(1, std::memory_order_acq_rel);
    }
    cv_.notify_all();
    fn(0);
    while (left_.load(std::memory_order_acquire) > 0) {}
}

void SoftBody::set_threads(int n) {
    pool_.reset();
    if (n > 1) pool_ = std::make_unique<WorkPool>(n - 1);
}

void SoftBody::beam_range(size_t b0, size_t b1, double dt, V3 *F, double &plastic) {
    const Hot *H = hot_.data();
    for (size_t bi = b0; bi < b1; bi++) {
        SBeam &be = beams[bi];
        if (be.broken) continue;
        const Hot &na = H[be.a], &nb = H[be.b];
        V3 d = nb.p - na.p;
        double len2 = d.dot(d);
        if (len2 < 1e-18) continue;
        double len = std::sqrt(len2);
        if (be.support && len >= be.rest) {
            if ((len - be.rest) / be.rest > be.break_strain) be.broken = true;
            continue;
        }
        V3 dir = d * (1 / len);
        double vrel = (nb.v - na.v).dot(dir);
        double f_el = be.k * (len - be.rest);
        if (std::abs(f_el) > be.break_force) {
            be.broken = true;
            continue;
        }
        bool yielding = false;
        if (f_el > be.yield_t) {
            double nr = len - be.yield_t / be.k;
            plastic += be.yield_t * std::abs(nr - be.rest);
            be.rest = nr;
            f_el = be.yield_t;
            yielding = true;
        } else if (f_el < -be.yield_c) {
            double nr = std::max(len + be.yield_c / be.k, 0.2 * be.rest0);
            plastic += be.yield_c * std::abs(nr - be.rest);
            be.rest = nr;
            f_el = -be.yield_c;
            yielding = true;
        }
        if ((len - be.rest) > be.break_strain * be.rest) {
            be.broken = true;
            continue;
        }
        double fsum = f_el;
        if (!yielding) {
            double red = na.m * nb.m / (na.m + nb.m);
            double lim = red * std::abs(vrel) / dt;
            double c = vrel > 0 ? be.c_reb : be.c;
            fsum += std::clamp(c * vrel, -lim, lim);
        }
        if (be.has_stops) {
            const Stops &st = be.stops;
            if (len < st.min_len) fsum -= st.k * (st.min_len - len) + st.c * std::max(-vrel, 0.0);
            else if (len > st.max_len) fsum += st.k * (len - st.max_len) + st.c * std::max(vrel, 0.0);
        }
        V3 fv = dir * fsum;
        F[be.a] = F[be.a] + fv;
        F[be.b] = F[be.b] - fv;
    }
}

void SoftBody::beam_forces(double dt) {
    hot_.resize(nodes.size());
    for (size_t i = 0; i < nodes.size(); i++) hot_[i] = {nodes[i].p, nodes[i].v, nodes[i].m};
    double plastic = 0;
    if (!pool_ || beams.size() < 1500) {
        f_.assign(nodes.size(), V3());
        beam_range(0, beams.size(), dt, f_.data(), plastic);
        for (size_t i = 0; i < nodes.size(); i++) nodes[i].f = nodes[i].f + f_[i];
    } else {
        int P = pool_->parts();
        pf_.resize(P);
        pplastic_.assign(P, 0.0);
        for (auto &v : pf_) v.assign(nodes.size(), V3());
        size_t nb = beams.size();
        std::function<void(int)> job = [&](int k) { beam_range(nb * k / P, nb * (k + 1) / P, dt, pf_[k].data(), pplastic_[k]); };
        pool_->run(job);
        for (int k = 0; k < P; k++) {
            plastic += pplastic_[k];
            const V3 *F = pf_[k].data();
            for (size_t i = 0; i < nodes.size(); i++) nodes[i].f = nodes[i].f + F[i];
        }
    }
    plastic_work += plastic;
}

void SoftBody::slide_forces() {
    for (auto &pn : pins) {
        if (pn.broken) continue;
        SNode &na = nodes[pn.a], &nb = nodes[pn.b];
        V3 f = (nb.p - na.p) * pn.k + (nb.v - na.v) * pn.c;
        if (f.len() > pn.strength) {
            pn.broken = true;
            continue;
        }
        na.f = na.f + f;
        nb.f = nb.f - f;
    }
    for (auto &sl : slides) {
        V3 p = nodes[sl.node].p;
        double best = 1e300;
        int bw = -1;
        double bt = 0;
        V3 bq;
        for (size_t w = 0; w + 1 < sl.rail.size(); w++) {
            V3 a = nodes[sl.rail[w]].p, b = nodes[sl.rail[w + 1]].p, d = b - a;
            double dd = d.dot(d);
            if (dd < 1e-12) continue;
            double t = (p - a).dot(d) / dd;
            bool last = w + 2 == sl.rail.size();
            if ((sl.capped || w > 0) && t < 0) t = 0;
            if ((sl.capped || !last) && t > 1) t = 1;
            V3 q = a + d * t;
            double dist = (p - q).dot(p - q);
            if (dist < best) best = dist, bw = (int)w, bt = t, bq = q;
        }
        if (bw < 0) continue;
        int ia = sl.rail[bw], ib = sl.rail[bw + 1];
        V3 e = p - bq;
        V3 vq = nodes[ia].v * (1 - bt) + nodes[ib].v * bt;
        V3 vrel = nodes[sl.node].v - vq;
        double el = e.len();
        V3 vn = el > 1e-9 ? e * (vrel.dot(e) / (el * el)) : V3();
        V3 f = e * -sl.k - vn * sl.c;
        nodes[sl.node].f = nodes[sl.node].f + f;
        nodes[ia].f = nodes[ia].f - f * (1 - bt);
        nodes[ib].f = nodes[ib].f - f * bt;
    }
}

static V3 friction(V3 vt, double fn, double mu, double m, double dt) {
    double s = vt.len();
    if (s < 1e-9) return V3();
    double max_stop = m * s / dt;
    double mag = std::min(mu * fn, max_stop) * std::max(std::min(s / (s + 0.02), 1.0), std::min(s, 0.2) / 0.2);
    return vt * (-mag / s);
}

void SoftBody::contacts(const CollisionWorld &w, double dt) {
    double peak = 0;
    gc_.resize(nodes.size());
    for (size_t ni = 0; ni < nodes.size(); ni++) {
        SNode &n = nodes[ni];
        if (!n.hit_ground) continue;
        GroundCache &g = gc_[ni];
        double dx = n.p.x - g.x, dy = n.p.y - g.y;
        // far above the cached ground: nothing to do
        if (dx * dx + dy * dy < 0.0025 && g.age < 8 && std::abs(n.p.z - g.zq) < 0.05) {
            g.age++;
        } else {
            g.s = w.surface(n.p.x, n.p.y, n.p.z - n.radius, 0.35);
            g.x = n.p.x, g.y = n.p.y, g.zq = n.p.z, g.age = 0;
        }
        Surface s = g.s;
        if (s.found) {
            // the cached plane, evaluated where the node is now
            s.height += (-(n.p.x - g.x) * s.normal.x - (n.p.y - g.y) * s.normal.y) / std::max(0.2, s.normal.z);
        }
        if (!s.found) continue;
        double depth = n.radius - (n.p - V3(n.p.x, n.p.y, s.height)).dot(s.normal);
        if (depth <= 0) continue;
        double vn = n.v.dot(s.normal);
        double fnm = std::max(0.0, ground.k * depth - ground.c * vn);
        V3 vt = n.v - s.normal * vn;
        n.f = n.f + s.normal * fnm + friction(vt, fnm, ground.mu * s.grip, n.m, dt);
        peak = std::max(peak, fnm);
    }
    peak_contact = peak;
    peak_world = 0;
}

void SoftBody::integrate(double dt) {
    for (auto &n : nodes) {
        n.v = n.v + n.f * (dt / n.m);
        n.p = n.p + n.v * dt;
        if (!(std::isfinite(n.p.x) && std::isfinite(n.p.y) && std::isfinite(n.p.z) && std::isfinite(n.v.x))) diverged = true;
    }
}

void SoftBody::resolve_world(const CollisionWorld &w, double dt) {
    V3 lo(1e300, 1e300, 1e300), hi(-1e300, -1e300, -1e300);
    double rmax = 0;
    for (auto &n : nodes)
        if (n.hit_world) {
            lo = V3(std::min(lo.x, n.p.x), std::min(lo.y, n.p.y), std::min(lo.z, n.p.z));
            hi = V3(std::max(hi.x, n.p.x), std::max(hi.y, n.p.y), std::max(hi.z, n.p.z));
            rmax = std::max(rmax, n.radius);
        }
    if (lo.x > hi.x) return;
    V3 m(rmax + 0.1, rmax + 0.1, rmax + 0.1);
    if (!w.any_near(lo - m, hi + m)) return;
    double mu = world.mu, peak = 0;
    for (auto &n : nodes) {
        if (!n.hit_world) continue;
        for (int it = 0; it < 2; it++) {
            w.sphere(n.p, n.radius, hits_, false);
            bool any = false;
            for (auto &h : hits_) {
                // upward faces are the ground's job (contacts()); walls, kerb faces and edges are handled here
                if (h.n.z > 0.7) continue;
                any = true;
                n.p = n.p + h.n * h.depth;
                double vn = n.v.dot(h.n);
                if (vn < 0) {
                    n.v = n.v - h.n * vn;
                    peak = std::max(peak, n.m * -vn / dt);
                    V3 vt = n.v - h.n * n.v.dot(h.n);
                    double s = vt.len();
                    if (s > 1e-9) n.v = n.v - vt * (std::min(mu * -vn, s) / s);
                }
            }
            if (!any) break;
        }
    }
    peak_world = std::max(peak_world, peak);
}

void SoftBody::step(double dt, const CollisionWorld &w) {
    clear_forces();
    beam_forces(dt);
    slide_forces();
    contacts(w, dt);
    integrate(dt);
    resolve_world(w, dt);
}

}  // namespace ec
