#include "geom.hpp"
#include <algorithm>
#include "../../third_party/earcut.hpp"

namespace mapbox {
namespace util {
template <>
struct nth<0, ec::V2> {
    inline static double get(const ec::V2 &t) { return t.x; }
};
template <>
struct nth<1, ec::V2> {
    inline static double get(const ec::V2 &t) { return t.y; }
};
}  // namespace util
}  // namespace mapbox

namespace ec {

double signed_area(const Ring &r) {
    double a = 0;
    for (size_t i = 0, n = r.size(); i < n; i++) a += r[i].cross(r[(i + 1) % n]);
    return a / 2;
}
void make_ccw(Ring &r) {
    if (signed_area(r) < 0) std::reverse(r.begin(), r.end());
}
void make_cw(Ring &r) {
    if (signed_area(r) > 0) std::reverse(r.begin(), r.end());
}

Ring simplify(const Ring &in, double min_d, double deg) {
    Ring r = in;
    double s = std::sin(deg * PI / 180);
    bool changed = true;
    while (changed && r.size() > 3) {
        changed = false;
        for (size_t i = 0; i < r.size() && r.size() > 3; i++) {
            size_t n = r.size();
            V2 a = r[(i + n - 1) % n], b = r[i], c = r[(i + 1) % n];
            V2 u = b - a, v = c - b;
            double lu = u.len(), lv = v.len();
            bool straight = lu > 1e-9 && lv > 1e-9 && std::abs(u.cross(v)) / (lu * lv) < s && u.dot(v) > 0;
            if (lu < min_d || straight) {
                r.erase(r.begin() + i);
                changed = true;
                break;
            }
        }
    }
    return r;
}

std::vector<int> triangulate(const std::vector<Ring> &rings) {
    std::vector<std::vector<V2>> poly(rings.begin(), rings.end());
    auto idx = mapbox::earcut<uint32_t>(poly);
    return std::vector<int>(idx.begin(), idx.end());
}

bool point_in_ring(V2 p, const Ring &r) {
    bool in = false;
    size_t n = r.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++)
        if ((r[i].y > p.y) != (r[j].y > p.y) && p.x < r[i].x + (p.y - r[i].y) / (r[j].y - r[i].y) * (r[j].x - r[i].x)) in = !in;
    return in;
}
bool point_in_poly(V2 p, const std::vector<Ring> &rings) {
    if (rings.empty() || !point_in_ring(p, rings[0])) return false;
    for (size_t k = 1; k < rings.size(); k++)
        if (point_in_ring(p, rings[k])) return false;
    return true;
}
double seg_dist(V2 p, V2 a, V2 b) {
    V2 d = b - a;
    double l2 = d.dot(d);
    double t = l2 > 0 ? std::clamp((p - a).dot(d) / l2, 0.0, 1.0) : 0;
    return (a + d * t - p).len();
}
double ring_dist(V2 p, const Ring &r) {
    double d = 1e300;
    for (size_t i = 0, n = r.size(); i < n; i++) d = std::min(d, seg_dist(p, r[i], r[(i + 1) % n]));
    return d;
}
Ring offset_ring(const Ring &r, double d) {
    size_t n = r.size();
    Ring o(n);
    for (size_t i = 0; i < n; i++) {
        V2 p = r[(i + n - 1) % n], c = r[i], q = r[(i + 1) % n];
        V2 n1 = V2((c - p).y, -(c - p).x).norm(), n2 = V2((q - c).y, -(q - c).x).norm();
        V2 m = (n1 + n2).norm();
        if (m.len() < 1e-9) {
            o[i] = c;
            continue;
        }
        double k = d / std::max(0.3, m.dot(n1));
        o[i] = c + m * k;
    }
    return o;
}
double perimeter(const Ring &r) {
    double p = 0;
    for (size_t i = 0, n = r.size(); i < n; i++) p += (r[(i + 1) % n] - r[i]).len();
    return p;
}
double corpus_depth(const Ring &r) {
    if (r.size() < 3) return 0;
    double a = std::abs(signed_area(r)), q = perimeter(r) / 4;
    return q - std::sqrt(std::max(0.0, q * q - a));
}
V2 centroid(const Ring &r) {
    double a = 0, cx = 0, cy = 0;
    for (size_t i = 0, n = r.size(); i < n; i++) {
        double f = r[i].cross(r[(i + 1) % n]);
        a += f;
        cx += (r[i].x + r[(i + 1) % n].x) * f;
        cy += (r[i].y + r[(i + 1) % n].y) * f;
    }
    if (std::abs(a) < 1e-9) {
        V2 s;
        for (auto &p : r) s = s + p;
        return s * (1.0 / std::max<size_t>(1, r.size()));
    }
    return {cx / (3 * a), cy / (3 * a)};
}
std::vector<double> chainage(const std::vector<V2> &p) {
    std::vector<double> s(p.size(), 0);
    for (size_t i = 1; i < p.size(); i++) s[i] = s[i - 1] + (p[i] - p[i - 1]).len();
    return s;
}
void normals(const std::vector<V2> &p, std::vector<V2> &n, std::vector<double> &scale) {
    size_t k = p.size();
    n.assign(k, V2(0, 1));
    scale.assign(k, 1);
    if (k < 2) return;
    std::vector<V2> seg(k - 1);
    for (size_t i = 0; i + 1 < k; i++) seg[i] = (p[i + 1] - p[i]).norm().perp();
    for (size_t i = 0; i < k; i++) {
        V2 a = seg[std::min(i == 0 ? 0 : i - 1, k - 2)], b = seg[std::min(i, k - 2)];
        V2 m = a + b;
        if (m.len() < 1e-9) m = a;
        m = m.norm();
        n[i] = m;
        scale[i] = 1.0 / std::max(0.45, m.dot(a));
    }
}
std::vector<V2> densify(const std::vector<V2> &p, double step) {
    std::vector<V2> o;
    for (size_t i = 0; i < p.size(); i++) {
        if (i > 0) {
            V2 q = p[i - 1];
            int k = (int)std::floor((p[i] - q).len() / step);
            for (int s = 1; s <= k; s++) o.push_back(q + (p[i] - q) * (double(s) / (k + 1)));
        }
        o.push_back(p[i]);
    }
    return o;
}
void split_tri(V3 a, V3 b, V3 c, double max, int depth, std::vector<std::array<V3, 3>> &out) {
    auto l = [](V3 p, V3 q) { return std::hypot(p.x - q.x, p.y - q.y); };
    double e0 = l(a, b), e1 = l(b, c), e2 = l(c, a);
    double m = std::max({e0, e1, e2});
    if (m <= max || depth <= 0) {
        out.push_back({a, b, c});
        return;
    }
    if (m == e0) {
        V3 h = (a + b) * 0.5;
        split_tri(a, h, c, max, depth - 1, out);
        split_tri(h, b, c, max, depth - 1, out);
    } else if (m == e1) {
        V3 h = (b + c) * 0.5;
        split_tri(a, b, h, max, depth - 1, out);
        split_tri(a, h, c, max, depth - 1, out);
    } else {
        V3 h = (c + a) * 0.5;
        split_tri(a, b, h, max, depth - 1, out);
        split_tri(h, b, c, max, depth - 1, out);
    }
}
void footprint_frame(const Ring &r, V2 &c, V2 &axis, double &len, double &wid) {
    double best = 0;
    axis = V2(1, 0);
    for (size_t i = 0, n = r.size(); i < n; i++) {
        V2 d = r[(i + 1) % n] - r[i];
        if (d.len() > best) {
            best = d.len();
            axis = d.norm();
        }
    }
    V2 c0 = centroid(r);
    double s0 = 1e300, s1 = -1e300, t0 = 1e300, t1 = -1e300;
    for (auto &p : r) {
        V2 q = p - c0;
        double s = q.dot(axis), t = axis.perp().dot(q);
        s0 = std::min(s0, s), s1 = std::max(s1, s), t0 = std::min(t0, t), t1 = std::max(t1, t);
    }
    len = s1 - s0;
    wid = t1 - t0;
    c = c0 + axis * ((s0 + s1) / 2) + axis.perp() * ((t0 + t1) / 2);
}

}  // namespace ec
