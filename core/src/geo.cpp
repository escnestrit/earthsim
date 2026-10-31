#include "geo.hpp"

namespace es {

double poly_area(const Poly &p) {
    double a = 0;
    for (size_t i = 0, n = p.size(); i < n; i++) a += cross(p[i], p[(i + 1) % n]);
    return a / 2;
}

bool point_in_poly(const Poly &p, V2 q) {
    bool in = false;
    for (size_t i = 0, n = p.size(), j = n - 1; i < n; j = i++) {
        V2 a = p[i], b = p[j];
        if ((a.y > q.y) != (b.y > q.y) && q.x < (b.x - a.x) * (q.y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

V2 poly_centroid(const Poly &p) {
    double a = 0;
    V2 c{0, 0};
    V2 o = p.empty() ? V2{0, 0} : p[0];
    for (size_t i = 0, n = p.size(); i < n; i++) {
        V2 u = p[i] - o, v = p[(i + 1) % n] - o;
        double k = cross(u, v);
        a += k;
        c += (u + v) * k;
    }
    if (std::fabs(a) < 1e-9) {
        V2 s{0, 0};
        for (auto q : p) s += q;
        return p.empty() ? s : s / double(p.size());
    }
    return o + c / (3 * a);
}

double seg_dist(V2 q, V2 a, V2 b, double *t) {
    V2 d = b - a;
    double l2 = dot(d, d);
    double u = l2 > 1e-18 ? clampd(dot(q - a, d) / l2, 0, 1) : 0;
    if (t) *t = u;
    return dist(q, a + d * u);
}

Poly clip_poly(const Poly &p, const Rect &r) {
    Poly cur = p;
    for (int e = 0; e < 4 && !cur.empty(); e++) {
        auto inside = [&](V2 q) {
            switch (e) {
                case 0: return q.x >= r.mn.x;
                case 1: return q.x <= r.mx.x;
                case 2: return q.y >= r.mn.y;
                default: return q.y <= r.mx.y;
            }
        };
        auto cut = [&](V2 a, V2 b) {
            double t;
            switch (e) {
                case 0: t = (r.mn.x - a.x) / (b.x - a.x); break;
                case 1: t = (r.mx.x - a.x) / (b.x - a.x); break;
                case 2: t = (r.mn.y - a.y) / (b.y - a.y); break;
                default: t = (r.mx.y - a.y) / (b.y - a.y); break;
            }
            V2 q = lerp(a, b, t);
            // snap exactly onto the clip line so neighbouring tiles share the seam
            if (e == 0) q.x = r.mn.x; else if (e == 1) q.x = r.mx.x; else if (e == 2) q.y = r.mn.y; else q.y = r.mx.y;
            return q;
        };
        Poly out;
        for (size_t i = 0, n = cur.size(); i < n; i++) {
            V2 a = cur[i], b = cur[(i + 1) % n];
            bool ia = inside(a), ib = inside(b);
            if (ia) out.push_back(a);
            if (ia != ib) out.push_back(cut(a, b));
        }
        cur.swap(out);
    }
    return cur.size() >= 3 ? cur : Poly{};
}

std::vector<std::vector<V2>> clip_line(const std::vector<V2> &l, const Rect &r) {
    std::vector<std::vector<V2>> out;
    std::vector<V2> cur;
    auto flush = [&] {
        if (cur.size() >= 2) out.push_back(cur);
        cur.clear();
    };
    for (size_t i = 0; i + 1 < l.size(); i++) {
        V2 a = l[i], b = l[i + 1];
        // Liang-Barsky
        double t0 = 0, t1 = 1;
        V2 d = b - a;
        double p[4] = {-d.x, d.x, -d.y, d.y};
        double q[4] = {a.x - r.mn.x, r.mx.x - a.x, a.y - r.mn.y, r.mx.y - a.y};
        bool vis = true;
        for (int k = 0; k < 4 && vis; k++) {
            if (std::fabs(p[k]) < 1e-15) {
                if (q[k] < 0) vis = false;
            } else {
                double t = q[k] / p[k];
                if (p[k] < 0) t0 = std::max(t0, t);
                else t1 = std::min(t1, t);
            }
        }
        if (!vis || t0 > t1) {
            flush();
            continue;
        }
        V2 A = t0 > 0 ? a + d * t0 : a, B = t1 < 1 ? a + d * t1 : b;
        if (t0 > 0) {
            flush();
            // snap onto the boundary that was hit
            if (std::fabs(A.x - r.mn.x) < 1e-6) A.x = r.mn.x;
            if (std::fabs(A.x - r.mx.x) < 1e-6) A.x = r.mx.x;
            if (std::fabs(A.y - r.mn.y) < 1e-6) A.y = r.mn.y;
            if (std::fabs(A.y - r.mx.y) < 1e-6) A.y = r.mx.y;
        }
        if (cur.empty()) cur.push_back(A);
        cur.push_back(B);
        if (t1 < 1) flush();
    }
    flush();
    return out;
}

bool line_intersect(V2 p, V2 r, V2 q, V2 s, double &t, double &u) {
    double d = cross(r, s);
    if (std::fabs(d) < 1e-12) return false;
    V2 qp = q - p;
    t = cross(qp, s) / d;
    u = cross(qp, r) / d;
    return true;
}

bool seg_intersect(V2 a, V2 b, V2 c, V2 d, double &t, double &u) {
    if (!line_intersect(a, b - a, c, d - c, t, u)) return false;
    return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

double polyline_length(const std::vector<V2> &l) {
    double s = 0;
    for (size_t i = 0; i + 1 < l.size(); i++) s += dist(l[i], l[i + 1]);
    return s;
}

V2 polyline_at(const std::vector<V2> &l, double s, V2 *dir) {
    if (l.size() < 2) {
        if (dir) *dir = {1, 0};
        return l.empty() ? V2{0, 0} : l[0];
    }
    for (size_t i = 0; i + 1 < l.size(); i++) {
        double d = dist(l[i], l[i + 1]);
        if (s <= d || i + 2 == l.size()) {
            if (dir) *dir = norm(l[i + 1] - l[i]);
            return d > 1e-12 ? lerp(l[i], l[i + 1], clampd(s / d, 0, 1)) : l[i];
        }
        s -= d;
    }
    return l.back();
}

uint64_t hash_str(const char *s, size_t n) {
    uint64_t h = 1469598103934665603ULL;
    for (size_t i = 0; i < n; i++) h = (h ^ uint8_t(s[i])) * 1099511628211ULL;
    return h;
}

}  // namespace es
