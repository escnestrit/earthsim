#include "ttgeom.hpp"

namespace ttg {

constexpr double TTG_PI = 3.14159265358979323846;

// ------------------------------------------------------------------ noise

static double hash3(int x, int y, int z) {
    uint32_t h = uint32_t(x) * 374761393u + uint32_t(y) * 668265263u + uint32_t(z) * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return (h & 0xffffff) / double(0xffffff) * 2.0 - 1.0;
}

double noise3(const V &p) {
    const int ix = int(std::floor(p.x)), iy = int(std::floor(p.y)), iz = int(std::floor(p.z));
    const double fx = p.x - ix, fy = p.y - iy, fz = p.z - iz;
    auto s = [](double t) { return t * t * (3 - 2 * t); };
    const double ux = s(fx), uy = s(fy), uz = s(fz);
    double r = 0;
    for (int c = 0; c < 8; c++) {
        const int dx = c & 1, dy = (c >> 1) & 1, dz = (c >> 2) & 1;
        const double w = (dx ? ux : 1 - ux) * (dy ? uy : 1 - uy) * (dz ? uz : 1 - uz);
        r += w * hash3(ix + dx, iy + dy, iz + dz);
    }
    return r;
}

// ------------------------------------------------------------------ mesh

void Mesh::tri(int a, int b, int c, uint8_t slot, bool smooth, const V &outward) {
    const V n = cross(base[c] - base[a], base[b] - base[a]);
    if (len(n) < 1e-6) return;  // degenerate (lathe centre, collapsed ring)
    if (dot(n, outward) < 0) std::swap(b, c);
    tris.push_back({a, b, c, slot, smooth});
}

double Mesh::dent(const V &centre, const V &dir, double depth, double radius, double max_offset, double seed) {
    double moved = 0;
    const double r2 = radius * radius;
    // sideways buckling direction: panels fold, they do not move like a stamp
    V side = cross(dir, V{0, 0, 1});
    if (len(side) < 0.1) side = cross(dir, V{1, 0, 0});
    side = norm(side);
    for (size_t i = 0; i < base.size(); i++) {
        const V q = base[i] + off[i];
        const V d = q - centre;
        const double d2 = dot(d, d);
        if (d2 >= r2) continue;
        const double f = 1.0 - std::sqrt(d2) / radius;
        const double w = f * f * (3 - 2 * f);
        // creases: low-frequency noise on the rest shape, same for coincident points (no cracks at seams)
        const double n1 = noise3(base[i] * 0.05 + V{seed, seed * 0.7, seed * 1.3});
        const double n2 = noise3(base[i] * 0.13 + V{seed * 1.9, seed, seed * 0.4});
        const double wr = 0.6 + 0.55 * (n1 * 0.5 + 0.5) + 0.25 * n2;
        V o = off[i] + dir * (depth * w * wr) + side * (depth * w * 0.3 * n2);
        const double ol = len(o);
        if (ol > max_offset) o = o * (max_offset / ol);
        moved += len(o - off[i]);
        off[i] = o;
        scrape[i] = float(std::min(1.0, scrape[i] + w * depth * 0.05));
    }
    return moved;
}

void Mesh::scratch(const V &centre, double radius, double amount) {
    const double r2 = radius * radius;
    for (size_t i = 0; i < base.size(); i++) {
        const V d = p(int(i)) - centre;
        const double d2 = dot(d, d);
        if (d2 >= r2) continue;
        const double f = 1.0 - std::sqrt(d2) / radius;
        const double streak = 0.5 + 0.5 * noise3(V{base[i].x * 0.02, base[i].y * 0.6, base[i].z * 0.6});
        scrape[i] = float(std::min(1.0, scrape[i] + amount * f * streak));
    }
}

void Mesh::repair(double fraction) {
    const double k = 1.0 - std::clamp(fraction, 0.0, 1.0);
    for (size_t i = 0; i < base.size(); i++) off[i] = off[i] * k, scrape[i] = float(scrape[i] * k);
}

void Mesh::build_groups() {
    group.assign(base.size(), 0);
    std::vector<std::pair<long long, int>> keys(base.size());
    for (size_t i = 0; i < base.size(); i++) {
        const long long qx = std::llround(base[i].x * 20), qy = std::llround(base[i].y * 20), qz = std::llround(base[i].z * 20);
        keys[i] = {(qx * 73856093LL) ^ (qy * 19349663LL) ^ (qz * 83492791LL), int(i)};
    }
    std::sort(keys.begin(), keys.end());
    int g = -1;
    for (size_t k = 0; k < keys.size(); k++) {
        const int i = keys[k].second;
        bool same = k > 0 && keys[k].first == keys[k - 1].first && len(base[i] - base[keys[k - 1].second]) < 0.1;
        if (!same) g++;
        group[i] = g;
    }
}

void Mesh::point_normals(std::vector<V> &out) const {
    out.assign(base.size(), V{0, 0, 0});
    const bool grouped = group.size() == base.size();
    std::vector<V> acc;
    if (grouped) acc.assign(base.size(), V{0, 0, 0});
    for (const Tri &t : tris) {
        if (!t.smooth) continue;
        const V n = cross(p(t.c) - p(t.a), p(t.b) - p(t.a));
        if (grouped) acc[group[t.a]] += n, acc[group[t.b]] += n, acc[group[t.c]] += n;
        else out[t.a] += n, out[t.b] += n, out[t.c] += n;
    }
    if (!grouped) {
        for (V &n : out) n = norm(n);
        return;
    }
    const bool own = nrm0.size() == base.size();
    for (size_t i = 0; i < base.size(); i++) {
        const V welded = norm(acc[group[i]]);
        if (!own) { out[i] = welded; continue; }
        // keep the artist's normal until the metal moves, then blend to the dented surface
        const double w = std::min(1.0, len(off[i]) / 3.0);
        out[i] = w <= 0 ? nrm0[i] : norm(lerp(nrm0[i], welded, w));
    }
}

V Mesh::centre() const {
    V lo, hi;
    bounds(lo, hi);
    return (lo + hi) * 0.5;
}

void Mesh::bounds(V &lo, V &hi) const {
    lo = V{1e9, 1e9, 1e9}, hi = V{-1e9, -1e9, -1e9};
    for (size_t i = 0; i < base.size(); i++) {
        const V q = p(int(i));
        lo = V{std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z)};
        hi = V{std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z)};
    }
}

// ------------------------------------------------------------------ car shape

namespace {

struct Key {
    double x, v;
};

// smooth interpolation through keys (sorted by x)
double curve(const std::vector<Key> &k, double x) {
    if (x <= k.front().x) return k.front().v;
    if (x >= k.back().x) return k.back().v;
    for (size_t i = 0; i + 1 < k.size(); i++)
        if (x <= k[i + 1].x) {
            double t = (x - k[i].x) / (k[i + 1].x - k[i].x);
            t = t * t * (3 - 2 * t);
            return k[i].v + (k[i + 1].v - k[i].v) * t;
        }
    return k.back().v;
}

double sat(double t) { return std::clamp(t, 0.0, 1.0); }

struct RingParams {
    double w, z0, zb, zt, g;  // half width, bottom edge, belt line, top, greenhouse 0..1
};

constexpr int RING = 29;  // points from the left bottom edge over the top to the right bottom edge
constexpr int MID = 14;   // index of the top centre

// one cross-section: 15 control points (left bottom .. top centre .. right bottom), Catmull-Rom
// subdivided to 29
void ring(double x, const RingParams &r, V out[RING]) {
    const double w = r.w, z0 = r.z0, zb = r.zb, zt = std::max(r.zt, r.zb + 1.0), g = r.g;
    auto L = [&](double deck, double cab) { return deck + (cab - deck) * g; };
    const double c[8][2] = {
        {-0.93 * w, z0},
        {-1.00 * w, z0 + 0.45 * (zb - z0)},
        {-0.985 * w, zb - 0.12 * (zb - z0)},
        {-0.93 * w, zb},
        {-L(0.84, 0.865) * w, L(zb + 0.45 * (zt - zb), zb + 3)},
        {-L(0.62, 0.70) * w, L(zb + 0.85 * (zt - zb), zt - 6)},
        {-L(0.38, 0.57) * w, L(zt - 0.6, zt - 0.5)},
        {0.0, zt + L(1.0, 2.0)},
    };
    V ctl[15];
    for (int k = 0; k < 8; k++) ctl[k] = V{x, c[k][0], c[k][1]};
    for (int k = 0; k < 7; k++) ctl[14 - k] = V{x, -c[k][0], c[k][1]};
    for (int k = 0; k < 15; k++) out[2 * k] = ctl[k];
    for (int k = 0; k < 14; k++) {
        const V p0 = ctl[std::max(k - 1, 0)], p1 = ctl[k], p2 = ctl[k + 1], p3 = ctl[std::min(k + 2, 14)];
        out[2 * k + 1] = (p1 + p2) * 0.5625 - (p0 + p3) * 0.0625;
    }
}

// side profile of the body (x in cm, front +x)
const std::vector<Key> TOP = {{-205, 84}, {-198, 93}, {-165, 97}, {-150, 99}, {-100, 134}, {-80, 140}, {15, 141},
                              {38, 137},  {96, 102},  {150, 97},  {198, 90},  {205, 84}};
const std::vector<Key> BELT = {{-205, 80}, {-195, 90}, {0, 91}, {195, 87}, {205, 80}};
const std::vector<Key> CABIN = {{-168, 0}, {-150, 1}, {96, 1}, {108, 0}};
const std::vector<Key> HALFW = {{-205, 80}, {-190, 87}, {-150, 90}, {150, 90}, {190, 85}, {205, 77}};
const std::vector<Key> BOTTOM = {{-205, 52}, {-196, 24}, {196, 24}, {205, 52}};

double arch_z(double x) {
    double z = 0;
    for (double ax : {WHEELBASE_HALF, -WHEELBASE_HALF}) {
        const double dx = x - ax, R = 40;
        if (std::fabs(dx) < R) z = std::max(z, WHEEL_R + std::sqrt(R * R - dx * dx));
    }
    return z;
}

RingParams body_params(double x) {
    RingParams r;
    r.w = curve(HALFW, x);
    r.zb = curve(BELT, x);
    r.zt = curve(TOP, x);
    r.g = curve(CABIN, x);
    r.z0 = std::max(curve(BOTTOM, x), arch_z(x));
    return r;
}

// axis-aligned box made of flat quads (no bottom when `open_bottom`)
void box(Mesh &m, const V &lo, const V &hi, uint8_t slot, bool open_bottom = false) {
    const V c = (lo + hi) * 0.5;
    int p[8];
    for (int i = 0; i < 8; i++) p[i] = m.add(V{(i & 1) ? hi.x : lo.x, (i & 2) ? hi.y : lo.y, (i & 4) ? hi.z : lo.z});
    auto f = [&](int a, int b, int cc, int d, V o) { m.quad(p[a], p[b], p[cc], p[d], slot, false, o); };
    f(0, 2, 6, 4, V{-1, 0, 0});
    f(1, 3, 7, 5, V{1, 0, 0});
    f(0, 1, 5, 4, V{0, -1, 0});
    f(2, 3, 7, 6, V{0, 1, 0});
    f(4, 5, 7, 6, V{0, 0, 1});
    if (!open_bottom) f(0, 1, 3, 2, V{0, 0, -1});
    (void)c;
}

// lofted shell between stations; `pick(xm, jm)` returns the target mesh (or null) and the slot
template <class Pick>
void loft(const std::vector<double> &xs, const std::vector<RingParams> &rp, double axis_z, Pick pick, std::vector<std::vector<int>> &ids,
          Mesh **meshes, int nmesh) {
    const int ns = int(xs.size());
    std::vector<std::vector<V>> grid(ns, std::vector<V>(RING));
    for (int i = 0; i < ns; i++) ring(xs[i], rp[i], grid[i].data());
    // per mesh, grid point -> mesh point (created on first use, so seams duplicate points exactly)
    ids.assign(nmesh, std::vector<int>(ns * RING, -1));
    auto pt = [&](int mi, int i, int j) {
        int &id = ids[mi][i * RING + j];
        if (id < 0) id = meshes[mi]->add(grid[i][j]);
        return id;
    };
    for (int i = 0; i + 1 < ns; i++)
        for (int j = 0; j + 1 < RING; j++) {
            const double xm = 0.5 * (xs[i] + xs[i + 1]);
            const int jm = j < MID ? j : RING - 2 - j;  // mirrored segment index (0 = bottom edge)
            uint8_t slot = PAINT;
            const int mi = pick(xm, jm, slot);
            if (mi < 0) continue;
            const V cen = (grid[i][j] + grid[i + 1][j] + grid[i + 1][j + 1] + grid[i][j + 1]) * 0.25;
            const V out = cen - V{cen.x, 0, axis_z};
            meshes[mi]->quad(pt(mi, i, j), pt(mi, i + 1, j), pt(mi, i + 1, j + 1), pt(mi, i, j + 1), slot, true, out);
        }
}

// flat cap over an end ring (fan with one inner ring so lights can be cut out of it)
template <class SlotFn>
void cap(Mesh &m, const std::vector<int> &ring_ids, double dir_x, SlotFn slot_of) {
    V c{0, 0, 0};
    for (int id : ring_ids) c += m.base[id];
    c = c * (1.0 / ring_ids.size());
    const int ci = m.add(c);
    std::vector<int> mid;
    for (int id : ring_ids) mid.push_back(m.add(lerp(c, m.base[id], 0.55)));
    const V out{dir_x, 0, 0};
    for (size_t j = 0; j + 1 < ring_ids.size(); j++) {
        const V a = m.base[mid[j]], b = m.base[mid[j + 1]];
        m.tri(ci, mid[j], mid[j + 1], slot_of((a + b + c) * (1.0 / 3)), false, out);
        const V q = (m.base[ring_ids[j]] + m.base[ring_ids[j + 1]] + a + b) * 0.25;
        m.quad(mid[j], ring_ids[j], ring_ids[j + 1], mid[j + 1], slot_of(q), false, out);
    }
    // close the bottom chord
    m.tri(ci, mid.back(), mid.front(), slot_of(c), false, out);
    m.quad(mid.back(), ring_ids.back(), ring_ids.front(), mid.front(), slot_of(c), false, out);
}

void torus(Mesh &m, const V &c, const V &axis, double R, double r, int nu, int nv, uint8_t slot) {
    V a = norm(axis), u = norm(cross(a, V{0, 1, 0}));
    if (len(u) < 0.5) u = norm(cross(a, V{1, 0, 0}));
    const V v = cross(a, u);
    std::vector<int> id(nu * nv);
    for (int i = 0; i < nu; i++)
        for (int k = 0; k < nv; k++) {
            const double t = 2 * TTG_PI * i / nu, s = 2 * TTG_PI * k / nv;
            const V ringc = c + (u * std::cos(t) + v * std::sin(t)) * R;
            const V rad = norm(ringc - c);
            id[i * nv + k] = m.add(ringc + (rad * std::cos(s) + a * std::sin(s)) * r);
        }
    for (int i = 0; i < nu; i++)
        for (int k = 0; k < nv; k++) {
            const int i1 = (i + 1) % nu, k1 = (k + 1) % nv;
            const V q = m.base[id[i * nv + k]], ringc = c + norm(cross(a, cross(q - c, a))) * R;
            m.quad(id[i * nv + k], id[i1 * nv + k], id[i1 * nv + k1], id[i * nv + k1], slot, true, q - ringc);
        }
}

}  // namespace

void build_car(CarGeom &g) {
    g = CarGeom{};
    // ---- main shell, hood and boot lid share the loft
    std::vector<double> xs;
    for (double x = -205; x <= 205.01; x += 5) xs.push_back(x);
    std::vector<RingParams> rp;
    for (double x : xs) rp.push_back(body_params(x));
    Mesh *shell[3] = {&g.body, &g.hood, &g.trunk};
    std::vector<std::vector<int>> ids;
    loft(xs, rp, 55.0, [](double xm, int jm, uint8_t &slot) -> int {
        slot = PAINT;
        if (jm >= 6 && xm > 102 && xm < 199) return 1;   // hood
        if (jm >= 6 && xm > -199 && xm < -154) return 2; // boot lid
        const bool bpillar = std::fabs(xm + 22) < 4.5;
        if (jm >= 8 && jm < 11 && xm > -138 && xm < 36 && !bpillar) slot = GLASS;          // side windows
        if (jm >= 11 && ((xm > 42 && xm < 94) || (xm > -148 && xm < -100))) slot = GLASS;  // windscreen, rear window
        return 0;
    }, ids, shell, 3);
    // end caps: headlights / grille at the front, tail lights at the back
    const int ns = int(xs.size());
    std::vector<int> front, rear;
    for (int j = 0; j < RING; j++) front.push_back(ids[0][(ns - 1) * RING + j]);
    for (int j = 0; j < RING; j++) rear.push_back(ids[0][j]);
    cap(g.body, front, 1.0, [](const V &q) { return uint8_t((q.z > 62 && std::fabs(q.y) > 36) ? HEAD : TRIM); });
    cap(g.body, rear, -1.0, [](const V &q) { return uint8_t((q.z > 58 && std::fabs(q.y) > 40) ? TAIL : PAINT); });
    // underbody between the wheels, dashboard, steering wheel, mirrors
    {
        const int nx = 15, ny = 5;
        std::vector<int> id(nx * ny);
        for (int i = 0; i < nx; i++)
            for (int k = 0; k < ny; k++) id[i * ny + k] = g.body.add(V{-196 + 392.0 * i / (nx - 1), -58 + 116.0 * k / (ny - 1), 22});
        for (int i = 0; i + 1 < nx; i++)
            for (int k = 0; k + 1 < ny; k++)
                g.body.quad(id[i * ny + k], id[(i + 1) * ny + k], id[(i + 1) * ny + k + 1], id[i * ny + k + 1], TRIM, false, V{0, 0, -1});
    }
    box(g.body, V{58, -76, 82}, V{92, 76, 95}, TRIM, true);
    torus(g.body, V{40, -36, 92}, V{-1, 0, 0.45}, 18, 1.7, 24, 6, TRIM);
    for (double s : {-1.0, 1.0}) box(g.body, V{62, s > 0 ? 91.0 : -103.0, 92}, V{72, s > 0 ? 103.0 : -91.0, 101}, PAINT);

    // ---- bumpers: short lofted shells with a capped face
    auto bumper = [&](Mesh &m, double sign) {
        std::vector<double> bx;
        std::vector<RingParams> brp;
        for (int i = 0; i <= 9; i++) {
            const double t = i / 9.0, x = 192 + 34 * t;  // outward distance along the car
            const double taper = sat((x - 198) / 28);
            bx.push_back(sign * x);
            brp.push_back(RingParams{83 - 17 * taper * taper, 18 + 5 * taper, 47 - 2 * taper, 55 - 4 * taper, 0});
        }
        if (sign < 0) std::reverse(bx.begin(), bx.end()), std::reverse(brp.begin(), brp.end());
        Mesh *mm[1] = {&m};
        std::vector<std::vector<int>> bid;
        loft(bx, brp, 35.0, [](double, int jm, uint8_t &slot) -> int {
            slot = jm < 2 ? TRIM : PAINT;
            return 0;
        }, bid, mm, 1);
        const int nb = int(bx.size());
        const int end = sign > 0 ? nb - 1 : 0;
        std::vector<int> r;
        for (int j = 0; j < RING; j++) r.push_back(bid[0][end * RING + j]);
        cap(m, r, sign, [](const V &q) { return uint8_t(q.z < 27 ? TRIM : PAINT); });
    };
    bumper(g.bumper_f, 1.0);
    bumper(g.bumper_r, -1.0);
}

void build_wheel(Mesh &m, double r, double width) {
    m = Mesh{};
    const double h = width * 0.5;
    struct P {
        double r, y;
    };
    // tyre: inner bead -> sidewall -> tread -> outer sidewall -> outer bead
    const std::vector<P> tyre = {{r * 0.64, -h * 0.95}, {r * 0.88, -h}, {r * 0.955, -h * 0.95}, {r, -h * 0.77}, {r * 1.015, -h * 0.36},
                                 {r * 1.015, h * 0.36}, {r, h * 0.77},  {r * 0.955, h * 0.95},  {r * 0.88, h}, {r * 0.64, h * 0.95}};
    // rim face (outer side, dished, with five spokes) and the plain inner face
    const std::vector<P> face = {{r * 0.64, h * 0.95}, {r * 0.60, h * 0.80}, {r * 0.52, h * 0.62}, {r * 0.36, h * 0.55}, {r * 0.22, h * 0.50},
                                 {r * 0.12, h * 0.58}, {r * 0.0, h * 0.58}};
    const std::vector<P> inner = {{0, -h * 0.65}, {r * 0.3, -h * 0.65}, {r * 0.64, -h * 0.95}};
    const int seg = 40;
    auto lathe = [&](const std::vector<P> &prof, uint8_t slot, bool spokes) {
        const int np = int(prof.size());
        std::vector<int> id(np * seg);
        for (int k = 0; k < np; k++)
            for (int s = 0; s < seg; s++) {
                const double t = 2 * TTG_PI * s / seg;
                double y = prof[k].y;
                if (spokes && prof[k].r > r * 0.2 && prof[k].r < r * 0.58) {
                    const double sp = std::pow(0.5 + 0.5 * std::cos(5 * t), 3.0);
                    y -= (1 - sp) * h * 0.45;
                }
                id[k * seg + s] = m.add(V{prof[k].r * std::cos(t), y, prof[k].r * std::sin(t)});
            }
        for (int k = 0; k + 1 < np; k++) {
            const double dr = prof[k + 1].r - prof[k].r, dy = prof[k + 1].y - prof[k].y;
            for (int s = 0; s < seg; s++) {
                const int s1 = (s + 1) % seg;
                const double t = 2 * TTG_PI * (s + 0.5) / seg;
                const V out = V{std::cos(t), 0, std::sin(t)} * dy + V{0, -dr, 0};  // profile normal (dy, -dr)
                m.quad(id[k * seg + s], id[(k + 1) * seg + s], id[(k + 1) * seg + s1], id[k * seg + s1], slot, true, out);
            }
        }
    };
    lathe(tyre, TYRE, false);
    lathe(face, RIM, true);
    lathe(inner, RIM, false);
}

}  // namespace ttg
