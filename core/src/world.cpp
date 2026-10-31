#include <cstdlib>
#include <cmath>
#include "world.hpp"
#include <array>
#include <cstdio>
#include "../third_party/earcut.hpp"

namespace es {

namespace {

V3 lv(V2 p, double z, V2 o) { return {p.x - o.x, p.y - o.y, z}; }

V3 rgb(uint32_t c) { return {((c >> 16) & 255) / 255.0, ((c >> 8) & 255) / 255.0, (c & 255) / 255.0}; }

uint32_t pick(const std::vector<uint32_t> &v, uint64_t h) { return v[hash32(h) % v.size()] | 0x1000000u; }

// triangulate a ring set (outer + holes) and append as a flat face at height z (normal up)
void flat_poly(Mesh &m, const Poly &outer, const std::vector<Poly> &holes, double z, V2 o, V3 c, double alpha, V2 uv2 = {0, 0}, std::vector<float> *col = nullptr) {
    using Pt = std::array<double, 2>;
    std::vector<std::vector<Pt>> poly;
    std::vector<V2> flat;
    poly.emplace_back();
    for (auto &p : outer) poly.back().push_back({p.x, p.y}), flat.push_back(p);
    for (auto &h : holes) {
        poly.emplace_back();
        for (auto &p : h) poly.back().push_back({p.x, p.y}), flat.push_back(p);
    }
    auto idx = mapbox::earcut<uint32_t>(poly);
    if (idx.empty()) return;
    int base = int(m.verts());
    for (auto &p : flat) m.vert(lv(p, z, o), {0, 0, 1}, {p.x, p.y}, uv2, float(c.x), float(c.y), float(c.z), float(alpha));
    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        // earcut keeps the input orientation; outer is CCW so triangles are CCW (up)
        V2 a = flat[idx[i]], b = flat[idx[i + 1]], cc = flat[idx[i + 2]];
        if (cross(b - a, cc - a) >= 0) m.tri(base + idx[i], base + idx[i + 1], base + idx[i + 2]);
        else m.tri(base + idx[i], base + idx[i + 2], base + idx[i + 1]);
        if (col) {
            for (uint32_t k : {idx[i], idx[i + 1], idx[i + 2]}) {
                V3 q = lv(flat[k], z, o);
                col->push_back(float(q.x)), col->push_back(float(q.y)), col->push_back(float(q.z));
            }
        }
    }
}

void push_tri(std::vector<float> &c, V3 a, V3 b, V3 d) {
    for (V3 p : {a, b, d}) c.push_back(float(p.x)), c.push_back(float(p.y)), c.push_back(float(p.z));
}

void collect(const Mesh &m, std::vector<float> &out, size_t from_idx = 0) {
    for (size_t i = from_idx; i + 2 < m.idx.size(); i += 3)
        for (int k = 0; k < 3; k++) {
            int v = m.idx[i + k];
            out.push_back(m.pos[v * 3]), out.push_back(m.pos[v * 3 + 1]), out.push_back(m.pos[v * 3 + 2]);
        }
}

// vertical box (pier, pole base): centre c, half extents along dir u and perp, from z0 to z1
void box(Mesh &m, V2 c, V2 u, double hu, double hv, double z0, double z1, V2 o, std::vector<float> *col) {
    V2 v = perp(u);
    V2 p[4] = {c - u * hu - v * hv, c + u * hu - v * hv, c + u * hu + v * hv, c - u * hu + v * hv};  // CCW
    size_t start = m.idx.size();
    for (int i = 0; i < 4; i++) {
        V2 a = p[i], b = p[(i + 1) % 4];
        double l = dist(a, b);
        add_quad(m, lv(a, z0, o), lv(b, z0, o), lv(b, z1, o), lv(a, z1, o), {0, 0}, {l, 0}, {l, z1 - z0}, {0, z1 - z0});
    }
    add_quad(m, lv(p[0], z1, o), lv(p[1], z1, o), lv(p[2], z1, o), lv(p[3], z1, o), {0, 0}, {1, 0}, {1, 1}, {0, 1});
    if (col) collect(m, *col, start);
}

AreaKind area_at(const std::vector<const RawArea *> &areas, V2 p, int &prio_out) {
    // later areas with higher priority win: forest > park > grass > pitch > paved > others
    auto prio = [](AreaKind k) {
        switch (k) {
            case AreaKind::Water: return 9;
            case AreaKind::Forest: return 7;
            case AreaKind::Pitch: return 6;
            case AreaKind::Paved: case AreaKind::Parking: return 6;
            case AreaKind::Park: return 5;
            case AreaKind::Grass: case AreaKind::Scrub: return 5;
            case AreaKind::Sand: case AreaKind::Cemetery: case AreaKind::Farmland: return 4;
            case AreaKind::Construction: case AreaKind::Industrial: return 2;
            case AreaKind::Residential: return 1;
            default: return 0;
        }
    };
    AreaKind best = AreaKind::Other;
    int bp = -1;
    for (auto *a : areas) {
        int pr = prio(a->kind);
        if (pr <= bp) continue;
        if (!point_in_poly(a->ring.outer, p)) continue;
        bool in_hole = false;
        for (auto &h : a->ring.holes)
            if (point_in_poly(h, p)) in_hole = true;
        if (in_hole) continue;
        best = a->kind, bp = pr;
    }
    prio_out = bp;
    return best;
}

}  // namespace

// ------------------------------------------------------------------ load

namespace {
struct Ctx {
    RoadGeom::Cache rc;
    std::vector<const RawArea *> areas;  // all areas touching the context rect
    std::vector<std::pair<const RawArea *, double>> water;  // water areas and their level
};
}  // namespace

static std::vector<double> &water_levels() {
    static std::vector<double> v;
    return v;
}

bool World::load(const std::string &root) {
    if (!region.load(root)) {
        log = region.log;
        return false;
    }
    log = region.log;
    net.build(region);
    for (auto &l : net.log) log.push_back(l);
    geom.build(net);
    // tile seams
    for (auto &t : region.tiles) {
        V2 a, b;
        region.frame.tile_rect(t, a, b);
        seam_x.push_back(a.x), seam_x.push_back(b.x), seam_y.push_back(a.y), seam_y.push_back(b.y);
    }
    std::sort(seam_x.begin(), seam_x.end()), seam_x.erase(std::unique(seam_x.begin(), seam_x.end()), seam_x.end());
    std::sort(seam_y.begin(), seam_y.end()), seam_y.erase(std::unique(seam_y.begin(), seam_y.end()), seam_y.end());
    // water levels: lowest smoothed ground along the shore, a little below
    auto &wl = water_levels();
    wl.assign(region.areas.size(), 0);
    for (size_t i = 0; i < region.areas.size(); i++) {
        auto &a = region.areas[i];
        if (a.kind != AreaKind::Water) continue;
        double mn = 1e9;
        auto &r = a.ring.outer;
        for (size_t k = 0; k < r.size(); k++) {
            V2 p = r[k], q = r[(k + 1) % r.size()];
            int n = std::max(1, int(dist(p, q) / 10));
            for (int j = 0; j < n; j++) mn = std::min(mn, region.dem.sample(lerp(p, q, double(j) / n)));
        }
        wl[i] = mn - 0.6;
    }
    prepare_buildings();
    char b[200];
    snprintf(b, sizeof b, "world: %zu buildings prepared, %zu chunks with buildings", buildings.size(), chunk_buildings.size());
    log.push_back(b);
    return true;
}

void World::chunk_range(int &x0, int &y0, int &x1, int &y1) const {
    x0 = int(std::floor(region.bounds.mn.x / CHUNK)), y0 = int(std::floor(region.bounds.mn.y / CHUNK));
    x1 = int(std::floor(region.bounds.mx.x / CHUNK)), y1 = int(std::floor(region.bounds.mx.y / CHUNK));
}

// ------------------------------------------------------------------ buildings

namespace {
double default_floors(const std::string &cls, double area, int &style) {
    if (cls == "garage" || cls == "garages" || cls == "shed" || cls == "carport" || cls == "roof" || cls == "kiosk" || cls == "toilets") return style = 6, 1;
    if (cls == "industrial" || cls == "warehouse" || cls == "service" || cls == "transportation" || cls == "hangar" || cls == "manufacture" || cls == "storage_tank")
        return style = 4, area > 2000 ? 3 : 2;
    if (cls == "house" || cls == "detached" || cls == "semidetached_house" || cls == "bungalow" || cls == "cabin" || cls == "farm") return style = 5, 2;
    if (cls == "church" || cls == "cathedral" || cls == "chapel" || cls == "religious") return style = 2, 4;
    if (cls == "school" || cls == "kindergarten" || cls == "hospital" || cls == "university" || cls == "college" || cls == "public" || cls == "civic")
        return style = 2, cls == "kindergarten" ? 2 : area > 2000 ? 5 : 3;
    if (cls == "office" || cls == "commercial" || cls == "retail") return style = area > 1500 ? 3 : 2, area > 1500 ? 6 : 3;
    if (cls == "apartments" || cls == "residential" || cls == "dormitory") return style = 0, area > 900 ? 9 : area > 300 ? 5 : 3;
    if (area < 40) return style = 6, 1;
    if (area < 150) return style = 5, 2;
    if (area < 600) return style = 2, 4;
    return style = 0, 5;
}
}  // namespace

void World::prepare_buildings() {
    buildings.clear();
    chunk_buildings.clear();
    std::unordered_map<std::string, int> by_id;
    auto &RB = region.buildings;
    for (size_t i = 0; i < RB.size(); i++) {
        auto &r = RB[i];
        if (r.underground || r.rings.empty()) continue;
        Building b;
        b.src = int(i);
        double best = -1;
        for (auto &ring : r.rings) {
            double a = std::fabs(poly_area(ring.outer));
            b.area += a;
            for (auto &p : ring.outer) b.box.add(p);
            if (a > best) best = a, b.centroid = poly_centroid(ring.outer);
        }
        if (b.area < 4) continue;
        uint64_t h = hash_str(r.id.data(), r.id.size());
        int style = 0;
        double df = default_floors(r.cls.empty() ? r.subtype : r.cls, b.area, style);
        if (!r.facade_material.empty()) {
            const auto &m = r.facade_material;
            style = m == "glass" ? 3 : m == "brick" ? 1 : m == "plaster" ? 2 : m == "metal" ? 4 : m == "wood" ? 5 : m == "concrete" || m == "panel" ? 0 : style;
        }
        b.style = style;
        b.floor_h = style == 3 ? 3.8 : style == 4 ? 5.0 : style == 6 ? 3.0 : (r.cls == "office" || r.cls == "commercial" || r.cls == "retail") ? 3.6 : 3.0;
        double floors = r.floors > 0 ? r.floors : df;
        b.height = r.height > 0 ? r.height : floors * b.floor_h + (r.floors > 0 ? 0.8 : 0.6);
        if (r.height > 0 && r.floors > 0) b.floor_h = std::clamp((r.height - (r.roof_height > 0 ? r.roof_height : 0)) / r.floors, 2.6, 6.0);
        b.min_h = r.min_height > 0 ? r.min_height : r.min_floor > 0 ? r.min_floor * b.floor_h : 0;
        b.roof_h = r.roof_height;
        b.roof_shape = r.roof_shape;
        static const std::vector<uint32_t> panel = {0xD8D4CC, 0xCFC9BE, 0xE3DED3, 0xC9CED3, 0xDCD3C4, 0xE8E4DA, 0xBFC4C6};
        static const std::vector<uint32_t> brick = {0x9C4B37, 0xA65B42, 0x8E4A3A, 0xB8846A, 0xC9A77C};
        static const std::vector<uint32_t> plaster = {0xEADBB0, 0xF0E6D2, 0xE8C9B8, 0xDDE4E8, 0xF4F1E8, 0xE9D9A6, 0xD9E2D0};
        static const std::vector<uint32_t> glass = {0x7C8C99, 0x6F8296, 0x8A9AA3};
        static const std::vector<uint32_t> indus = {0xA9A9A4, 0x8F9396, 0xB5B0A5};
        static const std::vector<uint32_t> house = {0xD9C7A6, 0xC4B59A, 0xE6E0D0, 0xB9A58A};
        static const std::vector<uint32_t> garage = {0x8D8D88, 0x9A968E};
        const std::vector<uint32_t> *pal[] = {&panel, &brick, &plaster, &glass, &indus, &house, &garage};
        b.facade = r.facade_color ? r.facade_color : pick(*pal[std::clamp(style, 0, 6)], h);
        bool pitched = r.roof_shape == "gabled" || r.roof_shape == "hipped" || r.roof_shape == "pyramidal" || r.roof_shape == "skillion" || r.roof_shape == "half-hipped";
        static const std::vector<uint32_t> flatr = {0x4A4A4A, 0x555555, 0x5E5E5E, 0x505452};
        static const std::vector<uint32_t> pitchr = {0x8B3A2B, 0x7A3B2E, 0x5C6B5C, 0x6E6E6E, 0x4E5A66};
        b.roof = r.roof_color ? r.roof_color : pick(pitched ? pitchr : flatr, h >> 7);
        {
            // data colours are swatches; real roofs are weathered: darken and desaturate a little
            uint32_t c = b.roof;
            double rr = ((c >> 16) & 255), gg = ((c >> 8) & 255), bb = (c & 255), l = (rr + gg + bb) / 3;
            auto f = [&](double v) { return uint32_t(std::clamp((l + (v - l) * 0.7) * 0.8, 0.0, 255.0)); };
            b.roof = (f(rr) << 16) | (f(gg) << 8) | f(bb) | 0x1000000u;
        }
        by_id[r.id] = int(buildings.size());
        buildings.push_back(std::move(b));
    }
    // parts: link to parents; hide parents mostly covered by their parts
    std::unordered_map<int, double> covered;
    for (auto &b : buildings) {
        auto &r = RB[b.src];
        if (!r.part) continue;
        auto it = by_id.find(r.parent);
        if (it != by_id.end()) b.parent = it->second, covered[it->second] += b.area;
    }
    for (auto &kv : covered)
        if (kv.second >= 0.75 * buildings[kv.first].area) buildings[kv.first].render = false;
    for (size_t i = 0; i < buildings.size(); i++) {
        auto &b = buildings[i];
        int cx = int(std::floor(b.centroid.x / CHUNK)), cy = int(std::floor(b.centroid.y / CHUNK));
        chunk_buildings[ckey(cx, cy)].push_back(int(i));
    }
}

// ------------------------------------------------------------------ terrain

static void make_ctx(const World &w, const Rect &r, Ctx &c) {
    w.geom.prefetch(r, c.rc);
    auto &wl = water_levels();
    for (size_t i = 0; i < w.region.areas.size(); i++) {
        auto &a = w.region.areas[i];
        Rect b;
        for (auto &p : a.ring.outer) b.add(p);
        if (!b.overlaps(r)) continue;
        c.areas.push_back(&a);
        if (a.kind == AreaKind::Water) c.water.push_back({&a, wl[i]});
    }
}

static double terrain_ctx(const World &w, V2 p, const Ctx &c, bool *hole, int *kind = nullptr) {
    double G = w.region.dem.sample(p);
    // water: lower the bed inside, slope the banks
    for (auto &wa : c.water) {
        auto &ring = wa.first->ring;
        double lvl = wa.second;
        bool in = point_in_poly(ring.outer, p);
        for (auto &hh : ring.holes)
            if (point_in_poly(hh, p)) in = false;
        if (in) {
            G = std::min(G, lvl - 1.4);
            continue;
        }
        double d = 1e9;
        auto &r = ring.outer;
        for (size_t k = 0; k < r.size(); k++) d = std::min(d, seg_dist(p, r[k], r[(k + 1) % r.size()]));
        if (d < 12) G = std::min(G, lvl - 1.4 + (G - lvl + 1.4) * smoothstep(0, 12, d) + 0.0);
    }
    SurfaceHit h = w.geom.query(p, c.rc, 30);
    if (kind) *kind = h.kind;
    if (h.kind == 0 && h.tunnel) {
        if (hole && G - h.tunnel_z < 6.4) *hole = true;
        return G;
    }
    if (h.kind == 1) return h.bridge ? std::min(G, h.z - 1.7) : h.z - 0.5;
    if (h.kind == 2) return h.bridge ? std::min(G, h.z - 1.9) : h.z - 0.03;
    if (h.dist < 30 && !h.bridge) {
        double target = h.znear - 0.03;
        double B = std::clamp(3 + 2.5 * std::fabs(target - G), 3.0, 30.0);
        double wgt = 1 - smoothstep(0, B, h.dist);
        G = lerpd(G, target, wgt);
    }
    return G;
}

double World::terrain_at(V2 p, const RoadGeom::Cache &rc, bool *hole) const {
    Ctx c;
    make_ctx(*this, Rect{p - V2{40, 40}, p + V2{40, 40}}, c);
    (void)rc;
    return terrain_ctx(*this, p, c, hole);
}

double World::ground_at(V2 p) const {
    Ctx c;
    make_ctx(*this, Rect{p - V2{45, 45}, p + V2{45, 45}}, c);
    SurfaceHit h = geom.query(p, c.rc, 0);
    if (h.kind == 2 || (h.kind == 1 && !h.bridge)) return h.z;
    if (h.kind == 1 && h.bridge) return h.z;
    return terrain_ctx(*this, p, c, nullptr);
}

double World::building_base(const Building &b, const RoadGeom::Cache &, double *lowest) const {
    Ctx c;
    make_ctx(*this, b.box.grown(35), c);
    double mx = -1e9, mn = 1e9;
    auto &r = region.buildings[b.src];
    for (auto &ring : r.rings) {
        std::vector<V2> pts = ring.outer;
        pts.push_back(poly_centroid(ring.outer));
        // just outside each wall: a pavement there must not be above the ground floor
        auto &P = ring.outer;
        for (size_t i = 0; i < P.size(); i++) {
            V2 a = P[i], q = P[(i + 1) % P.size()];
            V2 out = norm(perp(q - a) * -1.0);
            pts.push_back((a + q) * 0.5 + out * 0.8);
        }
        for (size_t k = 0; k < pts.size(); k++) {
            V2 p = pts[k];
            bool outside = k > ring.outer.size();
            SurfaceHit h = geom.query(p, c.rc, 0);
            if (outside) {
                if (h.kind == 2 && !h.bridge) mx = std::max(mx, h.z);
                continue;
            }
            double t = terrain_ctx(*this, p, c, nullptr);
            double z = (h.kind && !h.bridge) ? h.z : t;
            mx = std::max(mx, z);
            mn = std::min(mn, std::min(z, t));
        }
    }
    if (lowest) *lowest = mn;
    return mx;
}

// ------------------------------------------------------------------ building mesh

void World::emit_building(const Building &b, double base, double foot, V2 o, ChunkData &out) const {
    auto &r = region.buildings[b.src];
    Mesh &W = out.mats[M_WALL];
    Mesh &Rf = out.mats[M_ROOF];
    V3 fc = rgb(b.facade), rc = rgb(b.roof);
    float seed = float(hash01(hash_str(r.id.data(), r.id.size())));
    double top = base + b.height;
    double bottom = b.min_h > 0.3 ? base + b.min_h : foot;  // foundations go below the lowest ground
    auto on_seam = [&](V2 a, V2 c) {
        for (double x : seam_x)
            if (a.x == x && c.x == x) return true;
        for (double y : seam_y)
            if (a.y == y && c.y == y) return true;
        return false;
    };
    // pitched roofs only on simple quads that are not cut by tile seams
    bool simple = r.rings.size() == 1 && r.rings[0].holes.empty() && r.rings[0].outer.size() == 4;
    if (simple)
        for (size_t i = 0; i < 4; i++)
            if (on_seam(r.rings[0].outer[i], r.rings[0].outer[(i + 1) % 4])) simple = false;
    std::string rs = b.roof_shape;
    bool pitched = simple && (rs == "gabled" || rs == "hipped" || rs == "pyramidal" || rs == "skillion" || rs == "half-hipped");
    // apex roofs (pyramid / dome / spire, and hipped roofs of small houses of any shape): one ring, no holes, no seams
    bool single = r.rings.size() == 1 && r.rings[0].holes.empty() && r.rings[0].outer.size() >= 3;
    if (single)
        for (size_t i = 0; i < r.rings[0].outer.size(); i++)
            if (on_seam(r.rings[0].outer[i], r.rings[0].outer[(i + 1) % r.rings[0].outer.size()])) single = false;
    bool apex = !pitched && single &&
                (rs == "pyramidal" || rs == "dome" || rs == "onion" || rs == "cone" || (b.roof_h > 0.5 * b.height && b.roof_h > 3) ||
                 ((rs == "hipped" || rs == "gabled" || rs == "half-hipped") && b.area < 350));
    double rh = 0;
    if (pitched) {
        auto &q = r.rings[0].outer;
        double s1 = std::min(dist(q[0], q[1]), dist(q[1], q[2]));
        rh = b.roof_h > 0.3 ? b.roof_h : std::clamp(s1 * 0.35, 1.5, 5.0);
        rh = std::min(rh, b.height * 0.6);
    } else if (apex) {
        double rad = std::sqrt(b.area / PI);
        rh = b.roof_h > 0.3 ? b.roof_h : std::clamp(rad * (rs == "dome" || rs == "onion" ? 0.9 : 0.6), 1.5, 8.0);
        rh = std::min(rh, b.height * 0.97);
    }
    double eave = top - rh;
    double rim = (pitched || apex) ? 0.0 : (b.height > 6 ? 0.45 : 0.2);  // parapet of flat roofs
    double wall_top = (pitched || apex) ? eave : top + rim;
    size_t wall_start = W.idx.size();
    struct Seg {
        V2 a, c;
        double u0;
    };
    std::vector<Seg> outer_walls;
    for (auto &ring : r.rings) {
        std::vector<const Poly *> loops = {&ring.outer};
        for (auto &h : ring.holes) loops.push_back(&h);
        for (auto *lp : loops) {
            const Poly &P = *lp;
            double acc = 0;
            for (size_t i = 0; i < P.size(); i++) {
                V2 a = P[i], c = P[(i + 1) % P.size()];
                double l = dist(a, c);
                if (l < 0.05) continue;
                if (on_seam(a, c)) {
                    acc += l;
                    continue;
                }
                V3 n = norm(V3(perp(c - a) * -1.0, 0));  // outward for CCW outer and CW holes
                int i0 = W.vert(lv(a, bottom, o), n, {acc, bottom - base}, {b.floor_h, double(b.style)}, float(fc.x), float(fc.y), float(fc.z), seed);
                int i1 = W.vert(lv(c, bottom, o), n, {acc + l, bottom - base}, {b.floor_h, double(b.style)}, float(fc.x), float(fc.y), float(fc.z), seed);
                int i2 = W.vert(lv(c, wall_top, o), n, {acc + l, wall_top - base}, {b.floor_h, double(b.style)}, float(fc.x), float(fc.y), float(fc.z), seed);
                int i3 = W.vert(lv(a, wall_top, o), n, {acc, wall_top - base}, {b.floor_h, double(b.style)}, float(fc.x), float(fc.y), float(fc.z), seed);
                W.quad(i0, i1, i2, i3);
                if (rim > 0) {
                    // parapet: 25 cm thick wall around the flat roof (top + inner face; the outer face is the wall)
                    V2 in = perp(c - a) / std::max(l, 1e-6) * 0.25;  // towards the roof (left of travel)
                    V2 ai = a + in, ci = c + in;
                    V3 ni = n * -1.0;
                    V3 rc2 = rc * 0.85;
                    int j0 = Rf.vert(lv(ci, top, o), ni, {acc + l, 0}, {0, 0}, float(rc2.x), float(rc2.y), float(rc2.z), 0);
                    int j1 = Rf.vert(lv(ai, top, o), ni, {acc, 0}, {0, 0}, float(rc2.x), float(rc2.y), float(rc2.z), 0);
                    int j2 = Rf.vert(lv(ai, top + rim, o), ni, {acc, rim}, {0, 0}, float(rc2.x), float(rc2.y), float(rc2.z), 0);
                    int j3 = Rf.vert(lv(ci, top + rim, o), ni, {acc + l, rim}, {0, 0}, float(rc2.x), float(rc2.y), float(rc2.z), 0);
                    Rf.quad(j0, j1, j2, j3);
                    add_quad(Rf, lv(a, top + rim, o), lv(c, top + rim, o), lv(ci, top + rim, o), lv(ai, top + rim, o), {0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}, float(rc2.x), float(rc2.y), float(rc2.z), 0);
                }
                if (lp == &ring.outer) outer_walls.push_back({a, c, acc});
                acc += l;
            }
        }
    }
    collect(W, out.col_solid, wall_start);
    // balconies on residential blocks: slab + parapet boxes in every other window bay
    if ((b.style == 0 || b.style == 1) && b.min_h < 0.3) {
        int floors = int((eave - base) / b.floor_h);
        double bay = b.style == 0 ? 3.0 : 2.7;
        uint64_t hb = hash_str(r.id.data(), r.id.size());
        bool glazed = hash01(hb * 7) < 0.55;
        int pattern = int(hash32(hb * 11) % 3);  // which bays get balconies
        if (floors >= 3)
            for (auto &sg : outer_walls) {
                double l = dist(sg.a, sg.c);
                if (l < 9) continue;
                V2 dir = (sg.c - sg.a) / l, out = perp(dir) * -1.0;
                // bays are aligned to the facade shader's grid (u from the ring start)
                int k0 = int(std::ceil(sg.u0 / bay)), k1 = int(std::floor((sg.u0 + l) / bay)) - 1;
                for (int k = k0; k <= k1; k++) {
                    if ((k + pattern) % 3 == 0) continue;
                    double uc = (k + 0.5) * bay - sg.u0;
                    if (uc < 1.6 || uc > l - 1.6) continue;
                    V2 m = sg.a + dir * uc;
                    for (int f = 1; f < floors; f++) {
                        double z0 = base + f * b.floor_h;
                        double hw = bay * 0.46, dp = 1.05, ph = glazed ? b.floor_h * 0.92 : 1.05;
                        V2 p0 = m - dir * hw, p1 = m + dir * hw, q0 = p0 + out * dp, q1 = p1 + out * dp;
                        V3 bc = fc * 0.93;
                        V2 uv2{b.floor_h, glazed ? 8.0 : 7.0};
                        // front, sides (facing out), slab underside, top of the parapet
                        add_quad(W, lv(q0, z0 - 0.15, o), lv(q1, z0 - 0.15, o), lv(q1, z0 + ph, o), lv(q0, z0 + ph, o), {0, 0}, {hw * 2, 0}, {hw * 2, ph + 0.15}, {0, ph + 0.15}, uv2, float(bc.x), float(bc.y), float(bc.z), seed);
                        add_quad(W, lv(p0, z0 - 0.15, o), lv(q0, z0 - 0.15, o), lv(q0, z0 + ph, o), lv(p0, z0 + ph, o), {0, 0}, {dp, 0}, {dp, ph + 0.15}, {0, ph + 0.15}, uv2, float(bc.x), float(bc.y), float(bc.z), seed);
                        add_quad(W, lv(q1, z0 - 0.15, o), lv(p1, z0 - 0.15, o), lv(p1, z0 + ph, o), lv(q1, z0 + ph, o), {0, 0}, {dp, 0}, {dp, ph + 0.15}, {0, ph + 0.15}, uv2, float(bc.x), float(bc.y), float(bc.z), seed);
                        add_quad(W, lv(p0, z0 - 0.15, o), lv(p1, z0 - 0.15, o), lv(q1, z0 - 0.15, o), lv(q0, z0 - 0.15, o), {0, 0}, {1, 0}, {1, 1}, {0, 1}, {b.floor_h, 9.0}, float(bc.x), float(bc.y), float(bc.z), seed);
                        if (glazed || f == floors - 1)
                            add_quad(W, lv(q0, z0 + ph, o), lv(q1, z0 + ph, o), lv(p1, z0 + ph, o), lv(p0, z0 + ph, o), {0, 0}, {1, 0}, {1, 1}, {0, 1}, {b.floor_h, 9.0}, float(bc.x), float(bc.y), float(bc.z), seed);
                    }
                }
            }
    }
    if (apex) {
        const Poly &P = r.rings[0].outer;
        V2 c = poly_centroid(P);
        if (!point_in_poly(P, c)) {
            // strongly concave footprint: fall back to a flat roof at the eave
            flat_poly(Rf, P, {}, eave, o, rc, 0);
            return;
        }
        bool dome = rs == "dome" || rs == "onion";
        V3 A = lv(c, top, o);
        for (size_t i = 0; i < P.size(); i++) {
            V2 a = P[i], q = P[(i + 1) % P.size()];
            if (dist(a, q) < 0.05) continue;
            if (dome) {
                // two tiers: steep lower band, then a shallow cap
                V2 a2 = lerp(c, a, 0.72), q2 = lerp(c, q, 0.72);
                double zm = eave + rh * 0.62;
                V3 n1 = norm(cross(lv(q, eave, o) - lv(a, eave, o), lv(q2, zm, o) - lv(a, eave, o)));
                int i0 = Rf.vert(lv(a, eave, o), n1, {a.x, a.y}, {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                int i1 = Rf.vert(lv(q, eave, o), n1, {q.x, q.y}, {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                int i2 = Rf.vert(lv(q2, zm, o), n1, {q2.x, q2.y}, {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                int i3 = Rf.vert(lv(a2, zm, o), n1, {a2.x, a2.y}, {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                Rf.quad(i0, i1, i2, i3);
                V3 n2 = norm(cross(lv(q2, zm, o) - lv(a2, zm, o), A - lv(a2, zm, o)));
                int j0 = Rf.vert(lv(a2, zm, o), n2, {a2.x, a2.y}, {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                int j1 = Rf.vert(lv(q2, zm, o), n2, {q2.x, q2.y}, {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                int j2 = Rf.vert(A, n2, {c.x, c.y}, {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                Rf.tri(j0, j1, j2);
            } else {
                V3 pa = lv(a, eave, o), pq = lv(q, eave, o);
                V3 n = norm(cross(pq - pa, A - pa));
                int i0 = Rf.vert(pa, n, {a.x, a.y}, {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                int i1 = Rf.vert(pq, n, {q.x, q.y}, {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                int i2 = Rf.vert(A, n, {c.x, c.y}, {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                Rf.tri(i0, i1, i2);
            }
        }
        return;
    }
    if (!pitched) {
        for (auto &ring : r.rings) flat_poly(Rf, ring.outer, ring.holes, top, o, rc, 0);
        // a thin parapet line look is left to the shader; flat roofs of tall buildings get a roof box
        return;
    }
    auto q = r.rings[0].outer;  // CCW quad
    // long axis: edges 0-1 / 2-3 or 1-2 / 3-0
    if (dist(q[1], q[2]) + dist(q[3], q[0]) > dist(q[0], q[1]) + dist(q[2], q[3])) std::rotate(q.begin(), q.begin() + 1, q.end());
    // now 0-1 and 2-3 are the long sides; ridge runs between the midpoints of 1-2 and 3-0
    V2 m12 = (q[1] + q[2]) * 0.5, m30 = (q[3] + q[0]) * 0.5;
    double sl = std::min(dist(q[1], q[2]), dist(q[3], q[0]));
    V2 ax = norm(m12 - m30);
    V2 ra = m30, rb = m12;
    if (rs == "hipped" || rs == "half-hipped") {
        double in = std::min(sl * 0.5, dist(m30, m12) * 0.45) * (rs == "half-hipped" ? 0.5 : 1.0);
        ra = m30 + ax * in, rb = m12 - ax * in;
    } else if (rs == "pyramidal")
        ra = rb = (m30 + m12) * 0.5;
    V3 RA = lv(ra, top, o), RB = lv(rb, top, o);
    V3 Q[4] = {lv(q[0], eave, o), lv(q[1], eave, o), lv(q[2], eave, o), lv(q[3], eave, o)};
    auto tri = [&](Mesh &m, V3 a, V3 bb, V3 c, V3 col, bool solid) {
        V3 n = norm(cross(bb - a, c - a));
        V2 ua{a.x + o.x, a.y + o.y}, ub{bb.x + o.x, bb.y + o.y}, uc{c.x + o.x, c.y + o.y};
        if (&m == &W) ua = {0, a.z - base}, ub = {dist(a.xy(), bb.xy()), bb.z - base}, uc = {dist(a.xy(), c.xy()) * 0.5, c.z - base};
        int i0 = m.vert(a, n, ua, {b.floor_h, double(b.style)}, float(col.x), float(col.y), float(col.z), seed);
        int i1 = m.vert(bb, n, ub, {b.floor_h, double(b.style)}, float(col.x), float(col.y), float(col.z), seed);
        int i2 = m.vert(c, n, uc, {b.floor_h, double(b.style)}, float(col.x), float(col.y), float(col.z), seed);
        m.tri(i0, i1, i2);
        if (solid) push_tri(out.col_solid, a, bb, c);
    };
    // roof planes (CCW seen from above/outside)
    tri(Rf, Q[0], Q[1], RB, rc, false);
    tri(Rf, Q[0], RB, RA, rc, false);
    tri(Rf, Q[2], Q[3], RA, rc, false);
    tri(Rf, Q[2], RA, RB, rc, false);
    // ends: hips are roof, gables are wall
    bool gable = rs == "gabled" || rs == "skillion";
    tri(gable ? W : Rf, Q[1], Q[2], RB, gable ? fc : rc, false);
    tri(gable ? W : Rf, Q[3], Q[0], RA, gable ? fc : rc, false);
}

// ------------------------------------------------------------------ chunk

std::unique_ptr<ChunkData> World::build_chunk(int cx, int cy) const {
    auto out = std::make_unique<ChunkData>();
    ChunkData &C = *out;
    C.cx = cx, C.cy = cy;
    Rect rect = chunk_rect(cx, cy);
    V2 o = (rect.mn + rect.mx) * 0.5;
    C.origin = o;
    Ctx ctx;
    make_ctx(*this, rect.grown(45), ctx);

    // ---- terrain grid with a one-cell margin for normals
    const int M = TN + 3;  // samples per side incl. margin
    std::vector<double> T(size_t(M) * M);
    std::vector<uint8_t> hole(size_t(M) * M, 0), kind(size_t(M) * M, 0);
    for (int j = 0; j < M; j++)
        for (int i = 0; i < M; i++) {
            V2 p = rect.mn + V2{(i - 1) * TCELL, (j - 1) * TCELL};
            bool hl = false;
            int k = 0;
            T[size_t(j) * M + i] = terrain_ctx(*this, p, ctx, &hl, &k);
            hole[size_t(j) * M + i] = hl;
            kind[size_t(j) * M + i] = uint8_t(k);
        }
    // hard surfaces must cover the terrain triangles under them: every vertex is pushed below all
    // carriageway/pavement surface found in the cells around it
    for (int j = 0; j < M; j++)
        for (int i = 0; i < M; i++) {
            V2 v = rect.mn + V2{(i - 1) * TCELL, (j - 1) * TCELL};
            double zmin = 1e9;
            bool near = kind[size_t(j) * M + i] != 0;
            for (int dj = -1; dj <= 1 && !near; dj++)
                for (int di = -1; di <= 1 && !near; di++) {
                    int ii = i + di, jj = j + dj;
                    if (ii >= 0 && jj >= 0 && ii < M && jj < M && kind[size_t(jj) * M + ii]) near = true;
                }
            if (!near) continue;
            static const double off[] = {-0.97, -0.33, 0.33, 0.97};
            for (double oy : off)
                for (double ox : off) {
                    SurfaceHit h = geom.query(v + V2{ox * TCELL, oy * TCELL}, ctx.rc, 0);
                    if (h.kind && !h.bridge) zmin = std::min(zmin, h.z - (h.kind == 1 ? 0.12 : 0.05));
                }
            double &t = T[size_t(j) * M + i];
            if (zmin < t) t = zmin;
        }
    C.ground.resize(size_t(TN + 1) * (TN + 1));
    {
        Mesh &m = C.mats[M_TERRAIN];
        std::vector<int> id(size_t(TN + 1) * (TN + 1));
        for (int j = 0; j <= TN; j++)
            for (int i = 0; i <= TN; i++) {
                int gi = i + 1, gj = j + 1;
                double z = T[size_t(gj) * M + gi];
                C.ground[size_t(j) * (TN + 1) + i] = float(z);
                double dx = (T[size_t(gj) * M + gi + 1] - T[size_t(gj) * M + gi - 1]) / (2 * TCELL);
                double dy = (T[size_t(gj + 1) * M + gi] - T[size_t(gj - 1) * M + gi]) / (2 * TCELL);
                V3 n = norm(V3(-dx, -dy, 1));
                V2 p = rect.mn + V2{i * TCELL, j * TCELL};
                int pr;
                AreaKind ak = area_at(ctx.areas, p, pr);
                // colour weights: r grass, g forest floor, b bare/sand, a paved/urban dirt
                float r = 0.75f, g = 0.0f, b = 0.1f, a = 0.15f;
                switch (ak) {
                    case AreaKind::Forest: r = 0.3f, g = 0.7f, b = 0.0f, a = 0.0f; break;
                    case AreaKind::Park: case AreaKind::Grass: case AreaKind::Cemetery: r = 1.0f, g = 0.0f, b = 0.0f, a = 0.0f; break;
                    case AreaKind::Scrub: r = 0.6f, g = 0.3f, b = 0.1f, a = 0.0f; break;
                    case AreaKind::Pitch: r = 1.0f, g = 0.0f, b = 0.0f, a = 0.0f; break;
                    case AreaKind::Sand: case AreaKind::Construction: r = 0.1f, g = 0.0f, b = 0.9f, a = 0.0f; break;
                    case AreaKind::Paved: case AreaKind::Parking: r = 0.0f, g = 0.0f, b = 0.0f, a = 1.0f; break;
                    case AreaKind::Industrial: r = 0.25f, g = 0.0f, b = 0.25f, a = 0.5f; break;
                    case AreaKind::Farmland: r = 0.6f, g = 0.0f, b = 0.4f, a = 0.0f; break;
                    case AreaKind::Water: r = 0.2f, g = 0.0f, b = 0.8f, a = 0.0f; break;
                    default: break;
                }
                // next to roads the ground is trodden: more urban dirt
                if (kind[size_t(gj) * M + gi]) r *= 0.3f, a = 1.0f - r - g - b;
                id[size_t(j) * (TN + 1) + i] = m.vert(lv(p, z, o), n, {p.x, p.y}, {0, 0}, r, g, b, a);
            }
        C.hole.assign(size_t(TN) * TN, 0);
        for (int j = 0; j < TN; j++)
            for (int i = 0; i < TN; i++) {
                int gi = i + 1, gj = j + 1;
                if (hole[size_t(gj) * M + gi] || hole[size_t(gj) * M + gi + 1] || hole[size_t(gj + 1) * M + gi] || hole[size_t(gj + 1) * M + gi + 1]) {
                    C.hole[size_t(j) * TN + i] = 1;
                    continue;
                }
                int a = id[size_t(j) * (TN + 1) + i], b = id[size_t(j) * (TN + 1) + i + 1];
                int c = id[size_t(j + 1) * (TN + 1) + i + 1], d = id[size_t(j + 1) * (TN + 1) + i];
                // alternate diagonals for a less directional look; queries use the same rule
                if ((i + j + cx + cy) & 1) m.tri(a, b, c), m.tri(a, c, d);
                else m.tri(a, b, d), m.tri(b, c, d);
            }
        collect(m, C.col_ground);
    }

    // ---- roads
    std::vector<int> es_;
    net.query_rect(rect, es_);
    size_t road_from[M_COUNT];
    for (int k = 0; k < M_COUNT; k++) road_from[k] = C.mats[k].idx.size();
    for (int ei : es_) {
        auto &e = net.edges[ei];
        V2 mid = e.at(e.len * 0.5);
        if (!(mid.x >= rect.mn.x && mid.x < rect.mx.x && mid.y >= rect.mn.y && mid.y < rect.mx.y)) continue;
        geom.emit_edge(ei, o, C.mats);
        C.stats_edges++;
        // bridge piers
        if (e.bridge || e.level > 0) {
            for (double s = std::max(e.trim_a + 12, 12.0); s < e.len - e.trim_b - 12; s += 28) {
                V2 d;
                V2 p = e.at(s, &d);
                double deck = e.z_at(s) - 1.4;
                bool hl = false;
                double g = terrain_ctx(*this, p, ctx, &hl);
                if (deck - g < 2.5) continue;
                // keep piers off roads and railways below
                SurfaceHit below = geom.query(p, ctx.rc, 0);
                (void)below;
                box(C.mats[M_CONCRETE], p, d, 0.7, std::max(1.2, e.half() * 0.6), g - 0.5, deck, o, &C.col_solid);
            }
        }
    }
    {
        std::vector<int> nodes;
        for (int ei : es_) nodes.push_back(net.edges[ei].a), nodes.push_back(net.edges[ei].b);
        std::sort(nodes.begin(), nodes.end());
        nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
        for (int n : nodes) {
            V2 p = net.nodes[n].p;
            if (!(p.x >= rect.mn.x && p.x < rect.mx.x && p.y >= rect.mn.y && p.y < rect.mx.y)) continue;
            geom.emit_junction(n, o, C.mats);
            // traffic signals: one head per arm with traffic arriving, on the approaching driver's right
            if (!net.nodes[n].signals) continue;
            for (auto &A : geom.junctions[n].arms) {
                auto &e = net.edges[A.edge];
                int incoming_dir = A.at_a ? -1 : 1;  // traffic arriving at this end moves b->a when at_a
                if (e.oneway && e.oneway != incoming_dir) continue;
                if (e.prio < 3) continue;
                V2 out = perp(A.D);
                V2 pos = A.L.xy() + out * (A.swl > 0 ? std::min(1.0, A.swl * 0.4) : 1.2);
                Instance I;
                I.p = lv(pos, A.L.z + (A.swl > 0 ? KERB_H : 0), o);
                I.yaw = std::atan2(-A.D.x, A.D.y);
                C.inst[I_SIGNAL].push_back(I);
            }
        }
    }
    for (int k : {M_ASPHALT, M_JUNCTION, M_PAVEMENT, M_KERB, M_CONCRETE}) collect(C.mats[k], C.col_road, road_from[k]);

    // ---- buildings
    auto bit = chunk_buildings.find(ckey(cx, cy));
    if (bit != chunk_buildings.end()) {
        for (int bi : bit->second) {
            auto &b = buildings[bi];
            if (!b.render) continue;
            const Building &root = b.parent >= 0 ? buildings[b.parent] : b;
            double lowest;
            double base = building_base(root, ctx.rc, &lowest);
            if (&root != &b) building_base(b, ctx.rc, &lowest);
            emit_building(b, base, std::min(base - 1.0, lowest - 0.6), o, C);
            C.stats_buildings++;
        }
    }

    // ---- water
    for (auto &wa : ctx.water) {
        auto &ring = wa.first->ring;
        Poly oc = clip_poly(ring.outer, rect);
        if (oc.size() < 3) continue;
        std::vector<Poly> hs;
        for (auto &h : ring.holes) {
            Poly hc = clip_poly(h, rect);
            if (hc.size() >= 3) hs.push_back(hc);
        }
        if (poly_area(oc) < 0) std::reverse(oc.begin(), oc.end());
        for (auto &h : hs)
            if (poly_area(h) > 0) std::reverse(h.begin(), h.end());
        flat_poly(C.mats[M_WATER], oc, hs, wa.second, o, {0.2, 0.3, 0.35}, 1);
    }

    // ---- footpaths and railways (drawn on the shaped ground)
    for (auto &r : region.roads) {
        if (r.kind == RoadKind::Road) continue;
        if (!r.bridge.empty() || !r.tunnel.empty() || r.subclass == "crosswalk") continue;
        bool rail = r.kind == RoadKind::Rail;
        if (rail && (r.cls == "subway" || r.cls == "unknown")) continue;
        Rect bb;
        for (auto p : r.pts) bb.add(p);
        if (!bb.overlaps(rect)) continue;
        for (auto &piece : clip_line(r.pts, rect)) {
            double L = polyline_length(piece);
            int n = std::max(1, int(std::ceil(L / 2.0)));
            double w = rail ? (r.cls == "tram" ? 1.8 : 3.4) : r.cls == "pedestrian" ? 4.0 : r.cls == "path" ? 1.4 : 2.0;
            std::vector<V3> lft, rgt;
            std::vector<V2> ul, ur;
            std::vector<bool> ok;
            for (int k = 0; k <= n; k++) {
                double s = L * k / n;
                V2 d;
                V2 p = polyline_at(piece, s, &d);
                if (k == n) {
                    V2 q = polyline_at(piece, std::max(0.0, s - 0.1));
                    d = norm(p - q);
                }
                int kd = 0;
                bool hl = false;
                double z = terrain_ctx(*this, p, ctx, &hl, &kd);
                bool on_hard = kd != 0;
                if (rail && r.cls == "tram" && on_hard) {
                    SurfaceHit hh = geom.query(p, ctx.rc, 0);
                    z = hh.z - 0.02, on_hard = false;
                }
                ok.push_back(!on_hard);
                V2 nl = perp(d) * (w * 0.5);
                lft.push_back(lv(p + nl, z + 0.06, o)), rgt.push_back(lv(p - nl, z + 0.06, o));
                ul.push_back({0, s}), ur.push_back({w, s});
            }
            // mostly on pavements already (mapped sidewalks): skip whole piece
            int good = 0;
            for (bool b2 : ok) good += b2;
            if (!rail && good < int(ok.size()) * 0.5) continue;
            Mesh &m = C.mats[rail ? M_RAIL : M_PATH];
            V2 uv2{rail ? (r.cls == "tram" ? 2.0 : 1.0) : (r.surface == "unpaved" || r.surface == "dirt" || r.surface == "gravel" || r.cls == "path" ? 0.0 : 1.0), w};
            for (size_t k = 0; k + 1 < lft.size(); k++) {
                if (!ok[k] || !ok[k + 1]) continue;
                add_quad(m, lft[k], rgt[k], rgt[k + 1], lft[k + 1], ul[k], ur[k], ur[k + 1], ul[k + 1], uv2);
            }
        }
    }

    // ---- instances
    auto inside = [&](V2 p) { return p.x >= rect.mn.x && p.x < rect.mx.x && p.y >= rect.mn.y && p.y < rect.mx.y; };
    std::vector<const Building *> nearb;
    for (auto &b : buildings)
        if (b.box.overlaps(rect)) nearb.push_back(&b);
    auto in_building = [&](V2 p) {
        for (auto *b : nearb) {
            if (!b->box.contains(p)) continue;
            for (auto &ring : region.buildings[b->src].rings)
                if (point_in_poly(ring.outer, p)) return true;
        }
        return false;
    };
    std::vector<const std::vector<V2> *> near_paths;
    for (auto &r : region.roads) {
        if (r.kind != RoadKind::Path) continue;
        Rect bb;
        for (auto &q : r.pts) bb.add(q);
        if (bb.overlaps(rect.grown(3))) near_paths.push_back(&r.pts);
    }
    auto place_tree = [&](V2 p, uint64_t h, double sc) {
        int kd = 0;
        bool hl = false;
        double z = terrain_ctx(*this, p, ctx, &hl, &kd);
        if (kd || hl) return;
        Instance I;
        I.p = lv(p, z, o);
        I.yaw = hash01(h) * 2 * PI;
        I.scale = sc * (0.8 + 0.45 * hash01(h * 31 + 7));
        I.variant = int(hash32(h * 13) % 4);
        C.inst[I_TREE].push_back(I);
    };
    for (auto &pt : region.points) {
        if (!inside(pt.p)) continue;
        uint64_t h = hash32(uint64_t(pt.p.x * 100) * 7919 + uint64_t(pt.p.y * 100));
        if (pt.kind == "tree") place_tree(pt.p, h, 1.0);
        else if (pt.kind == "street_lamp") {
            RoadHit rh = net.nearest(pt.p, 25);
            Instance I;
            SurfaceHit sh = geom.query(pt.p, ctx.rc, 0);
            double z = sh.kind ? sh.z : terrain_ctx(*this, pt.p, ctx, nullptr);
            I.p = lv(pt.p, z, o);
            if (rh.edge >= 0) {
                V2 q = net.edges[rh.edge].at(rh.s);
                V2 f = norm(q - pt.p);
                I.yaw = std::atan2(-f.x, f.y);
            }
            C.inst[I_LAMP].push_back(I);
        } else if (pt.kind == "bench") {
            if (in_building(pt.p)) continue;
            Instance I;
            SurfaceHit sh = geom.query(pt.p, ctx.rc, 0);
            I.p = lv(pt.p, sh.kind ? sh.z : terrain_ctx(*this, pt.p, ctx, nullptr), o);
            I.yaw = hash01(h) * 2 * PI;
            C.inst[I_BENCH].push_back(I);
        }
    }
    for (auto &l : region.lines) {
        if (l.kind != "tree_row") continue;
        for (auto &piece : clip_line(l.pts, rect)) {
            double L = polyline_length(piece);
            for (double s = 3; s < L; s += 8) {
                V2 p = polyline_at(piece, s);
                place_tree(p, hash32(uint64_t(p.x * 10) * 31337 + uint64_t(p.y * 10)), 0.9);
            }
        }
    }
    for (auto *a : ctx.areas) {
        double step = 0, dens = 0;
        switch (a->kind) {
            case AreaKind::Forest: step = 7.0, dens = 0.85; break;
            case AreaKind::Park: step = 12.0, dens = 0.5; break;
            case AreaKind::Cemetery: step = 10.0, dens = 0.5; break;
            case AreaKind::Grass: step = 15.0, dens = 0.3; break;
            case AreaKind::Residential: step = 13.0, dens = 0.32; break;  // Minsk courtyards are full of trees
            default: break;
        }
        if (step <= 0) continue;
        Rect bb;
        for (auto &p : a->ring.outer) bb.add(p);
        double x0 = std::floor(std::max(bb.mn.x, rect.mn.x) / step) * step, y0 = std::floor(std::max(bb.mn.y, rect.mn.y) / step) * step;
        for (double y = y0; y < std::min(bb.mx.y, rect.mx.y); y += step)
            for (double x = x0; x < std::min(bb.mx.x, rect.mx.x); x += step) {
                uint64_t h = hash32((uint64_t(int64_t(x) + 100000) << 20) ^ uint64_t(int64_t(y) + 100000));
                if (hash01(h) > dens) continue;
                V2 p{x + (hash01(h * 3) - 0.5) * step * 0.8, y + (hash01(h * 5) - 0.5) * step * 0.8};
                if (!inside(p) || !point_in_poly(a->ring.outer, p)) continue;
                bool in_hole = false;
                for (auto &hh : a->ring.holes)
                    if (point_in_poly(hh, p)) in_hole = true;
                if (in_hole || in_building(p)) continue;
                // keep clear of footpaths
                bool on_path = false;
                for (auto *pl : near_paths)
                    for (size_t k = 0; k + 1 < pl->size() && !on_path; k++)
                        if (seg_dist(p, (*pl)[k], (*pl)[k + 1]) < 2.5) on_path = true;
                if (on_path) continue;
                place_tree(p, h, a->kind == AreaKind::Forest ? 1.15 : 1.0);
            }
    }
    // street trees: a row behind the pavement of urban streets
    for (int ei : es_) {
        auto &e = net.edges[ei];
        if (e.prio < 4 || e.prio > 8 || e.bridge || e.tunnel || e.level) continue;
        for (double side : {1.0, -1.0}) {
            double sw = e.sw(side);
            if (sw <= 0) continue;
            for (double s = e.trim_a + 6; s < e.len - e.trim_b - 6; s += 10) {
                V2 d;
                V2 c = e.at(s, &d);
                V2 p = c + perp(d) * (side * (e.half() + sw + 1.6));
                if (!inside(p)) continue;
                uint64_t h = hash32(uint64_t(int64_t(p.x * 7) + 1000000) * 2654435761ULL ^ uint64_t(int64_t(p.y * 7) + 1000000));
                if (hash01(h) > 0.55 || in_building(p)) continue;
                place_tree(p, h, 0.85);
            }
        }
    }
    return out;
}

double World::grid_height(const ChunkData &c, V2 p) {
    double fx = (p.x - (c.origin.x - CHUNK / 2)) / TCELL, fy = (p.y - (c.origin.y - CHUNK / 2)) / TCELL;
    int i = int(std::floor(fx)), j = int(std::floor(fy));
    if (i < 0 || j < 0 || i >= TN || j >= TN) return NAN;
    if (!c.hole.empty() && c.hole[size_t(j) * TN + i]) return NAN;
    double u = fx - i, v = fy - j;
    auto H = [&](int a, int b) { return double(c.ground[size_t(b) * (TN + 1) + a]); };
    double a = H(i, j), b = H(i + 1, j), cc = H(i + 1, j + 1), d = H(i, j + 1);
    if ((i + j + c.cx + c.cy) & 1) {
        // diagonal a-c: triangles (a,b,c) and (a,c,d)
        return u >= v ? a + (b - a) * u + (cc - b) * v : a + (cc - d) * u + (d - a) * v;
    }
    // diagonal b-d: triangles (a,b,d) and (b,c,d)
    return u + v <= 1 ? a + (b - a) * u + (d - a) * v : cc + (d - cc) * (1 - u) + (b - cc) * (1 - v);
}

bool World::spawn_point(V2 near, V3 &pos, double &heading) const {
    double best = 1e18;
    int be = -1;
    for (size_t i = 0; i < net.edges.size(); i++) {
        auto &e = net.edges[i];
        if (e.prio < 6 || e.tunnel || e.bridge || e.level || e.len < 80) continue;
        double d = dist(e.at(e.len * 0.5), near);
        if (d < best) best = d, be = int(i);
    }
    if (be < 0) return false;
    auto &e = net.edges[be];
    V2 d;
    double s = e.len * 0.5;
    V2 p = e.at(s, &d);
    if (e.oneway < 0) d = d * -1.0;
    // drive on the right: offset into the right-hand lane
    double off = e.oneway ? 0.0 : -std::min(e.half() * 0.5, 2.5);
    if (e.oneway == 0) p = p + perp(d) * off;
    pos = V3(p, e.z_at(s));
    heading = std::atan2(-d.x, d.y);
    return true;
}

}  // namespace es

namespace es {

void World::far_range(int &x0, int &y0, int &x1, int &y1) const {
    Rect r = region.bounds.grown(1500);
    x0 = int(std::floor(r.mn.x / FAR_CELL)), y0 = int(std::floor(r.mn.y / FAR_CELL));
    x1 = int(std::floor(r.mx.x / FAR_CELL)), y1 = int(std::floor(r.mx.y / FAR_CELL));
}

std::unique_ptr<ChunkData> World::build_far(int fx, int fy) const {
    auto out = std::make_unique<ChunkData>();
    ChunkData &C = *out;
    C.cx = fx, C.cy = fy;
    Rect rect{{fx * FAR_CELL, fy * FAR_CELL}, {(fx + 1) * FAR_CELL, (fy + 1) * FAR_CELL}};
    V2 o = (rect.mn + rect.mx) * 0.5;
    C.origin = o;
    const double S = 32.0;
    const int N = int(FAR_CELL / S);
    // terrain: DEM only, a little below the detailed ground so it never shows through it
    {
        Mesh &m = C.mats[M_TERRAIN];
        std::vector<const RawArea *> areas;
        for (auto &a : region.areas) {
            Rect b;
            for (auto &p : a.ring.outer) b.add(p);
            if (b.overlaps(rect) && (a.kind == AreaKind::Forest || a.kind == AreaKind::Water || a.kind == AreaKind::Park)) areas.push_back(&a);
        }
        std::vector<double> H(size_t(N + 3) * (N + 3));
        for (int j = 0; j < N + 3; j++)
            for (int i = 0; i < N + 3; i++) H[size_t(j) * (N + 3) + i] = region.dem.sample(rect.mn + V2{(i - 1) * S, (j - 1) * S}) - 1.2;
        std::vector<int> id(size_t(N + 1) * (N + 1));
        for (int j = 0; j <= N; j++)
            for (int i = 0; i <= N; i++) {
                int gi = i + 1, gj = j + 1;
                double z = H[size_t(gj) * (N + 3) + gi];
                double dx = (H[size_t(gj) * (N + 3) + gi + 1] - H[size_t(gj) * (N + 3) + gi - 1]) / (2 * S);
                double dy = (H[size_t(gj + 1) * (N + 3) + gi] - H[size_t(gj - 1) * (N + 3) + gi]) / (2 * S);
                V2 p = rect.mn + V2{i * S, j * S};
                int pr;
                AreaKind k = area_at(areas, p, pr);
                float r = 0.55f, g = 0.0f, b = 0.1f, a = 0.35f;
                if (k == AreaKind::Forest) r = 0.25f, g = 0.75f, b = 0, a = 0;
                else if (k == AreaKind::Park) r = 0.8f, g = 0.2f, b = 0, a = 0;
                else if (k == AreaKind::Water) r = 0.1f, g = 0, b = 0.9f, a = 0;
                id[size_t(j) * (N + 1) + i] = m.vert(lv(p, z, o), norm(V3(-dx, -dy, 1)), {p.x, p.y}, {0, 0}, r, g, b, a);
            }
        for (int j = 0; j < N; j++)
            for (int i = 0; i < N; i++) {
                int a = id[size_t(j) * (N + 1) + i], b = id[size_t(j) * (N + 1) + i + 1];
                int c = id[size_t(j + 1) * (N + 1) + i + 1], d = id[size_t(j + 1) * (N + 1) + i];
                m.quad(a, b, c, d);
            }
    }
    // buildings as blocks (walls + flat top) on the DEM
    for (auto &b : buildings) {
        if (!b.render || !rect.contains(b.centroid)) continue;
        if (b.area < 60) continue;
        auto &r = region.buildings[b.src];
        double base = region.dem.sample(b.centroid);
        double top = base + b.height - (b.roof_h > 0.4 * b.height ? b.roof_h * 0.85 : 0.0);
        double bot = b.min_h > 0.3 ? base + b.min_h : base - 4;
        V3 fc = rgb(b.facade) * 0.9, rc = rgb(b.roof);
        float seed = float(hash01(hash_str(r.id.data(), r.id.size())));
        for (auto &ring : r.rings) {
            auto &P = ring.outer;
            double acc = 0;
            for (size_t i = 0; i < P.size(); i++) {
                V2 a = P[i], c = P[(i + 1) % P.size()];
                double l = dist(a, c);
                if (l < 0.5) continue;
                add_quad(C.mats[M_WALL], lv(a, bot, o), lv(c, bot, o), lv(c, top, o), lv(a, top, o), {acc, bot - base}, {acc + l, bot - base}, {acc + l, top - base}, {acc, top - base},
                         {b.floor_h, double(b.style)}, float(fc.x), float(fc.y), float(fc.z), seed);
                acc += l;
            }
            flat_poly(C.mats[M_ROOF], ring.outer, ring.holes, top, o, rc, 0);
        }
    }
    // water surfaces
    auto &wl = water_levels();
    for (size_t i = 0; i < region.areas.size(); i++) {
        auto &a = region.areas[i];
        if (a.kind != AreaKind::Water) continue;
        Poly oc = clip_poly(a.ring.outer, rect);
        if (oc.size() < 3) continue;
        if (poly_area(oc) < 0) std::reverse(oc.begin(), oc.end());
        flat_poly(C.mats[M_WATER], oc, {}, wl[i] - 0.3, o, {0.2, 0.3, 0.35}, 1);
    }
    return out;
}

}  // namespace es
