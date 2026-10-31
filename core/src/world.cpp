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
//
// Every building gets an architectural era from what the data tells (floors, class, materials) and from
// its form: position relative to the city centre, whether it is built wall-to-wall with neighbours,
// courtyards, footprint complexity and elongation. The era drives the facade style, floor height,
// palette, roof form and details (cornices, balconies, entrances, shop fronts, roof plant rooms).

namespace {
const std::vector<uint32_t> P_HIST = {0xE8D9B0, 0xEBD58C, 0xD9B26A, 0xE2B4A6, 0xC9D7B3, 0xBFD2DC, 0xEFEDE6, 0xC98D6E, 0xE6C9A0, 0xD7C3D9};
const std::vector<uint32_t> P_STALIN = {0xE6D8BE, 0xDCCB9F, 0xEFE6D2, 0xD8C8A8, 0xE9D7A9, 0xC9B79A, 0xD6D2C8, 0xE3CFA1, 0xD9BFA0};
const std::vector<uint32_t> P_KH_BRICK = {0xD8D5CE, 0xCFCAC0, 0xE2DFD8, 0xC8C4BC, 0xA65B42, 0xB06A50};
const std::vector<uint32_t> P_KH_PANEL = {0xC9C6BE, 0xD3D0C8, 0xBDBBB5, 0xCFC7B8};
const std::vector<uint32_t> P_PANEL = {0xD8D4CC, 0xCFCBC2, 0xE3DED3, 0xC9CED3, 0xDCD3C4, 0xE8E4DA, 0xBFC4C6};
const std::vector<uint32_t> P_MODERN = {0xE9E6E0, 0xD7D9DC, 0xEDE3D3, 0xC4C9CE, 0xE3D3C0, 0xF0EEEA};
const std::vector<uint32_t> P_GLASS = {0x7C8C99, 0x6F8296, 0x8A9AA3, 0x5E7584};
const std::vector<uint32_t> P_INDUS = {0xA9A9A4, 0x8F9396, 0xB5B0A5, 0x9AA39A};
const std::vector<uint32_t> P_HOUSE = {0xD9C7A6, 0xC4B59A, 0xE6E0D0, 0xB9A58A, 0xC7D3C4, 0xE0CFA8};
const std::vector<uint32_t> P_SHED = {0x8D8D88, 0x9A968E, 0x7E8A84};
const std::vector<uint32_t> P_PUBLIC = {0xE9DCC0, 0xD9D2C4, 0xC98D6E, 0xE7E2D6};
const std::vector<uint32_t> P_CHURCH = {0xF2F0EA, 0xEDE4C8, 0xF0E0D2};
const std::vector<uint32_t> R_FLATC = {0x4A4A4A, 0x555555, 0x5E5E5E, 0x505452};
const std::vector<uint32_t> R_METAL = {0x5E6E5E, 0x6A7480, 0x4F5A55, 0x7A3A2A, 0x8C4B32, 0x55585C};
const std::vector<uint32_t> R_TILE = {0x8B3A2B, 0x7A3B2E, 0x9A4A32, 0x6E6E6E};

uint32_t soften(uint32_t c) {
    // OSM colour tags are swatches ("yellow", "#FFFF00"): pull them towards a plaster tone
    double r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255, l = (r + g + b) / 3;
    auto f = [&](double v) { return uint32_t(std::clamp(l + (v - l) * 0.55 + 18, 0.0, 255.0)); };
    return (f(r) << 16) | (f(g) << 8) | f(b) | 0x1000000u;
}
uint32_t lighten(uint32_t c, double k) {
    double r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
    auto f = [&](double v) { return uint32_t(std::clamp(v + (255 - v) * k, 0.0, 255.0)); };
    return (f(r) << 16) | (f(g) << 8) | f(b) | 0x1000000u;
}

// oriented bounding box: length and width of the min-area rectangle among edge directions
void obb(const Poly &P, double &L, double &W, V2 &axis) {
    double best = 1e18;
    L = W = 0, axis = {1, 0};
    for (size_t i = 0; i < P.size(); i++) {
        V2 d = norm(P[(i + 1) % P.size()] - P[i]);
        if (len(d) < 0.5) continue;
        V2 n = perp(d);
        double a0 = 1e18, a1 = -1e18, b0 = 1e18, b1 = -1e18;
        for (auto &p : P) a0 = std::min(a0, dot(p, d)), a1 = std::max(a1, dot(p, d)), b0 = std::min(b0, dot(p, n)), b1 = std::max(b1, dot(p, n));
        double ar = (a1 - a0) * (b1 - b0);
        if (ar < best) {
            best = ar;
            L = a1 - a0, W = b1 - b0, axis = d;
            if (W > L) std::swap(L, W), axis = n;
        }
    }
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
            b.verts += int(ring.outer.size());
            b.holes += int(ring.holes.size());
            if (a > best) best = a, b.centroid = poly_centroid(ring.outer);
        }
        if (b.area < 4) continue;
        by_id[r.id] = int(buildings.size());
        buildings.push_back(std::move(b));
    }
    // parts and parents
    std::unordered_map<int, double> covered;
    for (auto &b : buildings) {
        auto &r = RB[b.src];
        if (!r.part) continue;
        auto it = by_id.find(r.parent);
        if (it != by_id.end()) b.parent = it->second, covered[it->second] += b.area;
    }
    for (auto &kv : covered)
        if (kv.second >= 0.75 * buildings[kv.first].area) buildings[kv.first].render = false;

    // wall-to-wall neighbours: a footprint vertex on another building's outline
    {
        const double C = 24.0;
        std::unordered_map<uint64_t, std::vector<std::pair<int, int>>> grid;  // (building, ring)
        auto key = [](int x, int y) { return (uint64_t(uint32_t(x)) << 32) | uint32_t(y); };
        for (size_t bi = 0; bi < buildings.size(); bi++) {
            auto &b = buildings[bi];
            if (RB[b.src].part) continue;
            for (int x = int(std::floor(b.box.mn.x / C)); x <= int(std::floor(b.box.mx.x / C)); x++)
                for (int y = int(std::floor(b.box.mn.y / C)); y <= int(std::floor(b.box.mx.y / C)); y++) grid[key(x, y)].push_back({int(bi), 0});
        }
        for (size_t bi = 0; bi < buildings.size(); bi++) {
            auto &b = buildings[bi];
            if (RB[b.src].part) continue;
            for (auto &ring : RB[b.src].rings)
                for (auto &p : ring.outer) {
                    if (b.attached) break;
                    auto it = grid.find(key(int(std::floor(p.x / C)), int(std::floor(p.y / C))));
                    if (it == grid.end()) continue;
                    for (auto &oc : it->second) {
                        if (oc.first == int(bi) || b.attached) continue;
                        auto &o = buildings[oc.first];
                        if (!o.box.grown(1).contains(p)) continue;
                        for (auto &orr : RB[o.src].rings) {
                            auto &Q = orr.outer;
                            for (size_t k = 0; k < Q.size() && !b.attached; k++)
                                if (seg_dist(p, Q[k], Q[(k + 1) % Q.size()]) < 0.8) b.attached = true;
                        }
                    }
                }
        }
    }

    for (size_t bi = 0; bi < buildings.size(); bi++) {
        auto &b = buildings[bi];
        auto &r = RB[b.src];
        uint64_t h = hash_str(r.id.data(), r.id.size());
        std::string cls = r.cls.empty() ? r.subtype : r.cls;
        double L = 0, Wd = 0;
        V2 ax;
        obb(r.rings[0].outer, L, Wd, ax);
        b.axis = ax;
        double rect = b.area / std::max(1.0, L * Wd);
        double elong = L / std::max(1.0, Wd);
        double d0 = len(b.centroid);
        double cw = 1.0 - smoothstep(1300, 2900, d0);  // the region centre is the city centre
        int F = r.floors;
        if (F <= 0 && r.height > 0) F = std::max(1, int(r.height / 3.2 + 0.3));
        bool known = F > 0;
        auto is = [&](std::initializer_list<const char *> v) {
            for (auto s : v)
                if (cls == s) return true;
            return false;
        };
        int era;
        if (is({"garage", "garages", "shed", "carport", "roof", "kiosk", "toilets", "hut", "container"}) || b.area < 30) era = E_SHED;
        else if (is({"industrial", "warehouse", "service", "transportation", "hangar", "manufacture", "storage_tank", "train_station"})) era = E_INDUSTRIAL;
        else if (is({"house", "detached", "semidetached_house", "bungalow", "cabin", "farm", "terrace"})) era = E_HOUSE;
        else if (is({"church", "cathedral", "chapel", "religious", "mosque", "synagogue", "temple"})) era = E_CHURCH;
        else if (is({"school", "kindergarten", "hospital", "university", "college", "public", "civic", "government", "education", "medical"})) era = cw > 0.55 && b.attached ? E_STALIN : E_PUBLIC;
        else if (r.facade_material == "glass" || (is({"office", "commercial"}) && (F >= 7 || (!known && b.area > 1800)))) era = E_OFFICE;
        else {
            if (!known) {
                // estimate storeys from the footprint and the district
                if (b.area < 120) F = 2;
                else if (cw > 0.5) F = b.area > 700 ? 6 : b.area > 250 ? 4 : 3;
                else F = b.area > 900 ? 9 : b.area > 300 ? 5 : 3;
            }
            double hs = cw * 1.25 + (b.attached ? 0.7 : 0) + (b.holes ? 0.5 : 0) + (rect < 0.78 ? 0.3 : 0) + (b.verts > 9 ? 0.2 : 0) -
                        (F >= 10 ? 1.6 : 0) - (elong > 4.5 && rect > 0.85 ? 0.5 : 0);
            double rnd = hash01(h * 13);
            if (F <= 3 && b.area < 120 && !b.attached && cw < 0.4) era = E_HOUSE;
            else if (F <= 3) era = hs > 0.9 ? E_HISTORIC : rnd < 0.45 ? E_KHRUSH : rnd < 0.75 ? E_STALIN : E_PUBLIC;
            else if (F <= 9 && hs > 1.1) era = F <= 4 && rnd < 0.35 ? E_HISTORIC : E_STALIN;
            else if (F <= 5 && rect > 0.8) era = cw > 0.6 && rnd < 0.55 ? E_STALIN : E_KHRUSH;
            else if (F <= 9) era = cw > 0.55 ? (rnd < 0.6 ? E_STALIN : E_MODERN) : (rnd < 0.75 ? E_PANEL : E_MODERN);
            else era = rnd < 0.3 + 0.4 * cw ? E_MODERN : E_PANEL;
        }
        if (r.facade_material == "brick" && (era == E_PANEL || era == E_KHRUSH)) era = E_KHRUSH, b.variant = 0;
        if (r.facade_material == "panel" || r.facade_material == "concrete") era = era == E_STALIN || era == E_HISTORIC ? era : E_PANEL;
        b.era = era;
        b.variant = int(hash32(h * 7) % 4);
        // style / storey height / palette
        const std::vector<uint32_t> *pal = &P_PANEL;
        switch (era) {
            case E_HISTORIC: b.style = 2, b.floor_h = 3.8, pal = &P_HIST; break;
            case E_STALIN: b.style = 10, b.floor_h = 3.4, pal = &P_STALIN; break;
            case E_KHRUSH:
                b.style = b.variant < 2 ? 1 : 12, b.floor_h = 2.8, pal = b.style == 1 ? &P_KH_BRICK : &P_KH_PANEL;
                if (b.style == 1 && cw > 0.5 && b.variant == 0) b.variant = 1;  // few red-brick ones in the centre
                break;
            case E_PANEL: b.style = 0, b.floor_h = 2.8, pal = &P_PANEL; break;
            case E_MODERN: b.style = 11, b.floor_h = 3.0, pal = &P_MODERN; break;
            case E_OFFICE: b.style = 3, b.floor_h = 3.8, pal = &P_GLASS; break;
            case E_INDUSTRIAL: b.style = 4, b.floor_h = 5.0, pal = &P_INDUS; break;
            case E_HOUSE: b.style = 5, b.floor_h = 3.0, pal = &P_HOUSE; break;
            case E_SHED: b.style = 6, b.floor_h = 3.0, pal = &P_SHED; break;
            case E_PUBLIC: b.style = 13, b.floor_h = 3.6, pal = &P_PUBLIC; break;
            case E_CHURCH: b.style = 14, b.floor_h = 5.0, pal = &P_CHURCH; break;
        }
        if (F <= 0) F = era == E_INDUSTRIAL ? (b.area > 2000 ? 3 : 2) : era == E_SHED ? 1 : era == E_HOUSE ? 2 : era == E_CHURCH ? 3 : era == E_PUBLIC ? 3 : era == E_OFFICE ? 6 : 4;
        if (r.height > 0 && r.floors > 0) b.floor_h = std::clamp((r.height - (r.roof_height > 0 ? r.roof_height : 0)) / r.floors, 2.6, 6.0);
        b.floors = F;
        b.height = r.height > 0 ? r.height : F * b.floor_h + 0.4;
        b.min_h = r.min_height > 0 ? r.min_height : r.min_floor > 0 ? r.min_floor * b.floor_h : 0;
        b.roof_h = r.roof_height;
        b.facade = r.facade_color ? soften(r.facade_color) : pick(*pal, h);
        // red brick khrushchevkas
        if (era == E_KHRUSH && b.style == 1 && b.variant == 0) b.facade = pick({0xA65B42, 0xB06A50, 0x9C5540}, h);
        b.trim = era == E_STALIN || era == E_HISTORIC || era == E_PUBLIC ? lighten(b.facade, hash01(h * 3) < 0.5 ? 0.55 : 0.3) : b.facade;
        // roof form
        std::string rs = r.roof_shape;
        b.roof_shape = rs;
        bool data_pitched = rs == "gabled" || rs == "hipped" || rs == "half-hipped" || rs == "skillion" || rs == "gambrel" || rs == "mansard";
        if (rs == "dome" || rs == "onion" || rs == "pyramidal" || rs == "cone") b.roof_kind = RK_APEX;
        else if (data_pitched) b.roof_kind = RK_HIP;
        else if (rs == "flat") b.roof_kind = RK_FLAT;
        else {
            double rr = hash01(h * 17);
            switch (era) {
                case E_HISTORIC: b.roof_kind = RK_HIP; break;
                case E_STALIN: b.roof_kind = rr < 0.75 ? RK_HIP : RK_FLAT; break;
                case E_KHRUSH: b.roof_kind = rr < 0.35 ? RK_HIP : RK_FLAT; break;
                case E_HOUSE: b.roof_kind = RK_HIP; break;
                case E_CHURCH: b.roof_kind = RK_HIP; break;
                case E_PUBLIC: b.roof_kind = cw > 0.5 && rr < 0.5 ? RK_HIP : RK_FLAT; break;
                default: b.roof_kind = RK_FLAT;
            }
        }
        if (b.roof_kind == RK_HIP || b.roof_kind == RK_APEX) {
            const std::vector<uint32_t> &rp = era == E_HOUSE || era == E_HISTORIC ? (hash01(h * 23) < 0.5 ? R_TILE : R_METAL) : R_METAL;
            b.roof = r.roof_color ? r.roof_color : pick(rp, h >> 7);
        } else
            b.roof = r.roof_color ? r.roof_color : pick(R_FLATC, h >> 7);
        {
            // data colours are swatches; real roofs are weathered: darken and desaturate a little
            uint32_t c = b.roof;
            double rr = ((c >> 16) & 255), gg = ((c >> 8) & 255), bb = (c & 255), l = (rr + gg + bb) / 3;
            double sat = r.roof_color ? 0.45 : 0.75, dk = r.roof_color ? 0.78 : 0.88;
            auto f = [&](double v) { return uint32_t(std::clamp((l + (v - l) * sat) * dk, 0.0, 255.0)); };
            b.roof = (f(rr) << 16) | (f(gg) << 8) | f(bb) | 0x1000000u;
        }
        b.cornice = (era == E_STALIN || era == E_HISTORIC || (era == E_PUBLIC && cw > 0.4)) && b.height > 6;
        b.balconies = (era == E_PANEL || era == E_KHRUSH || era == E_MODERN || (era == E_STALIN && hash01(h * 29) < 0.5)) && F >= 3;
        b.entrances = (era == E_PANEL || era == E_KHRUSH || era == E_MODERN || era == E_STALIN) && L > 18 && F >= 3;
        b.roof_boxes = (era == E_PANEL || era == E_MODERN || era == E_OFFICE) && F >= 9;
        b.glazed = era == E_MODERN ? true : hash01(h * 7) < 0.55;
    }
    // parts inherit their parent's architecture so a complex building stays one design
    for (auto &b : buildings)
        if (b.parent >= 0) {
            auto &p = buildings[b.parent];
            b.era = p.era, b.style = p.style, b.facade = RB[b.src].facade_color ? b.facade : p.facade, b.trim = p.trim, b.variant = p.variant;
            if (!RB[b.src].floors && !(RB[b.src].height > 0)) b.floor_h = p.floor_h;
        }
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
    // under the carriageway the ground is hidden; on the margin of a road without pavement it is the
    // shoulder, flush with the road (the cover pass below keeps it under the asphalt edge)
    if (h.kind == 1) return h.bridge ? std::min(G, h.z - 1.7) : h.z - (h.margin ? 0.05 : 0.3);
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

double World::ground_at(V2 p, double zmax) const {
    Ctx c;
    make_ctx(*this, Rect{p - V2{45, 45}, p + V2{45, 45}}, c);
    SurfaceHit h = geom.query(p, c.rc, 0, zmax);
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

namespace {

// window bay per facade style; the facade shader uses the same grid (balconies and doors align to it)
double style_bay(int style) {
    switch (style) {
        case 1: return 2.7;
        case 2: return 3.3;
        case 10: return 3.2;
        case 11: return 3.0;
        case 12: return 3.2;
        case 13: return 3.4;
        case 5: return 3.2;
        default: return 3.0;
    }
}

// coloured box (any yaw) for building details; CCW footprint, outward faces
void cbox(Mesh &m, V2 c, V2 u, double hu, double hv, double z0, double z1, V2 o, V2 uv2, V3 col, float seed, bool under = false) {
    V2 v = perp(u);
    V2 p[4] = {c - u * hu - v * hv, c + u * hu - v * hv, c + u * hu + v * hv, c - u * hu + v * hv};
    double acc = 0;
    for (int i = 0; i < 4; i++) {
        V2 a = p[i], b = p[(i + 1) % 4];
        double l = dist(a, b);
        add_quad(m, lv(a, z0, o), lv(b, z0, o), lv(b, z1, o), lv(a, z1, o), {acc, 0}, {acc + l, 0}, {acc + l, z1 - z0}, {acc, z1 - z0}, uv2, float(col.x), float(col.y), float(col.z), seed);
        acc += l;
    }
    add_quad(m, lv(p[0], z1, o), lv(p[1], z1, o), lv(p[2], z1, o), lv(p[3], z1, o), {0, 0}, {1, 0}, {1, 1}, {0, 1}, uv2, float(col.x), float(col.y), float(col.z), seed);
    if (under) add_quad(m, lv(p[0], z0, o), lv(p[3], z0, o), lv(p[2], z0, o), lv(p[1], z0, o), {0, 0}, {1, 0}, {1, 1}, {0, 1}, uv2, float(col.x), float(col.y), float(col.z), seed);
}

Poly clean_ring(const Poly &P) {
    Poly q;
    for (auto &p : P)
        if (q.empty() || dist(q.back(), p) > 0.08) q.push_back(p);
    while (q.size() > 2 && dist(q.front(), q.back()) < 0.08) q.pop_back();
    // drop collinear vertices (keeps offset lines well conditioned)
    bool again = true;
    while (again && q.size() > 3) {
        again = false;
        for (size_t i = 0; i < q.size(); i++) {
            V2 a = q[(i + q.size() - 1) % q.size()], b = q[i], c = q[(i + 1) % q.size()];
            if (std::fabs(cross(norm(b - a), norm(c - b))) < 0.01 && dot(b - a, c - b) > 0) {
                q.erase(q.begin() + i);
                again = true;
                break;
            }
        }
    }
    return q;
}

// offset every edge of a ring to its left by d[i] (left = inside for CCW outer rings and CW holes)
bool inset_ring(const Poly &P, const std::vector<double> &d, Poly &out) {
    size_t n = P.size();
    out.assign(n, {});
    for (size_t i = 0; i < n; i++) {
        size_t ip = (i + n - 1) % n;
        V2 dp = norm(P[i] - P[ip]), dc = norm(P[(i + 1) % n] - P[i]);
        V2 A = P[i] + perp(dp) * d[ip], B = P[i] + perp(dc) * d[i];
        double den = cross(dp, dc);
        if (std::fabs(den) < 0.02) out[i] = (A + B) * 0.5;
        else out[i] = A + dp * (cross(B - A, dc) / den);
        if (dist(out[i], P[i]) > 4.5 * std::max(d[ip], d[i]) + 0.01) return false;  // spike at a very sharp corner
    }
    for (size_t i = 0; i < n; i++) {
        V2 e0 = P[(i + 1) % n] - P[i], e1 = out[(i + 1) % n] - out[i];
        if (dot(e0, e1) < -0.01) return false;  // edge collapsed and flipped
    }
    double a0 = poly_area(P), a1 = poly_area(out);
    if (a0 * a1 <= 0 || std::fabs(a1) < 0.01 * std::fabs(a0)) return false;
    return true;
}

bool rings_cross(const std::vector<Poly> &R) {
    std::vector<std::pair<V2, V2>> S;
    for (auto &r : R)
        for (size_t i = 0; i < r.size(); i++) S.push_back({r[i], r[(i + 1) % r.size()]});
    if (S.size() > 220) return true;
    for (size_t i = 0; i < S.size(); i++)
        for (size_t j = i + 1; j < S.size(); j++) {
            auto &a = S[i], &b = S[j];
            // shared endpoints (adjacent edges) are fine
            if (dist(a.first, b.first) < 1e-6 || dist(a.first, b.second) < 1e-6 || dist(a.second, b.first) < 1e-6 || dist(a.second, b.second) < 1e-6) continue;
            double t, u;
            if (seg_intersect(a.first, a.second, b.first, b.second, t, u) && t > 1e-4 && t < 1 - 1e-4 && u > 1e-4 && u < 1 - 1e-4) return true;
        }
    return false;
}

}  // namespace

bool World::street_facing(V2 mid, V2 out, const RoadGeom::Cache &rc) const {
    for (double t : {4.0, 9.0, 16.0}) {
        SurfaceHit h = geom.query(mid + out * t, rc, 0);
        if (h.kind == 2) return true;
        if (h.kind == 1) {
            if (h.edge >= 0) return net.edges[h.edge].prio >= 4;
            return true;
        }
    }
    return false;
}

void World::emit_building(const Building &b, double base, double foot, V2 o, ChunkData &out, const RoadGeom::Cache &rcache) const {
    auto &r = region.buildings[b.src];
    Mesh &W = out.mats[M_WALL];
    Mesh &Rf = out.mats[M_ROOF];
    V3 fc = rgb(b.facade), rc = rgb(b.roof), tc = rgb(b.trim ? b.trim : b.facade);
    uint64_t hb = hash_str(r.id.data(), r.id.size());
    float seed = float(hash01(hb));
    bool data_h = r.height > 0;
    double top = base + b.height;
    double bottom = b.min_h > 0.3 ? base + b.min_h : foot;  // foundations go below the lowest ground
    auto on_seam = [&](V2 a, V2 c) {
        for (double x : seam_x)
            if (std::fabs(a.x - x) < 1e-6 && std::fabs(c.x - x) < 1e-6) return true;
        for (double y : seam_y)
            if (std::fabs(a.y - y) < 1e-6 && std::fabs(c.y - y) < 1e-6) return true;
        return false;
    };
    std::string rs = b.roof_shape;
    const bool high_part = b.min_h > 0.3;

    // ---------------- roof form
    // 1. apex (pyramid / dome / spire): one ring, no holes, no seams
    bool single = r.rings.size() == 1 && r.rings[0].holes.empty() && r.rings[0].outer.size() >= 3;
    if (single)
        for (size_t i = 0; i < r.rings[0].outer.size(); i++)
            if (on_seam(r.rings[0].outer[i], r.rings[0].outer[(i + 1) % r.rings[0].outer.size()])) single = false;
    bool apex = single && (b.roof_kind == RK_APEX || (b.roof_h > 0.5 * b.height && b.roof_h > 3));
    // 2. gable on simple quads (data says gabled, or some houses)
    bool simple = r.rings.size() == 1 && r.rings[0].holes.empty() && r.rings[0].outer.size() == 4 && single;
    bool gable = !apex && simple && b.roof_kind == RK_HIP &&
                 (rs == "gabled" || rs == "skillion" || rs == "gambrel" || (rs.empty() && b.era == E_HOUSE && hash01(hb * 41) < 0.6));
    // 3. inset (hip / mansard-like) roof for any footprint: rings offset inwards, slopes + a flat top
    std::vector<std::vector<Poly>> inset_src, inset_dst;  // per ring set: [outer, holes...]
    std::vector<std::vector<std::vector<double>>> inset_d;
    double rh = 0, pitch = 0.5;
    bool inset = false;
    if (!apex && !gable && b.roof_kind == RK_HIP) {
        double frac = 0.45, dmax = 7.0, deg = 28;
        switch (b.era) {
            case E_HISTORIC: deg = 40, frac = 0.42, dmax = 5.5; break;
            case E_STALIN: deg = 27, frac = 0.45, dmax = 7.5; break;
            case E_KHRUSH: deg = 17, frac = 0.49, dmax = 7.0; break;
            case E_HOUSE: deg = 33, frac = 0.5, dmax = 8.0; break;
            case E_CHURCH: deg = 42, frac = 0.5, dmax = 9.0; break;
            case E_PUBLIC: deg = 24, frac = 0.45, dmax = 7.0; break;
            default: break;
        }
        if (rs == "mansard") deg = 60, frac = 0.25;
        pitch = std::tan(deg * PI / 180);
        double L, Wd;
        V2 ax;
        inset = true;
        for (auto &ring : r.rings) {
            std::vector<Poly> src = {clean_ring(ring.outer)};
            for (auto &h : ring.holes) src.push_back(clean_ring(h));
            bool okring = true;
            for (auto &p : src)
                if (p.size() < 3) okring = false;
            if (!okring) {
                inset = false;
                break;
            }
            obb(src[0], L, Wd, ax);
            double d0 = std::min(Wd * frac, dmax);
            if (b.roof_h > 0.3) d0 = std::min(d0, b.roof_h / pitch);
            bool done = false;
            for (int attempt = 0; attempt < 5 && !done && d0 > 0.8; attempt++, d0 *= 0.7) {
                std::vector<Poly> dst(src.size());
                std::vector<std::vector<double>> dd(src.size());
                bool ok = true;
                for (size_t k = 0; k < src.size() && ok; k++) {
                    auto &P = src[k];
                    dd[k].resize(P.size());
                    for (size_t i = 0; i < P.size(); i++) dd[k][i] = on_seam(P[i], P[(i + 1) % P.size()]) ? 0.0 : d0;
                    ok = inset_ring(P, dd[k], dst[k]);
                }
                if (ok && rings_cross(dst)) ok = false;
                if (ok)
                    for (size_t k = 1; k < dst.size() && ok; k++)
                        if (!point_in_poly(dst[0], dst[k][0])) ok = false;
                if (ok) {
                    done = true;
                    rh = std::max(rh, d0 * pitch);
                    inset_src.push_back(src), inset_dst.push_back(dst), inset_d.push_back(dd);
                }
            }
            if (!done) {
                inset = false;
                break;
            }
        }
        if (!inset) rh = 0;
    }
    double eave;
    if (apex) {
        double rad = std::sqrt(b.area / PI);
        rh = b.roof_h > 0.3 ? b.roof_h : std::clamp(rad * (rs == "dome" || rs == "onion" ? 0.9 : 0.6), 1.5, 8.0);
        rh = std::min(rh, b.height * 0.97);
        if (!data_h && b.roof_h <= 0.3) top += rh;
        eave = top - rh;
    } else if (gable) {
        auto &q = r.rings[0].outer;
        double s1 = std::min(dist(q[0], q[1]), dist(q[1], q[2]));
        rh = b.roof_h > 0.3 ? b.roof_h : std::clamp(s1 * 0.35, 1.5, 5.0);
        rh = std::min(rh, std::max(1.0, b.height * 0.6));
        if (!data_h) top += rh;
        eave = top - rh;
    } else if (inset) {
        if (data_h) {
            if (rh > b.height * 0.45) {
                // keep the walls; flatten the roof a little
                double k = b.height * 0.45 / rh;
                rh *= k;
            }
            eave = top - rh;
        } else
            eave = top, top += rh;
    } else
        eave = top;
    bool flat = !apex && !gable && !inset;
    double rim = flat ? (b.height > 6 ? (b.era == E_STALIN || b.era == E_HISTORIC ? 0.8 : 0.45) : 0.2) : 0.0;  // parapet / attic
    double wall_top = flat ? top + rim : eave;
    int nfl = std::max(1, int((eave - base - 0.2) / b.floor_h + 0.2));  // storeys that have windows
    double uv2y = double(b.style) + 32.0 * nfl;
    bool wants_street = !high_part && (b.entrances || b.era == E_HISTORIC || b.era == E_STALIN || b.era == E_MODERN || b.era == E_PANEL || b.era == E_PUBLIC);

    // ---------------- walls
    size_t wall_start = W.idx.size();
    struct Seg {
        V2 a, c;
        double u0;
        bool street;
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
                V2 dir = (c - a) / l, outv = perp(dir) * -1.0;
                bool street = false;
                if (wants_street && lp == &ring.outer && l > 6) street = street_facing((a + c) * 0.5, outv, rcache);
                bool shop = street && (b.era == E_HISTORIC || b.era == E_STALIN || b.era == E_MODERN || (b.era == E_PANEL && hash01(hb * 53 + i) < 0.3)) &&
                            hash01(hb * 31 + i) < (b.era == E_HISTORIC ? 0.85 : 0.6) && nfl >= 2;
                V2 uv2{b.floor_h + (shop ? 100.0 : 0.0), uv2y};
                V3 n = V3(outv, 0);
                int i0 = W.vert(lv(a, bottom, o), n, {acc, bottom - base}, uv2, float(fc.x), float(fc.y), float(fc.z), seed);
                int i1 = W.vert(lv(c, bottom, o), n, {acc + l, bottom - base}, uv2, float(fc.x), float(fc.y), float(fc.z), seed);
                int i2 = W.vert(lv(c, wall_top, o), n, {acc + l, wall_top - base}, uv2, float(fc.x), float(fc.y), float(fc.z), seed);
                int i3 = W.vert(lv(a, wall_top, o), n, {acc, wall_top - base}, uv2, float(fc.x), float(fc.y), float(fc.z), seed);
                W.quad(i0, i1, i2, i3);
                if (rim > 0) {
                    // parapet: 25 cm thick wall around the flat roof (top + inner face; the outer face is the wall)
                    V2 in = perp(dir) * 0.25;
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
                if (lp == &ring.outer) outer_walls.push_back({a, c, acc, street});
                acc += l;
            }
        }
    }
    collect(W, out.col_solid, wall_start);

    // ---------------- cornice and belt course (old town and Stalinist blocks)
    if (b.cornice && !high_part) {
        double zc = flat ? top + rim : eave;
        V2 uv2{b.floor_h, 15.0};
        for (auto &sg : outer_walls) {
            double l = dist(sg.a, sg.c);
            if (l < 1.0) continue;
            V2 dir = (sg.c - sg.a) / l, ov = perp(dir) * -1.0;
            double ext = 0.45;  // overlap at corners
            V2 a = sg.a - dir * ext, c = sg.c + dir * ext;
            double dp = b.era == E_STALIN ? 0.55 : 0.45, hh = b.era == E_STALIN ? 0.75 : 0.6;
            // sloped soffit, front band, top
            add_quad(W, lv(a, zc - hh, o), lv(c, zc - hh, o), lv(c + ov * dp, zc - hh * 0.35, o), lv(a + ov * dp, zc - hh * 0.35, o), {0, 0}, {l, 0}, {l, 1}, {0, 1}, uv2, float(tc.x), float(tc.y), float(tc.z), seed);
            add_quad(W, lv(a + ov * dp, zc - hh * 0.35, o), lv(c + ov * dp, zc - hh * 0.35, o), lv(c + ov * dp, zc, o), lv(a + ov * dp, zc, o), {0, 0}, {l, 0}, {l, 1}, {0, 1}, uv2, float(tc.x), float(tc.y), float(tc.z), seed);
            add_quad(W, lv(a + ov * dp, zc, o), lv(c + ov * dp, zc, o), lv(c, zc, o), lv(a, zc, o), {0, 0}, {l, 0}, {l, 1}, {0, 1}, uv2, float(tc.x * 0.8), float(tc.y * 0.8), float(tc.z * 0.8), seed);
            // belt course over the ground floor
            if (nfl >= 3 && l > 2) {
                double zb = base + b.floor_h, bd = 0.18;
                add_quad(W, lv(a + ov * bd, zb - 0.22, o), lv(c + ov * bd, zb - 0.22, o), lv(c + ov * bd, zb, o), lv(a + ov * bd, zb, o), {0, 0}, {l, 0}, {l, 1}, {0, 1}, uv2, float(tc.x), float(tc.y), float(tc.z), seed);
                add_quad(W, lv(a + ov * bd, zb, o), lv(c + ov * bd, zb, o), lv(c, zb, o), lv(a, zb, o), {0, 0}, {l, 0}, {l, 1}, {0, 1}, uv2, float(tc.x), float(tc.y), float(tc.z), seed);
                add_quad(W, lv(a, zb - 0.22, o), lv(c, zb - 0.22, o), lv(c + ov * bd, zb - 0.22, o), lv(a + ov * bd, zb - 0.22, o), {0, 0}, {l, 0}, {l, 1}, {0, 1}, uv2, float(tc.x * 0.7), float(tc.y * 0.7), float(tc.z * 0.7), seed);
            }
        }
    }

    // ---------------- balconies: slab + parapet boxes in some window bays (not on street walls of old buildings)
    if (b.balconies && !high_part && nfl >= 3) {
        double bay = style_bay(b.style);
        int pattern = int(hash32(hb * 11) % 3);
        bool glazed = b.glazed;
        for (auto &sg : outer_walls) {
            double l = dist(sg.a, sg.c);
            if (l < 9) continue;
            if (sg.street && b.era == E_STALIN) continue;
            V2 dir = (sg.c - sg.a) / l, outv = perp(dir) * -1.0;
            int k0 = int(std::ceil(sg.u0 / bay)), k1 = int(std::floor((sg.u0 + l) / bay)) - 1;
            for (int k = k0; k <= k1; k++) {
                if (b.era == E_STALIN ? (k + pattern) % 4 != 0 : (k + pattern) % 3 == 0) continue;
                double uc = (k + 0.5) * bay - sg.u0;
                if (uc < 1.6 || uc > l - 1.6) continue;
                V2 m = sg.a + dir * uc;
                for (int f = 1; f < nfl; f++) {
                    double z0 = base + f * b.floor_h;
                    double hw = bay * 0.46, dp = b.era == E_STALIN ? 0.8 : 1.05, ph = glazed ? b.floor_h * 0.92 : 1.05;
                    V2 p0 = m - dir * hw, p1 = m + dir * hw, q0 = p0 + outv * dp, q1 = p1 + outv * dp;
                    V3 bc = b.era == E_STALIN ? tc : fc * 0.93;
                    V2 uv2{b.floor_h, glazed ? 8.0 : 7.0};
                    add_quad(W, lv(q0, z0 - 0.15, o), lv(q1, z0 - 0.15, o), lv(q1, z0 + ph, o), lv(q0, z0 + ph, o), {0, 0}, {hw * 2, 0}, {hw * 2, ph + 0.15}, {0, ph + 0.15}, uv2, float(bc.x), float(bc.y), float(bc.z), seed);
                    add_quad(W, lv(p0, z0 - 0.15, o), lv(q0, z0 - 0.15, o), lv(q0, z0 + ph, o), lv(p0, z0 + ph, o), {0, 0}, {dp, 0}, {dp, ph + 0.15}, {0, ph + 0.15}, uv2, float(bc.x), float(bc.y), float(bc.z), seed);
                    add_quad(W, lv(q1, z0 - 0.15, o), lv(p1, z0 - 0.15, o), lv(p1, z0 + ph, o), lv(q1, z0 + ph, o), {0, 0}, {dp, 0}, {dp, ph + 0.15}, {0, ph + 0.15}, uv2, float(bc.x), float(bc.y), float(bc.z), seed);
                    add_quad(W, lv(p0, z0 - 0.15, o), lv(p1, z0 - 0.15, o), lv(q1, z0 - 0.15, o), lv(q0, z0 - 0.15, o), {0, 0}, {1, 0}, {1, 1}, {0, 1}, {b.floor_h, 9.0}, float(bc.x), float(bc.y), float(bc.z), seed);
                    if (glazed || f == nfl - 1)
                        add_quad(W, lv(q0, z0 + ph, o), lv(q1, z0 + ph, o), lv(p1, z0 + ph, o), lv(p0, z0 + ph, o), {0, 0}, {1, 0}, {1, 1}, {0, 1}, {b.floor_h, 9.0}, float(bc.x), float(bc.y), float(bc.z), seed);
                }
            }
        }
    }

    // ---------------- entrances: door, canopy, steps on courtyard walls; street doors for shops are in the shader
    if (b.entrances && !high_part) {
        double bay = style_bay(b.style);
        std::vector<const Seg *> yard;
        for (auto &sg : outer_walls)
            if (!sg.street && dist(sg.a, sg.c) > 12) yard.push_back(&sg);
        if (yard.empty()) {
            // no courtyard side found: use the longest wall
            const Seg *best = nullptr;
            for (auto &sg : outer_walls)
                if (!best || dist(sg.a, sg.c) > dist(best->a, best->c)) best = &sg;
            if (best && dist(best->a, best->c) > 12) yard.push_back(best);
        }
        // only the longest courtyard wall (one row of entrances, like real blocks)
        std::sort(yard.begin(), yard.end(), [](const Seg *x, const Seg *y) { return dist(x->a, x->c) > dist(y->a, y->c); });
        if (yard.size() > 1) yard.resize(b.era == E_STALIN && hash01(hb * 61) < 0.4 ? 2 : 1);
        V3 dc = rgb(pick({0x5A3A28, 0x3E4A3C, 0x6B4A2E, 0x2F3A45, 0x7A6A55}, hb * 3));
        V3 cc = V3(0.62, 0.61, 0.58);
        for (auto *sg : yard) {
            double l = dist(sg->a, sg->c);
            V2 dir = (sg->c - sg->a) / l, outv = perp(dir) * -1.0;
            int n = std::max(1, int(l / 22.0));
            double sp = l / n;
            for (int k = 0; k < n; k++) {
                // centre on a window bay of the facade grid
                double want = sp * (k + 0.5) + sg->u0;
                double uc = (std::floor(want / bay) + 0.5) * bay - sg->u0;
                if (uc < 2 || uc > l - 2) continue;
                V2 m = sg->a + dir * uc;
                double hw = 0.75, dh = 2.45;
                V2 p0 = m - dir * hw + outv * 0.04, p1 = m + dir * hw + outv * 0.04;
                add_quad(W, lv(p0, base, o), lv(p1, base, o), lv(p1, base + dh, o), lv(p0, base + dh, o), {0, 0}, {hw * 2, 0}, {hw * 2, dh}, {0, dh}, {b.floor_h, 16.0}, float(dc.x), float(dc.y), float(dc.z), seed);
                cbox(W, m + outv * 0.65, dir, 1.15, 0.65, base + 2.6, base + 2.75, o, {b.floor_h, 15.0}, cc, seed, true);
                cbox(W, m + outv * 0.55, dir, 1.0, 0.55, base - 0.4, base + 0.12, o, {b.floor_h, 15.0}, cc * 0.9, seed);
                // a bench by the door (the classic "лавочка у подъезда")
                if (hash01(hb * 13 + k) < 0.7) {
                    Instance I;
                    double sd = hash01(hb * 17 + k) < 0.5 ? -1.0 : 1.0;
                    I.p = lv(m + dir * (2.3 * sd) + outv * 1.3, base, o);
                    I.yaw = std::atan2(-outv.x, outv.y);
                    out.inst[I_BENCH].push_back(I);
                }
            }
        }
    }

    // ---------------- roofs
    if (apex) {
        const Poly &P = r.rings[0].outer;
        V2 c = poly_centroid(P);
        if (!point_in_poly(P, c)) {
            flat_poly(Rf, P, {}, eave, o, rc, 0);
            return;
        }
        bool dome = rs == "dome" || rs == "onion";
        V3 A = lv(c, top, o);
        for (size_t i = 0; i < P.size(); i++) {
            V2 a = P[i], q = P[(i + 1) % P.size()];
            if (dist(a, q) < 0.05) continue;
            if (dome) {
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
    if (inset) {
        double ztop = eave + rh;
        for (size_t s = 0; s < inset_src.size(); s++) {
            auto &src = inset_src[s];
            auto &dst = inset_dst[s];
            double dm = 0;
            for (auto &v : inset_d[s])
                for (double x : v) dm = std::max(dm, x);
            double zr = dm > 0 ? eave + rh : ztop;
            for (size_t k = 0; k < src.size(); k++) {
                auto &P = src[k];
                auto &Q = dst[k];
                for (size_t i = 0; i < P.size(); i++) {
                    size_t j = (i + 1) % P.size();
                    V3 a0 = lv(P[i], eave, o), a1 = lv(P[j], eave, o), b1 = lv(Q[j], zr, o), b0 = lv(Q[i], zr, o);
                    if (inset_d[s][k][i] <= 0) {
                        // seam edge: vertical gable piece in facade colour (hidden behind the neighbouring piece)
                        add_quad(W, a0, a1, b1, b0, {0, 0}, {dist(P[i], P[j]), 0}, {dist(P[i], P[j]), rh}, {0, rh}, {b.floor_h, 15.0}, float(fc.x), float(fc.y), float(fc.z), seed);
                        continue;
                    }
                    add_quad(Rf, a0, a1, b1, b0, P[i], P[j], Q[j], Q[i], {0, 0}, float(rc.x), float(rc.y), float(rc.z), 0);
                }
                // eaves soffit line: thin overhang strip below the slope
                (void)k;
            }
            std::vector<Poly> holes(dst.begin() + 1, dst.end());
            flat_poly(Rf, dst[0], holes, zr, o, rc * 0.92, 0);
            // chimneys / dormers on old roofs
            if ((b.era == E_HISTORIC || b.era == E_HOUSE || b.era == E_STALIN) && s == 0) {
                const Poly &T = dst[0];
                int nch = b.era == E_STALIN ? 2 : 1 + int(hash32(hb * 71) % 2);
                for (int k = 0; k < nch && T.size() >= 3; k++) {
                    V2 p = lerp(T[(k * 2) % T.size()], T[(k * 2 + 1) % T.size()], 0.5);
                    V2 c = poly_centroid(T);
                    p = lerp(p, c, 0.5);
                    if (!point_in_poly(T, p)) continue;
                    V3 chc = b.era == E_HOUSE ? V3(0.55, 0.32, 0.25) : fc * 0.85;
                    cbox(W, p, b.axis, 0.35, 0.5, zr - 0.2, zr + 1.1, o, {b.floor_h, 15.0}, chc, seed);
                }
            }
        }
        return;
    }
    if (gable) {
        auto q = r.rings[0].outer;
        if (dist(q[1], q[2]) + dist(q[3], q[0]) > dist(q[0], q[1]) + dist(q[2], q[3])) std::rotate(q.begin(), q.begin() + 1, q.end());
        V2 m12 = (q[1] + q[2]) * 0.5, m30 = (q[3] + q[0]) * 0.5;
        V2 ra = m30, rb = m12;
        if (rs == "half-hipped") {
            double sl = std::min(dist(q[1], q[2]), dist(q[3], q[0]));
            V2 ax = norm(m12 - m30);
            double in = std::min(sl * 0.5, dist(m30, m12) * 0.45) * 0.5;
            ra = m30 + ax * in, rb = m12 - ax * in;
        }
        V3 RA = lv(ra, top, o), RB = lv(rb, top, o);
        V3 Q[4] = {lv(q[0], eave, o), lv(q[1], eave, o), lv(q[2], eave, o), lv(q[3], eave, o)};
        auto tri = [&](Mesh &m, V3 a, V3 bb, V3 c, V3 col, double sty) {
            V3 n = norm(cross(bb - a, c - a));
            V2 ua{a.x + o.x, a.y + o.y}, ub{bb.x + o.x, bb.y + o.y}, uc{c.x + o.x, c.y + o.y};
            if (&m == &W) ua = {0, a.z - base}, ub = {dist(a.xy(), bb.xy()), bb.z - base}, uc = {dist(a.xy(), c.xy()) * 0.5, c.z - base};
            int i0 = m.vert(a, n, ua, {b.floor_h, sty}, float(col.x), float(col.y), float(col.z), seed);
            int i1 = m.vert(bb, n, ub, {b.floor_h, sty}, float(col.x), float(col.y), float(col.z), seed);
            int i2 = m.vert(c, n, uc, {b.floor_h, sty}, float(col.x), float(col.y), float(col.z), seed);
            m.tri(i0, i1, i2);
        };
        tri(Rf, Q[0], Q[1], RB, rc, 0);
        tri(Rf, Q[0], RB, RA, rc, 0);
        tri(Rf, Q[2], Q[3], RA, rc, 0);
        tri(Rf, Q[2], RA, RB, rc, 0);
        bool hip_end = rs == "half-hipped";
        // gable ends carry no storey windows (uv2.y = plain trim style)
        tri(hip_end ? Rf : W, Q[1], Q[2], RB, hip_end ? rc : fc, 15.0);
        tri(hip_end ? Rf : W, Q[3], Q[0], RA, hip_end ? rc : fc, 15.0);
        return;
    }
    // flat roof
    for (auto &ring : r.rings) flat_poly(Rf, ring.outer, ring.holes, top, o, rc, 0);
    if (b.roof_boxes && !r.rings.empty()) {
        // lift machine rooms / stair exits along the long axis
        const Poly &P = r.rings[0].outer;
        double L, Wd;
        V2 ax;
        obb(P, L, Wd, ax);
        V2 c = poly_centroid(P);
        int n = std::clamp(int(L / 28.0), 1, 6);
        V3 bc = fc * 0.9;
        for (int k = 0; k < n; k++) {
            double t = n == 1 ? 0 : (k - (n - 1) * 0.5) * (L / n);
            V2 p = c + ax * t;
            bool ok = true;
            for (double sx : {-1.0, 1.0})
                for (double sy : {-1.0, 1.0})
                    if (!point_in_poly(P, p + ax * (2.6 * sx) + perp(ax) * (2.0 * sy))) ok = false;
            if (!ok) continue;
            cbox(W, p, ax, 2.3, 1.8, top, top + 2.6, o, {3.0, 6.0}, bc, seed);
        }
    }
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
                // squares and car parks are paved: uv2.x = 1 (paving), 0.5 (asphalt)
                double paved = ak == AreaKind::Paved ? 1.0 : ak == AreaKind::Parking ? 0.5 : 0.0;
                id[size_t(j) * (TN + 1) + i] = m.vert(lv(p, z, o), n, {p.x, p.y}, {paved, 0}, r, g, b, a);
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
        geom.emit_edge(ei, o, C.mats, &ctx.rc);
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
                // keep piers off roads, pavements and railways below (and 2 m clear of them)
                bool blocked = false;
                for (double ox : {-2.5, 0.0, 2.5})
                    for (double oy : {-2.5, 0.0, 2.5}) {
                        V2 q = p + d * ox + perp(d) * oy;
                        SurfaceHit below = geom.query(q, ctx.rc, 0, deck - 2.0);
                        if (below.kind != 0) blocked = true;
                    }
                for (auto &rl : region.roads) {
                    if (blocked || rl.kind != RoadKind::Rail) continue;
                    for (size_t k = 0; k + 1 < rl.pts.size() && !blocked; k++)
                        if (seg_dist(p, rl.pts[k], rl.pts[k + 1]) < 4.0) blocked = true;
                }
                if (blocked) continue;
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
            geom.emit_junction(n, o, C.mats, &ctx.rc);
            // traffic signals: one head per arm with traffic arriving, on the approaching driver's right
            if (!net.nodes[n].signals) continue;
            for (auto &A : geom.junctions[n].arms) {
                auto &e = net.edges[A.edge];
                int incoming_dir = A.at_a ? -1 : 1;  // traffic arriving at this end moves b->a when at_a
                if (e.oneway && e.oneway != incoming_dir) continue;
                if (e.prio < 3) continue;
                V2 out = perp(A.D);
                V2 pos = A.L.xy() + out * (A.swl > 0 ? std::min(1.0, A.swl * 0.4) : 1.2);
                {
                    SurfaceHit sh = geom.query(pos, ctx.rc, 0, A.L.z + 2.0);
                    if (sh.kind == 1 && !sh.margin) continue;  // would stand in a carriageway (acute junctions)
                }
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
            emit_building(b, base, std::min(base - 1.0, lowest - 0.6), o, C, ctx.rc);
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

    // ---- road markings: zebra crossings (mapped crosswalks) and stop lines at signals
    {
        Mesh &mk = C.mats[M_MARKING];
        for (auto &r : region.roads) {
            if (r.kind != RoadKind::Path || r.subclass != "crosswalk") continue;
            Rect bb;
            for (auto p : r.pts) bb.add(p);
            if (!bb.overlaps(rect)) continue;
            for (auto &piece : clip_line(r.pts, rect)) {
                double L = polyline_length(piece);
                int n = std::max(1, int(std::ceil(L / 0.5)));
                std::vector<V3> lft, rgt;
                std::vector<double> ss;
                std::vector<bool> on;
                for (int k = 0; k <= n; k++) {
                    double s = L * k / n;
                    V2 d;
                    V2 p = polyline_at(piece, s, &d);
                    if (k == n) d = norm(p - polyline_at(piece, std::max(0.0, s - 0.1)));
                    SurfaceHit hh = geom.query(p, ctx.rc, 0);
                    on.push_back(hh.kind == 1 && !hh.margin);
                    V2 nl = perp(d) * 1.6;
                    lft.push_back(lv(p + nl, hh.z + 0.025, o)), rgt.push_back(lv(p - nl, hh.z + 0.025, o));
                    ss.push_back(s);
                }
                for (size_t k = 0; k + 1 < lft.size(); k++)
                    if (on[k] && on[k + 1]) add_quad(mk, lft[k], rgt[k], rgt[k + 1], lft[k + 1], {0, ss[k]}, {3.2, ss[k]}, {3.2, ss[k + 1]}, {0, ss[k + 1]}, {0, 0});
            }
        }
        for (int ei : es_) {
            auto &e = net.edges[ei];
            for (int end = 0; end < 2; end++) {
                int node = end == 0 ? e.a : e.b;
                if (!net.nodes[node].signals || e.prio < 3) continue;
                V2 np = net.nodes[node].p;
                if (!(np.x >= rect.mn.x && np.x < rect.mx.x && np.y >= rect.mn.y && np.y < rect.mx.y)) continue;
                // traffic arriving at this end: a->b arrives at b
                int arriving = end == 1 ? 1 : -1;
                if (e.oneway && e.oneway != arriving) continue;
                double s = end == 0 ? e.trim_a + 2.0 : e.len - e.trim_b - 2.0;
                if (s < 0.5 || s > e.len - 0.5) continue;
                V2 d;
                V2 c = e.at(s, &d);
                V2 dir = d * double(arriving);       // direction of travel
                V2 right = perp(dir) * -1.0;
                double z = e.z_at(s) + 0.025;
                double from = e.oneway ? -e.half() : 0.0, to = e.half();
                V2 a = c + right * (from + 0.2), b = c + right * (to - 0.3);
                V2 back = dir * -0.4;
                add_quad(mk, lv(a + back, z, o), lv(b + back, z, o), lv(b, z, o), lv(a, z, o), {0, 0}, {1, 0}, {1, 0.4}, {0, 0.4}, {1, 0});
            }
        }
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
    // ---------------- street furniture, parked cars, barriers, underpass entrances
    std::vector<std::pair<V2, double>> occupied;  // centre, radius: keeps trees out of cars and structures
    auto is_free = [&](V2 p, double r) {
        for (auto &oc : occupied)
            if (dist2(oc.first, p) < (oc.second + r) * (oc.second + r)) return false;
        return true;
    };
    auto surf_z = [&](V2 p, int *kind = nullptr) {
        SurfaceHit sh = geom.query(p, ctx.rc, 0);
        if (kind) *kind = sh.kind;
        return sh.kind == 2 ? sh.z : terrain_ctx(*this, p, ctx, nullptr);
    };
    auto on_carriageway = [&](V2 p) { return geom.query(p, ctx.rc, 0).kind == 1; };
    static const uint32_t car_cols[] = {0xE8E8E6, 0x1C1C1E, 0x8E9296, 0x5A5E63, 0xB8BCC0, 0x7A1E1E, 0x1F3A63, 0x2E4A34, 0xC9B79A, 0x9AA7B0, 0xF0F0EE, 0x3B3F44};
    auto put_car = [&](V2 p, V2 dir, uint64_t h) {
        // the whole footprint must be clear of carriageways, buildings and other cars
        V2 n = perp(dir);
        for (V2 q : {p, p + dir * 2.1 + n * 0.8, p + dir * 2.1 - n * 0.8, p - dir * 2.1 + n * 0.8, p - dir * 2.1 - n * 0.8}) {
            if (on_carriageway(q) || in_building(q)) return false;
        }
        if (!is_free(p, 2.2)) return false;
        bool hole = false;
        int kd = 0;
        double z = terrain_ctx(*this, p, ctx, &hole, &kd);
        if (hole || kd) return false;
        int sk = 0;
        double zs = surf_z(p, &sk);
        if (sk == 2) z = zs;
        Instance I;
        I.p = lv(p, z, o);
        I.yaw = std::atan2(-dir.x, dir.y) + (hash01(h * 3) < 0.5 ? PI : 0);
        I.variant = int(hash32(h * 5) % 3);
        uint32_t c = car_cols[hash32(h * 7) % 12];
        I.col[0] = float(((c >> 16) & 255) / 255.0), I.col[1] = float(((c >> 8) & 255) / 255.0), I.col[2] = float((c & 255) / 255.0), I.col[3] = 1;
        C.inst[I_CAR].push_back(I);
        occupied.push_back({p, 2.4});
        return true;
    };
    // parked cars on car parks: double rows across the lot's long axis
    for (auto *a : ctx.areas) {
        if (a->kind != AreaKind::Parking) continue;
        Rect bb;
        for (auto &p : a->ring.outer) bb.add(p);
        if (!bb.overlaps(rect)) continue;
        double area = std::fabs(poly_area(a->ring.outer));
        if (area < 60 || area > 60000) continue;
        double L, Wd;
        V2 ax;
        obb(a->ring.outer, L, Wd, ax);
        V2 ny = perp(ax), c0 = poly_centroid(a->ring.outer);
        uint64_t ha = hash32(uint64_t(int64_t(c0.x * 10) + 1000000) * 2654435761ULL ^ uint64_t(int64_t(c0.y * 10) + 1000000));
        double fill = 0.55 + 0.35 * hash01(ha);
        for (double v = -Wd * 0.5 + 2.6; v < Wd * 0.5 - 2.0; v += 6.5) {
            for (double u = -L * 0.5 + 1.4; u < L * 0.5 - 1.2; u += 2.6) {
                V2 p = c0 + ax * u + ny * v;
                if (!inside(p) || !point_in_poly(a->ring.outer, p)) continue;
                bool hole = false;
                for (auto &hh : a->ring.holes)
                    if (point_in_poly(hh, p)) hole = true;
                if (hole) continue;
                uint64_t h = hash32(ha ^ (uint64_t(int64_t(u * 10) + 50000) * 7919 + uint64_t(int64_t(v * 10) + 50000)));
                if (hash01(h) > fill) continue;
                put_car(p, ny, h);
            }
        }
    }
    // cars along courtyard driveways and quiet streets without a pavement
    for (int ei : es_) {
        auto &e = net.edges[ei];
        if (e.bridge || e.tunnel || e.level || (e.prio != 2 && e.prio != 3 && e.prio != 4)) continue;
        for (double side : {1.0, -1.0}) {
            if (e.prio == 4 && e.sw(side) > 0) continue;
            for (double s = e.trim_a + 8; s < e.len - e.trim_b - 8; s += 5.6) {
                V2 d;
                V2 c = e.at(s, &d);
                V2 p = c + perp(d) * (side * (e.hs(side > 0 ? 1 : -1) + 1.45));
                if (!inside(p)) continue;
                uint64_t h = hash32(uint64_t(int64_t(p.x * 7) + 1000000) * 2246822519ULL ^ uint64_t(int64_t(p.y * 7) + 1000000));
                if (hash01(h) > (e.prio == 2 ? 0.42 : 0.3)) continue;
                put_car(p, d, h);
            }
        }
    }
    // waste container sites, bus shelters
    for (auto &pt : region.points) {
        if (!inside(pt.p)) continue;
        bool bin = pt.kind == "waste_disposal" || pt.kind == "recycling";
        bool stop = pt.kind == "bus_stop";
        if (!bin && !stop) continue;
        RoadHit rh = net.nearest(pt.p, 30);
        V2 p = pt.p, f{0, 1};
        if (rh.edge >= 0) {
            V2 q = net.edges[rh.edge].at(rh.s);
            if (dist(q, p) > 0.1) f = norm(q - p);
        }
        if (stop) {
            // a shelter stands at the back of the pavement facing the road
            if (rh.edge < 0) continue;
            int guard = 0;
            while (on_carriageway(p) && guard++ < 12) p = p - f * 0.5;
            p = p - f * 1.2;
        }
        if (on_carriageway(p) || in_building(p) || !is_free(p, 2.0)) continue;
        Instance I;
        I.p = lv(p, surf_z(p), o);
        I.yaw = std::atan2(-f.x, f.y);
        C.inst[stop ? I_STOP : I_BIN].push_back(I);
        occupied.push_back({p, stop ? 3.0 : 2.6});
    }
    // playgrounds: a few pieces of equipment
    for (auto *a : ctx.areas) {
        if (a->cls != "playground") continue;
        V2 c0 = poly_centroid(a->ring.outer);
        if (!inside(c0)) continue;
        double area = std::fabs(poly_area(a->ring.outer));
        if (area < 40) continue;
        double L, Wd;
        V2 ax;
        obb(a->ring.outer, L, Wd, ax);
        int n = std::clamp(int(area / 120), 1, 4);
        uint64_t ha = hash32(uint64_t(int64_t(c0.x * 10) + 1000000) * 40503ULL ^ uint64_t(int64_t(c0.y * 10) + 1000000));
        for (int k = 0; k < n; k++) {
            double t = n == 1 ? 0 : (k - (n - 1) * 0.5) * std::min(6.5, L / n);
            V2 p = c0 + ax * t;
            if (!point_in_poly(a->ring.outer, p) || in_building(p) || on_carriageway(p) || !is_free(p, 2.0)) continue;
            Instance I;
            I.p = lv(p, surf_z(p), o);
            I.yaw = std::atan2(-ax.x, ax.y);
            I.variant = int((hash32(ha + k) + k) % 3);
            C.inst[I_PLAY].push_back(I);
            occupied.push_back({p, 2.6});
        }
    }
    // barriers: fences, walls, hedges, guard rails, retaining walls (M_FENCE; uv = along, height; uv2.x = type)
    {
        Mesh &F = C.mats[M_FENCE];
        std::vector<V2> gates;
        for (auto &pt : region.points)
            if ((pt.kind == "gate" || pt.kind == "lift_gate" || pt.kind == "sally_port" || pt.kind == "entrance" || pt.kind == "bollard") && rect.grown(5).contains(pt.p)) gates.push_back(pt.p);
        auto near_gate = [&](V2 p) {
            for (auto &g : gates)
                if (dist2(g, p) < 3.0 * 3.0) return true;
            return false;
        };
        auto solid_quad = [&](V3 a, V3 b, V3 c, V3 d) {
            push_tri(C.col_solid, a, b, c), push_tri(C.col_solid, a, c, d);
            push_tri(C.col_solid, a, c, b), push_tri(C.col_solid, a, d, c);  // both sides
        };
        for (auto &l : region.lines) {
            int type;
            double hgt, th;
            if (l.kind == "fence") type = 0, hgt = 1.6, th = 0;
            else if (l.kind == "wall") type = 1, hgt = 2.0, th = 0.3;
            else if (l.kind == "hedge") type = 2, hgt = 1.3, th = 0.9;
            else if (l.kind == "guard_rail") type = 3, hgt = 0.8, th = 0;
            else if (l.kind == "retaining_wall") type = 4, hgt = 0.9, th = 0.4;
            else continue;
            Rect lb;
            for (auto &q : l.pts) lb.add(q);
            if (!lb.overlaps(rect)) continue;
            for (auto &piece : clip_line(l.pts, rect)) {
                uint64_t hl = hash32(uint64_t(int64_t(piece[0].x * 10) + 1000000) * 31ULL ^ uint64_t(int64_t(piece[0].y * 10) + 1000000));
                double hh = hgt;
                int var = type;
                if (type == 0) {
                    double r = hash01(hl);
                    // metal mesh, picket, decorative cast iron (centre), concrete-panel fence (industrial)
                    var = r < 0.45 ? 0 : r < 0.7 ? 5 : r < 0.85 ? 6 : 7;
                    hh = var == 5 ? 1.3 : var == 6 ? 1.8 : var == 7 ? 2.4 : 1.7;
                }
                if (type == 1) hh = 1.6 + hash01(hl) * 1.2;
                double Lp = polyline_length(piece);
                if (Lp < 0.5) continue;
                double acc = 0;
                for (size_t k = 0; k + 1 < piece.size(); k++) {
                    V2 a = piece[k], b = piece[k + 1];
                    double sl = dist(a, b);
                    if (sl < 0.05) continue;
                    int ns = std::max(1, int(std::ceil(sl / 3.0)));
                    V2 dir = (b - a) / sl, nv = perp(dir);
                    // one surface query per sample point (kind + height), shared by adjacent pieces
                    std::vector<int> kk(ns + 1);
                    std::vector<double> zz(ns + 1);
                    for (int s = 0; s <= ns; s++) {
                        V2 p = lerp(a, b, double(s) / ns);
                        SurfaceHit sh = geom.query(p, ctx.rc, 0);
                        kk[s] = sh.kind;
                        zz[s] = sh.kind == 2 ? sh.z : sh.kind == 0 ? terrain_ctx(*this, p, ctx, nullptr) : 0;
                    }
                    for (int s = 0; s < ns; s++) {
                        V2 p = lerp(a, b, double(s) / ns), q = lerp(a, b, double(s + 1) / ns);
                        V2 m = (p + q) * 0.5;
                        double u0 = acc + sl * s / ns, u1 = acc + sl * (s + 1) / ns;
                        if (kk[s] == 1 || kk[s + 1] == 1 || in_building(m) || near_gate(m)) continue;
                        double zp = zz[s] - 0.15, zq = zz[s + 1] - 0.15;
                        V2 uv2{double(var), hh};
                        if (th <= 0) {
                            V3 A = lv(p, zp, o), B = lv(q, zq, o), Cq = lv(q, zq + hh + 0.15, o), D = lv(p, zp + hh + 0.15, o);
                            add_quad(F, A, B, Cq, D, {u0, -0.15}, {u1, -0.15}, {u1, hh}, {u0, hh}, uv2);
                            solid_quad(A, B, Cq, D);
                        } else {
                            V2 o1 = nv * (th * 0.5);
                            V2 pl = p + o1, ql = q + o1, pr = p - o1, qr = q - o1;
                            double zt = hh + 0.15;
                            V3 A = lv(pr, zp, o), B = lv(qr, zq, o), Cq = lv(qr, zq + zt, o), D = lv(pr, zp + zt, o);
                            V3 A2 = lv(ql, zq, o), B2 = lv(pl, zp, o), C2 = lv(pl, zp + zt, o), D2 = lv(ql, zq + zt, o);
                            add_quad(F, A, B, Cq, D, {u0, -0.15}, {u1, -0.15}, {u1, hh}, {u0, hh}, uv2);
                            add_quad(F, A2, B2, C2, D2, {u1, -0.15}, {u0, -0.15}, {u0, hh}, {u1, hh}, uv2);
                            add_quad(F, D, Cq, D2, C2, {u0, hh}, {u1, hh}, {u1, hh + th}, {u0, hh + th}, uv2);
                            push_tri(C.col_solid, A, B, Cq), push_tri(C.col_solid, A, Cq, D);
                            push_tri(C.col_solid, A2, B2, C2), push_tri(C.col_solid, A2, C2, D2);
                            // end caps where a segment run starts/ends (cheap: every segment)
                            add_quad(F, B2, A, D, C2, {0, -0.15}, {th, -0.15}, {th, hh}, {0, hh}, uv2);
                            add_quad(F, B, A2, D2, Cq, {0, -0.15}, {th, -0.15}, {th, hh}, {0, hh}, uv2);
                        }
                        if (type != 3) occupied.push_back({m, std::max(1.0, sl / ns * 0.5)});
                    }
                    acc += sl;
                }
            }
        }
    }
    // underpass entrances: footpath tunnels that pass under carriageways get parapets around the stairs
    for (auto &r : region.roads) {
        if (r.kind != RoadKind::Path || r.tunnel.empty() || r.pts.size() < 2) continue;
        Rect rb;
        for (auto &q : r.pts) rb.add(q);
        if (!rb.overlaps(rect.grown(20))) continue;
        double L = polyline_length(r.pts);
        double span = r.frac1 - r.frac0;
        if (span <= 0) continue;
        for (auto &tr : r.tunnel) {
            double s0 = std::clamp((tr.a - r.frac0) / span, 0.0, 1.0) * L, s1 = std::clamp((tr.b - r.frac0) / span, 0.0, 1.0) * L;
            if (s1 - s0 < 6 || s1 - s0 > 160) continue;
            // only underpasses below roads (not long tunnels inside buildings)
            bool under_road = false;
            for (double t = 0.2; t < 0.85 && !under_road; t += 0.15) under_road = net.nearest(polyline_at(r.pts, s0 + (s1 - s0) * t), 6).edge >= 0;
            if (!under_road) continue;
            for (int end = 0; end < 2; end++) {
                double s = end == 0 ? s0 : s1;
                if ((end == 0 && s0 < 0.5 && tr.a <= r.frac0 + 1e-6) || (end == 1 && s1 > L - 0.5 && tr.b >= r.frac1 - 1e-6)) {
                    // the tunnel continues in a neighbouring piece; that piece has the portal
                }
                V2 d;
                V2 p = polyline_at(r.pts, s, &d);
                V2 out_dir = end == 0 ? d * -1.0 : d;  // away from the tunnel
                // stairs go down towards the tunnel: the opening is 7 m long, centred behind the portal
                V2 c = p + out_dir * 3.2;
                if (!inside(c)) continue;
                if (on_carriageway(c) || on_carriageway(p + out_dir * 6.5) || in_building(c) || !is_free(c, 1.5)) continue;
                Mesh &F = C.mats[M_FENCE];
                double hw = 1.7, len = 7.0;
                V2 nv = perp(out_dir);
                V2 a0 = p - out_dir * 0.3, a1 = p + out_dir * (len - 0.3);
                double z0 = surf_z(c);
                // dark stairwell seen from above
                V3 S0 = lv(a0 - nv * hw, z0 + 0.03, o), S1 = lv(a0 + nv * hw, z0 + 0.03, o), S2 = lv(a1 + nv * hw, z0 + 0.03, o), S3 = lv(a1 - nv * hw, z0 + 0.03, o);
                add_quad(F, S0, S3, S2, S1, {0, 0}, {0, len}, {hw * 2, len}, {hw * 2, 0}, {8.0, 0});
                // parapets on three sides (open towards the pavement at the far end)
                auto parapet = [&](V2 p0, V2 p1) {
                    V2 dd = norm(p1 - p0), nn = perp(dd) * 0.13;
                    double l2 = dist(p0, p1), ph = 1.05;
                    V3 A = lv(p0 - nn, z0 - 0.2, o), B = lv(p1 - nn, z0 - 0.2, o), Cq = lv(p1 - nn, z0 + ph, o), D = lv(p0 - nn, z0 + ph, o);
                    V3 A2 = lv(p1 + nn, z0 - 0.2, o), B2 = lv(p0 + nn, z0 - 0.2, o), C2 = lv(p0 + nn, z0 + ph, o), D2 = lv(p1 + nn, z0 + ph, o);
                    V2 uv2{9.0, ph};
                    add_quad(F, A, B, Cq, D, {0, 0}, {l2, 0}, {l2, ph}, {0, ph}, uv2);
                    add_quad(F, A2, B2, C2, D2, {0, 0}, {l2, 0}, {l2, ph}, {0, ph}, uv2);
                    add_quad(F, D, Cq, D2, C2, {0, 0}, {l2, 0}, {l2, 0.26}, {0, 0.26}, uv2);
                    push_tri(C.col_solid, A, B, Cq), push_tri(C.col_solid, A, Cq, D);
                    push_tri(C.col_solid, A2, B2, C2), push_tri(C.col_solid, A2, C2, D2);
                };
                parapet(a0 - nv * (hw + 0.13), a1 - nv * (hw + 0.13));
                parapet(a1 + nv * (hw + 0.13), a0 + nv * (hw + 0.13));
                parapet(a0 + nv * (hw + 0.13), a0 - nv * (hw + 0.13));
                occupied.push_back({c, 4.5});
            }
        }
    }
    auto place_tree = [&](V2 p, uint64_t h, double sc) {
        int kd = 0;
        bool hl = false;
        if (!is_free(p, 1.2)) return;
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
            if (sh.kind == 1 && !sh.margin && !sh.bridge) continue;  // never in the carriageway
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
            if (sh.kind == 1) continue;
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
            case AreaKind::Residential: step = 12.0, dens = 0.4; break;  // Minsk courtyards are full of trees
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
