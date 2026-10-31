#include "earth_world.h"
#include <cmath>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

using namespace godot;

static inline Vector3 G(double x, double y, double z) { return Vector3(real_t(x), real_t(z), real_t(-y)); }
static inline es::V2 L(const Vector3 &p) { return {p.x, -p.z}; }

void EarthWorld::_bind_methods() {
    ClassDB::bind_method(D_METHOD("load", "dir"), &EarthWorld::load);
    ClassDB::bind_method(D_METHOD("is_loaded"), &EarthWorld::is_loaded);
    ClassDB::bind_method(D_METHOD("get_log"), &EarthWorld::get_log);
    ClassDB::bind_method(D_METHOD("chunk_range"), &EarthWorld::chunk_range);
    ClassDB::bind_method(D_METHOD("chunk_size"), &EarthWorld::chunk_size);
    ClassDB::bind_method(D_METHOD("build_chunk", "cx", "cy"), &EarthWorld::build_chunk);
    ClassDB::bind_method(D_METHOD("build_far", "fx", "fy"), &EarthWorld::build_far);
    ClassDB::bind_method(D_METHOD("far_range"), &EarthWorld::far_range);
    ClassDB::bind_method(D_METHOD("far_size"), &EarthWorld::far_size);
    ClassDB::bind_method(D_METHOD("spawn_near", "pos"), &EarthWorld::spawn_near);
    ClassDB::bind_method(D_METHOD("ground_height", "pos"), &EarthWorld::ground_height);
    ClassDB::bind_method(D_METHOD("latlon_to_world", "lat", "lon"), &EarthWorld::latlon_to_world);
    ClassDB::bind_method(D_METHOD("world_to_latlon", "pos"), &EarthWorld::world_to_latlon);
    ClassDB::bind_method(D_METHOD("road_at", "pos"), &EarthWorld::road_at);
    ClassDB::bind_method(D_METHOD("region_info"), &EarthWorld::region_info);
    ClassDB::bind_method(D_METHOD("follow", "pos", "heading", "ahead"), &EarthWorld::follow);
    ClassDB::bind_method(D_METHOD("edge_info", "e"), &EarthWorld::edge_info);
    ClassDB::bind_method(D_METHOD("lane_pose", "e", "s", "dir", "lane_off"), &EarthWorld::lane_pose);
    ClassDB::bind_method(D_METHOD("next_edge", "e", "dir", "rnd"), &EarthWorld::next_edge);
    ClassDB::bind_method(D_METHOD("traffic_spawns", "centre", "rmin", "rmax", "count", "seed"), &EarthWorld::traffic_spawns);
}

bool EarthWorld::load(const String &dir) {
    auto nw = std::make_unique<es::World>();
    bool ok = nw->load(std::string(dir.utf8().get_data()));
    log_.clear();
    for (auto &l : nw->log) log_.push_back(String::utf8(l.c_str()));
    if (ok) w = std::move(nw);
    return ok;
}

Rect2i EarthWorld::chunk_range() const {
    if (!w) return Rect2i();
    int x0, y0, x1, y1;
    w->chunk_range(x0, y0, x1, y1);
    return Rect2i(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
}

static PackedVector3Array tri_soup(const std::vector<float> &v) {
    PackedVector3Array a;
    size_t n = v.size() / 3;
    a.resize(int64_t(n));
    Vector3 *p = a.ptrw();
    // reverse each triangle's winding: core is CCW-front, Godot is CW-front
    for (size_t t = 0; t + 2 < n; t += 3) {
        p[t] = G(v[t * 3], v[t * 3 + 1], v[t * 3 + 2]);
        p[t + 1] = G(v[(t + 2) * 3], v[(t + 2) * 3 + 1], v[(t + 2) * 3 + 2]);
        p[t + 2] = G(v[(t + 1) * 3], v[(t + 1) * 3 + 1], v[(t + 1) * 3 + 2]);
    }
    return a;
}

static Dictionary to_dict(const es::ChunkData *c);

Dictionary EarthWorld::build_chunk(int cx, int cy) const {
    if (!w) return Dictionary();
    auto c = w->build_chunk(cx, cy);
    return to_dict(c.get());
}

Dictionary EarthWorld::build_far(int fx, int fy) const {
    if (!w) return Dictionary();
    auto c = w->build_far(fx, fy);
    return to_dict(c.get());
}

Rect2i EarthWorld::far_range() const {
    if (!w) return Rect2i();
    int x0, y0, x1, y1;
    w->far_range(x0, y0, x1, y1);
    return Rect2i(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
}

static Dictionary to_dict(const es::ChunkData *c) {
    Dictionary d;
    d["cx"] = c->cx;
    d["cy"] = c->cy;
    d["origin"] = G(c->origin.x, c->origin.y, 0);
    Array surfaces;
    for (int k = 0; k < es::M_COUNT; k++) {
        const es::Mesh &m = c->mats[k];
        if (m.idx.empty()) continue;
        size_t n = m.verts();
        PackedVector3Array pos, nrm;
        PackedVector2Array uv, uv2;
        PackedColorArray col;
        PackedInt32Array idx;
        pos.resize(int64_t(n)), nrm.resize(int64_t(n)), uv.resize(int64_t(n)), uv2.resize(int64_t(n)), col.resize(int64_t(n));
        Vector3 *pp = pos.ptrw(), *pn = nrm.ptrw();
        Vector2 *pu = uv.ptrw(), *pu2 = uv2.ptrw();
        Color *pc = col.ptrw();
        for (size_t i = 0; i < n; i++) {
            pp[i] = G(m.pos[i * 3], m.pos[i * 3 + 1], m.pos[i * 3 + 2]);
            pn[i] = G(m.nrm[i * 3], m.nrm[i * 3 + 1], m.nrm[i * 3 + 2]);
            pu[i] = Vector2(m.uv[i * 2], m.uv[i * 2 + 1]);
            pu2[i] = Vector2(m.uv2[i * 2], m.uv2[i * 2 + 1]);
            pc[i] = Color(m.col[i * 4], m.col[i * 4 + 1], m.col[i * 4 + 2], m.col[i * 4 + 3]);
        }
        idx.resize(int64_t(m.idx.size()));
        int32_t *pi = idx.ptrw();
        for (size_t t = 0; t + 2 < m.idx.size(); t += 3) pi[t] = m.idx[t], pi[t + 1] = m.idx[t + 2], pi[t + 2] = m.idx[t + 1];
        Array arr;
        arr.resize(Mesh::ARRAY_MAX);
        arr[Mesh::ARRAY_VERTEX] = pos;
        arr[Mesh::ARRAY_NORMAL] = nrm;
        arr[Mesh::ARRAY_TEX_UV] = uv;
        arr[Mesh::ARRAY_TEX_UV2] = uv2;
        arr[Mesh::ARRAY_COLOR] = col;
        arr[Mesh::ARRAY_INDEX] = idx;
        Array s;
        s.push_back(k);
        s.push_back(arr);
        surfaces.push_back(s);
    }
    d["surfaces"] = surfaces;
    d["col_road"] = tri_soup(c->col_road);
    d["col_ground"] = tri_soup(c->col_ground);
    d["col_solid"] = tri_soup(c->col_solid);
    Array inst;
    for (int k = 0; k < es::I_COUNT; k++) {
        PackedFloat32Array f;
        f.resize(int64_t(c->inst[k].size() * 6));
        float *pf = f.ptrw();
        size_t i = 0;
        for (auto &I : c->inst[k]) {
            Vector3 g = G(I.p.x, I.p.y, I.p.z);
            pf[i++] = g.x, pf[i++] = g.y, pf[i++] = g.z, pf[i++] = float(I.yaw), pf[i++] = float(I.scale), pf[i++] = float(I.variant);
        }
        inst.push_back(f);
    }
    d["inst"] = inst;
    // ready-made MultiMesh buffers (TRANSFORM_3D rows + optional colour + optional custom data)
    Dictionary mm;
    auto make = [&](int kind, int variant_sel, bool colors, bool custom) {
        PackedFloat32Array b;
        int stride = 12 + (colors ? 4 : 0) + (custom ? 4 : 0);
        int count = 0;
        for (auto &I : c->inst[kind])
            if (variant_sel < 0 || (variant_sel == 3) == (I.variant == 3)) count++;
        b.resize(int64_t(count) * stride);
        float *p = b.ptrw();
        static const float tint[4][3] = {{0.85f, 1.0f, 0.8f}, {1.0f, 1.0f, 1.0f}, {0.75f, 0.9f, 0.7f}, {1.05f, 0.95f, 0.75f}};
        for (auto &I : c->inst[kind]) {
            if (!(variant_sel < 0 || (variant_sel == 3) == (I.variant == 3))) continue;
            Vector3 g = G(I.p.x, I.p.y, I.p.z);
            float cy = float(std::cos(I.yaw)), sy = float(std::sin(I.yaw)), sc = float(I.scale);
            // Basis(UP, yaw) scaled: x = (cos, 0, -sin), y = (0, 1, 0), z = (sin, 0, cos) as columns
            p[0] = cy * sc, p[1] = 0, p[2] = sy * sc, p[3] = g.x;
            p[4] = 0, p[5] = sc, p[6] = 0, p[7] = g.y;
            p[8] = -sy * sc, p[9] = 0, p[10] = cy * sc, p[11] = g.z;
            p += 12;
            if (colors) {
                auto &t = tint[I.variant & 3];
                p[0] = t[0], p[1] = t[1], p[2] = t[2], p[3] = 1, p += 4;
            }
            if (custom) {
                // traffic lights: perpendicular approaches run opposite phases of the 40 s cycle
                bool ns = std::fabs(std::cos(I.yaw)) > 0.7071;
                p[0] = ns ? 0.0f : 20.0f, p[1] = p[2] = p[3] = 0, p += 4;
            }
        }
        Dictionary e;
        e["buffer"] = b;
        e["count"] = count;
        e["colors"] = colors;
        e["custom"] = custom;
        return e;
    };
    mm["tree_broad"] = make(es::I_TREE, 0, true, false);
    mm["tree_conifer"] = make(es::I_TREE, 3, true, false);
    mm["lamp"] = make(es::I_LAMP, -1, false, false);
    mm["signal"] = make(es::I_SIGNAL, -1, false, true);
    mm["bench"] = make(es::I_BENCH, -1, false, false);
    d["mm"] = mm;
    // ground heights for fast queries: (TN+1)^2 from south-west, plus grid geometry
    PackedFloat32Array gh;
    gh.resize(int64_t(c->ground.size()));
    for (size_t i = 0; i < c->ground.size(); i++) gh.ptrw()[i] = c->ground[i];
    d["ground"] = gh;
    // height map for collision (Godot HeightMapShape3D: centred, rows along +Z = south),
    // in units of one terrain cell so the shape can be scaled uniformly
    bool holes = false;
    for (auto h : c->hole) holes |= h != 0;
    d["has_holes"] = holes;
    if (!c->ground.empty() && !holes) {
        int n = es::TN + 1;
        PackedFloat32Array hm;
        hm.resize(int64_t(n) * n);
        float *ph = hm.ptrw();
        for (int zi = 0; zi < n; zi++)
            for (int xi = 0; xi < n; xi++) ph[zi * n + xi] = c->ground[size_t(es::TN - zi) * n + xi] / float(es::TCELL);
        d["hm"] = hm;
        d["hm_size"] = n;
        d["hm_scale"] = es::TCELL;
    }
    d["buildings"] = c->stats_buildings;
    d["edges"] = c->stats_edges;
    return d;
}

Dictionary EarthWorld::spawn_near(const Vector3 &p) const {
    Dictionary d;
    if (!w) return d;
    es::V3 pos;
    double yaw;
    if (!w->spawn_point(L(p), pos, yaw)) return d;
    d["pos"] = G(pos.x, pos.y, pos.z);
    d["yaw"] = yaw;
    return d;
}

// height of the surface under p: surfaces more than 2 m above p (bridges overhead) are ignored
double EarthWorld::ground_height(const Vector3 &p) const { return w ? w->ground_at(L(p), p.y == 0.0 ? 1e9 : p.y + 2.0) : 0.0; }

Vector3 EarthWorld::latlon_to_world(double lat, double lon) const {
    if (!w) return Vector3();
    es::V2 q = w->region.frame.from_ll(lat, lon);
    return G(q.x, q.y, 0);
}

Vector2 EarthWorld::world_to_latlon(const Vector3 &p) const {
    if (!w) return Vector2();
    double lat, lon;
    w->region.frame.to_ll(L(p), lat, lon);
    return Vector2(real_t(lat), real_t(lon));
}

Dictionary EarthWorld::road_at(const Vector3 &p) const {
    Dictionary d;
    if (!w) return d;
    es::RoadHit h = w->net.nearest_at(L(p), p.y - 0.4, 15, 3.5);
    if (h.edge < 0) return d;
    auto &e = w->net.edges[h.edge];
    d["name"] = String::utf8(e.name.c_str());
    d["class"] = String(e.cls.c_str());
    d["speed"] = e.speed;
    d["dist"] = h.d;
    d["width"] = e.width;
    d["lanes"] = e.lanes;
    return d;
}

Dictionary EarthWorld::region_info() const {
    Dictionary d;
    if (!w) return d;
    d["lat"] = w->region.frame.lat0;
    d["lon"] = w->region.frame.lon0;
    d["release"] = String(w->region.release.c_str());
    d["min"] = G(w->region.bounds.mn.x, w->region.bounds.mx.y, 0);
    d["max"] = G(w->region.bounds.mx.x, w->region.bounds.mn.y, 0);
    d["roads"] = int64_t(w->net.edges.size());
    d["buildings"] = int64_t(w->buildings.size());
    return d;
}

Dictionary EarthWorld::follow(const Vector3 &p, const Vector3 &heading, double ahead) const {
    Dictionary d;
    if (!w) return d;
    es::V2 t, c;
    int lim = 0;
    double wd = 0;
    if (!w->net.follow(L(p), es::V2{heading.x, -heading.z}, ahead, t, &lim, &c, &wd, p.y - 0.4)) return d;
    d["target"] = G(t.x, t.y, p.y);
    d["centre"] = G(c.x, c.y, p.y);
    d["width"] = wd;
    d["limit"] = lim;
    return d;
}

Dictionary EarthWorld::edge_info(int e) const {
    Dictionary d;
    if (!w || e < 0 || e >= int(w->net.edges.size())) return d;
    auto &E = w->net.edges[e];
    d["len"] = E.len;
    d["speed"] = E.speed;
    d["oneway"] = E.oneway;
    d["lanes"] = E.lanes;
    d["width"] = E.width;
    d["prio"] = E.prio;
    d["a"] = E.a;
    d["b"] = E.b;
    d["signals_a"] = w->net.nodes[E.a].signals;
    d["signals_b"] = w->net.nodes[E.b].signals;
    d["trim_a"] = E.trim_a;
    d["trim_b"] = E.trim_b;
    return d;
}

// Position on the lane: dir +1 travels a->b. lane_off > 0 is to the right of travel.
Dictionary EarthWorld::lane_pose(int e, double s, int dir, double lane_off) const {
    Dictionary d;
    if (!w || e < 0 || e >= int(w->net.edges.size())) return d;
    auto &E = w->net.edges[e];
    s = es::clampd(s, 0, E.len);
    es::V2 t;
    es::V2 c = E.at(s, &t);
    t = t * double(dir);
    es::V2 p = c - es::perp(t) * lane_off;
    double z = E.z_at(s);
    // pitch from the profile
    double s2 = es::clampd(s + dir * 2.0, 0, E.len);
    double slope = (E.z_at(s2) - z) / std::max(0.5, std::fabs(s2 - s));
    d["pos"] = G(p.x, p.y, z);
    d["yaw"] = std::atan2(-t.x, t.y);
    d["slope"] = slope;
    return d;
}

// Next edge at the end of e (travelling dir): straight-ish continuations preferred, oneways respected.
Dictionary EarthWorld::next_edge(int e, int dir, double rnd) const {
    Dictionary d;
    if (!w || e < 0) return d;
    auto &N = w->net;
    auto &E = N.edges[e];
    int node = dir > 0 ? E.b : E.a;
    es::V2 tin;
    E.at(dir > 0 ? E.len : 0.0, &tin);
    tin = tin * double(dir);
    std::vector<std::pair<double, std::pair<int, int>>> opts;
    double tot = 0;
    for (int nj : N.nodes[node].edges) {
        if (nj == e) continue;
        auto &F = N.edges[nj];
        int fd = F.a == node ? 1 : -1;
        if (F.oneway && F.oneway != fd) continue;
        if (F.prio < 3 || F.tunnel || F.len < 3) continue;
        es::V2 q = F.at(fd > 0 ? std::min(F.len, 8.0) : std::max(0.0, F.len - 8.0));
        double c = es::dot(es::norm(q - N.nodes[node].p), tin);
        if (c < -0.3) continue;  // no U-turns / very sharp turns
        int far = fd > 0 ? F.b : F.a;
        double wgt = (c > 0.8 ? 3.0 : 1.0) * (N.nodes[far].edges.size() <= 1 ? 0.1 : 1.0) * (F.prio >= E.prio ? 1.5 : 1.0);
        opts.push_back({wgt, {nj, fd}});
        tot += wgt;
    }
    if (opts.empty()) return d;
    double x = rnd * tot;
    for (auto &o : opts) {
        if (x <= o.first) {
            d["edge"] = o.second.first;
            d["dir"] = o.second.second;
            return d;
        }
        x -= o.first;
    }
    d["edge"] = opts.back().second.first;
    d["dir"] = opts.back().second.second;
    return d;
}

Array EarthWorld::traffic_spawns(const Vector3 &centre, double rmin, double rmax, int count, int seed) const {
    Array out;
    if (!w) return out;
    auto &N = w->net;
    std::vector<int> cand;
    es::V2 c = L(centre);
    N.query(c, rmax, cand);
    std::vector<int> good;
    for (int e : cand) {
        auto &E = N.edges[e];
        if (E.prio < 4 || E.tunnel || E.bridge || E.level || E.len < 30) continue;
        double d = es::dist(E.at(E.len * 0.5), c);
        if (d < rmin || d > rmax) continue;
        good.push_back(e);
    }
    if (good.empty()) return out;
    uint64_t h = uint64_t(seed) * 2654435761ULL + 12345;
    for (int i = 0; i < count; i++) {
        h = es::hash32(h + i * 7919);
        int e = good[h % good.size()];
        auto &E = N.edges[e];
        int dir = E.oneway ? E.oneway : ((h >> 8) & 1 ? 1 : -1);
        double s = E.trim_a + 4 + es::hash01(h * 31) * std::max(1.0, E.len - E.trim_a - E.trim_b - 8);
        Dictionary d;
        d["edge"] = e;
        d["dir"] = dir;
        d["s"] = s;
        out.push_back(d);
    }
    return out;
}
