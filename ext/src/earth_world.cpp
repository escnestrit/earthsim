#include "earth_world.h"
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
    // ground heights for fast queries: (TN+1)^2 from south-west, plus grid geometry
    PackedFloat32Array gh;
    gh.resize(int64_t(c->ground.size()));
    for (size_t i = 0; i < c->ground.size(); i++) gh.ptrw()[i] = c->ground[i];
    d["ground"] = gh;
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

double EarthWorld::ground_height(const Vector3 &p) const { return w ? w->ground_at(L(p)) : 0.0; }

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
    es::RoadHit h = w->net.nearest(L(p), 15);
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
    es::V2 t;
    int lim = 0;
    if (!w->net.follow(L(p), es::V2{heading.x, -heading.z}, ahead, t, &lim)) return d;
    d["target"] = G(t.x, t.y, p.y);
    d["limit"] = lim;
    return d;
}
