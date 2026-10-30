#include <godot_cpp/variant/packed_string_array.hpp>
#include "earth_stream.h"
#include "glue.h"
#include <godot_cpp/classes/http_client.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/os.hpp>
#include <godot_cpp/classes/tls_options.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <chrono>
#include <map>
#include <thread>

using namespace godot;

namespace {

// One HTTP request with Godot's client; connections are kept per thread and host.
bool http_request(const std::string &url, const std::vector<std::string> &headers, const std::string *post, std::string &out, int &status, std::string &err) {
    thread_local std::map<std::string, Ref<HTTPClient>> conns;
    out.clear();
    status = 0;
    bool tls = url.rfind("https://", 0) == 0;
    size_t hs = url.find("://");
    if (hs == std::string::npos) {
        err = "bad url";
        return false;
    }
    hs += 3;
    size_t ps = url.find('/', hs);
    std::string host = url.substr(hs, ps == std::string::npos ? std::string::npos : ps - hs);
    std::string path = ps == std::string::npos ? "/" : url.substr(ps);
    std::string ck = (tls ? "s:" : "p:") + host;
    for (int attempt = 0; attempt < 2; attempt++) {
        Ref<HTTPClient> &c = conns[ck];
        if (c.is_null()) c.instantiate();
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(post ? 200 : 60);
        auto timed_out = [&] { return std::chrono::steady_clock::now() > deadline; };
        if (c->get_status() != HTTPClient::STATUS_CONNECTED) {
            c->close();
            Error e = c->connect_to_host(String(host.c_str()), tls ? 443 : 80, tls ? TLSOptions::client() : Ref<TLSOptions>());
            if (e != OK) {
                err = "connect failed";
                c.unref();
                continue;
            }
            while (c->get_status() == HTTPClient::STATUS_CONNECTING || c->get_status() == HTTPClient::STATUS_RESOLVING) {
                c->poll();
                if (timed_out()) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            if (c->get_status() != HTTPClient::STATUS_CONNECTED) {
                err = "cannot connect to " + host;
                c.unref();
                continue;
            }
        }
        PackedStringArray hd;
        bool ua = false;
        for (auto &h : headers) {
            hd.push_back(String(h.c_str()));
            ua |= h.rfind("User-Agent", 0) == 0;
        }
        if (!ua) hd.push_back("User-Agent: earthsim/0.5");
        Error e;
        if (post) {
            PackedByteArray body;
            body.resize((int64_t)post->size());
            memcpy(body.ptrw(), post->data(), post->size());
            e = c->request_raw(HTTPClient::METHOD_POST, String(path.c_str()), hd, body);
        } else
            e = c->request(HTTPClient::METHOD_GET, String(path.c_str()), hd);
        if (e != OK) {
            c.unref();
            err = "request failed";
            continue;
        }
        while (c->get_status() == HTTPClient::STATUS_REQUESTING) {
            c->poll();
            if (timed_out()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (c->get_status() != HTTPClient::STATUS_BODY && c->get_status() != HTTPClient::STATUS_CONNECTED) {
            c.unref();
            err = "no response";
            continue;
        }
        status = c->get_response_code();
        while (c->get_status() == HTTPClient::STATUS_BODY) {
            c->poll();
            PackedByteArray ch = c->read_response_body_chunk();
            if (ch.size() > 0) out.append((const char *)ch.ptr(), (size_t)ch.size());
            else std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (timed_out()) {
                c.unref();
                err = "timeout";
                return false;
            }
        }
        return true;
    }
    return false;
}

}  // namespace

EarthStream::~EarthStream() { stop(); }

void EarthStream::start(double lat, double lon, String cache_dir, bool online, double detail_radius, double terrain_radius, bool enrich) {
    stop();
    ec::StreamConfig cfg;
    cfg.lat0 = lat, cfg.lon0 = lon;
    cfg.cache_dir = cache_dir.utf8().get_data();
    cfg.online = online;
    cfg.enrich = enrich;
    cfg.detail_radius = detail_radius;
    cfg.terrain_radius = terrain_radius;
    int cores = (int)std::thread::hardware_concurrency();
    cfg.build_threads = std::max(1, std::min(6, cores - 2));
    cfg.net_threads = 6;
    s_ = std::make_unique<ec::Streamer>(cfg, http_request);
}

void EarthStream::stop() { s_.reset(); }

void EarthStream::update(Vector3 p) {
    if (s_) s_->update(p.x, -p.z);
}

Dictionary EarthStream::poll() {
    Dictionary d;
    if (!s_) return d;
    ec::ChunkResult r;
    if (!s_->poll(r)) return d;
    const ec::ChunkOut &c = *r.chunk;
    d["cx"] = c.cx;
    d["cy"] = c.cy;
    d["terrain_only"] = r.terrain_only;
    d["version"] = r.version;
    d["x0"] = c.x0;
    d["y0"] = c.y0;
    d["size"] = ec::CHUNK;
    d["terrain_hi"] = mesh_arrays(c.terrain_hi);
    d["terrain_lo"] = mesh_arrays(c.terrain_lo);
    // heights for HeightMapShape3D: rows go south (Godot +z)
    int N = c.grid.n + 1;
    PackedFloat32Array h;
    h.resize((int64_t)N * N);
    float *H = h.ptrw();
    for (int zi = 0; zi < N; zi++)
        for (int i = 0; i < N; i++) H[zi * N + i] = c.grid.at(i, N - 1 - zi);
    d["heights"] = h;
    d["grid_n"] = N;
    d["grid_cell"] = c.grid.cell;
    d["splat0"] = bytes(c.splat0);
    d["splat1"] = bytes(c.splat1);
    d["splat_n"] = ec::SPLAT_N;
    Dictionary surf;
    for (auto &[mat, m] : c.ms.m)
        if (m.verts() > 0 && !m.idx.empty()) surf[mat] = mesh_arrays(m);
    d["surfaces"] = surf;
    PackedVector3Array coll;
    coll.resize((int64_t)c.coll.size() / 3);
    Vector3 *C = coll.ptrw();
    for (size_t t = 0; t + 8 < c.coll.size(); t += 9) {
        // flip winding with the axes so faces stay front-facing for Godot
        size_t k = t / 3;
        C[k] = Vector3(c.coll[t], c.coll[t + 2], -c.coll[t + 1]);
        C[k + 1] = Vector3(c.coll[t + 6], c.coll[t + 8], -c.coll[t + 7]);
        C[k + 2] = Vector3(c.coll[t + 3], c.coll[t + 5], -c.coll[t + 4]);
    }
    d["collision"] = coll;
    Dictionary inst;
    for (auto &in : c.inst) {
        if (!inst.has(in.kind)) inst[in.kind] = PackedFloat32Array();
        PackedFloat32Array a = inst[in.kind];
        // godot position, yaw about +y (local yaw is about +z from east, counter-clockwise from above: same sense)
        a.append((float)in.p.x);
        a.append((float)in.p.z);
        a.append((float)-in.p.y);
        a.append((float)in.yaw);
        a.append((float)in.scale);
        a.append(in.c.r);
        a.append(in.c.g);
        a.append(in.c.b);
        inst[in.kind] = a;
    }
    d["instances"] = inst;
    // text signs: 9 floats each (x, y, z in Godot axes, normal angle, max width, r, g, b, max height) and the strings
    PackedFloat32Array lab;
    PackedStringArray lab_text;
    for (auto &l : c.labels) {
        lab.append((float)l.p.x), lab.append((float)l.p.z), lab.append((float)-l.p.y), lab.append((float)l.ang);
        lab.append((float)l.max_w), lab.append(l.c.r), lab.append(l.c.g), lab.append(l.c.b);
        lab.append((float)(l.max_h > 0 ? l.max_h : l.max_w * 0.5));
        lab_text.append(String::utf8(l.text.c_str()));
    }
    d["labels"] = lab;
    d["label_text"] = lab_text;
    Array grass;
    for (auto &g : c.grass) {
        PackedFloat32Array a;
        a.resize((int64_t)g.size());
        if (!g.empty()) memcpy(a.ptrw(), g.data(), g.size() * sizeof(float));
        grass.push_back(a);
    }
    d["grass"] = grass;
    d["buildings"] = c.buildings;
    d["roads"] = c.roads;
    return d;
}

Array EarthStream::take_drops() {
    Array a;
    if (!s_) return a;
    for (auto &[x, y] : s_->take_drops()) a.push_back(Vector2i(x, y));
    return a;
}

double EarthStream::ground(double x, double z) { return s_ ? s_->ground(x, -z) : 0; }
bool EarthStream::wait_ground(double x, double z, double t) { return s_ ? s_->wait_ground(x, -z, t) : false; }
String EarthStream::status() { return s_ ? String(s_->status().c_str()) : String("stopped"); }

Vector2 EarthStream::latlon(Vector3 p) {
    if (!s_) return Vector2();
    double la, lo;
    s_->frame().to_latlon({p.x, -p.z}, la, lo);
    return Vector2((real_t)la, (real_t)lo);
}
Vector3 EarthStream::to_godot(double lat, double lon) {
    if (!s_) return Vector3();
    ec::V2 v = s_->frame().to_local(lat, lon);
    return Vector3((real_t)v.x, 0, (real_t)-v.y);
}

bool EarthStream::collision_ready(Vector3 p) {
    if (!s_) return false;
    return s_->collision()->has((int)std::floor(p.x / ec::CHUNK), (int)std::floor(-p.z / ec::CHUNK));
}

Dictionary EarthStream::nearest_road(Vector3 p, double radius) {
    Dictionary d;
    ec::V3 q;
    double h;
    if (s_ && s_->nearest_road(p.x, -p.z, radius, q, h)) {
        d["pos"] = Vector3((real_t)q.x, (real_t)q.z, (real_t)-q.y);
        d["yaw"] = h;
    }
    return d;
}

PackedFloat32Array EarthStream::traffic(double dt, Vector3 p, Vector3 v, int count) {
    PackedFloat32Array a;
    if (!s_) return a;
    auto &cars = s_->traffic_step(dt, ec::V3(p.x, -p.z, p.y), ec::V3(v.x, -v.z, v.y), count);
    a.resize((int64_t)cars.size() * 9);
    float *f = a.ptrw();
    for (size_t i = 0; i < cars.size(); i++) {
        auto &c = cars[i];
        float *o = f + i * 9;
        o[0] = (float)c.pos.x, o[1] = (float)c.pos.z, o[2] = (float)-c.pos.y;
        o[3] = (float)c.yaw, o[4] = (float)(c.pitch + c.body_pitch), o[5] = (float)c.roll;
        o[6] = c.col.r, o[7] = c.col.g, o[8] = c.col.b;
    }
    return a;
}

PackedFloat32Array EarthStream::people(double dt, Vector3 p, int count) {
    PackedFloat32Array a;
    if (!s_) return a;
    auto &ws = s_->people_step(dt, ec::V3(p.x, -p.z, p.y), count);
    a.resize((int64_t)ws.size() * 9);
    float *f = a.ptrw();
    for (size_t i = 0; i < ws.size(); i++) {
        auto &w = ws[i];
        float *o = f + i * 9;
        o[0] = (float)w.pos.x, o[1] = (float)w.pos.z, o[2] = (float)-w.pos.y, o[3] = (float)w.yaw;
        o[4] = w.col.r, o[5] = w.col.g, o[6] = w.col.b, o[7] = (float)std::fmod(w.phase, 6.283185307), o[8] = (float)w.scale;
    }
    return a;
}

void EarthStream::set_weather_grip(double wet, double snow) {
    if (!s_) return;
    auto c = s_->collision();
    c->wet = (float)wet;
    c->snow = (float)snow;
}

void EarthStream::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_weather_grip", "wet", "snow"), &EarthStream::set_weather_grip);
    ClassDB::bind_method(D_METHOD("people", "dt", "player", "count"), &EarthStream::people);
    ClassDB::bind_method(D_METHOD("traffic", "dt", "player", "player_vel", "count"), &EarthStream::traffic);
    ClassDB::bind_method(D_METHOD("nearest_road", "pos", "radius"), &EarthStream::nearest_road);
    ClassDB::bind_method(D_METHOD("collision_ready", "pos"), &EarthStream::collision_ready);
    ClassDB::bind_method(D_METHOD("start", "lat", "lon", "cache_dir", "online", "detail_radius", "terrain_radius", "enrich"), &EarthStream::start);
    ClassDB::bind_method(D_METHOD("stop"), &EarthStream::stop);
    ClassDB::bind_method(D_METHOD("update", "pos"), &EarthStream::update);
    ClassDB::bind_method(D_METHOD("poll"), &EarthStream::poll);
    ClassDB::bind_method(D_METHOD("take_drops"), &EarthStream::take_drops);
    ClassDB::bind_method(D_METHOD("ground", "x", "z"), &EarthStream::ground);
    ClassDB::bind_method(D_METHOD("wait_ground", "x", "z", "timeout"), &EarthStream::wait_ground);
    ClassDB::bind_method(D_METHOD("status"), &EarthStream::status);
    ClassDB::bind_method(D_METHOD("latlon", "pos"), &EarthStream::latlon);
    ClassDB::bind_method(D_METHOD("to_godot", "lat", "lon"), &EarthStream::to_godot);
}
