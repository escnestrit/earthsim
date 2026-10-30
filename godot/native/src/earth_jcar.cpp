#include "earth_jcar.h"
#include "jbeam/jtable.hpp"
#include "core/json.hpp"
#include <algorithm>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/classes/input.hpp>
#include <godot_cpp/classes/skin.hpp>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include "glue.h"
#include "jbeam/beamng.hpp"

using namespace godot;

namespace {
Vector3 conv(ec::V3 v) { return Vector3((real_t)v.x, (real_t)v.z, (real_t)-v.y); }
Basis frame_basis(ec::V3 x, ec::V3 y, ec::V3 z) {
    // rest-frame axes (x fwd, y left, z up) as world vectors -> Godot basis of a node using the same convention
    Basis b;
    b.set_column(0, conv(x));
    b.set_column(1, conv(z));
    b.set_column(2, -conv(y));
    return b;
}
ec::MeshBuf to_buf(const ec::SubMesh &s) {
    ec::MeshBuf m;
    for (size_t i = 0; i < s.pos.size(); i++) m.v({s.pos[i][0], s.pos[i][1], s.pos[i][2]}, {s.nrm[i][0], s.nrm[i][1], s.nrm[i][2]}, s.uv[i][0], s.uv[i][1], {1, 1, 1, 1});
    for (size_t k = 0; k + 2 < s.idx.size(); k += 3) m.tri((int)s.idx[k], (int)s.idx[k + 1], (int)s.idx[k + 2]);
    return m;
}
}  // namespace

EarthJCar::~EarthJCar() {
    stop_ = true;
    if (sim_.joinable()) sim_.join();
    if (loader_.joinable()) loader_.join();
}

String EarthJCar::beamng_vehicles_dir() { return String(ec::find_beamng_vehicles().c_str()); }

// what a BeamNG zip holds, from vehicles/<name>/info.json: drivable cars and trucks only (props, trailers
// and test rigs are skipped). Reads only the zip's directory and one small file.
static bool vehicle_info(const std::string &path, std::string &name, std::string &type) {
    ec::Zip z;
    std::string err;
    if (!z.open(path, err)) return false;
    bool has_jbeam = false;
    for (auto &e : z.entries) {
        if (!has_jbeam && e.name.size() > 6 && e.name.compare(e.name.size() - 6, 6, ".jbeam") == 0) has_jbeam = true;
        // vehicles/<model>/info.json (not the per-config info_*.json)
        if (e.name.size() < 10 || e.name.compare(e.name.size() - 10, 10, "/info.json") != 0) continue;
        if (std::count(e.name.begin(), e.name.end(), '/') != 2) continue;
        std::string txt;
        if (!z.read(e, txt, err)) continue;
        ec::Json j;
        if (!ec::Json::parse(ec::jbeam_normalize(txt), j)) continue;
        auto str = [&](const char *k) -> std::string {
            const ec::Json *v = j.get(k);
            return v && v->kind == ec::Json::Str ? v->s : "";
        };
        type = str("Type");
        std::string brand = str("Brand"), nm = str("Name");
        if (!nm.empty()) name = brand.empty() ? nm : brand + " " + nm;
        return true;
    }
    return has_jbeam;  // a mod without info.json: offer it anyway
}

Array EarthJCar::find_vehicles(PackedStringArray dirs) {
    Array out;
    std::vector<std::filesystem::path> ds;
    for (int i = 0; i < dirs.size(); i++) ds.push_back(std::filesystem::u8path(dirs[i].utf8().get_data()));
    std::string b = ec::find_beamng_vehicles();
    if (!b.empty()) ds.push_back(std::filesystem::u8path(b));
    std::error_code e;
    for (size_t di = 0; di < ds.size(); di++) {
        std::vector<std::filesystem::path> files;
        for (auto &f : std::filesystem::directory_iterator(ds[di], e))
            if (f.path().extension() == ".zip" && f.path().filename() != "common.zip") files.push_back(f.path());
        std::sort(files.begin(), files.end());
        for (auto &fp : files) {
            std::string path = fp.u8string();
            std::string name = fp.stem().u8string(), type;
            if (!vehicle_info(path, name, type)) continue;
            if (!type.empty() && type != "Car" && type != "Truck" && type != "Automation") continue;
            Dictionary v;
            v["path"] = String::utf8(path.c_str());
            v["name"] = String::utf8(name.c_str());
            v["type"] = String::utf8(type.c_str());
            v["mod"] = di + 1 < ds.size() || b.empty();
            out.push_back(v);
        }
    }
    return out;
}

void EarthJCar::load(Ref<EarthStream> stream, String path) {
    stream_ = stream;
    coll_ = stream->core() ? stream->core()->collision() : nullptr;
    load_path_ = path.utf8().get_data();
    load_state_ = 1;
    loader_ = std::thread([this] {
        std::string err;
        ec::Zip z;
        if (!z.open(load_path_, err)) {
            load_err_ = err, load_state_ = -1;
            return;
        }
        ec::Library lib;
        lib.add_zip(z, false);
        // the default configuration, as BeamNG spawns the vehicle (parts per slot, tuning variables)
        ec::VehicleConfig cfgv;
        std::string cerr;
        bool have_cfg = ec::default_config(z, cfgv, cerr);
        if (!have_cfg) notes_.push_back("no default configuration (" + cerr + "): default parts");
        // the vehicle's own main part (common.zip, added next, has none worth driving)
        std::string main;
        if (have_cfg && !cfgv.main_part.empty() && lib.parts.count(cfgv.main_part)) main = cfgv.main_part;
        if (auto mt = lib.by_type.find("main"); main.empty() && mt != lib.by_type.end() && !mt->second.empty()) {
            // prefer the part named like the zip (pickup.zip -> "pickup"), else the shortest name
            std::string stem = have_cfg ? cfgv.model : std::filesystem::u8path(load_path_).stem().u8string();
            for (auto &n : mt->second)
                if (n == stem) main = n;
            if (main.empty()) main = *std::min_element(mt->second.begin(), mt->second.end(), [](const std::string &a, const std::string &b) { return a.size() != b.size() ? a.size() < b.size() : a < b; });
        }
        std::string bdir = ec::find_beamng_vehicles();
        ec::Zip cz;
        ec::Zip *common = nullptr;
        if (!bdir.empty() && cz.open((std::filesystem::u8path(bdir) / "common.zip").u8string(), err)) {
            lib.add_zip(cz, true);
            common = &cz;
            notes_.push_back("BeamNG common parts from " + bdir);
        } else
            notes_.push_back("BeamNG installation not found: parts from its common.zip are missing");
        if (main.empty()) {
            load_err_ = "no part with slotType \"main\" in the mod", load_state_ = -1;
            return;
        }
        ec::JModel m = ec::build_model(ec::assemble(lib, main, have_cfg ? cfgv.parts : std::map<std::string, std::string>{}, have_cfg ? &cfgv.vars : nullptr));
        auto v = std::make_unique<ec::JVehicle>();
        if (!ec::JVehicle::from_model(m, *v, err)) {
            load_err_ = err, load_state_ = -1;
            return;
        }
        notes_.insert(notes_.end(), v->notes.begin(), v->notes.end());
        if (!m.asm_.missing.empty()) notes_.push_back(std::to_string(m.asm_.missing.size()) + " parts missing (BeamNG common files): defaults or empty slots");
        auto vis = std::make_unique<ec::CarVisual>();
        if (!ec::load_visual(z, common, m, *v, *vis, err)) notes_.push_back("meshes not loaded: " + err);
        notes_.insert(notes_.end(), vis->notes.begin(), vis->notes.end());
        v_ = std::move(v);
        vis_ = std::move(vis);
        load_state_ = 2;
    });
}

String EarthJCar::poll() {
    int s = load_state_;
    if (s == 1) return "loading";
    if (s == -1) return String("error: ") + load_err_.c_str();
    if (s == 2) {
        if (loader_.joinable()) loader_.join();
        build_nodes();
        load_state_ = 3;
    }
    return s == 0 ? "idle" : "ready";
}

String EarthJCar::notes() const {
    String s;
    for (auto &n : notes_) s += String(n.c_str()) + "\n";
    return s;
}

Ref<StandardMaterial3D> EarthJCar::material(const std::string &name, std::map<std::string, Ref<StandardMaterial3D>> &cache, std::map<std::string, Ref<Texture2D>> &tex) {
    auto it = cache.find(name);
    if (it != cache.end()) return it->second;
    Ref<StandardMaterial3D> m;
    m.instantiate();
    auto md = vis_->materials.find(name);
    if (md == vis_->materials.end()) {
        m->set_albedo(Color(0.35f, 0.35f, 0.36f));
        m->set_roughness(0.7f);
    } else {
        const ec::MatDef &d = md->second;
        m->set_albedo(Color(d.base_color[0], d.base_color[1], d.base_color[2], d.translucent ? std::min(d.base_color[3], 0.45f) : 1.0f));
        m->set_roughness(std::clamp(d.roughness, 0.05f, 1.0f));
        m->set_metallic(std::clamp(d.metallic, 0.0f, 1.0f));
        auto load_tex = [&](const std::string &path, bool normal) -> Ref<Texture2D> {
            if (path.empty()) return Ref<Texture2D>();
            auto t = tex.find(path);
            if (t != tex.end()) return t->second;
            Ref<Texture2D> r;
            auto bytes = vis_->textures.find(path);
            if (bytes != vis_->textures.end()) {
                PackedByteArray b = bytes_of(bytes->second);
                Ref<Image> img;
                img.instantiate();
                std::string l = path;
                for (auto &c : l) c = (char)tolower((unsigned char)c);
                Error e = l.size() > 4 && l.compare(l.size() - 4, 4, ".dds") == 0 ? img->load_dds_from_buffer(b) : l.size() > 4 && l.compare(l.size() - 4, 4, ".png") == 0 ? img->load_png_from_buffer(b) : img->load_jpg_from_buffer(b);
                if (e == OK && !img->is_empty()) {
                    if (!img->has_mipmaps() && !img->is_compressed()) img->generate_mipmaps(normal);
                    r = ImageTexture::create_from_image(img);
                }
            }
            tex[path] = r;
            return r;
        };
        Ref<Texture2D> a = load_tex(d.base_map, false);
        if (a.is_valid()) m->set_texture(BaseMaterial3D::TEXTURE_ALBEDO, a);
        Ref<Texture2D> n = load_tex(d.normal_map, true);
        if (n.is_valid()) {
            m->set_feature(BaseMaterial3D::FEATURE_NORMAL_MAPPING, true);
            m->set_texture(BaseMaterial3D::TEXTURE_NORMAL, n);
        }
        if (d.translucent) {
            m->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
            m->set_cull_mode(BaseMaterial3D::CULL_DISABLED);
        }
    }
    cache[name] = m;
    return m;
}

void EarthJCar::build_nodes() {
    ec::JVehicle &v = *v_;
    rest_com_ = v.rest_com();
    skel_ = memnew(Skeleton3D);
    add_child(skel_);
    Ref<Skin> skin;
    skin.instantiate();
    for (size_t i = 0; i < v.body.nodes.size(); i++) {
        Transform3D t(Basis(), conv(v.body.nodes[i].rest));
        int b = skel_->add_bone(String("n") + String::num_int64((int64_t)i));
        skel_->set_bone_rest(b, t);
        skel_->set_bone_pose_position(b, t.origin);
        skin->add_bind(b, t.affine_inverse());
    }
    std::map<std::string, Ref<StandardMaterial3D>> mats;
    std::map<std::string, Ref<Texture2D>> tex;
    Ref<ArrayMesh> body;
    body.instantiate();
    for (auto &p : vis_->parts) {
        for (size_t si = 0; si < p.subs.size(); si++) {
            const ec::SubMesh &s = p.subs[si];
            bool invisible = ec::is_invisible_material(s.material);
            bool swaps = !p.swap_group.empty() && p.swap_base == s.material;
            if (invisible && !swaps) continue;
            if (invisible) continue;  // glass that appears only when damaged: not shown
            Array a = mesh_arrays(to_buf(s));
            Ref<StandardMaterial3D> mat = material(s.material, mats, tex);
            if (p.wheel >= 0) {
                Ref<ArrayMesh> wm;
                wm.instantiate();
                wm->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a);
                wm->surface_set_material(0, mat);
                MeshInstance3D *mi = memnew(MeshInstance3D);
                mi->set_mesh(wm);
                add_child(mi);
                wheel_mi_.push_back(mi);
                wheel_of_mi_.push_back(p.wheel);
                continue;
            }
            PackedInt32Array bones;
            PackedFloat32Array w;
            bones.resize((int64_t)s.pos.size() * 4);
            w.resize((int64_t)s.pos.size() * 4);
            for (size_t k = 0; k < s.pos.size(); k++)
                for (int q = 0; q < 4; q++) {
                    bones.set((int64_t)k * 4 + q, p.bones[si][k][q]);
                    w.set((int64_t)k * 4 + q, p.weights[si][k][q]);
                }
            a[Mesh::ARRAY_BONES] = bones;
            a[Mesh::ARRAY_WEIGHTS] = w;
            body->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a);
            body->surface_set_material(body->get_surface_count() - 1, mat);
        }
    }
    MeshInstance3D *bm = memnew(MeshInstance3D);
    bm->set_mesh(body);
    skel_->add_child(bm);
    bm->set_skin(skin);
    bm->set_skeleton_path(NodePath(".."));
    for (auto &pr : vis_->props) {
        Node3D *n = memnew(Node3D);
        add_child(n);
        for (auto &s : pr.subs) {
            if (ec::is_invisible_material(s.material)) continue;
            Ref<ArrayMesh> m;
            m.instantiate();
            m->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, mesh_arrays(to_buf(s)));
            m->surface_set_material(0, material(s.material, mats, tex));
            MeshInstance3D *mi = memnew(MeshInstance3D);
            mi->set_mesh(m);
            n->add_child(mi);
        }
        props_.push_back(n);
    }
    // the meshes keep their CPU copies only on the GPU now
    for (auto &p : vis_->parts) p.subs.clear(), p.bones.clear(), p.weights.clear();
    vis_->textures.clear();
}

bool EarthJCar::place(Vector3 pos, double yaw) {
    if (!v_ || !coll_ || load_state_ != 3) return false;
    ec::V3 l(pos.x, -pos.z, pos.y);
    stop_ = true;
    if (sim_.joinable()) sim_.join();
    stop_ = false;
    {
        std::lock_guard<std::mutex> g(mu_);
        v_->place(*coll_, l.x, l.y, l.z + 1.0, yaw, 0);
        take_snapshot();
    }
    placed_ = true;
    sim_ = std::thread([this] { sim_loop(); });
    return true;
}

void EarthJCar::reset() {
    if (!placed_) return;
    ec::V3 f = snap_.fwd;
    place(conv(snap_.pos), std::atan2(f.y, f.x));
}

void EarthJCar::sim_loop() {
    using clk = std::chrono::steady_clock;
    auto last = clk::now();
    double acc = 0;
    const double DT = 1.0 / 400;
    while (!stop_) {
        auto now = clk::now();
        acc += std::chrono::duration<double>(now - last).count();
        last = now;
        acc = std::min(acc, 0.1);  // never try to catch up more than 0.1 s (slow machine: slow motion)
        if (acc < DT) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        ec::CarInput in;
        {
            std::lock_guard<std::mutex> g(in_mu_);
            in = input_;
        }
        int cx = (int)std::floor(snap_.pos.x / ec::CHUNK), cy = (int)std::floor(snap_.pos.y / ec::CHUNK);
        if (!coll_->has(cx, cy)) {
            acc = 0;
            continue;
        }
        std::unique_lock<std::mutex> g(mu_);
        int steps = 0;
        while (acc >= DT && steps < 8) {
            v_->step(DT, in, *coll_);
            acc -= DT;
            steps++;
        }
        if (steps == 8) acc = 0;  // behind real time: drop the backlog rather than lock up
        total_steps_ += steps;
        if (getenv("EARTHSIM_DEBUG_JCAR") && total_steps_ % 400 < (uint64_t)steps)
            printf("jcar: steps %llu speed %.1f thr %.2f gear %d rpm %.0f z %.2f contacts %d%d%d%d\n", (unsigned long long)total_steps_, v_->forward_speed() * 3.6, in.throttle, v_->drive.gear, v_->drive.rpm(), v_->pos.z,
                   v_->wheels.size() > 0 && v_->wheels[0].st.contact, v_->wheels.size() > 1 && v_->wheels[1].st.contact, v_->wheels.size() > 2 && v_->wheels[2].st.contact, v_->wheels.size() > 3 && v_->wheels[3].st.contact);
        if (v_->body.diverged) {
            ec::V3 f = v_->fwd;
            v_->place(*coll_, v_->pos.x, v_->pos.y, v_->pos.z + 1, std::atan2(f.y, f.x), 0);
        }
        take_snapshot();
    }
}

void EarthJCar::take_snapshot() {
    ec::JVehicle &v = *v_;
    Snap s;
    s.p.reserve(v.body.nodes.size());
    for (auto &n : v.body.nodes) s.p.push_back(n.p);
    s.pos = v.pos, s.fwd = v.fwd, s.left = v.left, s.up = v.up;
    v.rotation(s.cf, s.cl, s.cu);
    v.eye_pose(s.eye);
    for (size_t i = 0; i < v.wheels.size(); i++) {
        s.wc.push_back(v.wheel_center((int)i));
        s.wa.push_back(v.wheel_axis((int)i));
        s.spin.push_back(v.wheels[i].st.spin);
    }
    s.steer = v.steer_cmd, s.speed = v.forward_speed(), s.rpm = v.drive.rpm(), s.gear = v.drive.gear;
    // damage measure every ~0.5 s (it walks all nodes)
    if (++dmg_count_ % 40 == 0) last_dmg_ = v.damage();
    s.dmg = last_dmg_;
    std::lock_guard<std::mutex> g(in_mu_);
    snap_ = std::move(s);
}

void EarthJCar::set_auto_input(double t, double s, double b) {
    auto_ = true;
    auto_t_ = t, auto_s_ = s, auto_b_ = b;
}

void EarthJCar::_physics_process(double dt) {
    if (!placed_) return;
    Input *in = Input::get_singleton();
    ec::CarInput ci;
    double v = snap_.speed;
    if (auto_) {
        ci.throttle = std::abs(auto_t_), ci.reverse = auto_t_ < 0, ci.brake = auto_b_, ci.steer = auto_s_ * driver_.steer_max;
    } else {
        bool up = in->is_key_pressed(KEY_W) || in->is_key_pressed(KEY_UP), dn = in->is_key_pressed(KEY_S) || in->is_key_pressed(KEY_DOWN);
        bool lf = in->is_key_pressed(KEY_A) || in->is_key_pressed(KEY_LEFT), rt = in->is_key_pressed(KEY_D) || in->is_key_pressed(KEY_RIGHT);
        bool hb = in->is_key_pressed(KEY_SPACE);
        double ax = in->get_joy_axis(0, JOY_AXIS_LEFT_X), r2 = in->get_joy_axis(0, JOY_AXIS_TRIGGER_RIGHT), l2 = in->get_joy_axis(0, JOY_AXIS_TRIGGER_LEFT);
        if (std::abs(ax) > 0.08 || r2 > 0.05 || l2 > 0.05) ci = driver_.update_analog(std::abs(ax) > 0.08 ? -ax : 0.0, r2, l2, hb, v);
        else ci = driver_.update(dt, up, dn, lf, rt, hb, v);
    }
    std::lock_guard<std::mutex> g(in_mu_);
    input_ = ci;
}

void EarthJCar::_process(double dt) {
    if (!placed_ || !skel_) return;
    Snap s;
    {
        std::lock_guard<std::mutex> g(in_mu_);
        s = snap_;
    }
    if (s.p.empty()) return;
    // node frame: rigid chassis motion
    ec::V3 o = s.pos - (s.cf * rest_com_.x + s.cl * rest_com_.y + s.cu * rest_com_.z);
    Transform3D t(frame_basis(s.cf, s.cl, s.cu), conv(o));
    set_global_transform(t);
    Transform3D inv = t.affine_inverse();
    for (size_t i = 0; i < s.p.size(); i++) skel_->set_bone_pose_position((int)i, inv.xform(conv(s.p[i])));
    for (size_t k = 0; k < wheel_mi_.size(); k++) {
        int wi = wheel_of_mi_[k];
        if (wi >= (int)s.wc.size()) continue;
        ec::V3 y = s.wa[wi], x = y.cross(s.up).norm(), z = x.cross(y);
        double sp = s.spin[wi], c = std::cos(sp), sn = std::sin(sp);
        ec::V3 xr = x * c - z * sn, zr = x * sn + z * c;
        Transform3D wt(frame_basis(xr, y, zr), conv(s.wc[wi]));
        wheel_mi_[k]->set_transform(inv * wt);
    }
    if (!props_.empty() && v_) {
        // the steering wheel moves with the rest shape (its reference nodes sway too much on their own)
        std::vector<ec::V3> rest;
        rest.reserve(v_->body.nodes.size());
        for (auto &n : v_->body.nodes) rest.push_back(n.rest);
        for (size_t k = 0; k < props_.size() && k < vis_->props.size(); k++) {
            ec::V3 po, cols[3];
            vis_->props[k].pose(rest, s.steer, po, cols);
            Basis b;
            b.set_column(0, conv(cols[0]));
            b.set_column(1, conv(cols[2]));
            b.set_column(2, -conv(cols[1]));
            props_[k]->set_transform(Transform3D(b, conv(po)));
        }
    }
}

double EarthJCar::speed_kmh() const { return std::abs(snap_.speed) * 3.6; }
double EarthJCar::rpm() const { return snap_.rpm; }
int EarthJCar::gear() const { return snap_.gear; }
double EarthJCar::damage() const { return snap_.dmg; }
Vector3 EarthJCar::forward() const { return conv(snap_.fwd); }
Vector3 EarthJCar::eye() const {
    // eye in this node's frame
    Transform3D t = get_global_transform();
    return t.affine_inverse().xform(conv(snap_.eye));
}

void EarthJCar::_bind_methods() {
    ClassDB::bind_static_method("EarthJCar", D_METHOD("find_vehicles", "dirs"), &EarthJCar::find_vehicles);
    ClassDB::bind_static_method("EarthJCar", D_METHOD("beamng_vehicles_dir"), &EarthJCar::beamng_vehicles_dir);
    ClassDB::bind_method(D_METHOD("load", "stream", "path"), &EarthJCar::load);
    ClassDB::bind_method(D_METHOD("poll"), &EarthJCar::poll);
    ClassDB::bind_method(D_METHOD("notes"), &EarthJCar::notes);
    ClassDB::bind_method(D_METHOD("place", "pos", "yaw"), &EarthJCar::place);
    ClassDB::bind_method(D_METHOD("reset"), &EarthJCar::reset);
    ClassDB::bind_method(D_METHOD("speed_kmh"), &EarthJCar::speed_kmh);
    ClassDB::bind_method(D_METHOD("rpm"), &EarthJCar::rpm);
    ClassDB::bind_method(D_METHOD("gear"), &EarthJCar::gear);
    ClassDB::bind_method(D_METHOD("damage"), &EarthJCar::damage);
    ClassDB::bind_method(D_METHOD("is_placed"), &EarthJCar::is_placed);
    ClassDB::bind_method(D_METHOD("forward"), &EarthJCar::forward);
    ClassDB::bind_method(D_METHOD("eye"), &EarthJCar::eye);
    ClassDB::bind_method(D_METHOD("set_auto_input", "throttle", "steer", "brake"), &EarthJCar::set_auto_input);
}
