#include "earth_car.h"
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/input.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/skin.hpp>
#include <algorithm>

using namespace godot;

namespace {
Vector3 conv(ec::V3 v) { return Vector3((real_t)v.x, (real_t)v.z, (real_t)-v.y); }
ec::V3 unconv(Vector3 g) { return ec::V3(g.x, -g.z, g.y); }
// the model is centred between its axles; the lattice's axles are 0.1 m behind the origin
const double MODEL_DX = -0.1;
}  // namespace

void EarthCar::setup(Ref<EarthStream> stream, Dictionary body, Dictionary materials, Ref<Mesh> wheel_mesh) {
    stream_ = stream;
    coll_ = stream->core() ? stream->core()->collision() : nullptr;
    ec::CarParams p;
    car_ = std::make_unique<ec::SoftCar>(p);
    driver_.steer_max = p.steer_max;
    // skeleton: one bone per chassis node
    skel_ = memnew(Skeleton3D);
    skel_->set_name("Skeleton");
    add_child(skel_);
    bone_node_.clear();
    std::vector<ec::V3> rest;
    for (int i : car_->cage) {
        int b = skel_->add_bone(String("n") + String::num_int64(i));
        Transform3D t(Basis(), conv(car_->body.nodes[i].rest));
        skel_->set_bone_rest(b, t);
        skel_->set_bone_pose_position(b, t.origin);
        bone_node_.push_back(i);
        rest.push_back(car_->body.nodes[i].rest);
    }
    Ref<Skin> skin;
    skin.instantiate();
    for (int b = 0; b < (int)bone_node_.size(); b++) skin->add_bind(b, Transform3D(Basis(), conv(rest[b])).affine_inverse());
    // body mesh with bone weights: the 4 nearest chassis nodes, inverse-square weighted
    Ref<ArrayMesh> am;
    am.instantiate();
    Array keys = body.keys();
    for (int k = 0; k < keys.size(); k++) {
        Array a = body[keys[k]];
        PackedVector3Array pos = a[Mesh::ARRAY_VERTEX];
        PackedInt32Array bones;
        PackedFloat32Array weights;
        bones.resize(pos.size() * 4);
        weights.resize(pos.size() * 4);
        for (int64_t v = 0; v < pos.size(); v++) {
            Vector3 g = pos[v];
            g.x += (real_t)MODEL_DX;
            pos.set(v, g);
            ec::V3 r = unconv(g);
            std::pair<double, int> best[4] = {{1e9, 0}, {1e9, 0}, {1e9, 0}, {1e9, 0}};
            for (int b = 0; b < (int)rest.size(); b++) {
                double d = (rest[b] - r).len();
                if (d < best[3].first) {
                    best[3] = {d, b};
                    std::sort(best, best + 4);
                }
            }
            double ws[4], sum = 0;
            for (int q = 0; q < 4; q++) ws[q] = 1.0 / std::pow(best[q].first + 0.08, 2), sum += ws[q];
            for (int q = 0; q < 4; q++) {
                bones.set(v * 4 + q, best[q].second);
                weights.set(v * 4 + q, (float)(ws[q] / sum));
            }
        }
        a[Mesh::ARRAY_VERTEX] = pos;
        a[Mesh::ARRAY_BONES] = bones;
        a[Mesh::ARRAY_WEIGHTS] = weights;
        am->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a);
        Variant m = materials.get(keys[k], Variant());
        if (m.get_type() == Variant::OBJECT) am->surface_set_material(am->get_surface_count() - 1, m);
    }
    MeshInstance3D *mi = memnew(MeshInstance3D);
    mi->set_name("Body");
    mi->set_mesh(am);
    skel_->add_child(mi);
    mi->set_skin(skin);
    mi->set_skeleton_path(NodePath(".."));
    for (int i = 0; i < 4; i++) {
        wheels_[i] = memnew(MeshInstance3D);
        wheels_[i]->set_mesh(wheel_mesh);
        add_child(wheels_[i]);
    }
}

bool EarthCar::place(Vector3 pos, double yaw) {
    if (!car_ || !coll_) return false;
    ec::V3 l = unconv(pos);
    // the surface at the given height (a road under a bridge stays the road); far above when no height given
    ec::Surface s = l.z > -1000 && pos.y != 0 ? coll_->surface(l.x, l.y, l.z + 1.0, 1.5) : coll_->surface(l.x, l.y, 1e4, 2e4);
    if (!s.found) return false;
    car_->place(*coll_, l.x, l.y, s.height + 0.5, yaw, 0);
    placed_ = true;
    last_yaw_ = yaw;
    sync_visual();
    return true;
}

void EarthCar::reset() {
    if (!car_ || !placed_) return;
    ec::V3 f = car_->fwd;
    double yaw = std::atan2(f.y, f.x);
    Vector3 p = conv(car_->pos);
    place(p, yaw);
}

void EarthCar::set_auto_input(double t, double s, double b) {
    auto_ = true;
    auto_t_ = t, auto_s_ = s, auto_b_ = b;
}

void EarthCar::_physics_process(double dt) {
    if (!car_ || !placed_ || !coll_) return;
    // the world under the car must be loaded; otherwise hold still
    int cx = (int)std::floor(car_->pos.x / ec::CHUNK), cy = (int)std::floor(car_->pos.y / ec::CHUNK);
    if (!coll_->has(cx, cy)) return;
    Input *in = Input::get_singleton();
    ec::CarInput ci;
    double v = car_->forward_speed();
    if (auto_) {
        ci.throttle = std::max(0.0, auto_t_), ci.brake = auto_b_, ci.steer = auto_s_ * driver_.steer_max;
        ci.reverse = auto_t_ < 0;
        if (ci.reverse) ci.throttle = -auto_t_;
    } else if (input_) {
        bool up = in->is_key_pressed(KEY_W) || in->is_key_pressed(KEY_UP);
        bool dn = in->is_key_pressed(KEY_S) || in->is_key_pressed(KEY_DOWN);
        bool lf = in->is_key_pressed(KEY_A) || in->is_key_pressed(KEY_LEFT);
        bool rt = in->is_key_pressed(KEY_D) || in->is_key_pressed(KEY_RIGHT);
        bool hb = in->is_key_pressed(KEY_SPACE);
        // a gamepad or wheel takes over while it is used
        double ax = in->get_joy_axis(0, JOY_AXIS_LEFT_X), rt2 = in->get_joy_axis(0, JOY_AXIS_TRIGGER_RIGHT), lt2 = in->get_joy_axis(0, JOY_AXIS_TRIGGER_LEFT);
        if (std::abs(ax) > 0.08 || rt2 > 0.05 || lt2 > 0.05)
            ci = driver_.update_analog(std::abs(ax) > 0.08 ? -ax : 0.0, rt2, lt2, hb || in->is_joy_button_pressed(0, JOY_BUTTON_A), v);
        else
            ci = driver_.update(dt, up, dn, lf, rt, hb, v);
    }
    car_->step(dt, ci, *coll_);
    if (car_->body.diverged) {
        reset();
        return;
    }
    sync_visual();
}

void EarthCar::sync_visual() {
    ec::SoftCar &c = *car_;
    Basis b;
    b.set_column(0, conv(c.fwd));
    b.set_column(1, conv(c.up));
    b.set_column(2, -conv(c.left));
    Transform3D t(b, conv(c.to_world(ec::V3(0, 0, 0))));
    set_global_transform(t);
    Transform3D inv = t.affine_inverse();
    for (int k = 0; k < (int)bone_node_.size(); k++) skel_->set_bone_pose_position(k, inv.xform(conv(c.body.nodes[bone_node_[k]].p)));
    for (int i = 0; i < 4; i++) {
        Vector3 hp = inv.xform(conv(c.body.nodes[c.hubs[i]].p));
        Basis wb = Basis(Vector3(0, 1, 0), (real_t)c.wheels[i].steer) * Basis(Vector3(0, 0, 1), (real_t)-c.wheels[i].spin) * Basis(Vector3(1, 0, 0), (real_t)(Math_PI / 2));
        wheels_[i]->set_transform(Transform3D(wb, hp));
    }
}

double EarthCar::speed_kmh() const { return car_ ? car_->vel.len() * 3.6 : 0; }
double EarthCar::rpm() const { return car_ ? car_->drive.rpm() : 0; }
int EarthCar::gear() const { return car_ ? car_->drive.gear : 0; }
double EarthCar::damage() const { return car_ ? car_->damage() : 0; }
Vector3 EarthCar::forward() const { return car_ ? conv(car_->fwd) : Vector3(1, 0, 0); }

Dictionary EarthCar::telemetry() const {
    Dictionary d;
    if (!car_) return d;
    d["speed"] = car_->forward_speed();
    d["rpm"] = car_->drive.rpm();
    d["gear"] = car_->drive.gear;
    d["throttle"] = driver_.throttle;
    d["brake"] = driver_.brake;
    d["steer"] = driver_.steer;
    int broken = 0;
    d["damage"] = car_->damage(&broken);
    d["broken"] = broken;
    d["impact"] = car_->impact;
    Array slip;
    for (auto &w : car_->wheels) slip.push_back(Vector2((real_t)w.kappa, (real_t)w.tan_alpha));
    d["slip"] = slip;
    d["accel"] = Vector3((real_t)car_->accel.x, (real_t)car_->accel.y, (real_t)car_->accel.z);
    return d;
}

void EarthCar::_bind_methods() {
    ClassDB::bind_method(D_METHOD("setup", "stream", "body", "materials", "wheel_mesh"), &EarthCar::setup);
    ClassDB::bind_method(D_METHOD("place", "pos", "yaw"), &EarthCar::place);
    ClassDB::bind_method(D_METHOD("reset"), &EarthCar::reset);
    ClassDB::bind_method(D_METHOD("speed_kmh"), &EarthCar::speed_kmh);
    ClassDB::bind_method(D_METHOD("rpm"), &EarthCar::rpm);
    ClassDB::bind_method(D_METHOD("gear"), &EarthCar::gear);
    ClassDB::bind_method(D_METHOD("damage"), &EarthCar::damage);
    ClassDB::bind_method(D_METHOD("is_placed"), &EarthCar::is_placed);
    ClassDB::bind_method(D_METHOD("forward"), &EarthCar::forward);
    ClassDB::bind_method(D_METHOD("telemetry"), &EarthCar::telemetry);
    ClassDB::bind_method(D_METHOD("set_input_enabled", "enabled"), &EarthCar::set_input_enabled);
    ClassDB::bind_method(D_METHOD("set_auto_input", "throttle", "steer", "brake"), &EarthCar::set_auto_input);
}
