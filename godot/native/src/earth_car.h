#pragma once
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <memory>
#include "earth_stream.h"
#include "phys/softcar.hpp"

namespace godot {

// The player's soft-body car: steps the node-beam simulation against the streamed world's collision, skins the
// body mesh to the chassis lattice (crash deformation shows), places the wheels. The node's transform follows
// the chassis.
class EarthCar : public Node3D {
    GDCLASS(EarthCar, Node3D)
protected:
    static void _bind_methods();

public:
    // body: material id -> mesh arrays in the car's model frame (x forward, z up; as EarthCore.model returns)
    void setup(Ref<EarthStream> stream, Dictionary body, Dictionary materials, Ref<Mesh> wheel_mesh);
    // put the car on the ground at a Godot position, heading yaw (radians about +y, 0 = +x)
    bool place(Vector3 pos, double yaw);
    void _physics_process(double dt) override;
    void reset();

    double speed_kmh() const;
    double rpm() const;
    int gear() const;
    double damage() const;
    bool is_placed() const { return placed_; }
    Vector3 forward() const;
    // driver's eye point in the car's frame (for the cockpit camera)
    Vector3 eye() const;
    Dictionary telemetry() const;
    void set_input_enabled(bool e) { input_ = e; }
    void set_auto_input(double throttle, double steer, double brake);

private:
    Ref<EarthStream> stream_;
    std::shared_ptr<ec::CollisionWorld> coll_;
    std::unique_ptr<ec::SoftCar> car_;
    ec::Driver driver_;
    Skeleton3D *skel_ = nullptr;
    MeshInstance3D *wheels_[4] = {};
    Node3D *steering_ = nullptr;
    double last_steer_ = 0;
    std::vector<int> bone_node_;  // softbody node per bone
    bool placed_ = false, input_ = true;
    double auto_t_ = 0, auto_s_ = 0, auto_b_ = 0;
    bool auto_ = false;
    double last_yaw_ = 0;
    void sync_visual();
};

}  // namespace godot
