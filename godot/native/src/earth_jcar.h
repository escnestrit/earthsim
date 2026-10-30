#pragma once
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/skeleton3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include "earth_stream.h"
#include "jbeam/jvisual.hpp"

namespace godot {

// A BeamNG vehicle (the user's own, from their BeamNG.drive install or a mods folder) driven by our soft-body
// solver on its own thread. Same driving interface as EarthCar.
class EarthJCar : public Node3D {
    GDCLASS(EarthJCar, Node3D)
protected:
    static void _bind_methods();

public:
    ~EarthJCar() override;
    // vehicle zips found: [{path, name}]; dirs: extra folders to scan (e.g. a mods folder next to the exe)
    static Array find_vehicles(PackedStringArray dirs);
    static String beamng_vehicles_dir();
    // start loading in the background; poll() until it returns "ready" or "error: ..."
    // config: a .pc name from configs(), "" for the vehicle's default
    void load(Ref<EarthStream> stream, String path, String config = "");
    static PackedStringArray configs(String path);
    String poll();
    String notes() const;
    bool place(Vector3 pos, double yaw);
    void reset();
    void _process(double dt) override;
    void _physics_process(double dt) override;

    double speed_kmh() const;
    double rpm() const;
    int gear() const;
    double damage() const;
    bool is_placed() const { return placed_; }
    Vector3 forward() const;
    Vector3 eye() const;
    void set_auto_input(double throttle, double steer, double brake);

private:
    Ref<EarthStream> stream_;
    std::shared_ptr<ec::CollisionWorld> coll_;
    std::unique_ptr<ec::JVehicle> v_;
    std::unique_ptr<ec::CarVisual> vis_;
    std::thread loader_, sim_;
    std::atomic<int> load_state_{0};  // 0 idle, 1 loading, 2 loaded (not built), 3 ready, -1 error
    std::string load_err_, load_path_, load_config_;
    std::vector<std::string> notes_;
    std::atomic<bool> stop_{false}, placed_{false};
    mutable std::mutex mu_;      // guards the vehicle while the sim thread steps it
    std::mutex in_mu_;
    ec::CarInput input_;
    ec::Driver driver_;
    bool auto_ = false;
    double auto_t_ = 0, auto_s_ = 0, auto_b_ = 0;
    // snapshot for rendering
    struct Snap {
        std::vector<ec::V3> p;
        ec::V3 pos, cf, cl, cu, fwd, left, up, eye;
        std::vector<ec::V3> wc, wa;
        std::vector<double> spin;
        double steer = 0, speed = 0, rpm = 0, dmg = 0;
        int gear = 1;
        bool intact = true;
    } snap_;
    ec::V3 rest_com_;
    // per node: neighbours by beam and their rest offsets (Godot axes), for the bones' rotations
    std::vector<std::vector<std::pair<int, Vector3>>> nbr_;
    Skeleton3D *skel_ = nullptr;
    std::vector<MeshInstance3D *> wheel_mi_;
    std::vector<int> wheel_of_mi_;
    std::vector<Node3D *> props_;
    double dmg_timer_ = 0, last_dmg_ = 0;
    int dmg_count_ = 0;
    uint64_t total_steps_ = 0;
    void build_nodes();
    void sim_loop();
    void take_snapshot();
    Ref<StandardMaterial3D> material(const std::string &name, std::map<std::string, Ref<StandardMaterial3D>> &cache, std::map<std::string, Ref<Texture2D>> &tex);
};

}  // namespace godot
