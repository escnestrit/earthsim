#pragma once
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <memory>
#include "core/streamer.hpp"

namespace godot {

// The world streamer for scripts. Coordinates: Godot's (x, y up, z south) metres; the core's local frame is
// (x east, y north, z up) with x_godot = x, y_godot = z, z_godot = -y.
class EarthStream : public RefCounted {
    GDCLASS(EarthStream, RefCounted)
protected:
    static void _bind_methods();

public:
    ~EarthStream() override;
    void start(double lat, double lon, String cache_dir, bool online, double detail_radius, double terrain_radius, bool enrich);
    void stop();
    void update(Vector3 pos);
    Dictionary poll();
    Array take_drops();
    double ground(double x, double z);
    bool wait_ground(double x, double z, double timeout);
    String status();
    Vector2 latlon(Vector3 pos);
    Vector3 to_godot(double lat, double lon);
    ec::Streamer *core() { return s_.get(); }
    bool collision_ready(Vector3 pos);
    // {pos: Vector3, yaw: float} of the nearest road within radius, or empty
    Dictionary nearest_road(Vector3 pos, double radius);

private:
    std::unique_ptr<ec::Streamer> s_;
};

}  // namespace godot
