// Godot binding of the world core. All coordinates exposed to Godot are in Godot's frame:
// X east, Y up, Z south, metres, origin at the region centre.
#pragma once
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/rect2i.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <memory>
#include "world.hpp"

namespace godot {

class EarthWorld : public RefCounted {
    GDCLASS(EarthWorld, RefCounted)
    std::unique_ptr<es::World> w;
    PackedStringArray log_;

  protected:
    static void _bind_methods();

  public:
    bool load(const String &dir);
    bool load_spec(const String &root, const String &spec);
    // map overlay: roads of a transportation vector tile as polylines in 0..256 tile pixels
    static Array map_tile_roads(const PackedByteArray &mvt);
    // map background: shaded relief RGB8 bytes (256x256) from a Terrarium PNG
    static PackedByteArray map_shade(const PackedByteArray &png, int z, int ty);
    bool is_loaded() const { return bool(w); }
    PackedStringArray get_log() const { return log_; }
    Rect2i chunk_range() const;
    double chunk_size() const { return es::CHUNK; }
    // thread-safe once loaded
    Dictionary build_chunk(int cx, int cy) const;
    Dictionary build_far(int fx, int fy) const;
    Rect2i far_range() const;
    double far_size() const { return es::FAR_CELL; }
    Dictionary spawn_near(const Vector3 &p) const;
    double ground_height(const Vector3 &p) const;
    Vector3 latlon_to_world(double lat, double lon) const;
    Vector2 world_to_latlon(const Vector3 &p) const;
    Dictionary road_at(const Vector3 &p) const;
    Dictionary follow(const Vector3 &p, const Vector3 &heading, double ahead) const;
    // traffic helpers (road graph access)
    Dictionary edge_info(int e) const;
    Dictionary lane_pose(int e, double s, int dir, double lane_off) const;
    Dictionary next_edge(int e, int dir, double rnd) const;
    Array traffic_spawns(const Vector3 &centre, double rmin, double rmax, int count, int seed) const;
    Dictionary region_info() const;
};

}  // namespace godot
