#pragma once
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>

namespace godot {
class EarthCore : public RefCounted {
    GDCLASS(EarthCore, RefCounted)
protected:
    static void _bind_methods();
public:
    static String version();
    // facade texture set of a style (0..12) and variant (0..2): albedo, orm, emission, normal (RGBA8), depth (R8)
    static Dictionary facade_texture(int style, int variant);
    // tileable material textures (see core/textures.hpp TexKind): albedo, normal, orm (RGBA8)
    static Dictionary material_texture(int kind);
    // a placed-object model: material id -> mesh arrays
    static Dictionary model(int kind, int variant);
};
}
