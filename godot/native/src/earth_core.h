#pragma once
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>

namespace godot {
class EarthCore : public RefCounted {
    GDCLASS(EarthCore, RefCounted)
protected:
    static void _bind_methods();
public:
    static String version();
};
}
