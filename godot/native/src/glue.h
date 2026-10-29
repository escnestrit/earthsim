// Conversions between the core's buffers and Godot types.
#pragma once
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include "core/mesh.hpp"

namespace godot {
// mesh arrays (Mesh::ARRAY_*) in Godot axes, front faces clockwise, with tangents
Array mesh_arrays(const ec::MeshBuf &m);
PackedByteArray bytes(const std::vector<uint8_t> &v);
}  // namespace godot
