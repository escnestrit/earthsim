// Torque/BeamNG compiled shapes (.cdae / .dts, version 31): msgpack header, optionally zstd-compressed body
// of nodes, objects and meshes. Only the parts needed to get the meshes out.
#pragma once
#include <map>
#include "dae.hpp"

namespace ec {

struct Shape {
    std::map<std::string, std::vector<SubMesh>> objects;
    std::vector<std::string> materials;
    bool has(const std::string &n) const { return objects.count(n) > 0; }
};

bool parse_shape(const std::string &file, Shape &out, std::string &err);
// object names from the start of a file (header only); false if not a shape
bool shape_object_names(const std::string &prefix, std::vector<std::string> &out, size_t *header_len = nullptr);

}  // namespace ec
