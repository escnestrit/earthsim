// Collada (.dae) meshes as BeamNG ships them: named nodes with world matrices, triangles/polylists by material.
#pragma once
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ec {

struct SubMesh {
    std::string material;
    std::vector<std::array<float, 3>> pos, nrm;
    std::vector<std::array<float, 2>> uv;
    std::vector<uint32_t> idx;
};

struct DaeNode {
    std::string name, geometry;
    std::array<double, 16> matrix;
    std::vector<std::pair<std::string, std::string>> materials;  // symbol -> material id
};

class Dae {
public:
    bool parse(std::string text, std::string &err);
    const DaeNode *node(const std::string &name) const;
    bool mesh(const std::string &node_name, std::vector<SubMesh> &out, std::string &err) const;
    std::vector<DaeNode> nodes;

private:
    std::string text_;
    std::map<std::string, std::pair<size_t, size_t>> geoms_;
};

}  // namespace ec
