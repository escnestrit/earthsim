// BeamNG vehicle parts: a library of parts from .jbeam files, slot assembly from the main part with the
// defaults (and user overrides), variables resolved, then nodes and beams collected into one model.
#pragma once
#include <array>
#include <map>
#include <memory>
#include <unordered_map>
#include "jtable.hpp"
#include "zip.hpp"

namespace ec {

struct PartDef {
    std::string file;
    Json body;
};

struct Library {
    std::unordered_map<std::string, PartDef> parts;
    std::map<std::string, std::vector<std::string>> by_type;
    std::vector<std::string> warnings;
    void add_file(const std::string &file, const std::string &text);
    void add_zip(Zip &z, bool fallback_only);  // fallback_only: parts already present win
};

struct PartInstance {
    std::string name, slot;
    std::array<double, 3> offset{}, node_move{};
    Json body;
};

struct Assembly {
    std::vector<PartInstance> parts;
    std::map<std::string, Json> vars;
    std::vector<std::string> warnings;
    std::vector<std::pair<std::string, std::string>> missing;
};

Assembly assemble(const Library &lib, const std::string &root, const std::map<std::string, std::string> &overrides);

struct JNode {
    std::string name;
    int part = 0;
    std::array<double, 3> pos{};
    double mass = 25, friction = 1;
    bool collision = true, self_collision = false, fixed = false;
    std::vector<std::string> group;
    std::string material;
};

struct JBeam {
    int a, b, part;
    std::string kind;
    Row row;
    double spring() const { return row.num("beamSpring", 4300000); }
    double damp() const { return row.num("beamDamp", 580); }
    // < 0: no limit
    double deform() const;
    double strength() const;
};

// BeamNG (x right... -y forward) -> ours (x forward, y left, z up)
inline std::array<double, 3> to_ours(const std::array<double, 3> &p) { return {-p[1], p[0], p[2]}; }

struct JModel {
    Assembly asm_;
    std::vector<JNode> nodes;
    std::vector<JBeam> beams;
    std::unordered_map<std::string, int> node_index;
    std::vector<std::string> warnings;
    int skipped_beams = 0;
    std::vector<std::pair<int, Row>> table(const std::string &section) const;
    double total_mass() const;
};

JModel build_model(Assembly a);

}  // namespace ec
