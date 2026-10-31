// Node-beam soft body: point masses joined by springs that bend for good (plastic yield), crack under
// repeated load (fatigue) and break. Collisions are per node against contact planes supplied by the
// engine (static world), a ground function and the nodes of other bodies.
#pragma once
#include <functional>
#include <string>
#include <vector>
#include "ttmath.hpp"

namespace tt {

struct Node {
    V3 p, v, f;       // position, velocity, force accumulator (world, metres)
    V3 rest;          // position in the body frame at build time
    double m = 10, inv_m = 0.1;
    double radius = 0.08;   // collision radius
    double friction = 0.7;
    int part = -1;          // owning part (-1 = structure)
    bool collide = true;
};

enum BeamKind : uint8_t { BEAM_NORMAL = 0, BEAM_ATTACH = 1, BEAM_SUPPORT = 2 /*compression only*/ };

struct Beam {
    int a = 0, b = 0;
    double L0 = 1, L0_build = 1;  // current rest length (changes with plastic deformation), as built
    double k = 1e6, c = 500;      // stiffness N/m, damping N s/m
    double yield = 0.05;          // strain where plastic deformation starts
    double brk = 0.35;            // strain where the beam breaks
    double fatigue = 0;           // 0..1 accumulated damage from cycles above the endurance limit
    double plastic = 0;           // accumulated plastic strain (for damage reports)
    int part = -1;
    BeamKind kind = BEAM_NORMAL;
    bool broken = false;
    float strain = 0;             // last strain (for sounds/visuals)
};

struct ContactPlane {
    V3 p, n;             // a point on the surface and its outward normal
    double friction = 0.9;
    int surface = 0;     // engine surface id (asphalt, grass, ...)
};

struct GroundHit {
    double h = -1e9;
    V3 n{0, 0, 1};
    double friction = 1.0;
    int surface = 0;
};
using GroundFn = std::function<GroundHit(double x, double y)>;

struct Part {
    std::string name;
    std::vector<int> nodes, beams, attach;  // attach: beams that hold the part on the car
    double health = 1;                      // 1 intact .. 0 destroyed (from plastic strain)
    bool detached = false;
};

struct BodyStats {
    double deformation_energy = 0;  // J dissipated in plastic deformation
    int broken_beams = 0;
    double impact_peak = 0;         // largest contact impulse in the last step (N s)
};

class SoftBody {
  public:
    std::vector<Node> nodes;
    std::vector<Beam> beams;
    std::vector<Part> parts;
    std::vector<std::vector<ContactPlane>> contacts;  // per node, refreshed by the engine at its own rate
    BodyStats stats;
    double contact_k = 2.0e6, contact_c = 6.0e3;
    V3 gravity{0, 0, -9.81};

    int add_node(V3 p, double mass, int part = -1);
    int add_beam(int a, int b, double k, double c, double yield, double brk, int part = -1, BeamKind kind = BEAM_NORMAL);
    void finalize();  // rest positions, per-node contact lists

    // one integration substep (call at ~2 kHz): forces were accumulated by the caller (tyres,
    // suspension, engine), this adds beams, gravity, contacts and integrates
    void step(double dt, const GroundFn &ground);

    // node-node collisions between two bodies (call once per substep for close pairs)
    static void collide(SoftBody &a, SoftBody &b, double dt);

    V3 centre_of_mass() const;
    V3 velocity() const;
    double mass() const;
    void translate(const V3 &d);
    void update_parts();  // part health, detachment
    void repair_all();    // straighten everything (for respawn / full rebuild)
    void bounds(V3 &mn, V3 &mx) const;

  private:
    void beam_forces(double dt);
    void contact_forces(double dt, const GroundFn &ground);
};

}  // namespace tt
