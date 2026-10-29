// Node-beam soft-body solver (the kind of model BeamNG.drive uses): point masses joined by spring-damper
// beams that can yield (permanent deformation) and break, with contacts against the ground surface and the
// static world's triangles. Semi-implicit Euler with small fixed steps (0.25 ms typical).
#pragma once
#include <vector>
#include "collide.hpp"

namespace ec {

struct SNode {
    V3 p, v, f, rest;
    double m = 1;
    double radius = 0;
    bool hit_ground = false, hit_world = false;
};

struct Stops {
    double min_len, max_len, k, c;
};

struct SBeam {
    int a, b;
    double rest, k, c, c_reb;
    double yield_t = 1e300, yield_c = 1e300;
    double break_force = 1e300, break_strain = 1e300;
    bool broken = false, support = false;
    double rest0 = 0;
    bool has_stops = false;
    Stops stops{};
};

struct Slide {
    int node;
    std::vector<int> rail;
    double k, c;
    bool capped;
};

struct Pin {
    int a, b;
    double k, c, strength;
    bool broken = false;
};

struct ContactParams {
    double k, c, mu;
};

class SoftBody {
public:
    std::vector<SNode> nodes;
    std::vector<SBeam> beams;
    std::vector<Slide> slides;
    std::vector<Pin> pins;
    V3 gravity{0, 0, -9.81};
    ContactParams ground{4.0e5, 3000, 0.5}, world{2.0e6, 6000, 0.3};
    double peak_contact = 0, peak_world = 0, plastic_work = 0;
    bool diverged = false;

    int add_node(V3 p, double m);
    int add_beam(int a, int b, double k, double c);
    double total_mass() const;
    V3 centre_of_mass() const;

    void clear_forces();
    void beam_forces(double dt);
    void slide_forces();
    // ground (terrain, road decks) as penalty springs with Coulomb friction
    void contacts(const CollisionWorld &w, double dt);
    void integrate(double dt);
    // walls, kerbs, poles: inelastic push-out after integration
    void resolve_world(const CollisionWorld &w, double dt);
    void step(double dt, const CollisionWorld &w);

private:
    std::vector<V3> f_;
    std::vector<Hit> hits_;
};

}  // namespace ec
