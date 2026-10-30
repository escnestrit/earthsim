// Node-beam soft-body solver (the kind of model BeamNG.drive uses): point masses joined by spring-damper
// beams that can yield (permanent deformation) and break, with contacts against the ground surface and the
// static world's triangles. Semi-implicit Euler with small fixed steps (0.25 ms typical).
#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
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

// A few worker threads that run one job split in parts, many times a second (every physics step): they spin
// briefly between jobs and sleep when idle.
class WorkPool {
public:
    explicit WorkPool(int workers);
    ~WorkPool();
    int parts() const { return (int)th_.size() + 1; }
    // runs fn(part) for part 0..parts()-1, the caller doing part 0; returns when all are done
    void run(const std::function<void(int)> &fn);

private:
    std::vector<std::thread> th_;
    const std::function<void(int)> *fn_ = nullptr;
    std::atomic<uint64_t> gen_{0};
    std::atomic<int> left_{0};
    std::atomic<bool> stop_{false};
    std::mutex m_;
    std::condition_variable cv_;
    void loop(int part);
};

class SoftBody {
public:
    // beam forces on several threads (big vehicles); 0 = single-threaded
    void set_threads(int n);
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
    std::unique_ptr<WorkPool> pool_;
    std::vector<std::vector<V3>> pf_;   // per-part force accumulators
    std::vector<double> pplastic_;
    void beam_range(size_t b0, size_t b1, double dt, V3 *F, double &plastic);
    struct Hot {
        V3 p, v;
        double m;
    };
    std::vector<Hot> hot_;
    // ground under each node, refreshed when the node has moved or after a while
    struct GroundCache {
        double x = 1e30, y = 1e30, zq = 0;
        Surface s;
        int age = 0;
    };
    std::vector<GroundCache> gc_;
    std::vector<Hit> hits_;
};

}  // namespace ec
