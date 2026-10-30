// Pedestrians walking the sidewalks and paths around the player.
#pragma once
#include <random>
#include "chunk.hpp"

namespace ec {

struct Walker {
    int edge = -1;
    bool fwd = true;
    double s = 0, v = 1.3, side = 0;
    V3 pos;
    double yaw = 0, phase = 0, scale = 1;
    Col col;
};

class Crowd {
public:
    int target = 50;
    double radius = 220;
    void set_walks(const std::vector<std::pair<V3, V3>> &segs);
    void step(double dt, V3 player);
    const std::vector<Walker> &walkers() const { return w_; }

private:
    struct Edge {
        int a, b;
        V3 pa, pb;
        double len;
    };
    std::vector<Edge> e_;
    std::vector<std::vector<int>> ne_;
    std::vector<Walker> w_;
    std::mt19937 rng_{777};
    double rnd() { return std::uniform_real_distribution<double>(0, 1)(rng_); }
    void place(Walker &w);
};

}  // namespace ec
