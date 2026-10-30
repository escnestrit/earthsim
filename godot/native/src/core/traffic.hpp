// Traffic: cars driving the streamed road graph around the player, keeping to their lane, following the
// road's slope, slowing for turns, queueing behind each other and behind the player.
#pragma once
#include <random>
#include <unordered_map>
#include "chunk.hpp"

namespace ec {

struct TrafficCar {
    int edge = -1;
    bool fwd = true;    // travelling a -> b of the edge
    double s = 0;       // metres along the edge in the travel direction
    double v = 0, vmax = 12;
    double lane = 1.5;  // offset to the right of the centre line
    Col col;
    V3 pos;
    double yaw = 0, pitch = 0;
    int prev_node = -1;
};

class Traffic {
public:
    int target = 40;
    double radius = 380;
    // rebuild the graph from the loaded road pieces
    void set_roads(const std::vector<LaneSeg> &segs);
    void step(double dt, V3 player, V3 player_vel);
    const std::vector<TrafficCar> &cars() const { return cars_; }

private:
    struct Edge {
        int a, b;
        V3 pa, pb;
        double len, hw;
        int oneway, prio, lanes;
    };
    std::vector<Edge> edges_;
    std::vector<std::vector<int>> node_edges_;
    std::vector<TrafficCar> cars_;
    std::mt19937 rng_{12345};
    double rnd() { return std::uniform_real_distribution<double>(0, 1)(rng_); }
    bool spawn(V3 player);
    void place(TrafficCar &c);
    int next_edge(const TrafficCar &c, int node);
    V3 dir_of(const TrafficCar &c) const;
};

}  // namespace ec
