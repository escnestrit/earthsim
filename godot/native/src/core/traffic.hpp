// Traffic: cars driving the streamed road graph around the player. Each car commits to a route some way ahead,
// steers along it like a driver (pure pursuit of a point ahead in its lane, turning rate limited by speed, so
// corners are driven as arcs), plans its speed from the bends ahead (comfortable lateral and braking
// acceleration), changes lanes gradually and keeps a time gap to whatever is in front, the player included.
#pragma once
#include <deque>
#include <random>
#include <unordered_map>
#include "chunk.hpp"

namespace ec {

struct TrafficCar {
    std::deque<std::pair<int, bool>> route;  // (edge, travelling a->b); front is the current edge
    double s = 0;                            // metres along the current edge in the travel direction
    double v = 0, vmax = 12;
    double lane = 1.5, lane_target = 1.5;    // offset to the right of the centre line
    double aggr = 1;                         // driver temperament: speed and gap factor
    Col col;
    V3 pos;
    double yaw = 0, pitch = 0;
    int edge = -1;  // = route.front().first (for callers)
    int id = 0;
    int resets = 0;
};

class Traffic {
public:
    int target = 40;
    int total_resets = 0;  // cars put back on their path (diagnostics)
    double radius = 380;
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
    int next_id_ = 1;
    double rnd() { return std::uniform_real_distribution<double>(0, 1)(rng_); }
    bool spawn(V3 player);
    bool extend(TrafficCar &c);  // add one edge to the route; false at a dead end
    // point on the route `ahead` metres past the car's s, offset `lane` to the right; also its direction
    V3 route_point(const TrafficCar &c, double ahead, double lane, V2 *dir = nullptr) const;
    double lane_for(const Edge &e, int idx, int lanes_dir) const;
};

}  // namespace ec
