// The canonical road model: a graph of drivable roads with per-edge attributes and a vertical
// profile. Every other road product (meshes, collision, terrain shaping, traffic) derives from it.
#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "region.hpp"

namespace es {

struct RoadNode {
    V2 p;
    double ground = 0;  // smoothed terrain at the node
    double off = 0;     // vertical offset from structures (bridges +, tunnels -)
    double z = 0;       // road surface height at the node
    std::vector<int> edges;
    bool signals = false;  // traffic signals mapped near the node
};

struct RoadEdge {
    int a = -1, b = -1;
    std::vector<V2> pts;  // centreline a -> b
    double len = 0;
    std::string cls, name, surface;
    int prio = 0;          // 10 motorway ... 1 track
    double width = 7;      // carriageway width, m
    int lanes = 2;         // total lanes (both directions)
    int oneway = 0;        // 1 a->b, -1 b->a
    int speed = 0;         // km/h (from data or default)
    bool bridge = false, tunnel = false;
    int level = 0;
    bool sidewalk = false; // raised kerb + pavement on at least one side
    bool sw_l = false, sw_r = false;  // per side (left/right of a->b); none on the median side of a dual carriageway
    double sw_width = 0;   // pavement width
    double sw(double side) const { return (side > 0 ? sw_l : sw_r) ? sw_width : 0.0; }
    double trim_a = 0, trim_b = 0;  // distance cut off at each end by the junction polygon
    // vertical profile: z at arc length s (samples every <= 4 m, first at 0, last at len)
    std::vector<double> ps, pz;
    double z_at(double s) const;
    V2 at(double s, V2 *dir = nullptr) const { return polyline_at(pts, s, dir); }
    double half() const { return width * 0.5; }
};

struct RoadHit {
    int edge = -1;
    double s = 0;     // arc position of the closest centreline point
    double d = 1e9;   // distance to the centreline
    double side = 0;  // signed lateral offset (+ left of a->b)
};

struct RoadNet {
    std::vector<RoadNode> nodes;
    std::vector<RoadEdge> edges;
    std::vector<std::string> log;
    double grade = 0.08;  // ramp grade used to approach bridges/tunnels

    void build(const Region &R);
    // nearest edge centreline within maxd (spatial hash); returns edge=-1 if none
    RoadHit nearest(V2 p, double maxd) const;
    // all edges whose centreline passes within maxd of p
    void query(V2 p, double maxd, std::vector<int> &out) const;
    void query_rect(const Rect &r, std::vector<int> &out) const;
    // Point `ahead` metres further along the road the vehicle at p (heading h) is on, keeping to the
    // right-hand lane; at junctions the straightest allowed continuation is taken. Returns false if
    // no road is near. `speed_limit` receives the limit of the edge at the target (km/h).
    bool follow(V2 p, V2 h, double ahead, V2 &target, int *speed_limit = nullptr, V2 *centre = nullptr, double *width = nullptr) const;

  private:
    double cell = 32;
    std::unordered_map<uint64_t, std::vector<int>> grid;
    uint64_t key(int x, int y) const { return (uint64_t(uint32_t(x)) << 32) | uint32_t(y); }
    void index();
    void profiles(const Region &R);
    void medians();
    void merge_overlaps();
};

double road_default_width(const std::string &cls, const std::string &subclass, int oneway);
int road_prio(const std::string &cls);
bool road_drivable(const std::string &cls);

}  // namespace es
