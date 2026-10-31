// Road surfaces derived from the RoadNet: junction polygons with filleted kerbs, carriageway ribbons,
// kerbs and pavements, bridge decks and tunnel boxes. Also answers "what hard surface is here and at
// what height", which drives terrain shaping, building placement and tests.
#pragma once
#include <vector>
#include "mesh.hpp"
#include "roadnet.hpp"

namespace es {

constexpr double KERB_H = 0.15;  // kerb height above the carriageway

struct JArm {
    int edge = -1;
    bool at_a = true;  // the edge starts at this junction
    double h = 3, swl = 0, swr = 0;  // pavement widths on the arm's left/right (looking away), 0 = none
    double trim = 0;
    V2 P, D;      // centre point at trim and direction away from the junction
    V3 L, R;      // left/right carriageway corners at trim (left = CCW side looking away)
    double ang = 0;
};

struct JCorner {
    std::vector<V3> kerb;   // carriageway boundary L_i ... R_j (inclusive)
    std::vector<V3> outer;  // pavement outer boundary, same count (empty without pavement)
};

struct Junction {
    int node = -1;
    std::vector<JArm> arms;      // CCW by angle
    std::vector<JCorner> corners;  // corners[i] between arms[i] and arms[i+1]
    std::vector<V3> outline;     // carriageway polygon, CCW
    V3 centre;
    bool flat = false;           // degenerate (no polygon needed)
    Rect box;
};

struct SurfaceHit {
    int kind = 0;        // 0 none, 1 carriageway (incl. 1 m margin), 2 pavement
    bool margin = false; // kind 1: the point is in the 1 m margin just outside the carriageway
    int edge = -1, node = -1;  // kind 1: which carriageway (edge ribbon or junction polygon)
    double z = 0;        // surface height (kind > 0)
    double dist = 1e9;   // kind 0: distance to the nearest hard surface
    double znear = 0;    // kind 0: height of that surface's edge
    bool bridge = false; // the nearest surface is a bridge deck (terrain only cut below it)
    bool tunnel = false; // the point is above a tunnel (terrain left alone unless cover is thin)
    double tunnel_z = -1e9;     // road height of a tunnel below the point
};

struct RoadGeom {
    const RoadNet *net = nullptr;
    std::vector<Junction> junctions;  // index = node index (empty for degree < 2)
    void build(RoadNet &N);           // sets edge trims too

    // geometry emission (meshes are in the local frame, origin subtracted)
    // with a cache, pavement pieces lying on another carriageway are dropped (carriageway wins)
    struct Cache;
    void emit_edge(int e, V2 origin, Mesh *mats, const Cache *c = nullptr) const;
    void emit_junction(int n, V2 origin, Mesh *mats, const Cache *c = nullptr) const;
    bool on_other_carriageway(V2 p, double z, int self_edge, int self_node, const Cache &c) const;

    // hard-surface query with candidate edges prefetched for an area
    struct Cache {
        std::vector<int> edges, nodes;
        // local buckets (16 m) over the prefetched rect: indices into edges / nodes
        Rect rect;
        int bw = 0, bh = 0;
        std::vector<std::vector<int>> be, bn;
        void bucket_range(const Rect &q, int &x0, int &y0, int &x1, int &y1) const;
    };
    void prefetch(const Rect &r, Cache &c) const;
    // zmax: surfaces higher than this are ignored (a bridge above the point)
    SurfaceHit query(V2 p, const Cache &c, double reach = 30, double zmax = 1e9) const;
    // height of the drivable/walkable surface at p (pavement, carriageway) or NAN if none
    double surface_z(V2 p) const;
};

}  // namespace es
