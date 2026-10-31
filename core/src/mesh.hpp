// Triangle mesh buffers in the local frame (x east, y north, z up), converted to Godot on export.
#pragma once
#include <cstdint>
#include <vector>
#include "geo.hpp"

namespace es {

enum Mat : int {
    M_ASPHALT = 0,   // carriageway; uv = (lateral m from left edge, arc m), uv2 = (lanes code, width)
    M_JUNCTION,      // junction polygons (asphalt without lane marks)
    M_PAVEMENT,      // sidewalks, plazas; uv = world xy
    M_KERB,          // kerb stones, bridge sides, parapets
    M_CONCRETE,      // bridge decks underside, piers, tunnel walls, retaining walls
    M_TERRAIN,       // ground; col = surface class weights
    M_WALL,          // facades; uv = (along wall m, height above base m), uv2 = (floor height, style), col = facade colour
    M_ROOF,          // roofs; uv = world xy, col = roof colour
    M_WATER,
    M_PATH,          // footpaths
    M_RAIL,          // ballast + rails
    M_MARKING,       // painted markings on top of the carriageway: uv.y along, uv2.x = 0 zebra, 1 solid
    M_COUNT
};

struct Mesh {
    std::vector<float> pos, nrm, uv, uv2, col;  // 3,3,2,2,4 floats per vertex
    std::vector<int32_t> idx;
    size_t verts() const { return pos.size() / 3; }
    int vert(V3 p, V3 n, V2 uv_, V2 uv2_ = {0, 0}, float r = 1, float g = 1, float b = 1, float a = 1) {
        pos.push_back(float(p.x)), pos.push_back(float(p.y)), pos.push_back(float(p.z));
        nrm.push_back(float(n.x)), nrm.push_back(float(n.y)), nrm.push_back(float(n.z));
        uv.push_back(float(uv_.x)), uv.push_back(float(uv_.y));
        uv2.push_back(float(uv2_.x)), uv2.push_back(float(uv2_.y));
        col.push_back(r), col.push_back(g), col.push_back(b), col.push_back(a);
        return int(verts() - 1);
    }
    void tri(int a, int b, int c) { idx.push_back(a), idx.push_back(b), idx.push_back(c); }
    void quad(int a, int b, int c, int d) { tri(a, b, c), tri(a, c, d); }  // a,b,c,d counter-clockwise seen from the front
    void clear() { pos.clear(), nrm.clear(), uv.clear(), uv2.clear(), col.clear(), idx.clear(); }
    // recompute smooth normals from triangles (area weighted)
    void smooth_normals();
};

// A flat-shaded quad between 4 points, CCW from the front; normal computed.
void add_quad(Mesh &m, V3 a, V3 b, V3 c, V3 d, V2 ua, V2 ub, V2 uc, V2 ud, V2 uv2 = {0, 0}, float r = 1, float g = 1, float b_ = 1, float al = 1);

}  // namespace es
