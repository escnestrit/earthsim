// World = region data + road model + derived geometry, built chunk by chunk for streaming.
#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "mesh.hpp"
#include "region.hpp"
#include "roadgeom.hpp"
#include "roadnet.hpp"

namespace es {

constexpr double CHUNK = 256.0;  // chunk size, m (metric grid in the local frame)
constexpr double TCELL = 4.0;    // terrain grid spacing, m
constexpr int TN = int(CHUNK / TCELL);  // cells per chunk side
constexpr double FAR_CELL = 1024.0;     // far LOD cell size, m

struct Instance {
    V3 p;          // local frame, origin subtracted
    double yaw = 0;  // radians, rotation about up; 0 = facing local +y (north)
    double scale = 1;
    int variant = 0;
};

enum InstKind : int { I_TREE = 0, I_LAMP, I_SIGNAL, I_BENCH, I_STOP, I_SIGN, I_COUNT };

struct Building {
    int src = -1;          // index into region.buildings
    int parent = -1;       // building index of the parent (for parts)
    bool render = true;
    V2 centroid;
    Rect box;
    double area = 0;
    double height = 10, min_h = 0, roof_h = 0, floor_h = 3;
    int style = 0;
    double base = 0;       // base level (set when the chunk is built; parts use the parent's)
    bool base_set = false;
    uint32_t facade = 0, roof = 0;
    std::string roof_shape;
};

struct ChunkData {
    int cx = 0, cy = 0;
    V2 origin;  // local frame origin of this chunk's geometry (chunk centre)
    Mesh mats[M_COUNT];
    // collision triangle soups (local, origin subtracted), by surface group
    std::vector<float> col_road, col_ground, col_solid;
    std::vector<Instance> inst[I_COUNT];
    // terrain grid heights (TN+1)^2, row-major from south-west, for queries
    std::vector<float> ground;
    std::vector<uint8_t> hole;  // per grid cell (TN*TN): 1 = no terrain triangles
    std::vector<std::string> notes;
    int stats_buildings = 0, stats_edges = 0;
};

struct World {
    Region region;
    RoadNet net;
    RoadGeom geom;
    std::vector<Building> buildings;
    std::unordered_map<uint64_t, std::vector<int>> chunk_buildings;  // chunk key -> building ids
    std::vector<double> seam_x, seam_y;  // z14 tile seams (pieces of clipped polygons abut here)
    std::vector<std::string> log;

    bool load(const std::string &root);
    static uint64_t ckey(int cx, int cy) { return (uint64_t(uint32_t(cx)) << 32) | uint32_t(cy); }
    void chunk_range(int &x0, int &y0, int &x1, int &y1) const;  // chunks covering the region
    Rect chunk_rect(int cx, int cy) const { return {{cx * CHUNK, cy * CHUNK}, {(cx + 1) * CHUNK, (cy + 1) * CHUNK}}; }

    std::unique_ptr<ChunkData> build_chunk(int cx, int cy) const;
    // Coarse far-view cell (FAR_CELL m): terrain at 32 m, buildings as plain blocks. No collision.
    std::unique_ptr<ChunkData> build_far(int fx, int fy) const;
    void far_range(int &x0, int &y0, int &x1, int &y1) const;
    // height of the rendered terrain triangles of a built chunk at p (NAN outside / in a hole)
    static double grid_height(const ChunkData &c, V2 p);

    // ground height (terrain after shaping, or road/pavement surface where present)
    double ground_at(V2 p) const;
    double terrain_at(V2 p, const RoadGeom::Cache &c, bool *hole = nullptr) const;
    // a good place to start driving: on a carriageway near p, with heading along the road
    bool spawn_point(V2 near, V3 &pos, double &heading) const;
    double building_base(const Building &b, const RoadGeom::Cache &c, double *lowest = nullptr) const;

  private:
    void prepare_buildings();
    void emit_building(const Building &b, double base, double foot, V2 o, ChunkData &out) const;
};

}  // namespace es
