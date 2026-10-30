// A world chunk: a 256 m square of the local frame with everything in it built — terrain (flattened under
// roads, carved under water) with a ground-cover splat map, roads with junctions, kerbs, sidewalks and
// markings, bridges, paths, water surfaces, buildings, and placed objects (trees, lamps, parked cars).
// Chunks are built independently and agree at their edges: every feature that crosses an edge is computed
// from data both neighbours see, and each triangle is emitted by exactly one chunk.
#pragma once
#include <memory>
#include "buildings.hpp"
#include "mapdata.hpp"
#include "terrain.hpp"

namespace ec {

constexpr double CHUNK = 256;
constexpr int GRID_N = 256;     // terrain cells per chunk side (1 m)
constexpr int SPLAT_N = 256;    // ground-cover texels per side (1 m)
constexpr double FEATURE_MARGIN = 48;  // features this far outside a chunk still shape it

enum InstKind { I_TREE = 0, I_CONIFER = 1, I_BUSH = 2, I_LAMP = 3, I_CAR = 4, I_BENCH = 5, I_CAR_INTERIOR = 6, I_STEERING_WHEEL = 7, I_PERSON = 8 };

struct Instance {
    int kind = 0;
    V3 p;
    double yaw = 0, scale = 1;
    Col c;
};

// ground-cover layers (two RGBA8 images: layers 0-3 and 4-7)
enum Layer { L_GRASS = 0, L_FOREST = 1, L_SAND = 2, L_ASPHALT = 3, L_PAVERS = 4, L_FARM = 5, L_DIRT = 6, L_PITCH = 7 };

// a piece of drivable road centre line (for spawning, traffic and navigation)
struct LaneSeg {
    V3 a, b;          // centre line, road surface height
    double hw = 3;    // carriageway half width
    int oneway = 0;   // 1: only a->b, -1: only b->a
    int prio = 0;     // road class rank (motorway 9 .. service 1)
    int lanes = 2;
};

struct ChunkOut {
    int cx = 0, cy = 0;
    double x0 = 0, y0 = 0;
    HeightGrid grid;                 // (GRID_N+1)^2 heights, 1 m
    MeshBuf terrain_hi, terrain_lo;  // 1 m and 4 m terrain meshes
    std::vector<uint8_t> splat0, splat1;  // SPLAT_N^2 RGBA, row 0 = south
    std::vector<uint8_t> layer;           // SPLAT_N^2 dominant ground-cover layer (grip)
    MeshSet ms;                      // everything else, by material
    std::vector<float> coll;         // static collision triangles (xyz)
    std::vector<Instance> inst;
    std::vector<LaneSeg> lanes;  // owned road centre-line pieces
    std::vector<std::pair<V3, V3>> walks;  // pedestrian lines: sidewalks and paths (owned pieces)
    // grass tufts in 64 m blocks (row-major 4x4): MultiMesh buffers in Godot axes, 16 floats per tuft
    // (3x4 transform rows, then colour)
    std::vector<std::vector<float>> grass;
    int buildings = 0, roads = 0;
    std::vector<std::string> notes;
};

// `tiles`: decoded source tiles that overlap the chunk plus FEATURE_MARGIN.
ChunkOut build_chunk(int cx, int cy, const std::vector<std::shared_ptr<const RawTile>> &tiles, const Dem &dem, const LocalFrame &frame);

// lat/lon box of a chunk with the feature margin
Bounds chunk_bounds(int cx, int cy, const LocalFrame &frame, double margin);

}  // namespace ec
