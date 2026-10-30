// Static world collision for the soft-body solver: the terrain height grids of loaded chunks plus the
// triangles of everything built (roads, kerbs, buildings, bridges, poles), in the local frame (z up).
// Thread-safe: chunks are added/removed by the streamer while the car queries.
#pragma once
#include <map>
#include <memory>
#include <shared_mutex>
#include <unordered_map>
#include <vector>
#include "../core/terrain.hpp"

namespace ec {

struct Surface {
    double height = 0;
    V3 normal{0, 0, 1};
    double grip = 1;
    bool found = false;
};

struct Hit {
    V3 n;          // push-out direction (unit)
    double depth;  // penetration
    double grip;
};

struct ColChunk {
    int cx = 0, cy = 0;
    HeightGrid grid;
    std::vector<uint8_t> layer;  // ground-cover layer per 1 m texel (grip of the terrain)
    std::vector<float> tris;     // 9 floats per triangle
    // triangle index buckets on a 4 m grid over the chunk
    std::vector<std::vector<uint32_t>> cells;
    int ncell = 0;
    double x0 = 0, y0 = 0;
    void index();
};

// a moving box (traffic car): centre, heading about z, half extents
struct DynBox {
    V3 c;
    double yaw, hx, hy, hz;
};

class CollisionWorld {
public:
    void set_dynamic(std::vector<DynBox> boxes);
    void add(std::shared_ptr<ColChunk> c);
    void remove(int cx, int cy);
    bool has(int cx, int cy) const;
    // the walkable surface under (x, y) at or below z + step: the higher of the terrain and upward-facing
    // triangles (decks, roads, kerb tops)
    Surface surface(double x, double y, double z, double step = 0.6) const;
    // contacts of a sphere with triangles (and the terrain when `terrain`)
    void sphere(V3 c, double r, std::vector<Hit> &out, bool terrain) const;
    // any triangle within the box (cheap broad phase for a whole vehicle)
    bool any_near(V3 lo, V3 hi) const;

private:
    mutable std::shared_mutex mu_;
    std::map<std::pair<int, int>, std::shared_ptr<ColChunk>> chunks_;
    std::vector<DynBox> dyn_;
    const ColChunk *chunk_at(double x, double y) const;
};

double layer_grip(int layer);

}  // namespace ec
