// A region of the map decoded into local metric coordinates: roads reassembled across tile seams,
// buildings, land areas, point features and the elevation model.
#pragma once
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "geo.hpp"

namespace es {

// Elevation: mosaic of Terrarium tiles, sampled with a cubic B-spline (C2-smooth, light smoothing).
struct Dem {
    int z = 0, x0 = 0, y0 = 0, nx = 0, ny = 0;  // tile range
    int w = 0, h = 0;                             // samples
    std::vector<float> hgt;                       // row-major, north row first
    const Frame *frame = nullptr;
    double base = 0;  // mean height, used where there is no data
    bool add_tile(int tx, int ty, const uint8_t *png, size_t n);
    void finish();
    double sample(V2 p) const;
    bool empty() const { return hgt.empty(); }
};

struct Range {
    double a = 0, b = 1;  // fractions along the segment
};

enum class RoadKind : uint8_t { Road, Path, Rail };

struct RawRoad {
    std::string id;
    RoadKind kind = RoadKind::Road;
    std::string cls, subclass, name;
    std::vector<V2> pts;
    double frac0 = 0, frac1 = 1;  // fraction of the full Overture segment covered by pts
    double width = 0;             // from data, 0 = unknown
    int oneway = 0;               // 1 along, -1 against
    int speed = 0;                // km/h, 0 unknown
    std::string surface;
    std::vector<Range> bridge, tunnel;
    std::vector<std::pair<Range, int>> level;
    std::vector<std::pair<std::string, double>> connectors;  // id, fraction
};

struct Ring {
    Poly outer;
    std::vector<Poly> holes;
};

struct RawBuilding {
    std::string id, parent;  // parent: building_id of a part
    bool part = false, has_parts = false;
    std::vector<Ring> rings;  // pieces (clipped to their tiles; abut at seams)
    double height = 0, min_height = 0, roof_height = 0, roof_dir = -1;
    int floors = 0, min_floor = 0;
    std::string cls, subtype, roof_shape, roof_material, facade_material, name;
    uint32_t facade_color = 0, roof_color = 0;  // 0xRRGGBB + 0x1000000 flag when set
    bool underground = false;
};

enum class AreaKind : uint8_t { Water, Grass, Park, Forest, Scrub, Sand, Paved, Parking, Pitch, Residential, Industrial, Farmland, Cemetery, Construction, Other };

struct RawArea {
    AreaKind kind;
    Ring ring;
    std::string cls;
};

struct RawLine {
    std::string kind;  // "tree_row", "fence", "wall", "river", "stream", "power"...
    std::vector<V2> pts;
    double width = 0;
};

struct RawPoint {
    std::string kind;  // "tree", "street_lamp", "traffic_signals", "crossing", "bench", "bus_stop"...
    V2 p;
    std::string name;
};

struct Connector {
    V2 p;
    bool have = false;
};

struct Region {
    Frame frame;
    std::string release;
    std::vector<TileId> tiles;  // z14 detail tiles of the region
    Rect bounds;                // union of tiles
    Dem dem;
    std::vector<RawRoad> roads;
    std::vector<RawBuilding> buildings;
    std::vector<RawArea> areas;
    std::vector<RawLine> lines;
    std::vector<RawPoint> points;
    std::unordered_map<std::string, Connector> connectors;
    std::vector<std::string> log;

    // Load from a data directory laid out as <root>/region.json, overture/<rel>/<theme>/<z>/<x>/<y>.mvt,
    // dem/<z>/<x>/<y>.png. Returns false and fills log on fatal errors.
    // spec: region description file (default root/region.json); tiles are read from root
    bool load(const std::string &root, const std::string &spec = "");
};

uint32_t parse_color(const std::string &s);  // "#RRGGBB" -> 0x1RRGGBB, 0 if invalid
bool read_file(const std::string &path, std::string &out);

// world map: hypsometric + hill-shaded RGB of a Terrarium tile (256x256x3)
bool shade_terrarium(const uint8_t *png, size_t n, int z, int ty, std::vector<uint8_t> &rgb);

}  // namespace es
