// Terrain: elevation from "terrarium" PNG tiles (AWS Terrain Tiles: SRTM / GMTED / national DEMs, open data)
// sampled bicubically, and per-chunk height grids in the local frame.
#pragma once
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "geo.hpp"

namespace ec {

constexpr const char *TERRARIUM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium";
constexpr int DEM_ZOOM = 13;

// metres from a terrarium PNG (256 x 256); false when the bytes are not a terrarium tile
bool terrarium_decode(const std::string &png, std::vector<float> &out, std::string &err);

// Elevation over lat/lon from the tiles that have been added. Thread-safe.
class Dem {
public:
    void add(int x, int y, std::vector<float> heights);  // z = DEM_ZOOM
    bool has(int x, int y) const;
    // metres above sea level; `fallback` where no tile is loaded
    double height(double lat, double lon, double fallback = 0) const;
    // tiles needed for a lat/lon box
    static std::vector<TileId> tiles_for(const Bounds &b);

private:
    mutable std::mutex mu_;
    std::map<uint64_t, std::shared_ptr<const std::vector<float>>> tiles_;
    double texel(int gx, int gy, bool &ok) const;  // global pixel coordinates at DEM_ZOOM
};

// Square grid of heights in the local frame: (n+1) x (n+1) samples, `cell` metres apart, row-major from
// the south-west corner (x0, y0), rows going north.
struct HeightGrid {
    double x0 = 0, y0 = 0, cell = 1;
    int n = 0;
    std::vector<float> h;
    float at(int i, int j) const { return h[(size_t)j * (n + 1) + i]; }
    float &at(int i, int j) { return h[(size_t)j * (n + 1) + i]; }
    // bilinear inside; clamps outside
    double sample(double x, double y) const;
    V3 normal(double x, double y) const;
};

}  // namespace ec
