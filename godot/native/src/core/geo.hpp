// Geodesy: a local tangent frame (east/north metres around an origin) and web-mercator tile math.
#pragma once
#include "util.hpp"

namespace ec {

constexpr double EARTH_R = 6378137.0;

// Local east/north metres around (lat0, lon0) on the WGS84 ellipsoid's local radii. Accurate to centimetres
// within a few kilometres (the game re-bases the frame long before errors grow).
struct LocalFrame {
    double lat0 = 0, lon0 = 0;
    double m_per_deg_lat = 111320, m_per_deg_lon = 111320;
    LocalFrame() = default;
    LocalFrame(double lat, double lon) : lat0(lat), lon0(lon) {
        double phi = lat * PI / 180;
        // meridional and prime-vertical radii of curvature
        const double a = 6378137.0, e2 = 6.69437999014e-3;
        double s = std::sin(phi);
        double w = std::sqrt(1 - e2 * s * s);
        double M = a * (1 - e2) / (w * w * w);
        double N = a / w;
        m_per_deg_lat = M * PI / 180;
        m_per_deg_lon = N * std::cos(phi) * PI / 180;
    }
    V2 to_local(double lat, double lon) const { return {(lon - lon0) * m_per_deg_lon, (lat - lat0) * m_per_deg_lat}; }
    void to_latlon(V2 p, double &lat, double &lon) const {
        lat = lat0 + p.y / m_per_deg_lat;
        lon = lon0 + p.x / m_per_deg_lon;
    }
};

struct TileId {
    int z = 0;
    int x = 0, y = 0;
    bool operator==(const TileId &o) const { return z == o.z && x == o.x && y == o.y; }
    bool operator<(const TileId &o) const { return z != o.z ? z < o.z : x != o.x ? x < o.x : y < o.y; }
    uint64_t key() const { return ((uint64_t)z << 58) | ((uint64_t)x << 29) | (uint64_t)y; }
};

inline TileId tile_of(double lat, double lon, int z) {
    double n = std::ldexp(1.0, z);
    int x = (int)std::floor((lon + 180) / 360 * n);
    double latr = lat * PI / 180;
    int y = (int)std::floor((1 - std::asinh(std::tan(latr)) / PI) / 2 * n);
    int m = (int)n - 1;
    return {z, std::min(std::max(x, 0), m), std::min(std::max(y, 0), m)};
}

// (south, west, north, east)
struct Bounds {
    double s, w, n, e;
};
inline Bounds tile_bounds(TileId t) {
    double n = std::ldexp(1.0, t.z);
    auto lat_of = [&](double y) { return std::atan(std::sinh(PI * (1 - 2 * y / n))) * 180 / PI; };
    return {lat_of(t.y + 1), t.x / n * 360 - 180, lat_of(t.y), (t.x + 1) / n * 360 - 180};
}

// tile-local integer coordinates (extent units, y down) -> lat/lon
inline void tile_px_latlon(TileId t, int extent, double px, double py, double &lat, double &lon) {
    double n = std::ldexp(1.0, t.z);
    double x = (t.x + px / extent) / n;
    double y = (t.y + py / extent) / n;
    lon = x * 360 - 180;
    lat = std::atan(std::sinh(PI * (1 - 2 * y))) * 180 / PI;
}

}  // namespace ec
