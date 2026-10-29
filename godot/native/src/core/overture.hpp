// Overture Maps vector tiles (themes buildings, transportation, base) -> map features of a streaming tile.
#pragma once
#include "mapdata.hpp"
#include "mvt.hpp"

namespace ec {

constexpr const char *OVERTURE_BUCKET = "https://overturemaps-extras-us-west-2.s3.us-west-2.amazonaws.com";
constexpr const char *OVERTURE_FALLBACK_RELEASE = "2026-10-21.0";
constexpr int OVERTURE_ZOOM = 14;

std::string overture_newest_release(const std::string &list_xml);

// Features of `tile` (zoom >= 14) from the decoded layers of its z14 parent `src`. Buildings belong to the
// tile holding their centre; roads and paths are kept up to `margin_m` beyond the tile; areas are clipped to
// the tile.
void overture_features(TileId tile, TileId src, const std::vector<mvt::Layer> &buildings, const std::vector<mvt::Layer> &transport,
                       const std::vector<mvt::Layer> &base, double margin_m, RawTile &out, const TileId *base_src = nullptr);

// helpers shared with the OSM reader
std::vector<LL> clip_ring_ll(const std::vector<LL> &ring, const Bounds &b);
std::vector<std::pair<size_t, size_t>> clip_runs_ll(const std::vector<LL> &pts, const Bounds &b);
bool point_in_ll(const LL &p, const std::vector<LL> &ring);

}  // namespace ec
