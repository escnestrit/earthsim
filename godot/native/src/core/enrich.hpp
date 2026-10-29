#pragma once
#include "json.hpp"
#include "mapdata.hpp"
#include "overture.hpp"

namespace ec {

// The game's OSM query for a tile area: building tags (heights, use, names), entrances, road lanes/oneway.
std::string osm_enrich_query(const Bounds &b);

// Merge an Overpass answer (of `osm_enrich_query`) into the tile: building tags onto the Overture outline
// around each OSM centre, entrance nodes, road lanes/width/oneway onto the nearest road. Returns how many
// features changed.
int merge_osm(RawTile &t, const Json &osm);

}  // namespace ec
