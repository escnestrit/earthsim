// Map features of one streaming tile, as read from Overture (and OSM tags merged on top), in lat/lon.
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "geo.hpp"
#include "util.hpp"

namespace ec {

struct LL {
    double lat = 0, lon = 0;
};

enum class AreaKind { Water, Grass, Park, Forest, Sand, Parking, Pitch, Farmland };

struct RawBuilding {
    std::string id;
    std::vector<std::vector<LL>> rings;  // outer ring first, then courtyards (holes); open rings
    Tags tags;
    bool part = false;
};

struct RawRoad {
    std::vector<LL> pts;
    std::string cls;  // OSM-style highway class
    double width = 6.5;
    bool width_from_data = false;
    bool bridge = false;
    int oneway = 0;  // 1 along, -1 against
    int layer = 0;
    int speed = 0;  // km/h from the data, 0 unknown
    std::string name;
};

struct RawPath {
    std::vector<LL> pts;
    double width = 2;
    bool paved = true;
    bool bridge = false;
};

// rails (class: standard_gauge, tram, light_rail, narrow_gauge, subway...), power lines, fences, walls, hedges
struct RawLine {
    std::vector<LL> pts;
    std::string kind;  // "rail:<class>", "power:<class>", "barrier:<class>"
    bool bridge = false;
};

struct RawArea {
    AreaKind kind;
    std::vector<std::vector<LL>> rings;  // outer then holes
};

struct RawPoint {
    LL p;
    std::string kind;  // "entrance:<type>", "tree", "lamp"
};

struct RawTile {
    TileId tile;
    std::vector<RawBuilding> buildings;
    std::vector<RawRoad> roads;
    std::vector<RawPath> paths;
    std::vector<RawArea> areas;
    std::vector<RawPoint> points;
    std::vector<RawLine> lines;
    // rivers, canals, streams given as centre lines: (points, width m)
    std::vector<std::pair<std::vector<LL>, double>> waterlines;
    bool has_osm = false;
    bool has_furniture = false;  // Overture's infrastructure layer gave signals, crossings... (mapped, not guessed)  // an OSM enrich answer was merged (signals, signs, shops known)
    std::vector<std::string> notes;
};

// Default carriageway widths (m) for drivable classes; 0 = not drivable.
double road_default_width(const std::string &cls);

}  // namespace ec
