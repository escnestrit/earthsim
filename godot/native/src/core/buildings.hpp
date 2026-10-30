// Building meshes: walls (facade texture coordinates in whole window bays), roofs (flat with courtyards,
// gabled, hipped, pyramidal, dome, onion), parts raised on min_height, and exterior elements (plinth,
// parapet, rooftop machine rooms, balconies, stairwell windows, shop fronts, entrance porches, church towers).
#pragma once
#include "classify.hpp"
#include "geom.hpp"
#include "mesh.hpp"

namespace ec {

struct Door {
    V2 p;    // on the wall line
    V2 out;  // unit vector out of the building
    std::string kind;  // staircase, main, service, garage, yes...
};

// a mapped shop / amenity / office inside the building (from OSM nodes)
struct ShopPoi {
    V2 p;
    std::string key, value, name, brand;  // key: shop, amenity, office, tourism
};

struct BuildingGeom {
    std::vector<Ring> rings;  // outer CCW, holes CW (local metres)
    BuildingInfo info;
    double base = 0;  // ground floor level: the pavement at the street front (lowest corner when off-street)
    double ground_min = 1e300, ground_max = 0;  // ground_min: unset = base
    std::vector<Door> doors;
    std::vector<ShopPoi> pois;
    double lat = 0, lon = 0;  // for the sign language
    std::string amenity;
    std::string name;         // OSM name (malls, cinemas... get it in big letters)
};

// a bronze standing figure (statues, sculptures), feet at z, facing yaw
void statue(MeshSet &ms, V2 c, double yaw, double z, double h, Col col, uint64_t seed);

// Adds the building to `ms`; collision triangles (walls and roof, xyz floats) to `coll`.
void build_building(const BuildingGeom &b, MeshSet &ms, std::vector<float> &coll, std::vector<Label> *labels = nullptr);

}  // namespace ec
