// What a building is and how it looks: class from tags/name/shape, use, typical design (series), facade
// style and colours, storey height, roof shape, landmark (towers/domes of places of worship).
#pragma once
#include <string>
#include "geom.hpp"
#include "mesh.hpp"
#include "util.hpp"

namespace ec {

enum class Use { Housing, House, Religious, Education, Medical, Office, Retail, Garage, Industrial, Civic, Monument, Structure, Other };
enum class Balconies { None, Open, Glazed, Railing };
enum class StairWin { None, Landing, Band };
enum class Canopy { None, Slab, Gable };
enum class Landmark { None, Orthodox, Spire, Chapel, Mosque };
enum class Region { Any, ExUssr, Belarus, Gdr, EasternEurope, France, Uk, WesternEurope, NorthAmerica, Japan, China };

// facade style indices
namespace fs {
enum { PLASTER = 0, BRICK = 1, PANEL = 2, GLASS = 3, WOOD = 4, METAL = 5, TILE_PANEL = 6, SILICATE = 7, ARCHED = 8, SCHOOL = 9, SHOP = 10, OFFICE = 11, BLANK = 12 };
}

struct Design {
    const char *id = "";
    int style = 0, variant = 0;
    Col colour;
    float floor_h = 3;
    Balconies balconies = Balconies::None;
    int balcony_every = 1;
    float balcony_sides = 0.5f;
    StairWin stair = StairWin::None;
    Canopy canopy = Canopy::Slab;
    float plinth_h = 0.7f, parapet_h = 0.5f;
    bool tech_floor = false, shops = false;
    float section_len = 18, bay = 3;
    int typical_levels = 5;
    bool matched = false;  // from the series catalogue
};

struct BuildingInfo {
    std::string cls;  // effective class
    Use use = Use::Other;
    double height = 9, min_height = 0;
    std::string height_origin;  // tag, levels, series, default
    int levels = 0;
    Design d;
    Col roof_colour{0.32f, 0.30f, 0.29f, 1};
    std::string roof_shape;  // flat, gabled, hipped, pyramidal, dome, onion
    double roof_height = 0;
    Landmark landmark = Landmark::None;
    uint64_t seed = 1;
    bool part = false;
};

std::vector<Region> regions_at(double lat, double lon);
Use use_of(const Tags &t);
std::string infer_class(const Tags &t, const Ring &ring, double area);
// everything above from the tags, the local outline and its position
BuildingInfo classify(const std::string &id, const Tags &tags, bool part, const Ring &ring, double lat, double lon);
bool parse_colour(const std::string &s, Col &c);

}  // namespace ec
