// Where on the planet a region is, and what follows from it: architectural zone, driving side,
// climate / vegetation. Coarse boxes on purpose: the data (Overture) has no country tags on
// buildings, and a few hundred kilometres of error at a border only changes the defaults.
#pragma once
#include <string>

namespace es {

enum Zone : int {
    Z_POST_SOVIET = 0,  // Belarus, Russia, Ukraine, Baltics, Caucasus, Central Asia: Stalinist / khrushchevka / panel
    Z_EAST_EU,          // Poland, Czechia, Slovakia, Hungary, Balkans, Romania, Bulgaria: old town + prefab
    Z_WEST_EU,          // historic stone/plaster centres, brick, steep tiled roofs, post-war blocks
    Z_NORTH_AMERICA,    // detached houses with siding and shingle gables, downtown towers
    Z_LATIN_AMERICA,    // flat roofs, colourful plaster, water tanks
    Z_HOT_DRY,          // North Africa, Middle East, South Asia: flat roofs, sand-coloured plaster
    Z_TROPICAL,         // Sub-Saharan Africa, South-East Asia: flat/metal roofs, bright plaster
    Z_EAST_ASIA,        // China, Korea, Japan: residential towers, Japanese small houses
    Z_OCEANIA,          // Australia, New Zealand: detached houses with metal roofs
    Z_COUNT
};

enum Biome : int { B_TEMPERATE = 0, B_BOREAL, B_MEDITERRANEAN, B_ARID, B_TROPICAL, B_TUNDRA };

struct Locale {
    Zone zone = Z_POST_SOVIET;
    Biome biome = B_TEMPERATE;
    bool lht = false;  // left-hand traffic
    double conifer = 0.25;  // share of conifers among generated trees
    double palm = 0;        // share of palms
    std::string zone_name() const;
    std::string biome_name() const;
};

Locale locale_for(double lat, double lon, double elevation);

}  // namespace es
