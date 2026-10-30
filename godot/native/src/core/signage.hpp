// Shop-front and sign styles: colours, wording and icons by what the place is (OSM shop/amenity/office values,
// a few well-known brands), so storefronts read as what they are.
#pragma once
#include <string>
#include "mesh.hpp"

namespace ec {

struct ShopStyle {
    Col board, text, awning;
    bool awning_on = false;
    int icon = 0;  // 0 none, 1 cross (pharmacy), 2 cup (cafe), 3 scissors-ish bar (hair)
    std::string word;  // what to write when the place has no name
};

// `cyr`: label in Russian (Belarus, Russia, ...) else English
ShopStyle shop_style(const std::string &key, const std::string &value, const std::string &name, const std::string &brand, bool cyr);
bool cyrillic_region(double lat, double lon);

}  // namespace ec
