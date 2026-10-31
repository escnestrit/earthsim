#include "locale.hpp"

namespace es {

namespace {
struct Box {
    double lat0, lat1, lon0, lon1;
    bool in(double la, double lo) const { return la >= lat0 && la <= lat1 && lo >= lon0 && lo <= lon1; }
};
}  // namespace

std::string Locale::zone_name() const {
    static const char *n[] = {"постсоветский город", "Восточная Европа", "Западная Европа", "Северная Америка", "Латинская Америка",
                              "жаркий сухой пояс", "тропики", "Восточная Азия", "Австралия и Океания"};
    return n[zone];
}

std::string Locale::biome_name() const {
    static const char *n[] = {"умеренный", "тайга", "средиземноморский", "засушливый", "тропический", "тундра"};
    return n[biome];
}

Locale locale_for(double la, double lo, double elev) {
    Locale L;
    // ---- architectural zone
    if (Box{41, 82, 22, 180}.in(la, lo) || Box{35, 56, 46, 90}.in(la, lo) || Box{53.5, 60, 20.5, 28.5}.in(la, lo)) L.zone = Z_POST_SOVIET;
    else if (Box{39, 55, 12.1, 30}.in(la, lo) && !Box{39, 47.5, 12.1, 14.5}.in(la, lo)) L.zone = Z_EAST_EU;
    else if (Box{34, 72, -25, 32}.in(la, lo)) L.zone = Z_WEST_EU;
    else if (Box{24.5, 75, -170, -52}.in(la, lo)) L.zone = Z_NORTH_AMERICA;
    else if (Box{-56, 32.7, -118, -30}.in(la, lo)) L.zone = Z_LATIN_AMERICA;
    else if (Box{18, 54, 100, 146}.in(la, lo)) L.zone = Z_EAST_ASIA;
    else if (Box{-48, -10, 110, 180}.in(la, lo)) L.zone = Z_OCEANIA;
    else if (Box{12, 42, -18, 75}.in(la, lo) || Box{5, 37, 60, 92}.in(la, lo)) L.zone = Z_HOT_DRY;
    else L.zone = Z_TROPICAL;
    // Mexico-US border region belongs to North America above 32.7 N; the Caribbean to Latin America
    // ---- driving side (left: UK/IE, Japan, South Asia, SE Asia except PH/VN, southern & eastern Africa, AU/NZ)
    L.lht = Box{49.8, 61, -11, 1.8}.in(la, lo)        // Britain, Ireland
            || Box{24, 46, 128.5, 146}.in(la, lo)     // Japan
            || Box{5.5, 37, 60.5, 92.5}.in(la, lo)    // Pakistan, India, Bangladesh, Sri Lanka, Nepal
            || Box{-11, 20.5, 95, 104.5}.in(la, lo)   // Thailand, Malaysia, Singapore, Sumatra
            || Box{-11, 6, 104.5, 141}.in(la, lo)     // Indonesia
            || Box{-35, -15, 10, 41}.in(la, lo)       // southern Africa
            || Box{-12, 5, 29, 42}.in(la, lo)         // Kenya, Tanzania, Uganda
            || Box{-48, -10, 110, 180}.in(la, lo)     // Australia, New Zealand
            || Box{22.1, 22.6, 113.8, 114.5}.in(la, lo);  // Hong Kong
    // ---- biome
    double a = la < 0 ? -la : la;
    bool desert = Box{15, 33, -17, 60}.in(la, lo) || Box{31, 41, -120, -103}.in(la, lo) || Box{-32, -19, 115, 145}.in(la, lo) ||
                  Box{-30, -15, -72, -68}.in(la, lo) || Box{37, 46, 88, 112}.in(la, lo) || Box{24, 30, 68, 74}.in(la, lo) ||
                  Box{36, 46, 52, 66}.in(la, lo);
    bool medit = (Box{30, 45.5, -10, 37}.in(la, lo) && !Box{30, 35, -10, 37}.in(la, lo)) || Box{32, 40, -124, -116}.in(la, lo) ||
                 Box{-38, -30, 112, 140}.in(la, lo) || Box{-38, -30, -73, -70}.in(la, lo) || Box{-35, -33, 17, 21}.in(la, lo);
    if (a > 66 || elev > 3500) L.biome = B_TUNDRA;
    else if (desert) L.biome = B_ARID;
    else if (a < 23.5) L.biome = B_TROPICAL;
    else if (medit) L.biome = B_MEDITERRANEAN;
    else if (a > 57 || (a > 50 && lo > 60 && lo < 140) || elev > 1800) L.biome = B_BOREAL;
    else L.biome = B_TEMPERATE;
    switch (L.biome) {
        case B_BOREAL: L.conifer = 0.65; break;
        case B_TUNDRA: L.conifer = 0.8; break;
        case B_MEDITERRANEAN: L.conifer = 0.3, L.palm = 0.12; break;
        case B_ARID: L.conifer = 0.05, L.palm = 0.55; break;
        case B_TROPICAL: L.conifer = 0.0, L.palm = 0.45; break;
        default: L.conifer = 0.25;
    }
    return L;
}

}  // namespace es
