#include "classify.hpp"
#include <algorithm>
#include <cstring>

namespace ec {

std::vector<Region> regions_at(double lat, double lon) {
    auto in = [&](double a, double b, double c, double d) { return lat >= a && lat <= b && lon >= c && lon <= d; };
    std::vector<Region> r;
    if (in(51.2, 56.2, 23.1, 32.8)) r.push_back(Region::Belarus);
    if (in(41, 70, 21, 60) || in(40, 55, 46, 87) || in(50, 72, 60, 180)) r.push_back(Region::ExUssr);
    if (in(50.2, 54.7, 10.5, 15.1)) r.push_back(Region::Gdr);
    if (in(41, 55, 12, 30)) r.push_back(Region::EasternEurope);
    if (in(42, 51.2, -5, 8.2)) r.push_back(Region::France);
    if (in(49.9, 59, -8, 1.8)) r.push_back(Region::Uk);
    if (in(36, 60, -10, 20)) r.push_back(Region::WesternEurope);
    if (in(24, 60, -130, -60)) r.push_back(Region::NorthAmerica);
    if (in(30, 46, 129, 146)) r.push_back(Region::Japan);
    if (in(18, 50, 97, 123)) r.push_back(Region::China);
    return r;
}

static bool any_of(const std::string &v, std::initializer_list<const char *> l) {
    for (auto s : l)
        if (v == s) return true;
    return false;
}

Use use_of(const Tags &t) {
    std::string cls = tagv(t, "building", "yes"), am = tagv(t, "amenity"), bu = tagv(t, "building:use");
    auto any = [&](std::initializer_list<const char *> l) { return any_of(cls, l) || any_of(am, l) || any_of(bu, l); };
    if (any({"monument", "memorial", "obelisk"})) return Use::Monument;
    if (any({"chimney", "tower", "water_tower", "silo", "storage_tank", "transformer_tower", "bunker"})) return Use::Structure;
    if (any({"church", "cathedral", "chapel", "mosque", "synagogue", "temple", "shrine", "religious", "place_of_worship", "monastery"})) return Use::Religious;
    if (any({"school", "university", "college", "kindergarten", "education", "library"})) return Use::Education;
    if (any({"hospital", "clinic", "doctors", "medical"})) return Use::Medical;
    if (any({"garage", "garages", "carport"})) return Use::Garage;
    if (any({"office", "bank", "hotel"}) || tag(t, "office")) return Use::Office;
    if (any({"retail", "supermarket", "commercial", "kiosk", "marketplace", "mall"}) || tag(t, "shop")) return Use::Retail;
    if (any({"industrial", "warehouse", "manufacture", "factory", "hangar", "service"})) return Use::Industrial;
    if (any({"civic", "public", "government", "townhall", "train_station", "transportation", "courthouse", "theatre", "museum", "fire_station", "police"})) return Use::Civic;
    if (any({"apartments", "residential", "dormitory"})) return Use::Housing;
    if (any({"house", "detached", "semidetached_house", "terrace", "bungalow", "cabin", "farm", "hut", "dacha"})) return Use::House;
    return Use::Other;
}

std::string infer_class(const Tags &t, const Ring &ring, double area) {
    std::string cls = tagv(t, "building", "yes");
    if (!any_of(cls, {"yes", "building", "true", "1"})) return cls;
    std::string am = tagv(t, "amenity");
    if (am == "place_of_worship") return "religious";
    if (any_of(am, {"school", "kindergarten", "university", "college"})) return am;
    if (any_of(am, {"hospital", "clinic", "doctors"})) return "hospital";
    if (any_of(am, {"bank", "bureau_de_change"})) return "office";
    if (any_of(am, {"townhall", "courthouse", "police", "fire_station", "library", "theatre", "cinema", "arts_centre", "community_centre", "post_office"})) return "civic";
    if (any_of(am, {"fuel", "restaurant", "cafe", "fast_food", "bar", "pub", "marketplace", "pharmacy"})) return area < 40 ? "kiosk" : "retail";
    if (tag(t, "shop")) return area < 40 ? "kiosk" : "retail";
    if (tag(t, "office")) return "office";
    std::string tour = tagv(t, "tourism"), hist = tagv(t, "historic"), mm = tagv(t, "man_made");
    if (any_of(tour, {"hotel", "hostel", "motel", "guest_house"})) return "hotel";
    if (any_of(tour, {"museum", "gallery"})) return "civic";
    if (any_of(hist, {"memorial", "monument"}) || any_of(mm, {"obelisk", "monument"})) return "monument";
    if (any_of(mm, {"tower", "chimney", "water_tower", "silo", "storage_tank"})) return mm;
    std::string bu = tagv(t, "building:use");
    if (bu == "residential") return "residential";
    if (bu == "commercial") return "commercial";
    if (bu == "industrial") return "industrial";
    if (bu == "education") return "school";
    if (bu == "religious") return "religious";
    if (bu == "medical") return "hospital";
    if (bu == "civic" || bu == "entertainment") return "civic";
    if (bu == "transportation") return "transportation";
    if (bu == "outbuilding" || bu == "agricultural") return "shed";
    if (bu == "service") return "service";
    std::string name = lower_utf8(tagv(t, "name"));
    if (!name.empty()) {
        // Russian / Belarusian / English words in names (UTF-8 lower case)
        if (contains_any(name, {"жк ", "жилой", "residen", "apartment"})) return "residential";
        if (contains_any(name, {"бизнес", "офис", "business", "office", "банк", "bank", "tower", "плаза", "plaza"})) return "office";
        if (contains_any(name, {"гостиниц", "отель", "hotel", "хостел", "hostel"})) return "hotel";
        if (contains_any(name, {"торгов", "трц", "тц ", "универмаг", "универсам", "магазин", "рынок", "mall", "market", "shop", "store", "гипермаркет", "супермаркет"})) return "retail";
        if (contains_any(name, {"школ", "гимназ", "лицей", "school"})) return "school";
        if (contains_any(name, {"детский сад", "ясли", "kindergarten"})) return "kindergarten";
        if (contains_any(name, {"университет", "институт", "академи", "колледж", "university", "college"})) return "university";
        if (contains_any(name, {"больниц", "поликлин", "госпитал", "hospital", "clinic", "медицин"})) return "hospital";
        if (contains_any(name, {"церк", "храм", "собор", "костёл", "костел", "часовн", "church", "cathedral", "chapel"})) return "religious";
        if (contains_any(name, {"театр", "музей", "дворец", "библиотек", "кинотеатр", "вокзал", "филармон", "цирк", "theatre", "museum", "station", "library", "palace"})) return "civic";
        if (contains_any(name, {"памятник", "монумент", "обелиск", "мемориал", "monument", "memorial", "obelisk"})) return "monument";
        if (contains_any(name, {"завод", "фабрик", "комбинат", "factory", "plant", "склад", "warehouse"})) return "industrial";
        if (contains_any(name, {"гараж", "garage"})) return "garages";
        if (contains_any(name, {"стадион", "арена", "спорт", "бассейн", "stadium", "arena", "sport"})) return "civic";
    }
    double h = 0;
    bool hknown = tag(t, "height") && parse_num(*tag(t, "height"), h);
    if (hknown && area < 150 && h > 3 * std::sqrt(area)) return "monument";
    if (area < 22) return "kiosk";
    double depth = corpus_depth(ring), length = perimeter(ring) / 2 - depth;
    double lv = 0;
    bool lknown = tag(t, "building:levels") && parse_num(*tag(t, "building:levels"), lv);
    double storeys = lknown ? lv : hknown ? h / 3 : -1;
    bool slab = depth >= 8.5 && depth <= 14.5 && length >= 24 && length / depth >= 2.2;
    std::string mat = tagv(t, "building:material");
    if (slab && (storeys < 0 || (storeys >= 3 && storeys <= 26)) && mat != "glass" && mat != "metal") return "residential";
    return "yes";
}

bool parse_colour(const std::string &s0, Col &c) {
    std::string s = lower(s0);
    while (!s.empty() && s[0] == ' ') s.erase(0, 1);
    if (!s.empty() && s[0] == '#') {
        auto hx = [&](const std::string &h) { return std::strtol(h.c_str(), nullptr, 16) / 255.0f; };
        if (s.size() == 7) {
            c = {hx(s.substr(1, 2)), hx(s.substr(3, 2)), hx(s.substr(5, 2)), 1};
            return true;
        }
        if (s.size() == 4) {
            c = {hx(std::string(2, s[1])), hx(std::string(2, s[2])), hx(std::string(2, s[3])), 1};
            return true;
        }
        return false;
    }
    static const std::pair<const char *, Col> N[] = {
        {"white", {0.93f, 0.93f, 0.91f}}, {"black", {0.1f, 0.1f, 0.1f}}, {"grey", {0.55f, 0.55f, 0.55f}}, {"gray", {0.55f, 0.55f, 0.55f}},
        {"lightgrey", {0.75f, 0.75f, 0.75f}}, {"lightgray", {0.75f, 0.75f, 0.75f}}, {"silver", {0.75f, 0.75f, 0.75f}}, {"darkgrey", {0.35f, 0.35f, 0.35f}},
        {"darkgray", {0.35f, 0.35f, 0.35f}}, {"red", {0.62f, 0.2f, 0.16f}}, {"maroon", {0.45f, 0.14f, 0.12f}}, {"darkred", {0.45f, 0.14f, 0.12f}},
        {"brown", {0.45f, 0.3f, 0.2f}}, {"tan", {0.82f, 0.71f, 0.55f}}, {"beige", {0.88f, 0.82f, 0.68f}}, {"cream", {0.94f, 0.9f, 0.8f}},
        {"ivory", {0.94f, 0.9f, 0.8f}}, {"yellow", {0.92f, 0.82f, 0.45f}}, {"orange", {0.88f, 0.55f, 0.25f}}, {"pink", {0.9f, 0.68f, 0.68f}},
        {"green", {0.4f, 0.55f, 0.38f}}, {"lightgreen", {0.62f, 0.78f, 0.58f}}, {"blue", {0.35f, 0.48f, 0.7f}}, {"lightblue", {0.62f, 0.74f, 0.86f}},
        {"darkblue", {0.18f, 0.24f, 0.42f}}, {"navy", {0.18f, 0.24f, 0.42f}}, {"teal", {0.25f, 0.5f, 0.5f}}, {"purple", {0.48f, 0.35f, 0.55f}}};
    for (auto &n : N)
        if (s == n.first) {
            c = n.second;
            return true;
        }
    return false;
}

namespace {

struct Series {
    const char *id;
    std::initializer_list<const char *> aliases;
    std::initializer_list<Region> regions;
    int y0, y1, l0, l1;
    double d0, d1;
    Use use;
    int style, variant;
    std::initializer_list<Col> palette;
    float floor_h;
    Balconies bal;
    int every;
    float sides;
    StairWin stair;
    Canopy canopy;
    float plinth, parapet;
    bool tech, shops;
    float section, bay;
};

const Col GREY_PANEL[] = {{0.78f, 0.77f, 0.74f}, {0.84f, 0.82f, 0.76f}, {0.72f, 0.73f, 0.74f}, {0.8f, 0.8f, 0.8f}};

const Series CATALOGUE[] = {
    {"1-464", {"1464", "1464a", "1464d"}, {Region::ExUssr}, 1958, 1975, 4, 5, 9.5, 12, Use::Housing, fs::PANEL, 2,
     {{0.8f, 0.78f, 0.72f}, {0.86f, 0.8f, 0.62f}, {0.74f, 0.76f, 0.78f}, {0.84f, 0.72f, 0.62f}, {0.78f, 0.82f, 0.78f}}, 2.7f, Balconies::Open, 2, 0.5f, StairWin::Landing, Canopy::Slab, 0.6f, 0.25f, false, false, 15, 3.2f},
    {"m464", {"m464", "m4641", "464m"}, {Region::Belarus}, 1965, 1990, 5, 9, 10.5, 12.5, Use::Housing, fs::PANEL, 0,
     {{0.78f, 0.77f, 0.74f}, {0.84f, 0.82f, 0.76f}, {0.72f, 0.73f, 0.74f}, {0.8f, 0.8f, 0.8f}}, 2.7f, Balconies::Open, 2, 0.5f, StairWin::Landing, Canopy::Slab, 0.7f, 0.3f, false, false, 16, 3.2f},
    {"1-335", {"1335", "1335a"}, {Region::ExUssr}, 1958, 1972, 4, 5, 9.5, 12, Use::Housing, fs::PANEL, 2,
     {{0.78f, 0.77f, 0.74f}, {0.84f, 0.82f, 0.76f}, {0.72f, 0.73f, 0.74f}}, 2.7f, Balconies::Open, 3, 0.5f, StairWin::Landing, Canopy::Slab, 0.6f, 0.2f, false, false, 15, 3.2f},
    {"1-447", {"1447", "1447s"}, {Region::ExUssr}, 1958, 1975, 4, 5, 10, 12.5, Use::Housing, fs::SILICATE, 0,
     {{0.9f, 0.89f, 0.86f}, {0.86f, 0.85f, 0.82f}, {0.92f, 0.9f, 0.84f}}, 2.8f, Balconies::Open, 3, 0.5f, StairWin::Landing, Canopy::Slab, 0.5f, 0.4f, false, false, 14, 3.0f},
    {"1-515", {"15159", "1515"}, {Region::ExUssr}, 1966, 1985, 9, 9, 11, 13, Use::Housing, fs::PANEL, 0,
     {{0.78f, 0.77f, 0.74f}, {0.84f, 0.82f, 0.76f}, {0.72f, 0.73f, 0.74f}}, 2.7f, Balconies::Open, 2, 0.5f, StairWin::Landing, Canopy::Slab, 0.8f, 0.6f, true, false, 24, 3.2f},
    {"II-49", {"ii49", "249"}, {Region::ExUssr}, 1965, 1985, 9, 12, 11.5, 13.5, Use::Housing, fs::TILE_PANEL, 0,
     {{0.92f, 0.92f, 0.92f}, {0.8f, 0.86f, 0.94f}, {0.94f, 0.88f, 0.8f}, {0.9f, 0.8f, 0.78f}}, 2.7f, Balconies::Open, 1, 0.5f, StairWin::Landing, Canopy::Slab, 0.8f, 0.6f, true, false, 24, 3.2f},
    {"II-18", {"ii18", "218"}, {Region::ExUssr}, 1960, 1985, 9, 12, 14, 26, Use::Housing, fs::SILICATE, 1,
     {{0.9f, 0.89f, 0.86f}, {0.86f, 0.85f, 0.82f}}, 2.8f, Balconies::Open, 2, 1.0f, StairWin::Band, Canopy::Slab, 0.6f, 0.6f, false, false, 999, 3.0f},
    {"II-57", {"ii57", "257"}, {Region::ExUssr}, 1965, 1985, 9, 16, 14, 24, Use::Housing, fs::PANEL, 1,
     {{0.78f, 0.77f, 0.74f}, {0.72f, 0.73f, 0.74f}}, 2.8f, Balconies::Glazed, 2, 1.0f, StairWin::Band, Canopy::Slab, 0.8f, 0.6f, true, false, 999, 3.0f},
    {"90", {"90", "9001", "90m"}, {Region::Belarus}, 1975, 2005, 5, 10, 11.5, 13.5, Use::Housing, fs::PANEL, 0,
     {{0.94f, 0.9f, 0.78f}, {0.86f, 0.92f, 0.88f}, {0.92f, 0.84f, 0.8f}, {0.84f, 0.88f, 0.94f}, {0.96f, 0.94f, 0.9f}}, 2.8f, Balconies::Glazed, 2, 0.5f, StairWin::Landing, Canopy::Slab, 0.8f, 0.6f, true, false, 22, 3.0f},
    {"111-90", {"11190", "111"}, {Region::Belarus}, 1980, 2010, 9, 12, 12, 14, Use::Housing, fs::PANEL, 1,
     {{0.94f, 0.9f, 0.78f}, {0.86f, 0.92f, 0.88f}, {0.92f, 0.84f, 0.8f}, {0.84f, 0.88f, 0.94f}}, 2.8f, Balconies::Glazed, 1, 0.5f, StairWin::Landing, Canopy::Slab, 0.8f, 0.6f, true, false, 24, 3.0f},
    {"P-44", {"p44", "p44t", "p44k", "p46"}, {Region::ExUssr}, 1979, 2005, 10, 17, 12, 14, Use::Housing, fs::TILE_PANEL, 1,
     {{0.92f, 0.92f, 0.92f}, {0.8f, 0.86f, 0.94f}, {0.94f, 0.88f, 0.8f}}, 2.8f, Balconies::Glazed, 2, 0.5f, StairWin::None, Canopy::Slab, 0.9f, 0.8f, true, false, 30, 3.0f},
    {"P-3", {"p3", "p3m"}, {Region::ExUssr}, 1975, 1995, 16, 22, 15, 26, Use::Housing, fs::PANEL, 1,
     {{0.78f, 0.77f, 0.74f}, {0.72f, 0.73f, 0.74f}}, 2.8f, Balconies::Glazed, 1, 1.0f, StairWin::None, Canopy::Slab, 0.9f, 0.8f, true, false, 999, 3.0f},
    {"KOPE", {"kope", "kopemm"}, {Region::ExUssr}, 1985, 2010, 17, 25, 14, 26, Use::Housing, fs::TILE_PANEL, 2,
     {{0.92f, 0.92f, 0.92f}, {0.8f, 0.86f, 0.94f}}, 2.8f, Balconies::Glazed, 1, 1.0f, StairWin::None, Canopy::Slab, 0.9f, 0.8f, true, false, 999, 3.0f},
    {"stalinka", {"stalinka"}, {Region::ExUssr}, 1935, 1958, 3, 8, 12, 18, Use::Housing, fs::PLASTER, 1,
     {{0.9f, 0.82f, 0.64f}, {0.86f, 0.74f, 0.58f}, {0.88f, 0.7f, 0.62f}, {0.92f, 0.88f, 0.78f}}, 3.3f, Balconies::Railing, 3, 0.5f, StairWin::Landing, Canopy::Slab, 1.2f, 1.1f, false, true, 20, 3.3f},
    {"pre-1917", {}, {Region::ExUssr}, 1850, 1917, 2, 5, 10, 20, Use::Housing, fs::PLASTER, 2,
     {{0.9f, 0.82f, 0.64f}, {0.86f, 0.74f, 0.58f}, {0.92f, 0.88f, 0.78f}}, 3.6f, Balconies::None, 1, 0, StairWin::None, Canopy::None, 0.9f, 1.0f, false, true, 20, 3.4f},
    {"monolith", {}, {Region::ExUssr}, 2000, 2035, 9, 30, 12, 22, Use::Housing, fs::PLASTER, 0,
     {{0.92f, 0.9f, 0.86f}, {0.76f, 0.74f, 0.7f}, {0.86f, 0.78f, 0.66f}, {0.6f, 0.62f, 0.66f}, {0.94f, 0.94f, 0.94f}}, 3.0f, Balconies::Glazed, 1, 0.5f, StairWin::Band, Canopy::Slab, 1.0f, 0.9f, false, true, 28, 3.0f},
    {"WBS 70", {"wbs70", "p2", "qp71"}, {Region::Gdr}, 1972, 1990, 5, 11, 11, 13, Use::Housing, fs::PANEL, 0,
     {{0.94f, 0.9f, 0.78f}, {0.86f, 0.92f, 0.88f}, {0.92f, 0.84f, 0.8f}}, 2.8f, Balconies::Open, 2, 0.5f, StairWin::Band, Canopy::Slab, 0.9f, 0.5f, false, false, 24, 3.0f},
    {"panelak", {"t06b", "vvu", "wk70", "wufs"}, {Region::EasternEurope}, 1960, 1992, 4, 13, 10, 14, Use::Housing, fs::PANEL, 2,
     {{0.94f, 0.9f, 0.78f}, {0.86f, 0.92f, 0.88f}, {0.92f, 0.84f, 0.8f}, {0.84f, 0.88f, 0.94f}}, 2.8f, Balconies::Glazed, 2, 0.5f, StairWin::Band, Canopy::Slab, 0.8f, 0.5f, false, false, 22, 3.0f},
    {"haussmann", {"haussmann"}, {Region::France}, 1853, 1914, 5, 8, 10, 22, Use::Housing, fs::PLASTER, 1,
     {{0.9f, 0.86f, 0.76f}, {0.88f, 0.84f, 0.74f}, {0.92f, 0.88f, 0.8f}}, 3.2f, Balconies::Railing, 1, 1.0f, StairWin::None, Canopy::None, 1.0f, 0.4f, false, true, 20, 2.6f},
    {"eu-postwar", {}, {Region::WesternEurope}, 1950, 1985, 4, 12, 10, 14, Use::Housing, fs::PLASTER, 2,
     {{0.94f, 0.9f, 0.78f}, {0.86f, 0.92f, 0.88f}, {0.92f, 0.84f, 0.8f}}, 2.8f, Balconies::Open, 1, 0.5f, StairWin::Band, Canopy::Slab, 0.8f, 0.5f, false, false, 20, 3.0f},
    {"eu-modern", {}, {Region::WesternEurope}, 1990, 2035, 4, 12, 10, 18, Use::Housing, fs::PLASTER, 0,
     {{0.92f, 0.9f, 0.86f}, {0.76f, 0.74f, 0.7f}, {0.94f, 0.94f, 0.94f}}, 3.0f, Balconies::Glazed, 1, 0.5f, StairWin::Band, Canopy::Slab, 0.6f, 0.9f, false, true, 22, 3.0f},
    {"us-walkup", {}, {Region::NorthAmerica}, 1880, 1960, 3, 6, 10, 25, Use::Housing, fs::BRICK, 1,
     {{0.58f, 0.3f, 0.22f}, {0.66f, 0.36f, 0.26f}}, 3.1f, Balconies::None, 1, 0, StairWin::None, Canopy::Slab, 0.8f, 0.8f, false, true, 20, 3.0f},
    {"jp-mansion", {}, {Region::Japan}, 1970, 2035, 4, 15, 8, 15, Use::Housing, fs::PLASTER, 0,
     {{0.94f, 0.93f, 0.9f}, {0.88f, 0.86f, 0.82f}}, 2.9f, Balconies::Open, 1, 0.5f, StairWin::None, Canopy::Slab, 0.5f, 1.0f, false, false, 999, 2.8f},
    {"cn-tower", {}, {Region::China}, 1990, 2035, 12, 34, 14, 26, Use::Housing, fs::PLASTER, 2,
     {{0.92f, 0.9f, 0.86f}, {0.76f, 0.74f, 0.7f}}, 3.0f, Balconies::Glazed, 1, 1.0f, StairWin::None, Canopy::Slab, 1.0f, 1.2f, false, false, 999, 3.0f},
};

std::string normalise_series(const std::string &s) {
    std::string l = lower_utf8(s), o;
    // Cyrillic look-alikes to Latin, drop separators
    static const std::pair<const char *, char> M[] = {{"п", 'p'}, {"к", 'k'}, {"о", 'o'}, {"е", 'e'}, {"э", 'e'}, {"м", 'm'}, {"т", 't'}, {"а", 'a'},
                                                     {"с", 's'}, {"д", 'd'}, {"и", 'i'}, {"в", 'v'}, {"у", 'u'}, {"р", 'p'}, {"х", 'x'}, {"н", 'n'}};
    for (size_t i = 0; i < l.size();) {
        unsigned char c = l[i];
        if (c < 0x80) {
            if (std::isalnum(c)) o += (char)c;
            i++;
            continue;
        }
        bool done = false;
        for (auto &m : M) {
            size_t n = std::strlen(m.first);
            if (l.compare(i, n, m.first) == 0) {
                o += m.second;
                i += n;
                done = true;
                break;
            }
        }
        if (!done) i++;
    }
    return o;
}

const Series *match_series(const Tags &t, Use use, double lat, double lon, int levels, double depth, int year, uint64_t seed) {
    if (auto s = tag(t, "building:series")) {
        std::string n = normalise_series(*s);
        const Series *best = nullptr;
        size_t bl = 0;
        for (auto &e : CATALOGUE) {
            if (normalise_series(e.id) == n) return &e;
            for (auto a : e.aliases) {
                size_t al = std::strlen(a);
                if (n == a) return &e;
                if (al >= 2 && n.compare(0, al, a) == 0 && al > bl) {
                    best = &e;
                    bl = al;
                }
            }
        }
        if (best) return best;
    }
    if (use != Use::Housing) return nullptr;
    auto here = regions_at(lat, lon);
    std::vector<std::pair<double, const Series *>> cand;
    for (auto &e : CATALOGUE) {
        bool in = false;
        for (auto r : e.regions)
            if (r == Region::Any || std::find(here.begin(), here.end(), r) != here.end()) in = true;
        if (!in) continue;
        double score = 0;
        for (auto r : e.regions)
            if (!here.empty() && here[0] == r) score += 1;
        if (levels > 0) {
            if (levels >= e.l0 && levels <= e.l1) score += 4;
            else if (levels + 1 >= e.l0 && levels <= e.l1 + 1) score += 1.5;
            else continue;
        }
        if (depth >= e.d0 - 0.8 && depth <= e.d1 + 0.8) score += 2;
        else if (depth < e.d0 - 4 || depth > e.d1 + 8) continue;
        if (year > 0) {
            if (year >= e.y0 && year <= e.y1) score += 3;
            else if (year < e.y0 - 5 || year > e.y1 + 10) continue;
        }
        cand.push_back({score, &e});
    }
    if (cand.empty()) return nullptr;
    double top = 0;
    for (auto &c : cand) top = std::max(top, c.first);
    if (top < 3) return nullptr;  // too weak a match
    std::vector<const Series *> best;
    for (auto &c : cand)
        if (c.first >= top - 0.01) best.push_back(c.second);
    return best[seed % best.size()];
}

template <class L>
Col pick(const L &l, uint64_t seed) {
    size_t n = l.size(), k = (seed >> 8) % n;
    auto it = l.begin();
    std::advance(it, k);
    return *it;
}

}  // namespace

BuildingInfo classify(const std::string &id, const Tags &tags0, bool part, const Ring &ring, double lat, double lon) {
    BuildingInfo bi;
    bi.part = part;
    bi.seed = std::max<uint64_t>(1, hash_str(id));
    uint64_t seed = bi.seed;
    double area = std::abs(signed_area(ring));
    Tags tags = tags0;
    if (!part) {
        bi.cls = infer_class(tags, ring, area);
        set_tag(tags, "building", bi.cls);
    } else {
        bi.cls = "part";
    }
    bi.use = part ? use_of(tags) : use_of(tags);
    Use use = bi.use;
    // height
    double h = 0, lv = 0;
    bool has_h = tag(tags, "height") && parse_num(*tag(tags, "height"), h) && h > 0 && h < 1000;
    bool has_l = tag(tags, "building:levels") && parse_num(*tag(tags, "building:levels"), lv) && lv >= 1 && lv < 300;
    bi.levels = has_l ? (int)std::lround(lv) : 0;
    double depth = corpus_depth(ring);
    int year = 0;
    if (auto sd = tag(tags, "start_date"); sd && sd->size() >= 4) year = std::atoi(sd->substr(0, 4).c_str());
    const Series *ser = part ? nullptr : match_series(tags, use, lat, lon, bi.levels ? bi.levels : (has_h ? (int)std::lround(h / 3) : 0), depth, year, seed);
    Design &d = bi.d;
    if (ser) {
        d.id = ser->id;
        d.style = ser->style;
        d.variant = ser->variant;
        d.colour = pick(ser->palette, seed);
        d.floor_h = ser->floor_h;
        d.balconies = ser->bal;
        d.balcony_every = ser->every;
        d.balcony_sides = ser->sides;
        d.stair = ser->stair;
        d.canopy = ser->canopy;
        d.plinth_h = ser->plinth;
        d.parapet_h = ser->parapet;
        d.tech_floor = ser->tech;
        d.shops = ser->shops;
        d.section_len = ser->section;
        d.bay = ser->bay;
        d.typical_levels = ser->l0 + (int)(seed % (uint64_t)(ser->l1 - ser->l0 + 1));
        d.matched = true;
    }
    if (has_h) {
        bi.height = h;
        bi.height_origin = "tag";
    } else if (has_l) {
        double rl = 0;
        double extra = tag(tags, "roof:levels") && parse_num(*tag(tags, "roof:levels"), rl) ? rl * 2.6 : 0;
        bi.height = lv * (ser ? ser->floor_h : 3.0) + extra + (ser ? 0.5 : 0.3);
        bi.height_origin = "levels";
    } else if (ser) {
        bi.height = d.typical_levels * ser->floor_h + 0.5;
        bi.height_origin = "series";
    } else {
        double n = 5;
        std::string c = bi.cls;
        if (any_of(c, {"garage", "garages", "shed", "kiosk", "carport", "toilets", "hut"})) n = 1;
        else if (any_of(c, {"house", "detached", "chapel", "semidetached_house", "bungalow", "dacha", "cabin"})) n = 2;
        else if (any_of(c, {"retail", "supermarket"})) n = 1.6;
        else if (any_of(c, {"church", "mosque", "synagogue", "temple", "religious"})) n = 4;
        else if (c == "kindergarten") n = 2;
        else if (any_of(c, {"school", "hospital"})) n = 4;
        else if (c == "monument") n = 4;
        else if (any_of(c, {"industrial", "warehouse", "service"})) n = 3;
        else if (c == "yes") {
            // unknown: by footprint size (small sheds low, big blocks taller)
            n = area < 60 ? 1 : area < 200 ? 2 : area < 600 ? 4 : 5;
        }
        bi.height = n * 3.0;
        bi.height_origin = "default";
    }
    double mh = 0, ml = 0;
    if (tag(tags, "min_height") && parse_num(*tag(tags, "min_height"), mh) && mh > 0 && mh < bi.height - 0.5) bi.min_height = mh;
    else if (tag(tags, "building:min_level") && parse_num(*tag(tags, "building:min_level"), ml) && ml > 0 && ml * 3 < bi.height - 0.5) bi.min_height = ml * 3;

    // facade style
    std::string mat = lower(tagv(tags, "building:material"));
    bool brick = mat == "brick";
    int style;
    if (!mat.empty()) {
        if (mat == "brick") style = fs::BRICK;
        else if (mat == "glass" || mat == "mirror") style = fs::GLASS;
        else if (any_of(mat, {"concrete", "panel", "panels", "reinforced_concrete", "cement_block"})) style = fs::PANEL;
        else if (any_of(mat, {"wood", "timber", "timber_framing", "log"})) style = fs::WOOD;
        else if (any_of(mat, {"metal", "steel", "metal_plates", "aluminium"})) style = fs::METAL;
        else style = fs::PLASTER;
    } else {
        std::string c = bi.cls;
        if (use == Use::Garage) style = fs::BRICK;
        else if (any_of(c, {"office", "commercial", "retail"}) && bi.height > 24) style = fs::GLASS;
        else if (any_of(c, {"industrial", "warehouse", "hangar", "service", "manufacture"})) style = fs::METAL;
        else if (any_of(c, {"apartments", "residential", "dormitory"}) && bi.height >= 15) style = fs::PANEL;
        else style = fs::PLASTER;
    }
    int variant = (int)(seed % 3);
    Col col;
    switch (style) {
        case fs::BRICK: col = pick(std::initializer_list<Col>{{0.58f, 0.3f, 0.22f}, {0.66f, 0.38f, 0.28f}, {0.8f, 0.68f, 0.48f}, {0.5f, 0.26f, 0.2f}, {0.72f, 0.62f, 0.52f}}, seed); break;
        case fs::PANEL: col = pick(std::initializer_list<Col>{{0.78f, 0.77f, 0.74f}, {0.86f, 0.84f, 0.78f}, {0.7f, 0.72f, 0.74f}, {0.82f, 0.78f, 0.68f}, {0.74f, 0.7f, 0.66f}, {0.8f, 0.82f, 0.84f}}, seed); break;
        case fs::GLASS: col = pick(std::initializer_list<Col>{{0.55f, 0.62f, 0.68f}, {0.45f, 0.55f, 0.6f}, {0.6f, 0.6f, 0.58f}, {0.4f, 0.48f, 0.55f}}, seed); break;
        case fs::METAL: col = pick(std::initializer_list<Col>{{0.62f, 0.64f, 0.66f}, {0.55f, 0.6f, 0.68f}, {0.7f, 0.68f, 0.62f}, {0.5f, 0.56f, 0.52f}}, seed); break;
        case fs::WOOD: col = pick(std::initializer_list<Col>{{0.55f, 0.4f, 0.26f}, {0.45f, 0.32f, 0.22f}, {0.62f, 0.5f, 0.36f}}, seed); break;
        default: col = pick(std::initializer_list<Col>{{0.86f, 0.8f, 0.68f}, {0.92f, 0.88f, 0.78f}, {0.86f, 0.74f, 0.62f}, {0.8f, 0.84f, 0.82f}, {0.9f, 0.82f, 0.62f}, {0.84f, 0.7f, 0.66f}, {0.94f, 0.92f, 0.88f}, {0.78f, 0.8f, 0.72f}}, seed);
    }
    auto var = [&](int n) { return (int)((seed >> 3) % (uint64_t)n); };
    bool special = true;
    switch (use) {
        case Use::Religious:
            style = fs::ARCHED;
            variant = brick ? 1 : std::initializer_list<int>{0, 0, 1}.begin()[var(3)];
            col = pick(std::initializer_list<Col>{{0.95f, 0.94f, 0.9f}, {0.94f, 0.88f, 0.7f}, {0.93f, 0.86f, 0.76f}, {0.8f, 0.86f, 0.9f}, {0.92f, 0.8f, 0.78f}, {0.96f, 0.95f, 0.93f}}, seed);
            break;
        case Use::Civic:
            if (!mat.empty() && !brick) {
                special = false;
                break;
            }
            style = fs::ARCHED;
            variant = brick ? 1 : (var(2) ? 2 : 0);
            col = pick(std::initializer_list<Col>{{0.95f, 0.94f, 0.9f}, {0.94f, 0.88f, 0.7f}, {0.93f, 0.86f, 0.76f}, {0.92f, 0.8f, 0.78f}}, seed);
            break;
        case Use::Education:
        case Use::Medical:
            if (!mat.empty() && !brick && mat != "plaster") {
                special = false;
                break;
            }
            style = fs::SCHOOL;
            variant = brick ? 1 : use == Use::Medical ? 0 : std::initializer_list<int>{0, 0, 2}.begin()[var(3)];
            col = pick(std::initializer_list<Col>{{0.94f, 0.92f, 0.86f}, {0.92f, 0.84f, 0.66f}, {0.84f, 0.88f, 0.84f}, {0.96f, 0.95f, 0.93f}, {0.88f, 0.8f, 0.7f}}, seed);
            break;
        case Use::Retail:
            if (!mat.empty() || bi.height > 9) {
                special = false;
                break;
            }
            style = fs::SHOP;
            variant = area > 1500 ? 1 : 0;
            col = pick(std::initializer_list<Col>{{0.85f, 0.83f, 0.8f}, {0.7f, 0.7f, 0.72f}, {0.9f, 0.86f, 0.78f}}, seed);
            break;
        case Use::Garage:
            style = fs::SHOP;
            variant = 2;
            break;
        case Use::Office:
            if (!mat.empty()) {
                special = false;
                break;
            }
            if (bi.height <= 24) {
                style = fs::OFFICE;
                variant = var(3);
                col = pick(std::initializer_list<Col>{{0.85f, 0.86f, 0.88f}, {0.72f, 0.74f, 0.78f}, {0.88f, 0.84f, 0.76f}, {0.62f, 0.64f, 0.66f}}, seed);
            } else {
                style = fs::GLASS;
                variant = var(3);
            }
            break;
        case Use::Monument:
            style = fs::BLANK;
            variant = 0;
            col = pick(std::initializer_list<Col>{{0.82f, 0.8f, 0.76f}, {0.7f, 0.68f, 0.64f}, {0.88f, 0.86f, 0.82f}}, seed);
            break;
        case Use::Structure:
            style = fs::BLANK;
            variant = brick ? 2 : 1;
            break;
        default: special = false;
    }
    (void)special;
    if (d.matched && mat.empty() && use == Use::Housing) {
        style = d.style;
        variant = d.variant;
        col = d.colour;
    }
    Col c2;
    if (auto bc = tag(tags, "building:colour"); bc && parse_colour(*bc, c2)) col = c2;
    d.style = style;
    d.variant = variant;
    d.colour = col;
    // storey height used by the facade texture
    if (has_l) d.floor_h = (float)std::clamp((bi.height - bi.min_height) / lv, 2.4, 6.0);
    else if (!d.matched) {
        if (use == Use::Religious) d.floor_h = 5.5f;
        else if (use == Use::Education || use == Use::Medical) d.floor_h = 3.4f;
        else if (use == Use::Civic) d.floor_h = 4.0f;
        else if (use == Use::Garage) d.floor_h = 3.0f;
        else if (use == Use::Retail && bi.height <= 9) d.floor_h = (float)std::clamp(bi.height, 3.0, 4.5);
        else if (style == fs::METAL) d.floor_h = 6.0f;
        else d.floor_h = 2.8f + (float)((seed >> 20) % 5) * 0.1f;
    }
    if (!d.matched) {
        d.plinth_h = (use == Use::House || use == Use::Garage || use == Use::Monument) ? 0.4f : 0.7f;
        d.parapet_h = 0.4f + (float)hash01(seed, 1) * 0.6f;
        d.shops = (use == Use::Retail || use == Use::Office) && bi.height < 30;
        d.canopy = use == Use::House ? Canopy::Gable : (use == Use::Monument || use == Use::Structure || use == Use::Garage) ? Canopy::None : Canopy::Slab;
        if (use == Use::Housing) {
            d.balconies = hash01(seed, 20) < 0.75 ? (hash01(seed, 23) < 0.4 ? Balconies::Glazed : Balconies::Open) : Balconies::None;
            d.stair = bi.height > 9 ? StairWin::Landing : StairWin::None;
            d.balcony_every = 1 + (int)(hash01(seed, 21) * 2);
        }
    }
    // roof
    Col rc;
    if (auto r = tag(tags, "roof:colour"); r && parse_colour(*r, rc)) bi.roof_colour = rc;
    else if (use == Use::Religious) bi.roof_colour = pick(std::initializer_list<Col>{{0.3f, 0.48f, 0.38f}, {0.35f, 0.38f, 0.42f}, {0.55f, 0.3f, 0.22f}}, seed);
    else if (use == Use::House) bi.roof_colour = pick(std::initializer_list<Col>{{0.55f, 0.27f, 0.2f}, {0.35f, 0.33f, 0.32f}, {0.42f, 0.22f, 0.18f}, {0.3f, 0.36f, 0.42f}}, seed);
    std::string rs = tagv(tags, "roof:shape");
    if (rs == "gabled" || rs == "gambrel" || rs == "saltbox") bi.roof_shape = "gabled";
    else if (rs == "hipped" || rs == "half-hipped" || rs == "mansard") bi.roof_shape = "hipped";
    else if (rs == "pyramidal" || rs == "cone" || rs == "conical") bi.roof_shape = "pyramidal";
    else if (rs == "dome") bi.roof_shape = "dome";
    else if (rs == "onion") bi.roof_shape = "onion";
    else if (rs.empty()) {
        bool church = use == Use::Religious && bi.cls != "mosque" && tagv(tags, "religion") != "muslim";
        bool obelisk = use == Use::Monument && area < 150 && bi.height > 2.5 * std::sqrt(area);
        if (obelisk) bi.roof_shape = "pyramidal";
        else if ((use == Use::House && bi.height <= 12) || (church && bi.height <= 30)) bi.roof_shape = "gabled";
        else if (use == Use::Housing && bi.height <= 15 && area < 900 && !d.matched && hash01(seed, 77) < 0.35) bi.roof_shape = "hipped";
        else bi.roof_shape = "flat";
    } else
        bi.roof_shape = "flat";
    double rh;
    if (tag(tags, "roof:height") && parse_num(*tag(tags, "roof:height"), rh) && rh > 0) bi.roof_height = rh;
    // landmark
    if (use == Use::Religious && !part && area >= 40 && bi.min_height < 0.1 && !any_of(bi.roof_shape, {"dome", "onion", "pyramidal"})) {
        std::string rel = tagv(tags, "religion"), den = lower(tagv(tags, "denomination"));
        if (bi.cls == "mosque" || rel == "muslim") bi.landmark = Landmark::Mosque;
        else if (any_of(bi.cls, {"synagogue", "temple", "shrine"}) || (!rel.empty() && rel != "christian")) bi.landmark = Landmark::None;
        else {
            bool orth;
            if (den.find("orthodox") != std::string::npos) orth = true;
            else if (contains_any(den, {"catholic", "lutheran", "protestant", "anglican", "baptist", "evangelical"})) orth = false;
            else {
                auto r = regions_at(lat, lon);
                orth = std::find(r.begin(), r.end(), Region::ExUssr) != r.end() || std::find(r.begin(), r.end(), Region::Belarus) != r.end();
            }
            if (bi.cls == "chapel" || area < 120) bi.landmark = orth ? Landmark::Chapel : Landmark::Spire;
            else bi.landmark = orth ? Landmark::Orthodox : Landmark::Spire;
        }
    }
    return bi;
}

}  // namespace ec
