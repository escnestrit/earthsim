#include "overture.hpp"
#include <algorithm>
#include <regex>
#include <set>

namespace ec {

double road_default_width(const std::string &c) {
    static const std::pair<const char *, double> W[] = {
        {"motorway", 14}, {"trunk", 12}, {"primary", 11}, {"secondary", 9}, {"tertiary", 8},
        {"unclassified", 6.5}, {"residential", 6.5}, {"living_street", 5}, {"service", 4},
        {"motorway_link", 7}, {"trunk_link", 7}, {"primary_link", 7}, {"secondary_link", 6.5}, {"tertiary_link", 6}, {"road", 6}};
    for (auto &w : W)
        if (c == w.first) return w.second;
    return 0;
}

std::string overture_newest_release(const std::string &xml) {
    std::string best;
    size_t p = 0;
    const std::string key = "<Prefix>tiles/";
    while ((p = xml.find(key, p)) != std::string::npos) {
        p += key.size();
        size_t e = xml.find('/', p);
        if (e == std::string::npos) break;
        std::string r = xml.substr(p, e - p);
        if (r.size() >= 10 && r[0] >= '0' && r[0] <= '9' && r > best) best = r;
    }
    return best;
}

bool point_in_ll(const LL &p, const std::vector<LL> &r) {
    bool in = false;
    size_t n = r.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if ((r[i].lat > p.lat) != (r[j].lat > p.lat) && p.lon < r[i].lon + (p.lat - r[i].lat) / (r[j].lat - r[i].lat) * (r[j].lon - r[i].lon)) in = !in;
    }
    return in;
}

std::vector<LL> clip_ring_ll(const std::vector<LL> &ring, const Bounds &b) {
    std::vector<LL> pts = ring;
    for (int edge = 0; edge < 4 && !pts.empty(); edge++) {
        auto inside = [&](const LL &p) {
            switch (edge) {
                case 0: return p.lon >= b.w;
                case 1: return p.lon <= b.e;
                case 2: return p.lat >= b.s;
                default: return p.lat <= b.n;
            }
        };
        auto cut = [&](const LL &a, const LL &c) {
            double t;
            if (edge < 2) {
                double v = edge == 0 ? b.w : b.e;
                t = (v - a.lon) / (c.lon - a.lon);
            } else {
                double v = edge == 2 ? b.s : b.n;
                t = (v - a.lat) / (c.lat - a.lat);
            }
            return LL{a.lat + (c.lat - a.lat) * t, a.lon + (c.lon - a.lon) * t};
        };
        std::vector<LL> out;
        for (size_t i = 0; i < pts.size(); i++) {
            const LL &a = pts[i], &c = pts[(i + 1) % pts.size()];
            bool ia = inside(a), ic = inside(c);
            if (ia) out.push_back(a);
            if (ia != ic) out.push_back(cut(a, c));
        }
        pts.swap(out);
    }
    return pts.size() >= 3 ? pts : std::vector<LL>();
}

std::vector<std::pair<size_t, size_t>> clip_runs_ll(const std::vector<LL> &pts, const Bounds &b) {
    std::vector<std::pair<size_t, size_t>> runs;
    auto in = [&](const LL &p) { return p.lat >= b.s && p.lat <= b.n && p.lon >= b.w && p.lon <= b.e; };
    long start = -1;
    for (size_t i = 0; i < pts.size(); i++) {
        if (in(pts[i])) {
            if (start < 0) start = (long)(i > 0 ? i - 1 : i);
        } else if (start >= 0) {
            runs.push_back({(size_t)start, i});
            start = -1;
        }
    }
    if (start >= 0) runs.push_back({(size_t)start, pts.size() - 1});
    // drop single-point runs
    runs.erase(std::remove_if(runs.begin(), runs.end(), [](auto &r) { return r.second <= r.first; }), runs.end());
    return runs;
}

namespace {

// A flag in an Overture list property (road_flags, access_restrictions) given as JSON text, applying to the
// whole segment (or >= 80 % of it).
bool whole_flag(const std::string &flags, const char *flag) {
    size_t from = 0, i;
    std::string f = flag;
    while ((i = flags.find(f, from)) != std::string::npos) {
        from = i + f.size();
        size_t st = flags.rfind('{', i);
        size_t en = flags.find('}', i);
        std::string obj = flags.substr(st == std::string::npos ? 0 : st, (en == std::string::npos ? flags.size() : en) - (st == std::string::npos ? 0 : st));
        size_t b = obj.find("between");
        if (b == std::string::npos) return true;
        std::vector<double> nums;
        std::string cur;
        for (size_t k = b; k < obj.size() && nums.size() < 2; k++) {
            char c = obj[k];
            if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == 'e')
                cur += c;
            else if (!cur.empty()) {
                nums.push_back(atof(cur.c_str()));
                cur.clear();
            }
        }
        if (nums.size() < 2 || nums[1] - nums[0] >= 0.8) return true;
    }
    return false;
}

int oneway_of(const std::string &ar) {
    size_t from = 0, i;
    while ((i = ar.find("denied", from)) != std::string::npos) {
        from = i + 6;
        size_t st = ar.rfind('{', i);
        if (st == std::string::npos) st = 0;
        int depth = 0;
        size_t en = ar.size();
        for (size_t k = st; k < ar.size(); k++) {
            if (ar[k] == '{') depth++;
            else if (ar[k] == '}' && --depth == 0) {
                en = k + 1;
                break;
            }
        }
        std::string rule = ar.substr(st, en - st);
        if (rule.find("between") != std::string::npos) continue;
        bool cars = rule.find("mode") == std::string::npos || contains_any(rule, {"\"vehicle\"", "\"motor_vehicle\"", "\"car\""});
        if (!cars) continue;
        if (rule.find("\"backward\"") != std::string::npos) return 1;
        if (rule.find("\"forward\"") != std::string::npos) return -1;
    }
    return 0;
}

struct Conv {
    TileId src;
    int extent;
    LL ll(std::pair<int, int> p) const {
        LL r;
        tile_px_latlon(src, extent, p.first, p.second, r.lat, r.lon);
        return r;
    }
    std::vector<LL> line(const std::vector<std::pair<int, int>> &v) const {
        std::vector<LL> o;
        o.reserve(v.size());
        for (auto &p : v) o.push_back(ll(p));
        return o;
    }
};

// polygon parts: outer ring (positive area, y down) followed by its holes
std::vector<std::vector<std::vector<std::pair<int, int>>>> polygons(const mvt::Feature &f) {
    std::vector<std::vector<std::vector<std::pair<int, int>>>> out;
    for (auto &r : f.parts) {
        if (r.size() < 3) continue;
        if (mvt::ring_area(r) > 0)
            out.push_back({r});
        else if (!out.empty())
            out.back().push_back(r);
    }
    return out;
}

void put(Tags &t, const char *k, const std::string &v) {
    if (!v.empty()) t.push_back({k, v});
}

Tags building_tags(const mvt::Feature &f, bool part) {
    Tags t;
    std::string cls = f.str("class");
    if (!part) t.push_back({"building", cls.empty() ? "yes" : cls});
    else t.push_back({"building:part", cls.empty() ? "yes" : cls});
    double v;
    char buf[32];
    if (f.num("height", v) && v > 0) {
        snprintf(buf, sizeof buf, "%.2f", v);
        put(t, "height", buf);
    }
    if (f.num("min_height", v) && v > 0) {
        snprintf(buf, sizeof buf, "%.2f", v);
        put(t, "min_height", buf);
    }
    if (f.num("num_floors", v) && v > 0) put(t, "building:levels", std::to_string((int)std::lround(v)));
    if (f.num("min_floor", v) && v > 0) put(t, "building:min_level", std::to_string((int)std::lround(v)));
    put(t, "roof:shape", f.str("roof_shape"));
    if (f.num("roof_height", v) && v > 0) {
        snprintf(buf, sizeof buf, "%.2f", v);
        put(t, "roof:height", buf);
    }
    put(t, "roof:colour", f.str("roof_color"));
    put(t, "roof:material", f.str("roof_material"));
    put(t, "building:colour", f.str("facade_color"));
    put(t, "building:material", f.str("facade_material"));
    put(t, "name", f.str("@name"));
    put(t, "building:use", f.str("subtype"));
    return t;
}

bool area_kind(const std::string &layer, const mvt::Feature &f, AreaKind &k) {
    std::string st = f.str("subtype"), cls = f.str("class");
    if (layer == "water") {
        if (cls == "swimming_pool" || cls == "fountain") return false;
        k = AreaKind::Water;
        return true;
    }
    if (layer == "land") {
        if (st == "forest" || cls == "wood" || cls == "forest") k = AreaKind::Forest;
        else if (st == "grass" || st == "shrub" || st == "wetland" || cls == "grassland" || cls == "scrub" || cls == "heath" || cls == "meadow") k = AreaKind::Grass;
        else if (st == "sand" || cls == "beach" || cls == "sand") k = AreaKind::Sand;
        else return false;
        return true;
    }
    if (layer == "land_cover") {
        if (st == "forest" || st == "mangrove") k = AreaKind::Forest;
        else if (st == "grass" || st == "shrub" || st == "wetland" || st == "moss") k = AreaKind::Grass;
        else if (st == "crop") k = AreaKind::Farmland;
        else if (st == "barren") k = AreaKind::Sand;
        else return false;
        return true;
    }
    if (layer == "land_use") {
        if (cls == "forest" || cls == "wood") k = AreaKind::Forest;
        else if (cls == "park" || cls == "garden" || cls == "playground" || cls == "dog_park" || st == "park") k = AreaKind::Park;
        else if (cls == "pitch" || cls == "golf_course" || cls == "stadium" || cls == "track") k = AreaKind::Pitch;
        else if (cls == "grass" || cls == "meadow" || cls == "village_green" || cls == "recreation_ground" || cls == "cemetery" || cls == "allotments" || cls == "orchard" || cls == "grave_yard") k = AreaKind::Grass;
        else if (cls == "farmland" || cls == "farmyard" || st == "agriculture") k = AreaKind::Farmland;
        else if (cls == "beach") k = AreaKind::Sand;
        else if (cls == "parking" || cls == "parking_space") k = AreaKind::Parking;
        else return false;
        return true;
    }
    return false;
}

}  // namespace

void overture_features(TileId tile, TileId src, const std::vector<mvt::Layer> &buildings, const std::vector<mvt::Layer> &transport,
                       const std::vector<mvt::Layer> &base, double margin_m, RawTile &out, const TileId *base_src) {
    out.tile = tile;
    Bounds b = tile_bounds(tile);
    double clat = (b.s + b.n) / 2;
    double dlat = margin_m / 111320, dlon = margin_m / (111320 * std::max(0.05, std::cos(clat * PI / 180)));
    Bounds eb{b.s - dlat, b.w - dlon, b.n + dlat, b.e + dlon};
    Bounds sb = tile_bounds(src);
    auto in_tile = [&](const LL &p) { return p.lat >= b.s && p.lat < b.n && p.lon >= b.w && p.lon < b.e; };

    // buildings and parts: whole outlines, clipped to the source tile (vector tiles carry a buffer), owned by
    // the tile holding their centre
    for (auto &L : buildings) {
        bool part = L.name == "building_part";
        if (L.name != "building" && !part) continue;
        Conv cv{src, L.extent};
        for (auto &f : L.features) {
            if (f.kind != mvt::Geom::Polygon) continue;
            auto ug = f.prop("is_underground");
            if (ug && ug->kind == mvt::Value::Bool && ug->b) continue;
            for (auto &poly : polygons(f)) {
                RawBuilding rb;
                for (auto &ring : poly) {
                    auto r = clip_ring_ll(cv.line(ring), sb);
                    if (r.size() >= 3) rb.rings.push_back(std::move(r));
                    else if (rb.rings.empty()) break;
                }
                if (rb.rings.empty()) continue;
                LL c{0, 0};
                for (auto &p : rb.rings[0]) {
                    c.lat += p.lat;
                    c.lon += p.lon;
                }
                c.lat /= rb.rings[0].size();
                c.lon /= rb.rings[0].size();
                if (!in_tile(c)) continue;
                rb.tags = building_tags(f, part);
                rb.part = part;
                rb.id = "ov:" + std::to_string(f.id) + ":" + std::to_string((long long)std::llround(c.lat * 1e7)) + ":" + std::to_string((long long)std::llround(c.lon * 1e7));
                out.buildings.push_back(std::move(rb));
            }
        }
    }

    // transportation: roads, paths
    for (auto &L : transport) {
        if (L.name != "segment") continue;
        Conv cv{src, L.extent};
        for (auto &f : L.features) {
            if (f.kind != mvt::Geom::Line) continue;
            if (f.str("subtype") != "road") continue;
            std::string cls = f.str("class"), sub = f.str("subclass");
            std::string flags = f.str("road_flags");
            if (whole_flag(flags, "is_tunnel") || whole_flag(flags, "is_underground")) continue;
            bool bridge = whole_flag(flags, "is_bridge");
            static const std::set<std::string> pathc = {"footway", "pedestrian", "path", "cycleway", "steps", "track", "bridleway", "sidewalk", "crosswalk"};
            for (auto &part : f.parts) {
                if (part.size() < 2) continue;
                auto pts = cv.line(part);
                for (auto [s, e] : clip_runs_ll(pts, eb)) {
                    std::vector<LL> run(pts.begin() + s, pts.begin() + e + 1);
                    if (pathc.count(cls)) {
                        // pavements and crossings are drawn with the roads
                        if (sub == "sidewalk" || sub == "crosswalk" || cls == "sidewalk" || cls == "crosswalk") continue;
                        RawPath p;
                        p.pts = std::move(run);
                        p.width = cls == "pedestrian" ? 5.0 : cls == "track" ? 3.0 : cls == "path" ? 1.6 : cls == "cycleway" ? 2.2 : 2.0;
                        p.paved = !(cls == "path" || cls == "track" || cls == "bridleway");
                        p.bridge = bridge;
                        out.paths.push_back(std::move(p));
                        continue;
                    }
                    std::string hw = cls;
                    if (sub == "link" && (cls == "motorway" || cls == "trunk" || cls == "primary" || cls == "secondary" || cls == "tertiary")) hw = cls + "_link";
                    if (cls == "unknown" || cls.empty()) hw = "road";
                    double w = road_default_width(hw);
                    if (w <= 0) continue;
                    RawRoad r;
                    r.pts = std::move(run);
                    r.cls = hw;
                    r.width = w;
                    r.bridge = bridge;
                    r.oneway = oneway_of(f.str("access_restrictions"));
                    if (hw == "motorway" || hw == "motorway_link") r.oneway = r.oneway ? r.oneway : 1;
                    r.name = f.str("@name");
                    out.roads.push_back(std::move(r));
                }
            }
        }
    }

    // base: water, land, land use, land cover; rivers given as lines; trees and lamps as points
    for (auto &L : base) {
        Conv cv{base_src ? *base_src : src, L.extent};
        for (auto &f : L.features) {
            if (f.kind == mvt::Geom::Polygon) {
                AreaKind k;
                if (!area_kind(L.name, f, k)) continue;
                for (auto &poly : polygons(f)) {
                    RawArea a;
                    a.kind = k;
                    for (auto &ring : poly) {
                        auto r = clip_ring_ll(cv.line(ring), b);
                        if (r.size() >= 3) a.rings.push_back(std::move(r));
                        else if (a.rings.empty()) break;
                    }
                    if (!a.rings.empty()) out.areas.push_back(std::move(a));
                }
            } else if (f.kind == mvt::Geom::Line && L.name == "water") {
                std::string k = f.str("class");
                if (k.empty()) k = f.str("subtype");
                double w = k == "river" ? 22 : k == "canal" ? 10 : k == "stream" ? 3.5 : (k == "drain" || k == "ditch") ? 2 : 0;
                if (w <= 0) continue;
                if (f.str("is_tunnel") == "true") continue;
                for (auto &part : f.parts) {
                    if (part.size() < 2) continue;
                    auto pts = cv.line(part);
                    for (auto [s, e] : clip_runs_ll(pts, eb)) out.waterlines.push_back({std::vector<LL>(pts.begin() + s, pts.begin() + e + 1), w});
                }
            } else if (f.kind == mvt::Geom::Point) {
                std::string cls = f.str("class"), st = f.str("subtype");
                const char *kind = nullptr;
                if (cls == "tree" || st == "tree") kind = "tree";
                else if (cls == "street_lamp") kind = "lamp";
                if (!kind) continue;
                for (auto &part : f.parts)
                    for (auto &p : part) {
                        LL q = cv.ll(p);
                        if (in_tile(q)) out.points.push_back({q, kind});
                    }
            }
        }
    }
}

}  // namespace ec
