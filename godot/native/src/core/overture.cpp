#include "overture.hpp"
#include <functional>
#include <cmath>
#include "json.hpp"
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

// [from, to] fractions of a segment's length where `flag` holds (whole segment: {0, 1})
std::vector<std::pair<double, double>> flag_ranges(const std::string &flags, const char *flag) {
    std::vector<std::pair<double, double>> out;
    if (flags.find(flag) == std::string::npos) return out;
    Json j;
    if (!Json::parse(flags, j) || j.kind != Json::Arr) return out;
    for (auto &e : j.a) {
        const Json *v = e.get("values");
        if (!v || v->kind != Json::Arr) continue;
        bool has = false;
        for (auto &x : v->a) has |= x.kind == Json::Str && x.s == flag;
        if (!has) continue;
        const Json *b = e.get("between");
        if (b && b->kind == Json::Arr && b->a.size() == 2) out.push_back({b->a[0].num(0), b->a[1].num(1)});
        else out.push_back({0, 1});
    }
    return out;
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

// Splits roads and paths into bridge / non-bridge runs where `on` holds for points along them (sampled every
// 3 m); stretches are widened by `extend` metres each way and dropped when shorter than `min_len`.
void mark_bridges(RawTile &out, const std::function<bool(LL)> &on, double extend, double min_len) {
    auto split = [&](const std::vector<LL> &pts) {
        std::vector<LL> d;
        std::vector<double> cum;
        for (size_t i = 0; i < pts.size(); i++) {
            if (i) {
                double kx = 111320 * std::cos(pts[i].lat * PI / 180);
                double l = std::hypot((pts[i].lon - pts[i - 1].lon) * kx, (pts[i].lat - pts[i - 1].lat) * 111320);
                int n = std::max(1, (int)std::ceil(l / 3.0));
                for (int k = 1; k < n; k++) {
                    double t = (double)k / n;
                    d.push_back({pts[i - 1].lat + (pts[i].lat - pts[i - 1].lat) * t, pts[i - 1].lon + (pts[i].lon - pts[i - 1].lon) * t});
                    cum.push_back(cum.back() + l / n);
                }
                d.push_back(pts[i]);
                cum.push_back(cum.back() + l / n);
            } else
                d.push_back(pts[0]), cum.push_back(0);
        }
        std::vector<char> w(d.size());
        bool any = false;
        for (size_t i = 0; i < d.size(); i++) any |= (w[i] = on(d[i]));
        std::vector<std::pair<std::vector<LL>, bool>> runs;
        if (!any) return runs;
        std::vector<char> b(d.size(), 0);
        for (size_t i = 0; i < d.size(); i++)
            if (w[i])
                for (size_t k = 0; k < d.size(); k++)
                    if (std::abs(cum[k] - cum[i]) <= extend) b[k] = 1;
        for (size_t i = 0; i < d.size();) {
            size_t j = i;
            while (j < d.size() && b[j] == b[i]) j++;
            if (b[i] && cum[j - 1] - cum[i] < min_len) std::fill(b.begin() + i, b.begin() + j, 0);
            i = j;
        }
        for (size_t i = 0; i < d.size();) {
            size_t j = i;
            while (j < d.size() && b[j] == b[i]) j++;
            std::vector<LL> run(d.begin() + (i ? i - 1 : 0), d.begin() + j);
            if (run.size() >= 2) runs.push_back({run, (bool)b[i]});
            i = j;
        }
        return runs;
    };
    auto apply = [&](auto &items) {
        std::remove_reference_t<decltype(items)> res;
        for (auto &r : items) {
            if (r.bridge) {
                res.push_back(r);
                continue;
            }
            auto runs = split(r.pts);
            bool has = false;
            for (auto &x : runs) has |= x.second;
            if (!has) {
                res.push_back(r);
                continue;
            }
            for (auto &[pts, br] : runs) {
                auto q = r;
                q.pts = pts;
                q.bridge = br;
                res.push_back(std::move(q));
            }
        }
        items = std::move(res);
    };
    apply(out.roads);
    apply(out.paths);
}

// Roads and paths crossing water become bridges there (Overture marks bridges by fractions of whole segments,
// which tiles clip away; the water polygons tell where the crossing really is). The bridge run is extended
// onto the banks for the abutments.
static void bridges_over_water(RawTile &out) {
    struct WRing {
        std::vector<LL> r;
        double s, w, n, e;
    };
    std::vector<WRing> rings;
    for (auto &a : out.areas) {
        if (a.kind != AreaKind::Water || a.rings.empty()) continue;
        WRing w{a.rings[0], 1e9, 1e9, -1e9, -1e9};
        for (auto &p : w.r) w.s = std::min(w.s, p.lat), w.n = std::max(w.n, p.lat), w.w = std::min(w.w, p.lon), w.e = std::max(w.e, p.lon);
        rings.push_back(std::move(w));
    }
    if (rings.empty() && out.waterlines.empty()) return;
    auto pip = [](const std::vector<LL> &r, LL p) {
        bool in = false;
        for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++)
            if ((r[i].lat > p.lat) != (r[j].lat > p.lat) && p.lon < (r[j].lon - r[i].lon) * (p.lat - r[i].lat) / (r[j].lat - r[i].lat) + r[i].lon) in = !in;
        return in;
    };
    mark_bridges(out, [&](LL p) {
        for (auto &w : rings)
            if (p.lat >= w.s && p.lat <= w.n && p.lon >= w.w && p.lon <= w.e && pip(w.r, p)) return true;
        double kx = 111320 * std::cos(p.lat * PI / 180);
        for (auto &[pts, wd] : out.waterlines)
            for (size_t i = 0; i + 1 < pts.size(); i++) {
                double ax = (pts[i].lon - p.lon) * kx, ay = (pts[i].lat - p.lat) * 111320, bx = (pts[i + 1].lon - p.lon) * kx, by = (pts[i + 1].lat - p.lat) * 111320;
                double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
                double t = l2 > 0 ? std::clamp(-(ax * dx + ay * dy) / l2, 0.0, 1.0) : 0;
                double ex = ax + dx * t, ey = ay + dy * t;
                if (ex * ex + ey * ey < (wd / 2 + 0.5) * (wd / 2 + 0.5)) return true;
            }
        return false;
    }, 10, 24);
}

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
                {
                    auto hp = f.prop("has_parts");
                    if (hp && ((hp->kind == mvt::Value::Bool && hp->b) || f.str("has_parts") == "true")) set_tag(rb.tags, "earthsim:has_parts", "yes");
                }
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
            if (f.str("subtype") == "rail") {
                std::string cls = f.str("class");
                if (cls == "subway" || cls == "funicular") continue;  // underground / special
                std::string rf = f.str("rail_flags");
                if (whole_flag(rf, "is_tunnel") || whole_flag(rf, "is_underground")) continue;
                bool br = whole_flag(rf, "is_bridge");
                for (auto &part : f.parts) {
                    if (part.size() < 2) continue;
                    auto pts = cv.line(part);
                    for (auto [s0, e0] : clip_runs_ll(pts, eb)) out.lines.push_back({std::vector<LL>(pts.begin() + s0, pts.begin() + e0 + 1), "rail:" + (cls.empty() ? std::string("unknown") : cls), br});
                }
                continue;
            }
            if (f.str("subtype") != "road") continue;
            std::string cls = f.str("class"), sub = f.str("subclass");
            std::string flags = f.str("road_flags");
            if (whole_flag(flags, "is_underground")) continue;
            bool whole_tunnel = whole_flag(flags, "is_tunnel");
            bool whole_bridge = whole_flag(flags, "is_bridge");
            static const std::set<std::string> pathc = {"footway", "pedestrian", "path", "cycleway", "steps", "track", "bridleway", "sidewalk", "crosswalk"};
            // bridges and tunnels over part of a segment ("between" fractions of its length): split there. The
            // fractions refer to the whole segment, so only when the tile holds all of it (one part, not clipped).
            auto br = flag_ranges(flags, "is_bridge"), tu = flag_ranges(flags, "is_tunnel");
            bool unclipped = f.parts.size() == 1;
            for (auto &q : f.parts.empty() ? std::vector<std::pair<int, int>>{} : f.parts[0])
                if (q.first < 0 || q.second < 0 || q.first > (int)L.extent || q.second > (int)L.extent) unclipped = false;
            for (auto &part : f.parts) {
                if (part.size() < 2) continue;
                auto full = cv.line(part);
                // pieces: (points, state) with state 0 road, 1 bridge, 2 tunnel
                std::vector<std::pair<std::vector<LL>, int>> pieces;
                if (unclipped && (!br.empty() || !tu.empty())) {
                    std::vector<double> cum(full.size(), 0);
                    for (size_t i = 1; i < full.size(); i++)
                        cum[i] = cum[i - 1] + std::hypot((full[i].lat - full[i - 1].lat) * 111320, (full[i].lon - full[i - 1].lon) * 111320 * std::cos(full[i].lat * PI / 180));
                    double tot = std::max(cum.back(), 1e-9);
                    auto state_at = [&](double fr) {
                        for (auto &r : tu)
                            if (fr > r.first && fr < r.second) return 2;
                        for (auto &r : br)
                            if (fr > r.first && fr < r.second) return 1;
                        return 0;
                    };
                    std::vector<double> cuts = {0, 1};
                    for (auto *rs : {&br, &tu})
                        for (auto &r : *rs) cuts.push_back(std::clamp(r.first, 0.0, 1.0)), cuts.push_back(std::clamp(r.second, 0.0, 1.0));
                    std::sort(cuts.begin(), cuts.end());
                    auto at = [&](double fr) {
                        double d = fr * tot;
                        size_t i = 1;
                        while (i + 1 < full.size() && cum[i] < d) i++;
                        double t = std::clamp((d - cum[i - 1]) / std::max(cum[i] - cum[i - 1], 1e-9), 0.0, 1.0);
                        return LL{full[i - 1].lat + (full[i].lat - full[i - 1].lat) * t, full[i - 1].lon + (full[i].lon - full[i - 1].lon) * t};
                    };
                    for (size_t k = 0; k + 1 < cuts.size(); k++) {
                        double a = cuts[k], b = cuts[k + 1];
                        if (b - a < 1e-6) continue;
                        std::vector<LL> pc = {at(a)};
                        for (size_t i = 0; i < full.size(); i++)
                            if (cum[i] / tot > a + 1e-9 && cum[i] / tot < b - 1e-9) pc.push_back(full[i]);
                        pc.push_back(at(b));
                        int st = state_at((a + b) / 2);
                        if (!pieces.empty() && pieces.back().second == st) pieces.back().first.insert(pieces.back().first.end(), pc.begin() + 1, pc.end());
                        else pieces.push_back({pc, st});
                    }
                } else
                    pieces.push_back({full, whole_tunnel ? 2 : whole_bridge ? 1 : 0});
                for (auto &[pts, state] : pieces)
                for (auto [s, e] : clip_runs_ll(pts, eb)) {
                    if (state == 2 && pathc.count(cls)) continue;  // footway tunnels: not drawn
                    bool bridge = state == 1, tunnel = state == 2;
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
                    r.tunnel = tunnel;
                    r.oneway = oneway_of(f.str("access_restrictions"));
                    if (hw == "motorway" || hw == "motorway_link") r.oneway = r.oneway ? r.oneway : 1;
                    r.name = f.str("@name");
                    // whole-segment level (overpasses), speed limit and width from the rules
                    {
                        Json j;
                        auto whole_num = [&](const std::string &txt, const char *key, const char *sub) -> double {
                            if (txt.empty() || !Json::parse(txt, j) || j.kind != Json::Arr) return NAN;
                            for (auto &e : j.a) {
                                if (e.get("between") || e.get("when")) continue;
                                const Json *v = e.get(key);
                                if (v && sub) v = v->get(sub);
                                if (v && v->kind == Json::Num) return v->n;
                            }
                            return NAN;
                        };
                        double lv = whole_num(f.str("level_rules"), "value", nullptr);
                        if (!std::isnan(lv)) {
                            if (lv < 0 && !tunnel) continue;  // underground garages and the like
                            r.layer = (int)std::max(0.0, lv);
                        }
                        double sp = whole_num(f.str("speed_limits"), "max_speed", "value");
                        if (!std::isnan(sp)) r.speed = (int)sp;
                        double wd = whole_num(f.str("width_rules"), "value", nullptr);
                        if (!std::isnan(wd) && wd >= 3 && wd <= 40) r.width = wd, r.width_from_data = true;
                    }
                    out.roads.push_back(std::move(r));
                }
            }
        }
    }

    // base: water, land, land use, land cover; rivers given as lines; trees and lamps as points
    std::vector<std::vector<LL>> bridge_polys;
    for (auto &L : base) {
        Conv cv{base_src ? *base_src : src, L.extent};
        for (auto &f : L.features) {
            if (f.kind == mvt::Geom::Polygon && L.name == "infrastructure" && f.str("subtype") == "bridge") {
                for (auto &poly : polygons(f))
                    if (!poly.empty()) bridge_polys.push_back(cv.line(poly[0]));
                continue;
            }
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
                std::string kind;
                if (cls == "tree" || st == "tree") kind = "tree";
                else if (cls == "street_lamp") kind = "lamp";
                // street furniture of the infrastructure layer, in the same words as the OSM reader
                else if (cls == "traffic_signals") kind = "signal";
                else if (cls == "stop" && st == "transportation") kind = "stop";
                else if (cls == "give_way") kind = "giveway";
                else if (cls == "crossing") kind = "crossing:yes";
                else if (cls == "bus_stop" || cls == "tram_stop") kind = "busstop|" + f.str("@name");
                else if (cls == "bench") kind = "bench";
                else if (cls == "waste_basket" || cls == "recycling") kind = "bin";
                else if (cls == "catenary_mast" || cls == "power_pole" || cls == "power_tower" || cls == "pole") kind = "pole:" + cls;
                else if (cls == "subway_station" || cls == "subway_entrance") kind = "metro|" + f.str("@name");
                else if (cls == "artwork" || cls == "statue" || cls == "sculpture") kind = "artwork|" + f.str("artwork_type");
                else if (cls == "fountain" && st == "water") kind = "fountain";
                if (kind.empty()) continue;
                if (kind != "tree" && kind != "lamp") out.has_furniture = true;
                for (auto &part : f.parts)
                    for (auto &p : part) {
                        LL q = cv.ll(p);
                        if (in_tile(q)) out.points.push_back({q, kind});
                    }
            } else if (f.kind == mvt::Geom::Line && L.name == "infrastructure") {
                std::string cls = f.str("class"), st = f.str("subtype");
                std::string kind;
                if (cls == "power_line" || cls == "minor_line") kind = "power:" + cls;
                else if (cls == "fence" || cls == "wall" || cls == "hedge" || cls == "retaining_wall" || cls == "city_wall" || cls == "guard_rail") kind = "barrier:" + cls;
                else if ((cls == "bridge" || cls == "viaduct") && st == "bridge") kind = "bridge";
                if (kind.empty()) continue;
                for (auto &part : f.parts) {
                    if (part.size() < 2) continue;
                    auto pts = cv.line(part);
                    for (auto [s0, e0] : clip_runs_ll(pts, eb)) out.lines.push_back({std::vector<LL>(pts.begin() + s0, pts.begin() + e0 + 1), kind});
                }
            }
        }
    }
    // mapped bridge outlines (man_made=bridge areas) and bridge lines: roads on them are bridges there
    if (!bridge_polys.empty())
        mark_bridges(out, [&](LL p) {
            for (auto &r : bridge_polys)
                if (point_in_ll(p, r)) return true;
            return false;
        }, 1, 10);
    bridges_over_water(out);
}

}  // namespace ec

namespace ec {

// Overture Places category -> the OSM-style key=value the sign styles know
static std::pair<std::string, std::string> place_kind(const std::string &c) {
    auto has = [&](const char *w) { return c.find(w) != std::string::npos; };
    if (c.empty()) return {"", ""};
    if (has("pharmacy") || has("drugstore")) return {"amenity", "pharmacy"};
    if (has("coffee") || has("cafe") || has("tea_room")) return {"amenity", "cafe"};
    if (has("fast_food") || has("burger") || has("pizza") || has("shawarma") || has("sandwich")) return {"amenity", "fast_food"};
    if (has("restaurant") || has("diner") || has("bistro")) return {"amenity", "restaurant"};
    if (has("bar") && !has("barber")) return {"amenity", "bar"};
    if (has("pub")) return {"amenity", "pub"};
    if (has("bakery") || has("pastry")) return {"shop", "bakery"};
    if (has("supermarket") || has("grocery") || has("hypermarket")) return {"shop", "supermarket"};
    if (has("convenience")) return {"shop", "convenience"};
    if (has("bank") || has("credit_union")) return {"amenity", "bank"};
    if (has("atm")) return {"amenity", "atm"};
    if (has("barber") || has("hair") || has("beauty") || has("nail") || has("cosmetic")) return {"shop", "hairdresser"};
    if (has("florist") || has("flower")) return {"shop", "florist"};
    if (has("mobile_phone") || has("electronics") || has("computer")) return {"shop", "mobile_phone"};
    if (has("clothing") || has("fashion") || has("shoe") || has("boutique")) return {"shop", "clothes"};
    if (has("jewel")) return {"shop", "jewelry"};
    if (has("optic") || has("eyewear")) return {"shop", "optician"};
    if (has("book")) return {"shop", "books"};
    if (has("hotel") || has("hostel") || has("motel")) return {"tourism", "hotel"};
    if (has("gas_station") || has("fuel")) return {"amenity", "fuel"};
    if (has("car_wash")) return {"amenity", "car_wash"};
    if (has("dentist") || has("dental")) return {"amenity", "dentist"};
    if (has("clinic") || has("doctor") || has("medical")) return {"amenity", "clinic"};
    if (has("post_office")) return {"amenity", "post_office"};
    if (has("cinema") || has("movie")) return {"amenity", "cinema"};
    if (has("store") || has("shop") || has("market") || has("retail")) return {"shop", "yes"};
    if (has("office") || has("agency") || has("lawyer") || has("notary") || has("insurance") || has("company")) return {"office", "yes"};
    return {"", ""};
}

static std::string json_primary(const std::string &txt, const char *a, const char *b = nullptr) {
    if (txt.empty() || (txt[0] != '{' && txt[0] != '[')) return txt;
    Json j;
    if (!Json::parse(txt, j)) return "";
    const Json *v = j.get(a);
    if (v && b) v = v->get(b);
    return v && v->kind == Json::Str ? v->s : "";
}

int overture_places(TileId tile, const std::vector<mvt::Layer> &places, RawTile &out, TileId src) {
    Bounds b = tile_bounds(tile);
    // existing (OSM) places, to skip duplicates
    std::vector<LL> have;
    for (auto &p : out.points)
        if (p.kind.rfind("poi:", 0) == 0) have.push_back(p.p);
    int added = 0;
    for (auto &L : places) {
        Conv cv{src, L.extent};
        for (auto &f : L.features) {
            if (f.kind != mvt::Geom::Point || f.parts.empty() || f.parts[0].empty()) continue;
            double cf;
            if (f.num("confidence", cf) && cf < 0.5) continue;
            std::string name = f.str("@name");
            if (name.empty()) name = f.str("names.primary");
            if (name.empty()) name = json_primary(f.str("names"), "primary");
            if (name.empty()) continue;
            // the tiles may carry the category in several shapes depending on the release's tiling profile
            std::string cat = f.str("basic_category");
            for (const char *k : {"categories.primary", "@category", "category", "taxonomy.primary", "class"})
                if (cat.empty()) cat = f.str(k);
            if (cat.empty()) cat = json_primary(f.str("categories"), "primary");
            if (cat.empty()) cat = json_primary(f.str("taxonomy"), "primary");
            auto [k, v] = place_kind(cat);
            if (k.empty()) {
                static const char *NOT_BUSINESS[] = {"park", "bus", "station", "stop", "church", "school", "monument", "landmark", "residential", "street", "river", "lake", "city", "neighborhood", "plaza", "square", "parking", "atm"};
                bool skip = false;
                for (auto w : NOT_BUSINESS) skip |= cat.find(w) != std::string::npos;
                if (skip) continue;
                k = "shop", v = "yes";
            }
            std::string brand = f.str("brand.names.primary");
            if (brand.empty()) brand = json_primary(f.str("brand"), "names", "primary");
            LL q = cv.ll(f.parts[0][0]);
            if (q.lat < b.s || q.lat >= b.n || q.lon < b.w || q.lon >= b.e) continue;
            bool dup = false;
            double kx = 111320 * std::cos(q.lat * PI / 180);
            for (auto &h : have)
                if (std::hypot((h.lat - q.lat) * 111320, (h.lon - q.lon) * kx) < 12) {
                    dup = true;
                    break;
                }
            if (dup) continue;
            for (char &ch : name)
                if (ch == '|') ch = '/';
            out.points.push_back({q, "poi:" + k + "=" + v + "|" + name + "|" + brand});
            have.push_back(q);
            added++;
        }
    }
    return added;
}

}  // namespace ec
