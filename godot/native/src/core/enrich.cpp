// OSM tags (Overpass answer of the enrich query) merged onto the Overture features of a tile.
#include "enrich.hpp"
#include <algorithm>

namespace ec {

static const char *OVERRIDE[] = {"height", "building:levels", "min_height", "building:min_level", "roof:levels"};
static const char *FILL[] = {"roof:shape", "roof:height", "building:colour", "roof:colour", "building:material", "roof:material", "building:series", "start_date",
                             "building:architecture", "amenity", "religion", "denomination", "shop", "office", "tower:type", "tourism", "historic", "man_made",
                             "name", "building:use", "leisure", "entrance"};

static double ring_area_ll(const std::vector<LL> &r) {
    double a = 0;
    for (size_t i = 0, n = r.size(); i < n; i++) a += r[i].lon * r[(i + 1) % n].lat - r[(i + 1) % n].lon * r[i].lat;
    return std::abs(a) / 2;
}

static double seg_dist_m(const LL &p, const LL &a, const LL &b) {
    double k = std::cos(p.lat * PI / 180);
    V2 P(p.lon * k, p.lat), A(a.lon * k, a.lat), B(b.lon * k, b.lat);
    V2 d = B - A;
    double l2 = d.dot(d);
    double t = l2 > 0 ? std::clamp((P - A).dot(d) / l2, 0.0, 1.0) : 0;
    return (A + d * t - P).len() * 111320;
}

int merge_osm(RawTile &t, const Json &osm) {
    const Json *els = osm.get("elements");
    if (!els || els->kind != Json::Arr) return 0;
    Bounds b = tile_bounds(t.tile);
    int changed = 0;
    for (auto &e : els->a) {
        const Json *tags = e.get("tags");
        if (!tags || tags->kind != Json::Obj) continue;
        const Json *type = e.get("type");
        bool node = type && type->str() && *type->str() == "node";
        LL p;
        if (node) {
            p = {e.get("lat") ? e.get("lat")->num() : 0, e.get("lon") ? e.get("lon")->num() : 0};
        } else {
            const Json *c = e.get("center");
            if (!c) continue;
            p = {c->get("lat") ? c->get("lat")->num() : 0, c->get("lon") ? c->get("lon")->num() : 0};
        }
        if (node) {
            if (const Json *en = tags->get("entrance")) {
                if (p.lat >= b.s && p.lat < b.n && p.lon >= b.w && p.lon < b.e) t.points.push_back({p, "entrance:" + (en->str() ? *en->str() : std::string("yes"))});
            }
            continue;
        }
        if (tags->get("building")) {
            // the smallest building outline around the OSM centre point
            RawBuilding *best = nullptr;
            double best_a = 1e300;
            for (auto &bd : t.buildings) {
                if (bd.part || bd.rings.empty() || !point_in_ll(p, bd.rings[0])) continue;
                double a = ring_area_ll(bd.rings[0]);
                if (a < best_a) {
                    best_a = a;
                    best = &bd;
                }
            }
            if (!best) continue;
            bool did = false;
            auto osmv = [&](const char *k) -> const std::string * {
                const Json *v = tags->get(k);
                return v ? v->str() : nullptr;
            };
            // OSM's class when Overture only knows "a building"
            if (auto c = osmv("building"); c && *c != "yes") {
                for (auto &kv : best->tags)
                    if (kv.first == "building" && kv.second == "yes") {
                        kv.second = *c;
                        did = true;
                    }
            }
            for (auto k : OVERRIDE)
                if (auto v = osmv(k)) {
                    if (tagv(best->tags, k) != *v) {
                        set_tag(best->tags, k, *v);
                        did = true;
                    }
                }
            for (auto k : FILL)
                if (auto v = osmv(k)) {
                    if (!tag(best->tags, k)) {
                        best->tags.push_back({k, *v});
                        did = true;
                    }
                }
            // a level count from OSM beats an Overture height estimated from the footprint
            if (osmv("building:levels") && !osmv("height")) {
                auto &tg = best->tags;
                tg.erase(std::remove_if(tg.begin(), tg.end(), [](auto &kv) { return kv.first == "height"; }), tg.end());
            }
            if (did) changed++;
            continue;
        }
        if (const Json *hw = tags->get("highway")) {
            (void)hw;
            // lanes / width / oneway onto the nearest road (centre within 8 m of its line)
            RawRoad *best = nullptr;
            double bd = 8;
            for (auto &r : t.roads)
                for (size_t i = 0; i + 1 < r.pts.size(); i++) {
                    double d = seg_dist_m(p, r.pts[i], r.pts[i + 1]);
                    if (d < bd) {
                        bd = d;
                        best = &r;
                    }
                }
            if (!best) continue;
            auto sv = [&](const char *k) -> std::string {
                const Json *v = tags->get(k);
                return v && v->str() ? *v->str() : std::string();
            };
            double v;
            if (!sv("width").empty() && parse_num(sv("width"), v) && v > 2 && v < 60) {
                best->width = v;
                best->width_from_data = true;
            } else if (!sv("lanes").empty() && parse_num(sv("lanes"), v) && v >= 1 && v <= 12) {
                best->width = v * 3.3;
                best->width_from_data = true;
            }
            std::string ow = sv("oneway");
            if (ow == "yes" || ow == "true" || ow == "1") best->oneway = 1;
            else if (ow == "-1" || ow == "reverse") best->oneway = -1;
            else if (ow == "no") best->oneway = 0;
            changed++;
        }
    }
    return changed;
}

}  // namespace ec

namespace ec {
std::string osm_enrich_query(const Bounds &b) {
    char bb[128];
    snprintf(bb, sizeof bb, "%.6f,%.6f,%.6f,%.6f", b.s, b.w, b.n, b.e);
    std::string B = bb;
    return "[out:json][timeout:60];(way[\"building\"][\"building:levels\"](" + B + ");way[\"building\"][\"height\"](" + B +
           ");relation[\"building\"][\"building:levels\"](" + B + ");relation[\"building\"][\"height\"](" + B +
           ");way[\"building\"][~\"^(amenity|shop|office|tourism|historic|man_made|name|building:use)$\"~\".\"](" + B +
           ");way[\"building\"~\"^(church|cathedral|chapel|mosque|temple|synagogue|school|university|college|kindergarten|hospital|office|retail|commercial|supermarket|garages|garage|civic|public|government|train_station)$\"](" +
           B + "););out tags center;node[\"entrance\"](" + B + ");out body;(way[\"highway\"][\"oneway\"](" + B + ");way[\"highway\"][\"lanes\"](" + B +
           ");way[\"highway\"][\"width\"](" + B + "););out tags center;";
}
}  // namespace ec
