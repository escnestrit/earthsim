#include "region.hpp"
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include "json.hpp"
#include "mvt.hpp"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#include "../third_party/stb_image.h"

namespace fs = std::filesystem;

namespace es {

bool read_file(const std::string &path, std::string &out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

uint32_t parse_color(const std::string &s) {
    if (s.size() == 7 && s[0] == '#') {
        char *e = nullptr;
        unsigned long v = strtoul(s.c_str() + 1, &e, 16);
        if (e && *e == 0) return uint32_t(v) | 0x1000000u;
    }
    return 0;
}

// ---------------------------------------------------------------- DEM

bool Dem::add_tile(int tx, int ty, const uint8_t *png, size_t n) {
    int w0, h0, c;
    uint8_t *px = stbi_load_from_memory(png, int(n), &w0, &h0, &c, 3);
    if (!px) return false;
    if (w0 != 256 || h0 != 256) {
        stbi_image_free(px);
        return false;
    }
    int ox = (tx - x0) * 256, oy = (ty - y0) * 256;
    if (ox < 0 || oy < 0 || ox + 256 > w || oy + 256 > h) {
        stbi_image_free(px);
        return false;
    }
    for (int y = 0; y < 256; y++)
        for (int x = 0; x < 256; x++) {
            const uint8_t *p = px + (y * 256 + x) * 3;
            hgt[size_t(oy + y) * w + ox + x] = float(p[0] * 256.0 + p[1] + p[2] / 256.0 - 32768.0);
        }
    stbi_image_free(px);
    return true;
}

void Dem::finish() {
    // fill holes (NaN) with the mean of valid samples
    double s = 0;
    size_t k = 0;
    for (float v : hgt)
        if (std::isfinite(v) && v > -500) s += v, k++;
    base = k ? s / k : 0;
    for (float &v : hgt)
        if (!std::isfinite(v) || v <= -500) v = float(base);
    // SRTM-derived heights in cities carry roofs and tree canopy; real urban ground is much smoother.
    // Separable Gaussian, sigma 2.2 px (~25 m at z13).
    const double sigma = 2.2;
    const int R = 7;
    double kw[2 * R + 1], ks = 0;
    for (int i = -R; i <= R; i++) ks += kw[i + R] = std::exp(-i * i / (2 * sigma * sigma));
    for (double &x : kw) x /= ks;
    std::vector<float> tmp(hgt.size());
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            double s = 0;
            for (int i = -R; i <= R; i++) s += kw[i + R] * hgt[size_t(y) * w + std::clamp(x + i, 0, w - 1)];
            tmp[size_t(y) * w + x] = float(s);
        }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            double s = 0;
            for (int i = -R; i <= R; i++) s += kw[i + R] * tmp[size_t(std::clamp(y + i, 0, h - 1)) * w + x];
            hgt[size_t(y) * w + x] = float(s);
        }
}

double Dem::sample(V2 p) const {
    if (hgt.empty()) return 0;
    double mx, my;
    frame->to_merc(p, mx, my);
    double n = double(1 << z);
    // pixel centres: sample i covers [i, i+1) -> centre at i+0.5
    double fx = (mx * n - x0) * 256 - 0.5, fy = (my * n - y0) * 256 - 0.5;
    int ix = int(std::floor(fx)), iy = int(std::floor(fy));
    double tx = fx - ix, ty = fy - iy;
    // uniform cubic B-spline weights
    auto wts = [](double t, double *wv) {
        double t2 = t * t, t3 = t2 * t;
        wv[0] = (1 - 3 * t + 3 * t2 - t3) / 6;
        wv[1] = (4 - 6 * t2 + 3 * t3) / 6;
        wv[2] = (1 + 3 * t + 3 * t2 - 3 * t3) / 6;
        wv[3] = t3 / 6;
    };
    double wx[4], wy[4];
    wts(tx, wx), wts(ty, wy);
    double r = 0;
    for (int j = 0; j < 4; j++) {
        int yy = std::clamp(iy - 1 + j, 0, h - 1);
        double row = 0;
        for (int i = 0; i < 4; i++) {
            int xx = std::clamp(ix - 1 + i, 0, w - 1);
            row += wx[i] * hgt[size_t(yy) * w + xx];
        }
        r += wy[j] * row;
    }
    return r;
}

// ---------------------------------------------------------------- helpers

namespace {

struct Piece {
    std::vector<V2> pts;
};

std::vector<Range> ranges_for(const Json &rules, const char *flag) {
    std::vector<Range> out;
    if (rules.kind != Json::Arr) return out;
    for (auto &r : rules.a) {
        bool hit = false;
        for (auto &v : r["values"].a)
            if (v.s == flag) hit = true;
        if (!hit) continue;
        Range g;
        if (r["between"].size() == 2) g.a = r["between"][0].num(), g.b = r["between"][1].num(1);
        out.push_back(g);
    }
    return out;
}

AreaKind landuse_kind(const std::string &c, bool &keep) {
    keep = true;
    if (c == "grass" || c == "meadow" || c == "village_green" || c == "recreation_ground" || c == "greenfield" || c == "grassland") return AreaKind::Grass;
    if (c == "park" || c == "garden" || c == "dog_park" || c == "zoo") return AreaKind::Park;
    if (c == "forest" || c == "wood") return AreaKind::Forest;
    if (c == "scrub" || c == "heath" || c == "shrub" || c == "shrubland") return AreaKind::Scrub;
    if (c == "sand" || c == "beach") return AreaKind::Sand;
    if (c == "pitch" || c == "track" || c == "stadium" || c == "golf_course") return AreaKind::Pitch;
    if (c == "playground" || c == "pedestrian" || c == "plaza" || c == "square") return AreaKind::Paved;
    if (c == "parking") return AreaKind::Parking;
    if (c == "residential") return AreaKind::Residential;
    if (c == "industrial" || c == "railway" || c == "garages" || c == "commercial" || c == "retail") return AreaKind::Industrial;
    if (c == "farmland" || c == "allotments" || c == "orchard" || c == "farmyard" || c == "plant_nursery") return AreaKind::Farmland;
    if (c == "cemetery" || c == "grave_yard") return AreaKind::Cemetery;
    if (c == "construction" || c == "brownfield" || c == "landfill") return AreaKind::Construction;
    keep = false;
    return AreaKind::Other;
}

// MVT polygon parts -> rings (outer + holes) in local metres, CCW outer.
std::vector<Ring> to_rings(const Frame &F, const TileId &t, const mvt::Feature &ft, int ext, const Rect &clip) {
    std::vector<Ring> out;
    for (auto &part : ft.parts) {
        if (part.size() < 3) continue;
        double a = mvt::ring_area(part);
        if (a == 0) continue;
        Poly p;
        p.reserve(part.size());
        for (auto &q : part) p.push_back(F.from_tile(t, ext, q.x, q.y));
        if (a > 0) {
            out.push_back({});
            out.back().outer = std::move(p);
        } else if (!out.empty())
            out.back().holes.push_back(std::move(p));
    }
    std::vector<Ring> res;
    for (auto &r : out) {
        Ring c;
        c.outer = clip_poly(r.outer, clip);
        if (c.outer.size() < 3 || std::fabs(poly_area(c.outer)) < 0.5) continue;
        if (poly_area(c.outer) < 0) std::reverse(c.outer.begin(), c.outer.end());
        for (auto &h : r.holes) {
            Poly hc = clip_poly(h, clip);
            if (hc.size() < 3) continue;
            if (poly_area(hc) > 0) std::reverse(hc.begin(), hc.end());
            c.holes.push_back(std::move(hc));
        }
        res.push_back(std::move(c));
    }
    return res;
}

bool load_mvt(const std::string &path, std::vector<mvt::Layer> &L) {
    std::string d;
    if (!read_file(path, d)) return false;
    return mvt::decode((const uint8_t *)d.data(), d.size(), L);
}

std::string tile_path(const std::string &root, const std::string &rel, const char *theme, TileId t) {
    char b[160];
    snprintf(b, sizeof b, "/overture/%s/%s/%d/%d/%d.mvt", rel.c_str(), theme, t.z, t.x, t.y);
    return root + b;
}

}  // namespace

// ---------------------------------------------------------------- loader

bool Region::load(const std::string &root) {
    std::string js;
    if (!read_file(root + "/region.json", js)) {
        log.push_back("no region.json in " + root);
        return false;
    }
    Json R = Json::parse(js);
    frame = Frame(R["lat"].num(), R["lon"].num());
    release = R["release"].s;
    for (auto &t : R["tiles14"].a) tiles.push_back({14, int(t[0].num()), int(t[1].num())});
    if (tiles.empty()) {
        log.push_back("region.json lists no tiles");
        return false;
    }
    for (auto &t : tiles) {
        V2 a, b;
        frame.tile_rect(t, a, b);
        bounds.add(a), bounds.add(b);
    }

    // ---- elevation
    {
        std::vector<std::pair<TileId, std::string>> files;
        fs::path dd = fs::path(root) / "dem";
        std::error_code ec;
        if (fs::exists(dd, ec))
            for (auto &zdir : fs::directory_iterator(dd, ec))
                for (auto &xdir : fs::directory_iterator(zdir.path(), ec))
                    for (auto &f : fs::directory_iterator(xdir.path(), ec)) {
                        if (f.path().extension() != ".png") continue;
                        TileId t{atoi(zdir.path().filename().string().c_str()), atoi(xdir.path().filename().string().c_str()), atoi(f.path().stem().string().c_str())};
                        files.push_back({t, f.path().string()});
                    }
        if (!files.empty()) {
            int z = files[0].first.z, x0 = 1 << 30, y0 = 1 << 30, x1 = -1, y1 = -1;
            for (auto &f : files) {
                if (f.first.z != z) continue;
                x0 = std::min(x0, f.first.x), y0 = std::min(y0, f.first.y), x1 = std::max(x1, f.first.x), y1 = std::max(y1, f.first.y);
            }
            dem.frame = &frame;
            dem.z = z, dem.x0 = x0, dem.y0 = y0, dem.nx = x1 - x0 + 1, dem.ny = y1 - y0 + 1;
            dem.w = dem.nx * 256, dem.h = dem.ny * 256;
            dem.hgt.assign(size_t(dem.w) * dem.h, NAN);
            int ok = 0;
            for (auto &f : files) {
                std::string d;
                if (f.first.z == z && read_file(f.second, d) && dem.add_tile(f.first.x, f.first.y, (const uint8_t *)d.data(), d.size())) ok++;
            }
            dem.finish();
            log.push_back("dem: " + std::to_string(ok) + " tiles z" + std::to_string(z));
        } else
            log.push_back("dem: none (flat ground)");
        dem.frame = &frame;
    }

    // ---- transportation
    std::map<std::string, size_t> road_idx;
    std::vector<std::vector<Piece>> road_pieces;
    for (auto &t : tiles) {
        std::vector<mvt::Layer> L;
        if (!load_mvt(tile_path(root, release, "transportation", t), L)) {
            log.push_back("missing transportation tile " + std::to_string(t.x) + "/" + std::to_string(t.y));
            continue;
        }
        V2 a, b;
        frame.tile_rect(t, a, b);
        Rect clip{a, b};
        for (auto &l : L) {
            if (l.name == "connector") {
                for (auto &ft : l.features) {
                    if (ft.parts.empty() || ft.parts[0].empty()) continue;
                    V2 p = frame.from_tile(t, l.extent, ft.parts[0][0].x, ft.parts[0][0].y);
                    auto &c = connectors[ft.str("id")];
                    if (!c.have || clip.contains(p)) c.p = p, c.have = true;
                }
            } else if (l.name == "segment") {
                for (auto &ft : l.features) {
                    if (ft.kind != mvt::Geom::Line) continue;
                    std::string id = ft.str("id");
                    auto it = road_idx.find(id);
                    if (it == road_idx.end()) {
                        RawRoad r;
                        r.id = id;
                        r.cls = ft.str("class");
                        r.subclass = ft.str("subclass");
                        r.name = ft.str("@name");
                        std::string st = ft.str("subtype");
                        if (st == "rail") r.kind = RoadKind::Rail;
                        else if (st == "water") continue;
                        else if (r.cls == "footway" || r.cls == "pedestrian" || r.cls == "steps" || r.cls == "cycleway" || r.cls == "path" || r.cls == "bridleway" || r.cls == "sidewalk" || r.cls == "crosswalk")
                            r.kind = RoadKind::Path;
                        Json flags = Json::parse(ft.str(r.kind == RoadKind::Rail ? "rail_flags" : "road_flags"));
                        r.bridge = ranges_for(flags, "is_bridge");
                        r.tunnel = ranges_for(flags, "is_tunnel");
                        Json lv = Json::parse(ft.str("level_rules"));
                        for (auto &e : lv.a) {
                            Range g;
                            if (e["between"].size() == 2) g.a = e["between"][0].num(), g.b = e["between"][1].num(1);
                            int v = int(e["value"].num());
                            if (v) r.level.push_back({g, v});
                        }
                        Json sp = Json::parse(ft.str("speed_limits"));
                        if (sp.size()) {
                            double v = sp[0]["max_speed"]["value"].num();
                            if (sp[0]["max_speed"]["unit"].s == "mph") v *= 1.609;
                            r.speed = int(v + 0.5);
                        }
                        Json wr = Json::parse(ft.str("width_rules"));
                        for (auto &e : wr.a) r.width = std::max(r.width, e["value"].num());
                        Json su = Json::parse(ft.str("road_surface"));
                        if (su.size()) r.surface = su[0]["value"].s;
                        Json ar = Json::parse(ft.str("access_restrictions"));
                        for (auto &e : ar.a) {
                            if (e["access_type"].s != "denied") continue;
                            const Json &w = e["when"];
                            bool motor = !w.has("mode");
                            for (auto &m : w["mode"].a)
                                if (m.s == "motor_vehicle" || m.s == "vehicle" || m.s == "car") motor = true;
                            if (!motor || w.has("recognized") || w.has("during") || w.has("using")) continue;
                            if (w["heading"].s == "backward") r.oneway = 1;
                            else if (w["heading"].s == "forward") r.oneway = -1;
                        }
                        Json cn = Json::parse(ft.str("connectors"));
                        for (auto &e : cn.a) r.connectors.push_back({e["connector_id"].s, e["at"].num()});
                        road_idx[id] = roads.size();
                        roads.push_back(std::move(r));
                        road_pieces.emplace_back();
                        it = road_idx.find(id);
                    }
                    for (auto &part : ft.parts) {
                        std::vector<V2> pl;
                        for (auto &q : part) {
                            V2 p = frame.from_tile(t, l.extent, q.x, q.y);
                            if (pl.empty() || dist(pl.back(), p) > 0.01) pl.push_back(p);
                        }
                        for (auto &c : clip_line(pl, clip)) road_pieces[it->second].push_back({std::move(c)});
                    }
                }
            }
        }
    }
    // chain the clipped pieces of every segment back together; split where chaining fails
    {
        std::vector<RawRoad> out;
        out.reserve(roads.size());
        for (size_t i = 0; i < roads.size(); i++) {
            auto &P = road_pieces[i];
            std::vector<bool> used(P.size(), false);
            // start pieces: whose start does not continue another piece's end
            for (size_t guard = 0; guard < P.size(); guard++) {
                size_t start = SIZE_MAX;
                for (size_t a = 0; a < P.size() && start == SIZE_MAX; a++) {
                    if (used[a]) continue;
                    bool cont = false;
                    for (size_t b = 0; b < P.size(); b++)
                        if (b != a && !used[b] && dist(P[b].pts.back(), P[a].pts.front()) < 1.0) cont = true;
                    if (!cont) start = a;
                }
                if (start == SIZE_MAX)
                    for (size_t a = 0; a < P.size(); a++)
                        if (!used[a]) {
                            start = a;
                            break;
                        }
                if (start == SIZE_MAX) break;
                std::vector<V2> chain = P[start].pts;
                used[start] = true;
                for (bool grew = true; grew;) {
                    grew = false;
                    for (size_t b = 0; b < P.size(); b++)
                        if (!used[b] && dist(chain.back(), P[b].pts.front()) < 1.0) {
                            chain.insert(chain.end(), P[b].pts.begin() + 1, P[b].pts.end());
                            used[b] = true;
                            grew = true;
                        }
                }
                if (polyline_length(chain) < 0.3) continue;
                RawRoad r = roads[i];
                r.pts = std::move(chain);
                out.push_back(std::move(r));
            }
        }
        roads.swap(out);
    }
    // map Overture fractions onto each chain using connectors with known positions
    for (auto &r : roads) {
        double L = polyline_length(r.pts);
        std::vector<std::pair<double, double>> fs_;  // (fraction, arc)
        for (auto &c : r.connectors) {
            auto it = connectors.find(c.first);
            if (it == connectors.end() || !it->second.have) continue;
            double best = 1e9, arc = 0, acc = 0;
            for (size_t k = 0; k + 1 < r.pts.size(); k++) {
                double t;
                double d = seg_dist(it->second.p, r.pts[k], r.pts[k + 1], &t);
                double sl = dist(r.pts[k], r.pts[k + 1]);
                if (d < best) best = d, arc = acc + t * sl;
                acc += sl;
            }
            if (best < 2.0) fs_.push_back({c.second, arc});
        }
        bool touches = !bounds.grown(-1.5).contains(r.pts.front()) || !bounds.grown(-1.5).contains(r.pts.back());
        if (fs_.size() >= 2) {
            double mf = 0, ms = 0;
            for (auto &q : fs_) mf += q.first, ms += q.second;
            mf /= fs_.size(), ms /= fs_.size();
            double sff = 0, sfs = 0;
            for (auto &q : fs_) sff += (q.first - mf) * (q.first - mf), sfs += (q.first - mf) * (q.second - ms);
            if (sff > 1e-9 && sfs > 0) {
                double b = sfs / sff, a = ms - b * mf;
                r.frac0 = (0 - a) / b, r.frac1 = (L - a) / b;
                continue;
            }
        }
        // otherwise (whole segment inside, or unknown scale at the region edge) treat the chain as the whole segment
        (void)touches;
        r.frac0 = 0, r.frac1 = 1;
    }
    log.push_back("roads: " + std::to_string(roads.size()) + " chains, connectors " + std::to_string(connectors.size()));

    // ---- buildings
    {
        std::map<std::string, size_t> bidx;
        for (auto &t : tiles) {
            std::vector<mvt::Layer> L;
            if (!load_mvt(tile_path(root, release, "buildings", t), L)) continue;
            V2 a, b;
            frame.tile_rect(t, a, b);
            Rect clip{a, b};
            for (auto &l : L) {
                bool part = l.name == "building_part";
                if (!part && l.name != "building") continue;
                for (auto &ft : l.features) {
                    if (ft.kind != mvt::Geom::Polygon) continue;
                    std::string id = ft.str("id");
                    auto rings = to_rings(frame, t, ft, l.extent, clip);
                    if (rings.empty()) continue;
                    auto it = bidx.find(id);
                    if (it == bidx.end()) {
                        RawBuilding B;
                        B.id = id;
                        B.part = part;
                        B.parent = ft.str("building_id");
                        B.has_parts = ft.flag("has_parts");
                        B.height = ft.num("height");
                        B.min_height = ft.num("min_height");
                        B.roof_height = ft.num("roof_height");
                        B.roof_dir = ft.get("roof_direction") ? ft.num("roof_direction") : -1;
                        B.floors = int(ft.num("num_floors"));
                        B.min_floor = int(ft.num("min_floor"));
                        B.cls = ft.str("class");
                        B.subtype = ft.str("subtype");
                        B.roof_shape = ft.str("roof_shape");
                        B.roof_material = ft.str("roof_material");
                        B.facade_material = ft.str("facade_material");
                        B.facade_color = parse_color(ft.str("facade_color"));
                        B.roof_color = parse_color(ft.str("roof_color"));
                        B.underground = ft.flag("is_underground");
                        B.name = ft.str("@name");
                        bidx[id] = buildings.size();
                        buildings.push_back(std::move(B));
                        it = bidx.find(id);
                    }
                    auto &dst = buildings[it->second].rings;
                    for (auto &r : rings) dst.push_back(std::move(r));
                }
            }
        }
        log.push_back("buildings: " + std::to_string(buildings.size()));
    }

    // ---- base: land, land use, water, infrastructure (z13 tiles overlapping the region)
    {
        fs::path bd = fs::path(root) / "overture" / release / "base";
        std::error_code ec;
        std::vector<TileId> bt;
        if (fs::exists(bd, ec))
            for (auto &zdir : fs::directory_iterator(bd, ec))
                for (auto &xdir : fs::directory_iterator(zdir.path(), ec))
                    for (auto &f : fs::directory_iterator(xdir.path(), ec))
                        if (f.path().extension() == ".mvt")
                            bt.push_back({atoi(zdir.path().filename().string().c_str()), atoi(xdir.path().filename().string().c_str()), atoi(f.path().stem().string().c_str())});
        Rect keep = bounds.grown(50);
        for (auto &t : bt) {
            std::vector<mvt::Layer> L;
            if (!load_mvt(tile_path(root, release, "base", t), L)) continue;
            V2 a, b;
            frame.tile_rect(t, a, b);
            Rect clip{a, b};
            if (!clip.overlaps(keep)) continue;
            // clip to the intersection of the tile and the region (keeps data bounded)
            Rect cr{{std::max(clip.mn.x, keep.mn.x), std::max(clip.mn.y, keep.mn.y)}, {std::min(clip.mx.x, keep.mx.x), std::min(clip.mx.y, keep.mx.y)}};
            for (auto &l : L) {
                for (auto &ft : l.features) {
                    std::string cls = ft.str("class");
                    if (ft.num("level") < 0 || ft.flag("is_underground")) continue;
                    if (ft.kind == mvt::Geom::Polygon) {
                        AreaKind k;
                        bool ok = false;
                        if (l.name == "water") k = AreaKind::Water, ok = cls != "swimming_pool" && cls != "fountain";
                        else if (l.name == "land" || l.name == "land_use") k = landuse_kind(cls, ok);
                        else if (l.name == "infrastructure") {
                            if (cls == "parking" || cls == "parking_space") k = AreaKind::Parking, ok = cls == "parking";
                            else if (ft.str("subtype") == "pedestrian" && (cls == "pedestrian" || cls == "plaza")) k = AreaKind::Paved, ok = true;
                        }
                        if (!ok) continue;
                        for (auto &r : to_rings(frame, t, ft, l.extent, cr)) areas.push_back({k, std::move(r), cls});
                    } else if (ft.kind == mvt::Geom::Line) {
                        std::string kind;
                        double w = 0;
                        if (l.name == "water") {
                            kind = "waterline";
                            w = cls == "river" ? 18 : cls == "canal" ? 8 : cls == "stream" ? 3 : 1.5;
                        } else if (l.name == "land" && cls == "tree_row") kind = "tree_row";
                        else if (l.name == "infrastructure" && ft.str("subtype") == "barrier") kind = cls;  // fence, wall, retaining_wall...
                        else if (l.name == "infrastructure" && ft.str("subtype") == "power") kind = "power_" + cls;
                        if (kind.empty()) continue;
                        for (auto &part : ft.parts) {
                            std::vector<V2> pl;
                            for (auto &q : part) pl.push_back(frame.from_tile(t, l.extent, q.x, q.y));
                            for (auto &c : clip_line(pl, cr)) lines.push_back({kind, std::move(c), w});
                        }
                    } else if (ft.kind == mvt::Geom::Point) {
                        if (ft.parts.empty() || ft.parts[0].empty()) continue;
                        V2 p = frame.from_tile(t, l.extent, ft.parts[0][0].x, ft.parts[0][0].y);
                        if (!cr.contains(p)) continue;
                        std::string kind;
                        if (l.name == "land" && cls == "tree") kind = "tree";
                        else if (l.name == "infrastructure") {
                            std::string tags = ft.str("source_tags");
                            if (tags.find("\"traffic_signals\"") != std::string::npos && tags.find("\"highway\":\"traffic_signals\"") != std::string::npos) kind = "traffic_signals";
                            else if (cls == "crossing") kind = tags.find("traffic_signals") != std::string::npos ? "crossing_signals" : "crossing";
                            else if (tags.find("street_lamp") != std::string::npos) kind = "street_lamp";
                            else kind = cls;
                        }
                        if (!kind.empty()) points.push_back({kind, p, ft.str("@name")});
                    }
                }
            }
        }
        log.push_back("areas " + std::to_string(areas.size()) + ", lines " + std::to_string(lines.size()) + ", points " + std::to_string(points.size()));
    }
    return true;
}

}  // namespace es
