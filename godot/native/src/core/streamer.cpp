#include "streamer.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include "compress.hpp"
#include "enrich.hpp"
#include "mvt.hpp"
#include "overture.hpp"

namespace stdfs = std::filesystem;

namespace ec {

namespace {

bool read_file(const std::string &p, std::string &out) {
    std::ifstream f(p, std::ios::binary);
    if (!f) return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}
void write_file(const std::string &p, const std::string &data) {
    std::error_code ec;
    stdfs::create_directories(stdfs::path(p).parent_path(), ec);
    std::string tmp = p + ".part" + std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id()) % 100000);
    {
        std::ofstream f(tmp, std::ios::binary);
        if (!f) return;
        f.write(data.data(), (std::streamsize)data.size());
    }
    stdfs::rename(tmp, p, ec);
    if (ec) stdfs::remove(tmp, ec);
}

uint64_t k2(int x, int y) { return ((uint64_t)(uint32_t)x << 32) | (uint32_t)y; }

const char *OVERPASS[] = {"https://overpass-api.de/api/interpreter", "https://overpass.private.coffee/api/interpreter", "https://overpass.kumi.systems/api/interpreter"};

std::string urlenc(const std::string &s) {
    std::string o;
    char b[4];
    for (unsigned char ch : s) {
        if (isalnum(ch) || ch == '-' || ch == '_' || ch == '.' || ch == '~') o += (char)ch;
        else {
            snprintf(b, sizeof b, "%%%02X", ch);
            o += b;
        }
    }
    return o;
}

}  // namespace

Streamer::Streamer(StreamConfig cfg, HttpFn http) : cfg_(std::move(cfg)), http_(std::move(http)), frame_(cfg_.lat0, cfg_.lon0) {
    for (int i = 0; i < std::max(1, cfg_.net_threads); i++) threads_.emplace_back([this] { net_worker(); });
    for (int i = 0; i < std::max(1, cfg_.build_threads); i++) threads_.emplace_back([this] { build_worker(); });
    if (cfg_.enrich) enrich_thread_ = std::thread([this] { enrich_worker(); });
}

Streamer::~Streamer() {
    stop_ = true;
    net_cv_.notify_all();
    build_cv_.notify_all();
    for (auto &t : threads_) t.join();
    if (enrich_thread_.joinable()) enrich_thread_.join();
}

bool Streamer::fetch_cached(const std::string &rel, const std::string &url, const std::vector<std::string> &headers, const std::string *post, std::string &out, std::string &err) {
    std::string p = cfg_.cache_dir + "/" + rel;
    if (read_file(p, out)) return true;
    if (!cfg_.online || url.empty()) {
        err = "not cached: " + rel;
        return false;
    }
    int status = 0;
    for (int attempt = 0; attempt < 4 && !stop_; attempt++) {
        if (http_(url, headers, post, out, status, err) && (status == 200 || status == 206)) {
            write_file(p, out);
            return true;
        }
        if (status == 404 || status == 403) {
            // not there: remember as empty
            out.clear();
            write_file(p, out);
            return true;
        }
        err = "HTTP " + std::to_string(status) + " " + err + " " + url;
        std::this_thread::sleep_for(std::chrono::milliseconds(500 * (1 << attempt)));
    }
    return false;
}

std::string Streamer::release(std::string &err) {
    std::lock_guard<std::mutex> l(pm_mu_);
    if (!release_.empty()) return release_;
    // newest release present in the cache (offline / pre-fetched data) ...
    std::string best;
    std::error_code ec;
    for (auto &d : stdfs::directory_iterator(cfg_.cache_dir + "/overture", ec))
        if (d.is_directory()) best = std::max(best, d.path().filename().string());
    // ... or listed in the bucket
    if (cfg_.online) {
        std::string xml;
        int st = 0;
        if (http_(std::string(OVERTURE_BUCKET) + "/?list-type=2&prefix=tiles/&delimiter=/", {}, nullptr, xml, st, err) && st == 200) {
            std::string r = overture_newest_release(xml);
            if (!r.empty()) best = std::max(best, r);
        }
    }
    if (best.empty()) best = OVERTURE_FALLBACK_RELEASE;
    release_ = best;
    return best;
}

std::shared_ptr<PmTiles> Streamer::pmtiles(const std::string &theme, std::string &err) {
    std::string rel = release(err);
    {
        std::lock_guard<std::mutex> l(pm_mu_);
        auto it = pm_.find(theme);
        if (it != pm_.end()) return it->second;
    }
    auto pm = std::make_shared<PmTiles>();
    std::string url = std::string(OVERTURE_BUCKET) + "/tiles/" + rel + "/" + theme + ".pmtiles";
    RangeFn rf = [this, url, rel, theme](uint64_t off, uint64_t len, std::string &out, std::string &e) {
        // directory ranges are cached on disk too, by offset
        std::string relp = "overture/" + rel + "/" + theme + "/dir/" + std::to_string(off) + "_" + std::to_string(len);
        return fetch_cached(relp, url, {"Range: bytes=" + std::to_string(off) + "-" + std::to_string(off + len - 1)}, nullptr, out, e);
    };
    if (!pm->open(rf, inflate_any, err)) return nullptr;
    std::lock_guard<std::mutex> l(pm_mu_);
    pm_[theme] = pm;
    return pm;
}

bool Streamer::load_source(int x, int y, std::string &err) {
    std::string rel = release(err);
    std::vector<mvt::Layer> layers[3];
    const char *themes[3] = {"buildings", "transportation", "base"};
    TileId base_src{14, x, y};
    for (int k = 0; k < 3; k++) {
        // a theme may stop below zoom 14 (the base theme does): read its parent tile then
        int z = 14;
        std::shared_ptr<PmTiles> pm;
        if (cfg_.online) {
            pm = pmtiles(themes[k], err);
            if (!pm) return false;
            z = std::min(14, pm->max_zoom);
        } else {
            for (int zz = 14; zz >= 10; zz--) {
                std::error_code ec;
                if (stdfs::exists(cfg_.cache_dir + "/overture/" + rel + "/" + themes[k] + "/" + std::to_string(zz), ec)) {
                    z = zz;
                    break;
                }
            }
        }
        int tx = x >> (14 - z), ty = y >> (14 - z);
        if (k == 2) base_src = TileId{z, tx, ty};
        std::string relp = "overture/" + rel + "/" + themes[k] + "/" + std::to_string(z) + "/" + std::to_string(tx) + "/" + std::to_string(ty) + ".mvt";
        std::string data;
        if (!read_file(cfg_.cache_dir + "/" + relp, data)) {
            if (!pm) {
                data.clear();  // offline and not fetched: nothing there
            } else {
                std::string raw;
                if (!pm->get(z, tx, ty, raw, err)) return false;
                data = raw;
                write_file(cfg_.cache_dir + "/" + relp, data);
            }
        }
        if (!data.empty()) {
            std::string e2;
            if (!mvt::decode(data, layers[k], &e2)) {
                err = "mvt " + std::string(themes[k]) + ": " + e2;
                return false;
            }
        }
    }
    auto t = std::make_shared<RawTile>();
    TileId id{14, x, y};
    overture_features(id, id, layers[0], layers[1], layers[2], 0, *t, &base_src);
    // OSM tags already fetched earlier
    std::string js;
    if (read_file(cfg_.cache_dir + "/osm/enrich_14_" + std::to_string(x) + "_" + std::to_string(y) + ".json", js) && !js.empty()) {
        Json j;
        if (Json::parse(js, j)) merge_osm(*t, j);
        std::lock_guard<std::mutex> l(mu_);
        enrich_done_.insert(k2(x, y));
    }
    std::lock_guard<std::mutex> l(mu_);
    src_[k2(x, y)] = t;
    return true;
}

bool Streamer::load_dem(int x, int y, std::string &err) {
    std::string png;
    std::string relp = "dem/13/" + std::to_string(x) + "/" + std::to_string(y) + ".png";
    if (!fetch_cached(relp, std::string(TERRARIUM_URL) + "/13/" + std::to_string(x) + "/" + std::to_string(y) + ".png", {}, nullptr, png, err)) {
        if (cfg_.online) return false;
        png.clear();  // offline without the tile: flat
    }
    std::vector<float> h;
    if (png.empty() || !terrarium_decode(png, h, err)) h.assign(256 * 256, 0.0f);  // sea / missing: flat
    dem_.add(x, y, std::move(h));
    std::lock_guard<std::mutex> l(mu_);
    dem_ok_.insert(k2(x, y));
    return true;
}

void Streamer::update(double x, double y) {
    std::lock_guard<std::mutex> l(mu_);
    px_ = x, py_ = y;
    if (std::hypot(x - last_sched_x_, y - last_sched_y_) < 8) return;
    last_sched_x_ = x, last_sched_y_ = y;
    schedule_locked();
}

void Streamer::schedule_locked() {
    int r = (int)std::ceil(cfg_.terrain_radius / CHUNK) + 1;
    int pcx = (int)std::floor(px_ / CHUNK), pcy = (int)std::floor(py_ / CHUNK);
    std::set<Key> seen;
    for (int j = pcy - r; j <= pcy + r; j++)
        for (int i = pcx - r; i <= pcx + r; i++) {
            double cxm = (i + 0.5) * CHUNK, cym = (j + 0.5) * CHUNK;
            double d = std::max(0.0, std::hypot(cxm - px_, cym - py_) - CHUNK * 0.7071);
            int want = d <= cfg_.detail_radius ? 2 : d <= cfg_.terrain_radius ? 1 : 0;
            if (!want) continue;
            Key k{i, j};
            seen.insert(k);
            ChunkState &s = chunks_[k];
            s.want = want;
            s.dist = d;
        }
    for (auto it = chunks_.begin(); it != chunks_.end();) {
        if (!seen.count(it->first)) {
            // keep a hysteresis band before dropping
            double cxm = (it->first.x + 0.5) * CHUNK, cym = (it->first.y + 0.5) * CHUNK;
            double d = std::max(0.0, std::hypot(cxm - px_, cym - py_) - CHUNK * 0.7071);
            if (d > cfg_.terrain_radius + 300) {
                if (it->second.built) drops_.push_back({it->first.x, it->first.y});
                coll_->remove(it->first.x, it->first.y);
                lanes_.erase(it->first);
                if (!it->second.queued) {
                    it = chunks_.erase(it);
                    continue;
                }
                it->second.want = 0;
            }
        } else if (it->second.want == 1 && it->second.built == 2) {
            double cxm = (it->first.x + 0.5) * CHUNK, cym = (it->first.y + 0.5) * CHUNK;
            double d = std::max(0.0, std::hypot(cxm - px_, cym - py_) - CHUNK * 0.7071);
            if (d < cfg_.detail_radius + 250) it->second.want = 2;  // hysteresis: keep detail a while
        }
        ++it;
    }
    // what each chunk needs, nearest first
    std::vector<std::pair<double, Key>> order;
    for (auto &[k, s] : chunks_)
        if (s.want && !s.queued && (s.built < s.want || (s.want == 2 && s.built_version != s.version))) order.push_back({s.dist, k});
    std::sort(order.begin(), order.end(), [](auto &a, auto &b) { return a.first < b.first; });
    net_q_.clear();
    std::set<uint64_t> netq_src, netq_dem;
    for (auto &[d, k] : order) {
        ChunkState &s = chunks_[k];
        bool ready = true;
        Bounds b = chunk_bounds(k.x, k.y, frame_, FEATURE_MARGIN + 8);
        for (auto &t : Dem::tiles_for(b)) {
            uint64_t key = k2(t.x, t.y);
            if (dem_ok_.count(key)) continue;
            ready = false;
            if (!dem_pending_.count(key) && !netq_dem.count(key)) {
                net_q_.push_back({1, t.x, t.y, d});
                netq_dem.insert(key);
            }
        }
        if (s.want == 2) {
            TileId a = tile_of(b.n, b.w, 14), c = tile_of(b.s, b.e, 14);
            for (int y = a.y; y <= c.y; y++)
                for (int x = a.x; x <= c.x; x++) {
                    uint64_t key = k2(x, y);
                    if (src_.count(key)) continue;
                    ready = false;
                    if (!src_pending_.count(key) && !netq_src.count(key)) {
                        net_q_.push_back({0, x, y, d});
                        netq_src.insert(key);
                    }
                }
        }
        if (ready) {
            s.queued = true;
            build_q_.push_back({k, s.want});
        }
    }
    // builds nearest first
    std::stable_sort(build_q_.begin(), build_q_.end(), [&](auto &a, auto &b) { return chunks_[a.first].dist < chunks_[b.first].dist; });
    net_cv_.notify_all();
    build_cv_.notify_all();
}

void Streamer::net_worker() {
    while (!stop_) {
        NetJob job;
        {
            std::unique_lock<std::mutex> l(mu_);
            net_cv_.wait_for(l, std::chrono::milliseconds(250), [&] { return stop_ || !net_q_.empty(); });
            if (stop_) return;
            if (net_q_.empty()) continue;
            job = net_q_.front();
            net_q_.pop_front();
            uint64_t key = k2(job.x, job.y);
            auto &pend = job.kind == 0 ? src_pending_ : dem_pending_;
            if (pend.count(key) || (job.kind == 0 ? src_.count(key) > 0 : dem_ok_.count(key) > 0)) continue;
            pend.insert(key);
            net_running_++;
        }
        std::string err;
        bool ok = job.kind == 0 ? load_source(job.x, job.y, err) : load_dem(job.x, job.y, err);
        {
            std::lock_guard<std::mutex> l(mu_);
            net_running_--;
            (job.kind == 0 ? src_pending_ : dem_pending_).erase(k2(job.x, job.y));
            if (!ok) {
                last_error_ = err;
                fails_++;
            }
            last_sched_x_ = 1e30;  // re-plan on the next update
            schedule_locked();
        }
        if (!ok) std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void Streamer::build_worker() {
    while (!stop_) {
        std::pair<Key, int> job;
        std::vector<std::shared_ptr<const RawTile>> tiles;
        int version = 0;
        {
            std::unique_lock<std::mutex> l(mu_);
            build_cv_.wait_for(l, std::chrono::milliseconds(250), [&] { return stop_ || !build_q_.empty(); });
            if (stop_) return;
            if (build_q_.empty()) continue;
            job = build_q_.front();
            build_q_.pop_front();
            auto it = chunks_.find(job.first);
            if (it == chunks_.end()) continue;
            if (it->second.want == 0) {
                it->second.queued = false;
                continue;
            }
            job.second = it->second.want;
            version = it->second.version;
            if (job.second == 2) {
                Bounds b = chunk_bounds(job.first.x, job.first.y, frame_, FEATURE_MARGIN + 8);
                TileId a = tile_of(b.n, b.w, 14), c = tile_of(b.s, b.e, 14);
                for (int y = a.y; y <= c.y; y++)
                    for (int x = a.x; x <= c.x; x++) {
                        auto s = src_.find(k2(x, y));
                        if (s != src_.end()) tiles.push_back(s->second);
                    }
            }
            builds_running_++;
        }
        auto out = std::make_shared<ChunkOut>(build_chunk(job.first.x, job.first.y, tiles, dem_, frame_));
        {
            auto cc = std::make_shared<ColChunk>();
            cc->cx = out->cx, cc->cy = out->cy;
            cc->grid = out->grid;
            cc->layer = out->layer;
            cc->tris = out->coll;
            cc->index();
            coll_->add(cc);
            std::lock_guard<std::mutex> l(mu_);
            lanes_[job.first] = out->lanes;
        }
        {
            std::lock_guard<std::mutex> l(mu_);
            builds_running_--;
            auto it = chunks_.find(job.first);
            if (it != chunks_.end()) {
                it->second.queued = false;
                if (it->second.want) {
                    it->second.built = job.second;
                    it->second.built_version = job.second == 2 ? version : -1;
                    done_.push_back({out, job.second == 1, version});
                }
            }
            last_sched_x_ = 1e30;
            schedule_locked();
        }
    }
}

void Streamer::enrich_worker() {
    // one request at a time, politely spaced; nearest tiles first
    int ep = 0;
    while (!stop_) {
        int x = 0, y = 0;
        bool have = false;
        {
            std::lock_guard<std::mutex> l(mu_);
            double best = 1e300;
            for (auto &[k, t] : src_) {
                if (enrich_done_.count(k)) continue;
                Bounds b = tile_bounds(t->tile);
                V2 c = frame_.to_local((b.s + b.n) / 2, (b.w + b.e) / 2);
                double d = std::hypot(c.x - px_, c.y - py_);
                if (d < best && d < cfg_.detail_radius + 1500) best = d, x = t->tile.x, y = t->tile.y, have = true;
            }
        }
        if (!have || !cfg_.online) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        Bounds b = tile_bounds({14, x, y});
        std::string body = "data=" + urlenc(osm_enrich_query(b));
        std::string out, err;
        int status = 0;
        bool ok = false;
        for (int attempt = 0; attempt < 3 && !stop_ && !ok; attempt++) {
            ok = http_(OVERPASS[ep % 3], {"Content-Type: application/x-www-form-urlencoded", "User-Agent: earthsim/0.5 (game; github.com/escnestrit/earthsim)"}, &body, out, status, err) && status == 200 &&
                 !out.empty() && out.find('{') < 16;
            if (!ok) {
                ep++;
                std::this_thread::sleep_for(std::chrono::seconds(status == 429 ? 20 : 4));
            }
        }
        Json j;
        if (ok && Json::parse(out, j)) {
            write_file(cfg_.cache_dir + "/osm/enrich_14_" + std::to_string(x) + "_" + std::to_string(y) + ".json", out);
            std::lock_guard<std::mutex> l(mu_);
            auto it = src_.find(k2(x, y));
            if (it != src_.end()) {
                auto t = std::make_shared<RawTile>(*it->second);
                merge_osm(*t, j);
                it->second = t;
                // chunks that use this tile get rebuilt
                for (auto &[k, s] : chunks_) {
                    Bounds cb = chunk_bounds(k.x, k.y, frame_, FEATURE_MARGIN + 8);
                    if (cb.n < b.s || cb.s > b.n || cb.e < b.w || cb.w > b.e) continue;
                    s.version++;
                }
                last_sched_x_ = 1e30;
                schedule_locked();
            }
            enrich_done_.insert(k2(x, y));
        } else {
            std::lock_guard<std::mutex> l(mu_);
            last_error_ = "Overpass: " + std::to_string(status) + " " + err;
            enrich_done_.insert(k2(x, y));  // build without; retried next session
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
}

bool Streamer::poll(ChunkResult &out) {
    std::lock_guard<std::mutex> l(mu_);
    if (done_.empty()) return false;
    out = std::move(done_.front());
    done_.pop_front();
    return true;
}

std::vector<std::pair<int, int>> Streamer::take_drops() {
    std::lock_guard<std::mutex> l(mu_);
    std::vector<std::pair<int, int>> d;
    d.swap(drops_);
    return d;
}

double Streamer::ground(double x, double y) {
    double la, lo;
    frame_.to_latlon({x, y}, la, lo);
    return dem_.height(la, lo, std::nan(""));
}

bool Streamer::wait_ground(double x, double y, double timeout_s) {
    update(x, y);
    auto t0 = std::chrono::steady_clock::now();
    while (std::isnan(ground(x, y))) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return true;
}

bool Streamer::nearest_road(double x, double y, double radius, V3 &pos, double &heading) {
    std::lock_guard<std::mutex> l(mu_);
    double best = radius;
    bool found = false;
    V2 p(x, y);
    for (auto &[k, v] : lanes_)
        for (auto &s : v) {
            if (s.prio < 2) continue;
            V2 a(s.a.x, s.a.y), b(s.b.x, s.b.y), d = b - a;
            double l2 = d.dot(d);
            if (l2 < 1e-6) continue;
            double t = std::clamp((p - a).dot(d) / l2, 0.0, 1.0);
            V2 q = a + d * t;
            double dist = (q - p).len();
            if (dist < best) {
                best = dist;
                found = true;
                V2 dir = d * (1 / std::sqrt(l2));
                if (s.oneway < 0) dir = dir * -1;
                // keep right on two-way roads
                V2 off = s.oneway == 0 ? V2(dir.y, -dir.x) * (s.hw * 0.5) : V2();
                pos = V3(q.x + off.x, q.y + off.y, s.a.z + (s.b.z - s.a.z) * t);
                heading = std::atan2(dir.y, dir.x);
            }
        }
    return found;
}

std::string Streamer::status() {
    std::lock_guard<std::mutex> l(mu_);
    int full = 0, terr = 0, want_full = 0;
    for (auto &[k, s] : chunks_) {
        full += s.built == 2;
        terr += s.built == 1;
        want_full += s.want == 2;
    }
    char b[256];
    snprintf(b, sizeof b, "chunks %d/%d detail, %d terrain | net %d (%zu queued) | builds %d (%zu queued) | tiles %zu, osm %zu", full, want_full, terr, net_running_, net_q_.size(),
             builds_running_, build_q_.size(), src_.size(), enrich_done_.size());
    std::string s = b;
    if (!last_error_.empty()) s += " | " + last_error_.substr(0, 120);
    return s;
}

}  // namespace ec
