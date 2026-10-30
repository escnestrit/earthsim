// World streaming: decides which chunks the player needs, fetches what they are made of (Overture tiles over
// HTTP range requests, OSM tags from Overpass, terrain tiles), keeps a disk cache, and builds chunks on worker
// threads, nearest first. Engine independent: HTTP is a function supplied by the caller.
#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <set>
#include <thread>
#include "chunk.hpp"
#include "pmtiles.hpp"
#include "../phys/collide.hpp"
#include "traffic.hpp"
#include "people.hpp"

namespace ec {

// GET (post == nullptr) or POST; headers like "Range: bytes=0-99". Returns false on transport failure.
using HttpFn = std::function<bool(const std::string &url, const std::vector<std::string> &headers, const std::string *post, std::string &out, int &status, std::string &err)>;

struct StreamConfig {
    std::string cache_dir;     // also accepts the layout written by tools/fetch_region.py
    double lat0 = 53.9, lon0 = 27.56;
    double detail_radius = 650;   // full chunks within this distance (m)
    double terrain_radius = 2200; // terrain-only chunks beyond, up to this
    int net_threads = 4;
    int build_threads = 3;
    bool online = true;
    bool enrich = true;        // ask Overpass for OSM building tags / entrances
};

struct ChunkResult {
    std::shared_ptr<ChunkOut> chunk;
    bool terrain_only = false;
    int version = 0;
};

class Streamer {
public:
    Streamer(StreamConfig cfg, HttpFn http);
    ~Streamer();
    const LocalFrame &frame() const { return frame_; }
    // static collision of the loaded chunks, for the soft-body vehicles
    std::shared_ptr<CollisionWorld> collision() const { return coll_; }
    // player position in the local frame; cheap, call every frame
    void update(double x, double y);
    // a finished chunk, if any
    bool poll(ChunkResult &out);
    // chunks the caller should drop (out of range)
    std::vector<std::pair<int, int>> take_drops();
    // natural ground height at a local point; NaN until the terrain there is loaded
    double ground(double x, double y);
    std::string status();
    // nearest drivable road point to (x, y) within `radius`: position on the centre line (right-hand lane
    // offset applied), heading along the road; false when none is loaded
    bool nearest_road(double x, double y, double radius, V3 &pos, double &heading);
    // advance the traffic around the player (main thread); cars also become obstacles for the soft bodies
    const std::vector<TrafficCar> &traffic_step(double dt, V3 player, V3 player_vel, int count);
    const std::vector<Walker> &people_step(double dt, V3 player, int count);
    // blocks until terrain at (x, y) is loaded or `timeout_s` passes
    bool wait_ground(double x, double y, double timeout_s);

private:
    struct Key {
        int x, y;
        bool operator<(const Key &o) const { return x != o.x ? x < o.x : y < o.y; }
    };
    struct ChunkState {
        int want = 0;       // 0 none, 1 terrain, 2 full
        int built = 0;      // what the caller has (0 none, 1 terrain, 2 full)
        int version = 0;    // bumps when its data changes (enrich arrived)
        int built_version = -1;
        bool queued = false;
        double dist = 0;
    };
    struct NetJob {
        int kind;  // 0 source tile, 1 dem tile, 2 enrich
        int x, y;
        double prio;
    };

    StreamConfig cfg_;
    HttpFn http_;
    LocalFrame frame_;
    Dem dem_;
    std::shared_ptr<CollisionWorld> coll_ = std::make_shared<CollisionWorld>();
    std::string release_;
    std::mutex pm_mu_;
    std::map<std::string, std::shared_ptr<PmTiles>> pm_;

    std::mutex mu_;
    std::condition_variable net_cv_, build_cv_;
    std::atomic<bool> stop_{false};
    double px_ = 0, py_ = 0;
    std::map<Key, ChunkState> chunks_;
    std::map<uint64_t, std::shared_ptr<const RawTile>> src_;  // z14 tiles
    std::map<Key, std::vector<LaneSeg>> lanes_;
    int lanes_version_ = 0, traffic_version_ = -1;
    Traffic traffic_;
    std::map<Key, std::vector<std::pair<V3, V3>>> walks_;
    int walks_version_ = 0, crowd_version_ = -1;
    Crowd crowd_;
    std::set<uint64_t> src_pending_, dem_pending_, enrich_pending_, enrich_done_;
    std::set<uint64_t> dem_ok_;
    std::deque<NetJob> net_q_;
    std::deque<std::pair<Key, int>> build_q_;
    std::deque<ChunkResult> done_;
    std::vector<std::pair<int, int>> drops_;
    std::vector<std::thread> threads_;
    std::thread enrich_thread_;
    int builds_running_ = 0, net_running_ = 0;
    std::string last_error_;
    double last_sched_x_ = 1e30, last_sched_y_ = 1e30;
    int fails_ = 0;

    void schedule_locked();
    void net_worker();
    void build_worker();
    void enrich_worker();
    bool fetch_cached(const std::string &rel_path, const std::string &url, const std::vector<std::string> &headers, const std::string *post, std::string &out, std::string &err);
    bool load_source(int x, int y, std::string &err);
    bool load_dem(int x, int y, std::string &err);
    std::shared_ptr<PmTiles> pmtiles(const std::string &theme, std::string &err);
    std::string release(std::string &err);
};

}  // namespace ec
