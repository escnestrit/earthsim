// PMTiles v3 archive reader over a random-access byte source (HTTP range requests or a local file).
#pragma once
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "util.hpp"

namespace ec {

// Reads `len` bytes at `offset`; returns false with `err` on failure.
using RangeFn = std::function<bool(uint64_t offset, uint64_t len, std::string &out, std::string &err)>;

struct PmEntry {
    uint64_t tile_id = 0, offset = 0, length = 0, run_length = 0;
};

class PmTiles {
public:
    // Opens the archive (reads header and root directory).
    bool open(RangeFn src, GunzipFn gunzip, std::string &err);
    // Tile bytes, decompressed; returns true with empty `out` when the tile is not in the archive.
    bool get(int z, int x, int y, std::string &out, std::string &err);
    int min_zoom = 0, max_zoom = 0;

    static uint64_t tile_id(int z, uint32_t x, uint32_t y);
    static bool parse_directory(const std::string &b, std::vector<PmEntry> &out);

private:
    RangeFn src_;
    GunzipFn gunzip_;
    uint64_t leaf_offset_ = 0, data_offset_ = 0;
    int internal_comp_ = 0, tile_comp_ = 0;
    std::vector<PmEntry> root_;
    std::mutex mu_;
    std::map<uint64_t, std::shared_ptr<std::vector<PmEntry>>> leaves_;
    bool decompress(int kind, const std::string &in, std::string &out, std::string &err);
};

}  // namespace ec
