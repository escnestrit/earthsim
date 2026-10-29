#include "pmtiles.hpp"
#include <algorithm>
#include <cstring>

namespace ec {

static uint64_t u64le(const std::string &b, size_t o) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) v = (v << 8) | (uint8_t)b[o + i];
    return v;
}

uint64_t PmTiles::tile_id(int z, uint32_t x, uint32_t y) {
    uint64_t acc = ((1ull << (2 * z)) - 1) / 3;
    uint64_t n = 1ull << z;
    uint64_t tx = x, ty = y, d = 0;
    for (uint64_t s = n / 2; s > 0; s /= 2) {
        uint64_t rx = (tx & s) > 0, ry = (ty & s) > 0;
        d += s * s * ((3 * rx) ^ ry);
        if (ry == 0) {
            if (rx == 1) {
                tx = n - 1 - tx;
                ty = n - 1 - ty;
            }
            std::swap(tx, ty);
        }
    }
    return acc + d;
}

static bool varint(const std::string &b, size_t &p, uint64_t &v) {
    v = 0;
    int s = 0;
    while (p < b.size()) {
        uint8_t c = b[p++];
        v |= uint64_t(c & 0x7f) << s;
        if (!(c & 0x80)) return true;
        s += 7;
        if (s > 63) return false;
    }
    return false;
}

bool PmTiles::parse_directory(const std::string &b, std::vector<PmEntry> &e) {
    size_t p = 0;
    uint64_t n;
    if (!varint(b, p, n) || n > 10000000) return false;
    e.assign(n, PmEntry());
    uint64_t last = 0, v;
    for (auto &x : e) {
        if (!varint(b, p, v)) return false;
        last += v;
        x.tile_id = last;
    }
    for (auto &x : e)
        if (!varint(b, p, x.run_length)) return false;
    for (auto &x : e)
        if (!varint(b, p, x.length)) return false;
    for (size_t i = 0; i < n; i++) {
        if (!varint(b, p, v)) return false;
        e[i].offset = (v == 0 && i > 0) ? e[i - 1].offset + e[i - 1].length : v - 1;
    }
    return true;
}

bool PmTiles::decompress(int kind, const std::string &in, std::string &out, std::string &err) {
    if (kind == 0 || kind == 1) {
        out = in;
        return true;
    }
    if (kind == 2) {
        if (gunzip_ && gunzip_(in, out)) return true;
        err = "gzip data could not be read";
        return false;
    }
    err = "unsupported compression " + std::to_string(kind);
    return false;
}

bool PmTiles::open(RangeFn src, GunzipFn gunzip, std::string &err) {
    src_ = std::move(src);
    gunzip_ = std::move(gunzip);
    std::string first;
    if (!src_(0, 16384, first, err)) return false;
    if (first.size() < 127 || first.compare(0, 7, "PMTiles") != 0) {
        err = "not a PMTiles archive";
        return false;
    }
    if ((uint8_t)first[7] != 3) {
        err = "unsupported PMTiles version";
        return false;
    }
    uint64_t root_off = u64le(first, 8), root_len = u64le(first, 16);
    leaf_offset_ = u64le(first, 40);
    data_offset_ = u64le(first, 56);
    internal_comp_ = (uint8_t)first[97];
    tile_comp_ = (uint8_t)first[98];
    min_zoom = (uint8_t)first[100];
    max_zoom = (uint8_t)first[101];
    std::string raw;
    if (root_off + root_len <= first.size())
        raw = first.substr(root_off, root_len);
    else if (!src_(root_off, root_len, raw, err))
        return false;
    std::string dir;
    if (!decompress(internal_comp_, raw, dir, err)) return false;
    if (!parse_directory(dir, root_)) {
        err = "bad root directory";
        return false;
    }
    return true;
}

static const PmEntry *find(const std::vector<PmEntry> &e, uint64_t id) {
    auto it = std::upper_bound(e.begin(), e.end(), id, [](uint64_t v, const PmEntry &x) { return v < x.tile_id; });
    if (it == e.begin()) return nullptr;
    const PmEntry &x = *(it - 1);
    if (x.run_length == 0 || id < x.tile_id + x.run_length) return &x;
    return nullptr;
}

bool PmTiles::get(int z, int x, int y, std::string &out, std::string &err) {
    out.clear();
    uint64_t id = tile_id(z, (uint32_t)x, (uint32_t)y);
    std::shared_ptr<std::vector<PmEntry>> dir;
    const std::vector<PmEntry> *d = &root_;
    for (int depth = 0; depth < 4; depth++) {
        const PmEntry *e = find(*d, id);
        if (!e) return true;
        if (e->run_length > 0) {
            std::string raw;
            if (!src_(data_offset_ + e->offset, e->length, raw, err)) return false;
            return decompress(tile_comp_, raw, out, err);
        }
        uint64_t key = e->offset, len = e->length;
        {
            std::lock_guard<std::mutex> g(mu_);
            auto it = leaves_.find(key);
            if (it != leaves_.end()) dir = it->second;
            else dir.reset();
        }
        if (!dir) {
            std::string raw, dec;
            if (!src_(leaf_offset_ + key, len, raw, err)) return false;
            if (!decompress(internal_comp_, raw, dec, err)) return false;
            auto nd = std::make_shared<std::vector<PmEntry>>();
            if (!parse_directory(dec, *nd)) {
                err = "bad leaf directory";
                return false;
            }
            std::lock_guard<std::mutex> g(mu_);
            if (leaves_.size() > 256) leaves_.clear();
            leaves_[key] = nd;
            dir = nd;
        }
        d = dir.get();
    }
    err = "directory too deep";
    return false;
}

}  // namespace ec
