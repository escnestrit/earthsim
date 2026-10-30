#include "zip.hpp"
#include <algorithm>
#include "../core/compress.hpp"

namespace ec {

namespace {
uint16_t u16(const unsigned char *b) { return (uint16_t)(b[0] | b[1] << 8); }
uint32_t u32(const unsigned char *b) { return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24; }
uint64_t u64(const unsigned char *b) { return (uint64_t)u32(b) | (uint64_t)u32(b + 4) << 32; }
bool seek(FILE *f, uint64_t o) {
#ifdef _WIN32
    return _fseeki64(f, (long long)o, SEEK_SET) == 0;
#else
    return fseeko(f, (off_t)o, SEEK_SET) == 0;
#endif
}
uint64_t file_len(FILE *f) {
#ifdef _WIN32
    _fseeki64(f, 0, SEEK_END);
    return (uint64_t)_ftelli64(f);
#else
    fseeko(f, 0, SEEK_END);
    return (uint64_t)ftello(f);
#endif
}
}  // namespace

uint32_t crc32(const std::string &d) {
    static uint32_t T[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            T[i] = c;
        }
        init = true;
    }
    uint32_t c = ~0u;
    for (unsigned char b : d) c = T[(c ^ b) & 0xFF] ^ (c >> 8);
    return ~c;
}

Zip::~Zip() {
    if (f_) fclose(f_);
}

bool Zip::open(const std::string &p, std::string &err) {
    path = p;
    f_ = fopen(p.c_str(), "rb");
    if (!f_) {
        err = p + ": cannot open";
        return false;
    }
    uint64_t len = file_len(f_);
    uint64_t tl = std::min<uint64_t>(len, 65557 + 22);
    std::vector<unsigned char> tail(tl);
    seek(f_, len - tl);
    if (fread(tail.data(), 1, tl, f_) != tl) {
        err = "zip: read failed";
        return false;
    }
    long eocd = -1;
    for (long i = (long)tl - 22; i >= 0; i--)
        if (tail[i] == 0x50 && tail[i + 1] == 0x4b && tail[i + 2] == 5 && tail[i + 3] == 6) {
            eocd = i;
            break;
        }
    if (eocd < 0) {
        err = "zip: end of central directory not found";
        return false;
    }
    uint64_t count = u16(&tail[eocd + 10]), cd_size = u32(&tail[eocd + 12]), cd_off = u32(&tail[eocd + 16]);
    if ((cd_off == 0xFFFFFFFFu || count == 0xFFFF || cd_size == 0xFFFFFFFFu) && eocd >= 20 && tail[eocd - 20] == 0x50 && tail[eocd - 19] == 0x4b && tail[eocd - 18] == 6 && tail[eocd - 17] == 7) {
        uint64_t rec = u64(&tail[eocd - 20 + 8]);
        unsigned char r[56];
        seek(f_, rec);
        if (fread(r, 1, 56, f_) != 56 || !(r[0] == 0x50 && r[1] == 0x4b && r[2] == 6 && r[3] == 6)) {
            err = "zip64: bad end record";
            return false;
        }
        count = u64(r + 32), cd_size = u64(r + 40), cd_off = u64(r + 48);
    }
    if (cd_off + cd_size > len) {
        err = "zip: central directory out of range";
        return false;
    }
    std::vector<unsigned char> cd(cd_size);
    seek(f_, cd_off);
    if (fread(cd.data(), 1, cd_size, f_) != cd_size) {
        err = "zip: read failed";
        return false;
    }
    size_t p0 = 0;
    entries.reserve(count);
    while (p0 + 46 <= cd.size() && cd[p0] == 0x50 && cd[p0 + 1] == 0x4b && cd[p0 + 2] == 1 && cd[p0 + 3] == 2) {
        ZipEntry e;
        e.method = u16(&cd[p0 + 10]);
        e.crc = u32(&cd[p0 + 16]);
        e.csize = u32(&cd[p0 + 20]);
        e.size = u32(&cd[p0 + 24]);
        size_t nl = u16(&cd[p0 + 28]), el = u16(&cd[p0 + 30]), cl = u16(&cd[p0 + 32]);
        e.offset = u32(&cd[p0 + 42]);
        if (p0 + 46 + nl + el > cd.size()) break;
        e.name.assign((const char *)&cd[p0 + 46], nl);
        std::replace(e.name.begin(), e.name.end(), '\\', '/');
        const unsigned char *ex = &cd[p0 + 46 + nl];
        for (size_t q = 0; q + 4 <= el;) {
            uint16_t id = u16(ex + q);
            size_t sz = u16(ex + q + 2);
            if (id == 1) {
                size_t o = q + 4;
                if (e.size == 0xFFFFFFFFu && o + 8 <= el) e.size = u64(ex + o), o += 8;
                if (e.csize == 0xFFFFFFFFu && o + 8 <= el) e.csize = u64(ex + o), o += 8;
                if (e.offset == 0xFFFFFFFFu && o + 8 <= el) e.offset = u64(ex + o);
            }
            q += 4 + sz;
        }
        entries.push_back(std::move(e));
        p0 += 46 + nl + el + cl;
    }
    return true;
}

bool Zip::read_prefix(const ZipEntry &e, uint64_t n, std::string &out, std::string &err) {
    if (e.method != 0) return read(e, out, err);
    unsigned char h[30];
    if (!seek(f_, e.offset) || fread(h, 1, 30, f_) != 30 || !(h[0] == 0x50 && h[1] == 0x4b && h[2] == 3 && h[3] == 4)) {
        err = "zip: bad local header for " + e.name;
        return false;
    }
    uint64_t skip = u16(h + 26) + u16(h + 28);
    n = std::min(n, e.size);
    out.assign(n, '\0');
    seek(f_, e.offset + 30 + skip);
    return n == 0 || fread(&out[0], 1, n, f_) == n;
}

const ZipEntry *Zip::find(const std::string &name) const {
    for (auto &e : entries)
        if (e.name == name) return &e;
    return nullptr;
}

bool Zip::read(const ZipEntry &e, std::string &out, std::string &err) {
    unsigned char h[30];
    if (!seek(f_, e.offset) || fread(h, 1, 30, f_) != 30 || !(h[0] == 0x50 && h[1] == 0x4b && h[2] == 3 && h[3] == 4)) {
        err = "zip: bad local header for " + e.name;
        return false;
    }
    uint64_t skip = u16(h + 26) + u16(h + 28);
    std::string comp(e.csize, '\0');
    seek(f_, e.offset + 30 + skip);
    if (e.csize && fread(&comp[0], 1, e.csize, f_) != e.csize) {
        err = "zip: truncated " + e.name;
        return false;
    }
    if (e.method == 0) out = std::move(comp);
    else if (e.method == 8) {
        if (!inflate_raw(comp, out, e.size)) {
            err = e.name + ": inflate failed";
            return false;
        }
    } else {
        err = e.name + ": unsupported compression";
        return false;
    }
    if (out.size() != e.size || crc32(out) != e.crc) {
        err = e.name + ": CRC mismatch";
        return false;
    }
    return true;
}

}  // namespace ec
