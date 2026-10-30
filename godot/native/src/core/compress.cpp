#include "compress.hpp"
#include "../../third_party/miniz/miniz.h"
#include <cstring>

namespace ec {

static bool inflate_raw_p(const unsigned char *p, size_t n, std::string &out) {
    tinfl_decompressor d;
    tinfl_init(&d);
    out.clear();
    std::string buf(TINFL_LZ_DICT_SIZE, '\0');
    size_t in_pos = 0, dict_ofs = 0;
    for (;;) {
        size_t in_bytes = n - in_pos, out_bytes = TINFL_LZ_DICT_SIZE - dict_ofs;
        tinfl_status st = tinfl_decompress(&d, p + in_pos, &in_bytes, (mz_uint8 *)&buf[0], (mz_uint8 *)&buf[dict_ofs], &out_bytes,
                                           in_pos + in_bytes < n ? TINFL_FLAG_HAS_MORE_INPUT : 0);
        in_pos += in_bytes;
        out.append(&buf[dict_ofs], out_bytes);
        dict_ofs = (dict_ofs + out_bytes) & (TINFL_LZ_DICT_SIZE - 1);
        if (st == TINFL_STATUS_DONE) return true;
        if (st < 0) return false;
        if (st == TINFL_STATUS_NEEDS_MORE_INPUT && in_pos >= n) return false;
    }
}

bool inflate_any(const std::string &in, std::string &out) {
    const unsigned char *p = (const unsigned char *)in.data();
    size_t n = in.size();
    if (n >= 18 && p[0] == 0x1f && p[1] == 0x8b && p[2] == 8) {
        int flg = p[3];
        size_t i = 10;
        if (flg & 4) {
            if (i + 2 > n) return false;
            i += 2 + (p[i] | (p[i + 1] << 8));
        }
        if (flg & 8) {
            while (i < n && p[i]) i++;
            i++;
        }
        if (flg & 16) {
            while (i < n && p[i]) i++;
            i++;
        }
        if (flg & 2) i += 2;
        if (i >= n) return false;
        return inflate_raw_p(p + i, n - i - 8, out);
    }
    if (n >= 2 && (p[0] & 0x0f) == 8 && ((p[0] << 8) | p[1]) % 31 == 0) return inflate_raw_p(p + 2, n - 2, out);
    return inflate_raw_p(p, n, out);
}

bool inflate_raw(const std::string &in, std::string &out, size_t expect) {
    out.reserve(expect);
    return inflate_raw_p((const unsigned char *)in.data(), in.size(), out);
}

}  // namespace ec
