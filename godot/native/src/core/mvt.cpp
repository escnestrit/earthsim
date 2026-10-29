#include "mvt.hpp"
#include <cstring>

namespace ec::mvt {

namespace {
struct Pb {
    const uint8_t *b;
    size_t n, p = 0;
    bool ok = true;
    uint64_t varint() {
        uint64_t v = 0;
        int s = 0;
        while (p < n) {
            uint8_t c = b[p++];
            v |= uint64_t(c & 0x7f) << s;
            if (!(c & 0x80)) return v;
            s += 7;
            if (s > 63) break;
        }
        ok = false;
        return 0;
    }
    bool bytes(const uint8_t *&ptr, size_t &len) {
        len = (size_t)varint();
        if (!ok || p + len > n) {
            ok = false;
            return false;
        }
        ptr = b + p;
        p += len;
        return true;
    }
    bool key(uint64_t &field, int &wt) {
        if (p >= n) return false;
        uint64_t k = varint();
        if (!ok) return false;
        field = k >> 3;
        wt = int(k & 7);
        return true;
    }
    void skip(int wt) {
        const uint8_t *x;
        size_t l;
        switch (wt) {
            case 0: varint(); break;
            case 1: p += 8; break;
            case 2: bytes(x, l); break;
            case 5: p += 4; break;
            default: ok = false;
        }
        if (p > n) ok = false;
    }
};

int64_t zz(uint64_t v) { return int64_t(v >> 1) ^ -int64_t(v & 1); }

Value value(const uint8_t *b, size_t n) {
    Pb pb{b, n};
    Value v;
    uint64_t f;
    int wt;
    while (pb.ok && pb.key(f, wt)) {
        if (f == 1 && wt == 2) {
            const uint8_t *s;
            size_t l;
            if (pb.bytes(s, l)) {
                v.kind = Value::Str;
                v.s.assign((const char *)s, l);
            }
        } else if (f == 2 && wt == 5) {
            float x;
            if (pb.p + 4 <= n) memcpy(&x, b + pb.p, 4);
            pb.p += 4;
            v.kind = Value::Num;
            v.n = x;
        } else if (f == 3 && wt == 1) {
            double x;
            if (pb.p + 8 <= n) memcpy(&x, b + pb.p, 8);
            pb.p += 8;
            v.kind = Value::Num;
            v.n = x;
        } else if ((f == 4 || f == 5) && wt == 0) {
            v.kind = Value::Int;
            v.i = (int64_t)pb.varint();
        } else if (f == 6 && wt == 0) {
            v.kind = Value::Int;
            v.i = zz(pb.varint());
        } else if (f == 7 && wt == 0) {
            v.kind = Value::Bool;
            v.b = pb.varint() != 0;
        } else
            pb.skip(wt);
    }
    return v;
}

std::vector<uint64_t> packed(const uint8_t *b, size_t n) {
    Pb pb{b, n};
    std::vector<uint64_t> o;
    while (pb.ok && pb.p < n) o.push_back(pb.varint());
    return o;
}

bool layer(const uint8_t *b, size_t n, Layer &L) {
    Pb pb{b, n};
    std::vector<std::string> keys;
    std::vector<Value> vals;
    struct Raw {
        uint64_t id = 0;
        int type = 0;
        std::vector<uint64_t> tags, geom;
    };
    std::vector<Raw> raws;
    uint64_t f;
    int wt;
    while (pb.ok && pb.key(f, wt)) {
        const uint8_t *s;
        size_t l;
        if (f == 1 && wt == 2) {
            if (pb.bytes(s, l)) L.name.assign((const char *)s, l);
        } else if (f == 2 && wt == 2) {
            if (!pb.bytes(s, l)) break;
            Pb fp{s, l};
            Raw r;
            uint64_t ff;
            int fwt;
            while (fp.ok && fp.key(ff, fwt)) {
                const uint8_t *q;
                size_t ql;
                if (ff == 1 && fwt == 0)
                    r.id = fp.varint();
                else if (ff == 2 && fwt == 2) {
                    if (fp.bytes(q, ql)) r.tags = packed(q, ql);
                } else if (ff == 3 && fwt == 0)
                    r.type = (int)fp.varint();
                else if (ff == 4 && fwt == 2) {
                    if (fp.bytes(q, ql)) r.geom = packed(q, ql);
                } else
                    fp.skip(fwt);
            }
            raws.push_back(std::move(r));
        } else if (f == 3 && wt == 2) {
            if (pb.bytes(s, l)) keys.emplace_back((const char *)s, l);
        } else if (f == 4 && wt == 2) {
            if (pb.bytes(s, l)) vals.push_back(value(s, l));
        } else if (f == 5 && wt == 0) {
            L.extent = (int)pb.varint();
        } else
            pb.skip(wt);
    }
    if (!pb.ok) return false;
    for (auto &r : raws) {
        Feature ft;
        ft.id = r.id;
        ft.kind = (Geom)(r.type <= 3 ? r.type : 0);
        for (size_t i = 0; i + 1 < r.tags.size(); i += 2)
            if (r.tags[i] < keys.size() && r.tags[i + 1] < vals.size()) ft.props.push_back({keys[r.tags[i]], vals[r.tags[i + 1]]});
        int x = 0, y = 0;
        size_t i = 0;
        while (i < r.geom.size()) {
            uint64_t cmd = r.geom[i++];
            int id = int(cmd & 7);
            uint64_t cnt = cmd >> 3;
            if (id == 1 || id == 2) {
                for (uint64_t k = 0; k < cnt && i + 1 < r.geom.size(); k++) {
                    x += (int)zz(r.geom[i++]);
                    y += (int)zz(r.geom[i++]);
                    if (id == 1 || ft.parts.empty()) ft.parts.emplace_back();
                    ft.parts.back().push_back({x, y});
                }
            } else if (id == 7) {
                // ClosePath: rings stay open (first point not repeated)
            } else
                break;
        }
        L.features.push_back(std::move(ft));
    }
    return true;
}
}  // namespace

bool decode(const std::string &data, std::vector<Layer> &out, std::string *err) {
    Pb pb{(const uint8_t *)data.data(), data.size()};
    uint64_t f;
    int wt;
    while (pb.ok && pb.key(f, wt)) {
        if (f == 3 && wt == 2) {
            const uint8_t *s;
            size_t l;
            if (!pb.bytes(s, l)) break;
            Layer L;
            if (!layer(s, l, L)) {
                if (err) *err = "bad layer";
                return false;
            }
            out.push_back(std::move(L));
        } else
            pb.skip(wt);
    }
    if (!pb.ok && err) *err = "truncated vector tile";
    return pb.ok;
}

double ring_area(const std::vector<std::pair<int, int>> &r) {
    double a = 0;
    size_t n = r.size();
    for (size_t i = 0; i < n; i++) {
        auto p = r[i], q = r[(i + 1) % n];
        a += double(p.first) * q.second - double(q.first) * p.second;
    }
    return a / 2;
}

}  // namespace ec::mvt
