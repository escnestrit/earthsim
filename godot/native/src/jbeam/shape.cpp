#include "shape.hpp"
#include <cstring>
#include <functional>
#include "../../third_party/zstd/zstd.h"

namespace ec {

namespace {
struct MV {
    enum K { Nil, Bool, Int, Float, Str, Bin, Arr, Map } k = Nil;
    bool b = false;
    int64_t i = 0;
    double f = 0;
    std::string s;
    size_t off = 0, len = 0;
    std::vector<MV> a;
    std::vector<std::pair<MV, MV>> m;
    bool num(double &o) const {
        if (k == Int) return o = (double)i, true;
        if (k == Float) return o = f, true;
        return false;
    }
};

struct Mp {
    const uint8_t *b;
    size_t n, p = 0;
    std::string err;
    bool take(size_t k, const uint8_t *&out) {
        if (p + k > n) return err = "msgpack: truncated", false;
        out = b + p;
        p += k;
        return true;
    }
    bool be(size_t k, uint64_t &v) {
        const uint8_t *s;
        if (!take(k, s)) return false;
        v = 0;
        for (size_t i = 0; i < k; i++) v = v << 8 | s[i];
        return true;
    }
    bool val(MV &v) {
        const uint8_t *t;
        if (!take(1, t)) return false;
        uint8_t c = *t;
        uint64_t u;
        v = MV();
        if (c <= 0x7f) return v.k = MV::Int, v.i = c, true;
        if (c >= 0x80 && c <= 0x8f) return map(c & 15, v);
        if (c >= 0x90 && c <= 0x9f) return arr(c & 15, v);
        if (c >= 0xa0 && c <= 0xbf) return str(c & 31, v);
        if (c >= 0xe0) return v.k = MV::Int, v.i = (int8_t)c, true;
        switch (c) {
            case 0xc0: return true;
            case 0xc2: return v.k = MV::Bool, v.b = false, true;
            case 0xc3: return v.k = MV::Bool, v.b = true, true;
            case 0xc4:
            case 0xc5:
            case 0xc6: {
                static const int L[3] = {1, 2, 4};
                if (!be(L[c - 0xc4], u)) return false;
                v.k = MV::Bin, v.off = p, v.len = u;
                const uint8_t *s;
                return take(u, s);
            }
            case 0xca: {
                if (!be(4, u)) return false;
                uint32_t x = (uint32_t)u;
                float f;
                memcpy(&f, &x, 4);
                return v.k = MV::Float, v.f = f, true;
            }
            case 0xcb: {
                if (!be(8, u)) return false;
                double d;
                memcpy(&d, &u, 8);
                return v.k = MV::Float, v.f = d, true;
            }
            case 0xcc:
            case 0xcd:
            case 0xce:
            case 0xcf:
                if (!be(1u << (c - 0xcc), u)) return false;
                return v.k = MV::Int, v.i = (int64_t)u, true;
            case 0xd0:
                if (!be(1, u)) return false;
                return v.k = MV::Int, v.i = (int8_t)u, true;
            case 0xd1:
                if (!be(2, u)) return false;
                return v.k = MV::Int, v.i = (int16_t)u, true;
            case 0xd2:
                if (!be(4, u)) return false;
                return v.k = MV::Int, v.i = (int32_t)u, true;
            case 0xd3:
                if (!be(8, u)) return false;
                return v.k = MV::Int, v.i = (int64_t)u, true;
            case 0xd9:
            case 0xda:
            case 0xdb: {
                static const int L[3] = {1, 2, 4};
                if (!be(L[c - 0xd9], u)) return false;
                return str(u, v);
            }
            case 0xdc:
            case 0xdd:
                if (!be(c == 0xdc ? 2 : 4, u)) return false;
                return arr(u, v);
            case 0xde:
            case 0xdf:
                if (!be(c == 0xde ? 2 : 4, u)) return false;
                return map(u, v);
        }
        err = "msgpack: unsupported type";
        return false;
    }
    bool str(size_t k, MV &v) {
        const uint8_t *s;
        if (!take(k, s)) return false;
        v.k = MV::Str;
        v.s.assign((const char *)s, k);
        return true;
    }
    bool arr(size_t k, MV &v) {
        v.k = MV::Arr;
        v.a.resize(k);
        for (auto &x : v.a)
            if (!val(x)) return false;
        return true;
    }
    bool map(size_t k, MV &v) {
        v.k = MV::Map;
        v.m.resize(k);
        for (auto &x : v.m)
            if (!val(x.first) || !val(x.second)) return false;
        return true;
    }
    bool num(double &d) {
        MV v;
        if (!val(v)) return false;
        if (!v.num(d)) return err = "msgpack: expected number", false;
        return true;
    }
    bool integer(int64_t &i) {
        double d;
        if (!num(d)) return false;
        i = (int64_t)d;
        return true;
    }
    // pack_vector: count, element size, binary
    bool vector(size_t &cnt, size_t &es, const uint8_t *&data) {
        int64_t c, e;
        if (!integer(c) || !integer(e)) return false;
        MV v;
        if (!val(v)) return false;
        cnt = (size_t)c, es = (size_t)e;
        if (v.k == MV::Bin) {
            if (cnt * es != v.len && cnt != 0) return err = "pack_vector: size mismatch", false;
            data = b + v.off;
            return true;
        }
        if (v.k == MV::Nil && cnt == 0) return data = nullptr, true;
        return err = "pack_vector: expected bin", false;
    }
};

const MV *hget(const MV &h, const char *k) {
    if (h.k != MV::Map) return nullptr;
    for (auto &[a, b] : h.m)
        if (a.k == MV::Str && a.s == k) return &b;
    return nullptr;
}
uint32_t le32(const uint8_t *b) { return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24; }
int32_t i32(const uint8_t *b) { return (int32_t)le32(b); }
float f32(const uint8_t *b) {
    uint32_t x = le32(b);
    float f;
    memcpy(&f, &x, 4);
    return f;
}

using Q = std::array<double, 4>;
std::array<double, 3> qrot(Q q, std::array<double, 3> v) {
    double x = -q[0], y = -q[1], z = -q[2], w = q[3];
    double tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    return {v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)};
}
Q qmul(Q a, Q b) {
    return {b[3] * a[0] + b[0] * a[3] + b[1] * a[2] - b[2] * a[1], b[3] * a[1] - b[0] * a[2] + b[1] * a[3] + b[2] * a[0], b[3] * a[2] + b[0] * a[1] - b[1] * a[0] + b[2] * a[3],
            b[3] * a[3] - b[0] * a[0] - b[1] * a[1] - b[2] * a[2]};
}

bool zstd_decode(const uint8_t *b, size_t n, std::string &out, std::string &err) {
    ZSTD_DStream *ds = ZSTD_createDStream();
    ZSTD_initDStream(ds);
    ZSTD_inBuffer in{b, n, 0};
    std::string buf(ZSTD_DStreamOutSize(), '\0');
    for (;;) {
        ZSTD_outBuffer o{&buf[0], buf.size(), 0};
        size_t r = ZSTD_decompressStream(ds, &o, &in);
        if (ZSTD_isError(r)) {
            err = std::string("zstd: ") + ZSTD_getErrorName(r);
            ZSTD_freeDStream(ds);
            return false;
        }
        out.append(buf.data(), o.pos);
        if (r == 0 && in.pos >= in.size) break;
        if (in.pos >= in.size && o.pos < o.size) break;
    }
    ZSTD_freeDStream(ds);
    return true;
}
}  // namespace

bool shape_object_names(const std::string &pre, std::vector<std::string> &out, size_t *hl) {
    if (pre.size() < 8 || (le32((const uint8_t *)pre.data()) & 0xff) != 31) return false;
    size_t hs = le32((const uint8_t *)pre.data() + 4);
    if (hl) *hl = hs;
    if (pre.size() < 8 + hs) return false;
    Mp m{(const uint8_t *)pre.data() + 8, hs};
    MV h;
    if (!m.val(h)) return false;
    const MV *on = hget(h, "objectNames");
    if (!on || on->k != MV::Arr) return false;
    for (auto &v : on->a)
        if (v.k == MV::Str) out.push_back(v.s);
    return true;
}

bool parse_shape(const std::string &file, Shape &shape, std::string &err) {
    const uint8_t *fb = (const uint8_t *)file.data();
    if (file.size() < 8) return err = "shape: file too short", false;
    if ((le32(fb) & 0xff) != 31) return err = "shape: version not supported", false;
    size_t hs = le32(fb + 4);
    if (8 + hs > file.size()) return err = "shape: bad header size", false;
    Mp hm{fb + 8, hs};
    MV header;
    if (!hm.val(header)) return err = hm.err, false;
    const MV *comp = hget(header, "compression");
    std::string body;
    if (comp && comp->k == MV::Bool && comp->b) {
        if (!zstd_decode(fb + 8 + hs, file.size() - 8 - hs, body, err)) return false;
    } else
        body = file.substr(8 + hs);
    std::vector<std::string> object_names;
    if (const MV *on = hget(header, "objectNames"); on && on->k == MV::Arr)
        for (auto &v : on->a) object_names.push_back(v.k == MV::Str ? v.s : "");
    Mp m{(const uint8_t *)body.data(), body.size()};
    double d;
    int64_t iv;
    if (!m.num(d) || !m.integer(iv) || !m.num(d) || !m.num(d)) return err = m.err, false;
    for (int n : {3, 6}) {
        MV v;
        if (!m.val(v)) return err = m.err, false;
        if (v.k == MV::Float || v.k == MV::Int)
            for (int k = 1; k < n; k++)
                if (!m.num(d)) return err = m.err, false;
    }
    const char *order[] = {"nodes", "objects", "subShapeFirstNode", "subShapeFirstObject", "subShapeNumNodes", "subShapeNumObjects", "defaultRotations", "defaultTranslations", "nodeRotations", "nodeTranslations",
                           "nodeUniformScales", "nodeAlignedScales", "nodeArbitraryScaleFactors", "nodeArbitraryScaleRots", "groundTranslations", "groundRotations", "objectStates", "triggers", "details"};
    struct Vec_ {
        size_t n, es;
        const uint8_t *d;
    };
    std::map<std::string, Vec_> vecs;
    for (const char *o : order) {
        Vec_ v;
        if (!m.vector(v.n, v.es, v.d)) return err = m.err, false;
        vecs[o] = v;
    }
    int64_t nn;
    if (!m.integer(nn)) return err = m.err, false;
    std::vector<std::string> names;
    for (int64_t i = 0; i < nn; i++) {
        MV v;
        if (!m.val(v)) return err = m.err, false;
        names.push_back(v.k == MV::Str ? v.s : "");
    }
    auto &NV = vecs["nodes"];
    std::vector<int> parent(NV.n);
    for (size_t i = 0; i < NV.n; i++) parent[i] = NV.es >= 8 ? i32(NV.d + i * NV.es + 4) : -1;
    auto &RV = vecs["defaultRotations"], &TV = vecs["defaultTranslations"];
    std::vector<int> state(NV.n, 0);
    std::vector<Q> wq(NV.n);
    std::vector<std::array<double, 3>> wt(NV.n);
    std::function<void(size_t)> resolve = [&](size_t i) {
        if (state[i] == 2) return;
        if (state[i] == 1) {  // cycle guard
            wq[i] = {0, 0, 0, 1}, wt[i] = {0, 0, 0};
            state[i] = 2;
            return;
        }
        state[i] = 1;
        Q q{0, 0, 0, 1};
        if (RV.es >= 8 && RV.n > i) {
            const uint8_t *r = RV.d + i * RV.es;
            for (int k = 0; k < 4; k++) q[k] = (int16_t)(r[k * 2] | r[k * 2 + 1] << 8) / 32767.0;
        }
        std::array<double, 3> t{0, 0, 0};
        if (TV.es >= 12 && TV.n > i)
            for (int k = 0; k < 3; k++) t[k] = f32(TV.d + i * TV.es + k * 4);
        int p = parent[i];
        if (p >= 0 && (size_t)p < NV.n && (size_t)p != i) {
            resolve(p);
            auto r = qrot(wq[p], t);
            wq[i] = qmul(wq[p], q);
            wt[i] = {wt[p][0] + r[0], wt[p][1] + r[1], wt[p][2] + r[2]};
        } else
            wq[i] = q, wt[i] = t;
        state[i] = 2;
    };
    for (size_t i = 0; i < NV.n; i++) resolve(i);
    auto &OV = vecs["objects"];
    int64_t mesh_count;
    if (!m.integer(mesh_count)) return err = m.err, false;
    struct Raw {
        std::vector<float> verts, tverts, norms;
        std::vector<int32_t> prims;
        std::vector<uint32_t> indices;
        int64_t parent;
        bool ok = false;
    };
    std::vector<Raw> meshes(mesh_count);
    for (int64_t mi = 0; mi < mesh_count; mi++) {
        int64_t ty;
        if (!m.integer(ty)) return err = m.err, false;
        if (ty == 4) continue;
        int64_t fr, mf, par;
        if (!m.integer(fr) || !m.integer(mf) || !m.integer(par)) return err = m.err, false;
        for (int n : {6, 3}) {
            MV v;
            if (!m.val(v)) return err = m.err, false;
            if (v.k == MV::Float || v.k == MV::Int)
                for (int k = 1; k < n; k++)
                    if (!m.num(d)) return err = m.err, false;
        }
        if (!m.num(d)) return err = m.err, false;
        Vec_ v[9];
        for (auto &x : v)
            if (!m.vector(x.n, x.es, x.d)) return err = m.err, false;
        int64_t vpf, flags;
        if (!m.integer(vpf) || !m.integer(flags)) return err = m.err, false;
        if (ty == 1) {
            Vec_ skip;
            int extra = par < 0 ? 2 + 1 + 4 : 1;
            for (int k = 0; k < extra; k++)
                if (!m.vector(skip.n, skip.es, skip.d)) return err = m.err, false;
        }
        Raw &r = meshes[mi];
        r.parent = par;
        auto floats = [](const Vec_ &x, std::vector<float> &o) {
            size_t bytes = x.n * x.es;
            for (size_t k = 0; k + 4 <= bytes; k += 4) o.push_back(f32(x.d + k));
        };
        floats(v[0], r.verts);
        floats(v[1], r.tverts);
        floats(v[4], r.norms);
        size_t pb = v[6].n * v[6].es;
        if (v[6].es == 8)
            for (size_t k = 0; k + 8 <= pb; k += 8) r.prims.insert(r.prims.end(), {(int32_t)(v[6].d[k] | v[6].d[k + 1] << 8), (int32_t)(v[6].d[k + 2] | v[6].d[k + 3] << 8), i32(v[6].d + k + 4)});
        else
            for (size_t k = 0; k + 4 <= pb; k += 4) r.prims.push_back(i32(v[6].d + k));
        size_t ib = v[7].n * v[7].es;
        if (v[7].es == 2)
            for (size_t k = 0; k + 2 <= ib; k += 2) r.indices.push_back(v[7].d[k] | v[7].d[k + 1] << 8);
        else
            for (size_t k = 0; k + 4 <= ib; k += 4) r.indices.push_back(le32(v[7].d + k));
        r.ok = true;
    }
    int64_t nseq;
    if (m.integer(nseq) && nseq == 0) {
        int64_t nmat;
        if (m.integer(nmat))
            for (int64_t k = 0; k < nmat; k++) {
                MV v;
                if (!m.val(v) || v.k != MV::Str) break;
                shape.materials.push_back(v.s);
                for (int q = 0; q < 4; q++) m.integer(iv);
                m.num(d);
                m.num(d);
            }
    }
    for (size_t oi = 0; oi < OV.n; oi++) {
        const uint8_t *o = OV.d + oi * OV.es;
        if (OV.es < 16) break;
        int32_t nm = i32(o), num = i32(o + 4), start = i32(o + 8), node = i32(o + 12);
        std::string name = nm >= 0 && (size_t)nm < names.size() ? names[nm] : oi < object_names.size() ? object_names[oi] : "";
        if (num <= 0 || start < 0 || (size_t)start >= meshes.size() || !meshes[start].ok) continue;
        const Raw *src = &meshes[start];
        if (src->parent >= 0 && (size_t)src->parent < meshes.size() && meshes[src->parent].ok) src = &meshes[src->parent];
        Q q{0, 0, 0, 1};
        std::array<double, 3> t{0, 0, 0};
        if (node >= 0 && (size_t)node < NV.n) q = wq[node], t = wt[node];
        size_t nv = src->verts.size() / 3;
        SubMesh base;
        for (size_t i = 0; i < nv; i++) {
            auto p = qrot(q, {src->verts[i * 3], src->verts[i * 3 + 1], src->verts[i * 3 + 2]});
            base.pos.push_back({(float)(p[0] + t[0]), (float)(p[1] + t[1]), (float)(p[2] + t[2])});
            if (src->norms.size() >= (i + 1) * 3) {
                auto n = qrot(q, {src->norms[i * 3], src->norms[i * 3 + 1], src->norms[i * 3 + 2]});
                base.nrm.push_back({(float)n[0], (float)n[1], (float)n[2]});
            } else
                base.nrm.push_back({0, 0, 1});
            base.uv.push_back(src->tverts.size() >= (i + 1) * 2 ? std::array<float, 2>{src->tverts[i * 2], src->tverts[i * 2 + 1]} : std::array<float, 2>{0, 0});
        }
        std::map<size_t, std::vector<uint32_t>> by_mat;
        for (size_t k = 0; k + 2 < src->prims.size(); k += 3) {
            size_t st = std::max(0, src->prims[k]), cnt = std::max(0, src->prims[k + 1]);
            uint32_t mi = (uint32_t)src->prims[k + 2];
            size_t mat = mi & 0x0fffffff;
            uint32_t kind = mi & 0xc0000000u;
            size_t a0 = std::min(st, src->indices.size()), a1 = std::min(st + cnt, src->indices.size());
            const uint32_t *idx = src->indices.data() + a0;
            size_t len = a1 - a0;
            auto &e = by_mat[mat];
            if (kind == 0x40000000u) {
                for (size_t q2 = 2; q2 < len; q2++) {
                    uint32_t a = q2 % 2 == 0 ? idx[q2 - 2] : idx[q2 - 1], b = q2 % 2 == 0 ? idx[q2 - 1] : idx[q2 - 2], c = idx[q2];
                    if (a != b && b != c && a != c) e.insert(e.end(), {a, b, c});
                }
            } else if (kind == 0x80000000u) {
                for (size_t q2 = 2; q2 < len; q2++) e.insert(e.end(), {idx[0], idx[q2 - 1], idx[q2]});
            } else
                e.insert(e.end(), idx, idx + len / 3 * 3);
        }
        std::vector<SubMesh> subs;
        for (auto &[mat, ind] : by_mat) {
            bool bad = false;
            for (auto i : ind) bad |= i >= nv;
            if (bad || ind.empty()) continue;
            for (size_t k = 0; k + 2 < ind.size(); k += 3) std::swap(ind[k + 1], ind[k + 2]);
            SubMesh s = base;
            s.material = mat < shape.materials.size() ? shape.materials[mat] : "material" + std::to_string(mat);
            s.idx = std::move(ind);
            subs.push_back(std::move(s));
        }
        if (!subs.empty()) shape.objects[name] = std::move(subs);
    }
    return true;
}

}  // namespace ec
