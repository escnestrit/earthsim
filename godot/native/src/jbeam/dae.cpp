#include "dae.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string_view>
#include <algorithm>
#include <cstring>
#include <map>
#include <tuple>

namespace ec {

namespace {
const std::array<double, 16> IDENT = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
std::array<double, 16> mul(const std::array<double, 16> &a, const std::array<double, 16> &b) {
    std::array<double, 16> r{};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++) r[i * 4 + j] += a[i * 4 + k] * b[k * 4 + j];
    return r;
}
// tag text starting at i, and the index after it
std::pair<std::string_view, size_t> tag_at(std::string_view t, size_t i) {
    size_t e = t.find('>', i);
    e = e == std::string_view::npos ? t.size() : e + 1;
    return {t.substr(i, e - i), e};
}
bool attr(std::string_view tag, const char *name, std::string &out) {
    std::string pat = std::string(" ") + name + "=\"";
    size_t i = tag.find(pat);
    if (i == std::string_view::npos) return false;
    i += pat.size();
    size_t j = tag.find('"', i);
    if (j == std::string_view::npos) return false;
    out.assign(tag.substr(i, j - i));
    return true;
}
template <class T>
void numbers(std::string_view s, std::vector<T> &out) {
    std::string tmp(s);
    const char *p = tmp.c_str();
    char *end;
    for (;;) {
        double v = strtod(p, &end);
        if (end == p) break;
        out.push_back((T)v);
        p = end;
    }
}
size_t find_from(std::string_view s, const char *what, size_t from, size_t lim = std::string_view::npos) {
    size_t i = s.find(what, from);
    if (i == std::string_view::npos || (lim != std::string_view::npos && i >= lim)) return std::string_view::npos;
    return i;
}
}  // namespace

bool Dae::parse(std::string text, std::string &err) {
    if (text.find("<COLLADA") == std::string::npos) return err = "not a Collada file", false;
    text_ = std::move(text);
    std::string_view t(text_);
    size_t pos = 0;
    while ((pos = t.find("<geometry ", pos)) != std::string_view::npos) {
        auto [tag, after] = tag_at(t, pos);
        size_t e = t.find("</geometry>", after);
        if (e == std::string_view::npos) e = t.size();
        std::string id;
        if (attr(tag, "id", id)) geoms_[id] = {pos, e};
        pos = e;
    }
    size_t vs = t.find("<library_visual_scenes");
    if (vs != std::string_view::npos) {
        size_t ve = t.find("</library_visual_scenes>", vs);
        if (ve == std::string_view::npos) ve = t.size();
        std::string_view seg = t.substr(vs, ve - vs);
        struct S {
            std::string name;
            std::array<double, 16> m;
            bool seen;
        };
        std::vector<S> stack;
        size_t p = 0;
        while ((p = seg.find('<', p)) != std::string_view::npos) {
            auto [tag, after] = tag_at(seg, p);
            p = after;
            if (tag.rfind("<node", 0) == 0) {
                auto parent = stack.empty() ? IDENT : stack.back().m;
                std::string name;
                if (!attr(tag, "name", name)) attr(tag, "id", name);
                if (!(tag.size() >= 2 && tag[tag.size() - 2] == '/')) stack.push_back({name, parent, false});
            } else if (tag.rfind("</node", 0) == 0) {
                if (!stack.empty()) stack.pop_back();
            } else if (tag.rfind("<matrix", 0) == 0) {
                size_t e = seg.find("</matrix>", after);
                if (e == std::string_view::npos) e = after;
                std::vector<double> v;
                numbers(seg.substr(after, e - after), v);
                if (!stack.empty() && v.size() == 16 && !stack.back().seen) {
                    std::array<double, 16> m;
                    for (int i = 0; i < 16; i++) m[i] = v[i];
                    stack.back().m = mul(stack.back().m, m);
                    stack.back().seen = true;
                }
                p = e;
            } else if (tag.rfind("<instance_geometry", 0) == 0) {
                std::string url;
                attr(tag, "url", url);
                if (!url.empty() && url[0] == '#') url.erase(0, 1);
                size_t end = seg.find("</instance_geometry>", after);
                if (end == std::string_view::npos) end = after;
                DaeNode dn;
                size_t q = after;
                while ((q = find_from(seg, "<instance_material", q, end)) != std::string_view::npos) {
                    auto [mt, a2] = tag_at(seg, q);
                    std::string sym, tg;
                    if (attr(mt, "symbol", sym) && attr(mt, "target", tg)) {
                        if (!tg.empty() && tg[0] == '#') tg.erase(0, 1);
                        dn.materials.push_back({sym, tg});
                    }
                    q = a2;
                }
                if (!stack.empty()) {
                    dn.name = stack.back().name;
                    dn.geometry = url;
                    dn.matrix = stack.back().m;
                    nodes.push_back(std::move(dn));
                }
                p = end;
            }
        }
    }
    return true;
}

const DaeNode *Dae::node(const std::string &n) const {
    for (auto &d : nodes)
        if (d.name == n) return &d;
    return nullptr;
}

bool Dae::mesh(const std::string &node_name, std::vector<SubMesh> &out, std::string &err) const {
    const DaeNode *node = this->node(node_name);
    if (!node) return err = "mesh '" + node_name + "' not in the .dae", false;
    auto gi = geoms_.find(node->geometry);
    if (gi == geoms_.end()) return err = "geometry '" + node->geometry + "' missing", false;
    std::string_view g = std::string_view(text_).substr(gi->second.first, gi->second.second - gi->second.first);
    std::map<std::string, std::pair<std::vector<float>, int>> sources;
    size_t p = 0;
    while ((p = g.find("<source ", p)) != std::string_view::npos) {
        auto [tag, after] = tag_at(g, p);
        size_t end = g.find("</source>", after);
        if (end == std::string_view::npos) end = g.size();
        std::string_view body = g.substr(after, end - after);
        std::string id;
        size_t fa = body.find("<float_array");
        if (attr(tag, "id", id) && fa != std::string_view::npos) {
            auto [ft, fa_after] = tag_at(body, fa);
            size_t fe = body.find("</float_array>", fa_after);
            if (fe == std::string_view::npos) fe = body.size();
            std::vector<float> vals;
            numbers(body.substr(fa_after, fe - fa_after), vals);
            int stride = 3;
            size_t ac = body.find("<accessor");
            if (ac != std::string_view::npos) {
                std::string st;
                if (attr(tag_at(body, ac).first, "stride", st)) stride = atoi(st.c_str());
            }
            sources[id] = {std::move(vals), std::max(1, stride)};
        }
        p = end;
    }
    std::map<std::string, std::string> alias;
    size_t vi = g.find("<vertices ");
    if (vi != std::string_view::npos) {
        auto [tag, after] = tag_at(g, vi);
        size_t ve = g.find("</vertices>", after);
        if (ve == std::string_view::npos) ve = g.size();
        size_t ii = find_from(g, "semantic=\"POSITION\"", after, ve);
        if (ii != std::string_view::npos) {
            size_t ts = g.rfind('<', ii);
            std::string id, src;
            if (attr(tag, "id", id) && attr(tag_at(g, ts).first, "source", src)) alias[id] = src[0] == '#' ? src.substr(1) : src;
        }
    }
    const auto &m = node->matrix;
    auto xf = [&](const float *v) {
        double x = v[0], y = v[1], z = v[2];
        return std::array<float, 3>{(float)(m[0] * x + m[1] * y + m[2] * z + m[3]), (float)(m[4] * x + m[5] * y + m[6] * z + m[7]), (float)(m[8] * x + m[9] * y + m[10] * z + m[11])};
    };
    auto xn = [&](const float *v) {
        double x = v[0], y = v[1], z = v[2];
        double r[3] = {m[0] * x + m[1] * y + m[2] * z, m[4] * x + m[5] * y + m[6] * z, m[8] * x + m[9] * y + m[10] * z};
        double l = std::max(1e-12, std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]));
        return std::array<float, 3>{(float)(r[0] / l), (float)(r[1] / l), (float)(r[2] / l)};
    };
    p = 0;
    for (;;) {
        size_t tri = g.find("<triangles ", p), poly = g.find("<polylist ", p);
        if (tri == std::string_view::npos && poly == std::string_view::npos) break;
        bool is_poly = tri == std::string_view::npos || (poly != std::string_view::npos && poly < tri);
        size_t st = is_poly ? poly : tri;
        auto [tag, after] = tag_at(g, st);
        size_t end = g.find(is_poly ? "</polylist>" : "</triangles>", after);
        if (end == std::string_view::npos) end = g.size();
        p = end;
        std::string_view body = g.substr(after, end - after);
        std::string material;
        attr(tag, "material", material);
        if (material.size() > 9 && material.compare(material.size() - 9, 9, "-material") == 0) material.resize(material.size() - 9);
        int ov = -1, on = -1, ot = -1, stride = 0;
        std::string sv, sn, stx;
        size_t q = 0;
        while ((q = body.find("<input ", q)) != std::string_view::npos) {
            auto [it, a2] = tag_at(body, q);
            q = a2;
            std::string offs, src, sem, set;
            attr(it, "offset", offs);
            int off = atoi(offs.c_str());
            stride = std::max(stride, off + 1);
            attr(it, "source", src);
            if (!src.empty() && src[0] == '#') src.erase(0, 1);
            attr(it, "semantic", sem);
            if (sem == "VERTEX") {
                ov = off;
                auto a = alias.find(src);
                sv = a == alias.end() ? src : a->second;
            } else if (sem == "NORMAL")
                on = off, sn = src;
            else if (sem == "TEXCOORD" && (ot < 0 || (attr(it, "set", set) && set == "0")))
                ot = off, stx = src;
        }
        if (ov < 0) continue;
        auto ps = sources.find(sv);
        if (ps == sources.end()) continue;
        auto pn = on >= 0 ? sources.find(sn) : sources.end();
        auto pt = ot >= 0 ? sources.find(stx) : sources.end();
        size_t pk = body.find("<p>");
        if (pk == std::string_view::npos) continue;
        size_t pe = body.find("</p>", pk);
        std::vector<uint32_t> pi;
        {
            std::vector<double> tmp;
            numbers(body.substr(pk + 3, (pe == std::string_view::npos ? body.size() : pe) - pk - 3), tmp);
            pi.assign(tmp.begin(), tmp.end());
        }
        std::vector<int> vcount;
        if (is_poly) {
            size_t vk = body.find("<vcount>");
            if (vk != std::string_view::npos) {
                size_t ve = body.find("</vcount>", vk);
                std::vector<double> tmp;
                numbers(body.substr(vk + 8, (ve == std::string_view::npos ? body.size() : ve) - vk - 8), tmp);
                vcount.assign(tmp.begin(), tmp.end());
            }
        }
        SubMesh *sm = nullptr;
        for (auto &s : out)
            if (s.material == material) sm = &s;
        if (!sm) {
            out.push_back({});
            sm = &out.back();
            sm->material = material;
        }
        std::map<std::tuple<uint32_t, uint32_t, uint32_t>, uint32_t> dedup;
        stride = std::max(stride, 1);
        auto corner = [&](size_t c, uint32_t &k) {
            size_t base = c * stride;
            if (base + ov >= pi.size()) return false;
            uint32_t v = pi[base + ov];
            uint32_t ni = on >= 0 && base + on < pi.size() ? pi[base + on] : UINT32_MAX;
            uint32_t ti = ot >= 0 && base + ot < pi.size() ? pi[base + ot] : UINT32_MAX;
            auto key = std::make_tuple(v, ni, ti);
            auto it = dedup.find(key);
            if (it != dedup.end()) return k = it->second, true;
            auto &P = ps->second;
            if ((size_t)v * P.second + 3 > P.first.size()) return false;
            sm->pos.push_back(xf(&P.first[v * P.second]));
            if (pn != sources.end() && ni != UINT32_MAX && (size_t)ni * pn->second.second + 3 <= pn->second.first.size()) sm->nrm.push_back(xn(&pn->second.first[ni * pn->second.second]));
            else sm->nrm.push_back({0, 0, 1});
            if (pt != sources.end() && ti != UINT32_MAX && (size_t)ti * pt->second.second + 2 <= pt->second.first.size()) {
                const float *u = &pt->second.first[ti * pt->second.second];
                sm->uv.push_back({u[0], 1 - u[1]});
            } else
                sm->uv.push_back({0, 0});
            k = (uint32_t)sm->pos.size() - 1;
            dedup[key] = k;
            return true;
        };
        if (!vcount.empty()) {
            size_t c = 0;
            for (int n : vcount) {
                std::vector<uint32_t> ids;
                for (int k = 0; k < n; k++) {
                    uint32_t id;
                    if (corner(c + k, id)) ids.push_back(id);
                }
                for (size_t k = 1; k + 1 < ids.size(); k++) sm->idx.insert(sm->idx.end(), {ids[0], ids[k], ids[k + 1]});
                c += n;
            }
        } else {
            size_t nc = pi.size() / stride;
            for (size_t t = 0; t < nc / 3; t++) {
                uint32_t a, b, c;
                if (corner(t * 3, a) && corner(t * 3 + 1, b) && corner(t * 3 + 2, c)) sm->idx.insert(sm->idx.end(), {a, b, c});
            }
        }
    }
    std::vector<SubMesh> keep;
    for (auto &s : out)
        if (!s.idx.empty()) keep.push_back(std::move(s));
    out.swap(keep);
    return true;
}

}  // namespace ec
