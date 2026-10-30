#include "jvisual.hpp"
#include <algorithm>
#include <cstring>
#include <cctype>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace ec {

namespace {
std::string lc_(std::string s) {
    for (auto &c : s) c = (char)tolower((unsigned char)c);
    return s;
}
bool ends(const std::string &s, const char *e) {
    size_t n = strlen(e);
    return s.size() >= n && s.compare(s.size() - n, n, e) == 0;
}
bool is_mesh(const std::string &l) { return ends(l, ".dae") || ends(l, ".cdae") || ends(l, ".dts"); }

struct MeshFile {
    bool is_shape = false;
    Dae dae;
    Shape shape;
    bool has(const std::string &n) const { return is_shape ? shape.has(n) : dae.node(n) != nullptr; }
    bool mesh(const std::string &n, std::vector<SubMesh> &out) const {
        std::string err;
        if (is_shape) {
            auto it = shape.objects.find(n);
            if (it == shape.objects.end()) return false;
            out = it->second;
            return true;
        }
        return dae.mesh(n, out, err);
    }
};

bool parse_mesh_file(Zip &z, const ZipEntry &e, MeshFile &f, std::string &err) {
    std::string bytes;
    if (!z.read(e, bytes, err)) return false;
    std::string l = lc_(e.name);
    if (ends(l, ".cdae") || ends(l, ".dts")) {
        f.is_shape = true;
        return parse_shape(bytes, f.shape, err);
    }
    return f.dae.parse(std::move(bytes), err);
}

struct FileIndex {
    std::unordered_map<std::string, std::string> exact;
    std::unordered_map<std::string, std::vector<std::string>> stem;
    explicit FileIndex(const std::vector<ZipEntry> &es) {
        for (auto &e : es) {
            std::string l = lc_(e.name);
            exact[l] = e.name;
            size_t d = l.rfind('.');
            std::string s = d != std::string::npos && l.find('/', d) == std::string::npos ? l.substr(0, d) : l;
            stem[s].push_back(e.name);
        }
    }
    std::string resolve(std::string p) const {
        while (!p.empty() && (p[0] == '/' || p[0] == ' ')) p.erase(0, 1);
        std::replace(p.begin(), p.end(), '\\', '/');
        p = lc_(p);
        if (p.empty()) return "";
        auto it = exact.find(p);
        if (it != exact.end()) return it->second;
        std::vector<std::string> stems{p};
        size_t d = p.rfind('.');
        if (d != std::string::npos && p.find('/', d) == std::string::npos) stems.push_back(p.substr(0, d));
        for (auto &s : stems) {
            auto c = stem.find(s);
            if (c == stem.end()) continue;
            auto rank = [](const std::string &n) {
                std::string l = lc_(n);
                return ends(l, ".dds") ? 0 : ends(l, ".png") ? 1 : (ends(l, ".jpg") || ends(l, ".jpeg")) ? 2 : 9;
            };
            std::string best;
            int br = 9;
            for (auto &n : c->second)
                if (rank(n) < br) br = rank(n), best = n;
            if (!best.empty()) return best;
        }
        return "";
    }
};

void parse_materials(const std::string &text, const FileIndex &files, std::map<std::string, MatDef> &out, const std::string &prefix) {
    Json j;
    std::string err;
    if (!jbeam_parse(text, j, err) || j.kind != Json::Obj) return;
    for (auto &[key, m] : j.o) {
        if (m.kind != Json::Obj) continue;
        MatDef d;
        const Json *mt = m.get("mapTo");
        d.name = mt && mt->kind == Json::Str ? mt->s : key;
        const Json *stages = m.get("Stages");
        Json empty;
        empty.kind = Json::Obj;
        const Json &st = stages && stages->kind == Json::Arr && !stages->a.empty() ? stages->a[0] : empty;
        auto s = [&](const char *k) -> std::string {
            const Json *v = st.get(k);
            if (!v || v->kind != Json::Str) return "";
            std::string r = files.resolve(v->s);
            return r.empty() ? "" : prefix + r;
        };
        auto f4 = [&](const char *k, float *o) {
            const Json *v = st.get(k);
            if (!v || v->kind != Json::Arr || v->a.size() < 3) return false;
            for (int i = 0; i < 4; i++) o[i] = i < (int)v->a.size() ? (float)v->a[i].num(1) : 1.0f;
            return true;
        };
        if (!f4("baseColorFactor", d.base_color)) f4("diffuseColor", d.base_color);
        d.base_map = s("baseColorMap");
        if (d.base_map.empty()) d.base_map = s("colorMap");
        if (d.base_map.empty()) d.base_map = s("diffuseMap");
        d.normal_map = s("normalMap");
        const Json *r = st.get("roughnessFactor");
        d.roughness = r && r->kind == Json::Num ? (float)r->n : st.get("specularPower") ? 0.4f : 0.6f;
        const Json *mm = st.get("metallicFactor");
        d.metallic = mm && mm->kind == Json::Num ? (float)mm->n : 0;
        const Json *tr = m.get("translucent"), *op = st.get("opacityFactor");
        d.translucent = (tr && tr->kind == Json::Bool && tr->b) || (op && op->kind == Json::Num && op->n > 0 && op->n < 1);
        out[d.name] = d;
    }
}

V3 v3of(const Row &r, const char *k, double d) {
    const Json *o = r.get(k);
    auto c = [&](const char *a) {
        const Json *v = o ? o->get(a) : nullptr;
        return v && v->kind == Json::Num ? v->n : d;
    };
    return V3(c("x"), c("y"), c("z"));
}

using M3 = std::array<std::array<double, 3>, 3>;
M3 mmul(const M3 &a, const M3 &b) {
    M3 o{};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++) o[i][j] += a[i][k] * b[k][j];
    return o;
}
M3 rot_axis(V3 ax, double deg) {
    double l = ax.len();
    if (l < 1e-12 || deg == 0) return M3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    double x = ax.x / l, y = ax.y / l, z = ax.z / l, s = std::sin(deg * PI / 180), c = std::cos(deg * PI / 180), t = 1 - c;
    return M3{{{t * x * x + c, t * x * y - s * z, t * x * z + s * y}, {t * x * y + s * z, t * y * y + c, t * y * z - s * x}, {t * x * z - s * y, t * y * z + s * x, t * z * z + c}}};
}
}  // namespace

bool is_invisible_material(const std::string &n) {
    std::string l = lc_(n);
    return l.find("invis") != std::string::npos || l == "null" || l == "nodraw";
}

void Prop::pose(const std::vector<V3> &nodes, double signal, V3 &o, V3 cols[3]) const {
    o = nodes[refs[0]];
    V3 x = (nodes[refs[1]] - o).norm();
    V3 y0 = nodes[refs[2]] - o;
    V3 y = (y0 - x * y0.dot(x)).norm();
    V3 z = x.cross(y);
    M3 f{{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
    M3 base = mmul(rot_axis({0, 0, 1}, base_rot[2]), mmul(rot_axis({0, 1, 0}, base_rot[1]), rot_axis({1, 0, 0}, base_rot[0])));
    M3 r = mmul(f, mmul(base, rot_axis({axis[0], axis[1], axis[2]}, angle(signal))));
    V3 off(f[0][0] * base_trans[0] + f[0][1] * base_trans[1] + f[0][2] * base_trans[2], f[1][0] * base_trans[0] + f[1][1] * base_trans[1] + f[1][2] * base_trans[2],
           f[2][0] * base_trans[0] + f[2][1] * base_trans[1] + f[2][2] * base_trans[2]);
    o = o + off;
    for (int c = 0; c < 3; c++) cols[c] = V3(r[0][c], r[1][c], r[2][c]);
}

bool load_visual(Zip &z, Zip *common, const JModel &m, const JVehicle &v, CarVisual &vis, std::string &err) {
    std::vector<MeshFile> files;
    std::set<std::string> lower_names;
    for (auto &e : z.entries) lower_names.insert(lc_(e.name));
    for (auto &e : z.entries) {
        std::string l = lc_(e.name);
        if (!is_mesh(l)) continue;
        if (ends(l, ".dae") && lower_names.count(l.substr(0, l.size() - 4) + ".cdae")) continue;
        MeshFile f;
        std::string e2;
        if (parse_mesh_file(z, e, f, e2)) files.push_back(std::move(f));
        else vis.notes.push_back(e.name + ": " + e2);
    }
    auto have = [&](const std::string &n) {
        for (auto &f : files)
            if (f.has(n)) return true;
        return false;
    };
    auto flex_rows = m.table("flexbodies");
    if (common) {
        std::vector<std::string> needed;
        for (auto &[pi, r] : flex_rows) {
            std::string n = r.str("mesh");
            if (!n.empty() && !have(n)) needed.push_back(n);
        }
        for (auto &[pi, r] : m.table("props")) {
            std::string n = r.str("mesh");
            if (!n.empty() && r.str("func") == "steering" && !have(n)) needed.push_back(n);
        }
        if (!needed.empty()) {
            std::vector<const ZipEntry *> cents;
            for (auto &e : common->entries)
                if (is_mesh(lc_(e.name))) cents.push_back(&e);
            // index of object names: read only the headers of the compiled shapes
            std::unordered_map<std::string, const ZipEntry *> index;
            for (auto *e : cents) {
                if (!ends(lc_(e->name), ".cdae")) continue;
                std::string head, data, e2;
                size_t hl = 0;
                std::vector<std::string> names;
                if (!common->read_prefix(*e, 8, head, e2) || head.size() < 8) continue;
                shape_object_names(head, names, &hl);
                if (hl == 0 || hl > 64 * 1024 * 1024 || !common->read_prefix(*e, 8 + hl, data, e2)) continue;
                names.clear();
                if (shape_object_names(data, names))
                    for (auto &n : names) index.emplace(n, e);
            }
            std::set<std::string> tried;
            int found = 0;
            for (auto &name : needed) {
                if (have(name)) {
                    found++;
                    continue;
                }
                std::vector<const ZipEntry *> cands;
                auto it = index.find(name);
                if (it != index.end()) cands.push_back(it->second);
                if (cands.empty()) {
                    std::string n = lc_(name);
                    std::vector<std::string> toks;
                    size_t s = 0, p;
                    while ((p = n.find('_', s)) != std::string::npos) toks.push_back(n.substr(s, p - s)), s = p + 1;
                    toks.push_back(n.substr(s));
                    std::string key2 = toks.size() >= 2 ? toks[0] + "_" + toks[1] : toks[0], key1 = toks[0];
                    for (auto *e : cents) {
                        std::string stem = lc_(e->name);
                        stem = stem.substr(stem.rfind('/') + 1);
                        stem = stem.substr(0, stem.find('.'));
                        if (!stem.empty() && (stem.rfind(key2, 0) == 0 || key2.rfind(stem, 0) == 0 || (key1.size() >= 4 && stem.rfind(key1, 0) == 0))) cands.push_back(e);
                    }
                    std::sort(cands.begin(), cands.end(), [](auto *a, auto *b) { return a->size < b->size; });
                    if (cands.size() > 6) cands.resize(6);
                }
                for (auto *e : cands) {
                    if (!tried.insert(e->name).second) continue;
                    MeshFile f;
                    std::string e2;
                    if (parse_mesh_file(*common, *e, f, e2)) files.push_back(std::move(f));
                    if (have(name)) break;
                }
                if (have(name)) found++;
            }
            vis.notes.push_back(std::to_string(found) + " of " + std::to_string(needed.size()) + " missing meshes found in BeamNG's common.zip");
        }
    }
    if (files.empty()) return err = "no meshes (.dae/.cdae/.dts) in the mod", false;
    // ---- flexbodies
    std::vector<V3> rest;
    for (auto &n : v.body.nodes) rest.push_back(n.rest);
    V3 shift = m.nodes.empty() ? V3() : rest[0] - V3(to_ours(m.nodes[0].pos)[0], to_ours(m.nodes[0].pos)[1], to_ours(m.nodes[0].pos)[2]);
    auto ours_f = [](double x, double y, double z) { return V3(-y, x, z); };
    int missing = 0;
    for (auto &[pi, r] : flex_rows) {
        std::string mesh = r.str("mesh");
        if (mesh.empty()) continue;
        const PartInstance &part = m.asm_.parts[pi];
        auto groups = r.strings("[group]");
        VisPart vp;
        vp.mesh = mesh;
        vp.swap_group = r.str("deformGroup"), vp.swap_base = r.str("deformMaterialBase"), vp.swap_damaged = r.str("deformMaterialDamaged");
        if (vp.swap_group.empty() || vp.swap_base.empty()) vp.swap_group.clear();
        const MeshFile *mf = nullptr;
        for (auto &f : files)
            if (f.has(mesh)) {
                mf = &f;
                break;
            }
        if (!mf) {
            missing++;
            continue;
        }
        if (!mf->mesh(mesh, vp.subs)) continue;
        V3 fpos = v3of(r, "pos", 0), frot = v3of(r, "rot", 0), fsc = v3of(r, "scale", 1);
        double sx = std::sin(frot.x * PI / 180), cx = std::cos(frot.x * PI / 180), sy = std::sin(frot.y * PI / 180), cy = std::cos(frot.y * PI / 180), sz = std::sin(frot.z * PI / 180), cz = std::cos(frot.z * PI / 180);
        auto rot = [&](V3 p) {
            double x = p.x, y = p.y * cx - p.z * sx, zz = p.y * sx + p.z * cx;
            double x2 = x * cy + zz * sy, z2 = -x * sy + zz * cy;
            return V3(x2 * cz - y * sz, x2 * sz + y * cz, z2);
        };
        double cxs = 0;
        size_t cnt = 0;
        for (auto &s : vp.subs)
            for (auto &p : s.pos) cxs += p[0] * fsc.x, cnt++;
        double side = fpos.x + cxs / std::max<size_t>(1, cnt) >= 0 ? 1 : -1;
        for (auto &s : vp.subs) {
            for (auto &p : s.pos) {
                V3 q = rot(V3(p[0] * fsc.x, p[1] * fsc.y, p[2] * fsc.z));
                q = V3(q.x + fpos.x + part.offset[0] * side + part.node_move[0], q.y + fpos.y + part.offset[1] + part.node_move[1], q.z + fpos.z + part.offset[2] + part.node_move[2]);
                V3 o = ours_f(q.x, q.y, q.z) + shift;
                p = {(float)o.x, (float)o.y, (float)o.z};
            }
            for (auto &n : s.nrm) {
                V3 q = rot(V3(n[0], n[1], n[2]));
                V3 o = ours_f(q.x, q.y, q.z).norm();
                n = {(float)o.x, (float)o.y, (float)o.z};
            }
        }
        std::unordered_set<std::string> gset(groups.begin(), groups.end());
        std::vector<int> cand;
        for (size_t i = 0; i < m.nodes.size(); i++)
            for (auto &g : m.nodes[i].group)
                if (gset.count(g)) {
                    cand.push_back((int)i);
                    break;
                }
        // a wheel's own mesh (tyre, rim): carried rigidly by that wheel
        if (!cand.empty())
            for (size_t wi = 0; wi < v.wheels.size(); wi++) {
                bool sub = true;
                for (int c : cand) sub &= c == v.wheels[wi].n1 || c == v.wheels[wi].n2;
                if (sub) vp.wheel = (int)wi;
            }
        if (vp.wheel >= 0) {
            const JWheel &w = v.wheels[vp.wheel];
            V3 a = rest[w.n1], b = rest[w.n2], c = (a + b) * 0.5, y = (a - b).norm();
            if (y.y < 0) y = y * -1;
            V3 x = y.cross(V3(0, 0, 1)).norm(), zz = x.cross(y);
            for (auto &s : vp.subs) {
                for (auto &p : s.pos) {
                    V3 d = V3(p[0], p[1], p[2]) - c;
                    p = {(float)d.dot(x), (float)d.dot(y), (float)d.dot(zz)};
                }
                for (auto &n : s.nrm) {
                    V3 d(n[0], n[1], n[2]);
                    n = {(float)d.dot(x), (float)d.dot(y), (float)d.dot(zz)};
                }
            }
            vis.parts.push_back(std::move(vp));
            continue;
        }
        if (cand.size() < 3) {
            cand.clear();
            for (size_t i = 0; i < rest.size(); i++)
                if (!v.weak[i]) cand.push_back((int)i);
        }
        // skin weights: the 4 nearest nodes of the flexbody's groups
        for (auto &s : vp.subs) {
            std::vector<std::array<int, 4>> bs(s.pos.size());
            std::vector<std::array<float, 4>> ws(s.pos.size());
            for (size_t k = 0; k < s.pos.size(); k++) {
                V3 p(s.pos[k][0], s.pos[k][1], s.pos[k][2]);
                std::pair<double, int> best[4] = {{1e18, cand[0]}, {1e18, cand[0]}, {1e18, cand[0]}, {1e18, cand[0]}};
                for (int c : cand) {
                    double d = (rest[c] - p).dot(rest[c] - p);
                    if (d < best[3].first) {
                        best[3] = {d, c};
                        std::sort(best, best + 4);
                    }
                }
                double wsum = 0, wv[4];
                for (int q = 0; q < 4; q++) wv[q] = 1.0 / (best[q].first + 0.0025), wsum += wv[q];
                for (int q = 0; q < 4; q++) bs[k][q] = best[q].second, ws[k][q] = (float)(wv[q] / wsum);
            }
            vp.bones.push_back(std::move(bs));
            vp.weights.push_back(std::move(ws));
        }
        vis.parts.push_back(std::move(vp));
    }
    if (missing) vis.notes.push_back(std::to_string(missing) + " flexbody meshes not found");
    // ---- steering wheel prop
    double lock = 450;
    for (auto &[pi, r] : m.table("hydros"))
        if (r.has_num("steeringWheelLock")) {
            lock = r.num("steeringWheelLock");
            break;
        }
    for (auto &[pi, r] : m.table("props")) {
        if (r.str("func") != "steering") continue;
        std::string mesh = r.str("mesh");
        auto ni = [&](const char *k) {
            auto it = m.node_index.find(r.str(k));
            return it == m.node_index.end() ? -1 : it->second;
        };
        int a = ni("idRef"), b = ni("idX"), c = ni("idY");
        if (mesh.empty() || a < 0 || b < 0 || c < 0) continue;
        Prop p;
        bool ok = false;
        for (auto &f : files)
            if (f.has(mesh)) {
                ok = f.mesh(mesh, p.subs);
                break;
            }
        if (!ok) {
            vis.notes.push_back("prop mesh " + mesh + " not found");
            continue;
        }
        p.refs[0] = a, p.refs[1] = b, p.refs[2] = c;
        V3 br = v3of(r, "baseRotation", 0), bt = v3of(r, "baseTranslation", 0), ax = v3of(r, "rotation", 0);
        p.base_rot[0] = br.x, p.base_rot[1] = br.y, p.base_rot[2] = br.z;
        p.base_trans[0] = bt.x, p.base_trans[1] = bt.y, p.base_trans[2] = bt.z;
        p.axis[0] = ax.x, p.axis[1] = ax.y, p.axis[2] = ax.z;
        p.multiplier = r.num("multiplier", 1), p.offset = r.num("offset", 0), p.mn = r.num("min", -1e9), p.mx = r.num("max", 1e9), p.lock_deg = lock;
        vis.props.push_back(std::move(p));
    }
    // ---- materials and textures
    FileIndex idx(z.entries);
    for (auto &e : z.entries)
        if (ends(lc_(e.name), "materials.json")) {
            std::string text, e2;
            if (z.read(e, text, e2)) parse_materials(text, idx, vis.materials, "");
        }
    std::set<std::string> used;
    for (auto &p : vis.parts) {
        for (auto &s : p.subs)
            if (!is_invisible_material(s.material)) used.insert(s.material);
        if (!p.swap_damaged.empty() && !is_invisible_material(p.swap_damaged)) used.insert(p.swap_damaged);
    }
    for (auto &p : vis.props)
        for (auto &s : p.subs) used.insert(s.material);
    bool need_common = false;
    for (auto &u : used) need_common |= !vis.materials.count(u);
    if (common && need_common) {
        FileIndex cidx(common->entries);
        std::map<std::string, MatDef> cm;
        for (auto &e : common->entries)
            if (ends(lc_(e.name), "materials.json")) {
                std::string text, e2;
                if (common->read(e, text, e2)) parse_materials(text, cidx, cm, "common:");
            }
        for (auto &[k, d] : cm)
            if (used.count(k) && !vis.materials.count(k)) vis.materials[k] = d;
    }
    for (auto &u : used) {
        auto it = vis.materials.find(u);
        if (it == vis.materials.end()) continue;
        for (const std::string *t : {&it->second.base_map, &it->second.normal_map}) {
            if (t->empty() || vis.textures.count(*t)) continue;
            std::string data, e2;
            if (t->rfind("common:", 0) == 0) {
                if (!common) continue;
                const ZipEntry *e = common->find(t->substr(7));
                if (e && common->read(*e, data, e2)) vis.textures[*t] = std::move(data);
            } else {
                const ZipEntry *e = z.find(*t);
                if (e && z.read(*e, data, e2)) vis.textures[*t] = std::move(data);
            }
        }
    }
    return true;
}

}  // namespace ec
