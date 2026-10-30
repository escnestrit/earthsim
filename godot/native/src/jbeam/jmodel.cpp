#include "jmodel.hpp"
#include <algorithm>
#include <functional>

namespace ec {

void Library::add_file(const std::string &file, const std::string &text) {
    if (text.find_first_not_of(" \t\r\n") == std::string::npos) {
        warnings.push_back(file + ": empty file");
        return;
    }
    Json j;
    std::string err;
    if (!jbeam_parse(text, j, err)) {
        warnings.push_back(file + ": " + err);
        return;
    }
    if (j.kind != Json::Obj) {
        warnings.push_back(file + ": top level is not an object");
        return;
    }
    for (auto &[name, body] : j.o) {
        if (parts.count(name)) {
            warnings.push_back(file + ": duplicate part '" + name + "' ignored");
            continue;
        }
        const Json *st = body.get("slotType");
        by_type[st && st->kind == Json::Str ? st->s : ""].push_back(name);
        parts[name] = PartDef{file, body};
    }
}

void Library::add_zip(Zip &z, bool fallback_only) {
    Library other;
    for (auto &e : z.entries) {
        if (e.name.size() < 6 || e.name.compare(e.name.size() - 6, 6, ".jbeam") != 0) continue;
        std::string data, err;
        if (!z.read(e, data, err)) {
            warnings.push_back(e.name + ": " + err);
            continue;
        }
        other.add_file(e.name, data);
    }
    std::vector<std::string> names;
    for (auto &[n, d] : other.parts) names.push_back(n);
    std::sort(names.begin(), names.end());
    for (auto &n : names) {
        if (parts.count(n)) {
            if (!fallback_only) warnings.push_back(n + ": duplicate part ignored");
            continue;
        }
        const Json *st = other.parts[n].body.get("slotType");
        by_type[st && st->kind == Json::Str ? st->s : ""].push_back(n);
        parts[n] = std::move(other.parts[n]);
    }
    if (!fallback_only) warnings.insert(warnings.end(), other.warnings.begin(), other.warnings.end());
}

namespace {
struct SlotRef {
    std::string ty;
    std::vector<std::string> allow;
    std::string def;
    std::array<double, 3> off{}, move{};
};
std::array<double, 3> v3of(const Json *j) {
    std::array<double, 3> r{};
    if (!j || j->kind != Json::Obj) return r;
    const char *k[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; i++)
        if (const Json *v = j->get(k[i])) r[i] = v->num(0);
    return r;
}
std::vector<SlotRef> slots_of(const Json &body) {
    std::vector<SlotRef> out;
    if (const Json *s = body.get("slots"))
        for (auto &r : jbeam_rows(*s)) {
            SlotRef sr;
            sr.ty = r.str("type");
            sr.allow = {sr.ty};
            sr.def = r.str("default");
            sr.off = v3of(r.get("nodeOffset"));
            sr.move = v3of(r.get("nodeMove"));
            out.push_back(sr);
        }
    if (const Json *s = body.get("slots2"))
        for (auto &r : jbeam_rows(*s)) {
            SlotRef sr;
            sr.ty = r.str("name");
            sr.allow = r.strings("allowTypes");
            sr.def = r.str("default");
            sr.off = v3of(r.get("nodeOffset"));
            sr.move = v3of(r.get("nodeMove"));
            out.push_back(sr);
        }
    return out;
}
}  // namespace

Assembly assemble(const Library &lib, const std::string &root, const std::map<std::string, std::string> &ov, const std::map<std::string, Json> *vars) {
    Assembly a;
    if (!lib.parts.count(root)) {
        a.warnings.push_back("main part '" + root + "' not found");
        return a;
    }
    // variables: defaults from every part's "variables" table
    std::vector<std::string> names;
    for (auto &[n, p] : lib.parts) names.push_back(n);
    std::sort(names.begin(), names.end());
    for (auto &n : names)
        if (const Json *v = lib.parts.at(n).body.get("variables"))
            for (auto &r : jbeam_rows(*v)) {
                const Json *d = r.get("default");
                std::string nm = r.str("name");
                if (!nm.empty() && d && !a.vars.count(nm)) a.vars[nm] = *d;
            }
    // the configuration's tuning values over the defaults
    if (vars)
        for (auto &[k, v] : *vars) a.vars[k] = v;
    struct Pick {
        std::string name, slot;
        std::array<double, 3> off, move;
    };
    std::vector<Pick> chosen;
    std::unordered_map<std::string, Json> resolved;
    auto body = [&](const std::string &n) -> const Json & {
        auto it = resolved.find(n);
        if (it != resolved.end()) return it->second;
        std::vector<std::string> w;
        Json r = jbeam_resolve(lib.parts.at(n).body, a.vars, w);
        a.warnings.insert(a.warnings.end(), w.begin(), w.end());
        return resolved[n] = std::move(r);
    };
    std::function<void(const std::string &, const std::string &, std::array<double, 3>, std::array<double, 3>, int)> walk = [&](const std::string &name, const std::string &slot, std::array<double, 3> off,
                                                                                                                               std::array<double, 3> mv, int depth) {
        if (depth > 40) {
            a.warnings.push_back("slot tree too deep at '" + name + "'");
            return;
        }
        chosen.push_back({name, slot, off, mv});
        for (auto &s : slots_of(body(name))) {
            auto it = ov.find(s.ty);
            std::string want = it != ov.end() ? it->second : s.def;
            if (want.empty()) continue;
            std::string pick = want;
            if (!lib.parts.count(pick)) {
                a.missing.push_back({s.ty, want});
                std::string alt;
                for (auto &t : s.allow) {
                    auto bt = lib.by_type.find(t);
                    if (bt != lib.by_type.end() && !bt->second.empty()) {
                        alt = bt->second[0];
                        break;
                    }
                }
                if (alt.empty()) {
                    a.warnings.push_back("slot '" + s.ty + "': part '" + want + "' missing, slot left empty");
                    continue;
                }
                a.warnings.push_back("slot '" + s.ty + "': part '" + want + "' missing, using '" + alt + "'");
                pick = alt;
            }
            std::array<double, 3> o = off, m = mv;
            for (int i = 0; i < 3; i++) o[i] += s.off[i], m[i] += s.move[i];
            walk(pick, s.ty, o, m, depth + 1);
        }
    };
    walk(root, "", {0, 0, 0}, {0, 0, 0}, 0);
    for (auto &c : chosen) a.parts.push_back({c.name, c.slot, c.off, c.move, body(c.name)});
    return a;
}

static double limit_of(const Row &r, const char *k, double d) {
    const Json *j = r.get(k);
    if (j && j->kind == Json::Num) return j->n < 1e30 ? j->n : -1;
    if (j && j->kind == Json::Str && j->s.find("FLT_MAX") != std::string::npos) return -1;
    return d < 1e30 ? d : -1;
}
double JBeam::deform() const { return limit_of(row, "beamDeform", 220000); }
double JBeam::strength() const { return limit_of(row, "beamStrength", 1e300); }

std::vector<std::pair<int, Row>> JModel::table(const std::string &section) const {
    std::vector<std::pair<int, const Json *>> secs;
    for (size_t i = 0; i < asm_.parts.size(); i++)
        if (const Json *s = asm_.parts[i].body.get(section.c_str())) secs.push_back({(int)i, s});
    return jbeam_rows_merged(secs);
}
double JModel::total_mass() const {
    double m = 0;
    for (auto &n : nodes) m += n.mass;
    return m;
}

JModel build_model(Assembly as) {
    JModel m;
    m.asm_ = std::move(as);
    int dup = 0;
    for (auto &[pi, r] : m.table("nodes")) {
        const PartInstance &part = m.asm_.parts[pi];
        std::string id = r.str("id");
        if (id.empty()) continue;
        auto mv = v3of(r.get("nodeMove")), off = v3of(r.get("nodeOffset"));
        std::array<double, 3> raw{r.num("posX"), r.num("posY"), r.num("posZ")};
        double mirror = raw[0] < 0 ? -1 : 1;
        JNode n;
        for (int i = 0; i < 3; i++) {
            double o = i == 0 ? (part.offset[0] + off[0]) * mirror : part.offset[i] + off[i];
            n.pos[i] = raw[i] + o + part.node_move[i] + mv[i];
        }
        if (m.node_index.count(id)) {
            dup++;
            continue;
        }
        n.name = id;
        n.part = pi;
        n.mass = r.num("nodeWeight", 25);
        n.friction = r.num("frictionCoef", 1);
        const Json *c = r.get("collision");
        n.collision = !(c && c->kind == Json::Bool && !c->b);
        n.self_collision = r.flag("selfCollision");
        n.fixed = r.flag("fixed");
        n.group = r.strings("group");
        n.material = r.str("nodeMaterial");
        m.node_index[id] = (int)m.nodes.size();
        m.nodes.push_back(std::move(n));
    }
    int shown = 0;
    for (auto &[pi, r] : m.table("beams")) {
        std::string a = r.str("id1"), b = r.str("id2");
        if (a.empty() || b.empty()) continue;
        auto ia = m.node_index.find(a), ib = m.node_index.find(b);
        if (ia != m.node_index.end() && ib != m.node_index.end() && ia->second != ib->second) {
            std::string kind = r.str("beamType", "|NORMAL");
            while (!kind.empty() && kind[0] == '|') kind.erase(0, 1);
            m.beams.push_back({ia->second, ib->second, pi, kind, r});
        } else if (!r.flag("optional")) {
            m.skipped_beams++;
            if (shown++ < 12) m.warnings.push_back(m.asm_.parts[pi].name + ": beam " + a + "-" + b + " references a missing node");
        }
    }
    if (dup) m.warnings.push_back(std::to_string(dup) + " node ids were declared by more than one part (first definition kept)");
    return m;
}

}  // namespace ec
