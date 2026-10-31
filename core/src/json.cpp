#include "json.hpp"
#include <cstdlib>
#include <cstring>

namespace es {
namespace {
const Json NULL_JSON;

struct P {
    const char *c, *e;
    bool ok = true;
    void ws() {
        while (c < e && (*c == ' ' || *c == '\n' || *c == '\r' || *c == '\t')) c++;
    }
    void utf8(std::string &o, unsigned cp) {
        if (cp < 0x80) o += char(cp);
        else if (cp < 0x800) o += char(0xC0 | (cp >> 6)), o += char(0x80 | (cp & 63));
        else if (cp < 0x10000) o += char(0xE0 | (cp >> 12)), o += char(0x80 | ((cp >> 6) & 63)), o += char(0x80 | (cp & 63));
        else o += char(0xF0 | (cp >> 18)), o += char(0x80 | ((cp >> 12) & 63)), o += char(0x80 | ((cp >> 6) & 63)), o += char(0x80 | (cp & 63));
    }
    bool str(std::string &o) {
        if (c >= e || *c != '"') return ok = false;
        c++;
        while (c < e && *c != '"') {
            if (*c == '\\' && c + 1 < e) {
                c++;
                char x = *c++;
                switch (x) {
                    case 'n': o += '\n'; break;
                    case 't': o += '\t'; break;
                    case 'r': o += '\r'; break;
                    case 'b': o += '\b'; break;
                    case 'f': o += '\f'; break;
                    case 'u': {
                        if (e - c < 4) return ok = false;
                        unsigned cp = unsigned(strtoul(std::string(c, 4).c_str(), nullptr, 16));
                        c += 4;
                        if (cp >= 0xD800 && cp < 0xDC00 && e - c >= 6 && c[0] == '\\' && c[1] == 'u') {
                            unsigned lo = unsigned(strtoul(std::string(c + 2, 4).c_str(), nullptr, 16));
                            c += 6;
                            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                        }
                        utf8(o, cp);
                        break;
                    }
                    default: o += x;
                }
            } else
                o += *c++;
        }
        if (c >= e) return ok = false;
        c++;
        return true;
    }
    Json val(int depth) {
        Json j;
        ws();
        if (c >= e || depth > 64) {
            ok = false;
            return j;
        }
        if (*c == '{') {
            j.kind = Json::Obj;
            c++;
            ws();
            if (c < e && *c == '}') return c++, j;
            while (ok) {
                ws();
                std::string k;
                if (!str(k)) break;
                ws();
                if (c >= e || *c != ':') {
                    ok = false;
                    break;
                }
                c++;
                j.o.emplace_back(std::move(k), val(depth + 1));
                ws();
                if (c < e && *c == ',') {
                    c++;
                    continue;
                }
                if (c < e && *c == '}') {
                    c++;
                    break;
                }
                ok = false;
            }
        } else if (*c == '[') {
            j.kind = Json::Arr;
            c++;
            ws();
            if (c < e && *c == ']') return c++, j;
            while (ok) {
                j.a.push_back(val(depth + 1));
                ws();
                if (c < e && *c == ',') {
                    c++;
                    continue;
                }
                if (c < e && *c == ']') {
                    c++;
                    break;
                }
                ok = false;
            }
        } else if (*c == '"') {
            j.kind = Json::Str;
            str(j.s);
        } else if (e - c >= 4 && !strncmp(c, "true", 4)) {
            j.kind = Json::Bool, j.b = true, c += 4;
        } else if (e - c >= 5 && !strncmp(c, "false", 5)) {
            j.kind = Json::Bool, c += 5;
        } else if (e - c >= 4 && !strncmp(c, "null", 4)) {
            c += 4;
        } else {
            char *end = nullptr;
            std::string tmp(c, std::min<size_t>(e - c, 64));
            j.n = strtod(tmp.c_str(), &end);
            if (end == tmp.c_str()) ok = false;
            else j.kind = Json::Num, c += end - tmp.c_str();
        }
        return j;
    }
};
}  // namespace

Json Json::parse(const std::string &t) {
    P p{t.data(), t.data() + t.size()};
    Json j = p.val(0);
    return p.ok ? j : Json();
}

const Json &Json::operator[](const std::string &k) const {
    if (kind == Obj)
        for (auto &kv : o)
            if (kv.first == k) return kv.second;
    return NULL_JSON;
}
const Json &Json::operator[](size_t i) const { return kind == Arr && i < a.size() ? a[i] : NULL_JSON; }
bool Json::has(const std::string &k) const {
    if (kind == Obj)
        for (auto &kv : o)
            if (kv.first == k) return true;
    return false;
}

}  // namespace es
