#include "json.hpp"
#include <cstdlib>

namespace ec {

namespace {
struct P {
    const std::string &t;
    size_t i = 0;
    std::string err;
    void ws() {
        while (i < t.size() && (t[i] == ' ' || t[i] == '\n' || t[i] == '\r' || t[i] == '\t')) i++;
    }
    static void put_utf8(std::string &o, unsigned cp) {
        if (cp < 0x80)
            o += char(cp);
        else if (cp < 0x800) {
            o += char(0xC0 | (cp >> 6));
            o += char(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            o += char(0xE0 | (cp >> 12));
            o += char(0x80 | ((cp >> 6) & 0x3F));
            o += char(0x80 | (cp & 0x3F));
        } else {
            o += char(0xF0 | (cp >> 18));
            o += char(0x80 | ((cp >> 12) & 0x3F));
            o += char(0x80 | ((cp >> 6) & 0x3F));
            o += char(0x80 | (cp & 0x3F));
        }
    }
    bool string(std::string &o) {
        if (t[i] != '"') return false;
        i++;
        while (i < t.size() && t[i] != '"') {
            char c = t[i++];
            if (c != '\\') {
                o += c;
                continue;
            }
            if (i >= t.size()) return false;
            char e = t[i++];
            switch (e) {
                case 'n': o += '\n'; break;
                case 't': o += '\t'; break;
                case 'r': o += '\r'; break;
                case 'b': o += '\b'; break;
                case 'f': o += '\f'; break;
                case 'u': {
                    if (i + 4 > t.size()) return false;
                    unsigned cp = std::strtoul(t.substr(i, 4).c_str(), nullptr, 16);
                    i += 4;
                    if (cp >= 0xD800 && cp < 0xDC00 && i + 6 <= t.size() && t[i] == '\\' && t[i + 1] == 'u') {
                        unsigned lo = std::strtoul(t.substr(i + 2, 4).c_str(), nullptr, 16);
                        i += 6;
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    }
                    put_utf8(o, cp);
                    break;
                }
                default: o += e;
            }
        }
        if (i >= t.size()) return false;
        i++;
        return true;
    }
    bool value(Json &v, int depth) {
        if (depth > 200) return false;
        ws();
        if (i >= t.size()) return false;
        char c = t[i];
        if (c == '{') {
            v.kind = Json::Obj;
            i++;
            ws();
            if (i < t.size() && t[i] == '}') {
                i++;
                return true;
            }
            while (true) {
                ws();
                std::string k;
                if (i >= t.size() || !string(k)) return false;
                ws();
                if (i >= t.size() || t[i] != ':') return false;
                i++;
                v.o.emplace_back(std::move(k), Json());
                if (!value(v.o.back().second, depth + 1)) return false;
                ws();
                if (i < t.size() && t[i] == ',') {
                    i++;
                    continue;
                }
                if (i < t.size() && t[i] == '}') {
                    i++;
                    return true;
                }
                return false;
            }
        }
        if (c == '[') {
            v.kind = Json::Arr;
            i++;
            ws();
            if (i < t.size() && t[i] == ']') {
                i++;
                return true;
            }
            while (true) {
                v.a.emplace_back();
                if (!value(v.a.back(), depth + 1)) return false;
                ws();
                if (i < t.size() && t[i] == ',') {
                    i++;
                    continue;
                }
                if (i < t.size() && t[i] == ']') {
                    i++;
                    return true;
                }
                return false;
            }
        }
        if (c == '"') {
            v.kind = Json::Str;
            return string(v.s);
        }
        if (t.compare(i, 4, "true") == 0) {
            v.kind = Json::Bool;
            v.b = true;
            i += 4;
            return true;
        }
        if (t.compare(i, 5, "false") == 0) {
            v.kind = Json::Bool;
            i += 5;
            return true;
        }
        if (t.compare(i, 4, "null") == 0) {
            i += 4;
            return true;
        }
        const char *start = t.c_str() + i;
        char *end = nullptr;
        double d = std::strtod(start, &end);
        if (end == start) return false;
        v.kind = Json::Num;
        v.n = d;
        i += end - start;
        return true;
    }
};
}  // namespace

bool Json::parse(const std::string &text, Json &out, std::string *err) {
    P p{text};
    out = Json();
    bool ok = p.value(out, 0);
    if (!ok && err) *err = "JSON syntax error near byte " + std::to_string(p.i);
    return ok;
}

}  // namespace ec
