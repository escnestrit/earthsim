#include "jtable.hpp"
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace ec {

namespace {
bool fix_number(const std::string &tok, std::string &out) {
    std::string t = tok;
    if (!t.empty() && t[0] == '+') t = t.substr(1);
    bool neg = false;
    if (!t.empty() && t[0] == '-') neg = true, t = t.substr(1);
    size_t e = t.find_first_of("eE");
    std::string mant = e == std::string::npos ? t : t.substr(0, e), ex = e == std::string::npos ? "" : t.substr(e);
    size_t d = mant.find('.');
    std::string ip = d == std::string::npos ? mant : mant.substr(0, d), fp = d == std::string::npos ? "" : mant.substr(d + 1);
    if (ip.empty() && fp.empty()) return false;
    for (char c : ip)
        if (!isdigit((unsigned char)c)) return false;
    for (char c : fp)
        if (!isdigit((unsigned char)c)) return false;
    if (!ex.empty()) {
        std::string q = ex.substr(1);
        if (!q.empty() && (q[0] == '+' || q[0] == '-')) q = q.substr(1);
        if (q.empty()) return false;
        for (char c : q)
            if (!isdigit((unsigned char)c)) return false;
    }
    size_t nz = ip.find_first_not_of('0');
    ip = nz == std::string::npos ? "0" : ip.substr(nz);
    if (fp.empty()) fp = "0";
    out = std::string(neg ? "-" : "") + ip + "." + fp + ex;
    return true;
}
}  // namespace

std::string jbeam_normalize(const std::string &text) {
    enum Prev { None, Open, Val, Comma, Colon } prev = None;
    const std::string &b = text;
    size_t n = b.size(), i = 0;
    std::string out;
    out.reserve(n);
    auto value_start = [&] {
        if (prev == Val || prev == Comma) out.push_back(',');
    };
    while (i < n) {
        unsigned char c = b[i];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == 0xEF || c == 0xBB || c == 0xBF) {
            i++;
        } else if (c == '/' && i + 1 < n && b[i + 1] == '/') {
            while (i < n && b[i] != '\n') i++;
        } else if (c == '/' && i + 1 < n && b[i + 1] == '*') {
            i += 2;
            while (i + 1 < n && !(b[i] == '*' && b[i + 1] == '/')) i++;
            i = std::min(i + 2, n);
        } else if (c == '"') {
            size_t s = i;
            i++;
            while (i < n && b[i] != '"') i += b[i] == '\\' ? 2 : 1;
            size_t e = std::min(i + 1, n);
            value_start();
            for (size_t k = s; k < e; k++) {
                unsigned char ch = b[k];
                if (ch == '\n') out += "\\n";
                else if (ch == '\r') out += "\\r";
                else if (ch == '\t') out += "\\t";
                else if (ch < 0x20) {
                } else out.push_back((char)ch);
            }
            prev = Val;
            i = e;
        } else if (c == '{' || c == '[') {
            value_start();
            out.push_back((char)c);
            prev = Open;
            i++;
        } else if (c == '}' || c == ']') {
            out.push_back((char)c);
            prev = Val;
            i++;
        } else if (c == ',') {
            if (prev == Val) prev = Comma;
            i++;
        } else if (c == ':') {
            out.push_back(':');
            prev = Colon;
            i++;
        } else if (c == '-' || c == '+' || c == '.' || isdigit(c)) {
            size_t s = i++;
            while (i < n && (isdigit((unsigned char)b[i]) || b[i] == '.' || b[i] == 'e' || b[i] == 'E' || ((b[i] == '-' || b[i] == '+') && (b[i - 1] == 'e' || b[i - 1] == 'E')))) i++;
            std::string num;
            if (fix_number(b.substr(s, i - s), num)) {
                value_start();
                out += num;
                prev = Val;
            }
        } else if (isalpha(c)) {
            size_t s = i;
            while (i < n && isalnum((unsigned char)b[i])) i++;
            std::string w = b.substr(s, i - s);
            if (w == "true" || w == "false" || w == "null") {
                value_start();
                out += w;
                prev = Val;
            } else if (w == "NaN" || w == "nan" || w == "inf" || w == "Infinity") {
                value_start();
                out += "0";
                prev = Val;
            }
        } else
            i++;
    }
    return out;
}

bool jbeam_parse(const std::string &text, Json &out, std::string &err) { return Json::parse(jbeam_normalize(text), out, &err); }

const Json *Row::get(const std::string &k) const {
    for (auto it = vals.rbegin(); it != vals.rend(); ++it)
        if (it->first == k) return &it->second;
    return nullptr;
}
bool Row::has_num(const std::string &k) const {
    const Json *j = get(k);
    return j && j->kind == Json::Num;
}
double Row::num(const std::string &k, double d) const {
    const Json *j = get(k);
    return j && j->kind == Json::Num ? j->n : d;
}
std::string Row::str(const std::string &k, const std::string &d) const {
    const Json *j = get(k);
    return j && j->kind == Json::Str ? j->s : d;
}
bool Row::flag(const std::string &k) const {
    const Json *j = get(k);
    if (!j) return false;
    if (j->kind == Json::Bool) return j->b;
    if (j->kind == Json::Num) return j->n != 0;
    return false;
}
std::vector<std::string> Row::strings(const std::string &k) const {
    std::vector<std::string> o;
    const Json *j = get(k);
    if (!j) return o;
    if (j->kind == Json::Str && !j->s.empty()) o.push_back(j->s);
    if (j->kind == Json::Arr)
        for (auto &x : j->a)
            if (x.kind == Json::Str && !x.s.empty()) o.push_back(x.s);
    return o;
}

static void setv(std::vector<std::pair<std::string, Json>> &st, std::string k, const Json &v) {
    while (!k.empty() && k.back() == ':') k.pop_back();
    for (auto &e : st)
        if (e.first == k) {
            e.second = v;
            return;
        }
    st.push_back({k, v});
}

static std::string hname(const Json &c) {
    std::string s = c.kind == Json::Str ? c.s : "";
    while (!s.empty() && s.back() == ':') s.pop_back();
    return s;
}

std::vector<Row> jbeam_rows(const Json &section) {
    std::vector<Row> out;
    if (section.kind != Json::Arr) return out;
    std::vector<std::string> header;
    bool have = false;
    std::vector<std::pair<std::string, Json>> state;
    for (auto &it : section.a) {
        if (it.kind == Json::Obj) {
            for (auto &[k, v] : it.o) setv(state, k, v);
        } else if (it.kind == Json::Arr) {
            if (!have) {
                for (auto &c : it.a) header.push_back(hname(c));
                have = true;
                continue;
            }
            Row r;
            r.vals = state;
            for (size_t i = 0; i < it.a.size(); i++) {
                const Json &c = it.a[i];
                if (c.kind == Json::Obj && i >= header.size()) {
                    for (auto &[k, v] : c.o) setv(r.vals, k, v);
                } else if (i < header.size())
                    setv(r.vals, header[i], c);
            }
            out.push_back(std::move(r));
        }
    }
    return out;
}

std::vector<std::pair<int, Row>> jbeam_rows_merged(const std::vector<std::pair<int, const Json *>> &sections) {
    std::vector<std::pair<int, Row>> out;
    std::vector<std::string> header;
    bool have = false;
    std::vector<std::pair<std::string, Json>> state;
    for (auto &[part, sec] : sections) {
        if (!sec || sec->kind != Json::Arr) continue;
        bool seen = false;
        for (auto &it : sec->a) {
            if (it.kind == Json::Obj) {
                for (auto &[k, v] : it.o) setv(state, k, v);
            } else if (it.kind == Json::Arr) {
                if (!seen) {
                    seen = true;
                    if (!have) {
                        for (auto &c : it.a) header.push_back(hname(c));
                        have = true;
                    }
                    continue;
                }
                Row r;
                r.vals = state;
                for (size_t i = 0; i < it.a.size(); i++) {
                    const Json &c = it.a[i];
                    if (c.kind == Json::Obj && i >= header.size()) {
                        for (auto &[k, v] : c.o) setv(r.vals, k, v);
                    } else if (i < header.size())
                        setv(r.vals, header[i], c);
                }
                out.push_back({part, std::move(r)});
            }
        }
    }
    return out;
}

Json jbeam_resolve(const Json &j, const std::map<std::string, Json> &vars, std::vector<std::string> &warn) {
    if (j.kind == Json::Str && !j.s.empty() && j.s[0] == '$') {
        if (j.s.rfind("$=", 0) == 0) {
            std::map<std::string, double> nums;
            for (auto &[k, v] : vars)
                if (v.kind == Json::Num) nums[k] = v.n;
            Json r;
            std::string err;
            if (!jbeam_eval(j.s.substr(2), nums, r, err)) {
                warn.push_back("expression '" + j.s + "': " + err);
                return Json();
            }
            return r;
        }
        auto it = vars.find(j.s);
        return it == vars.end() ? j : it->second;
    }
    if (j.kind == Json::Arr) {
        Json o;
        o.kind = Json::Arr;
        for (auto &x : j.a) o.a.push_back(jbeam_resolve(x, vars, warn));
        return o;
    }
    if (j.kind == Json::Obj) {
        Json o;
        o.kind = Json::Obj;
        for (auto &[k, v] : j.o) o.o.push_back({k, k == "variables" ? v : jbeam_resolve(v, vars, warn)});
        return o;
    }
    return j;
}

// ---- expressions
namespace {
struct V {
    int k = 0;  // 0 nil, 1 bool, 2 num
    bool b = false;
    double n = 0;
    bool truthy() const { return !(k == 0 || (k == 1 && !b)); }
    bool operator==(const V &o) const { return k == o.k && (k == 0 || (k == 1 ? b == o.b : n == o.n)); }
};
V num(double n) { return V{2, false, n}; }
V boo(bool b) { return V{1, b, 0}; }
struct EP {
    const std::string &s;
    size_t i = 0;
    const std::map<std::string, double> &vars;
    std::string err;
    void ws() {
        while (i < s.size() && isspace((unsigned char)s[i])) i++;
    }
    bool eat(const char *t) {
        ws();
        size_t l = strlen(t);
        if (s.compare(i, l, t) == 0) return i += l, true;
        return false;
    }
    bool word(const char *w) {
        ws();
        size_t l = strlen(w), e = i + l;
        if (s.compare(i, l, w) == 0 && !(e < s.size() && (isalnum((unsigned char)s[e]) || s[e] == '_'))) return i = e, true;
        return false;
    }
    bool N(const V &v, double &o) {
        if (v.k != 2) return err = "arithmetic on nil/bool", false;
        o = v.n;
        return true;
    }
    bool orx(V &l) {
        if (!andx(l)) return false;
        while (word("or")) {
            V r;
            if (!andx(r)) return false;
            l = l.truthy() ? l : r;
        }
        return true;
    }
    bool andx(V &l) {
        if (!cmp(l)) return false;
        while (word("and")) {
            V r;
            if (!cmp(r)) return false;
            l = l.truthy() ? r : l;
        }
        return true;
    }
    bool cmp(V &l) {
        if (!add(l)) return false;
        for (const char *op : {"==", "~=", "!=", "<=", ">=", "<", ">"}) {
            if (eat(op)) {
                V r;
                if (!add(r)) return false;
                std::string o = op;
                bool res;
                if (o == "==") res = l == r;
                else if (o == "~=" || o == "!=") res = !(l == r);
                else {
                    if (l.k != 2 || r.k != 2) return err = "comparison of non-numbers", false;
                    res = o == "<=" ? l.n <= r.n : o == ">=" ? l.n >= r.n : o == "<" ? l.n < r.n : l.n > r.n;
                }
                l = boo(res);
                return true;
            }
        }
        return true;
    }
    bool add(V &l) {
        if (!mul(l)) return false;
        for (;;) {
            ws();
            if (i >= s.size() || (s[i] != '+' && s[i] != '-')) return true;
            char op = s[i++];
            V r;
            double a, b;
            if (!mul(r) || !N(l, a) || !N(r, b)) return false;
            l = num(op == '+' ? a + b : a - b);
        }
    }
    bool mul(V &l) {
        if (!unary(l)) return false;
        for (;;) {
            ws();
            if (i >= s.size() || (s[i] != '*' && s[i] != '/' && s[i] != '%')) return true;
            char op = s[i++];
            V r;
            double a, b;
            if (!unary(r) || !N(l, a) || !N(r, b)) return false;
            l = num(op == '*' ? a * b : op == '/' ? a / b : a - b * std::floor(a / b));
        }
    }
    bool unary(V &v) {
        if (eat("-")) {
            V x;
            double a;
            if (!unary(x) || !N(x, a)) return false;
            v = num(-a);
            return true;
        }
        if (eat("+")) return unary(v);
        if (word("not")) {
            V x;
            if (!unary(x)) return false;
            v = boo(!x.truthy());
            return true;
        }
        return pow(v);
    }
    bool pow(V &v) {
        if (!atom(v)) return false;
        if (eat("^")) {
            V e;
            double a, b;
            if (!unary(e) || !N(v, a) || !N(e, b)) return false;
            v = num(std::pow(a, b));
        }
        return true;
    }
    bool atom(V &v) {
        ws();
        if (i >= s.size()) return err = "unexpected end", false;
        char c = s[i];
        if (c == '(') {
            i++;
            if (!orx(v)) return false;
            if (!eat(")")) return err = "missing )", false;
            return true;
        }
        if (c == '$') {
            size_t st = ++i;
            while (i < s.size() && (isalnum((unsigned char)s[i]) || s[i] == '_')) i++;
            auto it = vars.find("$" + s.substr(st, i - st));
            v = it == vars.end() ? V() : num(it->second);
            return true;
        }
        if (word("nil")) return v = V(), true;
        if (word("true")) return v = boo(true), true;
        if (word("false")) return v = boo(false), true;
        if (isdigit((unsigned char)c) || c == '.') {
            size_t st = i;
            while (i < s.size() && (isdigit((unsigned char)s[i]) || s[i] == '.' || s[i] == 'e' || s[i] == 'E' || ((s[i] == '+' || s[i] == '-') && (s[i - 1] == 'e' || s[i - 1] == 'E')))) i++;
            v = num(atof(s.substr(st, i - st).c_str()));
            return true;
        }
        err = std::string("unexpected '") + c + "'";
        return false;
    }
};
}  // namespace

bool jbeam_eval(const std::string &src, const std::map<std::string, double> &vars, Json &out, std::string &err) {
    EP p{src, 0, vars, ""};
    V v;
    if (!p.orx(v)) return err = p.err, false;
    p.ws();
    if (p.i != src.size()) return err = "trailing input", false;
    out = Json();
    if (v.k == 1) out.kind = Json::Bool, out.b = v.b;
    else if (v.k == 2) out.kind = Json::Num, out.n = v.n;
    return true;
}

}  // namespace ec
