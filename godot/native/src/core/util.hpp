// Small shared types for the Earth core (plain C++17, no engine dependency, so it can be tested natively).
#pragma once
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace ec {

constexpr double PI = 3.14159265358979323846;

struct V2 {
    double x = 0, y = 0;
    V2() = default;
    V2(double a, double b) : x(a), y(b) {}
    V2 operator+(V2 o) const { return {x + o.x, y + o.y}; }
    V2 operator-(V2 o) const { return {x - o.x, y - o.y}; }
    V2 operator*(double k) const { return {x * k, y * k}; }
    double dot(V2 o) const { return x * o.x + y * o.y; }
    double cross(V2 o) const { return x * o.y - y * o.x; }
    double len() const { return std::sqrt(x * x + y * y); }
    V2 norm() const {
        double l = len();
        return l > 1e-12 ? V2(x / l, y / l) : V2(0, 0);
    }
    V2 perp() const { return {-y, x}; }  // left normal
};

struct V3 {
    double x = 0, y = 0, z = 0;
    V3() = default;
    V3(double a, double b, double c) : x(a), y(b), z(c) {}
    V3 operator+(V3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(V3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(double k) const { return {x * k, y * k, z * k}; }
    double dot(V3 o) const { return x * o.x + y * o.y + z * o.z; }
    V3 cross(V3 o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    double len() const { return std::sqrt(x * x + y * y + z * z); }
    V3 norm() const {
        double l = len();
        return l > 1e-12 ? V3(x / l, y / l, z / l) : V3(0, 0, 1);
    }
};

using Tags = std::vector<std::pair<std::string, std::string>>;

inline const std::string *tag(const Tags &t, const char *k) {
    for (auto &p : t)
        if (p.first == k) return &p.second;
    return nullptr;
}
inline std::string tagv(const Tags &t, const char *k, const char *def = "") {
    auto p = tag(t, k);
    return p ? *p : std::string(def);
}
inline void set_tag(Tags &t, const std::string &k, const std::string &v) {
    for (auto &p : t)
        if (p.first == k) {
            p.second = v;
            return;
        }
    t.push_back({k, v});
}
inline bool parse_num(const std::string &s, double &out) {
    // first number in the string, ',' as decimal point allowed
    std::string c;
    bool seen = false;
    for (char ch : s) {
        if ((ch >= '0' && ch <= '9') || ch == '.' || ch == ',' || (ch == '-' && !seen)) {
            c += ch == ',' ? '.' : ch;
            seen = true;
        } else if (seen)
            break;
        else if (ch != ' ')
            return false;
    }
    if (c.empty()) return false;
    char *end = nullptr;
    out = std::strtod(c.c_str(), &end);
    return end && end != c.c_str() && std::isfinite(out);
}

// deterministic hashes
inline uint64_t mix64(uint64_t h) {
    h ^= h >> 31;
    h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 27;
    h *= 0x94D049BB133111EBull;
    h ^= h >> 31;
    return h;
}
inline double hash01(uint64_t seed, uint64_t k) { return (mix64(seed ^ (k * 0x9E3779B97F4A7C15ull)) % 100000) / 100000.0; }
inline uint64_t hash_str(const std::string &s) {
    uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned char c : s) h = (h ^ c) * 0x100000001b3ull;
    return mix64(h);
}
inline uint64_t hash_ll(double lat, double lon, uint64_t k) {
    return mix64((uint64_t)(int64_t)std::llround(lat * 2e5) * 0x9E3779B97F4A7C15ull ^ (uint64_t)(int64_t)std::llround(lon * 2e5) * 0xC2B2AE3D27D4EB4Full ^ k);
}

inline std::string lower(std::string s) {
    for (auto &c : s)
        if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a';
    return s;
}

// Lowercase for ASCII and Cyrillic (UTF-8, 2-byte forms) so Russian/Belarusian names can be matched.
inline std::string lower_utf8(const std::string &s) {
    std::string o;
    o.reserve(s.size());
    for (size_t i = 0; i < s.size(); i++) {
        unsigned char c = s[i];
        if (c >= 'A' && c <= 'Z') {
            o += char(c - 'A' + 'a');
        } else if (c == 0xD0 && i + 1 < s.size()) {
            unsigned char d = s[i + 1];
            if (d >= 0x90 && d <= 0x9F) {  // А-П -> а-п
                o += char(0xD0);
                o += char(d + 0x20);
            } else if (d >= 0xA0 && d <= 0xAF) {  // Р-Я -> р-я (D1 80..8F)
                o += char(0xD1);
                o += char(d - 0x20);
            } else if (d == 0x81) {  // Ё -> ё
                o += char(0xD1);
                o += char(0x91);
            } else if (d == 0x86) {  // І -> і
                o += char(0xD1);
                o += char(0x96);
            } else if (d == 0x8E) {  // Ў -> ў
                o += char(0xD1);
                o += char(0x9E);
            } else {
                o += char(c);
                o += char(d);
            }
            i++;
        } else {
            o += char(c);
        }
    }
    return o;
}

inline bool contains_any(const std::string &s, std::initializer_list<const char *> ws) {
    for (auto w : ws)
        if (s.find(w) != std::string::npos) return true;
    return false;
}

// Injected by the engine glue (Godot) or by tests (zlib).
using GunzipFn = std::function<bool(const std::string &in, std::string &out)>;

}  // namespace ec
