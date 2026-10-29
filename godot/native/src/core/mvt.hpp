// Mapbox Vector Tile decoder.
#pragma once
#include <string>
#include <vector>
#include "util.hpp"

namespace ec::mvt {

struct Value {
    enum Kind { None, Str, Num, Int, Bool } kind = None;
    std::string s;
    double n = 0;
    int64_t i = 0;
    bool b = false;
    bool as_num(double &out) const {
        if (kind == Num) out = n;
        else if (kind == Int) out = (double)i;
        else if (kind == Str) return parse_num(s, out);
        else return false;
        return true;
    }
    std::string as_str() const {
        switch (kind) {
            case Str: return s;
            case Num: {
                char buf[64];
                snprintf(buf, sizeof buf, "%g", n);
                return buf;
            }
            case Int: return std::to_string(i);
            case Bool: return b ? "true" : "false";
            default: return "";
        }
    }
};

enum class Geom { Unknown = 0, Point = 1, Line = 2, Polygon = 3 };

struct Feature {
    uint64_t id = 0;
    Geom kind = Geom::Unknown;
    std::vector<std::pair<std::string, Value>> props;
    // parts: MoveTo starts a new one; polygon rings keep the closing point implicit
    std::vector<std::vector<std::pair<int, int>>> parts;
    const Value *prop(const char *k) const {
        for (auto &p : props)
            if (p.first == k) return &p.second;
        return nullptr;
    }
    std::string str(const char *k) const {
        auto v = prop(k);
        return v ? v->as_str() : std::string();
    }
    bool num(const char *k, double &out) const {
        auto v = prop(k);
        return v && v->as_num(out);
    }
};

struct Layer {
    std::string name;
    int extent = 4096;
    std::vector<Feature> features;
};

bool decode(const std::string &data, std::vector<Layer> &out, std::string *err = nullptr);

// signed area of a ring in tile units (y down): outer rings are positive
double ring_area(const std::vector<std::pair<int, int>> &r);

}  // namespace ec::mvt
