// Mapbox Vector Tile (protobuf) decoder. Self-contained, no allocation tricks.
#pragma once
#include <cstdlib>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace es::mvt {

struct Value {
    enum Kind : uint8_t { None, Str, Num, Int, Bool } kind = None;
    std::string s;
    double n = 0;
    int64_t i = 0;
    bool b = false;
    double num(double def = 0) const { return kind == Num ? n : kind == Int ? double(i) : kind == Bool ? (b ? 1 : 0) : def; }
};

enum class Geom : uint8_t { Unknown = 0, Point = 1, Line = 2, Polygon = 3 };

struct IPt {
    int32_t x, y;
};

struct Feature {
    uint64_t id = 0;
    Geom kind = Geom::Unknown;
    std::vector<std::pair<std::string, Value>> props;
    std::vector<std::vector<IPt>> parts;  // MoveTo starts a part; polygon rings are open (first point not repeated)
    const Value *get(std::string_view k) const {
        for (auto &p : props)
            if (p.first == k) return &p.second;
        return nullptr;
    }
    std::string str(std::string_view k) const {
        auto v = get(k);
        return v && v->kind == Value::Str ? v->s : std::string();
    }
    double num(std::string_view k, double def = 0) const {
        auto v = get(k);
        if (!v) return def;
        if (v->kind == Value::Str) {
            char *e = nullptr;
            double d = strtod(v->s.c_str(), &e);
            return e != v->s.c_str() ? d : def;
        }
        return v->num(def);
    }
    bool flag(std::string_view k) const {
        auto v = get(k);
        if (!v) return false;
        if (v->kind == Value::Str) return v->s == "true" || v->s == "1" || v->s == "yes";
        return v->num() != 0;
    }
};

struct Layer {
    std::string name;
    int extent = 4096;
    std::vector<Feature> features;
};

bool decode(const uint8_t *data, size_t size, std::vector<Layer> &out, std::string *err = nullptr);

// Signed area of an integer ring (tile coords, y down). In MVT exterior rings have positive area.
double ring_area(const std::vector<IPt> &r);

}  // namespace es::mvt
