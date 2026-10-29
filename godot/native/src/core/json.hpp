// Minimal JSON reader (Overpass answers, region index). Numbers as double, objects keep key order.
#pragma once
#include <memory>
#include <string>
#include <vector>

namespace ec {

struct Json {
    enum Kind { Null, Bool, Num, Str, Arr, Obj } kind = Null;
    bool b = false;
    double n = 0;
    std::string s;
    std::vector<Json> a;
    std::vector<std::pair<std::string, Json>> o;

    const Json *get(const char *k) const {
        if (kind != Obj) return nullptr;
        for (auto &p : o)
            if (p.first == k) return &p.second;
        return nullptr;
    }
    double num(double def = 0) const { return kind == Num ? n : def; }
    const std::string *str() const { return kind == Str ? &s : nullptr; }

    static bool parse(const std::string &text, Json &out, std::string *err = nullptr);
};

}  // namespace ec
