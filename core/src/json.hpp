// Minimal JSON reader for the nested attributes Overture stores as strings in vector tiles.
#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace es {

struct Json {
    enum Kind { Null, Bool, Num, Str, Arr, Obj } kind = Null;
    bool b = false;
    double n = 0;
    std::string s;
    std::vector<Json> a;
    std::vector<std::pair<std::string, Json>> o;

    static Json parse(const std::string &text);  // returns Null on error
    const Json &operator[](const std::string &k) const;
    const Json &operator[](size_t i) const;
    size_t size() const { return kind == Arr ? a.size() : kind == Obj ? o.size() : 0; }
    bool has(const std::string &k) const;
    double num(double def = 0) const { return kind == Num ? n : kind == Bool ? (b ? 1 : 0) : def; }
    const std::string &str() const { return s; }
    bool is_null() const { return kind == Null; }
};

}  // namespace es
