// JBeam text: lenient JSON (comments, missing/trailing commas, odd numbers), table sections with header rows
// and modifier objects, "$variables" and "$=expressions".
#pragma once
#include <map>
#include <string>
#include <vector>
#include "../core/json.hpp"

namespace ec {

// JBeam text -> strict JSON text
std::string jbeam_normalize(const std::string &text);
bool jbeam_parse(const std::string &text, Json &out, std::string &err);

struct Row {
    std::vector<std::pair<std::string, Json>> vals;
    const Json *get(const std::string &k) const;
    bool has_num(const std::string &k) const;
    double num(const std::string &k, double d = 0) const;
    std::string str(const std::string &k, const std::string &d = "") const;
    bool flag(const std::string &k) const;
    std::vector<std::string> strings(const std::string &k) const;
};

// rows of a table section (first array = header; objects set modifiers for the rows after them)
std::vector<Row> jbeam_rows(const Json &section);
// rows of the same section across several parts, header from the first, modifiers carried over; (part, row)
std::vector<std::pair<int, Row>> jbeam_rows_merged(const std::vector<std::pair<int, const Json *>> &sections);
// replace "$name" with variable values and evaluate "$=expr"
Json jbeam_resolve(const Json &j, const std::map<std::string, Json> &vars, std::vector<std::string> &warn);

// Lua-like expression of BeamNG "$=" values: numbers, $vars, + - * / % ^, comparisons, and/or/not
bool jbeam_eval(const std::string &src, const std::map<std::string, double> &vars, Json &out, std::string &err);

inline bool jnum(const Json &j, double &v) {
    if (j.kind == Json::Num) return v = j.n, true;
    return false;
}

}  // namespace ec
