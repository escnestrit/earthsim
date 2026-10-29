#include "earth_core.h"
#include "core/models.hpp"
#include "core/textures.hpp"
#include "glue.h"

using namespace godot;

String EarthCore::version() { return "earthcore 0.2"; }

Dictionary EarthCore::facade_texture(int style, int variant) {
    ec::FacadeTex t = ec::facade_texture(style, variant);
    Dictionary d;
    d["size"] = ec::FACADE_TEX;
    d["albedo"] = bytes(t.albedo);
    d["orm"] = bytes(t.orm);
    d["emission"] = bytes(t.emission);
    d["normal"] = bytes(t.normal);
    d["depth"] = bytes(t.depth);
    return d;
}

Dictionary EarthCore::material_texture(int kind) {
    ec::MatTex t = ec::material_texture(kind);
    Dictionary d;
    d["size"] = ec::GROUND_TEX;
    d["albedo"] = bytes(t.albedo);
    d["normal"] = bytes(t.normal);
    d["orm"] = bytes(t.orm);
    return d;
}

Dictionary EarthCore::model(int kind, int variant) {
    ec::MeshSet ms = ec::model_mesh(kind, variant);
    Dictionary d;
    for (auto &[mat, m] : ms.m)
        if (m.verts() && !m.idx.empty()) d[mat] = mesh_arrays(m);
    return d;
}

void EarthCore::_bind_methods() {
    ClassDB::bind_static_method("EarthCore", D_METHOD("version"), &EarthCore::version);
    ClassDB::bind_static_method("EarthCore", D_METHOD("facade_texture", "style", "variant"), &EarthCore::facade_texture);
    ClassDB::bind_static_method("EarthCore", D_METHOD("material_texture", "kind"), &EarthCore::material_texture);
    ClassDB::bind_static_method("EarthCore", D_METHOD("model", "kind", "variant"), &EarthCore::model);
}
