// Procedural facade textures. One texture covers 4 x 4 bays of 3 m (12 m square); walls carry u = metres along
// the footprint / 12 and v = -(height scaled to 3 m storeys) / 12, so the texture repeats every 12 m.
// These are generic styles chosen from map attributes, NOT the real facade of a building.
#pragma once
#include <cstdint>
#include <vector>

namespace ec {

constexpr int FACADE_TEX = 512;

struct FacadeTex {
    std::vector<uint8_t> albedo;    // RGBA8 sRGB
    std::vector<uint8_t> orm;       // RGBA8: R occlusion, G roughness, B metallic
    std::vector<uint8_t> emission;  // RGBA8 lit windows (night)
    std::vector<uint8_t> normal;    // RGBA8 tangent-space normal map (OpenGL convention, +y up)
    std::vector<uint8_t> depth;     // R8: 0 proud .. 255 deepest (glass)
};

// rows from the top (image order); v = 0 at the top of the image
FacadeTex facade_texture(int style, int variant);

}  // namespace ec

namespace ec {

constexpr int GROUND_TEX = 512;

// Tileable material textures (GROUND_TEX square): albedo RGBA8 (sRGB), normal RGBA8, orm RGBA8 (R occlusion,
// G roughness, B metallic). Kinds:
enum TexKind {
    T_GRASS = 0, T_FOREST_FLOOR, T_SAND, T_ASPHALT, T_PAVERS, T_FARM, T_DIRT, T_PITCH,  // ground-cover layers 0-7
    T_CONCRETE = 8, T_CURB, T_ROOF_TILE, T_ROOF_METAL, T_ROOF_FLAT, T_BARK, T_LEAVES, T_CONIFER, T_GRAVEL, T_ROCK, T_GRASS_BLADES,
    T_COUNT
};
struct MatTex {
    std::vector<uint8_t> albedo, normal, orm;
};
MatTex material_texture(int kind);

}  // namespace ec
