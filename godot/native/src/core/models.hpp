// Procedural object models placed many times (trees, bushes, street lamps, parked cars). Model frame: x forward
// (the instance's yaw direction), y left, z up, origin on the ground.
#pragma once
#include "mesh.hpp"

namespace ec {

constexpr int TREE_VARIANTS = 4, CONIFER_VARIANTS = 3, BUSH_VARIANTS = 2;
// kind: InstKind (chunk.hpp); variant < *_VARIANTS for trees/conifers/bushes, else 0
MeshSet model_mesh(int kind, int variant);

}  // namespace ec
