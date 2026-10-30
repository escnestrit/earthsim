// Railways, tram tracks, level crossings, power lines and poles, fences and walls, metro signs.
#pragma once
#include <memory>
#include <vector>
#include "chunk_ctx.hpp"

namespace ec {
void build_infra(Ctx &c, const std::vector<std::shared_ptr<const RawTile>> &tiles);
}
