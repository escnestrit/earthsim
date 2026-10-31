// Network snapshots of a vehicle: the whole deformed body in ~0.5-1 KB.
// Node positions are sent relative to the chassis frame, quantised to 2 mm.
#pragma once
#include <cstdint>
#include <vector>
#include "vehicle.hpp"

namespace tt {

struct Snapshot {
    std::vector<uint8_t> bytes;
};

Snapshot encode(const Vehicle &v, uint32_t tick);
// apply to a vehicle built from the same description; returns the tick (0 on error)
uint32_t decode(const Snapshot &s, Vehicle &v);

}  // namespace tt
