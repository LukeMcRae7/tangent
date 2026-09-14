// Tangent - welding coincident points into shared vertices.
//
// Triangles that arrive with their own copies of every corner -- an STL, or an
// exact body tessellated a face at a time -- share no vertices, so nothing is
// adjacent to anything and no surface is closed. Welding puts the sharing back:
// every point within `tolerance` of one already seen becomes that one.
//
// The grid is the tolerance wide, and a point is compared against its own cell
// and the twenty-six around it. Only its own cell is not enough: two points a
// hair apart on either side of a cell wall round to different cells, and would
// stay two vertices however close they were.
#pragma once

#include "core/math.h"

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace tg {

class Welder {
public:
    explicit Welder(Real tolerance) : tol_(tolerance), inv_(Real(1) / tolerance) {}

    // The index of the vertex `p` welds to, adding it if there is none.
    uint32_t add(Vec3 p) {
        const int64_t cx = cellOf(p.x), cy = cellOf(p.y), cz = cellOf(p.z);
        const Real tol2 = tol_ * tol_;
        for (int64_t dx = -1; dx <= 1; ++dx)
            for (int64_t dy = -1; dy <= 1; ++dy)
                for (int64_t dz = -1; dz <= 1; ++dz) {
                    const auto it = grid_.find(key(cx + dx, cy + dy, cz + dz));
                    if (it == grid_.end()) continue;
                    for (uint32_t i : it->second)
                        if (lengthSq(positions[i] - p) <= tol2) return i;
                }
        const auto index = static_cast<uint32_t>(positions.size());
        positions.push_back(p);
        grid_[key(cx, cy, cz)].push_back(index);
        return index;
    }

    void reserve(size_t n) {
        positions.reserve(n);
        grid_.reserve(n);
    }

    std::vector<Vec3> positions;

private:
    int64_t cellOf(Real v) const { return static_cast<int64_t>(std::floor(v * inv_)); }
    static uint64_t key(int64_t x, int64_t y, int64_t z) {
        return static_cast<uint64_t>(x) * 0x9E3779B97F4A7C15ull ^
               static_cast<uint64_t>(y) * 0xC2B2AE3D27D4EB4Full ^
               static_cast<uint64_t>(z) * 0x165667B19E3779F9ull;
    }

    Real tol_;
    Real inv_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> grid_;
};

} // namespace tg
