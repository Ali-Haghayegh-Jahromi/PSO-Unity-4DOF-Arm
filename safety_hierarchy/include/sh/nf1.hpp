// NF1 numerical navigation function (Latombe 1991, ch. 7) = the static
// costmap SCM of Sec. III-C:
//   * static obstacles of W_s(t_c), inflated by the robot radius -> -1
//   * every free or unknown cell starts at a large positive value
//   * goal cell -> 0, then a 4-connected (L1) wavefront adds 1 per cell.
// Cells the wavefront never reaches keep the large value.
#pragma once

#include <cstdint>

#include "sh/grid.hpp"

namespace sh {

constexpr int32_t kNf1Obstacle = -1;
constexpr int32_t kNf1Unreached = 1000000000;

struct Nf1 {
    Grid<int32_t> value;
    int32_t min_value = 0;  // underline(nf1) in Eqs. 5-7 (the goal: 0)
    int32_t max_value = 0;  // overline(nf1): largest value reached by the wavefront
};

Nf1 compute_nf1(const Grid<uint8_t>& inflated_static, const Cell& goal);

}  // namespace sh
