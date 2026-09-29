// Wavefront distance field used to grow the BW and SW models in space-time.
//
// A cell is in the grown set at slice k iff dist(cell) <= r_robot + v_omax * t_k,
// which is exactly the set produced by a wavefront that expands at v_omax
// (Sec. III-C), computed once instead of per slice.
//
// Each seed carries a continuous source point and an offset (e.g. the centre
// and radius of a sensed obstacle). Cells inherit the source of the neighbour
// that reached them and store the Euclidean distance to it (the propagation
// scheme of the ROS costmap_2d inflation layer), so distances are Euclidean in
// open space while the expansion itself cannot pass through blocked cells
// (footnote 4: no expansion through static obstacles).
#pragma once

#include <cstdint>
#include <vector>

#include "sh/grid.hpp"

namespace sh {

struct FieldSeed {
    int cell;       // flat index in `spec`
    Vec2 src;       // source point
    double offset;  // subtracted from the distance (obstacle radius, 0 for FOV boundary)
};

// dist[c] = |centre(c) - src| - offset for the source that reaches c first;
// +inf where unreached. Blocked cells are never entered (seed cells always are).
// Expansion stops once the distance exceeds max_dist.
void propagate_distance(const GridSpec& spec, const std::vector<uint8_t>& blocked,
                        const std::vector<FieldSeed>& seeds, double max_dist, std::vector<float>* dist);

}  // namespace sh
