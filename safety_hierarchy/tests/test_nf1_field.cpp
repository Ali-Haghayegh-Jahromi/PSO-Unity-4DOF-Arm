// NF1 navigation function and the growth distance field.
#include <cmath>

#include "sh/distance_field.hpp"
#include "sh/nf1.hpp"
#include "test.hpp"

using namespace sh;

TEST(nf1_is_l1_distance_in_free_space) {
    const GridSpec g{0.0, 0.0, 0.1, 20, 20};
    const Grid<uint8_t> occ(g, 0);
    const Nf1 nf = compute_nf1(occ, {5, 7});
    CHECK(nf.value.at(5, 7) == 0);
    CHECK(nf.value.at(9, 2) == 4 + 5);
    CHECK(nf.min_value == 0);
    CHECK(nf.max_value == 14 + 12);  // corner (19, 19)
}

TEST(nf1_marks_obstacles_and_routes_around) {
    const GridSpec g{0.0, 0.0, 0.1, 10, 10};
    Grid<uint8_t> occ(g, 0);
    for (int j = 0; j < 9; ++j) occ.at(5, j) = 1;  // wall with a gap at j = 9
    const Nf1 nf = compute_nf1(occ, {2, 0});
    CHECK(nf.value.at(5, 0) == kNf1Obstacle);
    CHECK(nf.value.at(6, 0) == 3 + 9 + 9 + 1);  // up, across the gap, down
}

TEST(nf1_keeps_large_value_where_unreachable) {
    const GridSpec g{0.0, 0.0, 0.1, 10, 10};
    Grid<uint8_t> occ(g, 0);
    for (int k = 0; k < 10; ++k) occ.at(5, k) = 1;  // full wall
    const Nf1 nf = compute_nf1(occ, {1, 1});
    CHECK(nf.value.at(8, 8) == kNf1Unreached);
    CHECK(nf.max_value == 3 + 8);  // farthest reachable cell (4, 9)
}

// In open space the field is the exact Euclidean distance to the cell square.
TEST(distance_field_is_euclidean_in_open_space) {
    const GridSpec g{0.0, 0.0, 0.1, 100, 100};
    const std::vector<uint8_t> blocked(static_cast<size_t>(g.size()), 0);
    std::vector<float> d;
    const Vec2 src{5.0, 5.0};
    propagate_distance(g, blocked, {{g.index(50, 50), src, 0.25}}, 100.0, &d);
    double worst = 0.0;
    for (int j = 0; j < g.ny; ++j)
        for (int i = 0; i < g.nx; ++i)
            worst = std::max(worst, std::fabs(d[static_cast<size_t>(g.index(i, j))] -
                                              (dist_to_cell(g.center(i, j), g.res, src) - 0.25)));
    CHECK(worst < 1e-4);
}

TEST(distance_field_does_not_cross_walls_or_cut_corners) {
    const GridSpec g{0.0, 0.0, 0.1, 20, 20};
    std::vector<uint8_t> blocked(static_cast<size_t>(g.size()), 0);
    for (int j = 0; j < g.ny; ++j) blocked[static_cast<size_t>(g.index(10, j))] = 1;  // full wall
    std::vector<float> d;
    propagate_distance(g, blocked, {{g.index(5, 5), g.center(5, 5), 0.0}}, 100.0, &d);
    CHECK(std::isinf(d[static_cast<size_t>(g.index(15, 5))]));
    // Diagonal wall: cells (k, k) blocked; (1,0) must not leak to (0,1).
    std::fill(blocked.begin(), blocked.end(), 0);
    for (int k = 0; k < g.nx; ++k) blocked[static_cast<size_t>(g.index(k, k))] = 1;
    propagate_distance(g, blocked, {{g.index(5, 2), g.center(5, 2), 0.0}}, 100.0, &d);
    CHECK(std::isinf(d[static_cast<size_t>(g.index(2, 5))]));
}

TEST(distance_field_stops_at_max_dist) {
    const GridSpec g{0.0, 0.0, 0.1, 100, 100};
    const std::vector<uint8_t> blocked(static_cast<size_t>(g.size()), 0);
    std::vector<float> d;
    propagate_distance(g, blocked, {{g.index(0, 0), g.center(0, 0), 0.0}}, 2.0, &d);
    CHECK(d[static_cast<size_t>(g.index(15, 0))] <= 2.0);
    CHECK(std::isinf(d[static_cast<size_t>(g.index(99, 99))]));
}
