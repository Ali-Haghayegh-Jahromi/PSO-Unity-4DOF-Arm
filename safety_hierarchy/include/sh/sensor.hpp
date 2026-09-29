// 360 deg line-scan range sensor (Sec. VI-A): range s_r, occluded by static and
// dynamic obstacles. One scan produces
//   * the visible (seen-free) cells  -> FOV used by the BW model,
//   * the static cells hit by rays   -> static map module W_s(t),
//   * the dynamic obstacles hit by at least one ray (all obstacles within
//     range in "perfect" mode, used by PF-ET).
// The simulation hands the observer the true position and velocity of each
// sensed obstacle (Sec. V-A: "for the simulation, this information is simply
// given").
#pragma once

#include <cstdint>
#include <vector>

#include "sh/grid.hpp"
#include "sh/obstacles.hpp"

namespace sh {

struct SensedObstacle {
    int id;
    Vec2 p;  // position at the sensing time
    Vec2 v;  // velocity at the sensing time
};

struct Scan {
    double t = 0.0;
    Pose robot;
    Grid<uint8_t> visible;  // 1 = seen free space
    std::vector<Cell> static_hits;
    std::vector<Vec2> range_endpoints;  // end points of rays that hit nothing within range
    std::vector<SensedObstacle> obstacles;
};

struct SensorConfig {
    double range = 6.0;
    int n_rays = 720;
    double r_obs = 0.25;
    bool perfect = false;
};

// `obs[i]` is the true state of obstacle i at the scan time.
void do_scan(const Grid<uint8_t>& occ_true, const Pose& robot, const std::vector<ObsState>& obs,
             const SensorConfig& cfg, double t, Scan* out);

}  // namespace sh
