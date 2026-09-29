// Observer (Sec. V-A): the static map module and the trajectory estimator.
#pragma once

#include <cstdint>
#include <vector>

#include "sh/grid.hpp"
#include "sh/obstacles.hpp"
#include "sh/sensor.hpp"

namespace sh {

// Static map module: W_s(t) = every static cell a ray has hit so far.
// Unknown cells are free (Sec. III-C).
class StaticMap {
public:
    StaticMap(const GridSpec& spec, double r_robot);
    // Adds the scan's static hits; returns true if the map changed.
    bool integrate(const Scan& scan);
    const Grid<uint8_t>& known() const { return known_; }        // raw sensed static cells
    const Grid<uint8_t>& inflated() const { return inflated_; }  // inflated by the robot radius

private:
    Grid<uint8_t> known_, inflated_;
    std::vector<Cell> offsets_;
};

// Trajectory estimator output for one sensed obstacle: its predicted centre at
// the costmap slice times t_start + k*dt, k = 0..m.
struct EtPrediction {
    int id;
    std::vector<Vec2> pos;
};

// Constant-velocity extrapolation from the sensing time (Sec. VI-A).
std::vector<EtPrediction> estimate_linear(const Scan& scan, double t_start, double dt, int m);

// Exact future (PF-ET, Sec. VI-B).
std::vector<EtPrediction> estimate_exact(const Scan& scan, DynamicWorld& world, double t_start, double dt, int m);

}  // namespace sh
