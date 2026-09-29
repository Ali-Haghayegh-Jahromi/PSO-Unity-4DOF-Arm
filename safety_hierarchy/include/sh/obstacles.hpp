// Dynamic obstacles (Sec. VI-A).
//
// * Erratic (highly / less erratic): straight line at v_omax for 2 s, then the
//   heading changes by a random alpha in [-alpha_max, alpha_max]. On predicted
//   contact with a wall or another obstacle a heuristic avoidance manoeuvre
//   picks a new random free heading [A10].
// * Periodic: shuttles along a fixed segment [A11].
//
// Obstacles neither avoid nor chase the robot (Sec. II-A), so their motion is
// independent of the robot: the trajectories are generated deterministically
// from a seed, lazily, and are identical for every algorithm run on the same
// scenario. This also gives PF-ET its "exact" future.
#pragma once

#include <cstdint>
#include <vector>

#include "sh/common.hpp"
#include "sh/grid.hpp"
#include "sh/map.hpp"
#include "sh/params.hpp"

namespace sh {

struct ObsState {
    Vec2 p;  // centre
    Vec2 v;  // velocity over the last simulation step
};

class DynamicWorld {
public:
    // `keep_out` points (robot start, goal) get no obstacle within the given radii at t = 0.
    DynamicWorld(const Grid<uint8_t>& occ_true, const Params& prm, const std::vector<Segment>& periodic_paths,
                 uint64_t seed, const Vec2& start, const Vec2& goal);

    int size() const { return n_; }
    double radius() const { return radius_; }
    double sim_dt() const { return dt_; }
    bool is_periodic(int i) const { return agents_[i].periodic; }

    // State at simulation step `step` (time step * sim_dt). Extends the tracks on demand.
    const ObsState& state(int i, int step);
    // State at time t (rounded to the nearest simulation step).
    const ObsState& at_time(int i, double t);

private:
    struct Agent {
        bool periodic = false;
        double heading = 0.0;  // erratic
        double timer = 0.0;    // time left on the current straight segment
        Segment path;          // periodic
        double s = 0.0;        // periodic: arc position on the path
        double dir = 1.0;      // periodic: +1 towards b, -1 towards a
    };

    void place_initial(const std::vector<Segment>& periodic_paths, const Vec2& start, const Vec2& goal);
    void advance();  // appends one simulation step
    bool static_blocked(const Vec2& p) const;
    bool obstacle_blocked(int i, const Vec2& from, const Vec2& to, const std::vector<ObsState>& next) const;

    int n_ = 0;
    double radius_, speed_, dt_, segment_time_, alpha_max_;
    Grid<uint8_t> blocked_;  // static obstacles inflated by the obstacle radius
    Rng rng_;
    std::vector<Agent> agents_;
    std::vector<ObsState> track_;  // [step * n_ + i]
    int steps_ = 0;
};

}  // namespace sh
