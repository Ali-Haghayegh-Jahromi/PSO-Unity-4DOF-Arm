// Exhaustive core planner (Sec. V-B): Delta_p is split into k equal control
// intervals, each takes one of p discrete (v, w) controls, giving n = p^k
// trajectories of identical duration. All are scored with Eq. (4) and the
// lowest cost wins.
#pragma once

#include <vector>

#include "sh/costmap.hpp"
#include "sh/params.hpp"

namespace sh {

struct Control {
    double v = 0.0, w = 0.0;
};

class TrajectoryLibrary {
public:
    explicit TrajectoryLibrary(const Params& p);

    int size() const { return n_; }
    int m() const { return m_; }
    double duration() const { return m_ * dt_; }
    // Control applied at time t_rel in [0, Delta_p) after the trajectory start.
    Control control_at(int traj, double t_rel) const;
    // Pose at t = k * dt (k = 0..m) for a trajectory starting at `start`.
    Pose sample(int traj, int k, const Pose& start) const;

private:
    Control control_of(int traj, int segment) const;

    std::vector<Control> controls_;
    int p_, k_, n_, m_;
    double dt_, seg_;
    std::vector<Pose> rel_;  // [traj * (m+1) + k], start at the origin heading +x
};

struct PlanResult {
    int traj = -1;
    TrajectoryCost cost;
    int n_valid = 0;               // trajectories not touching known static obstacles
    int n_free = 0;                // valid trajectories in F
    bool static_fallback = false;  // no valid trajectory: least static contact taken
    bool sh_consistent = true;     // Eq. (4) argmin == safety-hierarchy choice
};

// `all` (optional) receives the cost of every trajectory, for debugging.
PlanResult plan(const TrajectoryLibrary& lib, const DynamicCostmap& cm, const Pose& start,
                std::vector<TrajectoryCost>* all = nullptr);

}  // namespace sh
