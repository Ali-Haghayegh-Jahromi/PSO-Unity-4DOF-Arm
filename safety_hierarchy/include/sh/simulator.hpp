// One simulation run: interleaved planning and execution (Sec. IV, V-C),
// performance measures (Sec. VI-C) and the static-deadlock fix (Sec. VII-A).
//
// Cycle at time t_c (every Delta_e):
//   1. the trajectory planned during the previous cycle becomes active,
//   2. the robot senses (scan + static map + trajectory estimates) at t_c,
//   3. the next trajectory is planned; it starts at t_c + Delta_e from the pose
//      the active trajectory will reach then, and is scored against ET over
//      [t_c + Delta_e, t_c + Delta_e + Delta_p].
// The robot waits (zero velocity) during the first Delta_e.
#pragma once

#include <cstdint>
#include <functional>
#include <limits>

#include "sh/costmap.hpp"
#include "sh/map.hpp"
#include "sh/nf1.hpp"
#include "sh/observer.hpp"
#include "sh/params.hpp"
#include "sh/planner.hpp"
#include "sh/sensor.hpp"

namespace sh {

struct Scenario {
    int set = 1, map = 1, run = 0;
    Pose start;
    Vec2 goal;
    uint64_t obstacle_seed = 0;
};

// Random start/goal pair at opposite ends of the map [A12], and the seed of the
// obstacle trajectories. Depends only on (set, map, run).
Scenario make_scenario(const Grid<uint8_t>& occ, const Params& p, int set, int map, int run);

struct RunResult {
    bool reached = false;
    double time = 0.0;          // time to reach the goal (t_max if not reached)
    int dyn_collisions = 0;     // contact onsets with dynamic obstacles
    int static_collisions = 0;  // contact onsets with static obstacles
    double min_dist = std::numeric_limits<double>::infinity();  // surface distance, 0 if collided
    double path_length = 0.0;
    int cycles = 0;
    int deadlocks = 0;          // times the Sec. VII-A fix dropped BW
    int sh_violations = 0;      // cycles where Eq. (4) argmin != hierarchy choice
    int static_fallbacks = 0;   // cycles with no static-free trajectory
    double plan_ms_mean = 0.0;  // wall-clock NF1 + costmap + planning
    double plan_ms_max = 0.0;
};

// Everything known at the end of one planning cycle (debugging / images).
struct CycleInfo {
    int cycle;
    double t;
    const Pose& robot;
    const Pose& start;  // pose at which the new trajectory begins
    const Scan& scan;
    const StaticMap& static_map;
    const Nf1& nf1;
    const DynamicCostmap& costmap;
    const TrajectoryLibrary& library;
    const PlanResult& plan;
};

struct SimHooks {
    std::function<void(const CycleInfo&)> on_cycle;
    // Called every `trace_every` simulation steps with the true state.
    std::function<void(double t, const Pose& robot, const std::vector<ObsState>& obs)> on_trace;
    int trace_every = 10;
};

// One simulation step of the robot under control u. The robot stalls (does not
// move) if the new pose would overlap an occupied cell of `occ`; returns false
// then. Used with the true map for execution and with the robot's known map to
// predict where the next trajectory starts.
bool step_robot(const Grid<uint8_t>& occ, double r_robot, const Control& u, double dt, Pose* pose);

RunResult simulate(const Scenario& sc, const MapDef& map, const Grid<uint8_t>& occ, const Params& p, Algo algo,
                   const SimHooks* hooks = nullptr);

}  // namespace sh
