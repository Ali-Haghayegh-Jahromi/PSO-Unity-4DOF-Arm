// All parameters (Table I + the per-set values of Sec. VI-D) and the nine
// algorithms of Sec. VI-B. Values not given by the paper are marked "[A#]",
// referring to the assumption table in docs/DESIGN.md.
#pragma once

#include <string>
#include <vector>

namespace sh {

// Which models of the future are active in the costmap.
struct Layers {
    bool bw = false, sw = false, et = false;
};

enum class Algo { PF_ET, F_ET, SH, O_ET, O_SW, O_BW, ET_SW, ET_BW, SW_BW };

struct AlgoSpec {
    Algo algo;
    const char* name;
    Layers layers;
    double delta_e;  // execution horizon [s]
    bool perfect;    // PF-ET: sees occluded obstacles + exact future
};

const std::vector<AlgoSpec>& all_algorithms();  // in the paper's order 1..9
const AlgoSpec& algo_spec(Algo a);
bool parse_algo(const std::string& name, Algo* out);

struct Params {
    int set = 1;

    // ---- Table I: planner ----
    double delta_co = 0.10;   // observer computation time [s]
    double delta_crp = 0.60;  // planner computation time [s]
    double delta_o = 6.0;     // duration of estimated obstacle trajectories [s]
    int m = 30;               // number of time steps of a planned trajectory
    double dt = 0.1;          // duration of a time step [s]; Delta_p = m * dt = 3 s
    double delta_p() const { return m * dt; }

    // ---- Table I: robot ----
    double v_rmax = 1.0;        // [m/s]   (0.5 in set 3)
    double w_rmax = 0.8;        // [rad/s] (0.4 in set 3)
    double sensor_range = 6.0;  // s_r [m]

    // ---- Table I: obstacles ----
    double v_omax = 0.75;          // [m/s]
    double alpha_max_deg = 120.0;  // direction change range (30 for less erratic)
    double segment_time = 2.0;     // straight-line duration between direction changes [s]

    // ---- Sec. VI-D: obstacle population per set ----
    int n_erratic = 25;   // [A8]
    int n_periodic = 0;   // 4 in set 2

    // ---- not in the paper ----
    double cell = 0.1;        // dr [A4]
    double r_robot = 0.25;    // [A5]
    double r_obs = 0.25;      // [A5]
    std::vector<double> v_levels = {0.0, 0.25, 0.5, 0.75, 1.0};         // x v_rmax [A7]
    std::vector<double> w_levels = {-1.0, -0.75, -0.5, -0.25, 0.0,
                                    0.25, 0.5, 0.75, 1.0};               // x w_rmax [A7]
    int k_ctrl = 2;           // control changes per trajectory [A7]
    int n_rays = 720;         // laser rays over 360 deg [A15]
    double goal_tol = 0.3;    // [A13]
    double t_max = 600.0;     // [A14]
    double sim_dt = 0.01;     // [A21]
    double start_strip = 3.0; // width of the start / goal strips at the map ends [A12]

    // ---- Sec. VII-A deadlock fix ----
    int deadlock_cycles = 10;
    double deadlock_radius = 1.0;
    int deadlock_relax_cycles = 5;

    // ---- options ----
    bool bw_latency = false;  // grow BW/SW from the sensing time [A18]
    // BW seeds: false = FOV boundary incl. occlusion shadow edges (Figs. 1-2, default);
    // true = only the scan end points at maximum range (hypothesis H1, see DESIGN.md).
    bool bw_endpoint_seeds = false;

    int n_controls() const { return static_cast<int>(v_levels.size() * w_levels.size()); }
    int n_trajectories() const;
    // Half-size (cells) of the costmap window: max(s_r, Delta_p * v_rmax) / dr.
    int window_half_cells() const;
};

// Parameters of simulation set 1, 2 or 3 (Sec. VI-D-1..3).
Params params_for_set(int set);

// Eqs. (8), (9), (12) for a given Delta_e. Returns true if all hold;
// `report` receives a human-readable line per condition.
bool check_timing(const Params& p, double delta_e, std::string* report);

}  // namespace sh
