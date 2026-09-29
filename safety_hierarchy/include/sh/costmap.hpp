// Safety-hierarchy costmap (Sec. III-C): the 3-D dynamic costmap DCM(x, y, t)
// of Algorithm 1, the cost constants of Eqs. (5)-(7), and the trajectory cost
// of Eq. (4).
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "sh/grid.hpp"
#include "sh/nf1.hpp"
#include "sh/observer.hpp"
#include "sh/params.hpp"
#include "sh/sensor.hpp"

namespace sh {

// ---------------------------------------------------------------- Eqs. (5)-(7)
struct CostConstants {
    int64_t c_bw = 0, c_sw = 0, c_et = 0;
};

// c_bw = m (nf_max - nf_min) + 1                        (5)
// c_sw = m (nf_max + c_bw - nf_min) + 1                 (6)
// c_et = m (nf_max + c_sw - nf_min) + 1                 (7)
// A disabled level gets 0 and is skipped by the recursion, so each enabled
// level dominates everything below it (for the full SH: exactly (5)-(7)).
CostConstants cost_constants(int m, int64_t nf_min, int64_t nf_max, const Layers& enabled);

// ---------------------------------------------------------------- Algorithm 1
enum : uint8_t { kInBW = 1, kInSW = 2, kInET = 4 };

struct CostmapInput {
    Cell center;                                  // global cell of the trajectory start
    const Nf1* scm = nullptr;                     // NF1 on W_s(t_c)
    const Grid<uint8_t>* known_static = nullptr;  // blocks BW/SW growth (footnote 4)
    const Grid<uint8_t>* visible = nullptr;       // FOV of the scan (BW seeds)
    const std::vector<SensedObstacle>* sensed = nullptr;  // at the sensing time (BW/SW seeds)
    const std::vector<EtPrediction>* et = nullptr;        // ET at the slice times
    // Hypothesis H1 only: seed BW at these points instead of the FOV boundary.
    const std::vector<Vec2>* bw_endpoint_seeds = nullptr;
    Layers layers;
    double growth_offset = 0.0;  // BW/SW growth before slice 0 [s]; 0 = Alg. 1 literally [A18]
};

class DynamicCostmap {
public:
    explicit DynamicCostmap(const Params& p);

    void build(const CostmapInput& in);

    int m() const { return m_; }
    int width() const { return w_; }
    const GridSpec& window() const { return win_; }
    const CostConstants& constants() const { return c_; }
    const Layers& layers() const { return layers_; }

    // World point -> flat window index; false if outside the window.
    bool window_index(double x, double y, int* w) const;

    int64_t dcm(int k, int w) const { return dcm_[static_cast<size_t>(k) * w2_ + w]; }
    uint8_t members(int k, int w) const { return mem_[static_cast<size_t>(k) * w2_ + w]; }
    int32_t scm(int w) const { return scm_[static_cast<size_t>(w)]; }
    // Radius (from the seeds) of BW/SW at slice k.
    double growth_radius(int k) const;

    // Debug access to the growth fields.
    const std::vector<float>& bw_field() const { return d_bw_; }
    const std::vector<float>& sw_field() const { return d_sw_; }

private:
    void stamp_et(const std::vector<EtPrediction>& et);

    int m_, h_, w_, w2_;
    double dt_, r_robot_, r_obs_, v_omax_, growth_offset_ = 0.0;
    GridSpec win_;
    Layers layers_;
    CostConstants c_;
    std::vector<int32_t> scm_;   // [w]
    std::vector<uint8_t> blocked_;  // [w] known static or outside the map
    std::vector<float> d_bw_, d_sw_;
    std::vector<int64_t> dcm_;   // [k * w2 + w], k = 0..m
    std::vector<uint8_t> mem_;   // [k * w2 + w] model membership bits
};

// ---------------------------------------------------------------- Eq. (4)
struct TrajectoryCost {
    bool valid = true;     // touches no (inflated) known static cell [A17]
    int static_cells = 0;
    bool in_F = true;      // touches none of the enabled models
    int p_bw = 0, p_sw = 0, p_et = 0;  // number of (x,y,t) cells in each model
    int64_t sum_nf1 = 0;   // sum of SCM over k = 1..m
    int32_t nf1_end = 0;   // SCM at k = m
    int64_t cost = 0;      // Eq. (4)
};

// `cells[k-1]` = window index of the sample at t_c + k*dt, k = 1..m.
//   c(tr) = DCM(tr(m dt))                    if tr in F
//         = sum_{k=1..m} DCM(tr(k dt))       otherwise
TrajectoryCost evaluate_trajectory(const DynamicCostmap& cm, const int* cells);

// Direct statement of the safety hierarchy (Sec. III-B, steps 1-3) as a
// lexicographic key; smaller is better. Used to check that the Eq. (4)
// argmin selects the trajectory the hierarchy prescribes.
using ShKey = std::array<int64_t, 5>;
ShKey sh_key(const TrajectoryCost& c);

}  // namespace sh
