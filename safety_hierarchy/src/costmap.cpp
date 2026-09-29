#include "sh/costmap.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "sh/distance_field.hpp"

namespace sh {

CostConstants cost_constants(int m, int64_t nf_min, int64_t nf_max, const Layers& enabled) {
    CostConstants c;
    int64_t below = 0;  // constant of the next enabled level below (0 if none)
    auto level = [&](bool on, int64_t* out) {
        if (!on) return;
        *out = m * (nf_max + below - nf_min) + 1;
        below = *out;
    };
    level(enabled.bw, &c.c_bw);  // (5)
    level(enabled.sw, &c.c_sw);  // (6)
    level(enabled.et, &c.c_et);  // (7)
    return c;
}

DynamicCostmap::DynamicCostmap(const Params& p)
    : m_(p.m),
      h_(p.window_half_cells()),
      w_(2 * p.window_half_cells() + 1),
      w2_(w_ * w_),
      dt_(p.dt),
      r_robot_(p.r_robot),
      r_obs_(p.r_obs),
      v_omax_(p.v_omax) {
    scm_.resize(static_cast<size_t>(w2_));
    blocked_.resize(static_cast<size_t>(w2_));
    dcm_.resize(static_cast<size_t>(m_ + 1) * w2_);
    mem_.resize(static_cast<size_t>(m_ + 1) * w2_);
}

bool DynamicCostmap::window_index(double x, double y, int* w) const {
    const Cell c = win_.cell_of(x, y);
    if (!win_.inside(c.i, c.j)) return false;
    *w = win_.index(c.i, c.j);
    return true;
}

double DynamicCostmap::growth_radius(int k) const {
    // Robot-radius inflation at the first slice, then growth at v_omax (Sec. III-C).
    return r_robot_ + v_omax_ * (growth_offset_ + k * dt_);
}

void DynamicCostmap::stamp_et(const std::vector<EtPrediction>& et) {
    // ET inflated by the robot radius: cells any part of which is within r_obs + r_robot.
    const double r = r_obs_ + r_robot_;
    const int n = static_cast<int>(std::ceil(r / win_.res)) + 1;
    for (const EtPrediction& e : et) {
        if (static_cast<int>(e.pos.size()) != m_ + 1)
            throw std::invalid_argument("ET prediction must have m + 1 positions");
        for (int k = 0; k <= m_; ++k) {
            const Vec2& q = e.pos[static_cast<size_t>(k)];
            const Cell c = win_.cell_of(q);
            for (int j = c.j - n; j <= c.j + n; ++j) {
                for (int i = c.i - n; i <= c.i + n; ++i) {
                    if (!win_.inside(i, j) || dist_to_cell(win_.center(i, j), win_.res, q) > r) continue;
                    mem_[static_cast<size_t>(k) * w2_ + win_.index(i, j)] |= kInET;
                }
            }
        }
    }
}

void DynamicCostmap::build(const CostmapInput& in) {
    if (!in.scm || !in.known_static || !in.visible || !in.sensed || !in.et)
        throw std::invalid_argument("DynamicCostmap::build: missing input");
    const GridSpec& g = in.scm->value.spec;
    layers_ = in.layers;
    growth_offset_ = in.growth_offset;
    win_ = GridSpec{g.x0 + (in.center.i - h_) * g.res, g.y0 + (in.center.j - h_) * g.res, g.res, w_, w_};

    // Window copy of the SCM and of the known static map (outside the map = obstacle).
    for (int wj = 0; wj < w_; ++wj) {
        for (int wi = 0; wi < w_; ++wi) {
            const int gi = in.center.i - h_ + wi, gj = in.center.j - h_ + wj;
            const size_t w = static_cast<size_t>(win_.index(wi, wj));
            scm_[w] = in.scm->value.get(gi, gj, kNf1Obstacle);
            blocked_[w] = in.known_static->get(gi, gj, 1);
        }
    }
    c_ = cost_constants(m_, in.scm->min_value, in.scm->max_value, layers_);

    // ---- growth fields for BW and SW ----
    const double r_max = growth_radius(m_);
    auto clamp_to_window = [&](const Vec2& p) {
        Cell c = win_.cell_of(p);
        c.i = std::min(std::max(c.i, 0), w_ - 1);
        c.j = std::min(std::max(c.j, 0), w_ - 1);
        return win_.index(c.i, c.j);
    };
    std::vector<FieldSeed> obstacle_seeds;
    for (const SensedObstacle& o : *in.sensed) obstacle_seeds.push_back({clamp_to_window(o.p), o.p, r_obs_});

    if (layers_.sw) propagate_distance(win_, blocked_, obstacle_seeds, r_max, &d_sw_);
    if (layers_.bw && in.bw_endpoint_seeds) {
        std::vector<FieldSeed> seeds = obstacle_seeds;
        for (const Vec2& q : *in.bw_endpoint_seeds) {
            int w;
            if (window_index(q.x, q.y, &w)) seeds.push_back({w, q, 0.0});
        }
        propagate_distance(win_, blocked_, seeds, r_max, &d_bw_);
    } else if (layers_.bw) {
        // FOV boundary: unseen, non-static cells 4-adjacent to a visible cell.
        std::vector<FieldSeed> seeds = obstacle_seeds;
        const int di[4] = {1, -1, 0, 0}, dj[4] = {0, 0, 1, -1};
        for (int wj = 0; wj < w_; ++wj) {
            for (int wi = 0; wi < w_; ++wi) {
                const int gi = in.center.i - h_ + wi, gj = in.center.j - h_ + wj;
                const int w = win_.index(wi, wj);
                if (blocked_[static_cast<size_t>(w)] || in.visible->get(gi, gj, 0)) continue;
                bool boundary = false;
                for (int n = 0; n < 4 && !boundary; ++n) boundary = in.visible->get(gi + di[n], gj + dj[n], 0) != 0;
                if (boundary) seeds.push_back({w, win_.center(wi, wj), 0.0});
            }
        }
        propagate_distance(win_, blocked_, seeds, r_max, &d_bw_);
    }

    // ---- Algorithm 1 ----
    std::fill(mem_.begin(), mem_.end(), 0);
    if (layers_.et) stamp_et(*in.et);
    for (int k = 0; k <= m_; ++k) {
        const float r = static_cast<float>(growth_radius(k));
        int64_t* slice = &dcm_[static_cast<size_t>(k) * w2_];
        uint8_t* bits = &mem_[static_cast<size_t>(k) * w2_];
        for (int w = 0; w < w2_; ++w) {
            int64_t v = scm_[static_cast<size_t>(w)];  // DCM(:, :, t) = SCM
            if (layers_.bw && d_bw_[static_cast<size_t>(w)] <= r) {
                v += c_.c_bw;  // (x, y, t) in BW
                bits[w] |= kInBW;
            }
            if (layers_.sw && d_sw_[static_cast<size_t>(w)] <= r) {
                v += c_.c_sw;  // (x, y, t) in SW
                bits[w] |= kInSW;
            }
            if (bits[w] & kInET) v += c_.c_et;  // (x, y, t) in ET
            slice[w] = v;
        }
    }
}

TrajectoryCost evaluate_trajectory(const DynamicCostmap& cm, const int* cells) {
    TrajectoryCost r;
    int64_t sum_dcm = 0;
    const int m = cm.m();
    for (int k = 1; k <= m; ++k) {
        const int w = cells[k - 1];
        const int32_t s = cm.scm(w);
        if (s == kNf1Obstacle || s == kNf1Unreached) {
            r.valid = false;
            ++r.static_cells;
            r.wall_cells += cm.wall(w) ? 1 : 0;
        } else {
            r.sum_nf1 += s;
        }
        const uint8_t b = cm.members(k, w);
        if (b) r.in_F = false;
        r.p_bw += (b & kInBW) ? 1 : 0;
        r.p_sw += (b & kInSW) ? 1 : 0;
        r.p_et += (b & kInET) ? 1 : 0;
        sum_dcm += cm.dcm(k, w);
    }
    r.nf1_end = cm.scm(cells[m - 1]);
    r.cost = r.in_F ? cm.dcm(m, cells[m - 1]) : sum_dcm;  // Eq. (4)
    return r;
}

ShKey sh_key(const TrajectoryCost& c) {
    // Step 1: collision-free (w.r.t. every enabled model) first, closest to the goal.
    // Steps 2-3: otherwise fewest ET cells, then fewest SW, then fewest BW cells;
    // remaining ties by the summed NF1 (the tie-break Eq. 4 itself uses).
    if (c.in_F) return {0, 0, 0, 0, c.nf1_end};
    return {1, c.p_et, c.p_sw, c.p_bw, c.sum_nf1};
}

}  // namespace sh
