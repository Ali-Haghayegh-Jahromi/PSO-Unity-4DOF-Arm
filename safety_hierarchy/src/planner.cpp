#include "sh/planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sh {

TrajectoryLibrary::TrajectoryLibrary(const Params& p)
    : p_(p.n_controls()), k_(p.k_ctrl), n_(p.n_trajectories()), m_(p.m), dt_(p.dt), seg_(p.delta_p() / p.k_ctrl) {
    for (double fv : p.v_levels)
        for (double fw : p.w_levels) controls_.push_back({fv * p.v_rmax, fw * p.w_rmax});

    // Relative samples: integrate each control segment exactly.
    rel_.resize(static_cast<size_t>(n_) * (m_ + 1));
    for (int tr = 0; tr < n_; ++tr) {
        for (int k = 0; k <= m_; ++k) {
            const double t = k * dt_;
            Pose q;  // origin
            double done = 0.0;
            for (int s = 0; s < k_ && done < t - 1e-12; ++s) {
                const double d = std::min(seg_, t - done);
                const Control c = control_of(tr, s);
                q = integrate_unicycle(q, c.v, c.w, d);
                done += d;
            }
            rel_[static_cast<size_t>(tr) * (m_ + 1) + k] = q;
        }
    }
}

Control TrajectoryLibrary::control_of(int traj, int segment) const {
    // traj is a base-p number; digit `segment` (least significant first) is the control id.
    int id = traj;
    for (int s = 0; s < segment; ++s) id /= p_;
    return controls_[static_cast<size_t>(id % p_)];
}

Control TrajectoryLibrary::control_at(int traj, double t_rel) const {
    int s = static_cast<int>(std::floor(t_rel / seg_ + 1e-9));
    s = std::min(std::max(s, 0), k_ - 1);
    return control_of(traj, s);
}

Pose TrajectoryLibrary::sample(int traj, int k, const Pose& start) const {
    const Pose& r = rel_[static_cast<size_t>(traj) * (m_ + 1) + k];
    const double c = std::cos(start.th), s = std::sin(start.th);
    return {start.x + c * r.x - s * r.y, start.y + s * r.x + c * r.y, wrap_angle(start.th + r.th)};
}

PlanResult plan(const TrajectoryLibrary& lib, const DynamicCostmap& cm, const Pose& start,
                std::vector<TrajectoryCost>* all) {
    const int m = lib.m();
    if (m != cm.m()) throw std::logic_error("trajectory library and costmap disagree on m");
    std::vector<int> cells(static_cast<size_t>(m));
    if (all) all->assign(static_cast<size_t>(lib.size()), TrajectoryCost{});

    PlanResult res;
    int best_valid = -1, best_any = -1, best_key = -1;
    TrajectoryCost c_valid, c_any;
    ShKey key_best{};
    for (int tr = 0; tr < lib.size(); ++tr) {
        for (int k = 1; k <= m; ++k) {
            const Pose q = lib.sample(tr, k, start);
            if (!cm.window_index(q.x, q.y, &cells[static_cast<size_t>(k - 1)]))
                throw std::logic_error("trajectory leaves the costmap window");
        }
        const TrajectoryCost c = evaluate_trajectory(cm, cells.data());
        if (all) (*all)[static_cast<size_t>(tr)] = c;

        if (c.valid) {
            ++res.n_valid;
            if (c.in_F) ++res.n_free;
            if (best_valid < 0 || c.cost < c_valid.cost) best_valid = tr, c_valid = c;
            const ShKey key = sh_key(c);
            if (best_key < 0 || key < key_best) best_key = tr, key_best = key;
        }
        // Fallback order if nothing is valid: least static contact, then Eq. (4).
        if (best_any < 0 || c.static_cells < c_any.static_cells ||
            (c.static_cells == c_any.static_cells && c.cost < c_any.cost))
            best_any = tr, c_any = c;
    }

    if (best_valid >= 0) {
        res.traj = best_valid;
        res.cost = c_valid;
        // Consistent if the Eq. (4) winner is as good as the hierarchy's best (ties allowed).
        res.sh_consistent = !(key_best < sh_key(c_valid));
    } else {
        res.traj = best_any;
        res.cost = c_any;
        res.static_fallback = true;
    }
    return res;
}

}  // namespace sh
