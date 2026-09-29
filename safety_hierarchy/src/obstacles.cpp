#include "sh/obstacles.hpp"

#include <cmath>
#include <stdexcept>

namespace sh {

namespace {
constexpr double kStartKeepOut = 3.0;  // no obstacle this close to the robot start at t = 0 [m]
constexpr double kGoalKeepOut = 1.0;
constexpr int kAvoidTries = 24;
constexpr double kAvoidLookahead = 0.3;  // [s] a new heading must be free this far ahead
}  // namespace

DynamicWorld::DynamicWorld(const Grid<uint8_t>& occ_true, const Params& prm,
                           const std::vector<Segment>& periodic_paths, uint64_t seed, const Vec2& start,
                           const Vec2& goal)
    : radius_(prm.r_obs),
      speed_(prm.v_omax),
      dt_(prm.sim_dt),
      segment_time_(prm.segment_time),
      alpha_max_(prm.alpha_max_deg * kPi / 180.0),
      blocked_(inflate(occ_true, prm.r_obs)),
      rng_(seed) {
    n_ = prm.n_erratic + prm.n_periodic;
    agents_.resize(static_cast<size_t>(n_));
    for (int i = 0; i < prm.n_periodic; ++i) agents_[prm.n_erratic + i].periodic = true;
    if (prm.n_periodic > static_cast<int>(periodic_paths.size()))
        throw std::runtime_error("map defines fewer periodic paths than periodic obstacles");
    place_initial(periodic_paths, start, goal);
}

bool DynamicWorld::static_blocked(const Vec2& p) const {
    const Cell c = blocked_.spec.cell_of(p);
    return blocked_.get(c.i, c.j, 1) != 0;
}

void DynamicWorld::place_initial(const std::vector<Segment>& periodic_paths, const Vec2& start, const Vec2& goal) {
    std::vector<ObsState> s0(static_cast<size_t>(n_));
    const GridSpec& g = blocked_.spec;
    int periodic_idx = 0;
    for (int i = 0; i < n_; ++i) {
        Agent& a = agents_[i];
        if (a.periodic) {
            a.path = periodic_paths[static_cast<size_t>(periodic_idx++)];
            const double len = dist(a.path.a, a.path.b);
            a.s = rng_.uniform(0.0, len);
            a.dir = rng_.uniform() < 0.5 ? 1.0 : -1.0;
            const Vec2 u = (a.path.b - a.path.a) * (1.0 / len);
            s0[i].p = a.path.a + u * a.s;
            continue;
        }
        for (int attempt = 0;; ++attempt) {
            if (attempt > 100000) throw std::runtime_error("cannot place obstacles");
            const Vec2 p{rng_.uniform(g.x0, g.x0 + g.nx * g.res), rng_.uniform(g.y0, g.y0 + g.ny * g.res)};
            if (static_blocked(p) || dist(p, start) < kStartKeepOut || dist(p, goal) < kGoalKeepOut) continue;
            bool overlap = false;
            for (int j = 0; j < i && !overlap; ++j)
                if (!agents_[j].periodic && dist(p, s0[j].p) < 2.0 * radius_ + 0.1) overlap = true;
            if (overlap) continue;
            s0[i].p = p;
            break;
        }
        a.heading = rng_.uniform(-kPi, kPi);
        a.timer = rng_.uniform(0.0, segment_time_);  // desynchronise direction changes
    }
    track_ = s0;
    steps_ = 1;
}

// Moving from `from` to `to` is blocked if it brings obstacle i closer to an
// overlapping contact with another obstacle (periodic ones are ignored by the
// check for themselves, but erratic ones avoid them).
bool DynamicWorld::obstacle_blocked(int i, const Vec2& from, const Vec2& to,
                                    const std::vector<ObsState>& next) const {
    const double min_d = 2.0 * radius_;
    for (int j = 0; j < n_; ++j) {
        if (j == i) continue;
        const Vec2& q = next[j].p;
        const double d_new = dist(to, q);
        if (d_new < min_d && d_new < dist(from, q)) return true;
    }
    return false;
}

void DynamicWorld::advance() {
    const size_t base = static_cast<size_t>(steps_ - 1) * n_;
    // `next` starts as a copy of the current step; entries are overwritten in
    // index order, so obstacle i sees the updated positions of j < i.
    std::vector<ObsState> next(track_.begin() + base, track_.begin() + base + n_);
    for (int i = 0; i < n_; ++i) {
        Agent& a = agents_[i];
        const Vec2 from = next[i].p;
        Vec2 to = from;
        if (a.periodic) {
            const double len = dist(a.path.a, a.path.b);
            a.s += a.dir * speed_ * dt_;
            if (a.s > len) {  // bounce at the ends
                a.s = 2.0 * len - a.s;
                a.dir = -1.0;
            } else if (a.s < 0.0) {
                a.s = -a.s;
                a.dir = 1.0;
            }
            const Vec2 u = (a.path.b - a.path.a) * (1.0 / len);
            to = a.path.a + u * a.s;
        } else {
            a.timer -= dt_;
            if (a.timer <= 0.0) {
                a.heading = wrap_angle(a.heading + rng_.uniform(-alpha_max_, alpha_max_));
                a.timer += segment_time_;
            }
            auto step_to = [&](double h, double t) {
                return from + Vec2{std::cos(h), std::sin(h)} * (speed_ * t);
            };
            to = step_to(a.heading, dt_);
            if (static_blocked(to) || obstacle_blocked(i, from, to, next)) {
                // Heuristic avoidance manoeuvre [A10].
                to = from;
                for (int k = 0; k < kAvoidTries; ++k) {
                    const double h = rng_.uniform(-kPi, kPi);
                    const Vec2 cand = step_to(h, dt_);
                    if (static_blocked(cand) || static_blocked(step_to(h, kAvoidLookahead)) ||
                        obstacle_blocked(i, from, cand, next))
                        continue;
                    a.heading = h;
                    a.timer = segment_time_;
                    to = cand;
                    break;
                }
            }
        }
        next[i].v = (to - from) * (1.0 / dt_);
        next[i].p = to;
    }
    track_.insert(track_.end(), next.begin(), next.end());
    ++steps_;
}

const ObsState& DynamicWorld::state(int i, int step) {
    if (step < 0) step = 0;
    while (step >= steps_) advance();
    return track_[static_cast<size_t>(step) * n_ + i];
}

const ObsState& DynamicWorld::at_time(int i, double t) {
    return state(i, static_cast<int>(std::lround(t / dt_)));
}

}  // namespace sh
