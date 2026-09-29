#include "sh/simulator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <stdexcept>

#include "sh/obstacles.hpp"

namespace sh {

Scenario make_scenario(const Grid<uint8_t>& occ, const Params& p, int set, int map, int run) {
    Scenario sc;
    sc.set = set, sc.map = map, sc.run = run;
    Rng rng(mix_seed(static_cast<uint64_t>(set), static_cast<uint64_t>(map), static_cast<uint64_t>(run), 0x5ce7a110));
    sc.obstacle_seed = mix_seed(static_cast<uint64_t>(set), static_cast<uint64_t>(map), static_cast<uint64_t>(run), 0x0b57ac1e);

    const GridSpec& g = occ.spec;
    const double xmin = g.x0, xmax = g.x0 + g.nx * g.res, ymin = g.y0, ymax = g.y0 + g.ny * g.res;
    const Grid<uint8_t> clear = inflate(occ, p.r_robot + 0.3);  // keep start/goal away from walls
    const Grid<uint8_t> reach = inflate(occ, p.r_robot);
    auto sample = [&](double xa, double xb) {
        for (int attempt = 0; attempt < 100000; ++attempt) {
            const Vec2 q{rng.uniform(xa, xb), rng.uniform(ymin + 0.5, ymax - 0.5)};
            const Cell c = g.cell_of(q);
            if (!clear.get(c.i, c.j, 1)) return q;
        }
        throw std::runtime_error("cannot sample a free start/goal");
    };
    for (int attempt = 0;; ++attempt) {
        if (attempt > 1000) throw std::runtime_error("no reachable start/goal pair");
        Vec2 a = sample(xmin + 0.5, xmin + p.start_strip);
        Vec2 b = sample(xmax - p.start_strip, xmax - 0.5);
        if (rng.uniform() < 0.5) std::swap(a, b);
        // The goal must be reachable in the true (inflated) map.
        const Nf1 nf = compute_nf1(reach, g.cell_of(b));
        const Cell ca = g.cell_of(a);
        const int32_t v = nf.value.at(ca.i, ca.j);
        if (v < 0 || v == kNf1Unreached) continue;
        sc.start = {a.x, a.y, std::atan2(b.y - a.y, b.x - a.x)};  // facing the goal [A20]
        sc.goal = b;
        return sc;
    }
}

namespace {

// Sec. VII-A: robot stayed inside a circle of radius R for N planning cycles.
bool is_deadlocked(const std::deque<Vec2>& hist, int n, double radius) {
    if (static_cast<int>(hist.size()) < n) return false;
    Vec2 c;
    for (const Vec2& q : hist) c = c + q;
    c = c * (1.0 / hist.size());
    for (const Vec2& q : hist)
        if (dist(q, c) > radius) return false;
    return true;
}

}  // namespace

RunResult simulate(const Scenario& sc, const MapDef& map, const Grid<uint8_t>& occ, const Params& p, Algo algo,
                   const SimHooks* hooks) {
    const AlgoSpec& A = algo_spec(algo);
    DynamicWorld world(occ, p, map.periodic, sc.obstacle_seed, sc.start.pos(), sc.goal);
    StaticMap smap(occ.spec, p.r_robot);
    TrajectoryLibrary lib(p);
    DynamicCostmap cm(p);
    const SensorConfig scfg{p.sensor_range, p.n_rays, p.r_obs, A.perfect};

    const int steps_per_cycle = static_cast<int>(std::lround(A.delta_e / p.sim_dt));
    const int max_steps = static_cast<int>(std::lround(p.t_max / p.sim_dt));
    const Cell goal_cell = occ.spec.cell_of(sc.goal);
    const int n_obs = world.size();

    RunResult res;
    Pose pose = sc.start;
    int active = -1, active_t0 = 0;    // trajectory index (-1 = stand still) and its start step
    int pending = -1, pending_t0 = -1;
    Scan scan;
    Nf1 nf1;
    bool nf1_valid = false;
    std::deque<Vec2> history;
    int relax_left = 0;
    double plan_ms_sum = 0.0;
    std::vector<ObsState> obs(static_cast<size_t>(n_obs));
    std::vector<uint8_t> in_contact(static_cast<size_t>(n_obs), 0);
    bool static_contact = false;

    auto control_at = [&](int traj, int t0, int step) {
        return traj < 0 ? Control{} : lib.control_at(traj, (step - t0) * p.sim_dt);
    };

    for (int step = 0;; ++step) {
        const double t = step * p.sim_dt;
        for (int i = 0; i < n_obs; ++i) obs[static_cast<size_t>(i)] = world.state(i, step);

        // ---- performance measures at time t ----
        for (int i = 0; i < n_obs; ++i) {
            const double d = dist(pose.pos(), obs[static_cast<size_t>(i)].p) - p.r_robot - p.r_obs;
            res.min_dist = std::min(res.min_dist, std::max(0.0, d));
            if (d < 0.0) {
                if (!in_contact[static_cast<size_t>(i)]) ++res.dyn_collisions;
                in_contact[static_cast<size_t>(i)] = 1;
            } else {
                in_contact[static_cast<size_t>(i)] = 0;
            }
        }
        if (hooks && hooks->on_trace && step % hooks->trace_every == 0) hooks->on_trace(t, pose, obs);
        if (dist(pose.pos(), sc.goal) <= p.goal_tol) {
            res.reached = true;
            res.time = t;
            break;
        }
        if (step >= max_steps) {
            res.time = t;
            break;
        }

        // ---- planning cycle ----
        if (step % steps_per_cycle == 0) {
            if (pending_t0 == step) active = pending, active_t0 = step;

            do_scan(occ, pose, obs, scfg, t, &scan);
            const auto t_begin = std::chrono::steady_clock::now();
            if (smap.integrate(scan) || !nf1_valid) {
                nf1 = compute_nf1(smap.inflated(), goal_cell);
                nf1_valid = true;
            }

            Layers layers = A.layers;
            if (A.layers.bw) {  // Sec. VII-A static-deadlock fix
                history.push_back(pose.pos());
                if (static_cast<int>(history.size()) > p.deadlock_cycles) history.pop_front();
                if (relax_left == 0 && is_deadlocked(history, p.deadlock_cycles, p.deadlock_radius)) {
                    relax_left = p.deadlock_relax_cycles;
                    history.clear();
                    ++res.deadlocks;
                }
                if (relax_left > 0) {
                    layers.bw = false;  // c_bw = 0
                    --relax_left;
                }
            }

            // Pose at which the next trajectory starts (end of this cycle's execution).
            Pose start = pose;
            for (int s = step; s < step + steps_per_cycle; ++s) {
                const Control u = control_at(active, active_t0, s);
                start = integrate_unicycle(start, u.v, u.w, p.sim_dt);
            }
            const double t_start = t + A.delta_e;
            const std::vector<EtPrediction> et = A.perfect ? estimate_exact(scan, world, t_start, p.dt, p.m)
                                                           : estimate_linear(scan, t_start, p.dt, p.m);
            CostmapInput in;
            in.center = occ.spec.cell_of(start.pos());
            in.scm = &nf1;
            in.known_static = &smap.known();
            in.visible = &scan.visible;
            in.sensed = &scan.obstacles;
            in.et = &et;
            in.layers = layers;
            in.growth_offset = p.bw_latency ? A.delta_e : 0.0;
            if (p.bw_endpoint_seeds) in.bw_endpoint_seeds = &scan.range_endpoints;
            cm.build(in);
            const PlanResult pr = plan(lib, cm, start);
            const double ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_begin).count();

            pending = pr.traj;
            pending_t0 = step + steps_per_cycle;
            ++res.cycles;
            plan_ms_sum += ms;
            res.plan_ms_max = std::max(res.plan_ms_max, ms);
            res.sh_violations += pr.sh_consistent ? 0 : 1;
            res.static_fallbacks += pr.static_fallback ? 1 : 0;
            if (hooks && hooks->on_cycle)
                hooks->on_cycle(CycleInfo{res.cycles - 1, t, pose, start, scan, smap, nf1, cm, lib, pr});
        }

        // ---- execute the active trajectory for one simulation step ----
        const Control u = control_at(active, active_t0, step);
        const Pose next = integrate_unicycle(pose, u.v, u.w, p.sim_dt);
        if (disk_hits(occ, next.pos(), p.r_robot)) {
            if (!static_contact) ++res.static_collisions;  // stall against the wall
            static_contact = true;
        } else {
            static_contact = false;
            res.path_length += dist(pose.pos(), next.pos());
            pose = next;
        }
    }
    res.plan_ms_mean = res.cycles ? plan_ms_sum / res.cycles : 0.0;
    return res;
}

}  // namespace sh
