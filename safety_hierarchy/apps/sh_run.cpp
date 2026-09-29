// Runs one scenario with one algorithm and prints what happened. Debugging aid:
//   --verbose            one line per planning cycle
//   --trace FILE         robot + obstacle positions every 0.1 s (tools/plot_run.py)
//   --dump-dir DIR       costmap images (PPM) of the window every --dump-every cycles
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "sh/image.hpp"
#include "sh/simulator.hpp"

using namespace sh;

namespace {

void usage() {
    std::fprintf(stderr,
                 "usage: sh_run [--set S] [--map M] [--run R] [--algo NAME] [--maps-dir DIR]\n"
                 "              [--verbose] [--trace FILE] [--dump-dir DIR] [--dump-every N]\n"
                 "              [--bw-latency] [--bw-endpoints] [--t-max SEC]\n"
                 "algorithms: PF-ET F-ET SH O-ET O-SW O-BW ET+SW ET+BW SW+BW\n");
}

// One image of the costmap window at slice k:
//   black = known static, grey = inflated static (NF1 -1), white/cream = unseen/visible,
//   BW light blue, SW blue, ET red, true obstacles green, chosen trajectory magenta.
void render_slice(const CycleInfo& c, int k, const std::vector<ObsState>& truth, const std::string& path) {
    const DynamicCostmap& cm = c.costmap;
    const GridSpec& win = cm.window();
    const int s = 4;  // pixels per cell
    Image img(win.nx * s, win.ny * s);
    const GridSpec& g = c.static_map.known().spec;
    const int oi = static_cast<int>(std::lround((win.x0 - g.x0) / g.res));
    const int oj = static_cast<int>(std::lround((win.y0 - g.y0) / g.res));
    for (int wj = 0; wj < win.ny; ++wj) {
        for (int wi = 0; wi < win.nx; ++wi) {
            const int w = win.index(wi, wj);
            Rgb col{255, 255, 255};
            if (c.scan.visible.get(oi + wi, oj + wj, 0)) col = {255, 250, 225};
            const uint8_t b = cm.members(k, w);
            if (b & kInBW) col = {185, 215, 245};
            if (b & kInSW) col = {90, 140, 225};
            if (b & kInET) col = {235, 70, 60};
            if (cm.scm(w) == kNf1Obstacle) col = {150, 150, 150};
            if (c.static_map.known().get(oi + wi, oj + wj, 1)) col = {0, 0, 0};
            for (int y = 0; y < s; ++y)
                for (int x = 0; x < s; ++x) img.set(wi * s + x, wj * s + y, col);
        }
    }
    auto px = [&](const Vec2& p) { return Vec2{(p.x - win.x0) / win.res * s, (p.y - win.y0) / win.res * s}; };
    for (const ObsState& o : truth) {
        const Vec2 q = px(o.p);
        img.disk(q.x, q.y, 0.25 / win.res * s, {40, 170, 60});
    }
    for (int kk = 0; kk <= cm.m(); ++kk) {
        const Vec2 q = px(c.library.sample(c.plan.traj, kk, c.start).pos());
        img.disk(q.x, q.y, kk == k ? 4.0 : 1.5, {200, 0, 200});
    }
    const Vec2 r = px(c.robot.pos());
    img.disk(r.x, r.y, 3.0, {0, 0, 0});
    img.write_ppm(path);
}

}  // namespace

int main(int argc, char** argv) {
    int set = 1, map_id = 1, run = 0, dump_every = 0;
    std::string algo_name = "SH", maps_dir = SH_MAPS_DIR, trace_path, dump_dir;
    bool verbose = false, bw_latency = false, bw_endpoints = false;
    double t_max = -1.0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) usage(), std::exit(2);
            return argv[++i];
        };
        if (a == "--set") set = std::stoi(next());
        else if (a == "--map") map_id = std::stoi(next());
        else if (a == "--run") run = std::stoi(next());
        else if (a == "--algo") algo_name = next();
        else if (a == "--maps-dir") maps_dir = next();
        else if (a == "--verbose") verbose = true;
        else if (a == "--trace") trace_path = next();
        else if (a == "--dump-dir") dump_dir = next();
        else if (a == "--dump-every") dump_every = std::stoi(next());
        else if (a == "--bw-latency") bw_latency = true;
        else if (a == "--bw-endpoints") bw_endpoints = true;
        else if (a == "--t-max") t_max = std::stod(next());
        else return usage(), 2;
    }
    Algo algo;
    if (!parse_algo(algo_name, &algo)) return usage(), 2;

    Params p = params_for_set(set);
    p.bw_latency = bw_latency;
    p.bw_endpoint_seeds = bw_endpoints;
    if (t_max > 0) p.t_max = t_max;
    const MapDef map = load_map(maps_dir + "/map" + std::to_string(map_id) + ".txt");
    const Grid<uint8_t> occ = rasterize(map, p.cell);
    const Scenario sc = make_scenario(occ, p, set, map_id, run);

    std::string timing;
    check_timing(p, algo_spec(algo).delta_e, &timing);
    std::printf("set %d map %d run %d algo %s  start (%.2f, %.2f) goal (%.2f, %.2f)  n=%d trajectories\n%s", set,
                map_id, run, algo_name.c_str(), sc.start.x, sc.start.y, sc.goal.x, sc.goal.y, p.n_trajectories(),
                timing.c_str());

    std::ofstream trace;
    if (!trace_path.empty()) {
        trace.open(trace_path);
        trace << "t,rx,ry,rth";
        for (int i = 0; i < p.n_erratic + p.n_periodic; ++i) trace << ",o" << i << "x,o" << i << "y";
        trace << "\n";
    }
    std::vector<ObsState> truth;
    SimHooks hooks;
    hooks.trace_every = 1;
    hooks.on_trace = [&](double t, const Pose& r, const std::vector<ObsState>& obs) {
        truth = obs;
        if (!trace.is_open() || std::lround(t / p.sim_dt) % 10 != 0) return;
        trace << t << "," << r.x << "," << r.y << "," << r.th;
        for (const ObsState& o : obs) trace << "," << o.p.x << "," << o.p.y;
        trace << "\n";
    };
    hooks.on_cycle = [&](const CycleInfo& c) {
        const TrajectoryCost& b = c.plan.cost;
        if (verbose)
            std::printf("cycle %4d t=%6.1f pos (%6.2f,%6.2f) sensed %2zu valid %4d free %4d | best: F=%d "
                        "pET=%2d pSW=%2d pBW=%2d nf1_end=%4d cost=%lld%s%s\n",
                        c.cycle, c.t, c.robot.x, c.robot.y, c.scan.obstacles.size(), c.plan.n_valid,
                        c.plan.n_free, b.in_F, b.p_et, b.p_sw, b.p_bw, b.nf1_end, static_cast<long long>(b.cost),
                        c.plan.sh_consistent ? "" : "  [SH VIOLATION]",
                        c.plan.static_fallback ? "  [STATIC FALLBACK]" : "");
        if (!dump_dir.empty() && dump_every > 0 && c.cycle % dump_every == 0) {
            for (int k : {0, c.costmap.m() / 2, c.costmap.m()}) {
                char name[512];
                std::snprintf(name, sizeof name, "%s/cycle%04d_k%02d.ppm", dump_dir.c_str(), c.cycle, k);
                render_slice(c, k, truth, name);
            }
        }
    };

    const RunResult r = simulate(sc, map, occ, p, algo, &hooks);
    std::printf("reached=%d time=%.1f s  collisions dyn=%d static=%d  min_dist=%.3f m  path=%.1f m\n"
                "cycles=%d deadlock_fixes=%d sh_violations=%d static_fallbacks=%d  plan %.2f ms (max %.2f)\n",
                r.reached, r.time, r.dyn_collisions, r.static_collisions, r.min_dist, r.path_length, r.cycles,
                r.deadlocks, r.sh_violations, r.static_fallbacks, r.plan_ms_mean, r.plan_ms_max);
    return 0;
}
