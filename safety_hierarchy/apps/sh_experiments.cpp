// Runs the experiment matrix of Sec. VI-D (3 sets x 5 maps x 6 runs x 9
// algorithms = 810 simulations by default) and writes one CSV row per run.
// Every algorithm sees the same scenarios (start, goal, obstacle trajectories).
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "sh/simulator.hpp"

using namespace sh;

namespace {

std::vector<int> parse_list(const std::string& s) {
    std::vector<int> v;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) v.push_back(std::stoi(tok));
    return v;
}

struct Task {
    int set, map, run;
    Algo algo;
    RunResult result;
};

}  // namespace

int main(int argc, char** argv) {
    std::string out = "results.csv", maps_dir = SH_MAPS_DIR, algos_arg = "all";
    std::vector<int> sets = {1, 2, 3}, maps = {1, 2, 3, 4, 5};
    int runs = 6, first_run = 0;
    int threads = static_cast<int>(std::thread::hardware_concurrency());
    bool bw_latency = false, bw_endpoints = false;
    double t_max = -1.0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) std::exit(2);
            return argv[++i];
        };
        if (a == "--out") out = next();
        else if (a == "--maps-dir") maps_dir = next();
        else if (a == "--sets") sets = parse_list(next());
        else if (a == "--maps") maps = parse_list(next());
        else if (a == "--runs") runs = std::stoi(next());
        else if (a == "--first-run") first_run = std::stoi(next());
        else if (a == "--algos") algos_arg = next();
        else if (a == "--threads") threads = std::stoi(next());
        else if (a == "--bw-latency") bw_latency = true;
        else if (a == "--bw-endpoints") bw_endpoints = true;
        else if (a == "--t-max") t_max = std::stod(next());
        else {
            std::fprintf(stderr,
                         "usage: sh_experiments [--out FILE] [--sets 1,2,3] [--maps 1,2,3,4,5] [--runs 6]\n"
                         "       [--first-run 0] [--algos all|SH,F-ET,...] [--threads N] [--bw-latency] [--bw-endpoints] [--t-max S]\n");
            return 2;
        }
    }
    std::vector<Algo> algos;
    if (algos_arg == "all") {
        for (const auto& s : all_algorithms()) algos.push_back(s.algo);
    } else {
        std::stringstream ss(algos_arg);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            Algo a;
            if (!parse_algo(tok, &a)) return std::fprintf(stderr, "unknown algorithm %s\n", tok.c_str()), 2;
            algos.push_back(a);
        }
    }

    // Maps are read-only and shared by all threads.
    std::vector<MapDef> map_defs(6);
    std::vector<Grid<uint8_t>> occs(6);
    for (int m : maps) {
        map_defs[static_cast<size_t>(m)] = load_map(maps_dir + "/map" + std::to_string(m) + ".txt");
        occs[static_cast<size_t>(m)] = rasterize(map_defs[static_cast<size_t>(m)], Params{}.cell);
    }

    std::vector<Task> tasks;
    for (int s : sets)
        for (int m : maps)
            for (int r = first_run; r < first_run + runs; ++r)
                for (Algo a : algos) tasks.push_back({s, m, r, a, {}});

    std::atomic<size_t> next_task{0}, done{0};
    std::mutex io;
    auto worker = [&]() {
        for (size_t k; (k = next_task.fetch_add(1)) < tasks.size();) {
            Task& t = tasks[k];
            Params p = params_for_set(t.set);
            p.bw_latency = bw_latency;
    p.bw_endpoint_seeds = bw_endpoints;
            if (t_max > 0) p.t_max = t_max;
            const auto& occ = occs[static_cast<size_t>(t.map)];
            const Scenario sc = make_scenario(occ, p, t.set, t.map, t.run);
            t.result = simulate(sc, map_defs[static_cast<size_t>(t.map)], occ, p, t.algo);
            const size_t d = ++done;
            std::lock_guard<std::mutex> lock(io);
            std::fprintf(stderr, "[%zu/%zu] set %d map %d run %d %-6s reached=%d t=%6.1f coll=%d min_d=%.3f\n", d,
                         tasks.size(), t.set, t.map, t.run, algo_spec(t.algo).name, t.result.reached, t.result.time,
                         t.result.dyn_collisions + t.result.static_collisions, t.result.min_dist);
        }
    };
    std::vector<std::thread> pool;
    for (int i = 0; i < std::max(1, threads); ++i) pool.emplace_back(worker);
    for (auto& th : pool) th.join();

    std::ofstream f(out);
    f << "set,map,run,algo,reached,time,dyn_collisions,static_collisions,min_dist,path_length,cycles,deadlocks,"
         "sh_violations,static_fallbacks,plan_ms_mean,plan_ms_max\n";
    for (const Task& t : tasks) {
        const RunResult& r = t.result;
        f << t.set << "," << t.map << "," << t.run << "," << algo_spec(t.algo).name << "," << r.reached << ","
          << r.time << "," << r.dyn_collisions << "," << r.static_collisions << "," << r.min_dist << ","
          << r.path_length << "," << r.cycles << "," << r.deadlocks << "," << r.sh_violations << ","
          << r.static_fallbacks << "," << r.plan_ms_mean << "," << r.plan_ms_max << "\n";
    }
    std::fprintf(stderr, "wrote %s (%zu runs)\n", out.c_str(), tasks.size());
    return 0;
}
