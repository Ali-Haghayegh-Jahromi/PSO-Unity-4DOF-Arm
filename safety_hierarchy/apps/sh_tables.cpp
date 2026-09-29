// Summarises results.csv into Tables II-IV (PCFR, ANCR, AMD with one-sided
// p-values of "SH is better", average time to goal) and prints the paper's
// numbers next to ours, plus a check of the paper's main claims.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "results_io.hpp"
#include "sh/params.hpp"
#include "sh/stats.hpp"

using namespace sh;
using namespace sh_apps;

namespace {

struct Col {  // one algorithm in one set
    std::vector<double> cfr, ncoll, amd, time_reached;
    int n = 0, reached = 0, cycles = 0, deadlocks = 0, sh_viol = 0, fallbacks = 0, static_coll = 0;
    double plan_ms = 0.0;
};

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: sh_tables results.csv\n");
        return 2;
    }
    const std::vector<Row> rows = read_csv(argv[1]);
    const auto& algos = all_algorithms();
    std::map<int, std::map<std::string, Col>> cols;
    for (const Row& r : rows) {
        Col& c = cols[r.set][r.algo];
        const int coll = r.dyn + r.stat;
        c.cfr.push_back(r.reached && coll == 0 ? 1.0 : 0.0);
        c.ncoll.push_back(coll);
        c.amd.push_back(r.min_dist);
        if (r.reached) c.time_reached.push_back(r.time), ++c.reached;
        ++c.n;
        c.cycles += r.cycles, c.deadlocks += r.deadlocks, c.sh_viol += r.sh_viol, c.fallbacks += r.fallbacks;
        c.static_coll += r.stat;
        c.plan_ms += r.plan_ms;
    }

    const char* table_name[4] = {"", "II", "III", "IV"};
    for (auto& [set, byalgo] : cols) {
        if (!byalgo.count("SH")) continue;
        const Col& sh = byalgo["SH"];
        const Summary s_cfr = summarize(sh.cfr), s_n = summarize(sh.ncoll), s_amd = summarize(sh.amd);
        std::printf("\n## Set %d (Table %s) - %d runs per algorithm; cells: ours / paper\n\n", set,
                    table_name[set], sh.n);
        std::printf("| Measure |");
        for (const auto& a : algos) std::printf(" %s |", a.name);
        std::printf("\n|---|");
        for (size_t k = 0; k < algos.size(); ++k) std::printf("---|");
        std::printf("\n");

        auto line = [&](const char* label, auto ours, auto paper) {
            std::printf("| %s |", label);
            for (size_t k = 0; k < algos.size(); ++k) {
                if (!byalgo.count(algos[k].name)) {
                    std::printf(" - |");
                    continue;
                }
                const Col& c = byalgo[algos[k].name];
                std::printf(" %s / %s |", ours(c, algos[k].algo).c_str(), paper(kPaper[set - 1][k]).c_str());
            }
            std::printf("\n");
        };
        auto none = [](const PaperCol&) { return std::string("-"); };
        line("PCFR (%)", [&](const Col& c, Algo) { return fmt(100.0 * summarize(c.cfr).mean, "%.1f"); },
             [](const PaperCol& p) { return fmt(p.pcfr, "%.1f"); });
        line("p-value", [&](const Col& c, Algo a) {
                 if (a == Algo::SH) return std::string("N/A");
                 const Summary x = summarize(c.cfr);
                 return fmt(proportion_p_value(s_cfr.mean, s_cfr.n, x.mean, x.n), "%.4f");
             },
             [](const PaperCol& p) { return fmt(p.p_pcfr, "%.4f"); });
        line("ANCR", [&](const Col& c, Algo) { return fmt(summarize(c.ncoll).mean, "%.2f"); },
             [](const PaperCol& p) { return fmt(p.ancr, "%.2f"); });
        line("p-value", [&](const Col& c, Algo a) {
                 if (a == Algo::SH) return std::string("N/A");
                 return fmt(mean_p_value(s_n, summarize(c.ncoll), false), "%.4f");
             },
             [](const PaperCol& p) { return fmt(p.p_ancr, "%.4f"); });
        line("AMD (m)", [&](const Col& c, Algo) { return fmt(summarize(c.amd).mean, "%.2f"); },
             [](const PaperCol& p) { return fmt(p.amd, "%.2f"); });
        line("p-value", [&](const Col& c, Algo a) {
                 if (a == Algo::SH) return std::string("N/A");
                 return fmt(mean_p_value(s_amd, summarize(c.amd), true), "%.4f");
             },
             [](const PaperCol& p) { return fmt(p.p_amd, "%.4f"); });
        line("Time (s)", [&](const Col& c, Algo) { return fmt(summarize(c.time_reached).mean, "%.0f"); },
             [](const PaperCol& p) { return fmt(p.time, "%.0f"); });
        line("reached goal (%)", [&](const Col& c, Algo) { return fmt(100.0 * c.reached / c.n, "%.0f"); }, none);
        line("static collisions", [&](const Col& c, Algo) { return std::to_string(c.static_coll); }, none);
        line("deadlock fixes/run", [&](const Col& c, Algo) { return fmt(double(c.deadlocks) / c.n, "%.2f"); }, none);
        line("SH violations (%cycles)",
             [&](const Col& c, Algo) { return fmt(100.0 * c.sh_viol / std::max(1, c.cycles), "%.3f"); }, none);
        line("static fallbacks", [&](const Col& c, Algo) { return std::to_string(c.fallbacks); }, none);
        line("plan time (ms)", [&](const Col& c, Algo) { return fmt(c.plan_ms / c.n, "%.1f"); }, none);

        // ---- the paper's claims for this set ----
        std::printf("\nClaims (Sec. VI-D):\n");
        if (byalgo.count("F-ET")) {
            const Col& f = byalgo["F-ET"];
            const double d_cfr = s_cfr.mean - summarize(f.cfr).mean;
            const double d_n = summarize(f.ncoll).mean - s_n.mean;
            const double d_amd = s_amd.mean - summarize(f.amd).mean;
            std::printf("  SH better than F-ET on PCFR/ANCR/AMD: %s/%s/%s\n", d_cfr > 0 ? "yes" : "no",
                        d_n > 0 ? "yes" : "no", d_amd > 0 ? "yes" : "no");
            const double ts = summarize(sh.time_reached).mean, tf = summarize(f.time_reached).mean;
            std::printf("  SH time / F-ET time: %.2f (paper: about 2)\n", ts / tf);
        }
        int better_somewhere = 0, variants = 0;
        for (Algo a : {Algo::O_ET, Algo::O_SW, Algo::O_BW, Algo::ET_SW, Algo::ET_BW, Algo::SW_BW}) {
            const char* n = algo_spec(a).name;
            if (!byalgo.count(n)) continue;
            const Col& c = byalgo[n];
            const Summary x = summarize(c.cfr);
            const bool sig = proportion_p_value(s_cfr.mean, s_cfr.n, x.mean, x.n) < 0.05 ||
                             mean_p_value(s_n, summarize(c.ncoll), false) < 0.05 ||
                             mean_p_value(s_amd, summarize(c.amd), true) < 0.05;
            better_somewhere += sig, ++variants;
        }
        std::printf("  SH significantly better (p<0.05) than %d of %d variants on at least one measure\n",
                    better_somewhere, variants);

        // ---- per-map breakdown (not in the paper): mean time to goal / ANCR ----
        std::printf("\nPer map, mean time to goal (s) / ANCR:\n\n| map |");
        for (const auto& a : algos) std::printf(" %s |", a.name);
        std::printf("\n|---|");
        for (size_t k = 0; k < algos.size(); ++k) std::printf("---|");
        std::printf("\n");
        for (int m = 1; m <= 5; ++m) {
            std::printf("| %d |", m);
            for (const auto& a : algos) {
                double t = 0.0, n_coll = 0.0;
                int n = 0, n_reached = 0;
                for (const Row& r : rows) {
                    if (r.set != set || r.map != m || r.algo != a.name) continue;
                    ++n;
                    n_coll += r.dyn + r.stat;
                    if (r.reached) t += r.time, ++n_reached;
                }
                if (n == 0)
                    std::printf(" - |");
                else
                    std::printf(" %s / %.1f |", n_reached ? fmt(t / n_reached, "%.0f").c_str() : "-", n_coll / n);
            }
            std::printf("\n");
        }
    }
    return 0;
}
