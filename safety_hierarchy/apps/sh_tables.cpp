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

#include "sh/params.hpp"
#include "sh/stats.hpp"

using namespace sh;

namespace {

// Paper values, Tables II (set 1), III (set 2), IV (set 3), algorithms in the order of all_algorithms().
struct PaperCol {
    double pcfr, p_pcfr, ancr, p_ancr, amd, p_amd, time;
};
const PaperCol kPaper[3][9] = {
    {{93.3, .9988, .07, .9962, .09, .0707, 44},  {33.3, .0074, 1.10, .0090, .04, .0025, 55},
     {63.3, NAN, .47, NAN, .15, NAN, 103},       {23.3, .0003, 1.59, .0001, .05, .0070, 78},
     {50.0, .1465, 2.48, .0237, .11, .2454, 119}, {26.7, .0011, 4.14, .0000, .09, .1400, 120},
     {50.0, .1465, 1.29, .0150, .11, .2458, 88},  {46.7, .0941, 1.62, .0088, .17, .3915, 94},
     {50.0, .1465, 2.45, .0348, .14, .4095, 121}},
    {{96.7, .9994, .03, .9903, .08, .0186, 44},  {40.0, .0158, 1.07, .0055, .04, .0000, 45},
     {66.7, NAN, .37, NAN, .14, NAN, 104},       {33.3, .0031, 1.38, .0023, .04, .0001, 63},
     {50.0, .0920, 1.12, .0177, .09, .0712, 120}, {20.0, .0000, 2.50, .0076, .03, .0002, 152},
     {60.0, .2956, 0.60, .2106, .11, .1652, 130}, {43.3, .0308, 1.14, .0097, .06, .0354, 141},
     {40.0, .0158, 0.71, .1540, .11, .1978, 137}},
    {{93.3, .9999, .07, .9999, .06, .2856, 94},  {30.0, .3906, 1.63, .3161, .02, .0613, 79},
     {33.3, NAN, 1.43, NAN, .05, NAN, 182},      {10.0, .0111, 2.70, .0152, .05, .1256, 104},
     {20.0, .1188, 4.47, .0049, .06, .3241, 223}, {10.0, .0111, 7.80, .0000, .02, .1736, 242},
     {23.3, .1936, 2.86, .0167, .07, .2535, 193}, {20.0, .1188, 2.67, .0101, .06, .3354, 158},
     {20.0, .1188, 3.83, .0139, .06, .3701, 226}},
};

struct Row {
    int set, map, run;
    std::string algo;
    bool reached;
    double time, min_dist;
    int dyn, stat, cycles, deadlocks, sh_viol, fallbacks;
    double plan_ms;
};

struct Col {  // one algorithm in one set
    std::vector<double> cfr, ncoll, amd, time_reached;
    int n = 0, reached = 0, cycles = 0, deadlocks = 0, sh_viol = 0, fallbacks = 0, static_coll = 0;
    double plan_ms = 0.0;
};

std::vector<Row> read_csv(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open " + path);
    std::string line;
    std::getline(in, line);
    std::vector<std::string> head;
    {
        std::stringstream ss(line);
        std::string h;
        while (std::getline(ss, h, ',')) head.push_back(h);
    }
    std::vector<Row> rows;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::map<std::string, std::string> v;
        std::string tok;
        for (size_t k = 0; std::getline(ss, tok, ',') && k < head.size(); ++k) v[head[k]] = tok;
        Row r;
        r.set = std::stoi(v["set"]), r.map = std::stoi(v["map"]), r.run = std::stoi(v["run"]);
        r.algo = v["algo"];
        r.reached = v["reached"] == "1";
        r.time = std::stod(v["time"]), r.min_dist = std::stod(v["min_dist"]);
        r.dyn = std::stoi(v["dyn_collisions"]), r.stat = std::stoi(v["static_collisions"]);
        r.cycles = std::stoi(v["cycles"]), r.deadlocks = std::stoi(v["deadlocks"]);
        r.sh_viol = std::stoi(v["sh_violations"]), r.fallbacks = std::stoi(v["static_fallbacks"]);
        r.plan_ms = std::stod(v["plan_ms_mean"]);
        rows.push_back(r);
    }
    return rows;
}

std::string fmt(double v, const char* f) {
    if (std::isnan(v)) return "N/A";
    char b[32];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

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
