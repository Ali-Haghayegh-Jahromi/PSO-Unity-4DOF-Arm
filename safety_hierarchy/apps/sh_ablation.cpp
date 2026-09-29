// Ablation of the models of the future: every combination of BW, SW and ET
// (the paper's six variants and SH, plus NONE, which is not in the paper and
// completes the 2^3 design), compared with Tables II-IV.
//
//   sh_ablation ablation.csv [more.csv ...] [--paper-runs 6] [--summary FILE]
//
// --summary writes one CSV row per (set, combination, metric) with our value,
// its 95 % CI, our value for the paper's design and the paper's value; it is
// what tools/plot_ablation.py draws, so figure and tables share one source.
//
// For each set it prints, per combination:
//   * the paper's measures PCFR, ANCR, AMD and time to goal: ours with all
//     runs (95 % CI), ours restricted to the paper's design (runs < 6 per
//     map, n = 30) and the paper's value,
//   * reached-goal rate, collisions per minute (exposure-normalised),
//     deadlock-fix triggers, planning time,
//   * one-sided p-values "SH better than X" (the paper's test),
// then the rank agreement with the paper, the main effect of adding each
// model, and a check of the paper's ablation claims.
#include <cmath>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "results_io.hpp"
#include "sh/stats.hpp"

using namespace sh;
using namespace sh_apps;

namespace {

const char* kCombos[] = {"NONE", "O-ET", "O-SW", "O-BW", "ET+SW", "ET+BW", "SW+BW", "SH"};
const char* kPaperCombos[] = {"O-ET", "O-SW", "O-BW", "ET+SW", "ET+BW", "SW+BW", "SH"};
constexpr double kZ = 1.96;

struct Metrics {
    int n = 0, reached = 0;
    std::vector<double> cfr, ncoll, amd, time_reached;
    double total_time = 0.0, total_coll = 0.0, deadlocks = 0.0, plan_ms = 0.0;
    double pcfr() const { return 100.0 * summarize(cfr).mean; }
    double ancr() const { return summarize(ncoll).mean; }
    double amdm() const { return summarize(amd).mean; }
    double time() const { return summarize(time_reached).mean; }
};

using Table = std::map<std::string, Metrics>;  // combo -> metrics (one set)

void add(Metrics& m, const Row& r) {
    const int coll = r.dyn + r.stat;
    ++m.n;
    m.cfr.push_back(r.reached && coll == 0 ? 1.0 : 0.0);
    m.ncoll.push_back(coll);
    m.amd.push_back(r.min_dist);
    if (r.reached) m.time_reached.push_back(r.time), ++m.reached;
    m.total_time += r.time;
    m.total_coll += coll;
    m.deadlocks += r.deadlocks;
    m.plan_ms += r.plan_ms;
}

std::string pcfr_ci(const Metrics& m) {
    double lo, hi;
    wilson_interval(summarize(m.cfr).mean, m.n, kZ, &lo, &hi);
    return fmt(m.pcfr(), "%.1f") + " [" + fmt(100 * lo, "%.0f") + "-" + fmt(100 * hi, "%.0f") + "]";
}
std::string mean_ci(const std::vector<double>& v, const char* f) {
    const Summary s = summarize(v);
    return fmt(s.mean, f) + " ±" + fmt(mean_ci_half_width(s, kZ), f);
}

// p-values of "SH better than X" for PCFR, ANCR, AMD (the paper's test).
struct PVals {
    double pcfr = NAN, ancr = NAN, amd = NAN;
};
PVals p_values(const Metrics& sh, const Metrics& x) {
    const Summary c_sh = summarize(sh.cfr), c_x = summarize(x.cfr);
    return {proportion_p_value(c_sh.mean, c_sh.n, c_x.mean, c_x.n),
            mean_p_value(summarize(sh.ncoll), summarize(x.ncoll), false),
            mean_p_value(summarize(sh.amd), summarize(x.amd), true)};
}

void print_set(int set, Table& all, Table& design) {
    const char* tname[4] = {"", "II", "III", "IV"};
    const int n_all = all["SH"].n, n_design = design["SH"].n;
    std::printf("\n## Set %d (paper Table %s)\n\n", set, tname[set]);
    std::printf("Cells: **ours, n = %d** (95%% CI) · ours, paper design n = %d · *paper*. NONE is not in the paper.\n\n",
                n_all, n_design);
    std::printf("| Metric |");
    for (const char* c : kCombos) std::printf(" %s |", c);
    std::printf("\n|---|");
    for (size_t k = 0; k < std::size(kCombos); ++k) std::printf("---|");
    std::printf("\n");

    auto row = [&](const char* label, auto cell) {
        std::printf("| %s |", label);
        for (const char* c : kCombos) std::printf(" %s |", cell(std::string(c)).c_str());
        std::printf("\n");
    };
    auto three = [&](const std::string& c, const std::string& ours, const std::string& ours30, double paper,
                     const char* f) {
        const PaperCol* p = paper_value(set, c);
        return "**" + ours + "** · " + ours30 + " · *" + (p ? fmt(paper, f) : std::string("–")) + "*";
    };
    row("PCFR (%)", [&](const std::string& c) {
        const PaperCol* p = paper_value(set, c);
        return three(c, pcfr_ci(all[c]), fmt(design[c].pcfr(), "%.1f"), p ? p->pcfr : NAN, "%.1f");
    });
    row("ANCR", [&](const std::string& c) {
        const PaperCol* p = paper_value(set, c);
        return three(c, mean_ci(all[c].ncoll, "%.2f"), fmt(design[c].ancr(), "%.2f"), p ? p->ancr : NAN, "%.2f");
    });
    row("AMD (m)", [&](const std::string& c) {
        const PaperCol* p = paper_value(set, c);
        return three(c, mean_ci(all[c].amd, "%.3f"), fmt(design[c].amdm(), "%.3f"), p ? p->amd : NAN, "%.2f");
    });
    row("Time to goal (s)", [&](const std::string& c) {
        const PaperCol* p = paper_value(set, c);
        return three(c, mean_ci(all[c].time_reached, "%.0f"), fmt(design[c].time(), "%.0f"), p ? p->time : NAN,
                     "%.0f");
    });
    row("Reached goal (%)", [&](const std::string& c) { return fmt(100.0 * all[c].reached / all[c].n, "%.0f"); });
    row("Collisions / min (paper: 60·ANCR/time)", [&](const std::string& c) {
        const PaperCol* p = paper_value(set, c);
        return "**" + fmt(60.0 * all[c].total_coll / all[c].total_time, "%.2f") + "** · " +
               fmt(60.0 * design[c].total_coll / design[c].total_time, "%.2f") + " · *" +
               (p ? fmt(60.0 * p->ancr / p->time, "%.2f") : std::string("–")) + "*";
    });
    row("Deadlock fixes / run", [&](const std::string& c) { return fmt(all[c].deadlocks / all[c].n, "%.1f"); });
    row("Planning time (ms)", [&](const std::string& c) { return fmt(all[c].plan_ms / all[c].n, "%.1f"); });

    auto prow = [&](const char* label, double PVals::*field, double PaperCol::*pfield) {
        row(label, [&](const std::string& c) {
            if (c == "SH") return std::string("–");
            const double a = p_values(all["SH"], all[c]).*field;
            const double d = p_values(design["SH"], design[c]).*field;
            const PaperCol* p = paper_value(set, c);
            return fmt(a, "%.3f") + " · " + fmt(d, "%.3f") + " · *" + (p ? fmt(p->*pfield, "%.3f") : "–") + "*";
        });
    };
    prow("p: SH better, PCFR", &PVals::pcfr, &PaperCol::p_pcfr);
    prow("p: SH better, ANCR", &PVals::ancr, &PaperCol::p_ancr);
    prow("p: SH better, AMD", &PVals::amd, &PaperCol::p_amd);
}

// Metric of a combination; `paper` selects the paper's value.
double metric(Table& t, int set, const std::string& c, int which, bool paper) {
    if (paper) {
        const PaperCol* p = paper_value(set, c);
        if (!p) return NAN;
        const double v[5] = {p->pcfr, p->ancr, p->amd, p->time, 60.0 * p->ancr / p->time};
        return v[which];
    }
    const double v[5] = {t[c].pcfr(), t[c].ancr(), t[c].amdm(), t[c].time(), 60.0 * t[c].total_coll / t[c].total_time};
    return v[which];
}

// Collisions per minute: ours = total collisions / total time; paper = 60 ANCR / time.
constexpr int kNumMetrics = 5;
const char* kMetricName[kNumMetrics] = {"PCFR (pp)", "ANCR", "AMD (m)", "Time (s)", "Collisions / min"};

// One summary row per (set, combination, metric) for tools/plot_ablation.py.
void write_summary(const std::string& path, std::map<int, Table>& all, std::map<int, Table>& design) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) throw std::runtime_error("cannot write " + path);
    std::fprintf(f, "set,combo,metric,ours,lo,hi,ours_design,paper\n");
    for (auto& [set, t] : all) {
        for (const char* c : kCombos) {
            const Metrics& m = t[c];
            const Metrics& d = design[set][c];
            const PaperCol* p = paper_value(set, c);
            auto out = [&](const char* name, double v, double lo, double hi, double vd, double pv) {
                std::fprintf(f, "%d,%s,%s,%.6g,%.6g,%.6g,%.6g,%s\n", set, c, name, v, lo, hi, vd,
                             (p && !std::isnan(pv)) ? fmt(pv, "%.6g").c_str() : "");
            };
            double lo, hi;
            wilson_interval(summarize(m.cfr).mean, m.n, kZ, &lo, &hi);
            out("pcfr", m.pcfr(), 100 * lo, 100 * hi, d.pcfr(), p ? p->pcfr : NAN);
            for (int w : {1, 2, 3}) {
                const std::vector<double>& v = w == 1 ? m.ncoll : w == 2 ? m.amd : m.time_reached;
                const Summary s = summarize(v);
                const double h = mean_ci_half_width(s, kZ);
                const double vd = w == 1 ? d.ancr() : w == 2 ? d.amdm() : d.time();
                const double pv = !p ? NAN : w == 1 ? p->ancr : w == 2 ? p->amd : p->time;
                out(w == 1 ? "ancr" : w == 2 ? "amd" : "time", s.mean, s.mean - h, s.mean + h, vd, pv);
            }
            const double cpm = 60.0 * m.total_coll / m.total_time;
            out("coll_per_min", cpm, cpm, cpm, 60.0 * d.total_coll / d.total_time, p ? 60.0 * p->ancr / p->time : NAN);
            out("deadlocks_per_run", m.deadlocks / m.n, m.deadlocks / m.n, m.deadlocks / m.n, d.deadlocks / d.n, NAN);
        }
    }
    std::fclose(f);
}
const char* kMetricFmt[kNumMetrics] = {"%+.1f", "%+.2f", "%+.3f", "%+.0f", "%+.2f"};

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> files;
    std::string summary_path;
    int paper_runs = 6;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--paper-runs" && i + 1 < argc) paper_runs = std::stoi(argv[++i]);
        else if (a == "--summary" && i + 1 < argc) summary_path = argv[++i];
        else files.push_back(a);
    }
    if (files.empty()) {
        std::fprintf(stderr, "usage: sh_ablation ablation.csv [more.csv ...] [--paper-runs 6]\n");
        return 2;
    }
    std::map<int, Table> all, design;
    for (const Row& r : read_csvs(files)) {
        add(all[r.set][r.algo], r);
        if (r.run < paper_runs) add(design[r.set][r.algo], r);
    }
    for (auto& [set, t] : all)
        for (const char* c : kCombos)
            if (!t.count(c)) return std::fprintf(stderr, "set %d: no rows for %s\n", set, c), 1;

    if (!summary_path.empty()) write_summary(summary_path, all, design);
    std::printf("# Ablation of the models of the future (BW, SW, ET)\n\n");
    std::printf("All combinations use Delta_e = 0.8 s; combinations containing BW use the Sec. VII-A deadlock fix.\n");
    for (auto& [set, t] : all) print_set(set, t, design[set]);

    // ---- rank agreement with the paper (7 paper combinations) ----
    std::printf("\n## Rank agreement with the paper\n\nSpearman correlation, across the 7 paper combinations, between our "
                "values (all runs) and the paper's. +1 = same ordering.\n\n| Metric | Set 1 | Set 2 | Set 3 |\n"
                "|---|---|---|---|\n");
    for (int w = 0; w < kNumMetrics; ++w) {
        std::printf("| %s |", kMetricName[w]);
        for (auto& [set, t] : all) {
            std::vector<double> ours, paper;
            for (const char* c : kPaperCombos) ours.push_back(metric(t, set, c, w, false)), paper.push_back(metric(t, set, c, w, true));
            std::printf(" %s |", fmt(spearman(ours, paper), "%+.2f").c_str());
        }
        std::printf("\n");
    }

    // ---- main effects ----
    struct Factor {
        const char* name;
        std::vector<std::pair<const char*, const char*>> pairs;  // (without, with); last = the NONE pair
    };
    const Factor factors[] = {
        {"+BW", {{"O-ET", "ET+BW"}, {"O-SW", "SW+BW"}, {"ET+SW", "SH"}, {"NONE", "O-BW"}}},
        {"+SW", {{"O-ET", "ET+SW"}, {"O-BW", "SW+BW"}, {"ET+BW", "SH"}, {"NONE", "O-SW"}}},
        {"+ET", {{"O-SW", "ET+SW"}, {"O-BW", "ET+BW"}, {"SW+BW", "SH"}, {"NONE", "O-ET"}}},
    };
    std::printf("\n## Main effect of adding each model\n\nMean change of a metric when the model is added to a "
                "combination. Cells: **ours, the 3 pairs that exist in the paper** · ours, all 4 pairs (incl. NONE) · "
                "*paper, same 3 pairs*.\n\n| Model | Metric | Set 1 | Set 2 | Set 3 |\n|---|---|---|---|---|\n");
    for (const Factor& f : factors) {
        for (int w = 0; w < kNumMetrics; ++w) {
            std::printf("| %s | %s |", f.name, kMetricName[w]);
            for (auto& [set, t] : all) {
                double o3 = 0, o4 = 0, p3 = 0;
                for (size_t k = 0; k < f.pairs.size(); ++k) {
                    const double d = metric(t, set, f.pairs[k].second, w, false) - metric(t, set, f.pairs[k].first, w, false);
                    o4 += d / 4.0;
                    if (k < 3) {
                        o3 += d / 3.0;
                        p3 += (metric(t, set, f.pairs[k].second, w, true) - metric(t, set, f.pairs[k].first, w, true)) / 3.0;
                    }
                }
                std::printf(" **%s** · %s · *%s* |", fmt(o3, kMetricFmt[w]).c_str(), fmt(o4, kMetricFmt[w]).c_str(),
                            fmt(p3, kMetricFmt[w]).c_str());
            }
            std::printf("\n");
        }
    }

    // ---- the paper's ablation claims ----
    std::printf("\n## The paper's ablation claims\n\n| Claim | Set | Ours (all runs) | Ours (paper design) | Paper |\n"
                "|---|---|---|---|---|\n");
    for (auto& [set, t] : all) {
        Table& d = design[set];
        auto sig_count = [&](Table& tab, bool paper) {
            int k = 0;
            for (const char* c : kPaperCombos) {
                if (std::string(c) == "SH") continue;
                if (paper) {
                    const PaperCol* p = paper_value(set, c);
                    k += (p->p_pcfr < 0.05 || p->p_ancr < 0.05 || p->p_amd < 0.05);
                } else {
                    const PVals v = p_values(tab["SH"], tab[c]);
                    k += (v.pcfr < 0.05 || v.ancr < 0.05 || v.amd < 0.05);
                }
            }
            return std::to_string(k) + " of 6";
        };
        std::printf("| SH significantly better (p < 0.05) than a variant on at least one measure | %d | %s | %s | %s |\n",
                    set, sig_count(t, false).c_str(), sig_count(d, false).c_str(), sig_count(t, true).c_str());
        auto best = [&](Table& tab, int w, bool higher, bool paper) {
            std::string arg;
            double bv = 0;
            for (const char* c : kPaperCombos) {
                const double v = metric(tab, set, c, w, paper);
                if (arg.empty() || (higher ? v > bv : v < bv)) arg = c, bv = v;
            }
            return arg;
        };
        std::printf("| Highest PCFR of the 7 | %d | %s | %s | %s |\n", set, best(t, 0, true, false).c_str(),
                    best(d, 0, true, false).c_str(), best(t, 0, true, true).c_str());
        std::printf("| Lowest ANCR of the 7 | %d | %s | %s | %s |\n", set, best(t, 1, false, false).c_str(),
                    best(d, 1, false, false).c_str(), best(t, 1, false, true).c_str());
        std::printf("| Highest AMD of the 7 | %d | %s | %s | %s |\n", set, best(t, 2, true, false).c_str(),
                    best(d, 2, true, false).c_str(), best(t, 2, true, true).c_str());
        std::printf("| Lowest collisions per minute of the 7 (paper: 60 ANCR / time) | %d | %s | %s | %s |\n", set,
                    best(t, 4, false, false).c_str(), best(d, 4, false, false).c_str(), best(t, 4, false, true).c_str());
        const PVals es = p_values(t["SH"], t["ET+SW"]), ed = p_values(d["SH"], d["ET+SW"]);
        const PaperCol* pe = paper_value(set, "ET+SW");
        std::printf("| ET+SW close to SH (min p over the 3 measures) | %d | %s | %s | %s |\n", set,
                    fmt(std::fmin(es.pcfr, std::fmin(es.ancr, es.amd)), "%.3f").c_str(),
                    fmt(std::fmin(ed.pcfr, std::fmin(ed.ancr, ed.amd)), "%.3f").c_str(),
                    fmt(std::fmin(pe->p_pcfr, std::fmin(pe->p_ancr, pe->p_amd)), "%.3f").c_str());
    }
    return 0;
}
