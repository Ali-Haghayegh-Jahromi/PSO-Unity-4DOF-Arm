// Shared by the result-analysis apps: the paper's numbers (Tables II-IV),
// the results CSV reader and number formatting. Header-only.
#pragma once

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "sh/params.hpp"

namespace sh_apps {

// Paper values, Tables II (set 1), III (set 2), IV (set 3), algorithms in the order of all_algorithms().
struct PaperCol {
    double pcfr, p_pcfr, ancr, p_ancr, amd, p_amd, time;
};
inline const PaperCol kPaper[3][9] = {
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

inline std::vector<Row> read_csv(const std::string& path) {
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

inline std::string fmt(double v, const char* f) {
    if (std::isnan(v)) return "N/A";
    char b[32];
    std::snprintf(b, sizeof b, f, v);
    return b;
}


// Paper column of algorithm `name` in `set` (1..3), nullptr if not in the paper.
inline const PaperCol* paper_value(int set, const std::string& name) {
    const auto& algos = sh::all_algorithms();
    for (size_t k = 0; k < algos.size(); ++k)
        if (name == algos[k].name) return &kPaper[set - 1][k];
    return nullptr;
}

// Reads and concatenates several results CSVs.
inline std::vector<Row> read_csvs(const std::vector<std::string>& paths) {
    std::vector<Row> all;
    for (const auto& p : paths) {
        auto r = read_csv(p);
        all.insert(all.end(), r.begin(), r.end());
    }
    return all;
}

}  // namespace sh_apps
