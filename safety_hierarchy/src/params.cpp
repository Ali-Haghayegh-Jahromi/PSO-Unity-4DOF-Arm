#include "sh/params.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace sh {

const std::vector<AlgoSpec>& all_algorithms() {
    //                                            bw     sw     et     De    perfect
    static const std::vector<AlgoSpec> kAlgos = {
        {Algo::PF_ET, "PF-ET", {false, false, true}, 0.2, true},
        {Algo::F_ET, "F-ET", {false, false, true}, 0.2, false},
        {Algo::SH, "SH", {true, true, true}, 0.8, false},
        {Algo::O_ET, "O-ET", {false, false, true}, 0.8, false},
        {Algo::O_SW, "O-SW", {false, true, false}, 0.8, false},
        {Algo::O_BW, "O-BW", {true, false, false}, 0.8, false},
        {Algo::ET_SW, "ET+SW", {false, true, true}, 0.8, false},
        {Algo::ET_BW, "ET+BW", {true, false, true}, 0.8, false},
        {Algo::SW_BW, "SW+BW", {true, true, false}, 0.8, false},
    };
    return kAlgos;
}

const AlgoSpec& algo_spec(Algo a) {
    for (const auto& s : all_algorithms())
        if (s.algo == a) return s;
    throw std::logic_error("unknown algorithm");
}

bool parse_algo(const std::string& name, Algo* out) {
    for (const auto& s : all_algorithms()) {
        if (name == s.name) {
            *out = s.algo;
            return true;
        }
    }
    return false;
}

int Params::n_trajectories() const {
    return static_cast<int>(std::lround(std::pow(n_controls(), k_ctrl)));
}

int Params::window_half_cells() const {
    const double half = std::max(sensor_range, delta_p() * v_rmax);
    return static_cast<int>(std::ceil(half / cell - 1e-9));
}

Params params_for_set(int set) {
    Params p;
    p.set = set;
    switch (set) {
        case 1:  // highly erratic, fast robot
            p.alpha_max_deg = 120.0;
            p.n_erratic = 25;
            p.n_periodic = 0;
            break;
        case 2:  // 21 less erratic + 4 periodic, fast robot
            p.alpha_max_deg = 30.0;
            p.n_erratic = 21;
            p.n_periodic = 4;
            break;
        case 3:  // highly erratic, slow robot
            p.alpha_max_deg = 120.0;
            p.n_erratic = 25;
            p.n_periodic = 0;
            p.v_rmax = 0.5;
            p.w_rmax = 0.4;
            break;
        default:
            throw std::invalid_argument("set must be 1, 2 or 3");
    }
    return p;
}

bool check_timing(const Params& p, double delta_e, std::string* report) {
    const double dp = p.delta_p();
    const double ds = p.sensor_range / p.v_omax;  // Delta_s
    const bool eq8 = delta_e <= dp + 1e-12;
    const bool eq9 = delta_e + 1e-12 >= p.delta_co + p.delta_crp;
    const bool eq12 = dp < std::min(p.delta_o - delta_e, ds);
    if (report) {
        std::ostringstream os;
        os << "Eq.(8)  De=" << delta_e << " <= Dp=" << dp << " : " << (eq8 ? "ok" : "VIOLATED") << "\n"
           << "Eq.(9)  De=" << delta_e << " >= Dco+Dcrp=" << p.delta_co + p.delta_crp << " : "
           << (eq9 ? "ok" : "violated (nominal compute times of the paper's SH)") << "\n"
           << "Eq.(12) Dp=" << dp << " < min(Do-De=" << p.delta_o - delta_e << ", Ds=" << ds
           << ") : " << (eq12 ? "ok" : "VIOLATED") << "\n";
        *report = os.str();
    }
    return eq8 && eq12;  // Eq. 9 refers to compute times, which the simulation does not consume
}

}  // namespace sh
