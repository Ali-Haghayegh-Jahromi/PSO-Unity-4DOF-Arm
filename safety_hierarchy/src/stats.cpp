#include "sh/stats.hpp"

#include <cmath>

namespace sh {

double normal_cdf(double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); }

Summary summarize(const std::vector<double>& x) {
    Summary s;
    s.n = static_cast<int>(x.size());
    if (s.n == 0) return s;
    for (double v : x) s.mean += v;
    s.mean /= s.n;
    if (s.n > 1) {
        for (double v : x) s.var += (v - s.mean) * (v - s.mean);
        s.var /= (s.n - 1);
    }
    return s;
}

namespace {
// P(Z >= -diff/se) for a one-sided test where diff > 0 favours SH.
double one_sided(double diff, double se) {
    if (se <= 0.0) return diff > 0.0 ? 0.0 : (diff < 0.0 ? 1.0 : 0.5);
    return 1.0 - normal_cdf(diff / se);
}
}  // namespace

double proportion_p_value(double p_sh, int n_sh, double p_x, int n_x) {
    const double se = std::sqrt(p_sh * (1.0 - p_sh) / n_sh + p_x * (1.0 - p_x) / n_x);
    return one_sided(p_sh - p_x, se);
}

double mean_p_value(const Summary& sh, const Summary& x, bool higher_is_better) {
    const double se = std::sqrt(sh.var / sh.n + x.var / x.n);
    const double diff = higher_is_better ? sh.mean - x.mean : x.mean - sh.mean;
    return one_sided(diff, se);
}

}  // namespace sh
