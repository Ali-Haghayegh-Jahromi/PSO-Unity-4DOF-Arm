#include "sh/stats.hpp"

#include <algorithm>
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

void wilson_interval(double p, int n, double z, double* lo, double* hi) {
    if (n <= 0) {
        *lo = 0.0, *hi = 1.0;
        return;
    }
    const double z2 = z * z, den = 1.0 + z2 / n;
    const double centre = (p + z2 / (2.0 * n)) / den;
    const double half = z * std::sqrt(p * (1.0 - p) / n + z2 / (4.0 * n * n)) / den;
    *lo = std::max(0.0, centre - half);
    *hi = std::min(1.0, centre + half);
}

double mean_ci_half_width(const Summary& s, double z) { return s.n > 1 ? z * std::sqrt(s.var / s.n) : 0.0; }

namespace {
std::vector<double> ranks(const std::vector<double>& x) {
    std::vector<size_t> idx(x.size());
    for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;
    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) { return x[a] < x[b]; });
    std::vector<double> r(x.size());
    for (size_t i = 0; i < idx.size();) {
        size_t j = i;
        while (j + 1 < idx.size() && x[idx[j + 1]] == x[idx[i]]) ++j;
        for (size_t k = i; k <= j; ++k) r[idx[k]] = 0.5 * (i + j) + 1.0;  // average rank of the tie group
        i = j + 1;
    }
    return r;
}
}  // namespace

double spearman(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size() || a.size() < 2) return std::nan("");
    const std::vector<double> ra = ranks(a), rb = ranks(b);
    const double n = static_cast<double>(a.size());
    double ma = 0, mb = 0;
    for (size_t i = 0; i < a.size(); ++i) ma += ra[i] / n, mb += rb[i] / n;
    double cov = 0, va = 0, vb = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        cov += (ra[i] - ma) * (rb[i] - mb);
        va += (ra[i] - ma) * (ra[i] - ma);
        vb += (rb[i] - mb) * (rb[i] - mb);
    }
    return (va > 0 && vb > 0) ? cov / std::sqrt(va * vb) : std::nan("");
}

}  // namespace sh
