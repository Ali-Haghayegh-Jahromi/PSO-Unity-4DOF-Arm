// Statistics for Sec. VI-D: one-sided, unpooled two-sample z-tests of
// "SH is better than X". For proportions (PCFR) this test reproduces the
// p-values printed in Tables II-IV (see tests/test_stats.cpp); the same
// large-sample test is used for the means of ANCR and AMD (assumption).
#pragma once

#include <vector>

namespace sh {

double normal_cdf(double z);

struct Summary {
    int n = 0;
    double mean = 0.0;
    double var = 0.0;  // sample variance (n - 1)
};
Summary summarize(const std::vector<double>& x);

// p-value of H1: p_sh > p_x (proportions, variance p(1-p)/n).
double proportion_p_value(double p_sh, int n_sh, double p_x, int n_x);

// p-value of H1: SH better than X for means; `higher_is_better` selects the direction.
double mean_p_value(const Summary& sh, const Summary& x, bool higher_is_better);

// 95 % (z = 1.96) Wilson score interval of a proportion p observed in n trials.
void wilson_interval(double p, int n, double z, double* lo, double* hi);

// Half-width of the normal-approximation confidence interval of a mean.
double mean_ci_half_width(const Summary& s, double z);

// Spearman rank correlation (ties get average ranks); NaN if undefined.
double spearman(const std::vector<double>& a, const std::vector<double>& b);

}  // namespace sh
