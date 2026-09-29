// The one-sided unpooled z-test reproduces the PCFR p-values of Tables II-IV.
#include "sh/stats.hpp"
#include "test.hpp"

using namespace sh;

TEST(pcfr_p_values_match_paper_tables) {
    const int n = 30;
    struct Case {
        int sh, other;
        double paper;
    };
    const Case cases[] = {
        {19, 10, 0.0074},  // Table II, F-ET
        {19, 15, 0.1465},  // Table II, O-SW
        {19, 28, 0.9988},  // Table II, PF-ET
        {19, 7, 0.0003},   // Table II, O-ET
        {19, 14, 0.0941},  // Table II, ET+BW
        {20, 18, 0.2956},  // Table III, ET+SW
        {20, 12, 0.0158},  // Table III, F-ET
        {20, 6, 0.0000},   // Table III, O-BW
        {10, 9, 0.3906},   // Table IV, F-ET
        {10, 3, 0.0111},   // Table IV, O-ET
    };
    for (const Case& c : cases)
        CHECK_NEAR(proportion_p_value(c.sh / double(n), n, c.other / double(n), n), c.paper, 6e-5);
}

TEST(mean_test_direction) {
    const Summary a = summarize({0, 0, 1, 0, 1, 0});  // SH: fewer collisions
    const Summary b = summarize({2, 1, 3, 1, 2, 2});
    CHECK(mean_p_value(a, b, false) < 0.01);  // lower is better (ANCR)
    CHECK(mean_p_value(a, b, true) > 0.99);   // higher is better (AMD)
}
