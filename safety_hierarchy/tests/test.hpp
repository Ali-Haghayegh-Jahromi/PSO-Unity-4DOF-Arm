// Minimal dependency-free test harness.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace test {

struct Case {
    const char* name;
    std::function<void()> fn;
};
std::vector<Case>& registry();
extern int failures;

struct Register {
    Register(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};

}  // namespace test

#define TEST(name)                                          \
    static void name();                                     \
    static test::Register reg_##name(#name, name);          \
    static void name()

#define CHECK(cond)                                                                   \
    do {                                                                              \
        if (!(cond)) {                                                                \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);             \
            ++test::failures;                                                         \
        }                                                                             \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                      \
    do {                                                                                           \
        const double va_ = (a), vb_ = (b);                                                         \
        if (!(std::fabs(va_ - vb_) <= (tol))) {                                                    \
            std::printf("  FAIL %s:%d: %s = %.6g, expected %.6g (tol %g)\n", __FILE__, __LINE__, #a, \
                        va_, vb_, static_cast<double>(tol));                                       \
            ++test::failures;                                                                      \
        }                                                                                          \
    } while (0)
