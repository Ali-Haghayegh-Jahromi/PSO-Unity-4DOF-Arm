// Small shared types: 2-D vectors, poses, a platform-independent RNG.
#pragma once

#include <cmath>
#include <cstdint>
#include <random>

namespace sh {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
    double x = 0.0, y = 0.0;
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    double norm() const { return std::hypot(x, y); }
};

inline double dist(const Vec2& a, const Vec2& b) { return (a - b).norm(); }

struct Pose {
    double x = 0.0, y = 0.0, th = 0.0;
    Vec2 pos() const { return {x, y}; }
};

inline double wrap_angle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

// Exact unicycle motion for time t under constant (v, w).
inline Pose integrate_unicycle(const Pose& p, double v, double w, double t) {
    Pose q;
    if (std::fabs(w) < 1e-9) {
        q.x = p.x + v * t * std::cos(p.th);
        q.y = p.y + v * t * std::sin(p.th);
        q.th = p.th;
    } else {
        const double th1 = p.th + w * t;
        q.x = p.x + (v / w) * (std::sin(th1) - std::sin(p.th));
        q.y = p.y - (v / w) * (std::cos(th1) - std::cos(p.th));
        q.th = wrap_angle(th1);
    }
    return q;
}

// std::uniform_*_distribution is implementation-defined, so results would
// differ across compilers. This RNG only uses the raw mt19937_64 stream.
class Rng {
public:
    explicit Rng(uint64_t seed) : eng_(seed) {}
    double uniform() { return (eng_() >> 11) * (1.0 / 9007199254740992.0); }  // [0,1)
    double uniform(double a, double b) { return a + (b - a) * uniform(); }
    int uniform_int(int lo, int hi) {  // inclusive
        return lo + static_cast<int>(uniform() * (hi - lo + 1));
    }

private:
    std::mt19937_64 eng_;
};

// splitmix64-based seed combiner.
inline uint64_t mix_seed(uint64_t a, uint64_t b = 0, uint64_t c = 0, uint64_t d = 0) {
    uint64_t h = 0x9E3779B97F4A7C15ull;
    for (uint64_t v : {a, b, c, d}) {
        h ^= v + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
        h = (h ^ (h >> 30)) * 0xBF58476D1CE4E5B9ull;
        h = (h ^ (h >> 27)) * 0x94D049BB133111EBull;
        h ^= h >> 31;
    }
    return h;
}

}  // namespace sh
