#include "sh/sensor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sh {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Distance along the unit ray (o, d) to the circle (q, r); kInf if missed.
double ray_circle(const Vec2& o, const Vec2& d, const Vec2& q, double r) {
    const Vec2 oq = q - o;
    const double c = oq.x * oq.x + oq.y * oq.y - r * r;
    if (c <= 0.0) return 0.0;  // origin inside the circle
    const double b = oq.x * d.x + oq.y * d.y;
    const double disc = b * b - c;
    if (b <= 0.0 || disc < 0.0) return kInf;
    return b - std::sqrt(disc);
}

}  // namespace

void do_scan(const Grid<uint8_t>& occ_true, const Pose& robot, const std::vector<ObsState>& obs,
             const SensorConfig& cfg, double t, Scan* out) {
    const GridSpec& g = occ_true.spec;
    out->t = t;
    out->robot = robot;
    if (out->visible.spec.nx != g.nx || out->visible.spec.ny != g.ny) out->visible = Grid<uint8_t>(g, 0);
    out->visible.fill(0);
    out->static_hits.clear();
    out->range_endpoints.clear();
    out->obstacles.clear();

    const Vec2 o = robot.pos();
    // Obstacles that can possibly be hit.
    std::vector<int> near;
    for (int i = 0; i < static_cast<int>(obs.size()); ++i)
        if (dist(obs[i].p, o) - cfg.r_obs <= cfg.range) near.push_back(i);
    std::vector<uint8_t> seen(obs.size(), 0);

    const Cell c0 = g.cell_of(o);
    for (int k = 0; k < cfg.n_rays; ++k) {
        const double a = 2.0 * kPi * k / cfg.n_rays;
        const Vec2 d{std::cos(a), std::sin(a)};

        double t_dyn = kInf;
        int dyn_id = -1;
        for (int i : near) {
            const double th = ray_circle(o, d, obs[i].p, cfg.r_obs);
            if (th < t_dyn) t_dyn = th, dyn_id = i;
        }
        const double t_limit = std::min(cfg.range, t_dyn);

        // Amanatides-Woo grid traversal.
        Cell c = c0;
        const int si = d.x > 0 ? 1 : -1, sj = d.y > 0 ? 1 : -1;
        const double bx = g.x0 + (c.i + (d.x > 0 ? 1 : 0)) * g.res;
        const double by = g.y0 + (c.j + (d.y > 0 ? 1 : 0)) * g.res;
        double tmx = std::fabs(d.x) > 1e-12 ? (bx - o.x) / d.x : kInf;
        double tmy = std::fabs(d.y) > 1e-12 ? (by - o.y) / d.y : kInf;
        const double tdx = std::fabs(d.x) > 1e-12 ? g.res / std::fabs(d.x) : kInf;
        const double tdy = std::fabs(d.y) > 1e-12 ? g.res / std::fabs(d.y) : kInf;
        double t_enter = 0.0;
        bool blocked_by_static = false;
        while (t_enter < t_limit) {
            if (!g.inside(c.i, c.j)) break;
            if (occ_true.at(c.i, c.j)) {
                if (t_enter > 0.0) out->static_hits.push_back(c);
                blocked_by_static = true;
                break;
            }
            out->visible.at(c.i, c.j) = 1;
            if (tmx < tmy) {
                t_enter = tmx;
                tmx += tdx;
                c.i += si;
            } else {
                t_enter = tmy;
                tmy += tdy;
                c.j += sj;
            }
        }
        if (dyn_id >= 0 && t_dyn < cfg.range && !blocked_by_static) seen[static_cast<size_t>(dyn_id)] = 1;
        if (!blocked_by_static && t_dyn >= cfg.range) out->range_endpoints.push_back(o + d * cfg.range);
    }

    for (int i : near) {
        if (cfg.perfect || seen[static_cast<size_t>(i)]) out->obstacles.push_back({i, obs[i].p, obs[i].v});
    }
    // De-duplicate static hits (many rays hit the same cell).
    std::sort(out->static_hits.begin(), out->static_hits.end(),
              [](const Cell& a, const Cell& b) { return a.j != b.j ? a.j < b.j : a.i < b.i; });
    out->static_hits.erase(std::unique(out->static_hits.begin(), out->static_hits.end(),
                                       [](const Cell& a, const Cell& b) { return a.i == b.i && a.j == b.j; }),
                           out->static_hits.end());
}

}  // namespace sh
