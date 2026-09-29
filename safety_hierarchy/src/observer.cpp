#include "sh/observer.hpp"

#include "sh/map.hpp"

namespace sh {

StaticMap::StaticMap(const GridSpec& spec, double r_robot)
    : known_(spec, 0), inflated_(spec, 0), offsets_(inflation_offsets(r_robot, spec.res)) {
    // The outer boundary wall is part of the environment description the
    // robot starts with (the world extent); everything else must be sensed.
    for (int j = 0; j < spec.ny; ++j) {
        for (int i = 0; i < spec.nx; ++i) {
            if (i == 0 || j == 0 || i == spec.nx - 1 || j == spec.ny - 1) {
                known_.at(i, j) = 1;
                stamp(inflated_, i, j, offsets_);
            }
        }
    }
}

bool StaticMap::integrate(const Scan& scan) {
    bool changed = false;
    for (const Cell& c : scan.static_hits) {
        if (known_.at(c.i, c.j)) continue;
        known_.at(c.i, c.j) = 1;
        stamp(inflated_, c.i, c.j, offsets_);
        changed = true;
    }
    return changed;
}

std::vector<EtPrediction> estimate_linear(const Scan& scan, double t_start, double dt, int m) {
    std::vector<EtPrediction> out;
    for (const SensedObstacle& o : scan.obstacles) {
        EtPrediction e{o.id, {}};
        e.pos.reserve(static_cast<size_t>(m + 1));
        for (int k = 0; k <= m; ++k) {
            const double tau = t_start + k * dt - scan.t;
            e.pos.push_back(o.p + o.v * tau);
        }
        out.push_back(std::move(e));
    }
    return out;
}

std::vector<EtPrediction> estimate_exact(const Scan& scan, DynamicWorld& world, double t_start, double dt, int m) {
    std::vector<EtPrediction> out;
    for (const SensedObstacle& o : scan.obstacles) {
        EtPrediction e{o.id, {}};
        e.pos.reserve(static_cast<size_t>(m + 1));
        for (int k = 0; k <= m; ++k) e.pos.push_back(world.at_time(o.id, t_start + k * dt).p);
        out.push_back(std::move(e));
    }
    return out;
}

}  // namespace sh
