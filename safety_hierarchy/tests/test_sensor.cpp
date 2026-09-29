// Sensor, static map module, and dynamic obstacle motion.
#include <cmath>

#include "sh/map.hpp"
#include "sh/obstacles.hpp"
#include "sh/observer.hpp"
#include "sh/sensor.hpp"
#include "test.hpp"

using namespace sh;

namespace {
MapDef open_map() {
    MapDef m;
    m.xmin = -10, m.ymin = -10, m.xmax = 10, m.ymax = 10;
    return m;
}
bool visible_at(const Scan& s, double x, double y) {
    const Cell c = s.visible.spec.cell_of(x, y);
    return s.visible.get(c.i, c.j, 0) != 0;
}
}  // namespace

TEST(sensor_sees_a_disk_of_radius_sr) {
    const Grid<uint8_t> occ = rasterize(open_map(), 0.1);
    Scan s;
    do_scan(occ, {0.05, 0.05, 0.0}, {}, SensorConfig{}, 0.0, &s);
    CHECK(visible_at(s, 5.8, 0.05));
    CHECK(visible_at(s, -4.0, 4.0));  // 5.66 m
    CHECK(!visible_at(s, 6.3, 0.05));
    CHECK(!visible_at(s, 4.5, 4.5));  // 6.36 m
}

TEST(sensor_occlusion_by_static_and_dynamic_obstacles) {
    MapDef m = open_map();
    m.rects.push_back({2.0, -1.0, 2.2, 1.0});  // wall right of the robot
    const Grid<uint8_t> occ = rasterize(m, 0.1);
    std::vector<ObsState> obs = {{{0.0, 3.0}, {0.0, 0.0}},   // in the open, above
                                 {{4.0, 0.0}, {0.0, 0.0}},   // behind the wall
                                 {{0.0, 5.0}, {0.0, 0.0}}};  // behind obstacle 0
    Scan s;
    do_scan(occ, {0.05, 0.05, 0.0}, obs, SensorConfig{}, 0.0, &s);
    CHECK(!visible_at(s, 3.0, 0.05));  // shadow of the wall
    CHECK(visible_at(s, 1.5, 0.05));
    CHECK(!visible_at(s, 0.05, 4.0));  // shadow of obstacle 0
    bool wall_hit = false;
    for (const Cell& c : s.static_hits) wall_hit |= std::fabs(occ.spec.center(c.i, c.j).x - 2.05) < 0.06;
    CHECK(wall_hit);
    CHECK(s.obstacles.size() == 1 && s.obstacles[0].id == 0);

    SensorConfig perfect;
    perfect.perfect = true;  // PF-ET: occluded obstacles within range are sensed
    do_scan(occ, {0.05, 0.05, 0.0}, obs, perfect, 0.0, &s);
    CHECK(s.obstacles.size() == 3);
}

TEST(static_map_accumulates_hits) {
    MapDef m = open_map();
    m.rects.push_back({2.0, -1.0, 2.2, 1.0});
    const Grid<uint8_t> occ = rasterize(m, 0.1);
    StaticMap sm(occ.spec, 0.25);
    Scan s;
    do_scan(occ, {0.05, 0.05, 0.0}, {}, SensorConfig{}, 0.0, &s);
    CHECK(sm.integrate(s));
    CHECK(!sm.integrate(s));  // nothing new the second time
    const Cell c = occ.spec.cell_of(2.05, 0.05);
    CHECK(sm.known().at(c.i, c.j) == 1);
    const Cell ci = occ.spec.cell_of(1.8, 0.05);  // within robot radius of the wall face
    CHECK(sm.inflated().at(ci.i, ci.j) == 1);
    const Cell far = occ.spec.cell_of(3.5, 0.05);  // behind the wall: unknown
    CHECK(sm.known().at(far.i, far.j) == 0);
}

TEST(disk_hits_is_exact_against_cell_squares) {
    MapDef m = open_map();
    m.rects.push_back({0.0, 0.0, 0.2, 0.2});  // cells covering [0, 0.2]^2
    const Grid<uint8_t> occ = rasterize(m, 0.1);
    CHECK(disk_hits(occ, {0.44, 0.1}, 0.25));
    CHECK(!disk_hits(occ, {0.46, 0.1}, 0.25));
}

TEST(obstacles_move_at_v_omax_and_avoid_walls) {
    MapDef m = open_map();
    m.rects.push_back({-1.0, -1.0, 1.0, 1.0});
    const Grid<uint8_t> occ = rasterize(m, 0.1);
    Params p = params_for_set(1);
    p.n_erratic = 15;
    DynamicWorld w(occ, p, {}, 42, {-8.0, -8.0}, {8.0, 8.0});
    const Grid<uint8_t> inflated = inflate(occ, p.r_obs);
    double max_speed = 0.0;
    int inside = 0;
    for (int step = 0; step < 6000; ++step) {
        for (int i = 0; i < w.size(); ++i) {
            const ObsState& s = w.state(i, step);
            max_speed = std::max(max_speed, s.v.norm());
            const Cell c = occ.spec.cell_of(s.p);
            inside += inflated.at(c.i, c.j);
        }
    }
    CHECK(max_speed <= p.v_omax + 1e-9);
    CHECK(inside == 0);
    DynamicWorld w2(occ, p, {}, 42, {-8.0, -8.0}, {8.0, 8.0});
    CHECK(dist(w2.state(3, 5000).p, w.state(3, 5000).p) < 1e-12);  // deterministic
}
