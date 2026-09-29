// Parameters, timing constraints, the exhaustive trajectory library, planning.
#include <cmath>

#include "sh/map.hpp"
#include "sh/planner.hpp"
#include "sh/simulator.hpp"
#include "test.hpp"

using namespace sh;

TEST(table1_parameters_are_consistent) {
    const Params p = params_for_set(1);
    CHECK(p.n_trajectories() == 2025);  // Table I
    CHECK_NEAR(p.delta_p(), 3.0, 1e-12);
    // dt = min(dr / v_rmax, dr / v_omax) (Sec. III-C) -> 0.1 s with dr = 0.1 m.
    CHECK_NEAR(std::min(p.cell / p.v_rmax, p.cell / p.v_omax), p.dt, 1e-12);
    CHECK(p.window_half_cells() == 60);  // max(s_r, Dp * v_rmax) = 6 m
    std::string rep;
    CHECK(check_timing(p, 0.8, &rep));  // SH: 0.8 <= 3 < min(5.2, 8)
    CHECK(check_timing(p, 0.2, &rep));  // F-ET
}

TEST(library_samples_follow_the_controls) {
    const Params p = params_for_set(1);
    const TrajectoryLibrary lib(p);
    CHECK(lib.size() == 2025);
    const Pose start{1.0, 2.0, 0.5};
    // Control ids: traj = c0 + 45 * c1. Control id = v_idx * 9 + w_idx.
    const int stop = 0 * 9 + 4;             // v = 0, w = 0
    const int full = 4 * 9 + 4;             // v = v_max, w = 0
    const int turn = 4 * 9 + 8;             // v = v_max, w = +w_max
    const Pose a = lib.sample(stop + 45 * stop, p.m, start);
    CHECK_NEAR(a.x, 1.0, 1e-12);
    CHECK_NEAR(a.y, 2.0, 1e-12);
    const Pose b = lib.sample(full + 45 * full, p.m, start);
    CHECK_NEAR(dist(b.pos(), start.pos()), 3.0, 1e-9);
    // First 1.5 s turning, then 1.5 s straight.
    const Pose mid = integrate_unicycle(start, 1.0, 0.8, 1.5);
    const Pose end = integrate_unicycle(mid, 1.0, 0.0, 1.5);
    const Pose c = lib.sample(turn + 45 * full, p.m, start);
    CHECK_NEAR(c.x, end.x, 1e-9);
    CHECK_NEAR(c.y, end.y, 1e-9);
    const Control u0 = lib.control_at(turn + 45 * full, 0.7), u1 = lib.control_at(turn + 45 * full, 2.0);
    CHECK_NEAR(u0.w, 0.8, 1e-12);
    CHECK_NEAR(u1.w, 0.0, 1e-12);
}

// Regression: robot inside the inflated zone of a thin known wall it faces.
// Crossing the wall leaves the inflated zone fastest, but must never be chosen.
TEST(static_fallback_never_crosses_a_known_wall) {
    Params p = params_for_set(1);
    MapDef m;
    m.xmin = -15, m.ymin = -15, m.xmax = 15, m.ymax = 15;
    m.rects.push_back({0.3, -3.0, 0.6, 3.0});  // thin wall 0.3 m ahead
    const Grid<uint8_t> occ = rasterize(m, p.cell);
    const Nf1 nf1 = compute_nf1(inflate(occ, p.r_robot), occ.spec.cell_of(Vec2{10.0, 0.0}));
    Grid<uint8_t> visible(occ.spec, 1);
    std::vector<SensedObstacle> sensed;
    std::vector<EtPrediction> et;
    CostmapInput in;
    in.center = occ.spec.cell_of(Vec2{0.0, 0.0});
    in.scm = &nf1;
    in.known_static = &occ;
    in.visible = &visible;
    in.sensed = &sensed;
    in.et = &et;
    in.layers = {true, true, true};
    DynamicCostmap cm(p);
    cm.build(in);
    const TrajectoryLibrary lib(p);
    const PlanResult r = plan(lib, cm, {0.0, 0.0, 0.0});
    CHECK(r.static_fallback);
    for (int k = 1; k <= p.m; ++k) CHECK(!disk_hits(occ, lib.sample(r.traj, k, {0.0, 0.0, 0.0}).pos(), p.r_robot - 1e-6));
}

// Regression: robot touching a wall it drives along (the stuck case of set 2,
// map 2, run 1). The fallback must not pick a motion whose disk enters the wall,
// which would stall the robot forever.
TEST(static_fallback_never_pushes_into_a_touched_wall) {
    Params p = params_for_set(1);
    MapDef m;
    m.xmin = -15, m.ymin = -15, m.xmax = 15, m.ymax = 15;
    m.rects.push_back({-3.0, -1.0, 3.0, -0.3});  // wall cells up to y = -0.3; robot touching it
    const Grid<uint8_t> occ = rasterize(m, p.cell);
    const Nf1 nf1 = compute_nf1(inflate(occ, p.r_robot), occ.spec.cell_of(Vec2{-6.0, -3.0}));
    Grid<uint8_t> visible(occ.spec, 1);
    std::vector<SensedObstacle> sensed;
    std::vector<EtPrediction> et;
    CostmapInput in;
    in.center = occ.spec.cell_of(Vec2{0.0, 0.0});
    in.scm = &nf1;
    in.known_static = &occ;
    in.visible = &visible;
    in.sensed = &sensed;
    in.et = &et;
    in.layers = {false, true, true};
    DynamicCostmap cm(p);
    cm.build(in);
    const TrajectoryLibrary lib(p);
    const Pose start{0.0, -0.045, -2.86};  // 5 mm clearance, heading left and slightly into the wall
    CHECK(!disk_hits(occ, start.pos(), p.r_robot));
    const PlanResult r = plan(lib, cm, start);
    CHECK(r.static_fallback);
    Pose q = start;  // the executed first cycle must actually move or turn the robot
    int moved = 0;
    for (int s = 0; s < 80; ++s) moved += step_robot(occ, p.r_robot, lib.control_at(r.traj, s * 0.01), 0.01, &q);
    CHECK(moved == 80);
}

// A robot pushing against a known wall stays put: the predicted start pose of
// the next trajectory must not pass through the wall (regression).
TEST(robot_step_stalls_at_walls) {
    MapDef m;
    m.xmin = -5, m.ymin = -5, m.xmax = 5, m.ymax = 5;
    m.rects.push_back({1.0, -2.0, 1.2, 2.0});
    const Grid<uint8_t> occ = rasterize(m, 0.1);
    Pose q{0.5, 0.0, 0.0};
    int stalls = 0;
    for (int s = 0; s < 200; ++s) stalls += step_robot(occ, 0.25, {1.0, 0.0}, 0.01, &q) ? 0 : 1;
    CHECK(stalls > 0);
    CHECK(q.x < 1.0 - 0.25 + 1e-9);
    CHECK(step_robot(occ, 0.25, {0.0, 0.8}, 0.01, &q));  // turning in place is still possible
}

TEST(plan_heads_to_goal_in_free_space_and_avoids_walls) {
    Params p = params_for_set(1);
    MapDef m;
    m.xmin = -15, m.ymin = -15, m.xmax = 15, m.ymax = 15;
    m.rects.push_back({2.0, -0.5, 2.3, 0.5});  // small wall straight ahead
    const Grid<uint8_t> occ = rasterize(m, p.cell);
    const Grid<uint8_t> infl = inflate(occ, p.r_robot);
    const Nf1 nf1 = compute_nf1(infl, occ.spec.cell_of(Vec2{10.0, 0.0}));
    Grid<uint8_t> visible(occ.spec, 1);
    std::vector<SensedObstacle> sensed;
    std::vector<EtPrediction> et;
    CostmapInput in;
    in.center = occ.spec.cell_of(Vec2{0.0, 0.0});
    in.scm = &nf1;
    in.known_static = &occ;
    in.visible = &visible;
    in.sensed = &sensed;
    in.et = &et;
    in.layers = {true, true, true};
    DynamicCostmap cm(p);
    cm.build(in);
    const TrajectoryLibrary lib(p);
    const PlanResult r = plan(lib, cm, {0.0, 0.0, 0.0});
    CHECK(r.cost.valid && r.cost.in_F && r.sh_consistent);
    const Pose end = lib.sample(r.traj, p.m, {0.0, 0.0, 0.0});
    CHECK(end.x > 1.5);  // made progress to the goal around the wall
}
