// Parameters, timing constraints, the exhaustive trajectory library, planning.
#include <cmath>

#include "sh/map.hpp"
#include "sh/planner.hpp"
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
    CHECK(r.cost.wall_cells == 0);
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
