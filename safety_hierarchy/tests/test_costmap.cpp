// Tests of the safety-hierarchy costmap: Eqs. (4)-(7), the Proposition of
// Sec. III-C, and Algorithm 1 invariants on a real built costmap.
#include <algorithm>
#include <random>

#include "sh/costmap.hpp"
#include "sh/map.hpp"
#include "sh/nf1.hpp"
#include "test.hpp"

using namespace sh;

namespace {

// ---- synthetic trajectories: per-sample (nf1 value, model bits) ----
struct Sample {
    int64_t nf;
    uint8_t bits;
};

// Eq. (4) evaluated directly on samples k = 1..n_terms.
int64_t eq4(const std::vector<Sample>& tr, const CostConstants& c) {
    bool free = true;
    int64_t sum = 0;
    for (const Sample& s : tr) {
        free = free && s.bits == 0;
        sum += s.nf + ((s.bits & kInBW) ? c.c_bw : 0) + ((s.bits & kInSW) ? c.c_sw : 0) +
               ((s.bits & kInET) ? c.c_et : 0);
    }
    return free ? tr.back().nf : sum;
}

ShKey key_of(const std::vector<Sample>& tr) {
    TrajectoryCost c;
    for (const Sample& s : tr) {
        c.in_F = c.in_F && s.bits == 0;
        c.p_bw += (s.bits & kInBW) != 0, c.p_sw += (s.bits & kInSW) != 0, c.p_et += (s.bits & kInET) != 0;
        c.sum_nf1 += s.nf;
    }
    c.nf1_end = static_cast<int32_t>(tr.back().nf);
    return sh_key(c);
}

// True if the Eq. (4) argmin is (one of) the hierarchy's best trajectories.
bool argmin_respects_sh(const std::vector<std::vector<Sample>>& trs, const CostConstants& c) {
    size_t by_cost = 0;
    ShKey best_key = key_of(trs[0]);
    for (size_t i = 1; i < trs.size(); ++i) {
        if (eq4(trs[i], c) < eq4(trs[by_cost], c)) by_cost = i;
        best_key = std::min(best_key, key_of(trs[i]));
    }
    return !(best_key < key_of(trs[by_cost]));
}

}  // namespace

TEST(constants_match_eqs_5_6_7) {
    const int m = 30;
    const int64_t lo = 0, hi = 450;
    const CostConstants c = cost_constants(m, lo, hi, {true, true, true});
    const int64_t c_bw = m * (hi - lo) + 1;           // (5)
    const int64_t c_sw = m * (hi + c_bw - lo) + 1;    // (6)
    const int64_t c_et = m * (hi + c_sw - lo) + 1;    // (7)
    CHECK(c.c_bw == c_bw);
    CHECK(c.c_sw == c_sw);
    CHECK(c.c_et == c_et);
    CHECK(c.c_bw == 13501 && c.c_sw == 418531 && c.c_et == 12569431);
}

TEST(constants_for_variants_skip_disabled_levels) {
    const int m = 30;
    const int64_t hi = 100;
    const CostConstants et = cost_constants(m, 0, hi, {false, false, true});
    CHECK(et.c_bw == 0 && et.c_sw == 0 && et.c_et == m * hi + 1);
    const CostConstants et_bw = cost_constants(m, 0, hi, {true, false, true});
    CHECK(et_bw.c_bw == m * hi + 1 && et_bw.c_sw == 0 && et_bw.c_et == m * (hi + et_bw.c_bw) + 1);
    // Deadlock fix: c_bw = 0 reduces the full SH to the ET+SW constants.
    const CostConstants relaxed = cost_constants(m, 0, hi, {false, true, true});
    CHECK(relaxed.c_sw == m * hi + 1 && relaxed.c_et == m * (hi + relaxed.c_sw) + 1);
}

// The Proposition: with nested models (ET in SW in BW) and m-term sums, the
// Eq. (4) argmin satisfies the hierarchy. Random + adversarial trajectories.
TEST(proposition_holds_for_nested_models) {
    const int m = 30;
    const int64_t nf_max = 60;
    const CostConstants c = cost_constants(m, 0, nf_max, {true, true, true});
    std::mt19937 rng(7);
    const uint8_t level_bits[4] = {0, kInBW, kInBW | kInSW, kInBW | kInSW | kInET};
    int bad = 0;
    for (int trial = 0; trial < 20000; ++trial) {
        std::vector<std::vector<Sample>> trs(8);
        for (auto& tr : trs) {
            const int max_level = static_cast<int>(rng() % 4);
            const double p_hit = (rng() % 100) / 100.0;
            // Extreme NF1 profiles (all min / all max) are what the proof's bounds are about.
            const int profile = static_cast<int>(rng() % 3);
            for (int k = 0; k < m; ++k) {
                int64_t nf = profile == 0 ? 0 : profile == 1 ? nf_max : static_cast<int64_t>(rng() % (nf_max + 1));
                const bool hit = (rng() % 1000) / 1000.0 < p_hit;
                const int lvl = hit ? 1 + static_cast<int>(rng() % std::max(1, max_level)) : 0;
                tr.push_back({nf, level_bits[std::min(lvl, max_level)]});
            }
        }
        bad += argmin_respects_sh(trs, c) ? 0 : 1;
    }
    CHECK(bad == 0);
}

// Case 3 of the proof needs exactly m terms: with m+1 terms (inclusive
// t_c..t_c+m dt) a trajectory with fewer BW cells can lose.
TEST(sum_must_have_m_terms) {
    const int m = 30;
    const int64_t nf_max = 450;
    const CostConstants c = cost_constants(m, 0, nf_max, {true, false, false});
    for (int terms : {m, m + 1}) {
        std::vector<Sample> tr1(static_cast<size_t>(terms), {nf_max, 0}), tr2(static_cast<size_t>(terms), {0, 0});
        tr1[0].bits = kInBW;                      // 1 BW cell, far from the goal
        tr2[0].bits = tr2[1].bits = kInBW;        // 2 BW cells, at the goal
        const bool ok = argmin_respects_sh({tr1, tr2}, c);
        CHECK(ok == (terms == m));
    }
}

// Eq. (7) relies on nesting: an ET-free trajectory inside SW for all m samples
// loses to one that touches a single ET-only cell. Documented in DESIGN.md.
TEST(eq7_requires_nested_models) {
    const int m = 30;
    const CostConstants c = cost_constants(m, 0, 450, {true, true, true});
    std::vector<Sample> et_only(m, {0, 0}), sw_all(m, {0, kInBW | kInSW});
    et_only[5].bits = kInET;  // not nested: ET without SW/BW
    CHECK(!argmin_respects_sh({et_only, sw_all}, c));
    et_only[5].bits = kInBW | kInSW | kInET;  // nested
    CHECK(argmin_respects_sh({et_only, sw_all}, c));
}

// ---------------------------------------------------------------- built costmap
namespace {

struct Fixture {
    Params p = params_for_set(1);
    GridSpec g{-15.0, -15.0, 0.1, 300, 300};
    Grid<uint8_t> known{g, 0}, visible{g, 1};
    Nf1 nf1;
    std::vector<SensedObstacle> sensed;
    std::vector<EtPrediction> et;
    Fixture() { nf1 = compute_nf1(inflate(known, p.r_robot), g.cell_of(Vec2{10.0, 0.0})); }
    CostmapInput input(Layers layers) {
        CostmapInput in;
        in.center = g.cell_of(Vec2{0.0, 0.0});
        in.scm = &nf1;
        in.known_static = &known;
        in.visible = &visible;
        in.sensed = &sensed;
        in.et = &et;
        in.layers = layers;
        return in;
    }
};

}  // namespace

TEST(algorithm1_invariants_on_built_costmap) {
    Fixture f;
    // An obstacle 2 m ahead moving towards the robot; an occluded region on the left.
    f.sensed.push_back({0, {2.0, 0.0}, {-0.5, 0.0}});
    f.et.push_back({0, {}});
    for (int k = 0; k <= f.p.m; ++k) f.et[0].pos.push_back(Vec2{2.0 - 0.5 * (0.8 + k * f.p.dt), 0.0});
    for (int j = 0; j < f.g.ny; ++j)
        for (int i = 0; i < f.g.nx; ++i)
            if (f.g.center(i, j).x < -3.0) f.visible.at(i, j) = 0;

    DynamicCostmap cm(f.p);
    cm.build(f.input({true, true, true}));
    const CostConstants c = cm.constants();
    int n_bw = 0, n_sw = 0, n_et = 0, nested_bad = 0, value_bad = 0, monotone_bad = 0;
    const int w2 = cm.width() * cm.width();
    for (int k = 0; k <= cm.m(); ++k) {
        for (int w = 0; w < w2; ++w) {
            const uint8_t b = cm.members(k, w);
            n_bw += (b & kInBW) != 0, n_sw += (b & kInSW) != 0, n_et += (b & kInET) != 0;
            if ((b & kInSW) && !(b & kInBW)) ++nested_bad;  // SW must be inside BW
            const int64_t expect = cm.scm(w) + ((b & kInBW) ? c.c_bw : 0) + ((b & kInSW) ? c.c_sw : 0) +
                                   ((b & kInET) ? c.c_et : 0);
            if (cm.dcm(k, w) != expect) ++value_bad;
            if (k > 0) {
                const uint8_t prev = cm.members(k - 1, w);
                if ((prev & kInBW) && !(b & kInBW)) ++monotone_bad;  // grown sets only grow
                if ((prev & kInSW) && !(b & kInSW)) ++monotone_bad;
            }
        }
    }
    CHECK(n_bw > 0 && n_sw > 0 && n_et > 0);
    CHECK(nested_bad == 0);
    CHECK(value_bad == 0);
    CHECK(monotone_bad == 0);
}

TEST(sw_grows_at_v_omax_from_robot_radius) {
    Fixture f;
    f.sensed.push_back({0, {0.0, 3.0}, {0.0, 0.0}});
    DynamicCostmap cm(f.p);
    cm.build(f.input({false, true, false}));
    for (int k : {0, 10, 30}) {
        const double reach = f.p.r_obs + f.p.r_robot + f.p.v_omax * k * f.p.dt;  // from the obstacle centre
        int w;
        CHECK(cm.window_index(0.05, 3.05 - reach + 0.06, &w));
        CHECK((cm.members(k, w) & kInSW) != 0);
        CHECK(cm.window_index(0.05, 3.05 - reach - 0.06, &w));
        CHECK((cm.members(k, w) & kInSW) == 0);
    }
}

TEST(growth_is_blocked_by_known_static) {
    Fixture f;
    // Wall at x = 1 from y = -3 to 3; obstacle at (2, 0) behind it.
    for (int j = 0; j < f.g.ny; ++j)
        for (int i = 0; i < f.g.nx; ++i) {
            const Vec2 q = f.g.center(i, j);
            if (q.x > 0.9 && q.x < 1.1 && std::fabs(q.y) < 3.0) f.known.at(i, j) = 1;
        }
    f.sensed.push_back({0, {2.0, 0.0}, {0.0, 0.0}});
    DynamicCostmap cm(f.p);
    cm.build(f.input({false, true, false}));
    int w;
    CHECK(cm.window_index(0.5, 0.0, &w));                 // 1.5 m straight-line, but behind the wall
    CHECK((cm.members(cm.m(), w) & kInSW) == 0);
    CHECK(cm.window_index(2.0, 1.5, &w));                 // same side as the obstacle
    CHECK((cm.members(cm.m(), w) & kInSW) != 0);
}

TEST(bw_seeds_at_fov_boundary) {
    Fixture f;
    for (int j = 0; j < f.g.ny; ++j)
        for (int i = 0; i < f.g.nx; ++i)
            if (f.g.center(i, j).x > 2.0) f.visible.at(i, j) = 0;  // unseen beyond x = 2
    DynamicCostmap cm(f.p);
    cm.build(f.input({true, false, false}));
    const double reach = f.p.r_robot + f.p.v_omax * cm.m() * f.p.dt;  // ~2.5 m at the last slice
    int w;
    CHECK(cm.window_index(2.0 - reach + 0.1, 0.0, &w));
    CHECK((cm.members(cm.m(), w) & kInBW) != 0);
    CHECK(cm.window_index(2.0 - reach - 0.2, 0.0, &w));
    CHECK((cm.members(cm.m(), w) & kInBW) == 0);
    CHECK(cm.window_index(-4.0, 0.0, &w));
    CHECK((cm.members(cm.m(), w) & kInBW) == 0);
}

TEST(eq4_on_built_costmap) {
    Fixture f;
    f.sensed.push_back({0, {1.5, 0.0}, {0.0, 0.0}});
    f.et.push_back({0, std::vector<Vec2>(static_cast<size_t>(f.p.m + 1), {1.5, 0.0})});
    DynamicCostmap cm(f.p);
    cm.build(f.input({true, true, true}));
    std::vector<int> through(static_cast<size_t>(cm.m())), away(static_cast<size_t>(cm.m()));
    for (int k = 1; k <= cm.m(); ++k) {
        CHECK(cm.window_index(0.1 * k, 0.0, &through[static_cast<size_t>(k - 1)]));   // drives into it
        CHECK(cm.window_index(-0.02 * k, 0.0, &away[static_cast<size_t>(k - 1)]));  // backs off slowly
    }
    const TrajectoryCost a = evaluate_trajectory(cm, through.data());
    const TrajectoryCost b = evaluate_trajectory(cm, away.data());
    CHECK(!a.in_F && a.p_et > 0 && a.p_sw >= a.p_et && a.p_bw >= a.p_sw);
    int64_t sum = 0;
    for (int k = 1; k <= cm.m(); ++k) sum += cm.dcm(k, through[static_cast<size_t>(k - 1)]);
    CHECK(a.cost == sum);  // lower branch of Eq. (4)
    CHECK(b.cost < a.cost);
}
