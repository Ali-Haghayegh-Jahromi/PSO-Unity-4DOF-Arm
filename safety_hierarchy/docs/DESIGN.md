# Design: replication of L'Espérance & Gupta, "Safety Hierarchy for Planning With Time Constraints in Unknown Dynamic Environments", IEEE T-RO 30(6), 2014

This document is the specification the code follows. It has three parts:

1. **Layout** – which file owns which part of the paper.
2. **Paper → code mapping** – every equation / algorithm and where it lives.
3. **Assumptions** – every place where the paper is silent and a choice had to be
   made, with the reason for the choice. Nothing in this list is claimed to be
   what the authors did.

---------------------------------------------------------------------------

## 1. Layout

```
safety_hierarchy/
  CMakeLists.txt            one library (sh_core) + 3 apps + 1 test binary
  include/sh/               public headers, one per module
  src/                      implementation, one .cpp per header
  apps/
    sh_run.cpp              one scenario, verbose; writes trace + costmap images (debugging)
    sh_experiments.cpp      the full 3 sets x 5 maps x 6 runs x 9 algorithms matrix -> CSV
    sh_tables.cpp           CSV -> Tables II-IV (PCFR/ANCR/AMD/p-values/time) next to the paper's numbers
  tests/                    dependency-free unit tests (ctest)
  maps/                     map1..map5 (digitized from Figs. 7-11) + periodic obstacle paths
  tools/                    optional Python helpers (map digitizer, trace plotter)
  docs/DESIGN.md            this file
```

Module dependency order (each module only uses the ones above it):

| module | header | paper section | responsibility |
|---|---|---|---|
| common | `common.hpp` | – | `Vec2`, `Pose`, deterministic RNG, angle helpers |
| grid | `grid.hpp` | – | 2-D grid, world<->cell transforms |
| params | `params.hpp` | Table I, VI-B | all parameters; the 9 algorithms; timing check (Eqs. 8-13) |
| map | `map.hpp` | Figs. 7-11 | map file I/O, rasterization, inflation, disk-vs-grid collision |
| obstacles | `obstacles.hpp` | VI-A | dynamic obstacle motion models (highly erratic, less erratic, periodic) |
| sensor | `sensor.hpp` | VI-A | 360 deg line-scan range sensor with occlusion (6 m) |
| observer | `observer.hpp` | V-A | static map module W_s(t) + trajectory estimator tr_o(t, Delta_o) |
| nf1 | `nf1.hpp` | III-C | NF1 navigation function = static costmap SCM |
| distance_field | `distance_field.hpp` | III-C | space-time wavefront used to grow BW / SW at v_omax |
| costmap | `costmap.hpp` | III-C, Alg. 1 | 3-D dynamic costmap DCM(x,y,t), Eqs. 4-7 |
| planner | `planner.hpp` | V-B | exhaustive p^k trajectory generator, lowest-cost selection |
| simulator | `simulator.hpp` | IV, V-C, VII-A | plan/execute interleaving, metrics, deadlock fix |
| stats | `stats.hpp` | VI-C, VI-D | PCFR, ANCR, AMD, one-sided z-test |

---------------------------------------------------------------------------

## 2. Paper -> code mapping

### Models of the future (Sec. III-A)
All three are sets in (x, y, t). Grown sets are computed once per cycle as a
distance field and thresholded per time slice (`distance_field.cpp`), which is
the same set a slice-by-slice wavefront growing at `v_omax` produces, without
the "one cell per slice" quantisation (0.75 m/s is 0.75 cell per slice).

* **BW** – seeds: the FOV boundary (unseen, non-static cells that touch a
  visible cell) and the sensed dynamic obstacles. Inflated by the robot radius
  at the first slice, grown at `v_omax`, growth blocked by known static cells
  (footnote 4).
* **SW** – seeds: sensed dynamic obstacles only. Same inflation / growth.
* **ET** – the estimated trajectories, inflated by robot radius + obstacle radius.

### Safety hierarchy (Sec. III-B) and costmap (Sec. III-C)
* `nf1.cpp` – NF1 of Latombe on the *known* static map, obstacles inflated by
  robot radius, unknown = free, 4-connected wavefront (L1), obstacles = -1.
* `costmap.cpp::cost_constants()` – Eqs. (5), (6), (7):
  `c_bw = m (nf_max - nf_min) + 1`,
  `c_sw = m (nf_max + c_bw - nf_min) + 1`,
  `c_et = m (nf_max + c_sw - nf_min) + 1`.
  For the variants that use a subset of the models, the same recursion is run
  over the *enabled* levels only (for the full SH this reproduces 5-7 exactly;
  a unit test checks this).
* `costmap.cpp::DynamicCostmap::build()` – Algorithm 1, literally: copy SCM
  into each time slice, then `+= c_bw` / `+= c_sw` / `+= c_et` on cells in
  BW / SW / ET.
* `costmap.cpp::evaluate()` – Eq. (4): end-point value if the trajectory is in
  F (touches none of the enabled models), otherwise the sum over the `m`
  samples `t_c + k*dt, k = 1..m`.
* Window: side = 2 * max(s_r, Delta_p * v_rmax), centred at the robot;
  `dt = min(dr/v_rmax, dr/v_omax)`.

#### Two properties of Eqs. (4)-(7) found while re-deriving the proof
Both are checked by unit tests (`tests/test_costmap.cpp`):

1. The proof of the Proposition bounds a colliding trajectory's cost by
   `m * nf1`, i.e. the sum in Eq. (4) must have exactly **m** terms. With the
   inclusive reading `t = t_c ... t_c + m dt` (m+1 terms) Case 3 can fail. The
   code therefore sums `k = 1..m` (the start cell, common to all candidates, is
   excluded; the end point, which the upper branch uses, is included).
2. Eq. (7) is sufficient only when the models are **nested**,
   `ET ⊆ SW ⊆ BW`, which holds conceptually (each model is more conservative
   than the next) but not always after discretisation / latency. Counter-
   example: an ET-free trajectory that is inside SW for all m samples loses to
   a trajectory that touches one ET-only cell. The code keeps Eq. (7) as
   printed and **counts** such events: every planning cycle compares the Eq. (4)
   argmin with a direct lexicographic implementation of the SH steps 1-3 and
   records a "SH violation" if they disagree (reported per run).

### Timing (Sec. IV)
* `params.cpp::check_timing()` reports Eq. (8) `Delta_e <= Delta_p`,
  Eq. (9) `Delta_e >= Delta_co + Delta_crp`, and Eq. (12)
  `Delta_p < min(Delta_o - Delta_e, Delta_s)`, `Delta_s = s_r / v_omax`.
* `simulator.cpp` – at every cycle start t_c: the trajectory planned during the
  previous cycle becomes active, the observer senses at t_c, and the planner
  computes the next trajectory, which starts at `t_c + Delta_e` from the pose
  predicted by the active trajectory. The ET used for that trajectory is
  evaluated on `[t_c + Delta_e, t_c + Delta_e + Delta_p]` (Sec. IV, Fig. 5).
  Planning is instantaneous in simulated time; the latency is modelled by the
  one-cycle delay. Measured wall-clock planning time is reported so it can be
  compared with the paper's Delta_crp ~ 0.6 s.

### Planner (Sec. V-B)
Exhaustive: `Delta_p` split into `k` control intervals, `p` discrete controls,
`n = p^k` trajectories; each is scored with Eq. (4); lowest cost wins.
Table I gives `n = 2025`; since 2025 = 3^4 5^2 the only integer
factorisations `p^k` are 2025^1 and 45^2, so **p = 45, k = 2** (see A7).

### Algorithms compared (Sec. VI-B)
| # | name | models | Delta_e | sensing / estimation |
|---|---|---|---|---|
| 1 | PF-ET | ET | 0.2 s | sees occluded obstacles in range, exact future |
| 2 | F-ET | ET | 0.2 s | normal |
| 3 | SH | BW, SW, ET | 0.8 s | normal |
| 4 | O-ET | ET | 0.8 s | normal |
| 5 | O-SW | SW | 0.8 s | normal |
| 6 | O-BW | BW | 0.8 s | normal |
| 7 | ET+SW | SW, ET | 0.8 s | normal |
| 8 | ET+BW | BW, ET | 0.8 s | normal |
| 9 | SW+BW | BW, SW | 0.8 s | normal |

### Deadlock fix (Sec. VII-A)
If the robot stays inside a circle of radius 1 m for 10 planning cycles, BW is
dropped (`c_bw = 0`) for 5 planning cycles.

### Performance measures (Sec. VI-C) and statistics (Sec. VI-D)
* PCFR – % of runs that reach the goal with zero collisions.
* ANCR – mean number of collisions per run (simulation continues after a collision).
* AMD – mean over runs of the minimum robot-to-dynamic-obstacle distance (0 if collided).
* p-values: **one-sided, unpooled two-sample z-test** "SH better than X".
  For PCFR this was reverse-engineered from Tables II-IV: it reproduces
  0.0074, 0.1465, 0.9988, 0.0003 (Table II), 0.2956, 0.0000 (Table III) and
  0.3906 (Table IV) to 4 decimals, whereas the pooled test does not. The same
  (unpooled, large-sample) test is applied to the means of ANCR and AMD; that
  part is an assumption (the paper does not name the test).

---------------------------------------------------------------------------

## 3. Assumptions (where the paper is silent)

| id | item | choice | reason |
|---|---|---|---|
| A1 | simulator | own 2-D simulator instead of Player/Stage | keeps the project self-contained and deterministic |
| A2 | world size | 30 m x 15 m, x in [-15,15], y in [-7.5,7.5] | measured from the grid in Figs. 7-11 (14.66 px/m) |
| A3 | static geometry | rectangles digitized from Figs. 7-11, ~±0.07 m | only source available; `tools/digitize_maps.py` |
| A4 | cell size dr | 0.1 m | Table I gives dt = 0.1 s and dt = min(dr/v_rmax, dr/v_omax) with v_rmax = 1 m/s -> dr = 0.1 m |
| A5 | robot / obstacle radius | 0.25 m / 0.25 m | not given; Fig. 1 draws them the same size |
| A6 | robot model | unicycle (v, w), no acceleration limit | Table I only gives v_rmax and w_rmax |
| A7 | control set | v in {0, .25, .5, .75, 1} v_rmax, w in 9 values on [-w_rmax, w_rmax]; k = 2 | p = 45, k = 2 is forced by n = 2025; the 5 x 9 split is a choice |
| A8 | number of obstacles | 25 in every set | Sec. VI-D-2 gives 21 + 4 = 25 for set 2; Fig. 13 also uses 25 |
| A9 | obstacle speed | constant v_omax = 0.75 m/s | only the maximum is given |
| A10 | obstacle avoidance manoeuvre | on predicted contact with a wall or another obstacle, pick a random free heading and restart the 2 s segment | paper: "heuristic avoiding manoeuvre", no details |
| A11 | periodic obstacles | shuttle back and forth along a segment that starts behind a static obstacle; paths chosen per map in `maps/` | paper gives no geometry |
| A12 | start / goal | random free poses, start in the left 3 m strip and goal in the right 3 m strip (or swapped) | only "new initial and goal configuration pair" is stated; average times (44-240 s) indicate cross-map runs |
| A13 | goal reached | robot centre within 0.3 m of the goal | not given |
| A14 | timeout | 600 s (run counted as not collision-free, time excluded) | the paper reports all SH runs reached the goal |
| A15 | laser | 720 rays over 360 deg, occluded by static and dynamic obstacles | "line scan", resolution not given |
| A16 | collisions | robot/obstacle overlap is counted once per contact onset per obstacle; obstacles pass through the robot (their trajectories are independent of the robot, Sec. II-A) | sim must continue after a collision (Sec. VI-C) |
| A17 | static obstacles in Eq. (4) | NF1 marks them -1; a trajectory entering an (inflated) known static cell is discarded before Eq. (4) | using -1 literally would make obstacles the most attractive cells |
| A18 | BW/SW growth origin | growth starts at the first slice of the planned trajectory (literal Alg. 1 and Eq. 12). Option `--bw-latency` grows them from the sensing time instead | Eq. (12) bounds Delta_p (not Delta_p + Delta_e) by Delta_s |
| A19 | F-ET implementation | same exhaustive planner with only the ET layer active, Delta_e = 0.2 s | same decision rule; only the compute-time argument differs |
| A20 | robot heading at start | facing the goal | not given |
| A21 | sim step | 0.01 s for motion and collision checks | fine enough for 1.75 m/s closing speed (1.75 cm/step) |
| A23 | cell membership of BW / SW / ET | a cell belongs to a model if **any part** of the cell is inside it (distance to the cell square), the same conservative convention as the static inflation | with "cell centre inside", a robot centre elsewhere in a "free" cell could be ~7 cm too close: in set 1, 21 of 21 PF-ET collisions were such grazes (all < 10 cm deep) on trajectories scored collision-free; with this convention: 1 |
| A24 | no static-free trajectory (A17 fallback) | rank by the number of samples at which the robot **disk** would penetrate a known wall (physically impossible motion), then inflated-zone samples, then Eq. (4) | ranking by inflated samples (or by the centre cell entering a wall) picked motions into a wall the robot was touching or through a thin wall; the robot stalled and repeated the same plan until the 600 s timeout (3 runs of the first full experiment). Regression tests `static_fallback_never_crosses_a_known_wall`, `static_fallback_never_pushes_into_a_touched_wall` |
| A25 | start pose of the next trajectory | predicted by executing the active controls for Delta_e with the same stall rule as the robot (checked against the robot's *known* map) | predicting without stalls let the planner plan from a pose a stalled robot never reaches |
| A22 | ties in Eq. (4) | first trajectory in enumeration order (first control: v ascending, then w) | not specified |
| H1 | BW seeds (option `--bw-endpoints`, **off by default**) | seed BW only at scan end points at maximum range (plus sensed obstacles), i.e. no shadow edges | a *hypothesis* about the authors' implementation, motivated by Tables II-IV (O-BW ~ O-SW in time). It contradicts Figs. 1-2, so it is not the default |
