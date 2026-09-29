# Safety Hierarchy planner: C++ replication

Replication of

> B. L'Espérance and K. Gupta, "Safety Hierarchy for Planning With Time
> Constraints in Unknown Dynamic Environments," *IEEE Transactions on
> Robotics*, 30(6):1398-1411, 2014. doi:10.1109/TRO.2014.2354933

in dependency-free C++17: the three models of the future (BW, SW, ET), the
composite costmap (Algorithm 1, Eqs. 4-7), the plan/execute timing of
Sec. IV, the exhaustive planner of Sec. V-B, the nine algorithms, the five
maps and the three simulation sets of Sec. VI, and the statistics used for
Tables II-IV.

* `docs/DESIGN.md` – file layout, equation-to-code mapping, and **every
  assumption** made where the paper is silent (A1-A25, H1).
* `results/` – raw CSVs of the experiments and the generated tables.

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/sh_tests          # 30 unit tests (also: ctest --test-dir build)
```

Requires only a C++17 compiler and CMake >= 3.16. The optional Python tools
need matplotlib (plotting) or PyMuPDF/Pillow/SciPy (map digitizing).

## Run

```bash
# one scenario, one algorithm; per-cycle log and costmap images
./build/sh_run --set 1 --map 2 --run 0 --algo SH --verbose \
               --dump-dir /tmp/cm --dump-every 5 --trace /tmp/trace.csv
python3 tools/plot_run.py /tmp/trace.csv maps/map2.txt /tmp/run.png --goal X Y

# the paper's experiment: 3 sets x 5 maps x 6 runs x 9 algorithms = 810 runs
./build/sh_experiments --out results/results_default.csv
./build/sh_tables results/results_default.csv      # Tables II-IV, ours / paper
```

Algorithms: `PF-ET F-ET SH O-ET O-SW O-BW ET+SW ET+BW SW+BW`.
Options: `--runs N` (runs per map), `--sets`, `--maps`, `--algos`,
`--threads`, `--bw-latency` (A18), `--bw-endpoints` (hypothesis H1).

Costmap images (`--dump-dir`) show, for slices k = 0, m/2, m: known static
(black), inflated static (grey), visible area (cream), BW (light blue),
SW (blue), ET (red), true obstacles at the sensing time (green), and the
chosen trajectory (magenta).

## The costmap, as implemented

| paper | code |
|---|---|
| SCM = NF1 on the sensed static map (unknown = free, obstacles inflated, -1, L1 wavefront) | `src/nf1.cpp` |
| Eq. (5) `c_bw = m (nf_max - nf_min) + 1` | `cost_constants()` in `src/costmap.cpp` |
| Eq. (6) `c_sw = m (nf_max + c_bw - nf_min) + 1` | same |
| Eq. (7) `c_et = m (nf_max + c_sw - nf_min) + 1` | same |
| Algorithm 1: DCM(x,y,t) = SCM, `+= c_bw / c_sw / c_et` on BW / SW / ET cells | `DynamicCostmap::build()` |
| BW, SW: robot-radius inflation at the first slice, growth at v_omax, not through static obstacles | `src/distance_field.cpp` |
| Eq. (4): end-point value if the trajectory is in F, else the sum over the m samples | `evaluate_trajectory()` |
| window = max(s_r, Delta_p v_rmax) around the robot, dt = min(dr/v_rmax, dr/v_omax) | `Params::window_half_cells()`, test `table1_parameters_are_consistent` |

`tests/test_costmap.cpp` checks Eqs. (5)-(7) literally. It checks that the
Eq. (4) argmin selects the trajectory the safety hierarchy prescribes (on
random and adversarial cases), and it checks the Algorithm 1 invariants on
real built costmaps: every DCM value equals SCM plus the constants of the
models the cell belongs to, SW ⊆ BW, and grown sets never shrink over time.
Re-deriving the Proposition exposed two conditions the paper leaves
implicit; both are tested and explained in `docs/DESIGN.md`:

1. the lower branch of Eq. (4) must sum exactly **m** samples (k = 1..m);
   with m+1 terms, Case 3 of the proof can fail;
2. Eq. (7) is sufficient only when the models are nested (ET ⊆ SW ⊆ BW).
   Every planning cycle therefore compares the Eq. (4) choice with a direct
   lexicographic implementation of the hierarchy and counts disagreements
   ("SH violations" in the tables).

The p-values use a one-sided, unpooled two-sample z-test. For PCFR this
reproduces the paper's p-values to four decimals (10 cases in
`tests/test_stats.cpp`), whereas the pooled test does not.

## Results

All numbers below: 3 sets x 5 maps x 6 runs x 9 algorithms = 810 runs, as in
the paper (30 runs per algorithm and set). Full tables with p-values,
per-map breakdowns and diagnostics: `results/tables_default.md`; raw data:
`results/results_default.csv`. Regenerate with the commands above (~25 min
on 4 cores).

### Summary: ours vs paper (PCFR % / ANCR / time to goal s)

| set | algorithm | ours | paper |
|---|---|---|---|
| 1 | PF-ET | 96.7 / 0.03 / 42 | 93.3 / 0.07 / 44 |
| 1 | F-ET  | 26.7 / 1.70 / 44 | 33.3 / 1.10 / 55 |
| 1 | O-ET  | 10.0 / 2.73 / 45 | 23.3 / 1.59 / 78 |
| 1 | **SH** | **10.0 / 3.80 / 223** | **63.3 / 0.47 / 103** |
| 2 | PF-ET | 100.0 / 0.00 / 41 | 96.7 / 0.03 / 44 |
| 2 | F-ET  | 63.3 / 0.67 / 41 | 40.0 / 1.07 / 45 |
| 2 | O-ET  | 26.7 / 1.33 / 43 | 33.3 / 1.38 / 63 |
| 2 | **SH** | **16.7 / 3.63 / 277** | **66.7 / 0.37 / 104** |
| 3 | PF-ET | 63.3 / 0.43 / 80 | 93.3 / 0.07 / 94 |
| 3 | F-ET  | 13.3 / 2.77 / 78 | 30.0 / 1.63 / 79 |
| 3 | **SH** | **0.0 / 8.67 / 261** | **33.3 / 1.43 / 182** |

### What reproduces

* **The benchmark and the ET-only planners.** PF-ET (perfect sensing and
  future) matches the paper in sets 1-2 on every measure (PCFR, ANCR, AMD
  0.07/0.06 m vs 0.09/0.08 m, time). F-ET and O-ET are in the paper's
  range. So the simulated world, the sensing/estimation pipeline, the
  timing model (Delta_e latency) and the exhaustive planner behave like
  the paper's.
* **The costmap equations.** Eqs. (4)-(7) and Algorithm 1 are implemented
  as printed and verified by tests. The Eq. (4) choice agreed with a direct
  implementation of the hierarchy steps in **every** planning cycle of all
  810 runs (0 "SH violations").
* **The hierarchy's intended effect, per unit time.** SH collides less per
  minute than F-ET (set 1: 1.0 vs 2.3 collisions/min), and on the empty
  map SH takes about 3x F-ET's time (paper: about 2x over all maps).

### What does not reproduce

* **The paper's main claim:** SH is not safer than F-ET per run in our
  simulation (PCFR 10 % vs 27 %, ANCR 3.8 vs 1.7 in set 1). Every
  algorithm that uses SW or BW is 3-7x slower than F-ET (paper: about 2x),
  and because obstacles never avoid the robot, the longer exposure turns
  the per-minute safety gain into more collisions per run.
* **Why they are slow:** SH triggers the Sec. VII-A static-deadlock fix
  13.7 times per run in set 1. The paper reports the robot got stuck 12
  times in 90 SH runs. Per map, SH is slowest in clutter (maps 2-5:
  220-320 s) and closest to the paper on the empty map 1 (93 s).
  Collision classification (set 1, 114 SH collisions): 2 on trajectories
  scored collision-free, 57 on ET-free plans made 0.8-1.6 s earlier (an
  erratic obstacle turned), and 55 in situations with no ET-free
  trajectory at all. No implementation error is visible in these; they
  are consequences of the models as specified plus the long exposure.
* **Where BW gets restrictive:** the BW model seeds a wavefront at every
  FOV boundary, including the shadow edges behind static obstacles.
  Figs. 1-2 show exactly this, and the implementation follows them
  (`tools/plot_run.py` and `--dump-dir` images show the robot retreating
  into corners, away from shadow edges, as Sec. VII predicts). With
  obstacles 2-5 m apart, and BW growing 2.5 m in 3 s, few trajectories are
  BW-free in maps 2-5.
* **Hypothesis H1** (option `--bw-endpoints`, not the paper's method): seed
  BW only at the scan's max-range end points. It halves the deadlock
  fixes and brings SH's time close to the paper in sets 2-3 (151 s vs
  104 s; 170 s vs 182 s), but SH still collides 4-7x more than in the
  paper (`results/tables_h1_endpoints.md`). So the static-shadow part of
  BW explains part of the time gap, and none of the safety gap.

### Implementation bugs found by these experiments (all fixed, with regression tests)

1. BW/SW/ET cell membership used the cell *centre*. A robot centre
   elsewhere in a "free" cell could then be ~7 cm too close: all 21 PF-ET
   collisions in set 1 were such grazes. With the cell-overlap convention:
   1 (DESIGN.md A23).
2. The predicted start pose ignored stalls, and the "no static-free
   trajectory" fallback judged walls by the robot's centre cell. Together
   they made a robot touching a wall re-plan into it forever (3 timeouts)
   (A24, A25).

### Most likely sources of the remaining gap (not specified by the paper)

Obstacle speed distribution (we use a constant 0.75 m/s), obstacle count
in sets 1 and 3 (we use 25), robot and obstacle radius (0.25 m each),
start/goal placement, the obstacles' avoidance manoeuvre, and how the
authors' code derived the FOV boundary from the laser scan (H1). None of
these was tuned. `docs/DESIGN.md` lists every such choice.

