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
  assumption** made where the paper is silent (A1-A22, H1).
* `results/` – raw CSVs of the experiments and the generated tables.

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/sh_tests          # 26 unit tests (also: ctest --test-dir build)
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

RESULTS_PLACEHOLDER
