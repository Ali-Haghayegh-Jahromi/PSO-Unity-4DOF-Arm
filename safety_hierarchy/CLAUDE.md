# Context for new sessions: Safety Hierarchy replication

Read this first, then `README.md` (results) and `docs/DESIGN.md` (paper-to-code
map and every assumption). Summarise the status back to the user before
changing anything.

## What this is

A C++17 replication of B. L'Espérance and K. Gupta, "Safety Hierarchy for
Planning With Time Constraints in Unknown Dynamic Environments", IEEE T-RO
30(6), 2014. Everything lives in this folder (`safety_hierarchy/`). The rest of
the repository is an unrelated Unity project (PSO 4-DOF arm): do not touch it.

* Repository: github.com/Ali-Haghayegh-Jahromi/PSO-Unity-4DOF-Arm, branch
  `ccr-4392f207-38fzsg` (not merged into `main`).
* The paper PDF is **not** in the repository. Ask the user to attach it
  whenever a detail of the paper is needed; never guess what the paper says.

## Build and test (Ubuntu: `sudo apt install build-essential cmake`)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/sh_tests        # must print "32 tests, 0 failed checks"
```

## Layout (one module per paper section)

| Where | What |
|---|---|
| `src/`, `include/sh/` | the algorithm: `params` (Table I, sets, 9 algorithms + NONE), `map`, `obstacles`, `sensor`, `observer` (static map + ET), `nf1`, `distance_field` (BW/SW growth), `costmap` (Algorithm 1, Eqs. 4-7), `planner` (2025 trajectories), `simulator` (plan/execute loop, metrics, deadlock fix), `stats` |
| `apps/` | `sh_run` (one run, verbose, images, trace), `sh_experiments` (many runs -> CSV), `sh_tables` (Tables II-IV vs paper), `sh_ablation` (BW/SW/ET ablation report) |
| `tests/` | 32 dependency-free tests (equations, proposition, sensor, planner, stats, regressions) |
| `maps/` | maps 1-5 digitized from the paper's Figs. 7-11 |
| `tools/` | Python: `plot_run.py`, `plot_ablation.py`, `digitize_maps.py` |
| `results/` | experiment CSVs, generated tables, ablation figure |

## Status (2026-09-30)

Done:
* full implementation;
* the paper's experiment: 810 runs, `results/results_default.csv`, `tables_default.md`;
* hypothesis H1 (`--bw-endpoints`): `results/*h1_endpoints*`;
* BW/SW/ET ablation: 2400 runs, 100 per combination and set, `results/ablation*`.

Reproduces:
* PF-ET matches the paper in sets 1-2 on every measure; F-ET and O-ET are in
  the paper's range.
* The Eq. (4) choice agrees with the hierarchy's rules in every planning
  cycle (0 "SH violations" in all runs).
* The ordering of the 7 model combinations by **collisions per minute**
  matches the paper (Spearman +0.86 / +0.71 / +0.36 in sets 1-3), and SH has
  the lowest rate in sets 1-2.

Does not reproduce:
* SH's per-run advantage. Set 1, ours vs paper: PCFR 10 % vs 63.3 %, ANCR
  3.80 vs 0.47, time 223 s vs 103 s.
* Adding SW or BW costs about +100 s here, against +5 to +32 s in the paper.
  The ET-based planners collide 1.3-3.8x more per minute than in the paper.

Located cause (partial): BW grows from the shadow edges behind static
obstacles, as the paper's Figs. 1-2 draw it. In cluttered maps this triggers
the Sec. VII-A deadlock fix 13-21 times per SH run; the paper reports 12
stuck events in 90 runs. H1 halves this but does not close the safety gap.

## Proposed next steps (not started; the user decides)

1. Obstacle-speed sensitivity. Assumption A9 is a constant 0.75 m/s; try
   random speeds up to v_omax. It likely acts on both remaining gaps. Needs a
   new `Params` field, a change in `obstacles.cpp` and a command-line flag,
   then a re-run of the ablation.
2. Other parameters the paper does not give: obstacle count in sets 1 and 3,
   robot and obstacle radius, start/goal placement.
3. Optionally open a pull request merging the branch into `main`.

## Rules this project follows

* The costmap follows Algorithm 1 and Eqs. (4)-(7) exactly. Eq. (4)'s lower
  branch sums k = 1..m (m terms). Eq. (7) assumes nested models, so the
  SH-violation counter must stay 0.
* Never tune an unspecified parameter until the numbers fit the paper. Add it
  as an option, record it in `docs/DESIGN.md` as an assumption (A#) or
  hypothesis (H#), keep the faithful default, and report both.
* Runs are deterministic per (set, map, run) seed. Any change to simulation
  behaviour changes all results: re-run the experiments and regenerate the
  tables (commands in `README.md`), then commit CSVs and tables together.
* Explain a discrepancy only after a diagnostic shows its cause (collision
  classification, `sh_run --verbose --dump-dir DIR`, `tools/plot_run.py`).
* Three bugs were fixed and have regression tests; keep them: cell-overlap
  membership for BW/SW/ET (A23), disk-based static fallback (A24), stall-aware
  start-pose prediction (A25).

## Common commands

| Command | Purpose | Time (4 cores) |
|---|---|---|
| `./build/sh_run --set 1 --map 2 --run 0 --algo SH --verbose` | one run, per-cycle log | ~10 s |
| `./build/sh_experiments --out results/results_default.csv` | the paper's 810 runs | ~25 min |
| `./build/sh_tables results/results_default.csv` | Tables II-IV vs paper | instant |
| `./build/sh_experiments --runs 20 --algos NONE,O-ET,O-SW,O-BW,ET+SW,ET+BW,SW+BW,SH --out results/ablation.csv` | ablation runs | ~90 min |
| `./build/sh_ablation results/ablation.csv --summary results/ablation_summary.csv > results/ablation_tables.md` | ablation report | instant |
| `python3 tools/plot_ablation.py results/ablation_summary.csv results/ablation.png` | ablation figure | seconds |
