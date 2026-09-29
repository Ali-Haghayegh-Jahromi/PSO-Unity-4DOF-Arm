#!/usr/bin/env python3
"""Plot the BW/SW/ET ablation against the paper (Tables II-IV).

Input is the summary written by `sh_ablation ... --summary FILE`, so the figure
and the markdown tables come from the same numbers. Small multiples: one row per
metric, one column per simulation set; x = combination of models.
  ours  : filled circle, 95 % CI whisker (all runs)
  paper : hollow diamond (no value for NONE, which is not in the paper)

usage: plot_ablation.py SUMMARY.csv OUT.png
"""
import csv
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

# Reference data-viz palette, light mode; the two slots pass the CVD / contrast validator.
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"
OURS = "#2a78d6"   # categorical slot 1
PAPER = "#eb6834"  # categorical slot 2

COMBOS = ["NONE", "O-ET", "O-SW", "O-BW", "ET+SW", "ET+BW", "SW+BW", "SH"]
ROWS = [  # metric key, axis label, better direction
    ("pcfr", "PCFR (%)", "higher is better"),
    ("ancr", "Collisions per run", "lower is better"),
    ("coll_per_min", "Collisions per minute", "lower is better"),
    ("amd", "Min. distance AMD (m)", "higher is better"),
    ("time", "Time to goal (s)", "lower is better"),
]
SETS = {1: "Set 1: highly erratic, robot 1 m/s",
        2: "Set 2: less erratic + periodic, 1 m/s",
        3: "Set 3: highly erratic, robot 0.5 m/s"}


def load(path):
    d = defaultdict(dict)
    for r in csv.DictReader(open(path)):
        f = lambda k: float(r[k]) if r[k] not in ("", "nan", "N/A") else None  # noqa: E731
        d[(int(r["set"]), r["metric"])][r["combo"]] = (f("ours"), f("lo"), f("hi"), f("paper"))
    return d


def main():
    data = load(sys.argv[1])
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": TEXT_2,
                         "xtick.color": TEXT_2, "ytick.color": TEXT_2, "text.color": TEXT})
    fig, axes = plt.subplots(len(ROWS), 3, figsize=(14, 3.0 * len(ROWS)), sharex=True)
    fig.patch.set_facecolor(SURFACE)
    x = range(len(COMBOS))
    for i, (metric, label, better) in enumerate(ROWS):
        for j, s in enumerate((1, 2, 3)):
            ax = axes[i][j]
            ax.set_facecolor(SURFACE)
            vals = data[(s, metric)]
            for k, c in enumerate(COMBOS):
                v, lo, hi, p = vals[c]
                if lo is not None and hi is not None and hi > lo:
                    ax.plot([k - 0.12, k - 0.12], [lo, hi], color=OURS, lw=1.5, solid_capstyle="round", zorder=2)
                ax.plot(k - 0.12, v, "o", ms=7, color=OURS, mec=SURFACE, mew=1.5, zorder=3)
                if p is not None:
                    ax.plot(k + 0.12, p, "D", ms=7, mfc="none", mec=PAPER, mew=1.8, zorder=3)
            ax.axvspan(-0.5, 0.5, color=GRID, alpha=0.35, lw=0, zorder=0)  # NONE: not in the paper
            ax.grid(axis="y", color=GRID, lw=0.8)
            ax.set_axisbelow(True)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            ax.set_xlim(-0.5, len(COMBOS) - 0.5)
            ax.set_ylim(bottom=0)
            if j == 0:
                ax.set_ylabel(f"{label}\n({better})")
            if i == 0:
                ax.set_title(SETS[s], color=TEXT, fontsize=10, loc="left")
            if i == len(ROWS) - 1:
                ax.set_xticks(list(x))
                ax.set_xticklabels(COMBOS, rotation=35, ha="right")
    handles = [Line2D([], [], marker="o", ls="none", ms=7, color=OURS, mec=SURFACE, label="ours: mean and 95 % CI"),
               Line2D([], [], marker="D", ls="none", ms=7, mfc="none", mec=PAPER, mew=1.8, label="paper: Tables II-IV")]
    fig.legend(handles=handles, loc="upper left", ncol=2, frameon=False, bbox_to_anchor=(0.01, 1.0))
    fig.text(0.99, 0.995, "Shaded: NONE (no model) is not in the paper. Paper collisions/min = 60 x ANCR / time.",
             ha="right", va="top", color=TEXT_2, fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(sys.argv[2], dpi=110, facecolor=SURFACE)


if __name__ == "__main__":
    main()
