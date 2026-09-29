#!/usr/bin/env python3
"""Plot a trace written by `sh_run --trace FILE`.

Left: map, robot path (coloured by time) and obstacle paths.
Right: distance to the goal over time.

usage: plot_run.py TRACE.csv MAP.txt OUT.png [--goal X Y]
"""
import argparse
import csv
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402


def load_map(path):
    rects, world = [], (-15, -7.5, 15, 7.5)
    for line in open(path):
        line = line.split("#")[0].split()
        if not line:
            continue
        vals = [float(v) for v in line[1:5]]
        if line[0] == "world":
            world = tuple(vals)
        elif line[0] == "rect":
            rects.append(vals)
    return world, rects


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("map")
    ap.add_argument("out")
    ap.add_argument("--goal", nargs=2, type=float)
    a = ap.parse_args()

    rows = list(csv.DictReader(open(a.trace)))
    t = [float(r["t"]) for r in rows]
    rx = [float(r["rx"]) for r in rows]
    ry = [float(r["ry"]) for r in rows]
    n_obs = sum(1 for k in rows[0] if k.endswith("x") and k.startswith("o"))
    world, rects = load_map(a.map)

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 5), gridspec_kw={"width_ratios": [2.2, 1]})
    for x0, y0, x1, y1 in rects:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color="black"))
    for i in range(n_obs):
        ax.plot([float(r[f"o{i}x"]) for r in rows], [float(r[f"o{i}y"]) for r in rows],
                color="tab:green", lw=0.4, alpha=0.35)
    sc = ax.scatter(rx, ry, c=t, s=3, cmap="viridis")
    fig.colorbar(sc, ax=ax, label="t [s]")
    ax.plot(rx[0], ry[0], "o", color="tab:blue", label="start")
    if a.goal:
        ax.plot(*a.goal, "x", color="tab:red", ms=10, mew=2, label="goal")
    ax.set_xlim(world[0], world[2])
    ax.set_ylim(world[1], world[3])
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=8)

    if a.goal:
        ax2.plot(t, [math.hypot(x - a.goal[0], y - a.goal[1]) for x, y in zip(rx, ry)])
        ax2.set_ylabel("distance to goal [m]")
    ax2.set_xlabel("t [s]")
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)


if __name__ == "__main__":
    main()
