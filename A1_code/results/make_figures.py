"""Report figure generator -- the single script that turns
experiment_results.csv and the environment YAMLs into every figure/table
used in the B4 report. Separate from multi_drone.py: this is report-figure
tooling only, not part of the graded planner/demo code path, so it is free
to control its own camera angle (multi_drone.py's visualize_paths camera
is fixed and is left untouched to keep B3 demo behaviour faithful).

Produces:
  - results/figures/<Environment>.png   one top-down layout per scenario
    (E9 gets a Y-Z cross-section instead: it constrains y AND z at a fixed
    x, so a top-down view would show a solid wall with no visible gap)
  - results/figures/path_<Environment>.png  representative successful paths
    overlaid on the layout, one per environment except E9/E11/E13 (0%
    success, nothing to show) -- needs the path_<Environment>.npy files from
    run_experiments.py
  - results/summary_table.csv            per-environment n/success-rate/
    mean-time/95%-CI, for the B4 report's results table
  - results/figures/complexity_results.png
    baseline success rate and mean capped runtime with 95% confidence intervals

Run: python results/make_figures.py
"""

import csv
import glob
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Circle, Rectangle
from scipy.stats import t as student_t

RESULTS_DIR = os.path.dirname(__file__)
ENV_DIR = os.path.join(RESULTS_DIR, "..", "environments")
FIG_DIR = os.path.join(RESULTS_DIR, "figures")
CSV_PATH = os.path.join(RESULTS_DIR, "experiment_results.csv")
os.makedirs(FIG_DIR, exist_ok=True)

DRONE_COLORS = plt.cm.jet(np.linspace(0, 1, 5))

# E9 is constrained in y AND z at a fixed x -- see module docstring.
YZ_CROSS_SECTION = {"E9_3d_aperture.yaml": 25}


def mean_ci95(values):
    """Return the sample mean and two-sided 95% Student-t half-width."""
    values = np.asarray(values, dtype=float)
    mean = values.mean()
    if len(values) < 2:
        return mean, 0.0
    sem = values.std(ddof=1) / np.sqrt(len(values))
    return mean, student_t.ppf(0.975, len(values) - 1) * sem


# --------------------------------------------------------------------------
# Environment layouts (one PNG per scenario, filename matches environments/)
# --------------------------------------------------------------------------

def draw_environment(ax, env_path, title):
    env = yaml.safe_load(open(env_path))
    bounds = env["bounds"]
    x_lo, x_hi = bounds["x"]
    y_lo, y_hi = bounds["y"]

    for obs in env.get("obstacles") or []:
        if obs["type"] == "box":
            pos, size = obs["position"], obs["size"]
            ax.add_patch(Rectangle(
                (pos[0] - size[0] / 2, pos[1] - size[1] / 2), size[0], size[1],
                facecolor="firebrick", edgecolor="black", alpha=0.85, zorder=2,
            ))
        elif obs["type"] == "sphere":
            pos, r = obs["position"], obs["radius"]
            ax.add_patch(Circle((pos[0], pos[1]), r, facecolor="firebrick",
                                 edgecolor="black", alpha=0.85, zorder=2))
        elif obs["type"] == "cylinder":
            p0, _ = obs["endpoints"]
            r = obs["radius"]
            ax.add_patch(Circle((p0[0], p0[1]), r, facecolor="firebrick",
                                 edgecolor="black", alpha=0.85, zorder=2))

    n_drones = len(env["goals"])
    for i, g in enumerate(env["goals"]):
        pos, r = g["position"], g["radius"]
        color = DRONE_COLORS[i % 5]
        ax.add_patch(Circle((pos[0], pos[1]), r, facecolor=color, alpha=0.3,
                             edgecolor=color, zorder=3))

    for i, p in enumerate(env["initial_configuration"]):
        color = DRONE_COLORS[i % 5]
        ax.scatter([p[0]], [p[1]], color=color, s=45, zorder=4, edgecolor="black", linewidth=0.6)

    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(y_lo, y_hi)
    ax.set_aspect("equal")
    ax.set_title(f"{title} (K={n_drones})", fontsize=11)
    ax.set_xlabel("x")
    ax.set_ylabel("y")


def draw_yz_cross_section(ax, env_path, title, at_x):
    env = yaml.safe_load(open(env_path))
    bounds = env["bounds"]
    y_lo, y_hi = bounds["y"]
    z_lo, z_hi = bounds["z"]

    for obs in env.get("obstacles") or []:
        if obs["type"] != "box":
            continue
        pos, size = obs["position"], obs["size"]
        x_lo_o, x_hi_o = pos[0] - size[0] / 2, pos[0] + size[0] / 2
        if not (x_lo_o <= at_x <= x_hi_o):
            continue
        ax.add_patch(Rectangle(
            (pos[1] - size[1] / 2, pos[2] - size[2] / 2), size[1], size[2],
            facecolor="firebrick", edgecolor="black", alpha=0.85, zorder=2,
        ))

    ax.set_xlim(y_lo, y_hi)
    ax.set_ylim(z_lo, z_hi)
    ax.set_aspect("equal")
    ax.set_title(f"{title} (Y-Z slice at x={at_x})", fontsize=11)
    ax.set_xlabel("y")
    ax.set_ylabel("z")


def make_layout_figures():
    for env_path in sorted(glob.glob(os.path.join(ENV_DIR, "E*.yaml"))):
        name = os.path.splitext(os.path.basename(env_path))[0]
        fig, ax = plt.subplots(figsize=(4.5, 4.5))
        if os.path.basename(env_path) in YZ_CROSS_SECTION:
            draw_yz_cross_section(ax, env_path, name, at_x=YZ_CROSS_SECTION[os.path.basename(env_path)])
        else:
            draw_environment(ax, env_path, name)
        fig.tight_layout()
        out_path = os.path.join(FIG_DIR, f"{name}.png")
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"wrote {out_path}")


# --------------------------------------------------------------------------
# Representative path results (layout + an actual flown path overlaid)
# --------------------------------------------------------------------------

def make_path_figures():
    reps = [
        ("E0_empty.yaml", "path_E0_empty.npy",
         "K=5, seed=0, eta=4.330, timeout=20s, planning time=0.04s, waypoints=2"),
        ("E1_sparse.yaml", "path_E1_sparse.npy",
         "K=5, seed=0, eta=4.330, timeout=20s, planning time=0.12s, waypoints=20"),
        ("E2_moderate.yaml", "path_E2_moderate.npy",
         "K=5, seed=0, eta=4.330, timeout=20s, planning time=0.29s, waypoints=23"),
        ("E3_dense.yaml", "path_E3_dense.npy",
         "K=5, seed=0, eta=4.330, timeout=20s, planning time=0.42s, waypoints=22"),
        ("E4_bottleneck_moderate.yaml", "path_E4_bottleneck_moderate.npy",
         "K=5, seed=0, eta=4.330, timeout=20s, planning time=0.15s, waypoints=21"),
        ("E5_bottleneck_severe.yaml", "path_E5_bottleneck_severe.npy",
         "K=5, seed=0, eta=4.330, timeout=20s, planning time=2.18s, waypoints=36"),
        ("E6_multi_gap.yaml", "path_E6_multi_gap.npy",
         "K=5, seed=0, eta=4.330, timeout=20s, planning time=1.60s, waypoints=34"),
        ("E7_offset_passage.yaml", "path_E7_offset_passage.npy",
         "K=5, seed=14 (1/15 baseline success rate), eta=4.330, timeout=20s, "
         "planning time=12.37s, waypoints=52"),
        ("E8_swap_K2.yaml", "path_E8_swap_K2.npy",
         "K=2, seed=0, eta=4.330, timeout=20s, planning time=0.04s, waypoints=8"),
        ("E8_swap_K3.yaml", "path_E8_swap_K3.npy",
         "K=3, seed=0, eta=4.330, timeout=20s, planning time=0.04s, waypoints=10"),
        ("E8_swap_K4.yaml", "path_E8_swap_K4.npy",
         "K=4, seed=0, eta=4.330, timeout=20s, planning time=0.05s, waypoints=11"),
        ("E8_swap_K5.yaml", "path_E8_swap_K5.npy",
         "K=5, seed=0, eta=4.330, timeout=20s, planning time=0.06s, waypoints=13"),
        ("E10_swap_corridor.yaml", "path_E10_swap_corridor.npy",
         "K=5, seed=1 (seed=0 failed), eta=4.330, timeout=20s, "
         "planning time=14.88s, waypoints=33"),
        ("E12_partial_goal.yaml", "path_E12_partial_goal.npy",
         "K=5, seed=0, eta=4.330, timeout=20s, planning time=0.04s, waypoints=2"),
    ]
    for env_name, npy_name, caption in reps:
        npy_path = os.path.join(RESULTS_DIR, npy_name)
        if not os.path.exists(npy_path):
            print(f"skip {env_name}: {npy_name} not found (run run_experiments.py first)")
            continue
        path = np.load(npy_path)  # (waypoints, num_drones, 3)
        fig, ax = plt.subplots(figsize=(5.5, 5.5))
        draw_environment(ax, os.path.join(ENV_DIR, env_name), env_name.replace(".yaml", ""))
        for i in range(path.shape[1]):
            traj = path[:, i, :]
            ax.plot(traj[:, 0], traj[:, 1], "-", color=DRONE_COLORS[i % 5], linewidth=1.8, zorder=5)
        fig.text(0.5, -0.02, caption, ha="center", fontsize=8, wrap=True)
        fig.tight_layout()
        out_path = os.path.join(FIG_DIR, f"path_{env_name.replace('.yaml', '')}.png")
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"wrote {out_path}")


# --------------------------------------------------------------------------
# Summary table (n, success rate, mean time, 95% CI) for the B4 report
# --------------------------------------------------------------------------

def make_summary_table():
    if not os.path.exists(CSV_PATH):
        print(f"skip summary table: {CSV_PATH} not found (run run_experiments.py first)")
        return
    rows = list(csv.DictReader(open(CSV_PATH)))
    groups = defaultdict(list)
    for r in rows:
        groups[(r["table"], r["environment"], r["bridge_prob"])].append(
            (int(r["success"]), float(r["elapsed_s"]))
        )

    out_path = os.path.join(RESULTS_DIR, "summary_table.csv")
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["table", "environment", "bridge_prob", "n",
                                           "success_rate", "mean_time_s", "ci95_s"])
        w.writeheader()
        for (table, env, bp), trials in groups.items():
            n = len(trials)
            rate = sum(s for s, _ in trials) / n
            times = np.array([t for _, t in trials])  # failures already logged at ~20s
            mean_time, ci95 = mean_ci95(times)
            w.writerow({
                "table": table, "environment": env, "bridge_prob": bp, "n": n,
                "success_rate": f"{rate:.2f}", "mean_time_s": f"{mean_time:.2f}",
                "ci95_s": f"{ci95:.2f}",
            })
    print(f"wrote {out_path}")


def make_complexity_results_figure():
    rows = list(csv.DictReader(open(CSV_PATH)))
    environments = [
        "E0_empty.yaml",
        "E1_sparse.yaml",
        "E2_moderate.yaml",
        "E3_dense.yaml",
        "E4_bottleneck_moderate.yaml",
        "E5_bottleneck_severe.yaml",
        "E6_multi_gap.yaml",
        "E7_offset_passage.yaml",
        "E9_3d_aperture.yaml",
        "E10_swap_corridor.yaml",
        "E11_dead_end.yaml",
    ]
    labels = [e.split("_")[0] for e in environments]

    success_rates = []
    mean_times = []
    ci95 = []
    for environment in environments:
        # Match on bridge_prob=0.0 (baseline) directly rather than the
        # "table" tag, since E9-E11 are logged under "extra" but E5-E7
        # are logged under both "complexity" (0.0) and "mitigation" (0.3)
        # -- this keeps every bar a baseline-only comparison regardless.
        trials = [
            r for r in rows
            if r["environment"] == environment and r["bridge_prob"] == "0.0"
        ]
        successes = np.array([int(r["success"]) for r in trials])
        times = np.array([float(r["elapsed_s"]) for r in trials])
        success_rates.append(100.0 * successes.mean())
        mean_time, half_width = mean_ci95(times)
        mean_times.append(mean_time)
        ci95.append(half_width)

    x = np.arange(len(labels))
    group_splits = [3.5, 7.5]
    fig, (ax_success, ax_time) = plt.subplots(
        2, 1, figsize=(9.8, 5.6), sharex=True,
        gridspec_kw={"height_ratios": [1, 1.25]},
    )
    ax_success.bar(x, success_rates, color="steelblue", edgecolor="black", linewidth=0.5)
    ax_success.set_ylabel("Success rate (%)")
    ax_success.set_ylim(0, 105)
    ax_success.grid(axis="y", alpha=0.25)
    for split in group_splits:
        ax_success.axvline(split, color="grey", linestyle="--", linewidth=1)

    ax_time.bar(x, mean_times, color="indianred", edgecolor="black", linewidth=0.5)
    ax_time.errorbar(x, mean_times, yerr=ci95, fmt="none", ecolor="black", capsize=4)
    ax_time.set_ylabel("Mean capped runtime (s)")
    ax_time.set_xlabel("Environment")
    ax_time.set_xticks(x, labels)
    ax_time.set_ylim(bottom=0)
    ax_time.grid(axis="y", alpha=0.25)
    for split in group_splits:
        ax_time.axvline(split, color="grey", linestyle="--", linewidth=1)

    fig.tight_layout()
    out_path = os.path.join(FIG_DIR, "complexity_results.png")
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    make_layout_figures()
    make_path_figures()
    make_summary_table()
    make_complexity_results_figure()
