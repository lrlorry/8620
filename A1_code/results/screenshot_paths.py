"""Standalone 3D screenshot tool -- separate from make_figures.py by
design (the user asked for a screenshot script, not a pipeline change).

Uses multi_drone.py's own visualize_paths() unmodified (same camera,
same drawing code, so the screenshots look exactly like what --visualize
would show you interactively), but at runtime replaces the Plotter's
blocking .interactive() call with a no-op and calls .screenshot()
instead, so it can run unattended for many environments in one pass.
multi_drone.py itself is never edited.

Needs the path_<Environment>.npy files already produced by
run_experiments.py. Skips environments with no saved path (E9/E11/E13:
0% success, nothing to screenshot).

Run: python results/screenshot_paths.py
"""

import os

import numpy as np

RESULTS_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.join(RESULTS_DIR, "..")
FIG_DIR = os.path.join(RESULTS_DIR, "figures")

os.chdir(ROOT_DIR)
import sys
sys.path.insert(0, ROOT_DIR)

# offscreen must be set at Plotter construction time, not after -- patch
# vedo.Plotter before multi_drone imports it (multi_drone.py itself is
# untouched; this only affects how *this* script's own process renders).
import vedo
_RealPlotter = vedo.Plotter
vedo.Plotter = lambda *a, **kw: _RealPlotter(*a, **{**kw, "offscreen": True})

from multi_drone import MultiDrone

ENVS = [
    "E0_empty", "E1_sparse", "E2_moderate", "E3_dense", "E4_bottleneck_moderate",
    "E5_bottleneck_severe", "E6_multi_gap", "E7_offset_passage",
    "E8_swap_K2", "E8_swap_K3", "E8_swap_K4", "E8_swap_K5",
    "E10_swap_corridor", "E12_partial_goal",
]

for env_name in ENVS:
    npy_path = os.path.join(RESULTS_DIR, f"path_{env_name}.npy")
    if not os.path.exists(npy_path):
        print(f"skip {env_name}: no saved path")
        continue

    path = np.load(npy_path)
    num_drones = path.shape[1]
    sim = MultiDrone(num_drones=num_drones, environment_file=f"environments/{env_name}.yaml")
    sim._plotter.interactive = lambda *a, **kw: None  # skip the blocking wait-for-key step

    sim.visualize_paths([wp for wp in path])  # unmodified drawing logic from multi_drone.py

    out_path = os.path.join(FIG_DIR, f"path3d_{env_name}.png")
    sim._plotter.screenshot(out_path)
    sim._plotter.close()
    print(f"wrote {out_path}")

print("DONE")
