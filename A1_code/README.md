# A1 Part B — Centralised Multi-Drone RRT-Connect

## Demo commands (B3)

```bash
# Complete template; arguments in brackets are optional.
python3 run_planner.py --environment <scenario.yaml> \
  [--num-drones K] [--time-limit SECONDS] [--seed SEED] [--eta ETA] \
  [--bridge-prob P] [--bridge-sigma SIGMA] [--local-bridge] [--visualize]

# Minimal template.
python3 run_planner.py --environment <scenario.yaml>

# Example.
python3 run_planner.py --environment environments/E1_sparse.yaml
```

Replace `<scenario.yaml>` with whatever file the marker gives you.
`--num-drones` is inferred automatically from the file, so it does not need
to be typed. `--time-limit` defaults to 20 seconds and `--seed` defaults to 0.
Omitting `--bridge-prob` runs exactly the uniform-sampling
RRT-Connect described in B2, so what is demoed matches what the report
says. `--bridge-prob 0.3` (Hsu et al. 2003 bridge test) exists as an
optional B4 experiment, not the demo default — on the measured data its
apparent gain on E5 (80%->84%, n=25) is within one standard error of the
baseline, so there is no statistically real case for switching the
default on the day.

Prints one of:
```
SUCCESS: <n> waypoints, <total>s total (<setup>s setup + <planning>s planning), eta=<eta>
VERIFIED: every motion collision-free = True, goal reached = True
```
```
FAILURE (total <total>s incl. <setup>s setup, eta=<eta>)
```
`<total>` is the full wall-clock time from process start (imports +
simulator construction + planning), budgeted to stay under `--time-limit`
with a margin — see `run_planner.py`'s `SAFETY_MARGIN`.

Add `--visualize` to also open the simulator's own 3D viewer after a
success (not needed for grading; press ENTER in the window to close it).

## Setup

Requirements: numpy, scipy, pyyaml, vedo, python-fcl (same as
`multi_drone.py`'s own README). If not already installed on the lab
machine:
```bash
pip install numpy scipy pyyaml vedo python-fcl
```

## What each file is

- `multi_drone.py` — the provided simulator, unmodified.
- `AI_prompts.txt` — complete verbatim generative-AI prompt record.
- `planner.py` — RRT-Connect implementation (matches the B2 pseudocode).
- `run_planner.py` — the entry point used above.
- `environments/` — test scenarios (E0–E13, one YAML each; E11 has no
  number-15 counterpart because it was descoped, not missing).
- `results/run_experiments.py` — runs the full battery across all
  scenarios and seeds, writes `results/experiment_results.csv`.
- `results/make_figures.py` — turns that CSV and the environment files
  into the report's figures and `results/summary_table.csv`.
- `results/validate_hard_cases.py` — validates explicit feasible waypoint
  paths for E9 and E11 independently of the sampling planner.
- `results/screenshot_paths.py` — standalone 3D screenshots of
  representative successful paths (`results/figures/path3d_*.png`),
  separate from `make_figures.py` by design.
