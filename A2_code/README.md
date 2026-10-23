# COMP8620 Assignment 2 — online multi-drone MCTS under slip uncertainty

The submission entry point is `run_u8076655.py`. It accepts the teacher's YAML
without source edits and supports the original 8/26-direction, little-endian
joint-action encoding. No trained model, GPU or network service is used.

```bash
python run_u8076655.py --config <teacher_scenario.yaml>
python run_u8076655.py --config environments/demo_open_2d.yaml
python run_u8076655.py --config environments/demo_four_3d.yaml --visualize
```

Each executed step prints one progress line to stderr (step, drones at goal,
reward, planning time, simulations, positions; `*` marks a drone at its goal),
followed by a one-line result. The complete episode summary is printed as JSON
on stdout. `--quiet` suppresses the progress lines.

The default allowance is **9.5 seconds per decision** with at most 2,000 MCTS
simulations. Search stops at 97% of the allowance and checks the deadline
before every simulator call, so an overrun is limited to one simulator call
plus bookkeeping; on the development machine the largest default decision
took about 9.2 s. This is a measurement, not a hard real-time guarantee. The loop replans after
every executed transition. Planning is serial and single-threaded (the runner
sets numerical-library thread limits to one). A planning simulator and a
separate execution simulator prevent search samples from advancing the
execution RNG. The default seed is the YAML's `seed`, or 0 if absent; `--seed`
overrides it.

```bash
python run_u8076655.py --config environments/demo_four_3d.yaml \
  --planning-time 0.5 --seed 0 --output results/demo_trace.json
```

## Requirements and platforms

Headless dependencies are NumPy, SciPy and PyYAML; Matplotlib is needed only to
regenerate report figures. `--visualize` uses the supplied Vedo viewer; if
`vedo` is not installed the run continues headless with a warning.

```bash
python -m pip install -r requirements.txt
```

Python 3.9 or later. The unit tests and the demo were run with Python 3.12 /
NumPy 1.26 / SciPy 1.15 and with Python 3.9 / NumPy 2.0 / SciPy 1.13; the
visualisation path was exercised with Vedo 2025.5 / VTK 9.6. All file names
are lower case and paths are built with `pathlib`, so the archive runs
unchanged on Linux.

## Solver

`planner.py` implements UCT with action progressive widening, sampled-outcome
children, epsilon-greedy guided rollouts, finite-horizon truncation, a static
leaf estimate and incremental Monte Carlo backups. Guidance is risk-aware:

- For each goal, a Dijkstra cost-to-go map charges every entered cell
  `1 + lambda * rho(cell)`, where `rho` is the probability that the next move
  slips into an obstacle (computed from the configuration with the supplied
  `d/(d+alpha)` rule) and `lambda = (|collision penalty| + goal reward) / |step cost|`.
  A detour is taken only when it buys enough collision probability.
- A joint proposal adds the expected loss of another active drone slipping
  into the chosen cell this step (only drones in the same layer when altitude
  is fixed).
- The leaf estimate drops the goal reward of a drone that drones frozen at
  their goals disconnect from its goal.
- Exploratory proposals and rollout noise choose uniformly among each drone's
  admissible commands (inside the grid, not an obstacle, not already reserved);
  the tree never admits a command towards an invalid cell.

The final root decision is anchored to the greedy (base-policy) action: another
action is executed only if its mean return exceeds the base action's by more
than one pooled standard error of the difference. This is a heuristic test,
not a policy-improvement guarantee. Progressive widening admits
`ceil(0.5 * sqrt(N(s)+1))` actions, so each candidate receives enough visits to
be compared. On the development set the earlier most-visited rule made search
*worse* than its own base policy; the anchored rule removed that loss.

The planner only accesses `get_config()`, `simulate(state, action)`, and
`num_actions` (property or method). All maps are derived from the public
configuration; it does not read the simulator's private action vectors,
distance field, RNG, live positions or time counter. Parameters were selected
on a separate development set (`environments/dev/`, `results/dev/`, two rounds,
1,920 episodes) before any reported evaluation ran.

`multi_drone.py` and `multi_drone_config.py` are vendored from the teacher's
checkout, revision recorded in `UPSTREAM.json`. Only Vedo imports were moved
inside plotting functions to permit headless execution. All core methods have
identical ASTs; see `results/model_verification.json`.

Important model details preserved:

- Out-of-grid directions are discarded and probabilities renormalised;
  they do not produce a boundary stay action.
- Slips may hit obstacle cells and terminate the episode.
- Without any obstacle, the supplied distance transform measures distance to
  a virtual point outside the grid corner, so slips are rare but nonzero.
- Reached drones remain solid and frozen; their goal rewards are paid once.
- Step cost is multiplied by the total number of drones, including reached
  ones. Shared-cell penalties count all drones involved.
- Cell swaps and diagonal segment crossings are permitted by the teacher's
  endpoint-cell collision definition.
- `simulate` does not enforce the real execution's step counter. Search tracks
  the remaining horizon itself and stops after any terminal outcome.
- Positive `alpha` is required by the supplied model; `alpha=0` is rejected.

Intended-command masks used in the action-space experiments keep the original
physical transition model: `axial` / `face_edge` restrict the commands, and
`dupK` repeats every command K times (more labels, identical control).

## Verification and scenario coverage

```bash
python -m unittest discover -s tests -v
python scripts/verify_small_model.py
python scripts/verify_upstream.py
```

Tests cover encoding, probability rows, boundaries, obstacle and shared-cell
collisions, allowed swaps, one-time goal rewards, frozen goals, simulator side
effects, API restrictions, the planner's slip model against sampled simulator
outcomes, risk maps, admissible proposals, duplicated labels, progress output,
seeded reproducibility, independent RNGs, discount endpoints, zero/tiny budgets,
finite horizons, static infeasibility and invalid configurations.

`environments/manifest.json` lists 85 entries: 18 named regression cases,
27 factorial conditions, 16 generated layouts (drone count crossed with noise
level), 8 mechanism controls, and 16 development layouts used only for
parameter selection.

## Reproduce experiments and figures

All reported suites fix the search effort by a simulation cap (2,000, the demo
default; the budget suite varies it), so worker load changes run time but not
decisions. Episodes plan serially; independent episodes may run in parallel:

```bash
python scripts/make_scenarios.py
python scripts/run_all_experiments.py --workers 12 [--with-tuning]
python scripts/analyse_results.py
python scripts/release_validation.py      # single process: tests, demos, 10 s check
```

The reported design has 3,020 episodes (360 regression, 160 generated,
540 factorial, 880 component, 400 budget, 160 control, 520 search-free
baseline) plus 1,920 development-set episodes. Two planner fixes made after the
reported runs cannot fire in any reported scenario; `scripts/verify_equivalence.py`
replays every reported seed-0 episode with the final code and records the
step-by-step comparison in `results/planner_equivalence.json`. Raw outcomes, seeds, search diagnostics and
provenance (`*.meta.json`, planner hash) are in `results/`; seed-0
trajectories are in `results/trajectories/`. `summary.csv` gives means with
95% Student-t intervals and Wilson intervals for success rates. Factor and
component effects use paired seed blocks.

The failure-sensitive completion metric `penalized_steps` is the number of
steps on success and the episode step limit on failure.

## Report and package

Report sources are in the sibling `A2/` folder in the workspace and in
`report/` inside the submission archive.

```bash
python scripts/finish_report.py
# In the report directory:
latexmk -pdf -interaction=nonstopmode -halt-on-error u8076655_A2_report.tex
# Back in the code directory:
python scripts/package_submission.py
python scripts/check_package.py ../A2/u8076655_A2_code.zip
```

`check_package.py` verifies all file hashes, extracts into a temporary folder,
runs the entry point and unit tests there, regenerates analyses, and
recompiles the packaged report.
