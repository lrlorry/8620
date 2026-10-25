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
before every simulator call. Simulator calls and bookkeeping can extend past
the check; on the development machine the largest default decision
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
`vedo` is not installed the run continues headless with a warning. As in the
supplied template, the result is printed before the viewer turns interactive;
close the window (or press `q`) to exit.

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
children, greedy rollouts with epsilon exploration, finite-horizon truncation, a
heuristic leaf value and sample-mean return backups. Guidance comes from
single-drone value iteration (`guidance='vi'`, the default):

- Before the first decision, each drone's MDP is solved alone (ignoring the
  other drones) by finite-horizon value iteration on the supplied slip law,
  giving `Q[i][h](p, a)` and `V[i][h](p)` for every remaining horizon `h`.
  Proposals rank commands by `Q` and rollout cutoffs use `V`, both at the steps
  actually left. Rows are exact and sparse; it takes under 0.2 s on the demo maps.
- A joint proposal adds a risk surcharge for another active drone slipping
  into the chosen cell this step (only drones in the same layer when altitude
  is fixed); occupied and already reserved cells receive a large penalty.
- The leaf drops the goal value of a drone that drones frozen at their goals
  disconnect from its goal.
- Truncated softmax proposals sample among the cheapest commands; exploratory
  proposals and rollout noise choose uniformly among each drone's admissible
  commands (intended cell inside the grid, not an obstacle, not reserved).

`guidance='dijkstra'` selects the earlier risk-weighted Dijkstra cost-to-go
guidance (solver A); its complete results are archived in
`results/guidance_dijkstra/` for the paired comparison in the report. Value
guidance was adopted on the development set (round 3): it was not worse than
A in success or length, and on the evaluation suites it matches A's success
while reducing average penalised completion length on the pooled tested
conditions. The aggregate removes repeated conditions and uses execution-seed
cluster bootstrap intervals; this does not establish superiority on every suite.

The final root decision is anchored to the greedy (base-policy) action: another
action is executed only if its mean return exceeds the base action's by more
than one pooled standard error of the difference. This is a heuristic test,
not a calibrated significance test or a policy-improvement guarantee.
Progressive widening admits `ceil(0.5 * sqrt(N(s)+1))` actions to concentrate
simulations, without imposing a minimum visit count on every candidate.
The switching rule improves over the tested most-visited and largest-mean-Q
rules on S3, but search still has no consistent benefit over its guided base
policy across the evaluation suites.

The planner only accesses `get_config()`, `simulate(state, action)`, and
`num_actions` (property or method). All maps are derived from the public
configuration; it does not read the simulator's private action vectors,
distance field, RNG, live positions or time counter. Parameters were selected
on a separate development set (`environments/dev/`, `results/dev/`, three rounds,
2,880 episodes) before any reported evaluation ran.

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

`environments/manifest.json` lists 92 entries (some controls reuse a layout
with a different action mask): 18 named regression cases, 7 supplementary
end-to-end cases (solvable fixed-altitude single-drone maps, fixed-altitude
drones on different layers, a goal inside a teammate's only corridor),
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
python scripts/verify_equivalence.py --workers 12   # replay reported seed-0 episodes
python scripts/probe_risk_leaf.py --workers 12      # exploratory risk-charged leaf probe
python scripts/plot_risk_maps.py                    # B2 guidance heatmaps; no new episodes
python scripts/plot_scenario_gallery.py             # saved paths, timings and report appendix
python scripts/compute_ceilings.py                  # exact success ceilings (value iteration)
python scripts/verify_ceilings.py                   # extra corridor policy validation, 8,000 fixed seeds
```

Success-ceiling diagnostics optimise the probability of completing all goals, whereas the submitted planner optimises discounted reward. They are numerical finite-horizon dynamic programs, not evidence that the submitted policy is optimal. The original policy-validation batches remain in `results/ceilings.json`; `results/ceiling_validation.json` records an additional fixed corridor batch (seeds 2,000–9,999), its Wilson interval and source/configuration hashes. The initial 2,000-run corridor interval missed the DP value; the extension is a diagnostic check, separate from the 3,460 solver-evaluation episodes. The four-drone independent-policy comparison uses different sample sizes and does not establish a coordination benefit. Tests in `tests/test_ceilings.py` independently enumerate outcomes through the supplied simulator, covering boundaries, obstacles, shared cells and initially reached drones.

The scenario gallery reads stored seed-0 full-solver traces without running
the planner. It covers all 76 non-development manifest entries plus the
supplied example. Individual PNGs and a source/hash index are written to
`results/visualisations/`; the complete PDF and four sheets of twelve
representative cases are written to the report directory. Default-budget
demo timings are labelled separately from fixed-simulation experiments
with relaxed deadlines. Crosses mark unfinished drones' returned terminal
positions; raw collision targets are not logged. Single traces illustrate
behaviour rather than replace the repeated-run statistics.

The reported design has 3,460 episodes (360 regression, 160 generated,
540 factorial, 1,040 component, 400 budget, 160 control, 520 search-free
baseline, 280 supplementary) plus 2,880 development-set episodes in three rounds.
All 3,460 main evaluation episodes ran with the submitted planner (hash in each `*.meta.json`).
The earlier Dijkstra and development runs retain their recorded versions;
`provenance/planner_sources.json` maps all six planner hashes to exact source
files, including five historical snapshots in `provenance/planners/`.
Run `python scripts/verify_sources.py` to check this mapping.
`scripts/verify_equivalence.py` replays the main evaluation's stored seed-0 episodes with the
current code. Raw outcomes, seeds, search diagnostics and
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
