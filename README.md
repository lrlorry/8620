# COMP8620: intelligent robotics assignments

This repository contains the COMP8620 assignments, including the current A2
solver, completed experiments, report and submission archive. A1 material and
the initial A2 results are retained for comparison.

## Start here for an A2 Part B review

- [Assignment requirements](A2/a2.pdf): Part B questions, demo constraints and
  experimental-analysis requirements.
- [Solver](A2_code/planner.py): online MCTS, action widening, sampled outcomes,
  risk-aware guidance, rollouts and value backups.
- [Demo entry point](A2_code/run_u8076655.py) and
  [environment adapter](A2_code/environment.py).
- [Tests](A2_code/tests/test_solver.py).
- [Experiment design](A2_code/experiments.py),
  [scenario generator](A2_code/scripts/make_scenarios.py), and
  [analysis](A2_code/scripts/analyse_results.py).
- [Implementation and reproduction notes](A2_code/README.md) and
  [course-method alignment](A2_code/COURSE_ALIGNMENT.md).

## Read the results with their version context

- The completed evaluation has **3,300 episodes**: 360 regression, 160 generated
  layouts, 540 factorial, 880 component, 400 search-budget, 160 mechanism-control
  episodes, 520 search-free baseline episodes and 280 supplementary end-to-end
  episodes. The separate parameter study has
  **1,920 development episodes** in two rounds.
- Each suite retains its generating planner hash in `*.meta.json`. Two later
  edge-case fixes address fixed-altitude collision guidance and frozen-goal
  disconnection. [Equivalence validation](A2_code/results/planner_equivalence.json)
  records **137/137 identical seed-0 replays** with the current planner;
  this is not a claim to have rerun every evaluation episode.
- [Release validation](A2_code/results/release_validation.json) records 34 passing
  unit tests and four successful default-budget demo runs. The largest measured
  default decision took about 9.21 seconds on the development machine.
- The current [report PDF](A2/u8076655_A2_report.pdf),
  [LaTeX source](A2/u8076655_A2_report.tex) and
  [submission archive](A2/u8076655_A2_code.zip) include the revised study.
  [Package validation](A2/package_validation.json) records file-hash checks,
  isolated execution and tests, analysis regeneration and report compilation.
- [Archived initial results](archive/A2_v1_results/) contain the completed
  earlier study: 1,550 evaluation episodes, mostly using a 0.04-second planning
  allowance. The per-suite metadata records the earlier planner hash.
- The report finds that guidance carries most completion performance on the
  tested tasks; extra search has no statistically resolved benefit there.
  Local validation does not guarantee success on unknown marking scenarios or
  a hard real-time bound on different hardware.

## Earlier assignment

- [A1 report](A1/u8076655_A1_report.pdf) and
  [LaTeX source](A1/u8076655_A1_report.tex).
- [A1 implementation](A1_code/).

## Running the current A2 demo

From `A2_code/`, use a Python environment with the packages in
[`requirements.txt`](A2_code/requirements.txt):

```bash
python run_u8076655.py --config environments/demo_open_2d.yaml
python run_u8076655.py --config environments/demo_four_3d.yaml
python -m unittest discover -s tests -v
```

The normal demo uses a serial planner with a default 9.5-second per-decision
allowance. Research batches separately use fixed simulation caps and may run
independent episodes in worker processes. Their aggregate duration is not the
duration of a single demo, and concurrent batch timings are not standalone demo
benchmarks. See the current implementation for the exact stopping conditions.
