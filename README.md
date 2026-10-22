# COMP8620: intelligent robotics assignments

This repository is a **work-in-progress review snapshot** of the assignments.
It includes the current A2 implementation, its experimental design, the previous
completed results, and A1 material for comparison. It is not a declaration that
the new experiments or final submission have been completed.

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

- The current code contains revisions made after the initial code review.
  Its development-set parameter study is still in progress. Any committed
  `A2_code/results/dev/tuning.csv` is a partial snapshot of that running study.
- The revised design plans **2,280 evaluation episodes** and **1,280 separate
  development/tuning episodes**. These are planned counts, not completion claims.
  Check the actual CSVs and their metadata before drawing conclusions.
- [Archived initial results](archive/A2_v1_results/) contain the completed
  earlier study: 1,550 evaluation episodes, mostly using a 0.04-second planning
  allowance. The per-suite metadata records the earlier planner hash.
- The current [A2 report PDF](A2/u8076655_A2_report.pdf), its generated figures
  and tables, `package_validation.json`, and the existing submission ZIP are
  from that earlier report/release. They have **not yet been regenerated and
  validated for the revised solver and experiment design**. Some working
  documentation may already describe the planned revised design.
- Consequently, old timing, success-rate and package-verification results must
  not be treated as fresh validation of the current source code. This snapshot
  deliberately preserves both the draft and its provenance for review.

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
