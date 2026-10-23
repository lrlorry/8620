"""Simulation-capped episodes, raw CSV, and resume support.

Each episode plans serially (one process, one thread). Independent episodes may
run in separate worker processes; search effort is fixed by a simulation cap so
results do not depend on machine load. Timing claims come from release_validation.py.
"""

import os
for name in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name] = '1'

import argparse
import csv
import hashlib
import json
from multiprocessing import get_context
from pathlib import Path
import platform
import sys
import time

import yaml

from run_u8076655 import run_episode

ROOT = Path(__file__).resolve().parent
UNCAPPED = 3600.  # Effectively no wall-clock cap; the simulation cap binds instead.
CORNERS = [f'factor_S{s}_A{a}_C{c}' for s in (5,20) for a in (6,26) for c in (0,5)]
BUDGET_TASKS = ['demo_open_2d','strong_wind','demo_four_3d','factor_S10_A26_C5']
# Parameter-selection arms, evaluated only on the separate development layouts.
_R1 = {'switch_z': None, 'widening': 1.5}   # round 1: most-visited choice, wide widening
_R2 = {'risk_weight': 1., 'widening': 1.5}  # round 2: anchored choice variants
TUNING_ROUNDS = {
    1: {'risk0': {**_R1, 'risk_weight': 0.}, 'risk0.3': {**_R1, 'risk_weight': .3},
        'risk1': {**_R1, 'risk_weight': 1.}, 'risk3': {**_R1, 'risk_weight': 3.},
        'risk1_c0.1': {**_R1, 'risk_weight': 1., 'exploration': .1},
        'risk1_c1': {**_R1, 'risk_weight': 1., 'exploration': 1.},
        'greedy_risk0': {**_R1, 'risk_weight': 0., 'max_simulations': 0},
        'greedy_risk1': {**_R1, 'risk_weight': 1., 'max_simulations': 0}},
    2: {'anchor_z1': {**_R2, 'switch_z': 1.}, 'anchor_z2': {**_R2, 'switch_z': 2.},
        'anchor_z1_eps0': {**_R2, 'switch_z': 1., 'rollout_epsilon': 0.},
        'anchor_z1_w0.5': {**_R2, 'switch_z': 1., 'widening': .5}},
}
# The adopted default (planner.py) is risk_weight=1, switch_z=1, widening=0.5: round-2 arm anchor_z1_w0.5.


def jobs_for(suite, scenarios, repetitions, simulations, tuning_round=1):
    """Yield (scenario, variant, simulation cap, seed, planner options)."""
    by_name = {s['name']: s for s in scenarios}
    if suite in ('regression','random','factorial','control'):
        for s in scenarios:
            if s['suite'] == suite:
                for seed in range(repetitions):
                    yield s, 'full', simulations, seed, {}
    elif suite == 'components':
        for name in CORNERS:
            for variant in ('no_guidance','no_leaf','no_widening','no_risk','visits_choice','heuristic'):
                # Uniform search mostly times out; ten seeds already resolve its large effect.
                for seed in range(min(repetitions, 10) if variant == 'no_guidance' else repetitions):
                    yield by_name[name], variant, 0 if variant == 'heuristic' else simulations, seed, {}
    elif suite == 'budget':
        for name in BUDGET_TASKS:
            for cap in (16, 64, 256, 1024, 2000):
                for seed in range(repetitions):
                    yield by_name[name], 'full', cap, seed, {}
    elif suite == 'supplement':
        # Supplementary end-to-end cases, each paired with its search-free base policy.
        for s in scenarios:
            if s['suite'] == 'supplement':
                for variant in ('full', 'heuristic'):
                    for seed in range(repetitions):
                        yield s, variant, 0 if variant == 'heuristic' else simulations, seed, {}
    elif suite == 'baseline':
        # Search-free base policy on the regression and generated suites, paired with their full runs.
        for s in scenarios:
            if s['suite'] in ('regression','random'):
                for seed in range(repetitions if s['suite'] == 'regression' else min(repetitions, 10)):
                    yield s, 'heuristic', 0, seed, {}
    elif suite == 'tuning':
        for s in scenarios:
            if s['suite'] == 'dev':
                for arm, options in TUNING_ROUNDS[tuning_round].items():
                    for seed in range(repetitions):
                        yield s, arm, options.get('max_simulations', simulations), seed, options
    else:
        raise ValueError(f'Unknown suite: {suite}')


def execute(task):
    suite, scenario, variant, cap, seed, options, trace_dir = task
    solver = variant if suite != 'tuning' else 'full'
    result = run_episode(ROOT/scenario['path'], seed, UNCAPPED, solver, scenario['mask'],
                         planner_options={**options, 'max_simulations': cap}, time_limit=UNCAPPED)
    if seed == 0 and trace_dir is not None:
        trace = Path(trace_dir)/f'{suite}_{scenario["name"]}_{variant}_{cap}.json'
        trace.write_text(json.dumps(result, indent=2)+'\n')
    horizon = yaml.safe_load((ROOT/scenario['path']).read_text())['max_num_steps']
    row = {k: v for k, v in result.items() if k not in ('trajectory','searches','final_state','config')}
    collisions = [step['info'] for step in result['trajectory']]
    row.update(scenario=scenario['name'], suite=suite, variant=variant, simulation_cap=cap,
               S=scenario.get('S',''), A=scenario.get('A',''), C=scenario.get('C',''),
               N=scenario.get('N', row['num_drones']), alpha=scenario.get('alpha',''),
               obstacle_collision=int(any(i['num_collisions'] for i in collisions)),
               vehicle_collision=int(any(i['num_vehicle_collisions'] for i in collisions)),
               horizon=horizon, penalized_steps=result['steps'] if result['success'] else horizon)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=['regression','random','factorial','components',
                                            'budget','control','baseline','supplement','tuning'], required=True)
    parser.add_argument('--repetitions', type=int, default=20)
    parser.add_argument('--simulations', type=int, default=2000)
    parser.add_argument('--workers', type=int, default=1)
    parser.add_argument('--round', type=int, default=1, help='Tuning round (development set only)')
    parser.add_argument('--output', required=True)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    scenarios = json.loads((ROOT/'environments/manifest.json').read_text())
    target = ROOT/args.output
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and not args.resume:
        raise FileExistsError(f'{target} exists; use --resume')
    seen = set()
    if target.exists():
        with target.open() as stream:
            for row in csv.DictReader(stream):
                seen.add((row['scenario'], row['variant'], int(row['simulation_cap']), int(row['seed'])))
    meta = dict(python=sys.version, platform=platform.platform(), processor=platform.processor(),
                planning='serial per episode', workers=args.workers, suite=args.suite,
                repetitions=args.repetitions, simulation_cap=args.simulations, wall_clock_cap=UNCAPPED,
                planner_sha256=hashlib.sha256((ROOT/'planner.py').read_bytes()).hexdigest(),
                upstream=json.loads((ROOT/'UPSTREAM.json').read_text()))
    metadata_path = target.with_suffix('.meta.json')
    if target.exists() and metadata_path.exists():
        previous = json.loads(metadata_path.read_text())
        if previous['planner_sha256'] != meta['planner_sha256']:
            raise ValueError('The planner changed; use a new results file instead of mixing versions')
    metadata_path.write_text(json.dumps(meta, indent=2)+'\n')
    trace_dir = target.parent/'trajectories'
    trace_dir.mkdir(exist_ok=True)
    tasks = [(args.suite, s, v, cap, seed, options, str(trace_dir))
             for s, v, cap, seed, options in jobs_for(args.suite, scenarios, args.repetitions, args.simulations, args.round)
             if (s['name'], v, cap, seed) not in seen]
    tasks.sort(key=lambda t: -t[3])  # Larger searches first keeps workers busy until the end.
    began, written = time.perf_counter(), 0
    fieldnames = None
    if target.exists() and target.stat().st_size:
        with target.open() as stream:
            fieldnames = next(csv.reader(stream))
    with target.open('a', newline='') as stream:
        writer = None if fieldnames is None else csv.DictWriter(stream, fieldnames=fieldnames)
        with get_context('spawn').Pool(args.workers) as pool:
            for row in pool.imap_unordered(execute, tasks):
                if writer is None:
                    writer = csv.DictWriter(stream, fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
                stream.flush()
                written += 1
                if written % 10 == 0 or written == len(tasks):
                    print(f'{args.suite}: {written}/{len(tasks)} episodes; '
                          f'elapsed={time.perf_counter()-began:.0f}s', flush=True)


if __name__ == '__main__':
    main()
