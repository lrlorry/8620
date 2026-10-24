"""Replay every reported seed-0 episode with the current planner and compare it step by step.

Each reported suite ran with the planner hash recorded in results/*.meta.json. If the
code is changed later, these replays must still be identical before the results are reused.
"""

import os
for name in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name]='1'

import argparse
import csv
import hashlib
import json
from multiprocessing import get_context
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments import UNCAPPED
from run_u8076655 import run_episode

SUITES=['regression','random','factorial','components','budget','control']
RATIONALE=('Every reported suite records the planner hash that produced it; replaying the stored '
           'seed-0 episodes with the current planner must reproduce them step for step.')


def replay(task):
    trace,config,variant,cap,mask=task
    result=run_episode(ROOT/config,0,UNCAPPED,variant,mask,planner_options={'max_simulations':cap},
                       time_limit=UNCAPPED)
    stored=json.loads((ROOT/trace).read_text())['trajectory']
    return trace,json.loads(json.dumps(result['trajectory']))==stored


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=1)
    args=parser.parse_args()
    manifest={s['name']:s for s in json.loads((ROOT/'environments/manifest.json').read_text())}
    tasks,reported=[],set()
    for suite in SUITES:
        reported.add(json.loads((ROOT/f'results/{suite}.meta.json').read_text())['planner_sha256'])
        with (ROOT/f'results/{suite}.csv').open() as stream:
            for row in csv.DictReader(stream):
                if row['seed']=='0':
                    scenario=manifest[row['scenario']]
                    trace=f"results/trajectories/{suite}_{row['scenario']}_{row['variant']}_{row['simulation_cap']}.json"
                    tasks.append((trace,scenario['path'],row['variant'],int(row['simulation_cap']),scenario['mask']))
    with get_context('spawn').Pool(args.workers) as pool:
        outcomes=pool.map(replay,tasks,chunksize=1)
    mismatches=[trace for trace,same in outcomes if not same]
    result=dict(reported_planner_sha256=sorted(reported),
                current_planner_sha256=hashlib.sha256((ROOT/'planner.py').read_bytes()).hexdigest(),
                rationale=RATIONALE,replayed_episodes=len(outcomes),identical=len(outcomes)-len(mismatches),
                mismatches=mismatches)
    (ROOT/'results/planner_equivalence.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if mismatches:raise SystemExit('Replays differ from the reported episodes')


if __name__=='__main__':main()
