"""Exploratory probe: does charging collision risk in the leaf estimate help?

The leaf estimate charges no collision risk, so search can postpone an unavoidable
risky passage beyond its horizon. This probe subtracts the risk part of the
risk-weighted cost-to-go, |c| * (J_i - delta_i) per active drone, from the leaf value
and compares it with the submitted planner (500 simulations per decision, seeds
0-9). It is not part of the submitted solver.
"""

import os
for name in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name]='1'

import argparse
import json
import math
from multiprocessing import get_context
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
TASKS=['supp_single_wallgap_2d','supp_single_slalom_2d','supp_single_clutter_2d','supp_yield_corridor',
       'narrow_passage','strong_wind','demo_four_3d','factor_S10_A6_C5']


def risk_charged_leaf(planner_class):
    original=planner_class._leaf
    def _leaf(self,state,remaining):
        value=original(self,state,remaining)
        if not self.leaf_heuristic or remaining<=0 or self.costs is None:
            return value
        extra=0.
        for i in range(self.count):
            if state[i,3] or self._cut_off(i,state):
                continue
            d,J=self._distance(i,state[i,:3]),self._cost(i,state[i,:3])
            if math.isfinite(d) and math.isfinite(J):
                extra+=J-d
        return value-abs(self.cfg.step_cost)*extra/self.scale
    planner_class._leaf=_leaf


def run(task):
    name,seed,arm=task
    import planner
    if arm=='riskleaf':
        risk_charged_leaf(planner.MCTSPlanner)
    from run_u8076655 import run_episode
    result=run_episode(ROOT/f'environments/{name}.yaml',seed,3600.,'full','full',
                       planner_options={'max_simulations':500},time_limit=3600.)
    return dict(cfg=f'{name}.yaml',seed=seed,arm=arm,term=result['termination'],steps=result['steps'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=1)
    parser.add_argument('--output',default='results/probes/risk_leaf_probe.jsonl')
    args=parser.parse_args()
    tasks=[(name,seed,arm) for name in TASKS for seed in range(10) for arm in ('base','riskleaf')]
    # Fresh worker per task: the patched class must not leak into 'base' episodes.
    with get_context('spawn').Pool(args.workers,maxtasksperchild=1) as pool:
        rows=pool.map(run,tasks,chunksize=1)
    target=ROOT/args.output
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    for arm in ('base','riskleaf'):
        print(arm,sum(r['arm']==arm and r['term']=='success' for r in rows),'/',sum(r['arm']==arm for r in rows))


if __name__=='__main__':main()
