"""Validate the corridor success DP with a fixed additional 8,000 seeds.

The original 2,000-run result is retained. Seeds 2,000--9,999 form an
additional, disjoint diagnostic batch; this is not a solver evaluation.
"""

import hashlib
import json
from pathlib import Path

import yaml

from compute_ceilings import ROOT, drone_pair, run_policy
from analyse_results import wilson


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    name = 'supp_yield_corridor'
    path = ROOT/'environments'/f'{name}.yaml'
    cfg = yaml.safe_load(path.read_text())
    stored = ROOT/'results/ceilings.json'
    old = json.loads(stored.read_text())[name]
    horizon = cfg['max_num_steps']
    value, (joint, parked), index, base = drone_pair(cfg, horizon)
    assert abs(value-old['ceiling']) < 1e-10

    def choose(state, remaining):
        c1, c2 = (index[tuple(row[:3])] for row in state)
        if state[0, 3]:
            return [0, parked[1][remaining][c2]]
        if state[1, 3]:
            return [parked[2][remaining][c1], 0]
        action = joint[remaining][c1, c2]
        return [action//base, action % base]

    print('Checking fixed extra seeds 2,000--9,999 against corridor DP.', flush=True)
    rate = run_policy(path, horizon, choose, episodes=8000, seed_start=2000)
    extra = round(rate*8000)
    original = round(old['simulated_optimal']*2000)
    interval = wilson(original+extra, 10000)
    result = dict(
        scenario=name, horizon=horizon, dp_maximum=value,
        original=dict(successes=original, episodes=2000, seed_start=0, seed_end=1999),
        additional=dict(successes=extra, episodes=8000, seed_start=2000, seed_end=9999),
        combined=interval,
        dp_inside_wilson_95=bool(interval['lower'] <= value <= interval['upper']),
        source_hashes={
            'config': digest(path),
            'ceilings': digest(stored),
            'compute_ceilings': digest(ROOT/'scripts/compute_ceilings.py'),
            'simulator': digest(ROOT/'multi_drone.py'),
            'validation_script': digest(Path(__file__)),
        },
        scope='Fixed additional policy-validation batch, separate from MCTS solver experiments.',
    )
    target = ROOT/'results/ceiling_validation.json'
    target.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
