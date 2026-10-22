"""Exhaust every positive-probability outcome on a free 2x2, two-drone grid.

This is deliberately a tiny finite-model check, not a claim to exhaust all
multi-drone environments. It tests every admissible nonterminal joint cell
configuration and every encoded action, including frozen reached drones.
"""

from itertools import permutations, product
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import yaml
from environment import HeadlessMultiDroneUnc
from planner import action_vectors, decode_action

ROOT=Path(__file__).resolve().parents[1]


class Choices:
    def __init__(self,values):self.values=iter(values)
    def choice(self,count,p):
        if not np.isfinite(p).all() or not np.isclose(p.sum(),1.,atol=1e-6):
            raise AssertionError('Invalid transition probability row')
        value=next(self.values)
        if p[value]<=0:raise AssertionError('Outcome has zero probability')
        return value


def main():
    cfg=dict(grid_size=[2,2,1],change_altitude=False,start_positions=[[0,0,0],[0,1,0]],
             goal_positions=[[1,0,0],[1,1,0]],obstacle_cells=[],alpha=.5,
             discount_factor=.98,step_cost=-1.,collision_penalty=-100.,goal_reward=100.,max_num_steps=20)
    goals=np.asarray(cfg['goal_positions']);moves=action_vectors(False)
    states,actions,outcomes=0,0,0
    with TemporaryDirectory() as temporary:
        path=Path(temporary)/'tiny.yaml';path.write_text(yaml.safe_dump(cfg))
        env=HeadlessMultiDroneUnc(path);env.reset()
        for cells in permutations(list(product(range(2),range(2),[0])),2):
            positions=np.asarray(cells,dtype=np.int32)
            reached=np.all(positions==goals,axis=1)
            if reached.all():continue
            state=np.column_stack([positions,reached.astype(np.int32)])
            original=state.copy();states+=1
            for action in range(64):
                digits=decode_action(action,8,2);supports=[]
                for i in range(2):
                    supports.append([j for j,move in enumerate(moves)
                                     if np.all(positions[i]+move>=0)
                                     and np.all(positions[i]+move<np.array([2,2,1]))
                                     and not(reached[i] and j==digits[i])])
                actions+=1
                for choices in product(*supports):
                    env.rng=Choices(choices)
                    successor,reward,done,info=env.simulate(state,action)
                    assert np.array_equal(state,original), 'Input state was modified'
                    assert np.all(successor[:,:3]>=0) and np.all(successor[:,:3]<[2,2,1])
                    assert np.all(successor[:,3]>=state[:,3]), 'Reached flag decreased'
                    assert np.array_equal(successor[reached,:3],positions[reached]), 'Reached drone moved'
                    shared=np.array_equal(successor[0,:3],successor[1,:3])
                    assert info['num_vehicle_collisions']==(2 if shared else 0)
                    assert bool(done)==(shared or bool(successor[:,3].all()))
                    assert info['num_collisions']==0
                    expected=-2.-100.*(2 if shared else 0)+100.*int((successor[:,3]-state[:,3]).sum())
                    assert reward==expected
                    outcomes+=1
    result=dict(joint_states=states,state_action_pairs=actions,positive_probability_outcomes=outcomes,
                passed=True,scope='free 2x2x1 grid, two drones, all 64 actions, all admissible nonterminal states')
    (ROOT/'results/small_model_verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
