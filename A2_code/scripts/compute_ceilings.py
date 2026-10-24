"""Exact success ceilings for small cases by finite-horizon value iteration.

The supplied transition law is enumerated exactly (same distance transform, slip row,
out-of-grid renormalisation, obstacle and shared-cell collisions, frozen reached drones).
A ceiling is the largest probability, over all policies, of reaching every goal within
the step limit without collision. Each optimal policy is then run in the supplied
simulator to check the computed value. Four-drone joint problems are intractable; for
them, single-drone optima and their uncoordinated joint execution are reported.
"""

import itertools
import json
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from environment import make_env
from planner import action_vectors

EPISODES=2000


def transition_model(cfg):
    shape=tuple(cfg['grid_size']);vectors=action_vectors(cfg['change_altitude']);B=len(vectors)
    free=np.ones(shape,bool)
    for o in cfg['obstacle_cells'] or []:
        free[tuple(o)]=False
    distance=distance_transform_edt(free.astype(np.uint8)).astype(np.float32)
    cells=[c for c in itertools.product(*map(range,shape)) if free[c]]
    index={c:i for i,c in enumerate(cells)};n=len(cells)
    P=np.zeros((n,B,n+1))  # last outcome = crash into an obstacle
    for c in cells:
        success=distance[c]/(distance[c]+cfg['alpha'])
        targets=[tuple(np.add(c,v)) for v in vectors]
        valid=np.array([all(0<=t[k]<shape[k] for k in range(3)) for t in targets])
        for a in range(B):
            row=np.full(B,(1-success)/(B-1),np.float32);row[a]=success
            row[~valid]=0;row/=row.sum()
            for j,t in enumerate(targets):
                if row[j]>0:
                    P[index[c],a,index[t] if free[t] else n]+=row[j]
    return index,P,B


def single_drone(cfg,i,H):
    # Returns the ceiling and policy tables (tables[k][cell] with k steps left).
    index,P,B=transition_model(cfg);n=len(index);goal=index[tuple(cfg['goal_positions'][i])]
    V=np.zeros(n+1);V[goal]=1.;tables=[None]
    for _ in range(H):
        W=V.copy();W[goal]=1.;W[n]=0.
        Q=P@W;tables.append(Q.argmax(axis=1))
        V=np.append(Q.max(axis=1),0.);V[goal]=1.
    return float(V[index[tuple(cfg['start_positions'][i])]]),tables,index,B


def drone_pair(cfg,H):
    index,P,B=transition_model(cfg);n=len(index)
    g1,g2=(index[tuple(g)] for g in cfg['goal_positions'])
    s1,s2=(index[tuple(s)] for s in cfg['start_positions'])
    frozen={1:[np.zeros(n+1)],2:[np.zeros(n+1)]};frozen_tables={1:[None],2:[None]}
    for _ in range(H):  # one drone parked (and solid) on its goal, the other still flying
        for parked,goal,block in ((1,g2,g1),(2,g1,g2)):
            W=frozen[parked][-1].copy();W[goal]=1.;W[n]=0.;W[block]=0.
            Q=P@W;frozen_tables[parked].append(Q.argmax(axis=1))
            V=np.append(Q.max(axis=1),0.);V[goal]=1.;frozen[parked].append(V)
    V=np.zeros((n,n));tables=[None]
    for k in range(H):
        W=np.zeros((n+1,n+1));W[:n,:n]=V
        W[g1,:n]=frozen[1][k][:n];W[:n,g2]=frozen[2][k][:n];W[g1,g2]=1.
        np.fill_diagonal(W,0.);W[n,:]=0.;W[:,n]=0.
        Q=np.einsum('iaz,jbz->iajb',P,np.einsum('jbm,lm->jbl',P,W))
        flat=Q.transpose(0,2,1,3).reshape(n,n,B*B)
        tables.append(flat.argmax(axis=2));V=flat.max(axis=2);np.fill_diagonal(V,0.)
    if s1==g1 and s2==g2:
        value=1. if g1!=g2 else 0.
    elif s1==g1:
        value=frozen[1][H][s2]
    elif s2==g2:
        value=frozen[2][H][s1]
    else:
        value=V[s1,s2]
    return float(value),(tables,frozen_tables),index,B


def run_policy(path,H,choose,episodes=EPISODES,seed_start=0):
    wins=0
    for seed in range(seed_start,seed_start+episodes):
        env=make_env(path,seed);state=env.reset()
        if state[:,3].all():
            wins+=1
            continue
        for t in range(H):
            digits=choose(state,H-t)
            state,_,done,info=env.step(sum(int(d)*env._action_vectors.shape[0]**k for k,d in enumerate(digits)))
            if done:
                wins+=bool(info['success']) and not (info['num_collisions'] or info['num_vehicle_collisions'])
                break
    return wins/episodes


def main():
    results={}
    for name in ('supp_single_example_2d','supp_single_wallgap_2d','supp_single_slalom_2d','supp_single_clutter_2d'):
        path=ROOT/f'environments/{name}.yaml';cfg=yaml.safe_load(path.read_text());H=cfg['max_num_steps']
        value,tables,index,_=single_drone(cfg,0,H)
        simulated=run_policy(path,H,lambda s,k:[tables[k][index[tuple(s[0,:3])]]])
        results[name]=dict(ceiling=value,simulated_optimal=simulated,kind='exact single drone')
    # Two drones on different fixed layers never interact: the joint ceiling is the product.
    path=ROOT/'environments/supp_layers_cross_2d.yaml';cfg=yaml.safe_load(path.read_text());H=cfg['max_num_steps']
    parts=[single_drone(cfg,i,H) for i in range(2)]
    simulated=run_policy(path,H,lambda s,k:[0 if s[i,3] else parts[i][1][k][parts[i][2][tuple(s[i,:3])]] for i in range(2)])
    results['supp_layers_cross_2d']=dict(ceiling=float(np.prod([p[0] for p in parts])),simulated_optimal=simulated,
                                         kind='exact product (layers independent)')
    path=ROOT/'environments/supp_yield_corridor.yaml';cfg=yaml.safe_load(path.read_text());H=cfg['max_num_steps']
    value,(joint,parked),index,B=drone_pair(cfg,H)
    def choose(s,k):
        c1,c2=index[tuple(s[0,:3])],index[tuple(s[1,:3])]
        if s[0,3]:return [0,parked[1][k][c2]]
        if s[1,3]:return [parked[2][k][c1],0]
        flat=joint[k][c1,c2];return [flat//B,flat%B]
    results['supp_yield_corridor']=dict(ceiling=value,simulated_optimal=run_policy(path,H,choose),kind='exact joint pair')
    for name in ('random_15_N4','random_11_N4'):
        path=ROOT/f'environments/{name}.yaml';cfg=yaml.safe_load(path.read_text());H=cfg['max_num_steps']
        parts=[single_drone(cfg,i,H) for i in range(len(cfg['start_positions']))]
        simulated=run_policy(path,H,lambda s,k:[0 if s[i,3] else parts[i][1][k][parts[i][2][tuple(s[i,:3])]]
                                                 for i in range(len(s))],episodes=1000)
        results[name]=dict(single_drone_ceilings=[p[0] for p in parts],min_single=min(p[0] for p in parts),
                           uncoordinated_joint_success=simulated,kind='joint optimum intractable')
    (ROOT/'results/ceilings.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
