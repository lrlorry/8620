"""Draw the guidance maps used by the planner on two fixed-altitude single-drone cases.

Panels: breadth-first steps delta(p), departure risk rho(p) (probability that the next
move slips into an obstacle) and the single-drone value-iteration value V(p) that guides
the submitted solver. Routes are greedy proposals without slips; the last panel overlays
an executed episode of the submitted solver.
Map values come from planner.py; the executed path is read from a saved trace.
No stochastic episode is re-run.
"""

import json
import os
from pathlib import Path
import sys
import tempfile

os.environ.setdefault('MPLCONFIGDIR',str(Path(tempfile.gettempdir())/'comp8620-mpl'))
import matplotlib
matplotlib.use('Agg')
from matplotlib import patheffects
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from environment import make_env
from planner import MCTSPlanner

REPORT=next((p for p in (ROOT/'report',ROOT.parent/'A2')
             if (p/'u8076655_A2_report.tex').exists()),ROOT/'report')
INK,MUTED,OBSTACLE='#0b0b0b','#52514e','#3d3d3a'


def greedy_route(planner,state,limit=40):
    # Follow the deterministic greedy proposal as if every move succeeded.
    route=[tuple(state[0,:3])]
    state=state.copy()
    for step in range(min(limit,int(planner.cfg.max_num_steps))):
        if state[0,3]:
            break
        action=planner._proposal(state,randomize=False,remaining=planner.cfg.max_num_steps-step)
        state[0,:3]=state[0,:3]+planner.vectors[action]
        state[0,3]=int(np.array_equal(state[0,:3],planner.goals[0]))
        route.append(tuple(state[0,:3]))
    return route


def layer(values,blocked):
    grid=np.asarray(values,dtype=float)[:,:,0].T.copy()
    grid[blocked]=np.nan
    return grid


def panel(ax,grid,cmap,label,start,goal,route=None,style='-'):
    colours=plt.get_cmap(cmap).copy();colours.set_bad(OBSTACLE)
    low=min(0.,float(np.nanmin(grid)))
    high=float(np.nanmax(grid))
    if high<=low:high=low+1.
    image=ax.imshow(grid,origin='lower',cmap=colours,vmin=low,vmax=high)
    bar=plt.colorbar(image,ax=ax,fraction=.046,pad=.03)
    bar.ax.tick_params(labelsize=9,colors=MUTED);bar.set_label(label,fontsize=9,color=INK)
    if route is not None:
        xs,ys=[p[0] for p in route],[p[1] for p in route]
        halo=[patheffects.withStroke(linewidth=3.2,foreground='white')]
        ax.plot(xs,ys,style,color=INK,linewidth=1.4,marker='o',markersize=2.6,path_effects=halo)
    ax.plot(*start[:2],'o',color='white',markeredgecolor=INK,markersize=6)
    ax.plot(*goal[:2],'*',color='white',markeredgecolor=INK,markersize=10)
    ax.set_xticks([]);ax.set_yticks([])


def main():
    cases=[('supp_single_clutter_2d','Five obstacles\n$\\alpha=1$'),
           ('supp_single_wallgap_2d','Wall with one gap\n$\\alpha=0.5$')]
    plt.rcParams.update({'font.family':'serif','mathtext.fontset':'cm','font.size':10,'text.color':INK,
                         'axes.edgecolor':MUTED,'pdf.fonttype':42})
    fig,axes=plt.subplots(2,3,figsize=(7,4.3),
                          gridspec_kw={'height_ratios':[1,.7]},layout='constrained')
    for row,(name,title) in enumerate(cases):
        env=make_env(ROOT/f'environments/{name}.yaml',0);state=env.reset()
        risky,plain=MCTSPlanner(env),MCTSPlanner(env,guidance='dijkstra',risk_weight=0.)
        blocked=~risky.free[:,:,0].T
        start,goal=tuple(state[0,:3]),tuple(risky.goals[0])
        steps=layer(np.where(risky.maps[0]>=0,risky.maps[0],np.nan),blocked)
        panel(axes[row,0],steps,'Blues','steps',start,goal,
              greedy_route(plain,state),style='--')
        panel(axes[row,1],layer(risky.departure_risk,blocked),'Oranges','probability',start,goal)
        if row==0:
            route=greedy_route(risky,state)
        else:
            trace=json.loads((ROOT/f'results/trajectories/supplement_{name}_full_2000.json').read_text())
            route=[tuple(step['state'][0][:3]) for step in trace['trajectory']]+[tuple(trace['final_state'][0][:3])]
        horizon=len(risky.values[0])-1
        value=np.full(risky.free.shape,np.nan)
        value[risky.free]=risky.values[0][horizon][risky.value_index[risky.free]]
        panel(axes[row,2],layer(value,blocked),'Blues','value',start,goal,route)
        axes[row,0].set_ylabel(title,fontsize=10)
    for row,last in enumerate(['$V(p)$, value-guided route','$V(p)$, executed run']):
        for col,heading in enumerate(['$\\delta(p)$, risk-free route','$\\rho(p)$',last]):
            axes[row,col].set_title(heading,fontsize=10)
    (REPORT/'figures').mkdir(parents=True,exist_ok=True)
    fig.savefig(REPORT/'figures/risk_maps.pdf',bbox_inches='tight')
    fig.savefig(REPORT/'figures/risk_maps.png',dpi=200,bbox_inches='tight')
    print(REPORT/'figures/risk_maps.pdf')


if __name__=='__main__':main()
