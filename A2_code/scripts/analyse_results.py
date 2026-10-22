"""Summarize measured episodes and build report tables/figures.

The sampling unit is an episode. Seeds are reused across conditions, so factor
and component effects are computed on paired seed blocks, not pooled IID trials.
"""

from collections import defaultdict
import csv
import json
import math
import os
from pathlib import Path
import tempfile

os.environ.setdefault('MPLCONFIGDIR',str(Path(tempfile.gettempdir())/'comp8620-mpl'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

ROOT=Path(__file__).resolve().parents[1]
REPORT=next((p for p in (ROOT/'report',ROOT.parent/'A2')
             if (p/'u8076655_A2_report.tex').exists()),ROOT/'report')
TABLES=REPORT/'tables'
FIGURES=REPORT/'figures'
CHOSEN='risk1'  # development-set arm adopted as the solver default
EXPECTED={'regression':360,'random':160,'factorial':540,'components':720,'budget':400,'control':100}
VARIANTS=['no_guidance','no_leaf','no_widening','no_risk','heuristic']
NAMES={'full':'Full solver','no_guidance':'Uniform proposals/rollouts','no_leaf':'Zero leaf value',
       'no_widening':'No progressive widening','no_risk':'Risk-free guidance','heuristic':'Guided policy, no search'}
SERIES=['#2a78d6','#eb6834','#1baf7a','#eda100']  # validated categorical slots 1-4 (light surface)
INK,MUTED='#0b0b0b','#52514e'
INT_FIELDS=('seed','steps','simulations','generative_calls','num_actions','num_drones','obstacles',
            'simulation_cap','obstacle_collision','vehicle_collision','horizon')
FLOAT_FIELDS=('penalized_steps','discounted_return','setup_seconds','planning_seconds','mean_plan_seconds',
              'max_plan_seconds','mean_root_actions','mean_top_share','greedy_fraction','mean_new_depth')


def interval(values):
    values=np.asarray(list(values),dtype=float)
    mean=float(values.mean())
    half=float(stats.t.ppf(.975,len(values)-1)*values.std(ddof=1)/math.sqrt(len(values))) if len(values)>1 else 0.
    return dict(mean=mean,lower=mean-half,upper=mean+half,half=half,n=len(values))


def wilson(successes,n):
    z=stats.norm.ppf(.975);p=successes/n;d=1+z*z/n
    center=(p+z*z/(2*n))/d
    half=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d
    return dict(mean=p,lower=center-half,upper=center+half,n=n,successes=successes)


def read(suite,expected=None,folder='results'):
    with (ROOT/folder/f'{suite}.csv').open() as f:
        rows=list(csv.DictReader(f))
    if expected is not None and len(rows)!=expected:
        raise ValueError(f'{suite}: expected {expected} episodes, found {len(rows)}; wait for the run to finish')
    seen=set()
    for row in rows:
        key=row['scenario'],row['variant'],row['simulation_cap'],row['seed']
        if key in seen:raise ValueError(f'Duplicate episode {key}')
        seen.add(key)
        row['success']=row['success']=='True'
        for k in INT_FIELDS:row[k]=int(row[k])
        for k in FLOAT_FIELDS:row[k]=float(row[k])
        for k in ('S','A','C','N'):row[k]=int(row[k]) if row[k] else None
        row['alpha']=float(row['alpha']) if row['alpha'] else None
        row['calls_per_decision']=row['generative_calls']/row['steps'] if row['steps'] else 0.
    return rows


def summarize(rows):
    successes=[r for r in rows if r['success']]
    result={'success':wilson(len(successes),len(rows)),
            'collisions':sum(r['termination']=='collision' for r in rows),
            'obstacle_collisions':sum(r['obstacle_collision'] for r in rows),
            'vehicle_collisions':sum(r['vehicle_collision'] for r in rows),
            'timeouts':sum(r['termination']=='timeout' for r in rows)}
    for name in ('penalized_steps','discounted_return','calls_per_decision','mean_root_actions',
                 'mean_top_share','mean_new_depth','mean_plan_seconds','setup_seconds'):
        result[name]=interval(r[name] for r in rows)
    result['success_steps']=interval(r['steps'] for r in successes) if successes else None
    result['max_plan_seconds']=max(r['max_plan_seconds'] for r in rows)
    return result


def grouped(rows,fields):
    groups=defaultdict(list)
    for row in rows:groups[tuple(row[k] for k in fields)].append(row)
    return groups


def fmt(value,digits=1):
    return f"{value['mean']:.{digits}f} $\\pm$ {value['half']:.{digits}f}"


def ci(value,digits=1,scale=1.):
    return f"{scale*value['mean']:+.{digits}f} [{scale*value['lower']:+.{digits}f}, {scale*value['upper']:+.{digits}f}]"


def count(summary):
    return f"{summary['success']['successes']}/{summary['success']['n']}"


def latex_rows(name,rows):
    body='\n'.join(' & '.join(map(str,row))+r' \\' for row in rows)
    (TABLES/f'{name}.tex').write_text('\\newcommand{\\'+name.title().replace('_','')+'}{%\n'+body+'\n}\n')


class Numbers:
    """Values quoted in the prose (\\val{key}), so the text always matches the data."""
    def __init__(self):self.values={}
    def __setitem__(self,name,value):self.values[name]=value
    def write(self):
        lines=['\\providecommand{\\val}[1]{\\ifcsname num@#1\\endcsname\\csname num@#1\\endcsname'
               '\\else\\textbf{??#1??}\\fi}']
        lines+=[f'\\expandafter\\def\\csname num@{name}\\endcsname{{{value}}}' for name,value in sorted(self.values.items())]
        (TABLES/'numbers.tex').write_text('\n'.join(lines)+'\n')


def seed_means(rows,metric,seeds):
    blocks=defaultdict(list)
    for r in rows:blocks[r['seed']].append(r[metric])
    return np.array([np.mean(blocks[s]) for s in seeds],dtype=float)


def metric_of(row,metric):
    return 100.*row['success'] if metric=='success' else row[metric]


def analyse_factorial(rows,numbers,summary):
    seeds=range(20)
    table=[]
    for (s,a,c),cell in sorted(grouped(rows,('S','A','C')).items()):
        x=summarize(cell)
        table.append([s,a*a,c,count(x),fmt(x['penalized_steps']),fmt(x['discounted_return']),
                      f"{x['calls_per_decision']['mean']:,.0f}"])
    latex_rows('factorial_rows',table)
    levels={'S':[5,10,20],'A':[6,18,26],'C':[0,2,5]}
    marginals,effects={},{}
    for metric in ('penalized_steps','success'):
        marginals[metric],effects[metric]={},{}
        for factor,values in levels.items():
            blocks={}
            for level in values:
                subset=[r for r in rows if r[factor]==level]
                per_seed=defaultdict(list)
                for r in subset:per_seed[r['seed']].append(metric_of(r,metric))
                blocks[level]=np.array([np.mean(per_seed[s]) for s in seeds])
                marginals[metric].setdefault(factor,{})[level]=interval(blocks[level])
            deltas=blocks[values[-1]]-blocks[values[0]]
            effects[metric][factor]={**interval(deltas),'paired':deltas.tolist()}
    pairwise={f'{x}-{y}':interval(np.array(effects['penalized_steps'][x]['paired'])-effects['penalized_steps'][y]['paired'])
              for x,y in [('C','S'),('C','A'),('A','S')]}
    interactions={}
    for f,g in [('A','C'),('S','C'),('S','A')]:
        lo_f,hi_f,lo_g,hi_g=levels[f][0],levels[f][-1],levels[g][0],levels[g][-1]
        def cell(fv,gv):
            return seed_means([r for r in rows if r[f]==fv and r[g]==gv],'penalized_steps',seeds)
        interactions[f+'x'+g]=interval((cell(hi_f,hi_g)-cell(hi_f,lo_g))-(cell(lo_f,hi_g)-cell(lo_f,lo_g)))
    label={'S':'State space ($n_y$ 5$\\to$20)','A':'Action space ($|A|$ 36$\\to$676)','C':'Clutter ($|O|$ 0$\\to$5)'}
    latex_rows('factor_effect_rows',[[label[k],ci(effects['penalized_steps'][k]),ci(effects['success'][k])]
                                     for k in ('S','A','C')])
    for k in ('S','A','C'):
        e=effects['penalized_steps'][k];p=effects['success'][k]
        numbers[f'Eff{k}L']=f"{e['mean']:+.1f}";numbers[f'Eff{k}Lci']=f"[{e['lower']:+.1f}, {e['upper']:+.1f}]"
        numbers[f'Eff{k}P']=f"{p['mean']:+.1f}";numbers[f'Eff{k}Pci']=f"[{p['lower']:+.1f}, {p['upper']:+.1f}]"
    for k,v in pairwise.items():
        numbers['Pair'+k.replace('-','')]=f"{v['mean']:+.1f}";numbers['Pair'+k.replace('-','')+'ci']=f"[{v['lower']:+.1f}, {v['upper']:+.1f}]"
    for k,v in interactions.items():
        numbers['Int'+k.replace('x','')]=f"{v['mean']:+.1f}";numbers['Int'+k.replace('x','')+'ci']=f"[{v['lower']:+.1f}, {v['upper']:+.1f}]"
    overall=summarize(rows)
    numbers['FactSuccess']=f"{100*overall['success']['mean']:.1f}"
    numbers['FactCollisions']=overall['collisions'];numbers['FactObstacle']=overall['obstacle_collisions']
    numbers['FactVehicle']=overall['vehicle_collisions'];numbers['FactTimeouts']=overall['timeouts']
    for c in (0,5):
        x=summarize([r for r in rows if r['C']==c])
        numbers[f'CallsC{c}']=f"{x['calls_per_decision']['mean']:,.0f}"
        numbers[f'RootC{c}']=f"{x['mean_root_actions']['mean']:.1f}"
    for a in (6,26):
        x=summarize([r for r in rows if r['A']==a])
        numbers[f'RootA{a}']=f"{x['mean_root_actions']['mean']:.1f}";numbers[f'TopA{a}']=f"{100*x['mean_top_share']['mean']:.1f}"
    for s in (5,20):
        x=summarize([r for r in rows if r['S']==s])
        numbers[f'SetupS{s}']=f"{1000*x['setup_seconds']['mean']:.0f}"
    summary['factorial']=dict(effects=effects,marginals=marginals,pairwise=pairwise,interactions=interactions)
    return marginals


def analyse_components(rows,full_rows,numbers,summary):
    corners=[f'factor_S{s}_A{a}_C{c}' for s in (5,20) for a in (6,26) for c in (0,5)]
    full={(r['scenario'],r['seed']):r for r in full_rows if r['scenario'] in corners}
    by=grouped(rows,('variant','scenario'))
    level={'S':(5,20),'A':(6,26),'C':(0,5)}
    effects={};table=[];raw=[]
    full_cells=[summarize([full[c,s] for s in range(20)]) for c in corners]
    raw.append([NAMES['full']]+[count(x) for x in full_cells])
    for variant in VARIANTS:
        cells={c:{r['seed']:r for r in by[variant,c]} for c in corners}
        seeds=sorted(cells[corners[0]])
        raw.append([NAMES[variant]]+[count(summarize(list(cells[c].values()))) for c in corners])
        delta={c:np.array([cells[c][s]['penalized_steps']-full[c,s]['penalized_steps'] for s in seeds]) for c in corners}
        dsucc={c:np.array([100.*(cells[c][s]['success']-full[c,s]['success']) for s in seeds]) for c in corners}
        result={'overall':interval(np.mean([delta[c] for c in corners],axis=0)),
                'overall_success':interval(np.mean([dsucc[c] for c in corners],axis=0))}
        cells_row=[NAMES[variant],ci(result['overall']),ci(result['overall_success'])]
        for f,(lo,hi) in level.items():
            low=np.mean([delta[c] for c in corners if f'_{f}{lo}_' in c+'_'],axis=0)
            high=np.mean([delta[c] for c in corners if f'_{f}{hi}_' in c+'_'],axis=0)
            result[f]={'low':interval(low),'high':interval(high),'interaction':interval(high-low)}
            mark='$^\\ast$' if result[f]['interaction']['lower']>0 or result[f]['interaction']['upper']<0 else ''
            cells_row.append(f"{low.mean():+.1f} / {high.mean():+.1f}{mark}")
        effects[variant]=result;table.append(cells_row)
        numbers['Comp'+variant.replace('_','').title()]=f"{result['overall']['mean']:+.1f}"
        numbers['Comp'+variant.replace('_','').title()+'ci']=f"[{result['overall']['lower']:+.1f}, {result['overall']['upper']:+.1f}]"
        numbers['Comp'+variant.replace('_','').title()+'P']=f"{result['overall_success']['mean']:+.1f}"
        for f in level:
            i=result[f]['interaction']
            numbers['Comp'+variant.replace('_','').title()+f+'lo']=f"{result[f]['low']['mean']:+.1f}"
            numbers['Comp'+variant.replace('_','').title()+f+'hi']=f"{result[f]['high']['mean']:+.1f}"
            numbers['Comp'+variant.replace('_','').title()+f+'int']=f"[{i['lower']:+.1f}, {i['upper']:+.1f}]"
    latex_rows('component_rows',table)
    latex_rows('component_raw_rows',raw)
    summary['components']=effects
    return effects


def analyse_budget(rows,numbers,summary):
    table=[];curves={}
    short={'demo_open_2d':'Open 2D (1)','strong_wind':'Strong wind (1)','demo_four_3d':'Four 3D (4)',
           'factor_S10_A26_C5':'Factorial S10/A26/C5 (2)'}
    for (task,cap),cell in sorted(grouped(rows,('scenario','simulation_cap')).items(),
                                  key=lambda kv:(list(short).index(kv[0][0]),kv[0][1])):
        x=summarize(cell)
        curves.setdefault(task,[]).append((cap,x))
        table.append([short[task],f'{cap:,}',count(x),fmt(x['penalized_steps']),
                      f"{x['mean_root_actions']['mean']:.1f}",f"{x['mean_new_depth']['mean']:.2f}"])
        numbers['Budget'+''.join(ch for ch in task.title() if ch.isalnum())+str(cap)]=count(x)
    latex_rows('budget_rows',table)
    summary['budget']={t:{str(c):x for c,x in v} for t,v in curves.items()}
    return curves,short


def analyse_controls(control,factorial,numbers,summary):
    def cell(rows,name):return {r['seed']:r for r in rows if r['scenario']==name}
    out={};table=[]
    for alpha,prefix,rows in [(.25,'factor_S10_A26_C',factorial),(.01,'lownoise_C',control)]:
        cells={c:cell(rows,f'{prefix}{c}') for c in (0,2,5)}
        for c in (0,2,5):
            x=summarize(list(cells[c].values()))
            table.append([f'$\\alpha={alpha:g}$',c,count(x),fmt(x['penalized_steps']),
                          x['obstacle_collisions'],x['vehicle_collisions']])
        out[alpha]=interval([cells[5][s]['penalized_steps']-cells[0][s]['penalized_steps'] for s in range(20)])
        numbers['Clutter'+('Noisy' if alpha>.1 else 'Quiet')]=f"{out[alpha]['mean']:+.1f}"
        numbers['Clutter'+('Noisy' if alpha>.1 else 'Quiet')+'ci']=f"[{out[alpha]['lower']:+.1f}, {out[alpha]['upper']:+.1f}]"
    latex_rows('clutter_control_rows',table)
    branch=[];copies={1:cell(factorial,'factor_S10_A26_C2'),2:cell(control,'branch_x2'),4:cell(control,'branch_x4')}
    for k,cells in copies.items():
        x=summarize(list(cells.values()))
        branch.append([f'$\\times{k}$',f'{676*k*k:,}',count(x),fmt(x['penalized_steps']),
                       f"{x['mean_root_actions']['mean']:.1f}",f"{100*x['mean_top_share']['mean']:.1f}"])
    out['branch']=interval([copies[4][s]['penalized_steps']-copies[1][s]['penalized_steps'] for s in range(20)])
    numbers['Branch']=f"{out['branch']['mean']:+.1f}";numbers['Branchci']=f"[{out['branch']['lower']:+.1f}, {out['branch']['upper']:+.1f}]"
    latex_rows('branch_control_rows',branch)
    summary['controls']=out


def analyse_random(rows,numbers,summary):
    table=[]
    for label,field,values in [('Drones $N$','N',(1,2,3,4)),('Noise $\\alpha$','alpha',(.05,.25,1.,2.))]:
        for v in values:
            x=summarize([r for r in rows if r[field]==v])
            s=x['success']
            table.append([label if v==values[0] else '',f'{v:g}',count(x),
                          f"[{100*s['lower']:.0f}, {100*s['upper']:.0f}]",fmt(x['penalized_steps']),
                          f"{x['mean_new_depth']['mean']:.2f}"])
            numbers[f'Rand{field.title()}{str(v).replace(".","")}']=count(x)
    latex_rows('random_rows',table)
    for n in (1,4):
        x=summarize([r for r in rows if r['N']==n])
        numbers[f'DepthN{n}']=f"{x['mean_new_depth']['mean']:.2f}";numbers[f'TopN{n}']=f"{100*x['mean_top_share']['mean']:.1f}"
    summary['random']={'overall':summarize(rows)}


def analyse_regression(rows,numbers,summary):
    table=[]
    for (case,),cell in grouped(rows,('scenario',)).items():
        x=summarize(cell)
        table.append([r'\texttt{'+case.replace('_',r'\_')+'}',count(x),x['obstacle_collisions'],
                      x['vehicle_collisions'],fmt(x['penalized_steps']),f"{x['calls_per_decision']['mean']:,.0f}"])
        if case in ('demo_open_2d','demo_four_3d','strong_wind','altitude_detour','narrow_passage'):
            numbers['Reg'+''.join(ch for ch in case.title() if ch.isalnum())]=count(x)
    latex_rows('regression_rows',table)


def analyse_tuning(numbers,summary):
    path=ROOT/'results/dev/tuning.csv'
    if not path.exists():return
    rows=read('tuning',folder='results/dev')
    reference={(r['scenario'],r['seed']):r for r in rows if r['variant']==CHOSEN}
    table=[]
    for arm,cell in grouped(rows,('variant',)).items():
        x=summarize(cell)
        deltas=[]
        for s in sorted({r['seed'] for r in cell}):
            mine=[r for r in cell if r['seed']==s]
            deltas.append(np.mean([r['penalized_steps']-reference[r['scenario'],s]['penalized_steps'] for r in mine]))
        d=interval(deltas)
        table.append([arm[0].replace('_',r'\_'),count(x),x['obstacle_collisions'],x['vehicle_collisions'],
                      fmt(x['penalized_steps']),'---' if arm[0]==CHOSEN else ci(d)])
        numbers['Tune'+''.join(ch for ch in arm[0].title() if ch.isalnum())]=count(x)
    order=['risk0','risk0.3','risk1','risk3','risk1_c0.1','risk1_c1','greedy_risk0','greedy_risk1']
    table.sort(key=lambda row:order.index(row[0].replace(r'\_','_')))
    latex_rows('tuning_rows',table)


def figures(marginals,components,curves,short):
    plt.rcParams.update({'font.family':'serif','mathtext.fontset':'cm','font.size':9,'axes.edgecolor':MUTED,
                         'axes.labelcolor':INK,'xtick.color':MUTED,'ytick.color':MUTED,'text.color':INK,
                         'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
    fig,axes=plt.subplots(2,3,figsize=(7,4.2),sharey='row',layout='constrained')
    for col,(factor,levels,title,xlabel) in enumerate([
            ('S',[5,10,20],'State space',r'Grid height $n_y$'),
            ('A',[6,18,26],'Action space',r'Joint actions $|A|$'),
            ('C',[0,2,5],'Clutter','Obstacle cells')]):
        x=[v*v for v in levels] if factor=='A' else levels
        for row,metric in enumerate(('success','penalized_steps')):
            values=[marginals[metric][factor][level] for level in levels]
            ax=axes[row,col]
            ax.errorbar(x,[v['mean'] for v in values],yerr=[v['half'] for v in values],color=SERIES[0],
                        marker='o',markersize=5,linewidth=2,capsize=3)
            if factor=='A':ax.set_xscale('log')
            ax.set_xticks(x,[str(v) for v in x]);ax.minorticks_off();ax.grid(axis='y',alpha=.25)
            if row==0:ax.set_title(title)
            if row==1:ax.set_xlabel(xlabel)
    axes[0,0].set_ylabel('Success rate (%)');axes[1,0].set_ylabel('Penalised steps $L$')
    fig.savefig(FIGURES/'complexity_effects.pdf',bbox_inches='tight');fig.savefig(FIGURES/'complexity_effects.png',dpi=200,bbox_inches='tight');plt.close(fig)

    fig,axes=plt.subplots(1,3,figsize=(7,2.9),sharey=True,layout='constrained')
    y=np.arange(len(VARIANTS))
    for ax,(factor,(lo,hi)) in zip(axes,[('S',('$n_y=5$','$n_y=20$')),('A',('$|A|=36$','$|A|=676$')),('C',('$|O|=0$','$|O|=5$'))]):
        for k,(level,label,marker,offset) in enumerate([('low',lo,'o',-.15),('high',hi,'s',.15)]):
            values=[components[v][factor][level] for v in VARIANTS]
            ax.errorbar([v['mean'] for v in values],y+offset,xerr=[v['half'] for v in values],fmt=marker,
                        color=SERIES[k],markersize=5,capsize=2,linewidth=1.5,label=label,
                        markerfacecolor='white' if k==0 else SERIES[k])
        ax.axvline(0,color=MUTED,linewidth=.8);ax.grid(axis='x',alpha=.25)
        ax.legend(frameon=False,fontsize=7.5,loc='lower right')
        ax.set_title({'S':'State space','A':'Action space','C':'Clutter'}[factor])
        ax.set_xlabel(r'$\Delta L$ vs full solver')
    axes[0].set_yticks(y,[NAMES[v] for v in VARIANTS]);axes[0].invert_yaxis()
    fig.savefig(FIGURES/'component_effects.pdf',bbox_inches='tight');fig.savefig(FIGURES/'component_effects.png',dpi=200,bbox_inches='tight');plt.close(fig)

    fig,axes=plt.subplots(1,2,figsize=(7,2.7),layout='constrained')
    markers=['o','s','^','D']
    for k,(task,points) in enumerate(curves.items()):
        caps=[c for c,_ in points]
        success=[100*x['success']['mean'] for _,x in points]
        low=[100*(x['success']['mean']-x['success']['lower']) for _,x in points]
        high=[100*(x['success']['upper']-x['success']['mean']) for _,x in points]
        axes[0].errorbar(caps,success,yerr=[low,high],color=SERIES[k],marker=markers[k],markersize=5,
                         linewidth=2,capsize=2,label=short[task])
        axes[1].errorbar(caps,[x['penalized_steps']['mean'] for _,x in points],
                         yerr=[x['penalized_steps']['half'] for _,x in points],color=SERIES[k],
                         marker=markers[k],markersize=5,linewidth=2,capsize=2,label=short[task])
    for ax,label in zip(axes,('Success rate (%)','Penalised steps $L$')):
        ax.set_xscale('log',base=2);ax.set_xticks([16,64,256,1024,2000],['16','64','256','1024','2000'])
        ax.minorticks_off();ax.grid(axis='y',alpha=.25);ax.set_xlabel('Simulations per decision');ax.set_ylabel(label)
    axes[0].legend(frameon=False,fontsize=7.5,loc='lower right')
    fig.savefig(FIGURES/'budget_curve.pdf',bbox_inches='tight');fig.savefig(FIGURES/'budget_curve.png',dpi=200,bbox_inches='tight');plt.close(fig)


def main():
    TABLES.mkdir(parents=True,exist_ok=True);FIGURES.mkdir(parents=True,exist_ok=True)
    suites={name:read(name,n) for name,n in EXPECTED.items()}
    numbers=Numbers();summary={}
    for suite,rows in suites.items():
        summary[suite+'_cells']={'|'.join(map(str,k)):summarize(g) for k,g in
                                 grouped(rows,('scenario','variant','simulation_cap')).items()}
    marginals=analyse_factorial(suites['factorial'],numbers,summary)
    components=analyse_components(suites['components'],suites['factorial'],numbers,summary)
    curves,short=analyse_budget(suites['budget'],numbers,summary)
    analyse_controls(suites['control'],suites['factorial'],numbers,summary)
    analyse_random(suites['random'],numbers,summary)
    analyse_regression(suites['regression'],numbers,summary)
    analyse_tuning(numbers,summary)
    total=sum(map(len,suites.values()))
    numbers['TotalEpisodes']=f'{total:,}';summary['total_episodes']=total
    numbers.write()
    flat=[]
    for suite in suites:
        for key,data in summary[suite+'_cells'].items():
            record={'suite':suite,'case':key,'successes':data['success']['successes'],'n':data['success']['n'],
                    'success_lower':data['success']['lower'],'success_upper':data['success']['upper'],
                    'collisions':data['collisions']}
            for metric in ('penalized_steps','discounted_return','calls_per_decision','mean_root_actions','mean_new_depth'):
                for stat in ('mean','lower','upper'):record[metric+'_'+stat]=data[metric][stat]
            flat.append(record)
    with (ROOT/'results/summary.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(flat[0]));writer.writeheader();writer.writerows(flat)
    (ROOT/'results/summary.json').write_text(json.dumps(summary,indent=2,default=float)+'\n')
    figures(marginals,components,curves,short)
    print(json.dumps({'total_episodes':total,'factor_effects':{k:{m:round(v['mean'],2) for m,v in d.items() if isinstance(v,dict) and 'mean' in v}
                      for k,d in summary['factorial']['effects'].items()}},indent=2))


if __name__ == '__main__':main()
