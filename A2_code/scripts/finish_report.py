"""Insert verified release measurements into the report's generated tables."""

import json
import io
from pathlib import Path
import re
import sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from run_u8076655 import report_step
REPORT=next((p for p in (ROOT/'report',ROOT.parent/'A2')
             if (p/'u8076655_A2_report.tex').exists()),ROOT/'report')
LABELS=[('demo_open_2d','Open 2D, 1 drone, 8 actions'),('example_config','Supplied example, 2 drones'),
        ('strong_wind','Strong wind, 1 drone, $\\alpha=3$'),('demo_four_3d','Four 3D, 5 obstacles')]


def main():
    measured=json.loads((ROOT/'results/release_validation.json').read_text())
    small=json.loads((ROOT/'results/small_model_verification.json').read_text())
    summary=json.loads((ROOT/'results/summary.json').read_text())
    software=json.loads((ROOT/'results/software_environment.json').read_text())
    assert summary['total_episodes']==3460 and small['passed']
    test_log=(ROOT/'results/unit_tests.log').read_text()
    test_count=re.search(r'Ran (\d+) tests',test_log)
    assert test_count and test_log.rstrip().endswith('OK'), 'Unit-test log must record a passing run'
    rows=[]
    for name,label in LABELS:
        r=measured['default_demos'][name]
        rows.append(f"{label} & {r['termination']} & {r['steps']} & {r['simulations']/max(1,r['steps']):,.0f} "
                    f"& {r['mean_plan_seconds']:.2f} & {r['max_plan_seconds']:.2f}"+r' \\')
    text=(f"All {test_count.group(1)} unit tests pass. Local single-process runs with the\n"
          "default 9.5-second allowance (seed 0) give:\n\n"
          "\\begin{table}[H]\n\\centering\\small\n"
          "\\renewcommand{\\arraystretch}{1.13}\n"
          "\\begin{tabular*}{\\linewidth}{@{\\extracolsep{\\fill}}llrrrr@{}}\n\\toprule\n"
          "Task & Status & Steps & Sims/step & Mean (s) & Max (s) \\\\\n\\midrule\n"+
          '\n'.join(rows)+"\n\\bottomrule\n\\end{tabular*}\n\\end{table}\n")
    (REPORT/'tables/validation_summary.tex').write_text(text)
    trace=json.loads((ROOT/'results/demo_open_2d_default.json').read_text())
    assert trace['steps']==measured['default_demos']['demo_open_2d']['steps']
    horizon=yaml.safe_load((ROOT/'environments/demo_open_2d.yaml').read_text())['max_num_steps']
    excerpt=io.StringIO()
    for i in (0,1,trace['steps']-1):
        if i==trace['steps']-1:excerpt.write('...\n')
        step=trace['trajectory'][i]
        report_step(excerpt,i+1,horizon,np.asarray(step['next_state']),
                    step['action'],step['reward'],trace['searches'][i])
    excerpt.write(f"success: {trace['success']} ({trace['termination']}), steps: {trace['steps']}, "
                  f"total discounted reward: {trace['discounted_return']:.2f}\n")
    (REPORT/'tables/demo_excerpt.tex').write_text(
        '\\begin{lstlisting}[style=terminal,keepspaces=true]\n'+excerpt.getvalue()+'\\end{lstlisting}\n')
    architecture=software['architecture'].replace('_',r'\_')
    environment=(f"Episodes ran on {software['os'].replace('_',r'\_')} (\\texttt{{{architecture}}}), Python {software['python']},\n"
                 f"NumPy {software['numpy']}, SciPy {software['scipy']}, PyYAML {software['pyyaml']}.\n"
                 "Wall times depend on hardware and load; a fixed simulation cap controls search\n"
                 "effort only when the relaxed experimental deadline does not bind.\n")
    (REPORT/'tables/experiment_environment.tex').write_text(environment)
    print('Refreshed verified measurement tables.')


if __name__=='__main__':main()
