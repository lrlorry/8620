"""Insert verified release measurements into the report's generated tables."""

import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
REPORT=next((p for p in (ROOT/'report',ROOT.parent/'A2')
             if (p/'u8076655_A2_report.tex').exists()),ROOT/'report')
LABELS=[('demo_open_2d','Open 2D, 1 drone, 8 actions'),('example_config','Supplied example, 2 drones'),
        ('strong_wind','Strong wind, 1 drone, $\\alpha=3$'),('demo_four_3d','Four 3D, 5 obstacles')]


def main():
    measured=json.loads((ROOT/'results/release_validation.json').read_text())
    small=json.loads((ROOT/'results/small_model_verification.json').read_text())
    summary=json.loads((ROOT/'results/summary.json').read_text())
    software=json.loads((ROOT/'results/software_environment.json').read_text())
    assert summary['total_episodes']==3300 and small['passed']
    rows=[]
    for name,label in LABELS:
        r=measured['default_demos'][name]
        rows.append(f"{label} & {r['termination']} & {r['steps']} & {r['simulations']/max(1,r['steps']):,.0f} "
                    f"& {r['mean_plan_seconds']:.2f} & {r['max_plan_seconds']:.2f}"+r' \\')
    text=(f"All {measured['unit_tests']} unit tests pass. A separate exhaustive check of the free\n"
          f"$2\\times2\\times1$ two-drone model covers {small['joint_states']} admissible nonterminal\n"
          f"joint states, {small['state_action_pairs']} state--action pairs and all\n"
          f"{small['positive_probability_outcomes']:,} positive-probability outcomes. Single-process\n"
          "runs with the default 9.5-second allowance (seed 0) give:\n\n"
          "\\begin{table}[H]\n\\centering\\small\n"
          "\\begin{tabular}{@{}llrrrr@{}}\n\\toprule\n"
          "Task & Status & Steps & Sims/step & Mean (s) & Max (s) \\\\\n\\midrule\n"+
          '\n'.join(rows)+"\n\\bottomrule\n\\end{tabular}\n\\end{table}\n"
          f"A forced 10-second call with the trial cap raised to 100,000 returns in\n"
          f"{measured['forced_10s_check']['measured_elapsed']:.3f} seconds. On slower hardware the wall-clock\n"
          "deadline, not the trial cap, ends the search; these are measurements on one machine,\n"
          "not a hard real-time guarantee.\n")
    (REPORT/'tables/validation_summary.tex').write_text(text)
    architecture=software['architecture'].replace('_',r'\_')
    environment=(f"Episodes ran on {software['os'].replace('_',r'\_')} (\\texttt{{{architecture}}}), Python {software['python']},\n"
                 f"NumPy {software['numpy']}, SciPy {software['scipy']}, PyYAML {software['pyyaml']}.\n"
                 "Each episode plans in one process and one thread; independent episodes ran in\n"
                 "parallel worker processes. Because search effort is fixed by the simulation cap,\n"
                 "machine load changes wall time but not the decisions.\n")
    (REPORT/'tables/experiment_environment.tex').write_text(environment)
    print('Refreshed verified measurement tables.')


if __name__=='__main__':main()
