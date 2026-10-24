"""Plot saved full-solver traces; do not run the planner or change experiment data.

The complete gallery covers every non-development manifest entry and the
supplied example. Twelve cases form four readable report sheets. Default-budget
demo timings and relaxed-deadline experimental timings are labelled separately.
"""

import hashlib
import json
import os
from pathlib import Path
import tempfile

os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir())/'comp8620-mpl'))
import matplotlib
matplotlib.use('Agg')
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
REPORT = next(p for p in (ROOT/'report', ROOT.parent/'A2')
              if (p/'u8076655_A2_report.tex').exists())
OUT = ROOT/'results/visualisations'
COLOURS = ['#0072B2', '#E69F00', '#009E73', '#CC79A7']
DEFAULTS = {'demo_open_2d', 'example_config', 'strong_wind', 'demo_four_3d'}
TITLES = {
    'demo_open_2d': 'Open 2D',
    'example_config': 'Supplied two-drone example',
    'strong_wind': 'Strong wind',
    'demo_four_3d': 'Four drones in 3D',
    'already_at_goals': 'Initially at goals',
    'single_cell_goal': 'Single-cell goal',
    'one_reached': 'One drone already at its goal',
    'swap_2d': 'Two-drone swap',
    'boundary_reverse': 'Boundary reversal',
    'tiny_2d': 'Tiny grid',
    'thin_1d': 'One-cell-wide route',
    'infeasible_altitude': 'Unreachable altitude',
    'infeasible_wall': 'Blocking wall',
    'infeasible_shared_goal': 'Shared goal',
    'horizon_one': 'One-step horizon',
    'narrow_passage': 'Two-drone narrow passage',
    'altitude_detour': 'Altitude detour',
    'three_3d': 'Three drones in 3D',
    'four_2d': 'Four drones in 2D',
    'supp_single_example_2d': 'Single drone, fixed altitude',
    'supp_single_wallgap_2d': 'Wall with one gap',
    'supp_single_slalom_2d': 'Single-drone slalom',
    'supp_single_clutter_2d': 'Single drone, five obstacles',
    'supp_layers_cross_2d': 'Crossing on separate layers',
    'supp_layers_four_2d': 'Four drones on separate layers',
    'supp_yield_corridor': 'Goal inside a shared corridor',
}
SHEETS = [
    ('Default-budget demonstrations', [
        'demo_open_2d', 'example_config', 'strong_wind']),
    ('3D flight and constrained routes', [
        'demo_four_3d', 'altitude_detour', 'three_3d']),
    ('Clutter, bottlenecks and goal occupancy', [
        'supp_single_clutter_2d', 'supp_single_wallgap_2d', 'one_reached']),
    ('Altitude layers, coordination and a failure', [
        'supp_layers_cross_2d', 'supp_yield_corridor', 'narrow_passage']),
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_cases():
    entries = [r for r in json.loads((ROOT/'environments/manifest.json').read_text())
               if r['suite'] != 'dev']
    entries.insert(1, dict(name='example_config', suite='provided',
                           path='example_config.yaml', mask='full'))
    cases = []
    for entry in entries:
        name = entry['name']
        default = name in DEFAULTS
        source = (ROOT/f'results/{name}_default.json' if default else
                  ROOT/f"results/trajectories/{entry['suite']}_{name}_full_2000.json")
        config = ROOT/entry['path']
        data = json.loads(source.read_text())
        cfg = yaml.safe_load(config.read_text())
        assert data['variant'] == 'full' and data['seed'] == 0, source
        assert data['action_mask'] == entry.get('mask', 'full'), source
        assert Path(data['config']).name == config.name, source
        assert len(data['trajectory']) == len(data['searches']) == data['steps'], source
        initial = (data['trajectory'][0]['state'] if data['trajectory']
                   else data['final_state'])
        states = np.asarray([initial]+[step['next_state'] for step in data['trajectory']])
        assert states.shape == (data['steps']+1, len(cfg['start_positions']), 4), source
        assert np.array_equal(states[0, :, :3], cfg['start_positions']), source
        assert np.array_equal(states[-1], data['final_state']), source
        for i, step in enumerate(data['trajectory']):
            assert np.array_equal(states[i], step['state']), source
        elapsed = np.asarray([row['elapsed'] for row in data['searches']])
        assert np.isfinite(elapsed).all() and (elapsed >= 0).all(), source
        assert all(row['simulations'] <= 2000 for row in data['searches']), source
        assert data['planning_budget'] == (9.5 if default else 3600.0), source
        cases.append(dict(entry=entry, data=data, cfg=cfg, states=states,
                          elapsed=elapsed, source=source, config=config, default=default))
    assert len({c['entry']['name'] for c in cases}) == len(cases)
    return cases


def spatial_axis(fig, slot, case):
    positions = case['states'][:, :, :3]
    heights = np.concatenate((positions[:, :, 2].ravel(),
                              np.asarray(case['cfg']['goal_positions'])[:, 2]))
    is_3d = case['cfg']['change_altitude'] or len(np.unique(heights)) > 1
    return fig.add_subplot(slot, projection='3d' if is_3d else None), is_3d


def path_plot(ax, is_3d, case, compact=False):
    cfg, data, states = case['cfg'], case['data'], case['states']
    dims = cfg['grid_size']
    size = 10
    goals = np.asarray(cfg['goal_positions'])
    positions = states[:, :, :3]
    obstacles = cfg.get('obstacle_cells') or []
    layer = int(positions[0, 0, 2])
    if is_3d:
        for x, y, z in obstacles:
            ax.bar3d(x-.42, y-.42, z-.42, .84, .84, .84,
                     color='#777777', alpha=.35, shade=True)
    else:
        for x, y, z in obstacles:
            if z == layer:
                ax.add_patch(plt.Rectangle((x-.48, y-.48), .96, .96,
                                           color='#777777', alpha=.55, zorder=1))
    for i in range(positions.shape[1]):
        p, colour = positions[:, i], COLOURS[i % len(COLOURS)]
        coords = (p[:, 0], p[:, 1], p[:, 2]) if is_3d else (p[:, 0], p[:, 1])
        ax.plot(*coords, color=colour, lw=1.6 if compact else 1.8,
                marker='.', markersize=2.6 if compact else 3, zorder=3)
        start = p[0] if is_3d else p[0, :2]
        goal = goals[i] if is_3d else goals[i, :2]
        ax.scatter(*start, color=colour, marker='s', s=16 if compact else 36,
                   edgecolors='white', linewidths=.4, zorder=5)
        ax.scatter(*goal, color=colour, marker='*', s=43 if compact else 90,
                   edgecolors='#222222', linewidths=.35, zorder=6)
        if not data['success'] and not states[-1, i, 3]:
            end = p[-1] if is_3d else p[-1, :2]
            ax.scatter(*end, color='#D55E00', marker='x', s=22 if compact else 50,
                       linewidths=1.1, zorder=7)
    ax.set_xlim(-.6, dims[0]-.4)
    ax.set_ylim(-.6, dims[1]-.4)
    ax.set_xlabel('x', fontsize=size, labelpad=0)
    ax.set_ylabel('y', fontsize=size, labelpad=0)
    if is_3d:
        ax.set_zlim(-.6, dims[2]-.4)
        ax.set_zlabel('z', fontsize=size, labelpad=0)
        ax.set_box_aspect([max(v, 2) for v in dims], zoom=1.16 if compact else 1)
        ax.view_init(elev=27, azim=-64)
        ax.zaxis.set_major_locator(MaxNLocator(3, integer=True))
    else:
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.14)
    ax.xaxis.set_major_locator(MaxNLocator(5, integer=True))
    ax.yaxis.set_major_locator(MaxNLocator(4, integer=True))
    ax.tick_params(labelsize=size-1, pad=1)
    title = TITLES.get(case['entry']['name'], case['entry']['name'].replace('_', ' '))
    outcome = data['termination'].replace('infeasible_', 'reject: ').replace('_', ' ')
    info = (f"N={len(goals)}, |O|={len(set(map(tuple,obstacles)))}, "
            f"alpha={cfg['alpha']:g}" if compact else
            f"N={len(goals)}, grid={'x'.join(map(str,dims))}, "
            f"|O|={len(set(map(tuple,obstacles)))}, alpha={cfg['alpha']:g}")
    if not is_3d:
        info += f', z={layer}'
    heading = (f"{title} | {outcome}\n{info}; {data['steps']} steps" if compact else
               f"{title} | {outcome}, {data['steps']} steps\n{info}")
    ax.set_title(heading,
                 fontsize=size+.3, loc='left', pad=5)


def time_plot(ax, case, compact=False):
    data, elapsed = case['data'], case['elapsed']
    size = 10
    if len(elapsed):
        ax.bar(np.arange(1, len(elapsed)+1), elapsed, color=COLOURS[0], width=.7)
        ax.set_xlim(.2, len(elapsed)+.8)
        ax.set_ylim(0, max(10.8 if case['default'] else .1, float(elapsed.max())*1.18))
        stats = f"mean {elapsed.mean():.2f} s; max {elapsed.max():.2f} s"
        if case['default']:
            ax.axhline(10, color='#D55E00', ls='--', lw=1)
            ax.text(.98, 10.08, '10 s limit', ha='right', va='bottom',
                    transform=ax.get_yaxis_transform(), fontsize=size-1, color='#B34B00')
    else:
        ax.set_xlim(.5, 1.5)
        ax.set_ylim(0, 1)
        ax.text(.5, .5, 'No planning steps\n'+data['termination'].replace('_', ' '),
                transform=ax.transAxes, ha='center', va='center', fontsize=size)
        stats = 'no planning calls'
    if compact:
        protocol = ('Demo: 9.5 s; \u22642,000 sims' if case['default'] else
                    'Experiment: 2,000 sims; relaxed cap')
        heading = f'{stats}\n{protocol}'
    else:
        protocol = ('Demo: 9.5 s allowance, <=2,000 simulations' if case['default'] else
                    'Experiment: 2,000 simulations, relaxed deadline')
        heading = f'Planning time: {stats}\n{protocol}'
    ax.set_title(heading, fontsize=size+.3, loc='left', pad=5)
    ax.set_xlabel('Executed step', fontsize=size, labelpad=1)
    ax.set_ylabel('Search time (s)', fontsize=size, labelpad=1)
    ax.xaxis.set_major_locator(MaxNLocator(6, integer=True))
    ax.yaxis.set_major_locator(MaxNLocator(4))
    ax.tick_params(labelsize=size-1, pad=1)
    ax.grid(axis='y', alpha=.2)
    ax.set_axisbelow(True)


def legend(fig, y, compact=False, drone_count=4):
    items = [Line2D([0], [0], color=c, lw=1.5, label=f'Drone {i+1}')
             for i, c in enumerate(COLOURS[:drone_count])]
    for marker, colour, label in [('s', '#444444', 'Start'), ('*', '#444444', 'Goal'),
                                   ('s', '#888888', 'Obstacle'),
                                   ('x', '#D55E00', 'Unfinished end')]:
        items.append(Line2D([0], [0], color=colour, marker=marker, linestyle='',
                            markersize=5, label=label))
    fig.legend(handles=items, loc='upper center', bbox_to_anchor=(.5, y),
               ncol=4 if compact else len(items), frameon=False, fontsize=9 if compact else 8,
               handlelength=1.2, columnspacing=.9, handletextpad=.35)


def main():
    plt.rcParams.update({'font.family': 'serif', 'mathtext.fontset': 'cm',
                         'pdf.fonttype': 42, 'axes.spines.top': False,
                         'axes.spines.right': False})
    cases = load_cases()
    OUT.mkdir(parents=True, exist_ok=True)
    (REPORT/'figures').mkdir(exist_ok=True)
    catalog = []
    gallery = REPORT/'u8076655_A2_scenario_gallery.pdf'
    with PdfPages(gallery) as pdf:
        for case in cases:
            name = case['entry']['name']
            fig = plt.figure(figsize=(10.6, 4.9))
            gs = fig.add_gridspec(1, 2, left=.06, right=.98, bottom=.15,
                                  top=.77, wspace=.30)
            ax, is_3d = spatial_axis(fig, gs[0, 0], case)
            path_plot(ax, is_3d, case)
            time_plot(fig.add_subplot(gs[0, 1]), case)
            fig.suptitle(name.replace('_', ' '), y=.98, fontsize=13)
            legend(fig, .91, drone_count=len(case['cfg']['start_positions']))
            fig.text(.5, .025,
                     f"Saved full-solver trace, seed 0: {case['source'].relative_to(ROOT)}\n"
                     "Crosses mark returned positions of unfinished drones; collision targets are not logged.",
                     ha='center', fontsize=7)
            png = OUT/f'{name}.png'
            fig.savefig(png, dpi=150)
            pdf.savefig(fig)
            plt.close(fig)
            catalog.append(dict(
                name=name, suite=case['entry']['suite'], seed=0,
                source=str(case['source'].relative_to(ROOT)),
                source_sha256=digest(case['source']),
                config=str(case['config'].relative_to(ROOT)),
                config_sha256=digest(case['config']), mask=case['entry'].get('mask', 'full'),
                protocol='default_demo' if case['default'] else 'fixed_simulation_experiment',
                termination=case['data']['termination'], steps=case['data']['steps'],
                png=str(png.relative_to(ROOT))))
    lookup = {c['entry']['name']: c for c in cases}
    for number, (heading, names) in enumerate(SHEETS, 1):
        fig = plt.figure(figsize=(7.2, 8.1))
        gs = fig.add_gridspec(3, 2, left=.07, right=.985, bottom=.06,
                              top=.84, hspace=.65, wspace=.38)
        fig.suptitle(heading+' (seed 0)', y=.995, fontsize=12)
        legend(fig, .96, compact=True,
               drone_count=max(len(lookup[name]['cfg']['start_positions']) for name in names))
        for row, name in enumerate(names):
            case = lookup[name]
            ax, is_3d = spatial_axis(fig, gs[row, 0], case)
            path_plot(ax, is_3d, case, compact=True)
            time_plot(fig.add_subplot(gs[row, 1]), case, compact=True)
        target = REPORT/f'figures/scenario_examples_{number}'
        fig.savefig(target.with_suffix('.pdf'))
        fig.savefig(target.with_suffix('.png'), dpi=200)
        plt.close(fig)
    index = dict(case_count=len(catalog), development_cases_excluded=16,
                 data_policy='Stored seed-0 full-solver traces only; no planner runs or new episodes.',
                 timing='Recorded internal search elapsed; not end-to-end episode or rendering time.',
                 sheets=[dict(title=title, names=names) for title, names in SHEETS],
                 cases=catalog)
    (OUT/'index.json').write_text(json.dumps(index, indent=2)+'\n')
    print(f'{gallery}: {len(catalog)} cases; {len(SHEETS)} report sheets; no new simulations.')


if __name__ == '__main__':
    main()
