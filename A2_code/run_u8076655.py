"""Assignment entry point: python run_u8076655.py --config scenario.yaml."""

import os
for _name in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[_name] = '1'

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml

from environment import make_env
from planner import MCTSPlanner


VARIANTS = {
    'full': {},
    'no_guidance': {'guided': False},
    'no_leaf': {'leaf_heuristic': False},
    'no_widening': {'progressive': False},
    'no_risk': {'risk_weight': 0.},
    'visits_choice': {'switch_z': None},
    'max_q': {'switch_z': 0.},
    'heuristic': {'max_simulations': 0},
}


def report_step(stream, step, horizon, state, action, reward, stats):
    cells = ' '.join(f"({x},{y},{z}){'*' if r else ''}" for x, y, z, r in state.tolist())
    stream.write(f"step {step:3d}/{horizon} | {int(state[:, 3].sum())}/{len(state)} at goal | "
                 f"reward {reward:7.1f} | plan {stats.get('elapsed', 0.):5.2f}s "
                 f"{stats.get('simulations', 0):5d} sims | {cells}\n")
    stream.flush()


def run_episode(config, seed=None, planning_time=9.5, variant='full',
                action_mask='full', visualize=False, planner_options=None,
                time_limit=10., progress=None, before_viewer=None):
    # time_limit stays 10 s for the demo; simulation-capped experiments may raise it.
    # before_viewer(result) runs before the blocking viewer, as the supplied template prints first.
    if variant not in VARIANTS:
        raise ValueError(f'Unknown solver variant: {variant}')
    if not 0 <= planning_time <= time_limit:
        raise ValueError(f'planning_time must be between 0 and {time_limit:g} seconds')
    setup_start = time.perf_counter()
    if seed is None:
        supplied_seed = yaml.safe_load(Path(config).read_text()).get('seed')
        seed = 0 if supplied_seed is None else supplied_seed
    execution = make_env(config, seed=seed, visualize=visualize, action_mask=action_mask)
    state = execution.reset()
    model = make_env(config, seed=1000003 + seed, action_mask=action_mask)
    model.reset()
    options = {**VARIANTS[variant], 'budget_cap': time_limit, **(planner_options or {})}
    planner = MCTSPlanner(model, seed=2000003 + seed, **options)
    cfg = model.get_config()
    setup_seconds = time.perf_counter() - setup_start
    trajectory, planning_times, searches = [], [], []
    total_reward, discounted_return = 0., 0.
    failure = None
    success = bool(state[:, 3].all())
    if len({tuple(g) for g in cfg.goal_positions}) < len(cfg.goal_positions):
        failure, success = 'infeasible_shared_goals', False
    elif planner.unreachable_drones(state):
        failure, success = 'infeasible_static_path', False
    for step in range(cfg.max_num_steps):
        if success or failure:
            break
        begin = time.perf_counter()
        action = planner.plan(state, planning_time)
        elapsed = time.perf_counter() - begin
        planning_times.append(elapsed)
        searches.append(planner.last_stats)
        # The execution RNG is independent of all search simulations.
        successor, reward, done, info = execution.step(action)
        total_reward += float(reward)
        discounted_return += cfg.discount_factor**step * float(reward)
        trajectory.append(dict(state=state.tolist(), action=action, reward=float(reward),
                               next_state=successor.tolist(), done=bool(done),
                               info={k: (v.item() if hasattr(v, 'item') else v) for k,v in info.items()}))
        state = successor
        if progress is not None:
            report_step(progress, step+1, cfg.max_num_steps, state, action, float(reward), planner.last_stats)
        collision = bool(info['num_collisions'] or info['num_vehicle_collisions'])
        success = bool(info['success']) and not collision
        if collision:
            failure = 'collision'
        elif done and not success:
            failure = 'timeout'
    if not success and failure is None:
        failure = 'timeout'
    result = dict(
        config=str(config), seed=seed, variant=variant, action_mask=action_mask,
        success=success, termination='success' if success else failure,
        steps=len(trajectory), total_reward=total_reward, discounted_return=discounted_return,
        setup_seconds=setup_seconds, planning_seconds=sum(planning_times),
        mean_plan_seconds=float(np.mean(planning_times)) if planning_times else 0.,
        max_plan_seconds=max(planning_times, default=0.),
        planning_budget=planning_time, num_drones=len(cfg.start_positions),
        num_actions=planner.action_count, obstacles=len(set(map(tuple,cfg.obstacle_cells or []))),
        simulations=sum(s['simulations'] for s in searches),
        generative_calls=sum(s['generative_calls'] for s in searches),
        mean_root_actions=float(np.mean([s['root_actions'] for s in searches])) if searches else 0.,
        mean_top_share=float(np.mean([s['top_share'] for s in searches])) if searches else 0.,
        greedy_fraction=float(np.mean([s['greedy'] for s in searches])) if searches else 0.,
        mean_new_depth=float(np.mean([s['mean_new_depth'] for s in searches if s['simulations']]))
                       if any(s['simulations'] for s in searches) else 0.,
        final_state=state.tolist(), trajectory=trajectory, searches=searches,
    )
    if visualize:
        if before_viewer is not None:
            before_viewer(result)
        execution.show()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--planning-time', type=float, default=9.5)
    parser.add_argument('--seed', type=int, default=None,
                        help='Override the YAML seed; otherwise use its seed, or 0 if absent')
    parser.add_argument('--variant', choices=VARIANTS, default='full')
    parser.add_argument('--action-mask', choices=['full','axial','face_edge'], default='full')
    parser.add_argument('--max-simulations', type=int, default=2000)
    parser.add_argument('--visualize', action='store_true', help='Optional Vedo viewer (needs vedo/vtk)')
    parser.add_argument('--quiet', action='store_true', help='Do not print per-step progress to stderr')
    parser.add_argument('--output', help='Write the complete trajectory and search diagnostics as JSON')
    args = parser.parse_args()
    if args.visualize:
        try:
            import vedo  # noqa: F401
        except ImportError:
            sys.stderr.write('vedo is not installed; continuing headless (pip install vedo).\n')
            args.visualize = False

    def publish(result):
        if args.output:
            target = Path(args.output)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(json.dumps(result, indent=2) + '\n')
        concise = {k:v for k,v in result.items() if k not in ('trajectory','searches','final_state')}
        if not args.quiet:
            sys.stderr.write(f"success: {result['success']} ({result['termination']}), steps: {result['steps']}, "
                             f"total discounted reward: {result['discounted_return']:.2f}\n")
            if args.visualize:
                sys.stderr.write('Viewer is interactive; close the window (or press q) to exit.\n')
            sys.stderr.flush()
        print(json.dumps(concise, indent=2), flush=True)

    result = run_episode(args.config, args.seed, args.planning_time, args.variant,
                         args.action_mask, args.visualize,
                         {'max_simulations': 0 if args.variant == 'heuristic' else args.max_simulations},
                         progress=None if args.quiet else sys.stderr,
                         before_viewer=publish if args.visualize else None)
    if not args.visualize:
        publish(result)


if __name__ == '__main__':
    main()
