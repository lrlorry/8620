"""Headless execution and controlled intended-action masks.

The supplied stochastic transition/reward functions are not changed.
"""

from copy import copy
import math

import numpy as np

from multi_drone import MultiDroneUnc
from planner import action_vectors, decode_action, encode_action


def validate_config(cfg):
    if len(cfg.grid_size) != 3 or any(not isinstance(n, int) or n < 1 for n in cfg.grid_size):
        raise ValueError('grid_size must contain three positive integers')
    starts = np.asarray(cfg.start_positions)
    goals = np.asarray(cfg.goal_positions)
    if starts.ndim != 2 or starts.shape[1] != 3 or len(starts) == 0 or goals.shape != starts.shape:
        raise ValueError('Provide matching, nonempty N-by-3 start and goal arrays')
    for label, positions in [('start', starts), ('goal', goals)]:
        if not np.isfinite(positions).all() or not np.equal(positions, positions.astype(int)).all():
            raise ValueError(f'{label} positions must be integer cells')
        if ((positions < 0) | (positions >= np.asarray(cfg.grid_size))).any():
            raise ValueError(f'{label} position is outside the grid')
    obstacles = set()
    for cell in cfg.obstacle_cells or []:
        if len(cell) != 3 or any(not isinstance(x, int) for x in cell):
            raise ValueError('Obstacle coordinates must be integer triples')
        if any(not 0 <= cell[i] < cfg.grid_size[i] for i in range(3)):
            raise ValueError('Obstacle is outside the grid')
        if cell[0] == cfg.grid_size[0]-1:
            raise ValueError('The supplied simulator forbids obstacles on the final x plane')
        obstacles.add(tuple(cell))
    if len({tuple(p) for p in starts}) != len(starts):
        raise ValueError('Start positions already contain an inter-drone collision')
    if any(tuple(p) in obstacles for p in starts):
        raise ValueError('A start position is inside an obstacle')
    if any(tuple(p) in obstacles for p in goals):
        raise ValueError('A goal position is inside an obstacle')
    if not isinstance(cfg.max_num_steps, int) or cfg.max_num_steps < 1:
        raise ValueError('max_num_steps must be a positive integer')
    if not math.isfinite(cfg.alpha) or cfg.alpha <= 0:
        raise ValueError('alpha must be positive, as required by the supplied motion model')
    if not math.isfinite(cfg.discount_factor) or not 0 <= cfg.discount_factor <= 1:
        raise ValueError('discount_factor must lie in [0,1]')
    if any(not math.isfinite(getattr(cfg, name))
           for name in ('step_cost', 'goal_reward', 'collision_penalty')):
        raise ValueError('Rewards must be finite')


class HeadlessMultiDroneUnc(MultiDroneUnc):
    def __init__(self, path, seed=None):
        super().__init__(path)
        validate_config(self.cfg)
        if seed is not None:
            self.rng = np.random.default_rng(seed)

    def _init_plot(self):
        pass

    def _update_plot(self):
        pass


class ActionMask:
    """Restrict or duplicate intended commands; preserve the original slip distribution.

    step/simulate map compact labels to the SAME upstream 26-action model.
    This avoids changing uncertainty when measuring action-set size.
    'dupK' repeats every command K times: more labels, identical control authority.
    """
    def __init__(self, env, kind='full'):
        self.env = env
        cfg = env.get_config()
        vectors = action_vectors(cfg.change_altitude)
        if kind == 'full':
            indices = list(range(len(vectors)))
        elif kind == 'axial':
            indices = [i for i, v in enumerate(vectors) if np.count_nonzero(v) == 1]
        elif kind == 'face_edge':
            indices = [i for i, v in enumerate(vectors) if np.count_nonzero(v) <= 2]
        elif kind.startswith('dup') and kind[3:].isdigit() and int(kind[3:]) >= 1:
            indices = list(range(len(vectors))) * int(kind[3:])
        else:
            raise ValueError(f'Unknown intended-action mask: {kind}')
        self.cfg = copy(cfg)
        self.cfg.intended_action_indices = indices
        self.indices, self.physical_base = indices, len(vectors)
        self.count = len(cfg.start_positions)

    @property
    def num_actions(self):
        return len(self.indices) ** self.count

    def get_config(self):
        return self.cfg

    def map_action(self, action):
        digits = decode_action(action, len(self.indices), self.count)
        return encode_action([self.indices[d] for d in digits], self.physical_base)

    def simulate(self, state, action):
        return self.env.simulate(state, self.map_action(action))

    def step(self, action):
        return self.env.step(self.map_action(action))

    def reset(self):
        return self.env.reset()

    def show(self):
        return self.env.show()


def make_env(path, seed, visualize=False, action_mask='full'):
    if visualize:
        env = MultiDroneUnc(path)
        validate_config(env.get_config())
        env.rng = np.random.default_rng(seed)
    else:
        env = HeadlessMultiDroneUnc(path, seed=seed)
    return env if action_mask == 'full' else ActionMask(env, action_mask)
