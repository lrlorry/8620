"""Check finite-horizon success DP against outcomes of the supplied simulator."""

from functools import lru_cache
from itertools import product
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np
import yaml

from environment import make_env
from scripts.compute_ceilings import drone_pair, run_policy, single_drone, transition_model


class CaptureChoices:
    def __init__(self):
        self.rows = []

    def choice(self, count, p):
        row = np.asarray(p, dtype=float)
        self.rows.append(row / row.sum())
        return int(np.argmax(row))


class FixedChoices:
    def __init__(self, choices):
        self.choices = iter(choices)

    def choice(self, count, p):
        return next(self.choices)


class CeilingTests(unittest.TestCase):
    def setUp(self):
        self.folder = TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)

    def make_case(self, starts, goals):
        cfg = dict(grid_size=[3, 2, 1], change_altitude=False,
                   start_positions=starts, goal_positions=goals,
                   obstacle_cells=[[1, 1, 0]], alpha=.5,
                   discount_factor=.98, step_cost=-1., collision_penalty=-100.,
                   goal_reward=100., max_num_steps=3)
        path = Path(self.folder.name) / 'case.yaml'
        path.write_text(yaml.safe_dump(cfg))
        env = make_env(path, 0)
        state = env.reset()
        return cfg, path, env, state

    @staticmethod
    def enumerate_outcomes(env):
        count = len(env.cfg.start_positions)

        @lru_cache(None)
        def outcomes(key, action):
            state = np.frombuffer(key, dtype=np.int32).reshape(count, 4)
            capture = CaptureChoices()
            env.rng = capture
            env.simulate(state, action)
            rows = capture.rows
            results = []
            for choices in product(*(np.flatnonzero(row) for row in rows)):
                probability = float(np.prod([row[j] for row, j in zip(rows, choices)]))
                env.rng = FixedChoices(choices)
                successor, _, done, info = env.simulate(state, action)
                success = bool(info['success'] and not (
                    info['num_collisions'] or info['num_vehicle_collisions']))
                results.append((probability, successor.tobytes(), bool(done), success))
            return results
        return outcomes

    def exhaustive_success(self, env, initial, horizon):
        outcomes = self.enumerate_outcomes(env)
        count = len(initial)

        @lru_cache(None)
        def value(key, remaining):
            state = np.frombuffer(key, dtype=np.int32).reshape(count, 4)
            if state[:, 3].all():
                return 1.
            if remaining == 0:
                return 0.
            return max(sum(p * (float(success) if done else value(nxt, remaining-1))
                           for p, nxt, done, success in outcomes(key, action))
                       for action in range(env.num_actions))
        return value(initial.tobytes(), horizon)

    def test_transition_rows_match_upstream_at_boundaries_and_obstacles(self):
        cfg, _, env, _ = self.make_case([[0, 0, 0]], [[2, 0, 0]])
        index, matrix, base = transition_model(cfg)
        outcomes = self.enumerate_outcomes(env)
        for cell, i in index.items():
            state = np.asarray([[*cell, 0]], dtype=np.int32)
            for action in range(base):
                expected = np.zeros(len(index)+1)
                for p, nxt, done, success in outcomes(state.tobytes(), action):
                    successor = np.frombuffer(nxt, dtype=np.int32).reshape(1, 4)
                    target = len(index) if done and not success else index[tuple(successor[0, :3])]
                    expected[target] += p
                np.testing.assert_allclose(matrix[i, action], expected, rtol=0, atol=1e-6)

    def test_single_drone_values_match_exhaustive_simulator_outcomes(self):
        cfg, _, env, state = self.make_case([[0, 0, 0]], [[2, 0, 0]])
        for horizon in (1, 2, 3):
            value, _, _, _ = single_drone(cfg, 0, horizon)
            self.assertAlmostEqual(value, self.exhaustive_success(env, state, horizon), delta=2e-6)

    def test_pair_values_include_shared_cells_and_frozen_goal_occupants(self):
        for starts in ([[0, 0, 0], [0, 1, 0]], [[2, 0, 0], [0, 1, 0]]):
            with self.subTest(starts=starts):
                cfg, _, env, state = self.make_case(starts, [[2, 0, 0], [2, 1, 0]])
                for horizon in (1, 2):
                    value, _, _, _ = drone_pair(cfg, horizon)
                    self.assertAlmostEqual(value, self.exhaustive_success(env, state, horizon), delta=2e-6)

    def test_initial_goals_are_success_with_zero_remaining_steps(self):
        cfg, path, _, _ = self.make_case([[2, 0, 0]], [[2, 0, 0]])
        self.assertEqual(single_drone(cfg, 0, 0)[0], 1.)
        self.assertEqual(run_policy(path, 0, lambda *_: self.fail('already terminal'), episodes=3), 1.)
        cfg, _, _, _ = self.make_case([[2, 0, 0], [2, 1, 0]], [[2, 0, 0], [2, 1, 0]])
        self.assertEqual(drone_pair(cfg, 0)[0], 1.)


if __name__ == '__main__':
    unittest.main()
