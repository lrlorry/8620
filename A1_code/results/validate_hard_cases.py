"""Verify explicit feasible paths for the E9 and E11 stress tests.

These waypoint witnesses are independent of the sampling planner. They show
that a zero baseline success rate is a finite-time planning failure, not proof
that either environment is infeasible.

Run: python results/validate_hard_cases.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
os.chdir(os.path.join(os.path.dirname(__file__), ".."))

from multi_drone import MultiDrone


START = np.array([
    [2, 15, 25],
    [2, 20, 25],
    [2, 25, 25],
    [2, 30, 25],
    [2, 35, 25],
], dtype=np.float32)

GOAL = np.array([
    [48, 15, 25],
    [48, 20, 25],
    [48, 25, 25],
    [48, 30, 25],
    [48, 35, 25],
], dtype=np.float32)


def e9_witness():
    """Move the drones through the small y-z aperture one at a time."""
    path = [START.copy()]
    q = START.copy()
    for drone in range(5):
        for waypoint in ([20, 25, 25], [30, 25, 25], GOAL[drone]):
            q = q.copy()
            q[drone] = waypoint
            path.append(q)
    return path


def e11_witness():
    """Compress above the U-shaped trap, traverse, then expand to the goals."""
    detour_y = np.array([42, 43.5, 45, 46.5, 48], dtype=np.float32)
    left = np.column_stack((np.full(5, 2), detour_y, np.full(5, 25)))
    right = np.column_stack((np.full(5, 48), detour_y, np.full(5, 25)))
    return [START.copy(), left.astype(np.float32), right.astype(np.float32), GOAL.copy()]


def validate(environment, path):
    sim = MultiDrone(num_drones=5, environment_file=f"environments/{environment}")
    states_valid = all(sim.is_valid(q) for q in path)
    edges_valid = all(sim.motion_valid(a, b) for a, b in zip(path, path[1:]))
    goal_reached = sim.is_goal(path[-1])
    verified = states_valid and edges_valid and goal_reached
    print(
        f"{environment}: {'VERIFIED' if verified else 'INVALID'} "
        f"({len(path)} waypoints, {len(path) - 1} edges)"
    )
    return verified


def main():
    checks = [
        validate("E9_3d_aperture.yaml", e9_witness()),
        validate("E11_dead_end.yaml", e11_witness()),
    ]
    if not all(checks):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
