"""Entry point for the centralised RRT-Connect planner (Part B, Q3).

Usage:
    python run_planner.py --environment environments/E1_sparse.yaml \
        --time-limit 20 --seed 2026

--num-drones is optional and inferred from the environment file if omitted.
"""

import argparse
import time

PROGRAM_START = time.perf_counter()  # captures import/setup cost too (see main())

import numpy as np
import yaml

from multi_drone import MultiDrone
from planner import rrt_connect


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--environment", required=True)
    parser.add_argument(
        "--num-drones",
        type=int,
        default=None,
        help="Number of drones. Default: inferred from the environment "
        "file's initial_configuration length, so an unfamiliar scenario "
        "file doesn't require guessing this by hand.",
    )
    parser.add_argument("--time-limit", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--eta",
        type=float,
        default=None,
        help="RRT-Connect extension length. Default: 5%% of the workspace "
        "diagonal (B2 -- scaled to the problem instance, not a fixed constant).",
    )
    parser.add_argument(
        "--bridge-prob",
        type=float,
        default=0.0,
        help="Probability of a bridge-test sampling attempt in SAMPLE-FREE "
        "(Hsu et al. 2003, narrow-passage sampling; not covered in the "
        "course material). 0.0 (default) reproduces the B2 baseline exactly.",
    )
    parser.add_argument(
        "--bridge-sigma",
        type=float,
        default=None,
        help="Std dev of the bridge-test Gaussian offset. Default: same as eta.",
    )
    parser.add_argument(
        "--local-bridge",
        action="store_true",
        help="Anchor bridge-test attempts near the extending tree's own nodes "
        "(EST idea, week3) instead of drawing globally -- helps with "
        "sequential/compound narrow passages.",
    )
    parser.add_argument("--visualize", action="store_true")
    return parser.parse_args()


def load_environment(environment_file):
    with open(environment_file) as f:
        env = yaml.safe_load(f)
    bounds = np.array(
        [env["bounds"]["x"], env["bounds"]["y"], env["bounds"]["z"]], dtype=np.float32
    )
    goal_radii = np.array([g["radius"] for g in env["goals"]], dtype=np.float32)
    num_drones = len(env["initial_configuration"])
    return bounds, goal_radii, num_drones


def main():
    args = parse_args()
    rng = np.random.default_rng(args.seed)

    bounds, goal_radii, num_drones = load_environment(args.environment)
    if args.num_drones is not None and args.num_drones != num_drones:
        print(
            f"WARNING: --num-drones {args.num_drones} does not match "
            f"{num_drones} entries in {args.environment}; using {num_drones}."
        )
    sim = MultiDrone(num_drones=num_drones, environment_file=args.environment)

    eta = args.eta
    if eta is None:
        diagonal = np.linalg.norm(bounds[:, 1] - bounds[:, 0])
        eta = 0.05 * diagonal

    # Budget rrt_connect with whatever remains of --time-limit AFTER imports
    # and MultiDrone's construction (measured ~2-5s: it builds a vedo/VTK
    # scene), so the total wall-clock time from process start to answer
    # never exceeds --time-limit, not just the planning call in isolation.
    # SAFETY_MARGIN covers two things PROGRAM_START cannot see: (1) Python
    # interpreter startup before our own first line runs, measured at a
    # stable ~0.22-0.24s over 8 subprocess trials, and (2) the one
    # un-gated EXTEND the main loop finishes after its last deadline check
    # (a single motion_valid call, ~0-20ms). 0.3s covers the measured
    # interpreter overhead with headroom for a slower lab machine, without
    # eating heavily into the real planning budget.
    SAFETY_MARGIN = 0.3
    setup_elapsed = time.perf_counter() - PROGRAM_START
    remaining_budget = max(0.0, args.time_limit - setup_elapsed - SAFETY_MARGIN)

    plan_start = time.perf_counter()
    path = rrt_connect(
        sim, bounds, goal_radii, eta, remaining_budget, rng,
        bridge_prob=args.bridge_prob, bridge_sigma=args.bridge_sigma,
        local_bridge=args.local_bridge,
    )
    total_elapsed = time.perf_counter() - PROGRAM_START

    if path is None:
        print(f"FAILURE (total {total_elapsed:.2f}s incl. {setup_elapsed:.2f}s setup, eta={eta:.3f})")
        return

    # Independently re-check the returned path before claiming success:
    # every consecutive motion must be collision-free and the final
    # configuration must satisfy the goal test. rrt_connect already
    # guarantees this by construction (every tree edge passed
    # motion_valid), but re-verifying here means SUCCESS is never printed
    # ahead of the check that justifies it.
    valid = sim.is_valid(path[0])
    for i in range(len(path) - 1):
        valid = valid and sim.motion_valid(path[i], path[i + 1])
    reached_goal = sim.is_goal(path[-1])

    if not (valid and reached_goal):
        print(f"FAILURE: rrt_connect returned a path that failed re-validation "
              f"(collision-free={valid}, goal reached={reached_goal}) -- this "
              f"should never happen; please report it as a bug.")
        return

    print(f"SUCCESS: {len(path)} waypoints, {total_elapsed:.2f}s total "
          f"({setup_elapsed:.2f}s setup + {time.perf_counter() - plan_start:.2f}s planning), eta={eta:.3f}")
    print(f"VERIFIED: every motion collision-free = {valid}, goal reached = {reached_goal}")

    if args.visualize:
        sim.visualize_paths(path)


if __name__ == "__main__":
    main()
