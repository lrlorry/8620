"""B4 experiment runner -- the single script that produces every row in
results/experiment_results.csv (one row per trial) and the two
representative path .npy files used by make_figures.py. Four tables:
  1. complexity  -- B2 baseline (bridge_prob=0.0) across E0-E7
  2. mitigation  -- bridge-test (bridge_prob=0.3) on E5/E6/E7, paired
                    against table 1's 0.0 rows for the same environments
  3. scalability -- drone-swap E8, K=2..5, baseline
  4. extra       -- robustness/correctness probes E9--E13
Run: python results/run_experiments.py
"""

import csv
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
os.chdir(os.path.join(os.path.dirname(__file__), ".."))

import vedo
_RealPlotter = vedo.Plotter
vedo.Plotter = lambda *a, **kw: _RealPlotter(*a, **{**kw, "offscreen": True})

import numpy as np
import yaml
from multi_drone import MultiDrone
from planner import rrt_connect

TIME_LIMIT = 20.0
OUT_PATH = os.path.join(os.path.dirname(__file__), "experiment_results.csv")

FIELDS = ["table", "environment", "num_drones", "bridge_prob", "seed", "success", "elapsed_s"]


def load_env(env_path):
    """Build the simulator once per environment; MultiDrone's constructor
    sets up a vedo/VTK scene, which is far more expensive than a single
    rrt_connect call, so it must be reused across seeds, not rebuilt per
    trial. num_drones is inferred from the file, matching run_planner.py."""
    env = yaml.safe_load(open(env_path))
    num_drones = len(env["initial_configuration"])
    bounds = np.array([env["bounds"]["x"], env["bounds"]["y"], env["bounds"]["z"]], dtype=np.float32)
    goal_radii = np.array([g["radius"] for g in env["goals"]], dtype=np.float32)
    eta = 0.05 * np.linalg.norm(bounds[:, 1] - bounds[:, 0])
    sim = MultiDrone(num_drones=num_drones, environment_file=env_path)
    return sim, bounds, goal_radii, eta, num_drones


def run_trial(sim, bounds, goal_radii, eta, bridge_prob, seed):
    rng = np.random.default_rng(seed)
    t0 = time.perf_counter()
    path = rrt_connect(sim, bounds, goal_radii, eta, TIME_LIMIT, rng, bridge_prob=bridge_prob)
    elapsed = time.perf_counter() - t0
    return path is not None, elapsed


def main():
    # A full run is one reproducible data set. Write it atomically so reruns
    # cannot duplicate trials and an interrupted run preserves the last CSV.
    temporary_path = f"{OUT_PATH}.tmp"
    f = open(temporary_path, "w", newline="")
    writer = csv.DictWriter(f, fieldnames=FIELDS)
    writer.writeheader()

    def log(table, environment, num_drones, bridge_prob, seed, success, elapsed):
        writer.writerow({
            "table": table, "environment": environment, "num_drones": num_drones,
            "bridge_prob": bridge_prob, "seed": seed,
            "success": int(success), "elapsed_s": round(elapsed, 3),
        })
        f.flush()
        print(f"[{table}] {environment} K={num_drones} p={bridge_prob} seed={seed} "
              f"-> {'OK' if success else 'FAIL'} {elapsed:.2f}s", flush=True)

    # Table 1: complexity curve, B2 baseline (bridge_prob=0.0).
    # Table 2: bridge-test mitigation (bridge_prob=0.3), reusing the same
    # sim for E5/E6/E7 instead of rebuilding it for a separate pass.
    complexity_envs = [
        ("E0_empty.yaml", 25, False),
        ("E1_sparse.yaml", 25, False),
        ("E2_moderate.yaml", 25, False),
        ("E3_dense.yaml", 25, False),
        ("E4_bottleneck_moderate.yaml", 25, False),
        ("E5_bottleneck_severe.yaml", 25, True),
        ("E6_multi_gap.yaml", 25, True),
        ("E7_offset_passage.yaml", 15, True),
    ]
    for env_name, n_seeds, also_mitigation in complexity_envs:
        sim, bounds, goal_radii, eta, n = load_env(f"environments/{env_name}")
        for seed in range(n_seeds):
            ok, elapsed = run_trial(sim, bounds, goal_radii, eta, 0.0, seed)
            log("complexity", env_name, n, 0.0, seed, ok, elapsed)
        if also_mitigation:
            for seed in range(n_seeds):
                ok, elapsed = run_trial(sim, bounds, goal_radii, eta, 0.3, seed)
                log("mitigation", env_name, n, 0.3, seed, ok, elapsed)

    # Table 3: scalability, drone-swap K=2..5, baseline
    for K in [2, 3, 4, 5]:
        sim, bounds, goal_radii, eta, n = load_env(f"environments/E8_swap_K{K}.yaml")
        for seed in range(25):
            ok, elapsed = run_trial(sim, bounds, goal_radii, eta, 0.0, seed)
            log("scalability", f"E8_swap_K{K}.yaml", n, 0.0, seed, ok, elapsed)

    # Table 4: robustness/correctness probes -- E9 (compound 2D aperture),
    # E10 (obstacle+swap combined), E11 (dead-end trap), E12 (partial-goal
    # correctness), E13 (no-solution termination correctness)
    extra_envs = [
        ("E9_3d_aperture.yaml", 25),
        ("E10_swap_corridor.yaml", 25),
        ("E11_dead_end.yaml", 25),
        ("E12_partial_goal.yaml", 25),
        ("E13_no_solution.yaml", 10),
    ]
    for env_name, n_seeds in extra_envs:
        sim, bounds, goal_radii, eta, n = load_env(f"environments/{env_name}")
        for seed in range(n_seeds):
            ok, elapsed = run_trial(sim, bounds, goal_radii, eta, 0.0, seed)
            log("extra", env_name, n, 0.0, seed, ok, elapsed)

    f.close()
    os.replace(temporary_path, OUT_PATH)

    # Representative successful paths for make_figures.py's path-overlay
    # figures -- one per environment, except E9/E11/E13 which have 0% success
    # (0/25, 0/25, and 0/10; see the "extra" table above) and so have
    # no successful path to show. Tries a few seeds in case seed=0 happens
    # to be one of the rare failures for a given environment.
    path_envs = [
        "E0_empty.yaml", "E1_sparse.yaml", "E2_moderate.yaml", "E3_dense.yaml",
        "E4_bottleneck_moderate.yaml", "E5_bottleneck_severe.yaml", "E6_multi_gap.yaml",
        "E7_offset_passage.yaml", "E8_swap_K2.yaml", "E8_swap_K3.yaml",
        "E8_swap_K4.yaml", "E8_swap_K5.yaml", "E10_swap_corridor.yaml", "E12_partial_goal.yaml",
    ]
    for env_name in path_envs:
        sim, bounds, goal_radii, eta, n = load_env(f"environments/{env_name}")
        path, seed_used, plan_time = None, None, None
        for seed in range(20):
            rng = np.random.default_rng(seed)
            t0 = time.perf_counter()
            path = rrt_connect(sim, bounds, goal_radii, eta, TIME_LIMIT, rng, bridge_prob=0.0)
            plan_time = time.perf_counter() - t0
            if path is not None:
                seed_used = seed
                break
        if path is not None:
            arr = np.stack(path)
            out = os.path.join(os.path.dirname(__file__), f"path_{env_name.replace('.yaml', '')}.npy")
            np.save(out, arr)
            print(f"saved {out} shape={arr.shape} seed={seed_used} "
                  f"planning_time={plan_time:.2f}s waypoints={len(path)}")
        else:
            print(f"WARNING: no success in 20 seeds for {env_name}, no path saved")

    print("SKIPPED (0% success, nothing to show): E9_3d_aperture.yaml, "
          "E11_dead_end.yaml, E13_no_solution.yaml")
    print("DONE")


if __name__ == "__main__":
    main()
