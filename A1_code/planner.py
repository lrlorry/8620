"""Centralised RRT-Connect planner for the multi-drone problem (Part B, Q2/Q3).

Implements exactly the RRT-CONNECT / EXTEND / CONNECT pseudocode from the
report's B2 section. Every accepted tree edge is validated by
sim.motion_valid, so any path returned by rrt_connect is collision-free by
construction.
"""

import time

import numpy as np

TRAPPED, ADVANCED, REACHED, TIMEOUT = range(4)


class Node:
    __slots__ = ("config", "parent")

    def __init__(self, config, parent=None):
        self.config = config
        self.parent = parent


def distance(q1, q2):
    """d(q, q') = max_i ||p_i - p_i'||_2 (B2): the distance travelled by the
    slowest-converging drone, which is also exactly the quantity
    sim.motion_valid uses to decide how many interpolation steps a motion
    needs -- so 'nearest' under this metric means 'cheapest to validate'."""
    return float(np.linalg.norm(q1 - q2, axis=1).max())


def steer(q_near, q_target, eta):
    """Steer(q, q') = q + lambda (q' - q), lambda = min(1, eta / d(q, q'))."""
    d = distance(q_near, q_target)
    if d <= eta:
        return q_target.copy()
    lam = eta / d
    return (q_near + lam * (q_target - q_near)).astype(np.float32)


def nearest(tree, q_target):
    """Brute-force linear scan (B2): at K <= 5 drones, a single
    sim.motion_valid call is far more expensive than scanning the tree, so a
    spatial index buys negligible speedup at this scale."""
    return min(tree, key=lambda node: distance(node.config, q_target))


def sample_free(sim, bounds, num_drones, rng, deadline, bridge_prob=0.0, bridge_sigma=None, tree=None):
    """SAMPLE-FREE: uniform sample within the workspace bounds, rejected
    until sim.is_valid holds or the deadline passes.

    With probability `bridge_prob`, a bridge-test attempt (Hsu, Jiang,
    Kavraki & Latombe, "The Bridge Test for Sampling Narrow Passages with
    Probabilistic Roadmap Planners", ICRA 2003) is tried first: this is a
    narrow-passage-biased sampling heuristic from the wider motion-planning
    literature, NOT covered in the COMP4620/8620 weeks 1-3 material (which
    only covers uniform sampling for PRM/EST/RRT and the theoretical
    (epsilon, alpha, beta)-expansiveness analysis of why narrow passages are
    hard). bridge_prob defaults to 0.0 so the baseline B2 behaviour is
    reproduced exactly unless explicitly enabled.

    When `tree` (a list of Node) is also given, the bridge-test's first
    probe is anchored near an existing node of that tree instead of drawn
    uniformly from the whole workspace -- combining the bridge test with
    the EST idea taught in week3 ("select a node q to expand ... sample q'
    uniformly at random from all possible configurations within D distance
    from q"). A tree that has grown up to one narrow passage tends to hug
    that passage, so anchoring near its frontier makes a second, sequential
    passage far more likely to be found than a blind global draw."""
    lower, upper = bounds[:, 0], bounds[:, 1]
    while time.perf_counter() < deadline:
        if bridge_prob > 0.0 and rng.random() < bridge_prob:
            q = _bridge_attempt(sim, bounds, num_drones, rng, bridge_sigma, tree)
            if q is not None:
                return q
            continue
        q = rng.uniform(lower, upper, size=(num_drones, 3)).astype(np.float32)
        if sim.is_valid(q):
            return q
    return None


def _bridge_attempt(sim, bounds, num_drones, rng, sigma, tree=None):
    """One bridge-test draw: q1 (uniform, or EST-style near an existing tree
    node when `tree` is given); require q1 invalid; q2 ~ N(q1, sigma);
    require q2 also invalid; return the midpoint if it is valid, else None.
    A valid midpoint flanked by two invalid configurations is likely to sit
    inside a narrow passage between two obstacles."""
    lower, upper = bounds[:, 0], bounds[:, 1]
    if tree:
        anchor = tree[rng.integers(len(tree))].config
        q1 = (anchor + rng.normal(scale=sigma, size=(num_drones, 3))).astype(np.float32)
        np.clip(q1, lower, upper, out=q1)
    else:
        q1 = rng.uniform(lower, upper, size=(num_drones, 3)).astype(np.float32)
    if sim.is_valid(q1):
        return None
    q2 = (q1 + rng.normal(scale=sigma, size=(num_drones, 3))).astype(np.float32)
    np.clip(q2, lower, upper, out=q2)
    if sim.is_valid(q2):
        return None
    midpoint = ((q1 + q2) / 2.0).astype(np.float32)
    if sim.is_valid(midpoint):
        return midpoint
    return None


def valid_goal_sample(sim, goal_radii, rng, deadline):
    """VALID-GOAL-SAMPLE: try the goal centres first, falling back to a
    random point inside each drone's own goal ball until the joint sample
    is collision-free."""
    centres = sim.goal_positions.astype(np.float32)
    if sim.is_valid(centres):
        return centres

    num_drones = centres.shape[0]
    while time.perf_counter() < deadline:
        directions = rng.normal(size=(num_drones, 3))
        directions /= np.linalg.norm(directions, axis=1, keepdims=True)
        radii = rng.uniform(0.0, goal_radii, size=num_drones)
        q = (centres + directions * radii[:, None]).astype(np.float32)
        if sim.is_valid(q):
            return q
    return None


def extend(tree, q_target, eta, sim):
    """EXTEND: grow one step of `tree` towards q_target."""
    near = nearest(tree, q_target)
    if distance(near.config, q_target) == 0.0:
        return REACHED, near
    q_new = steer(near.config, q_target, eta)
    if not sim.motion_valid(near.config, q_new):
        return TRAPPED, near
    node = Node(q_new, parent=near)
    tree.append(node)
    if np.array_equal(q_new, q_target):
        return REACHED, node
    return ADVANCED, node


def connect(tree, q_target, eta, sim, deadline):
    """CONNECT: repeatedly EXTEND `tree` towards q_target until it is
    TRAPPED, REACHED, or the deadline passes. The deadline is checked
    *before* every EXTEND, including the first, so CONNECT never starts a
    new motion_valid call once time is already up."""
    status, node = ADVANCED, nearest(tree, q_target)
    while status == ADVANCED and time.perf_counter() < deadline:
        status, node = extend(tree, q_target, eta, sim)
    if status == ADVANCED:
        return TIMEOUT, node
    return status, node


def _trace_to_root(node):
    """Follow parent pointers from `node` back to its tree's root,
    returning the branch in root -> node order."""
    path = []
    while node is not None:
        path.append(node.config)
        node = node.parent
    path.reverse()
    return path


def join_paths(node_a, node_b, a_is_init):
    """JOIN-PATHS: splice the branch ending at node_a with the branch ending
    at node_b into a single q_I -> q_G path. `a_is_init` records which of
    the two (since T_a/T_b are swapped every iteration) is currently rooted
    at q_I, resolving the ambiguity noted in the B2 write-up."""
    branch_a = _trace_to_root(node_a)
    branch_b = _trace_to_root(node_b)
    if a_is_init:
        return branch_a + branch_b[::-1]
    return branch_b + branch_a[::-1]


def rrt_connect(sim, bounds, goal_radii, eta, time_limit, rng, bridge_prob=0.0, bridge_sigma=None, local_bridge=False):
    """RRT-CONNECT (B2): returns a list of (num_drones, 3) waypoints from
    the initial configuration to a valid goal sample, or None on failure.

    bridge_prob/bridge_sigma (default 0.0 / eta) enable the bridge-test
    narrow-passage sampling augmentation in sample_free -- see its
    docstring. local_bridge additionally anchors bridge attempts near the
    currently-extending tree's own nodes (EST-style) instead of drawing
    globally, which helps with sequential/compound narrow passages.
    Leaving bridge_prob at 0.0 reproduces the B2 baseline exactly."""
    deadline = time.perf_counter() + time_limit
    num_drones = sim.initial_configuration.shape[0]
    if bridge_sigma is None:
        bridge_sigma = eta

    q_I = sim.initial_configuration.astype(np.float32)
    q_G = valid_goal_sample(sim, goal_radii, rng, deadline)
    if q_G is None or not sim.is_valid(q_I):
        return None
    if sim.motion_valid(q_I, q_G):
        return [q_I, q_G]

    trees = [[Node(q_I)], [Node(q_G)]]  # trees[0] is always the init-side tree
    a, b = 0, 1

    while time.perf_counter() < deadline:
        q_rand = sample_free(
            sim, bounds, num_drones, rng, deadline, bridge_prob, bridge_sigma,
            tree=trees[a] if local_bridge else None,
        )
        if q_rand is None:
            return None

        status, node_new = extend(trees[a], q_rand, eta, sim)
        if status != TRAPPED:
            status_b, node_meet = connect(trees[b], node_new.config, eta, sim, deadline)
            if status_b == REACHED:
                return join_paths(node_new, node_meet, a_is_init=(a == 0))

        a, b = b, a

    return None
