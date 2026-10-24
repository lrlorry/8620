"""Serial, generative-model UCT with risk-aware guided widening and rollouts.

The only simulator interfaces used here are get_config(), simulate(), and
num_actions (property in the supplied implementation; methods also work).
Slip probabilities used by the guidance are recomputed from the public
configuration (obstacles and alpha), not read from the simulator.
"""

from collections import deque
from dataclasses import dataclass, field
import heapq
from itertools import product
import math
import time

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.sparse import csr_matrix

UNREACHABLE = 1e7   # statically disconnected from the goal
RESERVED = 1e9      # another drone's planned or frozen cell
INVALID = 1e12      # outside the grid or an obstacle cell


def action_vectors(change_altitude):
    return np.asarray([v for v in product((-1, 0, 1), repeat=3)
                       if any(v) and (change_altitude or v[2] == 0)], dtype=np.int32)


def encode_action(digits, base):
    value, multiplier = 0, 1
    for digit in digits:
        value += int(digit) * multiplier
        multiplier *= base
    return value


def decode_action(value, base, count):
    if not 0 <= int(value) < base ** count:
        raise ValueError('Action is outside the joint action space')
    digits = []
    for _ in range(count):
        digits.append(int(value) % base)
        value = int(value) // base
    return digits


@dataclass
class Edge:
    visits: int = 0
    value: float = 0.0
    spread: float = 0.0  # Welford sum of squared deviations of the sampled returns
    children: dict = field(default_factory=dict)

    def add(self, value):
        self.visits += 1
        delta = value - self.value
        self.value += delta / self.visits
        self.spread += delta * (value - self.value)


@dataclass
class Node:
    visits: int = 0
    actions: dict = field(default_factory=dict)


class Deadline(Exception):
    pass


class MCTSPlanner:
    def __init__(self, env, seed=0, tree_depth=8, rollout_depth=8,
                 exploration=0.35, widening=0.5, random_action_prob=0.15,
                 rollout_epsilon=0.10, max_simulations=2000,
                 guided=True, leaf_heuristic=True, progressive=True,
                 risk_weight=1.0, switch_z=1.0, guidance='vi', max_map_cells=150000,
                 max_value_cells=20000, budget_cap=10.):
        self.env = env
        self.cfg = env.get_config()
        self.goals = np.asarray(self.cfg.goal_positions, dtype=np.int32)
        self.count = len(self.goals)
        self.shape = tuple(self.cfg.grid_size)
        self.physical_vectors = action_vectors(self.cfg.change_altitude)
        # Optional action masks are used ONLY in controlled experiments.
        allowed = getattr(self.cfg, 'intended_action_indices', None)
        self.vectors = self.physical_vectors if allowed is None else self.physical_vectors[allowed]
        self.base = len(self.vectors)
        advertised = env.num_actions
        self.action_count = int(advertised() if callable(advertised) else advertised)
        if self.action_count != self.base ** self.count:
            raise ValueError('Action count does not match the documented per-drone encoding')
        if min(tree_depth, rollout_depth, max_simulations, risk_weight) < 0:
            raise ValueError('Depth, simulation and risk parameters must be nonnegative')
        self.tree_depth, self.rollout_depth = int(tree_depth), int(rollout_depth)
        self.exploration, self.widening = float(exploration), float(widening)
        self.random_action_prob = float(random_action_prob)
        self.rollout_epsilon = float(rollout_epsilon)
        self.max_simulations = int(max_simulations)
        self.guided, self.leaf_heuristic, self.progressive = guided, leaf_heuristic, progressive
        self.budget_cap = float(budget_cap)
        self.switch_z = None if switch_z is None else float(switch_z)
        if guidance not in ('dijkstra', 'vi'):
            raise ValueError(f'Unknown guidance: {guidance}')
        self.guidance = guidance
        self.rng = np.random.default_rng(seed)
        self.gamma = float(self.cfg.discount_factor)
        self.scale = max(1., self.count * (abs(self.cfg.goal_reward) +
                                         2 * abs(self.cfg.collision_penalty)))
        # A collision costs its penalty and forfeits the goal reward; express it in step costs.
        loss = abs(self.cfg.collision_penalty) + abs(self.cfg.goal_reward)
        self.risk_weight = float(risk_weight)
        self.risk_unit = max(abs(self.cfg.step_cost), 1e-3 * loss, 1e-9)
        self.risk_scale = self.risk_weight * loss / self.risk_unit
        self.blocked = {tuple(p) for p in (self.cfg.obstacle_cells or [])}
        self.maps = self.costs = self.clearance = self.free = None
        self.land_intended = self.land_slip = self.departure_risk = None
        if math.prod(self.shape) <= max_map_cells:
            free = np.ones(self.shape, dtype=bool)
            for position in self.blocked:
                free[position] = False
            self.free, self.clearance = free, distance_transform_edt(free)
            self._slip_model(free)
            self.maps = [self._distance_map(tuple(g), free) for g in self.goals]
            self.costs = [self._cost_map(tuple(g), free, m) for g, m in zip(self.goals, self.maps)]
        self.values = self.q_history = None
        if guidance == 'vi' and self.free is not None and self.free.size <= max_value_cells:
            self._value_iteration()
        self._options_cache, self._adjacent, self._frozen_maps = {}, {}, {}
        self.steps = 0
        self.last_stats = {}

    def _neighbour_sum(self, values):
        X, Y, Z = self.shape
        padded = np.pad(values.astype(float), 1)
        total = np.zeros(self.shape)
        for dx, dy, dz in self.physical_vectors:
            total += padded[1+dx:1+dx+X, 1+dy:1+dy+Y, 1+dz:1+dz+Z]
        return total

    def _slip_model(self, free):
        # Same rule as the supplied model: success d/(d+alpha), uniform slips, renormalised in-grid.
        alpha = float(self.cfg.alpha)
        success = self.clearance / np.maximum(self.clearance + alpha, 1e-12)
        slip = (1 - success) / max(1, len(self.physical_vectors) - 1)
        valid = self._neighbour_sum(np.ones(self.shape, dtype=bool))
        mass = np.maximum(success + (valid - 1) * slip, 1e-12)
        self.land_intended, self.land_slip = success / mass, slip / mass
        # Probability that one departure (towards a free cell) slips into an obstacle.
        self.departure_risk = self._neighbour_sum(~free) * self.land_slip

    def _value_iteration(self):
        # Finite-horizon single-drone values on the supplied slip law (value iteration); each
        # drone is solved alone, so collisions between drones are left to the search.
        cells = np.argwhere(self.free)
        index = np.full(self.shape, -1, dtype=np.int64)
        index[tuple(cells.T)] = np.arange(len(cells))
        n, B, A = len(cells), len(self.physical_vectors), self.base
        commands = [int(np.flatnonzero((self.physical_vectors == v).all(axis=1))[0]) for v in self.vectors]
        targets = cells[:, None, :] + self.physical_vectors[None, :, :]
        valid = ((targets >= 0) & (targets < np.asarray(self.shape))).all(axis=2)
        clipped = np.minimum(np.maximum(targets, 0), np.asarray(self.shape) - 1)
        lands = valid & self.free[tuple(np.moveaxis(clipped, 2, 0))]
        target_index = np.where(lands, index[tuple(np.moveaxis(clipped, 2, 0))], -1)
        clearance = self.clearance[tuple(cells.T)]
        success = clearance / np.maximum(clearance + float(self.cfg.alpha), 1e-12)
        rows, cols, probs, crash = [], [], [], np.zeros(n * A)
        for a, j in enumerate(commands):
            weights = np.repeat(((1 - success) / max(1, B - 1))[:, None], B, axis=1)
            weights[:, j] = success
            weights[~valid] = 0.
            weights /= np.maximum(weights.sum(axis=1, keepdims=True), 1e-12)
            crash[a::A] = (weights * (valid & ~lands)).sum(axis=1)
            k, b = np.nonzero(lands & (weights > 0))
            rows.append(k * A + a); cols.append(target_index[k, b]); probs.append(weights[k, b])
        P = csr_matrix((np.concatenate(probs), (np.concatenate(rows), np.concatenate(cols))), shape=(n * A, n))
        horizon = max(1, int(self.cfg.max_num_steps))
        if n * A * (horizon + 1) * len(self.goals) > 2.5e7:
            return  # Too large to keep per-horizon action values; use the Dijkstra guidance.
        step, hit, prize = self.cfg.step_cost, self.cfg.collision_penalty, self.cfg.goal_reward
        self.value_index, self.values, self.q_history = index, [], []
        for goal in self.goals:
            g = index[tuple(goal)]
            reward = step + hit * crash + (prize * P[:, g].toarray().ravel() if g >= 0 else 0.)
            V, history = np.zeros(n), [np.zeros(n)]
            actions = [np.zeros((n, A), dtype=np.float32)]
            for _ in range(horizon):
                Q = (reward + self.gamma * (P @ V)).reshape(n, A)
                V = Q.max(axis=1)
                if g >= 0:
                    V[g] = 0.  # Arrival is terminal for this drone; its reward is in the step.
                history.append(V)
                actions.append(Q.astype(np.float32))
            self.values.append(np.asarray(history))  # values[i][h]: h steps left
            self.q_history.append(np.asarray(actions))  # q_history[i][h]: action values, h left

    def _distance_map(self, goal, free):
        distance = np.full(self.shape, -1, dtype=np.int32)
        if not free[goal]:
            return distance
        queue = deque([goal])
        distance[goal] = 0
        vectors = self.physical_vectors.tolist()
        X, Y, Z = self.shape
        while queue:
            p = queue.popleft()
            d = distance[p] + 1
            for dx, dy, dz in vectors:
                q = p[0]+dx, p[1]+dy, p[2]+dz
                if 0 <= q[0] < X and 0 <= q[1] < Y and 0 <= q[2] < Z:
                    if distance[q] < 0 and free[q]:
                        distance[q] = d
                        queue.append(q)
        return distance

    def _cost_map(self, goal, free, steps):
        # Risk-weighted cost-to-go: entering a non-goal cell costs 1 + lambda * departure risk.
        if self.risk_scale == 0:
            return np.where(steps >= 0, steps.astype(float), math.inf)
        cost = np.full(self.shape, math.inf)
        if not free[goal]:
            return cost
        enter = (1. + self.risk_scale * self.departure_risk).tolist()
        cost[goal] = 0.
        heap = [(0., goal)]
        vectors = self.physical_vectors.tolist()
        X, Y, Z = self.shape
        while heap:
            c, p = heapq.heappop(heap)
            if c > cost[p]:
                continue
            c += 1. if p == goal else enter[p[0]][p[1]][p[2]]
            for dx, dy, dz in vectors:
                q = p[0]+dx, p[1]+dy, p[2]+dz
                if 0 <= q[0] < X and 0 <= q[1] < Y and 0 <= q[2] < Z and free[q] and c < cost[q]:
                    cost[q] = c
                    heapq.heappush(heap, (c, q))
        return cost

    def reset(self):
        self.steps = 0
        self.last_stats = {}

    def _distance(self, i, position):
        if self.maps is None:
            delta = np.abs(np.asarray(position) - self.goals[i])
            if not self.cfg.change_altitude and delta[2]:
                return math.inf
            return float(delta.max())
        d = self.maps[i][tuple(position)]
        return float(d) if d >= 0 else math.inf

    def _cost(self, i, position):
        if self.costs is None:
            return self._distance(i, position)
        return float(self.costs[i][tuple(position)])

    def unreachable_drones(self, state):
        if self.maps is None:
            return []
        return [i for i in range(self.count) if not state[i, 3] and
                (not math.isfinite(self._distance(i, state[i, :3])) or self._cut_off(i, state))]

    def _cut_off(self, i, state):
        # True if drones frozen at their goals disconnect drone i from its own goal.
        frozen = tuple(j for j in range(self.count) if state[j, 3] and j != i)
        if not frozen or self.free is None or self.free.size > 20000:
            return False
        key = (i, frozen)
        if key not in self._frozen_maps:
            free = self.free.copy()
            for j in frozen:
                free[tuple(self.goals[j])] = False
            self._frozen_maps[key] = self._distance_map(tuple(self.goals[i]), free)
        return self._frozen_maps[key][tuple(state[i, :3])] < 0

    def _options(self, i, position, remaining=None):
        cell = tuple(map(int, position))
        if self.q_history is not None:
            return self._value_options(i, cell, remaining)
        key = (i, *cell)
        if key in self._options_cache:
            return self._options_cache[key]
        targets = position + self.vectors
        scores = np.full(self.base, INVALID, dtype=float)
        for j, target in enumerate(targets):
            point = tuple(map(int, target))
            if all(0 <= point[k] < self.shape[k] for k in range(3)) and point not in self.blocked:
                cost = self._cost(i, point)
                scores[j] = cost if math.isfinite(cost) else UNREACHABLE
                scores[j] += 0.025 * np.abs(target - self.goals[i]).sum()
                if self.clearance is not None:
                    scores[j] += 0.3 / max(1., self.clearance[point])
        result = [tuple(map(int, q)) for q in targets], scores, int(np.count_nonzero(scores < INVALID))
        self._options_cache[key] = result
        return result

    def _value_options(self, i, cell, remaining):
        # Scores are the single-drone action values for the steps actually left (step-cost units).
        longest = len(self.q_history[i]) - 1
        h = longest if remaining is None else max(1, min(longest, int(remaining)))
        key = (i, *cell, h)
        if key in self._options_cache:
            return self._options_cache[key]
        static = self._options_cache.get((i, *cell))
        if static is None:
            targets = np.asarray(cell) + self.vectors
            inside = ((targets >= 0) & (targets < np.asarray(self.shape))).all(axis=1)
            valid = np.array([ok and tuple(map(int, t)) not in self.blocked for ok, t in zip(inside, targets)])
            tie = 0.025 * np.abs(targets - self.goals[i]).sum(axis=1)
            static = [tuple(map(int, t)) for t in targets], valid, tie
            self._options_cache[(i, *cell)] = static
        targets, valid, tie = static
        q = self.q_history[i][h][self.value_index[cell]]
        scores = np.where(valid, -q / self.risk_unit + tie, INVALID)
        result = targets, scores, int(valid.sum())
        self._options_cache[key] = result
        return result

    def _slip_penalty(self, position, others):
        # Expected-loss surcharge if another active drone could slip into this target now.
        if self.land_slip is None or self.risk_scale == 0:
            return 0.
        penalty = 0.
        for other in others:
            delta = (position[0]-other[0], position[1]-other[1], position[2]-other[2])
            if max(abs(delta[0]), abs(delta[1]), abs(delta[2])) > 2:
                continue  # No single move ends next to a drone three or more cells away.
            if not self.cfg.change_altitude and delta[2]:
                continue  # With fixed altitude, slips stay in the drone's own layer.
            mask = self._adjacent.get(delta)
            if mask is None:
                mask = (np.abs(self.vectors + np.asarray(delta)).max(axis=1) == 1).astype(float)
                self._adjacent[delta] = mask
            penalty = penalty + self.land_slip[other] * mask
        return self.risk_scale * self.land_intended[position] * penalty

    def _proposal(self, state, randomize=True, explore=None, remaining=None):
        reached = state[:, 3].astype(bool)
        if not self.guided:
            digits = self.rng.integers(self.base, size=self.count)
            digits[reached] = 0  # Reached drones are frozen; equivalent labels collapse.
            return encode_action(digits, self.base)
        if explore is None:
            explore = randomize and self.rng.random() < self.random_action_prob
        cells = [tuple(map(int, p)) for p in state[:, :3]]
        reserved = {cells[i] for i in np.flatnonzero(reached)}
        active = np.flatnonzero(~reached)
        if randomize or explore:
            active = self.rng.permutation(active)
        digits = np.zeros(self.count, dtype=int)
        for i in active:
            targets, cost, _ = self._options(int(i), state[i, :3], remaining)
            cost = cost.copy()
            for j, target in enumerate(targets):
                if target in reserved:
                    cost[j] += RESERVED
            if explore:
                # Uniform per-drone exploration over admissible (in-grid, free, unreserved) cells.
                admissible = np.flatnonzero(cost < RESERVED)
                chosen = int(self.rng.choice(admissible)) if len(admissible) else int(np.argmin(cost))
            else:
                cost += self._slip_penalty(cells[i], [cells[k] for k in active if k != i])
                order = np.argsort(cost, kind='stable')
                chosen = int(order[0])
                if randomize:
                    pool = order[cost[order] <= cost[chosen] + 1.05][:4]
                    if len(pool) > 1:
                        weights = np.exp(-3. * (cost[pool] - cost[chosen]))
                        chosen = int(self.rng.choice(pool, p=weights/weights.sum()))
            digits[i] = chosen
            reserved.add(targets[chosen])
        return encode_action(digits, self.base)

    def _effective_actions(self, state):
        # Guided search never admits commands towards invalid cells; count only proposable ones.
        active = np.flatnonzero(state[:, 3] == 0)
        if not self.guided:
            return self.base ** len(active)
        return math.prod(self._options(int(i), state[i, :3])[2] for i in active)

    def _leaf(self, state, remaining):
        if not self.leaf_heuristic or remaining <= 0:
            return 0.
        distances = [math.inf if self._cut_off(i, state) else self._distance(i, state[i, :3])
                     for i in range(self.count) if not state[i, 3]]
        if not distances:
            return 0.
        horizon = min(remaining, max(distances))
        if not math.isfinite(horizon):
            horizon = remaining
        horizon = int(horizon)
        costs = horizon if self.gamma == 1 else (1 - self.gamma**horizon)/(1-self.gamma)
        if self.values is not None:
            # Single-drone values for active drones; reached drones keep paying the step cost.
            h = min(int(remaining), len(self.values[0]) - 1)
            stuck = h if self.gamma == 1 else (1 - self.gamma**h)/(1-self.gamma)
            value = self.cfg.step_cost * (self.count - len(distances)) * costs
            for i in range(self.count):
                if not state[i, 3]:
                    value += (self.cfg.step_cost * stuck if self._cut_off(i, state) else
                              self.values[i][h][self.value_index[tuple(state[i, :3])]])
            return float(value) / self.scale
        value = self.cfg.step_cost * self.count * costs
        value += self.cfg.goal_reward * sum(self.gamma**(int(d)-1)
                                            for d in distances if 1 <= d <= remaining)
        return float(value) / self.scale

    def _simulate(self, state, action):
        if time.perf_counter() >= self.deadline:
            raise Deadline
        self.calls += 1
        return self.env.simulate(state, action)

    def _rollout(self, state, remaining):
        rewards = []
        terminal = False
        for _ in range(min(self.rollout_depth, remaining)):
            if time.perf_counter() >= self.deadline:
                break
            explore = self.rng.random() < self.rollout_epsilon
            action = self._proposal(state, randomize=False, explore=explore, remaining=remaining-len(rewards))
            state, reward, done, _ = self._simulate(state, action)
            rewards.append(float(reward)/self.scale)
            if done:
                terminal = True
                break
        tail = 0. if terminal else self._leaf(state, remaining-len(rewards))
        for reward in reversed(rewards):
            tail = reward + self.gamma * tail
        return tail

    def _search(self, node, state, remaining, depth):
        if remaining <= 0 or state[:, 3].all():
            return 0.
        if depth >= self.tree_depth:
            return self._rollout(state, remaining)
        if time.perf_counter() >= self.deadline:
            raise Deadline
        width = (max(1, math.ceil(self.widening * math.sqrt(node.visits+1)))
                 if self.progressive else math.inf)
        if len(node.actions) < width and len(node.actions) < self._effective_actions(state):
            action = self._proposal(state, randomize=bool(node.actions), remaining=remaining)
            for _ in range(12):
                if action not in node.actions:
                    node.actions[action] = Edge()
                    break
                action = self._proposal(state, randomize=True, remaining=remaining)
        if not node.actions:
            action = self._proposal(state, randomize=False, remaining=remaining)
            node.actions[action] = Edge()
        unseen = [a for a, edge in node.actions.items() if edge.visits == 0]
        if unseen:
            action = unseen[0]
        else:
            log_n = math.log(node.visits + 1)
            action = max(node.actions, key=lambda a: node.actions[a].value +
                         self.exploration * math.sqrt(log_n/node.actions[a].visits))
        edge = node.actions[action]
        successor, reward, done, _ = self._simulate(state, action)
        if done:
            tail = 0.
        else:
            key = np.asarray(successor, dtype=np.int32).tobytes()
            child = edge.children.get(key)
            if child is None:
                child = Node()
                edge.children[key] = child
                self.nodes += 1
                self.depth_sum += depth + 1
                tail = self._rollout(successor, remaining-1)
            else:
                tail = self._search(child, successor, remaining-1, depth+1)
        value = float(reward)/self.scale + self.gamma * tail
        edge.add(value)
        node.visits += 1
        return value

    def _anchored(self, edges, base, visited):
        # Keep the greedy base action unless another beats it by z pooled standard errors.
        dof = sum(edges[a].visits - 1 for a in visited)
        sigma = math.sqrt(sum(edges[a].spread for a in visited) / dof) if dof > 0 else 0.
        reference, best, gain = edges[base], base, 0.
        for a in visited:
            margin = (edges[a].value - reference.value - self.switch_z * sigma *
                      math.sqrt(1/edges[a].visits + 1/reference.visits))
            if margin > gain:
                best, gain = a, margin
        return best

    def plan(self, current_state, planning_time_per_step=9.5):
        start = time.perf_counter()
        budget = float(planning_time_per_step)
        if not math.isfinite(budget) or budget < 0:
            raise ValueError('Planning budget must be finite and nonnegative')
        state = np.asarray(current_state, dtype=np.int32)
        if state.shape != (self.count, 4):
            raise ValueError('State must have one (x,y,z,reached) row per drone')
        budget = min(budget, self.budget_cap)
        self.deadline = start + max(0., 0.97 * budget - 0.002)
        self.calls, self.nodes, self.depth_sum = 0, 1, 0
        if len(self._options_cache) > 50000:
            self._options_cache.clear()
        remaining = max(0, int(self.cfg.max_num_steps) - self.steps)
        fallback = self._proposal(state, randomize=False, remaining=remaining)
        root = Node()
        completed = 0
        try:
            while (completed < self.max_simulations and remaining > 0
                   and not state[:, 3].all() and time.perf_counter() < self.deadline):
                self._search(root, state, remaining, 0)
                completed += 1
        except Deadline:
            pass
        visited = [a for a, edge in root.actions.items() if edge.visits]
        if not visited:
            action = fallback
        elif self.switch_z is None or not root.actions.get(fallback, Edge()).visits:
            action = max(visited, key=lambda a: (root.actions[a].visits, root.actions[a].value))
        else:
            action = self._anchored(root.actions, fallback, visited)
        top = root.actions[action].visits if visited else 0
        self.last_stats = dict(elapsed=time.perf_counter()-start, simulations=completed,
                               generative_calls=self.calls, nodes=self.nodes,
                               root_actions=len(root.actions), root_visits=root.visits,
                               top_share=top/max(1, root.visits), greedy=int(action == fallback),
                               mean_new_depth=self.depth_sum/max(1, self.nodes-1),
                               action=int(action), budget=budget)
        self.steps += 1
        return int(action)
