# Course alignment and method choice

`results/course_inventory.json` inventories 41 PDFs from the local
`Documents/26 s2/COMP8620` course directory, with source names, page counts,
hashes and method keywords. The model and solver design uses the relevant
MDP lectures and tutorial exercises; later learning methods are considered as
alternatives rather than added without a training/evaluation protocol.

| Material | Role in this implementation |
| --- | --- |
| Weeks 1–3 motion planning; A1 Part B | Joint configurations, collision semantics, geometry-based guidance, systematic scenario design. A1 RRT-Connect is a deterministic relaxation, not the stochastic solver. |
| Week 4 / Week 5 frameworks under uncertainty | Observable Markov state, known stochastic transitions, rewards, terminal states and finite execution horizon. |
| Week 5 MDP Solving / Week 6 continuation | Use online generative-model UCT for the joint problem, and finite-horizon single-drone value iteration for action guidance and leaf estimates. Avoid full joint Bellman backups. Implement all four MCTS components explicitly. |
| Tutorial Week 6 | Compact Markov state, transition probability accounting, Bellman/RTDP reasoning, and consistent terminal handling. |
| Weeks 6–8 POMDP solving; Tutorials Weeks 7–8 | Fully observed A2 Part B does not require alpha-vector belief-space solving or POMCP. The Week 8 RNG question motivates careful sampling and separate search/execution streams. |
| Weeks 8–10 RL basics; Tutorials Weeks 9–10 | Incremental return averages, distinction between Monte Carlo return backups and TD/Q-learning, seed-controlled experiments. No offline training is claimed. |
| Week 11 DQN / policy gradients / DDPG / SAC / PPO | Not selected for the default solver: they need a separate training and generalization study; some target continuous control rather than this supplied discrete generative MDP. |

UCT selection, stochastic successor sampling, rollout evaluation and mean
backups follow the online-MDP lectures. Progressive action widening is an
explicit implementation extension for `26**N` joint commands. The default
guidance uses single-drone finite-horizon Bellman updates (Week 5, slide 9)
with the supplied slip law and discounted reward. The single-drone relaxation,
occupancy reservations and root switching margin are implementation choices,
not classroom guarantees about the joint problem. Risk-weighted Dijkstra
guidance is the earlier comparison and the fallback on large maps; it
instantiates the lecture's deterministic-solution rollout suggestion.

The report explains the approximation and tests guidance, leaf values and
widening as separate ablations. It does not claim that every added component
must improve every environment or every finite-budget run.
