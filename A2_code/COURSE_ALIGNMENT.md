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
| Week 5 MDP Solving / Week 6 continuation | Choose online generative-model UCT over full value/policy iteration and exact RTDP backups. Implement all four MCTS components explicitly. |
| Tutorial Week 6 | Compact Markov state, transition probability accounting, Bellman/RTDP reasoning, and consistent terminal handling. |
| Weeks 6–8 POMDP solving; Tutorials Weeks 7–8 | Fully observed A2 Part B does not require alpha-vector belief-space solving or POMCP. The Week 8 RNG question motivates careful sampling and separate search/execution streams. |
| Weeks 8–10 RL basics; Tutorials Weeks 9–10 | Incremental return averages, distinction between Monte Carlo return backups and TD/Q-learning, seed-controlled experiments. No offline training is claimed. |
| Week 11 DQN / policy gradients / DDPG / SAC / PPO | Not selected for the default solver: they need a separate training and generalization study; some target continuous control rather than this supplied discrete generative MDP. |

UCT selection, stochastic successor sampling, rollout evaluation and mean
backups follow the online-MDP lectures. Progressive action widening is an
explicit implementation extension for `26**N` joint commands. Static
single-drone goal-distance maps instantiate the lecture's suggestion to use
a deterministic problem solution for rollout guidance. They do not replace
the teacher's stochastic model.

The report explains the approximation and tests guidance, leaf values and
widening as separate ablations. It does not claim that every added component
must improve every environment or every finite-budget run.
