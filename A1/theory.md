# COMP8620 A1 Oral Theory MASTER

> 用途：准备 A1 in-person Q&A 中除程序 demo 以外的理论问答。  
> 范围：A1 specification、Week 1--3 lectures、Tutorial Week 2--3、A1 Part A/B。  
> 配套代码稿：`COMP8620_A1_Codewalk_MASTER.md`。本文件讲理论，代码稿讲真实实现。

---

# 0. 先明确口试范围

Assignment 把现场考核分为两部分：

1. B3 program demo；
2. 与 assignment 以及 Week 1--3 全部材料有关的 oral Q&A。

因此只会运行程序还不够。至少要能从下面这条主线解释：

```text
robot and motion-planning problem
-> configuration and C-space
-> C_obs and C_free
-> deterministic discretisation
-> sampling-based representation
-> state and edge validity
-> PRM / EST / RRT
-> probabilistic completeness
-> narrow passages and expansiveness
-> centralised multi-drone RRT-Connect
```

## 0.1 复习优先级

**P0：必须脱稿**

- configuration、C-space、DOF、workspace 的区别；
- `C_obs`、`C_free` 和 collision checking；
- Visibility Graph、Uniform Grid、PRM、EST、RRT 的核心流程与区别；
- state valid 不代表 edge valid；
- probabilistic completeness 和 one-sided failure；
- narrow passage 为什么困难；
- Part A 两道证明；
- Part B 为什么是一个 `3K` 维 joint planning problem。

**P1：应当能解释**

- Uniform、obstacle-based、Bridge Test；
- connection radius 的权衡与 lazy checking；
- fixed discretisation 与 distance-based edge checking；
- visibility、epsilon-good、beta-lookout、alpha-expansiveness；
- line-segment intersection、bounding-volume hierarchy；
- PRM、RRT、RRT-Connect、EST 的适用场景。

**P2：被深入追问时使用**

- `S^1`、`SE(2)`、`SE(3)`；
- biased sampling 如何保留 probabilistic completeness；
- Part A 中严格半径与 supremum 的区别；
- 理论模型与 supplied simulator 的边界/contact 差异。

---

# 1. Week 1：问题建模与 C-space

## 1.1 什么是 robot

课程采用的简化定义是：robot 是具有一定自主性、能够在环境中移动或操纵物体的可编程机器。典型系统包含：

- sensors：获取环境和自身信息；
- planning/decision making：决定做什么；
- actuators：执行动作。

典型流程是：

```text
Sense -> Think / Plan -> Act
```

课程还把 simulated agents 和 simulated physical environments 纳入讨论，所以游戏 agent 等也可以作为顺序决策对象。

## 1.2 Deterministic motion planning

它解决的问题是：已知初始状态和目标状态，让 robot 自动计算一条运动。

Week 1 的 deterministic 假设是：

1. action effect 完全已知且确定；
2. robot 与 environment model 完全已知；
3. initial state 精确已知。

本课程 A1 的 motion-planning 部分就是在这些假设下进行的。这里求的是 geometric/kinematic path，不是带速度、加速度和控制误差的完整 trajectory optimisation。

**口试短答**

> Deterministic motion planning computes a collision-free motion from a known initial state to a goal state, assuming the robot, environment, initial state, and action effects are known exactly.

## 1.3 Workspace、configuration、C-space、DOF

先区分四个概念：

- **Workspace**：robot 实际运动的物理空间，例如二维房间或三维空域。
- **Configuration**：能够完全确定 robot 上每一点位置的一组最小参数。
- **Configuration space (`C-space`)**：所有可能 configurations 的集合。
- **Degrees of freedom (`DOF`)**：描述 configuration 所需的独立参数数量，也就是 C-space 的维数。

例如二维 point robot：

```text
configuration q = (x,y)
C-space C = R^2
DOF = 2
```

一个 configuration 是 C-space 中的一个点。robot 在 workspace 中的连续运动，对应 C-space 中的一条连续曲线。

## 1.4 为什么 articulated robot 也能变成一个点

对于有 `K` 个转动关节的固定机械臂，robot 在 workspace 中有很多 links，但所有几何位置都由关节角决定：

\[
q=(\theta_1,\ldots,\theta_K).
\]

因此整个机械臂在 C-space 中仍然只是一个点。

若每个角度是完整周期变量，单关节空间是圆 `S^1`，所以：

\[
\mathcal C=(S^1)^K.
\]

计算中经常使用实数向量表示：

\[
q\in\mathbb R^K,
\]

但必须正确处理角度 wrap-around，例如 `0` 和 `2pi` 表示同一角度。

若关节被严格限制在非周期区间 `[theta_min,theta_max]`，可行 configuration set 更准确地说是这些区间的乘积。Tutorial solution 采用 `(S^1)^7` 的课程表示，再说明计算中通常映射到 `R^7`；现场应先按课件回答，再指出 joint limits 是额外约束。

## 1.5 常见 robot 的 C-space

| Robot | Configuration | C-space / practical dimension |
|---|---|---|
| 2D point robot | `(x,y)` | `R^2`, 2 DOF |
| Planar rigid robot | `(x,y,theta)` | `SE(2)=R^2 x S^1`, 3 DOF |
| 7-joint arm | seven joint angles | `(S^1)^7`, often represented in `R^7` |
| 3 planar mobile robots, centralised | three `(x,y,theta)` states | `SE(2)^3`, 9 DOF |
| Mobile manipulator | arm angles + base pose | `(S^1)^7 x SE(2)`, 10 DOF |
| General 3D rigid drone | position + orientation | `SE(3)=R^3 x SO(3)`, 6 DOF |
| A1 spherical translating drone | centre `(x,y,z)` only | `R^3`, 3 DOF |
| `K` A1 drones, centralised | `(p_1,...,p_K)` | subset of `R^(3K)` |

关键区别：一般 drone 可以有 orientation，所以是 `SE(3)`；A1 用 bounding sphere 且只考虑 translation，orientation 不影响几何，因此每架只需要三个坐标。

## 1.6 `C_obs` 与 `C_free`

不要先背公式。概念上：

- `C_obs` 是所有会让 robot 碰撞或违反约束的 configurations；
- `C_free` 是所有合法 configurations。

公式是：

\[
\mathcal C_{\mathrm{obs}}
=\{q\in\mathcal C\mid R(q)\cap Obs\neq\varnothing\},
\]

\[
\mathcal C_{\mathrm{free}}
=\mathcal C\setminus\mathcal C_{\mathrm{obs}}.
\]

这里 `R(q)` 表示 robot 放在 configuration `q` 时占据的 workspace 区域。

因此 motion-planning problem 被统一改写成：

```text
在 C_free 中找一条从 q_I 到 q_G 的连续路径。
```

约束也可以并入 `C_obs`。例如 joint limits、workspace boundary、危险区域、无人机之间的碰撞，都可以通过 configuration validity 排除。

## 1.7 Workspace obstacle 与 C-space forbidden region 不一样

对 point robot，workspace obstacle 通常可以直接对应 `C_obs`。

对有体积的 robot，一个 configuration 是否碰撞取决于 robot 的完整形状。因此要把 workspace obstacle 转换成 robot reference point 不能进入的 forbidden region。

对于半径 `k` 的球形 robot：

```text
扩大 obstacle k
<=>
把球缩成 point robot
```

可写成 obstacle 与半径 `k` 球的 Minkowski sum。即使原 obstacle 是 polygon/polyhedron，扩大后的边和角通常会出现曲面或圆弧。

**口试短答**

> Workspace obstacles are physical objects. A C-space obstacle is the set of robot configurations that cause collision. For a non-point robot, these sets are generally different and the C-space obstacle may have a more complicated geometry.

---

# 2. Week 1：把连续问题离散化

## 2.1 为什么需要离散化

C-space 是连续空间，因此候选下一步有无限多个。经典方法通过图或树保留少量代表性状态，再进行离散搜索。

课程前三种方法是：

```text
Visibility Graph
Uniform Grid
Sampling-based planning
```

## 2.2 Visibility Graph 的概念

标准 Visibility Graph 针对 polygonal obstacles。最短路径如果绕过 polygon，转弯会出现在 obstacle vertices，因此这些 vertices 是重要候选点。

构图方式：

1. 把 obstacle vertices、initial 和 goal 作为 graph nodes；
2. 若两个 nodes 之间的直线 collision-free，则添加 edge；
3. edge weight 可以设为欧氏长度；
4. 使用 graph search；若求最短路，可使用 Dijkstra/A*。

```text
I ---- visible vertex ---- obstacle vertex ---- G
```

课件给出的 naive complexity，令 `n` 为 obstacle vertices 总数：

- construction time：`O(n^3)`；
- space：`O(n^2)`。

## 2.3 Visibility Graph 的限制

1. 标准算法要求 polygonal obstacles/forbidden regions；
2. non-point robot 必须在 C-space 构图；
3. forbidden region 即使由 polygonal workspace obstacles 产生，也可能不是 polygonal；
4. 它依赖 forbidden region 的显式表示；
5. 高 DOF 下显式构造 `C_obs` 很昂贵。

所以 A1 Q1(a) 的准确结论不是“任何 visibility 方法永远不可能”，而是：

\[
\boxed{\text{the standard course visibility graph cannot be applied directly}.}
\]

## 2.4 Uniform Grid 的概念

Uniform Grid 不寻找 obstacle vertices，而是把 C-space 切成等大的 cells：

```text
+---+---+---+---+
|free|free|obs|free|
+---+---+---+---+
|free|free|obs|free|
+---+---+---+---+
```

构图方式：

1. 每个完全 free 的 cell 成为 node；
2. neighboring free cells 之间连 edge；
3. 找到包含 initial/goal 的 cells；
4. graph search 得到 cell sequence；
5. 用 cell centres 或 cell 内的 local paths 构造连续路径。

它只需要 free/occupied test，不要求 polygonal `C_obs`，也不一定要显式构造 `C_obs` boundary。

## 2.5 Uniform Grid 的限制

grid width `h` 是 resolution：

- `h` 太大：可能漏掉真实存在的 narrow passage；
- `h` 太小：cell 数量急剧增加。

若每个维度长度固定，`d` 维空间的 cell 数大致随：

\[
O(h^{-d})
\]

增长。这就是高维 grid 的 curse of dimensionality。

## 2.6 Visibility Graph 与 Uniform Grid 比较

| 问题 | Visibility Graph | Uniform Grid |
|---|---|---|
| 表示 | obstacle vertices 和 visibility edges | free cells 和邻接 edges |
| obstacle shape | 标准方法要求 polygonal forbidden regions | 不要求 polygonal boundary |
| 显式 `C_obs` | 通常需要 | 可通过 cell collision tests 隐式判断 |
| 路径性质 | polygonal point-robot setting 中可求 shortest path | 路径质量依赖 resolution |
| 主要问题 | 高维 forbidden-region construction | cell 数随维数指数增长 |

---

# 3. Week 2：Sampling-based Motion Planning 与 PRM

## 3.1 核心思想

Sampling-based planner 不显式计算整个 `C_free` 或 `C_obs`。它通过 validity tests 采集少量 configurations，用 graph/tree 近似保留 free-space connectivity。

```text
连续 C-space
-> sample configurations
-> validate states and local motions
-> graph/tree
-> discrete search
```

它需要四个核心组件：

1. sampling strategy：在哪里增加 vertices；
2. connection strategy：尝试增加哪些 edges；
3. state validity：configuration 是否 collision-free；
4. local planner：两个 configurations 之间的 motion 是否 collision-free。

## 3.2 Graph-based 与 Tree-based

### Graph-based: PRM

- 预先构建 roadmap；
- 可以为同一 robot/environment 回答多个 start-goal queries；
- roadmap 通常包含 cycles 和多个 connected components。

### Tree-based: EST/RRT

- 直接从 initial/goal 生长一棵或多棵 tree；
- 通常适合 single query；
- 找到当前 query 后即可停止。

**口试短答**

> PRM amortises roadmap construction over multiple queries in the same C-space. Tree-based planners grow directly for one query and are generally preferable when only one start-goal problem must be solved.

## 3.3 原始 PRM 的 roadmap construction

重复执行：

1. 从 C-space 均匀采样 `q`；
2. 若 `q` collision-free，把它加入 roadmap；
3. 找到 roadmap 中距离 `q` 不超过 `D` 的 vertices；
4. 对每个候选 neighbor 运行 local planner；
5. 只有 local motion collision-free 才添加 edge。

然后回答 query：

1. 把 initial 与 goal 连接到可见/可达 roadmap vertices；
2. 在 graph 中 search；
3. 把 graph path 转换为连续 local motions。

停止条件可以是：

- 已加入 `n` 个 vertices；
- 达到 time limit；
- interleave construction and search，直到 queries 都得到答案。

## 3.4 Narrow Passage Problem

均匀采样按 volume 分配概率。若 narrow passage 在 `C_free` 中体积很小：

```text
large room -- tiny passage -- large room
```

绝大多数 samples 会落入两个宽阔区域。roadmap 可能很好地覆盖两个 room，却没有 sample/edge 穿过 passage，因此 connectivity 没被捕获。

关键不是 obstacle 数量本身，而是 free-space geometry、visibility 和 passage measure。

## 3.5 Uniform sampling

最基本方式：从整个 C-space 均匀采样，再拒绝 colliding configurations。

优点：

- 简单；
- 不偏向某个特殊区域；
- 在常见条件下便于保留 probabilistic completeness。

缺点：

- 在狭窄、关键但 measure 很小的区域效率低。

## 3.6 Obstacle-based sampling

目的：增加 obstacle boundary 附近的 free samples。

1. 均匀采 `q1`；
2. 在 `q1` 的距离 `D` 内采 `q2`；
3. 若一个 colliding、另一个 free，保留 free 的那个。

```text
obstacle side: q1 x |boundary| o q2 : free side
```

这会把 samples 偏向 `C_obs/C_free` boundary，但 boundary sample 不一定处于连接 start 与 goal 所需的 passage。

## 3.7 Bridge Test

目的：增加 narrow passage 内的 samples。

1. 采样 `q1`；
2. 在附近采样 `q2`；
3. 要求 `q1,q2` 都 collision；
4. 计算中点 `qm=(q1+q2)/2`；
5. 若 `qm` free，保留 `qm`。

\[
q_1,q_2\in C_{\mathrm{obs}},
\qquad
q_m=\frac{q_1+q_2}{2}\in C_{\mathrm{free}}.
\]

直观图：

```text
obstacle        narrow free passage        obstacle
   q1 x ------------- qm o ------------- x q2
```

`q1`、`q2` 只是 probes，不会加入路径。`qm` 也只是一项更有希望的 sample，不保证当前 graph/tree 能通过直线连接到它。

课件版本在 `q1` 的距离 `D` 邻域内均匀采 `q2`。你的 `_bridge_attempt` 使用 Gaussian offset，标准差由 `bridge_sigma` 控制；二者共享同一个 invalid--invalid--valid-midpoint 原理。

## 3.8 Workspace-informed sampling

有些 C-space narrow passages 来自明显的 workspace bottleneck。课程中的方法是：

1. 暂时把 robot 上选定点 `r` 当作 point robot；
2. 在 2D/3D workspace grid 中规划一条粗略 path `pi`；
3. 从 `pi` 上的 cells 采 workspace point `p`；
4. 再采一个把 robot point `r` 放到 `p` 的 configuration。

它利用便宜的低维 workspace 信息引导高维 C-space sampling，但粗略 workspace path 本身不保证完整 robot collision-free。

## 3.9 组合 sampling strategies

最简单方式是给不同 strategies 固定 weights，然后按权重随机选择。

也可以将每个 strategy 看成 multi-armed bandit 的一只 arm：

- exploitation：更多使用目前有效的 strategy；
- exploration：继续尝试尚未证明有效的 strategies。

课程提到：

- epsilon-greedy：大概率选择当前最佳 strategy，小概率探索其他 strategies；
- EXP3：一种面向 adversarial bandit 的方法，目标是与最佳单一 strategy 保持竞争力。

A1 代码中的 `bridge_prob` 是固定 mixture，不是完整 bandit adaptation。

## 3.10 Connection strategy

不能盲目尝试所有 vertex pairs：

- edge 太长：collision probability 高，检查贵；
- connection radius 太小：需要更多 samples 才能连通；
- connection radius 太大：大量无效 local-planner calls；
- 还可以限制每个 vertex 的最大 neighbors/degree。

所以连接尺度通常需要理论指导与实验调节。

### Lazy collision checking

先把附近 vertices 的 candidate edges 暂时加入 graph，不立即 collision-check。graph search 找到候选 path 时，只检查这条 path 上真正需要的 edges。

优点：避免检查永远不会使用的 edges。  
代价：候选 path 可能因 invalid edge 被反复否定，需要重新搜索。

---

# 4. Week 2：Configuration Collision Checking

## 4.1 检查发生在哪里

configuration 位于 C-space，但 collision 通常在 workspace 中计算：

```text
q
-> forward geometry / place robot in workspace
-> robot-obstacle and self-collision tests
-> valid or invalid
```

这就是 Tutorial Week 2 Q3 的重点。不能只看 robot 的 endpoints；还要检查 link interior，并在必要时判断 robot 是否完整位于 obstacle 内部。

## 4.2 Axis-aligned rectangles

两个 axis-aligned rectangles 在二维中不相交，当且仅当至少有一个 separating axis condition 成立：

- A 完全在 B 左边；
- A 完全在 B 右边；
- A 完全在 B 上边；
- A 完全在 B 下边。

如果四种分离情况都不成立，则 rectangles overlap/collide。

## 4.3 三点 orientation

对二维点 `a,b,c`，orientation 可由 signed area 判断：

\[
\operatorname{Area}(a,b,c)
=
\det
\begin{bmatrix}
a_x&a_y&1\\
b_x&b_y&1\\
c_x&c_y&1
\end{bmatrix}.
\]

- `Area > 0`：counterclockwise；
- `Area < 0`：clockwise；
- `Area = 0`：collinear。

## 4.4 Line-segment intersection

对 segments `p1q1` 与 `p2q2`，一般位置下相交需要：

1. `p1q1p2` 与 `p1q1q2` orientation 不同；
2. `p2q2p1` 与 `p2q2q1` orientation 不同。

直观上，每条 segment 的两个 endpoints 都必须位于另一条无限直线的两侧。

collinear special case 还要检查两个 segments 在 `x`、`y` ranges 上是否 overlap。

可先检查 segments 的 axis-aligned bounding boxes。若 boxes 不相交，segments 必然不相交；只有 boxes overlap 才运行精确 orientation test。这是 broad phase + narrow phase 的基本思想。

## 4.5 Hierarchical Bounding Volumes

复杂 polygon/polyhedron 可能由大量 line segments 或 triangles 构成。逐 primitive pair 检查很贵，因此为每个 object 建 bounding-volume tree：

```text
large root sphere/box
├── smaller bounding volume
│   ├── primitive
│   └── primitive
└── smaller bounding volume
    ├── primitive
    └── primitive
```

collision traversal：

1. 先检查两个 root bounding volumes；
2. 若不相交，整对 objects 立即判定不相交；
3. 若相交，展开较大的 bounding volume；
4. 继续向下；
5. 到 leaves 后才做 primitive-level collision check。

效率来自 pruning：多数 subtree 在高层就能排除。

## 4.6 Bounding-volume distance computation

同一 hierarchy 也可计算 point/object distance。维护当前最好上界 `d_c`：

1. 若 point 到某 bounding volume 的 lower-bound distance 已经不小于 `d_c`，跳过整个 subtree；
2. 否则继续展开；
3. leaf 上计算 point-to-primitive exact distance并更新 `d_c`。

这与 A1 Part A Q2(b) 的 workspace clearance `d_W(q,Obs)` 有直接联系。

---

# 5. Week 3：Local Planner 与 Edge Validity

## 5.1 为什么 endpoints free 不够

```text
q0 free -------- obstacle -------- q1 free
```

两个 endpoints 都在 `C_free`，中间 segment 仍可能穿过 `C_obs`。因此必须分开：

- `IS-VALID(q)`：检查一个 configuration；
- `MOTION-VALID(q0,q1)`：检查整个 local motion。

## 5.2 固定离散检查

最简单方法把 segment 分成许多小段，在 representative points 上调用 state validity。

问题：

- step 太大可能漏掉细小 collision；
- step 太小会调用大量昂贵 collision checks；
- 有限 samples 本身不是 exact continuous-collision proof。

你的 supplied `motion_valid` 使用这种方法，step size 是 drone radius 的一半。它提供的是相对于 supplied primitive 的有效性，不是解析意义上的绝对连续碰撞保证。

## 5.3 Distance-based adaptive checking

概念：若 configuration `q` 到最近 forbidden region 的距离是 `d_q`，以 `q` 为中心、`d_q` 为半径的 open ball 全部 free。

对 segment `qq'`：

1. 若 endpoints invalid，segment invalid；
2. 分别构造 endpoints 的 guaranteed-free balls；
3. 令 midpoint `qm=(q+q')/2`；
4. 若 `qm` 同时位于两个 free balls 中，两个半段分别被 free balls 覆盖，segment safe；
5. 否则递归检查 `q--qm` 与 `qm--q'`。

它根据 clearance 自适应细分：宽阔区检查少，靠近障碍处检查细。

A1 Part A Q2 的意义正是构造一个 conservative C-space free radius：用 workspace clearance 和 robot-motion bound 推出某个 neighborhood 中所有 straight motions 都安全。

---

# 6. Week 3：EST、RRT 与 RRT-Connect

## 6.1 EST 的核心思想

EST 希望优先扩展 tree 中较稀疏、较少探索的区域。

课程版本：

1. initial configuration 作为 root；
2. 以与局部密度 `N(q)` 成反比的概率选择 tree node `q`；
3. 在 `q` 周围距离 `D` 内采样 `q'`；
4. 若 `q'` 和 segment `qq'` 都 valid，加入 child；
5. 常使用 bidirectional search。

\[
P(\text{select }q)\propto\frac{1}{N(q)}.
\]

低密度 node 更可能被选，避免 tree 一直挤在已经充分探索的区域。

## 6.2 RRT 的核心思想

RRT 不直接选要扩展的 tree node。它先全局均匀采样 `q_rand`，再让离该 sample 最近的 node 扩展：

1. `q_rand <- SAMPLE(C)`；
2. `q_near <- NEAREST(T,q_rand)`；
3. 沿 `q_near -> q_rand` 产生一步距离不超过扩展尺度的 `q_new`；课程写作 `D`，你的实现写作 `eta`；
4. 若 local motion valid，则添加 `q_new`，parent 为 `q_near`。

边界 nodes 往往拥有更大的 Voronoi regions，更容易成为随机 sample 的 nearest node，所以 tree 会快速向未探索区域伸展。这叫 Voronoi bias。

课程伪代码从概念上要求选取一个能让 local segment 保持 free 的 `q_new`。你的实现更具体：先用 `Steer` 提议一个最多 `eta` 的 endpoint，再用 supplied `motion_valid` 检查整段；若失败，本次不会自动截短到碰撞前的最后一个 free point。

## 6.3 RRT 与 EST 的区别

| | EST | RRT |
|---|---|---|
| 先选什么 | 按 inverse local density 选 tree node | 全局采 `q_rand` |
| sample 范围 | 选中 node 附近 | 整个 C-space |
| exploration bias | 显式低密度 bias | nearest/Voronoi bias |
| 共同点 | tree-based、local validation、常可双向 | tree-based、local validation、常可双向 |

你的 `local_bridge` 只是从 tree 中均匀选 anchor，再做 Gaussian perturbation；它没有 inverse-density node selection，所以不是完整 EST。

## 6.4 RRT-Connect 比 RRT 多了什么

普通 `EXTEND` 朝 target 走一步。`CONNECT` 固定一个 target，重复调用 `EXTEND`，直到：

- `REACHED`：到达 target；
- `TRAPPED`：下一步碰撞；
- `TIMEOUT`：预算结束。

双树版本每轮：

```text
T_a 朝随机 sample 扩展一步得到 q_new
T_b 贪心 CONNECT 到 q_new
若未相遇，交换角色
```

交换只改变谁主动随机探索、谁贪心追赶，不改变两棵 tree 的 roots。

## 6.5 一次 iteration 保存什么

- active tree 的 valid `q_new` 立即成为 node；
- opposite tree 在 `CONNECT` 中每个 valid intermediate node 也立即保存；
- 发生 collision 的 candidate 不保存；
- 即使本轮没有完整 path，已经验证的 nodes/edges 不会丢弃。

很多 iteration 中不同方向的局部直线，累计成为分叉 tree；单次 `CONNECT` 自身只追赶一个固定 target。

---

# 7. Week 3：Probabilistic Completeness

## 7.1 先讲含义

Probabilistic completeness 不等于“每次都能找到”，也不等于“在 20 秒内保证找到”。

它表示：若 feasible path 存在，随着 samples/运行时间不断增加，planner 失败的概率趋近于零。

正式写法：

\[
\lim_{n\to\infty}
P(\operatorname{fail}(n,q_I,q_G))=0,
\]

前提是从 `q_I` 到 `q_G` 存在 valid path，并满足相应 planner 的理论假设。

课程指出 original PRM、EST 和 RRT 的 failure probability 可随 samples 数以 exponential rate 下降；具体 bound 依赖 C-space geometry。

## 7.2 One-sided planner

Sampling-based planner 是 one-sided：

```text
找到 valid path -> 可以证明成功
没有找到 path   -> 不知道无解，还是尚未找到
```

证明不存在路径叫 disconnection proof，通常比找到一条 witness path 困难，仍是较开放的问题。

因此 A1 输出 `FAILURE`/timeout 只能表示本次预算内没有找到，不能直接说 environment infeasible。

## 7.3 Biased sampling 与 completeness

偏向 narrow passage 的策略不自动保留 probabilistic completeness。如果某些有正 measure 的必要 free regions 永远没有采样概率，算法可能永远错过它们。

常见保护方式是保留非零概率的 global uniform sampling：

```text
p: biased strategy
1-p: global uniform strategy
```

你的 baseline `bridge_prob=0` 是 uniform sampling。若 `bridge_prob=0.3`，仍保留约 70% uniform attempts；但不能仅凭这一点替整个具体实现宣称已经完成形式 completeness proof。

---

# 8. Week 3：为什么 Sampling-based Planning 有效

## 8.1 Visibility region

一个 free configuration `q` 能“看到” `q'`，表示 local planner 可以直接、collision-free 地连接二者。

\[
V(q)=\{q'\in F\mid qq'\text{ is accepted by the local planner}\},
\qquad F=C_{\mathrm{free}}.
\]

如果多数 points 都能看到 free space 中较大区域，少量 milestones 就可能覆盖空间并捕获 connectivity。

注意这里的 visibility 由 local planner 定义，不是只看 workspace 中有没有视觉遮挡。

## 8.2 Epsilon-goodness

`F` 是 epsilon-good，若每个 `q in F` 都至少能看到 `F` 的 epsilon 比例：

\[
\mu(V(q))\geq\epsilon\mu(F),
\qquad\forall q\in F.
\]

- epsilon 大：普通 point 的 local visibility 好，容易用少量 milestones cover `F`；
- epsilon 小：存在只能看见极小区域的 points。

## 8.3 Beta-lookout

假设 `S subset F` 是当前已经探索到的 connected region。一个 `q in S` 是 beta-lookout，如果它能看到 `S` 外部至少 beta 比例的 free space：

\[
\operatorname{LOOKOUT}_{\beta}(S)
=\left\{
q\in S:
\mu(V(q)\setminus S)
\geq\beta\mu(F\setminus S)
\right\}.
\]

beta 回答：

```text
找到一个 lookout 后，它能看到多少新的外部空间？
```

## 8.4 Alpha-expansiveness

对每个 connected subset `S subseteq F`，要求其中有足够多的 lookout points：

\[
\mu(\operatorname{LOOKOUT}_{\beta}(S))
\geq\alpha\mu(S).
\]

alpha 回答：

```text
当前区域中，有多大比例的点是有用的 lookouts？
```

`F` 为 `(epsilon,alpha,beta)`-expansive，表示：

1. `F` 是 epsilon-good；
2. 上述 alpha/beta lookout 条件对每个 connected `S` 成立。

一句话记忆：

```text
epsilon：普通点能看多少
beta：一个 lookout 能看多少新区域
alpha：这样的 lookout 有多少
```

## 8.5 Narrow passage 如何影响参数

假设已经探索左侧 room，只有 passage entrance 附近的 points 能看到外面：

- entrance 区域很小 -> lookout 很少 -> alpha 小；
- 从 entrance 只能看到 passage 中一点点新空间 -> beta 小；
- passage 内 local visibility 也可能较小 -> epsilon 可能小。

alpha/beta 越小，为高概率捕获 connectivity 所需 samples 越多。

## 8.6 为什么不直接计算 epsilon/alpha/beta

精确计算这些参数需要知道 `C_free` 的 geometry；sampling-based planning 的优势正是避免显式构造高维 `C_free`。

它们的价值主要是：

- 理论上解释 sample complexity；
- 从 workspace geometry 定性预测 planner 难度；
- 指导 experimental environments 的设计。

## 8.7 Tutorial recap 的两个细节

`S` 不是人为挑一个看起来方便的 subset。Expansiveness condition 必须对**每个 connected `S subseteq F`**成立，因此有效的 `alpha,beta` 最终受最坏 connected subset 限制。

若 `S=F`，则 $F\setminus S$ 为空，已经没有外部空间需要发现。若进一步 `C_obs` 为空，则所有 configurations 彼此直线可见，tight values 可以取：

\[
\epsilon=\alpha=\beta=1.
\]

---

# 9. Tutorial Week 2 必会内容

## 9.1 C-space 建模问题

回答任何 C-space 问题用三步：

1. 什么最小参数能完全确定 robot？
2. 每个参数的 topology/domain 是什么？
3. 多个 bodies centralised 时做 Cartesian product。

特别注意：non-holonomic movement restriction 通常不改变 configuration 的定义，而是成为 allowable motions/local planner 的约束。例如 wheelchair base 的状态仍包含 `(x,y,theta)`。

## 9.2 Segment intersection 题

Tutorial 给出的结论：

```text
AB intersects EF
AB does not intersect CD
CD does not intersect EF
```

口试重点不是背答案，而是能用两组 orientation conditions 推导，并知道 collinear special case。

## 9.3 Mobile-link collision 题的真正目的

configuration 是 `(x,y,theta)`，但 validity 要把 link 放回 workspace：

1. 由 base position 和 angle 算 link endpoints；
2. 检查 link segment 与 obstacles；
3. 还要处理 link/object containment，而不只是 surface intersection。

核心结论：

\[
\boxed{\text{configuration is represented in C-space, but geometric collision is usually evaluated in workspace}.}
\]

---

# 10. Tutorial Week 3 必会内容

## 10.1 两个同尺寸 narrow passages 为什么难度不同

passage 的 width 和 length 相同，不代表 sampling difficulty 相同。还要看它们如何连接 large free regions，以及 local visibility。

```text
aligned passage：一个 point 可能直线看到较远区域
offset/dogleg passage：必须转弯，每次只能看到局部
```

offset arrangement 通常更难，因为：

- useful lookout region 更小，alpha 更小；
- lookout 能看见的新区域更少，beta 更小；
- 需要更多 milestones 才能形成转弯并捕获 connectivity。

这也是 E7 offset passage 的理论动机。不能只用 obstacle count 衡量 complexity。

## 10.2 MultiDrone supporting code 题

Tutorial 要求理解四件事：

1. 创建带 rectangular obstacles 的 workspace；
2. 设置多架 drones；
3. 调用 state validity 检查 joint configuration；
4. 调用 local-motion validity 检查两个 joint configurations 之间的直线。

它与 A1 的直接关系是：A1 不要求你重新实现底层 collision geometry，而是要求你利用这些 primitives 设计和实现 sampling-based planner。

---

# 11. A1 Part A Q1：Visibility Graph 与 Uniform Grid

## 11.1 Q1(a) 推理链

先说概念：标准 Visibility Graph 依赖 polygonal forbidden-region vertices。球形 robot 有体积，必须在 C-space 中规划；把 polyhedral obstacle 按球半径扩张后，边角变圆，因此 planar forbidden region 一般含 curved arcs。

证明链：

\[
k>0
\Longrightarrow
\text{non-point robot}
\Longrightarrow
\text{construct the problem in C-space}.
\]

球心只在平面平移：

\[
q=(x,y),\qquad\mathcal C\subseteq\mathbb R^2.
\]

若 `O` 是 workspace polyhedral obstacle、`B_k^3` 是半径 `k` 的球、`Pi` 是球心运动平面：

\[
\mathcal C_{\mathrm{obs}}
=\left(O\oplus B_k^3\right)\cap\Pi.
\]

于是：

\[
O\text{ polyhedral},\ k>0
\Longrightarrow
\mathcal C_{\mathrm{obs}}\text{ generally contains curved arcs}
\Longrightarrow
\mathcal C_{\mathrm{obs}}\text{ generally non-polygonal}.
\]

最终：

\[
\boxed{\text{the standard visibility graph cannot be applied directly}.}
\]

## 11.2 Q1(b) 推理链

Uniform Grid 只需判断整个 cell 是否 free：

\[
\operatorname{FREE\text{-}CELL}(c)
\Longleftrightarrow
c\cap\mathcal C_{\mathrm{obs}}=\varnothing.
\]

因此 `C_obs` boundary 是否 polygonal 不影响建图。free cells 是 nodes，neighboring free cells 相连，再对 initial/goal cells 做 graph search。

必须补充的限制：

\[
\boxed{\text{coarse grid may miss an existing narrow passage; the method is resolution-dependent}.}
\]

---

# 12. A1 Part A Q2(a)：Workspace Motion Bound

## 12.1 先理解要证明什么

一个 joint angle 发生变化，会带动它下游的整条 subchain。离 joint 越远的 robot point，转过相同角度时在 workspace 中走得越远。

题目要找一个统一常数 `alpha`，保证 robot 上任意 point 的 workspace path length 不超过：

\[
\alpha d_C(q,q').
\]

## 12.2 配置距离控制每个 joint angle

令：

\[
\Delta\theta_i=\theta_i'-\theta_i,
\qquad
\delta=d_C(q,q')=\max_i|\Delta\theta_i|.
\]

于是每个 joint 都满足：

\[
|\Delta\theta_i|\leq\delta.
\]

## 12.3 一个 point 受到哪些 joints 影响

取 link `m` 上任意 material point `P`。只有 upstream joints `1,...,m` 会影响它。

`P` 到 joint `i` 的距离最多是 downstream link lengths 总和：

\[
r_i(P,s)\leq(m-i+1)L.
\]

joint `i` 旋转小角度 `|dtheta_i|` 时，对 `P` 的弧长贡献最多为：

\[
d\ell_i=r_i(P,s)|d\theta_i|.
\]

所有 joints 同时运动，实际速度向量的长度不超过各 contribution lengths 之和，这是 triangle inequality。

## 12.4 求和

沿 straight C-space path 积分：

\[
\begin{aligned}
\ell(P)
&\leq\int_0^1\sum_{i=1}^{m}
r_i(P,s)|\Delta\theta_i|\,ds\\
&\leq L\delta\sum_{i=1}^{m}(m-i+1)\\
&=\frac{Lm(m+1)}2\delta\\
&\leq\frac{LK(K+1)}2\delta.
\end{aligned}
\]

代回 `delta=d_C(q,q')`：

\[
\boxed{
\ell(P)\leq
\frac{LK(K+1)}2d_C(q,q')
},
\qquad
\boxed{\alpha=\frac{LK(K+1)}2}.
\]

## 12.5 为什么这是 upper bound 而不一定 tight

推导中连续使用了保守上界：

- 把 `P` 到 joint 的实际距离替换为最大 downstream length；
- 把每个 `|Delta theta_i|` 都替换为最大值 `delta`；
- 把多个 velocity vectors 的合成长度替换为长度之和；
- 最后把 `m` 替换为 `K`。

所以它对所有 points 都安全，但通常不是最小可能常数。

---

# 13. A1 Part A Q2(b)：Guaranteed Safe Neighborhood

## 13.1 先理解核心

假设当前 robot 离 obstacles 最近还有 clearance：

\[
\rho=d_W(q,Obs).
\]

Part 2(a) 告诉我们：configuration 最多改变 `tau` 时，robot 上任意 point 最多移动 `alpha tau`。只要这个最大移动严格小于 clearance，robot 就不可能碰到 obstacle。

```text
initial clearance rho
- maximum possible approach alpha*tau
> 0
```

## 13.2 Straight path 中任意时刻都在 neighborhood 内

对任意 `q'` 满足 `d_C(q,q')<=tau`，令：

\[
q(s)=q+s(q'-q),\qquad s\in[0,1].
\]

由 max metric：

\[
\begin{aligned}
d_C(q,q(s))
&=\max_i|\theta_i+s(\theta_i'-\theta_i)-\theta_i|\\
&=s\max_i|\theta_i'-\theta_i|\\
&=s\,d_C(q,q')\\
&\leq s\tau\leq\tau.
\end{aligned}
\]

## 13.3 Clearance 下界

由 Part 2(a)，每个 robot point 到 `q(s)` 时位移不超过 `alpha tau`。triangle inequality 给出：

\[
d_W(q(s),Obs)
\geq d_W(q,Obs)-\alpha\tau.
\]

要让整个 path 始终保持正 clearance：

\[
d_W(q,Obs)-\alpha\tau>0
\Longrightarrow
\alpha\tau<d_W(q,Obs)
\Longrightarrow
\boxed{\tau<\frac{d_W(q,Obs)}{\alpha}}.
\]

代入 `alpha`：

\[
\boxed{
\tau<\frac{2d_W(q,Obs)}{LK(K+1)}
}.
\]

## 13.4 为什么是严格小于与 supremum

题目定义的是 closed neighborhood `d_C<=tau`，而 obstacle contact (`d_W=0`) 算 collision。若直接取 equality，上界只保证最终 clearance `>=0`，不能保证 `>0`。

因此任何严格小于比值的 `tau` 都有保证；保证半径的 supremum 是：

\[
\boxed{
\tau_{\mathrm{sup}}
=\frac{d_W(q,Obs)}{\alpha}
}.
\]

supremum 是最小上界，不要求它本身属于允许集合。

---

# 14. A1 Part B 的理论模型

## 14.1 为什么多个起点仍然只有一个 root

workspace 中有 `K` 个 drone centres：

\[
p_i=(x_i,y_i,z_i).
\]

centralised planner 把整个团队的一张 snapshot 当成一个 configuration：

\[
q=(p_1,\ldots,p_K)\in\mathbb R^{3K}.
\]

因此物理上有 `K` 个起始位置，但 C-space 中只有一个 joint initial configuration `q_I`，也只有一个 start-tree root。

## 14.2 Joint free space

一个 joint configuration free，需要同时满足：

1. 每架 drone 在允许的 workspace bounds 内；
2. 每个 bounding sphere 不与 obstacle 相交；
3. 任意两架 drones 不相撞。

理论报告使用：

\[
\mathcal C_{\mathrm{free}}
=\left\{q:\
B_r(p_i)\cap Obs=\varnothing,\ 
\|p_i-p_j\|_2>2r
\right\}.
\]

这是应当用于解释 B1 的理论模型。真实 supplied simulator 有两个 boundary conventions：它只检查 drone centre 位于 raw bounds，并允许两球距离恰好等于 `2r`。现场若被追问代码语义，应按 `Codewalk MASTER` 客观区分理论定义与 supplied implementation。

goal 不是一个固定 point，而通常是每架 drone goal ball 的 Cartesian product。代码先采一个 valid joint goal configuration，作为 goal-tree root。

## 14.3 Joint straight-line motion

`q_near -> q_target` 中，每架 drone 有自己的 displacement direction 和 length。所有 drones 共用相同 interpolation fraction `lambda`：

\[
p_i^{new}=p_i^{near}
+\lambda(p_i^{target}-p_i^{near}).
\]

使用 max individual-drone metric：

\[
d(q,q')=\max_i\|p_i-p_i'\|_2.
\]

令：

\[
\lambda=\min\left(1,\frac{\eta}{d(q,q')}\right),
\]

则移动最远的 drone 最多走 `eta`，其他 drones 按同一比例走更短距离；方向不必相同，也可以保持不动。

## 14.4 为什么选择 RRT-Connect

1. 这是 continuous `3K`-dimensional C-space 中的 single query；
2. simulator 提供 state 和 local-motion validity oracle；
3. bidirectional growth 可分别从 initial 与 goal 探索；
4. greedy `CONNECT` 在 open regions 中能快速缩短两树距离；
5. PRM 的 reusable roadmap 对单次 demo query 没有明显收益；
6. RRT* 的 path-cost optimisation 不是 B3 的评分目标，会消耗 20 秒预算。

## 14.5 为什么 K 增大时变难

`K` 增大同时影响：

- dimension：`3K`；
- pairwise collision checks：`K(K-1)/2 = O(K^2)`；
- 每个 nearest distance：`O(K)`；
- 一个 random joint configuration 同时合法的概率下降；
- coordinated bottlenecks 在 product space 中占比变小；
- local-motion checking 要验证更多 drones 与 pairs。

所以 scalability 问题不只是某一行代码变慢，而是计算成本和采样难度同时上升。

## 14.6 E7 为什么能解但成功率低

E7 有两个错位 passages：先经过 `y about 25` 的 opening，再转向 `y about 15` 的 opening。

RRT 能通过多轮不同随机 directions 累积弯曲 tree，所以不是理论上不能解。但：

- 单次 local motion 是 straight joint interpolation；
- 很多 attempts 在 wall 前 `TRAPPED`；
- 只有 valid nodes 被保存，碰墙方向不会保存；
- 多架 drones 的联合安全过孔 configurations 很难采到；
- 20 秒只允许有限 attempts。

一次成功说明 feasible；低 success rate 说明 finite-budget sampling difficulty。

## 14.7 B4 为什么必须做 controlled experiments

单次运行无法区分算法能力与随机运气。系统实验要做到：

1. 固定 planner、`K`、start/goal、workspace size、`eta` 和 time limit；
2. 每次只改变一个主要 environment feature；
3. 使用多个 seeds 重复；
4. 统一记录 success 和 capped runtime；
5. 报告 mean 与 95% confidence interval；
6. 把观察结果联系到 collision rejection、narrow passages 和 expansiveness。

你的 B4 comparisons 分别隔离：

```text
E0--E3: obstacle density without a critical bottleneck
E4 vs E5: passage width
E5 vs E6: route redundancy
E7: sequential offset passages
E9--E11: coupled constraints and local traps
```

对 `n` 次 runtime observations，sample mean 为：

\[
\bar t=\frac{1}{n}\sum_{i=1}^{n}t_i.
\]

使用 sample standard deviation `s` 的 two-sided Student-t 95% interval：

\[
\bar t\pm
t_{0.975,n-1}\frac{s}{\sqrt n}.
\]

failed runs 保留约 20 秒 timeout，得到 capped runtime。这样 mean 同时反映成功运行速度与 timeout burden，但它不是“只在成功条件下的平均 planning time”。

实验的核心结论是：raw obstacle count 会增加 rejection 和 checking cost，但 free-space geometry 更重要。窄、错位或低 visibility 的 passages 会让 `alpha/beta` 变小，使 finite-time success 显著下降；增加 alternative passages 可以恢复 connectivity 被采样捕获的机会。

---

# 15. 方法比较：口试最容易追问

| Method | Representation | Query type | Main strength | Main weakness |
|---|---|---|---|---|
| Visibility Graph | graph of polygon vertices | usually one query | shortest polygonal path | explicit polygonal forbidden regions |
| Uniform Grid | graph of free cells | one/multiple | simple occupancy tests | resolution and dimensionality |
| PRM | sampled roadmap graph | multiple queries | roadmap reuse | narrow passages, construction cost |
| EST | sparse-region-biased tree | single query | explicitly expands low-density regions | density computation/parameter choices |
| RRT | Voronoi-biased tree | single query | fast exploration | path quality and narrow passages |
| RRT-Connect | two RRT-style trees + greedy connect | single query | fast feasible-path search | greedy straight connections can trap |
| RRT* | rewiring tree | asymptotically optimal | improves path cost | more work; unnecessary when only feasibility is scored |

## 15.1 Completeness 类型不要混淆

- **Complete**：若有解必在有限过程找到；若无解也能正确报告。
- **Resolution-complete**：相对于给定 discretisation/resolution 完备。
- **Probabilistically complete**：有解时，失败概率随 samples 趋于无限而趋近零。
- **Asymptotically optimal**：samples 趋于无限时，solution cost 趋近最优；这比 probabilistic completeness 更强，不是普通 RRT/RRT-Connect 的性质。

## 15.2 Feasible、valid、optimal

- feasible problem：至少存在一条满足所有约束的 path；
- valid configuration/edge/path：通过对应 validity definition；
- optimal path：在给定 cost 下最优，例如最短；
- RRT-Connect 的目标主要是快速找到 feasible path，不保证 optimal。

---

# 16. 高频英文口试短答

## 1. What is a configuration?

> A configuration is a minimal set of parameters that completely specifies the position of every point on the robot.

## 2. What is configuration space?

> Configuration space is the set of all possible robot configurations. A robot configuration is represented as one point in this space, and a robot motion becomes a continuous path through it.

## 3. What is the difference between workspace and C-space?

> Workspace is the physical space occupied by the robot and obstacles. C-space is the parameter space of robot configurations. Collision geometry is usually evaluated in workspace, while planning is formulated as finding a path in the free subset of C-space.

## 4. What are `C_obs` and `C_free`?

> `C_obs` is the set of configurations that violate collision or other validity constraints. `C_free` is its complement within the admissible configuration space.

## 5. Why can the standard visibility graph not be directly used in A1 Q1(a)?

> The non-point sphere must be planned for in C-space. Expanding a polyhedral obstacle by the sphere radius generally creates curved forbidden-region boundaries, whereas the standard visibility graph assumes polygonal forbidden regions and uses their vertices as nodes.

## 6. Why can uniform grid discretisation still be used?

> It only needs a free-or-occupied test for each cell and adjacency between free cells. It does not require an explicit polygonal representation of the forbidden-region boundary, although its success depends on grid resolution.

## 7. What are the four main components of a sampling-based planner?

> A sampling strategy, a connection strategy, a configuration-validity test, and a local planner that validates motions between configurations.

## 8. Why is PRM suited to multiple queries?

> Its roadmap captures reusable free-space connectivity. After construction, different initial and goal configurations can be attached to the same roadmap and answered by graph search.

## 9. Why are narrow passages difficult?

> They occupy a small measure of free space and often have poor local visibility. Uniform samples rarely land in the useful lookout region, so the planner may cover large open regions without capturing their connectivity.

## 10. Explain the Bridge Test.

> It samples two nearby colliding configurations and accepts their midpoint if that midpoint is collision-free. The midpoint is then likely to lie between obstacles, possibly inside a narrow passage. It biases sampling but does not guarantee that the roadmap or tree can connect to that sample.

## 11. Why do free endpoints not imply a free edge?

> The local segment can pass through a forbidden region even when both endpoints are free, so the entire motion must be collision-checked.

## 12. What is the difference between EST and RRT?

> EST explicitly favours low-density tree regions and samples locally around the selected node. RRT samples globally first and extends the nearest tree node, obtaining exploration bias through Voronoi regions.

## 13. What does probabilistic completeness mean?

> If a valid path exists, the probability that the planner fails to find one approaches zero as the number of samples approaches infinity. It is not a finite-time guarantee.

## 14. Why does planner failure not prove infeasibility?

> Sampling-based planners are one-sided. A returned valid path certifies success, but failure may mean either that no path exists or that the finite sampling budget did not discover it.

## 15. Explain epsilon, alpha and beta in one sentence each.

> Epsilon measures how much free space an ordinary point can see. Beta measures how much unexplored free space a lookout can see. Alpha measures how many such lookout points exist in an explored connected region.

## 16. Why can two equal-width passages have different difficulty?

> Their local visibility and connection to the surrounding free space may differ. An offset passage requires more turns, produces smaller lookout regions, and usually has smaller alpha or beta than an aligned passage.

## 17. Why is the A1 multi-drone C-space `3K` dimensional?

> Each spherical drone contributes only its three translational centre coordinates. Centralised planning forms one joint configuration by taking the Cartesian product of all `K` drone positions.

## 18. Does one tree node represent one drone?

> No. One node stores the complete `(K,3)` joint configuration, so it represents the simultaneous positions of all drones.

## 19. Do all drones move in the same direction during `Steer`?

> No. They move toward their own target positions and may have different directions and distances. They share the same interpolation fraction, and the maximum individual-drone displacement is bounded by `eta`.

## 20. Why use RRT-Connect for Part B?

> It is a single-query feasibility problem with a short time budget. Bidirectional exploration and greedy connection can find feasible paths quickly without the roadmap-reuse cost of PRM or the optimisation overhead of RRT-star.

## 21. Why swap the two trees?

> Swapping makes both the initial-rooted and goal-rooted trees alternate between random exploration and greedy connection. The roots do not change; only their roles in the next iteration change.

## 22. Why does increasing `K` hurt performance?

> The joint dimension grows as `3K`, pairwise collision checks grow quadratically, and useful coordinated free configurations occupy a smaller fraction of the joint space.

## 23. Is your returned path mathematically guaranteed continuously collision-free?

> It is validated relative to the supplied fixed-step `motion_valid` primitive. That primitive checks many interpolated states, but it is not an analytic continuous-collision detector and can theoretically miss sufficiently small collision features.

## 24. What is the result of Part A Q2(a)?

> Every robot point travels at most `alpha d_C(q,q')`, with the valid upper bound `alpha = LK(K+1)/2`. It follows by summing the maximum downstream lever-arm contribution of every upstream joint.

## 25. What is the result of Part A Q2(b)?

> Any radius strictly smaller than `d_W(q,Obs)/alpha` guarantees the straight path remains collision-free. The ratio itself is the supremum when obstacle contact is treated as collision.

---

# 17. 一页速记

```text
Configuration
= completely specifies every point on robot

C-space
= all configurations; dimension = DOF

C_obs
= invalid/colliding configurations

C_free
= C \ C_obs

Planning
= find continuous path q_I -> q_G inside C_free

Visibility Graph
= polygon vertices + visible straight edges
= explicit polygonal forbidden region

Uniform Grid
= free cells + neighboring-cell edges
= resolution-dependent; high-dimensional explosion

PRM
= sampled reusable roadmap; multiple queries

EST
= inverse-density node selection + local sample

RRT
= global random sample -> nearest -> steer -> validate -> add

RRT-Connect
= one tree extends once, other greedily connects, then swap

State free != edge free

Bridge Test
= invalid q1 + invalid q2 + free midpoint
= narrow-passage bias, not connection guarantee

Probabilistic completeness
= path exists => P(failure) -> 0 as samples -> infinity
= no finite-time guarantee; Failure != infeasible

epsilon
= ordinary-point visibility

beta
= new space visible from one lookout

alpha
= fraction of points that are lookouts

Part A 2(a)
alpha = LK(K+1)/2

Part A 2(b)
tau < d_W/alpha
tau_sup = d_W/alpha

Part B
q=(p1,...,pK) in R^(3K)
one node = all drones
pair checks = O(K^2)
```

---

# 18. 与 Codewalk MASTER 的分工

先用本文件回答：

- 课程概念是什么；
- 公式为什么成立；
- 不同 planner 有什么区别；
- assignment 为什么这样建模。

再用 `COMP8620_A1_Codewalk_MASTER.md` 回答：

- 真实函数在哪一行；
- `Node`、`EXTEND`、`CONNECT` 如何实现；
- simulator 实际检查了什么；
- timeout、path recovery、experiments 如何工作；
- 理论报告与真实代码有哪些细微差异。

两份文件合起来才覆盖完整准备范围：

\[
\boxed{
\text{Oral Theory MASTER}
+
\text{Codewalk MASTER}
=
\text{Week 1--3 theory + assignment + implementation defence}
}.
\]

---

# 19. 本地来源

- A1 specification：`/Users/apple/github/8620/a1.pdf`
- Week 1：`/Users/apple/Documents/26 s2/COMP8620/w1/week1-Intro.pdf`
- Week 2：`/Users/apple/Documents/26 s2/COMP8620/w2/week2-SamplingBasedMP.pdf`
- Week 3：`/Users/apple/Documents/26 s2/COMP8620/w3/week3-SamplingBasedMP-Cont.pdf`
- Tutorial Week 2：`/Users/apple/Documents/26 s2/COMP8620/tutorial-w2.pdf`
- Tutorial Week 2 solutions：`/Users/apple/Documents/26 s2/COMP8620/tutorial-w2-SolEx.pdf`
- Tutorial Week 3：`/Users/apple/Documents/26 s2/COMP8620/tutorial-w3.pdf`
- Tutorial Week 3 recap：`/Users/apple/Documents/26 s2/COMP8620/tutorial-w3-lectureRecap.pdf`
- Submitted report source：`/Users/apple/github/8620/A1/u8076655_A1_report.tex`
- Implementation guide：`/Users/apple/github/8620/COMP8620_A1_Codewalk_MASTER.md`
