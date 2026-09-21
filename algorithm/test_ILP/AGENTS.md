# PR_tool / algorithm/test_ILP 工程指南

## 项目目的

`test_ILP` 是 PR_tool 内用于研究和验证 FPIA/Silicon Interposer 布线方法的独立可执行程序。它读取与正式 PR_tool 相同的配置，把 Bnet、Tnet、PNnet 和 SyncBus 归一化为统一连接问题，并在真实 Track/COB/TOB 硬件资源上完成：

- Channel/COBUnit 粒度的 HiGHS Global Routing；
- Track/TOB-switch 粒度的 CaDiCaL 或 Z3 详细布线；
- UNSAT-core 驱动的 distance/scope 扩展；
- SAT 可行解之后的 ILP 或 Maze/RRR 线长优化；
- 模型规模、求解时间、路径和物理合法性的诊断。

该目录是方法实验入口，不应无意修改 `source/algo/router/` 中的正式路由流程。

## 当前求解流程

### 入口模式

| 命令行模式 | 求解流程 |
|---|---|
| 默认 | 统一细粒度图 + CaDiCaL hard SAT，从较小 distance/scope 开始，根据 UNSAT core 扩展 |
| `--z3-optimize` | 使用相同 hard CNF，通过 Track/Bump occupancy soft clauses 最小化物理并集线长 |
| `--global-route-v17` | HiGHS Global Routing 选择 Channel guide/COBUnit，随后用 Z3 Weighted Partial MaxSAT 详细布线 |
| `--global-route-v18` | PN source/unit 预选 + 容量剪切版 HiGHS Global Routing + CaDiCaL 纯 SAT 详细布线 |
| `--global-route-v18 --ilp-optimize` | 在 SAT 成功后固定 SyncBus，对所有非 bus net 执行 V22 pair-flow ILP |
| `--global-route-v18 --maze-optimize` | 在 SAT 成功后固定 SyncBus，并保留所有非 Sync net 的 SAT physical source、COBUnit 和最终 scope，执行 V20 局部 Maze/RRR |

`--ilp-optimize` 与 `--maze-optimize` 互斥，且都要求 `--global-route-v18`。Global Routing 模式不能再使用 `-s/-d`，因为其 scope 和 distance 由 guide 初始化。

### 方法版本来源

当前代码以现有实现为准，方法文档用于说明设计来源：

- 第十四版：统一图、exact-distance `D/A`、TOB/bus hard constraints 与 core-guided expansion；
- 第十六版：Z3 Weighted Partial MaxSAT 的 Track/Bump occupancy 并集线长；
- 第十七版：Channel/COBUnit 粒度的 HiGHS MCF Global Routing；
- 第十八版：PN source/unit 预选、无稠密 `W` 的容量剪切和 CaDiCaL 纯 SAT；
- 第十九至二十一版：fixed-unit 精确容量、maze MIP start、bbox/guide/scope 策略、TOB 峰值代价和 scope 闭包；
- 第二十二版：固定 SyncBus 的 post-SAT 无向 pair-flow ILP、全局 physical-edge union 与 scope-overlap 精确分解。

文档位于 `../../../问题定义与方法/`。旧 `problem_formulation/` 文件用于历史对照，不能代替当前代码语义。

## 目录结构与关键文件

| 路径 | 主要职责 |
|---|---|
| `main.cc` | 读取 CLI/配置，初始化 `debug.log`/`highs.log`，调用统一求解入口并输出总结 |
| `test_ilp_cli.*` | 参数解析和模式互斥检查 |
| `common/` | 路由类型、结果统计、COBUnit mask 和硬件坐标映射 |
| `scope/build_routing_nets.*` | 把原始 circuit net 归一化为 `RoutingNet/Source/Demand` |
| `scope/pair_routing_state.*` | 保存每个 `PairKey` 的 distance domain、pair bbox、Channel guide 和反馈计数 |
| `scope/scope_bbox.*` | 原始 bbox、guide repair、one-hop 扩展和 Channel 闭包 |
| `graph/unified_routing_graph.*` | 建立 Track/Bump/HLine/VLine/port 顶点和 COB/TOB 有向弧；PNnet 可选虚拟 source |
| `global_route_v17/global_router.*` | 构建 COB-terminal/Channel 图，建立 `Q/X/W/F/S/Z/H7/H8` 模型，实现 MIP start、容量 cut、提取与 guide 应用 |
| `global_route_v17/pn_source_preselection.*` | 按 `(PNnet,bump,unit,source)` 预选 PN 物理 source/unit，并转换为 fixed-source multi-sink Tnet |
| `global_route_v17/global_guide_log.*` | 重建和打印 Global Routing pair guide，诊断 residual/cycle/branch |
| `delay/pair_delay_precompute.*` | scoped BFS、`d_min`、distance domain 和 reachable-layer active `D/A` mask |
| `sat/unified_sat_scope.*` | 把 bbox 或 Channel guide/unit 投影到细粒度 node/arc scope |
| `sat/unified_sat_encoder.*` | 建立 `D/A/Q/alpha/gamma/Y/M` 主模型和全局 exclusivity |
| `sat/encode_tob_special.*` | TOB physical-switch OR、partial matching 和 straight/swap mode |
| `sat/encode_bus_sync.*` | SyncBus detailed-distance 等长 |
| `sat/routing_feedback.*` | V18 CaDiCaL 求解、物理验证、core 分类和 pair-level distance/scope 扩展 |
| `sat/z3_routing_feedback.cc` | Z3/V17 求解编排和 occupancy objective |
| `sat/sat_solution_extract.*` | 从带 distance 的 SAT 状态恢复 source-sink 路径、switch 和 mode |
| `sat/routing_solution_validate.*` | 独立检查端点、scope、COBUnit、节点/开关互斥、TOB matching/mode 和 bus 等长 |
| `sat/routing_path_log.*` | 路径、final pair scope 和线长日志 |
| `sat_allocation/` | CaDiCaL session 与 Z3 Optimize wrapper |
| `post_sat_ilp/` | V22 无向物理图、pair-local `F/D`、全局 physical-edge `Y`、pairwise exclusivity、interaction component、HiGHS warm start、提取与回退 |
| `post_sat_rrr/` | V20 在最终 SAT scope 内的局部 rip-up-and-reroute |
| `test/` | 合成单测、配置 smoke/golden 回归与单一测试入口 |

## 关键数据结构和不变量

- `RoutingNet` 是归一化 net；`RoutingDemand` 描述 sink 及候选 source；`SourceSinkPairPath` 是最终细粒度路径。
- `PairKey=(net_id,demand_id,source_index)` 是 guide、distance、scope 和 UNSAT 反馈的基本索引。
- `RoutingProblemState` 必须保留 pair-local `allowed_channels`；SAT 编码可使用同 net pair scope 的并集，但 post-SAT ILP 必须回到 pair-local scope。
- `UnifiedGraph` 中 Track/Bump/HLine/VLine 是物理资源顶点；SAT 使用有向弧，V22 post-SAT ILP 则把相同端点的正反向弧折叠成一条无向物理边，并保留 switch/matching/mode 属性。
- Global Routing 的 `estimated_wirelength` 以 Channel 为单位；Detailed Routing 的 `total_wirelength` 是每个 net 去重后的 Track+Bump 数，两者不能混用。
- V18 Global Routing 只编码 TOB/COBUnit 的必要条件；最终 CaDiCaL/Z3 hard model 和独立 validator 才证明详细硬件合法。
- SyncBus 的 Global Routing Channel-count 等长是宏观代理，SAT 阶段仍必须检查 exact detailed distance 等长。

## 关键算法概要

### V18 Global Routing

1. PN 预选先固定 physical source/unit，并把非空 `(PNnet,source)` 组转为 Tnet。
2. 每个 owner 用 `Q` 选 unit，用 `X` 表示 Channel 并集，每个 pair 用 `F` 表示 Channel 图有向流。
3. 正式 MIP 仅为 pair bbox+1 并集内的资源建变量，并在首次求解前加 fixed-unit 精确容量行和 maze-routing MIP start。
4. 无稠密 `W` 模式通过 incumbent 超载检查逐轮增加 Channel-unit cut。
5. TOB 负载必须满足 unit<=8 和 bank-residue<=8；V21 的 `H7/H8` 代价用于降低 TOB 下方 Channel 峰值占用。
6. guide 应用后加 TOB patch；TrackToBump(s) 和 PN source-tree 再执行一次 one-hop 外推及已达 COB 集合的内部 Channel 闭包。

### Detailed SAT 与反馈

- `D(s,v,d)` 表示 source 在精确 distance 到达顶点；`A` 只显式表示 TOB 转移；`Y/M` 聚合物理开关与 mode。
- reachable-layer mask 同时要求顶点从 source 可达且能在允许 distance 到达 sink。
- alpha core 每次只扩展对应 pair 的 distance；同一 pair 连续 4 次 distance-only 失败后，第 5 次同时把该 pair scope 外推一跳。
- gamma core 只释放 core 中对应 Bnet/source 的 Global Routing unit assumption。
- V18 Pose/Nege physical-source tree 初始 distance domain 为 `{d_min,d_min+1,d_min+2}`；其他普通 pair 为 `{d_min}`；SyncBus 使用 member `d_min` 的共享最大值。

### SAT 后优化

- V22 ILP 只固定 SyncBus。每个非 bus pair 在自己的 final pair scope 并入 incumbent path 后，对无向物理边/节点建立 `F/D`；端点度为 1，内部节点度为 2 或 0。
- 每条 component 候选物理边只建一个全局 `Y`，用 `min sum(Y)` 且边代价全为 1；Track+Bump `total_wirelength` 在求解后独立重算。
- `pairs_by_edge`/`pairs_by_node` 倒排索引只为 scope 实际重叠的不同-net pair 建立论文公式（9）/（10）的 pairwise 边/节点互斥；同 net pair 允许共享主干。
- TOB matching 和 straight/swap `M` 仍是硬约束；不额外建立环路消除约束。
- 候选真实节点/边/matching/mode 资源有交集的 net 归入同一 interaction component，分量分别用 SAT path warm start 求解。
- 任一分量求解、提取、物理验证失败，或线长退化，都整体回退 SAT baseline。
- V20 Maze/RRR 以 SyncBus 为硬障碍，按 owner 整棵树 rip-up；其他非 Sync net 可临时 overflow，未收敛或不严格变短则回滚。每个 owner 始终限制在自己的 final SAT scope，并固定 SAT 已确定的端点、physical source 和 COBUnit；V18 PN 预选生成的 `pn_source_tree` 同样保留预选 source/unit。

## 构建与运行

以 `PR_tool/xmake.lua` 所在目录为工程根目录：

```bash
xmake f --cadical=y --z3=y
xmake build test_ILP
xmake build test_ILP_unit
```

依赖查找顺序：HiGHS 优先使用 `HIGHS_HOME/HIGHS_ROOT`，否则使用 `third_party/HiGHS/install-macos` 或 `third_party/HiGHS/install`；Z3 使用 `Z3_HOME/Z3_ROOT`、`third_party/z3/install` 或 macOS Homebrew；CaDiCaL 使用 `third_party/cadical/src` 和 `build`/`build-macos`。

常用运行方式：

```bash
./output/test_ILP <config_path> -v -o <output_dir>
./output/test_ILP <config_path> --z3-optimize -v -o <output_dir>
./output/test_ILP <config_path> --global-route-v17 -v -o <output_dir>
./output/test_ILP <config_path> --global-route-v18 -v -o <output_dir>
./output/test_ILP <config_path> --global-route-v18 --ilp-optimize -v -o <output_dir>
./output/test_ILP <config_path> --global-route-v18 --maze-optimize -v -o <output_dir>
```

`-o` 目录中的 `debug.log` 是主日志，`highs.log` 是 PN 预选、Global Routing 和 post-SAT ILP 的 HiGHS 日志。`-v/-vv/-vvv` 逐级增加模型统计、HiGHS 回显和 scope 诊断。`--time-limit MIN` 限制正式 Global Routing 的共享容量剪切时间；启用 post-SAT ILP 时，同一数值也作为每个 interaction component 的时限。

## 测试方法与分布

所有合成测试仍编译为一个 `test_ILP_unit` 可执行文件。`test/unit_main.cc` 只保留公共 include、分类文件组装和测试调用顺序；各 `.inc` 在同一匿名 namespace/编译单元中展开，因此共享 fixture 不对生产代码导出符号。

| 测试文件 | 职责 |
|---|---|
| `test/unit_main.cc` | 单测入口、分类文件组装和固定调用顺序 |
| `test/unit/common_graph_cases.inc` | 共享 synthetic fixture、net 归一化、bbox/geometry、统一图和 CaDiCaL session |
| `test/unit/global_route_core_cases.inc` | PN 预选、V17/V18/V19 Global Routing 变量/容量/目标与 Channel 拓扑 |
| `test/unit/global_route_scope_cases.inc` | V21 TOB peak、guide apply/repair、distance 初始化、pair scope 扩展和 guide 日志 |
| `test/unit/post_sat_cases.inc` | V22 无向 pair-flow ILP、scope-overlap/pairwise exclusivity、固定 bus/mode、固定 source/unit/scope 的 V20 Maze/RRR、V18 guided smoke 与底层 SAT fixture builder |
| `test/unit/sat_feedback_cases.inc` | 约束工具、distance state、assumption/core feedback、reachable-layer 和 COBUnit mask |
| `test/unit/sat_encoding_cases.inc` | `D/A/Q/Y/M`、bus equality、CLI、initial padding、TOB switch 聚合和 encoding stats |
| `test/unit/extract_validate_cases.inc` | 解提取、path/scope 日志、物理 validator、ideal wirelength、PN virtual source 和 golden 回归 |

运行：

```bash
xmake build test_ILP_unit
./output/test_ILP_unit
```

新测试应放入职责最接近的分类文件，不要再把大段测试体写回 `unit_main.cc`。每个测试文件保持在 1000 行以内；若某分类接近上限，按算法责任继续拆分，不要仅按行数平均切割。基本建模错误必须用小型 synthetic graph 复现，不依赖大型真实 case。

## 日志和统计约定

- Global Routing 统计必须区分 COB/terminal 图节点、physical Channel 资源和 directed traversal arc。
- HiGHS model stats 必须输出各类变量/约束及总数；V18 还需输出 scope 槽位裁剪率、MIP start 和 capacity-cut 轮数。
- Global guide 日志必须区分有序 source-target walk、residual selected arcs、TOB patch、initial expansion 和 final pair scope。
- Detailed SAT 每轮记录 vars/clauses、alpha/gamma assumptions、core 分类和 distance/scope 扩展。
- V22 post-SAT ILP 记录 non-bus/fixed-bus net、pair-local node/undirected-edge slots、interaction components、`F/D/Y/M`、pairwise edge/node conflict rows、warm start、physical-edge objective、Track+Bump 线长、GAP/耗时和 fallback 原因。
- 路径打印、final scope 打印等诊断耗时必须写入 `excluded_diagnostic_ms`，不计入 Global Routing/SAT/total routing time。
- 不要把 Global Routing Channel objective 记为 detailed wirelength，也不要把 Channel 数直接当作 SAT distance。

## 修改要求

- 改动前核对方法文档和 `source/hardware` 映射；优先只改 `algorithm/test_ILP/`，不修改 `source/algo/router/` 的正式路由流程。
- 关键约束必须有合成单测；不要依赖大 case 才暴露基本建模错误。
- 单文件保持紧凑，避免无关重构；新增关键步骤保留日志和规模/耗时统计。
- 对算法做工程实现的时候，需要注意一下写出来的代码的运行速度，在不影响算法正确实现的前提下尽可能使速度更快
- 代码文件的单次修改超过 200 行时，在实现完成后进行分离审查：如果实现过程是启动子agent实现的，那么可以由原主agent自己审查；如果实现是由主agent自己实现的，那么需要启动一个子agent审查。如果单词修改不超过200行，可以由实现的agent自己审查。如果审查之后有问题需要修正，但是修正之后不需要再进行单独的审查工作。
