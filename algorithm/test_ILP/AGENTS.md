# PR_tool / algorithm/test_ILP 工程指南

本目录实现统一细粒度 SAT 路由及其优化前端。默认流程是第十四版 CaDiCaL 可行性路由；`--z3-optimize` 是第十六版 Z3 Weighted Partial MaxSAT；`--global-route-v17` 是第十七版 HiGHS Channel/COBUnit Global Routing 后接同一套 Z3 Detailed Routing。第十七版定义以 `../../问题定义与方法/第十七版方法.md` 为准。

## 当前三条入口

- 默认：统一图 + D/A 精确距离状态 + CaDiCaL assumptions，UNSAT core 驱动 bbox/distance 扩展。
- `--z3-optimize`：原 CNF 全部作为 hard constraints，所有 pair alpha 作为 external assumptions，以物理 Track/Bump 占用 `U_v` 的单位软约束最小化并集线长。
- `--global-route-v17`：自动启用 Z3 Optimize；先用 HiGHS 在 Channel 图上联合选择 COBUnit、MCF route guide 和 bus Channel 数等长，再以 guide 和 unit assumption 初始化详细求解。

旧 `--ilp-optimize/-L/-R/--time-limit` 与 `ilp_v15/` Gurobi refinement 已删除。

## 第十七版流水线

1. `build_routing_nets` 归一化 Bnet、Tnet、PNnet、fanout 和 SyncNet。
2. `build_unified_graph` 构造真实 Track/TOB/COB 细粒度图；`augment_graph_for_pnnet` 加 PN virtual source。
3. `build_global_channel_graph` 使用显式 COB/terminal 节点和物理 Channel 边资源。普通 Channel 连接相邻 COB；TOB 挂接节点插在其下方 Channel 两个 COB 之间，两个半段共享一个 `channel_id`；实际出现的 external/01 port 建立私有 terminal 节点，并保留 42 个 boundary terminal。
4. HiGHS MIP 使用：
   - owner/unit 变量 `Q`，普通 bump net 可选 16 unit，external track 固定 `map_track(track)`；
   - owner/Channel 占用 `X`；
   - 非固定 unit 的 `W=X∧Q`；
   - per-pair/commodity 带 `channel_id` 的拓扑弧流 `F`，port 弧只对对应 commodity 建变量；
   - PN candidate source-choice；
   - 节点 flow conservation、terminal Channel、`F_a⇒X_{channel(a)}` 与 `X⇒incident F/source`；
   - `(Channel,unit)` 容量不超过 8；
   - 每 TOB/unit load 不超过 8、每 TOB/bank/residue load 不超过 8；
   - 2-pin SyncBus members 的 `sum X` 相等。
5. 目标最小化所有 owner 的 Channel 并集 `sum X`。宏观模型不增加 MTZ/无环约束；无用 `X` 由正目标排除，`F` 在已选 Channel 内允许环。
6. `apply_global_route_v17` 写入 per-pair 非矩形 Channel guide、per-source unit 和由选中宏观弧数加端点开销得到的 detailed distance cap。
   - multi-sink PNnet 在现有共享 virtual-root 语义下保留各 demand 所选等价同极性 source 的并集，并屏蔽其余 virtual arcs；第三层不再保留 per-demand source-choice 标签。
7. `compute_pair_delays` 在 guide 的细粒度投影中求 `d_min`，首轮 domain 初始化为连续区间 `{d_min,...,max(d_min,L_pair)}`。
8. Bnet 的 Global Routing unit 通过可追踪 assumption `gamma⇒Q_sat(unit)` 固定；不写不可撤销 unit clause。
9. Z3 hard-UNSAT 时分别处理：
   - alpha core：critical pair 每次扩一个 distance；同 net 每第二次失败把非矩形 Channel guide 扩一跳；
   - gamma core：只取消 core 中对应 Bnet/source 的 unit 固定，并在原 guide 内开放全部 16 unit；
   - 非 core net 的 unit 保持不变。
10. Z3 Optimal 后仍使用现有提取、物理合法性校验以及 `objective == reconstructed union wirelength` 不变量。

第一层只编码必要条件，不能保证 TOB mux、Wilton lane、跨 COB lane 一致性或详细资源互斥可解；最终 Z3 hard model 才是物理可行性证明。第十七版第一次 Optimal 只保证当前 guide/domain 内最优，不声称完整硬件图上的全局线长最优。

SyncBus 的 Channel-count 等长是用户选定的宏观代理约束，不是细粒度 exact-distance 等长的数学必要条件。因此 `GLOBAL_ROUTE_Infeasible` 只表示 V17 前端未生成 guide，不能报告整个设计物理无解。当 guide 和 distance 未达到完整域时，反馈轮数耗尽统一返回 `SEARCH_LIMIT`，也不报告全局 `UNSAT`。

## 目录职责

| 路径 | 职责 |
|---|---|
| `common/` | RoutingNet、结果统计、COBUnit/硬件坐标映射 |
| `scope/` | net 聚合、pair 状态、bbox 与 Channel guide 状态 |
| `graph/` | 统一细粒度图与 PN virtual source |
| `global_route_v17/` | Channel 图、HiGHS MCF、结果提取、guide 应用/扩展 |
| `delay/` | scoped BFS、连续 distance domain、active D/A 稀疏 mask |
| `sat/unified_sat_scope` | bbox 或 V17 Channel/unit guide 到细粒度 node/arc scope |
| `sat/unified_sat_encoder` | D/A/Q/alpha/gamma/Y/M、TOB 与 bus hard constraints |
| `sat/z3_routing_feedback` | V17 编排、alpha/gamma core 分类和局部反馈 |
| `sat/node_occupancy` | Track/Bump `U_v` 与 `D⇒U` |
| `sat_allocation/` | CaDiCaL session 与 Z3 Optimize wrapper |
| `test/unit_main.cc` | 合成图、MIP、scope、assumption 与原 SAT 单元验证 |

## 构建和合成测试

```bash
xmake f --cadical=y --z3=y
xmake build test_ILP
xmake build test_ILP_unit
./output/test_ILP_unit
```

路径均相对 `xmake.lua` 所在工程根目录解析，可在子目录执行 `xmake`。HiGHS：`HIGHS_HOME`、`HIGHS_ROOT`、macOS `third_party/HiGHS/install-macos`、`third_party/HiGHS/install`（Linux 可用 `lib` 或 `lib64`）。Z3：`Z3_HOME`、`Z3_ROOT`、`third_party/z3/install`、macOS Homebrew。CaDiCaL：`third_party/cadical/src` + `build`（macOS 若存在则优先 `build-macos`）。

第十七版运行：

```bash
./output/test_ILP <config_path> --global-route-v17 -v -o <output_dir>
```

`--global-route-v17` 不与 `-s/-d` 联用，因为 guide 和 distance cap 已由第一层初始化。普通 `--z3-optimize` 和默认 CaDiCaL 流程仍支持原 `-s/-d`。

合成单测必须至少覆盖：

- 普通 2-pin net 恰好一个 unit、terminal 连通和 Channel 目标重算；
- external track fixed unit；
- PN reachable candidate source/unit；
- SyncBus member Channel 数等长；
- fixed/released unit 的 guide lane 开放范围；
- `gamma` assumption 冲突能出现在 failed core；
- 既有 TOB/COB/SAT、路径提取、bus detailed 等长和 Z3 objective 不变量。

无需用 `test/config` 真实 case 作为第十七版的基本回归。

## 日志规范

关键阶段使用 `debug::info_fmt`，字段稳定、可统计：

- `V17 Global Routing graph`：COB/TOB/port/boundary 节点数、physical_channels、directed_traversal_arcs、collapsed_track_nodes；
- `prepare`：nets/owners/commodities/buses；
- `model built`：vars/constraints/build_ms；
- `V17 Global Routing ILP model stats (-v)`：图节点/Channel/owner/commodity 维度，`Q/X/W/F/S` 变量分解，13 类线性约束及与 HiGHS 总数的一致性；
- per-owner/per-pair（`-v`）：net、owner、unit、Channel 数、selected arcs；
- `validation`：objective、最大 Channel-unit load、pair 数；
- `summary`：status、规模、objective、total/build/solve ms；
- Z3 每轮：alpha/unit assumptions、soft 数、core 分类、release/guide expansion；
- main 汇总：Global Routing 规模/耗时/released sources，SAT 规模/耗时，最终 wirelength。

不要把 Global Routing Channel objective 记为 detailed wirelength，也不要把 Channel 数直接用作 SAT distance。

## 修改要求

- 改动前核对方法文档和 `source/hardware` 映射；优先只改 `algorithm/test_ILP/`。
- 不修改 `source/algo/router/` 的正式路由流程。
- 关键约束必须有合成单测；不要依赖大 case 才暴露基本建模错误。
- 单文件保持紧凑，避免无关重构；新增关键步骤保留日志和规模/耗时统计。
- 单次修改超过 100 行时，在实现完成后启动独立 reviewer 子 agent。
