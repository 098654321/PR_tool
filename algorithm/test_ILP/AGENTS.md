# PR_tool / algorithm/test_ILP 工程指南

本文件是 `algorithm/test_ILP/` 子工程的入口说明。当前 `test_ILP` 默认实现**第十四版 D/A 距离语义 SAT 可行性布线**（见 `problem_formulation/第十四版方法.md`）；可选的 `--ilp-optimize -L <percent>` 会在 SAT 成功后执行第十五版 Gurobi MCF 线长优化。不直接替代 `source/algo/router/` 的正式路由流程。

## 项目总体介绍

`test_ILP` 将 TOB 与 COB/track 布线建模到**一张有向图**上，用 CaDiCal 求解可行性 SAT。核心流水线：

1. **Net 聚合**：`build_routing_nets` → `validate_v14_routing_nets`（非 PNnet 每 demand 恰好一个 candidate source；**PNnet 允许多候选 track**）。
2. **Pair 状态初始化**：`init_routing_problem_state` 为每个 `(net, demand)` 建立 `PairRoutingState`（`delays` 集合、`pair_bbox`）；PNnet 逻辑源 `source_index=0`（虚拟 \(r_n\)）；`apply_state_to_nets` 写回 `net.scope_bbox`。
3. **统一图**：`build_unified_graph`（track mesh + 16 TOB 子图）→ **`augment_graph_for_pnnet`**（每 PNnet 追加 `VirtualSource` 节点 \(r_n\) 及 \(r_n\to s_j\) 虚拟弧）→ **可选首轮扩展** `apply_initial_search_padding`（CLI `-s`/`-d`，见下）。
4. **反馈环**（`solve_with_feedback`）：每轮 `apply_state_to_nets` → `build_all_scopes` → `compute_pair_delays(state)` → `build_unified_sat_model`（连通性仅由 `α_{s,t}⇒⋁D` 门控）→ `assume(α)` → CaDiCal `solve()`。
   - **SAT**：`extract_sat_solution` 从 sink 按 delay 递减回溯；PNnet 经虚拟弧回到 \(r_n\)，路径展示从**选中 track** 起算（扣 1 虚拟跳）。
   - **UNSAT**：`failed(α)` 收集 critical pairs → `apply_feedback_expansion` 返回 `Expanded/Exhausted` → 按 net 失败次数奇偶扩边：奇数次只加 `delays` 的 `max+1`，偶数次再加 `pair_bbox` 四边 ±1 → fanout/bus/**PNnet 同 net 多汇**同步 → 全量重建 session/model；critical net 已满片时其它 net 各自按本 net 失败计数扩边；刚扩到全片仍重建求解一次，只有全片状态已求解仍 UNSAT 才 `Exhausted`；`MEMORY_LIMIT` 不扩边。
5. **首轮搜索扩展**（`apply_initial_search_padding`，图构建后、反馈环前执行一次；`UnifiedSatSolveOptions.initial_scope_pad` / `initial_delay_pad`；CLI `-s S` / `-d D`；默认均为 0）：
   - `-s S`：每个 `pair_bbox` 四边各外扩 S 格（重复 `expand_pair_bbox_one_cell` S 次）→ `apply_state_to_nets` 更新 `net.scope_bbox`。
   - `-d D`：在**当前** scope（若已 `-s` 则为扩后 scope）上 `build_all_scopes` + `compute_pair_delays` 得 `d_min`，再设 `delays={d_min,…,d_min+D}`。
   - 普通 2-pin、单源多汇与 PNnet 每个 pair 按自己的 `d_min` 独立扩展，不跨 demand 合并；bus 先由 `compute_pair_delays` 对齐到统一 `bus_d_min`，再为各 member 添加相同区间。
   - `-d` 不修改任何 `pair_bbox`；首轮 padding 不调用反馈阶段使用的 fanout/bus 同步函数。
   - 反馈 round 0 的 `compute_pair_delays(state)` **保留**已填充的 `delays`；仅 scope 扩展、未设 `-d` 时 round 0 才首次写入 `{d_min}`。
6. **Delay 与稀疏域预计算**：先用普通最短路 BFS 求各 pair 的 `d_min`；得到当前 `delays` 与 `d_max` 后，再在当前 scope、当前 `d_max` 内计算分层前向可达与各 sink 的反向可达，只保留位于某个当前允许长度 source-to-sink walk 上的 D 状态。同一 source 的 fanout/PNnet 多 sink 对有效状态取并集；feedback 后按新 scope/delay 全量重算，不预建未来轮次状态。
7. **COBUnit 裁剪与 SAT 编码**：Tnet/IO track source 使用其唯一 unit mask，PNnet \(r_n\) 使用候选 source track 的 unit 并集，Bnet 静态保留全部 16 unit。Track、VLine 与相关弧先按 mask 过滤，再创建有效 `D`；TOB `A_{s,u→v,d}` 仅在两端 D 都有效时创建。每个 Bnet bump source 另建 16 个 `Q(s,u)` 并以 sequential ExactlyOne 选择一个 unit，所有有效 VLine-Track `A` 按 Track 端 unit 编码 `A⇒Q`。另含 `α⇒⋁D`、`Y`、`M_g`、bus `∀d` 等长；PNnet 不建 Q，物理 track 仍只允许 \(d=1\)，多个候选 source 可并存、汇合。

默认流程不包含结果写回 interposer。

方法依据：`problem_formulation/第十四版方法.md`。历史版本见同目录 `第一版方法.md` … `第十三版方法.md`。

## 工作流程要求

- 改代码前读清方法文档与 `source/hardware`、`source/circuit` 映射；**不允许修改方法文档**。
- 优先改动 `algorithm/test_ILP/`；非必要不改 `source/` 主流程。
- 修改后评估是否同步更新本文件（≤200 行）。
- 单次修改 >100 行时，启动子 agent 审查。
- 单文件职责紧凑，不超过 1500 行；关键步骤用 `debug::info_fmt` 打日志。

## 目录结构

```text
algorithm/test_ILP/
├── main.cc
├── common/           # RoutingNet、SatRoutingResult、hw_map
├── scope/            # build_routing_nets、scope_bbox、pair_routing_state
├── graph/            # unified_routing_graph（含 VirtualSource / augment_graph_for_pnnet）
├── delay/            # pair_delay_precompute（BFS、bus_d_min、PNnet r_n 偏移）
├── sat/              # encoder、routing_feedback、encode_tob_special、encode_bus_sync、extract
├── sat_allocation/   # cadical_solver（assume/solve/failed）
├── problem_formulation/
├── mcf/ precompute/ ilp_allocation/ visualization/   # 第十二版遗留，未链接 test_ILP
```

## 关键模块

| 模块 | 职责 |
|------|------|
| `graph/unified_routing_graph` | `VirtualSource` 节点；`is_virtual_source_arc`；`augment_graph_for_pnnet` |
| `scope/scope_bbox` | PNnet：`compute_pnnet_demand_pair_bbox`（按 demand 合并各 \((s_j,t_i)\) Tnet bbox） |
| `scope/pair_routing_state` | per-pair `delays`/`pair_bbox`；`apply_initial_search_padding`（CLI 首轮 scope/delay 预扩展）；fanout/bus/PNnet 同步；全片扩 |
| `delay/pair_delay_precompute` | 普通 BFS 求 `d_min`；当前 `d_max` 内前向+反向精确可达 mask；source unit mask；PNnet 从 \(r_n\) |
| `sat/routing_feedback` | UNSAT core 驱动 scope/delay 扩展；显式 `Expanded/Exhausted` 状态；每轮新建 `CadicalSession` |
| `sat/routing_round_diagnostics` | 反馈轮次星号框、UNSAT 失败 net 汇总、SAT 成功后非最短 net 线长对比（诊断日志，不改求解结果） |
| `sat/ideal_shortest_wirelength` | 按 net 类型计算理想最短 `net_wirelength` 下界（2-pin/bus 用 UnifiedGraph BFS，fanout/PNnet 用 Interposer maze 树） |
| `sat/unified_sat_scope` | per-net 紧凑 scope；PNnet 强制含 \(r_n\)、全部候选 track、虚拟弧 |
| `sat/unified_sat_encoder` | 按有效 mask 稀疏创建 `D`/`A`；Bnet `Q(s,u)` ExactlyOne 与 `A⇒Q`；`α⇒⋁D`；PNnet track \(d\neq1\) 禁止 |
| `sat/encode_tob_special` | `A⇒D`、三类物理连接 `Y` 聚合、`Y⇒M_g/¬M_g`、四类 partial matching |
| `sat/encode_bus_sync` | `∀d`：`D_{ref,t_ref,d} ↔ D_{member,t_i,d}`；不存在的 D 按 false |
| `sat/routing_solution_validate` | SAT 结果诊断校验（路径结构、D/A 回放、跨网资源冲突、bus/PNnet 规则），仅记录日志不改 `out.ok` |
| `sat/sat_solution_extract` | sink→source 回溯；PNnet 剥离 \(r_n\)、记录 `physical_source_node` |
| `sat/sat_encoding_stats` | `-v`：dense/unit-eligible/active D/A、Q、aux、8 类 CNF |
| `ilp_v15/` | 可选的第十五版 Gurobi segment/parent MCF 后优化：parent/segment 拆解、SAT guide MIP start、模型、提取、物理校验和阶段编排 |

### 变量与约束（v14）

- **D**：逻辑 source `s` 到节点 `n` 的精确距离 `d`；只为通过 unit mask 且同时满足当前前向/反向精确可达的 `(n,d)` 创建。`D(source,0)=true`；其余无效槽位不存在。fanout/PNnet 对同一 source 的多 pair 取有效槽位并集。
- **A**：仅 TOB 弧的转移选择；仅当 `D(u,d-1)` 与 `D(v,d)` 都存在时创建 `A_{s,u→v,d}`。
- **Q**：仅 Bnet bump source 创建 16 个 `Q(s,u)`，sequential ExactlyOne；每个 VLine-Track A 根据 Track 的精确 unit 满足 `A⇒Q(s,u)`。Tnet 由静态 mask 固定 unit；PNnet 允许候选 unit 并集，因此不创建 Q。
- **α**：每 pair 一个假设字面量；连通性只由 `α⇒⋁_{d∈delays(s,t)} D_{sink,d}` 门控；若当前允许长度没有有效 sink D，则直接得到 `¬α`，交给现有 UNSAT feedback 扩展。
- **Y / M_g**：Bump-HLine、HLine-VLine、VLine-Track 三类物理开关与 vline-track 模式（1024 组全局 `M_g`）。
- **Bus**：各 member 独立最短 → `bus_d_min=max`；SAT 侧 `∀d` 等等长。
- **PNnet**：整网一个 \(r_n\)；`D_{r_n,r_n,0}=true`；物理 track 仅在 \(d=1\) 可达；路径 hop 统计扣 1 虚拟跳。

### v15 可选后优化

- 仅当同时给出 `--ilp-optimize -L <非负百分比>` 时启动；`-R <非负整数>` 与 `--time-limit <正数小时>` 也只能与该组合一起使用，省略时分别默认 `R=0`、Gurobi 不限时；未启用时不会创建 Gurobi 环境或 `./gurobi/`。
- 对增长率 `actual/shortest-1 >= L` 的整网重布。普通 2-pin、SyncBus member 各为一个 parent+segment；TrackToBumps / PNnet 多扇出先拆 segment，ILP 用 parent 级 `x/y` 与 segment 级 `f` 建模。SyncBus 以 parent 线长等长约束连接。
- 未选 net 的物理节点与 TOB 开关被锁定；模型包含 TOB physical-switch 唯一性、Bump-HLine/HLine-VLine partial matching 和最后一级 straight/swap mode 约束。Gurobi 异常、无可用解或提取校验失败会记录 `fallback_to_SAT=true` 并完整回退 SAT 解。`--time-limit` 触发 `TIME_LIMIT`（或中断）且已有可行 incumbent 时按 Suboptimal 接受当前最优解，不回退 SAT。
- `--ilp-optimize` 的 MIP start 来自 SAT 恢复树 / segment `guide` 弧，填 `f`（segment）与 `x/y`（parent）；`M_g` 仍来自 SAT `vline_mode_straight_by_group`。PNnet 固定**全部** SAT 实际选中的 candidate track，每个 track 保留唯一 `r_n→track` 虚拟首跳；普通 segment 禁止虚拟节点。若 BFS 去除多 source 重汇合后会使任一已选 track 脱离所有 sink，当前单入边 parent-tree 表达不了该结构，会作为不变量错误终止而非静默改源。多扇出 / PNnet 按 BFS 父树剪除死支后拆成 2-pin segment（`-R` 控制 bbox 外推，默认 0）；guide 自洽或覆盖检查失败同样终止，不回退 SAT。`-v` 额外记录已选 track、剪枝前后树规模和 guide coverage。
- 原生日志固定覆盖 `./gurobi/v15_ilp.log`（不输出到控制台）；`debug.log` 记录筛选、模型规模、耗时、状态、前后线长及校验结果。

### v14 不支持

- 非 PNnet 网的多候选 source（`validate_v14_routing_nets` 拒绝）。
- 多源多汇当前仅支持 `TracksToBumpsNet`（归一化为 PNnet）；其他多源多汇 net 类型不在当前 case 范围内。
- SAT+MCF 分阶段、Gurobi MCF。

## 构建与测试

```bash
xmake f --cadical=y
xmake build test_ILP
xmake build test_ILP_unit
./output/test_ILP_unit
./output/test_ILP algorithm/test_ILP/test/case_2btb -v --max-rss-mb 8192
./output/test_ILP algorithm/test_ILP/test/case_2btb -v --max-rss-mb 8192 -s 0 -d 1
./output/test_ILP algorithm/test_ILP/test/case_2btb -v --max-rss-mb 8192 -s 1 -d 1
./output/test_ILP test/config/case7 -v
./output/test_ILP test/config/case7 -v -o output/case7_run
./output/test_ILP test/config/case7 -v --ilp-optimize -L 10
./output/test_ILP test/config/case7 -v --ilp-optimize -L 10 --time-limit 2
```

**输出目录**（可选）：`-o DIR` / `--output DIR` 将 `debug.log` 写到 `DIR/debug.log`（目录不存在时创建）；省略时仍为 `./debug.log`。`--sat-log` 的 `./cadical-log` 与 `--ilp-optimize` 的 `./gurobi/` 路径不受 `-o` 影响。

**ILP 时限**（可选）：`--time-limit H` 仅限制 Gurobi `optimize()` 墙钟（单位小时，须 `H>0`，可小数）；须与 `--ilp-optimize` 联用。超时且已有可行解时接受 incumbent（日志 status 可为 `TIME_LIMIT`，内部按 Suboptimal）。省略则不限时。

**首轮扩展**（可选，与反馈扩边独立）：`-s S` 外扩 pair bbox；`-d D` 初始 delay 集合 `{d_min,…,d_min+D}`。`-v` 时 `main.cc` 打印 `initial search padding: scope_pad=… delay_pad=…`；初始 scope 与 `scope after initial search padding` 分别展示扩展前后范围，round 0 的 `delay net=… delays=[…]` 展示最终 pair delay。

集成 case：`case_2btb`、`case_2btt`、`case_2fanout`、`case_bus2btb`、`case_bus2btt`（各验证基线、`-s 0 -d 1`、`-s 1 -d 1` 三组）；`test/config/case5`（PNnet）；`test/module_test/test_function/testlength` 下 `testiosimple`/`testchipletsimple`/`testchipletbus`/`testiobus` 的 `total_wirelength` 须与各自 `golden.txt` 一致（`testpn` 仅要求 SAT 成功，线长允许与 golden 不同）；建议 `--max-rss-mb 8192`。

`-v` 日志含 scope、delay、`feedback round=`、`feedback critical`、`unified SAT encoding stats`（D/A 的 dense、unit-eligible、active 数量与比例，Q/aux 变量，8 类 CNF）、路径（PNnet 含选中 track；每个 net 末行 `net_wirelength=` 为 net 内 bump+track 去重计数）；成功时末尾分三行汇总：`unified SAT:`（`paths/vars/clauses`，`total_ms` 为整个 SAT 阶段墙钟，`solve_ms` 为各轮 CaDiCaL `solve()` 累计，`pre_ms=total_ms-solve_ms`）、`unified ILP:`（`requested/status/fallback_to_SAT/vars/constraints`，`total_ms` 为整个 ILP 阶段，`pre_ms` 为进入 `optimize()` 前的准备+建模，`solve_ms` 为 Gurobi 求解）、`routing result:`（`total_wirelength`）；轮次内仍有 `delay_precompute_ms`、`model_build_ms`、`round_solve_ms`；`run_main total elapsed` 是完整端到端时间。

反馈扩边示例：`feedback round=2 critical net=3 demand=1 delays=10->10,11 bbox=(2,5,0,6)->(2,5,0,6)`（奇数次）；`feedback round=3 ... delays=10,11->10,11,12 bbox=(2,5,0,6)->(1,6,0,7)`（偶数次）。

## 已知限制

- **反馈环**：每轮全量重建 CNF；`max_feedback_rounds`（默认 64）防止无限循环；只有全片 bbox 状态已经完成一次求解且仍 UNSAT 才终止。
- **CaDiCal core**：`failed()` 不保证最小；空 core 时回退到 max-delay pair。
- **规模**：全图固定 1024 个 `M_g`；D/A 已按当前精确可达与 unit 资格稀疏创建，但大 scope、较大 `d_max` 或 feedback 轮次仍会增加 mask 预计算与 CNF 规模；`MEMORY_LIMIT` 仍用于硬性保护。
- **首轮扩展**：`-s`/`-d` 无配置上界；过大值会膨胀首轮 CNF/RSS。实现会拒绝 `d_min+d` 的整数溢出；fanout、bus、PNnet 的 padding 语义均有单测覆盖。

## 工程风格

- 小步、可解释；不做无关重构。
- `net_id` 由 `build_routing_nets` 顺序分配。
- Tnet / SyncNet member：**track = source，bump = sink**；PNnet：**逻辑源 = \(r_n\)**，物理 track 由解中虚拟弧后继确定。
- 第十二版 SAT+MCF 应独立目标，不与 v14 混用。
