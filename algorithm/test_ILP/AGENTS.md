# PR_tool / algorithm/test_ILP 工程指南

本文件是 `algorithm/test_ILP/` 子工程的入口说明。当前 `test_ILP` 目标实现**第十三版统一图 SAT 可行性布线**（见 `problem_formulation/第十三版方法.md`），不直接替代 `source/algo/router/` 的正式路由流程。

## 项目总体介绍

`test_ILP` 将 TOB 分配与 COB/track 布线建模到**一张有向图**上，用 CaDiCal 单次求解可行性 SAT。核心流水线：

1. **Net 聚合**：`build_routing_nets` 为每个原始 `circuit::Net` 生成一个 `RoutingNet`，用 `RoutingDemand` 保留固定配对或候选 source 关系。
2. **Scope 计算**：`assign_scope_bboxes` 仅按几何需要拆出 bbox 子项，再取 RectHull 得到原始 net 的 COB 级 `scope_bbox`（无扩边重试）。
3. **统一图构建**：`build_unified_graph` 生成 track mesh + TOB 子图（bump / hline / vline / track 节点及弧）。
4. **SAT 编码**：`build_unified_sat_model` 直接向一个 `CadicalSession` 流式写入数值变量和子句，编码 `P` / `p` / `x`、连通性、互斥、`M_g` / `Y`、SyncBus 二进制等长与去环约束。
5. **求解与提取**：同一 CaDiCal session 单次求解，随后按模型中的数值变量直接输出路径。

**不包含**（第十三版明确剔除）：路径预计算、tier 放开、SAT/ MC F 分阶段、Gurobi MCF、scope 扩边重试、结果写回 interposer。

硬件与电路基础来自项目根目录 `source/hardware` 与 `source/circuit`。方法依据以 `problem_formulation/第十三版方法.md` 为准；历史版本见同目录 `第一版方法.md` … `第十二版方法.md`（第十二版为 SAT+MCF 两阶段，已不再由 `test_ILP` 目标链接）。

## 工作流程中必须要做的事情

- 改代码前先读清 `第十三版方法.md`、硬件映射和现有实现，不要凭记忆改模型。方法文档在 `problem_formulation/`；硬件映射在 `source/hardware` 与 `source/circuit`。不允许修改方法文档。
- 修改后评估是否同步更新本文件（不超过 200 行），以及是否在项目根目录 `.plan/` 下新增改动记录。本文件描述当前工程状态，不记录单次修改流水账。
- 100 行以上的修改完成后，应启动子 agent 独立评估实现是否完整、正确。
- 允许改动范围：优先修改 `algorithm/test_ILP/` 内部；除非必要，不改 `source/` 主流程接口语义。
- 关键步骤应打日志（`debug::info` / `debug::info_fmt`），便于从 `output/debug.log` 追踪。

## 目录结构

头文件以 `algorithm/test_ILP` 为 include 根，例如 `#include "common/routing_types.hh"`。

```text
algorithm/test_ILP/
├── main.cc                      # CLI：读 config → solve_unified_sat
├── common/                      # Bump_coord、RoutingNet、SatRoutingResult、hw_map
├── scope/                       # build_routing_nets、scope_bbox
├── graph/                       # unified_routing_graph（track + TOB 子图）
├── sat/                         # sat_cnf、constraint_kits、unified_sat_encoder、solve/extract
├── sat_allocation/              # cadical_solver（通用 CNF 求解封装）
├── problem_formulation/         # 方法定义文档
│
│  # 以下为第十二版遗留，未编入 test_ILP 目标，仅供 wirelength_study 等对照：
├── mcf/                         # BusMCF / SimpleMCF（legacy）
├── precompute/                  # path precompute、bbox（legacy）
├── ilp_allocation/              # TOB ILP / Gurobi（legacy）
└── visualization/               # MCF 资源可视化脚本（legacy）
```

## 关键文件与职责

- `main.cc`
  - 解析 `config_path`、`-v`/`-vv`、`--sat-log`。
  - 调用 `parse::read_config`、`algo::build_nets`、`solve_unified_sat`。
  - 打印峰值 RSS 与总耗时。

- `common/routing_types.hh`
  - `RoutingNet`（`net_id`、`kind`、去重后的 `sources`、`demands`、`scope_bbox`、`is_sync_bus` 及原始 net 元数据）。
  - `RoutingDemand`（`demand_id`、`sink`、`candidate_source_indices`、`fixed_pair`）；`fixed_pair=true` 表示保留原始 2-pin 配对，`false` 表示 sink 可从候选 source 中选择。
  - `sinks` 仅为其他旧调用方的兼容视图；当前 SAT encoder 直接读取 `demands`。
  - `GraphNodeRef`（track / bump / hline / vline 端点引用）。
  - `SatRoutingResult`、`SourceSinkPairPath`。

- `common/ilp_types.hh`
  - `Bump_coord`、`map_track()`、`kSyncBusOriginPrefix`。

- `scope/build_routing_nets.cc`
  - 每个原始 net 只生成一个 `RoutingNet`；多端口 net 不在这里拆成多个 SAT net。
  - `TracksToBumpsNet`：全部 begin track 为去重 source；每个 bump 一个非固定 demand，候选为全部 source。
  - `TrackToBumpsNet`：一个 track source；每个 bump 一个固定 demand。
  - `SyncNet`：每个 2-pin member 保留一个固定 demand；`btt`/`ttb` 均规范为 **track source、bump sink**，混合 Bnet/Tnet member 时拒绝。
  - `BumpToBumpNet`、`BumpToTrackNet`、`TrackToBumpNet` 各生成一个固定 demand。
  - `BumpToBumpsNet`、`BumpToTracksNet` 不支持，并在错误中报告类型和 net 名。

- `scope/scope_bbox.cc`
  - bbox 子项仅用于几何 scope：SyncNet 每个 member 一个子项；TrackToBumpsNet 每个 sink 一个 Tnet 子项；普通 2-pin net 一个子项。
  - TracksToBumpsNet 每个 source 一个 PN 子项；该子项先合并此 source 到各 bump 的 Tnet bbox，再由所有 source 子项取 RectHull。
  - 原始 `RoutingNet.scope_bbox` 为全部子项的 RectHull，最后并入所有 source/sink 端点 COB；SAT 仍求解原始 net。

- `graph/unified_routing_graph.cc`
  - 构建固定的完整硬件图：track 级 mesh（Wilton 开关）与全部 16 个 TOB 的 bump/hline/vline 子图；弧属性使用带 `-1` 哨兵的 `mode_group_id`、`physical_switch_id` 及物理开关类型。

- `sat/unified_sat_encoder.cc`
  - 每个原始 net 一个紧凑 scope；每个逻辑 `(net, source)` 的 `P`；仅为 demand 明确列出的候选 source 分配 pair `p`/`x`。
  - 使用全局节点/弧 ID 的扁平 offset 表，不创建变量名或 pair 级嵌套 map；逻辑 source 节点和候选 pair 预先索引，多源争用节点使用线性规模 Sinz `P` 互斥。
  - 连通性、流入/流出 AtMostOne；固定分配全部 1024 个全局 `M_g`，并聚合物理开关 `Y`（四类 TOB partial matching）；SyncBus 二进制距离等长与去环。

- `sat/sat_solution_extract.cc`
  - 从 live CaDiCal 数值赋值沿每个 active demand pair 的 `x` 提取完整 `SourceSinkPairPath`，同时提取全部 `M` 与已用 `Y`。

- `sat_allocation/cadical_solver.cc`
  - `CadicalSession`：边编码边写 CaDiCal；启用内存上限时按至多 4096 次全局编码操作采样，超长子句每 4096 个 literal 额外采样；`--sat-log` 时写入 `cadical-log/`。`solve_sat_cnf` 只为旧单元测试保留。

## 构建、运行、测试

在项目根目录（仅需 CaDiCal，**不需要 Gurobi**）：

```bash
xmake f --cadical=y
xmake build test_ILP
./output/test_ILP <config_path>
./output/test_ILP <config_path> -v
./output/test_ILP <config_path> --sat-log
```

CLI 参数：

- `-v` / `-vv`：`-v` 打印每条 `RoutingNet` 的 kind、scope、源汇数量；编码/求解阶段日志更详细。
- `--sat-log`：CaDiCal 轨迹写入 `cadical-log/`。

典型日志阶段：

```text
unified graph: nodes=... arcs=... track_nodes=... tob_nodes=...
streaming unified numeric SAT model into CaDiCal...
unified numeric SAT model: scopes=... pairs=... sources=... vars=... clauses=...
unified SAT model built: vars=... clauses=...
solving unified SAT with CaDiCal...
unified SAT ok: paths=... vars=... clauses=... ms=...
Process peak RSS: ... MB
```

最小验证建议：

```bash
xmake build test_ILP
./output/test_ILP test/config/case7 -v
./output/test_ILP test/config/case8 -v
./output/test_ILP test/config/case9 -v
```

## 已知限制与排错

- **规模**：数值流式编码已移除命名 CNF 的整份内存副本，但 PNnet 多 demand × 大 scope 仍会为每个候选 pair 分配独立 `p`/`x`；SyncBus 还会按 `O((|V|+|A|) log |V|)` 生成二进制距离约束。
- **UNSAT**：当前无 scope 扩边重试、无 tier；UNSAT 直接失败退出。
- **external port 坐标错误**：若出现 `is not a valid external port coord`，可检查 `source/hardware/interposer.hh` 中 `COB_ARRAY_WIDTH`（常见为 12 或 13）。

## 工程风格

- 小步、局部、可解释：每个改动应对应方法文档条目或明确 bug/需求。
- 不做无关重构；不顺手改 `source/` 主流程。
- `net_id` 由 `build_routing_nets` 顺序分配，编码与解提取均依赖其对齐。
- Tnet / PNnet 端点语义：**track = source，bump = sink**（含 SyncNet 内 `btt`）。
- 新日志应含 `net_id`、`net` 名、kind、scope、`pairs`/`vars`/`clauses` 等定位信息。
- 若恢复第十二版 SAT+MCF 能力，应作为独立 xmake 目标或显式开关，避免与第十三版流水线混用。
