# PR_tool / algorithm/test_ILP 工程指南

本文件是 `algorithm/test_ILP/` 子工程的入口说明。该目录用于验证和迭代“SAT TOB 分配 + MCF 全局布线”方法，不直接替代 `source/algo/router/` 的正式路由流程。

## 项目总体介绍

`test_ILP` 是一个独立的算法实验入口，核心流程包括：

- TOB 阶段：用 CaDiCal SAT 求解 bump/track 到 TOB 资源的可行分配。
- 路径预计算阶段：为每个 record 的 `(end_track, start_track)` 预计算同 COBUnit 内受限最短路、path bbox 和 path-length 分层。
- MCF 阶段：可选启用 Gurobi，在 track 级图上求解 BusMCF 和 SimpleMCF。

该子工程的硬件和电路基础来自项目根目录下的 `source/hardware` 与 `source/circuit`。算法依据优先参考 `algorithm/test_ILP/problem_formulation/` 中的方法文档，尤其是当前实现对应的“第七版方法与分析（SAT1）”。

## 工作流程中必须要做的事情

- 改代码前先读清相关方法文档、硬件映射和现有实现，不要凭记忆改模型。其中方法文档在`problem_formulation/`，硬件映射在"PR_tool根目录/source/hardware"和"PR_tool根目录/source/circuit"中。不允许修改方法文档。
- 修改后，需要评估是否同步更新本文件（不超过200行），以及是否在项目根目录 `.plan/` 下新增或更新改动记录。注意，本文件不应该单纯记录某次修改，而是基于项目内容来写
- 100行以上的修改完成后，必须启动一个新的子agent，让子agent独立评估修改的内容是否完整、正确
- 允许改动的范围：优先修改 `algorithm/test_ILP/` 内部文件，除非确实需要，不改 `source/` 主流程接口语义。
- 代码的关键行为要添加日志信息，在日志文件中打印展示，便于理解软件内部执行的重要步骤

## 目录结构

头文件引用以 `algorithm/test_ILP` 为 include 根目录，例如 `#include "common/ilp_types.hh"`。

```text
algorithm/test_ILP/
├── main.cc                 # CLI、build_records、阶段调度
├── common/                 # record、SAT/MCF 结果、path tier 状态等共用类型
├── sat_allocation/         # TOB SAT 编码、CaDiCal 封装、SAT-only 与 SAT+MCF pipeline
├── precompute/             # bbox、path precompute、path-length tier 放开
├── mcf/                    # track 级 BusMCF / SimpleMCF、bbox 可行图、硬件映射
├── ilp_allocation/         # legacy TOB ILP、MPS 导出
├── visualization/          # MCF 资源使用可视化辅助脚本
├── case1/、case2/           # 历史样例
└── problem_formulation/    # 方法定义与分析文档
```

## 关键文件与职责

- `main.cc`
  - 解析 CLI 参数，读取 config，调用 `algo::build_nets`。
  - `build_records()` 将电路 net 展平为 2-pin 粒度 `Net_cost_record`，分配 `record_id` 与 `bit_id`。
  - 调度 SAT-only、SAT+MCF、legacy MPS 导出、结果日志输出。

- `common/ilp_types.hh`
  - 定义 `Net_cost_record`、`Net_type`、端点类型、reach step、`map_track()` 等 SAT/MCF 共用数据。
  - `record_id` 是跨 SAT、MCF 对齐数据的主键，不能随意重排或复用。

- `common/tob_allocation_types.hh` 与 `common/tob_bbox_expansion.hh`
  - `TobIlpResult` 保存 SAT 分配结果、track endpoint、`tier_by_record`。
  - `TobTierState` 保存 per-record path-length tier，用于逐层放开 start track 候选。

- `sat_allocation/`
  - `tob_sat_encoder.*` 负责 W/S/QS/QW/Y/A 等 CNF 编码。
  - `solve_tob_sat.*` 负责单轮 SAT 与 SAT-only 局部 bbox retry。
  - `solve_tob_mcf_pipeline.*` 负责 SAT+MCF 外层局部 bbox retry。
  - `tob_allocation_result.*` 将 SAT 赋值转换成 `TobIlpResult`。

- `precompute/`
  - `ilp_bounding_box.*` 计算 record 的 bbox。
  - `tob_path_precompute.*` 预计算 `(end_track, start_track)` 受限最短路与路径 bbox，按 **path length** 分层缓存。
  - `tob_reach_with_range.*` 根据 per-record **tier** 放开 path length 层级，生成 SAT `starttrack_by_endtrack`。

- `mcf/`
  - `cob_mcf_router.*` 构建 track 级全局图，准备 commodity，求解 BusMCF 与 SimpleMCF。
  - PNnet（`TracksToBumpsNet`）在 SAT 选定的物理 `end_track` 上终止；SimpleMCF 不再创建 per-unit 虚拟 P/N hub。
  - `mcf_bbox.*` 根据 SAT 选定路径的 bbox 构造 MCF 可行图；对 track 节点施加 H/V 边界修剪（§第七版 6.2）。PNnet 与 TTB 相同：单 child 用 per-record path bbox，多扇出 origin 用 child bbox 的 RectHull；PNnet `snk` 为 SAT 选定的物理 `end_track` 节点。
  - `mcf_hw_map.hh` 封装 TOB/COB/track 坐标映射。

- `ilp_allocation/`
  - `tob_ilp_model.*` 与 `gurobi.*` 保留 legacy TOB ILP 求解（`wirelength_study`）与 `--export-ilp-mps` 对照能力。

## 构建、运行、测试方法

在项目根目录运行：

```bash
xmake build test_ILP
```

常用运行方式：

```bash
./output/test_ILP <config_path>
./output/test_ILP <config_path> -v
./output/test_ILP <config_path> --enable-mcf-routing --enable-mcf-obj
./output/test_ILP <config_path> --enable-mcf-routing --enable-pre-routing
./output/test_ILP <config_path> --enable-presat-parallel
./output/test_ILP <config_path> --enable-presat-parallel --enable-mcf-routing --enable-pre-routing
./output/test_ILP <config_path> --export-ilp-mps <path>
```

常用参数：

- `-v` / `-vv`：增加日志详细程度；`-v` 会打印路径预计算结果（每个 `end_track` 的可达 `start_tracks`、`length_layer`、`path_len` 及路径 bbox 四角坐标），以及每轮 SAT 当前 tier（按 path length 分层）下激活的 start track 子集。
- `--enable-presat-parallel`：并行执行 SAT 前路径预计算（按 `(record, end_track)` 分块，只读 `Interposer`）。
- 路径预计算会输出 `path precompute progress: [####------] N% (done/total)` 进度条日志（串行/并行均支持）。
- `--enable-mcf-routing`：SAT 成功后继续执行 MCF。
- `--enable-mcf-obj`：SimpleMCF 使用 `min Σ x` 目标；不加时 SimpleMCF 只做可行性求解。与 `--enable-pre-routing` 同时开启时，对 warm start 成功路径上的 `x^H_e` 使用 `kSimpleMcfWarmStartUsedEdgeCost`（0.95，见 `cob_mcf_router.cc`）软加权，其余 `x` 为 1.0，用于软破坏对称性。
- `--enable-pre-routing`：为 **MCF** Gurobi 提供 warm start 初值（`cob_mcf_router` 内 MCF 图 BFS），不改变硬约束。BusMCF warm start 在 Bus 求解前执行；SimpleMCF warm start 在 Bus 为 `Optimal`/`Suboptimal`/`Skipped` 时执行，按 COBUnit 以 Bus 实际占用初始化后再 BFS。多扇出 origin（`TrackToBumpsNet` / `TracksToBumpsNet`）采用增量 frontier：TTB 以共享 snk 为 hub、按 `end_bumps()` 顺序；PNnet 以本 unit 的 `vp`/`vn` 为 hub、按 record 顺序；部分 child 失败时成功的仍写入 warm start。与 TOB 阶段无关。
- `--show-resource-usage`：自动开启 `--enable-pre-routing`；按 COBUnit 增量写入 `resource-usage/unit{N}.txt`（含 `pre-route` 与 `post-solve` 两段）；`debug.log` 仅写索引行，不输出资源块。要求 `--enable-mcf-routing`。Bus 失败时不写任何 unit 文件；失败/Skipped unit 写空文件。
- `--disable-01-mcf`：跳过顶层 `TracksToBumpsNet`，即不生成 Pnet/Nnet records；SyncNet 内部拆分不受影响。
- `--disable-multipin-io`：跳过顶层 `TrackToBumpsNet`，即不生成对应多扇出 IO split records。
- `--disable-2pin-io`：跳过顶层 `TrackToBumpNet` 与 `BumpToTrackNet`；SyncNet 内部 btt/ttb 不受影响。
- `--sat-log`：输出 SAT 求解日志。
- MCF Gurobi 日志（第十版修改0）：`--enable-mcf-routing` 时自动写入 `gurobi-log/`：`bus.log`、`simple-unit{N}.log`（0–15）、`prm/{stage}_solve{K}.prm`；每次 Gurobi 调用追加一段（含 `timestamp`、`solve_id`、`tier`、`sat_tier_attempt`、`bbox_attempt`、`warm_start`、`retry_kind`）；失败尝试保留；未进 Gurobi 的失败写 stub；Skipped/empty 写 skipped 段。每次 test_ILP 运行清空 `gurobi-log/`。`modelinfo.log` 仍为矩阵诊断（rows/cols/nnz/heavy rows），与上述文件分工不变。

MCF 计时日志：

- `timing phase=mcf_bus_solve ms=...`：单轮 BusMCF 求解时间（累计 `solve_ms`，不含 breakdown）。
- `timing phase=simple_mcf_unitN_solve ms=...`：单轮 SimpleMCF unit N 求解时间（累计 `solve_ms`）。
- `timing phase=mcf_bus_solve_total ms=...` 与 `simple_mcf_unitN_solve_total`：SAT+MCF retry 全部尝试轮的累计时间。
- BusMCF / SimpleMCF **每次 stage 尝试**结束日志含 `model_status=`、`solution_class=` 及耗时细分（第十版修改1）；warm-start 重试、bbox expand 每轮各打一行。汇总行（`timing phase=*`、`SimpleMCF unit N: ok=...`）仍只有 `solve_ms`。
- `solve_ms`：该次 stage 调用的 wall-clock 总耗时，为汇总权威值；与下列五段之和可能差几毫秒。
- `model_build_ms`：C++ 约束/变量组装 + Gurobi `addVar`/`addConstr`/`model.update` + MIP start 赋值。
- `matrix_diag_ms`：约束矩阵稀疏度诊断（写入 `modelinfo.log`）。
- `gurobi_optimize_ms`：Gurobi 日志配置、`optimize()`、读取 incumbent 解（`ObjVal`/`X`）。
- `compute_iis_ms`：不可行时 `computeIIS()` 与 IIS 行号解析（否则为 0）。
- `extract_path_ms`：解提取（`used_edges`/`used_nodes` + `append_paths_from_f_solution`）；失败/早退/skipped 为 0。
- BusMCF / SimpleMCF 阶段结束日志含 `model_status=` 与 `solution_class=`（`Optimal`/`Suboptimal`/`TimeLimit`/`Failed`/`Skipped`）；`ok=true` 当且仅当 class 为 `Optimal`、`Suboptimal` 或 `Skipped`。`Suboptimal` 与 `Optimal` 均提取 Gurobi 解。warm start 导致 `Suboptimal` 直接接受不重试；warm start 导致 `Failed`/`TimeLimit` 时无 warm start 重试 Gurobi 一次。

MCF 失败重试（第九版修改4，内层 bbox 扩边）：

- Gurobi 在无 warm start 重试后仍 `Failed`/`TimeLimit` 时，先在 **MCF 阶段**扩大失败对象 bbox（四向 ±1，clamp 到全 COB 阵列），再重跑 warm start（若 `--enable-pre-routing`）+ Gurobi；日志含 `MCF bbox expand:`。
- **BusMCF**：扩 `per_bus_key` hull；多 bus 同时扩；无法定位 `bus_key` 时扩全部 bus；任一失败 bus 已到全阵列则 Bus 阶段彻底失败，**跳过 SimpleMCF**。
- **SimpleMCF**：按失败 origin group 的 RectHull overlay 扩边（`(unit, origin_key)`）；无法按 origin 定位时扩该 unit 内全部 origin group；任一失败 group 无法扩则 unit 彻底失败。
- 内层 bbox 耗尽后 `all_ok=false`，pipeline 再 `tier++` 扩 `start_track`（日志 `MCF bbox expand exhausted` → `tier iteration: MCF expand fail_set=`）。
- 串行 SimpleMCF（默认）：某 unit bbox 耗尽后后续 unit 标 `Skipped`；`--enable-mcf-parallel` 时各 unit 独立扩边互不影响。

SimpleMCF LP 松弛强化（第十版修改 2.1–2.3，仅 SimpleMCF）：

- **2.1 `f_le_x_lower`**：由逐有向弧 `f<=x` 改为 per-commodity 无向边 `f^n_ij + f^n_ji <= x^H_e`；`f_le_x_upper` 不变。
- **2.2 `o_endpoint_eq`**：每个 origin group 的物理 `src`/全部物理 `snk` 加等式 `o^H=1`。
- **2.3 Bus 残余过滤**：建模前按 Bus 占用过滤弧/边；`edge_capacity` 仅对实际出现 `x` 的边 lazy 创建；当前 origin group 的物理 endpoint `node residual=0` 早退 `endpoint_residual_zero`；transit 节点 residual=0 过滤穿越弧。不同 origin 不能通过彼此 endpoint 绕过 Bus residual node 过滤；`residual_disconnected` / `endpoint_no_o_var` 与 `bbox_disconnected` 同类失败（`Failed` + origin retry hint + gurobi stub）。

最小验证建议：

```bash
./output/test_ILP test/config/case7 --enable-mcf-routing  --enable-pre-routing
./output/test_ILP test/config/case8 --enable-mcf-routing  --enable-pre-routing
./output/test_ILP test/config/case9 --enable-mcf-routing  --enable-pre-routing
```

第九版修改3：上述命令日志应含 `solution_class=`。warm start 导致 `Failed`/`TimeLimit` 时可能出现 `warm start led to Failed; retrying without warm start`（仅一次）。最后一行验证对称性软破坏（日志应含 `objective symmetry-break`）。

`algorithm/test_ILP/visualization/` 从 `resource-usage/unitN.txt` 解析资源使用并绘图。`matlab_main.m` 中 `resource_phase` 可选 `post-solve`（默认）或 `pre-route`（需 `--show-resource-usage`）；`visualize_cob_unit_usage(..., 'Phase', ...)` 同理。

一个排错方法：

如果出现类似`Build system >> Add external ports >> { row: 7, col: 13, dir: PR_tool::hardware::TrackDirection::Horizontal, index: 45 } is not a valid external port coord!`的错误，可以去修改source/hardware/interposer.hh: COB_ARRAY_WIDTH 这个参数，要么是12，要么是13

## 项目工程风格

- 小步、局部、可解释：每个改动都应能对应到方法文档、bug 或用户明确需求。
- 不做无关重构；不要顺手改格式、命名或主工程接口。
- `records.size()`、`assignments.size()`、`record_track_endpoints.size()` 必须保持一致。
- `record_id` 全局唯一，由 `build_records()` 输出顺序分配，SAT/MCF 都依赖它对齐。
- `tier_by_record[record_index]` 是局部 path-length 层级；`max_tier` 只作为全局摘要，不应作为 MCF 真实范围来源。
- 默认日志不打印完整 tier 数组；需要定位局部扩展时优先看 `max_tier`、`changed_records`、`fail_set`。
- SAT UNSAT 当前不做 UNSAT core 归因；按 pipeline 规则扩展相关 `Tnet/PNnet`。
- BusMCF 内层 bbox 耗尽后 tier++ 扩展失败 `bus_key` 的 member records；无法定位 `bus_key` 时 MCF 内层先扩全部 bus，耗尽后再 tier++。
- SimpleMCF 失败按失败 unit 的 simple records 扩展；多扇出 origin 要扩展同 origin 的所有 child records。
- PNnet 在 SimpleMCF 中与 TTB 使用相同 path bbox 裁剪（单 child：`SimpleCommodity`；多扇出：`SimpleOriginGroup` + RectHull）；`tier` 仍主要影响 SAT 候选 start tracks。
- track 只能在所属 COBUnit 内连通；path precompute 不允许跨 COBUnit 搜索。
- MCF bbox 使用 SAT 选中 path 的 bbox；当前 commodity 的 `src/snk` endpoint node 及其 bbox 内 endpoint 接入边可做局部豁免，避免边界修剪切断 SAT 固定端点。
- 新日志要包含足够定位信息，例如 `record_id`、`origin_key`、`bit_id`、`tier`、`bus_key`、unit、bbox。
- 如果新增 CLI 参数，必须同步更新本文件的运行说明；如果只是内部策略变化，优先保持 CLI 不变。
