# PR_tool / algorithm/test_ILP 工程指南

本文件是 `algorithm/test_ILP/` 子工程的入口说明。该目录用于验证和迭代“SAT TOB 分配 + MCF 全局布线”方法，不直接替代 `source/algo/router/` 的正式路由流程。

## 项目总体介绍

`test_ILP` 是一个独立的算法实验入口，核心流程包括：

- TOB 阶段：用 CaDiCal SAT 求解 bump/track 到 TOB 资源的可行分配。
- 路径预计算阶段：为每个 record 的 `(end_track, start_track)` 预计算同 COBUnit 内受限最短路、path bbox 和 path-length 分层。
- MCF 阶段：可选启用 Gurobi，在 track 级图上求解 BusMCF 和 SimpleMCF。
- 诊断阶段：可选用 maze-check 或 simple-maze 判断 MCF 失败是否来自建模/容量限制。

该子工程的硬件和电路基础来自项目根目录下的 `source/hardware` 与 `source/circuit`。算法依据优先参考 `algorithm/test_ILP/problem_formulation/` 中的方法文档，尤其是当前实现对应的“第七版方法与分析（SAT1）”。

## 工作流程中必须要做的事情

- 改代码前先读清相关方法文档、硬件映射和现有实现，不要凭记忆改模型。其中方法文档在`problem_formulation/`，硬件映射在"PR_tool根目录/source/hardware"和"PR_tool根目录/source/circuit"中。不允许修改方法文档。
- 中到大规模修改后，需要评估是否同步更新本文件（不超过200行），以及是否在项目根目录 `.plan/` 下新增或更新改动记录。同时启动一个新的子agent，让子agent评估需要修改的内容是否完整、正确的完成
- 允许改动的范围：优先修改 `algorithm/test_ILP/` 内部文件，除非确实需要，不改 `source/` 主流程接口语义。
- 关键行为要能够在日志文件中打印展示，便于理解软件内部执行的重要步骤

## 目录结构

头文件引用以 `algorithm/test_ILP` 为 include 根目录，例如 `#include "common/ilp_types.hh"`。

```text
algorithm/test_ILP/
├── main.cc                 # CLI、build_records、阶段调度
├── common/                 # record、SAT/MCF 结果、path tier 状态等共用类型
├── sat_allocation/         # TOB SAT 编码、CaDiCal 封装、SAT-only 与 SAT+MCF pipeline
├── precompute/             # bbox、path precompute、reach、legacy ILP reach 预计算
├── mcf/                    # track 级 BusMCF / SimpleMCF、bbox 可行图、硬件映射
├── maze_check/             # MCF 失败后的 maze 诊断与 simple-maze
├── ilp_allocation/         # legacy TOB ILP、MPS 导出、Interposer apply
├── visualization/          # MCF 资源使用可视化辅助脚本
├── case1/、case2/           # 历史样例
└── problem_formulation/    # 方法定义与分析文档
```

## 关键文件与职责

- `main.cc`
  - 解析 CLI 参数，读取 config，调用 `algo::build_nets`。
  - `build_records()` 将电路 net 展平为 2-pin 粒度 `Net_cost_record`，分配 `record_id` 与 `bit_id`。
  - 调度 SAT-only、SAT+MCF、legacy MPS 导出、maze-check、结果日志输出。

- `common/ilp_types.hh`
  - 定义 `Net_cost_record`、`Net_type`、端点类型、reach step、`map_track()` 等 SAT/MCF 共用数据。
  - `record_id` 是跨 SAT、MCF、maze-check 对齐数据的主键，不能随意重排或复用。

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
  - `mcf_bbox.*` 根据 SAT 选定路径的 bbox 构造 MCF 可行图；对 track 节点施加 H/V 边界修剪（§第七版 6.2）。
  - `mcf_hw_map.hh` 封装 TOB/COB/track 坐标映射。

- `maze_check/`
  - `maze_check.*` 做 MCF 失败后的连通性诊断。
  - `maze_route_ilp_fixed.*` 复用 SAT 固定端点做 maze。
  - `simple_maze_routing.*` 用 simple-maze 替代 SimpleMCF Gurobi。

- `ilp_allocation/`
  - `tob_ilp_model.*` 与 `gurobi.*` 保留 legacy TOB ILP/MPS 对照能力。
  - `ilp_apply_interposer.*` 将 SAT/ILP 分配应用到 `Interposer`。
  - `ilp_speedup.*` 提供 `cobunit_to_tracks()`、`track_to_jk()` 等辅助映射。

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
./output/test_ILP <config_path> --enable-mcf-routing --enable-mcf-obj --maze-check-mcf
./output/test_ILP <config_path> --enable-mcf-routing --simple-maze --disable-01-mcf
./output/test_ILP <config_path> --export-ilp-mps <path>
```

常用参数：

- `-v` / `-vv`：增加日志详细程度；`-v` 会打印路径预计算结果（每个 `end_track` 的可达 `start_tracks`、`length_layer`、`path_len` 及路径 bbox 四角坐标），以及每轮 SAT 当前 tier（按 path length 分层）下激活的 start track 子集。
- `--enable-presat-parallel`：并行执行 SAT 前路径预计算（按 `(record, end_track)` 分块，只读 `Interposer`）。
- 路径预计算会输出 `path precompute progress: [####------] N% (done/total)` 进度条日志（串行/并行均支持）。
- `--enable-mcf-routing`：SAT 成功后继续执行 MCF。
- `--enable-mcf-obj`：SimpleMCF 使用 `min Σ x` 目标；不加时 SimpleMCF 只做可行性求解。
- `--enable-pre-routing`：仅影响 MCF warm start，不改变硬约束。
- `--simple-maze`：BusMCF 仍用 Gurobi，SimpleMCF 改为 SAT 固定端点下的 maze。
- `--disable-01-mcf`：跳过顶层 `TracksToBumpsNet`，即不生成 Pnet/Nnet records；SyncNet 内部拆分不受影响。
- `--disable-multipin-io`：跳过顶层 `TrackToBumpsNet`，即不生成对应多扇出 IO split records。
- `--disable-2pin-io`：跳过顶层 `TrackToBumpNet` 与 `BumpToTrackNet`；SyncNet 内部 btt/ttb 不受影响。
- `--maze-check-mcf` / `--maze-check-ilp-mcf`：MCF 后追加失败诊断，不改变 MCF 求解逻辑。
- `--sat-log` / `--gurobi-log`：分别输出 SAT trace 与 MCF Gurobi 日志。

MCF 计时日志：

- `timing phase=mcf_bus_solve ms=...`：单轮 BusMCF 求解时间。
- `timing phase=simple_mcf_unitN_solve ms=...`：单轮 SimpleMCF unit N 求解时间；simple-maze 模式下为 0。
- `timing phase=mcf_bus_solve_total ms=...` 与 `simple_mcf_unitN_solve_total`：SAT+MCF retry 全部尝试轮的累计时间。

TOB SAT最小验证建议：

```bash
xmake build test_ILP
./output/test_ILP test/config/case7
./output/test_ILP test/config/case8
./output/test_ILP test/config/case9
```

涉及 MCF 的改动建议额外运行：

```bash
./output/test_ILP test/config/case7 --enable-mcf-routing  --enable-pre-routing
./output/test_ILP test/config/case8 --enable-mcf-routing  --enable-pre-routing
./output/test_ILP test/config/case9 --enable-mcf-routing  --enable-pre-routing
```

## 项目工程风格

- 小步、局部、可解释：每个改动都应能对应到方法文档、bug 或用户明确需求。
- 不做无关重构；不要顺手改格式、命名或主工程接口。
- `records.size()`、`assignments.size()`、`record_track_endpoints.size()` 必须保持一致。
- `record_id` 全局唯一，由 `build_records()` 输出顺序分配，SAT/MCF/maze-check 都依赖它对齐。
- `tier_by_record[record_index]` 是局部 path-length 层级；`max_tier` 只作为全局摘要，不应作为 MCF 真实范围来源。
- 默认日志不打印完整 tier 数组；需要定位局部扩展时优先看 `max_tier`、`changed_records`、`fail_set`。
- SAT UNSAT 当前不做 UNSAT core 归因；按 pipeline 规则扩展相关 `Tnet/PNnet`。
- BusMCF 失败只扩展可定位 `bus_key` 的 member records；无法定位时应失败并记录原因。
- SimpleMCF 或 simple-maze 失败按失败 unit 的 simple records 扩展；多扇出 origin 要扩展同 origin 的所有 child records。
- PNnet 在 MCF 中继续不裁剪；它的 tier 只影响 SAT 阶段候选 start tracks。
- track 只能在所属 COBUnit 内连通；path precompute 不允许跨 COBUnit 搜索。
- MCF bbox 使用 SAT 选中 path 的 bbox；当前 commodity 的 `src/snk` endpoint node 及其 bbox 内 endpoint 接入边可做局部豁免，避免边界修剪切断 SAT 固定端点。
- 新日志要包含足够定位信息，例如 `record_id`、`origin_key`、`bit_id`、`tier`、`bus_key`、unit、bbox。
- 如果新增 CLI 参数，必须同步更新本文件的运行说明；如果只是内部策略变化，优先保持 CLI 不变。
