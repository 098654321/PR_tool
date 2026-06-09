# PR_tool /algorithm/test_ILP 工程指南（面向 AI Agent）

本文件是 `algorithm/test_ILP/` 子工程的入口说明。目标是：理解并扩展"**SAT TOB 分配 + MCF 全局布线**"实验链路。

该目录是一个独立的算法验证入口，不直接替代 `source/algo/router/` 的正式路由流程。它强调：

- TOB 阶段用 CaDiCal SAT 求可行 track 分配（第六版 SAT1）
- MCF 阶段用 Gurobi 求解 BusMCF / SimpleMCF（第五版建模 + 第六版 bbox 可行图裁剪）
- 在 `test_ILP` 范围内隔离实验逻辑，避免污染主流程

### 工作流程中一些必须要做的事情

- 在完成一次任务后，需要评估是否需要修改本文件，以及是否需要修改/添加项目根目录中.plan目录下的改动记录文件
- 我给你的资料必须深入理解，在理解的前提下进行动作
- 基于algorithm/test_ILP/problem_formulation中文件的中、大幅改动，在改动结束后必须新开一个子agent，让子agent独立评估改动是否完整且正确的完成了

### 目录结构（C++ 源码）

头文件引用统一以 `algorithm/test_ILP` 为 include 根目录，使用子目录前缀（例如 `#include "common/ilp_types.hh"`）。

```
algorithm/test_ILP/
├── main.cc                 # CLI、build_records、阶段调度
├── common/
│   ├── ilp_types.hh        # Net_cost_record、MCF 共用类型
│   ├── tob_allocation_types.hh  # TobIlpResult、bbox_expand_by_record 等
│   └── tob_bbox_expansion.hh    # per-record ρ 状态与 expand_records()
├── sat_allocation/         # 阶段 A：TOB SAT 编码与求解
│   ├── tob_sat_encoder.{hh,cc}
│   ├── cadical_solver.{hh,cc}
│   ├── tob_allocation_result.{hh,cc}
│   ├── solve_tob_sat.{hh,cc}
│   └── solve_tob_mcf_pipeline.{hh,cc}  # SAT+MCF 局部 bbox retry 外层循环
├── ilp_allocation/         # TOB ILP 模型（仅 --export-ilp-mps）、apply、wirelength_study 遗留
│   ├── tob_ilp_model.{hh,cc}
│   ├── gurobi.{hh,cc}
│   ├── ilp_speedup.{hh,cc}
│   └── ilp_apply_interposer.{hh,cc}
├── precompute/
│   ├── ilp_reach_precompute.{hh,cc}
│   ├── ilp_bounding_box.{hh,cc}
│   ├── tob_reach_with_range.{hh,cc}
│   ├── tob_channel_kshortest.{hh,cc}
│   └── pre_routing_warm_start.{hh,cc}  # 已不被 main 调用（遗留）
├── mcf/                    # 阶段 B：track 级 BusMCF + SimpleMCF（含 bbox 可行图）
│   ├── cob_mcf_router.{hh,cc}
│   ├── mcf_bbox.{hh,cc}    # MCF bbox 上下文、弧判定、bus/origin RectHull
│   ├── mcf_graph.hh
│   └── mcf_hw_map.hh
├── maze_check/
│   ├── maze_check.{hh,cc}           # MCF 失败后的 maze 诊断（--maze-check-*）
│   ├── maze_route_ilp_fixed.{hh,cc} # ILP 固定 TOB 端点 maze（供 maze-check / simple-maze）
│   └── simple_maze_routing.{hh,cc}  # --simple-maze：按 origin net 替代 SimpleMCF Gurobi
├── test/
│   └── tob_bbox_expansion_state_test.cc  # TobBBoxExpansionState 单元测试（`test_ILP_unit`）
├── case1/、case2/
└── problem_formulation/
```

---

## 1. 快速上手（构建 / 运行）

`test_ILP` 目标由仓库根目录 `xmake.lua` 配置（含 `sat_allocation/*`、`precompute/ilp_bounding_box` 等；需 `xmake f --cadical=y`）。常用命令（仓库根目录）：

```bash
xmake build test_ILP
./output/test_ILP <config_path> [-v|-vv|...] [--export-ilp-mps <path>] [--enable-mcf-routing] [--disable-bus-mcf] [--enable-mcf-parallel] [--enable-mcf-obj] [--enable-pre-routing] [--sat-log] [--gurobi-log] [--simple-maze] [--maze-check-ilp-mcf | --maze-check-mcf] [--check-golden]
```

参数语义（以 `main.cc` 为准）：

- `config_path`：配置目录（例如 `algorithm/test_ILP/case1`）
- `-v` / `-vv` / …：设置 verbose 模式（`-v` 计数越多越详细），启用后将 debug level 设为 `Debug`；`-v` 时 SAT 阶段额外打印每条 record 的 bbox / end_track / start_tracks
- `--export-ilp-mps <path>`：可选，导出 legacy TOB ILP 的 MPS 文件（`tob_ilp_model`，与 SAT 求解解耦，用于对照/debug）
- `--sat-log`：为 CaDiCal 写出 API trace（`./cadical-log/sat_range{N}.trace`；`N` 为当轮 `max_rho` 兼容摘要）
- `--enable-mcf-routing`：在 SAT TOB 分配成功后继续执行 MCF 阶段
- `--disable-bus-mcf`：须与 `--enable-mcf-routing` 联用；跳过 BusMCF（不占用 Bus 边/节点残余），仅求解 SimpleMCF；SyncNet bus commodity 不会得到 MCF 路径
- `--enable-mcf-parallel`：SimpleMCF 按 COBUnit 并行求解（`std::async`，每个 unit 独立 Gurobi 模型）
- `--enable-mcf-obj`：与 `--enable-mcf-routing` 联用时，SimpleMCF 加入 `min Σ x` 目标函数；**省略时 SimpleMCF 为纯可行性求解**（所有变量成本为 0）。BusMCF 始终带 `min Σ f` 目标
- `--enable-pre-routing`：**仅 MCF 阶段** warm start。须与 `--enable-mcf-routing` 联用；在 `mcf/cob_mcf_router.cc` 的 `GlobalGraph` 上按 BusMCF/SimpleMCF 顺序跑 BFS maze，把路径转为 MCF 变量初值。单独指定时无效果并打 warning。失败的预布线只记录日志；若 Gurobi 使用 warm start 后未返回 optimal，会自动无 warm start 重试
- `--gurobi-log`：**仅 MCF 阶段**（BusMCF、各 SimpleMCF unit）。在与 `debug.log` 同目录下的 `gurobi-log/` 写出求解器日志（`./gurobi-log/gurobi_{stage}_{seq}.log`）。约束矩阵诊断写入 `./gurobi-log/modelinfo.log`（见 §5.5）
- `--simple-maze`：须与 `--enable-mcf-routing` 联用；与 `--maze-check-*` **互斥**。BusMCF 仍用 Gurobi；**SimpleMCF 改为 maze**：`reset_regs` → `apply_tob_ilp_result_to_interposer` → suspend BusMCF 路径后，按 **origin net**（`record_origin_group_uid`）顺序、SAT 固定 TOB 端点做 BFS maze（`maze_check/maze_route_ilp_fixed.{hh,cc}`）。**与 `--maze-check-mcf` 不同**：`TracksToBumpsNet`（PNnet）从 SAT 分配的 bump `start_track` 布到**任意可达 0/1 端口**（BFS 起点**不含** 0/1 端口，避免平凡路径）；`TrackToBumpsNet` 每条 split `Tnet` 从 bump `start_track` 布到共享 COB `end_track`（与 MCF commodity 方向一致）。日志：`simple-maze origin="..." result=OK|FAILED`（失败含 `failed_at_record_index`）；失败 record 另打 `simple-maze record_id=... result=FAILED`；`-v` 下每条 record 成功也打明细，origin 级 `path=` 用 `[rec=N] ... | ...` 分段。失败 origin 所在 SimpleMCF unit 会进入局部 bbox retry 集合。`--enable-mcf-parallel` / `--enable-mcf-obj` 无效果（warning）
- `--maze-check-ilp-mcf` / `--maze-check-mcf`：须与 `--enable-mcf-routing` 联用，**二者互斥**。MCF 结束后（即使 SimpleMCF 失败）在真实 `Interposer` 上先 `apply_tob_ilp_result_to_interposer`，再 `suspend` 已有 BusMCF + 成功 SimpleMCF 路径，对 **SimpleMCF 失败 unit** 中的 net 按 `origin_key` 去重做 maze 诊断：
  - `--maze-check-ilp-mcf`：调用主工程 `Net::route(MazeRouteStrategy)`（完整 maze，TOB track 可重选）
  - `--maze-check-mcf`：复用 SAT 已 apply 的 TOB 分配，对 origin_net 做 COB 段 BFS maze。一般 2-pin net 验证 SAT 固定起终点是否可达；**`TracksToBumpsNet`（PNnet）** 与 simple-maze 相同修复后的 `route_tracks_to_bumps_net_ilp_fixed`（bump start → 任意 0/1 端口，起点不含 0/1 端口）。失败 record 打 `maze-check-mcf record_id=... result=FAILED`
  - 日志末尾 `log_record_shared_results` 输出每个失败 MCF record 的 maze 结果（用于区分 MCF 建模问题与真实不可达）
- MCF 完成后（未启用 maze-check 时），`mcf/cob_mcf_router.cc` 会将已有路径 `suspend` 到 `Interposer`；成功时按 **`origin_key`（与 `build_nets()` 得到的逻辑 net 名一致）** 分组打印 track 级路径；仍保留按 COBUnit 的 commodity 摘要行便于对照容量

---

## 2. 总体数据流（两阶段）

入口：`algorithm/test_ILP/main.cc` 的 `run_main()`

1) `parse::read_config` + `algo::build_nets`  
2) `build_records()`：将 `circuit::Net` 展平为 2-pin 级 `Net_cost_record`，并分配 `record_id` 和 `bit_id`；`TrackToBumpsNet` 按 bump 拆成多条 `Tnet`（`from_track_to_bumps_split`）参与 TOB 分配，原 net 记入 `BuildRecordsResult::track_to_bumps_nets`；`BumpToBumpsNet` / `BumpToTracksNet` 为非法类型，直接报错退出  
3) 可选 `--export-ilp-mps`：`precompute_reach_for_records()` + `write_mps_file()` 导出 legacy ILP MPS  
4) TOB + 可选 MCF 使用 per-record 局部扩展状态 `rho[record_id] ∈ [0,4]`：  
   - **未启用** `--enable-mcf-routing`：`solve_tob_sat_with_cadical()` 在 SAT UNSAT 时只扩展 `Tnet/PNnet` records  
   - **启用** `--enable-mcf-routing`：`solve_tob_mcf_with_range_iteration()` 统一外层循环；每轮 `solve_tob_sat_with_bbox_state` → `run_mcf_global_routing_cob_units`。SAT UNSAT 扩展 `Tnet/PNnet`；BusMCF 失败扩展失败 `bus_key` 的 member records；SimpleMCF / `--simple-maze` 失败扩展失败 unit 的 simple records，并把相关多扇出 origin 的所有 child records 一并扩展  
5) MCF 细节：COB 网格固定为 `hardware::Interposer::COB_ARRAY_HEIGHT/WIDTH`；读取 SAT 结果中的 `bbox_expand_by_record`，经 `build_mcf_bbox_context()` 构造 bbox 可行图 $E_n^c$ / $E_H^c$ 做 BusMCF + SimpleMCF。`range_level` 字段仅保留为 `max(rho)` 兼容摘要，不作为真实 MCF 范围来源。`--enable-pre-routing` 时 warm start BFS 受 bbox 限制；pipeline 成功后在 `Interposer` 上 `suspend()`（maze-check 模式由 `maze_check` 负责 apply+suspend）

建议把该链路理解为：

- **阶段 A（SAT TOB）**：决定每条 2-pin net 的 `COBUnit` 与 bump track 分配
- **阶段 B（MCF）**：在 track 级全局图上按 commodity 做容量约束整数流路由（Gurobi）；Bnet/Tnet 与 bus / 多扇出 origin 受第六版 bbox 裁剪，PNnet 不裁剪

### 2.1 分阶段耗时（写入 `debug.log`）

| 日志前缀 | 含义 |
| --- | --- |
| `timing phase=tob_sat_solve` | SAT 阶段耗时；MCF 路径下为 pipeline 内**所有尝试轮** SAT 之和（含失败轮） |
| `timing phase=mcf_warm_start` | MCF warm path 构造；pipeline 下为**所有尝试轮**之和；未使用 `--enable-pre-routing` 时为 **0** |
| `timing phase=mcf_solve` | BusMCF + SimpleMCF Gurobi 求解；pipeline 下为**所有尝试轮**之和 |

`run_main` 退出前汇总：`timing breakdown (ms): tob_sat_solve=..., mcf_warm_start=..., mcf_solve=...`（未执行 MCF 时后两项为 **0**）。MCF 重试日志使用 `bbox iteration: attempt=A ... max_rho=R`，并打印 `changed_records` 与 `rho_by_record` 摘要。

---

## 3. 关键文件与职责

### 3.1 入口与数据预处理

- `algorithm/test_ILP/main.cc`
  - CLI 参数解析（含 verbose `-v` 计数）
  - 2-pin 记录构建（`build_records` / `BuildRecordsResult`）与类型拆分（`classify_net`）
  - 可选 `--export-ilp-mps` 调度
  - SAT TOB 求解（SAT-only：`solve_tob_sat_with_cadical`；SAT+MCF：`solve_tob_mcf_with_range_iteration`）与结果输出
  - 可选 MCF 阶段（含 `--enable-pre-routing` MCF warm start、maze-check）

- `algorithm/test_ILP/ilp_allocation/ilp_apply_interposer.hh/.cc`
  - `apply_tob_ilp_result_to_interposer`（S 配置 `hori_to_vert`、W 写 `allocated_track`/`intersect_access_unit`）

### 3.2 SAT TOB 分配（主路径）

- `algorithm/test_ILP/sat_allocation/solve_tob_sat.{hh,cc}`：单轮 SAT（`solve_tob_sat_with_bbox_state`；`solve_tob_sat_at_range_level` 仅为 uniform `rho` 兼容包装）；SAT-only 时按局部 `rho` 重试（`solve_tob_sat_with_cadical`）
- `algorithm/test_ILP/sat_allocation/solve_tob_mcf_pipeline.{hh,cc}`：SAT+MCF 统一局部 bbox retry 外层循环（per-record `rho[record_id]`）
- `algorithm/test_ILP/sat_allocation/tob_sat_encoder.{hh,cc}`：W/S/QS/QW/Y/A CNF（语义对齐第六版 §3–11）
- `algorithm/test_ILP/sat_allocation/cadical_solver.{hh,cc}`：CaDiCal 封装
- `algorithm/test_ILP/sat_allocation/tob_allocation_result.{hh,cc}`：SAT 赋值 → `TobIlpResult`
- `algorithm/test_ILP/precompute/tob_reach_with_range.{hh,cc}`、`ilp_bounding_box.{hh,cc}`、`tob_channel_kshortest.{hh,cc}`：可达性与 per-record bbox 扩展
- `algorithm/test_ILP/common/tob_bbox_expansion.hh`：`TobBBoxExpansionState`（`rho_by_record`、`expand_records`）
- `algorithm/test_ILP/common/tob_allocation_types.hh`：`TobIlpResult`（含 `bbox_expand_by_record`；`range_level` 仅为 `max(rho)` 摘要）等

### 3.3 类型与元数据

- `algorithm/test_ILP/common/ilp_types.hh`
  - `Bump_coord`、`Net_type`（`Bnet` / `Tnet` / `PNnet`）
  - `IlpPowerKind`（`None` / `Pose` / `Nege`）、`IlpEndpointKind`（`Bump` / `Track`）
  - `IlpReachStep`（Wilton 转弯步：`from_dir`、`to_dir`、`index_in`、`index_out`）
  - `Net_cost_record`（包含 ILP 与 MCF 共用字段）
  - `map_track()`（track -> cobunit 映射）

`Net_cost_record` 关键字段：

- `origin_key`：原始逻辑 net 名（`net->name()`），人类可读
- `origin_uid`：电路 net 的稳定 uid（`net->uid()`），用于 MCF Origin 分组、maze-check 聚合、`bit_id` 计数
- `record_id`：`build_records` 输出序中的全局唯一 id（用于 ILP/MCF 对齐）
- `bit_id`：同一 `origin_uid` 内的位序号（拆分出的子 record 共享父 uid 时递增）
- `power_kind`：`Pose / Nege / None`
- `mcf_start_kind / mcf_end_kind`
- `mcf_start_track / mcf_end_track`
- `mcf_has_start_track / mcf_has_end_track`
- `pn_end_tracks` / `pn_end_track_coord_by_index`：PNnet 的所有 0/1 端口 track
- `end_tracks`：可达性预计算后的 end track 列表
- `starttrack_by_endtrack`：每个 end_track 对应的可达 start_track 列表
- `reach_by_end_start`：每个 `(end_track, start_track)` 对对应的 Wilton 转弯步序列

### 3.4 Legacy TOB ILP 模型（`--export-ilp-mps` / `wirelength_study`）

- `algorithm/test_ILP/ilp_allocation/tob_ilp_model.hh/.cc`
  - `TobIlpModel`：行列式模型拼装
  - `build_tob_ilp_model()`：约束与目标构建
  - `linear_data()`：导出求解器无关的线性模型数据

ILP 变量体系：
- `W(bump, j, k)`：bump 到 (j, k) 的分配决策
- `S(tob, v)`：TOB 的 s 寄存器决策（`v = j*8 + k`）
- `QS(bump, j, k)` = W ∧ S（线性化乘积，对应 straight 路径）
- `QW(bump, j, k)` = W ∧ ¬S（线性化乘积，对应 wrap 路径）
- `Y(n, r_end)`：PNnet n 选择 end_track r_end 的决策

ILP 约束组：
- 约束 1（`R_WONE`）：每个 active bump 恰好选一个 (j, k)
- 约束 2（`R_HORI`）：同一 (TOB, Bank, Group) 内的水平线 j 至多被 1 个 Index 使用
- 约束 3（`R_VERT`）：同一 (TOB, Bank) 内的 (j, k) 组合至多被 1 个 Group 使用
- QS/QW 线性化约束（`R_QS1/2/3`、`R_QW1/2/3`）
- Bnet 可达性（`R_BEND0`、`R_BREACH`）：基于 `starttrack_by_endtrack` 预计算结果
- Tnet 可达性（`R_TREACH0`）：基于固定 end_track 的 start_track 可达集
- PNnet 端口选择（`R_PNYSUM`、`R_PNREACH`）：恰好选 1 个 end_track，且 start_track 必须在可达集内

- `algorithm/test_ILP/ilp_allocation/gurobi.hh/.cc`
  - `solve_tob_ilp_with_gurobi()`：**仅 `wirelength_study` 目标使用**；`test_ILP` 主路径已改 SAT

### 3.5 MCF 路由

- `algorithm/test_ILP/mcf/cob_mcf_router.hh/.cc`
  - 负责 track 级全局 MCF 的完整流水线
  - 入口函数 `run_mcf_global_routing_cob_units(..., hardware::Interposer* interposer, ...)`：`interposer` 可为空指针；当 MCF `all_ok` 且指针非空时，在返回前根据 MCF 路径对相应 `COBConnector::suspend()`，避免后续 deferred 迷宫与 MCF 已用开关冲突
  - 内部关键步骤：`build_track_graph()` → `prepare_commodities()` → `solve_bus_mcf()`（全局）→ `solve_simple_mcf_unit()` × 16（按 COBUnit）
  - `GlobalGraph`：track 级全局路由图（节点 = `(unit, dir, row, col, track)` 五元组 + VP/VN 虚拟节点）
  - `NodeMeta`：节点元数据，包含 `track_dir`（0=Horizontal, 1=Vertical）、`track_row`、`track_col`、`unit`、`track`
  - `Arc`：有向弧，包含 `u`/`v` 端点、`is_virtual`/`is_turn` 标记、`unit`、`cob`（所属 COB 线性编号）、`track_in`/`track_out`（输入/输出 track）、`from_dir`/`to_dir`（COBDirection，Wilton 转弯方向）
  - `PreparedCommodity`：每个 commodity 的 `label`、`origin_name`、源/汇节点、类别（`Plain`/`P`/`N`）、bus 标识、reach 步序列
  - `mcf/mcf_bbox.{hh,cc}`：`build_mcf_bbox_context()`、`physical_arc_in_bbox()`、`resolve_mcf_bbox()`；与 `ilp_bounding_box` 共用 `compute_bounding_box(record, rho[record_id])`
  - `StageSolveResult`：单阶段求解结果（已用边/节点、路径、`failed_bus_keys` / `failed_record_indices`）
  - `CobMcfRetryHints`：MCF 失败时回传 pipeline 的局部扩展提示（`failed_bus_keys`、`failed_simple_units`、`failed_record_indices`、`bus_failure_unlocalized`）
  - `arc_usable_for_class()`：按 `unit` 和 P/N 类别过滤 arc
  - `extract_path()`：从整数流解中通过 BFS 提取单 commodity 路径

- `algorithm/test_ILP/mcf/mcf_hw_map.hh`
  - COB/TOB 线性编号与坐标映射
  - `track_to_cob()` 规则封装（用于将 track 端点映射到 COB 图节点）
  - `tob_pair_cob_coords()`：TOB 对应的两个相邻 COB 坐标

### 3.6 ILP 加速辅助

- `algorithm/test_ILP/ilp_allocation/ilp_speedup.hh/.cc`
  - `cobunit_to_tracks()`：给定 cobunit 返回其包含的 8 条 track 列表（`bank*64 + g*8 + unit_local`，`g` ∈ 0..7）
  - `track_to_jk()`：track -> `(j, k)` 坐标映射

### 3.7 可达性预计算

- `algorithm/test_ILP/precompute/ilp_reach_precompute.hh/.cc`
  - `precompute_reach_for_records()`：在 ILP 求解前为每条 record 预计算可达 end_track / start_track 边集，并通过 Wilton 转弯映射（`hardware::COBUnit::index_map`）生成 `IlpReachStep` 序列
  - 分三种空间关系处理：vertical（同列直通）、horizontal（水平两步转弯）、diagonal（对角多步转弯）
  - 预计算结果写入 `record.starttrack_by_endtrack` 和 `record.reach_by_end_start`，供 **ILP** 约束使用
  - 返回 `IlpReachPrecomputeStats` 统计信息
  - 内部 `std::logic_error`（如 diagonal 上 `delta == 0`、或 `starts` 为空）会在消息中带 **`net_name` / `origin_key` / `record_id` / `end_track`** 及几何标志，便于定位是哪条 2-pin record 触发异常

- `algorithm/test_ILP/precompute/ilp_bounding_box.{hh,cc}`、`tob_reach_with_range.{hh,cc}`
  - `TobBBoxExpansionState` 保存 `rho_by_record`，每条 record 的 `rho_n` 上限为 4；`range_level` 只作为 `max(rho)` 兼容摘要
  - `precompute_reach_for_bbox_state(records, state)`：先执行 base `precompute_reach_for_records()`，再按每条 record 自己的 `rho_n` 扩展候选 start tracks
  - `rho_n=1..3`：逐层使用 `compute_bounding_box(record, rho)`，对每个 `end_track` 在新 bbox 内追加最近且未出现过的最多 2 条 start tracks（无 `reach` 步）
  - `rho_n=4`：每个 `end_track` 加入与 `end_track` 同 COBUnit 的 8 条 track（`cobunit_to_tracks(map_track(end_track))`，无 `reach` 步）
  - `precompute_reach_for_range(records, range_level)` 保留为 uniform `rho` 兼容包装

### 3.8 MCF 结果展示

- `run_mcf_global_routing_cob_units()` 末尾：先按 COBUnit 打印每条 commodity 的摘要（`path_count` 等），再按 **MCF 求解分组** 输出完整 track 路径：**BusMCF** 按 SyncNet `origin_key`；**SimpleMCF** 按 `(COBUnit, origin_uid)`（`record_origin_group_uid()`，与 `build_origin_groups()` 一致；TTB/PN 多扇出共一组，独立 B2B 各一组），`display` 为可读 `origin_key`
- **MCF 资源用量**（路径输出之后）：`MCF resource usage (post-solve, all_ok=...)` 起，对路由 Unit **U0–U15** 各打印完整 9×12 COB 网格的 `switches COB(r,c)=used/48`，以及全部水平/垂直邻接 `channel H/V COB(...)-COB(...)=used/total` 与 `unit_summary`
- **MCF 失败约束诊断**（`all_ok=false` 时）：`MCF infeasibility diagnosis: stage=...` 按 BusMCF / SimpleMCF_unitN 输出 Gurobi IIS 映射到第五版约束 kind（`edge_capacity`、`node_capacity`、`flow_conservation`、`bus_equal_length`、`f_le_x_*`、`x_le_o` 等）及可读 detail；warm start 重试路径不产生 IIS 日志

第一版文档中的「MCF 走廊内 mazeRoute」实验代码（`ilp_maze_search` / `ilp_maze_finalize`）已从本目标中移除；track 级结果以 MCF 直接输出的路径为准。

---

## 4. Net 拆分与语义（必须先理解）

`build_records()` 是该目录最关键的数据标准化步骤。

### 4.1 基本类型

- `Bnet`：bump -> bump
- `Tnet`：bump -> track（统一方向：起点是 bump，终点是 track。`BumpToTrackNet` 和 `TrackToBumpNet` 均按此约定构造 record）
- `PNnet`：由 `TracksToBumpsNet` 拆分得到（通常来自 pose/nege 固定网）

### 4.2 SyncNet 展平

`SyncNet` 会被拆成多条 2-pin 记录：

- `btb` -> `Bnet`
- `btt` -> `Tnet`（含 end_track）
- `ttb` -> `Tnet`（含 end_track，原始 begin_track 存为 `mcf_end_track`，bump 存为 `start_bumps`）

并统一写入 `origin_key = 原始 SyncNet 名`，用于后续 MCF 回并。

### 4.3 TracksToBumpsNet 拆分

`TracksToBumpsNet` 按 end_bump 拆成多条 `PNnet`：每条 PNnet 共享所有 begin_tracks 作为 `pn_end_tracks`，`power_kind` 由 net 名称推断（`"Pose nets"` → `Pose`，其余 → `Nege`）。

### 4.4 TrackToBumpsNet 与非法多扇出

| 类型 | `build_records` 行为 | ILP | MCF | COB 段 |
|------|----------------------|-----|-----|--------|
| `TrackToBumpsNet` | 每个 `end_bump` + 共享 `begin_track` 拆成一条 `Tnet`（`from_track_to_bumps_split=true`） | 与普通 `Tnet` 相同 | SimpleMCF（Origin 级 `x/o` 共享） | MCF 路径直接输出 |
| `BumpToBumpsNet` / `BumpToTracksNet` | `runtime_error` | — | — | — |

`BuildRecordsResult::deferred_multi_fanout` 正常应为空（遗留字段）。

`classify_net()` 不应再收到多扇出类型；若收到则抛 `std::logic_error`（内部错误）。

### 4.5 record_id 与 bit_id

`build_records()` 末尾为每条 record 赋值：
- `record_id`：按输出顺序的全局唯一 id（0, 1, 2, …）
- `bit_id`：同一 `origin_uid` 内的位序号（0, 1, 2, …），用于按 bit 粒度对齐

### 4.6 bits 语义

- ILP 阶段：`bits` 仍是记录级负载权重输入
- MCF 阶段：可能按类型重新解释（例如 PN 按 TOB 分组计数）

改动时不要混淆"ILP 的成本权重"与"MCF 的 commodity demand"。

---

## 5. MCF 建模框架（当前实现：track 级全局图 + 两阶段求解）

当前 MCF 实现已从 COB 级粗粒度图重构为 **track 级全局图**，并采用 **两阶段求解**策略。

### 5.1 Track 级全局图构建（`build_track_graph()`）

图节点（`NodeMeta`）：

- **物理节点**：每个节点对应 `(unit, track_dir, track_row, track_col, track)` 五元组，表示 COB 网格边界上的一个 track 位置。`track_dir=0` 为 Horizontal（位于 COB 左/右侧边界），`track_dir=1` 为 Vertical（位于 COB 上/下侧边界）。每个 `(unit, inner)` 组合产生 `(rows+1)×cols` 个 Vertical 节点和 `rows×(cols+1)` 个 Horizontal 节点。总物理节点数 = `16 × 8 × ((rows+1)×cols + rows×(cols+1))`
- **虚拟节点**：`V_P`（`virtual_kind=1`）、`V_N`（`virtual_kind=2`）用于 pose/nege
- **节点显示**：`node_text()` 格式为 `"U{unit} H/V({row},{col}) T{track}"` 或 `"V_P"` / `"V_N"`

图边（`Arc`）：

所有物理边在统一的单循环中构建：遍历每个 COB tile `(cob_r, cob_c)`，对每个 `(unit, inner)`，枚举所有 `(from_dir, to_dir)` 方向对（`from != to`）：

- `side_track_pos(from, cob_r, cob_c)` 计算入节点在 COB 网格上的位置（Down→V(r,c)，Up→V(r+1,c)，Left→H(r,c)，Right→H(r,c+1)）
- `side_track_pos(to, cob_r, cob_c)` 计算出节点位置
- `hardware::COBUnit::index_map(from, inner, to)` 计算 Wilton 映射后的输出 inner index
- `is_straight_through(from, to)` 判断是否为直通（Left↔Right / Up↔Down）：直通时 `is_turn=false`，否则 `is_turn=true`

按此方式，直通边和转弯边统一生成：
- **直通边**（`is_turn=false`）：相对方向对（如 Left→Right），连接同一 COB tile 两侧的边界节点
- **Wilton 转弯边**（`is_turn=true`）：非相对方向对（如 Left→Up），连接同一 COB tile 不同侧的边界节点，inner index 通过 Wilton 映射改变，bank 和 unit_local 不变
- **虚拟边**（`is_virtual=true`）：由 `prepare_commodities()` 按需添加，连接 PNnet 的 end_track 节点到 `V_P`/`V_N`

边去重：`directed_arc_set` 保证同一 `(u, v)` 有向边不重复添加。

### 5.2 Commodity 准备（`prepare_commodities()`）

从 ILP 的 `record_track_endpoints` 和 `records` 构建 `PreparedCommodity` 列表：

- **src 节点**：通过 `node_from_bump_track()` 从 `start_bumps.front().TOB` 位置定位（所有 Tnet 均统一为 bump=start 方向）
- **snk 节点**：PNnet 连到 `V_P`/`V_N`（通过遍历 `starttrack_by_endtrack` 找到与 `start_track` 可达的 `end_track`，为其添加虚拟边）；Tnet 通过 `node_from_track_coord()` 从 `mcf_end_track` 定位；Bnet 通过 `end_bumps.front().TOB` 定位
- **类别（McfClass）**：PNnet Pose → `P`，PNnet Nege → `N`，其余 → `Plain`
- **bus 标识（BusMCF）**：仅 `origin_key` 匹配 `SyncNet in group {正整数}`（`group > 0`）的 commodity 标记为 `is_bus=true`，`bus_key = origin_name`；`BumpToBumpNet_*_in_group_-1` 等 **不** 进 BusMCF
- **SimpleMCF Origin 分组**（`build_origin_groups()`）：统一按 `(cob_unit, record_origin_group_uid(record))` 聚合；`origin_uid` 来自 `net->uid()`。同一父 net 的拆分 record（TTB/PN）共享 uid → 共享 `x^H`；每条独立 B2B 有唯一 uid → 独立 Origin
- **reach_steps**：从 `record.reach_by_end_start` 提取，当前仅用于 SAT/legacy ILP 可达性约束与日志；MCF 不注入 Wilton 转弯等式约束

### 5.2.1 MCF bbox 可行图（第六版 §456–717）

`run_mcf_global_routing_cob_units()` 在 `prepare_commodities()` 之后从 `ilp_result.bbox_expand_by_record` 构造 `TobBBoxExpansionState`，再调用 `build_mcf_bbox_context(records, commodities, state)`：

| 对象 | MCF 范围 |
|------|----------|
| Bnet / Tnet | `compute_bounding_box(record, rho[record_id])` |
| BusMCF（SyncNet） | 同 `bus_key` 内所有成员 record 当前 bbox 的 `rect_hull_boxes` |
| SimpleMCF 多扇出 origin | 各 child record 当前 bbox 的 RectHull（TTB 等） |
| Pnet / Nnet | **不裁剪**（第五版全图 + virtual 边） |

实现要点：

- **弧过滤**：`arc_allowed_for_commodity()` = `arc_usable_for_class` ∧ `physical_arc_in_bbox`（转弯边看 `arc.cob`；通道边要求相邻两 COB 均在 bbox 内，与 `tob_channel_kshortest` 一致）
- **变量**：范围外不建 `f` / `x`；约束求和自然限于可行边集
- **预检**：受限 commodity 在允许弧上 BFS 不可达 → `bbox disconnected`，跳过 Gurobi
- **日志**：`MCF using SAT bbox max_rho=... rho_by_record=...`、`MCF bbox: max_rho=... commodities=... bus_groups=...`
- **warm start**：`route_one_mcf_warm_path` 使用与求解相同的 bbox 过滤

**MCF 失败重试**（`solve_tob_mcf_pipeline`）：

- TOB SAT UNSAT：扩展所有 `Tnet/PNnet` records；不做 UNSAT core 归因。
- BusMCF 失败：扩展失败 `bus_key` 下所有 member records。bbox disconnected 可直接定位 commodity；Gurobi infeasible 从 IIS metadata 收集 `bus_key`。若无法定位任何 `bus_key`，直接失败，不扩展全部 bus。
- SimpleMCF / `--simple-maze` 失败：扩展失败 unit 内所有 simple records；若其中包含多扇出 origin，则扩展该 origin 的所有 child records。
- 终止条件：成功返回；或某轮失败需要扩展但 `expand(fail_set)` 没有改变任何 record（说明相关 records 已到 `rho=4`）。

### 5.3 两阶段求解（第五版 SimpleMCF 无向 `x` + bbox：`solve_bus_mcf` + `solve_simple_mcf_unit`）

建模仍用**有向弧** `f` 做流守恒；**无向物理边**语义用于 BusMCF 边容量与 SimpleMCF 的 `x^H_e` / 残余容量（第五版；未采用第四版「全局无向 `f`」）。

`run_mcf_global_routing_cob_units()` 流程：

1. `build_mcf_bbox_context()`（沿用 SAT 成功轮每条 record 的局部 `rho`）
2. **BusMCF**（`solve_bus_mcf()`，全局一次）：仅在各 commodity 的 bus RectHull 可行图上建 `f^{c,n}`、`o^{c,n}`；目标 `min Σ f`；约束含流守恒、边容量、节点占用、同步线长
3. **SimpleMCF**（`solve_simple_mcf_unit(c)`，每个 COBUnit 独立 Gurobi 模型）：按 commodity / origin 组 bbox 建 `f^{c,n}`、`x^{c,H}_e`、`o^{c,H}_i`；目标 `min Σ x_e` **仅当** `--enable-mcf-obj`，否则纯可行性

#### BusMCF 约束组与日志

建模型时输出：

- `BusMCF model graph: nodes=… arcs=… commodities=…`
- 每种约束的行数：`flow_conservation`、`edge_capacity`、`f_le_o_link`、`node_capacity`、`bus_equal_length`
- 变量规模：`f=… o=… cols=… rows=…`

#### SimpleMCF 约束组与日志

每个 unit 建模型时输出：

- `SimpleMCF_unit{c} model graph: nodes=… arcs=… commodities=…`
- 约束行数：`flow_conservation`、`edge_capacity`、`f_le_x_lower`、`f_le_x_upper`、`x_le_o`、`o_le_sum_x`、`node_capacity`
- 变量规模：`f=… x=… o=… origin_groups=… cols=… rows=…`
- `objective min_sum_x: enabled/disabled`

#### 变量与约束概要

**BusMCF**：

- 决策变量：`f[k][a]`（commodity 流）、`o[k][n]`（节点占用）
- 流守恒、无向物理边容量 `Σ_n(f_{ij}+f_{ji}) ≤ 1`、节点 `f≤o` 且 `Σ_n o≤1`、bus 等长（仅 SyncNet bus）：
  - `total_flow_n = Σ_{(i,j)∈E_n^c} f^{c,n}_{ij}`（弧集由 class + bus bbox 限定）
  - 同 `bus_key` 内：`total_flow_n = total_flow_m`

**SimpleMCF**（第五版）：

- 决策变量：`f[k][a]`（有向弧流）、`x[h][e]`（Origin 级**无向物理边**占用，`e={(i,j),(j,i)}`）、`o[h][n]`
- `f_{ij}, f_{ji} ≤ x^H_e` 且 `x^H_e ≤ Σ_{n∈H.child}(f^n_{ij}+f^n_{ji})`；边容量 `Σ_H x^H_e ≤ capacity_e - used^{Bus,c}_e`
- 节点：`x^H_e ≤ o^H_i`（`e∈δ(i)`）、`o^H_i ≤ Σ_{e∈δ(i)} x^H_e`、`Σ_H o^H_i ≤ 1 - used^{Bus,c}_i`
- P/N 路径组成通过 `arc_usable_for_class()` 隐式保证（不连 virtual 节点的 commodity 无对应弧变量）

求解后通过 `extract_path()` 从整数流解中 BFS 提取每个 commodity 的节点路径。

### 5.4 路径输出

求解结果存入 `CobMcfFullResult.paths_by_unit[16]`，每个 `McfPathInfo` 包含：
- `label`：commodity 标签（`"{net_name}#{record_id}"`）
- `origin_name`：原始 net 标识（用于回并）
- `record_id`、`start_track`、`end_track`、`cob_unit`：关联信息
- `src`、`snk`、`demand`：源汇节点和需求量
- `record_indices`：关联的 record_id 列表
- `unit_paths`：节点 id 序列（可用 `node_text()` 格式化为 `"U{unit} H/V({row},{col}) T{track}"` 或 `"V_P"` / `"V_N"`）
- `track_paths`：从节点路径提取的去重 track index 序列

### 5.5 日志输出（便于诊断）

- **Gurobi 约束矩阵诊断**（每个 Gurobi 模型 `optimize()` 前，写入 `./gurobi-log/modelinfo.log`）：前缀 `{stage} constraint matrix:`，输出 `rows/cols/nnz/density/sparse/max_row_nnz/avg_row_nnz`（求解前原始模型，非 presolve 后）。前缀 `{stage} heavy coupling rows:` 列出非零系数偏多的约束行（阈值 `max(10, 5×avg_row_nnz)` 或 top-10）；MCF 行附加 `kind/detail`（与 IIS 诊断字段一致）。启用 `--gurobi-log` 时求解器日志另写 `./gurobi-log/gurobi_{stage}_{seq}.log`，路径也会记入 `modelinfo.log`
- **MCF 建模型**：每个 BusMCF / SimpleMCF_unit 求解前打印图规模（nodes/arcs/commodities）、每种约束的行数、变量/col/row 总数；SimpleMCF 另打印 objective 是否启用
- **ILP 路由细节**：`main.cc` 中通过 `result.route_details` 输出每条 net 的完整分配信息（bump 坐标、j/k 线、s、orient、track、COBUnit），格式示例：`net "...": bump(T0,B0,G1,I1) -> j=1 (horizontal line), k=4 (vertical line), s=12, orient=straight(QS), track=12, COBUnit=4`
- **ILP W 变量明细**：输出所有 active W 及其对应 bump、j、k、orient、track
- **ILP S 变量明细**：输出所有 active S 及其对应 TOB、v、j、k
- **MCF 按 unit 汇总**：每个 unit 的 bus/simple commodity 数量及求解状态
- **每个 commodity 的路径明细**：按 `commodity -> path#i` 打印完整节点链（`U{unit} H/V({row},{col}) T{track}` / `V_P` / `V_N`）
- **MCF 资源用量**：见 §3.8；前缀 `MCF resource usage`、`switches COB`、`channel H/V`、`unit_summary`
- **MCF 不可行诊断**：见 §3.8；前缀 `MCF infeasibility diagnosis`、`IIS constraint kinds`

---

## 6. 与主工程的边界

该目录用于算法验证，不直接承担 `source/algo/router/` 的线上职责。

强约束：

- 尽量不改 `source/` 下主流程接口语义
- 新实验字段优先放在 `algorithm/test_ILP` 内
- CLI 行为变更先在 `test_ILP` 自洽，再考虑迁移到主入口

---

## 7. 常见改动场景与建议

### 7.1 想改 ILP 目标/约束

优先修改：

- `ilp_allocation/tob_ilp_model.cc` 的 `build_tob_ilp_model()`（约束与变量定义）
- `precompute/ilp_reach_precompute.cc` 的 `fill_*_case()` 系列（可达性预计算逻辑）
- 必要时同步 `main.cc` 的 record/cost 生成逻辑

### 7.2 想改 MCF 图拓扑或路径规则

优先修改：

- `mcf/cob_mcf_router.cc`（`build_track_graph()` / `prepare_commodities()` / `solve_bus_mcf()` / `solve_simple_mcf_unit()` / `arc_usable_for_class()`）
- `mcf/mcf_hw_map.hh`（track / COB / TOB 映射规则）

### 7.3 想改 net 回并策略

优先修改：

- `main.cc` 的 `build_records()`（保证 `origin_key` 正确）
- `mcf/cob_mcf_router.cc` 的 `prepare_commodities()`（bus 分组与 commodity 构建）

---

## 8. 关键不变量（修改前后都要守住）

1. **SAT TOB 可独立成功**  
   不加 `--enable-mcf-routing` 时，TOB 分配流程应可独立完成。

2. **record 与 assignment 数量一致**  
   MCF 假设 `records.size() == result.assignments.size()` 且 `records.size() == result.record_track_endpoints.size()`。

3. **类型内语义一致**  
   同一回并组不应混合 `Bnet/Tnet/PNnet`，否则应显式报错。

4. **track 端点映射可复现**  
   `track_to_cob()` 规则必须稳定、可解释，不能引入随机性。

5. **日志可诊断**  
   每个 unit 的 bus/simple commodity 数、可行性、目标值、耗时应有日志。

6. **record_id 全局唯一**  
   `record_id` 由 `build_records()` 按输出顺序分配，ILP/MCF 各阶段通过 `record_id` 对齐数据。

7. **`active_s` 与 `active_w` 配对**  
   `active_s` 中的 `(tob,v)` 必须满足存在 `active_w` 中某条使 `w.bump.TOB==tob` 且 `v=w.j×8+w.k`。解析时不直接信任 `S=1`；orphan S 不参与 Interposer 配置（`ilp_apply_interposer` 仅应用 W 推导后的 `active_s`）。

---

## 9. 最小验证清单（每次改动后）

1) `xmake build test_ILP` 与 `xmake build test_ILP_unit` 成功（后者覆盖 `tob_bbox_expansion_state_test`）  
2) SAT TOB only：

```bash
./output/test_ILP <config_path>
```

3) SAT TOB + MCF（推荐带 `--enable-mcf-obj`）：

```bash
./output/test_ILP <config_path> --enable-mcf-routing --enable-mcf-obj
```

历史端到端验证：`test/config/case1`、`case5`、`case6` 在 base bbox 下通过（旧日志为 `range_level=0`；当前日志应看 `max_rho=0`、`MCF bbox:` / `MCF using SAT bbox`）。

4) MCF with graph warm start：

```bash
./output/test_ILP <config_path> --enable-mcf-routing --enable-pre-routing
```

```bash
./output/test_ILP <config_path> --enable-mcf-routing --enable-mcf-obj
```

5) SimpleMCF 失败后的 maze 连通性诊断（case5 等）：

```bash
./output/test_ILP test/config/case5 --enable-mcf-routing --enable-mcf-obj --maze-check-ilp-mcf
./output/test_ILP test/config/case5 --enable-mcf-routing --enable-mcf-obj --maze-check-mcf
```

预期：MCF 可能仍 exit 1；日志含 `apply ILP to interposer`、`MCF→Interposer: suspended`；ilp-mcf 含 `maze-check-ilp-mcf origin=...`；mcf 含 `maze-check-mcf origin=...` 与 `maze-check-mcf record_id=...`。

注意：`--enable-mcf-parallel` 对 SimpleMCF per-unit 求解生效。`--enable-mcf-obj` 仅影响 SimpleMCF 目标函数。`--enable-pre-routing` 仅影响 MCF warm start，不改变 SAT/MCF 硬约束。`--gurobi-log` 仅写 MCF 阶段日志；TOB 用 `--sat-log`。`--maze-check-ilp-mcf` / `--maze-check-mcf` 不改变 MCF 求解与 exit code，仅追加诊断日志。

6) 若改了模型结构，建议附带：

- `--export-ilp-mps` 导出样例（legacy ILP 对照）
- 至少一个 case 的前后对比日志

---

## 10. 术语约定（本目录）

- **record**：`Net_cost_record`，2-pin 粒度建模单元
- **record_id**：record 在 `build_records` 输出中的全局唯一序号
- **bit_id**：同一 `origin_uid` 内的位序号
- **origin_key**：原始 net 名（人类可读）
- **origin_uid**：电路 net uid，MCF Origin / maze-check / bit_id 的分组键
- **assignment**：TOB 分配输出的 record -> cobunit 结果
- **record_track_endpoint**：TOB 分配输出的 record -> `(cob_unit, has_start_track, start_track, has_end_track, end_track)` 结构
- **commodity**：MCF 中单一供需流对象（`PreparedCommodity`）
- **cobunit**：16 个布线资源分区之一（由 `map_track()` 规则定义）
- **track graph**：track 级全局路由图（`GlobalGraph`），节点粒度为 `(unit, dir, row, col, track)`，节点位于 COB 网格边界上
- **直通边**：同一 COB tile 内相对方向对（Left↔Right / Up↔Down）的边，`is_turn=false`
- **Wilton 转弯边**：同一 COB tile 内非相对方向对的边，`is_turn=true`，inner index 通过 Wilton 映射改变
- **BusMCF**：第一阶段求解，仅 `SyncNet in group {正整数}` commodity，带同步等长约束
- **SimpleMCF**：第二阶段，按 COBUnit 独立求解其余 commodity（含 `in_group_-1` 的 BumpToBumpNet、Tnet、TTB 等）；默认纯可行性，可选 `--enable-mcf-obj` 启用 `min Σ x`
- **rho / MCF bbox**：每条 record 的局部扩展量 `rho[record_id] ∈ [0,4]`；MCF 与 TOB SAT 共用 `compute_bounding_box(record, rho[record_id])`。`range_level` 仅表示 `max(rho)` 兼容摘要
- **bbox_expand_by_record**：`TobIlpResult` 中每条 record 的 `rho` 向量，MCF 阶段的真实 bbox 来源

**case5（`test/config/case5`）MCF 诊断预期**（`--enable-mcf-routing`）：`BusMCF commodities=80`、`bus_equal_length=64`（16 组 SyncNet：4×(8−1) + 12×(4−1)）；`BumpToBumpNet in_group_-1` 的 32 条记录在 SimpleMCF 中各用独立 `origin_uid`（每条 1 Origin）。
- **reach_steps**：Wilton 转弯步序列（`IlpReachStep`），描述 end_track 到 start_track 的转弯路径

术语尽量统一，不要在同一文档或代码注释里混用"子网/边/commodity/net"而不加限定。
