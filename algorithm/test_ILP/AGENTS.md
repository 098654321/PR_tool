# PR_tool /algorithm/test_ILP 工程指南（面向 AI Agent）

本文件是 `algorithm/test_ILP/` 子工程的入口说明。目标是：在不破坏现有 ILP 功能的前提下，理解并扩展"**ILP 分配 + MCF 全局布线**"实验链路。

该目录是一个独立的算法验证入口，不直接替代 `source/algo/router/` 的正式路由流程。它强调：

- 快速构建可复现的数学模型（ILP / MCF）
- 用 HiGHS 求解并导出可解释结果
- 在 `test_ILP` 范围内隔离实验逻辑，避免污染主流程

---

## 1. 快速上手（构建 / 运行）

`test_ILP` 目标由仓库根目录 `xmake.lua` 配置，构建时会编译：

- `algorithm/test_ILP/main.cc`
- `algorithm/test_ILP/tob_ilp_model.cc`
- `algorithm/test_ILP/highs.cc`
- `algorithm/test_ILP/ilp_speedup.cc`
- `algorithm/test_ILP/ilp_reach_precompute.cc`
- `algorithm/test_ILP/pre_routing_warm_start.cc`
- `algorithm/test_ILP/cob_mcf_router.cc`

常用命令（仓库根目录）：

```bash
xmake build test_ILP
./output/test_ILP <config_path> [output_mps_path] [-v|-vv|...] [--enable-ilp-parallel] [--cob-rows N --cob-cols M] [--enable-mcf-routing] [--enable-mcf-parallel] [--enable-mcf-obj] [--enable-pre-routing]
```

参数语义（以 `main.cc` 为准）：

- `config_path`：配置目录（例如 `algorithm/test_ILP/case1`）
- `output_mps_path`：可选，导出 ILP MPS 文件
- `-v` / `-vv` / …：设置 verbose 模式（`-v` 计数越多越详细），启用后将 debug level 设为 `Debug`
- `--enable-ilp-parallel`：HiGHS 并行求解 ILP
- `--cob-rows N` / `--cob-cols M`：可选，**必须成对出现或均省略**。省略时 MCF 构图使用 `hardware::Interposer::COB_ARRAY_HEIGHT` 与 `COB_ARRAY_WIDTH`。若显式传入，数值必须与上述常量完全一致，否则程序报错退出（保证 `track_to_cob`、Bump/TOB 坐标与 MCF 物理假设一致）
- `--enable-mcf-routing`：在 ILP 分配成功后继续执行 MCF 阶段
- `--enable-mcf-parallel`：SimpleMCF 按 COBUnit 并行求解（`std::async`，每个 unit 独立 HiGHS 实例）
- `--enable-mcf-obj`：与 `--enable-mcf-routing` 联用时，SimpleMCF 加入 `min Σ x` 目标函数；**省略时 SimpleMCF 为纯可行性求解**（所有变量成本为 0）。BusMCF 始终带 `min Σ f` 目标
- `--enable-pre-routing`：启用两处 maze warm start。ILP 前在 shadow `Interposer/BaseDie` 上调用主工程 `MazeRouteStrategy`，把已得到的 TOB 连接选择转为 HiGHS MIP start；MCF 前在 `cob_mcf_router.cc` 的 `GlobalGraph` 上按 BusMCF/SimpleMCF 顺序跑 BFS maze，把路径转为 MCF 变量初值。失败的预布线只记录日志，不作为硬约束；若 HiGHS 使用 warm start 后未返回 optimal，会自动无 warm start 重试
- MCF 成功后，`cob_mcf_router.cc` 会按 **`origin_key`（与 `build_nets()` 得到的逻辑 net 名一致）** 分组打印 track 级路径；仍保留按 COBUnit 的 commodity 摘要行便于对照容量

---

## 2. 总体数据流（两阶段）

入口：`algorithm/test_ILP/main.cc` 的 `run_main()`

1) `parse::read_config` + `algo::build_nets`  
2) `build_records()`：将 `circuit::Net` 展平为 2-pin 级 `Net_cost_record`，并分配 `record_id` 和 `bit_id`；`TrackToBumpsNet` 按 bump 拆成多条 `Tnet`（`from_track_to_bumps_split`）参与 ILP，原 net 记入 `BuildRecordsResult::track_to_bumps_nets`；`BumpToBumpsNet` / `BumpToTracksNet` 为非法类型，直接报错退出  
3) `precompute_reach_for_records()`：为每条 record 预计算可达 end_track / start_track 边集及 Wilton 转弯步序列（`IlpReachStep`），返回 `IlpReachPrecomputeStats` 统计信息  
4) 可选 `write_mps_file()`：导出 MPS 文件  
5) 若启用 `--enable-pre-routing`：`build_ilp_warm_start_from_maze()` 在 shadow 硬件对象上跑主工程 maze，并生成 `TobIlpWarmStart`  
6) `solve_tob_ilp_with_highs()`：ILP 求解，输出每条 2-pin net 的 `COBUnit` 分配、W/S/QS/QW 决策变量值、`record_track_endpoints`（每条 record 对应的 start_track / end_track）；若传入 warm start，则先交给 HiGHS 作为 MIP start  
7) 若启用 `--enable-mcf-routing`：  
   `run_mcf_global_routing_cob_units()`，在 track 级全局图上做 BusMCF（全局）+ SimpleMCF（按 COBUnit）；TTB 子 Tnet 在 SimpleMCF 中以 Origin 级 `x/o` 建模。若同时启用 `--enable-pre-routing`，会在求解前对 BusMCF/SimpleMCF commodity 生成 graph-maze 初始路径并传入 HiGHS；MCF 返回前在 `Interposer` 上对 MCF 路径经过的 `COBConnector` 调用 `suspend()`（见 [`source/AGENTS.md`](source/AGENTS.md)）

建议把该链路理解为：

- **阶段 A（ILP）**：先决定每条 2-pin net 落在哪个 `COBUnit`，同时确定每个 bump 对应的 track
- **阶段 B（MCF）**：在 track 级全局图上按 commodity 做容量约束整数流路由，分 BusMCF（等长约束）和 SimpleMCF（残余容量）两阶段

### 2.1 分阶段耗时（写入 `debug.log`）

与「ILP 预热 → ILP 求解 → MCF 预热 → MCF 求解」四段直接相关的墙钟时间会单独打日志（单位 ms）：

| 日志前缀 | 含义 |
| --- | --- |
| `timing phase=ilp_warm_start` | `build_ilp_warm_start_from_maze()`；未使用 `--enable-pre-routing` 时为 **0** |
| `timing phase=ilp_solve` | `solve_tob_ilp_with_highs()` 整体（含 HiGHS 在 warm start 不佳时自动无初值重试） |
| `timing phase=mcf_warm_start` | MCF 前在 `GlobalGraph` 上的 warm path 构造；未使用 `--enable-pre-routing` 时约为 **0** |
| `timing phase=mcf_solve` | BusMCF + 各 COBUnit SimpleMCF 的 HiGHS 求解之和 |

`run_main` 在成功或 ILP/MCF 失败退出前会再打一行汇总：`timing breakdown (ms): ilp_warm_start=..., ilp_solve=..., mcf_warm_start=..., mcf_solve=...`（未执行 MCF 时后两项为 **0**）。另：`run_main total elapsed` 表示整个 `run_main` 墙钟；`MCF detailed(track-level) summary` 中的 `total_elapsed` 表示 `run_mcf_global_routing_cob_units()` 整段墙钟（含 `prepare_commodities`、路径打印等），一般会大于 `mcf_warm_start + mcf_solve`。`CobMcfRunSummary::mcf_warm_start_ms` / `mcf_solve_ms` 与上述 MCF 两段一致，便于程序内读取。

---

## 3. 关键文件与职责

### 3.1 入口与数据预处理

- `algorithm/test_ILP/main.cc`
  - CLI 参数解析（含 verbose `-v` 计数）
  - 2-pin 记录构建（`build_records` / `BuildRecordsResult`）与类型拆分（`classify_net`）；`TrackToBumpsNet` ILP 拆分 + MCF 成功后 post-MCF 迷宫
  - `algorithm/test_ILP/ilp_apply_interposer.hh/.cc`：`apply_tob_ilp_result_to_interposer`（S 配置 `hori_to_vert`、W 写 `allocated_track`/`intersect_access_unit`）
  - 可达性预计算调度（`precompute_reach_for_records`）
  - 可选 pre-routing warm start 调度（`--enable-pre-routing`）
  - ILP 求解调用
  - ILP 结果输出（`route_details`、`active_w`、`active_s`）
  - 可选 MCF 阶段调度

- `algorithm/test_ILP/pre_routing_warm_start.hh/.cc`
  - 在 shadow `Interposer/BaseDie` 上复用主工程 `MazeRouteStrategy`，生成 ILP MIP start
  - 从 `PathPackage` 的 TOB connector 中提取 `W/S/QS/QW` 初值，并为 PNnet 尝试推导 `Y` 初值
  - 预布线失败只影响 warm start 覆盖率，不中止主 ILP 流程

### 3.2 ILP 类型与元数据

- `algorithm/test_ILP/ilp_types.hh`
  - `Bump_coord`、`Net_type`（`Bnet` / `Tnet` / `PNnet`）
  - `IlpPowerKind`（`None` / `Pose` / `Nege`）、`IlpEndpointKind`（`Bump` / `Track`）
  - `IlpReachStep`（Wilton 转弯步：`from_dir`、`to_dir`、`index_in`、`index_out`）
  - `Net_cost_record`（包含 ILP 与 MCF 共用字段）
  - `map_track()`（track -> cobunit 映射）

`Net_cost_record` 关键字段：

- `origin_key`：把拆分后的 2-pin 子网回并到原始 net
- `record_id`：`build_records` 输出序中的全局唯一 id（用于 ILP/MCF 对齐）
- `bit_id`：同一 `origin_key` 内的位序号
- `power_kind`：`Pose / Nege / None`
- `mcf_start_kind / mcf_end_kind`
- `mcf_start_track / mcf_end_track`
- `mcf_has_start_track / mcf_has_end_track`
- `pn_end_tracks` / `pn_end_track_coord_by_index`：PNnet 的所有 0/1 端口 track
- `end_tracks`：可达性预计算后的 end track 列表
- `starttrack_by_endtrack`：每个 end_track 对应的可达 start_track 列表
- `reach_by_end_start`：每个 `(end_track, start_track)` 对对应的 Wilton 转弯步序列

### 3.3 ILP 模型构建

- `algorithm/test_ILP/tob_ilp_model.hh/.cc`
  - `TobIlpModel`：行列式模型拼装
  - `build_tob_ilp_model()`：约束与目标构建
  - `to_highs_lp()`：转换为 HiGHS LP 对象

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

### 3.4 ILP 求解

- `algorithm/test_ILP/highs.hh/.cc`
  - `TobIlpResult`：包含 `assignments`、`active_w`、`active_s`、`route_details`、`record_track_endpoints`
  - `TobIlpRecordTrackEndpoint`：每条 record 的 `(record_id, cob_unit, has_start_track, start_track, has_end_track, end_track)` 结构
  - `solve_tob_ilp_with_highs()`：设置 HiGHS 选项、运行求解、解析 W/S/QS/QW 决策变量、推导 track 和 cobunit、构建 `record_track_endpoints`
  - 支持并行配置与线程信息输出

### 3.5 MCF 路由

- `algorithm/test_ILP/cob_mcf_router.hh/.cc`
  - 负责 track 级全局 MCF 的完整流水线
  - 入口函数 `run_mcf_global_routing_cob_units(..., hardware::Interposer* interposer, ...)`：`interposer` 可为空指针；当 MCF `all_ok` 且指针非空时，在返回前根据 MCF 路径对相应 `COBConnector::suspend()`，避免后续 deferred 迷宫与 MCF 已用开关冲突
  - 内部关键步骤：`build_track_graph()` → `prepare_commodities()` → `solve_bus_mcf()`（全局）→ `solve_simple_mcf_unit()` × 16（按 COBUnit）
  - `GlobalGraph`：track 级全局路由图（节点 = `(unit, dir, row, col, track)` 五元组 + VP/VN 虚拟节点）
  - `NodeMeta`：节点元数据，包含 `track_dir`（0=Horizontal, 1=Vertical）、`track_row`、`track_col`、`unit`、`track`
  - `Arc`：有向弧，包含 `u`/`v` 端点、`is_virtual`/`is_turn` 标记、`unit`、`cob`（所属 COB 线性编号）、`track_in`/`track_out`（输入/输出 track）、`from_dir`/`to_dir`（COBDirection，Wilton 转弯方向）
  - `PreparedCommodity`：每个 commodity 的 `label`、`origin_name`、源/汇节点、类别（`Plain`/`P`/`N`）、bus 标识、reach 步序列、bbox
  - `StageSolveResult`：单阶段求解结果（已用边/节点、路径）
  - `arc_usable_for_class()`：按 `unit` 和 P/N 类别过滤 arc
  - `extract_path()`：从整数流解中通过 BFS 提取单 commodity 路径

- `algorithm/test_ILP/mcf_hw_map.hh`
  - COB/TOB 线性编号与坐标映射
  - `track_to_cob()` 规则封装（用于将 track 端点映射到 COB 图节点）
  - `tob_pair_cob_coords()`：TOB 对应的两个相邻 COB 坐标

### 3.6 ILP 加速辅助

- `algorithm/test_ILP/ilp_speedup.hh/.cc`
  - `cobunit_to_tracks()`：给定 cobunit 返回其包含的 8 条 track 列表（`bank*64 + g*8 + unit_local`，`g` ∈ 0..7）
  - `track_to_jk()`：track -> `(j, k)` 坐标映射

### 3.7 可达性预计算

- `algorithm/test_ILP/ilp_reach_precompute.hh/.cc`
  - `precompute_reach_for_records()`：在 ILP 求解前为每条 record 预计算可达 end_track / start_track 边集，并通过 Wilton 转弯映射（`hardware::COBUnit::index_map`）生成 `IlpReachStep` 序列
  - 分三种空间关系处理：vertical（同列直通）、horizontal（水平两步转弯）、diagonal（对角多步转弯）
  - 预计算结果写入 `record.starttrack_by_endtrack` 和 `record.reach_by_end_start`，供 **ILP** 约束使用
  - 返回 `IlpReachPrecomputeStats` 统计信息
  - 内部 `std::logic_error`（如 diagonal 上 `delta == 0`、或 `starts` 为空）会在消息中带 **`net_name` / `origin_key` / `record_id` / `end_track`** 及几何标志，便于定位是哪条 2-pin record 触发异常

### 3.8 MCF 结果展示

- `run_mcf_global_routing_cob_units()` 末尾：先按 COBUnit 打印每条 commodity 的摘要（`path_count` 等），再按 **`origin_key` / `McfPathInfo::origin_name`** 分组输出完整 `path_to_text` track 路径，便于与 `build_nets()` 得到的原始 net 对应（SyncNet / TracksToBumpsNet 拆分出的子 record 共享同一 `origin_key`）

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
- `bit_id`：同一 `origin_key` 内的位序号（0, 1, 2, …），用于按 bit 粒度对齐

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
- **SimpleMCF Origin 分组**（`build_origin_groups()`）：
  - `from_track_to_bumps_split`（TrackToBumpsNet 子 Tnet）：按 `(cob_unit, origin_key)` 聚合，共享 `x/o`
  - `BumpToBumpNet` 且 `origin_key` 含 `in_group_-1`：按 `(cob_unit, commodity.label)` 独立 Origin（每条 2-pin 不共享 `x/o`）
  - 其余：按 `(cob_unit, origin_key)`
- **reach_steps**：从 `record.reach_by_end_start` 提取，当前仅用于 ILP 可达性约束与日志；MCF 不注入 Wilton 转弯等式约束
- **bbox_cobs**：src 和 snk 的 COB 坐标构成的矩形范围内的 COB 列表

### 5.3 两阶段求解（第三版：`solve_bus_mcf` + `solve_simple_mcf_unit`）

`run_mcf_global_routing_cob_units()` 流程：

1. **BusMCF**（`solve_bus_mcf()`，全局一次）：变量 `f^{c,n}`、`o^{c,n}`；目标 `min Σ f`；约束含流守恒、边/节点容量、同步线长
2. **SimpleMCF**（`solve_simple_mcf_unit(c)`，每个 COBUnit 独立 HiGHS）：变量 `f^{c,n}`、`x^{c,H}`、`o^{c,H}`（所有 Origin）；目标 `min Σ x` **仅当** `--enable-mcf-obj`，否则纯可行性；约束含 `f≤x`、Bus 残余边/节点容量

#### BusMCF 约束组与日志

建模型时输出：

- `BusMCF model graph: nodes=… arcs=… commodities=…`
- 每种约束的 HiGHS 行数：`flow_conservation`、`edge_capacity`、`f_le_o_link`、`node_capacity`、`bus_equal_length`
- 变量规模：`f=… o=… cols=… rows=…`

#### SimpleMCF 约束组与日志

每个 unit 建模型时输出：

- `SimpleMCF_unit{c} model graph: nodes=… arcs=… commodities=…`
- 约束行数：`flow_conservation`、`edge_capacity`、`f_le_x`、`x_le_o_link`、`node_capacity`
- 变量规模：`f=… x=… o=… origin_groups=… cols=… rows=…`
- `objective min_sum_x: enabled/disabled`

#### 变量与约束概要

**BusMCF**：

- 决策变量：`f[k][a]`（commodity 流）、`o[k][n]`（节点占用）
- 流守恒、边容量 `Σ_n f ≤ 1`、节点 `f≤o` 且 `Σ_n o≤1`、bus 等长（仅 SyncNet bus）：
  - `total_flow_n = Σ_{(i,j)∈E^c} f^{c,n}_{ij}`，`c` = commodity `n` 所在 COBUnit（弧已由 `arc_usable_for_class` 限定）
  - 同 `bus_key` 内：`total_flow_n = total_flow_m`

**SimpleMCF**：

- 决策变量：`f[k][a]`、`x[h][a]`（Origin 级边占用）、`o[h][n]`（Origin 级节点占用）
- `f ≤ x`；边容量 `Σ_H x ≤ capacity - used^{Bus,c}`；节点 `x≤o^H` 且 `Σ_H o^H ≤ 1 - used^{Bus,c}`
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

- **MCF 建模型**：每个 BusMCF / SimpleMCF_unit 求解前打印图规模（nodes/arcs/commodities）、每种约束的 HiGHS 行数、变量/col/row 总数；SimpleMCF 另打印 objective 是否启用
- **ILP 路由细节**：`main.cc` 中通过 `result.route_details` 输出每条 net 的完整分配信息（bump 坐标、j/k 线、s、orient、track、COBUnit），格式示例：`net "...": bump(T0,B0,G1,I1) -> j=1 (horizontal line), k=4 (vertical line), s=12, orient=straight(QS), track=12, COBUnit=4`
- **ILP W 变量明细**：输出所有 active W 及其对应 bump、j、k、orient、track
- **ILP S 变量明细**：输出所有 active S 及其对应 TOB、v、j、k
- **MCF 按 unit 汇总**：每个 unit 的 bus/simple commodity 数量及求解状态
- **每个 commodity 的路径明细**：按 `commodity -> path#i` 打印完整节点链（`U{unit} H/V({row},{col}) T{track}` / `V_P` / `V_N`）

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

- `tob_ilp_model.cc` 的 `build_tob_ilp_model()`（约束与变量定义）
- `ilp_reach_precompute.cc` 的 `fill_*_case()` 系列（可达性预计算逻辑）
- 必要时同步 `main.cc` 的 record/cost 生成逻辑

### 7.2 想改 MCF 图拓扑或路径规则

优先修改：

- `cob_mcf_router.cc`（`build_track_graph()` / `prepare_commodities()` / `solve_bus_mcf()` / `solve_simple_mcf_unit()` / `arc_usable_for_class()`）
- `mcf_hw_map.hh`（track / COB / TOB 映射规则）

### 7.3 想改 net 回并策略

优先修改：

- `main.cc` 的 `build_records()`（保证 `origin_key` 正确）
- `cob_mcf_router.cc` 的 `prepare_commodities()`（bus 分组与 commodity 构建）

---

## 8. 关键不变量（修改前后都要守住）

1. **ILP 回归不破坏**  
   不加 `--enable-mcf-routing` 时，流程与结果应可独立成功。

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

---

## 9. 最小验证清单（每次改动后）

1) `xmake build test_ILP` 成功  
2) ILP-only：

```bash
./output/test_ILP <config_path>
```

3) ILP + MCF：

```bash
./output/test_ILP <config_path> --enable-mcf-routing
```

4) ILP / MCF with pre-routing warm start：

```bash
./output/test_ILP <config_path> --enable-pre-routing
./output/test_ILP <config_path> --enable-mcf-routing --enable-pre-routing
```

```bash
./output/test_ILP <config_path> --enable-mcf-routing --enable-mcf-obj
```

可选：显式传入与 Interposer 一致的 COB 行列（行为应与省略该参数相同）：

```bash
./output/test_ILP <config_path> --cob-rows <H> --cob-cols <W> --enable-mcf-routing
```

（将 `<H>`/`<W>` 替换为当前 `Interposer::COB_ARRAY_HEIGHT` / `COB_ARRAY_WIDTH` 的数值。）

注意：`--enable-mcf-parallel` 对 SimpleMCF per-unit 求解生效。`--enable-mcf-obj` 仅影响 SimpleMCF 目标函数。`--enable-pre-routing` 传入的是 warm start，不改变 ILP/MCF 的硬约束。

5) 若改了模型结构，建议附带：

- MPS/LP 导出样例
- 至少一个 case 的前后对比日志

---

## 10. 术语约定（本目录）

- **record**：`Net_cost_record`，2-pin 粒度建模单元
- **record_id**：record 在 `build_records` 输出中的全局唯一序号
- **bit_id**：同一 `origin_key` 内的位序号
- **origin_key**：原始 net 标识，用于回并
- **assignment**：ILP 输出的 record -> cobunit 结果
- **record_track_endpoint**：ILP 输出的 record -> `(cob_unit, has_start_track, start_track, has_end_track, end_track)` 结构
- **commodity**：MCF 中单一供需流对象（`PreparedCommodity`）
- **cobunit**：16 个布线资源分区之一（由 `map_track()` 规则定义）
- **track graph**：track 级全局路由图（`GlobalGraph`），节点粒度为 `(unit, dir, row, col, track)`，节点位于 COB 网格边界上
- **直通边**：同一 COB tile 内相对方向对（Left↔Right / Up↔Down）的边，`is_turn=false`
- **Wilton 转弯边**：同一 COB tile 内非相对方向对的边，`is_turn=true`，inner index 通过 Wilton 映射改变
- **BusMCF**：第一阶段求解，仅 `SyncNet in group {正整数}` commodity，带同步等长约束
- **SimpleMCF**：第二阶段，按 COBUnit 独立求解其余 commodity（含 `in_group_-1` 的 BumpToBumpNet、Tnet、TTB 等）；默认纯可行性，可选 `--enable-mcf-obj` 启用 `min Σ x`

**case5（`test/config/case5`）MCF 诊断预期**（`--enable-mcf-routing`）：`BusMCF commodities=80`、`bus_equal_length=64`（16 组 SyncNet：4×(8−1) + 12×(4−1)）；`BumpToBumpNet in_group_-1` 的 32 条记录在 SimpleMCF 中按 label 独立 Origin。
- **reach_steps**：Wilton 转弯步序列（`IlpReachStep`），描述 end_track 到 start_track 的转弯路径

术语尽量统一，不要在同一文档或代码注释里混用"子网/边/commodity/net"而不加限定。
