# PR_tool / algorithm/test_ILP 工程指南

本目录实现统一细粒度 SAT 路由及其优化前端。默认流程是第十四版 CaDiCaL 可行性路由；`--z3-optimize` 是第十六版 Z3 Weighted Partial MaxSAT；`--global-route-v17` 是稠密 `W` 的 HiGHS Global Routing 后接 Z3 Detailed Routing；`--global-route-v18` 将第一层改为无稠密 `W` 的容量剪切，第三层改为 CaDiCaL 纯 SAT，并已接入第十九版的 fixed-unit 初始精确容量行、maze-routing MIP start 和固定 bbox+1 Global Routing scope。方法定义以 `../../问题定义与方法/第十七版方法.md`、`第十八版方法.md` 和 `第十九版方法.md` 为准。

## 当前四条入口

- 默认：统一图 + D/A 精确距离状态 + CaDiCaL assumptions，UNSAT core 驱动 bbox/distance 扩展。
- `--z3-optimize`：原 CNF 全部作为 hard constraints，所有 pair alpha 作为 external assumptions，以物理 Track/Bump 占用 `U_v` 的单位软约束最小化并集线长。
- `--global-route-v17`：自动启用 Z3 Optimize；先用 HiGHS 在 Channel 图上联合选择 COBUnit、MCF route guide 和 bus Channel 数等长，再以 guide 和 unit assumption 初始化详细求解。
- `--global-route-v18`：先用小型 HiGHS 模型为 PNnet bump 预选物理 source/unit，并按 source 转换为 fixed-source multi-sink Tnet；正式 HiGHS 前端将第十四版每个子连接的 bbox 各外推 1 格并取并集，只为 scope 内资源建立 `F/X`及关联约束；随后建立不需要 `W` 的 fixed-unit 精确容量行，再用受同一 scope 限制的 2-pin/单源多-pin maze routing 生成部分 MIP start，未覆盖的 Channel--unit 资源由迭代容量剪切补齐；详细阶段不生成 occupancy `U`、`D⇒U` 和 soft objective，直接流式送入 CaDiCaL，首个可行解即返回。当前不扩展 Global Routing bbox，bbox+1 下 HiGHS 失败会直接返回错误。

旧 `-L/-R` 与 `ilp_v15/` Gurobi refinement 已删除。第20版重新使用
`--ilp-optimize` 名称显式启用新的 HiGHS SAT 后 refinement；默认 V18 不运行该阶段。
`--maze-optimize` 启用 SAT 后局部 Maze/RRR，二者互斥且都要求
`--global-route-v18`。

## 第十七/十八版共享流水线

1. `build_routing_nets` 归一化 Bnet、Tnet、PNnet、fanout 和 SyncNet。
2. `build_unified_graph` 构造真实 Track/TOB/COB 细粒度图。V17/普通流程随后由 `augment_graph_for_pnnet` 加 PN virtual source；V18 在调用它之前完成 PN 预选和 Tnet 转换，因此转换成功后不会建立 PN virtual source。
3. `build_global_channel_graph` 使用显式 COB/terminal 节点和物理 Channel 边资源。普通 Channel 连接相邻 COB；TOB 挂接节点插在其下方 Channel 两个 COB 之间，两个半段共享一个 `channel_id`；实际出现的 external/01 port 建立私有 terminal 节点，并保留 42 个 boundary terminal。V18 先用包含全部 PN 候选端口的临时 Channel 图运行小型 HiGHS 0--1 模型，按 `(PNnet,bump,unit,physical-source)` 预选端点，再把每个非空 `(PNnet,physical-source)` 组转成 fixed-source/unit 的 multi-sink Tnet；正式 MCF 使用转换后的 nets 重建 Channel 图，从而删除未选 source 的 port 节点。
4. HiGHS MIP 使用：
   - owner/unit 变量 `Q`，普通 bump net 可选 16 unit，external track 固定 `map_track(track)`；
   - owner/Channel 占用 `X`；
   - V17 为非固定 unit 创建稠密 `W=X∧Q`；V18 不创建 `W`，首次求解前对潜在 fixed owner 数超过 8 的 `(Channel,unit)` 加入 `sum X<=8`，然后对整数 incumbent 中其余超载资源迭代加入 9-owner cut `sum(X+Q)<=17`；
   - V18 在首次 HiGHS 求解前用 Global Routing 图上的 maze routing 生成部分 MIP start：2-pin owner 取新增物理 Channel 数最少的路径，单源多-pin owner 从已有树反复连接 Channel 增量最小的 sink，TOB 同一 Channel 的两个半段只计一次；只提交已确定的 `F/X/source-choice=1`，其余变量由 HiGHS 补全；
   - 每个 per-pair/commodity 使用带 `channel_id` 的二进制拓扑弧流 `F`，port 弧只对对应 commodity 建变量；
   - V17 基线的 PNnet 保留 per-demand owner、`Q/F/X/S/Z`；V18 预选后的 PNnet 已是 fixed-source/unit Tnet，正式 MCF 不再产生 PN `S/Z`，每棵 physical source-tree 以其 `X` 独立计长和占用容量；
   - 节点 flow conservation、terminal Channel、`F_a⇒X_{channel(a)}` 与 `X⇒incident F/source`；
   - `(Channel,unit)` 容量不超过 8；
   - 每 TOB/unit load 不超过 8、每 TOB/bank/residue load 不超过 8；
   - 2-pin SyncBus members 的 `sum X` 相等。
5. V17 基线目标最小化非 PN owner 的 `sum X` 与 PNnet 的 `sum Z`；V18 则对预选后的每棵 physical source-tree 直接累加 `sum X`，不同 source-tree 使用同一 Channel 仍分别计长和占用资源。宏观模型不增加 MTZ/无环约束；无用 `X/Z` 由目标和双向 support 约束排除，`F` 在已选 Channel 内允许环。
6. `apply_global_route_v17` 写入 per-pair 非矩形 Channel guide、per-source unit 和由选中宏观弧数加端点开销得到的 detailed distance cap。随后，每个 pair 只在自己的 TOB 端点加入固定局部修补：非 TOB--TOB pair 使用两侧相邻 COB 的两行三列 9-Channel 模板（中央 TOB Channel、上下边界 Channel、四条横向和左右两条纵向）；TOB--TOB pair 使用紧凑 7-Channel 模板（中央 TOB Channel、上下两个纵向 Channel、上下相邻 COB 各两条横向 Channel），不含左右外侧纵向 Channel。物理边界外的不存在 Channel 自动裁剪。`PairRoutingState` 保留各 pair 自己的 guide，详细 SAT scope 才取同一 net 所有 pair guide 的并集。
   - V17 基线的 multi-sink PNnet 直接从每个 demand 的 `S/F` 恢复 source 和路径；V18 转换后的 Tnet 直接从预选物理 source 建立 guide，详细 SAT 不再开放 PN virtual-source 候选。
7. `compute_pair_delays` 在 guide（含初始 TOB 修补）的细粒度投影中求 `d_min`；V18 预选后的 Pose/Nege physical-source tree 首轮 domain 为 `{d_min,d_min+1,d_min+2}`，其余普通 pair 仍为 `{d_min}`，SyncBus 共享 `{max(member d_min)}`。`detailed distance cap` 保留为 Global Routing 诊断，不能扩大首轮详细 distance domain。
8. Bnet 的 Global Routing unit 通过可追踪 assumption `gamma⇒Q_sat(unit)` 固定；不写不可撤销 unit clause。
9. Z3 或 V18 CaDiCaL hard-UNSAT 时分别处理：
   - alpha core：critical pair 每次扩一个 distance；每个 pair 独立计数，累计4次 distance-only 失败后的第5次，在保留本次 distance 扩展的同时，只把该 pair 的局部 guide 扩一跳；TOB--TOB pair 使用相同阈值。详细 SAT 使用同一 net 所有 pair 局部 guide 的并集，不同 demand 的失败不互相累计；
   - gamma core：只取消 core 中对应 Bnet/source 的 unit 固定，并在原 guide 内开放全部 16 unit；
   - 非 core net 的 unit 保持不变。
10. V17 Z3 Optimal 后校验 `objective == reconstructed union wirelength`；V18 CaDiCaL SAT 后使用同一提取与物理合法性校验，仅把 `total_wirelength` 作为后验统计。

第20版：仅 `--global-route-v18` 在 Global Routing apply 和 TOB repair 后、详细 SAT 前打印按 `PairKey` 重建的 raw selected guide。日志必须区分有序 source--target walk、`residual_selected_arcs`（环/分支/非唯一流）、TOB repair 增量和 final pair scope；Sync 额外打印每 member 的 raw Channel 数及 min/max。该日志的 wall time 写入 `SatRoutingResult::excluded_diagnostic_ms`，不能计入 Global Routing、SAT 或 `run_main total elapsed`。SAT 得到完整合法解后，只有 `--ilp-optimize` 才用 HiGHS 联合优化显式标记的 `TrackToBump(s)` 和 PN physical-source tree；`BumpToTrack`、`Sync`、`BumpToBump` 固定。优化固定 physical source/COBUnit，使用 SAT 树拆出的 segment、`bbox(R_ILP=0) ∩ final SAT scope` 稀疏域和 parent-level `x/y` 资源并集目标，完整 SAT 解作为 MIP start，gap 为 1.5%；冻结资源冲突、提取/硬件校验失败或 detailed wirelength 退化时必须原样回退 SAT 解。`--maze-optimize` 则按 stretch 逐个 rip-up 非 Sync 整棵树；Sync 为硬障碍，其他非 Sync 允许临时 overflow，dirty owner 按 FPIA-RRR 先全拆再逐个重布。每个 owner 限制在自己的 final SAT scope 并固定端点/source/unit；局部 RRR 不收敛、校验失败或总 detailed wirelength 不严格下降时事务式回滚。

第一层只编码必要条件，不能保证 TOB mux、Wilton lane、跨 COB lane 一致性或详细资源互斥可解；最终 Z3/CaDiCaL hard model 才是物理可行性证明。V17 的第一次 Optimal 只保证当前 guide/domain 内最优；V18 只返回当前 guide/domain 内的第一个可行解，两者都不声称完整硬件图上的全局线长最优。

V18 PN 预选中的 TOB 固定负载只包含已有 Tnet：external/01 track 决定 unit，连接到它的每个物理 bump 按所在 TOB、bank 和 unit/residue 分别计数。Bnet 的 unit 尚未确定，不进入该固定负载。预选使用距离、source-tree 激活代价和 unit-aware COB-grid RUDY，是启发式端点固定而不是可行性证明；若后续正式 MCF 因该分配 Infeasible，只能说明当前预选失败。当前实现不会自动释放 PN source/unit 后重跑预选。

SyncBus 的 Channel-count 等长是用户选定的宏观代理约束，不是细粒度 exact-distance 等长的数学必要条件。因此 `GLOBAL_ROUTE_Infeasible` 只表示 V17 前端未生成 guide，不能报告整个设计物理无解。当 guide 和 distance 未达到完整域时，反馈轮数耗尽统一返回 `SEARCH_LIMIT`，也不报告全局 `UNSAT`。

## 目录职责

| 路径 | 职责 |
|---|---|
| `common/` | RoutingNet、结果统计、COBUnit/硬件坐标映射 |
| `scope/` | net 聚合、pair 状态、bbox 与 Channel guide 状态 |
| `graph/` | 统一细粒度图与 PN virtual source |
| `global_route_v17/` | Channel 图、V18 PN source/unit 预选与 Tnet 转换、HiGHS MCF、结果提取、guide 应用/扩展 |
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

第十八版纯 SAT 运行：

```bash
./output/test_ILP <config_path> --global-route-v18 -v -o <output_dir>
```

可选 SAT 后优化（二选一）：

```bash
./output/test_ILP <config_path> --global-route-v18 --ilp-optimize -v -o <output_dir>
./output/test_ILP <config_path> --global-route-v18 --maze-optimize -v -o <output_dir>
```

`--time-limit MIN` 只限制正式 Global Routing 的 HiGHS 墙钟时间（分钟）；省略或未设置表示不限时。容量剪切多轮重求解共用同一预算。超时若已有可行 incumbent 则继续提取 guide，否则报失败。PN 预选不受该参数约束。

`--global-route-v17/--global-route-v18` 不与 `-s/-d` 联用，因为 guide 和 distance cap 已由第一层初始化。V18 不与 `--z3-optimize` 联用。普通 `--z3-optimize` 和默认 CaDiCaL 流程仍支持原 `-s/-d`。

合成单测必须至少覆盖：

- 普通 2-pin net 恰好一个 unit、terminal 连通和 Channel 目标重算；
- external track fixed unit；
- PN reachable candidate source/unit；per-demand `X/Q/F/S` 必须保留；net-level `Z` 必须等于全部 demand `X` 的 Channel 并集，并对目标中的共享 Channel 去重；
- V18 PN 预选必须覆盖 Tnet 固定 bump 对 TOB unit/bank-residue 的逐 bump 占用、unit-aware RUDY 对等距离 source 选择的实际影响、`lambda_A=lambda_A_base*k_hat` 的 physical-01-port 平均树规模缩放，以及 PNnet 按物理 source 转换为 fixed-source multi-sink Tnet；
- SyncBus member Channel 数等长；
- fixed/released unit 的 guide lane 开放范围；
- `gamma` assumption 冲突能出现在 failed core；
- 容量剪切在无拥塞 case 中保持相同 objective 且 `W=0`；9 个 fixed-unit owner 共用 Channel 时必须由首次求解前的精确容量行直接判定不可行；另外覆盖可选 unit/source 超载在加 cut 后改到可行解并收敛；
- maze MIP start 必须分别覆盖 2-pin 和单源多-pin owner，记录提交的 commodity/变量数，且不改变原 MIP objective 和最终容量校验结果；
- bbox+1 Global Routing scope 必须验证无关 `F/X` 变量被实际删除，以及唯一绕路在 scope 外时直接返回 Infeasible，不进行自动扩展；
- V18 合成 case 必须完成 HiGHS guide 并由 CaDiCaL 在无 `U`/无 soft clauses 的 hard CNF 上找到可行解；TOB repair 单测必须覆盖内部 TOB 的9/7-Channel 模板、pair-local guide 与整网 union、首轮 singleton domain 和所有 pair 第5次反馈阈值；scoped-path 不可达时必须携带准确 `PairKey`，只扩展该 pair 所属 net；
- V20 post-SAT maze 合成测试必须覆盖 scope 内直接缩短、Sync 硬障碍不变，以及 trigger 产生 non-Sync overflow 后 dirty owner 全拆并收敛到更短合法解；
- 既有 TOB/COB/SAT、路径提取、bus detailed 等长和 Z3 objective 不变量。

无需用 `test/config` 真实 case 作为第十七版的基本回归。

## 日志规范

关键阶段使用 `debug::info_fmt`，字段稳定、可统计：

- `V17 Global Routing graph`：COB/TOB/port/boundary 节点数、physical_channels、directed_traversal_arcs、collapsed_track_nodes；
- `V18 PN source preselection model built`：PNnet/bump/去重 physical-01-port/candidate 数，`Y/A/O` 变量规模、五类约束、Tnet 固定 bump 数、`lambda_A_base/k_hat/lambda_A/lambda_R/alpha` 与 build 时间；`-v` 时输出变量和约束分类；
- `V18 PN source preselection summary`：status、source-tree 数、转换后 net 数、objective、total/build/solve 时间；
- `prepare`：nets/owners/demands/PNnets/buses；
- `model built`：vars/constraints/build_ms 与 `capacity_mode`；
- `V19 fixed-unit capacity initialization`：首次求解前建立的 fixed-unit 精确容量行数、fixed owner 数和 Channel 数；
- `V19 maze MIP start`：已路由 owner、2-pin/multi-pin owner、commodity、跳过 owner、提交变量数及构造时间；
- `V19 Global Routing scope`：`full-graph` 或 `bbox-plus-one`，以及 `X/F` 的 active slots、dense slots 和裁剪比例；
- `V17 Global Routing ILP model stats (-v)`：图节点/Channel/owner/demand 维度，`Q/X/Z/W/F/S` 变量分解，15 类线性约束及与 HiGHS 总数的一致性；
- per-owner/per-pair（`-v`）：net、owner、unit、Channel 数、selected arcs；
- `-vv`：终端回显 HiGHS 求解日志；无论是否 `-vv`，PN 预选与正式 Global Routing 都会把求解日志写入 `-o` 目录下的 `highs.log`（求解过程中逐行 flush）；
- `-vvv`：额外打印 scope child bbox；
- `validation`：objective、最大 Channel-unit load、pair 数；
- V18 capacity cuts：每轮 overloaded Channel--unit 数、新增/累计 cuts、累计 solve ms，以及收敛时的 rounds/cuts/final constraints；
- `summary`：status、vars、constraints、objective、estimated_wirelength、total/build/solve ms；
- Z3 每轮：alpha/unit assumptions、soft 数、core 分类、release/guide expansion；V18 CaDiCaL 每轮记录 hard clauses、alpha/unit assumptions、`occupancy_soft_clauses=0`、core 分类与 expansion；Global Routing guide 初始化额外记录 TOB repair 的 net/TOB/Channel 数，反馈记录 net 类型、连续 distance failure、阈值和 scope expansion；
- main 汇总：`global route` 与 summary 相同字段，SAT 规模/耗时，最终 wirelength。
- V20 guide：`global guide` 的 source/target/unit、selected Channel/COB/arc 数、有序 walk、residual、TOB repair 增量和 final scope；结尾打印 `body_log_ms`，并在 `run_main total elapsed` 中打印 raw/excluded 值。
- V20 post-SAT ILP：目标/fixed net 数、segment bbox 资源槽、locked node/switch 数，`F/X/Y/M` 变量、九类约束、warm-start 提交规模、每目标 net 与总体 wirelength 改善、gap/耗时、校验状态或 fallback 原因。
- V20 post-SAT maze：non-Sync owner 数、每个 trigger 的 stretch/状态/回滚、每轮 overflow/dirty owner/线长，以及 trigger/接受/RRR 迭代/累计重布 owner/最终线长汇总。

不要把 Global Routing Channel objective 记为 detailed wirelength，也不要把 Channel 数直接用作 SAT distance。

## 修改要求

- 改动前核对方法文档和 `source/hardware` 映射；优先只改 `algorithm/test_ILP/`。
- 不修改 `source/algo/router/` 的正式路由流程。
- 关键约束必须有合成单测；不要依赖大 case 才暴露基本建模错误。
- 单文件保持紧凑，避免无关重构；新增关键步骤保留日志和规模/耗时统计。
- 单次修改超过 100 行时，在实现完成后启动独立 reviewer 子 agent。
