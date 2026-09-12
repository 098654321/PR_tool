# FPIA-RRR：基于 Rip-up-and-Reroute 的对比实验规范

## 1. 目的与边界

本目录实现一个独立的 FPIA `rip-up-and-reroute`（RRR）全局布线器，作为
`algorithm/test_ILP/` 中 SAT + ILP 方法的对比实验。

两种方法必须使用相同的配置输入、相同的 `source/` 解析器、相同的
`hardware::Interposer` 物理拓扑，以及相同的 wirelength 统计语义。两者仅在
路由搜索与资源分配策略上不同。本实现只负责按本文档完成 RRR 求解与日志，
不规定实验时选用 SAT 还是 SAT+ILP 作为对照列。

本实现的边界如下：

- 使用 `parse::read_config()` 和 `algo::build_nets()`，不执行布局。
- 每次运行只路由一个固定 mode（`mode = 0`）。实验输入不含多 mode，也不支持
  mode 切换、旧路径加载、跨 mode 复用或 incremental routing。
- 不调用 controlbits writer，不生成配置文件，不调用 `PathPackage::connect_all()`。
- 不在路由过程中修改 `Interposer` 的寄存器状态；不调用 connector 的
  `suspend()`、`give_out()` 或 `connect()`。
- 在指定输出目录保留 `debug.log`；默认输出到当前目录的 `debug.log`。
- 保持现有 `algorithm/test_ILP/` 与其他 `algorithm/` 目录不变。本实现所需的
  逻辑网适配、硬件图构建、路径日志和 wirelength 工具均复制到本目录维护。
- 搜索全图。不使用 SAT 的 `pair_bbox` / `delays` 裁剪。

## 2. 入口与数据流

新增独立 xmake 目标 `FPIA_RRR`，建议命令行形式为：

```bash
xmake build FPIA_RRR
./output/FPIA_RRR <config_path> [-v|-vv] [-o DIR] [--max-iterations N] [--seed N]
```

数据流：

```text
config folder (single mode only)
  -> parse::read_config(config_path, 0, false)
  -> algo::build_nets(basedie, interposer)
  -> FPIA-RRR copied net adapter
  -> FPIA-RRR copied hardware routing graph
  -> initial maze routing
  -> iterative RRR
  -> debug.log only
```

`-o DIR` 创建目录并将日志写为 `DIR/debug.log`。本程序不加载已有
controlbits，也不写出任何 controlbits。

计时口径：

- `RRR_routing_time` / `routing_ms`：从 initial maze 开始到 RRR 主循环结束
  （含 SyncNet 等长修复），不含配置解析、`build_nets` 与图构建。
- `elapsed_ms`：从 `main` 入口到进程即将退出的端到端墙钟。

两者都写入 `debug.log`。

## 3. 本目录文件组织

```text
algorithm/FPIA-RRR/
├── spec.md
├── main.cc                 # CLI、日志初始化、parse/build_nets、总计时
├── rrr_cli.hh/.cc          # 命令行选项
├── rrr_types.hh            # 路由结果、route owner、路径与统计类型
├── net_adapter.hh/.cc     # 从 test_ILP 复制并独立维护的 RoutingNet 构建
├── hardware_graph.hh/.cc  # 从 test_ILP 复制并独立维护的统一物理图
├── hw_map.hh/.cc          # 从 test_ILP 复制的坐标/TOB 线性编号辅助
├── resource_model.hh/.cc  # 物理资源 claim/release、overflow、history
├── maze_search.hh/.cc      # 带 congestion/history 代价的 Dijkstra
├── rrr_router.hh/.cc      # initial route、rip-up、reroute 主循环
├── route_log.hh/.cc       # 路径、资源统计、wirelength 与 debug 输出
└── test/                  # RRR 专属单测与集成用例（后续新增）
```

根目录 `xmake.lua` 增加 `FPIA_RRR` 与 `FPIA_RRR_unit` 目标；不链接 CaDiCal
或 Gurobi。

初始复制来源与职责：

| FPIA-RRR 副本 | 参考来源 | 保留内容 |
|---|---|---|
| `net_adapter.*` | `test_ILP/scope/build_routing_nets.*` | `circuit::Net` 到 Bnet/Tnet/PNnet/SyncNet 的适配 |
| `hardware_graph.*` | `test_ILP/graph/unified_routing_graph.*` | COB track mesh、TOB bump/HLine/VLine 图、物理开关与 mode group 标识 |
| `hw_map.*` | `test_ILP/common/hw_map.*` | Track/Bump/TOB 坐标与线性编号 |
| `route_log.*` | `test_ILP/sat/routing_path_log.*` | 节点格式化、路径日志、去重 wirelength 统计 |

复制后的代码必须去除 SAT、CaDiCal、ILP、scope-feedback、VirtualSource 与
`augment_graph_for_pnnet`；不会反向修改 `test_ILP` 中的文件。统一图只保留
硬件实际存在的邻接 arc。

## 4. 路由图与逻辑网

### 4.1 路由图

路由图是有向图，节点包括：

- `Track`：COB 之间的轨道；
- `Bump`、`HLine`、`VLine`：TOB 内部三级可编程网络；
- RRR 不显式创建 SAT 使用的 `VirtualSource` 节点。

图中的 arc 表示可用物理连接。TOB arc 保留：

- `physical_switch_id`：同一可编程开关的双向 arc 共用同一编号；
- `physical_switch_kind`：`BumpH`、`HLineVLine`、`VLineTrack`；
- `mode_group_id` 与 straight/swap 标志：末级 VLine--Track 的配置兼容性。

PNnet 不需要虚拟源节点：其候选 track 直接作为 multi-source Dijkstra 的起点。

maze 只沿硬件合法邻接扩展。统一图由 `Interposer` 拓扑构建，搜索时不得额外
施加 SAT 的 `cob_unit_mask` 过滤。Tnet 从固定源 Track 出发，只能走到硬件已经
接上的邻接边，因此不会进入未连接的 unit。Bnet 仍按 4.3 在首次经过
VLine--Track 后锁定 unit。

### 4.2 Route owner

资源冲突的 owner 不是一概等同于 `RoutingNet`：

- 普通二端网、普通 fanout 和 PNnet：同一个 net 内允许共享已经形成的树，
  owner 为 `net_id`。
- `SyncNet` 的各成员是独立信号，不能因其被包装在同一个 `RoutingNet` 而共享
  物理资源；owner 为 `(net_id, demand_id)`。

每个 owner 保存其 demand path、树节点/arc、已 claim 的资源及引用计数。引用计数
保证一条网内多分支共用资源时，rip-up 一个分支不会错误释放其他分支。

### 4.3 特殊网语义

- Tnet：固定 Track 或 Bump 源到 Bump 汇；多汇 Tnet 增量生长一棵共享树。
- Bnet：Bump 到 Bump。每个 owner 第一次经过 VLine--Track arc 时固定一个 track
  unit；之后只允许使用该 unit，等价于 SAT 的 `Q` 选择约束。
- PNnet：所有 candidate source track 均为同一 owner 的根。每个 sink 可连接到任一
  根，允许多个候选 root 实际被使用。
- SyncNet：每个成员独立路由，并最终满足精确路径长度等长。这里的“长度”不使用
  统一图 arc hop 数，而是按访问顺序计数 Track 节点；若成员端点包含 bump，再按现有
  `PathPackage::_length` 规则加入固定的 TOB 端点长度。由于当前 SyncNet 不混合
  Bnet 与 Tnet，组内所有成员的端点常数相同，比较该完整长度等价于比较 Track 节点数。

## 5. 物理资源模型

`resource_model` 维护局部、可回滚的 occupancy，不改变 `hardware` 对象。

每条 path 在提交时把其节点和 arc 投影为若干 `ResourceKey`；同一 owner 对同一
资源的重复 claim 只计一次容量占用。

| 路由图对象 | ResourceKey / 约束 | 容量或规则 |
|---|---|---|
| `Track`、`Bump`、`HLine`、`VLine` 节点 | node key | 不同 owner 容量 1 |
| TOB 物理开关 | `physical_switch_id` | 不同 owner 容量 1 |
| Bump--HLine | bump endpoint 与 HLine endpoint | 各自 partial matching 容量 1 |
| HLine--VLine | HLine endpoint 与 VLine endpoint | 各自 partial matching 容量 1 |
| VLine--Track | `mode_group_id` | 所有使用者必须同为 straight 或同为 swap |
| Bnet 的末级 arc | owner 的 selected unit | 首次选择后固定 |

HLine/VLine 节点占用与 matching、开关约束同时生效：即使两个 owner 使用不同
开关，也不能共享同一条 HLine 或 VLine 节点。

对每个容量资源，`overflow = max(0, owner_count - 1)`。mode group 若同时存在
straight 与 swap 使用，则该组为冲突资源，所有冲突模式的 owner 都要进入候选
rip-up 集合。

每轮维护 `history_cost[resource]`：资源 overflow 时增加惩罚，非拥塞资源按固定
衰减率降低。该机制迁移 FastRoute 的 congestion history，但以 FPIA 可编程资源为
粒度而非二维 grid edge 为粒度。

## 6. Maze 搜索

搜索采用 Dijkstra；图不规则且无 admissible 的统一 Manhattan 启发式，因此首版不
使用 A* 启发项。只扩展统一图中已有的硬件邻接 arc；静态不存在的 arc 一律不可用。

对候选 arc 的增量代价定义为：

```text
C(a | owner) = base_cost(a)
             + Σ present_cost(r, predicted_owner_count(r))
             + Σ history_weight(r) × history(r)
             + detour_bias(a)
```

其中求和范围是候选 arc 新增 claim 的物理资源；当前 owner 已拥有的树资源不重复
收费。四部分的定义如下：

1. `base_cost(a)`：基础路径代价。首版所有可路由 arc 均取 1，使无拥塞时的 maze
   优先较短路径；后续可按硬件时延为 TOB/COB arc 设置不同常数，但不会改变资源
   合法性。
2. `present_cost(r, u)`：当前预测拥塞。`u` 是候选 owner 加入后的不同 owner 数，
   普通资源容量 `cap=1`。参考 FastRoute 的 logistic cost：

   ```text
   P(u) = 1 + H / (exp(k × (cap - u)) + 1)
            + [u > cap] × H / s × (u - cap)
   present_cost(r, u) = type_weight(r) × P(u)
   ```

   `H` 是拥塞高度，`k` 是 logistic 陡峭度，`s` 是超容量后的线性斜率。空闲资源
   被使用时已有中等代价；被另一 owner 使用后再选用时产生有限但显著更高的代价。
3. `history_weight(r) × history(r)`：长期热点惩罚。每轮 RRR 后更新

   ```text
   history_next(r) = decay × history(r) + increment × overflow(r)
   ```

   其中 `0 < decay < 1`。因此即使当前 momentarily 不 overflow，反复成为瓶颈的
   Track、switch 或 matching endpoint 仍会被后续 maze 主动避开。
4. `detour_bias(a)`：可选的紧凑性偏置。首版取 0；若启用，仅对离开当前端点
   bounding box 的 Track 加很小代价，用来在拥塞代价相同的候选间避免无必要绕远。
   它不参与 SyncNet 的等长判定。

VLine--Track mode group 不是普通容量 1 资源：同一 mode 的多条使用可共存，只有
straight 与 swap 同时出现时产生 mode conflict。实现中为这种反向组合建立虚拟冲突
资源，并使用高 `type_weight` 的 present/history 罚分；最终校验要求该冲突为零。

搜索时应执行以下动态合法性检查：

- Bnet owner 已选定 unit 后，不能使用其他 unit；静态不存在的 arc 一律不可用；
- 已属于当前 owner 的树节点为零附加占用，可作为 multi-source 起点；
- 对容量资源和 VLine--Track mode 冲突，初始路由与 RRR 中均允许产生有限的暂态
  overflow，但在代价中给予强惩罚；只有最终 overflow 为零才是合法解；
- 对 PNnet，以所有 candidate source track 和已形成树节点作为起点；
- 对 fanout，以已形成树节点作为起点，得到增量 branch。

相同 `C(a | owner)` 时，Track 邻居按 clockwise 打破平局：对当前 Track 的相邻
COB 按 North → East → South → West 扫描，同一 COB 内保持
`Interposer::adjacent_connectors` 的既有顺序；TOB 内部弧保持建图顺序。

初始路由允许软拥塞，确保所有可达 demand 都有初解；RRR 轮次通过逐步增大的
present/history 罚分消除 overflow。若某 demand 在图上不可达，则整次运行立即失败
（`status=unroutable`），不生成不完整结果。

### 6.1 首版默认超参数

数值按 OpenROAD FastRoute maze 主循环的首轮量级选取，全部写入日志。CLI 只暴露
`--max-iterations` 与 `--seed`；其余为编译期常数。

| 符号 | 缺省 | 来源与用途 |
|---|---|---|
| `max_iterations` | 64 | CLI 可覆盖；overflow 迭代上限 |
| `stagnation_limit` | 8 | 字典序解连续无改进则停止 |
| `sync_tail_extra_tracks` | 64 | tail maze 超出当前最长 lane 的额外 Track 节点预算 |
| `seed` | 1 | 写入日志；首版搜索与排序完全确定，不使用随机数 |
| `H` | 4 | FastRoute `COSHEIGHT` |
| `k` | 1.0 | logistic 陡峭度 |
| `s` | 20 | FastRoute maze `slope` |
| `decay` | 0.9 | FastRoute `last_usage *= 0.9` |
| `increment` | 1 | overflow 资源每轮累加 |
| `history_weight` | 1 | history 项系数 |
| `detour_bias` | 0 | 首版关闭 |
| `r` 序列 | `0.5, 0.75, 1.0` | SyncNet tail 拆除比例 |

`type_weight(r)`：

- Track / Bump / HLine / VLine 节点：`1`
- TOB 物理开关、matching endpoint：`2`
- VLine--Track mode 冲突虚拟资源：`8`

连续 `stagnation_h_boost = 4` 轮字典序未改进时，令 `H ← min(H + 4, 16)`。这对应
FastRoute“若本轮未改善则增强惩罚”。

## 7. RRR 主循环

初始阶段先路由所有 `BusRouteGroup`，再路由普通 owner；这是论文中“bus routing
must be completed in advance”的约束。

排序键一律降序，最后用稳定 id 升序打破平局：

- Bus group：lane 数、HPWL、`net_id`。组内 lane 按 `demand_id` 升序。
- 普通 owner：端口数（source 数 + demand 数）、HPWL、稳定 owner id。
- 被 rip-up 的 dirty owner：congestion exposure、retry count、HPWL、稳定 id。

HPWL 定义为 owner 全部终端（source 与 sink 的 Track/Bump 坐标）包围盒
`(row_max - row_min) + (col_max - col_min)`。PNnet 计入全部候选源。congestion
exposure 定义为该 owner 当前已 claim 资源的 `overflow` 之和。retry count 为该
owner 被 rip-up 的次数。

`--seed` 只记录到日志，首版不参与打乱。

```text
best_solution = initial_solution
for iter in [0, max_iterations):
    overflow = analyze_overflow()
    save_best_if_improved(overflow, wirelength)
    if overflow == 0:
        return success

    update_history_cost(overflow)
    maybe_increase_H()
    dirty_owners = owners_touching_overflow_resources()
    expand SyncNet members to whole BusRouteGroup
    ripup(dirty_owners)
    sort dirty_owners by congestion exposure, retry count, HPWL, stable id

    reroute each dirty owner with congestion/history-aware Dijkstra
    repair_sync_length_if_needed()
    if no improvement for stagnation_limit:
        stop

restore best_solution
return success only if best_solution overflow == 0
```

`save_best_if_improved` 使用字典序 `(overflow, total_wirelength)`。overflow 更小
视为改进；overflow 相同且 wirelength 更小也视为改进。stagnation 使用同一字典序。

fanout 与 PNnet 的 owner 是整棵树：rip-up 拆除该 owner 的全部 branch，再按
`demand_id` 升序重新增量生长。不得只补一条脏分支而留下旧树。

与 FastRoute 的对应关系：保留 overflow 驱动循环、选择性 rip-up、history cost、
动态重排与最优解保留；不迁移二维容量表、Steiner tree split/reconnect、hyper edge、
via cost、详细布线及布局回调。

## 8. SyncNet 等长修复

SyncNet 的成员以独立 owner 搜索，并由一个 `BusRouteGroup` 原子调度。实现遵循
论文 Algorithm 2 与现有 `MazeRouteStrategy::sync_preroute/sync_reroute` 的策略，而
不是为所有成员从头执行固定长度搜索。

### 8.1 初始路由

先以普通、congestion-aware maze 为每个 lane 求一条完整路径，并计算其 Track 节点
长度 `N_i`。令：

```text
N_MAX = max_i(N_i)
```

所有 `N_i < N_MAX` 的短 lane 进入 equalization 队列；长度等于 `N_MAX` 的 lane
保留不动。

### 8.2 Tail rip-up and reroute

对一个短 lane `p_i = (t_1, ..., t_Ntrack)`，使用 reroute ratio `0 < r < 1`，首选
`r = 0.5`，并令：

```text
N_r = floor(N_track × (1 - r))
```

保留从 source 到 `t_Nr` 的前缀，释放 `t_Nr` 之后的 Track、COB connector、sink
TOB access 与对应局部资源 claims。从 cut Track 开始进行 tail maze：

- 维护完整 parent chain；当前 lane 已保留的前缀及该 chain 中的祖先一律不可再次
  使用，禁止以回环凑长度；
- 同一 SyncNet 的其他 lane 视为硬阻塞，确保各 lane 不共享物理资源；
- 其他已提交普通网保持 RRR 的软 congestion/history 代价；
- 搜索队列改为按第 6 节 `C(a | owner)` 的 min-heap，而不是论文原始 FIFO；相同
  cost 时按第 6 节 clockwise 邻居顺序打破平局；
- 每个 label 同时维护当前访问的 Track 节点数；为避免无界绕路，该数超过
  `N_MAX - TOB_constant + sync_tail_extra_tracks` 时停止扩展，并在 `debug.log`
  记录一次 tail cutoff。缺省 `sync_tail_extra_tracks=64`。

当搜索到 sink 的合法 TOB access 时，计算前缀与候选 tail 拼接后的完整长度 `N_F`：

- `N_F < N_MAX`：该候选不能完成等长，继续扩展；
- `N_F == N_MAX`：提交该 lane，处理下一条短 lane；
- `N_F > N_MAX`：保留该 lane 为新的最长路径，更新 `N_MAX = N_F`，重新收集并处理
  所有仍短于新 `N_MAX` 的 lane。

若 tail maze 无解，按 `0.5 -> 0.75 -> 1.0` 扩大被切除的比例并重试；只有全部比例
都失败时，才将整个 `BusRouteGroup` 标为本轮失败并交给外层 RRR 处理。

### 8.3 与 RRR 的交互

当任意 SyncNet 成员触及全局 overflow 资源时，先 rip-up **整个** BusRouteGroup，
再依次执行“普通初始 maze -> `N_MAX` -> tail equalization”。整组成功前不处理下一
个 dirty owner；成功后才一次性提交全部成员的 resource claims。每轮结束都校验所有
成员的 Track-segment 长度均为同一个 `N_MAX`。

## 9. 日志与实验指标

必须通过 `debug::initial_log()` 写 `debug.log`。至少记录：

```text
FPIA RRR: graph nodes=<N> arcs=<M> route_owners=<K>
FPIA RRR: params max_iterations=<N> seed=<S> H=<H> k=<k> s=<s> sync_tail_extra_tracks=<T>
FPIA RRR: initial overflow=<O> total_wirelength=<W>
FPIA RRR: iter=<I> overflow=<O> max_resource_overflow=<M>
          dirty_owners=<D> rerouted=<R> total_wirelength=<W>
FPIA RRR: status=<success|iteration_limit|stagnated|unroutable>
          iterations=<I> best_overflow=<O>
          routing_ms=<T> elapsed_ms=<E>
routing result: total_wirelength=<W>
                RRR_routing_time=<T> elapsed_ms=<E>
```

`total_wirelength` 沿用 SAT 路径统计：每个逻辑 net 内去重计数 `Track + Bump` 节点，
再跨 net 求和；TOB 内部 HLine/VLine 和 virtual root 不计入。

布线结束后始终用 `info` 按 RoutingNet 聚合打印最终路径：块头记录 net 类型、全部
逻辑 source、demand 数和该 net 的去重 `wirelength`；普通多汇网在块内列出每个 sink
的分支路径，SyncNet 在块内列出全部 lane 与其 `N_i`。建议在 `-v` 下记录每个 overflow
资源的 owner 集合，以及 SyncNet equalize / tail maze 过程行。`-vv` 下再记录 maze
搜索的 explored nodes、route cost 和 history contribution。

## 10. 验证与完成标准

实现按以下顺序验证：

1. 单元测试资源 `claim/release`、同 owner 分支共享、跨 owner overflow、mode 冲突、
   partial matching、HLine/VLine 节点独占与 Bnet unit 锁定。
2. 单元测试 maze 搜索：无拥塞最短路、history 绕行、rip-up 后资源回收、PNnet
   multi-source、fanout tree 生长。
   另有确定性 SyncNet tail-reroute 单测：一条短 lane 必须拆尾并改走精确等长替代路径。
3. 合成 `run_rrr` 集成单测：5-lane Tnet SyncNet 的初始 Track 数为 `9/7/6/5/3`，
   后四条各有 9-Track 等长替代路径；一条普通网初始占用 lane 0 的中间 Track。测试
   要求出现至少一轮 overflow RRR、任一 lane 的 dirty 状态扩展为整个 BusRouteGroup、
   普通网因 history 改走独立绕路、全部 lane 最终 `N_i=10`，并通过独立 validator。
4. 集成测试 `algorithm/test_ILP/test/case_2btb`、`case_2btt`、`case_2fanout`、
   `case_bus2btb`、`case_bus2btt` 与 `test/config/case5`（PNnet）。
5. 对每个成功结果执行独立合法性检查：所有 demand 连通、无 Track/Bump/HLine/VLine
   overflow、无 TOB switch 冲突、所有 matching 合法、mode group 一致、Bnet unit
   一致、SyncNet 等长。
6. 日志同时提供 `RRR_routing_time` 与 `elapsed_ms`，便于后续对比实验读取。

`FPIA_RRR_unit` 的两层 SyncNet 测试会打印初始候选路径与最终路径，行内含 Track
节点数和 `N_i`；第二层同时打印最终 `status`、`iterations` 与 `overflow`，用于人工
核验绕行和等长结果。

RRR 只有在最终 `best_overflow == 0` 且全部合法性检查通过时才报告成功；否则保留
最佳中间解统计并返回非零退出码。
