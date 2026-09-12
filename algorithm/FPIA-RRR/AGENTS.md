# PR_tool / algorithm/FPIA-RRR 工程指南

本目录实现独立的 FPIA `rip-up-and-reroute`（RRR）全局布线器，作为
`algorithm/test_ILP/` 中 SAT + ILP 方法的对比实验。方法规范见同目录 `spec.md`。

**当前状态：RRR 主循环、SyncNet 等长修复、TOB mux 端口独占与独立合法性校验已接通。**
不写 controlbits，不改 `Interposer` 寄存器。

## 工作流程要求

- 改代码前读清方法文档与 `source/hardware`、`source/circuit` 映射；**不允许修改方法文档**。
- 优先改动 `algorithm/test_ILP/`；非必要不改 `source/` 主流程。
- 修改后评估是否同步更新本文件（≤200 行）。
- 单次修改 >100 行时，启动子 agent 审查。
- 单文件职责紧凑，不超过 1000 行；关键步骤用 `debug::info_fmt` 打日志。
- 只可以在本git分支工作，绝对不可以切换到其它分支，或者将本分支的内容合并到其它分支。也不能使用git push推送到远程仓库
- 在与用户交流的过程中，如果涉及到关键的数据结构或者方法，就算用户没有问，也需要主动简要的解释设计思路，设计时需要注意程序的运行速度

## 目的与边界

- 与 `test_ILP` 使用相同配置输入、`source/` 解析器、`hardware::Interposer` 拓扑与
  wirelength 语义；仅路由策略不同。
- 每次运行只路由 `mode = 0`。不切换 mode，不加载旧路径，不做 incremental routing。
- 不调用 `suspend()` / `give_out()` / `connect()` / `PathPackage::connect_all()`。
- occupancy 只存在于 `ResourceModel`。不修改 `algorithm/test_ILP/`。不链接 CaDiCal/Gurobi。

## 目录结构

```text
algorithm/FPIA-RRR/
├── spec.md / AGENTS.md / main.cc / rrr_cli.hh/.cc / rrr_types.hh
├── net_adapter.* / hw_map.hh / hardware_graph.* / resource_model.*
├── maze_search.* / rrr_router.* / route_log.* / sync_equalize.*
├── route_validate.hh/.cc
└── test/unit_main.cc / tob_mux_fanout.* / tob_mux_cases.cc
```

## CLI、入口与计时

```bash
./output/FPIA_RRR <config_path> [-v|-vv] [-o DIR] [--max-iterations N] [--seed N]
xmake build FPIA_RRR
xmake build FPIA_RRR_unit && ./output/FPIA_RRR_unit
xmake build FPIA_RRR_mux_test && ./output/FPIA_RRR_mux_test
```

`-o` 写 `DIR/debug.log`。无 `-v` 也输出 spec §9 汇总行，并在整次布线结束时打印
全部 demand 路径。`-v`/`-vv` 另将级别设为 `Debug`（过程细节见下方「日志分层」）。
`main`：`Elapsed::start()` → parse/`build_nets` → `build_routing_nets` →
`build_hardware_graph` → `run_rrr` → 成功则 `validate_rrr_solution`。CLI
`max_iterations`/`seed` 写入 `RrrParams`。退出码 0 仅当 `status==success` 且
`best_overflow==0` 且独立校验通过；校验失败保留 `run_rrr` 已写统计并返回非零。

计时：`RRR_routing_time`/`routing_ms` 覆盖 initial maze + RRR 循环（不含 parse/图构建）；
`elapsed_ms` 为 `main` 入口到即将退出的墙钟（`Elapsed::milliseconds()`）。

## `run_rrr`

`run_rrr(graph, nets, params, interposer, verbose_level=0) -> RrrResult`。`interposer` 必须非空；
这保证 SyncNet 的等长检查不会被静默跳过。
`RrrResult`：`status`、`iterations`、`best_overflow`、`total_wirelength`、`paths`、
`routing_ms`。`paths[net][demand]` 为节点 id 序列。

Owner：普通 Bnet/Tnet/PNnet/fanout 为 `{net_id,0}`；SyncNet 成员为
`{net_id,demand_id}`。初始顺序：全部 bus owner（lane 数、组 HPWL、`net_id`，组内
`demand_id` 升序）再普通网（端口数、HPWL、id）。Dirty：congestion exposure、retry、
HPWL、id（除 id 外均降序）。HPWL 为 Track/Bump 终端包围盒；Bump 用 `tob_anchor_cob`。
PNnet 计入全部候选源。fanout/PNnet rip 整棵树后按 `demand_id` 升序重生。
`add_tree_node` 只插入 Track；后续 demand 不得从已占用 HLine/VLine 起步。

调度：一组的第一个 SyncNet owner 走 `route_sync_group`（maze 全 lane，sibling
物理资源 `hard_block`（Node/Switch/Matching/TobMux；Mode/BnetUnit 不互斥），再
`equalize_sync_group`）；已填 sibling 经 `routed_sync` 跳过。Dirty
任一 SyncNet 成员扩到整组后再 rip。overflow 0 且各组 `N_i` 相等才 success。

循环（spec §7）：analyze → legal best 字典序 `(overflow, total_wirelength)`（bus 须
等长）→ overflow 0 且等长则 success → `history_next` → 连续 4 轮无改进则
`H=min(H+4,16)` → dirty → 先全部 rip 再 reroute。`stagnation_limit` 后 `stagnated`；
用尽 `max_iterations` 则 `iteration_limit`。结束 restore best。maze 不可达 →
`unroutable`。Claim 使用 `path_resource_keys`（与 maze `arc_resource_keys` 同一投影），
Bnet 另 claim `bnet_unit_key`。`route_demand` 传入 `interposer` 做 NESW。

## 独立校验 `validate_rrr_solution`

`validate_rrr_solution(graph, nets, result, interposer=nullptr) -> bool`。

不读取路由器 live occupancy。用 `path_resource_keys` 把 `result.paths` claim 进
**新的** `ResourceModel`。失败时 `debug::error` 并返回 false。

检查（spec §10.4）：每条 demand 非空路径，末节点为 sink，首节点为候选源或同 owner
**树 Track**（不得以 HLine/VLine 为起点）；相邻节点在 `directed_arc_set`；
`collect_illegal_tob_fanout` 为空；Track/Bump/HLine/VLine、物理开关、matching、
TOB mux port 与 mode-conflict overflow 均为 0；Bnet 每 owner 至多一个
`selected_unit`；SyncNet 组成员 `sync_lane_length` 相等（需 `interposer`）且
Node/Switch/Matching/TobMux claimed key 互斥（Mode/BnetUnit 为兼容与 per-owner 锁，
不按跨 lane 独占）。

## SyncNet 长度与等长 API

`N_i`：按访问序计数 Track 节点，得到 `N_track`，再加 PathPackage TOB 端点常数：
Bnet +2，Tnet +1。组成员不同时混 Bnet/Tnet。

`sync_equalize.hh/.cc`：`sync_track_cut_index(Ni, r) = floor(Ni*(1-r))`；
`sync_lane_length(...)`；`equalize_sync_group(...)` 短 lane 按 `r={0.5,0.75,1.0}`
切尾。其他 SyncNet lane 硬阻塞；前缀与 parent chain 禁止回环。仅当全部 lane `N_i`
相等返回 true。

## Maze / 资源 / 图

`route_demand(..., tree={}, is_bnet=false, interposer=nullptr, hard_block={})`
Dijkstra，不 claim。`tree` 起点只收 Track。资源 cap-1 overflow；mode
straight+swap 冲突；Bnet `selected_unit`。wirelength：每 net 去重 Track+Bump。

TOB mux：同一 owner 对一个 mux port 只能使用一个 peer；Bump–HLine、HLine–VLine、VLine–Track 各投影
`TobMuxInput(node,peer)` / `TobMuxOutput(node,peer)`。同一 exact connection
（相同 extra）可被同 owner 复用；同一 mux port 指向不同 peer 则
`overflow = mux_distinct_peers-1`，即使 owner 相同。`key_is_free` 对 mux 只认
exact key。history 按 `(kind,id)` 共享。Track Steiner 主干仍可同 owner 共享；
禁止同一 VLine 接多个 HLine、同一 Track mux output 接多个 VLine。

## 缺省超参数（spec §6.1）

CLI 只暴露 `--max-iterations` 与 `--seed`；其余为编译期常数，全部写入 params 日志。
`type_weight`：node=1，switch/matching/mux=2，mode-conflict=8。

| 参数 | 缺省 | 含义 |
|---|---|---|
| `seed` | 1 | 只记日志；搜索与排序确定，不使用随机数 |
| `max_iterations` | 64 | overflow 迭代上限 |
| `stagnation_limit` | 8 | 字典序连续无改进则停止 |
| `H` | 4 | 拥塞高度（FastRoute `COSHEIGHT`） |
| `k` | 1.0 | logistic 陡峭度 |
| `s` | 20 | 超容量后的线性斜率 |
| `decay` / `increment` / `history_weight` | 0.9 / 1 / 1 | `history_next = decay×history + increment×overflow` |
| `detour_bias` | 0 | 首版关闭 |
| `sync_tail_extra_tracks` | 64 | tail maze 超出当前最长 lane 的 Track 预算 |
| `r` | 0.5, 0.75, 1.0 | SyncNet 切尾比例 |

`H/k/s` 进入 maze 的 present cost（`cap=1`；普通资源 `u` 为加入后的 owner 数，mux 用 distinct peers）：

```text
P(u) = 1 + H/(exp(k×(cap-u))+1) + [u>cap]×H/s×(u-cap)
present_cost = type_weight × P(u)
```

- 增大 `H`：空闲占用和冲突占用都更贵，冲突项按 `H` 放大，maze 更倾向绕开热点，线长往往变大。连续 4 轮字典序无改进时 `H ← min(H+4, 16)`。
- 增大 `k`：从“还能再挤一个”到“已经 overflow”的代价跳变更陡，更早避开将满资源。
- 减小 `s`：已经 overflow 时线性罚分 `H/s × (u-cap)` 更陡，更强力驱离热点；增大 `s` 则允许更长地挤占。

`initial overflow` 是全部网 maze 完、RRR 循环开始前的 `Σ max(0, owner_count-1)`
（外加 mode 冲突、Bnet 双 unit、同 owner 的 mux 多 peer）。为 0 则初解已合法，循环只打 `iter=0` 后成功退出。

## 日志分层

无 `-v`（`Info`）：spec §9 汇总行；`finish` 时按 RoutingNet 聚合打印最终路径。
块头为 `route net_id= name= kind= demands= wirelength= sources=[...]`，其中
`wirelength` 是该 net 内去重的 Track+Bump 节点数；不打印 `nodes`。普通多汇网在
同一块内按 `demand=/start=/sink=/path=[...]` 列出所有分支；SyncNet 在同一块内按
`lane=/source=/sink=/N_i=/path=[...]` 列出所有成员。过程中不提前 dump 路径。

`-v`/`-vv`（`Debug`，二者目前同级）：在过程中额外打印
- 每轮 RRR 的 overflow owner 集合（`overflow owners=[name,...]`；SyncNet lane 为
  `name#demand_id`，`demand_id` 是组内 lane 编号）
- SyncNet `equalize r= N_MAX= short_lanes=`、`sync tail maze ...`、`N_MAX raised`
不打印 `sync net_id=... N_i=... equal=`。`sync tail cutoff` 用 `info`，无 `-v`
也会出现。结束时仍打最终路径；`-v` 再追加一次结束时的 overflow 明细。

`iter=` 行每轮都会打（含初解已合法的 `iter=0`）。`dirty_owners` 是本轮计划拆掉的
owner 数（碰到 overflow 资源的网；SyncNet 任一成员脏则扩到整组）。`rerouted` 是
本轮实际重新 maze 的 owner 数。当前先按同一 `dirty_ids` 全部 rip 再 reroute，
成功 iter 行上两者通常相等；早退成功时都为 0。maze 中途 `unroutable` 不打部分计数的 iter 行。

## 测试矩阵（spec §10.3）

| 用例 | 内容 |
|---|---|
| `algorithm/test_ILP/test/case_2btb` | 两 Bnet |
| `algorithm/test_ILP/test/case_2btt` | 两 Tnet |
| `algorithm/test_ILP/test/case_2fanout` | 一 Tnet 两汇 |
| `algorithm/test_ILP/test/case_bus2btb` | SyncNet Bnet |
| `algorithm/test_ILP/test/case_bus2btt` | SyncNet Tnet |
| `test/config/case5` | PNnet/bus 混合，约 50s |
| `FPIA_RRR_unit` 第二层 | 5-lane SyncNet（初始 Track 数 9/7/6/5/3）与一条局部冲突普通网；验证整组 rip-up、history 绕行、等长与独立校验 |
| `tob_mux_fanout` | 同 owner 两 peer overflow；四 bump 非法 `VLine→HLine` tail 校验失败；合法 Track Steiner 通过；`run_rrr` 为四 bump 分配互斥 TOB access |
| `FPIA_RRR_mux_test` | 扫 `algorithm/test_ILP/test` 与 `test/config` 全部 `config.json`；多端口 net 无 illegal fanout |

单测另覆盖 claim/maze/RRR 排序、empty path 与跨 owner overflow 校验失败、合法短路径
通过。两层 SyncNet 测试均向标准输出打印初始候选和最终路径、每路径 Track 数与 `N_i`；
第二层还打印 RRR status/iteration/overflow。命令：`./output/FPIA_RRR_unit`。

## 日志字段（spec §9，必须出现）

```text
FPIA RRR: graph nodes=<N> arcs=<M> route_owners=<K>
FPIA RRR: params max_iterations=<N> seed=<S> H=<H> k=<k> s=<s> sync_tail_extra_tracks=<T>
FPIA RRR: initial overflow=<O> total_wirelength=<W>
FPIA RRR: iter=<I> overflow=<O> max_resource_overflow=<M>
          dirty_owners=<D> rerouted=<R> total_wirelength=<W>
FPIA RRR: status=<success|iteration_limit|stagnated|unroutable>
          iterations=<I> best_overflow=<O> routing_ms=<T> elapsed_ms=<E>
routing result: total_wirelength=<W> RRR_routing_time=<T> elapsed_ms=<E>
```

`iter` 行中 `overflow` / `max_resource_overflow` 是本轮 rip **前** 的值；若发生了
reroute，`total_wirelength` 是 rip **后** 的新线长。

命名空间 `PR_tool`；`std::Vector` / `std::String`。单测 Catch-free `require()`。
本文件 ≤200 行。
