# PR_tool / algorithm/FPIA-RRR 工程指南

本目录实现独立 FPIA rip-up-and-reroute（RRR）全局布线器，用作 test_ILP 的 baseline。
方法规范见 spec.md；当前主循环、SyncNet 等长、TOB mux 独占与独立校验均已接通。
不写 controlbits，不改 Interposer 寄存器。

## 工作流程要求

- 改代码前读清方法文档与 source/hardware、source/circuit 映射；不允许修改方法文档。
- 优先改动 algorithm/test_ILP；非必要不改 source 主流程。本 baseline 任务不改 test_ILP。
- 修改后评估是否同步更新本文件（≤200 行）。
- 单次修改 >100 行时，启动子 agent 审查。
- 单文件职责紧凑，不超过 1000 行；关键步骤用 debug::info_fmt 打日志。
- 只在当前 git 分支工作，禁止切换分支、跨分支合并、git push。
- 涉及关键数据结构或方法时主动简要解释设计，注意运行速度。

## 目的与边界

- 与 test_ILP 使用相同输入、解析器、Interposer 拓扑与 wirelength 语义，仅路由策略不同。
- 每次只路由 mode=0，不切换 mode，不加载旧路径，不做 incremental routing。
- 不调用 suspend/give_out/connect/PathPackage::connect_all。
- occupancy 只存在于 ResourceModel，不修改 source 主流程；不链接 CaDiCaL/Gurobi。

## 目录结构

入口：main.cc / rrr_cli.* / rrr_types.hh；图与映射：net_adapter.* / hw_map.hh /
hardware_graph.*；路由：rrr_router.*（外层调度）、rrr_routing.*（原辅助函数拆分）、
maze_search.* / resource_model.* / sync_equalize.*；日志与校验：route_log.* /
route_validate.*；测试：test/unit_main.cc / adapter_cases.* / rrr_budget_cases.* /
tob_mux_fanout.* / tob_mux_cases.cc。

## CLI、入口与计时

```bash
./output/FPIA_RRR <config_path> [-v|-vv] [-o DIR] [--max-iterations N] [--seed N] [--time-budget-seconds S]
xmake build FPIA_RRR
xmake build FPIA_RRR_unit && ./output/FPIA_RRR_unit
xmake build FPIA_RRR_mux_test && ./output/FPIA_RRR_mux_test
```

-o 写 DIR/debug.log。无 -v 也输出 spec §9 汇总及最终全部 demand 路径。
-v/-vv 均设为 Debug。main：Elapsed::start → parse/build_nets → build_routing_nets →
build_hardware_graph → run_rrr → 成功则独立 validate_rrr_solution。
CLI max_iterations/seed/time_budget_seconds 写入 RrrParams，budget_start 为入口计时起点。
退出码 0 仅当 status=success、best_overflow=0、独立校验通过，否则非零。

routing_ms/RRR_routing_time 覆盖参考预布线（启用预算时）、initial maze + RRR，不含 parse/建图。
elapsed_ms 使用 Elapsed::milliseconds()。最终 total_ms 与 test_ILP/main.cc 同口径：
日志初始化后、配置解析前开始，包含解析、建图、全部布线、最终路径输出和独立校验。
预算起点与 total_ms 相同；直接调用 run_rrr 未传 budget_start 则从 routing 开始。

## run_rrr 与外层调度

run_rrr(graph,nets,params,interposer,verbose_level=0) → RrrResult；interposer 必须非空。
结果含 status/stop_reason、iterations/optimization_rounds、best_overflow、
unequal_sync_groups/total_sync_gap、total_wirelength/paths/routing_ms。
first_legal_ms、best_legal_ms、budget_elapsed_ms 相对预算起点；未有合法解的前两者为 -1。
paths[net][demand] 为节点 id 序列。

Owner：普通 Bnet/Tnet/PNnet/fanout 为 {net_id,0}，SyncNet lane 为 {net_id,demand_id}。
初始顺序：bus（lane 数、组 HPWL 降序、net_id 升序，组内 demand_id 升序）再普通网
（端口数、HPWL 降序、id 升序）。Dirty：exposure、retry、HPWL 降序，id 升序。
HPWL 为终端包围盒，Bump 使用 tob_anchor_cob；PNnet 计入全部候选源。
fanout/PNnet rip 整棵树后按 demand_id 重生；树起点只收 Track，不得从 HLine/VLine 起步。

SyncNet 第一个 owner 调用 route_sync_group：全 lane maze 后 equalize；sibling 的
Node/Switch/Matching/TobMux 硬阻塞（Mode/BnetUnit 不互斥），已填 sibling 经 routed_sync 跳过。
Dirty 任意 lane 扩到整组后先全部 rip 再 reroute；目标增长时整组回滚并按新目标重新等长。
overflow=0、各组 N_i 相等、全部连接才是候选合法解，还须独立校验通过才保存。

完整 live state 按 (overflow,unequal_sync_groups,total_sync_gap,total_wirelength)
字典序保存诊断 best；另单独保存经过独立校验的最好合法解，仅在线长严格降低时更新。
快照同时保存 owners 与完整 ResourceModel，结束优先恢复最好合法解。
history_next（统一 decay，默认 0.9）→ 原 overflow dirty 集+不等长组 → 排序 → 全 rip → reroute；每连续 4 轮无改进则 H=min(H+4,16)
并重置停滞计数。无预算时保留 first-legal 停止、H=16 后 stagnation_limit 停止，
maze 异常则 unroutable。Claim 用 path_resource_keys，与 maze arc_resource_keys 同投影；
Bnet 另 claim bnet_unit_key，route_demand 传入 interposer 做 NESW。

时间预算 S>0：初始布线及每轮完成后 checkpoint，记录当前时间。有历史合法解且时间
达到预算则停止；无合法解超时仍继续，首次找到合法解后停止。预算模式不因 stagnation
退出。max_iterations 仍是安全上限：触顶有合法解返回 success，无合法解返回 iteration_limit。
默认 S=0 关闭预算。预算不硬中断单次 maze/equalize，允许一轮执行及最终输出/校验造成超时。

启用预算时先为每个 net 独立预布线：每次新建空 ResourceModel（无其他 net、无 history），
调用原 route_owner / route_sync_group，保留自身硬件约束与 SyncNet 组内互斥/等长。
只记录独立校验通过的参考线长 L_ref（net 内去重 Track+Bump），不提交临时路径到正式解。
这是无外部竞争的 maze 参考值，不是严格最优下界；失败记日志，该 net 不参与额外优化。
预布线耗时计入 routing_ms、total_ms 与预算；无预算时不做参考预布线。

额外优化只改外层调度：当前完整解合法且尚有预算时，仅选 L_current > 1.1×L_ref 的
net（严格超过 10%，恰好 10% 不选）。普通多汇网整体拆、SyncNet 整组拆，其余保持不动。
对所选集合先全部 rip，再调用原 route_owner / route_sync_group；每轮把筛选后初始排序
的起点循环移动一个 net/组，不拆组、不改组内 lane 顺序。无候选时立即返回最好合法解，
stop_reason=no_optimization_candidates，不回退到全量重布，不等待预算耗尽。
新解冲突/不等长时进入原 dirty 修复（不受线长筛选限制），合法后再优化。预算停止仍以
历史 best_legal 为条件，live 非法但已有 best_legal 时超时直接返回它，不等待下一次合法。
预算模式 maze 异常时恢复完整 best（无则清空）并整轮重试；异常轮不保存部分路径。
单次 maze、资源投影、同步组等长算法不变。

## 独立校验 validate_rrr_solution

不读取 live occupancy，使用 path_resource_keys 将结果 claim 到新的 ResourceModel。
失败时 debug::error 并返回 false；需 interposer 才能检查 SyncNet N_i。
检查：每 demand 路径非空、sink 正确、首节点为候选源或同 owner 树 Track（非 H/VLine）、
相邻节点属于 directed_arc_set、collect_illegal_tob_fanout 为空；
Track/Bump/HLine/VLine、物理开关、matching、TOB mux port、mode-conflict overflow 均为 0；
Bnet 每 owner 至多一个 selected_unit；SyncNet N_i 相同且 lane 物理 claimed key 互斥。
Mode/BnetUnit 是兼容/per-owner 锁，不作为跨 lane 独占资源。

## SyncNet 长度与等长 API

N_i：按访问序计数 Track 节点，加 PathPackage TOB 端点常数：Bnet +2、Tnet +1。
同组不得混 Bnet/Tnet；sync_track_cut_index(N_i,r)=floor(N_i*(1-r))。
equalize_sync_group 短 lane 按 r={0.5,0.75,1.0} 切尾，每次整组固定同一 target。
某 lane 只找到更长 N_f 时，整组回滚，以实际 N_f 更新 target 后重来；普通不可达不盲目
target+1。sibling 硬阻塞，前缀及 parent chain 禁回环。仅全部 N_i 精确相等才提交。

## Maze / 资源 / 线长

route_demand(graph,resources,owner,sources,sink,params,tree={},is_bnet=false,
interposer=nullptr,hard_block={}) 使用 Dijkstra，不 claim；树起点只收 Track。
容量为 1；mode straight+swap 冲突；Bnet selected_unit 锁。wirelength 按每 net 去重
Track+Bump 再求和，不统计 HLine/VLine，不按 SyncNet 等长 N_i 求和。

TOB mux 同一 owner 对一个 port 只能用一个 peer；Bump–HLine、HLine–VLine、
VLine–Track 投影 TobMuxInput(node,peer)/TobMuxOutput(node,peer)。
相同 exact connection/extra 可复用，不同 peer 的 overflow=distinct_peers-1（即使同 owner）。
key_is_free 对 mux 只认 exact key，history 按 (kind,id) 共享。Track Steiner 可同 owner
共享；禁止 VLine 接多个 HLine、Track mux output 接多个 VLine。

## 缺省超参数

CLI 只暴露 max_iterations/seed/time_budget_seconds，其余编译期常数。
type_weight：node=1、switch/matching/mux=2、mode-conflict=8。

| 参数 | 缺省 | 含义 |
|---|---|---|
| seed | 1 | 只记日志，搜索/排序确定，无随机数 |
| max_iterations | 1000000000 | 修复与优化轮次安全上限 |
| time_budget_seconds | 0 | 关闭预算，正数启用软预算 |
| optimization_excess_percent | 10 | 编译期筛选比例，严格大于参考线长的 110% |
| stagnation_limit | 8 | 仅无预算时 H=16 阶段连续无改进停止 |
| H / k / s | 4 / 1.0 / 2 | 拥塞高度 / logistic 陡峭度 / overflow 线性斜率 |
| decay / increment / history_weight | 0.9 / 1 / 1 | 所有轮次统一用配置 decay；history_next=decay×history+increment×overflow |
| detour_bias | 0 | 关闭 |
| sync_tail_extra_tracks | 64 | target 超出初始最长 lane 的 Track 预算 |
| r | 0.5,0.75,1.0 | 切尾比例 |

P(u)=1+H/(exp(k×(cap-u))+1)+[u>cap]×H/s×(u-cap)，present_cost=type_weight×P(u)。
cap=1，普通 u 为加入后的 owner 数，mux 用 distinct peers。H 增大促使绕热点且线长常增；
k 增大代价跳变更陡；s 减小 overflow 惩罚更陡。H 连续停滞后依次尝试 4/8/12/16。

## 日志与测试

初始 overflow 是 Σmax(0,owner_count-1)，另含 mode、Bnet 双 unit、同 owner mux 多 peer。
无预算且初解合法时打 iter=0 后退出。保留 spec §9 graph/params/initial/iter/status/
routing result 汇总字段；params 追加 time_budget_seconds；status 追加 stop_reason、
optimization_rounds、first_legal_ms、best_legal_ms、budget_elapsed_ms。
stop_reason 是退出原因；触及预算/上限但有合法解时 status 仍为 success。
checkpoint 记录每步时间与 incumbent；最终入口打印 total_ms/time_budget_seconds/over_budget_ms。
iter 中 overflow/max_resource_overflow 为 rip 前值；new_overflow/wirelength 为 rip 后值；
dirty_owners 是计划 rip 的 owner 数，rerouted 是调度 owner 数（不累计等长内部 tail 搜索），成功轮通常相同。
初解早退两者为 0；maze 异常不打印部分计数 iter 行。params/iter 均记录统一的实际 history_decay。

Info：最终按 RoutingNet 聚合 dump 所有路径，不提前 dump。route 块头含
net_id/name/kind/demands/wirelength/sources；普通分支 demand/start/sink/path，
SyncNet lane/source/sink/N_i/path。wirelength 是本 net 去重 Track+Bump，不打印 nodes。
Debug：每轮 overflow owners=[name,...]（lane 用 name#demand_id）、equalize r/target/limit/
short_lanes、sync tail maze、sync target advance；结束再打印 overflow 明细。
sync tail cutoff 用 Info；不打印 sync net_id=... N_i=... equal=。

完整单测 ./output/FPIA_RRR_unit；--synthetic-only 跳过配置 adapter；
--budget-only 仅跑预算与超容量代价回归。配置尺寸必须匹配当前 Interposer。
配置矩阵：case_2btb/case_2btt/case_2fanout/case_bus2btb/case_bus2btt/case5（PNnet/bus）。
另测 claim/maze/排序/线长、非法路径/overflow、合法短路径、TOB mux fanout/Steiner/access；
5-lane SyncNet（Track 数 9/7/6/5/3）+局部冲突验证整组修复/history/等长/原子 target 更新。
SyncNet 测试打印初始/最终路径、Track 数与 N_i，第二层打印 status/iteration/overflow。
mux_test 扫 algorithm/test_ILP/test 与 test/config 的全部 config.json，检查非法 fanout。
预算回归：初解超时、无解超时修复、优化/修复交替、冲突 trial 后恢复最好合法解、
安全上限、禁用 stagnation、整组 SyncNet、实际预算停止；另测参考隔离、fanout/PNnet
共享树线长、严格 10% 边界、选择性重布、无候选提前退出；超容量代价验证 s=2 的线性项
比 s=20 强十倍，未超容量代价不变（H=4/16）。

## TODO

增强超容量项（已实现，case6/15 收敛效果待实验）：所有迭代统一 s=20→2，将 H/s×overflow
增强十倍，u≤1 代价不变；统一 decay=0.9、H 上限16。可能增加绕行/耗时，须单独对照验证。

命名空间 PR_tool；std::Vector / std::String；单测 Catch-free require()；本文件 ≤200 行。
