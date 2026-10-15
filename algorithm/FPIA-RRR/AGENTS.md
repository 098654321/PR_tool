# PR_tool / algorithm/FPIA-RRR 工程指南

本目录实现独立 baseline：同步长度预计算 → 原始顺序 maze 初始化 →
事务式 RRR 补布/优化。普通 net 参照 test_ILP 的 route_rrr/route_search；
同步组保留 FPIA 的整组目标增长与切尾等长。spec.md 是旧版方法记录，不修改该文件。

## 工作流程要求

- 修改范围限 algorithm/FPIA-RRR；不改 test_ILP、source 或根 xmake.lua。
- 改代码前读方法文档及 source/hardware、source/circuit 映射。
- 单次修改 >100 行时启动独立子 agent 审查；单文件不超过 1000 行。
- 当前分支 dev.ILP_RRR_add_iteration；未经用户要求不切换、合并或 push。
- 关键中间步骤使用 debug::info_fmt；保持搜索速度并验证硬件合法性。
- 修改持久约定时同步更新本文件，保持 ≤200 行。

## 边界与构建

与 test_ILP 共用配置解析、真实 Interposer 拓扑及 Track+Bump 并集线长。
只运行 mode=0，不加载旧路径、不切换 mode，不调用 SAT/ILP 或写 controlbits。
只有原始 maze 初始化临时调用 source 路由器的 suspend/give_out。
初始化结束（包括失败/异常）清理 package 并重置临时寄存器；后续 RRR 占用仅在
ResourceModel 中维护，绝不调用 connect_all。不得将预留状态带入后续 RRR。

```bash
xmake build FPIA_RRR
xmake build FPIA_RRR_unit && ./output/FPIA_RRR_unit
xmake build FPIA_RRR_mux_test && ./output/FPIA_RRR_mux_test
./output/FPIA_RRR CONFIG [-v|-vv] [-o DIR] [--max-iterations N] [--seed N] [--time-budget-seconds S]
```

-o 写 DIR/debug.log；默认输出到当前目录。-v/-vv 打开 Debug。
不链接 CaDiCaL、Gurobi 或 HiGHS；复现事务算法而不调用 ILP 求解器。
所有新模块在本目录维护；initial_route.hh、transaction.hh、transaction_search.hh
为头文件模块，因此无需修改目录外的构建列表。

## 入口与计时

main：parse/build_nets → build_routing_nets/build_hardware_graph → run_rrr →
最终独立校验。保留 raw_nets，并传给 run_rrr 的末尾可选参数。
生产 CLI 必须走真实原始 Sort/Resources/Maze；未传 raw_nets 的路径仅供合成图单测使用，
用严格资源阻塞的图搜索作为初始布线测试接缝，不宣称与原始 BFS 选路相同。

total_ms 与 test_ILP 入口同口径：日志初始化后、配置解析前开始，包含解析、建图、
同步最短长度预计算、原始 maze、RRR、最终路径输出和独立校验。
routing_ms/RRR_routing_time 包含预计算、初始化与 RRR，不含解析、建图和最终路径输出。
budget_start 与 total_ms 起点相同；直接调用 run_rrr 未设置则从 routing 开始。

退出码 0 仅当 status=success、missing_owners=0、best_overflow=0 且完整独立校验通过。
零 overflow 的部分解不能报告成功。

## 同步组共同长度初始化

正式布线前，每个 SyncNet 的每个 2-pin lane 独立搜索，不考虑任何其他 net/lane
的资源占用。只按 Track 节点数最短搜索，仍保持真实邻接、Bnet unit 及单路内部合法性。
不得用拥塞加权 cost 最小或图 arc hop 最小代替最短同步长度。
N_i=按访问次序计数 Track + TOB 常数，Bnet +2、Tnet +1；同组不得混合 Bnet/Tnet。
初始目标 L0=max_i Lmin_i。只写 OwnerRecord::sync_target，不 claim 预计算路径，
不更新 history。日志逐 lane 输出 shortest N_i，逐组输出 initial target。

正式同步等长首先尝试预计算目标；不直接以普通 maze 实际最长长度覆盖它。
只有切尾搜索找到更长候选时，才按实际长度推进整组目标。
r={0.5,0.75,1.0}，保留 floor(N_track*(1-r)) 个 Track 的前缀再搜索尾段。
出现更长 N_f 时整组恢复快照，以实际 N_f 更新目标后重来，不盲目 target+1。
保留 sibling 物理硬阻塞、前缀/parent-chain 禁回环及内部 mux/mode 合法检查。
搜索上限仍为 sync_tail_extra_tracks=64、尾部 labels 至多 80000。
组路径与 sync_target 只随成功事务一起提交；失败整体回滚，不能泄漏长度更新。

## 原始顺序 maze 初始化（initial_route.hh）

直接调用 source/algo/router 的 Sort::execute 和 Resources::execute，
严格保留 priority 更新、端口数归一化及原始比较器；不额外按 HPWL 或 ID 重排。
按此顺序逐条执行原始 Net::route(MazeRouteStrategy)；普通 net 使用原始 BFS。
SyncNet 使用 initial_sync_route.hh 在本目录适配原始私有 sync_preroute/BFS，
保持 COB unit 候选排序、btb→ttb→btt 预布线顺序及其他同步 Track 端口保护。
导入每条 lane 后，若全部达到预计算共同目标，原样保留，不进入等长调整；
否则从这些原始路径开始 FPIA 切尾等长，不重新执行 FPIA 首次 maze。
不调用原始 SyncNet 的另一个等长循环；不以预布线最长长度覆盖预计算目标。

成功 package 转成统一图中的完整 source→sink 路径，包括 COB 和 TOB 三层连接。
fanout/PNnet 从 package 的实际连接并集重建每个 demand，不能直接拼接扁平 regular_path。
导入后按 RRR 资源规则检查完整合法部分解；不可共享的跨 owner 资源仍不允许共享。
首次无路或导入结果非法时，恢复此前成功前缀，立即停止原始阶段；
失败 net 的部分分支/同步 lane 不保留，后续未尝试 net 也交给 RRR。
不继续尝试剩余原始 maze，不再使用旧的软拥塞初始补布。
原始初始化结束后清理所有临时硬件预留，包括失败 fanout 尚未写回 package 的预留。

## 事务式 RRR（transaction.hh / transaction_search.hh）

已提交状态始终为零 overflow 的完整 owner 集合，允许部分 owner 尚未布通。
普通 net 为整棵树 owner={net_id,0}；SyncNet lane 各有独立 owner，
但调度、冲突挤出和目标变化以整个同步组作为原子事务单元。

每 sweep 先按原始排序处理缺失 net/组，再处理已连接者：
1. 种子尝试 6 个变体，事务内先拆种子，再搜索新路径。
2. 找到与新路径冲突的其他 owner/组，拆除并加入 displacement 队列。
3. 被挤出对象至多重搜 4 次，选择不触及已锁定对象且冲突对象最少的候选。
4. 新挤出对象继续入队，成功者锁定；失败则放弃整个试探。
5. 只有零 overflow、完整 owner/组合法且未布通 owner 数减少，
   或未布通数不变且总线长严格下降，才提交整个事务。

普通候选参照 test_ILP：完整物理 source 路径，轮换 demand/unit/等代价邻接顺序，
每 source/unit 最多 32 次内部合法性去边回退，每次搜索最多 300000 扩展。
Track unit 与 VLine local unit 筛选与 test_ILP 一致；不做随机扰动或旧路径加价。
同步组调用现有 FPIA maze+tail equalize，变体 1/2/3 分别优先 50%/75%/100% 切尾。
不使用旧 dirty 排序、全量拆线优化、轮换全网起点或停滞后提高 H。

## Cost 与资源规则

每 arc 基础代价 1，新增资源加 type_weight*P(u)+history_weight*history：
P(u)=1+H/(exp(k*(1-u))+1)+[u>1]*(H/s)*(u-1)。
生产缺省 H=4、k=1、s=20，不自适应提高 H；decay=0.9、increment=1、
history_weight=1。Node 权重 1，switch/matching/mux 为 2，mode conflict 为 8。
同 owner 树/exact mux 复用不重复收费，mux 用 distinct peer 占用。
每次试探从已提交状态复制占用，路径生长与同步前缀保留立即更新事务占用。
sweep 末只按已提交状态 history_next=0.9*history+overflow；
失败试探不积累历史。提交状态零 overflow，因此从零开始的 history 一直为零。

容量资源含 Track/Bump/HLine/VLine、开关和 matching；straight/swap 是兼容约束，
Bnet unit 是 per-owner 锁，不作为跨同步 lane 独占资源。
同 owner mux port 不允许多个 peer，SyncNet lane 不能共享物理资源。
独立校验重建全部占用，检查完整 demand、端点、图邻接、简单路径、物理并集无环、
TOB fanout、unit、mode、容量及同步长度，不依赖 live occupancy。
allow_partial 仅允许缺少整个 net/同步组，不能跳过残缺 owner。

## 终止、incumbent 与日志

max_iterations 默认 1000000000，定义为外层 sweep 安全上限，不是单次 maze 次数。
正预算时取消原 test_ILP 的 2 sweep/无改善提前停止，持续尝试至预算或安全上限。
到预算且已有完整合法解：在安全检查点停止并返回线长最小的完整合法 incumbent。
到预算仍无完整合法解：继续，第一次找到完整合法解即停，但仍受 max_iterations 约束。
无预算时沿用 test_ILP 的最多 2 sweeps/无改善停止；合法初解仍会尝试事务优化。
预算不硬中断单次搜索/事务；最后一次事务及路径输出/校验可能导致软预算超时。
每次接受后检查并保存完整合法解；接受条件保证部分解按 (missing,wirelength) 单调改进。
触顶没有完整解时返回合法部分结果与非成功状态，不能伪装成 success。

保留 graph/params/initial/iter/status、routing result、最终全部路径日志与 CLI 字段。
新增 initial interrupted position/net、sync shortest/initial target、transaction accept、
history committed_overflow/total、missing_owners；first_legal_ms/best_legal_ms 保持原口径。
iter 的 overflow/new_overflow 现在是提交状态，通常为 0；dirty_owners 为计划事务单元数，
rerouted 为种子试探次数，accepted 为提交次数（不再表示旧全量拆线计数）。
paths[net][demand]、total_wirelength、RRR_routing_time、total_ms 及最终 route 块保持不变。
线长仍按每逻辑 net 去重 Track+Bump 求和，不计 HLine/VLine，不直接累加同步目标。

## 回归测试

FPIA_RRR_unit 支持 --synthetic-only、--budget-only；断言使用 require，Release 有效。
覆盖独立最短同步目标（包括忽略互相占用、长度而非 cost）、目标增长整组回滚、
原始真实 Sort 的端口偏置、首次失败前缀和寄存器清理、事务补布/优化/失败回滚、
原始 Btb/Btt/Ttb 同步初始路径在共同目标下原样保留及临时预留清理、
预算超时补布、零 overflow 部分解失败、安全上限和最好合法解单调保留。
真实矩阵：case_2btb、case_2btt、case_2fanout、case_bus2btb、case_bus2btt、test/config/case5。
配置需匹配当前 Interposer 宽度；不得为测试越界修改目录外硬件配置。
命名空间 PR_tool，使用 std::Vector/std::String，单测使用 Catch-free require()。
