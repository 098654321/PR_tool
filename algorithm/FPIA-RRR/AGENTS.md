# PR_tool / algorithm/FPIA-RRR 工程指南

本目录实现独立 AIR-style baseline，依据用户 note.md 的场景适配：
独立参考布线 → 空实际解 → 增量补布 → 合法性判断/最好解保存 →
更新 OpenROAD 节点 cost → 冲突剪枝 → 仅当前合法时主动线长拆线 → 下一轮。
不再采用旧原始 maze warm start、事务式 displacement RRR 或旧 FPIA 外加 history 的 cost。
spec.md 是旧版方法记录，不修改该文件。

## 工作流程要求

- 修改范围限 algorithm/FPIA-RRR；不改 test_ILP、source 或根 xmake.lua。
- 改代码前读本文件及 source/hardware、source/circuit 映射。
- 单次修改 >100 行时启动独立子 agent 审查；单文件不超过 1000 行。
- 当前分支 dev.ILP_RRR_add_iteration；未经用户要求不切换、合并或 push。
- 关键中间步骤使用 debug::info_fmt；保持搜索速度并验证硬件合法性。
- 修改持久约定时同步更新本文件，保持 ≤200 行。

## 边界与构建

共用配置解析、真实 Interposer 拓扑及 Track+Bump 并集线长。
只运行 mode=0，不加载旧路径、不切换 mode，不调用 SAT/ILP 或写 controlbits。
真实 source Sort/Resources 只用于确定顺序，不执行原始 Route、不预留硬件；
实际占用全部由 ResourceModel 管理，不调用 suspend/give_out/connect_all。

```bash
xmake build FPIA_RRR
xmake build FPIA_RRR_unit
./output/FPIA_RRR_unit
xmake build FPIA_RRR_mux_test
./output/FPIA_RRR_mux_test
./output/FPIA_RRR CONFIG [-v|-vv] [-o DIR] [--seed N] [--time-budget-seconds S]
```

-o 写 DIR/debug.log；默认输出到当前目录。-v/-vv 打开 Debug。
--max-iterations 已删除，不再接受该参数；没有 convergence 上限。
必须为有限时实验提供正时间预算；省略/设 0 表示禁用预算，没有自动终止条件。
--seed 保留兼容解析/记录；不额外引入随机选路。
不链接 CaDiCaL、Gurobi 或 HiGHS。
air_search.hh / air_routing.hh 为头文件模块，无需修改目录外构建列表。

## 入口、顺序与计时

main：parse/build_nets → build_routing_nets/build_hardware_graph → run_rrr →
最终独立校验。传入 BaseDie::nets().at(0) 原生构建顺序 raw_nets。
生产入口直接执行 source 的 Sort::execute 和 Resources::execute，
严格保留 priority 更新、端口数归一化及其比较器；整个过程保持该顺序。
未传 raw_nets 的路径仅供合成图单测使用，按类型优先级排序。

total_ms 与 test_ILP 入口同口径：日志初始化后、配置解析前开始，
包括解析、建图、独立参考布线、AIR 迭代、最终路径输出和独立校验。
routing_ms/RRR_routing_time 不含解析、建图和最终路径输出。
budget_start 与 total_ms 起点相同；直接调用 run_rrr 未设置则从 routing 开始。
first_legal_ms/best_legal_ms 相对预算起点记录。

## INIT：参考长度与空间 bins

每个 net 分别在空资源模型中完整 maze route 一次，记录其实际 Track+Bump
并集长度作为 reference；它不是理论下界。同步组独立布线并完成等长后记录组总线长。
参考结果不 claim 到正式占用，不更新正式 history，也不作为实际初始路径。
若参考搜索失败，记录 unavailable，不用零值代替。
正式 owners/current 从空解开始。

每个 net 按全部 terminal bbox 建立正方形 bins：
bin_area=4*Area(bbox)/num_sinks，实际 bin_size=1+sqrt(bin_area)，不取整。
Area 按坐标差值相乘；退化维度至少按 1。bin 原点取原始 terminal bbox 左下角，
不使用搜索框的外扩面积，后续扩框也不改变 bin_size。
每次路径变化后同步刷新当前 tree 和 bins。

## 每轮增量布线

1. 按固定排序处理 net，只重连尚未布通的 sink；已经合法连通的 sink 不重搜。
2. 起点来自当前保留树和允许的原始源；新分支提交时重建完整 root→sink 路径，
   不能把不含物理 source 的枝段误当完整 demand。
3. 总端口数（sources+sinks）≥16 时先只用目标 sink 所在 bin 中的树节点初始化 heap；
   bin 为空或附近搜索失败，回退到整棵树。
4. 普通连接使用 OpenROAD cost 加权 Dijkstra，无 criticality、delay 或 A* lookahead。
   搜索严格限制在本轮固定 router_bb 内，不在单次搜索中扩框、不回退全图。
5. 仍保留 Track unit、VLine local unit、内部 mux/mode、简单路径/树无环检查；
   每 unit 至多 32 次内部合法性去边回退，单次搜索至多 300000 扩展。
6. 每个新分支立即刷新 live occupancy，使后续搜索看到当前占用。
   允许跨 net 冲突暂时存在，不再要求每个试探为零 overflow 的事务。

## 轮末 dynamic bbox

原始 terminal bbox 四边各外扩 1 格并裁剪，作为每个逻辑 net 的初始 router_bb。
物理边界取当前 COB 阵列的闭区间 row=[0,H]、col=[0,W]，保留外侧 Track/I/O；
不能误裁成 COB 中心索引 [0,H-1]/[0,W-1]。合成图单测从节点范围推导边界。
框跨轮次持久保留，bin 原点/边长独立，不随 router_bb 增长而重置。

每轮路由完成、准备下一轮且尚未拆线时，只检查本轮实际补布/重布成功、
当前已完整连通且自身合法的 net；允许其与其他 net 存在 overflow。
从所有实际路径节点（含 Track/TOB 节点及同步组全部 lane）统计 curr_bb。
分别计算 d_L=C_cmin-B_cmin、d_R=B_cmax-C_cmax、
d_B=C_rmin-B_rmin、d_T=B_rmax-C_rmax；仅 d==0 对应的边向外扩 1 格并裁边。
四条边独立判断，新框仅供下一轮使用；不是 VPR 的 d<=0 条件。
未重布、失败、残缺、内部非法的 net 不扩框；Sync 还须整组无资源冲突且等长。
因此框内完全无路的 net 可能一直失败，不添加失败扩框或全图兜底。
普通 AIR 搜索、同步基础 maze 和尾部等长搜索均遵守同一 net 的 router_bb。

## 同步线

同步组作为原子调度/拆线单元，任何 lane 冲突或未等长均整组拆除。
重布时预布所有 lane，以本次实际最长 lane 为共同目标，使用已有 FPIA
切尾等长/实际目标增长策略；不再预计算并强制独立 lane 最短长度目标。
N_i=按访问次序计数 Track + TOB 常数，Bnet +2、Tnet +1；
同组不得混合 Bnet/Tnet。最终组线长仍按逻辑 net 的 Track+Bump 并集计。
r={0.5,0.75,1.0}，保留 floor(N_track*(1-r)) 个 Track 的前缀再搜索尾段。
出现更长 N_f 时整组恢复快照，以实际 N_f 更新目标后重来，不盲目 target+1。
保留 sibling 物理硬阻塞、前缀/parent-chain 禁回环及内部 mux/mode 合法检查。
sync_tail_extra_tracks=64，尾部 labels 至多 80000。
等长或内部合法性失败时清理整组，不能泄漏部分 lane 占用。
同步基础 maze/切尾搜索复用原实现，但 arc 增量也只采用 OpenROAD 图节点 cost。

## OpenROAD cost 与硬件资源规则

严格采用用户 note.md 的 OpenROAD 形式，映射到图节点：
扩展 u→v：g(v)=g(u)+c(v)，无额外 arc 基价、转弯项或延迟项。
q(v)=usage(v)+red(v)+L*h(v)，本场景 red=0、L=0.9、C=1。
c(v)=1+H/(1+exp(k*(C-q(v))))+(H/s)*max(q(v)-C,0)。
H=4、k=1、s=20 沿用旧 FPIA_RRR 的固定默认值，不自适应提高 H。

usage 是当前 ResourceModel 占用（本次已拆路径不再计入），不加候选的 +1；
同 owner 已占用的树节点增量为 0，不重复收费。
历史项进入 q，同时影响 logistic 与线性项；不得在函数外另加 history，
不得采用旧 FPIA 的额外 arc=1/type_weight 叠加，也不再使用 VPR a*p 乘积。
L=0.9 是输入权重，不是历史衰减系数。

note.md 未另给 OpenROAD 历史更新规则，因此保留 AIR 的逐轮拥塞累积：
h 初值为 0，h_next(v)=h(v)+acc_fac*max(0,usage(v)-1)，acc_fac=1。
每轮在拆线之前按当前拥塞快照更新，不衰减、不在 convergence 时清空。
OpenROAD 公式没有 pres_fac，因此删除 VPR 的 0→0.5→×1.3 调度和 reset_present；
合法后占用随实际拆线释放，自然降低 usage，保留历史惩罚。
这里只替换 cost，不照搬 OpenROAD 的其他历史更新/参数阶段调度。

只有图节点（Track/Bump/HLine/VLine）计费。
PhysicalSwitch、matching endpoint、mux peer、straight/swap 不额外计费或累积 history，
但继续参与 overflow 检测、冲突剪枝与最终硬件合法性检查。
容量/兼容规则不因 cost 限定为节点而删除；Bnet unit 是 per-owner 锁。
同 owner mux port 不允许多个 peer；SyncNet lane 不能共享物理资源。

## 拆线与两种工作状态

每轮先记录当前完整路径、missing、overflow、同步长度；完整合法才更新 incumbent。
合法性包括所有端口连通、无容量/mode/mux/unit 冲突及同步等长。

非法时仅执行冲突修复，不主动做线长拆线；即使过去已经找到过合法解也一样。
全部 net 使用同一拆线前拥塞快照，不能因先拆某个占用者而漏拆其他冲突参与者。
普通 net 总端口数（sources+sinks）≥16 时，沿 root→sink 路径截断第一个冲突
节点/开关，删除失去 root 连通性的下游，保留合法根前缀和未受影响的 sink。
端口数 <16 或内部结构非法则整 net 拆除；同步组始终整组拆。
保留的非空前缀不代表该 sink 已布通，下一轮仍会重连它。

仅当前轮完整合法时，主动拆除 W_current > reference 的 net；
没有候选时拆全部多端口 net（含多 lane 同步组），再没有则拆全部 net。
比较为严格大于，不再有 10% 阈值；同步候选仍整组拆。
重布使当前再次非法后，暂停主动拆线，直到重新合法。

## 终止、incumbent 与输出

没有 max_iterations、convergence、无改善/停滞提前退出条件。
预算未到持续迭代；到预算且已有合法 incumbent，返回线长最小的合法 incumbent，
即使当前正处于非法修复状态，也不返回当前较差/冲突的解。
到预算仍无合法 incumbent，继续迭代到第一次完整合法解，再停止。
最好解只由完整独立校验通过的路径更新，同线长保留更早解。
预算是软预算，在 net/轮次安全检查点检查，不中断一次同步等长/maze 搜索；
参考布线、最后一次搜索及最终日志/校验可能导致超时。
没有合法解的不可布通配置可能持续运行，不能伪造成功或返回零 overflow 部分解。

退出码 0 仅当 success、missing_owners=0、best_overflow=0、同步等长且独立校验通过。
校验重新构造全部占用，检查完整 demand、端点、邻接、简单路径、物理并集无环、
TOB fanout、unit、mode、容量及同步长度，不依赖 live occupancy。
paths[net][demand] 与最终全部 route 日志格式保留。
iter 输出当前实际 overflow/线长/missing/convergences/has_legal；
overflow owners 每轮合并为一行，资源分组写作 [owner,...], [owner,...]；不输出 sync tail maze 搜索参数。
prepare 输出 illegal_ripup、wirelength_ripup、bbox_updates 与 repair/optimize；
spatial/bbox update 日志输出 bin 边长、初始框、实际路径框及四边余量。
保留 status、total_wirelength、RRR_routing_time、total_ms、最好解时间信息。

## 回归测试

FPIA_RRR_unit 支持 --synthetic-only、--budget-only，require 在 Release 有效。
覆盖 OpenROAD 公式/当前占用非 prospective/历史进入两个项/仅节点计费、
历史从零累加且不衰减、资源共享与硬件冲突、Sync 尾部等长/失败回滚、
预算超时继续至合法、预算内持续优化及 incumbent 保留、同快照冲突剪枝、
大树 root 前缀保留/不误判已连通、小 net 和 Sync 整组拆线、
bin +1 偏置、统一端口阈值、初始 padding/物理裁边、四边轮末独立扩张、
未重布/失败/残缺/组内非法不扩、下一轮利用扩框、Sync maze/tail 固定框、
nearby→full fallback 和分支完整 root 路径重建。
真实小矩阵：case_2btb、case_2btt、case_2fanout、case_bus2btb、case_bus2btt。
所有直接调用 run_rrr 的有限时测试必须设置正预算。
配置须匹配当前 Interposer 宽度；不得为测试修改目录外硬件配置。
命名空间 PR_tool，使用 std::Vector/std::String，单测用 Catch-free require()。
