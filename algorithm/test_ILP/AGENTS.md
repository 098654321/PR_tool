# PR_tool / algorithm/test_ILP 工程指南

## 当前目标和流程

`test_ILP` 当前实现为**第二十四版方法**：从原始配置建立细粒度硬件图，对每个 `RoutingNet` 使用 `bbox+1`（不加 TOB patch）构造范围，先生成完整物理路径候选并用 HiGHS 做 route 变量 LP/ILP，再将允许未布通 owner 的合法部分解交给 RRR 补布和优化。只有所有 owner 布通并通过完整验证才报告成功。当前入口不运行 PNnet 预分配、Global Routing、SAT 或按边直接详细布线 ILP。方法说明见 `../../../问题定义与方法/第二十四版方法.md`；旧流程保存在 `dev.ILP_SAT` 分支，第二十三版方法仍可作对照。

该目录是实验入口，不应无意修改 `source/algo/router/` 的正式路由流程。`source/hardware/` 和 `source/circuit/` 决定真实拓扑与端点。当前分支仍保留一部分旧方法源文件供其他实验目标使用，但 `test_ILP` 仅编译新入口所需模块。

## 关键模块和不变量

| 路径 | 职责 |
|---|---|
| `main.cc`, `test_ilp_cli.*` | 唯一入口、参数、日志、两次验证及结果 |
| `scope/build_routing_nets.*`, `scope/scope_bbox.*` | 原始 net、端点及 bbox 规则 |
| `graph/unified_routing_graph.*` | 细粒度硬件图及 PN 虚拟源 |
| `common/routing_scope.hh` | ILP、RRR 共用的局部节点/arc scope 容器 |
| `direct_ilp/direct_scope.*` | net 级 `bbox+1`、端点 Bump 的 TOB 接入节点和局部 arc scope |
| `route_ilp/route_search.*` | 物理 owner、候选路径搜索及节点/TOB 资源集合 |
| `route_ilp/route_master.*` | 完整候选的 LP 定价与 HiGHS 整数选择，允许未布通 owner |
| `route_ilp/route_rrr.*` | 接收合法部分解，补布缺失 owner 并事务式重布 |
| `direct_ilp/direct_validate.*` | 独立检查部分或完整路径、scope、互斥、TOB、同步长度及线长 |
| `common/highs_log_sink.hh` | HiGHS 原生日志输出 |
| `common/route_metrics.*` | Track+Bump 物理并集线长 |
| `test/route_ilp_unit.cc` | 当前流程的合成回归入口 |

- Bnet/Tnet/PNnet 的同一多 sink net 共享一个物理 owner；SyncBus 每个 member 独立占用物理节点，但受同组 Track+Bump 等长约束。
- PNnet 各 demand 可独立选择原始候选 source；虚拟根只用于表达候选 source，不出现在物理路径中，也不计线长。
- 完整候选必须先验证端点、简单路径、物理环路与内部开关配置；HiGHS 只选择经过验证的候选。
- 目标和最终线长都是 Track+Bump 节点并集数。
- SyncBus 的每条 lane 是独立物理 owner，共享总线的 `bbox+1` scope；RRR 可重布 lane，但长度须保持当前共同长度。
- RRR 事务中可暂时拆线或发生冲突；只提交零 overflow、每个已布通 owner 完整合法且未布通数减少，或未布通数不变而线长下降的结果。

## 构建和测试

在 `PR_tool/` 下运行：

```bash
xmake build test_ILP
xmake build route_ilp_unit
./output/route_ilp_unit
./output/test_ILP algorithm/test_ILP/test/multi-pin/1 --time-limit 1 -o /tmp/route_ilp_case
```

`--time-limit MIN` 限制 route ILP 与 RRR 的总运行时间。`-v` 输出额外的 bbox 与 scope 诊断。`debug.log` 记录模型规模、求解时间、结果路径与 RRR 统计；同目录的 `highs.log` 保存求解器原生日志，两者均不依赖 `-v`。

测试时有一个常见的问题，就是source/hardware/interposer.hh当中给出的COB_ARRAY_WIDTH不一定和当前测试case使用的WIDTH一致，因为有些case用的是13，有些case用的是12。如果遇到类似“invalid external port coord: { row: 7, col: 12, H, index: 63 }”这样的问题，说明是WIDTH不一致导致的错误。你可以在认为其余测试已经足够的情况下直接忽略这个报错的测试，也可以回到source/hardware/interposer.hh，把WIDTH调整为12后重新构建并测试。

## 修改要求

- 修改前核对方法文档和硬件映射；只改当前流程直接需要的代码。
- 关键约束应有小型合成测试；测试断言必须在 Release 构建中生效。真实 case 用于检查规模、耗时与最终物理合法性。
- 单文件保持紧凑，避免无关重构；保证正确性的前提下关注求解速度。
- 代码文件的单次修改超过 200 行时，在实现完成后进行分离审查：如果实现过程是启动子agent实现的，那么可以由原主agent自己审查；如果实现是由主agent自己实现的，那么需要启动一个子agent审查。如果单次修改不超过200行，可以由实现的agent自己审查。如果审查之后有问题需要修正，但是修正之后不需要再进行单独的审查工作。
- 在写代码的时候，关键的算法中间信息与结果信息（例如建模中的变量/约束数量、求解时间、求解出来的布线路径结果）等需要加入日志（debug.log），即使不含-v也要输出；一些次级信息，用于辅助深入debug的，例如算法参数、bbox范围等，在-v等情况下也要输出；如果算法调用了ILP/SAT等求解器等，求解器的原生求解日志需要生成在debug.log同目录下，即使运行时没有-v也要
