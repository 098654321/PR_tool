# TOB 与 unit 连通性验证

## 复现

在 `PR_tool/` 目录运行：

```bash
xmake build tob_unit_graph_verify
./output/tob_unit_graph_verify algorithm/TOB/unit_graph_result.json
```

程序直接调用 `graph/unified_routing_graph.cc` 建立与直接详细布线 ILP 相同的细粒度图，再调用 `direct_ilp/undirected_graph.cc` 合并反向 arc。当前结果对应 `COB_ARRAY_WIDTH=13`、`COB_ARRAY_HEIGHT=9`、`TOB_SIZE=16`。可机器读取的完整计数见 [unit_graph_result.json](unit_graph_result.json)。

## 结果

| 检查项 | 结果 |
|---|---:|
| 物理图节点 / 无向边 | 38,912 / 126,720 |
| Track 节点 | 32,768 |
| 只保留 Track--Track 边后的连通分量 | 16 |
| 每个 unit 的分量 / Track 节点 | 1 / 2,048 |
| Bump 总数 | 2,048 |
| 每个 Bump 可通过单条 Bump--HLine--VLine--Track 链接入的 unit | 全部 16 个 |
| 每个 Bump 的具体接入链数量 | 128，即每个 unit 8 条 |
| 原始无向图中的跨 unit `Track--VLine--Track` 两边路径 | 2,048 |
| 逐一检查的 TOB Track 起点 | 2,048 |
| 满足 TOB matching/mode 的 TOB 内 Track-to-Track 路径 | 0 |

Track--Track 边从不跨 unit。原始无向图中的 2,048 条 `Track--VLine--Track` 跨 unit 路径说明，仅按普通图连通性分析会误判：同一 VLine 的两条 Track 边属于同一个 mode group，且分别要求 straight 与 swap，不能同时选中。

程序还对每个 TOB Track 起点搜索 TOB 内的简单路径，按当前直接 ILP 的 Bump--HLine matching、HLine--VLine matching、straight/swap 互斥规则逐边检查。没有找到可行的 Track-to-Track 路径。其结构原因是：选中配置中，每个 VLine 最多接一条 Track 边和一条 HLine 边；每个 HLine 最多接一条 VLine 边和一条 Bump 边；每个 Bump 最多接一条 HLine 边。由 Track 进入 TOB 的路径最多沿 `Track--VLine--HLine--Bump` 到达端点，不能从 TOB 再接出另一条 Track。

## 对布线模型的含义与边界

在当前场景里，Bump 只会是路径端点或不被使用。因此，一个 **2-pin 物理路径**进入某个 Track unit 后不能经 TOB 换到另一 unit；Bump 端点仍可选择全部 16 个 unit 中的任意一个进行接入。这支持把详细布线候选按 unit 构造，同时保留 TOB 开关与 mode 的跨候选冲突约束。

这里验证的是图和当前 ILP 所采用的开关互斥语义，不是“一个逻辑 net 必定只用一个 unit”的普遍证明。TrackToBumpsNet 的各 demand 有同一固定 Track source；PNnet 的不同 demand 可以选择不同的 source Track，可能属于不同 unit。SyncBus 成员也应分别看作路径。程序没有检查信号方向、实际电路实例或 ILP 解中允许存在的断开闭环；结论针对从端点到端点的实际提取路径。硬件映射或开关约束改变后须重新运行验证。
