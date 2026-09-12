# TODO: 禁止同一 RoutingNet 在 TOB mux 中非法扇出

## 状态

**未修复，影响 RRR 与 source maze 的线长对比有效性。**

该问题在 `case7` 已确认：RRR 报告总线长 `1283`，而 source maze 为
`1489`。差值 `206` 几乎全部来自 `Nege nets` 与 `Pose nets` 两个
`TracksToBumpsNet`（在 RRR adapter 中表示为 `PNnet`）：

| 范围 | source maze length | RRR wirelength | 差异 |
| --- | ---: | ---: | ---: |
| 全部 66 个 net | 1489 | 1283 | -206 |
| Nege nets | 187 | 80 | -107 |
| Pose nets | 185 | 80 | -105 |
| 其余 net 合计 | 1117 | 1123 | +6 |

因此不能将 case7 的 `1283` 当作一个比 source 更优的可实现结果。

## 复现证据

RRR 将一个非 Sync 的多端口 `PNnet` 聚合为一个 owner：
`OwnerId {net_id, 0}`。在 `case7` 的 Nege/Pose 中，每个目标 TOB 有四个
bump sink；第一条 demand 到达该 TOB 的 VLine 后，后续三条 demand 直接以
该 VLine 为 tree start，例如 Nege 的 `TOB(2,3)`：

```text
demand 0: ... -> TOB(2,3) B1 V80 -> B1 G0 J2 -> B1 G0 I2
demand 1:       TOB(2,3) B1 V80 -> B1 G2 J2 -> B1 G2 I2
demand 2:       TOB(2,3) B1 V80 -> B1 G4 J2 -> B1 G4 I2
demand 3:       TOB(2,3) B1 V80 -> B1 G6 J2 -> B1 G6 I2
```

每个单独的 arc 都存在于统一图中，但上述四条路径共同要求同一个 TOB
HLine-to-VLine mux output（`V80`）同时连接四个不同的 HLine input，真实
硬件不允许。

case7 中两个 PNnet 各有 48 个 demand；每个 PNnet 都有 36 条仅含
`VLine -> HLine -> Bump` 的 tail。这正是每个目标 TOB 首条路径之外的三条
非法复用。RRR 的每个 PNnet 只统计到 `48 Bump + 32 unique Track = 80`；
这甚至小于 48 个 sink 各自需要一个独立 Track-to-TOB 接入所给出的终端
Track 下界。

作为辅助证据，source 的打印路径中 Nege/Pose 分别出现 113/114 个唯一
Track 坐标（149/150 个打印条目）；这也说明差异并非仅由 source 对共享
segment 的重复计数造成。

## 根因

### 1. RRR 将 TOB 内部节点当作可共享 Steiner tree 节点

`route_owner()` 将一个 demand 的全部路径节点都插入 `owner.tree`。下一条
demand 的 `route_demand()` 将 `sources ∪ tree` 全部以距离 0 入堆，包含
Track、Bump、HLine 与 VLine。因此它可以从已使用的 VLine 直接走向另一个
HLine/bump，而不是从一个合法的 Track tree 叶子重新申请 TOB access。

### 2. 同 owner 的任何资源都被视为免费复用

`key_is_free()` 对 `owner_holds_key(resources, owner, key)` 直接返回 true。
`ResourceModel::owner_count()` 统计拥有某资源的不同 owner 数，而不统计同一
owner 的 reference 数；同一 PNnet 对 VLine 的四次 claim 仍显示为一名 owner、
零 overflow。

这种语义适用于共享的 Track wire/tree segment，但不适用于 TOB mux 的 input/
output 和方向寄存器。

### 3. 统一资源模型未表示 TOB mux 的端口独占性

当前 `path_resource_keys()` 有 Node、PhysicalSwitch 和 MatchingEndpoint，
但缺少每一级 TOB mux 的输入 register、输出 register 资源。真实 source
硬件的 `TOBMux::available_output_indexes()` 会在一个 output 已 `give_out()`
后拒绝其他 input；RRR 的 VLine Node 资源不能表达“同一 owner 也不能把它
接到不同 HLine”的约束。

## 为什么 source 不会发生同样的复用

source 的 `route_tracks_to_bumps_net()` 每连接一个 end bump 都调用
`available_tracks_track_to_bump(end_bump)`，并对选中的 `TOBConnector` 调用
`give_out()`。后续候选 connector 会经过每一级 `TOBMux` 的 input/output
availability 检查，故不能复用同一个 VLine mux output。

source 的 `_length` 与 RRR 的 `net_wirelength()` 仍不是同一统计口径：source
按每个 branch 的 COB-compressed `path_length(path)+1` 累加，RRR 对一个 net 的
Track+Bump 节点去重并忽略 HLine/VLine。但 case7 的非法 TOB fanout 是独立于
这个统计差异的可实现性错误。

## 修复要求

1. **禁止 TOB 内部资源被同一 owner 以不同连接方式复用。** 至少要保证同一
   VLine 不能同时连接多个 HLine，且同一 Track mux output 不能同时连接多个
   VLine。
2. **保留合法的 Track Steiner tree 共享。** 不应简单把同 owner 的所有资源
   都改为容量 1，否则会错误禁止普通 multi-fanout net 的共享 Track 主干。
3. 修复后 `validate_rrr_solution()` 必须在重建资源时验证 TOB mux input/output
   独占，而不是只验证统一图 arc 与 owner-level overflow。

## 候选实现路径

优先采用“显式 TOB mux 端口资源”而非只做局部补丁：

- 为 Bump-to-HLine、HLine-to-VLine、VLine-to-Track 三层 mux 分别建立 input
  与 output ResourceKey；同一 exact connection 可以由同一 owner 重用，但同一
  mux port 指向不同 connection 时必须冲突，即使 owner 相同。
- 在 path claim 中记录 connector identity，而非仅记录 VLine/HLine Node。
- `route_demand()` 的 tree start 应限制为 Track 节点。
- 保持 COB Track tree 的同-owner 共享规则；TOB mux port 的“同 owner 冲突”
  需要独立于通用 `owner_holds_key()` 处理。

## 回归测试

新增最小测试：一个 Track source、同一 TOB 内四个 end bump，四个 bump 的
HLine 均能候选到同一个 VLine。修复前应复现 3 条
`VLine -> HLine -> Bump` tail；修复后应为四个 bump 分配互不冲突的 TOB
connector/Track access，并由 validator 通过。

对 PNnet 日志增加诊断项：每个 `(TOB, VLine)` 的
不同 incident HLine/Track connector 数。任何数大于 1 都应报告为非法；修复后
必须为 1。
