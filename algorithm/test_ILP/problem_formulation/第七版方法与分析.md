## 第七版方法与分析（SAT1）

本版在**第六版方法与分析（SAT1）**基础上，仅修改「给定一个 `end_track`，预计算其可达 `start_track` 集合及对应 MCF bbox」这一过程，以及与之配套的外层失败重试策略。第六版中的 SAT 编码（W/S/QS/QW/Y/A 及可达性 forbidden clauses）、BusMCF / SimpleMCF 建模与求解顺序、bus 级 `RectHull`、PNnet 在 MCF 阶段不裁剪、`--enable-pre-routing` 触发的 MCF warm start 等**均保留**。

本版**废弃**第六版中的下列内容：

- 每条 record 的 `ρ_n` 与 `BBox(n, ρ_n) = Expand(BBox_0(n), ρ_n)`；
- 在固定 bbox 内用 k-shortest path 补充 `start_track`；
- Wilton 序列形式的 `reach` / `IlpReachStep` 预计算（`reach_by_end_start` 不再作为 SAT 或 MCF 的依赖；MCF warm start 在 SAT 分配结果与路径 bbox 限定后的可行图内做 BFS，不受影响）。

---

### 1. 动机（case13）

第六版在 case13 上出现 BusMCF 求解时间过长。根因之一是 TOB SAT 预计算阶段对范围的刻画不合理：需要把局部扩展推到 `ρ=4`、对同 COBUnit 的 8 条 TOB channel track 全部纳入候选后 SAT 才可行，但此时 MCF 仍可能沿用与真实布通路径不匹配的过大范围，导致 BusMCF 变量与搜索空间膨胀。

旧流程「先定 bbox → 在 bbox 内 k-shortest → 得到 start_track」存在：

1. k-shortest 在 bbox 内重复计算，较慢；
2. 多次扩展后候选 start_track 高度重合，扩层收益递减；
3. `ρ=4` 时一次性放开 8 条 track，却未区分各 track 到 `end_track` 的实际路径范围。

第七版改为：**先对同 COBUnit 的 8 条候选 start_track 各算一条受限最短路及其路径 bbox，再按路径长度（path length）分层逐步放开候选**；SAT 选定 start_track 后，**直接用该路径的 bbox（含边界方向修剪，见第 6 节）** 作为该 record 的 MCF 范围来源。

---

### 2. 候选 start_track 的固定枚举

对一条 record $n$、一个端点 track $r_e$（`end_track`）：

1. 取起点 bump 所在 TOB 的 channel：`tob_channel_track_coords(start_tob)`；
2. 在 channel 中筛选与 $r_e$ **位于同一 COBUnit** 的 track，记为集合 $C_{n,r_e}$，$|C_{n,r_e}| \le 8$；
3. $C_{n,r_e}$ 是本次预计算的全部候选 start_track，不再通过 bbox 裁剪或 k-shortest 扩充。

**record 类型：**

- **Tnet**：仅一个固定 `end_track`，对其实施本节逻辑；
- **PNnet**：对 `Endtrack_n` 中**每个**候选 `end_track` 各自独立实施；
- **Bnet**：对全部 128 个候选 `end_track` 各自独立实施（与第六版 `end_tracks = all` 一致，但 `Starttrack_{n,r_e}` 的生成规则改为本节及下节）。

---

### 3. 受限最短路及其路径 bbox（一次性预计算并缓存）

对每个 $(n, r_e, r_s)$，$r_s \in C_{n,r_e}$，在**全 COB 阵列**上预计算一条最短路径及其 bbox，并缓存，外层迭代只调整「哪些层已纳入候选」，**不重复**跑最短路。若存在多条同 path length 的最短路径，任取其中一条可复现的路径作为缓存路径即可；不要求在同 path length 内继续最小化 bbox。

#### 3.1 图与最短路

- 使用 `hardware::Interposer::adjacent_tracks` 的邻接关系建图做 BFS：对每个合法 track 调用一次 `adjacent_tracks(track)`，并将其返回的 `(next_track, connector)` 直接记录为预计算图中的邻接边；**不额外补充 reverse arc**，也不另行改写为无向图；
- 该邻接关系与 `adjacent_idle_tracks` 使用相同的 track–track 连通语义；区别在于**预计算阶段忽略资源占用**，不检查 connector / track 是否已被占用；
- 路径长度 = 路径经过的 **track 节点数**，与当前 MCF 流量统计方式一致；
- 搜索方向：从 `end_track` 的 `TrackCoord` 出发，到达 start TOB channel 上 track 号为 $r_s$ 的节点；该方向仅用于预计算查找，不改变 `adjacent_tracks` 定义的邻接语义。

若在下列约束下 $r_s$ 不可达 $r_e$，则**从该 $r_e$ 的候选集中剔除** $r_s$，不参与分层与排序。

#### 3.2 路径约束

1. **简单路径**：同一条路径不得重复经过同一 track 资源；
2. **Wilton 防打转**：每一步扩展对应一个 `COBConnector`，其 `coord()` 标识该次 Wilton 所在的 COB 方格。若上一步与当前步的 `connector.coord()` 相同，则视为在同一 COBUnit 内**连续第二次**使用 Wilton，该扩展禁止；直穿对边计 1 次；
3. **允许再入**：两步扩展的 `connector.coord()` 不同时，不算连续；离开某 COBUnit 后再次进入并只使用 1 次 Wilton **不算**违规。

> 实现提示：BFS 状态为 `(current_track, last_wilton_cob)`，其中 `last_wilton_cob` 为上一步所用 `COBConnector` 的 `coord()`；状态中**不携带完整 visited set**，避免状态数随路径历史指数膨胀。用 `prev` 恢复候选路径，若恢复出的路径存在重复 track，则丢弃该 target 状态并继续搜索；一旦找到第一条满足简单路径与 Wilton 防打转约束的最短路径，即可缓存该路径及其 bbox。若所有 target 状态恢复出的路径均不满足简单路径要求，则认为该 $(r_e,r_s)$ 不可达。

#### 3.3 路径 bbox（用于 MCF 裁剪）

对求得的 track 序列，取每个 `TrackCoord` 经 `track_to_cob` 得到的 COB 坐标，连同：

- start bump 的 TOB anchor：`tob_anchor_cob(start_tob)`（TOB 正上/下方与该 TOB channel 直接相连的 COB，是起点 track 最自然的邻接范围，**保留在 bbox 内**）；
- 路径上各 track 映射得到的 COB 坐标；
- 若终点为 bump：end bump 对应 `end_track` 端口的 COB 坐标；

一并纳入，求轴对齐最小外接矩形，得到 `IlpBoundingBox`（仍以 **COB 坐标** 描述，记为 $BBox_{path}(n, r_e, r_s)$）。

**坐标约定：** COB 阵列中 **row 从下往上递增**——`row_min` 为 bbox 最下方一行 COB，`row_max` 为最上方一行 COB。该 bbox **不用于 SAT tier 分层**（分层见 §3.4），仅用于 SAT 成功后的 MCF 可行图裁剪（见第 6 节）。

可选地仍可缓存矩形面积 $\text{area} = (row_{max} - row_{min} + 1) \times (col_{max} - col_{min} + 1)$ 供日志诊断，但 **tier 开放候选不以 area 为依据**。

#### 3.4 按路径长度分层

对每个 $(n, r_e)$，将 $C_{n,r_e}$ 中**可达**的 $r_s$ 按预计算得到的 **path length**（路径经过的 track 节点数，与 §3.1 一致）升序分组为长度层级 $L_0 < L_1 < \cdots < L_{m-1}$（$m \le 8$）。**同一 path length** 的多个 $r_s$ 属于**同一层**，同时纳入或同时滞后。

分层规则（与旧版 area 分层结构相同，仅排序键改为 path length）：

- $L_0$：可达 $r_s$ 中 path length **最小**的全部 track；
- $L_k$：path length **严格大于** $L_{k-1}$ 中最大长度、且为当前剩余候选里最小长度**的那一挡**上的全部 $r_s$；
- `tier(n) = k` 时，$Starttrack_{n,r_e} = L_0 \cup L_1 \cup \cdots \cup L_k$（不足 $k+1$ 层时取已有层的并集）。

为每个 $(n, r_e, r_s)$ 缓存：

- 最短路径长度 `path_length`；
- 路径 bbox `BBox_path(n, r_e, r_s)`。

---

### 4. tier：按 record 同步的候选放开

废弃 `ρ_n`。对每条 record $n$ 维护一个整数 **tier**：

- 初始 `tier(n) = 0`；
- `tier(n) = k` 表示：对该 record 的**每一个** `end_track` $r_e$，`Starttrack_{n,r_e}` 已包含该 $r_e$ 下路径长度层级 $L_0, L_1, \ldots, L_k$ 中的全部可达 $r_s$（见 §3.4）；
- **失败时** `tier(n) ← tier(n) + 1`，该 record 下**所有** `end_track` **同步**再开放下一 path length 层级。

某一 `end_track` 若已无更高层级，则 `tier++` 对该 $r_e$ 不再新增 track；只要 fail_set 中**任一** record 的**任一** `end_track` 新增了 start_track，即视为本轮扩展有效。

当 fail_set 中所有 record 的所有 `end_track` 均已纳入其全部可达候选（至多 8 条）后仍失败，则 `expand` 无进展，**报告失败**。

SAT 前，**tier 分层只依赖 path length**，不依赖 bbox 面积。路径 bbox 在 SAT 阶段仅预计算并缓存，**不作为** SAT 硬约束；MCF 范围在 SAT 成功后由选中路径的 bbox（加 §6.2 边界修剪）决定。

---

### 5. 外层迭代与失败扩展

整体流程与第六版「SAT → MCF → 失败则重试」一致，但扩展动作由 `ρ++` 改为 `tier++`（放开下一层 start_track）。

```python
for each record n:
    tier(n) = 0
    for each end_track r_e of n:
        precompute and cache all (r_s, path, BBox_path) for r_s in C_{n,r_e}
        Starttrack_{n,r_e} = { r_s in layer L_0 }

for attempt in range(max_attempt):
    build TOB SAT using current Starttrack_{n,r_e}
    solve TOB SAT

    if TOB SAT is UNSAT:
        fail_set = { n | type(n) is Tnet or PNnet }   # Bnet 不扩展
        if expand_tier(fail_set) added no new start_track:
            report failure
        continue

    extract assigned_track(p) and Y_{n,r_e} if applicable
    allocateNettoCOBUnit()
    prepare MCF feasible graph using path bbox of SAT-chosen (r_e, r_s) per record

    solve BusMCF
    if BusMCF is infeasible:
        fail_set = all member records of failed bus_key
        if expand_tier(fail_set) added no new start_track:
            report failure
        continue

    solve SimpleMCF for each COBUnit
    if any SimpleMCF unit is infeasible:
        fail_set = simple records in failed units
        fail_set += all child records of failed multi-fanout origins
        if expand_tier(fail_set) added no new start_track:
            report failure
        continue

    return final routing result

report failure
```

其中 `expand_tier(fail_set)`：对 fail_set 中每条 record $n$ 执行 `tier(n) += 1`，并按第 4 节更新其**所有** `end_track` 的 `Starttrack_{n,r_e}`。

**说明：**

1. 若 `expand_tier` 未使**任何** start_track 候选增加，则终止（含「已全 8 条仍失败」的情形）；
2. MCF 失败后扩的是**下一轮 SAT 的候选池**，不是把上一轮已选 start_track 的路径 bbox 做 `Expand`；
3. SAT 在多轮中反复选择同一 `start_track`、而 tier 仍在推进，属于**预期行为**，直到更高层候选被纳入或终局失败。

**失败集合**（与第六版「分析」章节一致，仅扩展动作不同）：

| 阶段 | fail_set |
|------|----------|
| SAT UNSAT | 全部 `Tnet` 与 `PNnet`；**不含** `Bnet` |
| BusMCF 失败 | 失败 `bus_key` 下全部 member records |
| SimpleMCF 失败 | 失败 unit 内全部 simple records，外加失败多扇出 origin 的全部 child records |

BusMCF 失败若无法定位 `bus_key`，应报告诊断不足并失败，**不得**扩大无关 bus。

**Bnet 扩展策略：** SAT UNSAT 时不扩展 Bnet，是本版的建模假设：Bnet 通常不是 SAT UNSAT 的主要原因。只有当 BusMCF / SimpleMCF 求解或诊断明确发现某条 Bnet 参与失败时，才将对应 Bnet record 放入 fail_set 并执行 `tier++`。

---

### 6. SAT 成功后的 MCF bbox

TOB SAT 成功后，对每条需要 MCF bbox 限制的 record $n$：

1. 读取 SAT 分配的 start bump track $r_s$；
2. 读取该 record 在本轮有效的 `end_track` $r_e$：
   - **Tnet**：$r_e$ 为固定的 track 端点；
   - **PNnet**：$r_e$ 由唯一满足的 $Y_{n,r_e}=\text{true}$ 确定；
   - **Bnet**：$r_e$ 为 end bump 分配到的 track；
3. 取缓存的 $BBox^{MCF}_n = BBox_{path}(n, r_e, r_s)$，并在 MCF 可行图上施加 §6.2 的 **track 边界修剪**（bbox 仍为 COB 矩形，不改为 track 级 hull）。

#### 6.2 MCF 可行图上的 bbox 裁剪与 track 边界修剪

**COB 矩形包含（物理 arc）：** 对非 virtual 的 `McfArc`，先按其物理 COB 位置判断是否在 $BBox^{MCF}$ 内。普通 turn arc 以 `arc.cob` 标识的 COB 为准；straight-through arc 沿用现有物理图裁剪逻辑，需要其所在 COB 以及直穿方向对应的相邻 COB 均在 bbox 内。virtual arc 或 `arc.cob < 0` 的连接不受 COB 矩形裁剪。

**track 节点弧（需区分 H/V）：** MCF 图中 track 级节点由 `(track_row, track_col, H|V, track_index)` 标识。在 COB 矩形已包含路径相关 COB 的前提下，对 **track 节点之间的弧** 额外施加下列规则，避免「COB 在框内但同 COB 上未使用的 H/V track 也被放行」：

| 边界 | 禁用条件（track 节点） | 原因 |
|------|------------------------|------|
| 左边界 `col_min` | **水平（H）** track，且 `track_col = col_min` | H track 在 `track.cc` 中邻接 $(row,\, col-1)$ 与 $(row,\, col)$，会向左用到 $col_{min}-1$，超出 bbox |
| 右边界 `col_max` | **水平（H）** track，且 `track_col = col_max + 1` | 邻接 $(row,\, col_{max})$ 与 $(row,\, col_{max}+1)$，向右超出 bbox |
| 下边界 `row_min` | **垂直（V）** track，且 `track_row = row_min` | 阵列 row 向上增大；$row_{min}$ 为 bbox **最下一行** COB，该处 V track 向下连到 $row_{min}-1$，超出 bbox |
| 上边界 `row_max` | **垂直（V）** track，且 `track_row = row_max + 1` | 该 V track 向上接到 $row_{max}$ 所在 COB，track 节点本身位于 $row_{max}+1$，超出 bbox 上沿 |

实现上：先按 COB 矩形判断 arc 是否可进入候选，再对 **track 节点弧** 用上表剔除；被删 track 节点的邻接边自然失效。

**端点连通例外（commodity-specific）：** track 边界修剪可能把 SAT 已选的 $r_s$ 或 $r_e$ 对应 track 节点落在 bbox 边界外侧。为避免 MCF 裁剪切断 SAT 固定端点，对每个 commodity 单独允许：

1. 当前 commodity 的 `src` / `snk` endpoint node 不受上表 track 节点边界修剪限制；
2. 若某条 arc 按普通物理 bbox 规则会被拒绝，但它 incident 到当前 commodity 的 `src` 或 `snk`，且该 arc 的 `arc.cob` 对应 COB 在当前 effective bbox 内，则该 arc 作为 endpoint 接入边保留；
3. 该例外只对当前 commodity 生效，不放宽其它 commodity 的 endpoint，也不全局改变 straight-through arc 的 bbox 规则。

因此，路径预计算 BFS、MCF warm start 与 MCF 求解均在“普通 bbox 裁剪 + 当前 commodity endpoint 例外”的可行图上进行。仍然**不要求**把 $r_s$/$r_e$ 的 track 坐标单独并入 bbox 矩形；endpoint 的连通性由上述局部例外保证。

**聚合规则**（与第六版相同，成员 bbox 来源改为路径 bbox + 上表修剪）：

- 普通 2-pin record（`Tnet` / `Bnet`）：$G^{MCF}_n$ 由 $BBox^{MCF}_n$ 裁剪；
- BusMCF：$BBox^{MCF}_{Bus} = RectHull(\{BBox^{MCF}_n \mid n \in Bus,\ n \text{ 需要 bbox 限制}\})$；bus 内各 member 的 per-record bbox 来自各自 SAT 选定的 $(r_e, r_s)$，再取最小外接矩形；
- SimpleMCF 多扇出 origin $H$：$BBox^{MCF}_H = RectHull(\{BBox^{MCF}_n \mid n \in H.child\_net\})$；
- **PNnet** 在 SimpleMCF 中**仍不裁剪**（$G^{MCF}_n = G^c$），与第六版约定一致。PNnet 的 path length 分层 **仅用于** SAT 阶段的 `Starttrack` 候选，**不**用于 MCF 可行图裁剪。

下一轮外层迭代中，若 SAT 重新选择了不同的 $(r_e, r_s)$，MCF 使用**该轮**对应的路径 bbox，不与上一轮混用。

**MCF 失败后的 bbox 变化**：失败时执行的是 `tier++`（扩大 SAT 候选 `start_track`，即纳入更长 path length 层），**不会**自动将当前路径 bbox 外扩一圈。下一轮 MCF bbox 取决于新一轮 SAT 分配的 $(r_e, r_s)$，可能与上一轮相同，也可能因选中更长路径或不同走线而变大。

---

### 7. record 类型补充说明

#### 7.1 Tnet 与 Bnet 的 `end_track` 区别

| | **Tnet** | **Bnet** |
|---|----------|----------|
| `end_track` 个数 | 1 个，布线前已知 | 128 个假设值（end bump 的 track 由 SAT 决定） |
| SAT 角色 | $r_e$ 固定；起点只能在 `Starttrack_{n,r_e}` 中选 | 对每个假设 $r_e$，约束 start/end track 的配对可达性 |
| 路径 bbox 端点 | start：TOB anchor；end：track 端口 COB | start / end：两端 bump 的 TOB anchor 与各自 track 端口 COB |

二者对 $(n, r_e, r_s)$ 的最短路、path length 分层、bbox 缓存与 tier 规则相同；差异仅在 $r_e$ 是否事先固定及预计算规模。

#### 7.2 PNnet

对每个候选 `end_track` $r_e \in Endtrack_n$ 独立预计算 `Starttrack_{n,r_e}` 与 `BBox_path`。SAT 阶段用 $Y_{n,r_e}$ 选择实际 $r_e$；MCF 阶段不裁剪（见第 6 节）。

---

### 8. 与第六版的对照摘要

| 项目 | 第六版 | 第七版 |
|------|--------|--------|
| start_track 来源 | `BBox_0` + Wilton reach；`ρ>0` 时 bbox 内 k-shortest | 同 COBUnit 8 track + 受限最短路；按 **path length** 分层 |
| 扩展变量 | `ρ_n ∈ {0,…,4}` per record | `tier(n)` per record，其下所有 `end_track` 同步 |
| SAT 分层键 | bbox 扩展 / k-shortest | **path length**（非 bbox area） |
| MCF bbox | `Expand(BBox_0(n), ρ_n)` | SAT 选中 $(r_e,r_s)$ 的路径 COB 矩形 + H/V 边界 track 修剪 + commodity endpoint 接入例外 |
| reach / k-shortest | 使用 | 废弃 |
| SAT 编码 / MCF 模型 | — | **保留** |
| MCF warm start | — | **保留** |

---

### 9. 预期效果与边界

1. 减少与真实布通路径无关的过大 MCF 范围，缓解 case13 类 BusMCF 膨胀；
2. 预计算一次、迭代只扩候选层，避免 bbox 内重复 k-shortest；
3. **按 path length 分层**在候选最短路径长度不同时区分同 `end_track` 下的多条 start_track；当多条候选 path length 相同，它们仍属于同一层并同时开放，避免仅因 TOB anchor 撑满矩形而导致 area 相同、无法分层；
4. MCF 裁剪在 COB 矩形基础上对 **H/V track 边界**修剪，减轻「COB 在框内则该 COB 上所有方向 track 均可走」的过宽问题；
5. 对 SAT 选中的 `src/snk` endpoint node 与 bbox 内 endpoint 接入边做 commodity-specific 例外，避免边界修剪把已固定端点从 MCF 可行图中切断。

**边界：**

- SAT UNSAT 仍无法精确定位致因 net，故沿用「扩展全部 Tnet/PNnet」的近似策略；
- Bnet 不在 SAT UNSAT 时扩 tier，仅在有 BusMCF/SimpleMCF 证据时扩；
- Bnet 对 128 个 `end_track` 各做预计算，规模为 $128 \times 8$ 次最短路 per Bnet record，属可接受的离线预计算成本。
