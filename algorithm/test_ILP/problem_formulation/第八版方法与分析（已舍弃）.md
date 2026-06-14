## 第八版方法与分析（SAT1）

本版在**第七版方法与分析（SAT1）**基础上，针对 SimpleMCF 阶段多端口 net 求解过慢的问题，增加 **Steiner 拆解 + 混合 SimpleMCF + 末尾 origin 级 maze 精修**。第七版中的 TOB SAT 编码、path length 分层（`tier`）、路径预计算与 MCF bbox（含 §6.2 H/V track 边界修剪与 endpoint 例外）、BusMCF 建模与求解顺序、外层 `tier++` 失败扩展规则等**均保留**。

**启用条件：** 仅当 CLI 指定 `--enable-steiner-decompose` 时启用本版流程；**未指定时完全回退第七版**（含第七版 `--enable-pre-routing` 对普通 2-pin net 的 MCF warm start 语义）。

本版**不修改** TOB SAT 的 CNF 编码与 path precompute 规则；主要改动在 **BusMCF 之后 → SimpleMCF 之前/之中** 的多端口 net 处理，以及 **全部 MCF 完成之后** 对多端口 origin 的 maze 后处理。

---

### 1. 动机

第七版在 `--enable-mcf-obj` 模式下，SimpleMCF 对多端口 net（`TrackToBumpsNet`、`TracksToBumpsNet` 及其拆分出的 `PNnet` child）求解很慢。主要原因：

1. **多扇出 origin 建模**（第六版起）：同一 origin 下多条 child 共享 `x^H_e` / `o^H_i`，变量与 `f_le_x` 双向耦合规模大；
2. **PNnet 在 MCF 阶段不裁剪**：`snk` 连 `virtual_Pnode` / `virtual_Nnode`，候选 end track 多，搜索空间大；
3. **origin 级 bbox 为 child `RectHull`**：范围偏大，进一步膨胀 Gurobi 列数。

第八版思路：**在 BusMCF 之后、SimpleMCF 之前**，对可拆解的多端口 net 用近似 Steiner 树拆成独立 2-pin segment（临时 commodity），在更小的 per-segment bbox 内做快速 MCF；拆解失败或未拆解端口仍用第七版 origin 建模。SimpleMCF 阶段**不强制**拆出的各段拼成一棵树（各段独立寻路）；**全部 MCF 结束后**，对每个多端口 **origin** 用与 `source/algo/router` 一致的 maze（含 TOB 重分配）整体重布，修复「拆开各算各的」导致的树形不一致或失败，并可择优替换 MCF 结果。

---

### 2. 与第七版的开关与职责划分

| 项目 | 第七版（默认） | 第八版（`--enable-steiner-decompose`） |
|------|----------------|----------------------------------------|
| TOB SAT / tier / path precompute | 同左 | **不变** |
| BusMCF | 同左 | **不变** |
| SimpleMCF 多端口建模 | origin `x^H` / `o^H`；PNnet 用 virtual P/N | Steiner 拆解 + 混合 2-pin / origin |
| PNnet MCF `snk` | `virtual_Pnode` / `virtual_Nnode` | SAT 选定的固定 `end_track` |
| MCF warm start | `--enable-pre-routing` 时对 commodity BFS | 普通 2-pin **保留** warm start；拆解段按 2-pin warm start；残余 origin 按原 origin warm start |
| MCF 后处理 | 无 | 多端口 origin maze 精修 |

**CLI：**

- `--enable-steiner-decompose`：启用本版整条路径（Steiner 拆解、混合 SimpleMCF、拆解段 warm start、末尾 maze）。
- 未加该参数：行为与第七版一致；此时 `--enable-pre-routing` 仍仅表示第七版 MCF graph warm start。
- 实现上可将已废弃的 `--enable-pre-routing` 在启用第八版时映射或告警，但**第八版的正式开关名为 `--enable-steiner-decompose`**。

`--enable-steiner-decompose` 须与 `--enable-mcf-routing` 联用；单独指定且无 MCF 时应打印 warning 并忽略。

---

### 3. 总体流程

```python
# 外层 tier 循环与第七版相同（见第七版 §5）
for attempt in range(max_attempt):
    build and solve TOB SAT
    if UNSAT: expand_tier(fail_set); continue

    extract SAT assignment (start_track, Y_{n,r_e} for PNnet)
    allocateNettoCOBUnit()
    build MCF bbox context from path precompute (第七版 §6)

    solve BusMCF
    if BusMCF infeasible: expand_tier(...); continue

    if not --enable-steiner-decompose:
        # 第七版路径
        optional: MCF warm start (--enable-pre-routing) for all SimpleMCF commodities
        solve SimpleMCF per COBUnit
        if any unit fails: expand_tier(...); continue
        return success

    # -------- 第八版：BusMCF 之后 --------
    deduct BusMCF used edges/nodes -> residual capacity

    group multi-port records for Steiner (§4)
  for each Steiner instance H in current COBUnit:
        run GraphSteinerDecomposeTo2PinSegments_NoCachedPath (§6)
        on success: emit temporary 2-pin commodities (§7)
        on failure: keep affected ports in origin group (§7.3)

    build mixed SimpleMCF model (§7)
    run warm start (§8): plain 2-pin + decomposed segments + residual origin groups
    solve SimpleMCF per COBUnit
    if any unit fails: expand_tier(...); continue   # 与第七版相同，不因 maze 救回而跳过

    # -------- 第八版：全部 MCF 成功之后 --------
    for each multi-port origin O (最初 origin，非临时 segment):
        run origin-level maze (§9)
        compare wire length with MCF; maybe replace whole origin result (§9.3)
        # maze 不更新 tier

    return final routing result
```

**时序要点：**

- Steiner **必须在 BusMCF 之后**执行，建图时使用 **残余容量** `capacity - used^{Bus}`。
- SimpleMCF 失败仍触发外层 `tier++` 并重跑 SAT；**maze 救回不改变 tier**（maze 在整轮 SimpleMCF 已成功的前提下作为后处理；若需 maze 弥补当轮 MCF 失败，见 §9.2 与外层 retry 的关系）。

---

### 4. 多端口 net 的 Steiner 输入分组

#### 4.1 `TrackToBumpsNet`

与 `build_records` 一致：一个 origin 对应一个 driver `end_track` 与多个 bump child（`from_track_to_bumps_split`）。Steiner 实例：

- `source`：driver track 的 MCF 节点；
- `sinks`：各 bump 经 SAT 分配后的 TOB/track 端点节点；
- `bbox_H`：该 origin 在第七版下的 `RectHull`（各 child 路径 bbox），或拆解前单组 bbox 规则。

#### 4.2 `TracksToBumpsNet` / `PNnet`

SAT 成功后，每条 `PNnet` child 有确定的 `start_track` 与 `Y_{n,r_e}` 选定的 `end_track`。TOB SAT 保证：**不会把与某 `end_track` 不同 COBUnit 的 bump 分配到该 track**。因此按 **`end_track` 分组**：

- 同一 `end_track` 下的全部 bump 与该校验 track 构成一个 **类 `TrackToBumpsNet`** 的 Steiner 实例（1 track → 多 bump）；
- 不同 `end_track` 各做一个 Steiner 实例。

**PNnet 在 SimpleMCF 中的端点（第八版）：**

- 不再创建 `virtual_Pnode` / `virtual_Nnode`；
- 每条已确定 `end_track` 的 child，其 commodity `snk` 为 SAT 选定的 track 节点（与第七版 SAT 结果一致）；
- 参与 Steiner 分组时，以分组后的 **track 源 + 多 bump 汇** 建 Steiner；未进入任何成功拆解分组的 child 仍按 §7.3 留在 origin 建模中。

---

### 5. Steiner 建图规则（`G_view`）

Steiner 与 SimpleMCF 使用同一套 **track 级物理连通语义**：

- 邻接来自 `Interposer::adjacent_tracks`（与第七版 path precompute 一致）；**不刻意区分有向/无向**，按相邻 track 建边即可；
- 节点集合：落在 `bbox_H` 内的节点，以及 origin 的 **endpoint-exception** 节点（第七版 §6.2 commodity-specific 端点豁免）；
- 边过滤：
  1. 两端点均在 `G_view` 中；
  2. **BusMCF 后残余边容量为 0 的边剔除**；
  3. 与第七版 §3.2 相同的 **简单路径** 与 **Wilton 防打转** 约束（在 Steiner 的 Dijkstra / ShortestPath 中施加）；
  4. 非法 class P/N 边、virtual 边规则与 MCF 一致。

**`CanBeBranchNode(v)`：** 仅 **普通 track 图节点** 可为 Steiner 分支点；**bump 节点、TOB 连接器节点、virtual 节点** 不可。

**`EdgeCost(e)`：** 与 MCF 边代价一致（通常为 1）；用于终端闭包 MST 与展开后子图去环。

**`SymmetricDistance(d_uv, d_vu)`：** 取双向可达距离的最小值（若仅单向可达则取该向），用于终端度量图。

---

### 6. Steiner 拆解算法

对每个 Steiner 实例 $H$ 调用 `GraphSteinerDecomposeTo2PinSegments_NoCachedPath`：在 `bbox_H` 内建 `G_view`，对终端集做度量闭包 → MST → 回展开为路径树 → 剪非终端叶 → 压缩 degree-2 节点，保留 **终端** 与 **合法 degree≥3 分支点**，相邻 important 节点之间形成 **2-pin segment**（含 `guide_path`、`bbox`、`edges`）。

**失败与 fallback：**

- 任一步标记 `requiring fallback`（终端不可达、分支点非法、展开路径为空等）→ 该 Steiner 实例**整体拆解失败**；
- 该实例涉及的端口 **全部** 保留在第七版 **origin 组** 中参与 SimpleMCF（§7.3）；
- **部分拆解**（仅当算法能对**端口子集**成功时；当前伪代码以实例为单位成功/失败）：同一物理 origin 在 SimpleMCF 中可同时存在 **已拆解的临时 2-pin commodity** 与 **残余 origin 组**（见 §7.2）。实现上若按实例全成功/全失败，则「部分」体现在 **同一 origin 下多个 Steiner 实例**（例如多个 `end_track` 分组）中一部分成功、一部分失败。

拆解算法伪代码：

```text
Algorithm: GraphSteinerDecomposeTo2PinSegments_NoCachedPath

Input:
    G = (V, E)                         // track-level routing resource graph (residual after Bus)
    H                                  // one multi-terminal Steiner instance (§4)
    source                             // source track node of H
    sinks = {t1, t2, ..., tk}          // sink nodes (bumps / track endpoints)
    bbox_H                             // bbox after TOB SAT (§5)
    EdgeCost(e)
    CanBeBranchNode(v)

Output:
    segments                           // 2-pin sub-connections
    guide_tree_edges
    branch_nodes

Procedure GraphSteinerDecomposeTo2PinSegments_NoCachedPath(
    G, H, source, sinks, bbox_H
):

    // Step 1: construct G_view inside bbox_H (§5)

    G_view ← empty graph

    for each node v in G.V:
        if v is inside bbox_H or v is an endpoint-exception node of H:
            G_view.V.insert(v)

    for each edge e = (u, v) in G.E:
        if u not in G_view.V or v not in G_view.V:
            continue
        if e is illegal for H:          // §5: Wilton, simple path, class, zero residual cap
            continue
        G_view.E.insert(e)
        weight[e] ← EdgeCost(e)

    // Step 2: collect terminals
    terminals ← {source} ∪ sinks
    if |terminals| <= 1:
        return empty decomposition
    for each terminal t in terminals:
        if t not in G_view.V:
            return failure (fallback)

    if |terminals| == 2:
        path ← ShortestPath(G_view, source, only_sink, weight)
        if path is empty: return failure
        return SteinerDecomposition { segments = {MakeSegment(u,v,path)}, ... }

    // Step 3: terminal metric closure (Dijkstra on G_view per terminal)
    // Step 4: MST on terminal graph K
    // Step 5: expand MST edges to paths in G_view
    // Step 6: prune non-terminal leaves; guide_tree_edges ← T.E
    // Step 7: important_nodes = terminals ∪ {v | deg≥3 ∧ CanBeBranchNode(v)}
            // walk chains between important nodes → segments

    return SteinerDecomposition { origin_net=H, terminals, branch_nodes,
                                  guide_tree_edges, segments }
```

```text
Function SymmetricDistance(d_uv, d_vu):
    if d_uv < +∞ and d_vu < +∞: return min(d_uv, d_vu)
    else if d_uv < +∞: return d_uv
    else if d_vu < +∞: return d_vu
    else: return +∞

Function MakeSegment(u, v, path):
    segment.from       ← u
    segment.to         ← v
    segment.guide_path ← path
    segment.bbox       ← BoundingBox(path) with 第七版 §6.2 H/V trimming rules
    segment.edges      ← ConsecutivePhysicalEdges(path)
    return segment
```

---

### 7. 混合 SimpleMCF 建模

#### 7.1 拆解成功的 segment → 临时 2-pin commodity

- 每个 segment 生成 **临时 commodity**（实现上可不写入永久 `records` 向量，但须有稳定 `record_id` / label 供日志与 warm start 对齐；建议 `origin_uid + segment_index`）。
- **不建立** origin 级 `x^H_e` / `o^H_i`；在对应 COBUnit 的 SimpleMCF 中与普通 `Tnet` 一样仅有 **流量变量 `f`** 与 **容量约束**。
- `src`/`snk` 为 segment 两端点；**在 segment `bbox` 内可自由寻路**（不强制沿 `guide_path`）；`guide_path` 用于 warm start 与诊断。
- 与 **残余 origin 组**、其他 net **不共享 origin 耦合**；仅通过 **同一 unit 的全局边/节点容量** 竞争资源。

#### 7.2 同一 origin 的混合状态

允许：**同一最初 origin** 在某一 COBUnit 内同时存在

- 若干 **已拆解** 的临时 2-pin commodity；以及
- 一个 **残余 origin 组**（第七版 `x^H` 建模），包含拆解失败或未参与成功 Steiner 实例的 child 端口。

二者独立参与容量约束，无 `f ≤ x^H` 跨组关系。

#### 7.3 拆解失败与残余 origin 的 bbox

- 对 **整实例拆解失败** 的端口集合，仍用第七版 **origin 聚合**（`x^H` / `o^H`）；
- 残余 origin 的 MCF bbox：$BBox^{MCF}_{H'} = \mathrm{RectHull}\big(\{ BBox^{MCF}_n \mid n \in \mathrm{failed\_ports} \}\big)$，其中各 $BBox^{MCF}_n$ 来自第七版 path bbox（SAT 选定的 $(r_e,r_s)$ + §6.2 修剪）；
- **PNnet** 若整条 child 固定 `snk` 后按普通 2-pin 裁剪，则按该 child 的 path bbox；若仍在残余 origin 内，则与第七版 origin 规则一致。

#### 7.4 SimpleMCF 成败判定

与第七版一致：若某 COBUnit 内 **任一** commodity（含临时 segment、残余 origin、普通 2-pin）导致 SimpleMCF **infeasible**，则该 unit 判失败 → 按第七版收集 `fail_set` 并 `tier++`。**不因预期末尾 maze 而放宽 SimpleMCF 可行性要求**。

---

### 8. MCF warm start（第八版）

在 `--enable-steiner-decompose` 下，SimpleMCF 求解前仍构造 Gurobi warm start，规则如下：

| Commodity 类型 | warm start 方式 |
|----------------|-----------------|
| 原本就是 2-pin 的 net（`Tnet` / `Bnet` 等，非多端口拆解） | **保留第七版** `--enable-pre-routing` 语义：在 effective bbox 内 BFS/迷宫预布线 |
| Steiner **拆解成功** 的每个 segment | 按 **普通 2-pin** 预布线（与上相同，在 segment `bbox` 内） |
| **残余 origin 组**（拆解失败的端口） | 按 **第七版多扇出 origin** 方式预布线（按 child 逐条路径，写入 `f` 与 `x^H`/`o^H`） |

预布线须 respect **BusMCF 后残余容量**；后跑的 warm start 路径应更新已用边/节点，避免互相冲突（与第七版 pre-routing 在 unit 内顺序一致）。

**说明：** 第八版将 Steiner 与 warm start **解耦**：Steiner 在 Bus 之后、SimpleMCF 建模时执行；warm start 在混合模型建好后、Gurobi `optimize` 之前执行。

---

### 9. MCF 完成后的 origin 级 maze 精修

#### 9.1 范围与对象

- **仅**对 **多端口 origin**（最初 `origin_uid` / `origin_key`，即 `TrackToBumpsNet`、`TracksToBumpsNet` 及其拆分 child 所属的 origin）执行；
- 按 **拆解前的完整 origin** 做一次 maze，**不是**只补失败 segment 或只补残余 origin；
- 参考 `source/algo/router`：`hopcroft_karp` 式 TOB bump↔track 分配 + `MazeRouteStrategy` 对 `TrackToBumpsNet` / `TracksToBumpsNet` 的逐分支扩展 `begin_tracks`（共享主干）。

#### 9.2 与 SAT+MCF 的关系

- maze 允许 **舍弃本轮 TOB SAT 对该 origin 的 bump↔track 分配**，从 **TOB 资源分配** 重新做（与主 router 一致），再在 COB 上布线；
- **不更新 `tier`**：maze 结果仅作为该 origin 的 **最终布线输出** 写回；外层若已进入下一轮 `tier++` retry，仍以 SAT+tier 状态为准，除非整轮 pipeline 尚未失败。

#### 9.3 线长比较与写回

- **线长度量**须与 `test_ILP` MCF 汇总一致：对多扇出 origin 使用与 `wire_length_multi_fanout_group` **相同规则**（物理 track 节点去重 + 各 child bump 计数），**不得**用拆解后各独立 2-pin 线长简单相加与 maze 比较。
- **采纳 maze 的条件**（满足其一即可替换该 origin 的 MCF 结果）：
  1. MCF 该 origin **失败**（或当轮判定无解），maze **成功**；
  2. MCF **成功**且 maze **成功**，且 maze 线长 **严格更短**。
- **写回顺序**：先 **释放** 该 origin 在 MCF 中占用的 COB 边/节点（及 TOB connector 占用），再写入 maze 路径。
- 替换粒度：**整体替换**该 origin 在 MCF 阶段的全部结果（含已拆解 segment 与残余 origin 的合并视图），而不是只替换其中一段。

---

### 10. 外层 `tier` 与失败扩展

与第七版 **完全一致**（见第七版 §5 表格）：

| 阶段 | fail_set |
|------|----------|
| SAT UNSAT | 全部 `Tnet` 与 `PNnet`；不含 `Bnet` |
| BusMCF 失败 | 失败 `bus_key` 下全部 member records |
| SimpleMCF 失败 | 失败 unit 内全部 simple records + 失败多扇出 origin 的全部 child records |

`expand_tier(fail_set)` 语义不变。SAT 重选 `Y_{n,r_e}` 或 `start_track` 后，须 **重新** 做 §4 分组与 §6 Steiner 拆解。

**maze 精修**不参与 `fail_set`、不触发 `tier++`、不在 SAT UNSAT 时替代扩 tier。

---

### 11. 设计 rationale 与边界

**为何独立 2-pin 不强制成树？**  
Steiner 仅用于缩小 bbox、减少 origin 变量；SimpleMCF 内各 segment 独立求解以换速度。树形连通性与全局最优由 §9 maze 在 origin 级兜底。

**为何 Steiner 在 Bus 之后？**  
建图须扣除 `used^{Bus}`，否则 Steiner /guide 路径可能与 Bus 占用冲突。

**边界与假设：**

- 未加 `--enable-steiner-decompose` 时，行为与第七版逐位一致；
- Steiner 为终端 MST 近似，不保证 Steiner 最优；
- 部分 origin 仅部分 Steiner 实例成功时，混合建模 + 末尾 maze 是必要的正确性补丁；
- maze TOB 重分配仅作用于多端口 origin，不影响其他 net 的 SAT 分配；
- Bnet / SyncNet BusMCF 逻辑不受本版影响。

---

### 12. 与第七版对照摘要

| 项目 | 第七版 | 第八版（`--enable-steiner-decompose`） |
|------|--------|----------------------------------------|
| TOB SAT / tier / path bbox | — | **保留** |
| BusMCF | — | **保留** |
| SimpleMCF 多端口 | origin `x^H`；PNnet virtual P/N | Steiner 拆 2-pin + 残余 origin；PNnet 固定 `snk` |
| Steiner 时机 | 无 | **BusMCF 之后** |
| MCF warm start | `--enable-pre-routing` | 2-pin 保留 + segment 2-pin + 残余 origin |
| MCF 后 | 无 | 多端口 origin maze，可替换 MCF |
| 失败扩展 | `tier++` | **相同** |

---

### 13. 实现检查清单（非规范性）

- [ ] CLI：`--enable-steiner-decompose`；与 `--enable-mcf-routing` 联用检查
- [ ] Steiner `G_view` 与第七版 Wilton/简单路径、Bus 残余、endpoint 例外一致
- [ ] 临时 segment commodity 的 `record_id`/日志与 warm start 映射
- [ ] 混合 SimpleMCF：segment 无 `x^H`；残余 origin 有 `x^H`
- [ ] warm start 三路分支（§8 表）
- [ ] maze：origin 级线长与 `wire_length_multi_fanout_group` 一致；写回前释放资源
- [ ] SAT 重选后重新分组与 Steiner
