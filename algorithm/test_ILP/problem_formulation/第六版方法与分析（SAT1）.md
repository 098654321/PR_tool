## 第六版方法与分析(SAT + ILP-based MCF)

这个版本保留之前的思路：先给TOB分配资源，再使用ILP-based MCF建模并求解COB阵列上的布线问题。但是之前使用ILP求解TOB资源的速度太慢，而且之前只是想得到一个可行解，所以这个过程可以使用SAT代替。代替之后，与ILP-based MCF的接口不变，仍然得到每一个bump分配的track。之后的MCF流程也需要修改一下，在求解的时候给每个net加一个范围，这个范围与分配TOB资源时每个net自己的bounding box范围一样，从而默认超出这个范围的布线资源这条net是不使用的。理由是，既然TOB分配资源时确定的分配结果基于这样的范围限制，那么求解路径的时候就应该继承这个思想。

之前的TOB资源分配之前会划一个范围，在范围内求解可行的track集合，然后在这个集合当中给micro-bump分配track。但是现在发现这种分配可能会失败，说明这个范围画的太小。但是如果范围扩大，得到的路径长度可能会变长。所以做一个尝试：先用小范围求解，如果无解就扩大范围重新求解，直到有解为止。这样做的前提是建出来的SAT问题求解速度可以比较快。



### 对“第二版方法与分析”中全局TOB上资源分配步骤的约束条件和变量定义做出修改

第三～五版都是修改的COB阵列上MCF的建模与求解，TOB上资源分配的方法目前实现的是第二版中的内容。

#### 0. SAT求解器

使用CaDiCal求解当前的纯SAT问题

#### 1. SAT 记号约定

为了把约束写清楚，先定义几个 SAT 中常用的缩写。

对于一组 Boolean 变量 $X=\{x_1,x_2,\dots,x_m\}$：

$$
AtLeastOne(X) := x_1 \vee x_2 \vee \cdots \vee x_m
$$

$$
AtMostOne(X) := \bigwedge_{1\le a<b\le m}(\neg x_a \vee \neg x_b)
$$

$$
ExactlyOne(X) := AtLeastOne(X)\wedge AtMostOne(X)
$$

其中 `AtMostOne` 可以用 pairwise clauses 表示；如果变量很多，也可以实现为 sequential counter 或 cardinality network，但语义仍然是“最多一个为 true”。

记一个 bump pin 为：

$$
p=(t,b,g,i)
$$

记：

$$
v(j,k)=8j+k
$$

TOB 中每个vert line到track到交叉开关映射写成两个辅助函数：

$$
r_{\text{straight}}(b,v),\quad r_{\text{swap}}(b,v)
$$

其中 $r_{\text{straight}}(b,v)$ 表示 bank $b$ 的第 $v$ 条 vertical line 在 straight 模式下连接到的 track，$r_{\text{swap}}(b,v)$ 表示 swap 模式下连接到的 track。满足关系：

$$
r_{\text{straight}}(0,v)=v,\quad r_{\text{swap}}(0,v)=v+64
$$

$$
r_{\text{straight}}(1,v)=v+64,\quad r_{\text{swap}}(1,v)=v
$$

#### 2. Boolean 变量

保留第二版中的核心决策变量，只把 0/1 改成 false/true。

$$
W_{t,b,g,i,j,k}\in\{\text{false},\text{true}\}
$$

含义：如果 TOB $t$ 的 bank $b$ 中，group $g$ 的第 $i$ 个 bump 通过第 $j$ 根 horizontal line 连接到第 $j$ 组中的第 $k$ 根 vertical line，则为 true。

$$
S_{t,v}\in\{\text{false},\text{true}\}
$$

含义：TOB $t$ 上第 $v$ 对 vertical line 的 mux 状态。$S_{t,v}=\text{true}$ 表示 straight 模式，$S_{t,v}=\text{false}$ 表示 swap 模式。

对于 Pnet/Nnet的选择变量：

$$
Y_{n,r}\in\{\text{false},\text{true}\},\quad n\in Pnet\cup Nnet,\ r\in Endtrack_n
$$

含义：Pnet/Nnet $n$ 选择 $r$ 作为 end track。

为了让 SAT 约束更清楚，增加以下 Tseitin 辅助变量。它们不改变建模语义，只是把原来 ILP 里的乘积和求和关系拆成 CNF 方便求解。

$$
Q^{str}_{t,b,g,i,j,k}\in\{\text{false},\text{true}\}
$$

$$
Q^{swp}_{t,b,g,i,j,k}\in\{\text{false},\text{true}\}
$$

其中：

$$
Q^{str}_{t,b,g,i,j,k}\Leftrightarrow W_{t,b,g,i,j,k}\wedge S_{t,v(j,k)}
$$

$$
Q^{swp}_{t,b,g,i,j,k}\Leftrightarrow W_{t,b,g,i,j,k}\wedge \neg S_{t,v(j,k)}
$$

再定义辅助变量：

$$
A_{p,r}\in\{\text{false},\text{true}\}
$$

含义：bump pin $p=(t,b,g,i)$ 最终被 TOB 分配到 track $r$。

#### 3. 每个 bump pin 必须选择唯一 TOB 内连接

对于所有出现在 net 端点中的 bump pin $p=(t,b,g,i)$，必须且只能选择一个 $(j,k)$：

$$
ExactlyOne\left(\{W_{t,b,g,i,j,k}\mid j\in J,\ k\in K\}\right)
$$

展开为 CNF：

$$
\bigvee_{j\in J,k\in K} W_{t,b,g,i,j,k}
$$

以及对任意两个不同选择 $(j,k)\ne(j',k')$：

$$
\neg W_{t,b,g,i,j,k}\vee \neg W_{t,b,g,i,j',k'}
$$

这个约束对应原 ILP 中：

$$
\sum_{j\in J}\sum_{k\in K}w_{P,j,k}=1
$$

#### 4. horizontal line 互斥约束

同一个 TOB、同一个 bank、同一个 group 内，一根 horizontal line 最多只能被一个 bump 使用，并且最多只能连到一根 vertical line。

对所有 $t\in T,b\in B,g\in G,j\in J$：

$$
AtMostOne\left(\{W_{t,b,g,i,j,k}\mid i\in I,\ k\in K\}\right)
$$

展开为 CNF：对任意两个不同的 $(i,k)\ne(i',k')$：

$$
\neg W_{t,b,g,i,j,k}\vee \neg W_{t,b,g,i',j,k'}
$$

这个约束对应原 ILP 中：

$$
\sum_{i\in I}\sum_{k\in K}w_{t,b,g,i,j,k}\le 1
$$

#### 5. vertical line 互斥约束

同一个 TOB、同一个 bank 内，一根 vertical line 最多只能被一个 bump/horizontal line 使用。

对所有 $t\in T,b\in B,j\in J,k\in K$：

$$
AtMostOne\left(\{W_{t,b,g,i,j,k}\mid g\in G,\ i\in I\}\right)
$$

展开为 CNF：对任意两个不同的 $(g,i)\ne(g',i')$：

$$
\neg W_{t,b,g,i,j,k}\vee \neg W_{t,b,g',i',j,k}
$$

这个约束对应原 ILP 中：

$$
\sum_{g\in G}\sum_{i\in I}w_{t,b,g,i,j,k}\le 1
$$

#### 6. 定义 straight/swap 辅助变量

对每个 $p=(t,b,g,i)$ 和每个 $(j,k)$，令 $v=v(j,k)$。

定义：

$$
Q^{str}_{t,b,g,i,j,k}\Leftrightarrow W_{t,b,g,i,j,k}\wedge S_{t,v}
$$

其 CNF 为：

$$
\neg Q^{str}_{t,b,g,i,j,k}\vee W_{t,b,g,i,j,k}
$$

$$
\neg Q^{str}_{t,b,g,i,j,k}\vee S_{t,v}
$$

$$
Q^{str}_{t,b,g,i,j,k}\vee \neg W_{t,b,g,i,j,k}\vee \neg S_{t,v}
$$

定义：

$$
Q^{swp}_{t,b,g,i,j,k}\Leftrightarrow W_{t,b,g,i,j,k}\wedge \neg S_{t,v}
$$

其 CNF 为：

$$
\neg Q^{swp}_{t,b,g,i,j,k}\vee W_{t,b,g,i,j,k}
$$

$$
\neg Q^{swp}_{t,b,g,i,j,k}\vee \neg S_{t,v}
$$

$$
Q^{swp}_{t,b,g,i,j,k}\vee \neg W_{t,b,g,i,j,k}\vee S_{t,v}
$$

这一步等价于原 ILP 中对 $q^{straight}=w\cdot s$ 和 $q^{swap}=w\cdot(1-s)$ 的线性化。

#### 7. 定义 bump 最终分配到的 track

对每个 bump pin $p=(t,b,g,i)$ 和每条 track $r\in R$，定义候选 literal 集合：

$$
L_{p,r}=
\{Q^{str}_{t,b,g,i,j,k}\mid r_{\text{straight}}(b,v(j,k))=r\}
\cup
\{Q^{swp}_{t,b,g,i,j,k}\mid r_{\text{swap}}(b,v(j,k))=r\}
$$

然后定义：

$$
A_{p,r}\Leftrightarrow \bigvee_{\ell\in L_{p,r}}\ell
$$

其 CNF 为：

$$
\neg \ell\vee A_{p,r},\quad \forall \ell\in L_{p,r}
$$

以及：

$$
\neg A_{p,r}\vee \bigvee_{\ell\in L_{p,r}}\ell
$$

如果 $L_{p,r}=\emptyset$，则直接加入：

$$
\neg A_{p,r}
$$

为了增强 SAT 传播，也显式加入：

$$
ExactlyOne\left(\{A_{p,r}\mid r\in R\}\right)
$$

这个约束表示每个 bump pin 最终必须且只能落到一条 track 上。虽然它理论上可以由 $W,S,Q$ 推出，但显式加入后可以减少无意义搜索。

#### 8. 消除未使用 vertical mux 的等价解

第五版分析中指出：当某个 vertical mux pair 没有被任何 bump 使用时，$S_{t,v}$ 的 true/false 都没有物理意义，会产生大量等价解。SAT 中可以直接加入 symmetry-breaking 约束：

令 $j(v)=\lfloor v/8\rfloor$，$k(v)=v\bmod 8$。对所有 $t\in T,v\in V$：

$$
S_{t,v}\Rightarrow
\bigvee_{b\in B,g\in G,i\in I}W_{t,b,g,i,j(v),k(v)}
$$

CNF 为：

$$
\neg S_{t,v}\vee
\bigvee_{b\in B,g\in G,i\in I}W_{t,b,g,i,j(v),k(v)}
$$

含义是：如果没有任何 bump 使用这个 vertical line pair，则强制 $S_{t,v}=\text{false}$。这个约束不限制被使用的 mux 选择 straight 或 swap，只消除未使用 mux 的等价状态。

#### 9. Bnet 的可达性约束

对于 $n\in Bnet$，两个端点都是 bump。记：

$$
p_s=n.Pstart,\quad p_e=n.Pend
$$

首先，终点 bump 只能选择 $Endtrack_n$ 中的 track：

$$
\neg A_{p_e,r},\quad \forall r\notin Endtrack_n
$$

其次，如果终点 bump 选择了 $r_e$，那么起点 bump 只能选择 $Starttrack_{n,r_e}$ 中的 track。因此对所有非法组合加入 forbidden-pair clause：

$$
\neg A_{p_s,r_s}\vee \neg A_{p_e,r_e},
\quad
\forall r_e\in Endtrack_n,\ \forall r_s\notin Starttrack_{n,r_e}
$$

这个约束对应第二版 ILP 中：

$$
u_{n.Pstart,r_{start}}+u_{n.Pend,r_{end}}\le 1
$$

但是 SAT 形式更直接：把所有不可达的 start/end track 组合直接禁止。

#### 10. Tnet 的可达性约束

对于 $n\in Tnet$，终点 track 已知，记：

$$
r_e=n.Pend,\quad p_s=n.Pstart
$$

起点 bump 只能选择可以到达该终点 track 的 start track：

$$
\neg A_{p_s,r_s},
\quad
\forall r_s\notin Starttrack_{n,r_e}
$$

#### 11. Pnet/Nnet 的 end track 选择与可达性约束

对于 $n\in Pnet\cup Nnet$，起点是 bump，终点是一组可选 track。记：

$$
p_s=n.Pstart
$$

首先，Pnet/Nnet 必须且只能选择一个 end track：

$$
ExactlyOne\left(\{Y_{n,r_e}\mid r_e\in Endtrack_n\}\right)
$$

展开为：

$$
\bigvee_{r_e\in Endtrack_n}Y_{n,r_e}
$$

以及对任意 $r_e\ne r'_e$：

$$
\neg Y_{n,r_e}\vee \neg Y_{n,r'_e}
$$

然后加入可达性约束。如果选择了 end track $r_e$，则起点 bump 不允许选择任何不在 $Starttrack_{n,r_e}$ 中的 start track：

$$
\neg Y_{n,r_e}\vee \neg A_{p_s,r_s},
\quad
\forall r_e\in Endtrack_n,\ \forall r_s\notin Starttrack_{n,r_e}
$$

这个约束对应第二版 ILP 中：

$$
u_{n.Pstart,r_{start}}+y_{n,r_{end}}\le 1
$$



### 加入扩大范围的迭代

第六版计划“先用小范围求解，如果无解就扩大范围重新求解”。在 SAT 建模中，这个范围不需要额外引入变量，而是通过每一轮预计算得到的：

$$
Endtrack_n
$$

$$
Starttrack_{n,r_e}
$$

来控制。

每一轮求解流程为：

```python
for range_level in increasing_range:
    compute bounding_box(net, range_level)  # range_level = 0的时候，bounding_box就是第二版方法当中规定的范围；range_level每增加1，box上下左右的边界扩大1，但是不能超过COB阵列的真实范围
    endtrack的计算方法不变，但根据bounding_box重新计算start_tracks
    build SAT clauses using the constraints above
    solve SAT
    if SAT:
        extract assigned_track(p)
        break
        
# 其中increasing_range = [0,1,2,3,4]
# 重新计算start_tracks的方法如下：
# 1. 如果是range_level=0，保持原来的计算方法
# 2. 如果range_level > 0但range_level < 4，在新的bounding_box当中计算从end_track到start_track所在Channel（也就是TOB位置处的那个Channel）的k-shortest path，然后看一下能够到达的track，把track加入start_track集合。其中k=1+2*range_level。注意，这些新加入的start_track并没有计算reach信息，因为暂时不需要
# 3. 如果range_level=4，对于每一个end_track，其对应的start_track可以是与end_track位于相同COBUnit下的所有track（应该是8条），同样不需要计算reach信息
```

如果某一轮范围太小，SAT 返回 UNSAT，说明在当前范围内不存在满足 TOB 资源互斥和可达性约束的 track 分配。扩大范围后，只需要重新生成 `Endtrack`、`Starttrack` 和对应 forbidden clauses。



### SAT 求解结果的信息整合

SAT 求解成功后，对每个 bump pin $p$，读取唯一满足的：

$$
A_{p,r}=\text{true}
$$

并令：

$$
assigned\_track(p)=r
$$

然后后续步骤保持第五版不变：

```C++
COBUnit = r < 64 ? r % 8 : r % 8 + 8
```

也就是说，`allocateNettoCOBUnit()` 仍然根据 SAT 输出的 assigned track 决定 net 被分配到哪个 COBUnit。

对于 Pnet/Nnet，若后续需要记录本次 SAT 选择的 endpoint，也可以读取唯一满足的：

$$
Y_{n,r_e}=\text{true}
$$

但当前第五版后续 MCF 主要依赖起点 bump 被分配到的 track，以及原始 Pnet/Nnet 允许连接的端口集合。因此 $Y$ 可以只作为 TOB SAT 可达性约束中的辅助选择信息，不一定要改变后续数据结构。



***



### 对“第五版方法与分析”中ILP-based MCF求解流程

第六版不改变第五版中 BusMCF 和 SimpleMCF 的整体求解顺序，也不改变 `allocateNettoCOBUnit()`、`makeNodeforPNnet()`、`merge_multi_fanout_net()`、`setStartEndforNet()` 等接口。修改点只有一个：MCF 建模时不再默认每条 net 都可以使用整个

$$
G^c=(V^c,E^c)
$$

而是先为需要限制范围的 net 构造自己的 MCF 可行图，再只在这个可行图内建立流量变量、占用变量和相关约束。范围外的边不建立变量，也就等价于该 net 不能使用范围外的布线资源。

#### 1. MCF 范围的来源

TOB SAT 在某一轮 `range_level` 下求解成功后，对每条拆分后的 net 都已经使用了这一轮的：

$$
bounding\_box(n, range\_level)
$$

MCF 阶段沿用同一轮 `range_level`，并按照下面方式得到 MCF 使用的范围。

1. 对于普通 2-pin net，包括 Bnet 和 Tnet，MCF 范围与 TOB SAT 阶段保持一致：

$$
BBox^{MCF}_n=bounding\_box(n, range\_level)
$$

2. 对于 SimpleMCF 中来自同一个 TrackToBumpsNet 的多扇出 origin net $H$，先对每个 child net $n\in H.child\_net$ 按照 1 中的方法得到 $BBox^{MCF}_n$，然后取这些范围的最小外接矩形：

$$
BBox^{MCF}_H
=
RectHull\left(\{BBox^{MCF}_n\mid n\in H.child\_net\}\right)
$$

其中 `RectHull` 表示能覆盖所有 child bounding box 的最小矩形，并且矩形边界不能超过真实 COB 阵列范围。为了允许多个 child net 在 SimpleMCF 中共享树形主干，属于同一个 origin net $H$ 的所有 child flow 都使用 $BBox^{MCF}_H$，而不是只使用各自单独的 $BBox^{MCF}_n$。

3. 对于 SimpleMCF 中的 Pnet/Nnet，本版暂时不加 bounding box 限制，保持第五版原来的求解方式。原因是 Pnet/Nnet 的终点是 `virtual_Pnode` 或 `virtual_Nnode`，它们连接的是当前 COBUnit 内一组可选端口；如果不改变后续数据结构，就不应该在 MCF 阶段只保留 SAT 中某一个 $Y_{n,r_e}$ 对应的 endpoint。因此：

$$
G^{c,MCF}_n=G^c,\quad n\in Pnet^c\cup Nnet^c
$$

4. 对于 BusMCF，同一条 bus 内的所有 2-pin net 共用一个范围。这个范围取该 bus 中所有 2-pin net 的 bounding box 的最小外接矩形：

$$
BBox^{MCF}_{Bus}
=
RectHull\left(\{BBox^{MCF}_n\mid n\in Bus\}\right)
$$

同一条 bus 内所有 net 都使用 $BBox^{MCF}_{Bus}$。这样可以避免同一条 bus 中不同 bit 使用不同范围，导致后面的同步线长约束更容易无解。

#### 2. 每条 net 的 MCF 可行图

对于每个 COBUnit $c$ 下的 net $n$，定义它在 MCF 中实际允许使用的可行图：

$$
G_n^c=(V_n^c,E_n^c)
$$

如果 $n$ 需要 bounding box 限制，则有向边集合定义为：

$$
E_n^c
=
\{(i,j)\in E^c\mid \text{physical edge } e=\{i,j\} \text{ 落在 } BBox^{MCF}_n \text{ 内}\}
$$

同时定义对应的物理无向边集合：

$$
\overline{E}_n^c
=
\{e=\{i,j\}\mid (i,j)\in E_n^c \lor (j,i)\in E_n^c\}
$$

节点集合需要同时包含这些边覆盖到的物理节点，以及该 net 自己的起点和终点：

$$
V_n^c
=
\{i\mid \exists j,(i,j)\in E_n^c \lor (j,i)\in E_n^c\}
\cup \{n.Pstart,n.Pend\}
$$

这样即使某一轮 range 裁剪后起点或终点暂时没有被任何可行边覆盖，流量守恒约束仍然会在该端点上生效，并使这一轮 MCF 正确地判定为 infeasible。

对于 SimpleMCF 中的 origin net $H$，类似定义：

$$
G_H^c=(V_H^c,E_H^c)
$$

其中 $E_H^c$ 由 $BBox^{MCF}_H$ 得到，并定义对应的物理无向边集合：

$$
\overline{E}_H^c
=
\{e=\{i,j\}\mid (i,j)\in E_H^c \lor (j,i)\in E_H^c\}
$$

origin net $H$ 的节点集合也需要包含所有 child net 的起点和终点：

$$
V_H^c
=
\{i\mid \exists j,(i,j)\in E_H^c \lor (j,i)\in E_H^c\}
\cup
\bigcup_{n\in H.child\_net}\{n.Pstart,n.Pend\}
$$

若 $n\in H.child\_net$，则 $n$ 的流量变量也建立在 $E_H^c$ 上。

如果某一轮 `range_level` 下，某个需要求解的 net 或 child net 的起点/终点没有被对应可行图中的边覆盖到，或者该可行图中不存在从起点到终点的连通路径，则这一轮 MCF 可以直接判定为 infeasible，不必继续建立完整 ILP。

#### 3. BusMCF 中公式的范围修改

BusMCF 中，流量变量只在对应 net 的可行图上建立：

$$
f^{c,n}_{ij}\in\{0,1\},
\quad (i,j)\in E_n^c
$$

目标函数改为：

$$
\min \sum_{c}\sum_{n\in BusNet^c}\sum_{(i,j)\in E_n^c} f^{c,n}_{ij}
$$

容量约束改为：对于每一条物理无向边 $e=\{i,j\}$，只统计允许使用这条边的 net：

$$
\sum_{n\in BusNet^c:\ (i,j)\in E_n^c\lor(j,i)\in E_n^c}
\left(f^{c,n}_{ij}+f^{c,n}_{ji}\right)
\le capacity^c_e
$$

节点流量约束改为只在 $V_n^c$ 和 $E_n^c$ 上求和：

$$
\sum_{j:(i,j)\in E_n^c} f^{c,n}_{ij}
-
\sum_{j:(j,i)\in E_n^c} f^{c,n}_{ji}
=
\begin{cases}
bits_n, & i=n.Pstart\\
-bits_n, & i=n.Pend\\
0, & \text{otherwise}
\end{cases}
,\quad \forall n,\forall i\in V_n^c
$$

同步线长约束也需要使用每条 net 自己的可行边集合：

$$
total\_flow_n
=
\sum_c\sum_{(i,j)\in E_n^c} f^{c,n}_{ij}
$$

$$
total\_flow_n=total\_flow_m,
\quad \forall n,m\in the\_same\_Bus
$$

其他路径组成约束保持第五版含义不变，只是求和范围从 $E^c$ 换成对应的 $E_n^c$。没有建立的变量默认视为 0。

#### 4. SimpleMCF 中公式的范围修改

SimpleMCF 中，仍然使用第五版的 origin net 变量：

$$
x^{c,H}_e\in\{0,1\}
$$

但只在 origin net $H$ 的可行边集合 $E_H^c$ 上建立：

$$
x^{c,H}_e\in\{0,1\},
\quad e\in \overline{E}_H^c
$$

对于 $n\in H.child\_net$，流量变量只在 $E_H^c$ 上建立：

$$
f^{c,n}_{ij}\in\{0,1\},
\quad (i,j)\in E_H^c
$$

目标函数改为：

$$
\min \sum_{H\in Origin^c}\sum_{e\in \overline{E}_H^c} x^{c,H}_e
$$

$x$ 和 $f$ 的关系改为：

$$
f^{c,n}_{ij}\le x^{c,H}_e,\quad
f^{c,n}_{ji}\le x^{c,H}_e,
\quad
\forall n\in H.child\_net,\forall e=\{i,j\}\in \overline{E}_H^c
$$

$$
x^{c,H}_e
\le
\sum_{n\in H.child\_net}
\left(f^{c,n}_{ij}+f^{c,n}_{ji}\right),
\quad
\forall H\in Origin^c,\forall e=\{i,j\}\in \overline{E}_H^c
$$

节点流量约束改为只在 $E_H^c$ 上求和：

$$
\sum_{j:(i,j)\in E_H^c} f^{c,n}_{ij}
-
\sum_{j:(j,i)\in E_H^c} f^{c,n}_{ji}
=
\begin{cases}
bits_n, & i=n.Pstart\\
-bits_n, & i=n.Pend\\
0, & \text{otherwise}
\end{cases}
,
\quad
\forall n\in H.child\_net,\forall i\in V_H^c
$$

BusMCF 已使用资源之后，SimpleMCF 的残余边容量约束改为：

$$
\sum_{H\in Origin^c:\ e\in \overline{E}_H^c}
x^{c,H}_e
\le
capacity^c_e-used^{Bus,c}_e,
\quad \forall e\subseteq E^c
$$

残余节点资源约束改为：

$$
\sum_{H\in Origin^c:\ i\in V_H^c}
o^{c,H}_i
\le
1-used^{Bus,c}_i,
\quad \forall i\in V^c_{\text{phys}}
$$

其中 $V^c_{\text{phys}}$ 不包含 `virtual_Pnode` 和 `virtual_Nnode`。对于 Pnet/Nnet 连接到 virtual node 的边，仍然按照第五版的方式处理，不参与物理节点资源互斥。

#### 5. 实现时的等价理解

实现时不需要在范围外显式加入：

$$
f^{c,n}_{ij}=0
$$

更简单的做法是：范围外的边不创建对应变量；所有目标函数和约束求和时，流量变量只遍历对应的有向边集合 $E_n^c$ 或 $E_H^c$，origin 占用变量 $x$ 只遍历对应的物理无向边集合 $\overline{E}_H^c$。这样既表达了“不能走出范围”，也能减少 MCF 阶段的变量和约束数量。



***



### 应对MCF求解失败的方法

SAT 阶段只保证在当前 `range_level` 下存在满足 TOB 资源互斥和可达性约束的 track 分配，但它不保证后续 BusMCF 和 SimpleMCF 一定能布通。因此第六版的外层迭代不能只在 SAT UNSAT 时扩大范围，也需要在 MCF infeasible 时扩大范围。

整体流程改为：

```python
for range_level in increasing_range:
    compute bounding_box(net, range_level)
    compute Endtrack and Starttrack using this range_level

    build TOB SAT model
    solve TOB SAT
    if SAT is UNSAT:
        continue

    extract assigned_track(p)
    allocateNettoCOBUnit()
    prepare MCF feasible graph G_n^c or G_H^c using this range_level

    solve BusMCF
    if BusMCF is infeasible:
        continue

    solve SimpleMCF for each COBUnit
    if any SimpleMCF is infeasible:
        continue

    return final routing result

report failure
```

其中：

1. 如果 TOB SAT 返回 UNSAT，说明当前范围内不存在合法的 TOB track 分配，直接扩大 `range_level`。
2. 如果 TOB SAT 返回 SAT，但 BusMCF 无解，说明当前范围和当前 TOB 分配下，Bus 线在 COB 阵列内无法同时满足容量、节点互斥和等长约束。此时丢弃本轮 SAT 结果，令 `range_level+1`，重新从 TOB SAT 开始求解。
3. 如果 BusMCF 有解，但某个 COBUnit 的 SimpleMCF 无解，说明当前范围和当前 Bus 占用结果下，普通 net 或多扇出 net 无法布通。此时同样丢弃本轮 SAT 和 MCF 结果，令 `range_level+1`，重新从 TOB SAT 开始求解。





## 分析

### 1. 将全局范围扩展改为失败驱动的局部范围扩展

前面章节中的 `range_level` 是一个全局变量。只要求解失败，下一轮所有 net 的 bounding box 都会同时扩大。这种做法比较保守，但会带来两个问题：

1. 失败通常只和一部分 net 有关，成功 net 的范围没有必要扩大。
2. 全局扩大范围会让更多 net 获得更长的候选路径，可能增加后续 MCF 的变量数量，也可能使最终路径变长。

因此，范围扩展策略改为以 record 为单位维护。记拆分后的 2-pin record 为 $n$，为每个 record 定义独立的扩展量：

$$
\rho_n\in\{0,1,2,3,4\}
$$

其中 $\rho_n=0$ 表示使用第二版方法中的基础 bounding box。若 $\rho_n>0$，则在基础 bounding box 的上、下、左、右四条边各扩展 $\rho_n$ 格，并将结果裁剪到真实 COB 阵列范围内：

$$
BBox(n,\rho_n)=Expand(BBox_0(n),\rho_n)
$$

求解失败后，只有被判定为失败相关的 record 才执行：

$$
\rho_n\leftarrow \min(\rho_n+1,4)
$$

其余已经通过当前轮求解、且没有被归入失败集合的 record 保持原来的 $\rho_n$ 不变。这样可以把范围放宽限制在真正需要放宽的局部区域内。

### 2. TOB SAT 失败时的扩展对象

TOB SAT 返回 UNSAT 时，纯 SAT 结果通常不能直接指出是哪一条 net 导致无解。这里不对所有 record 做全局扩展，而是只扩展端口中带有固定 track 的 record。原因是这类 net 的可达空间更受端口位置约束，当前 bounding box 过小时更容易导致可选 start track 不足。

在原始 net 类型上，TOB SAT 失败时扩展以下对象：

- `TrackToBumpNet`
- `BumpToTrackNet`
- `TrackToBumpsNet`
- Pnet / Nnet

在 `test_ILP` 的 record 表示中，上述对象对应：

- `Tnet`：包括普通 `BumpToTrackNet`、`TrackToBumpNet`，以及 `TrackToBumpsNet` 拆出的 child record。如果 SyncNet 中的 BTT/TTB 子网被拆成 `Tnet`，也按同一规则处理，因为它同样带有固定 track 端点。
- `PNnet`：由 Pnet / Nnet 对应的 `TracksToBumpsNet` 拆分得到。

因此，TOB SAT 失败时的扩展集合可以写为：

$$
FailSet_{SAT}=\{n\mid type(n)=Tnet\ \lor\ type(n)=PNnet\}
$$

`Bnet` 不因为一次 TOB SAT UNSAT 被直接扩展。它的两个端点都是 bump，约束主要来自 TOB 资源互斥和两个 bump 之间的配对可达性。若后续 BusMCF 或 SimpleMCF 证明某些 `Bnet` 所在组布线失败，再由 MCF 失败规则扩展。

### 3. BusMCF 失败时的扩展对象

BusMCF 失败说明某条同步 bus 在当前 TOB 分配和当前 MCF 可行图内无法同时满足容量、节点互斥和等长约束。由于 BusMCF 的等长约束以 bus 为单位耦合多个 bit，失败不能只归因到单个 child record。

因此，BusMCF 失败时扩展失败 bus 下的所有 member records。若失败 bus 的标识为 `bus_key`，则：

$$
FailSet_{Bus}=\{n\mid bus\_key(n)=bus\_key_{fail}\}
$$

这些 record 的 $\rho_n$ 在下一轮各增加 1。其它 bus 和普通 net 不扩展。

实现上需要在 BusMCF 失败诊断中保留可定位的 `bus_key`。本策略不使用“无法定位时扩展所有 bus”的兜底规则；如果无法定位失败 `bus_key`，应当报告诊断信息不足，而不是扩大无关 bus 的范围。

### 4. SimpleMCF 失败时的扩展对象

SimpleMCF 按 COBUnit 求解。若某个 COBUnit 的 SimpleMCF 失败，说明该 unit 内的普通 net 或 origin net 在 BusMCF 已占用资源之后无法完成布线。

对于失败的 COBUnit $c$，首先收集该 unit 中参与 SimpleMCF 的所有 simple records：

$$
FailSet_{Simple,c}=\{n\mid n\text{ belongs to failed SimpleMCF unit }c\}
$$

若其中某些 record 属于多扇出 origin net $H$，则扩展该 origin 下的所有 child records，而不是只扩展失败路径中某一个 child。这样做的原因是 SimpleMCF 中多扇出 net 共享 origin-level 的 $x^{c,H}_e$ 和 $o^{c,H}_i$ 变量，多个 child flow 会共同决定树形主干是否可行。只扩展单个 child 可能破坏 origin 级共享路径的建模意图。

因此多扇出扩展规则为：

$$
FailSet_{Simple,c}
\leftarrow
FailSet_{Simple,c}\cup
\{n\mid origin(n)=H,\ H\text{ intersects failed unit }c\}
$$

若多个 COBUnit 同时失败，则对所有失败 unit 的扩展集合取并集。

### 5. 每次扩展后如何更新 start track 集合

当某个 record $n$ 的 $\rho_n$ 从 $r$ 增加到 $r+1$ 后，需要基于新的 $BBox(n,r+1)$ 重新扩展其可达 start track 集合。对每个 end track $r_e$，设上一轮已有集合为：

$$
Starttrack_{n,r_e}^{old}
$$

在新的 bounding box 内，从 $r_e$ 到 start bump 所在 TOB channel 计算最近的可达 track，并选出不在旧集合中的新 track。每次扩展期望最多加入 2 条新的 start track：

$$
NewStart_{n,r_e}
=
\text{nearest 2 tracks in }BBox(n,\rho_n)
\setminus Starttrack_{n,r_e}^{old}
$$

然后更新：

$$
Starttrack_{n,r_e}
\leftarrow
Starttrack_{n,r_e}^{old}\cup NewStart_{n,r_e}
$$

如果新范围内不足 2 条未加入过的可达 track，则加入实际能够找到的数量。一个 end track 在同一个 COBUnit 下理论上最多对应 8 条 start track。因此 $\rho_n$ 的上限取 4；当某条 record 已经扩展 4 次时，认为它已经达到本策略允许的最大候选范围。

需要注意，新增 start track 主要用于 TOB SAT 的可达性约束和 MCF 端点选择。若新增 track 没有完整的 Wilton reach step 信息，仍然可以先作为候选 start track 使用；后续需要实际恢复详细路径时，再根据选中的 start/end pair 补充或重新计算对应路径信息。

### 6. MCF 阶段如何使用局部扩展后的 bbox

MCF 阶段继续遵循前文“MCF 范围的来源”中的原则，只是把全局 `range_level` 替换为每条 record 自己的 $\rho_n$。

对于普通 2-pin net：

$$
BBox^{MCF}_n = BBox(n,\rho_n)
$$

对于 BusMCF，同一条 bus 内所有 member records 共用一个 bus bbox。该 bbox 由成员 record 当前的局部 bbox 取最小外接矩形得到：

$$
BBox^{MCF}_{Bus}
=
RectHull(\{BBox(n,\rho_n)\mid n\in Bus\})
$$

对于 SimpleMCF 中的多扇出 origin net $H$，origin bbox 同样由 child records 当前的局部 bbox 取最小外接矩形得到：

$$
BBox^{MCF}_H
=
RectHull(\{BBox(n,\rho_n)\mid n\in H.child\_net\})
$$

对于 Pnet / Nnet 对应的 `PNnet`，TOB SAT 阶段会使用 $\rho_n$ 扩展候选 start track 集合。但按照前文 MCF 建模约定，SimpleMCF 中的 Pnet / Nnet 暂时不加 MCF bounding box 限制，仍然通过 `virtual_Pnode` / `virtual_Nnode` 连接当前 COBUnit 内可选端口。也就是说，$\rho_n$ 对 `PNnet` 的主要作用是影响 TOB SAT 阶段的 start track 可选集合，而不是在 MCF 阶段裁剪 PNnet 的虚拟端口图。

### 7. 完整迭代流程

调整后的外层流程如下：

```python
for each record n:
    rho[n] = 0

for attempt in range(max_attempt):
    compute BBox(n, rho[n]) for each record n
    compute Endtrack and Starttrack using each record's own BBox(n, rho[n])

    build TOB SAT model
    solve TOB SAT

    if TOB SAT is UNSAT:
        fail_set = {n | type(n) is Tnet or PNnet}
        if expand(fail_set) changed nothing:
            report failure
        continue

    extract assigned_track(p)
    allocateNettoCOBUnit()
    prepare MCF feasible graph using BBox(n, rho[n])

    solve BusMCF
    if BusMCF is infeasible:
        fail_set = all member records of failed bus_key
        if expand(fail_set) changed nothing:
            report failure
        continue

    solve SimpleMCF for each COBUnit
    if any SimpleMCF unit is infeasible:
        fail_set = all simple records in failed units
        fail_set += all child records of failed multi-fanout origins
        if expand(fail_set) changed nothing:
            report failure
        continue

    return final routing result

report failure
```

其中 `expand(fail_set)` 表示对集合中尚未达到上限的 record 执行 $\rho_n\leftarrow \rho_n+1$。若集合中所有 record 的 $\rho_n$ 都已经等于 4，则本轮扩展没有产生任何新范围，说明当前局部扩展策略已经耗尽，应当报告失败，而不是继续重复求解同一个问题。

### 8. 该策略的预期效果与边界

这个调整保留了原方法“先小范围、失败后放宽”的思想，但把放宽对象从所有 net 缩小到失败相关 net。它的预期效果是：

1. 减少不相关 net 的候选 start track 数量，避免 SAT 搜索空间无谓增大。
2. 减少 MCF 中不相关 net 的可行边集合，避免 MCF 变量和约束无谓增多。
3. 尽量保持已经可布通 net 的短范围约束，降低最终路径变长的风险。

该策略也有一个边界条件：TOB SAT 的 UNSAT 失败不提供精确失败 net。因此这里采用工程上更稳定的近似规则，只扩展带 track 端口的 `Tnet` 和 `PNnet`。这不会保证每次扩展都是最小必要集合，但比全局扩展更有针对性，也避免了在 SAT 层引入复杂的 UNSAT core 分析。

