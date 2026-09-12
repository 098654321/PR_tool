# Weighted Partial MaxSAT 对比方法规格

## 1. 目的与范围

本目录实现一个独立的 Weighted Partial MaxSAT（以下简称 WMaxSAT）布线基线，用于与 `algorithm/test_ILP/` 中的 ExactSAT（SAT 可行性 + 可选 ILP 线长优化）比较，回应如下问题：

> Why not use Weighted MAXSAT directly? Hard constraint returns no partial solution when infeasible. Weighted MAXSAT would naturally handle both routability and wirelength optimization as soft clauses, potentially reaching better results.

WMaxSAT 的目标是在**不违反物理资源、TOB 开关和同步总线规则**的前提下，优先最大化完成的 source--sink pair 数，再最小化已选布线的总线长。它不执行布局、不生成 controlbits、不调用 `PathPackage::connect_all()`；运行结果仅写入 `debug.log`。

本规格指定 EvalMaxSAT 为 MaxSAT 后端；本阶段只记录其安装与调用协议，不实现后端调用代码或构建配置。

## 2. 与 `test_ILP` 的共同前端

WMaxSAT 必须保持与 `test_ILP` 相同的输入语义和硬件语义：

1. 调用 `parse::read_config(config_path, 0, false)` 解析配置，得到 `hardware::Interposer` 和 `circuit::BaseDie`。
2. 调用 `algo::build_nets(basedie.get(), interposer.get())`，不执行 placement。
3. 将 `BaseDie` 中的 net 适配为 routing net；支持范围应与 `test_ILP/scope/build_routing_nets.cc` 保持一致。遇到该实现已拒绝的 net 类型必须明确报错，不能静默跳过。
4. 构建与 `test_ILP/graph/unified_routing_graph.cc` 等价的统一有向图：COB track、TOB bump、h-line、v-line、TOB 开关、straight/swap mode，以及 PNnet 的虚拟源。
5. 延用 `test_ILP` 的 scope、精确距离域和可达状态裁剪语义：每个 pair 的 `D` 状态只在当前 scope 和允许终端距离上创建。

不得把 `Interposer::available_tracks()` 压缩成 bump-to-track 候选边。该接口无法保留 TOB 内 bump--hline、hline--vline 的独占关系，也无法表达 vline--track 的 straight/swap 互斥；这样得到的 WMaxSAT 实例不再是公平比较。

## 3. Weighted Partial MaxSAT 定义

一个 Weighted Partial MaxSAT 实例由硬子句集合 \(H\) 与带权软子句集合 \(S=\{(C_i,w_i)\}\) 构成。求解目标是：

\[
\max_{x\models H}\sum_i w_i\,[x\models C_i],
\]

等价地，最小化被违反软子句的总权重。物理合法性必须保留为硬约束；只有“需求是否完成”和“尽量少用有线长代价的物理资源”属于软目标。

本方法的优先级为：

\[
\text{先最大化 routed pair 数，后最小化 wirelength}.
\]

若采用单一加权目标，令 \(B\) 为当前实例中所有可计线长 track/bump 权重的安全上界，则：

\[
\max\quad W_R\sum_{p\in P}q_p-\sum_{n\in N}\sum_{v\in V_{\mathrm{wl}}}w_vU_{n,v},
\qquad W_R=B+1.
\]

其中 \(P\) 是全部 source--sink pair，\(q_p\) 表示 pair \(p\) 完成，\(V_{\mathrm{wl}}\) 是 track 与 bump 节点。实现时也可采用分层/两阶段优化以避免大权重；无论采用哪一种，必须保持相同的字典序语义。

## 4. 变量

### 4.1 继承 ExactSAT 的变量

以下变量及其语义与 `test_ILP/sat/unified_sat_encoder.cc` 一致：

- \(D_{s,v,d}\)：逻辑源 \(s\) 在精确距离 \(d\) 到达统一图节点 \(v\)。
- \(A_{s,u\to v,d}\)：经 TOB arc \(u\to v\) 到达 \(v\) 的 transition selector。
- \(Y_e\)：物理 TOB switch \(e\) 被使用。
- \(M_{T,k}\)：TOB \(T\) 的第 \(k\) 组 vline--track 开关的 straight/swap mode。
- Bnet 的 COBUnit selector；PNnet 的虚拟源与虚拟弧。

### 4.2 pair 完成变量 \(q_p\)

对 `PairDelayInfo` 的每个 pair \(p=(n,\mathrm{demand},\mathrm{source})\)，创建 Boolean 变量 \(q_p\)。令 \(L_p\) 为该 pair 在 `pair.delays` 中有效的 sink `D` literal 集合。仅加入硬子句：

\[
q_p\Rightarrow\bigvee_{\ell\in L_p}\ell.
\]

其 CNF 形式为：

\[
\neg q_p\lor\bigvee_{\ell\in L_p}\ell.
\]

这是对当前 `alpha_lit` 子句的**唯一语义替换**：`test_ILP` 当前创建 \(\alpha_p\Rightarrow\bigvee L_p\)，并在求解前把所有 \(\alpha_p\) 作为 assumption；WMaxSAT 改为创建 \(q_p\)，不再施加 assumption，而加入权重为 \(W_R\) 的软单位子句 \((q_p,W_R)\)。

不要额外加入 \(\bigvee L_p\Rightarrow q_p\)。现有 ExactSAT 的 \(\alpha\) 也不是双向等价；保持单向形式能保证除目标替换外其余可行域不变。

### 4.3 资源使用变量 \(U_{n,v}\)

仅对计入论文 wirelength 的物理节点创建 \(U_{n,v}\)：

- track：\(w_v=1\)；
- bump：\(w_v=1\)；
- h-line、v-line、virtual source：不创建线长软子句。

\(U_{n,v}=1\) 表示 net \(n\) 的实际路由树使用了物理资源 \(v\)。对每一个非根的有效 \(D_{s,v,d}\)，加入：

\[
D_{s,v,d}\Rightarrow U_{n,v}.
\]

对一个物理 source 根 \(s\)，不得由固定常量 \(D_{s,s,0}=1\) 推出 \(U_{n,s}=1\)。而应由该 net 至少有一个完成的 pair 激活：

\[
\left(\bigvee_{p\in P(n,s)}q_p\right)\Rightarrow U_{n,s},
\]

其中 \(P(n,s)\) 仅包含以该物理 source \(s\) 为根的 pair，不能误用 net 的全部 pair；同步 bus 的各 bit 可以具有不同 source。PNnet 的根是虚拟节点，不计线长；其实际被选 source track 由距离 1 的非根 `D` 状态激活 \(U\)。若 `D` 与实际路径选择的关系在实现中扩展，\(U\) 也可定义成所有已选入射路径边的析取；无论具体辅助编码如何，都必须满足：未完成的 net 不因逻辑根状态支付 source bump/track 的线长。

对每个 \(U_{n,v}\) 加入软子句：

\[
(\neg U_{n,v},w_v).
\]

当一条路径使用 \(v\) 时，\(U_{n,v}=1\) 使该软子句被违反，代价恰为 \(w_v\)。同一多汇 net 的共享主干只有一个 \(U_{n,v}\)，因此只计一次；不同 net 的资源独占仍由硬约束保证。

## 5. 硬约束

除第 4.2 节把 \(\alpha\) 换成 \(q\) 外，以下约束必须按 `test_ILP` 原语义保留为硬约束：

1. 根状态、其他逻辑源禁入、`D` 的距离零和正距离常量约束。
2. 非根 `D` 状态的递归连通性；TOB arc 用 \(A\) 连接，普通 arc 用前一距离层的 \(D\) 连接。
3. `A -> D`、TOB switch 使用变量 \(Y\) 与 transition selector 的关系。
4. 不同逻辑源/距离状态的节点排他性，以及 Bnet COBUnit 的 ExactlyOne 选择。
5. bump--hline、hline--vline 开关的 AtMostOne，vline--track 的 straight/swap mode 一致性。
6. PNnet 的虚拟源、候选 track 及其物理 track 仅在距离 1 可达的限制。
7. `encode_bus_sync_constraints` 的原始等长约束。

### 5.1 同步 bus：当前确定的语义

按当前决定，bus 等长约束不加 \(q\) 门控，保持 `encode_bus_sync_constraints` 完全一致。对同一同步 bus 的任意成员 pair，在每个可表示的距离 \(d\) 上保持：

\[
D_{s_1,t_1,d}\Leftrightarrow D_{s_i,t_i,d}.
\]

缺失的任一侧 literal 按当前实现强制另一侧为 false。这意味着：如果一个 bus bit 的 \(q_p\) 被满足，它会强制对应 sink 的某个 \(D\) 为真；原始 bus 等长约束继而强制该 bus 的其它 bit 在同一距离可达。因此在当前规格下，**同步 bus 的物理路由仍是全 bus 原子完成或不完成**。

虽然每个 bit 都有独立软 \(q_p\)，一旦某个 bit 可完成，所有其它 bit 的 `D` 也会成为真，MaxSAT 会无额外线长代价地把所有相应 \(q_p\) 设为真并取得其软权重。因此，该精确保留的等长编码不会产生“只路由一部分 bus bit”的最终解。

若未来确实要允许部分 bus，则必须修改这条硬等价约束，例如用 \(q_i\land q_j\) 对等长约束加门控；这与“除 \(\alpha\to q\) 外其它全部不动”的当前规格冲突，故本版本不采用。

## 6. WCNF 输出语义

WCNF 使用一个严格大于所有软权重之和的 `TOP` 值：

- 每个硬 CNF clause 以 `TOP` 写出；
- 每个 pair 完成软子句写作 `W_R q_p 0`；
- 每个线长软子句写作 `w_v -U_{n,v} 0`。

必须在写 header 前确定变量数、硬/软 clause 数和安全的 `TOP`。所有权重与求和使用检查过溢出的无符号 64 位整数；若无法安全表达 \(W_R\) 或 `TOP`，编码器必须报错，不能静默截断。

## 7. 推荐实现拆分

目录建议保持独立，避免修改 `test_ILP` 的行为：

```text
algorithm/weighted_maxsat/
  spec.md
  main.cc
  wmaxsat_cli.hh / .cc
  net_adapter.hh / .cc
  unified_graph.hh / .cc
  domain.hh / .cc
  wmaxsat_encoder.hh / .cc
  wcnf_writer.hh / .cc
  result_extract.hh / .cc
  route_log.hh / .cc
  test/
```

实施顺序：

1. 新增 `weighted_maxsat` xmake target，并写 CLI 骨架：`<config_path> [-v|-vv] [-o DIR] [-s S] [-d D]`。启动后建立 `-o/debug.log`。
2. 接入 parser 和 `algo::build_nets`；打印 mode、net 类型、pair 数和不支持类型。
3. 从 `test_ILP` 提取或等价复用 net adapter、统一图、scope 和 delay precompute。第一版必须逐项对照 graph 节点数、arc 数、pair 数与 `test_ILP` 日志，证明前端一致。
4. 建立变量编号器和 CNF clause collector。先实现全部硬约束；对小 case，将每个 \(q_p\) 当作硬单位子句，所得可行性结果应与相同 scope/delay 下的 `test_ILP` 一致。
5. 新增 \(q\)、\(U\) 和 WCNF writer；验证“所有 \(q\) 硬化”与第 4 步结果一致。
6. 接入 MaxSAT 后端后，解析模型，按现有 `sat_solution_extract` 的反向回溯规则恢复路径。不得写回 interposer、不得输出 controlbits。
7. 仅在 `debug.log` 输出路径、每个 pair 的 \(q\)、总 routed pair 数、总线长、硬/软 clause 数、变量数、编码/求解/总时间和最优性状态。

## 8. 结果与实验口径

日志及最终实验表必须区分：

- `optimal-complete`：所有 \(q_p=1\)，且 MaxSAT 后端证明最优；
- `optimal-partial`：硬约束可满足，但最优模型仍有 \(q_p=0\)；
- `time-limit`：只有 incumbent，不能声称最优；
- `hard-unsat`：空布线都无法满足硬约束，通常表示编码或输入结构错误，而非普通“不可全布通”。

在 partial 结果中，`routed_wirelength` 只能解释为已完成部分的资源长度，不能与完整解总线长直接比较。对比 SAT+ILP 时必须固定输入、mode、placement 状态、scope/delay 策略、时限和内存上限。

## 9. EvalMaxSAT 后端

### 9.1 选择与职责

本方法使用 [EvalMaxSAT](https://github.com/normal-account/EvalMaxSAT2022) 求解 Weighted Partial MaxSAT。EvalMaxSAT 是以 CaDiCaL 为底层 SAT oracle 的现代 C++ MaxSAT 求解器，支持 weighted soft clauses、WCNF 输入、最优代价及布尔模型输出。项目自己的 CaDiCaL wrapper 不承担 MaxSAT 优化；它仍可继续供 `test_ILP` 使用。

第一版采用**命令行后端**，而非把 EvalMaxSAT 链接进 `weighted_maxsat` target。这样可避免 EvalMaxSAT 内部 CaDiCaL 与项目现有 CaDiCaL 的版本、符号和 xmake/CMake 构建冲突，也使 WCNF 成为可复现实验输入。

### 9.2 安装位置

用户负责将 EvalMaxSAT 安装在仓库内：

```text
third_party/
  EvalMaxSAT/
    CMakeLists.txt
    build/
      EvalMaxSAT_bin
```

推荐的安装流程如下（在 `PR_tool` 根目录执行）：

```bash
git clone --recursive https://github.com/normal-account/EvalMaxSAT2022.git third_party/EvalMaxSAT
cmake -S third_party/EvalMaxSAT -B third_party/EvalMaxSAT/build -DCMAKE_BUILD_TYPE=Release
cmake --build third_party/EvalMaxSAT/build --config Release
```

安装后，WMaxSAT 的默认可执行文件路径为：

```text
third_party/EvalMaxSAT/build/EvalMaxSAT_bin
```

应记录实际使用的 EvalMaxSAT commit hash、编译器和构建类型至实验日志，以保证结果可复现。若可执行文件位置因平台或 CMake generator 不同而变化，后续 CLI 应提供 `--solver <path>` 覆盖默认路径；路径不存在时必须报错，不得回退为普通 SAT。

### 9.3 WCNF 调用与结果解析

编码器将 WCNF 写入运行输出目录下的临时文件，例如 `wmaxsat_instance.wcnf`，再调用：

```text
third_party/EvalMaxSAT/build/EvalMaxSAT_bin <temporary.wcnf>
```

调用器必须捕获 stdout、stderr 和退出状态，并识别标准 MaxSAT 输出：

- `s OPTIMUM FOUND`：已证明当前模型最优；
- `s SATISFIABLE`：获得 incumbent，但未证明最优；
- `s UNSATISFIABLE`：硬子句不可满足；
- `o <cost>`：被违反软子句的最小加权代价；
- `v ...`：DIMACS 变量赋值，用于提取 \(q\)、\(U\)、\(D\)、\(A\)、\(Y\) 和 \(M\)。

若没有完整 `v` 模型，或状态与模型不一致，运行必须判为失败，不能据此输出路由结果。求解结束后默认删除临时 WCNF；`debug.log` 必须记录其路径、SHA-256、变量/子句统计、EvalMaxSAT 路径、commit hash、状态、目标代价和 stdout/stderr 摘要。后续如需保留实例以复现实验，应显式提供 `--keep-wcnf`，而不是默认产生额外输出文件。
