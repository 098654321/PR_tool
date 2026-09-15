## 关于TOB资源分配的问题

### 问题设置

给TOB每一个bump随机的分配一个COBUnit，同时确保每个COBUnit被分配到8个bump。然后基于分配好的对应关系，判断是否总是存在一组TOB开关的解，让bump可以连接到属于其对应COBUnit的一根track上。

### 结论（注意这里有个条件：TOB所有128个bump都被分配了COBUnit。实际情况不会遇到128个bump全部被使用的）

- 某个 bank 里，residue r 的 bump 数（unit 𝑟 与 unit 𝑟 + 8 之和）只要不是恰好 8，这个 bank 就布不通。

    -> “任意 bank 里，residue r 的 bump 数（unit 𝑟 与 unit 𝑟 + 8 之和）恰好是 8”是否是一定存在可行解的充要条件？（已知是必要条件）
    -> 用 CaDiCaL 对完整 TOB 开关约束做完备 SAT 求解。count-ok 抽样按权重 \(C(8,k)^2\) 选每个 residue 的拆分 \(k\)（在「每 unit 8 个 + 每 bank 每 residue 8 个」的赋值上均匀），每个 trial 独立种子。
    -> seed=20260915 的 10000 个 count-ok 实例全部 SAT，占用检查通过；wrap/rotate \(k=0..7\) 也可行。无 residue 约束的随机 8-per-unit 里 9999/10000 因 count 不合法被剪枝，剩下 1 个碰巧 count-ok 的也 SAT。仍无反例，一般情形下的充分性仍未形式证明。
    **寻找形式证明，然后看看怎么用到Global Routing里面**

- 循环后分组：((bump + k) % 128) // 8 例如 𝑘 = 1 时 unit 0 是 bump {127,0,1,2,3,4,5,6}，跨了两个 bank，但每个 residue 在每个 bank 里仍恰好 8 个，SAT 能给出开关解

- 同一组 8 连号 bump，只转 unit 编号：(bump // 8 + k) % 16 这是 bump // 8 的循环换标，结构一样，也可行。

- 在 count-ok 的 bump→COBUnit 之上，再给每个 unit 的 8 个 bump 随机、不重复地分配该 unit 的 8 根 track（路径由 (bump, track) 唯一确定）。`test_TOB_track.py` + CaDiCaL：把 unit 可行解钉死成 track 后仍 SAT；sequential `color=bump%8` 也 SAT。seed=0 的 1000 次随机 track 置换全部 UNSAT。说明「unit 层总有开关解」推不出「随机选中的那根 track 也有开关解」。
