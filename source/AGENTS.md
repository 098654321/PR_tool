# PR_tool /source 工程指南（面向 AI Agent）

本文件是 PR_tool 工程的“唯一入口说明书”。目标是：AI 只阅读本文件，就能理解项目的核心对象模型、算法流程、关键不变量以及常见修改点，从而对工程进行细致修改而不引入逻辑错误。

PR_tool 是面向 chiplet interposer 的布局布线工具：输入一份系统配置（topdie/topdieinst/external ports/connections），在 interposer 的资源模型上完成放置与布线，并输出硬件可用的 controlbits（寄存器配置比特）。

---

## 0. 2个工作规则

- 必须深入理解我给你的材料，在理解的基础上进行后续动作
- 完成修改之后，评估是否需要维护相应的 AGENTS.md文件。
- 如果改动比超过100行，需要在项目根目录的.plan目录下生成改动记录文件，内容可以参考该目录下已有的改动记录。同时必须启动一个独立的子agent审核代码，判断改动结果是否完整、正确、符合需求
- 

## 1. 快速上手（构建 / 运行 / 测试）

构建系统使用 xmake，主要入口在仓库根目录的 `xmake.lua`。

常用目标：

- `PR_tool`：主程序（支持 CLI 与 GUI）
- `view2d` / `view3d`：加载配置并执行 P&R，然后用 2D/3D 视图展示结果
- `module_test` / `regression_test`：测试目标（用例与说明见仓库根目录 README）

典型命令（在仓库根目录）：

```bash
xmake build PR_tool
xmake run PR_tool <config_folder> [OPTIONS]

xmake build regression_test
xmake run regression_test
```

语言标准：`xmake.lua` 将 C++ 语言设置为 `c++23`（Linux 平台额外设置 `-std=c++2b`）。

---

## 2. 总体数据流（从配置到 controlbits）

CLI 模式的端到端流程在 `source/app/cli/cli.cc`：

1) `parse::read_config(config_folder, mode, try_all_modes)`  
   - 解析输入配置到 `parse::Config`  
   - 构建 `hardware::Interposer` 与 `circuit::BaseDie`

2) `algo::build_nets(basedie, interposer)`（`source/algo/netbuilder/`）  
   - 将 `BaseDie` 中的 `Connection`（按 mode/sync 分组）转为 `circuit::Net`/`SyncNet`

3) 可选布局（`-p/--placement`）：`algo::place(...)`（`source/algo/placer/`）  
   - 当前默认策略：模拟退火 `SAPlaceStrategy`

4) 布线（`algo::route_nets(...)`，`source/algo/router/`）  
   - 非增量：排序 → 资源准备 → Maze 路由  
   - 增量：设置复用类型 → 排序 → 资源准备 → 初始化 recorder（如存在旧路径）→ 迭代增量路由

5) 输出 controlbits（`parse::output_from_routing_results`，`source/parse/writer/`）  
   - `PathPackage::connect_all()` 将路径绑定到 TOB/COB 寄存器对象  
   - `parse::Writer` 从硬件对象抓取寄存器值并写入 `controlbits_<mode>.txt`

---

## 3. 核心概念与术语（必须先理解）

### 3.1 物理模型：hardware/

核心对象是 `hardware::Interposer`（`source/hardware/interposer.hh`）：

- interposer 由 TOB/COB/Track/Bump 构成
- `Track`（`source/hardware/track/track.hh`）是布线图的节点（路由在 track 图上搜索）
- `COBConnector`（`source/hardware/cob/cobconnector.hh`）描述 Track ↔ Track 的可编程连接（对应 COB 的开关/选择寄存器）
- `TOBConnector`（`source/hardware/tob/tobconnector.hh`）描述 Bump ↔ Track 的可编程连接（bump → hori → vert → track 的 mux 链 + 方向寄存器）

邻接关系：

- `Interposer::adjacent_idle_tracks(track)`：返回可走的邻接 Track 及其需要使用的 `COBConnector`
- Maze/增量路由都以此作为图遍历的“边”

坐标体系（常见混淆点）：

- `hardware::Coord(row,col)`：基础 2D 坐标（`source/hardware/coord.hh`）
- `hardware::TOBCoord(row,col)`：TOB 阵列坐标（不是 interposer 的绝对坐标）
- `hardware::TrackCoord(row,col,dir,index)`：Track 坐标（`source/hardware/track/trackcoord.hh`）

### 3.2 逻辑模型：circuit/

`circuit::BaseDie`（`source/circuit/basedie.hh`）是逻辑设计容器：

- `topdies`：芯粒类型（pin_map：pin 名 → bump index）
- `topdie_insts`：芯粒实例（绑定到某个 TOB）
- `external_ports`：外部端口（绑定到某个 TrackCoord）
- `connections`：原始连接（按 `mode -> sync -> vector<Connection>` 组织）
- `nets`：由 `NetBuilder` 构建出的线网集合（按 mode 存储）

Pin/Connection（`source/circuit/connection/`）：

- `Connection` 由 input_pin/output_pin 组成
- `Pin` 可能指向：
  - topdieinst 的某个 bump（通过 pin 名映射到 bump index）
  - external port（对应 interposer 某个 track）
  - 固定电源/地（以 `pose`/`nege` 后缀区分）

Net（`source/circuit/net/net.hh`）是布线抽象接口，核心职责：

- `route(interposer, strategy)`：非增量布线
- `incremental_route(interposer, incre_strategy, engine, shared)`：增量布线
- `check_accessable_cobunit()` / `accessable_cobunit()`：资源可达性分析（用于资源管理与匹配）
- `coords()`：用于布局评估的坐标集合（HPWL）
- `pathpackage()`：保存当前路由结果（见下一节）
- `modes()`：此 net 属于哪些 mode（多 mode => 可复用线网）
- `reuse_type()`：增量路由代价模型中的“复用/非复用”标记

### 3.3 路径表示：PathPackage（输出 controlbits 的基石）

`circuit::PathPackage`（`source/circuit/path/pathpackage.hh`）是路由结果的唯一真源：

- `_regular_path`：`[(Track*, optional<COBConnector>), ...]`，正向顺序保存轨道与 COB 连接器
- `_tob_to_track`：bump 作为源端时的 TOB 连接器（Bump → Track）
- `_track_to_tob`：bump 作为汇端时的 TOB 连接器（Track → Bump）
- `_length`：包含 TOB 端点与 Track 段的长度计数（不同 net 类型会有不同加法）

连接器状态机（非常关键）：

- 路由阶段通常只占用/预留连接器：  
  - MazeRouteStrategy / IncreRouting 会对路径中的 `COBConnector` 调用 `suspend()`  
  - 对端点 `TOBConnector` 调用 `give_out()` 并写入 package
- 输出阶段才真正连通：  
  - `PathPackage::connect_all()` 将所有 connector `connect()`，并在 Track/Bump 上建立连接关系
- 清理/回滚必须同步恢复寄存器状态：  
  - `PathPackage::reset_all()` / `clear_all()` / `occupy_all()` 负责批量状态维护

---

## 4. 输入解析与 controlbits（parse/）

### 4.1 读取配置（Config → Interposer/BaseDie）

入口：`parse::read_config`（`source/parse/reader/module.cc`）

- `load_config`：`source/parse/reader/config/config.cc`（serde/json 驱动）
- `Reader::build()`：`source/parse/reader/reader.cc`

Pin 名的解析规则（`Reader::parse_connection_pin`）：

- 以 `pose` 结尾：VDD 固定端口
- 以 `nege` 结尾：GND 固定端口
- 不包含 `.`：external port（在 `basedie->external_ports` 中查找）
- `topdieinst.pin` 形式：芯粒实例 pin（先找 topdieinst，再用 topdie 的 pin_map 映射到 bump index）

`ports_01`（`Reader::add_01ports_to_basedie`）：

- 配置中必须有 `pose`/`nege` 两组 TrackCoord
- Reader 会校验这些 TrackCoord 满足 `Interposer::is_external_port_coord`

### 4.2 读取已有 controlbits（用于跳过布线或增量 warm-start）

入口：`parse::read_controlbits`（`source/parse/reader/module.cc`）

- 若存在 `controlbits_<mode>.txt`：
  - `load_controlbits` 解析到 `parse::Controlbits`（`source/parse/reader/controlbits/controlbits.hh`）
  - `bits_to_paths(interposer, basedie, controlbits, mode)` 将 bit 反推回 `PathPackage`
- 增量模式且当前 mode 没有 bit：会按 net 规模从大到小尝试加载其它 mode 的 controlbits，并把这些路径作为旧路径

`read_controlbits` 返回值语义（见实现）：

- 第一个 bool：当前 mode 的 controlbits 是否存在并被加载
- 第二个 bool：是否加载到了任意旧路径（可能是当前 mode，也可能是其它 mode）

### 4.3 写出 controlbits（从 PathPackage 到寄存器文件）

入口：`parse::output_from_routing_results`（`source/parse/writer/module.cc`）

- `connect_registers(interposer, basedie, mode)`：
  - 遍历 `basedie->nets_to_vector()`
  - 对属于该 mode 的 net 调用 `net->pathpackage().connect_all()`
- `write_control_bits(interposer, output_path, mode)`：
  - `parse::Writer::fetch_and_write(...)`（`source/parse/writer/writer.hh`）
  - Writer 会从 interposer 的寄存器对象抓取 bit，并写成文本格式

---

## 5. Net 构建（algo/netbuilder）

入口：`algo::build_nets`（`source/algo/netbuilder/netbuilder.cc`）

输入：`basedie->connections()`（按 mode、sync 分组）  
输出：`basedie->nets()[mode]`（`std::Rc<Net>` 集合），并把 net 指针挂到相关 `TopDieInstance::_nets` 上

### 5.1 非同步连接（sync == -1）

按驱动端节点聚合为一个 net：

- Track → Bump：`TrackToBumpNet` / `TrackToBumpsNet`
- Bump → Bump：`BumpToBumpNet` / `BumpToBumpsNet`
- Bump → Track：`BumpToTrackNet` / `BumpToTracksNet`
- Track → Track：非法，直接报错

### 5.2 同步连接（sync != -1）

同一个 sync group 内的多条连接会被包装为一个 `SyncNet`（`source/circuit/net/types/syncnet.hh`）：

- 内部持有三个列表：BTB/BTT/TTB 三类二端网
- `SyncNet::route` 会先对三类网预布线，再通过 reroute 拉齐长度（保证同步）

### 5.3 固定电源/地（VDD/GND）

`build_fixed_nets` 会将所有被标记为 `pose/nege` 的 bump 收集起来，构造：

- `TracksToBumpsNet(ports_01.pose → bumps_with_pose)`
- `TracksToBumpsNet(ports_01.nege → bumps_with_nege)`

---

## 6. 放置（algo/placer）

入口：`algo::place`（`source/algo/placer/place.hh`）

当前策略：`SAPlaceStrategy`（`source/algo/placer/sa/saplacestrategy.cc`）

算法要点（按当前 `SAPlaceStrategy` 实现整理的伪代码）：

```text
Input:
  interposer, topdies
Hyper-parameters (base 默认值):
  T_init = 100.0                // 初始温度
  T_freeze = 0.5                // 停止温度
  alpha = 0.97                  // 冷却速率（几何降温）
  base_solve_num = 50           // 每层扰动次数基数
  base_max_no_improvement = 50  // 无改进停搜层数基数

Search budget（按 chip 数缩放，始终启用）:
  n_chips = topdies.size()
  effective_solve_num = base_solve_num * max(1, n_chips)
  effective_max_no_improvement = base_max_no_improvement * max(1, n_chips / 2)

Cost definitions:
  net_cost(net):
    coords = net.coords()
    if coords empty: return 0
    HPWL = (max_row - min_row) + (max_col - min_col)
    return HPWL

  total_net_cost(nets) = Σ net_cost(net)
  total_cost = total_net_cost(all_nets)

State:
  每个 TopDieInstance 绑定一个 TOB
  best_solution = 当前 placement 快照
  best_cost = total_cost
  T = T_init
  no_improvement_count = 0
  iteration = 0

budget = compute_search_budget(n_chips)

while T > T_freeze and no_improvement_count < budget.max_no_improvement:
  improved = false

  repeat budget.solve_num times:
    new_total_cost = total_cost

    // 扰动类型选择（温度相关）
    // p_swap = 0.7 - 0.2 * (T_init - T) / T_init
    // 因此温度从高到低时，swap 概率约从 0.7 线性降到 0.5
    if random(0,1) < p_swap:
      // --- swap 扰动 ---
      随机选两个不同 topdie inst: a, b
      若任一为空 -> continue
      tob_a = a.tob, tob_b = b.tob，若任一为空 -> continue
      若 !is_changable(a, tob_b) 或 !is_changable(b, tob_a) -> continue
      若 tob_a 或 tob_b 被第三方 chip 占用 -> continue

      changed_nets = nets(a) ∪ nets(b)
      new_total_cost -= Σ net_cost(net), net in changed_nets

      执行 a 与 b 的 TOB 交换

      new_total_cost += Σ net_cost(net), net in changed_nets

      delta = new_total_cost - total_cost
      if delta <= 0:
        接受该解，total_cost = new_total_cost，improved = true
      else if random(0,1) <= exp(-delta / T):
        按 Metropolis 准则接受，total_cost = new_total_cost
      else:
        拒绝，交换回退到原状态

    else:
      // --- move 扰动 ---
      随机选一个 topdie inst: a；随机选一个 idle TOB: tob_new
      若任一不存在 -> continue
      若 !is_changable(a, tob_new) -> continue
      若 tob_new 已被其他 chip 占用 -> continue

      tob_old = a.tob
      new_total_cost -= Σ net_cost(net), net in nets(a)

      执行 a 从 tob_old 移到 tob_new

      new_total_cost += Σ net_cost(net), net in nets(a)

      delta = new_total_cost - total_cost
      if delta <= 0:
        接受该解，total_cost = new_total_cost，improved = true
      else if random(0,1) <= exp(-delta / T):
        按 Metropolis 准则接受，total_cost = new_total_cost
      else:
        拒绝，a 回退到 tob_old

  // 温度层结束后更新全局最优
  if total_cost < best_cost:
    best_cost = total_cost
    best_solution = 当前 placement 快照
    no_improvement_count = 0
  else:
    no_improvement_count += 1

  T = T * alpha
  iteration += 1

循环结束后：restore_placement(best_solution)
日志：启动时打印 effective 超参；结束时打印 initial_HPWL 与 best_HPWL

约束函数 is_changable(inst, target_tob):
  若 inst 的任意 net 已经包含 target_tob 端口，则返回 false；否则 true
```

---

## 7. 布线（algo/router）

### 7.1 命令链框架（command_mode）

`route_nets`（`source/algo/router/route_nets.cc`）内部使用 `Invoker` 执行一串命令（`source/algo/router/command_mode/invoker.cc`）。

非增量（当前默认）：

1) `Sort`：按 `Net::port_number()` 归一化并提升 priority，然后按 priority 降序排序  
2) `Resources`：对每条 net 调用 `check_accessable_cobunit()`，并由 interposer 汇总管理 cobunit 资源  
3) `Route`：逐条调用 `Net::route(...)`（通常是 MazeRouteStrategy）

增量：

1) `Set_reuse_type`：`net->modes().size() > 1` 视为 reuse  
2) `Sort`：命令链中存在该步骤，但当前增量排序主要在 `Incre_route::sort_incre` 内部完成  
3) `Resources`  
4) `Init_recorder`：若加载了旧路径则初始化 cost/占用状态  
5) `Incre_route`：迭代增量布线

`RouteEngine`（`source/algo/router/routeengine.hh`）是命令链的上下文：

- 管理 nets 的视图（按 mode 过滤、按是否存在旧路径过滤）
- 跟踪当前 routing position（用于失败恢复/回退）
- 持有 `HardwareRecorder`、`RouteData` 等统计对象

### 7.2 非增量 MazeRouteStrategy（common/maze）

`MazeRouteStrategy`（`source/algo/router/common/maze/mazeroutestrategy.cc`）的 `maze_search` 是 BFS（队列 + prev map），并非 A*：

- 输入：begin_tracks（候选起点轨道集合）、end_tracks（目标轨道集合）、occupied_tracks（不可用轨道集合）
- 扩展：使用 `Interposer::adjacent_idle_tracks`
- 输出：`[(Track*, optional<COBConnector>), ...]`
- 对路径中的 `COBConnector` 调用 `suspend()` 以预留资源
- 对端点 TOB 的 `TOBConnector` 调用 `give_out()` 并写入 `PathPackage`

同步网（`SyncNet`）：

- 先分别对三类子网预布线，得到当前 max_length
- 再调用 `MazeRerouter::bus_reroute` 对短路径做 reroute 拉齐长度

`MazeRerouter`（`source/algo/router/common/maze/mazererouter.cc`）使用优先队列扩展（实现中标注为 A*），用于“在不短于 max_length 的前提下找更长或等长路径”。

### 7.3 资源分配（common/allocate）

项目实现了基于 Hopcroft–Karp 的 bump→track 最大二分匹配（`source/algo/router/common/allocate/hopcroft_karp.cc`），但默认命令链中 Allocate 当前被注释掉。

基本思想：

- 对每个 TOB 独立构造二分图：左侧 bump index，右侧 track index
- 边：由 `Net::accessable_cobunit()` 推导“该 bump 可到达的 track indexes”
- 通过 HK 求完美匹配后：
  - 在 bump 上记录 allocated_track
  - 为对应 net 的 `PathPackage` 写入 TOBConnector（bump ↔ track 的 mux 链）

### 7.4 增量路由（incremental）

增量路由由两部分组成：

1) `Incre_route` 命令：负责多 cycle 迭代、收敛判断、失败回退  
   - 入口：`source/algo/router/command_mode/command/incremental_route.cc`  
   - 每个 cycle：
     - 把当前包保存到 history（`net->set_history_pathpackage()`）
     - 清空当前包（`net->reset_pathpackage()`）
     - 逐 net 调 `Net::incremental_route(...)`
     - `HardwareRecorder` 更新 current/history 记录并重新计算 cost
   - 收敛条件：`fabs(cost_nonreuse - last_cost_nonreuse) < bound`
   - 若某 cycle 失败：返回上一 cycle 的 history 路径作为最终结果

2) `IncreRouting` 策略：负责单条 net 的增量搜索  
   - 入口：`source/algo/router/incremental/maze/routing.cc`  
   - begin/end 搜索点会合并“当前可用资源 + 与该端点共享节点的已布线路径”（`existing_path_vec`）  
   - `maze_search` 使用 min-heap，以 `HardwareRecorder::expand_cost(...)` 作为扩展代价  
   - 注意：实现中只记录首次到达的 prev，不做传统 Dijkstra 的松弛更新；修改代价模型时需验证这一点是否满足预期

`HardwareRecorder` 与代价模型（`source/algo/router/incremental/recorders/`）：

- `TypeRecorder` 记录某个资源单元的 reuse/non-reuse 历史次数，并据此动态更新两类 cost
- `HardwareRecorder` 以 Track/TOB/COB 为粒度管理 recorder，并提供：
  - `track_cost(...)` / `bump_to_track_cost(...)` / `expand_cost(...)`
  - update_current/update_history/clear_history 等迭代维护接口

`bound_bits`（`source/algo/router/incremental/bound_bits/`）：

- 将 Track/COB/TOB 的资源使用按组聚合统计（`BitsGroup<N>`）
- `GlobalBoundBits::get_rate()` 用于输出 “monopolized_by_reuse / has_nonreuse” 等指标

---

## 8. 目录与关键文件索引（修改时优先查这些）

### 8.1 GUI 可视化主链路（widget）

GUI 入口在 `source/app/gui/gui.cc`，创建 `QApplication` 后启动 `widget::Window`。  
`Window`（`source/widget/window.*`）是可视化总调度器，内部共享同一组 `Interposer/BaseDie` 指针给所有页面（schematic/layout/view2d/view3d）。

核心交互流程（按 `Window` 当前实现）：

1) `loadConfig()`：读取配置到 `_interposer/_basedie`，并触发  
   - `SchematicWidget::reload()`  
   - `LayoutWidget::reload()`

2) `executePlaceRoute()`：在 `PRThread` 中异步执行 P&R（避免阻塞 UI）  
   - `algo::build_nets(...)`  
   - `algo::route_nets(..., MazeRouteStrategy{}, HK{}, mode=0, ...)`  
   完成后触发：
   - `View2DWidget::reload()`（2D 结果重绘）
   - `View3DWidget::displayRoutingResult()`（3D 结果渲染）

3) `generateControlBitAs()`：通过 `parse::write_control_bits(...)` 导出 `.ctb`

状态约束（改 GUI 行为时必须保持）：

- `_finishPR == true` 后禁止二次 P&R，且禁用 schematic/layout 编辑（`disableEdit()`）
- 未完成 P&R 时禁用导出 controlbits 按钮
- 页面切换由左侧 toolbar + `QStackedWidget` 统一管理，不应让各页面自行持有流程状态

### app/

- `source/app/PR_tool.cc`：命令行参数解析与模式选择（GUI/CLI/增量/compare/placement）
- `source/app/cli/cli.cc`：端到端主流程（read_config → build_nets → place → route → write）
- `source/app/gui/gui.cc`：GUI 入口（Qt window）

### widget/

- `source/widget/window.*`：GUI 主窗口与系统状态机（菜单/工具栏/页面容器/P&R 触发）
- `source/widget/prthread.h`：后台 P&R 线程（build_nets + route_nets）
- `source/widget/schematic/*`：原理图编辑视图（器件库 + 场景 + 信息面板）
- `source/widget/layout/*`：布局编辑视图（拓扑摆放与信息统计）
- `source/widget/view2d/*`：2D 结果视图（QGraphicsScene，按 COB 开关状态绘制连线）
- `source/widget/view3d/*`：3D 结果视图（OpenGL instancing，支持旋转/缩放/拾取）

### circuit/

- `source/circuit/basedie.hh`：逻辑设计容器（topdies/topdieinst/external_ports/connections/nets）
- `source/circuit/net/net.hh`：Net 抽象接口（route / incremental_route / resource analysis / pathpackage）
- `source/circuit/net/types/*.hh`：具体 net 类型（BB/BT/TB/多端/Sync）
- `source/circuit/path/pathpackage.hh`：路径与 connector 状态机
- `source/circuit/topdieinst/topdieinst.hh`：placement 的 move/swap

### hardware/

- `source/hardware/interposer.hh`：硬件图与资源查询入口（available_tracks / adjacent_idle_tracks / reset_regs）
- `source/hardware/track/track.hh`：Track 连通关系（prev/next/connected_bump）
- `source/hardware/cob/cobconnector.hh`：COB 连接器（connect/suspend/disconnect）
- `source/hardware/tob/tobconnector.hh`：TOB 连接器（give_out/stay_inside/connect）

### algo/

- `source/algo/netbuilder/netbuilder.cc`：Connection → Net 的分类与构建
- `source/algo/placer/sa/saplacestrategy.cc`：模拟退火放置
- `source/algo/router/route_nets.cc`：布线总入口（Invoker + RouteEngine + analyze_results）
- `source/algo/router/routeengine.*`：net 视图与旧路径存在性策略
- `source/algo/router/common/maze/*`：非增量 BFS maze + reroute
- `source/algo/router/incremental/*`：增量迭代路由与代价模型

### parse/

- `source/parse/reader/reader.cc`：配置到 BaseDie/Interposer 的落地逻辑（Pin 名解析规则在这里）
- `source/parse/reader/controlbits/controlbits.*`：controlbits 文本格式解析与反推 PathPackage
- `source/parse/writer/module.cc`：PathPackage::connect_all + Writer 输出
- `source/parse/writer/writer.*`：寄存器抓取与文件输出格式

---

## 9. 常见修改指南（避免踩坑）

### 9.1 修改/扩展输入配置格式

需要同时更新：

- `parse/reader/config/*`：Config 结构与 load_config 的解析逻辑
- `serde/de.hh` + 对应类型的 `DESERIALIZE_STRUCT/ENUM` 宏展开
- `Reader::build()`：把新增字段落到 `BaseDie/Interposer`

### 9.2 新增一种 Net 类型

必须同步更新这些层：

1) `circuit/net/types/`：新增 Net 子类并实现 `Net` 虚函数  
2) `algo/router/common/routestrategy.hh`：为新 net 增加虚接口  
3) `MazeRouteStrategy`：实现对应的 route_*  
4) `IncreRouting`：实现对应的 route_*（如果增量模式需要支持）  
5) `algo/netbuilder`：把 Connection 分类到新的 net 类型  
6) `PathPackage`：确认新 net 的端点/长度计数与 connect/reset 逻辑一致

### 9.3 修改路由算法/代价模型

高风险点：

- 连接器状态机必须一致：route 阶段 suspend/give_out，输出阶段 connect_all
- 增量路由依赖 `HardwareRecorder` 的 cost 更新；修改 cost 需验证收敛条件与失败回退逻辑
- `RouteEngine::nets()` 在存在旧路径时会过滤 shared nets；改动筛选规则会影响增量复用策略

### 9.4 修改 controlbits 输出格式

需要同时改：

- `parse/writer/writer.*`：写出格式
- `parse/reader/controlbits/*`：解析格式 + bits_to_paths 反推逻辑
- 回归测试：确保 `read_controlbits → bits_to_paths → connect_all → write_control_bits` 能闭环

---

## 10. 代码风格与约定

### 10.1 命名约定

- 命名空间：snake_case（`PR_tool::hardware`, `PR_tool::algo`），所有工程代码都在 `namespace PR_tool` 下
- 类/结构体：PascalCase（`RouteEngine`, `TopDieInstance`）
- 函数：snake_case（`check_accessable_cobunit`）
- 成员变量：`_snake_case`（带前导 `_`）

### 10.2 日志与异常

- 统一使用 `PR_tool::debug`（`source/global/debug/`）
- 典型用法：`debug::info_fmt("...", ...)`、`debug::fatal(...)`、`THROW_UP_WITH("context")`

### 10.3 标准库封装（强约束）

工程核心代码倾向使用 `source/global/std/` 中的封装类型：

- `std::Vector`, `std::String`, `std::HashMap`, `std::HashSet`, `std::Option`, `std::Rc`, `std::Box` 等
- 头文件中尽量避免直接暴露原生 `std::vector/std::string/...`

### 10.4 序列化（serde/）

- serde 宏位于 `source/serde/ser.hh` 与 `source/serde/de.hh`
- 常用：`DESERIALIZE_STRUCT`, `DESERIALIZE_ENUM`, `FORMAT_STRUCT`, `FORMAT_ENUM`
