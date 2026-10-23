# PR_tool /source 工程指南（面向 AI Agent）

本文件是 `source/` 的导航入口：项目概况、目录结构、核心对象索引、构建与测试。算法与格式细节以源码为准。

PR_tool 面向 chiplet interposer 的布局布线：输入系统配置（topdie / topdieinst / external ports / connections），在 interposer 资源模型上完成放置与布线，输出硬件可用的 controlbits（寄存器配置比特）。

---

## 0. 工作原则

- 修改前核对方法文档和硬件映射；只改当前流程直接需要的代码。
- 关键约束、方法、步骤应有小型单元测试，测试可以生成在`项目根目录/test/module_test/test_unit`；测试断言必须在 Release 构建中生效。test/config里面真实 case 用于检查规模、耗时与最终物理合法性。
- 单文件保持紧凑，功能清晰，避免无关重构；单个文件不要超过1000行
- 实现算法的过程中，保证正确性的前提下关注求解速度；数据结构的高效性；可以尝试多使用C++新版本的功能，但是不要超过C++20
- 代码文件的单次修改超过 200 行时，在实现完成后进行分离审查：如果实现过程是启动子agent实现的，那么可以由原主agent自己审查；如果实现是由主agent自己实现的，那么需要启动一个子agent审查。如果单次修改不超过200行，可以由实现的agent自己审查。如果审查之后有问题需要修正，但是修正之后不需要再进行单独的审查工作。
- 在写代码的时候，关键的算法中间信息与结果信息（例如求解时间、迭代论述、求解出来的布线路径结果）等需要加入日志（debug.log），即使不含-v也要输出；一些次级信息，用于辅助深入debug的，例如算法参数、bbox范围等，在-v等情况下也要输出
- 修改代码后查看是否需要维护本AGENTS.md文件，本文件保持当前的章节结构，长度不要超过200行
- 当我问你“是否理解我的意思”或者类似的话的时候，你必须先在聊天窗口回复我，而不是直接开始动手做

---

## 1. 项目概况

端到端流水线（CLI：`source/app/cli/cli.cc`；v1.0.0 CLI 不支持增量 `-i/-c`）：

1. `parse::read_config` → `Interposer` + `BaseDie` + `RegisterMapConfig`（`config.json` 字段 **`reigster_adder`** → 文件 **`register_adder.json`**）
2. `algo::build_nets` → `Connection` 转为 `circuit::Net` / `SyncNet`
3. 可选 `algo::place`（默认 `SAPlaceStrategy`，5 参：init/freeze/solve_num/cooling/max_no_improvement）
4. `algo::route_nets`（非增量；`--router maze` 或 `--router sat`；Maze 首次失败交给 RRR；最终无合法解时跳过 REG 写出）
5. `parse::output_from_routing_results` → `{output}/regnamecontrolbit_4part/` 四文件

GUI：`source/app/gui/gui.cc` → `widget::Window`；Place 在 `PlaceThread`、Route 在 `PRThread`（均 `widget/prthread.*`）中异步执行；可导出 controlbits 到输出根目录。阶段机、四视图与约束见 `source/widget/AGENTS.md`。

SAT/ILP 布线实现位于 `algo/router/sat_ilp/` + `algo/router/backend/`（`SatRouterBackend`）；`algorithm/test_ILP/` 为过渡壳与 fixture（见该目录 `AGENTS.md`）。

**`test/config` 与 `COB_ARRAY_WIDTH`**：跑 `test/config/caseN` 前须使 `Interposer::COB_ARRAY_WIDTH`（`hardware/interposer.hh`）与 case 的 `description.txt` 一致；规则见 `test/AGENTS.md`。

---

## 2. 目录结构

```text
source/
  app/          # CLI / GUI 入口与参数解析；PR_tool_cli 无 Qt
  circuit/      # 逻辑模型：BaseDie / Net / PathPackage / TopDie*
  hardware/     # 物理模型：Interposer / Track / COB / TOB / Bump
  algo/         # netbuilder / placer / router
  parse/        # reader（配置、controlbits）/ writer / comparator
  widget/       # Qt GUI（入口 `widget/AGENTS.md`）：schematic / layout / view2d / view3d
  global/       # debug、std 封装、utility
  serde/        # 序列化 / 反序列化宏
```

更细的树与配置格式见仓库根目录 `README.md`。默认 `Interposer::COB_ARRAY_WIDTH = 12`；`test/config` 用例可能要求 12 或 13（见 `test/AGENTS.md`）。

---

## 3. 核心文件 / 函数 / 数据结构

### app / widget

| 路径 | 职责 |
|------|------|
| `app/PR_tool.cc` | 参数解析与模式选择（CLI / GUI / placement） |
| `app/cli/cli.cc` | 端到端主流程；写出在 route 之后由 CLI 触发 |
| `app/gui/gui.cc` | GUI 入口 |
| `widget/AGENTS.md` | GUI 约束、阶段机、规格索引 |
| `widget/window.*` | 主窗口、四视图、`_placed` / `_finishPR` |
| `widget/prthread.*` | `PlaceThread` / `PRThread`（后台 Place 与 Route） |
| `widget/frame/controlbitexportdialog.*` | GUI controlbits 导出 |

### circuit

| 类型 / 文件 | 要点 |
|-------------|------|
| `BaseDie`（`basedie.hh`） | topdies / topdie_insts / external_ports / connections / nets |
| `Net`（`net/net.hh`） | `route` / `pathpackage` / `coords` / 稳定 `uid` |
| `net/types/*.hh` | BB/BT/TB/多端/`SyncNet` 等具体线网 |
| `PathPackage`（`path/pathpackage.hh`） | 路由结果真源；route 阶段 `suspend`/`give_out`，写出时 `connect_all` |
| `TopDieInstance` | placement 的 move/swap |

### hardware

| 类型 / 文件 | 要点 |
|-------------|------|
| `Interposer`（`interposer.hh`） | 资源图入口；`adjacent_idle_tracks` / `reset_regs` |
| `Track` | 布线图节点 |
| `COBConnector` | Track↔Track 可编程连接（`connect` / `suspend`） |
| `TOBConnector` | Bump↔Track 可编程连接（`give_out` / `connect`） |
| `Coord` / `TOBCoord` / `TrackCoord` | 三类坐标（勿混用） |

### algo

| 入口 | 要点 |
|------|------|
| `netbuilder/netbuilder.cc` → `build_nets` | Connection → Net；sync 组 → `SyncNet`；pose/nege → 固定电源地网 |
| `placer/place.hh` → `place` | 默认 `SAPlaceStrategy`（HPWL + 模拟退火） |
| `router/route_nets.cc` → `route_nets` | Invoker 命令链 + `RouteEngine`；`--router maze\|sat` 选后端 |
| `router/backend/maze_backend.cc` | 初始 Sort + Resources + Maze，再接 `rrr/`；RRR 恢复完整最优合法解后写出 |
| `router/rrr/` | FPIA-RRR/AIR 移植；生产路径导入/提交、owner 资源与共享驱动、同步等长、物理方向验证 |
| `router/common/routing_budget.hh` | 线程局部单调时钟预算，搜索内部取消；不限时按三次合法解收敛（首解计 1） |
| `router/backend/sat_backend.cc` | `--router sat`；调用 `sat_ilp/` 统一图 SAT（可选 v15 ILP） |
| `router/sat_ilp/` | v14 SAT 编码、反馈扩边、commit 到 `PathPackage`（`xmake f --cadical=y`） |
| `router/incremental/*` | 增量代码仍在树中；v1.0.0 CLI 已拒绝 `-i/-c` |

### parse

| 入口 | 要点 |
|------|------|
| `reader/module.cc` → `read_config` | 返回 Interposer / BaseDie / RegisterMapConfig |
| `reader/module.cc` → `read_controlbits` | 旧单文件；四文件读回尚未适配 |
| `writer/module.cc` → `output_from_routing_results` | `connect_all` + Writer 写四文件（`hex address reg_name`） |
| `writer/writer.*` / `register_defaults.hh` | 寄存器抓取；`-s` 省略默认行；缺少活动寄存器地址时拒绝写出 |

---

## 4. 构建与测试

构建系统：仓库根目录 `xmake.lua`（C++23）。SAT 后端默认开启：`xmake f --sat_router=y|n`（默认 `y`）；关闭时 `PR_TOOL_HAS_SAT_ROUTER=0`，`--router sat` 不可用。

```bash
xmake f --sat_router=y --cadical=y   # 默认；SAT 需 CaDiCal，可选 Gurobi（v15 ILP）
xmake build PR_tool
xmake run PR_tool <config_folder> [OPTIONS]

xmake build PR_tool_cli
xmake run PR_tool_cli <config_folder> --router sat --scope-pad 1   # SAT 首轮 scope 扩展

# module_test 只编译 test/module_test/test_unit/**.cc
xmake build module_test
cd output && ./module_test all

xmake build regression_test
./output/regression_test "[flow]"

xmake build gui_test
xmake run gui_test                 # offscreen；见 widget/AGENTS.md 与 test/AGENTS.md
```

Maze：`--time-limit <seconds>` 为可选正有限秒数，覆盖初始路由和 RRR；到期恢复最短完整合法解，无解返回失败。省略则首次合法计 1、改善重置为 1、合法序列计到 3 停止；非法轮不改变计数。SAT/ILP 同名参数单位也改为秒，范围仍为 ILP 阶段。

常用 CLI：`--router maze|sat`（默认 maze）、`--scope-pad N` / `--delay-pad N`（仅 SAT；勿与写出稀疏 `-s` 混淆）。配置目录约定见根 `README.md`；测试与 `COB_ARRAY_WIDTH` 见 `test/AGENTS.md`；`algorithm/test_ILP/` 见该目录 `AGENTS.md`。
