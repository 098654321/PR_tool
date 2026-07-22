# PR_tool /source 工程指南（面向 AI Agent）

本文件是 `source/` 的导航入口：项目概况、目录结构、核心对象索引、构建与测试。算法与格式细节以源码为准。

PR_tool 面向 chiplet interposer 的布局布线：输入系统配置（topdie / topdieinst / external ports / connections），在 interposer 资源模型上完成放置与布线，输出硬件可用的 controlbits（寄存器配置比特）。

---

## 0. 4个工作原则

- 必须深入理解我给你的材料，在理解的基础上进行后续动作
- 完成修改之后，评估是否需要维护相应的 AGENTS.md 文件
- 如果改动超过 100 行，需要在项目根目录的 `.plan` 目录下生成改动记录文件，同时必须启动一个独立的子 agent 审核代码，判断改动结果是否完整、正确、符合需求
- 本文件不应超过 200 行；只更新概况 / 目录 / 核心索引 / 构建测试，算法细节不进本文件

---

## 1. 项目概况

端到端流水线（CLI：`source/app/cli/cli.cc`；v1.0.0 CLI 不支持增量 `-i/-c`）：

1. `parse::read_config` → `hardware::Interposer` + `circuit::BaseDie`
2. `algo::build_nets` → `Connection` 转为 `circuit::Net` / `SyncNet`
3. 可选 `algo::place`（默认 `SAPlaceStrategy`）
4. `algo::route_nets`（非增量 Maze；或增量 `IncreRouting`）
5. `parse::output_from_routing_results` → `{output}/regnamecontrolbit_4part/` 四文件

GUI：`source/app/gui/gui.cc` → `widget::Window`；P&R 在 `PRThread` 中异步执行。

---

## 2. 目录结构

```text
source/
  app/          # CLI / GUI 入口与参数解析
  circuit/      # 逻辑模型：BaseDie / Net / PathPackage / TopDie*
  hardware/     # 物理模型：Interposer / Track / COB / TOB / Bump
  algo/         # netbuilder / placer / router
  parse/        # reader（配置、controlbits）/ writer / comparator
  widget/       # Qt：schematic / layout / view2d / view3d
  global/       # debug、std 封装、utility
  serde/        # 序列化 / 反序列化宏
```

更细的树与配置格式见仓库根目录 `README.md`。

---

## 3. 核心文件 / 函数 / 数据结构

### app / widget

| 路径 | 职责 |
|------|------|
| `app/PR_tool.cc` | 参数解析与模式选择（CLI / GUI / 增量 / placement） |
| `app/cli/cli.cc` | 端到端主流程 |
| `app/gui/gui.cc` | GUI 入口 |
| `widget/window.*` | 主窗口与页面调度 |
| `widget/prthread.*` | 后台 P&R（`build_nets` + `route_nets`） |

### circuit

| 类型 / 文件 | 要点 |
|-------------|------|
| `BaseDie`（`basedie.hh`） | topdies / topdie_insts / external_ports / connections / nets |
| `Net`（`net/net.hh`） | `route` / `incremental_route` / `pathpackage` / `coords` |
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
| `router/route_nets.cc` → `route_nets` | Invoker 命令链 + `RouteEngine` |
| `router/common/maze/*` | 非增量 BFS maze；`MazeRerouter` 拉齐 SyncNet 长度 |
| `router/incremental/*` | 增量迭代；`HardwareRecorder` 代价模型 |

### parse

| 入口 | 要点 |
|------|------|
| `reader/module.cc` → `read_config` | Config → Interposer/BaseDie（落地逻辑在 `reader.cc`） |
| `reader/module.cc` → `read_controlbits` | 旧单文件 warm-start；四文件读回尚未适配 |
| `writer/module.cc` → `output_from_routing_results` | `connect_all` + Writer 写四文件（`hex address reg_name`） |
| `writer/writer.*` | 寄存器抓取；`-s` 省略等于默认 hex 的行 |

---

## 4. 构建与测试

构建系统：仓库根目录 `xmake.lua`（C++23；Linux 另有平台标准设置）。

```bash
# 主程序
xmake build PR_tool
xmake run PR_tool <config_folder> [OPTIONS]

# 测试
xmake build module_test      && xmake run module_test
xmake build regression_test  && xmake run regression_test

# 典型的回归测试流程

```

常用 CLI 选项与配置目录约定见根 `README.md`；测试目录与用例格式见 `test/AGENTS.md`。
