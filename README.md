# PR_tool

面向 chiplet interposer 的布局布线工具。输入系统配置（topdie / topdieinst / external ports / connections），在 interposer 资源模型上完成放置与布线，输出硬件可用的 controlbits（寄存器配置比特）。

更详细的工程说明见：

- [`source/AGENTS.md`](./source/AGENTS.md)：源码概况、目录结构、核心索引与构建/测试入口
- [`test/AGENTS.md`](./test/AGENTS.md)：测试目录结构与用例格式

---

## 快速开始

### 依赖

- [xmake](https://xmake.io/mirror/zh-cn/guide/installation.html)
- 支持 **C++23** 的编译器（`xmake.lua` 中 `set_languages("c++23")`）
- 构建 `PR_tool` / `view2d` / `view3d` 需要 **Qt**（含 OpenGL）
- 构建 `regression_test` 需要 **Catch2**（macOS / Windows 由 xmake 自动拉取；Linux 需通过 conda 等方式安装并设置 `CONDA_PREFIX`）

### 构建与运行

```bash
# 主程序
xmake build PR_tool
xmake run PR_tool <config_folder> [OPTIONS]

# 默认 target 为 regression_test
xmake build regression_test
xmake run regression_test
```

产物输出到 `./output/`。

---

## 源码结构（`source/`）

`source/` 是工程核心，按“物理模型 → 逻辑模型 → 算法 → 解析/输出 → 应用/GUI”分层组织。

### 端到端数据流

```
config JSON (+ optional reigster_adder → register_map)
  → parse::read_config          # Interposer + BaseDie + RegisterMapConfig
  → algo::build_nets            # Connection → Net / SyncNet
  → algo::place (可选)          # 模拟退火布局
  → algo::route_nets            # Maze / 增量布线
  → parse::output_from_routing_results
  → {output}/regnamecontrolbit_4part/
       botleft_REG0.txt … topright_REG3.txt   # hex address reg_name
```

CLI 主流程在 [`source/app/cli/cli.cc`](./source/app/cli/cli.cc)；入口参数解析在 [`source/app/PR_tool.cc`](./source/app/PR_tool.cc)。

### 目录说明

| 目录 | 职责 |
|------|------|
| [`app/`](./source/app/) | 程序入口：CLI / GUI 模式切换 |
| [`hardware/`](./source/hardware/) | 物理模型：`Interposer`、`Track`、`COBConnector`、`TOBConnector`、`Bump` |
| [`circuit/`](./source/circuit/) | 逻辑模型：`BaseDie`、`TopDie`、`Connection`、`Net` 及各类线网子类型 |
| [`circuit/path/`](./source/circuit/path/) | `PathPackage`：路由结果与连接器状态机（输出 controlbits 的基石） |
| [`algo/netbuilder/`](./source/algo/netbuilder/) | 将 `Connection` 按 mode/sync 分类并构建 `Net` |
| [`algo/placer/`](./source/algo/placer/) | 布局；当前默认策略为模拟退火 `SAPlaceStrategy` |
| [`algo/router/`](./source/algo/router/) | 布线总入口、命令链框架、Maze 路由与增量路由 |
| [`parse/`](./source/parse/) | 配置读取、controlbits 解析/写出、结果比较 |
| [`serde/`](./source/serde/) | JSON 序列化/反序列化宏与解析器 |
| [`global/`](./source/global/) | 标准库封装、日志、异常、工具函数 |
| [`widget/`](./source/widget/) | Qt GUI：原理图、布局编辑、2D/3D 布线结果可视化 |

### 布线框架要点

`algo::route_nets`（[`source/algo/router/route_nets.cc`](./source/algo/router/route_nets.cc)）通过命令链（`command_mode/`）组织流程：

**非增量（默认）**：`Sort` → `Resources` → `Route`（`MazeRouteStrategy`，BFS 在 track 图上搜索）

**增量（`-i/--incremental`）**：`Set_reuse_type` → `Sort` → `Resources` → `Init_recorder` → `Incre_route`（多 cycle 迭代，带代价模型与失败回退）

### 输入配置

典型 config 目录包含：

| 文件 | 含义 |
|------|------|
| `config.json` | 主配置，指向其余 JSON |
| `interposer.json` | interposer 物理定义 |
| `topdies.json` | 芯粒类型与 pin_map |
| `topdie_insts.json` | 芯粒实例及 TOB 坐标 |
| `external_ports.json` | 外部 I/O 端口 |
| `connections.json` | 线网连接（按 mode / sync 分组） |
| `01_ports.json` | VDD/GND（pose/nege）端口 |
| `reigster_adder.json`（或 `register_adder.json`） | 寄存器名 → 地址 map（写出四文件用；由 `config.json` 指向） |
| `controlbits_<mode>.txt` | （可选、legacy）旧单文件布线结果；读回尚未适配四文件输出 |

Pin 名解析规则、连接器状态机、增量代价模型等细节见 [`source/AGENTS.md`](./source/AGENTS.md)。

---

## 测试（`test/`）

```
test/
├── config/            # 回归测试用例（case1 … case22）
├── module_test/       # 模块级单元测试
├── regression_test/   # 端到端回归（Catch2）
└── transform_format/  # 配置格式转换工具（txt ↔ json）
```

### module_test

对各模块进行隔离测试，入口为 [`test/module_test/test.cc`](./test/module_test/test.cc)。

```bash
xmake build module_test
cd output
./module_test <module>    # 或 xmake run module_test <module>
```

| 模块名 | 测试内容 |
|--------|----------|
| `cob` | COB 硬件对象 |
| `tob` | TOB 硬件对象 |
| `interposer` | Interposer 资源查询 |
| `router` | 布线逻辑 |
| `placer` | 布局策略 |
| `config` | 配置解析 |
| `comparator` | controlbits 比较 |
| `path_length` | 路径长度计算 |
| `debug` | 日志系统 |
| `all` | 运行上表全部快速测试 |
| `placer_iteratively [config] [iterations]` | 慢速稳定性测试（放置+布线；默认 case1、100 次；不在 `all` 中） |
| `router_iteratively [config] [iterations]` | 慢速稳定性测试（仅布线；默认 case1、100 次；不在 `all` 中） |

`module_test/test_function/` 与 `module_test/test_writer/` 下还有带独立数据集的专项测试（线长、writer、bbox 等），由对应 `*.cc` 编译进 `module_test` target。

### regression_test

使用 Catch2，在 [`test/regression_test/`](./test/regression_test/) 中按场景组织：

| 文件 | 标签 | 内容 |
|------|------|------|
| `test.cc` | `[basic]` `[CPU_MEM_AI]` `[CPU_MEM]` `[AI_core]` | 读配置 → build_nets → route，校验总线长 ≤ `golden.txt` |
| `incremental_test.cc` | `[incremental]` | 增量布线统计与循环测试 |
| `flow_test.cc` | `[flow]` | 编排：COB=12 → rebuild → case5 布局/布线×10 → writer test1–5 |

```bash
xmake build regression_test
cd output
./regression_test              # 全部场景
./regression_test "[basic]"    # 仅运行指定标签
```

Linux 上需确保 `CONDA_PREFIX` 指向已安装 Catch2 的环境。

### 回归用例（`test/config/`）

目前共 **22** 个 case 目录（无 case6）。每个 case 通常含输入 JSON、`golden.txt`（期望总线长上界）及 `description.txt`。

#### case 1–5：基础功能（Muyan 小规模）

| case | 说明 |
|:----:|------|
| [case1](./test/config/case1) | 仅同步线（bus）布线；`placer_iteratively` / `router_iteratively` 默认用例 |
| [case2](./test/config/case2) | 同步线 + 额外非同步线，测试 bump 复用 |
| [case3](./test/config/case3) | 仅非同步线布线 |
| [case4](./test/config/case4) | 含 VDD/GND、更多线网（回归中暂未启用） |
| [case5](./test/config/case5) | 更多连接 |

#### case 7–9：CPU–AI–MEM 芯粒系统

| case | 规模 |
|:----:|------|
| case 7 | 最少 bus 数量 |
| case 8 | 中等 bus 数量 |
| case 9 | 最多 bus 数量 |

#### case 10–12：CPU 芯粒系统

| case | 规模 |
|:----:|------|
| case 10 | 最少 bus |
| case 11 | 中等 bus |
| case 12 | 最多 bus |

#### case 13–16：AI core 芯粒系统

| case | 规模 |
|:----:|------|
| case 13 | 最少 bus |
| case 14–16 | 逐步增大线网规模 |

#### case 17–22：专项测试

| case | 说明 |
|:----:|------|
| case 17 | controlbits 反推路径（`bit_to_path`） |
| case 18–19 | 增量布线相关配置 |
| case 20 | 增量回归（`incremental_test` mode 1/2） |
| case 21–22 | 扩展测试配置 |

部分大规模 case（如 8、9、14–16）在 `test.cc` 中标注为已知失败，仍保留作 benchmark。

---

## 命令行参数

```bash
PR_tool <input folder path> [OPTIONS]
```

| 选项 | 说明 |
|------|------|
| `-o, --output <PATH>` | 输出根目录；其下生成 `regnamecontrolbit_4part/` |
| `-g, --gui` | GUI 模式 |
| `-p, --placement` | 启用布局（模拟退火） |
| `-i, --incremental [MODE]` | 增量布线；可跟正整数 mode，省略则尝试所有 mode |
| `-c, --compare <MODE>` | 与指定 mode 的 controlbits 对比（需配合 `-i`；仍假设旧单文件，待适配） |
| `-s, --simplify-controlbits-file` | 写出四文件时省略等于默认 hex 的行（稀疏输出） |
| `-v, --verbose` | 输出 Debug 日志 |
| `-h, --help` | 帮助 |
| `-V, --version` | 版本信息 |

示例：

```bash
xmake run PR_tool test/config/case1 -v
xmake run PR_tool test/config/case20 -i 2 -o ./output
xmake run PR_tool -g
```

---

## 工具程序

`tools/` 与 `test/transform_format/` 提供辅助程序。正式产品输出由 Writer 写四文件；`split_regs.py` 仅 legacy 旧单文件离线拆分（详见 [`tools/AGENTS.md`](./tools/AGENTS.md)）。

| target | 说明 |
|--------|------|
| `cobmap` | 计算 COB 端口映射 |
| `view2d` | 加载配置、执行 P&R、2D 可视化 |
| `view3d` | 加载配置、执行 P&R、3D 可视化 |
| `parse_controlbits` | 解析旧单文件 controlbits（读回未跟四文件） |
| `txt2json` | 旧版 txt 配置转 JSON |
| `json2txt` | JSON 配置转旧版 txt 连接格式 |

```bash
xmake build view2d
xmake run view2d <config_folder>
```

---

## 项目结构（其他目录）

| 目录 | 说明 |
|------|------|
| [`algorithm/`](./algorithm/) | 算法实验与原型（增量布线、ILP 等），非主程序依赖 |
| [`document/`](./document/) | 项目文档 |
| [`resource/`](./resource/) | Qt GUI 资源 |
| [`tools/`](./tools/) | 独立工具源码 |
| [`output/`](./output/) | 构建产物与运行日志（`debug.log` 等） |

---

## 分支说明

| 分支 | 说明 |
|------|------|
| `master` | 主开发线；algo 部分采用命令模式框架 |
| `version_before_commands` | 旧版完整布线流程，不支持增量布线 |
| `dev.incre_no_sharing` | 增量布线：失败时不共享资源 |
| `dev.algo` / `dev.linux` | 平台与算法开发分支 |
| `fix.controlbits` | controlbits 相关修复 |

增量布线的高层流程（多 cycle 迭代、按 reuse frequency 排序、失败回退）见 `algorithm/incremental_routing/` 下的实验记录；生产实现位于 `source/algo/router/incremental/`。
