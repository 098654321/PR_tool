# PR_tool

面向 chiplet interposer 的布局布线工具。输入系统配置（topdie / topdieinst / external ports / connections），在 interposer 资源模型上完成放置与布线，输出硬件可用的 controlbits（寄存器配置比特）。

更详细的工程说明见：

- [`source/AGENTS.md`](./source/AGENTS.md)：源码概况、目录结构、核心索引与构建/测试入口
- [`test/AGENTS.md`](./test/AGENTS.md)：测试目录结构与用例格式
- [`tools/AGENTS.md`](./tools/AGENTS.md)：外围辅助工具
- [`algorithm/test_ILP/AGENTS.md`](./algorithm/test_ILP/AGENTS.md)：实验性 SAT/ILP 布线（不替代正式 router）

---

## 快速开始

### 依赖

- [xmake](https://xmake.io/mirror/zh-cn/guide/installation.html)
- 支持 **C++23** 的编译器（`xmake.lua` 中 `set_languages("c++23")`）
- 构建 `PR_tool` / `view2d` / `view3d` 需要 **Qt**（含 OpenGL）
- `PR_tool_cli` 是无 Qt 的 headless CLI target
- 构建 `regression_test` 需要 **Catch2**（macOS / Windows 由 xmake 自动拉取；Linux 需通过 conda 等方式安装并设置 `CONDA_PREFIX`）

### 构建与运行

```bash
# Qt GUI 程序
xmake build PR_tool
xmake run PR_tool <config_folder> [OPTIONS]

# 无 Qt 的 CLI 程序
xmake build PR_tool_cli
xmake run PR_tool_cli <config_folder> [OPTIONS]

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
config JSON (+ optional reigster_adder → register_adder.json → register_map)
  → parse::read_config          # Interposer + BaseDie + RegisterMapConfig
  → algo::build_nets            # Connection → Net / SyncNet
  → algo::place (可选)          # 模拟退火布局
  → algo::route_nets            # Maze（默认）或 SAT 布线（`--router sat`）
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

**非增量（默认，v1.0.0 CLI）**：`Sort` → `Resources` → `Route`。后端由 `--router` 选择：

- `maze`（默认）：`MazeRouteStrategy`，BFS 在 track 图上搜索
- `sat`：统一图 SAT（`source/algo/router/sat_ilp/`），可选 v15 Gurobi 线长优化；构建需 `xmake f --sat_router=y`（默认）与 `--cadical=y`

**增量**：源码仍在 `source/algo/router/incremental/`，但 v1.0.0 CLI 已拒绝 `-i/--incremental` 与 `-c/--compare`。

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
| `register_adder.json` | 寄存器名 → 地址 map（写出四文件用）；由 `config.json` 的 **`reigster_adder`** 键指向 |
| `controlbits_<mode>.txt` | （可选、legacy）旧单文件布线结果；读回尚未适配四文件输出 |

Pin 名解析规则、连接器状态机、增量代价模型等细节见 [`source/AGENTS.md`](./source/AGENTS.md)。

---

## 测试（`test/`）

```
test/
├── config/            # 回归用例 case1 … case22（含 case6）
├── config_3dblox/     # 3DBlox / DEF / LEF 风格测例
├── module_test/
│   ├── test_unit/     # module_test 全部 C++ 源（xmake: test_unit/**.cc）
│   ├── test_writer/   # writer golden cases + run_case.sh 等脚本（无 .cc）
│   └── test_function/ # 线长/bbox 等数据集（不链入 module_test）
├── regression_test/   # Catch2 端到端 + [flow]
└── transform_format/  # txt ↔ json
```

另见实验性 SAT/ILP：[`algorithm/test_ILP/`](./algorithm/test_ILP/)（CaDiCal；见其 `AGENTS.md`）。

### module_test

入口：[`test/module_test/test_unit/test.cc`](./test/module_test/test_unit/test.cc)。

```bash
xmake build PR_tool_cli   # iterative 测试会调用 ./PR_tool_cli
xmake build module_test
cd output
./module_test <module>    # 或 xmake run module_test <module>
```

| 模块名 | 测试内容 |
|--------|----------|
| `cob` / `tob` / `interposer` | 硬件对象 |
| `router` / `placer` / `config` | 布线 / 布局 / 配置解析 |
| `comparator` / `path_length` / `debug` | 比较器 / 线长 / 日志 |
| `all` | 上表全部快速测试 |
| `placer_iteratively [config] [N]` | 放置+布线稳定性（默认 case1、100 次；不在 `all`） |
| `router_iteratively [config] [N]` | 仅布线稳定性（默认 case1、100 次；不在 `all`） |
| `writer <case> <path.txt> <out> [mode]` | controlbits 四文件写出（实现：`test_unit/test_writer.cc`） |

`test_writer/` 下的 case + `check-controlbits-file/scripts/run_case.sh` 编排 kiwi 对比；`test_function/` 提供独立数据集，**不会**编进 `module_test` target。

### regression_test

| 文件 | 标签 | 内容 |
|------|------|------|
| `test.cc` | `[basic]` `[CPU_MEM_AI]` `[CPU_MEM]` `[AI_core]` | 读配置 → build_nets → route，校验总线长 ≤ `golden.txt` |
| `incremental_test.cc` | `[incremental]` | 增量布线统计（默认 Catch 排除，需显式标签） |
| `flow_test.cc` | `[flow]` | COB=12 → rebuild → case5 布局/布线×10 → writer test1–5 |

```bash
xmake build regression_test
cd output
./regression_test              # 全部场景
./regression_test "[flow]"     # 产品流回归
./regression_test "[basic]"    # 仅指定标签
```

Linux 上需确保 `CONDA_PREFIX` 指向已安装 Catch2 的环境。

### 回归用例（`test/config/`）

目前共 **22** 个 case 目录（`case1` … `case22`，**含 case6**）。每个 case 通常含输入 JSON、`register_adder.json`、`golden.txt`（期望总线长上界）及 `description.txt`。

跑 maze 或 SAT 前须按 case 设置 `Interposer::COB_ARRAY_WIDTH`（见 [`test/AGENTS.md`](./test/AGENTS.md) **COB_ARRAY_WIDTH vs test/config cases**）。

#### case 1–6：基础功能（Muyan 小规模）

| case | 说明 |
|:----:|------|
| [case1](./test/config/case1) | 仅同步线（bus）布线；`placer_iteratively` / `router_iteratively` 默认用例 |
| [case2](./test/config/case2) | 同步线 + 额外非同步线，测试 bump 复用 |
| [case3](./test/config/case3) | 仅非同步线布线 |
| [case4](./test/config/case4) | 含 VDD/GND、更多线网（回归中暂未启用） |
| [case5](./test/config/case5) | 更多连接；`[flow]` 默认布局/布线用例 |
| [case6](./test/config/case6) | 扩展基础用例 |

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
| `-s, --simplify-controlbits-file` | 写出四文件时省略等于默认 hex 的行（稀疏输出） |
| `--router maze\|sat` | 布线后端（默认 `maze`）；`sat` 需 `xmake f --sat_router=y`（默认开启） |
| `--scope-pad N` | SAT 首轮 pair bbox 外扩格数（仅 `--router sat`；默认 0） |
| `--delay-pad N`, `-d N` | SAT 首轮 delay 扩展（仅 `--router sat`） |
| `--sat-log`, `--max-rss-mb N` | CaDiCal 日志与内存上限（SAT） |
| `--ilp-optimize -L <percent>` | 可选 v15 Gurobi 线长优化（SAT） |
| `-v, --verbose` | 输出 Debug 日志 |
| `-h, --help` | 帮助 |
| `-V, --version` | 版本信息 |

构建选项：`xmake f --sat_router=y|n`（默认 `y`，关闭后 `--router sat` 不可用）；SAT 求解器 `xmake f --cadical=y`。

示例：

```bash
xmake run PR_tool test/config/case1 -v
xmake run PR_tool_cli test/config/case7 --router sat --scope-pad 1 -v
xmake run PR_tool -g
```

**v1.0.0 不支持**：`-i/--incremental` 与 `-c/--compare` 已从 CLI 移除；传入会 FATAL 退出。

---

## 工具程序

`tools/` 与 `test/transform_format/` 提供辅助程序。正式产品输出由 Writer 写四文件；`split_regs.py` 仅 legacy 旧单文件离线拆分（详见 [`tools/AGENTS.md`](./tools/AGENTS.md)）。

| target / 脚本 | 说明 |
|--------|------|
| `cobmap` | 计算 COB 端口映射 |
| `view2d` / `view3d` | 加载配置、执行 P&R、可视化 |
| `parse_controlbits` | 解析旧单文件 controlbits（读回未跟四文件） |
| `txt2json` / `json2txt` | 旧版 txt ↔ JSON |
| `convert_prtool_configs_to_3dblox.py` | JSON case → 3DBlox 相关文件 |
| `vis_sat/` | SAT 路径可视化（配合 `test_ILP`） |

```bash
xmake build view2d
xmake run view2d <config_folder>
```

---

## 项目结构（其他目录）

| 目录 | 说明 |
|------|------|
| [`algorithm/test_ILP/`](./algorithm/test_ILP/) | 实验性统一图 SAT（+可选 ILP）布线；`xmake build test_ILP` |
| [`document/`](./document/) | 项目文档 |
| [`resource/`](./resource/) | Qt GUI 资源 |
| [`tools/`](./tools/) | 独立工具源码 |
| [`output/`](./output/) | 构建产物与运行日志（`debug.log` 等） |

---

## 分支说明

| 分支 | 说明 |
|------|------|
| `master` | 主开发线 |
| `fix.controlbits` | controlbits / CLI / writer 修复线（已合入 `test_ILP` 与 `test_unit` 布局） |
| `dev.algo_SAT_MCF_latest` 等 | 算法实验历史分支 |

生产布局布线实现位于 `source/algo/`；SAT/ILP 实验不替代正式 router。
