# PR_tool /tools 工程指南（面向 AI Agent）

本文件是 `tools/` 目录的入口说明书。目标是：AI 阅读本文件后，能正确选用辅助工具、执行常用命令，并理解与 `source/`、`test/` 的边界。

`tools/` 是 PR_tool 的**外围辅助工具集**，不是核心布局布线实现。算法、对象模型与主程序流程以 `source/AGENTS.md` 为准；测试用例与回归以 `test/AGENTS.md` 为准。

---

## 0. 工作规则

- 必须深入理解材料，在理解的基础上再改动
- 完成修改后，评估是否需要维护本文件及其他相关 `AGENTS.md`
- 若单次改动超过 100 行，在项目根目录 `.plan/` 下生成改动记录（可参考该目录已有文件），并启动独立子 agent 审核完整性和正确性
- **本文件（`tools/AGENTS.md`）行数不得超过 200 行**。增补时优先压缩低频工具说明与重复内容，不要拆成多份入口文档

---

## 1. 定位与目录边界

| 文件 | 作用 |
|------|------|
| `split_regs.py` | **Legacy**：旧单文件 `controlbits_*.txt` → 4 个 REG（离线；非正式 Writer） |
| `register_map.json` | 寄存器名 → 地址/分区 map（对照模板；case 侧文件名为 `register_adder.json`，`config.json` 键仍为 `reigster_adder`） |
| `port_allocator.py` | 约束目录中的 `connection.txt` → `connections.json` |
| `parse_controlbits.cc` | 从旧单文件 controlbits 反推路径（调试；**读回未跟四文件** `regnamecontrolbit_4part/`） |
| `convert_prtool_configs_to_3dblox.py` | PR_tool JSON case → 3DBlox 相关文件（实验/格式转换） |
| `vis_sat/` | SAT 路径可视化脚本与样例图（配合 `algorithm/test_ILP`） |
| `view2d.cc` / `view3d.cc` | 加载配置并 P&R 后 2D/3D 可视化（低频） |
| `cobmap.cc` | COB 方向索引映射查询（低频） |
| `count_lines.py` | 统计 `./source` 代码行数 |

边界：

- 主程序 CLI/GUI、Writer/Reader、P&R → `source/`
- 集成测例、module/regression → `test/`
- 正式产品输出是 Writer 的 `regnamecontrolbit_4part/`；`tools/` 不替代该链路

---

## 2. split_regs + register_map（legacy 离线）

### 2.1 用途

**不是**正式产品路径。PR_tool Writer 已直接写出 `{output}/regnamecontrolbit_4part/` 下 4 个文件（三列 `hex address reg_name`）。

本脚本仅用于 **legacy / 外部** 旧单文件 `controlbits_<mode>.txt`（两列 `hex reg_name`）→ 同上 4 文件：

- `botleft_REG0.txt` / `botright_REG1.txt` / `topleft_REG2.txt` / `topright_REG3.txt`

### 2.2 命令

```bash
python3 tools/split_regs.py \
  -c <path/to/controlbits_0.txt> \
  -j tools/register_map.json \
  -o <output_dir> \
  [-s]
```

- `-c/--controlbits`：旧单文件输入
- `-j/--json_map`：map（模板 `tools/register_map.json`）
- `-o/--output_dir`：输出目录；省略则写到 controlbits 所在目录
- `-s`：split 写出时省略默认 hex 行（规则同 `register_defaults.hh`）

### 2.3 `-s` 与推荐入口

- **推荐**：`PR_tool` 全量四文件，或 `PR_tool -s` 稀疏四文件（见 `source/AGENTS.md`）
- **不要**把 `split_regs.py -s` 当作主 `-s` 入口
- 改默认省略逻辑时：同步 C++ `register_defaults.hh` 与本脚本

---

## 3. port_allocator（测例准备）

### 3.1 用途

在约束目录中根据 `connection.txt` 与若干 JSON，自动分配端口并写出 `connections.json`。

必备输入（同目录）：

- `01_ports.json`
- `external_ports.json`
- `topdie_insts.json`
- `topdies.json` 或 `topdie.json`
- `connection.txt`（可用 `--connection-file` 改名）

### 3.2 命令

```bash
python3 tools/port_allocator.py <constraint_dir> \
  [--connection-file connection.txt] \
  [--output connections.json]
```

### 3.3 能力摘要

`connection.txt` 支持（细节见脚本顶部 docstring）：

- `# mode N` 分段；`pose`/`nege` 计数请求；`simple`/`bus` 连接
- `# usable_ex_port`：限制可用外部端口
- `# multi_fanout_chip_port`：允许同 mode 内端口多扇出

分配失败抛 `AllocationError` 并以非零退出码结束。

---

## 4. parse_controlbits（反推调试）

### 4.1 用途

从已有 **旧单文件** `controlbits_<mode>.txt` 加载寄存器，反推 bump/track 端点与路径。调试用；**不是**主输出链路。读回尚未适配 `regnamecontrolbit_4part/`。

### 4.2 构建与运行

```bash
xmake build parse_controlbits
./output/parse_controlbits -folder <controlbits_dir> -mode <mode>
```

日志默认写到当前目录 `./parse_controlbits.log`。

### 4.3 流程摘要

1. 解析 `-folder` / `-mode`，加载 controlbits
2. 从 TOB bits 收集 begin/end bump 与 track
3. 校验端点并构建 TOBConnector
4. 根据 COB bits 搜索路径，挂到 `BaseDie` nets
5. 检查剩余 track→bump，展示 net 信息

算法细节不在此展开；改行为时直接读 `tools/parse_controlbits.cc`。

---

## 5. 低频工具（简述）

### view2d / view3d

加载 config → `build_nets` + Maze 布线 → Qt 2D/3D 展示。日常几乎不用。

```bash
xmake build view2d   # 或 view3d
./output/view2d <config_path>
./output/view3d <config_path>
```

### cobmap

查询 COB `cob_index` / `track_index`（实现里子命令拼写为 `tracl_index`）在方向间的映射。日常几乎不用。

```bash
xmake build cobmap
./output/cobmap <cob_index|tracl_index> <from_dir> <from_index> <to_dir>
# from_dir/to_dir: left|l / right|r / up|u / down|d
```

### count_lines.py

```bash
python3 tools/count_lines.py   # 从仓库根目录运行；统计 ./source
```

---

## 6. 构建速查与修改指南

### 6.1 C++ 目标（均在根目录 `xmake.lua`）

| target | 源文件 | 说明 |
|--------|--------|------|
| （Python 工具） | `tools/*.py` | 无需 xmake，直接 `python3 tools/...` |
| `parse_controlbits` | `tools/parse_controlbits.cc` | `set_default(false)`，产物 `./output/` |
| `view2d` / `view3d` | `tools/view2d.cc` / `view3d.cc` + widget | Qt；`set_default(false)` |
| `cobmap` | `tools/cobmap.cc` | `set_default(false)` |

### 6.2 常见修改

- 改 split 简化规则 → 同步 `tools/split_regs.py` 与 `source/parse/writer/register_defaults.hh`
- 改寄存器分区/地址 → 更新 `tools/register_map.json` 与各 case 的 **`register_adder.json`**（勿改 `config.json` 键名 `reigster_adder`）；正式写出走 Writer
- 改 `port_allocator` 语法 → 更新脚本 docstring，并回归相关约束目录生成的 `connections.json`
- 核心 P&R / Writer 行为 → 改 `source/`，并维护 `source/AGENTS.md`
- SAT 可视化 / 3DBlox 转换 → `vis_sat/`、`convert_prtool_configs_to_3dblox.py`；算法本体见 `algorithm/test_ILP/AGENTS.md`

### 6.3 日志约定

- C++：`PR_tool::debug`（`parse_controlbits` 另有 `./parse_controlbits.log`）
- Python：`split_regs` 打印 REPORT；`port_allocator` 用 `AllocationError`

