# tools/AGENTS.md Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create `tools/AGENTS.md` — a Chinese, medium-depth agent guide for the `tools/` directory (≤200 lines).

**Architecture:** Single new markdown file under `tools/`, structured by workflow (split_regs / port_allocator / parse_controlbits first; view2d/view3d/cobmap one-liners). Content locked by spec `docs/superpowers/specs/2026-07-21-tools-agents-md-design.md`. No code or xmake changes.

**Tech Stack:** Markdown documentation only; verify with `wc -l`.

**Spec:** `docs/superpowers/specs/2026-07-21-tools-agents-md-design.md`

---

## File map

| File | Action | Responsibility |
|------|--------|----------------|
| `tools/AGENTS.md` | Create | Sole deliverable: agent entry guide for `tools/` |
| `docs/superpowers/specs/2026-07-21-tools-agents-md-design.md` | Reference only | Do not edit unless user requests |

---

### Task 1: Create `tools/AGENTS.md`

**Files:**
- Create: `tools/AGENTS.md`

- [ ] **Step 1: Write the file with the exact content below**

Create `tools/AGENTS.md` with this full content (do not expand beyond it; keep ≤200 lines). Outer fence uses four backticks so nested triple-backtick commands stay intact:

````markdown
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
| `split_regs.py` | 全量 controlbits → 4 个 REG 文件 |
| `register_map.json` | 寄存器名 → 地址/分区 map（供 split 使用） |
| `port_allocator.py` | 约束目录中的 `connection.txt` → `connections.json` |
| `parse_controlbits.cc` | 从 controlbits 反推路径/连通（调试） |
| `view2d.cc` / `view3d.cc` | 加载配置并 P&R 后 2D/3D 可视化（低频） |
| `cobmap.cc` | COB 方向索引映射查询（低频） |
| `count_lines.py` | 统计 `./source` 代码行数 |

边界：

- 主程序 CLI/GUI、Writer/Reader、P&R → `source/`
- 集成测例、module/regression → `test/`
- `tools/` **不替代**正式 Writer 输出链路；当前是下游桥接与测例准备

---

## 2. split_regs + register_map（主工作流）

### 2.1 用途

PR_tool 写出全量 `controlbits_<mode>.txt`（两列：`hex reg_name`）后，本脚本按 `register_map.json` 拆成 4 个文件（三列：`hex address reg_name`）：

- `botleft_REG0.txt`
- `botright_REG1.txt`
- `topleft_REG2.txt`
- `topright_REG3.txt`

### 2.2 命令

在仓库根目录：

```bash
python3 tools/split_regs.py \
  -c <path/to/controlbits_0.txt> \
  -j tools/register_map.json \
  -o <output_dir> \
  [-s]
```

- `-c/--controlbits`：输入 controlbits（**推荐全量**，即 PR_tool 未加 `-s` 的输出）
- `-j/--json_map`：寄存器 map（默认对照用 `tools/register_map.json`）
- `-o/--output_dir`：输出目录；省略则写到 controlbits 所在目录
- `-s/--simplify-controlbits-file`：在 **split 写出阶段** 省略默认 hex 行

### 2.3 `-s` 与默认规则

- 省略规则与 `source/parse/writer/register_defaults.hh` 一致（`tob_*_track2tob_*` 与部分 `tob2bump_bank*_en_*` 默认 `ffffffff`，其余默认 `00000000`）
- 推荐工作流：PR_tool **全量**写出 → `split_regs.py -s` 做下游简化
- 不要用「已简化的 controlbits」再跑 `-s` 当作完整对照

### 2.4 报告与踩坑

脚本结束打印 REPORT：

- map 有、controlbits 无 → missing
- controlbits 有、map 未用到 → extra
- `-s` 时统计 omitted / written 行数

改默认省略逻辑时：**必须**同步 C++ `register_defaults.hh` 与本脚本。

前瞻：根目录 `TODO.md` 计划让 PR_tool 直接写 4 个 split 文件；在落地前，本脚本仍是正式下游桥接。

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

从已有 `controlbits_<mode>.txt` 加载寄存器，反推 bump/track 端点与路径，打印 net 信息。用于调试/对照，**不是**主输出链路。

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
- 改寄存器分区/地址 → 更新 `tools/register_map.json`，并核对该 map 的下游比对流程
- 改 `port_allocator` 语法 → 更新脚本 docstring，并回归相关约束目录生成的 `connections.json`
- 核心 P&R / Writer 行为 → 改 `source/`，并维护 `source/AGENTS.md`

### 6.3 日志约定

- C++：`PR_tool::debug`（`parse_controlbits` 另有 `./parse_controlbits.log`）
- Python：`split_regs` 打印 REPORT；`port_allocator` 用 `AllocationError`

````

- [ ] **Step 2: Verify line count ≤ 200**

Run:

```bash
wc -l tools/AGENTS.md
```

Expected: a number **≤ 200** (target roughly 170–195). If over 200, delete blank lines / shorten §5–§6 before committing — do not split into a second AGENTS file.

- [ ] **Step 3: Spec checklist (manual)**

Confirm against `docs/superpowers/specs/2026-07-21-tools-agents-md-design.md`:

1. Chinese + numbered sections 0–6
2. §0 includes「本文件不得超过 200 行」
3. `view2d`/`view3d`/`cobmap` have no workflow sections
4. `split_regs` / `port_allocator` / `parse_controlbits` include runnable commands
5. Cross-refs to `source/AGENTS.md` and `test/AGENTS.md` present

- [ ] **Step 4: Commit**

```bash
git add tools/AGENTS.md
git commit -m "$(cat <<'EOF'
docs(tools): add AGENTS.md for auxiliary tools

Give agents a ≤200-line Chinese entry guide covering split_regs,
port_allocator, parse_controlbits, and brief notes on view*/cobmap.
EOF
)"
```

---

## Self-review (plan author)

| Spec requirement | Task coverage |
|------------------|---------------|
| Create Chinese medium guide ≤200 lines | Task 1 Steps 1–2 |
| §0 includes 200-line rule | Embedded in Step 1 content |
| Workflow for split_regs / port_allocator / parse_controlbits | §2–§4 in Step 1 |
| Brief view2d/view3d/cobmap only | §5 in Step 1 |
| Build/modify guide | §6 in Step 1 |
| `wc -l` verification | Step 2 |

No placeholders. Single-file scope — no subsystem split needed.
