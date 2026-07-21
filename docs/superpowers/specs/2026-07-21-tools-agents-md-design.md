# Design: `tools/AGENTS.md`

Date: 2026-07-21  
Status: approved for drafting (pending user review of this spec)

## Goal

为 `tools/` 目录新增面向 AI Agent 的入口说明 `tools/AGENTS.md`，使 Agent 在不阅读全部脚本源码的情况下，能正确选用辅助工具、执行常用命令，并理解与 `source/`、`test/` 的边界。

## Constraints

- 中文，章节风格对齐 `source/AGENTS.md`（含工作规则小节）
- 深度：中等手册（目录 + 重点工作流 + 踩坑 + 构建速查），不超过 **200 行**
- 组织：按工作流切分；`view2d` / `view3d` / `cobmap` 仅极简描述，不写工作流
- 工作规则中必须明确：**维护本文件时行数不得超过 200 行**

## Non-goals

- 不复述 `source/` 中 P&R / Writer 深讲
- 不展开 `parse_controlbits` 算法细节
- 不把 `TODO.md` 中的「PR_tool 直接写 4 文件」当作已实现行为（可一句前瞻）

## Document outline

```text
# PR_tool /tools 工程指南（面向 AI Agent）
0. 工作规则（含 ≤200 行约束）
1. 定位与目录边界
2. split_regs + register_map（主工作流）
3. port_allocator（测例准备）
4. parse_controlbits（反推调试）
5. 低频工具（view2d / view3d / cobmap / count_lines）
6. 构建速查与修改指南
```

## Section content

### §0 工作规则

- 必须深入理解材料后再改
- 改完评估是否维护本 `AGENTS.md` 及其他相关 AGENTS
- 大改动（如超过 100 行）在根目录 `.plan/` 写改动记录，并启动子 agent 审核
- **本文件（`tools/AGENTS.md`）行数不得超过 200 行**；增补时优先压缩低频工具与重复说明，而不是拆成多份入口

### §1 定位与目录边界

- `tools/` = 外围辅助，不是核心 P&R
- 文件一览表（一句话/文件）
- 边界：算法与主程序 → `source/AGENTS.md`；用例与回归 → `test/AGENTS.md`

### §2 `split_regs.py` + `register_map.json`

- 输入：全量 `controlbits_<mode>.txt`（`hex reg_name`）
- 输出：4 文件（`hex address reg_name`），文件名与 map 一致
- CLI：`-c` / `-j` / `-o` / `-s`
- `-s`：对全量输入，在 split 写出阶段按与 `register_defaults.hh` 相同规则省略默认 hex
- 踩坑：missing/extra 报告；改默认规则必须与 C++ 同步
- 前瞻：`TODO.md` 计划主程序直接写 4 文件；落地前本脚本仍是正式桥接

### §3 `port_allocator.py`

- 约束目录 → `connections.json`
- 必备输入与 CLI 入口
- 能力关键词（mode / pose|nege / simple|bus / usable_ex_port / multi_fanout）；细节指向脚本 docstring

### §4 `parse_controlbits`

- 构建与用法：`xmake build parse_controlbits`；`-folder` / `-mode`
- 流程摘要（load → 端点 → 搜路径 → 检查 → 展示），不写算法

### §5 低频工具

- `view2d` / `view3d`：各 1–2 句 + 构建命令
- `cobmap`：1–2 句 + Usage
- `count_lines.py`：1 句

### §6 构建速查与修改指南

- C++ 目标均在 `xmake.lua`，`set_default(false)`，产物 `./output/`
- 改 `split_regs` 简化规则 ↔ `register_defaults.hh`
- 改 map 结构 ↔ 下游比对流程
- 日志：C++ 用 `debug`；Python 用 REPORT / `AllocationError`

## Delivery

1. 写入并提交本 design spec（本文件）
2. 用户审阅本 spec
3. 经 `writing-plans` 产出实现计划后，生成 `tools/AGENTS.md`（≤200 行，含 §0 行数约束）
4. 用 `wc -l` 验证行数上限

## Success criteria

- [ ] `tools/AGENTS.md` 存在且中文、结构与上表一致
- [ ] `wc -l tools/AGENTS.md` ≤ 200
- [ ] §0 明确写出「本文件不得超过 200 行」
- [ ] `view2d`/`view3d`/`cobmap` 无工作流展开
- [ ] `split_regs` / `port_allocator` / `parse_controlbits` 含可执行命令与关键约定
