# TODO

## 输出模块：直接写出 4 个 split 寄存器文件

设计与实现计划见：

- `docs/superpowers/specs/2026-07-21-split-register-output-design.md`
- `docs/superpowers/plans/2026-07-21-split-register-output.md`

### 已锁定决策（摘要）

| 项 | 选择 |
|----|------|
| 产品输出 | 仅 `{output}/regnamecontrolbit_4part/` 下 4 文件（`hex address reg_name`）；**不再**写 `controlbits_<mode>.txt` |
| `-s` 主入口 | `PR_tool -s` 直接写稀疏四文件 |
| `split_regs.py` | 保留为 **legacy/外部** 旧单文件离线工具（A1）；**不是**正式产品或主 `-s` 路径 |
| CLI | 只写一棵树（C1）；full+simplified pair 仅测试 |
| 缺/多余寄存器 | `debug::warning` → `debug.log`；不用 `report.log` |

### 推荐工作流

- 全量：`PR_tool <config> -o <out>`
- 稀疏：`PR_tool <config> -o <out> -s`
- **不要**再以「PR 全量单文件 → `split_regs.py -s`」作为主路径

### 本交付已完成（实现侧）

- Case `reigster_adder` → `RegisterMapConfig`；`read_config` 传 map
- Writer `write_split_files`；CLI 一棵树；test pair API + golden / `-s` 校验脚本

### 仍待后续（代码已标 `TODO(split-output)`）

| 位置 | 说明 |
|------|------|
| `load_controlbits` / `read_controlbits` | 跳过布线 / 增量 warm-start 仍读旧单文件 |
| `-c/--compare` + `controlbits_parser` | 仍假设两列单文件 |
| `tools/parse_controlbits.cc` | 输入约定未改 |
| 多 mode | 需 `mode_<m>/regnamecontrolbit_4part/` 避免覆盖（注释已标） |

GUI 工具栏导出已改为选输出根目录 +「智能简化寄存器输出」勾选（见 `docs/superpowers/specs/2026-07-21-gui-controlbit-export-design.md`）。

### 参考

| 文件 | 作用 |
|------|------|
| `tools/register_map.json` | map 模板；复制到各 case `reigster_adder` |
| `tools/split_regs.py` | legacy：旧单文件 → 4 文件 |
| `source/parse/writer/writer.cc` | `write_split_files` / fetch |
| `test/module_test/test_writer/check-controlbits-file/scripts/compare_controlbits.py` | 四文件 vs golden |
| `test/module_test/test_writer/check-controlbits-file/scripts/verify_simplify_split.py` | `-s` omit/keep 校验 |
