# TODO

## [待实现] 输出模块改造：直接写出 4 个 split 寄存器文件

本文档汇总「舍弃 `controlbits_<mode>.txt`、改为直接输出 4 个 REG 文件」及相关改动的需求与实现说明，供后续开发使用。

---

### 1. 背景与目标

**现状**

- PR_tool 通过 `parse::Writer` 写出单个 `controlbits_<mode>.txt`，格式为：`hex reg_name`（2 列）。
- 下游工具 `tools/split_regs.py` 再读取该文件 + `tools/register_map.json`，拆成 4 个文件，格式为：`hex address reg_name`（3 列）。
- 金标准工具直接输出 `regnamecontrolbit_4part/` 下 4 个文件（2 列 `hex reg_name`，无 address）；test_writer 的 SKILLs 流程用 `split_regs.py` 桥接后与 golden 比 hex。

**目标**

- **方案 C**：PR_tool **只**输出 4 个 split 文件，**不再**生成 `controlbits_<mode>.txt`。
- 输出目录：`{output}/regnamecontrolbit_4part/`。
- 每行格式与当前 `split_regs.py` 产物一致：`hex address reg_name`（3 列）。
- 4 个文件名与 `register_map.json` 一致：
  - `botleft_REG0.txt`
  - `botright_REG1.txt`
  - `topleft_REG2.txt`
  - `topright_REG3.txt`
- **`-s/--simplify-controlbits-file`** 继续生效，且**同样作用于** 4 个 split 文件：对 COB 全零、TOB `dly`/`drv` 全零的行整行省略（hex、address、name 均不输出）。
- 寄存器名 → 地址的映射从**每个 case 的配置**读取（见 §4），不再依赖 `tools/register_map.json` 作为运行时默认路径（该文件可保留作生成/对照用）。

---

### 2. 输出数据流（目标架构）

```text
config.json
  └─ reigster_adder.json   # 每个 case 一份完整 register map（与 tools/register_map.json 同构）

read_config(config_folder)
  └─ load_config() → Config（含 register_map）
  └─ 返回 (interposer, basedie, register_map)   # 方案 C

connect_registers → Writer::fetch → 内存 HashMap<reg_name, hex>

write_split_registers(register_map, reg_values, output/regnamecontrolbit_4part/, simplify)
  └─ 按 map 中 4 个文件分组遍历
  └─ 每行：maybe_write_line(hex, address, name)  # -s 时跳过默认 COB/dly/drv
```

**`-s` 省略规则（与现有 Writer 一致）**

| 寄存器范围 | 默认 hex | 可省略 |
|-----------|---------|--------|
| 全部 COB | `00000000` | 是 |
| TOB `dly` / `drv` | `00000000` | 是 |
| 其它 TOB、`xinzhai` | — | 否（始终输出） |

---

### 3. 代码改动清单

#### 3.1 配置解析（`source/parse/reader/config/`）

**现状**：`config.json` 中已有 `"reigster_adder": "reigster_adder.json"`，但 `ConfigFilepaths` / `Config` / `load_config()` **均未加载**该文件。

**需做**

1. 在 `config.hh` 增加类型与字段：

```cpp
using RegisterAddress = std::String;
using RegisterFileMap = std::HashMap<std::String, RegisterAddress>;       // reg_name -> address
using RegisterMapConfig = std::HashMap<std::String, RegisterFileMap>;    // output_filename -> regs

struct Config {
    // ... 现有字段 ...
    RegisterMapConfig register_map;
};
```

2. 在 `config.cc` 的 `ConfigFilepaths` 中增加可选字段（注意项目内拼写为 `reigster_adder`）：

```cpp
std::Option<std::FilePath> reigster_adder;  // DE_OPTION_FILED(reigster_adder)
```

3. 新增 `load_register_map_config(path, config.register_map)`：
   - JSON 结构与 `tools/register_map.json` 相同。
   - 可选校验：4 个文件名是否存在；`reg_name` 跨文件不重复。

4. 在 `load_config()` 末尾：若 `config_paths.reigster_adder` 有值，则加载。

5. **每个 test case** 的 `reigster_adder.json` 放入完整 map（由 `tools/register_map.json` 复制或脚本生成）。当前各 case 多为空 `{}`，需补齐。

**命名说明**：部分 case 的 `config.json` 指向 `register_adder.json`，部分为 `reigster_adder.json`；以 `config.json` 中的路径字符串为准即可，无需统一重命名历史文件。

#### 3.2 方案 C：将 register_map 传到 Writer

**需做**

1. 修改 `read_config()` 返回值（`source/parse/reader/module.hh` / `module.cc`）：

```cpp
// 原：Tuple<Interposer, BaseDie>
// 新：Tuple<Interposer, BaseDie, RegisterMapConfig>
// 或单独 struct ReadConfigResult { ... };
```

2. 更新所有 `read_config` 调用方：`cli.cc`、`test_writer.cc`、`regression_test`、`gui` 等，解构第三项并传给输出模块。

3. `output_from_routing_results` / `write_control_bits` 签名增加 `const RegisterMapConfig&`（或 `RegisterMapConfig` 拷贝/移动）。

4. `Reader::build()` **不必**为 map 改布线逻辑；map 仅用于写出。

#### 3.3 Writer / 输出模块（`source/parse/writer/`）

**需做**

1. `Writer::fetch` 阶段不变；在写出阶段：
   - 先在内存收集全部 `reg_name → hex`（遍历 COB/TOB/xinzhai 与现有 `write_*_template` 逻辑一致）。
   - **不再**写 `controlbits_<mode>.txt`。

2. 新增 `write_split_files(const RegisterMapConfig&, const HashMap<reg_name, hex>&, output_dir, simplify)`：
   - 创建 `{output_dir}/regnamecontrolbit_4part/`。
   - 对 map 中每个 `(filename, {reg_name: address})` 写对应文件。
   - 每行：`hex address reg_name`；`-s` 时对可省略项调用与现 `maybe_write_line` 相同逻辑。
   - 若 `reg_name` 在内存 map 中不存在（不应发生）：记 error 或 skip + warning（与现 `split_regs.py` 行为对齐：缺失则跳过并报告）。

3. 可选：写 `report.log` 到 `regnamecontrolbit_4part/`（移植 `split_regs.py` 的 missing/extra 报告），或改用 `debug::info` / `debug::warning`。

4. 删除或废弃 `write()` 写单文件的路径；`fetch_and_write` 改名为 `fetch_and_write_split` 或改参数。

#### 3.4 CLI（`source/app/PR_tool.cc`）

- 保持 `-s/--simplify-controlbits-file`。
- `-o/--output` 语义变为：输出根目录，其下生成 `regnamecontrolbit_4part/`。
- `print_help()` 更新说明。

#### 3.5 多 mode 输出（暂不实现）

- `try_all_modes` / 增量多 mode 时，4 个固定文件名会互相覆盖。
- **当前需求不使用多 mode 输出**；在 `output_from_routing_results` 的 `try_all_modes` 分支加注释，例如：

```cpp
// TODO: 多 mode 时需使用 mode_<m>/regnamecontrolbit_4part/ 子目录，避免覆盖
```

#### 3.6 直接失效、暂用 TODO 注释标记（后续再改）

以下功能依赖 `controlbits_<mode>.txt`（2 列单文件），改造后**会失效**。本次仅在相关代码处加 `TODO` / `待修改` 注释，不实现修复。

| 位置 | 现状 | TODO 说明 |
|------|------|-----------|
| `source/parse/reader/controlbits/controlbits.cc` `load_controlbits` | 读 `config_folder/controlbits_<mode>.txt` | 待支持从 `regnamecontrolbit_4part/` 读回；`-s` 稀疏文件需补默认 |
| `source/parse/reader/module.cc` `read_controlbits` | 跳过布线 / 增量 warm-start | 同上 |
| `source/app/cli/cli.cc` `parse::compare`（`-c/--compare`） | 比较两个单文件 | 待支持 split 格式或合并后比较 |
| `source/parse/comparator/controlbits_parser.cc` | 解析 `value name` 2 列 | 待支持 3 列 split 或先合并 |
| `source/widget/window.cc` `generateControlBitAs` | 保存单个 `.ctb` 路径 | 待改为选择输出目录 |
| `tools/parse_controlbits.cc` | 依赖 `load_controlbits` | 待改输入约定 |
| `tools/split_regs.py` | 后处理单文件 | 可保留作历史对照，文档标明已废弃 |

注释示例：

```cpp
// TODO(split-output): read_controlbits 仍假设 controlbits_<mode>.txt；输出已改为 regnamecontrolbit_4part/，读回待实现
```

---

### 4. 配置与数据准备

- **策略**：每个 `test/config/case*/` 目录内放一份完整 `reigster_adder.json`（与 `tools/register_map.json` 同构，约 5200 项）。
- **迁移**：一次性脚本将 `tools/register_map.json` 复制到各 case 的 `reigster_adder.json`（或 `config.json` 所指向的文件名）。
- `test/module_test/test_writer/test*/reigster_adder.json` 同样需补齐（writer 测例也走 `read_config`）。

---

### 5. 测试与 SKILLs 修改

#### 5.1 `compare_controlbits.py`

**Golden 对比前增加寄存器总数检查**（在逐步比 hex 之前）：

```python
if len(split_regs) != len(golden_regs):
    print("ERROR: register count mismatch ({split} vs {golden}).")
    print("Hint: golden compare requires full output; disable -s/--simplify-controlbits-file.")
    return 1
```

- Golden 对比必须在**未启用 `-s`** 的全量输出上进行。
- 仍按寄存器名对齐比 hex；golden 2 列，PR 输出 3 列（address 列忽略）。

#### 5.2 `verify_simplify_controlbits.py`

- 改为目录级对比：`--full-dir` / `--simplified-dir`，各含 4 个 `*_REG*.txt`。
- `load_split_dir()`：合并 4 文件为 `dict[name, hex]`（reg 名取第 3 列）。
- 逻辑不变：子集关系、可省略项、禁止误删、应省略项完整性。

#### 5.3 `check-controlbits-file/SKILLs.md`

| 原步骤 | 新步骤 |
|--------|--------|
| 目的：生成 `controlbits_0.txt` | 生成 `regnamecontrolbit_4part/` 下 4 文件 |
| §5：`split_regs.py` 拆解 | 删除；PR_tool 直接输出 split 格式 |
| 6：运行 test_writer 得单文件 | 运行 test_writer（**不带 `-s`**）得 `regnamecontrolbit_4part/` |
| 7：`split_regs.py` | **删除** |
| 8：比 `split_4files` vs golden | 比 PR 输出目录 vs `golden/regnamecontrolbit_4part/` |

补充注意：

- 金标准对比前依赖 `compare_controlbits.py` 的 count 检查。
- 简化专项：单独用 `-s` + `verify_simplify_controlbits.py`，不与 golden 混用。

#### 5.4 测例目录 `check_run/`

- 用 `check_run/regnamecontrolbit_4part/` 存放 PR 直接输出。
- 可废弃 `check_run/split_4files/`（原 split_regs 产物）、`check_run/pr_output/controlbits_0.txt`。
- simplify 对比：`check_run/full/` 与 `check_run/simplified/` 各含一套 4 文件。

#### 5.5 其它测试

| 测试 | 说明 |
|------|------|
| test_writer 两次全量输出 | 4 文件合并后应完全一致 |
| SKILLs test2 + compare（无 `-s`） | 主正确性验证 |
| verify_simplify（有 `-s`） | 简化专项 |
| `placer_iteratively` / `router_iteratively` case1 | 冒烟；不检查 controlbits |

---

### 6. 文档同步

| 文件 | 更新内容 |
|------|----------|
| `README.md` | 数据流图：`controlbits_<mode>.txt` → `regnamecontrolbit_4part/*.txt`；I/O 表 |
| `source/AGENTS.md` | §4.3 写出 controlbits；§9.4 输出格式；`reigster_adder` 配置说明 |
| `test/AGENTS.md` | 若提到单文件 controlbits，改为 4 文件 |

---

### 7. 建议实现顺序

1. `Config` + `load_register_map_config`；各 case 补齐 `reigster_adder.json`。
2. `read_config` 返回 `register_map`（方案 C）；调用方传参。
3. Writer 内存收集 + `write_split_files`；去掉单文件输出。
4. 更新 `compare_controlbits.py` count 检查；`verify_simplify_controlbits.py` 目录模式。
5. 更新 SKILLs.md；跑 test2 金标准流程。
6. 失效路径加 `TODO` 注释；README / AGENTS.md。
7. （可选）`report.log` 与 `split_regs.py` 行为对齐。

---

### 8. 风险与约束

- **破坏性变更**：任何依赖 `controlbits_0.txt` 的脚本/流程需迁移或标记 TODO。
- **`-s` 与 golden**：启用 `-s` 后寄存器行数少于 golden，**不能**做金标准对比；必须先做 count 检查。
- **register_map 缺失**：若 case 未配置 `reigster_adder` 或文件为空，写出时应 `fatal` 并提示配置 map。
- **改动量**：C++ 约 200–350 行 + 配置数据复制 + 测试脚本/文档；超过 100 行 C++ 时按 `source/AGENTS.md` §0 在 `.plan/` 留改动记录并做代码审核。

---

### 9. 参考文件

| 文件 | 作用 |
|------|------|
| `tools/register_map.json` | map 格式金样；复制到各 case |
| `tools/split_regs.py` | 当前后处理逻辑（待废弃） |
| `source/parse/writer/writer.cc` | 现有 fetch / `maybe_write_line` |
| `source/parse/writer/module.cc` | `write_control_bits` 入口 |
| `source/parse/reader/config/config.cc` | `load_config` |
| `test/module_test/test_writer/compare_controlbits.py` | golden 对比 |
| `test/module_test/test_writer/verify_simplify_controlbits.py` | 简化专项 |
| `test/module_test/test_writer/check-controlbits-file/SKILLs.md` | writer 调试流程 |
