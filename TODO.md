# TODO

## 输出模块：直接写出 4 个 split 寄存器文件

设计与实现计划见：

- `docs/superpowers/specs/2026-07-21-split-register-output-design.md`
- `docs/superpowers/plans/2026-07-21-split-register-output.md`

### 已锁定决策（摘要）


| 项               | 选择                                                                                                    |
| --------------- | ----------------------------------------------------------------------------------------------------- |
| 产品输出            | 仅 `{output}/regnamecontrolbit_4part/` 下 4 文件（`hex address reg_name`）；**不再**写 `controlbits_<mode>.txt` |
| `-s` 主入口        | `PR_tool -s` 直接写稀疏四文件                                                                                 |
| `split_regs.py` | 保留为 **legacy/外部** 旧单文件离线工具（A1）；**不是**正式产品或主 `-s` 路径                                                   |
| CLI             | 只写一棵树（C1）；full+simplified pair 仅测试                                                                    |
| 缺/多余寄存器         | `debug::warning` → `debug.log`；不用 `report.log`                                                        |


### 推荐工作流

- 全量：`PR_tool <config> -o <out>`
- 稀疏：`PR_tool <config> -o <out> -s`
- **不要**再以「PR 全量单文件 → `split_regs.py -s`」作为主路径



### 本交付已完成（实现侧）

- Case `reigster_adder` → `RegisterMapConfig`；`read_config` 传 map
- Writer `write_split_files`；CLI 一棵树；test pair API + golden / `-s` 校验脚本



### 仍待后续（代码已标 `TODO(split-output)`）


| 位置                                      | 说明                                               |
| --------------------------------------- | ------------------------------------------------ |
| `load_controlbits` / `read_controlbits` | 跳过布线时仍读旧单文件                                      |
| `-c/--compare` + `controlbits_parser`   | 仍假设两列单文件                                         |
| `tools/parse_controlbits.cc`            | 输入约定未改                                           |
| 多 mode                                  | 需 `mode_<m>/regnamecontrolbit_4part/` 避免覆盖（注释已标） |


GUI 工具栏导出已改为选输出根目录 +「智能简化寄存器输出」勾选（见 `docs/superpowers/specs/2026-07-21-gui-controlbit-export-design.md`）。

### 参考


| 文件                                                                                     | 作用                                |
| -------------------------------------------------------------------------------------- | --------------------------------- |
| `tools/register_map.json`                                                              | map 模板；复制到各 case `reigster_adder` |
| `tools/split_regs.py`                                                                  | legacy：旧单文件 → 4 文件                |
| `source/parse/writer/writer.cc`                                                        | `write_split_files` / fetch       |
| `test/module_test/test_writer/check-controlbits-file/scripts/compare_controlbits.py`   | 四文件 vs golden                     |
| `test/module_test/test_writer/check-controlbits-file/scripts/verify_simplify_split.py` | `-s` omit/keep 校验                 |




## CLI 发布前审查（2026-07-21）

结论：当前命令行版本不应直接发布。普通 case 可以完成“读配置 → 建网 → 布线 → 写四个寄存器文件”，但失败语义、输入安全、多 mode 输出和 CLI 打包仍有发布阻断问题。

### P0：发布阻断

- [ ] **禁用增量布线功能。** 当前版本不支持增量布线；CLI 应拒绝 `-i` / `--incremental` 以及关联的 `-c` / `--compare` 参数，并在帮助文本中明确该限制。

- [ ] **普通布线失败必须失败退出，禁止写出成功制品。** `route_nets` 捕获 `RetryExpt` 后只记录日志，补救逻辑被注释；当前命令已出队，剩余 net 不会继续路由。随后 CLI 仍可能写控制位并返回 0。涉及 `source/algo/router/route_nets.cc`、`source/algo/router/command_mode/invoker.cc`、`source/app/cli/cli.cc`。修复：将失败状态传播到 CLI；只在所有目标 net 均成功且输出校验通过后返回 0。

- [ ] **初始化 TOB COB-unit 资源计数。** `TOB::_cobunit_resources` 未初始化，却被 `collect_cobunit_usage()` 以 `+=` 更新，并作为 maze 起始轨道排序依据。涉及 `source/hardware/tob/tob.hh`、`source/hardware/tob/tob.cc`、`source/algo/router/common/maze/mazeroutestrategy.cc`。修复：值初始化，并明确每轮资源统计是否需要清零。

- [ ] **修复 TXT 配置解析的越界与未初始化读取。** `numbers[11]` 无初始化、无字段数检查，长行可越界写，短行会读取残留值；`externs[info[4]]` 也没有范围检查。涉及 `source/parse/reader/config/config.cc`。修复：逐行严格要求 11 个字段，拒绝多/少字段，并验证所有索引与坐标。

- [ ] **移除 JSON 连接对** `assert` **的依赖。** 连接 pair 仅通过 `assert(net.size() == 2)` 验证，Release 构建下会直接访问越界元素。涉及 `source/parse/reader/config/config.cc`。修复：使用运行时异常并附带文件、mode、连接位置。

- [ ] **闭合四文件输出与回读/比较。** 当前正式输出为 `regnamecontrolbit_4part/`，但 `read_controlbits` 和比较功能仍只读取 `controlbits_<mode>.txt`；比较失败只记日志，不影响退出码。涉及 `source/parse/reader/controlbits/controlbits.cc`、`source/parse/comparator/controlbits_parser.cc`、`source/app/cli/cli.cc`。修复：支持四文件读回、比较与路径参数；删除或明确隔离 legacy 单文件路径。

- [ ] **拆分 CLI 与 GUI 构建目标。** 当前 `PR_tool` 目标包含全部 `source/**.cc`、Qt Widget/OpenGL 规则与资源；生成的 CLI 二进制依赖 QtCore/Gui/Widgets/OpenGL。涉及 `xmake.lua`。修复：创建无 Qt 的 `PR_tool_cli` 目标，只编译 CLI 必需源码，并提供可发布的 headless 包。



### P1：高风险

- [ ] **将布局合法性改为运行时校验，并拒绝重复 TOB 放置。** 当前合法性仅 `assert`；相同 TOB 的后一个实例可覆盖前一个实例。涉及 `source/app/cli/cli.cc`、`source/circuit/basedie.cc`。

- [ ] **修复仅剩一个空闲 TOB 时的除零。** `randomly_get_a_idle_tob()` 传入 `random_i64(0, size - 1)`，而随机函数取模 `(max-min)`；size=1 会模零，且 size>1 时最后一个候选永远不会被选中。涉及 `source/hardware/interposer.cc`、`source/global/utility/random.cc`。

- [ ] **让输出成为完整、原子的制品。** register map 中缺项或多项目前只警告；四文件逐个截断写，失败可留下新旧混合；未核验最终写入状态。涉及 `source/parse/writer/writer.cc`。修复：严格覆盖校验、临时目录写入、flush/close 检查后原子替换。

- [ ] **在开始布线前验证必需的 register map。** 配置将 `reigster_adder` 视为 optional，但 writer 在完成布线后才因空 map 终止。涉及 `source/parse/reader/config/config.cc`、`source/parse/writer/writer.cc`。

- [ ] **明确** `-p` **与已有控制位的行为。** CLI 在回读旧控制位前先修改布局，可能以新位置加载旧路径并跳过重布线。涉及 `source/app/cli/cli.cc`。

- [ ] **加强回归测试的成功判定。** 基础测试仅检查总线长度上界，不检查 `_failed_net == 0`；路由失败反而可能缩短长度并通过。涉及 `test/regression_test/test.cc`。README 还标注 case 8、9、14--16 为已知失败，不能作为发布通过基线。



### P2：中风险与可用性

- [ ] 非法枚举值没有抛出异常，只构造了未使用的 `runtime_error`；涉及 `source/serde/de.hh`。
- [ ] JSON scanner 不支持字符串转义，未闭合字符串可能越界扫描；根对象后的多余内容未校验，重复 key 静默保留第一项；涉及 `source/serde/json/jsonscanner.cc`、`source/serde/json/json.cc`。
- [ ] 重写 CLI 参数解析：拒绝未知参数、检查 `stoi` 是否完整消费，并补全帮助文本；涉及 `source/app/PR_tool.cc`。
- [ ] 将日志与错误分流：错误输出到 stderr；非 TTY 不输出 ANSI；移除 `std::ends` 写入的 NUL 字节。涉及 `source/global/debug/console.cc`。
- [ ] 处理零 net/零同步 net 的统计除零，避免 `NaN/Inf`；涉及 `source/algo/route_data.cc`、`source/algo/router/routeengine.cc`。
- [ ] 将外部 IO 默认选择位从硬编码行为改为显式、可验证的配置或架构约束；涉及 `source/parse/writer/module.cc`。
- [ ] 保证输出稳定排序，避免 `unordered_map` 导致的文件与行顺序不稳定；涉及 `source/parse/reader/config/config.hh`、`source/parse/writer/writer.cc`。
- [ ] 修复 `BumpToBumpsNet::connection_state()` 中未写入 routable/unroutable 容器的三元表达式，保证失败诊断有效；涉及 `source/circuit/net/types/bbsnet.cc`。
- [ ] 修复回归测试对当前工作目录的隐式依赖；从仓库根目录执行会找错 `../test/config`。涉及 `test/regression_test/test.cc`。



### 已做验证（2026-07-21）

- `PR_tool` 与 `regression_test` 可以构建；case1 CLI 正常返回并生成四个 split 寄存器文件。
- 在 `output/` 目录运行 `./regression_test '[basic]'` 通过（1 个场景、3 个断言）；该结果不能替代失败 net 检查。
- 指定不可创建的输出路径时，程序会先完成昂贵布线、再因创建输出目录失败而退出；应尽早验证输出目录。
