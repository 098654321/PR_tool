> **Archived.** Current skill entry: [SKILL.md](SKILL.md); scripts live under [scripts/](scripts/).

## check-controlbits-file

本技能用于对PR_TOOL的输出模块进行debug。

## 目的

PR_TOOL接受json格式的输入，运行后在 output 下写出 `regnamecontrolbit_4part/` 四个寄存器文件。目前怀疑寄存器配置生成有错误，所以对 Writer 输出部分进行debug。

本 SKILL 只验证 writer 输出；路径来自金标准 net_path_info.txt，不验证 PR_TOOL 的 router/placer。

## 参考资料

1. 查bug的金标准工具

本工具的前身是 `/Users/jiaheng/FDU_files/Tao_group/PR_tool/jsy_version_tool/xinzhai_sourcecode1_new`，这个工具在运行后会输出 `regnamecontrolbits_4part/` 目录与一个 `net_path_info.txt`文件，其中目录包含4个controbit文件，而路径信息文件以旧工具的坐标体系为基准生成路径坐标信息。这个工具接受txt格式的输入

2. 旧工具坐标体系与PR_TOOL工具坐标体系

可以参考文件`/Users/jiaheng/FDU_files/Tao_group/PR_tool/PR_tool/test/transform_format/txt&json_coord_relationship.txt`

3. PR_TOOL的内容

可以查看`source/AGENTS.md`文件了解PR_TOOL的项目规范与内容

4. 一些测例

在 `test/module_test/test_writer`下有一些测例，用于本类debug

5. 格式转换工具

在`test/transform_format`下有`json2txt.cc`与`txt2json.cc`，可以在json和txt之间转换测例的文件形式。其中txt代表了旧工具的输入，json代表PR_TOOL的输入。
测例 `config.json` 的 `reigster_adder` 指向寄存器 map（与 `tools/register_map.json` 同结构）：四个文件名、寄存器名与地址。PR_TOOL Writer 直接写出 `regnamecontrolbit_4part/{botleft_REG0,botright_REG1,topleft_REG2,topright_REG3}.txt`（三列：`hex address reg_name`），不再依赖 `tools/split_regs.py` 做金标准对照桥接。`split_regs.py` 仅用于离线处理旧的单文件 `controlbits_<mode>.txt`。
`test/module_test/test_writer/trans_path_old2new.py`可以把金标准工具生成的路径信息转换为PR_TOOL新坐标体系下的路径信息

 
## 工作流程

0. 准备PR_TOOL

准备好当前版本的PR_TOOL。其中的硬件模块在hardware, circuit部分，解析模块在parse部分，一些工具函数在global，顶层模块在app。这些模块里面可能会有一些接口在第5步可以用到

1. 选定测例

在test/module_test/test_writer下按编号顺序选一个测例目录进行测试。

2. 构造金标准输入文件

使用 `json2txt.cc` 工具把 /path/to/testcase 下的所有json格式构成的输入整体转换为txt格式，记为 connections.txt

3. 金标准工具运行

参考`/Users/jiaheng/FDU_files/Tao_group/PR_tool/jsy_version_tool/xinzhai_sourcecode1_new`下的AGENTS.md文件了解金标准工具的使用方法。使用金标准工具，读取connections.txt，生成`regnamecontrolbits_4part/`与`net_path_info.txt`

4. 路径转换

把`net_path_info.txt`当中的路径按照新、旧坐标体系转换为新坐标体系下的路径信息，可以使用`test/module_test/test_writer/trans_path_old2new.py`文件。得到的文件记为 net_path_info_new.txt

5. 编写test_writer.cc文件，基于新工具的数据结构构造一模一样的路径

在`test/module_test/test_writer/test_writer.cc`当中，使用新工具的相关数据结构与方法，把新路径信息一模一样的设置到新工具的硬件结构当中，并利用parse的输出功能写好输出部分。相关的API可以参考 source/AGENTS.md 当中对于项目的表述进行学习，然后自行选择需要使用的API。只要能够保持PR_TOOL工具逻辑，完成路径设置、写出 `regnamecontrolbit_4part/` 四个文件就可以。本步骤是一次性开发的工作，对于同一份路径信息，只需要写一次，但是写完之后需要开一个独立的子AGENT评估这个文件写的是否正确

6. 生成新工具的四个 REG 文件（全量，不加 `-s`）

在仓库根目录构建后，于 `output/` 运行（路径按测例调整）：

```bash
xmake build module_test
cd output
./module_test writer ../test/module_test/test_writer/<testcase> \
  <path_to_net_path_info_new.txt> ./check_run 0
```

**不要**加 `-s/--simplify-controlbits-file`：金标准对照需要全量输出。产物为 `./check_run/regnamecontrolbit_4part/{botleft_REG0,botright_REG1,topleft_REG2,topright_REG3}.txt`。

7. 直接对比四个文件与金标准

用 `compare_controlbits.py` 按寄存器名对齐比较（忽略地址列；hex 规范化）：

```bash
python3 ../test/module_test/test_writer/compare_controlbits.py \
  --golden-dir <golden>/regnamecontrolbit_4part \
  --split-dir ./check_run/regnamecontrolbit_4part
```

`--split-dir` 指向 PR Writer 的 `regnamecontrolbit_4part/`（四个 `*_REG*.txt`），不是旧的 `split_regs.py` 输出。脚本会先检查每个文件的寄存器数量是否与金标准一致；数量不一致时直接失败并提示禁用 `-s`。

金标准侧文件名仍为 `botleft_controlbit.txt` 等；PR 侧为 `botleft_REG0.txt` 等，由脚本内部映射。

8. 解读差异并迭代

每个对应文件中，每一个寄存器的值都应能按名称对齐且相同。

如果有不同，在确保前面的步骤都执行正确的情况下（坐标转换、json 与 txt 转换等可以认为是正确的），可以归因为：
- PR_TOOL的输出逻辑有问题，导致描述实际使用互连资源的寄存器输出的值不正确。如果是这个原因分析寄存器不同的地方到底在哪，再阅读PR_TOOL和金标准工具各自在输出逻辑上的写法，把实际不同的输出对应到输出逻辑，然后修改PR_TOOL的输出逻辑，回到第6步重新运行新工具。
- PR_TOOL和金标准工具在处理TOB寄存器的时候，针对TOB的`bump_to_hori_muxs``hori_to_vert_muxs``vert_to_track_muxs`当中未实际使用的mux资源的填充方式不一致（PR_TOOL的`randomly_map_remain_indexes()`）。如果是这个原因，可以允许不一样，但是必须严格检查是真的因为这个原因。COB / dly / drv 等非 mux-fill 差异视为意外，需要调查。

如果都一样，说明新工具的输出逻辑正确，可以结束本测例的测试。然后回到第1步，选择下一个测例，重复相应步骤测试。如果所有测例都测过了，那么整个测试过程结束

### T1：全量 + 简化成对写出校验

`test_writer` 支持一次写出全量树与简化树（不经过 `split_regs.py`）：

```bash
./module_test writer <config> <net_path_info_new.txt> ./check_run_full 0 \
  --simplified-output-dir ./check_run_simplified
python3 ../test/module_test/test_writer/verify_simplify_split.py \
  --full-dir ./check_run_full/regnamecontrolbit_4part \
  --simplified-dir ./check_run_simplified/regnamecontrolbit_4part
```

`verify_simplify_split.py` 按与 `register_defaults.hh` / `tools/split_regs.py` 相同的默认 hex 规则（严格字面相等）检查：全量中应省略的行不得出现在简化输出；非默认行 hex 必须一致；简化侧不得有全量没有的寄存器名。期望退出码 0。

9. 强制终止条件

如果连续debug运行了30分钟还没有修改出正确的输出逻辑，强制终止，并报告运行结果、简要总结已的修改内容


## 注意点

1. 只能在当前git分支上进行该测试，并且不允许使用git add / commit等指令提交更改

2. 不允许修改金标准工具

3. 需要仔细理解金标准工具和PR_TOOL的坐标体系差异，以及一个你哟功能坐标体系的差异，例如COB的端口变换关系、Track坐标等等。如果在执行上述流程的过程中，发现了除第8步不一样以外的错误，那么需要回头读金标准工具的代码，从坐标体系的角度理解差异。
