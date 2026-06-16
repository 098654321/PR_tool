## check-controlbits-file

本技能用于对PR_TOOL的输出模块进行debug。

## 目的

PR_TOOL接受json格式的输入，运行后会在output目录下生成一个controlbits_0.txt，这里面包含了一些寄存器的配置信息。但是目前怀疑这个配置信息的生成方法有错误，所以对输出这个文件的部分进行debug

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
另外还有`tools/register_map.json`描述了生成的配置文件包含哪四个文件，每个文件下有哪些寄存器，以及这些寄存器的地址。`tools/split_regs.py`可以根据这个json map把PR_TOOL生成的controlbits_0.txt转换为json map指定的四个txt文件。这样转换之后的四个文件格式与旧工具`regnamecontrolbits_4part/`中的四个文件格式一样。
`test/module_test/test_writer/trans_path_old2new.py`可以把金标准工具生成的路径信息转换为PR_TOOL新坐标体系下的路径信息

 
## 工作流程

0. 准备PR_TOOL

准备好当前版本的PR_TOOL。其中的硬件模块在hardware, circuit部分，解析模块在parse部分，一些工具函数在global，顶层模块在app。这些模块里面可能会有一些接口在第5步可以用到

1. 选定测例

在test/module_test/test_writer下选定test1测例目录进行测试。

2. 构造金标准输入文件

使用 `json2txt.cc` 工具把 /path/to/testcase 下的所有json格式构成的输入整体转换为txt格式，记为 connections.txt

3. 金标准工具运行

参考`/Users/jiaheng/FDU_files/Tao_group/PR_tool/jsy_version_tool/xinzhai_sourcecode1_new`下的AGENTS.md文件了解金标准工具的使用方法。使用金标准工具，读取connections.txt，生成`regnamecontrolbits_4part/`与`net_path_info.txt`

4. 路径转换

把`net_path_info.txt`当中的路径按照新、旧坐标体系转换为新坐标体系下的路径信息，可以使用`test/module_test/test_writer/trans_path_old2new.py`文件。得到的文件记为 net_path_info_new.txt

5. 编写test_writer.cc文件，基于新工具的数据结构构造一模一样的路径

在`test/module_test/test_writer/test_writer.cc`当中，使用新工具的相关数据结构与方法，把新路径信息一模一样的设置到新工具的硬件结构当中，并利用parse的输出功能写好输出部分。相关的API可以参考 source/AGENTS.md 当中对于项目的表述进行学习，然后自行选择需要使用的API。只要能够保持PR_TOOL工具逻辑，完成路径设置、输出controlbits_0.txt文件就可以。本步骤是一次性开发的工作，对于同一份路径信息，只需要写一次，但是写完之后需要开一个独立的子AGENT评估这个文件写的是否正确

6. 生成新工具的controlbits_0.txt文件

运行`test/module_test/test_writer/test_writer.cc`，得到新工具的输出

7. 拆解新工具输出

使用register_map.json + split_regs.py把controlbits_0.txt拆成四个文件，记为`split_4files`

8. 对比新工具和金标准工具的输出

对比`split_4files`和`regnamecontrolbit_4part/`下的每个文件。每个文件当中，每一个寄存器的值都应该能够对应上且相同。（按照寄存器名称对齐，对十六进制值做规范化，文件名称目前已经对应上了）
如果有不同，在确保前面的步骤都执行正确的情况下（前面那些环节涉及的坐标转换、json与txt的转换、split等环节可以认为是正确的），可以归因为PR_TOOL的输出逻辑有问题。分析不同的地方到底在哪，再阅读PR_TOOL和金标准工具各自在输出逻辑上的写法，把实际不同的输出对应到输出逻辑，然后修改PR_TOOL的输出逻辑，回到第6步重新运行新工具。
如果都一样，说明新工具的输出逻辑正确，可以结束

9. 强制终止条件

如果连续debug运行了40分钟还没有修改出正确的输出逻辑，强制终止，并报告运行结果、简要总结已的修改内容


## 注意点

1. 只能在当前git分支上进行该测试，并且不允许使用git add / commit等指令提交更改

2. 不允许修改金标准工具

