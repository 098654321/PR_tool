## check-controlbits-file

本技能用于对PR_TOOL的输出模块进行debug。

## 目的

PR_TOOL接受json格式的输入，运行后会在output目录下生成一个controlbits_0.txt，这里面包含了一些寄存器的配置信息。但是目前怀疑这个配置信息的生成方法有错误，所以对输出这个文件的部分进行debug

## 参考资料

1. 查bug的金标准工具

本工具的前身是 `/Users/jiaheng/FDU_files/Tao_group/PR_tool/jsy_version_tool/xinzhai_sourcecode1_new`，这个工具在运行后会输出 `regnamecontrolbits_4part/` 目录与一个 `net_path_info.txt`文件，其中目录包含4个controbit文件，而路径信息文件以旧工具的坐标体系为基准生成路径坐标信息。这个工具接受txt格式的输入

2. 旧工具坐标体系与PR_TOOL工具坐标体系

可以参参考文件`/Users/jiaheng/FDU_files/Tao_group/PR_tool/PR_tool/test/transform_format/txt&json_coord_relationship.txt`

3. PR_TOOL的内容

可以查看`source/AGENTS.md`文件了解PR_TOOL的项目规范与内容

4. 一些测例

在 `test/module_test/test_writer`下有一些测例，用于本类debug

5. 格式转换工具

在`test/transform_format`下有`json2txt.cc`与`txt2json.cc`，可以在json和txt之间转换测例的文件形式。其中txt代表了旧工具的输入，json代表PR_TOOL的输入。另外还有

 
## 工作流程

1. 选定测例

在test/module_test/test_writer下选定一个测例目录进行测试。此处的测例目录应由用户指定，并记为 /path/to/testcase

2. 构造金标准输入文件

使用 `json2txt.cc` 工具把 /path/to/testcase 下的json格式输入文件转换为txt格式，记为 connections.txt

3. 金标准工具运行

使用金标准工具，读取connections.txt，生成`regnamecontrolbits_4part/`与`net_path_info.txt`

4. 路径转换

把`net_path_info.txt`当中的路径按照新、旧坐标体系转换为新坐标体系下的路径信息，可以使用`test/module_test/test_writer/trans_path_old2new.py`文件。得到的文件记为 net_path_info_new.txt

5. 运行新工具

把新路径信息一模一样的设置到新工具的硬件结构当中，然后调用parse的输出功能，生成controlbits_0.txt文件

6. 拆解新工具输出



7. 对比新旧工具输出


