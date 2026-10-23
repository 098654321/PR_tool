# 布线顺序实验

本目录独立构建现有Maze布线器的实验副本，比较三组基础优先级。全部源码副本、编译产物、日志和结果均在本目录内，原始`source/`、`test/`及根构建配置不修改。

在仓库根目录运行，Python只使用标准库，C++需要支持C++20及`std::format`，Linux建议GCC 13或更新版本：

```bash
# case1冒烟：current重复两次，再运行new2/new3，共4次。
python3 algorithm/Order/run_experiments.py --smoke --cxx g++ --jobs 4

# 正式实验：3种顺序 × case2/13/5/7，各运行一次，共12次。
python3 algorithm/Order/run_experiments.py --cxx g++ --jobs 4

# 显式指定同一组测例，随机种子与此前一致。
python3 algorithm/Order/run_experiments.py --cases case2 case13 case5 case7 --seed 20261023 --cxx g++ --jobs 4

# 查看计划，只读操作，无编译及文件写入。
python3 algorithm/Order/run_experiments.py --plan

# 仅测试一个包含多种Net类别的测例。
python3 algorithm/Order/run_experiments.py --cases case5 --cxx g++ --jobs 4
```

不需要Qt、SAT求解器、Gurobi、Catch2或xmake。保留完整仓库的相对目录结构上传，包括当前`source/`、四个case输入和本目录；可不上传`.build/`和`logs/`。脚本也支持从其他工作目录调用。
`--jobs`仅控制编译并行度，布线实验始终串行。内存不足时降低编译并行度。

## 三组优先级

基础优先级越小越先布线。

| Net class | current | new2 | new3 |
| --- | ---: | ---: | ---: |
| SyncNet | 0 | 0 | 2 |
| TrackToBumpsNet | 1 | 2 | 3 |
| BumpToTracksNet | 1 | 2 | 3 |
| BumpToBumpsNet | 2 | 1 | 4 |
| BumpToTrackNet | 3 | 4 | 0 |
| TrackToBumpNet | 3 | 4 | 0 |
| BumpToBumpNet | 4 | 3 | 1 |
| TracksToBumpsNet | 5 | 5 | 5 |

new2采用更正后的bump优先顺序，未单列的BumpToTrackNet与TrackToBumpNet继续并列；new3把独立二端Net放在前面。同一基础档位的并列关系保留。

Python通过命令行传入方案编号0、2、3（依次对应current、new2、new3）；C++在读取配置、构建Net之前设置方案。构造函数通过`order_policy.hh`查询基础优先级，随后调用原有`route_nets()`的Sort → Resources → Route流程。无需每个方案重新编译。默认测例为case2/13/5/7，case2/5的COB宽度为12，case13/7为13，分别编译隔离可执行文件。case1仍用于冒烟，case12也可通过`--cases`单独选择。

## 控制条件与保持的行为

- 每次新建进程、读取原始case并重新构建Net，mode固定为0，非增量Maze布线，不运行SA或写出controlbits。
- 默认随机种子`20261023`；普通随机工具改为固定种子的mt19937_64。非增量默认流程没有执行Allocate命令，因此不会调用HK内的时间播种随机挑选函数。
- 普通Net保留`base + 0.9*port_number/max_port_number`并升序排列，即同档端口少者先。SyncNet只将偏置传给内部成员，外层不加偏置，保持当前源码行为。
- SyncNet整体参与顶层调度，内部成员不拆出来与独立二端Net混排；同步预布线、拉齐及失败处理保持现有实现。
- 顶层同分按UID打破平局；TOB接入候选仍按COB-unit使用量排序，同使用量按Track坐标打破平局。所有方案采用相同处理。基准是采用这些复现规则的现有优先级，不保证等于原CLI某次无固定平局规则的路径。
- 当前线长计数沿用现有`Net::length()`和`path_length()`，不引入SA目录之外的新线长定义或修改底层搜索算法。
- 冒烟对current重复两次，比较实际排序、路径坐标与连接器、同步成员结果、布局和无耗时的汇总；失败布线也可以是可复现的有效实验观测。

## 日志及结果

每次启动创建独立的`logs/smoke-时间-标识/`或`logs/sweep-时间-标识/`：

- `manifest.json`：种子、完整优先级表、输入哈希、源码/工具链指纹、平台、超时和完成状态。
- `runner.log`：批次进度、失败原因和每次布通数量、线长、耗时。
- `build-width12.log`、`build-width13.log`：编译/链接命令及诊断；`self-test-width*.log`记录八种真实Net构造函数在三种策略下的验证。
- `results.csv`：每组的顶层Net类别数量、实际顺序哈希、布通/失败数量、布通率、总/最长线长、同步组及成员统计、耗时。
- `tables.md`：三策略对照表。完整case2/13/5/7运行结束后同步更新本目录`experiments.md`；冒烟和部分case运行不会覆盖正式报告。

每次运行子目录包含：

- `command.json`：可执行文件与完整参数。
- `placement.tsv`：按实例名排序的固定TOB位置。
- `order.tsv`：实际顶层布线顺序，包含UID、名称、class、基础优先级、端口数及偏置后的实际优先级。
- `nets.tsv`：实际执行顺序、UID、class、成功/Retry状态、线长、耗时和同步成员最小/最大线长。
- `sync_members.tsv`：同步组每条成员的UID、class、最终线长、组完成与等长检查。
- `paths.tsv`：每条普通Net/同步成员的路径坐标、COB连接器和TOB接入信息，不写内存地址。
- `debug.log`、`console.log`：原生路由日志及实验开始/结束记录。
- `result.json`：经计数、排序和同步结果检查的汇总。

SyncNet整体计为一个顶层Net，成员另计。只有全部成员非零且等长、没有Retry时才计为布通。普通Net要求未Retry且线长非零。总线长累计原始Net长度，可能包含失败Net的残留部分路径；报告会将未全部布通的线长标注为“未布通”，不能因线长较小就判为较好结果。

汇总报告另列各case的顶层类别数量，以及实际顺序是否与current相同。case1只有同步组；case5没有BumpToBumpsNet等新2主要调序涉及的类别，所以修改新2优先级并不改变它的实际顺序。是否真正调序应结合此覆盖信息判断；不会为制造差异而替换输入或拆开同步组。

`routing_seconds`包括排序、资源检查、布线、原生日志和路径导出，不含编译、解析和线网构建。单Net计时包括其日志/路径导出。失败或超时没有完整总耗时时不会填入正常测量值。单次实验适合探索顺序影响，不提供统计显著性结论。

默认每次布线进程超时600秒，可用`--timeout 1800`等调整。正常的不可布通结果记录为有效测量并继续；超时、异常退出或统计不一致单独标记为TIMEOUT/ERROR，也继续后续组合。部分日志逐Net刷新，可用于定位最后尝试的Net。批次在有进程级错误时返回非零；仅有正常布线失败时仍返回0，需查看`all_routed`与`failed_nets`。

构建使用内容指纹缓存；插入点严格检查，当前源码变动使匹配失效时会停止，避免遗漏统计。除编译器使用系统临时目录外，脚本生成的文件都位于本目录。

删除旧`logs/`不影响原始输入或默认种子。再次运行会自动重建日志目录，并重新记录输入文件哈希、种子20261023以及每组的`placement.tsv`。此次默认测例调整仅改变实验集和汇总说明，C++布线代码、优先级表和候选排序规则保持不变。
