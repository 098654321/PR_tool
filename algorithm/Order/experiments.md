# 布线顺序实验

运行目录：`logs/sweep-20261023T161741+0800-727acc`；随机种子：20261023。
源码/工具链指纹：`fbf9e37c0f34c1c3b830534e2091cf50fd81576feb0dbd0f0292ffba8997b55e`。

每种顺序、每个测例独立运行一次。固定原始布局，不执行SA；mode=0，非增量Maze布线。
只切换下表的基础优先级，小数值先布线。

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

new2采用用户更正后的bump优先顺序；BumpToTrackNet与TrackToBumpNet保持并列，均为4。
其他参数、端口数偏置、搜索规则及原有线长定义均保持一致。普通Net偏置为 `0.9*port_number/max_port_number`，数值升序，因此同档端口少者先。
保留SyncNet仅更新内部成员优先级、外层不加端口偏置的现状。同步组不拆开参与顶层调度；完全同分的顶层Net按UID排序。
TOB接入候选保留COB-unit使用量排序，并在使用量相同时按Track坐标打破平局。
COB阵列：case2为9×12；case13为9×13；case5为9×12；case7为9×13。

## 顶层Net类别覆盖

同步组成员不在此表拆开计数，避免误认为存在可独立调序的二端Net。

| case | SyncNet | TrackToBumpsNet | BumpToTracksNet | BumpToBumpsNet | BumpToTrackNet | TrackToBumpNet | BumpToBumpNet | TracksToBumpsNet |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| case2 | 4 | 0 | 0 | 0 | 1 | 0 | 0 | 0 |
| case13 | 16 | 6 | 0 | 0 | 0 | 101 | 189 | 2 |
| case5 | 16 | 16 | 0 | 0 | 0 | 0 | 32 | 2 |
| case7 | 16 | 16 | 0 | 0 | 0 | 0 | 32 | 2 |

某些测例缺少被调序的类别，修改优先级后实际顺序可能不变。下表通过排序后的UID序列哈希比较；相同表示本次没有真正改变顶层布线序列。

## 实际顺序与current的比较

| case | current | new2 | new3 |
| --- | ---: | ---: | ---: |
| case2 | 相同 | 相同 | 不同 |
| case13 | 相同 | 不同 | 不同 |
| case5 | 相同 | 相同 | 不同 |
| case7 | 相同 | 相同 | 不同 |

## 完成状态

| case | current | new2 | new3 |
| --- | ---: | ---: | ---: |
| case2 | 布通 | 布通 | 未布通 |
| case13 | 布通 | 未布通 | 未布通 |
| case5 | 布通 | 布通 | 布通 |
| case7 | 布通 | 布通 | 布通 |

## 失败顶层Net数量

| case | current | new2 | new3 |
| --- | ---: | ---: | ---: |
| case2 | 0 | 0 | 1 |
| case13 | 0 | 10 | 3 |
| case5 | 0 | 0 | 0 |
| case7 | 0 | 0 | 0 |

## 顶层Net布通率 %

| case | current | new2 | new3 |
| --- | ---: | ---: | ---: |
| case2 | 100 | 100 | 80 |
| case13 | 100 | 96.8153 | 99.0446 |
| case5 | 100 | 100 | 100 |
| case7 | 100 | 100 | 100 |

## 总线长

| case | current | new2 | new3 |
| --- | ---: | ---: | ---: |
| case2 | 460 | 460 | 323（未布通） |
| case13 | 3550 | 3466（未布通） | 3394（未布通） |
| case5 | 1117 | 1117 | 1117 |
| case7 | 1479 | 1479 | 1480 |

## 最长成员/普通Net线长

| case | current | new2 | new3 |
| --- | ---: | ---: | ---: |
| case2 | 12 | 12 | 15（未布通） |
| case13 | 308 | 308（未布通） | 299（未布通） |
| case5 | 26 | 26 | 26 |
| case7 | 182 | 182 | 184 |

## 布线耗时 s

| case | current | new2 | new3 |
| --- | ---: | ---: | ---: |
| case2 | 1.4587 | 1.4451 | 1.2928 |
| case13 | 10.3125 | 10.3920 | 10.3208 |
| case5 | 7.7333 | 7.7212 | 7.7633 |
| case7 | 9.5804 | 9.5773 | 9.6938 |

## 同步组等长检查失败数量

| case | current | new2 | new3 |
| --- | ---: | ---: | ---: |
| case2 | 0 | 0 | 0 |
| case13 | 0 | 0 | 0 |
| case5 | 0 | 0 | 0 |
| case7 | 0 | 0 | 0 |

## 统计口径与日志

一个SyncNet整体计为一个顶层Net；成员数与成员线长另外记录。SyncNet只有成员全部非零且等长、并且未抛出路由失败时才计为布通。
总线长直接累计现有Net::length()，沿用当前压缩/计数规则；失败运行可能包含残留部分路径，不能与完整布通结果直接比较线长优劣。
`routing_seconds`包含排序、资源检查、布线、路径导出与日志开销，不包含编译、读取配置、线网构建和布局。单次运行不提供统计显著性结论。
`results.csv`保存完整汇总；每次运行的`order.tsv`为实际排序，`nets.tsv`为逐Net结果，`sync_members.tsv`为同步成员结果，`paths.tsv`为坐标与连接器路径，`placement.tsv`为固定布局。
`debug.log`与`console.log`保留原生路由输出；超时/进程错误保留已有日志并继续其他组合。
每个进程的完整参数在`command.json`中；环境、输入哈希和编译信息见`manifest.json`。
