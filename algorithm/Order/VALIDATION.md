# 本地验证记录

日期：2026-10-23。环境：macOS系统C++编译器，C++20，`-O2 -DNDEBUG`。
完整四测例16次正式批次尚未运行；Linux环境需在目标服务器验证。

## 已执行命令

```bash
python3 algorithm/Order/run_experiments.py --smoke --jobs 4 --timeout 30
python3 algorithm/Order/run_experiments.py --cases case5 --jobs 4 --timeout 30

# 故意设置极短超时，用于验证失败隔离；预期退出码为1。
python3 algorithm/Order/run_experiments.py --cases case1 --timeout 0.0001 --jobs 4
```

每次构建后自动执行真实C++类测试：八种Net构造函数 × 四种方案，验证基础优先级、原有端口数偏置、SyncNet外层不加偏置而子成员加偏置的行为，以及新2、新3的关键顺序关系。两个COB宽度均通过。

case1冒烟5/5通过；两次current的汇总、实际排序、同步成员、路径坐标和连接器、布局一致。case1有3个顶层SyncNet，四方案均布通，总线长165；此测例没有其他顶层类别可供类别间调序。

case5四方案均布通66个顶层Net：

| 指标 | current | new1 | new2 | new3 |
| --- | ---: | ---: | ---: | ---: |
| 总线长 | 1117 | 1118 | 1117 | 1117 |
| 实际顺序与current相比 | 相同 | 不同 | 相同 | 不同 |

case5顶层类别为SyncNet 16、TrackToBumpsNet 16、BumpToBumpNet 32、TracksToBumpsNet 2。其余类别缺失，因此新2的赋值调整未改变本例实际顺序。这是类别覆盖限制，不代表调序后一定没有效果。

另外构建宽度13并运行current：case7布通66/66，总线长1479；case12重复两次，均布通30/30，总线长2839。case12两次汇总、排序、同步成员和路径完全一致。

强制超时批次完成四次尝试，全部标记TIMEOUT、保留已有日志、报告不填正常线长，最终返回1；没有在第一次超时后停止后续组合。

## 其他检查

- 16个`case × scheme`组合唯一且完整，新2映射与更正后的要求一致。
- 源码插入点变动会被拒绝；实际日志中的class优先级及执行顺序会被校验。
- 不完整布线线长标注“未布通”，TIMEOUT/ERROR及未运行项不作为零值。
- 各方案布局文件一致；从其他工作目录运行`--plan`得到16次计划。
- Python语法、文件空白及修改范围检查通过，其他目录没有修改。

## 日志位置

- 最终冒烟：`logs/smoke-20261023T151913+0800-48a51a/`
- case5四方案：`logs/sweep-20261023T151527+0800-61f64a/`
- 宽度13与case12重复：`logs/width13-20261023T151645/`
- 超时隔离：`logs/sweep-20261023T151959+0800-5d92a5/`
- 计划、覆盖及报告检查：`logs/checks/validation.json`、`logs/checks/case5-coverage.md`

`logs/`和`.build/`按本目录`.gitignore`排除，上传代码时可不包含。各次构建指纹见对应manifest或validation文件。正式运行会在独立目录保存服务器结果，并更新`experiments.md`。
