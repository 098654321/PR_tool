# 本地验证记录

实验保留current、new2、new3，命令行编号分别为0、2、3。默认case2/13/5/7，每组运行一次，共12组；case1冒烟为重复current两次，加上new2、new3，共4组。

```bash
python3 algorithm/Order/run_experiments.py --cases case2 case13 case5 case7 --seed 20261023 --jobs 4
```

随机种子固定为20261023，直接使用原始输入TOB坐标，不运行SA。原始`source/`和`test/config/`未修改。

已重新运行12/12组，没有超时或进程错误；9组全部布通，3组记录正常布线失败。新日志目录：`logs/sweep-20261023T161741+0800-727acc/`。完整结果见[experiments.md](experiments.md)。

此前的保留测量已整理为三策略记录，逐Net、同步成员、路径及布局文件保持原样。新运行的12组汇总、实际顺序、路径、同步成员和布局逐项核对这些记录，全部一致。12份布局也与原始输入完全一致，检查结果保存在新目录的`verification.json`。

- case2/new3失败1个同步组；case13/new2失败10个Net、new3失败3个Net。
- case5三个方案均布通，线长1117；case7/new3线长1479→1480。
- 顶层同分按UID排序，TOB接入候选的平局规则及原有端口数偏置保持一致。
- 两种COB宽度的真实Net构造及三策略赋值检查通过；不支持的命令行策略编号会被拒绝。

本地日志和构建缓存按本目录`.gitignore`排除；后续完整运行会生成独立日志目录，并更新`experiments.md`。
