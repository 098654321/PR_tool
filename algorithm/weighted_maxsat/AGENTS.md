# weighted_maxsat

## 目的

本目录实现 ExactSAT 的直接 Weighted Partial MaxSAT 对比基线：在相同硬件和物理硬约束下，优先完成更多 source--sink pair，再最小化 track/bump 使用数。它只布线；不布局、不写回硬件、不输出 controlbits 或配置文件。

## 实现边界

- 输入、net 归一化、统一图、PNnet 虚拟源、D/A 状态裁剪和全部物理硬约束复用 `algorithm/test_ILP/`。
- 每个 pair 的搜索域固定为初始 bbox 外扩 1 格、距离 `[d_min,d_min+10]`；不使用 UNSAT core、`assume()` 或 feedback expansion。
- 同步 bus 原样复用逐距离硬等价，因此物理上仍是全 bus 完成或不完成。
- `q_p <=> OR D(sink,d)` 是硬关系而非硬需求；软子句 `10000 q_p` 奖励完成。
- `U_(net,node)` 只覆盖 track/bump，软子句 `-U` 权重为 1。物理源的 `U` 由完成的 `q` 激活，不能由固定根 `D(s,s,0)` 激活。
- `W_R=10000`；若最终 wirelength >= 10000，`debug.log` 会提示不再保证严格字典序。

## 目录

```text
weighted_maxsat/
├── main.cc                 # 解析、日志、WCNF 生命周期、运行后端
├── wmaxsat_cli.hh/.cc      # CLI: config [-v] [-o DIR] [--solver PATH] [--keep-wcnf]
├── wmaxsat_router.hh/.cc   # 编码、WCNF、EvalMaxSAT 调用、路径回溯
├── spec.md                 # 方法与实验口径
└── test/
    ├── unit_main.cc        # CLI / WCNF 构造单测
    └── integration_main.cc # parser -> graph -> encoding -> WCNF
```

## 关键接口

- `build_wmaxsat_encoding(interposer, basedie)`: 生成固定域的硬 CNF、q/U 软子句和变量映射。
- `write_wcnf(encoding, path)`: 用 `TOP=sum(soft)+1` 输出 Weighted Partial WCNF。
- `run_evalmaxsat(solver, wcnf, vars)`: 执行外部 EvalMaxSAT，解析 `s/o/v` 输出。
- `log_wmaxsat_solution(...)`: 默认（即使没有 `-v`）打印每条完成 pair 的回溯路径及总 wirelength 到控制台和 `debug.log`。

为让 WMaxSAT 不生成 SAT feedback 用的 alpha，`test_ILP/sat/unified_sat_encoder` 新增末参 `create_alpha_vars`，默认 `true`；本目录传 `false`，不改变 `test_ILP` 既有行为。`CadicalSession::clauses()` 用于导出已验证的硬 CNF 到 WCNF；仅 WMaxSAT 启用 `capture_clauses`，避免改变普通 `test_ILP` 的内存占用。

## 构建与测试

在 `PR_tool/` 根目录执行：

```bash
xmake f --cadical=y
xmake build weighted_maxsat
xmake build weighted_maxsat_unit
xmake build weighted_maxsat_integration
xmake run weighted_maxsat_unit
xmake run weighted_maxsat_integration
```

macOS 若原 `third_party/cadical/build` 是 Linux 产物，可在 `third_party/cadical/build-macos/` 中执行 `../configure` 和 `make -j4 libcadical.a`；xmake 会优先使用该本机目录。

EvalMaxSAT 由用户安装在 `third_party/EvalMaxSAT/build/EvalMaxSAT_bin` 后运行：

```bash
xmake run weighted_maxsat test/module_test/test_function/testlength/testiosimple -o output/wmaxsat
```

没有该可执行文件时程序会明确失败；不会退化为普通 SAT。加 `--keep-wcnf` 可保留输出目录中的 `wmaxsat_instance.wcnf` 以便复现。
