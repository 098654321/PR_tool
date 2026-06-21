# PR_tool

针对 [PR_toolmore](https://www.PR_toolmoore.com/) 设计的一款 chiplet interposer，所制作的布局布线工具。


## 项目分支

主要分支（以仓库当前状态为准）：

- `master`：主工程布局布线流程（命令模式框架）
- `dev.algo_ILP_MCF` / `dev.algo_SAT_MCF_latest` / `dev.algo_ILP_newMCF` ：SAT+MCF 算法实验分支（`algorithm/test_ILP/`）。其中dev.algo_ILP_MCF是算法主分支，分出了dev.algo_SAT_MCF_latest(目前最新的)和dev.algo_ILP_newMCF（调整SimpleMCF变量的建模语义，但是没有拆解多扇出net）
- `dev.incre_no_sharing` / `version_before_commands`：历史增量布线分支

增量布线逻辑见 `source/algo/router/incremental/`；`test/config/case17`、`case18` 为增量布线配置样例，但当前 `regression_test` 未纳入这两例。


## 项目结构

- [algorithm](./algorithm/)：算法开发与实验（含 `test_ILP` SAT+MCF 实验入口，见 `algorithm/test_ILP/AGENTS.md`）
- [document](./document/)：项目文档
- [resource](./resource/): GUI 资源
- [source](./source/)：项目源码
- [test](./test/)：项目模块测试
- [tools](./tools/)：工具程序




## 项目构建

本项目由 [xmake](https://github.com/xmake-io/xmake) 工具构建。

安装xmake：[xmake安装](https://xmake.io/mirror/zh-cn/guide/installation.html)

保证系统安装 xmake 以及支持 C++23 的编译器。可执行文件输出到 `./output/`。在根目录调用：

````bash
xmake build PR_tool
# 运行
./output/PR_tool <config_folder> [OPTIONS]
# 或
xmake run PR_tool <config_folder> [OPTIONS]
````



## 命令行参数

````bash
./output/PR_tool <input folder path> [OPTIONS]
````


Options：
- `-o, --output <OUTPUT_PATH>`：指定输出 controlbit 文件目录
- `-g, --gui`：使用 GUI 模式
- `-h, --help`：打印帮助信息
- `-V, --version`：打印版本信息
- `-v, --verbose`: 输出 Debug 信息
- `-p, --placement`：启用放置（模拟退火 `SAPlaceStrategy`）
- `-i, --incremental [mode]`：进入增量布线；省略 mode 时尝试所有 mode，指定正整数 mode 时仅跑该 mode
- `-c, --compare <mode>`：增量布线后与指定 mode 的 controlbits 比对（需与 `-i` 联用）



## 工具程序

tools 目录下保存一些实用的工具小程序：

- [cobmap](./tools/cobmap.cc)：计算 COB 端口信息的映射关系
- [view2d](./tools/view2d.cc)：对指定 config 进行布线，使用 2D 显示布线结果
- [view3d](./tools/view3d.cc)：对指定 config 进行布线，使用 3D 显示布线结果
- [parse_controlbits](./tools/parse_controlbits.cc)：解析 controlbits 文件

根据 xmake.lua 中的 target，执行 `xmake build <tool>` 构建、`xmake run <tool> [args]` 运行相应工具。除此，还提供了 count_lines.py 来计算源代码总行数。



## 测试

test 目录下保存项目的测试代码，分为 module_test 和 regression_test 两个部分。

### module_test

对各个模块进行单独测试。

编译：

```bash
xmake build module_test
```

运行：

````bash
xmake run module_test [module]
````

`module` 指定要测试的模块（见 `test_unit/` 下注册的 `cob`、`tob`、`router` 等），`all` 表示全部测试。

### regression_test

集成系统各个模块，用于回归测试。当前 `test/regression_test/test.cc` 启用 **13** 个 case（case 1–3、7–16）；case 4、6 在源码中注释掉。
- case 1-3 测试基本的功能 ：
    |case|说明|
    |:---:|:---:|
    |[case1](./test/config/case1)|仅测试同步线布线功能|
    |[case2](./test/config/case2)|测试同步线，并对同步线中的一个bump增加一根连到I/O的线，测试已布线bump的复用|
    |[case3](./test/config/case3)|仅测试非同步线布线功能|
- case 4-6（配置存在，但 case 4、6 当前未纳入回归）：
    |case|说明|
    |:---:|:---:|
    |[case4](./test/config/case4)|包含 VDD/GND|
    |[case5](./test/config/case5)|重复连接|
    |[case6](./test/config/case6)|更多的连接数量|
- case 7-16 使用构造的芯粒系统进行测试，系统中包含所有类型的线网与较多的线网数量 ：
    |case|说明|
    |:---:|:---:|
    |case 7-9|一个 cpu-ai-mem 芯粒系统|
    |case 10-12|一个 cpu 芯粒系统|
    |case 13-16|一个 AI core 芯粒系统|
- case 17-18：增量布线配置样例（`test/config/case17`、`case18`），不在 `regression_test` 中自动运行

编译：

```bash
xmake build regression_test
```

运行：

```bash
xmake run regression_test
```
