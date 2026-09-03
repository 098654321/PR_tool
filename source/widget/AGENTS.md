# PR_tool /source/widget 工程指南（面向 AI Agent）

本文件是 Qt GUI 的导航入口。算法、电路/硬件模型与 CLI 以 `source/AGENTS.md` 为准；测例与 `gui_test` 布局以 `test/AGENTS.md` 为准；设计复盘见 `docs/summaries/gui.md`。

入口：`app/gui/gui.cc`（Fusion + `ChromeTokens` + `:/qss/qss/app.qss`）→ `EntryDialog` → `widget::Window`。启动：`xmake run PR_tool -g`。

---

## 0. 工作原则

- 必须先读材料再改；改完评估是否维护本文件与 `source/AGENTS.md`
- 默认只改 `source/widget/**` 与 `source/app/gui/**`。禁止改 `source/algo/**`（可读 `failed_net_names` 等结果，不改算法契约）
- 碰 `window.h` / `window.cc`（阶段机）的任务必须串行，禁止并行子 agent
- 改动超过 100 行：根目录 `.plan/` 留记录，并开独立子 agent 审查
- **本文件不得超过 200 行**；控件细节以源码与规格为准
- 编译通过 ≠ 视觉完成：`code-done` 可本地标；`vis-done` 仅在对照已批准 HTML/截图并由人放行后
- GUI 文件改完后，项目 Hook（`.cursor/hooks.json`）会追问 `gui_test` 与工作原则 2–5

---

## 1. 目录

```text
widget/
  window.*              # 主窗、四视图调度、阶段旗标
  prthread.*            # PlaceThread / PRThread（后台 Place 与 Route）
  chrometokens.h        # 语义色；QSS 用 @accent 等占位，applyToQss 长名优先
  schematictypography.h # 画布字号（pin < Header）
  frame/                # EntryDialog、Export、Place/Route 忙窗、GraphicsView
  schematic/            # 原理图：Navi/Palette、canvas、Inspector、Mini Map
  layout/               # TopDieInst ↔ TOB 放置
  view2d/  view3d/      # 布线结果；Load/Route 后必须 reload
  setting/              # 设置页（弱）
```

---

## 2. 四视图

| 视图 | 何时可用 | 职责 |
|------|----------|------|
| Schematic | 加载后 | 编辑/阅读拓扑。Navi（Palette + 网过滤器 + Search + 树）、canvas、Inspector。单击树/Pin Map = 选中+高亮（不抢镜头）；双击/Locate 才 zoom |
| Layout | 加载后 | 已有实例放到 TOB / 互换。不编辑网。Place 后只读 |
| 2D | **Route 成功后** | 平面布线；未解锁时 disabled，tooltip 说明原因 |
| 3D | **Route 成功后** | 立体布线；点击对象看详情。headless 测试不切此页、不断言 OpenGL |

视图是顶栏 **分段控件**（`QPushButton`，勿用 macOS 会吞 QSS 的 `QToolButton`），与 View 菜单双向同步。不是顶级菜单项，不是连续滑块，没有独立左侧图标墙。Ctrl+1..4 切页；锁定页只 toast。

侧栏（Navi / Inspector）可折叠，优先最大化 canvas。View 菜单另有 Fit in View（Ctrl+0）/ Reset Zoom。

---

## 3. 现行阶段机（覆盖 08-11 第二章旧文案）

旗标只在 `Window`：`_placed`、`_finishPR`。不要在各 page 再分叉一套布尔量。

| 阶段 | 标志 | Schematic | Layout | 2D/3D | 顶栏 |
|------|------|-----------|--------|-------|------|
| Design | 皆 false | 可编 | 可编 | 锁 | **Place** + **Route**；Export 禁用 |
| Placed | `_placed` | **仍可编**（逻辑视图；拖动不改真实放置） | 只读 | 锁 | Place 槽变为 **Edit Design**（只撤销 Place）；Route 仍可用 |
| Results | `_finishPR` | 只读回看 | 只读 | 开 | Edit Design 撤销 Place+Route；Export 启用；Route 禁用 |

- **现行是 Place 与 Route 两个动作**；Place 槽在 Placed/Results 变成 Edit Design。
- Route 失败：停在原阶段，禁止走成功收尾（reload / 只读锁 / 开 Export / `_finishPR`）。
- **Edit Design 必须对称清理**：丢掉 routed path、寄存器占用、Net 指针后再恢复可编。早期 plan「只改阶段、不 wipe」已证伪。
- Load：凡展示设计或结果的视图都要 `reload()`（含 2D/3D）。Place/Results 下 Load 禁用，需先 Edit Design。
- 进度隐喻：Place = 真实 SA 目标采样折线；Route = 曼哈顿线网 + routed/total。

---

## 4. Chrome 与画布语义

- 产品类型：EDA工具，不是落地页。禁止大 CTA 留白、支付绿当主按钮、紫粉渐变、emoji 当锁图标。成功绿（`ChromeTokens::success`）只表示 Valid/Routed。
- 强调色唯一：`#0071E3`。改皮肤用角色 token，禁止每个控件自写一种蓝。改 QSS 后必须**整进程重启**；核对 qrc 路径 `:/qss/qss/app.qss`。
- Schematic（现行，覆盖部分旧章）：直角 Header/Body；类型+实例名始终可见；4px 类型色边、名称居中偏大、类型名在下（不要深色整条 Header）。连线可进入半透明 body。选中 = 加粗描边，**不要 glow**。最多一个 selected die。
- Pin LOD / Port Group / Bus 聚合 / Focus+Context / 电源轨：缩放改变「显示什么」。Filter ≠ Focus：Navi 两列 `QCheckBox`（Signal/Bus/Power/Ground/External Net），默认全开；勾掉 = **不画**；弱化 = 仍占空间。Search 搜的是树，不是 Palette。
- 空间稳定：高亮/过滤/聚合不重排坐标。跨 Schematic↔Layout↔2D↔3D 选择同步：**不做**。
- 禁止在 `mousePressEvent` 等事件里 `delete this`。

---

## 5. Standing constraints（任务默认粘贴）

后续任何 GUI 任务，默认复制下列约束。被本任务明确推翻的，写在 Non-goals 对面。

```text
栈：Qt Widgets + Fusion；语义色只走 ChromeTokens；不要引入 Web UI 栈。
范围：默认只改 source/widget/**, source/app/gui/**，resource/** 和 test/module_test/test_gui/**。对于默认范围以外的内容，
      在评估后认为必须要改的情况下，请求用户权限，并简要概括想改什么，当用户同意之后才可以改。
      对于核心算法，可以读算法结果（failed_net_names、reload API），但不要为 GUI 改算法契约。
分支：留在当前分支（通常 dev.gui）。不 push。是否 commit 看任务预设。
画布优先：chrome 不得再抢面积；侧栏可折叠。
视图：Schematic | Layout | 2D | 3D 是分段切换，不是菜单、不是滑块。
阶段：旗标只在 Window（_placed / _finishPR）。
      Design：Sch/Layout 可编，2D/3D 锁。
      Place 后：Layout 锁，Schematic 仍可编，2D/3D 仍锁。
      Route 成功：Sch/Layout 只读，2D/3D 开，Export 开。
      Route 失败：停在原阶段，禁止走成功收尾。
      Edit Design：按深度撤销（仅 Place / Place+Route），并清理 routed 状态。
Load：凡展示设计或结果的视图都必须 reload（含 2D/3D）。
识别 ≠ 定位：Navi/Pin Map 单击 = 选中+高亮；双击或 Locate 才 zoom。
Filter ≠ Focus：勾掉 = 不画；弱化 = 仍在。默认过滤器全开。
选中：加粗描边，不要 glow。最多一个 selected die。
Header：直角矩形；类型+实例名始终可见。
空间稳定：高亮/过滤/聚合不重排坐标。
跨视图选择同步：不做。
平台：分段/CTA 用 QPushButton；改 QSS 后必须重启进程；核对 qrc 路径。
并行：不要并行改 window.h/cc。
测试：改状态机或主路径交互则补 gui_test；改 GUI 默认范围以以外则跑回归测试 [flow]。
视觉：和用户交流的时候，如果涉及视觉设计而非纯文字修改的内容，最好用HTML做一个效果展示，由用户选择、批准 HTML 的效果后再做实现；不得把 code-done 说成 vis-done。
```

---

## 6. 构建与测试

```bash
xmake build PR_tool
xmake run PR_tool -g
xmake build gui_test
xmake run gui_test          # QT_QPA_PLATFORM=offscreen；fixture=test/config/case5
```

`gui_test` 负责自动化的进行基本测试，每一次出现新bug、新内容都需要思考一下是否要更细gui_test。test分为三层：
- `component/` 控件契约，负责控件测试。如果是QT自带的、非常基础的控件，可以不测；如果是结构复杂，或者是自定义出来的控件，必须要测试其设计的功能
- `interaction/` 局部动作，针对涉及用户交互的单个动作、响应进行测试。该测试需要较高的覆盖率
- `workflow/` 工作流，这个测试需要先考虑本软件的主要用途，找到用户使用本软件最常出现的、最基本的工作流程。从打开软件开始，用户按照某种使用需求不断的按顺序与软件进行交互，得到最终结果，形成一个完整的工作流。这部分的测试需要保证能够考虑到的工作流都可以完整执行，得到预期结果
在gui_test尽量覆盖测试内容、提高测试自动化程度的前提下，如果还有无法测试的，则由开发者人力测试。

---

## 7. 规格索引

| 产物 | 路径 |
|------|------|
| 人读流程 | `document/gui/flow.md` |
| 复盘 / harness | `docs/summaries/gui.md` |

章节正文若与 **§3–§4 现行裁定** 冲突，以本文件与源码为准（再回写规格）。
