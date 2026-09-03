# GUI Flow

现行界面以 `dev.gui` / `source/widget/` 为准。Agent 约束与阶段机见 `source/widget/AGENTS.md`；方法学见 `docs/superpowers/specs/2026-08-11-chiplet_schematic_GUI_优化与设计方法学/`。下文截图来自早期左侧 Toolbar 版本，**只用来辨认四个视图**，不要按图上的图标墙去实现。

启动：`xmake run PR_tool -g`。先弹出入口对话框：空白工程，或选一个 config 文件夹 Load。空白时创建空的 `Interposer` 与 `BaseDie`——原理图为空，布局只有空 TOB，2D/3D 在布线成功前锁定。

## 顶栏与阶段

菜单只保留 **File** / **View**（+ 弱 Settings）。**没有**独立左侧 Toolbar，也没有把 Schematic/Layout/2D/3D 做成顶级菜单。

顶栏同一层：

- 分段切换：`Schematic | Layout | 2D | 3D`（`QPushButton`）。2D/3D 在 Route 成功前 disabled，并说明原因。
- **Place**：自动布局。成功后阶段变为 Placed，该按钮文案变成 **Edit Design**。
- **Route**：按当前 Layout 布线。成功后进入 Results，解锁 2D/3D，启用 Export。
- Export（File → Export Controlbits）：仅 Results 可用。

| 阶段 | Schematic | Layout | 2D / 3D | 主操作 |
|------|-----------|--------|---------|--------|
| Design | 可编辑 | 可编辑 | 锁定 | Place、Route |
| Placed | 仍可编辑（拖动不改真实放置） | 只读 | 锁定 | Edit Design 撤销 Place；可 Route |
| Results | 只读回看 | 只读 | 解锁 | Edit Design 撤销 Place+Route |

Route 失败停留在原阶段，不会当成成功去锁设计或打开 Export。Edit Design 会清掉布线结果（以及相应的放置快照），不是只改状态栏。Place/Results 下不能 Load 新 config，需先 Edit Design。

状态栏常显 `Stage: Design | Placed | Results`，以及坐标 / 缩放等。View 菜单：显示/隐藏 Navi 与 Inspector、Fit in View（Ctrl+0）、Reset Zoom。

可以从空白绘制再 Place/Route，也可以 Load 后修改。

![image-20250114210540138](./pics/flow/entry.png)

## 原理图

![image-20250114221504617](./pics/flow/schematic.png)

左侧是 **Navigator**（可折叠），不是旧的按钮墙：

- **Palette**：`+` 类型芯片（CPU/MEM/AI、Export、VDD/GND），用于往画布添加元件
- **网可见性**：两列复选框（Signal / Bus / Power / Ground / External Net），默认全开；取消勾选则该类连线不画
- **Search**：过滤树节点名，不搜 Palette
- **树**：单击 = 选中并在画布高亮（不自动 zoom）；需要靠近时用 Locate / 双击 Pin Map

中间 canvas 是主工作面：Header/Body 节点、语义缩放（远看模块与束，近看 pin）、Focus+Context（相关变强，其余变弱但仍在）、直角走线。即使左右栏收起，也应能读懂拓扑。

右侧 **Inspector**：选中对象的属性 / 连通性 / Pin Map。空选时提示选一个对象。

## 布局

![image-20250114221514190](./pics/flow/layout.png)

已有 TopDie Instance 与 TOB 之间放置或互换（实例不应没有 TOB）。右侧信息：实例数量、位置、估计线长。不在此编辑网。

原理图改动会刷新布局显示；布局改 TOB 后也会 reload 原理图放置。Place 成功后本页只读。

## 2D 视图

![image-20250114221837679](./pics/flow/view2d.png)

仅 Route 成功后可进。展示平面布线；Load 新设计后必须与 Schematic/Layout 一起 reload。部分 COB/TOB 可进详细视图。

## 3D 视图

![image-20250114221918998](./pics/flow/view3d.png)

仅 Route 成功后可进。展示立体布线；点击物体看详情。同样，Load 后必须 reload。
