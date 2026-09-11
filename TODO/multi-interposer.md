## 多中介层布局布线扩展（2026-09-03）

目标：将系统扩展为 `m` 行 `n` 列、共 `m*n` 块结构相同的中介层，同时保持现有单块输入和路由功能。建议以运行时参数化的全局路由 fabric 实现：COB/Track 使用全局坐标搜索，TOB 与寄存器保留中介层坐标和块内坐标，以避免物理映射与寄存器命名冲突。

以下逐项撰写需求规格文档，每添加一项内容，都需要注意和前面已有部分的联系与兼容

### 配置文件与解析模块改动

现有输入由 `config.json`、`interposer.json`、`topdies.json`、`topdie_insts.json`、`external_ports.json`、`connections.json`、`register_adder.json`、`01_ports.json` 组成。多中介层格式将 `topdie_insts.json`、`external_ports.json`、`01_ports.json` 合并到 `interposer.json`；即使阵列规模为 `1x1`，也必须使用下述新格式，不保留旧格式回退。

#### 保持不变的文件

- `topdies.json`：芯粒类型与 `pin_map` 格式不变。
- `connections.json`：连接格式不变。所有 `topdie_insts` 实例名和 `external_ports` 端口名必须在整个 m×n 系统内全局唯一，因此连接端点仍可直接使用现有名称。

#### config.json

由于三个独立文件已合并，`config.json` 必须删除 `topdie_insts`、`external_ports`、`ports_01` 三个路径；其余字段及含义不变：

```json
{
  "interposer": "interposer.json",
  "topdies": "topdies.json",
  "connections": "connections.json",
  "reigster_adder": "register_adder.json"
}
```

#### interposer.json

阵列属性只定义一次，所有 tile 共用同一组单块结构参数；每个 tile 描述自身的局部实例、端口和 padctrl 端口位置。下例为有效 JSON，数值仅作示例：

```json
{
  "properties": {
    "interposer_array_rows": 1,
    "interposer_array_cols": 1,
    "interposer_cob_rows": 9,
    "interposer_cob_cols": 12,
    "latency_inner_track": 1.0,
    "latency_inter_track": 1.5
  },
  "tiles": [
    {
      "coord": { "row": 0, "col": 0 },
      "topdie_insts": {
        "Topdie_inst_4_9": {
          "topdie": "cpu",
          "coord": { "row": 2, "col": 3 }
        }
      },
      "external_ports": {},
      "ports_01": {
        "pose": {},
        "nege": {}
      },
      "padctrl": [
        {
          "cob_row": 8,
          "cob_col": 3,
          "direction": "H",
          "index": 71
        }
      ]
    }
  ]
}
```

约束如下：

- `interposer_array_rows`、`interposer_array_cols`、`interposer_cob_rows`、`interposer_cob_cols` 均为正整数；`tiles` 必须恰有 `rows*cols` 个元素，tile 坐标唯一且完整覆盖 `[0, rows) × [0, cols)`。
- `latency_inner_track` 表示单块内部一个 Channel track segment 的延迟，`latency_inter_track` 表示跨相邻中介层 seam Channel track segment 的延迟，单位均为 ns，且必须满足 `latency_inter_track > latency_inner_track > 0`。
- `topdie_insts.*.coord`、`external_ports.*.coord`、`ports_01` 内的 Track 坐标都是**块内局部坐标**；Reader 负责转换为全局资源图坐标。
- `external_ports` 与 `ports_01` 的端口只能位于整个拼接基板的外边界。位于 tile 内侧拼接边的坐标是 seam Channel，必须拒绝作为外部端口或 0/1 端口。
- 单块 TOB 阵列仍为当前硬件定义的 `4x4`；因此 `topdie_insts.*.coord.row`、`col` 均须位于 `[0, 4)`。
- `tiles[].padctrl` 是该 tile 的 padctrl 端口位置真源，替代现有 `is_padctrl_port()` 中写死的坐标数组。每个条目以块内局部 `cob_row`、`cob_col`、`direction`、`index` 唯一定位一个 padctrl port；`direction` 取 `H` 或 `V`，`index` 必须位于硬件允许的 track index 范围，重复条目和越界坐标均为配置错误。
- Reader 为每个 `Interposer` 构建 `LocalTrackCoord → PadCtrlPortInfo` 查询索引；`Interposer::is_padctrl_port(local_track)` 和 `Substrate::is_padctrl_port(track_address)` 均查询此索引，不再依赖固定坐标。只有同时位于 `Substrate` 外边界的 padctrl port 才能作为外部端口使用；内部 tile 边缘上的 padctrl 信息保留为硬件属性，但不能暴露为外部端口。
- 当前四个定位字段足以完成 padctrl port 的位置校验与查询。若后续需要完全移除 `XinzhaiRegister` 对具体控制寄存器组的硬编码，则每项还需增加如 `register_group` 的字段，或者在文档中定义由位置到寄存器组的无歧义推导规则。

#### register_adder.json

每块中介层使用**独立的寄存器地址空间**，因此为每个 tile 保存一份原有四文件映射。使用数组和显式坐标，而不是 `"(0, 1)"` 这类需要额外字符串解析的对象键：

```json
{
  "tiles": [
    {
      "coord": { "row": 0, "col": 0 },
      "register_map": {
        "botleft_REG0.txt": {
          "cob_0_0_right_sel_0": "32'h90400000"
        },
        "botright_REG1.txt": {},
        "topleft_REG2.txt": {},
        "topright_REG3.txt": {}
      }
    }
  ]
}
```

每个 tile 的 `register_map` 必须完整包含 `botleft_REG0.txt`、`botright_REG1.txt`、`topleft_REG2.txt`、`topright_REG3.txt`。其中地址和寄存器名均是 tile 内局部命名；Writer 后续按 tile 分别写出四个寄存器文件，避免跨 tile 地址或名称冲突。

### hardware部分的硬件资源设计

#### 总体模型

保留现有 `source/hardware/interposer.hh/.cc` 中的 `Interposer`，使其继续表示**一块物理中介层**；在其内部将 COB/TOB 尺寸、`TOB_COORD_MAP` 和边界判断从编译期单块常量改为运行时实例属性。每个 `Interposer` 独立保存本块的 COB、TOB、块内 Track 和本地寄存器状态。

新增基板级 `Substrate`，表示 m×n 块 `Interposer` 组成的整体，并拥有 tile 网格、拼接 Channel、整体边界和全局资源查询能力：

```text
Substrate
├── TileCoord(row, col) -> Interposer
│   ├── 局部 COB / TOB
│   └── 块内 Track
└── SeamChannel
    └── 连接两个相邻 tile 的边缘 COB
```

`Substrate` 是后续布局、布线、全局边界判断和跨中介层延迟计算所面对的硬件入口；`Interposer` 不再承担整个 m×n 系统的资源图职责。

#### 对象归属、生命周期与父层级查询

对象的所有权必须单向向下：`Substrate` 唯一拥有各个 `Interposer` 和所有 `SeamChannel`；每个 `Interposer` 唯一拥有本块的 COB、TOB 和块内 Track。跨中介层的 `SeamTrack` 由所属 `SeamChannel` 拥有，而 `SeamChannel` 由 `Substrate` 拥有。

```text
Substrate (unique owner)
├── Interposer (unique owner of local COB / TOB / Track)
└── SeamChannel (unique owner of SeamTrack)
```

COB、TOB 与块内 Track 在构造时接收其 `Interposer&`，并保存 `const Interposer* const` 形式的私有父指针；该指针只能在构造函数中初始化，后续不能改指向，也不能经由该指针修改父对象。`Interposer` 以同样方式保存 `const Substrate* const` 与自身 `TileCoord`。`SeamTrack` 保存 `const SeamChannel* const`，`SeamChannel` 保存 `const Substrate* const`。这些均为非拥有观察指针，不使用 `shared_ptr`/`weak_ptr`，以保持所有权唯一、避免循环引用和引用计数开销。

`Substrate`、`Interposer` 和其子硬件对象均不可复制；由 `std::unique_ptr` 管理的对象地址在其生命周期内保持稳定，且父对象的生命周期必须覆盖全部子对象。

不公开 `track.interposer()`、`cob.interposer()`、`tob.interposer()` 这类返回完整父对象的访问器。子对象只公开所需的窄查询接口，例如：

```cpp
auto Track::local_coord() const -> const LocalTrackCoord&;
auto Track::tile_coord() const -> TileCoord;
auto Track::address() const -> TrackAddress;
auto Track::global_position() const -> PhysicalCoord;
auto Track::is_on_outer_boundary() const -> bool;
```

因此，任意给定一个块内 `Track` 都可通过 `tile_coord()` 或 `address()` 得到其所在 `Interposer` 的 tile 身份，而无需向调用方暴露或允许其任意修改父层级对象。COB、TOB 采用相同原则；只有 `Substrate`、`Interposer` 及受控的资源管理代码可在内部使用父指针。

#### 坐标与资源标识

采用分层坐标，而不是将所有 COB、TOB 的全局坐标作为主存储键。COB、TOB 和块内 Track 均由“tile 坐标 + 块内局部坐标”唯一确定：

```cpp
struct TileCoord {
    std::i64 row;
    std::i64 col;
};

struct CobAddress {
    TileCoord tile;
    LocalCOBCoord local;
};

struct TobAddress {
    TileCoord tile;
    LocalTOBCoord local;
};

struct TrackAddress {
    TileCoord tile;
    LocalTrackCoord local;
};
```

其中 `CobAddress`、`TobAddress`、`TrackAddress` 是资源的层级地址，而不是新增硬件对象；例如 `CobAddress` 表示“第 `(tile.row, tile.col)` 块中介层内、局部坐标为 `local` 的 COB”。为避免 `COBCoord`、`TOBCoord` 和普通 `Coord` 在类型系统中互相混用，逐步取消现有 `using COBCoord = Coord`、`using TOBCoord = Coord`，改为具有独立类型的局部坐标结构。

全局二维位置仍可由 `Substrate` 按需计算，用于 GUI、距离/延迟估计和 SAT scope；它不是 COB、TOB 或块内 Track 的主身份。

#### 跨中介层 Channel

`SeamChannel` 直接描述两个相邻 tile 上、彼此正对的边缘 COB；TOB 不作为 seam 端点，因为物理上跨中介层连接发生在 COB 边缘，TOB 仍经各自 tile 内的 Channel 接入 COB。

```cpp
struct SeamEndpoint {
    CobAddress cob;
    COBDirection side;
};

struct SeamChannel {
    SeamEndpoint first;
    SeamEndpoint second;
    std::Vector<SeamTrack> tracks;
    double latency_ns;
};
```

每条 seam 的两个端点必须位于相邻 tile 且方向相对；`tracks` 中每个 `SeamTrack` 对应一个物理 track index，具有唯一的占用状态和延迟。这样，同一条跨中介层物理通道不会被左右两个 `Interposer` 重复创建、重复占用或重复计数。

#### Substrate 的初始接口

以下接口是第一批必要接口，不构成接口上限；后续可随算法和 GUI 的需要增加：

- `tile(TileCoord)`：取得指定的 `Interposer`。
- `get_cob(CobAddress)`、`get_tob(TobAddress)`、`get_track(TrackAddress)`：按层级地址取得块内资源。
- `adjacent_resources(...)`：统一返回块内 COB 连通边与跨 tile seam 连通边。
- `is_outer_boundary_port(TrackAddress)`：判断端口是否位于整个拼接基板外边界。
- `is_padctrl_port(TrackAddress)`：查询 tile 配置中的 padctrl 端口信息。
- `global_position(...)`：按需生成资源的全局二维位置。
- `reset_regs()`、`for_each_tile()`：按 tile 重置资源、遍历并写出控制位。

#### 除数据结构外需要同步处理的问题

- 现有 `Interposer::adjacent_tracks()` 只返回 `COBConnector`。跨 tile seam 没有 COB 寄存器开关，需引入能同时表示 `COBConnector` 与 `SeamChannel` 的统一路由边类型，并支持占用、释放、双向连通和延迟查询。
- `PathPackage` 与历史路径目前只记录 Track 和可选 `COBConnector`；必须记录经过的 seam track，才能正确回放、释放资源、统计延迟和输出控制位。
- `TOB_COORD_MAP`、`available_tracks()` 以及路径读回中通过 `/2`、`/3` 反推 TOB 的逻辑，必须改为由所属 `Interposer` 和局部地址查找。
- Maze、SAT/ILP、布局器、GUI 和 Writer 必须通过 `Substrate` 查询资源和边界，不再直接使用单块 `Interposer::COB_ARRAY_*` 或自行计算跨 tile 邻接关系。
- 外部端口/0-1 端口只能映射到 `Substrate` 的整体外边界；内部 tile 边缘必须映射为 seam。
- 以 `tiles[].padctrl` 构建每块 `Interposer` 的 padctrl 查询索引，替换 `is_padctrl_port()` 和 `XinzhaiRegister` 中的固定坐标。若 padctrl 还承担控制寄存器分组选择，需补充 `register_group` 或等价的无歧义规则。
- 寄存器地址空间按 tile 独立；Writer 必须逐 tile 读取本地 `register_map` 并分别输出四个寄存器文件。

| 模块 | 必须改动 |
| --- | --- |
| 放置 | `source/algo/placer/sa/saplacestrategy.cc`：动态读取 fabric 的 TOB 边界和空闲 TOB；移动/交换逻辑可复用。若芯粒限定在特定中介层，需在配置和 `is_changable` 中加入约束。 |
| Maze 布线 | 复用现有资源图搜索，但为 seam 增加更高延迟/代价；若延迟参与同步线约束，BFS 需改为加权搜索，并同步更新重布线和长度统计。 |
| SAT / ILP 布线 | `source/algo/router/sat_ilp/graph/unified_routing_graph.cc`、`common/hw_map.hh`、`scope/`：移除对 `Interposer::COB_ARRAY_*` 和固定 TOB 线性编号的依赖；为 seam arc 编码更高 delay，并控制随 `m*n` 增长的 SAT 规模。 |
| 路径持久化 | `source/circuit/path/pathpackage.cc`：禁止通过 Bump 物理坐标 `/2`、`/3` 反推 TOB，改用 `Interposer` 提供的坐标反查 API。 |
| Controlbits 输出 | `source/parse/writer/writer.cc`、`source/parse/writer/registers/*`：将单块循环、寄存器名和 `RegisterValue` 扩展到多 tile，避免 `cob_0_0_*` / `tob_0_0_*` 重名。建议每块输出 `tile_r_c/regnamecontrolbit_4part/`，块内保留原四文件与命名。 |
| GUI / 可视化 | `source/widget/layout/`、`view2d/`、`view3d/`：改造单块画布边界、COB/TOB 遍历和坐标换算，显示 tile 分界及跨 tile 路径。CLI 完成后实施。 |
| 测试与文档 | 新增 `1x2`、`2x1`、`2x2` 测例；保留 `case5` 的 `1x1` 回归与 controlbits golden。覆盖内部 seam 非外部 IO、跨 seam 连通、seam 延迟与多 tile 无重名输出；更新 `source/AGENTS.md`、`test/AGENTS.md`。 |
