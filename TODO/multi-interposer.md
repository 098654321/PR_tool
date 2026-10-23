## 多中介层布局布线扩展（2026-09-03）

目标：将系统扩展为 `m` 行 `n` 列、共 `m*n` 块结构相同的中介层，保留单块布局布线能力；包括 `1x1` 在内的所有规模使用新输入格式。`Interposer` 表示单块，`Substrate` 表示整个阵列；资源身份采用分层地址，全局位置仅用于距离计算和显示，寄存器地址与命名保留 tile 内局部含义。本阶段只考虑单 mode。

以下逐项撰写需求规格文档，每添加一项内容，都需要注意和前面已有部分的联系与兼容

### 配置文件与解析模块改动

现有输入由 `config.json`、`interposer.json`、`topdies.json`、`topdie_insts.json`、`external_ports.json`、`connections.json`、`register_adder.json`、`01_ports.json` 组成。多中介层格式将 `topdie_insts.json`、`external_ports.json`、`01_ports.json` 合并到 `interposer.json`；即使阵列规模为 `1x1`，也必须使用下述新格式，不保留旧格式回退。

#### 保持不变的文件

- `topdies.json`：芯粒类型与 `pin_map` 格式不变。
- `connections.json`：连接格式不变。所有 `topdie_insts` 实例名和 `external_ports` 端口名必须在整个 m×n 系统内全局唯一，因此连接端点仍可直接使用现有名称。

#### config.json

由于三个独立文件已合并，`config.json` 必须删除 `topdie_insts`、`external_ports`、`ports_01` 三个路径；其余字段及含义不变。保留现有拼写 `reigster_adder`，该字段仍可省略用于只加载、放置或布线；正式输出 controlbits 时必须提供完整的寄存器映射：

```json
{
  "interposer": "interposer.json",
  "topdies": "topdies.json",
  "connections": "connections.json",
  "reigster_adder": "register_adder.json"
}
```

#### interposer.json

阵列属性只定义一次，所有 tile 共用同一组单块结构参数；每个 tile 描述自身的局部实例、端口和 padctrl 端口位置。下例取自 `test/config/case5` 的部分实例、外部端口和 0/1 端口，用于展示合并后的完整字段结构，并非完整 case5 测例；padctrl 沿用已有位置示例：

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
      "external_ports": {
        "IO_0_0_0_0": {
          "coord": { "row": 9, "col": 0, "dir": "vert", "index": 0 }
        },
        "IO_1_2_13_5": {
          "coord": { "row": 7, "col": 12, "dir": "hori", "index": 45 }
        }
      },
      "ports_01": {
        "pose": {
          "1": { "row": 9, "col": 6, "dir": "vert", "index": 1 },
          "5": { "row": 5, "col": 0, "dir": "hori", "index": 8 }
        },
        "nege": {
          "1": { "row": 9, "col": 6, "dir": "vert", "index": 0 },
          "5": { "row": 5, "col": 0, "dir": "hori", "index": 9 }
        }
      },
      "padctrl": [
        {
          "cob_row": 8,
          "cob_col": 3,
          "direction": "hori",
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
- `topdie_insts.*.coord` 是块内 TOB 阵列坐标；`external_ports.*.coord`、`ports_01` 和 `padctrl` 中的位置是**块内局部 Track 坐标**。Reader 将所属 tile 与局部坐标组合为分层地址，再通过 `Substrate` 获取资源，不把全局二维位置作为资源身份。
- `external_ports` 保留原文件的 `端口名 → {"coord": TrackCoord}` 结构；`ports_01` 保留 `pose/nege → 局部编号 → TrackCoord` 结构。每个 tile 必须提供 `topdie_insts`、`external_ports`、`ports_01` 和 `padctrl`，无相应资源时使用空对象或空数组；`ports_01` 的 `pose`、`nege` 均须保留。
- 局部坐标延续当前约定：row 向上增加，col 向右增加；COB `(r,c)` 与其左侧的水平 Track `(r,c,hori,index)`、下侧的垂直 Track `(r,c,vert,index)` 共用行列坐标。其右侧 Track 为 `(r,c+1,hori,index)`，上侧 Track 为 `(r+1,c,vert,index)`；方向统一使用旧格式 `hori/vert`。
- 设单块 COB 尺寸为 `R×C`：COB 坐标满足 `0≤row<R, 0≤col<C`；水平 Track 满足 `0≤row<R, 0≤col≤C`；垂直 Track 满足 `0≤row≤R, 0≤col<C`。Track index 满足 `0≤index<COB::INDEX_SIZE`，当前为 `[0,128)`；最上侧垂直 Track 和最右侧水平 Track 的行或列允许等于 `R` 或 `C`。
- `external_ports` 与 `ports_01` 的端口只能位于整个拼接基板的外边界。位于 tile 内侧拼接边的坐标是 seam Channel，必须拒绝作为外部端口或 0/1 端口。
- 单块 TOB 阵列保持当前硬件定义的 `4x4`，TOB 阵列坐标 `(row,col)` 在所属 tile 内映射到局部 Channel 位置 `(1+2*row,3*col)`。此映射只在单块内部成立，不能应用于整个系统的 tile 坐标或全局位置；`topdie_insts.*.coord.row`、`col` 均须位于 `[0,4)`，所有 TOB 映射位置及两侧接入 COB 必须落在配置的单块资源范围内，不满足时拒绝配置。
- `tiles[].padctrl` 是该 tile 的 padctrl Track 位置真源，替代现有 `is_padctrl_port()` 中写死的坐标数组。字段名保持 `cob_row`、`cob_col`、`direction`、`index`，但前两个字段描述上述 Track 坐标，转换为 `LocalTrackCoord{row,col,dir,index}`；`direction` 取 `hori/vert`，重复条目和越界坐标均为配置错误。
- Reader 为每个 `Interposer` 构建 padctrl 局部 Track 坐标集合；`Interposer::is_padctrl_port(local_track)` 和 `Substrate::is_padctrl_port(track_address)` 查询此集合。padctrl 标识是预先给定的硬件属性，不等同于系统 IO 身份：示例 `(8,3,hori,71)` 表示 tile 内该位置的 Track，可以合法列入 padctrl，但不能因此作为外部端口。整体外边界判断独立执行；内侧拼接边上的 padctrl 标识保留在两端 tile 的局部配置中，实际资源解析为同一条 SeamTrack。
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

提供寄存器映射时，`register_adder.json.tiles` 必须与 `interposer.json.tiles` 一一对应，tile 坐标不得缺失、重复或超出阵列。每个 tile 的 `register_map` 必须完整包含 `botleft_REG0.txt`、`botright_REG1.txt`、`topleft_REG2.txt`、`topright_REG3.txt`。其中地址和寄存器名均是 tile 内局部命名；Writer 后续按 tile 分别写出四个寄存器文件，避免跨 tile 地址或名称冲突。

#### 解析与逻辑模型的构建流程

读取流程调整为：`加载并校验配置 → 按 properties 构建 Substrate、各 Interposer 与 seam → 创建 BaseDie 中的实例和端口 → 解析连接`。不能像当前 `reader/module.cc` 一样先构造固定尺寸的 Interposer，再加载硬件配置。

- `config/interposer.hh`、`config/config.hh/.cc`：定义阵列属性与 tile 配置，移除三个旧独立文件的加载入口；寄存器映射改为按 tile 保存。第一版多中介层输入使用上述 JSON；现有 TXT 导入缺少 tile 身份，不自动套用到多块系统。
- `reader.hh/.cc`、`module.hh/.cc`：硬件入口与返回值改为 `Substrate`，按 tile 查找 TOB、Track，校验类型、实例名、端口名、占用和端点引用；同一个 TOB 不能初始放置多个实例。
- `BaseDie` 保持系统级逻辑容器：汇总所有 tile 的实例和外部端口；同一类型的 topdie 定义共用，实例名与外部端口名按前述规则保持全局唯一。`tiles[].topdie_insts` 描述初始位置，默认允许后续放置跨 tile 移动。
- `ports_01` 的局部编号可在不同 tile 内重复。Reader 将全部 pose/nege 端口分别合并为带 tile 身份的系统级候选资源集合，不能在遍历每块时覆盖前一块的数据；VDD/GND 连接可使用整个系统中对应极性的合法端口。
- `circuit/basedie.*`、`circuit/export/export.*`、`algo/netbuilder/netbuilder.*`：外部端口及 0/1 端口改为保存分层地址或已解析的资源引用，NetBuilder 按实例的实际 TOB 与 bump index 获取端点，禁止丢失 tile 身份。

### hardware部分的硬件资源设计

#### 总体模型

保留现有 `source/hardware/interposer.hh/.cc` 中的 `Interposer`，使其继续表示**一块物理中介层**；将 COB 尺寸和边界判断改为运行时实例属性，TOB 阵列仍为 `4x4`，TOB 到 Channel 的映射保留每块内部的 `(1+2*row,3*col)` 规则。每个 `Interposer` 独立保存本块的 COB、TOB、块内 Track、向系统外侧伸出的 IO Track 和本地寄存器状态。

新增基板级 `Substrate`，表示 m×n 块 `Interposer` 组成的整体，并拥有 tile 网格、拼接 Channel、整体边界和全局资源查询能力：

```text
Substrate
├── TileCoord(row, col) -> Interposer
│   ├── 局部 COB / TOB
│   └── 块内 Track / 整体外边界 IO Track
└── SeamChannel
    └── SeamTrack：连接两个相邻 tile 的边缘 COB
```

`Substrate` 是后续布局、布线、全局边界判断和跨中介层延迟计算所面对的硬件入口；`Interposer` 不再承担整个 m×n 系统的资源图职责。

#### 对象归属、生命周期与父层级查询

对象的所有权必须单向向下：`Substrate` 唯一拥有各个 `Interposer` 和所有 `SeamChannel`；每个 `Interposer` 唯一拥有本块的 COB、TOB、块内 Track 和整体外边界 IO Track。跨中介层的 `SeamTrack` 由所属 `SeamChannel` 拥有，而 `SeamChannel` 由 `Substrate` 拥有。内侧拼接边不能在两个 Interposer 中各创建一份 Track；两端局部坐标只是同一条 SeamTrack 的定位入口。

```text
Substrate (unique owner)
├── Interposer (unique owner of local COB / TOB / Track / outer IO Track)
└── SeamChannel (unique owner of SeamTrack)
```

COB、TOB 与块内 Track 在构造时接收其 `Interposer&`，并保存 `const Interposer* const` 形式的私有父指针；该指针只能在构造函数中初始化，后续不能改指向，也不能经由该指针修改父对象。`Interposer` 以同样方式保存 `const Substrate* const` 与自身 `TileCoord`。`SeamTrack` 保存 `const SeamChannel* const`，`SeamChannel` 保存 `const Substrate* const`。这些均为非拥有观察指针，不使用 `shared_ptr`/`weak_ptr`，以保持所有权唯一、避免循环引用和引用计数开销。

`Substrate`、`Interposer`、COB、TOB、Bump、Track、SeamChannel 和 SeamTrack 等具有唯一身份的真硬件对象不可复制，也不能直接移动对象本体；地址、COBConnector/TOBConnector 等连接描述与搜索状态仍可按值复制，其内部引用必须指向真对象。容器保存 `std::unique_ptr`，允许移动拥有指针，但被指向的对象地址保持稳定。父对象的生命周期必须覆盖全部子对象；硬件对象不逃逸到父对象销毁之后。重建或清空硬件前必须先清理引用它的 Net、PathPackage 和搜索状态；普通路由重置只清理状态，不销毁资源对象。

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

struct BumpAddress {
    TobAddress tob;
    std::usize index;
};
```

其中 `CobAddress`、`TobAddress`、`TrackAddress` 是资源的层级地址，而不是新增硬件对象；例如 `CobAddress` 表示“第 `(tile.row, tile.col)` 块中介层内、局部坐标为 `local` 的 COB”。为避免 `COBCoord`、`TOBCoord` 和普通 `Coord` 在类型系统中互相混用，逐步取消现有 `using COBCoord = Coord`、`using TOBCoord = Coord`，改为具有独立类型的局部坐标结构。

`BumpAddress` 由所属 TOB 的分层地址及 bump index 唯一确定，不通过局部物理坐标 `/2`、`/3` 反推身份。Net、PathPackage、TOB 分配器及端点检查必须使用完整地址或硬件真对象指针判等，不能只比较不同 tile 中重复的局部坐标。

全局二维位置由 `Substrate` 按需计算，用于 GUI、距离/延迟估计和 SAT scope；它不是资源的主身份。第一版距离坐标延续当前 COB 网格单位：块内 COB、Track 或 TOB 接入位置 `(r,c)` 对应全局 `(tile.row*R+r,tile.col*C+c)`，其中 `R/C` 是单块 COB 行列数；tile 的 row 同样向上增加、col 向右增加。Bump 使用所属 TOB 的接入位置；seam 两端的局部定位入口得到同一个全局 Track 位置。`1x1` 时退化为现有局部位置，保持 HPWL 的坐标口径；不能混用 TOB 阵列序号、COB 网格位置与实际 ns 延迟，GUI 的线段绘制位置可另行派生。

#### 跨中介层 Channel

`SeamChannel` 直接描述两个相邻 tile 上、彼此正对的边缘 COB；TOB 不作为 seam 端点，因为物理上跨中介层连接发生在 COB 边缘，TOB 仍经各自 tile 内的 Channel 接入 COB。

```cpp
struct SeamEndpoint {
    CobAddress cob;
    COBDirection side;
};

struct SeamAddress {
    SeamEndpoint first;
    SeamEndpoint second;
};

struct SeamTrackAddress {
    SeamAddress channel;
    std::usize index;
};

class SeamTrack {
    // 示意字段；构造时接收所属 SeamChannel，父指针和地址私有保存。
    const SeamChannel* const _channel;
    std::usize _index;
    bool _enabled;
    // 占用状态由路由资源管理维护，与 enabled 分开。
};

struct SeamChannel {
    SeamEndpoint first;
    SeamEndpoint second;
    std::vector<std::unique_ptr<SeamTrack>> tracks;
    double latency_ns;
};
```

以上为结构示意，实际类需落实不可复制、不可移动、私有父指针和受控状态访问；`SeamTrack` 定义前需前向声明 `SeamChannel`。端点按 TileCoord 的 `(row,col)` 字典序规范化为 first/second，保证同一 Channel 的地址不因查询方向改变。

相邻 tile 的正对边缘 COB 自动生成 seam Channel。第一版两端使用相同的物理 track index 直连，不进行 index 重排；每条 Channel 包含当前硬件的 128 条 SeamTrack。左右相邻 tile 每对产生 `R` 条 Channel，上下相邻 tile 每对产生 `C` 条 Channel，整体数量为 `m*(n-1)*R+(m-1)*n*C`。seam 支持双向通信，但同一条 SeamTrack 在两个方向上共享唯一占用状态和 `latency_inter_track`。

边缘 Track 的身份按整个阵列确定：

- 最外圈朝外的 Track 保持系统 IO Track；例如最左列 tile 的 `hori,col=0`、最右列的 `hori,col=C`、最下行的 `vert,row=0`、最上行的 `vert,row=R`。
- 相邻左右 tile 的右边缘 `(r,C,hori,index)` 与左边缘 `(r,0,hori,index)` 解析为同一条 SeamTrack；相邻上下 tile 的上边缘 `(R,c,vert,index)` 与下边缘 `(0,c,vert,index)` 同理。
- SeamTrack 直接连接两端 COB，不附加两份本地边缘 Track 或额外接入线段。通过任一端查询、占用、释放和统计延迟都操作同一个资源；内侧拼接边不能声明为 external_ports 或 ports_01。

每条 SeamTrack 增加 `enabled` 属性，表示该物理 track 是否启用，与“当前是否被占用”独立。默认全启用，可在构建硬件时按实际可用集合将部分 track 标记为禁用；本阶段尚需确定启用集合的输入载体和格式，不在上述 JSON 示例中隐含新增字段。禁用资源保留地址并可被查询，但不得作为搜索邻居、TOB 接入候选或已提交路径的一部分，也不能退化成系统 IO。路由期间启用集合固定，释放或重置占用不改变 enabled。

统一路由资源引用采用 `Track* / SeamTrack*` 的可区分表示，例如 `RoutingTrackRef = std::variant<Track*,SeamTrack*>`；统一资源身份对应 `TrackAddress / SeamTrackAddress`。COB 内的可编程连接仍由 `COBConnector` 描述，但其两个关联 Track 资源可以是块内 Track、IO Track 或 SeamTrack。seam 本身没有可编程开关，进入、离开它仍需配置两端 COB 的方向和开关。路由边应返回目标资源、连接信息、可用性和代价，不能要求 SeamTrack 假造一个 COBConnector。

#### Substrate 的初始接口

以下接口是第一批必要接口，不构成接口上限；后续可随算法和 GUI 的需要增加：

- `tile(TileCoord)`：取得指定的 `Interposer`。
- `get_cob(CobAddress)`、`get_tob(TobAddress)`、`get_bump(BumpAddress)`：按层级地址取得相应资源；`Interposer` 保留同类局部坐标查询。
- `get_track(TrackAddress)`：取得块内 Track 或整体外边界 IO Track，内侧 seam 坐标不创建本地 Track。
- `get_seam_track(SeamTrackAddress)`、`resolve_track(TrackAddress)`：分别按规范化 seam 地址查询资源，或将任一 tile 的局部 Track 地址解析为普通 Track / SeamTrack 的统一引用。`Interposer` 可提供局部 `resolve_track`，内部交由 Substrate 解析拼接边。
- `adjacent_resources(RoutingTrackRef)`、`adjacent_available_resources(...)`：统一遍历目标资源及 COB 连接信息；可用邻接必须同时满足资源已启用、占用和寄存器开关约束。对 SeamTrack 可访问两端 COB，但不得重复统计线段。
- `is_outer_boundary_port(TrackAddress)`：判断端口是否位于整个拼接基板外边界。
- `is_padctrl_port(TrackAddress)`：查询 tile 配置中的 padctrl 端口信息。
- `global_position(...)`：按需生成资源的全局二维位置。
- `is_enabled(SeamTrackAddress)`：查询 seam track 的硬件启用状态。
- `reset_regs()`、`reset_routing_state()`、`for_each_tile()`：分别重置寄存器、清理全系统 Track/seam 占用及连接状态、遍历 tile 并写出控制位；重置不改变资源地址、对象归属或启用集合。

无效地址返回空结果或配置错误；禁用 SeamTrack 与不存在的地址须可区分。普通资源查询不会获得修改 enabled 的权限。


### 放置模块改动

#### 放置范围与目标

保留现有模拟退火框架，每个芯粒实例仍占用一个 TOB。`tiles[].topdie_insts` 表示初始位置，默认允许在整个 Substrate 的合法 TOB 间移动和交换，不将初始 tile 当作固定约束。第一版不新增限定 tile 或固定实例字段；如后续需要，须明确增加放置约束并统一作用于初始校验、移动、交换和恢复。

模拟退火的目标函数采用基于全局 bbox 路径的延迟估计，必须体现普通 Track 与 SeamTrack 的延迟差异。HPWL 用于确定估计路径的几何范围和长度，不再单独作为最终代价。所有 Net 端点使用 Substrate 的统一距离坐标，不同 tile 上相同局部坐标不能被视为重合。

#### 基于 bbox 的延迟估计

先由端点位置计算 bounding box，再沿 bbox 边界构造贴着角的一次转弯路径，即先水平后垂直、或先垂直后水平的上/下两种 L 形走法。二端 Net 的候选连接实际源、目的端点；多端 Net 以全部端点的 bbox 两个相对角点形成 HPWL 估计，不将该折线路径视为已经连接全部端点的真实路由树。

第一版比较两种候选的估计延迟，取较小值作为该 Net 的放置代价；bbox 退化为一条直线时只需计算该路径。路径跨越 tile 的边界时，按对应 Channel 的归属区分普通 Track 与 SeamTrack，不能把只是沿 tile 边界平行行进的线段误计为 seam。

```text
HPWL = (max_row - min_row) + (max_col - min_col)
D_est(P) = N_inner_est(P) * latency_inner_track
         + N_seam_est(P) * latency_inter_track
cost(net) = min(D_est(P_upper), D_est(P_lower))
cost(placement) = sum(cost(net))
```

`N_inner_est`、`N_seam_est` 为 bbox 折线路径中按网格线段估计的普通 Track、SeamTrack 数量；每个估计线段使用对应的延迟系数，一次跨 seam 只计一段。等价地，可在 HPWL 对应的普通线段代价上加入每段 seam 的额外延迟。估计只用于放置，不选择实际 track index、不预留硬件资源，也不保证拥塞或 enabled 约束下布线可行；最终可行性及延迟由真实布线结果确定。

SyncNet/bus 按各成员线分别形成 bbox 并累计估计代价，不能用整个 bus 的一个大 bbox 替代全部成员。普通多端 Net 暂沿用上述 bbox 估计，不在本阶段增加 Steiner 树估计器。Bump 的长度计数与延迟分开处理；bbox 放置估计覆盖 Track 部分，Bump 延迟如需纳入再按后面的统一延迟模型补充。

#### TOB 选择、端点更新与恢复

- `place.hh/.cc`、`placestrategy.hh`、`sa/saplacestrategy.hh/.cc` 的硬件入口改为 Substrate。目标 TOB 从整个系统的合法空闲集合中抽样，不能先均匀选 tile 再选该 tile 的 TOB，以免各 tile 空闲数不同时产生偏差。
- `SAPlaceStrategy::is_changable()`、`Net::has_tob_in_ports()`、`place.cc` 的一致性检查采用真实 TOB 对象或完整 TobAddress，禁止只比较局部 TOB 坐标。跨 tile 的两个 `(row,col)` 相同 TOB 仍是不同资源。
- `Net::coords()` 及 `circuit/net/types/*` 提供全局位置供 `net_cost()` 计算 bbox 折线路径及延迟；SyncNet 的成员应分别评估。资源判等、端点更新和历史路径继续使用分层身份，不能用位置代替身份。
- `net_cost()`、`total_net_cost()`、`evaluate_placement()`、进度回调和日志的代价类型支持非整数延迟，不截断 `latency_inner_track`、`latency_inter_track`。SA 的增量代价、接受概率与最优解比较统一使用该延迟代价；日志区分 HPWL 几何统计和以 ns 表示的估计代价。
- 复用 `TopDieInstance::move_to_tob()`、`swap_tob_with()`、`Bump::update_bump()`、`swap_bump()` 的指针更新机制；跨 tile 操作必须同步更新全部受影响 Net 的 Bump，原 TOB 释放、目标 TOB 占用，实例和 TOB 的绑定一致。
- `save_current_placement()`、`restore_placement()` 可继续保存稳定的 TOB 真对象指针；拒绝一次移动或交换后，以及恢复最优解后，都必须恢复位置、端点与占用。放置开始前清除旧路由结果；放置结束后重新布线，不能保留旧位置的 PathPackage。
- 保留当前随实例数调整的 SA 搜索预算；只有实际多 tile 测例显示搜索不足时再调整参数，不将 tile 数直接当作必须新增的退火规则。

#### 验收条件

- 新格式 `1x1` 下仍可完成当前单块放置，同一布局的 HPWL 几何值与当前单块实现一致；没有 seam 时，bbox Track 估计代价退化为 HPWL 对应的普通线段数乘 `latency_inner_track`。
- `1x2`、`2x1`、`2x2` 下可跨 tile 移动和交换；全系统无空闲 TOB 时仍可交换已占用 TOB。
- 两个不同 tile 的同局部坐标 TOB 不误判为同一对象，bbox 路径与估计代价随跨 tile 移动正确变化；改变 seam 延迟系数时，跨 seam 的候选代价同步变化。
- 相同几何长度下，普通 Track 与 SeamTrack 数量不同的估计路径得到相应不同的延迟代价；上/下 L 形路径、退化 bbox 和 SyncNet 成员的累计均符合上述公式。
- 被拒绝的操作和最优解恢复均无重复占用或端点残留；不要求随机退火得到与旧实现完全相同的最终布局。


### Maze 布线模块改动

#### 入口、资源与 TOB 分配

保留现有 BB、BT、TB、多端和 SyncNet 的线网类型、TOB/Wilton 连接规则及单 mode 流程，将 `RouterBackend`、`route_nets`、`RouteEngine`、`RouteStrategy` 和 `Net::route()` 的硬件入口改为 Substrate。

- `common/maze/mazeroutestrategy.hh/.cc` 的起点集合、终点集合、搜索队列、前驱记录与回溯路径改用统一资源引用，能够表示 Track 和 SeamTrack。通过 Substrate 邻接查询遍历两端 COB，不自行计算跨 tile 邻接，也不重复创建边缘 Track。
- Bump 的接入候选通过所属 tile 的 TOB 获取，沿用当前 TOBConnector 和 COBUnit 约束并过滤不可用候选。按本版固定 TOB 映射和两侧接入 COB 均在本块内的要求，TOB 接入为块内垂直 Track；跨 tile 传输仍须经本块 COB 进入已启用的 SeamTrack。
- `common/allocate/hopcroft_karp.hh/.cc`、`allocatestrategy.hh` 的资源分组改为 TobAddress 或真实 TOB 对象；匹配结果在对应 tile 的 TOB 上落实，禁止将不同 tile 的同局部坐标 TOB 合并。
- `check_found()`、`sync_preroute()`、Net 相关性检查、路径查找和资源集合使用完整资源身份或唯一真对象引用；不同 tile 上同局部坐标的资源不得误判为终点、同一 TOB 或已占用资源。

#### 路径、占用与同步重布线

`routed_path`、`PathPackage`、`PathInOrder` 及历史路径记录资源类型、完整地址与相应 COB/TOB 连接信息。SeamTrack 在线段序列中只出现一个物理资源；进入和离开所需的 COBConnector 属于各自端点 tile，seam 本身没有额外开关。

- 搜索和提交均检查 SeamTrack.enabled；两个方向、两个局部定位入口共享占用。多端 Net 可复用自身已经形成的路径树，其他不允许共享的 Net 不能占用该 seam；同一物理 seam 不因方向不同而获得两份容量。
- 路径预留、`occupy_all()`、`connect_all()`、`reset_all()`、切断路径、失败回滚和系统重置统一覆盖 Track、SeamTrack、COBConnector 与 TOBConnector。失败后释放本次新增资源，不破坏其他已提交 Net；释放占用不改变 enabled。
- `common/maze/mazererouter.hh/.cc` 的搜索节点保存真资源引用与搜索状态，不复制或 `new` 硬件 Track；删除当前 `NodeTrackInterface` 及搜索扩展中的 Track 复制逻辑。回溯结果仍指向 Substrate/Interposer/SeamChannel 拥有的对象。
- `MazeRerouter` 的终点判断、祖先去环、距离估计和剪枝改用全系统资源身份与统一位置。bus/SyncNet 按等延迟要求进行绕行与拉齐，保留能搜索不同延迟路径的状态；不能直接套用只按资源保留一个最短距离的策略来完成同步拉齐。
- `HistoryPathPackage` 使用 TrackAddress、SeamTrackAddress、BumpAddress 及带 tile 的连接信息恢复真对象。即使本阶段不做多 mode，单 mode 同步重布线和失败恢复仍需适配。

#### 路径长度、延迟与搜索目标

长度和延迟分别记录。路径长度直接统计有序路径中的 Track 条目数量，普通 Track、系统 IO Track 和 SeamTrack 每个条目均计长度 1；若 Net 端点是 Bump，每个 Bump 端点再计长度 1。二端 Bump-to-Bump 路径加 2，Bump-to-Track 或 Track-to-Bump 路径加 1，Track-to-Track 路径不增加 Bump 长度：

```text
L(P) = N_inner(P) + N_seam(P) + N_bump(P)
```

`N_inner` 包括普通块内 Track 和系统最外圈 IO Track，`N_seam` 包括有序路径中的 SeamTrack。不得继续使用 `common/maze/path_length.hh/.cc` 现有的公共 COB 分组、压缩或去重统计；每个路径条目按一次计数，即使它与前后条目共享同一个 COB。SeamTrack 的两端局部地址必须在构建路径时解析为一个资源条目，不能把地址别名表示成两段路径；这属于表示一致性，不是统计时的去重。

`PathPackage::_length`、`Net::length()`、历史路径、切断路径后的重算以及日志均改用上述口径。多端 Net 分别记录源到各接收端的有序路径长度和延迟；Net 总长度如需汇总，则按其记录中的 Track 条目和 Bump 端点条目统计，不再对整棵树做公共 COB 压缩，也不能将树总量当作单条端到端路径长度。

实际路径延迟为 `D(P) = N_inner(P)*latency_inner_track + N_seam(P)*latency_inter_track + D_bump(P)`。普通 Track 与系统 IO Track 使用 `latency_inner_track`，SeamTrack 使用 `latency_inter_track`，计数来自同一有序路径；`D_bump` 为 Bump/TOB 端点接入延迟，其取值或本阶段是否忽略仍需明确。Bump 长度为 1 不意味着其延迟为 1 ns 或自动等于 `latency_inner_track`。延迟计算不额外按 COBConnector 数量收费，也不恢复旧的路径压缩口径。

普通最短路目标与 bus 同步约束分别规定；bus 已确定采用等延迟：

- 普通布线是否也要求最小延迟仍待明确：保留 BFS 时按等步数搜索并记录延迟；若要求最小延迟，则使用 Dijkstra，或使用有效延迟下界的 A*。加权搜索从起点 Track 的延迟开始累计，每扩展到一个新 Track 加上该资源的延迟，端点 Bump 按最终确定的端点延迟模型处理，保证搜索代价与最终路径统计一致。
- bus/SyncNet 的每个成员按实际端到端路径延迟满足共同目标 `D_i=D_target`，可以具有不同 Track 数量和路径长度。`sync_preroute()`、`sync_reroute()`、`MazeRerouter` 的拉齐目标、排序、比较、剪枝、聚合统计和验收均由长度改为延迟；长度只作为独立统计，不能用于判断 bus 是否同步。
- 同步初始目标可取各成员预布线后的最大延迟，再通过绕行寻找所有成员共同可达的延迟目标；当前最大值若无法精确拉齐，须在有界搜索内调整共同目标或报告失败，不能把“超过目标”视为同步成功。优先使用能够精确表达配置延迟系数的离散时间单位；若采用浮点，必须明确数值误差容差，不自行引入允许物理 skew 的放宽规则。
- 例如忽略端点延迟、块内为 1 ns、seam 为 1.5 ns 时，6 段块内路径为 6 ns，4 段块内加 2 段 seam 为 7 ns；相同线段数不保证相同延迟。将 BFS 替换为 Dijkstra 本身也不能实现等延迟拉齐。
- 同步重布线保留有界失败机制，无法达到共同延迟目标时返回失败，释放本次预留资源；不能为了拉齐延迟使用禁用 seam。

#### 验收条件

- 新格式 `1x1` 可完成原单块各类 Net 和 SyncNet 布线，长度按 Track 数量加 Bump 端点数校验；旧的去重/压缩长度 golden 和阈值按新定义更新。已有 Writer golden 继续用于硬件配置语义回归，不强制随机搜索选择同一条路径。
- `1x2`、`2x1`、`2x2` 可分别经过水平、垂直及多条 seam；两端查询得到同一资源，提交正确设置两端 tile 的 COB 状态，跨 seam 延迟仅计一次。
- 禁用部分 seam index 后，搜索绕行或正常失败；仅剩禁用通路时不能成功。禁用不改变系统 IO 边界，也不因回滚或重置被自动启用。
- 不同 tile 的同局部坐标资源不会误命中终点或被错误合并；两条不允许共享的 Net 不能以相反方向同时占用同一 SeamTrack。
- 同一 COB 内连续经过多条 Track 时逐条计数，BB/BT/TB 路径分别增加正确的 Bump 长度；改变延迟系数不改变长度统计，但应改变相应路径的延迟。
- 同步重布线结果引用硬件真对象；切断、失败回滚及重置后 seam、COB 和 TOB 的占用一致。bus 成员必须等延迟，允许等延迟但长度不同，等长度却延迟不同的结果必须判为不满足同步要求。

### 布线完成后的后续改进

以下仅记录后续方向，不在本次多中介层扩展中展开实现细节：

- 参考 `/Users/jiaheng/FDU_files/Tao_group/论文/papers/SAT-router-for-interposer/PR_tool/algorithm/FPIA-RRR` 的 rip-up-and-reroute 方法，进一步修改布线算法；借鉴其迭代重布线思路，延迟与长度口径仍遵循本需求文档。
- 去除 connector 数据结构，使布线阶段只处理硬件合法路径和资源约束；寄存器信息在布线完成后根据最终路径计算。当前章节中对 COBConnector/TOBConnector 的适配属于本阶段方案，后续再进行此项改造。



### 除数据结构外需要同步处理的问题

- 现有 `Interposer::adjacent_tracks()` 只返回 `Track* + COBConnector`。需适配统一 Track/SeamTrack 引用、COB 两端资源解析及启用检查，保留原有 Wilton 开关约束，支持占用、释放、双向连通和延迟查询。
- `PathPackage` 与历史路径目前只记录 Track 和可选 `COBConnector`；必须记录经过的 seam track，才能正确回放、释放资源、统计延迟和输出控制位。
- `TOB_COORD_MAP`、`available_tracks()` 以及路径读回中通过 `/2`、`/3` 反推 TOB 的逻辑，必须改为由所属 `Interposer` 和局部地址查找。
- Maze、SAT/ILP、布局器、GUI 和 Writer 必须通过 `Substrate` 查询资源和边界，不再直接使用单块 `Interposer::COB_ARRAY_*` 或自行计算跨 tile 邻接关系。
- 外部端口/0-1 端口只能映射到 `Substrate` 的整体外边界；内部 tile 边缘必须映射为 seam。
- 以 `tiles[].padctrl` 构建每块 `Interposer` 的 padctrl 查询索引，替换 `is_padctrl_port()` 和 `XinzhaiRegister` 中的固定坐标。若 padctrl 还承担控制寄存器分组选择，需补充 `register_group` 或等价的无歧义规则。
- 寄存器地址空间按 tile 独立；Writer 必须逐 tile 读取本地 `register_map` 并分别输出四个寄存器文件。
- `MazeRerouter` 当前复制和重新创建 Track 的做法必须移除；搜索节点引用唯一硬件对象，不拥有或复制资源。HK 分配器按 `TobAddress` 或真实 TOB 对象分组，防止不同 tile 的同坐标 TOB 被合并。

| 模块 | 必须改动 |
| --- | --- |
| SAT / ILP 布线 | `source/algo/router/sat_ilp/graph/unified_routing_graph.cc`、`common/hw_map.hh`、`scope/`：移除对 `Interposer::COB_ARRAY_*` 和固定 TOB 线性编号的依赖；编码普通 Track/seam 的对应延迟，长度与 bus 等延迟约束采用本文的新统计口径，并控制随 `m*n` 增长的 SAT 规模。 |
| 路径持久化 | `source/circuit/path/pathpackage.cc`：禁止通过 Bump 物理坐标 `/2`、`/3` 反推 TOB，改用 `Interposer` 提供的坐标反查 API。 |
| Controlbits 输出 | `source/parse/writer/writer.cc`、`source/parse/writer/registers/*`：将单块循环、寄存器名和 `RegisterValue` 扩展到多 tile，避免 `cob_0_0_*` / `tob_0_0_*` 重名。建议每块输出 `tile_r_c/regnamecontrolbit_4part/`，块内保留原四文件与命名。 |
| GUI / 可视化 | `source/widget/layout/`、`view2d/`、`view3d/`：改造单块画布边界、COB/TOB 遍历和坐标换算，显示 tile 分界及跨 tile 路径。CLI 完成后实施。 |
| 测试与文档 | 将 case5 转为新格式的 `1x1` 回归，保留原连接与 controlbits golden；新增 `1x2`、`2x1`、`2x2` 测例。覆盖内部 seam 非外部 IO、跨 seam 连通、禁用 seam 不可使用、seam 延迟与多 tile 无重名输出；实现完成后更新 `source/AGENTS.md`、`test/AGENTS.md`。 |
