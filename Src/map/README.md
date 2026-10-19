# Src/map — Lanelet2 地图组件（Wave 4/5：M1→M5）

本组件按 `Docs/component_development_plan.md` §2.1（M1–M5）、§3.6 实现，
目标为 ROS-free、C++17 的 Lanelet2 地图封装：导入、投影与坐标、元信息与
导入诊断、查询与局部窗口裁剪、公共 JSON 快照与微基准。公共 API
（`include/planner/map/`）不暴露任何 Lanelet2 / ROS / 浏览器类型，所有结果
转换为本组件值类型与 `planner::common` 类型，对象 ID 一律以十进制字符串保留
原始 OSM id。

## 1. 构建适配方式（M1）

上游 Lanelet2 以 Catkin/`mrt_cmake_modules` 为主要构建方式，仓库根目录没有
`CMakeLists.txt`，也不导出普通 CMake 目标。因此本组件采用**轻量包装
CMake**（不改上游任何文件）：

- `dependencies.lock.json` 的 `lanelet2` 条目（tag `1.2.3`，commit
  `ae39c8d6…`，URL+SHA256）由 `FetchContent` 拉取并校验；由于上游根目录没有
  `CMakeLists.txt`，`FetchContent_MakeAvailable()` 只下载解压，不会执行上游
  构建脚本。
- 组件在 `CMakeLists.txt` 中提供**精选源文件列表**，编译为私有静态库
  `planner_lanelet2`（core、io、projection、traffic_rules、routing、validation；
  不含 Python 绑定、示例与 ROS）。
- 上游注册表（OSM 解析器、交通规则、regulatory element、validation 检查）依赖
  静态初始化；静态库链接会丢弃未被引用的目标文件，因此消费方使用
  `$<LINK_LIBRARY:WHOLE_ARCHIVE,planner_lanelet2>`（CMake ≥ 3.24）整体归档。
- 未对上游源码或构建脚本做任何补丁。

上游依赖获取方式（全部在组件内可复现）：

| 依赖 | 来源 | 说明 |
| --- | --- | --- |
| Lanelet2 1.2.3 | `dependencies.lock.json`（URL + SHA256） | 唯一锁定来源 |
| Boost（filesystem、serialization）、Eigen3 | 系统包（`libboost-*-dev`、`libeigen3-dev`） | 编译期由 `find_package` 定位 |
| pugixml 1.16 | `dependencies.lock.json`（URL + SHA256） | 已晋升；组件曾以本地 pin 先行接入，锁条目出现后自动优先使用 |
| GeographicLib 2.5 | `dependencies.lock.json`（URL + SHA256） | 已晋升；同上 |
| cpp-httplib v0.60.1 | `dependencies.lock.json`（URL + SHA256） | W2 本地 HTTP 服务；header-only，TLS/压缩选项关闭 |

离线/受限网络环境可用 `FETCHCONTENT_SOURCE_DIR_LANElet2`（以及
`_PUGIXML`/`_GEOGRAPHICLIB`）指向已解压源码目录，方式与根工程一致；此时不再
校验 URL 哈希。

## 2. 公共接口

- `MapLoadOptions`：显式配置投影方式（`utm` / `mercator` / `spherical_mercator`）、
  投影原点（必填，拒绝 Lanelet2 默认原点）、交通规则 location/participant 与
  是否运行导入校验。
- `Map::import_osm_file` → `ImportResult`（`common::Status`、原因、诊断条目、
  `MapInfo`）。无效文件、缺失/非法原点、投影失败、无可用对象均返回
  `invalid_input` 明确原因；可恢复解析问题与校验发现进入 `diagnostics`。
- `MapInfo`：`map_id`（文件名 stem）、`map_version`（源文件字节 SHA-256）、
  对象计数（points/line_strings/polygons/lanelets/areas/regulatory_elements）、
  局部坐标包围盒、校验统计。
- 查询：`lanelet_ids` / `lanelet_geometry`（左右边界、中心线、属性）、
  `area_ids` / `area_geometry`、`points_in_bounds`、`lanelets_in_bounds`、
  `following` / `preceding` / `adjacent`（基于懒构建的 Lanelet2 路由图）、
  `route`（最短 lanelet 序列与二维长度）。
- 方向语义：`following`/`preceding`/`adjacent` 遵循 lanelet 的存储方向；
  Lanelet2 将双向 lanelet 的反向通行建模为独立（inverted）路由顶点，因此 A 的
  后继 B 不保证把 A 列为其前驱（该连接属于 B 的反向顶点）。跨方向连通性一律用
  `route` 判断，测试 `directional_lanelets_pin_inversion_semantics` 固定该语义。
- 坐标：`to_gps` / `to_local`（投影往返，区域内误差 ≤1e-3 m，见测试）；
  `Map::project_to_local` / `Map::project_to_gps` 为静态独立助手（显式投影与
  原点，无需先加载地图），用于 GPS 选点换算与合成地图生成。
- 裁剪：`crop_to_bounds` 返回 `CropResult`——窗口几何（保留原始点 ID，裁剪
  切出的合成点为**空 ID**）、`window_frame`（窗口最小角、yaw=0）与
  `local_to_map`/`map_to_local` 变换。裁剪是纯几何操作：**拓扑查询始终在完整
  地图上进行**；`ClippedPath.start_clip_edge`/`end_clip_edge` 精确表示"该端不是
  原始折线端点"，裁剪边缘不得被解释为道路终点。`lanelet_ids` 为包围盒与窗口
  相交的语义引用。
- 空间查询/裁剪使用 Lanelet2 图层的 R-tree（`search`）筛选候选，再做精确
  过滤/裁剪；语义与按 bbox 相交一致，性能见 M5 基准。
- 公共快照（M4）：`Map::to_snapshot_json(SnapshotOptions)` 返回带公共包络的
  `"map_snapshot"`（schema_version 1）文档——地图版本、坐标系与投影原点、
  范围、对象计数、带字符串 ID 的 lanelet/area 几何与属性、regulatory element
  角色引用；只输出有限数值、按数字 ID 排序。字段说明与样例见
  `examples/data/README.md` 与 `examples/data/mapping_example.snapshot.json`。
- 对象摘要与命中测试（W2 服务）：`query_point(x_m, y_m)`（lanelet/area 多边形
  覆盖该点，边界计入）与 `query_bounds(window)`（包围盒相交）返回
  `ObjectSummary`（原始 ID + 属性 + 包围盒）。
- 可行驶域栅格导出（W5 服务接入）：`Map::export_driveable_grid(window, resolution_m)`
  返回 `DriveableGridResult`（`common::GridView`）——与任一 lanelet 可行驶多边形
  相交（闭集，接触也算）的单元为 `kCellFree`，其余为 `kCellOccupied`（从不产生
  unknown）；区域（area）对象不参与首版。栅格遵循 ROS OccupancyGrid 约定（原点
  = 窗口最小角、yaw=0，宽高按 `ceil` 覆盖窗口）；窗口与地图不相交时返回全占据
  栅格而非错误。失败原因：`invalid_window` / `empty_window` / `invalid_resolution`
  / `grid_too_large`（单元数上限防失控分配）。语义与确定性保证见 `map.hpp`
  头注释；服务侧接入见 §7 与 `Src/service/README.md`。

## 3. 示例

```sh
# 经统一入口构建后（本波次使用独立构建目录 build/map，原因见 §5）：
planner_map_info mapping_example.osm --origin-lat 49.00345654351 --origin-lon 8.42427590707
planner_map_info mapping_example.osm --origin-lat 49.0 --origin-lon 8.4 \
    --window -100 -100 100 100 --route 38992 38994
planner_map_info mapping_example.osm --origin-lat 49.00345654351 --origin-lon 8.42427590707 \
    --export-json mapping_example.snapshot.json
```

退出码：0 成功，1 导入/查询失败，2 用法错误。

## 4. 测试与数据

- `tests/`：GoogleTest（label `map`）覆盖导入诊断、元信息与 ID 保留、三种投影
  往返、对象/边界/邻域/路由查询、窗口裁剪与引用完整性、公共快照结构与
  ID/属性一致性、静态投影助手；另有 `map_bench_smoke` 以微型批次跑通基准
  输出契约（生成/加载/查询/裁剪/路由/CSV/SVG）。
- `tests/data/mapping_example.osm`：官方 `lanelet2_maps` 小型示例的工作副本
  （暂放组件内，未写入 `tests/fixtures/`；晋升为版本化夹具由协调者决定）。
  其余小样本为本仓库自建夹具，来源见 `tests/data/README.md`。

## 5. 公共快照（M4）

- 接口：`Map::to_snapshot_json(SnapshotOptions)`（可选开关：中心线、区域、
  regulatory elements）。文档类型 `"map_snapshot"`，`schema_version: 1`，
  遵循公共包络规则；字段表、语义与生成命令见
  `examples/data/README.md`。
- 样例：`examples/data/mapping_example.snapshot.json`（436 381 B，
  sha256 `fa1c5ed1…`），由官方示例地图工作副本以固定原点 UTM 导出；样例不是
  冻结夹具，晋升由协调者决定。

## 6. 微基准（M5）

- 二进制：`planner_map_bench`（`benchmarks/map_bench.cpp`），契约
  `--suite v1 --output-dir <dir>`（`tools/README.md` §7.2）。注册条目：
  `benchmark --component map --suite v1 --build-dir build/map`。
- 输入规则来自 `tests/fixtures/v5/benchmarks/map/specs.json`（优先；缺失时
  依次回退 v4/v3/v2/v1 旧夹具）：确定性合成
  lanelet 网格（small 1 000、medium 10 000、large 100 000），固定查询/裁剪/
  路由序列，预热 5 批、测量 30 批、单线程。未显式 `--scales`/`--scale` 时，
  默认规模 = spec 中所有非 `pending_generation` 的规模（v4 及更早 =
  small+medium；v5 起 large 物化后自动包含）。
- `--generate-only`（v5 起）：只做确定性生成并写出
  `synthetic_maps/<scale>.osm/.sha256/.meta.json` 后退出，不执行测量批次；
  用于物化 large 输入并按 §9 建立 SHA256 寻址归档
  `artifacts/data/sha256/<sha256>/large.osm(+.sha256/.meta.json)`。若 spec 的
  `archive.sha256` 非空，生成后强制校验生成文件 SHA256 与之一致，不一致则
  明确报错（exit≠0）并跳过该规模，不静默继续；spec archive 的 sha256 与
  relative_path 写入 `.meta.json` 与 `bench_summary.json` 的 `spec_archive`
  字段。
- **规格解释（历史；v1/v2 矛盾已由 v4 修正）**：v1/v2 的 specs 曾声明
  `lanelet_length_m=20.0` 与 `x_pitch_m=25.0`，二者矛盾且无法被 Lanelet2
  连通——后继判定要求相邻 lanelet 共享端点对象（`geometry::follows`）。
  v4 起规格自洽（`lanelet_length_m=25.0=x_pitch_m`，`geometry_rule` 的 x
  区间同步为 +25.0），旧文本矛盾已由 v4 修正。生成器始终令 lanelet 长度等于
  列间距（25 m）并共享边界节点，从而保持 extent、行列数、窗口/路由公式与
  拓扑规则不变；解析到的实际 spec 值、生成值与解释写入每张合成地图的
  `.meta.json` 与 `bench_summary.json`。
- 输出（均在 `--output-dir` 内）：`benchmark_metrics.csv`（§7.2 固定列 +
  `scale/operation/batch_size/avg_op_ms/ops_per_second/result_count_*` 等组件
  列）、`bench_summary.json`/`summary.md`、`plots/p50_p95_<scale>.svg/.csv`、
  `synthetic_maps/<scale>.osm(.sha256/.meta.json)`、查询/裁剪/路由逐例证据
  JSON。`cache_mode`：加载批 `cold`（每批全新地图对象），查询/裁剪/路由批
  `warm`（复用同一地图）；OS 文件缓存未清理，不称物理冷盘测试。
- 结果见本波次报告（`artifacts/runs/20261010T013640890647Z_map_0bcda802`）。

## 7. 本地 HTTP 服务（W2）

- 二进制：`planner_map_service`（`src/service_main.cpp`），默认构建产物
  `{build_dir}/Src/map/planner_map_service`；启动时以显式投影原点导入 OSM
  （与库同一路径），随后提供 JSON API。运行入口（components.json `service`）：
  `run --component map --workers 1 --build-dir build/map`。
- CLI：`--map <osm> --origin-lat <deg> --origin-lon <deg> --port <port>
  [--run-dir <dir>]`；`--port 0` 自动分配并打印实际端口；默认端口 8765、
  默认 host `127.0.0.1`。其余可选项：`--projection`、`--origin-ele`、
  `--traffic-rules-*`、`--no-validation`、`--quiet`。
- 响应契约：所有响应为 UTF-8 JSON，带公共包络 `schema_version: 1` 与 `type`；
  错误用 `Status` 语义 + HTTP 状态码：400 无效输入（`invalid_input`）、
  404 未知对象 ID（`invalid_input`）或不可达路由（`no_path`）、500 内部错误。
  地图访问经互斥锁串行化（首个拓扑查询懒构建路由图）。
- 端点：

  | 方法与路径 | 请求 | 响应 `type` |
  | --- | --- | --- |
  | `GET /health` | — | `health`（status/map_id/map_version/uptime_s/grid_protocol_version/scene_rules_version） |
  | `GET /map/info` | — | `map_info`（坐标系/原点/计数/范围/校验统计） |
  | `GET /map/snapshot` | — | `map_snapshot`（M4，启动后首次请求缓存） |
  | `POST /map/query/point` | `{x_m, y_m}` | `point_query`（命中 lanelets/areas 摘要） |
  | `POST /map/query/bounds` | `{min_x_m, min_y_m, max_x_m, max_y_m}` | `bounds_query`（相交 lanelets/areas 摘要） |
  | `GET /map/lanelet/{id}` | — | `lanelet`（属性/边界/中心线） |
  | `GET /map/lanelet/{id}/following` \| `/preceding` | — | `lanelet_relations`（ids） |
  | `GET /map/lanelet/{id}/adjacent?side=left\|right` | — | `lanelet_relations`（ids） |
  | `GET /map/area/{id}` | — | `area`（外/内环与属性） |
  | `POST /map/query/window` | `{bounds}` 或 `{center, size}` | `map_window`（裁剪几何 + `window_frame`/`transform`） |
  | `POST /map/route` | `{from_id, to_id}` | `route_result`（lanelet 序列与 `length_m`） |
  | `POST /map/grid/driveable`（X-M1） | `{window, resolution_m, request_seq?}` | `grid_snapshot`（`source="driveable"`） |
  | `POST /map/scene/preview`（X-M1） | `{window, resolution_m, obstacles?, request_seq?}` | `grid_snapshot`（`source="planning_scene"`，含 `scene_id`） |

- 栅格/场景端点（X-M1，栅格快照协议 v1，`Docs/expansion/grid_protocol.md`）：
  - `/map/grid/driveable`：按请求窗口调用 `Map::export_driveable_grid`（窗口不裁剪，
    地图外为占据；从不产生 unknown）；单元预算 16,777,216，超限按 2 的幂降级分辨率并
    置 `degraded=true`（`budget_cells` 回显），32 次倍增后仍超限返回
    400 `invalid_input`/`grid_too_large`。响应不含 `scene_id`/`scene_version`。
  - `/map/scene/preview`：窗口先裁剪到地图边界（空交集 400 `window_outside_map`），
    再调用 `planner::scene` 的显式窗口组合（可行驶域基础 + 障碍叠加，与规划服务
    同一实现）；超限 400 `window_too_large`。`scene_id` 由场景库按协议 §5 规范
    序列化（键排序、定点 9 位小数、障碍按 id 排序）计算 SHA-256；`scene_version`
    为 `"scene_v1"`。
  - 两端点请求/响应使用公共包络；错误体为 `{"status":"invalid_input","reason":...}`，
    `reason` 为协议 §8 稳定码（`parse_error`、`missing_field`、`type_error`、
    `non_finite_value`、`invalid_window`、`empty_window`、`invalid_resolution`、
    `invalid_scene`、`window_outside_map`、`grid_too_large`、`window_too_large`；
    内部错误 500 `internal_error`。未使用 413。
  - `window`、分辨率与障碍坐标按协议 §5.1 量化（1e-9）后作为唯一有效输入，快照
    回显量化后的有效值；`request_seq` 原样回显（协议 §6 陈旧拒绝）。
  - 链接集：`planner_map_service` 传递链接 `planner_scene`/`planner_collision`
    （+ `planner_grid`），**不链接** `hybrid_astar`/`dftpav`/`rs` 等算法组件
    （证据：`Src/map/CMakeFiles/planner_map_service.dir/link.txt`）。
  - `PLANNER_BUILD_SCENE=OFF` 的缩减构建仍编译服务（既有端点不变），两个栅格
    端点返回 500 `internal_error`，组件 CMake 给出警告；`PLANNER_MAP_WITH_SERVICE=OFF`
    时不构建服务、不获取 httplib（X-A1-08）。

- 测试：`planner_map_service_tests`（label `map`）以 `--port 0` 启动真实服务
  进程，用 httplib 客户端逐端点断言与库内结果一致（选点命中、窗口裁剪与
  变换、路由、400/404 错误、未知路径）；`planner_map_grid_service_tests`
  覆盖 X-M1 栅格/场景端点（协议 P1–P11 服务侧语义、base64 长度自检、
  降级 k=2/k=3、guard 耗尽、v6 P11/P13 冻结 `scene_id` 与独立逐格复算）。
- `--run-dir` 提供时写入 `service_status.json`（端口、地图身份、启动时间），
  供运行证据引用。

## 8. 当前边界与遗留

- 首版只导入 `.osm`（不构建 Python 绑定，不提供 `.bin` 公开导入接口；内部
  仍编译 BinHandler 以保持 io 完整）。
- 空间查询/裁剪已用图层 R-tree 筛选候选，但裁剪仍按候选逐个精确裁剪；
  large（100 000 lanelet）规模在 v4 及更早 specs 中为 `pending_generation`
  （未以缩小输入替代）；v5 起已确定性物化并按 §9 归档
  （`artifacts/data/sha256/<sha256>/`，specs `archive` 字段引用），正式基准
  默认包含 small+medium+large。
- 路由图首次查询时懒构建（构建耗时不计入单独的“构建”批，含在首个拓扑查询
  内）；构建失败以 `invalid_input` + 原因返回。
- 服务端地图访问由单互斥锁串行化（懒路由图非并发安全）；W2 本地交互规模
  足够，后续如需并发可预构建路由图并改用只读共享。
- 本波次构建/测试/基准/服务使用独立目录 `build/map`（共享 `build/release`
  曾因其他组件进行中的编译错误不可用）；入口命令均带 `--build-dir build/map`，
  若共享目录可用亦可省略。
- 示例地图与 M4 快照已由协调者晋升为版本化夹具
  `tests/fixtures/v3/map/`（来源与哈希见其 README）；组件内工作副本保留。
- v4 及更早的 `pending_generation` 标签在物化前不得以缩小输入沿用；v5 起
  large 已按原标签（100 000 lanelet、原规则）生成并归档，正式比较使用 v5。
- 可行驶域首版只含 lanelet 车道（area 对象不参与）；服务场景域以此为栅格基础，
  请求障碍叠加为占据，起点/终点必须落在可行驶域单元内（见
  `Src/service/README.md` §6）。
- X-M1 栅格/场景预览端点在 `PLANNER_BUILD_SCENE=OFF` 的缩减构建中不可用
  （500 `internal_error`）；默认配置（components.json map 条目）启用 scene。
  端点路径的 `window_too_large` 在小型地图上因窗口先裁剪到地图边界而不可达
  （协议 §4 已记录）；场景库层面的 guard 路径由服务单元测试覆盖。
