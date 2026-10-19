# tests/fixtures/v3/map：M4 地图快照与示例 OSM（版本化夹具）

本集合把地图组件的工作副本与 M4 公共快照**晋升为版本化夹具**，供浏览器工作台 W1–W4（地图显示、查询交互、场景编辑、固定结果回放）直接消费。数据为只读；变更必须形成新的夹具版本。

## 文件与哈希

| 文件 | 大小 | SHA256 | 说明 |
| --- | --- | --- | --- |
| `mapping_example.osm` | 594,826 B | `11561c763ed50322185139ec2dd041d6da72e1bd4e5fc3bb66f6b340d92c088f` | 官方 Lanelet2 `mapping_example.osm` 工作副本（与锁定归档逐字节一致） |
| `mapping_example.snapshot.json` | 436,381 B | `fa1c5ed1aeef73c14b2f698460f54622216eee0d4e2522629d96396cd17d8b62` | 上述 OSM 经 `planner_map_info --export-json` 生成的 M4 公共地图快照（`type: "map_snapshot"`，`schema_version: 1`） |

两个哈希已用 `sha256sum` 复核，并与源文件（`Src/map/tests/data/`、`Src/map/examples/data/`）一致；v3 的 `fixture_manifest.sha256.json` 统一校验本集合全部文件。

## 来源与许可证

- OSM 来源：`lanelet2_maps/res/mapping_example.osm`，来自锁定归档 Lanelet2 **1.2.3**（`dependencies.lock.json`，归档 SHA256 `7617046cdb4aa8893df1f4c396051903a3f2d9783fd7dad434f5b084bfb47dac`）；上游许可证 **BSD-3-Clause**（`Lanelet2-1.2.3/LICENSE`）。本地副本与归档内文件逐字节一致。
- 晋升来源：`Src/map/tests/data/mapping_example.osm`（组件测试工作副本）与 `Src/map/examples/data/mapping_example.snapshot.json`（M4 快照样例）；本集合按字节复制，未做任何修改。源目录中的文件仍为组件工作副本，其说明见各自 README。
- 投影原点（WGS84，快照 `coordinate_system.origin`）：lat `49.00345654351`、lon `8.42427590707`（投影 `utm`，`ele_m = 0.0`）；地图位于慕尼黑附近。

## 快照字段与校验

- 字段表（权威说明）：`Src/map/examples/data/README.md` —— `map_snapshot`（schema_version 1）的顶层字段、`coordinate_system`、`lanelets[]`、`areas[]`、`regulatory_elements[]`、`Point` 定义及语义/校验规则（排序稳定性、只读版本语义、未知字段容忍等）。
- `map_id = "mapping_example"`；`map_version` = 源 OSM 文件字节 SHA256（等于上表 `mapping_example.osm` 的哈希，即 `11561c76…`）；消费方必须以同一版本键查询与显示，地图更新后旧结果不得覆盖当前结果。
- 样例统计（快照 `counts`）：lanelets 371、areas 76、regulatory_elements 9、points 2258、line_strings 1140、polygons 0；`bounds_m` 为 x∈[-899.49, 2526.14]、y∈[-185.26, 855.84]（局部米制）。
- 快照不含拓扑（邻接/路由）结论；拓扑查询由地图组件公共接口按需返回。
- 重新生成命令（组件构建产物存在时；**本阶段未执行**）：
  `./build/map/Src/map/planner_map_info Src/map/tests/data/mapping_example.osm --origin-lat 49.00345654351 --origin-lon 8.42427590707 --export-json Src/map/examples/data/mapping_example.snapshot.json`

## 消费方与约定（Web W1–W4）

- W1 地图显示：加载 `mapping_example.snapshot.json`，按 `lanelets[]`（左/右边界、可选中心线）、`areas[]` 绘制；坐标系为局部米制 `frame_id = "map"`（x 东、y 北），缩放与比例尺以 `bounds_m` 为准。
- W2 查询交互：对象选择与 ID/属性查看使用快照字符串 ID；局部窗口与路由查询结果必须携带同一 `map_id`/`map_version`。
- W3 场景编辑：编辑内容作为覆盖层，不改写本夹具文件；保存/重载引用本快照。
- W4 结果回放：`playback/` 固定结果样例与本快照无拓扑关联，仅共用显示坐标约定；回放数据自身标记 `synthetic: true`。
- 字段权威顺序：快照字段以 `Src/map/examples/data/README.md` 为准；公共序列化包络规则以 `serialization/README.md` 为准；本 README 只负责来源、哈希与消费约定。

## 冻结与变更

- 变更不做就地修改：更新地图数据必须形成新夹具版本（如 v4），重新 `fixtures freeze/verify`，并同步更新 `index.json` 与本 README 的修订说明。
- 本阶段除 `fixtures freeze/verify`（哈希清单）外未执行任何地图加载、渲染或查询。
