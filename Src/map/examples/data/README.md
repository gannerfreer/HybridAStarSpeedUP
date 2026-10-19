# Src/map/examples/data — 公共地图快照样例（M4）

本目录保存 `planner_map_info --export-json` 生成的**公共 JSON 地图快照**样例。
快照是地图组件的文档类型（`type: "map_snapshot"`），供浏览器/显示工作台直接
消费，无需解析 Lanelet2 原生对象；它遵循公共序列化契约的包络规则
（`schema_version`/`type`、对象 ID 一律字符串、枚举字符串、数值有限、禁止
NaN/Infinity），但**不在** `tests/fixtures/v1/serialization/` 的公共类型表内
（该表由集成负责人维护，地图快照字段以本文为准）。

| 文件 | 说明 |
| --- | --- |
| `mapping_example.snapshot.json` | 官方 `mapping_example.osm` 工作副本的完整快照样例；436 381 B，sha256 `fa1c5ed1aeef73c14b2f698460f54622216eee0d4e2522629d96396cd17d8b62` |

样例不是冻结夹具：是否晋升到 `tests/fixtures/` 由协调者决定。重新生成命令：

```sh
./build/map/Src/map/planner_map_info \
  Src/map/tests/data/mapping_example.osm \
  --origin-lat 49.00345654351 --origin-lon 8.42427590707 \
  --export-json Src/map/examples/data/mapping_example.snapshot.json
```

## 1. 字段表（map_snapshot，schema_version 1）

顶层（✔ 必填，○ 可选）：

| 字段 | JSON 类型 | 说明 |
| --- | --- | --- |
| `schema_version` | integer | ✔；唯一合法值 `1` |
| `type` | string | ✔；恒为 `"map_snapshot"` |
| `map_id` | string | ✔；源文件名 stem |
| `map_version` | string | ✔；源 OSM 文件字节的 SHA-256 十六进制（不透明版本串，按相等比较） |
| `source` | object | ✔；`{path, bytes}`：导入路径与文件字节数 |
| `coordinate_system` | object | ✔；坐标系与投影原点，见下 |
| `bounds_m` | object | ✔；点层包围盒 `{min_x_m, min_y_m, max_x_m, max_y_m}` |
| `counts` | object | ✔；`{points, line_strings, polygons, lanelets, areas, regulatory_elements}` 整数 |
| `traffic_rules` | object | ✔；`{location, participant}`（路由/邻接查询配置） |
| `lanelets` | array | ✔；lanelet 数组，按数字 ID 升序 |
| `areas` | array | ○；区域数组（`include_areas=false` 时缺省），按数字 ID 升序 |
| `regulatory_elements` | array | ○；交通规则元素（`include_regulatory_elements=false` 时缺省），按数字 ID 升序 |

`coordinate_system`：

| 字段 | 值 | 说明 |
| --- | --- | --- |
| `frame_id` | `"map"` | 局部米制地图系 |
| `unit` | `"m"` | 长度单位 |
| `x_axis` / `y_axis` | `"east"` / `"north"` | 右手系；注意这不是 `planner::common::Pose2d` 的车辆系（x 前/y 左） |
| `projection` | `"utm"` \| `"mercator"` \| `"spherical_mercator"` | 导入时显式配置的投影 |
| `origin` | `{lat_deg, lon_deg, ele_m}` | 投影原点（WGS84） |

`lanelets[]`：

| 字段 | JSON 类型 | 说明 |
| --- | --- | --- |
| `id` | string | ✔；原始 OSM relation ID（十进制字符串，保精度） |
| `attributes` | object<string,string> | ✔；OSM 标签原样保留（不自动转数值） |
| `left` | array<Point> | ✔；左/北边界，点序与地图一致 |
| `right` | array<Point> | ✔；右/南边界 |
| `centerline` | array<Point> | ○；几何中心线（`include_centerline=false` 时缺省） |

`areas[]`：`id`、`attributes`、`outer`（array<Point>，多段线串接的外环）、
`inner`（array<array<Point>>，每个洞环一项）。

`regulatory_elements[]`：`id`、`attributes`（含 `type`/`subtype`）、
`roles`（object：角色名 → 被引用对象 ID 字符串数组，如 `refers`、`ref_line`）。

`Point`：`{id: string, x_m: number, y_m: number, z_m: number}`。来自地图的点
保留原始 OSM 节点 ID；中心线点携带 Lanelet2 计算出的 ID。本快照不含裁剪合成点
（裁剪结果由 `CropResult` 表达，合成点 ID 为空字符串）。

## 2. 语义与校验

- 所有数值有限；double 以最短往返表示保存，解析后与生成值精确相等。
- 对象 ID 与枚举均为字符串；未知字段按公共契约容忍并忽略，`schema_version`
  不等于 1 或缺失必填字段应被消费者拒绝。
- 快照是**只读地图版本**：`map_id`/`map_version` 与查询结果必须使用同一版本。
- 数组排序（lanelet/area/regulatory_element 按数字 ID 升序）保证同一地图
  重复导出字节稳定；边界点序按地图存储顺序保留。
- 快照不包含拓扑（邻接/路由）结论；拓扑查询由 `Map` 公共接口按需返回。
