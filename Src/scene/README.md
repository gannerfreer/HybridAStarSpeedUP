# planner_scene（地图窗口 + 可行驶域 + 用户障碍的场景组合与快照）

场景组合与快照库（`planner::scene`，X-A1-18 从 W5 服务场景构建器迁移，X-M1
增加内容标识与快照组装）：由请求几何推导本地窗口，以地图 lanelet 可行驶域为
栅格基础，叠加请求障碍，产出不可变的 `common::GridView` + `collision::Scene`
与内容标识 `scene_id`/`scene_version`。依赖 map/grid/collision 的公共接口；
不依赖规划器、HTTP 或显示。

当前状态：测试 6/6（标签 `scene`，只读 v3 `mapping_example.osm` 夹具）；
被地图服务（栅格/场景预览端点）与规划服务（规划输入快照）消费；无基准套件。
组合规则版本 `scene_v1`，栅格快照协议 `grid_protocol_version="1.0"`
（[`Docs/expansion/grid_protocol.md`](../../Docs/expansion/grid_protocol.md)）。

## 目录

```text
Src/scene/
  CMakeLists.txt                       组件构建（planner_scene；依赖 map/grid/collision）
  include/planner/scene/
    scene.hpp                          组合入口：SceneBuildOptions/Result、build_scene、build_scene_preview
    content_id.hpp                     量化、定点格式、规范序列化与 scene_content_id（SHA-256）
    snapshot.hpp                       grid_snapshot 组装与 base64_u8 单元编码
  src/
    scene.cpp                          窗口推导、可行驶域基础、障碍叠加、预算降级、稳定错误码
    content_id.cpp                     量化/定点格式/规范 JSON/SHA-256（FIPS 180-4）
    snapshot.cpp                       base64 编码与协议 §2.1 文档组装
  tests/
    test_scene.cpp                     GoogleTest（标签 scene，6 例；v3 地图夹具）
```

## 公开接口

- `scene.hpp`
  - `SceneBuildOptions{resolution_m=0.1, margin_m=5.0, max_cells=16×1024×1024}`；
  - `SceneBuildResult{ok, reason, detail, grid, scene, window,
    requested_resolution_m, effective_resolution_m, scene_id, scene_version}`；
  - `build_scene(map, vehicle, start, goal, obstacles, options)`：面向规划请求的
    窗口自动推导入口（起点/终点必须在地图边界内且落在可行驶域单元上）；
  - `ScenePreviewOptions{window, resolution_m=0.1, max_cells=16M}` 与
    `build_scene_preview(map, obstacles, options)`：显式窗口预览入口（协议 §3.3，
    不做起点/终点校验）。
- `content_id.hpp`
  - `kSceneVersion = "scene_v1"`（组合规则版本，`scene_id` 的哈希输入）；
  - `quantize_scene_value(value)`：四舍五入（round half away from zero）到 1e-9
    的整数倍，`-0` 归一化为 0；
  - `format_scene_fixed(value)`：恰好 9 位小数的定点格式（无指数、无前导 `+`）；
  - `canonical_scene_input(...)` / `scene_content_id(...)`：协议 §5.2 规范输入
    的规范 JSON 文本与 SHA-256（64 位小写十六进制）。
- `snapshot.hpp`
  - `kGridProtocolVersion = "1.0"`；
  - `encode_cells_base64(cells | grid)`：RFC 4648 §4 标准字母表 + `=` 填充，单元
    值必须 ∈ {0,1,2}，否则抛 `std::invalid_argument`；
  - `GridSnapshotOptions{source, scene_id, scene_version, window,
    requested_resolution_m, budget_cells, request_seq}` 与
    `make_grid_snapshot_json(grid, options)`：组装协议 §2.1 的 `grid_snapshot`
    文档（`degraded` 由 effective/requested 派生，`request_seq` 可选回显）。

## 组合规则 `scene_v1`（`build_scene`）

1. **窗口推导**：取起点、终点与全部障碍 AABB 的包围盒，按
   `margin_m + 车辆外接圆半径` 扩展（外接圆由车身后悬/轴距+前悬/半宽与
   `safety_margin_m` 推导，保证起终点整车足迹落在网格内），再裁剪到地图边界；
   空交集失败 `window_outside_map`。
2. **量化**：窗口与分辨率按协议 §5.1 量化（1e-9）后作为唯一有效输入，快照与
   `scene_id` 均以量化值为准；量化后窗口退化为空同样报 `window_outside_map`。
3. **可行驶域基础**：`Map::export_driveable_grid(窗口, 有效分辨率)`——与任一
   lanelet 可行驶多边形相交（闭集，接触也算）的单元为空闲，其余为占据；
   Lanelet2 area 对象不参与（地图组件边界）。
4. **预算降级**：请求分辨率无效（非正）时回落到 0.1 m；单元数超 `max_cells`
   时按 2 的幂上调（最多 32 次），仍超限失败 `window_too_large`；导出失败映射为
   `window_too_large`（`grid_too_large`）或 `invalid_scene`（其余）。
5. **起终点可行驶校验**：起点/终点必须落在可行驶域单元上，否则
   `start_outside_driveable` / `goal_outside_driveable`；该检查先于障碍物栅格化，
   因此“障碍物压在起点上”仍按既有 Hybrid 阶段 `invalid_input` 上报。
6. **障碍叠加**：障碍坐标量化后栅格化——单元格中心位于闭合多边形内（含边界）
   记为占据；顶点数 < 3 的障碍跳过栅格化。精确凸多边形同时保留在
   `collision::Scene` 中（网格势场与整车碰撞检查都能感知）；`collision::Scene::valid`
   失败报 `invalid_scene`。
7. **结果**：成功后填充 `grid`/`scene`/有效窗口与分辨率/`scene_id`/
   `scene_version="scene_v1"`；失败结果 `scene_id`/`scene_version` 为空，
   window/分辨率字段可能仍已填充。稳定失败码：`start_outside_map`、
   `goal_outside_map`、`start_outside_driveable`、`goal_outside_driveable`、
   `window_outside_map`、`window_too_large`、`invalid_scene`。

## 场景预览（`build_scene_preview`）

- 请求窗口必须有限（`invalid_window`）且 `min < max`（`empty_window`），随后
  裁剪到地图边界（不相交 `window_outside_map`）并量化；不校验起点/终点
  （起终点不参与预览栅格内容）。
- 组合尾部与 `build_scene` 完全共用（可行驶域基础 + 障碍叠加 + 校验 + 标识），
  失败码为 `invalid_window`、`empty_window`、`window_outside_map`、
  `window_too_large`、`invalid_scene`。
- 地图服务预览不承诺保留快照（内容寻址、可确定性重生成）；规划服务保留
  （见下）。

## 内容标识（`scene_id`）

- **规范输入字段**（协议 §5.2，键按 Unicode 码点升序、无多余空白、定点 9 位
  小数、字符串最小 JSON 转义）：`domain="planner.scene.content.v1"`、
  `scene_version`、`map_id`、`map_version`、`window`、`resolution_m`
  （有效分辨率）、`width`、`height`、`origin`、`obstacles`。
- **进入标识**：地图身份、有效窗口、有效分辨率、尺寸、原点、障碍（含 `id`）、
  组合规则版本、域分隔串；**不进入标识**：`requested_resolution_m`、`degraded`、
  `request_seq`、`budget_cells`、`cells`、`source`——它们描述“请求”或“表示”。
  因此同一产出内容只应有一个标识：预览与规划在有效分辨率相同时得到同一
  `scene_id`，即使请求分辨率不同。
- **障碍规范化**：4 顶点且边轴对齐（量化后精确）的障碍按 `rectangle` 形式输出
  （取顶点包围盒 `x_min/x_max/y_min/y_max`），其余按 `convex_polygon` 输出
  （顶点顺序沿用 collision 的规范化顺序，不做循环旋转归一化）；障碍按 `id`
  升序、`id` 相同按规范字节序排列。矩形与等价 4 顶点多边形共享同一标识、
  顶点起点不同会改变标识，均属文档化边界（协议 §5 审阅点）。
- `scene_content_id` = 规范 JSON 的 SHA-256，64 位小写十六进制；实现为
  FIPS 180-4 标准 SHA-256（场景库不反向依赖地图组件内部头）。量化步长 1e-9 m：
  低于该步长的输入差异被归并，任一 ≥1e-9 的变化得到不同标识（基于 SHA-256
  碰撞假设）。

## 预览↔规划一致性（P11）

- 协议 §3.3 语义 5：预览与规划快照**逐格一致**的条件是地图版本、有效窗口、
  有效分辨率、障碍集合与组合规则版本一致；起点/终点可行驶校验不改变单元值，
  预览不做该校验也不影响一致性。
- 统一实现保证跨服务一致：`scene_id` 只由本库计算，地图服务
  `POST /map/scene/preview` 与规划服务的场景构建调用同一组合与同一标识函数。
- 验收证据（X-M1）：协议 §5.3 冻结示例复算 `scene_id=20821dc8…e776`；v6 P11
  冻结 `scene_id=044007e5…` 逐格相等；服务级 `PreviewMatchesPlanSnapshotCellByCell`
  （cells 解码逐字节相等）。

## 消费关系（两服务）

- **地图服务**（`Src/map/src/service_main.cpp`）：`POST /map/grid/driveable`
  （`source="driveable"`，不含 scene 字段）与 `POST /map/scene/preview`
  （`source="planning_scene"`，含 `scene_id`/`scene_version`）；`/health` 增加
  `grid_protocol_version` 与 `scene_rules_version`。服务不依赖 Hybrid A*/Dftpav。
- **规划服务**（`Src/service`）：库内 `planner::service::build_scene` 委托本库
  （仅做 `ServiceObstacle → collision::Obstacle` 类型转换，公开类型与错误码不变，
  新代码应直接用 `planner::scene`）；场景快照为进程内 LRU（16、按 `scene_id`
  去重、重启失效），`GET /plan/scene/{scene_id}` 命中按保存原文逐字节返回；
  status/result 在场景构建后（成功与失败）携带 `scene_id`/`scene_version`。

## 复现命令

```sh
python3 tools/task_runner.py build --component scene --profile release --parallel 4
python3 tools/task_runner.py test --component scene --workers 2
```

入口按 `tools/components.json` 同时开启 `PLANNER_BUILD_SCENE/MAP/GRID/COLLISION`；
测试只读夹具 `tests/fixtures/v3/map/mapping_example.osm`（投影原点由测试 CMake
固化）。`scene` 无注册基准套件。

## 已知限制

1. 可行驶域基础只含 lanelet 车道，Lanelet2 area（停车场等）不参与；单元语义为
   “与车道多边形相交即空闲”，精确约束由碰撞检查（占据单元 + 精确障碍多边形）
   承担。
2. 窗口限制在起点/终点/障碍包围盒外扩边距后裁剪到地图边界，不会扩展到整张
   地图路网。
3. `window_too_large` 在小型地图的服务端点路径不可达（窗口先裁剪到地图边界）；
   库级 guard 路径由单元测试覆盖（X-M1 遗留）。
4. `PLANNER_BUILD_SCENE=OFF` 的缩减构建中地图服务栅格/场景端点返回 500
   `internal_error`（默认配置均启用 scene）。
5. 规划服务快照为进程内内存 LRU 16、按 `scene_id` 去重、重启失效；容量档
   （2048²/4096²）不进快照（在线范围 ≤1024²）。
6. 障碍归一化与 1e-9 定点量化边界：等价矩形/4 顶点多边形共享 `scene_id`，
   低于量化步长的输入差异被归并（文档化边界）。
7. 预览↔规划一致性的前提（同地图版本/有效窗口/有效分辨率/障碍集合/`scene_v1`）
   必须同时满足；`scene_version` 变化会使全部标识变化，客户端应以 `scene_id`
   比较内容。
8. 无基准套件；端点冒烟不构成性能证据。

## 报告

- 提取与验收（X-A1-18，6/6）：[`Docs/reports/expansion/X-A1.md`](../../Docs/reports/expansion/X-A1.md)
  （§3.2 独立构建/测试：build `fcbc0348`、test `9dbd311f`、全量回归 `dff25082`；
  首次构建 `ab7df3b1` 失败证据保留，重试后成功）。
- 标识/快照与预览一致性（X-M1，scene 6/6 原样）：[`Docs/reports/expansion/X-M1.md`](../../Docs/reports/expansion/X-M1.md)
  （§3.2 P11 五项一致性证据；run `666fc2b0`）。
- 协议与字段清单：[`Docs/expansion/grid_protocol.md`](../../Docs/expansion/grid_protocol.md)（§2/§3.3/§5/§7）。
- 问题跟踪：[`Docs/reports/ISSUES.md`](../../Docs/reports/ISSUES.md)（无 `scene-*` 条目）。
