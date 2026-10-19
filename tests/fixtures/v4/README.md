# tests/fixtures/v4：E0 固定夹具（C 组交付部分，v4 修订版）

本目录保存跨组件复用的版本化测试输入。当前文件全部为手写 JSON；**除 `fixtures freeze/verify`（哈希清单校验）外，本阶段未执行任何加载或验证**，语法与几何自洽性经人工逐文件检查与一次性只读复算脚本核对。执行计划见 `Docs/component_development_plan.md`。

## v4 修订说明（相对 v3，2026-10-11）

v4 = v3 全量复制（不含 v3 清单）+ 本表修订；v1–v3 目录保持冻结不动。修订依据：ISSUES `fixtures-002`（地图基准 specs 字段矛盾，复查条件：fixture_version 升版重跑）与 `fixtures-003`（v3 playback 索引 notes 陈旧路径，改动需重 freeze）。

| 文件 | v4 变更 | 原因 |
| --- | --- | --- |
| `benchmarks/map/specs.json` | `lanelet_length_m` 20.0 → 25.0；`geometry_rule` 的 x 区间同步为 `[column*25.0, column*25.0+25.0]`；`fixture_version` 升为 `v4`；notes 记录修订 | 原 v1–v3 文本 `lanelet_length_m=20` 与 `x_pitch_m=25` 自相矛盾（Lanelet2 后继检测要求相邻 lanelet 共享端点对象）；生成器一直按 `x_pitch` 共享端点解释并在 `.meta.json`/`bench_summary.json` 记录，本版将该解释固化为规范文本 |
| `playback/index.json` | notes 中 `tests/fixtures/v1/playback/` → `tests/fixtures/v4/playback/` | 索引自 v1 复制以来路径表述陈旧；样例文件与 SHA256 不变 |
| `index.json`、`README.md` | 增加 v4 修订说明 | 版本化修订记录 |
| 其余文件 | 与 v3 逐字节一致（含 `map/`、`vehicles/`、`paths/`、`collision/`、`serialization/`、`grids/`、`benchmarks/` 其余文件） | 避免无关变更 |

消费方迁移：`benchmarks/map` → `planner_map_bench`（v4 起优先读取，v1/v2 保留回退）；`playback` → Web W4 回放（v3 → v4）。`map/`、`vehicles/` 等未修订集合的消费方（Web W1–W3、map/service 命令）维持在 v3。

## v3 修订说明（相对 v2，2026-10-10）

v3 = v2 全量复制（不含 v2 的 `fixture_manifest.sha256.json`）+ 晋升地图集 `map/`；v1、v2 目录保持冻结不动。晋升依据：M4 公共快照与官方示例 OSM 已稳定，协调者批准将其从组件工作副本（`Src/map/tests/data/`、`Src/map/examples/data/`）晋升为版本化夹具，供 Web W1–W4 消费。

| 文件 | v3 变更 | 原因 |
| --- | --- | --- |
| `map/mapping_example.osm` | 新增（594,826 B，SHA256 `11561c763ed50322185139ec2dd041d6da72e1bd4e5fc3bb66f6b340d92c088f`） | 官方 Lanelet2 1.2.3 `lanelet2_maps/mapping_example.osm`（BSD-3-Clause）的字节级工作副本，供导入/显示回归 |
| `map/mapping_example.snapshot.json` | 新增（436,381 B，SHA256 `fa1c5ed1aeef73c14b2f698460f54622216eee0d4e2522629d96396cd17d8b62`） | 上述 OSM 的 M4 `map_snapshot` 导出（schema_version 1，`map_version` = OSM 字节 SHA256），供 W1–W4 直接消费 |
| `map/README.md` | 新增 | 来源与许可、投影原点（UTM lat 49.00345654351 / lon 8.42427590707）、哈希、字段指引与消费约定 |
| `index.json`、`README.md` | 增加 map 集与 v3 修订说明 | 版本化修订 |
| 其余文件 | 与 v2 逐字节一致（含 `serialization/`、`playback/`、`paths/`、`collision/`、`benchmarks/`、`vehicles/`、`grids/`） | 避免无关变更 |

消费约定（Web W1–W4）：快照字段以 `Src/map/examples/data/README.md` 为准；`map_id = "mapping_example"`、`map_version` = OSM 字节 SHA256；查询/显示/回放必须携带同一 `map_id`/`map_version`，地图更新后旧结果不得覆盖当前结果；场景编辑内容作为覆盖层，不改写夹具文件。详见 `map/README.md`。

## v2 修订说明（相对 v1，2026-10-10）

v2 = v1 全量复制 + 本表修订；v1 目录保持冻结不动，E0-5 的既有证据不受影响。修订依据：Hybrid A* Wave 4 实测（run `20261009T175408170666Z_hybrid_astar_28821807` 与 `20261009T174607194255Z_hybrid_astar_5729bd98`）发现的 6 例 `fixture_conflict`（含 0.1 m 安全裕量的终点足迹碰撞或出界）与 8 例 `unresolved`（起点终点无碰撞但搜索穷举 no_path），以及 §7.1“修改基准数据必须形成新版本”的要求。

| 文件 | v2 变更 | 原因 |
| --- | --- | --- |
| `paths/cases.json` | 修正 14 例（保持六类×5 + 4 边界例与 case id 不变）；14 例 `grid.map_version` 升为 `v2`；扩展 `self_check.v2_revision` | 6 例 fixture_conflict 的终点/障碍/网格在 0.1 m 安全裕量语义下碰撞或出界；8 例 unresolved 的停车/死胡同几何对 4.9 m × 2.0 m 包络不可行 |
| `collision/random_spec.json` | 仅文本澄清位姿层 `reverse_motion` 归属（`pose_concentrated_breakdown_note`）；`fixture_version` 升为 `v2` | `reverse_motion` 只属运动段检查；位姿集中样本由其余 4 类覆盖。不改任何计数、种子或分布参数 |
| `README.md`、`index.json` | 注明 v2 修订清单与原因 | 版本化修订说明 |
| 其余文件 | 与 v1 逐字节一致（含 `serialization/` 冻结契约副本与 `playback/` SHA256 索引） | 未发现缺陷；避免无关变更 |

### 14 例修订与可行性论证摘要

- **fixture_conflict 修复**：`corridor_112` 终点 x 23.0→21.5 m；`park_117` 目标 y 5.0→4.5 m、车位按倒车 S 形扫掠几何扩为 9.0 m（6.6 m 下会擦碰前车左下角）、起点移至 (10.0,2.5)；`park_120` 目标与后墙冲突修复并统一为可行模板；`unreachable_126/129/130` 终点分别移至 x 15.2/14.6/15.2 m，使含裕量足迹完全落在腔内且与墙体不相交（封闭性不变）。
- **unresolved 重设计**（4.9 m × 2.0 m 包络，R = wheelbase/tan(max_steer) ≈ 5.125 m）：
  - `park_116/118/119/120`：口部 y=8.8 m；构造路径 = 前进至 (15.125,2.475) → 倒车四分之一圆弧至 (10,7.6,-90°) → 直线倒车入位（5.1 m；119 为 7.9 m）。圆弧包络最低点 y=0.392 m、顶点 y_max=8.7 < 8.8；库宽 2.6～3.2 m（单侧 0.3～0.6 m）。
  - `deadend_121～125`：口部 y=10.4 m；构造路径 = 沿袋形轴线直线倒车至 (7.5,6.4,+90°) → 倒车四分之一圆弧至 (2.375,1.275,0°) → 前进 2 m 至目标。圆弧包络最低点 y=0.177 m、顶点 y_max=10.2 < 10.4；袋形净宽 2.6～3.0 m。
- 上述几何为手写构造与逐段包络手工核算，**未运行规划器**；构造路径的存在性为几何论证，不构成 Hybrid A* 实测通过。若 Hybrid 侧仍因离散化/裕量化语义无法求解，属组件调优问题（见遗留风险），不得放宽安全裕量。

## 目录结构

| 路径 | 内容 | 状态 |
| --- | --- | --- |
| `index.json` | 夹具索引：集合清单与说明；不内嵌哈希 | E0 完备 |
| `vehicles/default.json` | 默认测试车辆（§7.1 基准参数，不代表实车） | E0 完备 |
| `grids/minimal.json` | GridView 最小样例（20×20、0.1 m，含 occupied/unknown 与边界情形） | E0 完备 |
| `serialization/` | 公共类型 JSON 契约的有效/无效样例与字段说明 | 由公共序列化负责人（B 组）维护；冻结契约已发布，本组字段已按其对齐 |
| `paths/cases.json` | 路径场景：六类 × 5 例（seed 101～130）+ 4 个非法/边界用例（seed 201～204），共 34 例 | v2 修订版：14 例已修复（见上）；算法专用细化（如终点碰撞派生例）在对应组件验收前补齐（§5.1 E0-5） |
| `collision/random_spec.json` | 碰撞随机差分规范：10,000 / 100,000 例、种子 0～99、分布、用例 ID、独立参考 | v2 文本澄清：位姿层 `reverse_motion` 归属（不改计数）；实际样本由生成器按规范产出，不保存在本目录（大样本与证据进 `artifacts/runs/<run_id>/`） |
| `benchmarks/bspline/inputs.json` | B-spline 求值/导数、构造、等弧长采样基准输入与批大小 | E0 输入完备（8/32/128 控制点、显式节点向量与参数域） |
| `benchmarks/polynomial/inputs.json` | 多项式构造/求值基准输入与边界条件规则 | E0 输入完备（三次/五次、1,000 段、10,000 时间点） |
| `benchmarks/map/specs.json` | 合成地图 1,000 / 10,000 / 100,000 lanelet 规格与查询/裁剪/路由/加载基准 | 小、中规模为确定性生成规则；100,000 规模标注“待生成”并按 §9 归档策略 |
| `playback/index.json` | W4 固定回放索引：类型、schema 版本、地图版本、相对文件名、SHA256、预期显示行为 | E0 完备 |
| `playback/*.json` | 地图快照、RS/Hybrid A* 路径（含失败/换向）、B-spline、多项式、走廊、Dftpav 轨迹、碰撞、过期请求样例 | E0 样例完备；全部 `synthetic: true` |
| `map/mapping_example.osm` | 官方 Lanelet2 1.2.3 示例 OSM（BSD-3-Clause，594,826 B） | v3 晋升；SHA256 与来源见 `map/README.md` |
| `map/mapping_example.snapshot.json` | M4 公共地图快照（`type: "map_snapshot"`，schema_version 1，436,381 B） | v3 晋升；字段表见 `Src/map/examples/data/README.md` |
| `map/README.md` | 来源/许可/投影原点/哈希/消费约定 | E0 完备 |

## JSON 顶层约定

- 每个 JSON 文件为 UTF-8；顶层含整数 `schema_version: 1` 与字符串 `type`。
- 对象 ID 一律使用字符串；枚举使用字符串；拒绝 NaN/Infinity；数值保存足以往返 double 的精度。
- 单位后缀约定：`_m`（米）、`_rad`（弧度）、`_s`（秒）、`_mps`、`_mps2`、`_inv_m`（1/米，曲率）。
- 必填字段缺失或主版本未知应报错；未知可选字段允许读取，但不得改变已有字段含义。
- 引用其他夹具文件时，除非文件内另有说明，使用以 `tests/fixtures/v3/` 为根的相对路径；`playback/index.json` 中的文件名相对 `playback/` 目录。
- `playback/` 内文件哈希对保存后的原始文件字节计算（sha256sum 格式小写十六进制），不要求不同语言生成相同文本布局。

## 字段命名约定（与 serialization 冻结契约对齐）

以下字段与 `tests/fixtures/v3/serialization/README.md`（B 组已发布的冻结契约）一致；其他夹具与其冲突时，以冻结契约为准（冻结契约自身声明的对账规则）。

- `Pose2d`：`x_m, y_m, yaw_rad`；后轴中心，局部右手坐标（x 向前、y 向左），航向逆时针为正并归一化到 `[-π, π)`（+π 不出现，等价写为 -π）。
- `VehicleModel`：`id, wheelbase_m, width_m, front_overhang_m, rear_overhang_m, max_steer_rad, safety_margin_m`；最小转弯半径 = 轴距 / tan(最大转角)；首版矩形整车足迹（长 = 前悬 + 轴距 + 后悬 = 4.7 m，宽 1.8 m；计入 0.1 m 安全裕量后的包络为 4.9 m × 2.0 m）。
- `GridView`：`map_id, map_version, width, height, resolution_m, origin(Pose2d), cells`（行优先整数数组：0=free，1=occupied，2=unknown）。`origin` 为**栅格原点位姿（cell (0,0) 左下角，ROS OccupancyGrid 约定）**；cell (r,c)（r 行、c 列，行沿 origin 的 +y 递增）中心 = `origin + ((c+0.5)*resolution_m, (r+0.5)*resolution_m)`；unknown 与地图外视为占据。
- `PathPoint`：`pose(Pose2d), s_m, curvature_inv_m, direction`；`s_m` 为该段内累计非负弧长、段内严格递增；曲率定义 `tan(steer)/wheelbase`，满足 `yaw_rate = signed_speed * curvature`。
- `PathSegment`：`points[]`（同向段；换向点开始新段，换向点姿态同时作为前段末点与后段首点，方向字段区分档位）。
- `Direction`：`"forward"` / `"reverse"`。
- `TrajectoryPoint`：`t_s, pose(Pose2d), velocity_mps, acceleration_mps2, curvature_inv_m`；时间严格递增，有符号速度，换向处速度为零；不从零速位置强行计算航向。
- `Corridor`：`id, map_id, map_version, frame_id, polygons[]`；每个 `CorridorPolygon` 为 `s_start_m, s_end_m, vertices[]`（`vertices` 为 `{x_m, y_m}` 数组，≥3 个、逆时针、凸）；有序多边形表示自由空间，整车包含约束由后端检查。
- `PlanRequest`：`request_id, map_id, map_version, vehicle(VehicleModel), start, goal, config, time_budget_s, node_budget, cancelled`（预算缺省 = 无限制）。
- `PlanResult`：`request_id, status, reason, path[]（PathSegment 数组）, trajectory[]（TrajectoryPoint 数组）, statistics`；空数组等价于字段缺失。
- `Status`：`success, invalid_input, no_path, timeout, cancelled, optimization_failed, validation_failed`；资源终止由任务运行器记录，不伪装成算法无解。

### paths 场景字段

- 每例字段：`id, class, vehicle, seed, grid, start(Pose2d), goal(Pose2d), obstacles, expected, notes`；边界例另有 `boundary_kind` 与可选 `request_overrides`。
- `vehicle` 为 `vehicles/default.json` 的 `id`（`default_test_vehicle`），消费方按 `vehicle_ref` 解析整车参数。
- `grid`：`map_id, map_version, resolution_m, origin(Pose2d), width, height`（width/height 为格数，非米）；占据格由 `obstacles` 按“接触即占据”栅格化得到，无 unknown 格。
- `obstacles`：世界坐标、米。矩形 `{"id","type":"rectangle","x_min_m","y_min_m","x_max_m","y_max_m"}`（含边界）；凸多边形 `{"id","type":"convex_polygon","vertices":[{"x_m","y_m"},...]}`。
- `expected`：`{"status":"success"}` 或 `{"status":"failure","failure_reason":"no_path|timeout|invalid_input|cancelled"}`。
- `request_overrides`：映射 `PlanRequest` 字段，目前使用 `cancelled`、`time_budget_s`、`node_budget`。

### collision 规范字段

- 顶层：`regimes`（dev 10,000 / acceptance 100,000）、`strata`（宽随机 + 集中对抗五类：角点擦碰、精确接触、薄障碍、倒车、地图边缘）、`case_id_rule`、`independent_reference`、`inconclusive_samples`、`failure_capture`、`reporting`。
- 样本 ID 规则：`cr-<regime>-s<seed>-<seq>-<kind>`，如 `cr-acc-s07-0342-pose`；`kind ∈ {pose, motion}`。

### benchmarks 字段

- B-spline：`control_groups[]`（`name, control_point_count, knots[], control_points[]`）、`eval_batch`（10,000 等距参数、阶次 0～3）、`construct_batch`（每组 100 条平移变体）、`arc_length_batch`（每组 100 条 × 1,000 点）、`timing_boundaries`。
- 多项式：`boundary_condition_sets.{cubic,quintic}`（各 1,000 段规则）、`construct_batch`、`eval_batch`（10,000 局部时间点、阶次 0～3）、`validation_examples`、`timing_boundaries`；系数按升幂，段内局部时间。
- 地图：`synthetic_map_family`（几何/拓扑/属性规则）、`maps[]`（small/medium/large 规模状态）、`query_batch`（每批 1,000 固定序列）、`crop_batch`（100 个 50 m × 50 m 窗口）、`route_batch`（100 对可达 lanelet）、`load_batch`（30 批）。

### playback 字段

- `playback/index.json`：顶层 `schema_version, type, fixture_version, map_id, map_version, hash_algorithm, hash_scope, samples[], notes`；每个样例含 `id, kind, type, schema_version, map_id, map_version, file, sha256, expected_display, synthetic`。
- 路径/轨迹样例：`type: "plan_result"`（公共 `PlanResult` 字段 + 附加 `map_id/map_version/synthetic/synthetic_note/notes`）。
- 走廊样例：`type: "corridor"`（公共 `Corridor` 字段 + 附加显示元数据）。
- 显示专用类型：`map_snapshot`、`bspline_curve`、`polynomial_curve`、`collision_case`、`stale_result`，字段见各文件与 `playback/index.json` 的 `expected_display`。
- 修改样例内容必须同步更新 `playback/index.json` 的 SHA256 与预期显示行为。

## 状态与待补

- `paths/cases.json`：v2 修正 14 例（6 fixture_conflict + 8 unresolved 重设计）并通过逐例几何自检，见文件内 `self_check.v2_revision`；v1 保持冻结不动。终点碰撞派生例可在组件验收前按 `boundary_201` 的约定派生。
- `collision/`：含 v2 位姿层澄清（仅文本）；10,000 / 100,000 级样本、不确定样本补样与失败实例保存到运行目录，报告写明可判定样本数、真碰撞数、漏检数、误报数与不确定数。
- `benchmarks/map/specs.json`：100,000 lanelet 地图待生成，按 §9 使用 SHA256 归档引用；未生成前不得以缩小输入沿用原比较标签。
- `serialization/`：v1 冻结契约的逐字节副本（B 组维护）；本组路径、轨迹、走廊与栅格字段与其一致。`Src/common/include/planner/common/types.hpp` 的 `origin` 注释现已与冻结契约（cell (0,0) 左下角，ROS 约定）一致，此前的表述差异已闭环。
- `map/`：v3 晋升的只读地图集（官方 OSM + M4 快照）；除 freeze/verify（哈希清单）外未执行地图加载、渲染或查询。
- 本 README 描述的“可加载”“完备”均为交付状态，不代表已经执行过加载或验证；v2、v3 已做 freeze/verify（哈希清单）校验，未运行规划器或地图加载。

## 合成数据声明

`playback/` 下全部文件标记 `synthetic: true`：仅作显示测试，不能作为算法正确性证据，也不代表任何上游实现的实际输出。`paths/`、`benchmarks/`、`collision/` 中的数据为确定性测试输入或生成规范，同样不构成算法正确性结论。
