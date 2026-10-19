# tests/fixtures/v1：E0 固定夹具（C 组交付部分）

本目录保存跨组件复用的版本化测试输入。当前文件全部为手写 JSON；**本阶段未执行任何加载或验证**，语法与几何自洽性经人工逐文件检查。执行计划见 `Docs/component_development_plan.md`。

## 目录结构

| 路径 | 内容 | 状态 |
| --- | --- | --- |
| `index.json` | 夹具索引：集合清单与说明；不内嵌哈希 | E0 完备 |
| `vehicles/default.json` | 默认测试车辆（§7.1 基准参数，不代表实车） | E0 完备 |
| `grids/minimal.json` | GridView 最小样例（20×20、0.1 m，含 occupied/unknown 与边界情形） | E0 完备 |
| `serialization/` | 公共类型 JSON 契约的有效/无效样例与字段说明 | 由公共序列化负责人（B 组）维护；冻结契约已发布，本组字段已按其对齐 |
| `paths/cases.json` | 路径场景：六类 × 5 例（seed 101～130）+ 4 个非法/边界用例（seed 201～204），共 34 例 | E0 完备；算法专用细化（如终点碰撞派生例）在对应组件验收前补齐（§5.1 E0-5） |
| `collision/random_spec.json` | 碰撞随机差分规范：10,000 / 100,000 例、种子 0～99、分布、用例 ID、独立参考 | E0 规范完备；实际样本由生成器按规范产出，不保存在本目录（大样本与证据进 `artifacts/runs/<run_id>/`） |
| `benchmarks/bspline/inputs.json` | B-spline 求值/导数、构造、等弧长采样基准输入与批大小 | E0 输入完备（8/32/128 控制点、显式节点向量与参数域） |
| `benchmarks/polynomial/inputs.json` | 多项式构造/求值基准输入与边界条件规则 | E0 输入完备（三次/五次、1,000 段、10,000 时间点） |
| `benchmarks/map/specs.json` | 合成地图 1,000 / 10,000 / 100,000 lanelet 规格与查询/裁剪/路由/加载基准 | 小、中规模为确定性生成规则；100,000 规模标注“待生成”并按 §9 归档策略 |
| `playback/index.json` | W4 固定回放索引：类型、schema 版本、地图版本、相对文件名、SHA256、预期显示行为 | E0 完备 |
| `playback/*.json` | 地图快照、RS/Hybrid A* 路径（含失败/换向）、B-spline、多项式、走廊、Dftpav 轨迹、碰撞、过期请求样例 | E0 样例完备；全部 `synthetic: true` |

## JSON 顶层约定

- 每个 JSON 文件为 UTF-8；顶层含整数 `schema_version: 1` 与字符串 `type`。
- 对象 ID 一律使用字符串；枚举使用字符串；拒绝 NaN/Infinity；数值保存足以往返 double 的精度。
- 单位后缀约定：`_m`（米）、`_rad`（弧度）、`_s`（秒）、`_mps`、`_mps2`、`_inv_m`（1/米，曲率）。
- 必填字段缺失或主版本未知应报错；未知可选字段允许读取，但不得改变已有字段含义。
- 引用其他夹具文件时，除非文件内另有说明，使用以 `tests/fixtures/v1/` 为根的相对路径；`playback/index.json` 中的文件名相对 `playback/` 目录。
- `playback/` 内文件哈希对保存后的原始文件字节计算（sha256sum 格式小写十六进制），不要求不同语言生成相同文本布局。

## 字段命名约定（与 serialization 冻结契约对齐）

以下字段与 `tests/fixtures/v1/serialization/README.md`（B 组已发布的冻结契约）一致；其他夹具与其冲突时，以冻结契约为准（冻结契约自身声明的对账规则）。

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

- `paths/cases.json`：30 个正/负例 + 4 个边界例的几何自检完成（见文件内 `self_check`）；终点碰撞派生例可在组件验收前按 `boundary_201` 的约定派生。
- `collision/`：只含规范与种子；10,000 / 100,000 级样本、不确定样本补样与失败实例保存到运行目录，报告写明可判定样本数、真碰撞数、漏检数、误报数与不确定数。
- `benchmarks/map/specs.json`：100,000 lanelet 地图待生成，按 §9 使用 SHA256 归档引用；未生成前不得以缩小输入沿用原比较标签。
- `serialization/`：冻结契约已发布（B 组）；本组路径、轨迹、走廊与栅格字段已按冻结契约生成。已知差异：`Src/common/include/planner/common/types.hpp` 的 GridView 注释写为“origin 为 cell (0,0) 中心”，与冻结契约（cell (0,0) 左下角，ROS OccupancyGrid 约定）矛盾；按冻结契约以冻结稿为准，该差异留在 B 组范围并要求协调者统一修订。
- 本 README 描述的“可加载”“完备”均为交付状态，不代表已经执行过加载或验证。

## 合成数据声明

`playback/` 下全部文件标记 `synthetic: true`：仅作显示测试，不能作为算法正确性证据，也不代表任何上游实现的实际输出。`paths/`、`benchmarks/`、`collision/` 中的数据为确定性测试输入或生成规范，同样不构成算法正确性结论。
