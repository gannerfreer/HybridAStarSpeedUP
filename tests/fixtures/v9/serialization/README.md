# tests/fixtures/v1/serialization：公共类型 v1 JSON 契约（冻结）

本目录是 E0 公共类型 JSON 编解码的**冻结契约**：字段表、枚举值、必填/可选与校验规则以本文为准，`Src/common/src/serialization.cpp` 是唯一实现者，`valid/`、`invalid/` 样例是其可执行证据。其他夹具、组件或工具若与本文冲突，以本文为准并由协调者统一对账修订（已知差异见第 8 节）。

**当前状态：** 本阶段（E0，B 组续跑）只写文件；**未执行任何构建、测试或示例运行**。加载证据由协调者通过任务入口构建 `planner_common` 后产生。

修改规则：字段或规则变化必须同时更新本文、`Src/common/` 实现、`valid/`/`invalid/` 样例与 `Src/common/src/serialization.cpp` 末尾的显式实例化列表；不兼容修改必须提升 `schema_version` 主版本。

## 1. 文档包络与通用规则

- 每个独立文档是 UTF-8 JSON 对象，必须含：
  - `schema_version`：JSON 整数，v1 唯一合法值为 `1`；
  - `type`：字符串类型名（第 2 节）。
- 类型错误映射：缺失必填 → `missing_field`；整数值 ≠ 1 → `unsupported_schema_version`；JSON 类型不符 → `type_error`。
- 未知字段：顶层与嵌套对象中的未知字段**解码时容忍并忽略**；编码只输出契约字段，不输出未知字段。
- 对象 ID 一律字符串（避免浏览器整数精度丢失）；枚举一律字符串。
- 数值：禁止 NaN/Infinity。编码对每个 double（含 `config`/`statistics` 内递归）检查；解码遇到超出 double 范围的数字字面量（如 `1e999`）报错。JSON 文本布局不是契约，但 double 以最短往返表示保存，解码 → 编码 → 解码必须恢复相等的值（double 精确相等）。
- `yaw_rad` 在编码与解码两侧都归一化到 `[-π, π)`；输入可为任意有限角，输出恒在范围内，已在范围内的值精确幂等。
- 嵌套 `pose`、`origin`、`vehicle` 等对象同样允许未知字段。
- 文档根不是对象 → `invalid_document`；JSON 解析失败 → `parse_error`。

## 2. 类型名总表

| `type` | C++ 类型（`planner::common`） | 说明 |
| --- | --- | --- |
| `pose2d` | `Pose2d` | 后轴中心位姿 |
| `vehicle_model` | `VehicleModel` | 矩形足迹车辆参数 |
| `grid_view` | `GridView` | 只读占据栅格 |
| `path_point` | `PathPoint` | 路径点 |
| `path_segment` | `PathSegment` | 同向路径段 |
| `trajectory_point` | `TrajectoryPoint` | 轨迹采样点 |
| `plan_request` | `PlanRequest` | 规划请求 |
| `plan_result` | `PlanResult` | 规划结果 |
| `status` | `Status` | 状态码独立文档 |
| `corridor` | `Corridor` | 安全走廊 |

## 3. 字段表（逐类型）

标注：✔ = 必填，○ = 可选；“默认”为字段缺省时的取值。单位后缀：`_m` 米、`_rad` 弧度、`_s` 秒、`_mps` 米/秒、`_mps2` 米/秒²、`_inv_m` 1/米。

### 3.1 `pose2d` → `Pose2d`

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `x_m` | number | m | ✔ | 有限 |
| `y_m` | number | m | ✔ | 有限 |
| `yaw_rad` | number | rad | ✔ | 有限；两侧归一化 `[-π, π)` |

语义：后轴中心，局部右手坐标（x 向前、y 向左），航向逆时针为正。

### 3.2 `vehicle_model` → `VehicleModel`

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `id` | string | — | ✔ | 非空 |
| `wheelbase_m` | number | m | ✔ | `> 0` |
| `width_m` | number | m | ✔ | `> 0` |
| `front_overhang_m` | number | m | ✔ | `>= 0` |
| `rear_overhang_m` | number | m | ✔ | `>= 0` |
| `max_steer_rad` | number | rad | ✔ | `0 < v < π/2` |
| `safety_margin_m` | number | m | ✔ | `>= 0` |

语义：首版矩形整车足迹；最小转弯半径 = `wheelbase_m / tan(max_steer_rad)`（派生量，不单独序列化）。§7.1 基准车辆：轴距 2.8、宽 1.8、前悬 0.9、后悬 1.0、最大转角 0.5、安全裕量 0.1。

### 3.3 `grid_view` → `GridView`

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `map_id` | string | — | ✔ | 非空 |
| `map_version` | string | — | ✔ | 非空；不透明版本串，按相等比较 |
| `width` | integer | 格 | ✔ | `>= 0` |
| `height` | integer | 格 | ✔ | `>= 0` |
| `resolution_m` | number | m/格 | ✔ | `> 0` |
| `origin` | object（Pose2d） | — | ✔ | 见下 |
| `cells` | array<integer> | — | ✔ | 行优先；长度 = `width × height`；取值仅 0/1/2 |

语义：`cells` 长度不足/超出或含非法值报错。`origin` 为**栅格原点位姿**，即 cell `(row=0, col=0)` 左下角在地图局部系中的位姿（ROS OccupancyGrid 约定）：cell `(r, c)`（r 行、c 列）中心 = `origin` 平移 `((c + 0.5) × resolution_m, (r + 0.5) × resolution_m)`（沿 origin 坐标系 +x/+y）。unknown 与地图外一律按占据处理（消费方规则）。

### 3.4 `path_point` → `PathPoint`

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `pose` | object（Pose2d） | — | ✔ | 有限；yaw 归一化 |
| `s_m` | number | m | ✔ | `>= 0`，累计非负弧长 |
| `curvature_inv_m` | number | 1/m | ✔ | 有限；定义 `tan(steer)/wheelbase`，满足 `yaw_rate = signed_speed × curvature` |
| `direction` | string | — | ✔ | `forward`/`reverse` |

### 3.5 `path_segment` → `PathSegment`

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `points` | array<PathPoint> | — | ✔ | 至少 1 个点；段内 `s_m` 严格递增；`direction` 与首点一致 |

语义：换向点必定开启新段；段内 `s_m` 不得相等或回退。

### 3.6 `trajectory_point` → `TrajectoryPoint`

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `t_s` | number | s | ✔ | `>= 0` |
| `pose` | object（Pose2d） | — | ✔ | 有限；yaw 归一化 |
| `velocity_mps` | number | m/s | ✔ | 有限；有符号（倒车为负） |
| `acceleration_mps2` | number | m/s² | ✔ | 有限；纵向 |
| `curvature_inv_m` | number | 1/m | ✔ | 有限 |

语义：跨点级不变量（时间严格递增、换向处速度为零）在 `plan_result.trajectory` 与轨迹校验组件中检查，见 3.8/第 5 节。

### 3.7 `plan_request` → `PlanRequest`

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `request_id` | string | — | ✔ | 非空 |
| `map_id` | string | — | ✔ | 非空 |
| `map_version` | string | — | ✔ | 非空 |
| `vehicle` | object（VehicleModel） | — | ✔ | 同 3.2 |
| `start` | object（Pose2d） | — | ✔ | 有限；yaw 归一化 |
| `goal` | object（Pose2d） | — | ✔ | 有限；yaw 归一化 |
| `config` | object | — | ○ | 默认 `{}`；原样保留，递归拒绝 NaN/Infinity |
| `time_budget_s` | number | s | ○ | 缺省 = 无限制；出现时 `> 0` |
| `node_budget` | integer | 节点 | ○ | 缺省 = 无限制；出现时 `> 0` |
| `cancelled` | boolean | — | ○ | 默认 `false` |

### 3.8 `plan_result` → `PlanResult`

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `request_id` | string | — | ✔ | 非空 |
| `status` | string | — | ✔ | Status 枚举（第 4 节） |
| `reason` | string | — | ○ | 默认 `""`；失败原因/说明 |
| `path` | array<PathSegment> | — | ○ | 默认 `[]`；空数组等价于缺省 |
| `trajectory` | array<TrajectoryPoint> | — | ○ | 默认 `[]`；空数组等价于缺省；段内 `t_s` 严格递增 |
| `statistics` | object | — | ○ | 默认 `{}`；生产方自定义，原样保留，递归拒绝 NaN/Infinity |

### 3.9 `status` → `Status`

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `status` | string | — | ✔ | Status 枚举（第 4 节） |

### 3.10 `corridor` → `Corridor`

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `id` | string | — | ✔ | 非空 |
| `map_id` | string | — | ✔ | 非空 |
| `map_version` | string | — | ✔ | 非空 |
| `frame_id` | string | — | ✔ | 非空；多边形坐标系 |
| `polygons` | array<CorridorPolygon> | — | ✔ | 允许空数组；按 `s_start_m` 非递减有序 |

`CorridorPolygon`：

| 字段 | JSON 类型 | 单位 | 必填 | 约束 |
| --- | --- | --- | --- | --- |
| `s_start_m` | number | m | ✔ | `>= 0` |
| `s_end_m` | number | m | ✔ | `> s_start_m` |
| `vertices` | array<Point2d `{x_m, y_m}`> | m | ✔ | 至少 3 个顶点；数值有限 |

语义：多边形表示自由空间，关联路径弧长区间 `[s_start_m, s_end_m)`；凸性、逆时针方向与“整车包含”由走廊/Dftpav 消费方检查，本编解码只校验结构。不得混用仅供车辆中心使用的膨胀空间。

## 4. 枚举与取值

| 枚举 | 值 | 说明 |
| --- | --- | --- |
| `direction` | `forward` | 前进 |
| | `reverse` | 倒车 |
| `status` | `success` | 成功 |
| | `invalid_input` | 输入非法（含缺失必需输入） |
| | `no_path` | 算法判定无解 |
| | `timeout` | 时间预算耗尽 |
| | `cancelled` | 取消标志生效 |
| | `optimization_failed` | 优化失败 |
| | `validation_failed` | 轨迹校验失败 |
| `cells` 取值 | `0` free / `1` occupied / `2` unknown | 整数，不是字符串 |

资源终止（如监控强制停止）由任务运行器记录为 `resource_stopped`，**不得**伪装成 `no_path` 或其他算法状态。

## 5. 编码/解码校验规则（对称）

编解码执行同一组规则，因此凡能写出的文档都能读回：

1. 结构：包络与字段类型、必填存在性、对象/数组形状。
2. 数值：所有 double 有限；范围约束（正负、开闭区间）如第 3 节。
3. 枚举：字符串必须属于第 4 节闭集。
4. 跨记录：`PathSegment.points` 内 `s_m` 严格递增且方向一致；`PlanResult.trajectory` 内 `t_s` 严格递增；`Corridor.polygons` 按 `s_start_m` 非递减。
5. `config`/`statistics` 原样保留但对其中所有 double 递归做有限性检查。
6. 生产方职责（编解码不代检）：换向处速度为零、时间/弧长由运动学一致性生成、走廊凸性与整车包含、地图快照不可变等。

## 6. 错误码

`planner::common::ErrorCode` 及稳定字符串（`to_string`）：

| 错误码 | 字符串 | 触发条件 |
| --- | --- | --- |
| `kParseError` | `parse_error` | 不是合法 JSON（含裸 `NaN` 等非法字面量） |
| `kInvalidDocument` | `invalid_document` | 文档根不是对象 |
| `kMissingField` | `missing_field` | 必填字段缺失 |
| `kTypeError` | `type_error` | JSON 类型不符（如 ID 为数字、整型字段为浮点、数组含字符串） |
| `kUnsupportedSchemaVersion` | `unsupported_schema_version` | `schema_version` 为不等于 1 的整数 |
| `kUnknownType` | `unknown_type` | `type` 与目标 C++ 类型不匹配 |
| `kInvalidValue` | `invalid_value` | 范围/不变量违反（含长度不匹配、枚举值越界为整数单元格） |
| `kUnknownEnumValue` | `unknown_enum_value` | 枚举字符串不在闭集内 |
| `kNonFiniteValue` | `non_finite_value` | NaN/Infinity：编码遇到非有限 double；解码遇到超范围数字字面量（如 `1e999`） |

## 7. 夹具清单

`valid/`（解码必须成功；往返后与首次解码结果精确相等）：

| 文件 | type | 覆盖点 |
| --- | --- | --- |
| `pose2d.json` | `pose2d` | 负坐标、范围内 yaw、未知可选字段 `frame_id` |
| `vehicle_model.json` | `vehicle_model` | §7.1 基准车辆；未知可选字段 |
| `grid_view.json` | `grid_view` | 4×3、三态 cells、非零 origin；未知可选字段 |
| `path_point.json` | `path_point` | 嵌套 pose、非零曲率 |
| `path_segment.json` | `path_segment` | 4 点前进段、`s_m` 严格递增 |
| `trajectory_point.json` | `trajectory_point` | 有符号速度、负加速度 |
| `plan_request.json` | `plan_request` | 嵌套 vehicle、config 保留、时间/节点预算、未知顶层字段 |
| `plan_result.json` | `plan_result` | 前进+倒车两段、t 严格递增轨迹、statistics、未知字段 |
| `status.json` | `status` | 枚举文档、未知可选字段 |
| `corridor.json` | `corridor` | 2 个有序多边形 |

`invalid/`（解码必须以表中错误码拒绝）：

| 文件 | 期望错误码 | 触发规则 |
| --- | --- | --- |
| `schema_version_unknown.json` | `unsupported_schema_version` | `schema_version = 2` |
| `schema_version_missing.json` | `missing_field` | 缺 `schema_version` |
| `schema_version_wrong_type.json` | `type_error` | `schema_version = "1"` |
| `type_unknown.json` | `unknown_type` | `type = "hybrid_astar_path"` |
| `type_missing.json` | `missing_field` | 缺 `type` |
| `malformed_json.json` | `parse_error` | 截断的 JSON |
| `nan_literal.json` | `parse_error` | 裸 `NaN` 非法字面量 |
| `nonfinite_overflow.json` | `non_finite_value` | `x_m = 1e999` |
| `pose_wrong_type.json` | `type_error` | `x_m` 为字符串 |
| `vehicle_missing_required.json` | `missing_field` | 缺 `wheelbase_m` |
| `vehicle_numeric_id.json` | `type_error` | `id` 为数字 |
| `vehicle_invalid_wheelbase.json` | `invalid_value` | `wheelbase_m = 0` |
| `grid_cells_length_mismatch.json` | `invalid_value` | cells 长度 ≠ width×height |
| `grid_invalid_cell_value.json` | `invalid_value` | 单元格值 3 |
| `grid_resolution_not_positive.json` | `invalid_value` | `resolution_m = 0` |
| `path_point_unknown_direction.json` | `unknown_enum_value` | `direction = "backward"` |
| `path_segment_direction_mixed.json` | `invalid_value` | 段内方向不一致 |
| `path_segment_s_not_increasing.json` | `invalid_value` | `s_m` 相等 |
| `plan_request_budget_wrong_type.json` | `type_error` | `time_budget_s = "5s"` |
| `plan_result_unknown_status.json` | `unknown_enum_value` | `status = "ok"` |
| `plan_result_trajectory_time_not_increasing.json` | `invalid_value` | `t_s` 相等 |
| `corridor_polygon_too_few_vertices.json` | `invalid_value` | 顶点 2 个 |
| `corridor_s_interval_empty.json` | `invalid_value` | `s_end_m = s_start_m` |
| `corridor_polygons_unordered.json` | `invalid_value` | 多边形 `s_start_m` 回退 |

`Src/common/tests/test_fixtures.cpp` 通过 CMake 注入的绝对路径加载以上全部文件；另外它直接加载共享夹具 `../vehicles/default.json` 与 `../grids/minimal.json`，验证公共契约能消费 C 组夹具（E0-5）。

## 8. 与其他夹具的对账状态

`tests/fixtures/v1/README.md` 的“字段命名约定”最初为 C 组样例的**临时命名**并声明以本契约为准；截至当前工作区，C 组的 `paths/`、`playback/`、`grids/` 夹具已按本契约生成（嵌套 `pose`、`curvature_inv_m`、`velocity_mps`、`acceleration_mps2`、走廊 `s_start_m/s_end_m`、栅格左下角 origin）。原始差异记录如下，供历史对账：

| 类型 | C 组临时命名（已弃用） | 冻结契约 | 状态 |
| --- | --- | --- | --- |
| `PathPoint`/`TrajectoryPoint` 位姿 | 扁平 `x_m, y_m, yaw_rad` | 嵌套 `pose: {x_m, y_m, yaw_rad}` | C 组已改为嵌套 |
| `TrajectoryPoint` 速度 | `speed_mps` | `velocity_mps`（有符号） | C 组已对齐 |
| `TrajectoryPoint` 加速度 | `accel_mps2` | `acceleration_mps2` | C 组已对齐 |
| 曲率 | `curvature_inv_m` | `curvature_inv_m` | 一致 |
| 走廊多边形 | `{id, path_segment_index, vertices}` | `{s_start_m, s_end_m, vertices}` | C 组已对齐 |
| 栅格 `origin` 语义 | cell (0,0) 左下角（ROS 约定） | 相同 | 一致 |
| 栅格/车辆附加字段 | `cell_encoding`、`footprint_shape`、`source`、`notes` 等 | 未知可选字段容忍并忽略 | 保留 |

遗留的一条 C 组说明需要协调者核对：`tests/fixtures/v1/README.md` 末尾称 `Src/common/include/planner/common/types.hpp` 中 `GridView::origin` 注释仍写“cell (0,0) 中心”，该注释**已在 B 组修订中改为左下角约定**（与冻结契约一致，见 types.hpp 第 86–92 行）；该说明可删除或标记为已解决。C 组的 `vehicles/default.json`、`grids/minimal.json` 已由 `Src/common/tests/test_fixtures.cpp` 直接加载校验。

## 9. 权威实现与未来命令（未执行）

- 实现：`Src/common/src/serialization.cpp`（编解码）、`Src/common/include/planner/common/{types,errors,serialization}.hpp`；示例：`Src/common/examples/roundtrip_example.cpp`。
- 单测：`Src/common/tests/`（标签 `common`），依赖锁 `dependencies.lock.json` 固定 nlohmann/json v3.11.3 与 GoogleTest v1.15.2。
- 离线构建：经 `FETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON`、`FETCHCONTENT_SOURCE_DIR_GOOGLETEST` 指向本地已解压源树，或一次成功下载后使用 `FETCHCONTENT_FULLY_DISCONNECTED=ON`。

供协调者通过任务入口执行（示例，**本阶段均未执行**）：

```sh
cmake -S . -B build/release -DPLANNER_BUILD_COMMON=ON -DPLANNER_BUILD_TESTS=ON
cmake --build build/release --parallel 4
ctest --test-dir build/release -L common --output-on-failure
cmake --build build/release --target planner_common_roundtrip_example
./build/release/Src/common/planner_common_roundtrip_example --self-check
```
