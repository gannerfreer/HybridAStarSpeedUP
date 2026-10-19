# planner_common（公共类型与 v1 JSON 序列化契约）

所有规划组件共享的第一版公开类型及其 UTF-8 JSON 编解码器（`planner::common`，
C++17、ROS-free、不依赖 Lanelet2/浏览器）。线上契约（字段名、单位、枚举值、
必填/可选与校验规则）冻结在
[`tests/fixtures/v1/serialization/README.md`](../../tests/fixtures/v1/serialization/README.md)；
`Src/common/src/serialization.cpp` 是唯一实现者，其他组件不得重新实现编码
（依据 `Docs/component_development_plan.md` §6.1）。

当前状态：测试 45/45（`test_types` 10 + `test_serialization` 32 + `test_fixtures` 3，
标签 `common`）；示例 `planner_common_roundtrip_example` 随构建产出（E0 验收记录
其已构建、未运行，非门槛）；无基准套件。

## 目录

```text
Src/common/
  CMakeLists.txt                  组件构建（planner_common；PLANNER_BUILD_EXAMPLES 控制示例）
  include/planner/common/
    types.hpp                     公开类型、kSchemaVersion=1、三态常量、角度归一化声明
    serialization.hpp             载荷/文档编解码入口、严格枚举解析
    errors.hpp                    ErrorCode（9 个稳定错误码）与 SerializationError
  src/
    types.cpp                     角度归一化、枚举字符串、GridView::cell、值相等比较
    serialization.cpp             全部 v1 类型编解码实现与显式实例化列表
  examples/roundtrip_example.cpp  命令行往返示例（--self-check / --list-types / <type> <in> [out]）
  tests/
    test_types.cpp                类型语义与枚举字符串
    test_serialization.cpp        编解码与拒绝路径（未知字段容忍、double 精度保留等）
    test_fixtures.cpp             冻结 v1 夹具驱动（10 valid / 24 invalid + 共享车辆/栅格夹具）
```

## 公开接口

- 常量：`kSchemaVersion = 1`（每个独立文档顶层 `schema_version`）、`kPi`、
  三态单元值 `kCellFree=0` / `kCellOccupied=1` / `kCellUnknown=2`；
  `normalize_angle_rad()` 归一化到 `[-π, π)`（输入须有限，编解码器先检查）。
- 枚举：`Direction{kForward, kReverse}`、`Status{kSuccess, kInvalidInput, kNoPath,
  kTimeout, kCancelled, kOptimizationFailed, kValidationFailed}`；`to_string()`
  输出规范字符串（`forward`/`reverse`、`success`/`invalid_input`/…），
  `direction_from_string()` / `status_from_string()` 严格解析（未知值抛
  `SerializationError`）。
- 几何与车辆：`Point2d{x_m, y_m}`、`Pose2d{x_m, y_m, yaw_rad}`（后轴中心，局部
  右手系，yaw 由编解码器归一化）、`VehicleModel`（矩形足迹；最小转弯半径
  `min_turn_radius_m() = wheelbase_m / tan(max_steer_rad)` 为派生量）。
- 栅格：`GridView{map_id, map_version, width, height, resolution_m, origin, cells}`
  （`cells` 行主序；`in_bounds()` / `cell()`，越界抛 `std::out_of_range`）。
- 路径/轨迹：`PathPoint`（位姿、非负累积弧长 `s_m`、带符号曲率
  `curvature_inv_m = tan(steer)/wheelbase`、方向）、`PathSegment`（同向片段，
  换向必开新段）、`TrajectoryPoint`（时间、位姿、带符号速度/加速度、曲率）。
- 请求/结果：`PlanRequest`（`config` 为调用方 JSON，原样保留）、`PlanResult`
  （`statistics` 同上；空数组等价字段缺失）。
- 走廊：`CorridorPolygon`（`s_start_m`/`s_end_m` + ≥3 顶点）、`Corridor`
  （同一坐标系的有序凸多边形集合，与地图快照/弧长参数绑定）。
- 编解码：`encode_payload()` / `decode_payload()`（嵌套载荷，无信封）、
  `encode_document()` / `decode_document()`（信封
  `{"schema_version":1,"type":"<type-name>",…}`）；`json_codec<T>` 模板仅对
  10 个显式实例化类型（`Pose2d`、`VehicleModel`、`GridView`、`PathPoint`、
  `PathSegment`、`TrajectoryPoint`、`PlanRequest`、`PlanResult`、`Status`、
  `Corridor`）可编译。
- 错误：`ErrorCode` 9 值 + 稳定字符串；`SerializationError::code()` 供调用方
  不解析消息文本即可区分拒绝原因。
- 值相等：每个公开类型的 `operator==`（精确 double 比较，供往返测试与缓存）。

## 冻结契约与版本

- 独立文档是 UTF-8 JSON 对象，含整数 `schema_version`（v1 唯一合法值 `1`）与
  字符串 `type`；`type` 名称表、字段表与枚举闭集见冻结契约文档第 2–4 节。
- 未知字段在顶层与嵌套对象中**解码容忍并忽略**，编码只输出契约字段；
  对象 ID 一律字符串、枚举一律字符串。
- 禁止 NaN/Infinity：编码对每个 double（含 `config`/`statistics` 内递归）检查；
  解码遇到超出 double 范围的数字字面量（如 `1e999`）报错。double 以最短往返
  表示保存，解码 → 编码 → 解码恢复精确相等值。
- `yaw_rad` 在编码与解码两侧都归一化到 `[-π, π)`，已在范围内的值精确幂等。
- 跨记录不变量由编解码校验：`PathSegment.points` 内 `s_m` 严格递增且方向一致、
  `PlanResult.trajectory` 内 `t_s` 严格递增、`Corridor.polygons` 按 `s_start_m`
  非递减。换向处速度为零、运动学一致性、走廊凸性与整车包含属**生产方职责**。
- 修改规则：字段或规则变化必须同步更新契约文档、本组件实现、`valid/`/`invalid/`
  样例与 `serialization.cpp` 的显式实例化列表；不兼容修改提升 `schema_version`
  主版本。
- 可执行证据：`tests/fixtures/v1/serialization/` 的 10 个 valid 样例（解码成功且
  往返精确相等）与 24 个 invalid 样例（按表中错误码拒绝），由 `test_fixtures.cpp`
  经 CMake 注入的绝对路径加载，测试从不依赖工作目录。

## GridView 坐标与三态语义

- `origin` 是栅格原点位姿，即单元 `(row=0, col=0)` **左下角**在地图局部坐标系
  中的位姿；单元 `(r, c)` 中心 = `origin` 沿其 +x 平移 `(c+0.5)·resolution_m`、
  沿 +y 平移 `(r+0.5)·resolution_m`（`origin.yaw_rad` 参与旋转），与 v1 夹具
  使用的 ROS OccupancyGrid 约定一致。
- `cells` 按行主序存储，取值仅 `0/1/2`（空闲/占据/未知）；消费方将未知与
  地图外一律视为占据（`types.hpp` 与冻结契约第 3.3 节）。
- `test_types.cpp` 与 `test_serialization.cpp` 固定索引、边界与三态取值行为。

## 错误码与拒绝路径

| `ErrorCode` | 稳定字符串 | 触发条件 |
| --- | --- | --- |
| `kParseError` | `parse_error` | 不是合法 JSON（含裸 `NaN` 等非法字面量） |
| `kInvalidDocument` | `invalid_document` | 文档根不是对象 |
| `kMissingField` | `missing_field` | 必填字段缺失 |
| `kTypeError` | `type_error` | JSON 类型不符 |
| `kUnsupportedSchemaVersion` | `unsupported_schema_version` | `schema_version` 为不等于 1 的整数 |
| `kUnknownType` | `unknown_type` | `type` 与目标 C++ 类型不匹配 |
| `kInvalidValue` | `invalid_value` | 范围/不变量违反（长度不匹配、越界单元格等） |
| `kUnknownEnumValue` | `unknown_enum_value` | 枚举字符串不在闭集内 |
| `kNonFiniteValue` | `non_finite_value` | NaN/Infinity（编码）或超范围字面量（解码） |

## 被全组件消费

- 全部 C++ 组件（rs、collision、hybrid_astar、corridor、validation、bspline、
  polynomial、map、dftpav、service、graph_search、grid_search、sampling_search，
  以及 grid/scene）均链接 `planner::common`：`Status`/`Pose2d`/`GridView` 等是
  跨组件值类型，JSON 契约是服务与工具的共同线上格式。
- `GridView` 按冻结补充暂留 common（不迁往 grid），`planner::grid` 只在其上
  提供数学与校验；scene 与两个服务消费同一坐标/三态语义。
- 本组件保持 ROS-free；公开头文件只依赖 nlohmann/json（不依赖 ROS、Lanelet2
  或浏览器类型）。

## 复现命令

```sh
python3 tools/task_runner.py build --component common --profile release --parallel 4
python3 tools/task_runner.py test --component common --workers 2

# 示例（构建目录内；--self-check 不依赖工作目录）
{build_dir}/Src/common/planner_common_roundtrip_example --self-check
{build_dir}/Src/common/planner_common_roundtrip_example --list-types
{build_dir}/Src/common/planner_common_roundtrip_example <type> <input.json> [output.json]
```

示例退出码：0 成功、1 往返不一致、2 序列化错误、3 文件 I/O 错误、4 用法错误。
`{build_dir}` 为任务入口构建目录（如 `build/release`）。`common` 无注册基准套件
（`tools/components.json` 的 `benchmark_suites` 为空），`benchmark --component
common` 会因缺少套件报错（预期，不静默回退）。

## 已知限制

1. 只支持 v1 主版本；不兼容修改必须提升 `schema_version` 并同步契约文档、
   实现、样例与实例化列表，旧客户端不跨主版本解析。
2. `json_codec<T>` 只对 10 个显式实例化类型可编译；`Point2d`/`CorridorPolygon`
   仅作为嵌套载荷出现，没有独立文档类型。
3. 编解码只做结构、数值、枚举与文档内跨记录校验；生产方语义（换向速度为零、
   运动学一致性、走廊凸性/整车包含、地图快照不可变）不代检。
4. 无基准套件；往返示例已随构建产出，E0 验收记录其未运行（非门槛）。

## 报告

- 组件验收（E0-1，common 45/45）：[`Docs/reports/e0/latest.md`](../../Docs/reports/e0/latest.md)
  → [E0 验收报告](../../Docs/reports/e0/20261009T152456187416Z_e0_0002de45.md)
  （§2.1 主要 run、§3 E0-1/E0-5 证据；common 测试 run `72a311f2`；首次构建失败
  `b62dbe2e` 的证据保留，修复后构建 run `972aa33a` 成功）。
- 冻结契约与夹具清单：[`tests/fixtures/v1/serialization/README.md`](../../tests/fixtures/v1/serialization/README.md)。
- 问题跟踪：[`Docs/reports/ISSUES.md`](../../Docs/reports/ISSUES.md)（无 `common-*` 条目）。
