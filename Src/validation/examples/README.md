# planner_validation 示例

`planner_validation_example` 读取一条预制轨迹 JSON 并运行 `planner::validation::validate_trajectory`，在 stdout 输出结论 JSON。

```sh
planner_validation_example examples/data/valid_open_road.json
planner_validation_example --self-check
```

## 退出码

| 码 | 含义 |
| --- | --- |
| 0 | 校验结论 `valid`（或 `--self-check` 全部符合预期） |
| 1 | 读到了合法文档，但结论为 `invalid_input` / `validation_failed` |
| 2 | 命令行用法错误 |
| 3 | 文件 IO 或文档解析/结构错误 |
| 4 | `--self-check` 期望不符 |

## 输入文档格式（示例私有，schema_version 1，type `validation_request`）

顶层字段：

| 字段 | 必填 | 说明 |
| --- | --- | --- |
| `schema_version` | 建议 | 出现时必须为整数 `1` |
| `type` | 建议 | 出现时必须为 `"validation_request"` |
| `vehicle` | 是 | 公共 `VehicleModel` payload |
| `limits` | 是 | `max_speed_mps`、`max_acceleration_mps2`、`max_deceleration_mps2`、`max_curvature_inv_m`，均须为有限正数 |
| `goal` | 是 | 公共 `Pose2d` payload（终点检查基准） |
| `grid` | 否 | 公共 `GridView` payload；缺省表示无栅格（自由空间） |
| `obstacles` | 否 | 数组；每项 `{"id": str, "vertices": [{"x_m": num, "y_m": num}, ...]}`，凸多边形，接触视为碰撞 |
| `corridor` | 是 | 公共 `Corridor` payload（整车包含检查） |
| `trajectory` | 是 | 公共 `TrajectoryPoint` payload 数组，至少 1 个点 |

嵌套的公共类型使用 `planner/common/serialization.hpp` 的 payload 编解码（不含 `schema_version`/`type` 包络）；本文件格式是组件示例的诊断输入，不是公共类型契约。

## 输出文档

stdout 输出 `validation_result` JSON：`valid`、`status`（`success` / `invalid_input` / `validation_failed`）、`reason`（稳定代码）、`detail`、`first_failure`（`constraint`、`time_s`、`point_index`、`actual`、`limit`、`detail`；不适用或输入错误时为 `null`）与 `stats`（轨迹点数、走廊采样数、碰撞位姿/运动检查数）。

## `examples/data/`

- `valid_open_road.json`：直线前进（加速—匀速—减速到停）、障碍物在走廊外、走廊覆盖全程的合法轨迹，预期结论 `valid`。
