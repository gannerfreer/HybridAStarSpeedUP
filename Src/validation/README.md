# planner_validation（轨迹校验 V：独立于优化器的硬门禁）

依据 `Docs/component_development_plan.md` §2/§6.2/§7.1（“轨迹校验 V”行）交付：
对采样轨迹做独立于优化器的后验检查，输出 `valid`、首个失败时刻、被违反的
约束与实际值/限值对。当前状态：`passed`（standalone）；测试 60/60（58 个
gtest + 2 个示例自检）；固定容限是组件契约、不可配置。真实 Dftpav 优化轨迹的
端到端证据挂 Dftpav 报告 §3.3/§4 并单列 VALIDATION（本组件独立验收不宣称
端到端通过）。

## 目录

| 路径 | 内容 |
| --- | --- |
| `include/planner/validation/validation.hpp` | 公开 API：`validate_trajectory`、`Constraint`、`VehicleLimits`、`ValidationRequest`/`ValidationViolation`/`ValidationOptions`/`ValidationResult` 与固定容限常量 |
| `src/validation.cpp` | 固定顺序检查、常曲率运动重建、走廊包含、碰撞复用与首次失败归约 |
| `tests/` | GoogleTest 七套：输入校验 15、时间与有限值 9、动力学限值 10、终点与顺序 7、走廊包含 5、碰撞 6、有效轨迹 6；CTest 标签 `validation` |
| `examples/validate_trajectory.cpp` | 命令行示例：读取验证请求 JSON 并输出结论 JSON；`--self-check` |
| `examples/README.md` + `examples/data/valid_open_road.json` | 示例输入文档格式与随附样例（合法直线轨迹） |

## 公开接口

- `validate_trajectory(request, options = {}) -> ValidationResult`。
- 输入（`ValidationRequest`）：`trajectory`（`common::TrajectoryPoint` 序列，
  必需非空）、`vehicle`（必需，足迹有效）、`limits`（必需，全部 > 0）、
  `goal`、`scene`（`collision::Scene`：只读 GridView + 凸障碍物）、
  `corridor`（`common::Corridor`）；后三者为 `std::optional`，缺失时显式返回
  `invalid_input`，而不是被静默跳过。
- 输出（`ValidationResult`）：`status`（`kSuccess`/`kInvalidInput`/
  `kValidationFailed`）、`valid`、稳定 `reason`、`detail`、`first_failure`
  （`constraint`/`time_s`/`point_index`/`actual`/`limit`/`detail`），以及工作量
  统计（轨迹点数、走廊采样数、碰撞位姿/运动检查数；失败结果也会填充）。
- 固定容限常量（组件契约，不可配置）：`kEndpointPositionToleranceM = 0.05`、
  `kEndpointHeadingToleranceRad = 0.02`、`kDirectionChangeSpeedToleranceMps = 1e-3`、
  `kLimitRelativeTolerance = 1e-6`（允许值 = `limit + 1e-6·max(1, limit)`）。
- `ValidationOptions` 只调整采样与浮点保护：`corridor_sample_step_m`（0.02 m）、
  `containment_tolerance_m`（1e-6 m）、`collision`（原样转发给
  `planner::collision` 的 `CheckOptions`）；不改变任何检查自身的容差。
- `to_string(Constraint)` 返回稳定约束名；无栅格场景（width == height == 0）
  是合法自由空间，空走廊永远无法覆盖非空轨迹（包含性失败）。

## 检查项与判定顺序

1. **输入有效性**：`trajectory`/`vehicle`/`limits`/`goal`/`scene`/`corridor`
   必需且合法；场景含栅格时 `map_id`/`map_version` 不匹配拒绝。失败 →
   `Status::kInvalidInput`，`reason` 为 `missing_trajectory`、`invalid_vehicle`、
   `invalid_limits`、`missing_goal`、`invalid_goal`、`missing_scene`、
   `invalid_scene`、`missing_corridor`、`invalid_corridor`、`map_mismatch`、
   `invalid_options`、`invalid_motion_reconstruction` 等稳定代码。
2. **有限值**：所有存储值必须有限；NaN/Inf 拒绝且不伪造失败时刻。
3. **时间严格递增**：`t_s` 非严格递增拒绝。
4. **换向停车**：相邻点速度符号相反时，`min(|v|) ≤ 1e-3 m/s`。
5. **速度 / 加速度 / 减速度 / 曲率限值**：按幅值与相对容差判定；倒车负速度
   按幅值检查。
6. **走廊包含**：沿轨迹稠密重采样（默认 0.02 m，含常曲率圆弧重建的中间
   位姿），整车保守足迹必须落在 s 区间覆盖该采样点的走廊多边形内；缺少覆盖
   多边形即覆盖失败；绝不以中心线代替整车。
7. **静态碰撞**：每个存储位姿精确检查；每相邻一对用由轨迹重建的
   `MotionSegment`（长度/方向/曲率）调用 `collision::check_motion` 扫掠；
   仅有离散位姿永远不够。
8. **端点**：终点位置 ≤ 0.05 m、航向 ≤ 0.02 rad（航向环绕正确）。

结构性检查（输入、有限值、时间）立即失败；其余各项报告其最早违规时刻，全局
结果保留最小时间；时间完全并列时按上述顺序取靠前者（顺序本身是契约的一部分，
例如速度限制与碰撞同在 t = 0 失败时报速度限制）。

## 硬门禁语义

- 组件绝不放松硬安全约束：任何违规一律 `Status::kValidationFailed`
  （`valid = false`），并给出首次失败时刻与实际值/限值对；碰撞类约束没有
  数值对时 `actual`/`limit` 为 NaN。
- 检查只有在对应输入缺失时才省略，而这本身是 `invalid_input` 失败；不存在
  “跳过即通过”的路径。
- 固定容限（0.05 m / 0.02 rad / 1e-3 m/s / `1e-6·max(1, limit)`）不可配置，
  也不允许下游为单个候选放宽（计划 §7.1；Dftpav 报告 §3.3“不得单方面放宽
  V 容限”）。
- 判定确定：同一输入得到同一结果（检查顺序与时间并列规则固定）。

## 与服务/优化器的关系

- V 独立于任何优化器（不依赖 Dftpav），只复用公开的 `planner::collision` /
  `planner::corridor` 接口；单测使用预制轨迹，不依赖 Dftpav 运行（计划 §6.2）。
- 集成位置：Dftpav 只有优化结果通过 V 后才能对外返回有效轨迹（计划 §6.2）。
- 端到端证据（计划 §6.2 强制链接）：真实 Dftpav 优化轨迹的 12 个链路用例断言
  `result.validation.valid`、走廊采样/碰撞检查计数、终点容差与换向停车，证据
  挂 Dftpav 报告 §3.3/§4（入口见 [dftpav/latest.md](../../Docs/reports/dftpav/latest.md)）。
- 坐标系与 s 偏移：V 不自行变换坐标系，直接消费调用方传入的 `Scene` 与
  `Corridor`；`map_id`/`map_version` 仅在场景含栅格时比较。

## 复现命令

```sh
python3 tools/task_runner.py build --component validation --profile release --parallel 4 \
    --build-dir build/validation
python3 tools/task_runner.py test --component validation --workers 1 \
    --build-dir build/validation
# 示例（构建目录下）：
#   planner_validation_example --self-check
#   planner_validation_example examples/data/valid_open_road.json
```

最终证据：build `20261009T172859718522Z_validation_b6bf2d95`（0 告警）、
test `20261009T173001758948Z_validation_c45e81dc`（60/60）；两个 run 均首次
通过，无失败链。

## 已知限制

1. 独立验收无真实 Dftpav 输出联调（端到端证据挂 Dftpav 报告 §3.3/§4）；本组件
   不宣称端到端通过。
2. 坐标系与 s 偏移假设需在集成时核对；`map_id`/`map_version` 仅在有栅格时比较。
3. 无独立性能基准（计划 §7.3 未要求 V）、无并发/线程安全验证。
4. 重建运动段：零长度步与 |曲率| > 1e6 1/m 的步不参与扫掠（由两端点精确位姿
   检查覆盖）；仅有离散位姿检查不充分是设计前提，不是可选路径。
5. 约束顺序与固定容限是契约；时间并列时按文档顺序取靠前者，不允许按候选
   单独放宽。

## 报告

- 入口：[Docs/reports/validation/latest.md](../../Docs/reports/validation/latest.md)
- 独立验收：[20261009T181558134265Z_validation_f646fa91.md](../../Docs/reports/validation/20261009T181558134265Z_validation_f646fa91.md)
- 端到端 VALIDATION 单列：Dftpav 报告 §3.3/§4（入口 [dftpav/latest.md](../../Docs/reports/dftpav/latest.md)）
- 问题跟踪：[ISSUES.md](../../Docs/reports/ISSUES.md)（当前无 validation 条目）
