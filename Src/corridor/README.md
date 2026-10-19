# planner_corridor（安全走廊生成：有序凸自由空间与独立几何验证）

依据 `Docs/component_development_plan.md` §6.2“安全走廊”行交付：从路径种子与
静态凸障碍生成与弧长区间绑定的有序凸多边形序列，供轨迹优化与校验消费。
当前状态：`passed`；测试 55/55（生成 23 + 几何 15 + 独立验证 16 + 示例自检 1）；
corridor-001（中心线障碍绕行走廊被远处障碍全局收紧成刀口宽度）与
corridor-002（tight 口袋走廊对位姿扰动敏感）均已修复并回归；构造包络
`robustness_margin_m ≈ 0.0895 m`（默认 ±1° 航向 / ±2 cm 位置）。无正式性能
基准（计划 §7.3 未要求本组件）。

## 目录

| 路径 | 内容 |
| --- | --- |
| `include/planner/corridor/corridor_generator.hpp` | 公开 API：`generate_corridor`、`verify_corridor`、`Obstacle`、`CorridorOptions`/`CorridorRequest`/`CorridorStats`/`CorridorResult`、`CorridorVerification`、`CorridorFailure` |
| `include/planner/corridor/geometry.hpp` | 2D 凸几何辅助：`HalfPlane`、凸包、点包含、多边形间隙、最小卡尺宽度、半平面求交、多边形外偏移 |
| `src/corridor_generator.cpp` | 路径展平/重采样、足迹扫掠、逐边扩张、失败分类、生成后自检与独立验证器 |
| `src/geometry.cpp` | 几何谓词与凸多边形运算实现 |
| `tests/` | GoogleTest（`CorridorGenerationTest` 23、`CorridorGeometryTest` 15、`CorridorVerificationTest` 16）+ 测试本地独立几何；CTest 标签 `corridor` |
| `examples/corridor_generator_example.cpp` | 命令行示例：`--list-scenarios` / `--scenario <name> [--output <file>]` / `--self-check` |

## 公开接口

- `generate_corridor(const CorridorRequest&) -> CorridorResult`：输入
  `map_id`/`map_version`/`frame_id`、车辆模型、路径种子（`common::PathSegment`
  的有序序列）与障碍物；成功时返回 `common::Corridor`（第一版公开类型：同一
  坐标系下与弧长区间绑定的有序凸多边形）。
- `verify_corridor(corridor, seed, obstacles, vehicle, options)`：独立几何
  验证器，只使用公开的走廊几何（不依赖生成器内部状态），因此可应用于任意
  来源的走廊。
- `make_rectangle_obstacle` / `make_convex_obstacle`：障碍物工厂；`Obstacle.kind`
  仅作说明，不改变顶点的解释方式。
- `CorridorFailure` + `to_string()`：稳定失败分类字符串；`CorridorResult`
  携带 `status`/`failure`/`reason`/`detail`/`stats`。
- `CorridorOptions`（长度单位 m、角度单位 rad）：`sample_step_m` 0.25、
  `construction_step_m` 0.05、`verification_step_m` 0.02、`clearance_m` 0.05、
  `max_expansion_m` 4.0、`perturbation_yaw_rad` 1°、`perturbation_position_m` 0.02。
- 调用方不得自行实现 JSON 编码：用 `planner/common/serialization.hpp` 持久化
  走廊文档（`common::encode_document` / `decode_document`）。
- 公开 API 不暴露生成器内部类型；`geometry.hpp` 的半平面等辅助函数由生成器、
  验证器与组件测试共用。

## 走廊语义与成功保证

成功时（头文件契约）：

- 多边形严格凸、逆时针且顶点数 ≥ 3；
- 多边形 s 区间为正、有序、连续，并覆盖整个种子弧长；
- 相邻多边形以正面积重叠；
- 对每个路径点，保守整车足迹都包含在 s 区间覆盖该路径点的多边形内；
- 任何障碍物都不接触多边形（最小间隙见 `clearance_m`）；
- 多边形最小宽度至少为车辆宽度 + 2 × 安全裕度。

保守足迹 = 车辆矩形（后悬 → 轴距 + 前悬、宽 `width_m`）每条边外扩
`safety_margin_m`；可通行性按整车检查，绝不以中心线无碰撞代替。

### 第一版算法（§6.2）

1. 展平并校验种子：段内弧长严格递增、每段方向单一；换向重复锚点（同弧长
   同位姿，容差 1e-6）去重；锚点间位置线性插值、yaw 沿最短弧插值。
2. 按 `sample_step_m` 重采样；每相邻两个采样点区间生成一个凸多边形：先取该
   区间保守足迹扫掠（子采样 `construction_step_m`）的凸包。
3. 种子多边形膨胀：相邻构造位姿间的插值垂度上界
   `corner_radius·(1−cos(Δyaw/2))·1.05`，再加扰动包络（见下节）。
4. 逐边扩张至 `max_expansion_m`：每条边按精确的多边形-障碍物分离量推进到
   极限，其余各边保持当前偏移（逐坐标最大化、多轮扫描 + 二分）；每个被接受
   的候选状态都满足所请求间隙且无障碍，不放松任何安全裕度。远处障碍物只限制
   它实际能触及的边（corridor-001 修复）。
5. 返回前用 `verify_corridor`（步长 `verification_step_m`）做常规自检；不通过
   则返回 `no_corridor`。

间隙：`clearance_m` 默认 0.05 m；若种子本身与障碍更近，则减小为可用间距的
一半（多边形绝不允许接触障碍物）。`CorridorStats` 提供 `min_polygon_width_m`、
`min_obstacle_clearance_m`、`obstacle_limited_edges`、`robustness_margin_m` 等统计。

## 扰动包络鲁棒性（corridor-002）

- 构造包络：`robustness_margin_m = (corner_radius·2sin(Δyaw/2) + Δposition)·1.01
  + 1e-12`；默认 ±1° / ±2 cm，测试车辆 `corner_radius ≈ 3.9294 m` 时约
  0.08947 m。两个扰动界限置零可禁用包络。
- 严格包含链：任一配置界内扰动位姿的足迹角点位移 ≤ 包络 → 扰动足迹包含在
  膨胀后的种子多边形内；逐边生长只扩大多边形 → 扰动足迹仍包含在最终走廊
  多边形内。
- 回归用例 `tight_pocket_corridor_contains_perturbed_footprints`：27 组合扰动
  （±1° × ±2 cm × ±2 cm），只用测试本地几何校验，容限 1e-6 m；修复前
  3.6167 cm 越界 → 修复后 ≤ 1e-6 m。
- 注意：`verify_corridor` 的公共契约按名义种子验证，本身不检查扰动界；鲁棒性
  由构造包络与上述独立回归用例覆盖。

## 失败分类与原因码

失败时走廊为空，`failure`/`reason`/`detail` 均被设置；`reason` 为稳定字符串。

| `CorridorFailure` | `reason` | `Status` | 触发条件 |
| --- | --- | --- | --- |
| `kInvalidInput` | `invalid_input` | `kInvalidInput` | 空 `map_id`/`map_version`/`frame_id`；车辆或选项非法；障碍非凸/退化/重复顶点；种子为空、单点、弧长非递增、长度非正 |
| `kBlocked` | `blocked` | `kNoPath` | 种子进入障碍物（含中心线触碰）；或足迹扫掠无法避开周围障碍 |
| `kNarrow` | `narrow` | `kNoPath` | 两侧障碍物自由宽度 < 车宽 + 2 × 安全裕度 |
| `kNoCorridor` | `no_corridor` | `kNoPath` | 重采样无正区间、凸包/种子多边形退化、扩张失败、生成后独立自检失败 |

`to_string(CorridorFailure)` 与用例 `failure_reason_names_are_stable` 锁定这些
名称；狭窄或断裂走廊显式失败，生成器绝不返回违反自身不变量的走廊。

### 独立验证器

`verify_corridor` 对任意来源的走廊做与生成器独立的几何检查：结构（严格凸、
逆时针、顶点数、s 区间为正/有序/非负）、覆盖（起止与无缺口）、相邻正面积
重叠、障碍分离（间隙与顶点严格在内）、最小宽度、沿稠密重采样的整车足迹包含。
失败原因码包括 `invalid_input`、`invalid_options`、`structural`、
`polygon_not_convex_ccw`、`empty_s_interval`、`negative_s_start`、
`unordered_polygons`、`coverage`、`coverage_gap`、`no_overlap`、
`obstacle_intersection`、`obstacle_vertex_inside`、`narrow`、`containment`，
并带 `failed_polygon` 与 `s_m` 定位。包含容差 `containment_tolerance_m` 默认
1e-6 m（仅浮点保护量）。

## 复现命令

```sh
python3 tools/task_runner.py build --component corridor --profile release --parallel 4 \
    --build-dir build/corridor
python3 tools/task_runner.py test --component corridor --workers 1 \
    --build-dir build/corridor
# 示例（构建目录下）：
#   planner_corridor_example --self-check
#   planner_corridor_example --list-scenarios
#   planner_corridor_example --scenario <name> [--output <file>]
```

最终证据：build `20261010T050720149721Z_corridor_3d2bd5d7`（0 告警）、
test `20261010T050724118547Z_corridor_5bd960e1`（55/55）。
失败链已保留：test `8c1202d7`（测试支持符号 bug，9.04 m 假越界）、
`c9f45562`（修复前复现 3.6167 cm 越界）、`85ebcf89`（修复 v1 自检设计过严，
`generation_is_deterministic` 返回 `no_corridor`）；规范目录 `build/release`
多次构建被 map 组件陈旧对象阻塞（跨组件流程竞态，非本组件缺陷），走廊验收链
使用隔离目录 `build/corridor`。

## 已知限制

1. 鲁棒性保证仅限 `CorridorOptions` 配置界内的扰动（默认 1°/2 cm）；界外无
   保证，`verify_corridor` 本身也不检查扰动界。
2. 贴墙种子语义：保守足迹与障碍的间隙小于构造包络（测试车辆 ≈ 8.95 cm）时，
   膨胀后的种子多边形触及障碍，生成显式返回 `blocked`/`narrow`（不再输出对
   扰动退化的“刀口”走廊）；该边界的成功/失败结果与旧版不同。
3. 无正式性能基准（§7.3 未要求）、无并发/线程安全验证。
4. 仅支持静态凸障碍（矩形/凸多边形）；非凸/退化输入明确拒绝。
5. 坐标系与地图版本一致性由调用方保证（`map_id`/`map_version` 原样透传；
   validation 侧会检查与场景栅格的不匹配）。

## 报告

- 入口：[Docs/reports/corridor/latest.md](../../Docs/reports/corridor/latest.md)
- corridor-002 修复与验收：[20261010T054750886642Z_corridor_eaa7be7e.md](../../Docs/reports/corridor/20261010T054750886642Z_corridor_eaa7be7e.md)
- corridor-001 修复：[20261010T023049716506Z_corridor_3a1872bd.md](../../Docs/reports/corridor/20261010T023049716506Z_corridor_3a1872bd.md)
- 首版验收（Wave 4，53/53）：[20261009T181558133161Z_corridor_1e9af40b.md](../../Docs/reports/corridor/20261009T181558133161Z_corridor_1e9af40b.md)
- 问题跟踪：[ISSUES.md](../../Docs/reports/ISSUES.md)（corridor-001/corridor-002 均 `resolved`）
