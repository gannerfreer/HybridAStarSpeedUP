# planner_collision（碰撞检测：位姿 / 运动段 / 路径 + 独立参考与随机差分）

依据 `Docs/component_development_plan.md` §6.1、§6.2、§7.1～§7.3 交付。
当前状态：朴素几何路径为基线（加速可选、默认关闭）；组件测试 85/85 通过；
100k 正式随机差分验收 100/100 种子、0 漏检 / 0 误报（15 inconclusive 全部补样）；
正式性能基准 `planner_collision_bench --suite v1` 已交付（8 配置 × 30 = 240 测量
trial，全部 success/valid）。本组件是 VALIDATION、Hybrid A* 与 Dftpav 链路的
碰撞安全基础。

## 目录

| 路径 | 内容 |
| --- | --- |
| `include/planner/collision/collision_checker.hpp` | 公开 API：`check_pose`/`check_motion`/`check_path`、`CollisionChecker`、`CheckResult`/`CheckOptions`/`MotionSegment` |
| `include/planner/collision/scene.hpp` | 场景描述：`Scene`/`Obstacle`/`Footprint`/`GridFrame` 与足迹校验 |
| `include/planner/collision/geometry.hpp` | 2D 几何工具箱：AABB、凸多边形相交/距离/凸包、尺度感知接触容差 |
| `include/planner/collision/reference.hpp` | 独立参考实现（不复用生产加速结构；精度下限处给出 inconclusive） |
| `include/planner/collision/differential.hpp` | 确定性随机用例生成器（regime/分层/补样），与 `random_spec.json` 对应 |
| `src/` | 实现（`geometry.cpp`、`scene.cpp`、`collision_checker.cpp`、`reference.cpp`、`differential*.cpp`） |
| `tests/` | GoogleTest（`CheckPose`/`CheckMotion`/`CheckPath`/`Geometry`/`Reference`/`CheckInvalidInput`/`AcceptanceRegression`/`AccelerationEquivalence`/`DifferentialSmoke`，CTest 标签 `collision`）+ 3 个差分集成 CTest 用例 |
| `examples/check_scene.cpp` | 命令行场景检查器 `planner_collision_check_scene`（`--scene FILE` / `--demo`） |
| `examples/random_differential.cpp` | 差分运行器 `planner_collision_differential`（dev/acceptance/smoke） |
| `examples/collision_bench.cpp` | 微基准 `planner_collision_bench`（`--suite v1 --output-dir <dir>`） |
| `CMakeLists.txt` | 组件库、示例与基准目标 |

## 公开接口

- `check_pose(pose, vehicle, scene, options)`、`check_motion(MotionSegment, ...)`、
  `check_path(PathSegment | vector<PathSegment>, ...)`：返回 `CheckResult`。
- `CheckResult`：`status`（`kClear`/`kCollision`/`kInvalidInput`）、`reason_code`
  （`kObstacle`/`kGridOccupied`/`kGridUnknown`/`kOutsideMap`/`kPrecisionFloor`/
  `kInvalidInput`）、`detail`、`first_pose`、`first_s_m`、`path_segment_index`；
  辅助判定 `collided()`/`is_clear()`/`is_valid()`。
- `CheckOptions`：`sweep_precision_m`（默认 1e-3 m，扫掠细分下限）、
  `enable_acceleration`（默认 false；启用后结果与纯几何路径一致，由差分对比与
  单测验证）。
- `CollisionChecker`：构造时校验车辆/场景/选项一次并可选预计算加速结构，可复用；
  构造被拒绝时 `valid() == false` 且所有检查返回 `kInvalidInput`
  （`invalid_reason()` 给出原因）；使用期间不得修改输入快照。
- `MotionSegment`：起始位姿、弧长、挡位与带符号曲率；
  `yaw_change = curvature * signed_length`（前进取 `+length`，倒车取 `-length`）。
- `scene.hpp`：`Obstacle::rectangle`/`convex_polygon`（顶点归一化为逆时针，非法
  输入返回空顶点）、`Footprint`（整车矩形，含安全裕量；v1 默认车辆包络
  4.9 m × 2.0 m）、`GridFrame`（世界 ↔ 栅格原点坐标系、地图边界距离）、
  `vehicle_footprint_valid`/`make_footprint`。
- `geometry.hpp`：`contact_tolerance`（尺度感知，最大 1e-12·scale）、AABB 与
  点/段/凸多边形距离、凸多边形相交/凸包/逆时针归一化等。
- `reference.hpp`：`reference_check_pose/motion/path` 与 `ReferenceVerdict`
  （`kCollision`/`kClear`/`kInconclusive`/`kInvalidInput`）。
- `differential.hpp`：`RegimeSpec`（dev 10,000 / acceptance 每种子 1,000）、
  五类对抗分层（角点擦碰、精确接触、薄障碍、倒车运动、地图边缘）、
  `generate_case`、`production_check`、`reference_check`；用例可由
  `(regime.seed, ordinal, attempt)` 复现。

## 正确性与语义约定（§6.1/§6.2/§7.1）

### 语义约定

- 三态：`kClear`（未发现碰撞）/`kCollision`（含接触）/`kInvalidInput`（输入被拒绝，
  无判定结果）。
- 接触（相切）计为碰撞：两形状接触当且仅当最小**欧氏**距离 ≤ `contact_tolerance(scale)`
  （尺度感知容差，最大 1e-12·scale）；生产检查器与独立参考使用同一策略，使接触
  样本得到一致分类。
- 栅格语义（冻结 v1 约定）：单元取值 0=空闲、1=占据、2=未知；**未知单元与地图矩形
  之外的一切区域均视为占据**；`origin` 为单元 (0,0) 左下角。
- 运动检查：自适应细分 + 保守位移包络；包络无法证明无碰撞时继续细分，在车辆任一点
  位移上限 1e-3 m 以下仍未解决的区间报告为碰撞（`kPrecisionFloor`，保守方向）。
- `check_path` 还会在相邻路径点之间扫掠：过渡段由首个路径点存储的曲率、弧长差与
  分段方向重建为常曲率圆弧；存储点位姿始终精确检查（仅查离散点会漏掉采样点之间的
  薄障碍物）。
- 足迹：后轴中心位姿下的整车矩形，安全裕量计入；默认测试车辆（轴距 2.8 m、
  宽 1.8 m、前后悬 0.9/1.0 m、裕量 0.1 m）检查 4.9 m × 2.0 m 包络。

### 验收结果

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 组件测试（85 例） | 85/85，0 failure / 0 error / 0 skipped | test `20261010T072340369459Z_collision_eb873310` |
| 开发差分 10k（dev，seed 1000，`--accel both`） | 10,003 生成 / 10,000 可判定；0 漏检 / 0 误报；inconclusive 3（补样）；加速/窗口不一致 0 | `artifacts/collision_differential/20261009T170844Z_1376417/summary.json` |
| 正式 100k 验收（种子 0–99） | 100/100 种子；100,015 生成 / 100,000 可判定；真碰撞 58,589；**0 漏检 / 0 误报**；inconclusive 15（全部补样） | `artifacts/collision_differential/acceptance_20261009T173329Z_1415514/rollup.json` |
| 加速等价 | 两轮验收与 dev 均 `acceleration_mismatches=0`、`window_mismatches=0` | 上述各 rollup/summary |
| 验收回归 | 种子 04/82 与角点接触策略用例固化（`AcceptanceRegression` 5 例） | `tests/test_regression_acceptance.cpp` |

- 85 例构成：CheckPose 16 / CheckMotion 15 / CheckPath 7 / Geometry 10 / Reference 9 /
  InvalidInput 18 / AcceptanceRegression 5 / AccelerationEquivalence 1 /
  DifferentialSmoke 1 + CTest 差分集成 3（smoke 300 例、验收单种子、dev 10k）。
- 首轮 100k 验收 2 例漏检（种子 04/82）源于参考侧接触判据过宽，已统一为
  “接触 ⇔ 最小欧氏距离 ≤ 尺度感知容差”并回归固化（`collision-001`，resolved）；
  首轮失败证据保留：`artifacts/collision_differential/acceptance_20261009T172336Z_1399037/`，
  另有一次规范校验中止的尝试 `acceptance_20261009T171656Z_1387430/` 同样保留。
- 100k 为固定种子集合（0–99），**不构成形式化安全证明**；“零漏检”仅指该批次测试。

## 正式性能基准（§7.2/§7.3）

`planner_collision_bench --suite v1`（run `20261010T072417723044Z_collision_246281dd`；
独占窗口 70.4 s；预热 5 / 测量 30，顺序单线程；输入生成与检查器构建在计时外；
seed 20261010；240/240 测量 trial 全部 success/valid，`summary.json passed=true`）：

| 配置 | 操作 / 场景 / 模式 | 批大小 | P50 批耗时 (ms) | P95 批耗时 (ms) | 摊还 (µs/op) |
| --- | --- | --- | --- | --- | --- |
| pose_typical | check_pose / typical / plain | 10,000 | 12.899 | 13.359 | 1.290 |
| pose_dense | check_pose / dense / plain | 10,000 | 41.792 | 42.531 | 4.179 |
| pose_dense_accel | check_pose / dense / accelerated | 10,000 | 36.729 | 36.998 | 3.673 |
| motion_typical | check_motion / typical / plain | 10,000 | 74.017 | 74.433 | 7.402 |
| motion_dense | check_motion / dense / plain | 10,000 | 724.271 | 726.752 | 72.427 |
| motion_dense_accel | check_motion / dense / accelerated | 10,000 | 686.980 | 692.046 | 68.698 |
| path_dense | check_path / dense / plain | 2,000 | 209.697 | 210.921 | 104.848 |
| pose_dense_cold | check_pose / dense / plain_cold | 1,000 | 140.401 | 142.210 | 140.401 |

- **P50/P95 为每批耗时；`ns/op`、`ops/s` 为批均值摊还值，不是单次调用 P95**
  （契约见 `tools/README.md` §7.2）。
- `cold` 语义为每次操作内重建检查器（含场景校验），非物理冷盘；`accelerated`
  为显式加速模式（默认仍关闭，未改变正确性语义）。
- 基准正确性冒烟（计时外）：手算接触用例（typical，`first_s ≈ 2.2 m`）、朴素 vs 加速
  逐批聚合完全一致（0 差异）、每批聚合与库逐例结果一致（`checks.failures=[]`）。
- 场景：typical 40 m × 40 m / 2 障碍；dense 60 m × 60 m / 87 障碍；分辨率 0.1 m；
  车辆 `default_test_vehicle`。

## 复现命令

```sh
python3 tools/task_runner.py build --component collision --profile release --parallel 4
python3 tools/task_runner.py test --component collision --workers 1
python3 tools/task_runner.py benchmark --component collision --suite v1
# 命令行示例：<build_dir>/Src/collision/planner_collision_check_scene --demo
# 开发差分 10k 已注册为 CTest 用例（标签 collision），随 test 子命令运行。

# 100k 正式验收（种子 0–99 串行；Wave 4 为受监控直跑，非 task_runner run）
for s in $(seq 0 99); do
  build/release/Src/collision/planner_collision_differential \
    --regime acceptance --seed "$s" --accel both \
    --spec tests/fixtures/v1/collision/random_spec.json \
    --output-dir artifacts/collision_differential/<run_id>/seed_$(printf '%02d' "$s") --quiet
done
```

一次性验收任务也可按 `tools/README.md` §3 经
`python3 tools/task_runner.py exec --component collision --workers 1 -- ...`
统一登记与封存；本组件已记录的 100k 证据来自上述直跑（`tools-003` 已 resolved）。
验收规范 `tests/fixtures/v1/collision/random_spec.json` 的 SHA256 为
`ae43474b42aad9bbb10b000e6e4f405fa9e831c91b2e27bff0509122def4c1f5`
（与 `tests/fixtures/v1/fixture_manifest.sha256.json` 一致）。

最终证据：build `20261010T072143907065Z_collision_fe74dbbd`（exit 0、无告警）、
test `20261010T072340369459Z_collision_eb873310`（85/85）、
benchmark `20261010T072417723044Z_collision_246281dd`（240/240）。早期编译失败
`20261010T071738838752Z_collision_25ee964a` 与中间构建
`20261010T071810502342Z_collision_deb36bf2` 保留为历史证据。

## 已知限制

1. 100k 是种子 0–99 的固定集合，不构成形式化安全证明；“零漏检”仅指该批次测试。
2. 加速路径默认关闭（`CheckOptions::enable_acceleration=false`）；`*_accel` 配置仅
   测量显式加速模式，未改变默认行为，报告未单列加速相对收益（仅验证等价）。
3. `cold` 为每次操作内重建检查器（含场景校验），非物理冷盘；OS 文件缓存未清理、
   无磁盘 I/O 计入。
4. P50/P95 为每批耗时，`ns/op`/`ops/s` 为摊还值，不得当作单次调用 P95。
5. v1 套件的 8 个配置不构成跨组件/硬件横向排名；基准不生成差分案例，差分指标沿用
   Wave 4 正式验收。
6. 参考实现与生产检查器共享接触判据（同一容差公式），差分验证的是实现一致性而非
   独立几何模型；参考 inconclusive 不充当真值。

## 报告

- 入口：[Docs/reports/collision/latest.md](../../Docs/reports/collision/latest.md)
- 正式性能基准：[Docs/reports/collision/20261010T100000445099Z_collision_91af387c.md](../../Docs/reports/collision/20261010T100000445099Z_collision_91af387c.md)
- 100k 差分验收（Wave 4）：[Docs/reports/collision/20261009T181421120582Z_collision_eb8b4f83.md](../../Docs/reports/collision/20261009T181421120582Z_collision_eb8b4f83.md)
- 问题跟踪：[Docs/reports/ISSUES.md](../../Docs/reports/ISSUES.md)（`collision-001` 已 resolved）
