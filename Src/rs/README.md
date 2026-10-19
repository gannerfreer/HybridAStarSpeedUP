# planner_rs（Reeds–Shepp 路径库：原生解析求解器 + OMPL 1.6.0 对照适配器）

依据 `Docs/component_development_plan.md` §3.7、§6.1、§6.2、§7.1～§7.3 交付。
当前状态：交付求解器为原生解析式 48 曲线枚举，公开 API 不暴露任何 OMPL 类型；
OMPL 1.6.0 仅作私有封装一致性对照（`planner_rs_ompl_reference`，开关
`PLANNER_RS_WITH_OMPL` 默认 ON）。组件测试 54/54 通过；基准 44 例 × 5 项操作
全部通过（`checks.all_ok=true`），与 OMPL 对照最大绝对误差 2.13e-14 m，终点重建
最大位置误差 3.59e-11 m、航向误差 1.92e-12 rad（容差 1e-6）。RS 可作为 Hybrid A*
的正式 RS 提供方（Hybrid 集成验收另行记录）。

## 目录

| 路径 | 内容 |
| --- | --- |
| `include/planner/rs/reeds_shepp.hpp` | 公开 API：`RsSegment`/`RsPath`/`RsPlan`、`plan`/`distance`/`path`、`integrate_segment`/`sample`/`sample_at_s`、冻结容差 |
| `src/reeds_shepp.cpp` | 原生解析求解器：48 曲线枚举（CSC/CCC/CCCC/CCSC/CCSCC）、单位半径归一化、终点硬验证 |
| `src/path_sampling.cpp` | 弧长采样：规则 ds 网格与尖点边界合并、`sample_at_s` 精确求值、采样数上限 |
| `src/ompl_reference.{hpp,cpp}` | 私有 OMPL 1.6.0 对照适配器（不进入公开 include 树，仅测试/基准链接） |
| `tests/` | GoogleTest（`ReedsSheppDistance`/`Path`/`Sampling`/`Transform`/`InvalidInput`/`OmplConsistency`），CTest 标签 `rs` |
| `benchmarks/rs_bench.cpp` | 微基准 `planner_rs_bench`（`--suite v1 --output-dir <dir>`） |
| `examples/rs_demo.cpp` | 命令行示例；`--self-check` 注册为 CTest 用例 |
| `CMakeLists.txt` | 组件库、可选对照库、示例与基准目标；OMPL 经 `dependencies.lock.json` 锁定 |

## 公开接口

- `RsSegment`：常曲率分段（`direction`、`length_m >= 0`、
  `curvature_inv_m = tan(steer)/wheelbase`）；路径按行进顺序排列，方向变化（尖点）
  总是开启新分段；每个 `plan()` 结果满足曲率绝对值不超过 `1 / min_turn_radius_m`。
- `plan(start, goal, vehicle)`：从不抛异常；返回 `kSuccess` 前对终点做
  1e-6 m / 1e-6 rad 硬验证（`kEndpointPositionToleranceM`/`kEndpointYawToleranceRad`），
  超出时返回 `kOptimizationFailed`（绝不静默接受）；无效输入返回 `kInvalidInput`。
- `distance`/`path`：`plan` 的抛异常便捷包装；`is_valid_query` 与 `min_turn_radius_m`
  提供查询校验与最小转弯半径计算。
- `integrate_segment`：沿分段推进 `ds_m ∈ [0, length_m]`，超限以 `std::invalid_argument` 拒绝。
- `sample(path, ds_m)`/`sample_at_s(path, s_m)`：首末点精确（`s = 0` / `s = length_m`）、
  包含全部尖点边界（边界点携带自该处开始的分段方向/曲率）、相邻点间距不超过 `ds_m`；
  采样数上限 2e7，超限或畸形路径拒绝。
- 几何约定（冻结契约，见 `tests/fixtures/v1/serialization/`）：`Pose2d` 表示后轴中心
  （x 向前、y 向左、yaw 逆时针为正），角度归一化到 `[-pi, pi)`；
  `yaw_rate = signed_speed * curvature_inv_m`，曲率符号描述物理转向方向（正 = 左转），
  与挡位无关。

## OMPL 1.6.0 对照与正确性（§3.7/§7.1）

### 对照定位与独立证据

- 原生求解器沿用 OMPL 1.6.0 `ReedsSheppStateSpace.cpp`（BSD-3-Clause，Copyright (c)
  2010 Rice University）的结构与公式，实现 Reeds & Shepp (1990) 的同一 48 曲线族；
  重写去除了全部 OMPL 类型。两侧实现同一曲线族，因此与 OMPL 的比较是
  **封装一致性测试**，明确不是独立双重验证（§3.7）。
- 独立证据来自：解析直线/圆弧（`quarter_turn_matches_min_radius_arc`、
  `uturn_at_min_radius_is_pi_times_radius`、`cusp_case_matches_pi_times_radius` 等）、
  刚体变换不变性（`distance_invariant_under_rigid_transforms`、
  `path_endpoint_maps_to_transformed_goal` 等 4 例）、RK4 定步长（1e-3 m）分段积分
  终点重建（基准 44 例）、曲率上限（最大比值 1.0，不超限）。

### 开关与依赖

- `PLANNER_RS_WITH_OMPL`（默认 ON）：OFF 时不 FetchContent、不下载 OMPL，测试与基准
  也不链接对照库，组件只构建原生求解器（离线/受限构建应急开关）。
- 锁条目 `ompl` 1.6.0（commit `e2994e580fcba0eb117e38bd1dc7439c4beb35ef`）；上游构建
  关闭 tests/demos/pybindings/registration。
- 系统 Eigen3（X-A1-02）：OMPL 自身构建经 `find_package` 消费系统 Eigen3（≥3.3），
  适配器优先链接同一个 `Eigen3::Eigen`，以保证与 OMPL 编译时使用的头文件一致；
  锁条目 `eigen`（3.4.0）只服务 bspline/polynomial 的对照后端，不在此混用；
  仅当系统 Eigen 不可用且锁定头文件目标已存在时才回退到后者。

### 数值证据（基准 run `20261009T160325631053Z_rs_dde8519c`，44 例）

| 检查 | 结果 | 容限 |
| --- | --- | --- |
| 终点重建（RK4，44 例） | 最大位置误差 3.59e-11 m；最大航向误差 1.92e-12 rad | 1e-6 m / 1e-6 rad |
| 与 OMPL 对照一致性（44 例） | 44/44 一致；最大绝对误差 2.13e-14 m，最大相对 3.71e-16 | `abs <= 1e-6 + 1e-6 * ref`（本例限值 5.85e-5 m） |
| 曲率约束（44 例路径） | 最大比值 1.0（不超限） | `1/R`（R ≈ 5.1254 m → 0.195108 1/m） |
| 组件测试 | 54/54，0 failure / 0 error | N/A |

- 44 例 = 12 个命名场景（同位姿、直行/倒车、最小半径转弯、换向、角度跨界、远距目标、
  倒车泊车）+ 32 个确定性随机位姿对（seed 20261009）。
- 测试 54 例构成：Distance 12 / Path 12 / Sampling 12 / InvalidInput 10 / Transform 4 /
  OMPL 一致性 3 / 示例自检 1；开关 OFF 时 OMPL 一致性 3 例标记为跳过（`GTEST_SKIP`）。

### 性能对照（P50；预热 5 / 测量 30，单线程）

求解器无状态（cold = warm），批大小为 `distance` 1,000、`path` 200、`sample` 100。

| 操作 | 原生（ns/op） | OMPL 封装（ns/op） | 相对变化 | 原生批 P95 |
| --- | --- | --- | --- | --- |
| `distance` | **1785.433** | 2413.842 | −26.0% | 1.826 ms/批（1000 次） |
| `path` | **1813.025** | 2597.380 | −30.2% | 0.380 ms/批（200 次） |
| `sample` | **2299.98** | N/A（无对照） | N/A | 0.233 ms/批（100 次） |

有效测量行 6,690（223 批 × 30；`sample` 跳过 2 个零长度路径案例）。

## 复现命令

```sh
python3 tools/task_runner.py build --component rs --profile release --parallel 4
python3 tools/task_runner.py test --component rs --workers 1
python3 tools/task_runner.py benchmark --component rs --suite v1
# 示例自检：<build_dir>/Src/rs/planner_rs_demo --self-check（亦为 CTest 用例）
# 离线构建：cmake ... -DPLANNER_RS_WITH_OMPL=OFF（仅构建原生求解器）
```

最终证据：build `20261009T160407619277Z_rs_4ead6555`（exit 0）、
test `20261009T160434956019Z_rs_13efd51c`（54/54）、
benchmark `20261009T160325631053Z_rs_dde8519c`（44 例 × 5 项操作，`all_ok`）。
失败链已保留：build `80b3e311`/`09bfde68`（OMPL 传递依赖 `Eigen/Core` 包含路径）、
build `d7239501`（缺头文件与 `using` 声明）、test `246220e9`（49/53，修复后重试）；
中间通过版本 test `bb90c214`（53/53）。

## 已知限制

1. 与 OMPL 的对照是封装一致性，不是独立双重验证（两侧实现同一曲线族）。
2. ≤1e-14（单位半径）的舍入幽灵原始段会被丢弃，对距离/路径的影响 ≤ 5e-14·R。
3. `sample` 采样数上限 2e7，超限拒绝。
4. 多线程与跨平台未测：基准为单线程、单机 Linux x86_64（16 逻辑核）。
5. OMPL 上游归档字节稳定性提示项：归档变化需重新锁定哈希。

## 报告

- 入口：[Docs/reports/rs/latest.md](../../Docs/reports/rs/latest.md)
- 验收与实验：[Docs/reports/rs/20261009T181421074430Z_rs_8e933944.md](../../Docs/reports/rs/20261009T181421074430Z_rs_8e933944.md)
- 问题跟踪：[Docs/reports/ISSUES.md](../../Docs/reports/ISSUES.md)（当前无 RS 条目）
