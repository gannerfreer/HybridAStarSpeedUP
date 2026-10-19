# planner_bspline（B-spline 曲线库：TinySpline 基线与内部对照后端）

依据 `Docs/component_development_plan.md` §3.3/§6.1/§6.2/§7.1/§7.3 交付。
当前状态：默认后端保持 `tinyspline`；`eigen_splines` 与 `ego_uniform`
（EGO-Planner UniformBspline，GPL-3.0 隔离对照库）为内部对照后端；
测试 82/82 通过；v1 基准 54 用例（3 后端 × 18），跨后端一致性
5.669e-08 ≤ 1e-6，等弧长最大区间误差 1.815e-09 m ≤ 1e-3 m。
`bspline-001`（EGO 对照缺口）本轮已完成对照（见下文“EGO 对照结论”）。

## 目录

| 路径 | 内容 |
| --- | --- |
| `include/planner/bspline/curve.hpp` | 公开 API：`Bspline`、`BackendKind`、`ParameterKind`、弧长重采样与 `clamped_uniform_knots` |
| `src/curve.cpp` | 公开语义：构造校验、参数映射、右侧分段求值、批量采样与后端派发 |
| `src/tinyspline_backend.cpp` | 基线后端（TinySpline 0.6.0，缩放域隔离其 1e-4 节点 epsilon） |
| `src/eigen_backend.cpp` | 对照后端（Eigen 3.4.0 unsupported/Splines） |
| `src/ego_backend.cpp` + `src/upstream_ego/` | EGO-Planner UniformBspline 对照后端与上游复制（GPL-3.0，见 `PROVENANCE.md`） |
| `src/resample.cpp` | 弧长积分与近似等弧长重采样 |
| `tests/` | GoogleTest（`BsplineBackends`、`BsplineBackendEgo` 等），CTest 标签 `bspline` |
| `benchmarks/bspline_bench.cpp` | 微基准 `planner_bspline_bench`（`--suite v1 --output-dir`，`--backend all|both|tinyspline|eigen|ego`） |
| `examples/bspline_demo.cpp` | 命令行示例 |

## 公开接口

- 显式控制点 + 显式节点向量构造（次数由规模推导），拒绝非法输入；
  内部节点处右侧分段、域末点取最后分段；参数域外拒绝（1e-12 相对舍入容差）；
- `evaluate(parameter, order)`（0..3 阶）与 `sample()` 批量求值；
- 几何/时间参数区分（`ParameterKind`，时间参数按链式法则返回对时间导数）；
- `total_length_m()` 与近似等弧长重采样（`sample_equal_arc_length` /
  `resample_uniform_interval`，区间误差可度量，容差 1e-3 m）；
- 公开 API 不暴露任何上游类型；后端仅以 `BackendKind` 出现。

## 后端与选型（§3.3）

### 后端清单

| 后端 | 名称 | 状态 | 说明 |
| --- | --- | --- | --- |
| TinySpline 0.6.0 | `tinyspline` | **默认后端** | 首版封装基线（MIT；缩放域隔离其绝对节点 epsilon） |
| Eigen 3.4.0 Splines | `eigen_splines` | `pending_optimization` | 对照候选（MPL-2.0；零拷贝导数字句） |
| EGO-Planner UniformBspline | `ego_uniform` | 对照后端（`pending_optimization`） | 仅对照：GPL-3.0 隔离库 `planner_bspline_ego`，只被测试/基准链接；上游 commit `bfda5128…` |

### 选型结论（2026-10-11，基准 run `20261011T082657308830Z_bspline_80438c4e`）

规则不变（曲线库版 §7.2 适配）：先过正确性门槛（跨后端 ≤1e-6）与质量门槛
（等弧长 ≤1e-3 m），再按求值用例批 P95 等权均值排名；领先者与基线差距
<5% 时比较全体用例均值，仍接近则保留基线。

| 后端 | 求值 P95 等权均值 (ms) | 全体 P95 等权均值 (ms) | 合格 | 裁定 |
| --- | --- | --- | --- | --- |
| `tinyspline`（基线） | **0.356227** | 22.6870 | 是 | **默认后端（不变）** |
| `ego_uniform`（对照） | 1.136232 | 53.1460 | 是 | `pending_optimization` |
| `eigen_splines`（候选） | 1.330453 | 53.0421 | 是 | `pending_optimization` |

- 正确性：基准内跨后端一致性最大绝对差 **5.669e-08** ≤ 1e-6
  （其中 `ego_uniform` 相对基线 9.786e-10，`eigen_splines` 相对基线
  5.669e-08，均在共享夹具曲线上、0..3 阶）；质量门槛：三后端等弧长最大
  区间误差均 1.815e-09 m ≤ 1e-3 m。
- 性能对照示例（P95 ms）：构造 cp8 0.0630/0.0245/0.0190，求值 cp32-d0
  0.3755/1.1472/1.1019，等弧长 cp128 173.81/530.13/397.34
  （tinyspline / ego_uniform / eigen_splines）。
- 测试：82/82（70 既有 + 12 新增 `BsplineBackendEgo`），既有对照通过标准与
  容差未改动；默认后端 `kDefaultBackend` 未改动。
- 基准运行：wall 81,873 ms、CPU 80,790 ms、任务树 RSS 峰值 8.88 MB；
  独占窗口采样 CPU 4.12–6.29%、内存 19.06–19.23%。

### EGO 对照结论（`bspline-001`）

**可行性**：可 ROS-free 抽离并接入对照。上游 `uniform_bspline.cpp` 对 ROS 的
唯一引用是未使用的 `#include <ros/ros.h>`（删除后无任何 `ros::`/`ROS_` 符号）；
头文件只依赖 Eigen。固定来源、复制范围与改动见
[`src/upstream_ego/PROVENANCE.md`](src/upstream_ego/PROVENANCE.md)。

**语义差异与适配**（对照测试 `BsplineBackendEgo` 锁定，未放宽任何容差）：

1. 上游在内部节点处取**左侧**分段（`u_(k+1) >= ub`），本组件契约要求右侧
   分段；适配层仅在参数恰为节点值时用 `nextafter(u, +inf)` 推入右侧分段
   （1 ulp 量级，对照测试实测偏差 ≤ 1.4e-13，容差 1e-9）。
2. 上游对域外参数**夹持**而非拒绝；适配层先做与 Eigen 后端一致的域检查。
3. 上游构造函数只生成均匀节点；任意显式节点经公开 `setKnot` 注入。
4. 上游位置间断（重数 degree+1）的导数控制点含 `inf` 项，但其求值分段选择
   不会引用这些项；gap/C0/重复节点用例已由对照测试覆盖。

**数值证据**：`ego_uniform` 相对独立参考（右侧分段 de Boor + hodograph）在
非均匀节点用例最大偏差 **1.35e-13**；相对基线 `tinyspline` 在基准夹具上
最大偏差 **9.786e-10**。注意：`tinyspline` 的节点 epsilon 会把 1e-12 参数
吸附到节点（`impl.hpp` 记录），在高三阶导数曲线（~1.2e3）上产生 ~1.18e-9
偏差；因此非均匀用例与独立参考对照（容差仍为 1e-9），不构成 EGO 的误差。

**性能**：求值批 P95 等权均值约为基线的 3.2 倍（上游逐点求值、每次调用分配
Eigen 向量），不改变默认后端。

### GPL-3.0 隔离说明

- EGO 上游代码（`src/upstream_ego/`）为 GPL-3.0，只编译进独立静态库
  `planner_bspline_ego`，由 `planner_bspline_tests` 与 `planner_bspline_bench`
  链接；`planner_bspline` 主库不编译、不链接该目录，通过运行时注册的工厂
  （公开头文件 `detail::install_ego_backend()`，实现由对照库提供）按需获得
  后端。未注册时构造 `kEgoUniform` 曲线会失败并给出说明。
- 该后端仅用于内部选型对照。未做许可证决策前，不得把 `planner_bspline_ego`
  链接进交付组件；若未来产品路径需要 EGO 实现，须单独评审 GPL 影响。
- 来源锁定：`dependencies.lock.json` → `ego_planner`（commit、归档 URL 与
  SHA256 `848e6cbe…`、许可证路径）。

## 复现命令

```sh
python3 tools/task_runner.py build --component bspline --profile release --parallel 4 \
    --build-dir build/w15-bspline
python3 tools/task_runner.py test --component bspline --workers 1 \
    --build-dir build/w15-bspline
python3 tools/task_runner.py benchmark --component bspline --suite v1 \
    --build-dir build/w15-bspline
# 基准二进制可显式选择后端：planner_bspline_bench --suite v1 \
#   --backend all|both|tinyspline|eigen|ego --output-dir <dir>
```

最终证据：build `20261011T090159078602Z_bspline_a29b5faf`（0 告警）、
test `20261011T090202864760Z_bspline_fe5cefcb`（82/82）、
benchmark `20261011T082657308830Z_bspline_80438c4e`（54 用例，selection.json）。
失败链已保留：build `9102828b`（新构建目录首次配置时 nlohmann_json 下载
瞬时 SSL 失败）、build `93a2e93e`（上游 `evaluateDeBoor` 非 const 的编译错误 +
上游告警）、test `881babc4`（81/82，非均匀用例 TinySpline epsilon 吸附）。

## 已知限制

1. EGO 对照后端仅满足共享子集语义（连续曲线、显式节点、右侧分段经适配层
   补偿）；它不是通用 B-spline 库，不参与默认后端替换评估。
2. Eigen 候选未做批量快速路径（逐点基函数求值），提升空间在候选侧。
3. 微基准为 warm（输入预热前构造），无冷启动对照。

## 报告

- 内部选型（TinySpline vs Eigen）：`Docs/reports/bspline/20261010T150507859812Z_bspline_da8e3be8.md`
- 入口：`Docs/reports/bspline/latest.md`；问题跟踪：`Docs/reports/ISSUES.md`（`bspline-001`）
