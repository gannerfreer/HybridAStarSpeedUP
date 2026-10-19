# planner_polynomial（三次/五次多项式构造与分段轨迹求值）

依据 `Docs/component_development_plan.md` §3.4、§6.1、§6.2 与 §7.3 交付。当前状态：
默认实现 `planner_horner`（闭式边界求解 + Horner 求值/解析导数）；内部对照候选
`mav_trajectory_generation` 求值核心已接入（登记 `pending_optimization`）；
正确性 66/66（test `20261010T151553415914Z_polynomial_6a8c2935`）；基准 18 用例、
双后端、跨后端一致性最大绝对差 0.0（benchmark `20261010T151559522314Z_polynomial_b00ca49b`）；
`polynomial-001`（GCOPTER/MINCO 表示级/构造级对照评估）已按“不新增无意义比较”
完成决策记录（见“选型与对照”）。

## 目录

| 路径 | 内容 |
| --- | --- |
| `include/planner/polynomial/polynomial.hpp` | 系数容器（升幂）、三次/五次闭式边界求解、Horner 求值、解析导数、批处理、时间缩放 |
| `include/planner/polynomial/trajectory_segment.hpp` | 二维分段：每轴一个三次或五次多项式，在局部时间 `[0, duration_s]` 求值 |
| `include/planner/polynomial/piecewise_trajectory.hpp` | 分段轨迹（全局时间、衔接点连续性校验、批处理、时间缩放） |
| `include/planner/polynomial/result.hpp` | `Result`/`BatchResult`、稳定 `Code`、`kMaxDerivativeOrder = 3` |
| `src/` | 实现（`polynomial.cpp`、`trajectory_segment.cpp`、`piecewise_trajectory.cpp`、`bench_main.cpp`） |
| `compare/mav_segment_evaluator.{hpp,cpp}` | 内部对照求值器（mav_trajectory_generation 求值核心；不随包安装、不进公开 API） |
| `tests/` | 正确性、分段/连续性、mav 对照、独立高斯消元参考、基准夹具输入检查（CTest 标签 `polynomial`） |
| `examples/polynomial_demo.cpp` | 命令行示例（`--self-check` / `--samples N`） |
| `CMakeLists.txt` | 组件库、对照库、示例与基准目标；Eigen/glog/mav 经 `dependencies.lock.json` 锁定 |

## 接口语义（§6.1/§6.2）

- 系数按升幂保存，时间轴为分段局部时间（秒）；求值使用 Horner 法则，导数为解析
  形式（系数向量按阶乘比缩放）。
- 求导阶数首版为 0～3；时间超出 `[0, duration_s]` 返回 `kOutOfRange`，不隐式外推；
  内部衔接点选择右侧（较晚）分段，最终端点选择末段。
- `PiecewiseTrajectory2d::create` 按 `ContinuityPolicy`（`kAutomatic`：两个三次之间
  要求 C1，任一侧为五次要求 C2）在 `continuity_tolerance` 内校验衔接点，违反时返回
  `kInvalidInput`。
- 时间缩放 `scaled_time` 保持路径形状、拉伸时间轴；批处理在第一个被拒绝输入处停止并
  报告索引。
- 预期输入问题不抛异常：工厂与求值返回带稳定 `Code` 的 `Result`，可映射到共享
  `planner::common::Status`。

## 构建与测试

```sh
python3 tools/task_runner.py build --component polynomial --profile release --parallel 4
python3 tools/task_runner.py test --component polynomial --workers 1
python3 tools/task_runner.py benchmark --component polynomial --suite v1
```

- 基准二进制可显式选择后端：
  `planner_polynomial_bench --suite v1 --backend both|horner|mav --output-dir <dir>`。
- 最近成功证据：test `20261010T151553415914Z_polynomial_6a8c2935`（66/66，含 6 个
  新增对照/独立参考用例）；benchmark `20261010T151559522314Z_polynomial_b00ca49b`
  （18 用例 × 30 批，写 `selection.json`）。报告见
  [latest.md](../../Docs/reports/polynomial/latest.md) 与
  [20261010T151559522314Z_polynomial_b00ca49b.md](../../Docs/reports/polynomial/20261010T151559522314Z_polynomial_b00ca49b.md)。

## 选型与对照

### 当前选型状态（mav_trajectory_generation 对照）

| 项 | 值 |
| --- | --- |
| 默认实现 | `planner_horner`（闭式求解 + Horner 求值；§6.2“多项式”行） |
| 对照候选 | `mav_trajectory_generation` @ `7aeebd9f819217502fd1c5489da49efadf90d411`（Apache-2.0；求值核心，`pending_optimization`） |
| 正确性门槛 | 共享系数/时间点/阶数 0..3 的最大绝对差 ≤ 1e-6；实测 **0.0** |
| 质量门槛 | 候选 8 个求值用例全部完成；构造对比 N/A（mav 无等价闭式边界求解 API） |
| 排名规则 | 求值用例批 P95 等权均值；差距 <5% 时比较全体用例均值，仍接近保留基线 |
| 基准结果 | 求值 P95 等权均值 0.1847 ms（基线）vs 0.2240 ms（mav，慢 21.2%） |
| 独立参考 | 闭式三次/五次系数与测试内带主元高斯消元一致（1e-9，绝对+相对）；对应 §3.4 PythonRobotics 五次公式参照角色 |

门槛与默认后端在本轮评估中保持不变；完整数据与失败链见上述报告。

### `polynomial-001` 决策记录：GCOPTER/MINCO 不接入对照

评估日期 2026-10-11（网络可用，读取上游源码与接口快照）：

| 项 | 值 |
| --- | --- |
| 上游 | `https://github.com/ZJU-FAST-Lab/GCOPTER` |
| 评估 commit | `e0444f6d47b84f972ced91746b05feb36ce1fd4f`（main 最新提交，2023-06-06）；上游另有 tag `v0.0`（`4024bd2ae4ca65b210892786e7c88ef2bd5ccfdc`） |
| 许可证 | **MIT**（仓库根 `LICENSE`；`minco.hpp`/`trajectory.hpp` 文件头同为 MIT，Copyright 2021 Zhepei Wang / Fei Gao） |
| MINCO 表示与实现 | `gcopter/include/gcopter/minco.hpp`（`MINCO_S2NU/S3NU/S4NU`：稀疏参数化 `(q, T)` + 边界状态，经最优性条件用无主元带状 LU 直接求解系数，O(N)）+ `gcopter/include/gcopter/trajectory.hpp`（`Piece<D>`/`Trajectory<D>`，显式系数求值辅助类型） |
| 优化器层（求值不需要） | `gcopter.hpp`、`lbfgs.hpp`、`sdlp.hpp`、`firi.hpp`/`sfc_gen.hpp`、`geo_utils.hpp`、`quickhull.hpp`：时间分配、连续时间约束、梯度传播、走廊生成 |

对三个评估问题的结论：

1. **表示级对照：不存在（组件级不等价）。** MINCO 的表示是优化参数化
   `T_MINCO = (q, T, 边界状态) → c`，系数由耦合带状求解导出，并非“显式系数直接
   求值”；本组件的表示是显式系数 + 分段局部时间求值。`Trajectory<D>`/`Piece<D>`
   虽可用给定系数求值，但那是上游通用求值辅助类型而非 MINCO 表示本身，且与本组件
   语义不对齐：固定三维（本组件二维）、系数降幂存储（本组件升幂）、域外时间不报错
   而按首/末段外推（本组件返回 `kOutOfRange`）、衔接点取左段（本组件取右段）、无
   错误码/批处理/时间缩放契约。用同一系数对比求值只会重复 mav 已覆盖的角色（§3.4
   指定 mav 为“分段多项式表达、求值与连续性处理参考”），无法改变选型（该类型不满足
   组件后端契约，不可能成为默认候选），属无意义比较。
2. **构造级对比：仅单段（N=1）弱等价。** `MINCO_S2NU`/`S3NU` 在 `pieceNum = 1`
   时，带状系统恰好退化为本组件的三次（4×4）/五次（6×6）边界系统，解出同一组
   系数（直接线性求解，非迭代优化）；但该场景与测试内独立高斯消元参考完全重复，
   不提供新的独立证据。N>1 时 MINCO 解的是耦合问题（内部路径点 + 段间 C1/C2 连续
   性 + 首尾状态），本组件没有对应 API（按完整边界条件逐段求解、只校验连续性，
   不求解 `(q, T)` 参数化），不存在“同一问题”的对比；S4（七次）也超出本组件
   三次/五次范围。`selection.json` 的 `construction_comparison = N/A` 文案针对
   mav_trajectory_generation，MINCO 的细分结论记录于本节。
3. **若做表示级对照的代码量：** 纯求值路径约两个头文件（`trajectory.hpp` 15.5 KB +
   其 include 的 `root_finder.hpp` 32 KB，求值本身不用根查找）+ Eigen（已锁定）；
   含 N=1 构造对比再加 `minco.hpp`（约 38 KB）；整个优化器只在比较“参数化 + 时间
   分配 + 约束 + 梯度”时才需要，远超本组件表示级范围。任何接入都属新增上游集成
   （需来源记录/依赖锁条目），而对选型零影响。

**决策**：不新增 MINCO 对照后端或对照库；默认实现 `planner_horner`、正确性/质量
门槛与选型规则全部保持不变。MINCO 属**未来轨迹优化参数化选型**：其价值在空间-时间
形变、时间分配、连续时间约束与梯度传播，比较对象应是未来规划器的优化后端——Dftpav
已在自身组件内移植 MINCO 并验证其优化表示（[PROVENANCE.md](../dftpav/PROVENANCE.md)）；如未来
规划器需要表示级选型，应另立工作包，按锁定 commit + `dependencies.lock.json` 条目 +
MIT 归属 + 明确对照协议（越界语义、衔接点规则、是否含优化器层）接入。集成负责人可
据本节关闭 `polynomial-001`。

## 限制

- 对照候选 mav_trajectory_generation 的求值未做路径优化（按上游逐点 Eigen 实现）；
  微基准为 warm、无冷启动；候选与基线共享夹具 10 个分段，代表性有限。
- 构造/求解正确性仅由测试内独立高斯消元参考覆盖（mav 无等价闭式边界求解 API）。
- `polynomial-001` 的 MINCO 结论基于固定 commit 的源码与接口核对，属决策记录而非
  新增运行证据；本轮未改动既有基准产物与报告。
