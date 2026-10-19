# RATES.md — libm 分步提速实验（ros_free）

仓库：`/home/gray/Planner/HybridAStarSpeedUP`（分支 `libm`）；基线 = NewStructPlanner 原始快照（排除 artifacts/.git/build*/Docs）。
实验日期：2026-10-19（安静窗口；Phase 3 计时与 Phase 4 valgrind 串行）。

## 0. 结论摘要

| 步骤 | 提交 | 全 166 例几何平均 | 全 166 例求和口径（ΣP50） | 旧 16 例锚定子集（GM） |
| --- | --- | ---: | ---: | ---: |
| ① hypot→sqrt 快速路径 | `8845174` | **1.261×（−20.7%）** | **1.361×（−26.5%）** | 1.262×（−20.8%） |
| ② sincos 重复计算削减 | `ddf8252` | **1.010×（−0.9%）** | **1.013×（−1.3%）** | 1.010×（−1.0%） |
| 累计（原始→sincos） | — | **1.273×（−21.4%）** | **1.378×（−27.4%）** | 1.275×（−21.6%） |

- 三步口径：① base→hypot；② hypot→sincos；③ base→sincos。
- 两步均通过全量位级等价核验（状态翻转 0，720 个点位值 bitwise 全等）与 ctest 全量（各 239/239）。
- 步骤②预期上限就是 ~1.3%：**在原始树上 sincos 自身只占 ~1.3% 指令**（被 hypot 大量掩盖；补丁注释中的"7%"是 T3 重构后的残差口径）。削减后 sincos 调用 Ir 降 68–70%（case_39 3.86→1.23 G），方向与预期一致。

## 1. 协议与环境（Phase 3）

- 轮转交错（套件级）：R1 base→hypot→sincos；R2 hypot→sincos→base；R3 sincos→base→hypot。
  每轮每个状态依次跑 v3、v4 两个进程（进程内全量 115/51 例顺序跑）；每（状态×套件）3 样本 → 每例 P50（3 样本中位）。
- 命令：`taskset -c 6 ./bin/bench-<state> --suite v3|v4 --fixtures-dir tests/fixtures/v8|v9 --vehicle parking_bench.json|parkbench_vehicle.json --output-dir timing/r<R>/<state>/<suite> --cold-only --warmup-runs 0 --measured-runs 1 --jobs 1 --time-budget-s 120 --backends ros_free`
- 机器安静：起始 loadavg 0.81/1.09/0.97；全程每进程前后 loadavg ∈ [1.16, 1.66]；无编译/valgrind 并发。
  证据：`timing/env_pre.txt`、`timing/wall_log.tsv`。
- 每进程墙钟（秒，3 轮）：base v3 153.97/154.01/154.11，v4 115.03/115.16/115.14；
  hypot v3 111.57/111.57/111.65，v4 86.34/86.22/86.31；
  sincos v3 110.60/110.59/110.65，v4 84.82/84.79/84.87。（轮内/轮间重复性 ±0.05%）
- 成功数一致性：166 例 × 3 状态 = 498 次 trial **全部 success**；非 success 0；跨状态状态翻转 0。

## 2. 核心表（每步提升率）

几何平均 = 每例速度倍率（base/hypot、hypot/sincos、base/sincos）的几何平均；求和口径 = Σ(base P50)/Σ(状态 P50)。
下降 % = 1 − 1/倍率（GM 行）或 1 − Σ该状态/Σbase（求和行）。

| 分组 | n | ① base→hypot GM（↓%） | ① 求和倍率 | ② hypot→sincos GM（↓%） | ② 求和倍率 | ③ 累计 GM（↓%） | ③ 求和倍率 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 全 166 例 | 166 | 1.261×（−20.7%） | 1.361× | 1.010×（−0.9%） | 1.013× | 1.273×（−21.4%） | 1.378× |
| v3 115 例 | 115 | 1.248×（−19.9%） | 1.381× | 1.007×（−0.7%） | 1.009× | 1.257×（−20.4%） | 1.393× |
| v4 51 例 | 51 | 1.289×（−22.4%） | 1.334× | 1.015×（−1.5%） | 1.017× | 1.309×（−23.6%） | 1.358× |
| 慢档 base≥5 s | 18 | 1.364×（−26.7%） | 1.363× | 1.012×（−1.2%） | 1.013× | 1.381×（−27.6%） | 1.380× |
| 中档 0.5–5 s | 58 | 1.359×（−26.4%） | 1.362× | 1.012×（−1.2%） | 1.012× | 1.376×（−27.3%） | 1.379× |
| 快档 <0.5 s | 90 | 1.182×（−15.4%） | 1.314× | 1.007×（−0.7%） | 1.011× | 1.190×（−16.0%） | 1.329× |
| 旧 16 例锚定子集 | 16 | 1.262×（−20.8%） | 1.356× | 1.010×（−1.0%） | 1.015× | 1.275×（−21.6%） | 1.376× |

P50 求和（秒）：base 268.6 → hypot 197.4 → sincos 194.9。
逐例明细：`timing/per_case_rates.csv`；原始数据：`timing/r{1,2,3}/{state}/{suite}/benchmark_metrics.csv`。

个别极值：
- ① 最大 case_97 1.469×、case_31 1.446×；最小有效值 ~1.03×（快例）。
- ② 最大 pb-024 1.032×、pb-019 1.029×、pb-020 1.028×、pb-049 1.028×；40/166 例 >1.5%；
  个别极快例（pb-001，2.4 ms）样本噪声 0.984×；其余 <1% 的负值均属噪声带（±1%）。
- 锚定子集（v3 case_39/25/3/48/30/7/80/88；v4 pb-042/028/049/032/017/015/007/001）单列见上表。

## 3. 指令数对照（Phase 4 callgrind，完整搜索）

| 例 | 节点/检查 | base Ir | +hypot Ir | +hypot+sincos Ir | Ir 倍率（①/②） |
| --- | --- | ---: | ---: | ---: | ---: |
| v3 case_30 | 298 / 2577 | 4.267 G | 3.342 G | 3.287 G | 1.28× / 1.02× |
| v3 case_39 | 20453 / 152961 | 304.4 G | 229.5 G | 226.5 G | 1.33× / 1.01× |
| v4 pb-017 | 910 / 7556 | 14.79 G | 11.44 G | 11.28 G | 1.29× / 1.01× |

节点数与碰撞检查数三状态逐位一致（298/20453/910、2577/152961/7556）→ 同一搜索路径上的常数因子下降；
valgrind 下（taskset -c 12、`--time-budget-s 7200`）墙钟：case_39 1156→887→875 s，pb-017 60→48→48 s，case_30 18→14→13 s。

7 桶等权宏平均占比（图口径，三状态同一映射；原始行合计=100%）：

| 桶 | base | +hypot | +hypot+sincos |
| --- | ---: | ---: | ---: |
| 碰撞几何/碰撞检查 | 59.07% | 70.84% | 70.69% |
| libm hypot/三角函数 | 35.58% | 1.67% | 0.72% |
| 哈希/容器 | 0.90% | 0.90% | 0.90% |
| 势场 Dijkstra | 0.84% | 0.84% | 0.84% |
| 内存分配 | 1.91% | 1.91% | 1.91% |
| RS 距离求解/连接 | 0.14% | 0.14% | 0.14% |
| 其它/未分类 | 1.56% | 0.72% | 0.71% |
| 合计 | 100.0% | 77.0% | 75.9% |

读法：hypot→sqrt 后，glibc `__hypot_finite/hypot` 被 `hypot_fast`（碰撞文件内、含内联 `sqrtsd`）取代，
故 libm 段塌缩到 ~1.7%，而"碰撞几何"段被动升至 ~71%（其中含 hypot_fast 自身）；"其它"段腰斩（PLT/lazy 绑定匿名行消失）。
第二步 sincos 把 libm 段再压到 0.72%。

## 4. 门槛记录

| 门槛 | 步骤① hypot | 步骤② sincos |
| --- | --- | --- |
| ctest 全量（`-j2`，含 OMPL 参考一致性套件） | 239/239 通过 | 239/239 通过 |
| 位级等价（bench-base vs bench-<state>，v3 case_39/case_115 + v4 pb-042，`--cold-only` 导出） | 720 个点位值 bitwise 全等，max\|Δ\|=0.0；status/点数/长度/端点全等；状态翻转 0 | 同上 |
| 成功数（166 例×3 状态） | 全 success | 全 success，翻转 0 |

比对脚本：`/tmp/opencode/libm-lab/tools/compare_exports.py`；导出与日志：`/tmp/opencode/libm-lab/smoke/`。

## 5. 偏差点与风险

1. **sincos 补丁移植偏差**（提交信息已列）：
   - hunk 10（`world_hull` 内联公式）**跳过**：依赖 T3 重构后的 `local_envelope_for/LocalPrimitiveEnvelope`，原始树不存在；
   - hunk 9 尾部上下文修正（原补丁上下文含空行，本树是 `result_from_decision` 行）；其余 hunk `git apply` 干净应用；
   - 原补丁的 `pose_on_arc` 三参包装在本树无调用点（会触发 `-Wunused-function`），删除包装；Release 二进制逐字节未变（死代码，验证无行为影响）；
   - **额外融合**：`differential_strata.cpp` 的 `place_at_edge/place_at_corner` 改为复用一次 `world_corners`（2/5 次 `transform_point` → 1 次融合计算，公式逐位一致）；
   - 未融合（记录）：`reference.cpp`（独立参考实现，刻意保留），`test_helpers.hpp`（仅测试夹具），`differential.cpp` 的固定起点 `pose_on_arc`（非 ros_free 热路径）。
2. **步骤②收益小的原因**：原始树 sincos 占比只有 ~1.3% Ir（其大头 hypot 掩盖）；削减后 sincos 调用 Ir −68–70%，墙钟 −1%（与 Ir 口径一致）。若叠加 T3 重构（残差 sincos 6.7%），该步价值会放大。属"预期上限内"，非实现失误。
3. **hypot 步墙钟增益（~1.36×）> Ir 增益（~1.29×）**：hypot 是长延迟串行链（缩放/分支多），替换为 SQRT+少量检查对墙钟更有利；两口径方向一致。
4. **出图 7 桶映射为重建**：tarball 未含 T1 `analyze_callgrind.py`，按 `evidence/PROFILING.md` 的桶定义用函数名匹配重建（三状态同一套，图内可比）。与本仓库 T7 旧图 base 宏平均差异 ≤1.6 pp（如 哈希 2.49%→0.90%、内存 0.89%→1.91%——内联行归属规则差异）；绝对水平（碰撞 ~59%、libm ~35.6%）与旧图（58.6%、35.2%）一致。
5. **噪声**：快档（毫秒级）单例 3 样本 P50 ±1–2%；≤1% 的"负提升"均在该噪声带内。常驻轻载进程（chrome/gnome）全程未变，taskset 固定单核 + 轮转抵消。
6. 计时未包含 JIT/首次磁盘 IO 影响；`--time-budget-s 120` 下无预算截断（三状态成功数一致、非 success 0）。

## 6. 产物索引

- 图：`figures/fig_libm_hypot.html`（原始→+hypot）、`figures/fig_libm_sincos.html`（原始→+hypot→+sincos）；数据 `figures/data/`（stages_totals、stages_buckets、stages_buckets_normalized、functions_all）；自检 `figures/verify_fig_libm.py`。
- 计时：`timing/`（wall_log.tsv、env_pre.txt、r1-3/、per_case_rates.csv、summary.txt、summary.json）。
- 剖析：`profiling/`（callgrind.<state>.<case>.out ×9、wall_log.tsv、out/ 逐例 CSV/日志）。
- 二进制：`bin/bench-{base,hypot,sincos}[-prof]`（Release 与 RelWithDebInfo）。
- 提交：`a49bd7a` baseline → `320f63c` tooling → `8845174` hypot → `ddf8252` sincos。
