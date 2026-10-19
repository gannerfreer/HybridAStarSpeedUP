# planner_graph_search 基准工具（X-B1：graph_gen v1 + 正式矩阵运行器）

- 状态：X-B1 交付（harness + 烟测）；**全量正式矩阵留待独占窗口阶段**，本目录不
  预设赢家、不替代验收结论。
- 依据：[benchmark_tiers.md](../../../Docs/expansion/benchmark_tiers.md) §2.1/§2.3/§5/§6；
  [component_development_plan.md](../../../Docs/component_development_plan.md) §7.2/§7.3；
  [tools/README.md](../../../tools/README.md) §7.2 基准输出契约。
- 二进制：`planner_graph_search_bench`（链接全部已构建适配器；igraph 属
  GPL-2.0-or-later 隔离组，只进入本专用离线基准可执行文件）。
- 注册片段：[`benchmark_suite.json`](benchmark_suite.json)（供单一写者并入
  `tools/components.json` 的 `graph_search` 条目；本目录不改该文件）。

## 1. 命令

```sh
# 正式矩阵（独占窗口；预热 5 + 测量 30，点到点预算 5 s、距离场 30 s）
planner_graph_search_bench --suite v1 --output-dir <dir>

# 物化 6 个确定性实例（含 >10 MiB 的 SHA256 寻址归档）并写实例表
planner_graph_search_bench --generate-only --output-dir <dir>

# 结构自检 + 已物化文件/归档与确定性再生成的一致性校验
planner_graph_search_bench --check --output-dir <dir>

# 烟测（1k 图；2 个点到点后端 + 1 个距离场后端；预热 1 + 测量 2）
planner_graph_search_bench --smoke --output-dir <dir>

# 打印实例与后端矩阵
planner_graph_search_bench --list
```

常用过滤：`--instances/--scales/--classes/--backends/--cache-modes`，
`--warmup/--repeats/--p2p-budget-s/--field-budget-s/--archive-root/--rss-sample-us`。
退出码：0 完成；1 生成器/一致性/正确性门禁失败；2 用法错误。

## 2. graph_gen v1 冻结规则

`graph_gen v1`（`graph_gen.hpp/.cpp`，版本常量 `kGraphGenVersion = 1`）按
benchmark_tiers §2.1 生成 6 个实例，规则冻结如下：

| 实例 ID | 类别 | 规模 | 顶点 | 种子 |
| --- | --- | ---: | ---: | ---: |
| `graph-road-1k` | road | 1k | 1,000 | 101 |
| `graph-road-10k` | road | 10k | 10,000 | 102 |
| `graph-road-100k` | road | 100k | 100,000 | 103 |
| `graph-rand-1k` | rand | 1k | 1,000 | 201 |
| `graph-rand-10k` | rand | 10k | 10,000 | 202 |
| `graph-rand-100k` | rand | 100k | 100,000 | 203 |

- **road（道路状稀疏）**：主链取 `c = round(0.6·(n-1))` 个顶点，沿 x 方向
  pitch 10 m 布置、y 方向 `40·sin(0.05·i)` 弯曲并加 ≤0.75 m 的种子派生抖动；
  其余 `n-1-c` 个顶点分成 `b = max(1, c/8)` 条支路（长度由种子派生，总长恰为
  `n-1-c`），支路沿附着点的法向逐点外推、带 ≤0.75 m 的切向抖动；链与支路均
  双向边，权重 = 端点欧氏距离。有向边数 = 2(n-2)，平均出度 ≈ 2。
- **rand（固定随机稀疏）**：G(n, m)，m = 2n 有向边。活动顶点 0..n-2 的坐标
  为 [0,1000]² 均匀采样（种子派生，精确重复重抽）；先生成随机生成树
  （i = 1..n-2，parent ∈ [0, i)，方向 parent → i，保证弱连通且 0 可达全部
  活动顶点），再补随机有向边（u ≠ v、去重）到 m = 2n；权重 =
  欧氏距离 × (1 + u)，u ∈ [0,1) 种子派生。
- **断连保证**：两类实例的顶点 `n-1` 为孤立顶点（无任何关联边）；断连对
  `(0, n-1)` 由生成规则保证不可达，不依赖搜索行为。
- **起终点对（每实例 10 个）**：
  - `r01`..`r08`：源点 0 的 BFS 跳数分布按最近秩取分位点
    q = {0.05, 0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 0.99} 作为目标顶点
    （去重），覆盖短/中/长；
  - `disconnected`：(0, n-1)，期望 `no_path`；
  - `same_point`：(0, 0)，期望 success、代价 0、路径 `{0}`。
- **数值性质**：非负权、无零权边；所有权重 ≥ 端点欧氏距离（欧氏启发式可采纳）。
- **PRNG**：单流 SplitMix64，消费顺序固定；实例完全确定（生成规则 + 种子为
  权威输入）。

### 2.1 物化文件与 SHA256 归档（§3.4）

`--generate-only` 在 `<output-dir>/graphs/` 写入：

- `<instance_id>.graph.txt`：v1 文本格式
  （`V <i> <x> <y>`、`E <from> <to> <weight>`、`P <pair> <start> <goal> <hops>
  <expected>`，`%.17g` 精确往返）；
- `<instance_id>.sha256`、`<instance_id>.meta.json`；
- 单文件 >10 MiB 时另按 SHA256 寻址复制到
  `<archive-root>/<sha256>/<instance_id>.graph.txt`（附 `.sha256`、
  `.meta.json`），`archive-root` 默认 `artifacts/data/sha256`（相对 CWD），
  归档不入 Git；
- `<output-dir>/graph_instances.json` 记录每实例的 `relative_path`/`sha256`/
  `size_bytes`/`generated_by_run_id`/归档引用与对表；
- `<output-dir>/graph_instance_checks.json` 记录结构自检结果；
- 所有写盘文件在写入后立即回读校验 SHA256/大小（§3.4“生成器每次校验输出
  SHA256”），不一致即失败。

正式矩阵运行也会写实例表（未物化时记录序列化内容的 SHA256），并对 >10 MiB
实例执行归档；若 `<output-dir>/graphs/` 已存在同名文件，则强制校验其 SHA256
与确定性再生成一致，不一致即失败（不静默使用不同输入）。

### 2.2 生成器自检（`--check` 与运行前门禁）

- `validate_graph` 契约校验（端点/有限非负权）；
- 无零权边、无自环、权重 ≥ 端点欧氏距离（1e-12 松弛）；
- 孤立顶点无关联边；road 边数 = 2(n-2)，rand 边数 = 2n；平均出度 ∈ [1.9, 2.1]；
- 8 个可达对目标互异，跳数与 BFS 一致；
- 独立参考 Dijkstra（本目录内实现，非适配器）：可达对有限、断连对 +inf、
  同点对距离 0；
- 已物化文件与归档文件的 SHA256/大小与再生成一致。

## 3. 基准矩阵（suite v1）

后端注册表（按编译定义自动裁剪；未链接的后端标记为未编译并跳过）：

| 组 | 后端 | 说明 |
| --- | --- | --- |
| 点到点主组 | `boost_astar`、`boost_dijkstra`、`lmrp_astar`、`justinhj_astar_pool`、`justinhj_astar_nopool`、`lemon_dijkstra`、`igraph_dijkstra` | 6 实例 × 8 可达对；A* 用欧氏启发式，Dijkstra 零启发式 |
| 零启发式对照 | `<astar>__zero`（boost_astar / lmrp_astar / justinhj 池开/关） | 同实例同对，**单列，不混入主排名**（§2.1/§4.2） |
| 距离场 | `boost_distance_field`、`lemon_distance_field`、`igraph_distance_field` | 6 实例 × 2 固定源点（s01 = 0；s02 = r05 目标）；预算 30 s |

- 每用例预热 5 + 测量 30（`--warmup/--repeats` 可调；预热不写入指标）；
- 冷/热分列：冷启动每 trial 新建后端（构造时间单列），热启动复用一个后端
  实例（适配器无状态，热启动复用内容 = 后端对象）；
- 负例/边界对（断连、同点）单次运行（`measured=false`），只记录状态与终止
  耗时，不进排名；
- 点到点预算 5 s、距离场 30 s；超时 `status=timeout`、`timeout=true`，
  不从成功率剔除、不自动重跑。

### 3.1 计时边界

每个 trial 分列（毫秒）：`construct_ms`（冷启动后端构造）、`wall_ms`
（harness 计时的适配器调用）、`e2e_ms`（构造 + 调用）、`preprocess_ms`
（图/邻接转换，来自适配器 Stats）、`search_ms`（算法核心）、`postprocess_ms`
（wall − preprocess − search）、`adapter_wall_ms`（适配器自报总耗时）。
输入生成、参考 Dijkstra、缓冲区分配与磁盘输出在计时外。CPU 时间用
`std::clock()` 环绕调用；峰值 RSS 用 `/proc/self/status` 1 ms 采样（采样
估计），另单列 `rss_hwm_bytes`（VmHWM 进程高水位参考）。

线程环境：igraph 静态链接的 OpenBLAS 在进程启动时会拉起线程池，若不限制会
使 `std::clock()` 显著虚高（实测约 16×）。正式运行必须使用 task_runner 默认
注入的 `OMP_NUM_THREADS=1`/`OPENBLAS_NUM_THREADS=1`/`MKL_NUM_THREADS=1`
（`tools/task_runner.py` 的 `DEFAULT_THREAD_ENV`），即基准进程单线程。

### 3.2 正确性门禁

- 可达对 success：返回路径逐边存在且端点正确；路径按输入图重算的 double
  代价与独立参考 Dijkstra 一致（§6.2 容差 `1e-8 + 1e-8·|ref|`）；原生代价
  同样核对；
- 断连对：状态必须为 `no_path`；同点对：success、代价 0、路径 `{0}`；
- 距离场：逐顶点与参考一致（不可达标记一致、有限距离在 §6.2 容差内）；
- 任一 trial 不满足 → `valid=false` 写入指标并保留证据，运行退出码 1；
  该后端不进入信息性排名（`ranking_eligible=false`）。

### 3.3 justinhj float32 口径（X-B1 登记门槛）

justinhj 原生 g/h/f 与代价为 float32（provenance.md §5）。正式档长路径不
沿用 §6.2 严格容差；X-B1 冻结登记门槛：

```text
tol(ref, edges) = 1e-5 + max(1e-5, 1.2e-7 × edges) × |ref|
```

（1.2e-7 ≈ 2 × 2⁻²⁴，即逐边 float32 累加允许量；`edges` 为返回路径边数。）
原生代价与 double 重算代价都用该门槛核对；质量对比使用重算 double 代价
（`recomputed_cost`/`cost_delta_rel`）。若该门槛在正式窗口仍不足以容纳
justinhj 的偏差，按 §5.4 登记问题并单列，不静默放宽其他后端的门禁。

## 4. 输出与指标列

`<output-dir>/benchmark_metrics.csv`（UTF-8 CSV，每个测量 trial 一行；负例
单次也写入；失败与超时保留；未测量留空）：

- 固定列（tools/README §7.2）：`run_id,case_id,repeat_id,cache_mode,seed,
  status,failure_reason,preprocess_ms,wall_ms,cpu_ms,peak_rss_bytes,
  expanded_nodes,collision_checks,path_length_m,gear_changes,
  endpoint_position_error_m,endpoint_yaw_error_rad,valid`；
- 图/距离场列（§5）：`instance_id,scale,class,neighborhood,pair_id,budget_s,
  timeout,source_id,unreachable_cells,max_distance_m,reference_cost`；
  `neighborhood` 对图不适用留空；`path_length_m` 为可达对的 double 重算
  路径代价（权重即米，断连/距离场行留空）；`unreachable_cells`/
  `max_distance_m` 为距离场行；
- 组件列：`group,backend_id,variant,kind,measured,expected,construct_ms,
  e2e_ms,search_ms,postprocess_ms,adapter_wall_ms,path_edges,hops,
  native_cost,recomputed_cost,cost_delta_rel,rss_hwm_bytes`。

其余产物：`bench_summary.json`（协议、实例表、按组聚合 P50/P95
nearest-rank、成功率、超时/无效计数、负例单列、信息性排名与口径）、
`plots/p50_p95_<group>.csv` 与 `plots/p50_wall_<group>.svg`、
`plots/p95_wall_<group>.svg`、`graph_instances.json`、
`graph_instance_checks.json`、`graphs/*`。

聚合口径：`p50/p95_wall_ms` 为有效成功样本；成功率 = 成功 /
（成功 + 正常失败 + 超时）；`mean_case_p95_cold_ms` 为冷启动每
(实例, 对/源点) P95 的等权均值，仅用于信息性排序（正式选型属独占窗口
阶段，零启发式对照与距离场不参与点到点排名）。

## 5. 资源与证据规则

- 构建/运行经 `python3 tools/task_runner.py exec --component graph_search`；
  编译在命令内持全局编译锁 `flock .git/planner-runtime/build.lock` 且
  `-j4`；监控 agent 2 s 采样（门禁见 AGENTS.md）。
- 正式矩阵必须在独占窗口运行（`benchmark --suite v1`）；本目录交付的烟测
  证据见 `Docs/reports/` 的 X-B1 报告与对应 `artifacts/runs/<run_id>/`。
- 失败与超时保留；重试使用新 run ID 并记录 `retry_of`。
