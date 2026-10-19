# planner_sampling_search_bench（X-B1 基准工具：RRT* 族）

- 日期：2026-10-12。
- 范围：`Src/sampling_search/benchmarks/`（场景生成器 v1、运行器、注册片段与本文）。
  **不修改** `tools/components.json`、`dependencies.lock.json`、其他 `Src/**` 与 `Docs/**`；
  正式基准的注册由 wiring 代理按 `benchmark_suite.json` 合并。
- 依据（只读）：[benchmark_tiers.md](../../../Docs/expansion/benchmark_tiers.md)
  §2.2/§2.4/§3/§5/§6、[candidate_registration.md](../../../Docs/expansion/candidate_registration.md)
  §4.1/§7、[provenance.md](../provenance.md)、[tools/README.md](../../../tools/README.md) §7.2。
- 现状：本工具已在**烟测**范围运行（证据 run 见 §7）；全量正式矩阵与容量档留待
  独占窗口阶段，由入口 `benchmark --component sampling_search --suite v1` 执行。

## 1. 入口与输出

```sh
# 正式基准（独占窗口；经 wiring 注册后）
python3 tools/task_runner.py benchmark --component sampling_search --suite v1

# 烟测（小预算/少种子；经 exec 入口，构建与运行均持监控门禁）
python3 tools/task_runner.py exec --component sampling_search --workers 1 -- \
  <build>/Src/sampling_search/benchmarks/planner_sampling_search_bench --suite v1 --smoke \
  --output-dir '{run_dir}/outputs'

# 单 trial（由驱动循环内部使用；也可手工复现一个 trial）
planner_sampling_search_bench --single --instance grid-maze-128-s0 \
  --backend vss2sn --budget 0.5 --seed 3 --trial-out /tmp/trial.json

# 实例表与生成器自检（含容量档实例；不运行后端）
planner_sampling_search_bench --dump-instances <dir>
```

输出写入 `--output-dir`（§7.2 契约；组件绝不写其他位置）：

| 文件 | 内容 |
| --- | --- |
| `benchmark_metrics.csv` | §7.2 契约文件：每 trial 一行，含 §9 固定列、RRT* 专有列（`budget_s/first_solution_ms/solved/best_length_m/best_cost/improvements`）与容量档列（`rss_input_bytes/rss_search_bytes/rss_output_bytes/conversion_ms`）；入口将其并入运行级 `metrics.csv` |
| `metrics.csv` | 同一内容的独立副本（便于脱离入口查看；运行级文件仍由入口生成） |
| `summary.json` / `summary.md` | 聚合：成功率（含超时/失败，§5.2 口径）、nearest-rank P50/P95、逐后端/预算/实例分解、harness 自检 |
| `trials.jsonl` | 逐 trial 原始记录（含子进程退出码、中断标记、完整记录） |
| `instances.json` / `instances/<id>.json` / `instances.sha256.txt` | 实例表、规范 JSON 与 SHA256（生成器证据） |
| `children_failures.log` | 非成功/异常退出子进程的 stdout+stderr（失败保留） |
| `plots/*.svg` + `*.csv` | 连续组成功率/首次解 P50、栅格组成功率、容量档峰值 RSS |

## 2. 进程模型（OMPL 进程级种子）

- 每个 trial 由**独立子进程**执行：驱动进程顺序派发 `/proc/self/exe --single ...`，
  子进程只创建一次后端并调用一次 `plan()`，写回 JSON 记录。
- 原因：OMPL 1.6.0 的 `RNG::setSeed` 是进程全局单例，进程内首次使用后再播种
  被忽略（provenance.md §2），因此**每 trial 独立进程**是 OMPL 种子可复现的
  必要条件。所有后端统一采用该模式（不区分 OMPL），同时获得干净的每 trial
  峰值 RSS（`getrusage(RUSAGE_SELF).ru_maxrss`）与崩溃隔离。
- 硬超时：`budget_s + --child-grace-s`（默认 60 s）。超时先 SIGTERM 进程组，
  5 s 后 SIGKILL；该 trial 记 `interrupted`，**不从成功率剔除**而是单列
  （§5.2：取消与资源中断单列并补跑，重试用新 run ID）。
- 崩溃（子进程非零退出且无记录）记 `failed` 并计入成功率分母；`children_failures.log`
  保留其日志。
- 同种子跨进程行为（探针 `20261012T134856320142Z_sampling_search_cec845b1`）：
  同一 (实例, 后端, 种子) 的两次独立进程运行给出逐位一致的预算末最优解
  （OMPL 113.14062321619376 ×2；Informed 113.13708498984485 ×2）；迭代数/
  改进次数因墙钟预算截断存在极小波动（±0.1% 量级），属时间预算语义而非播种
  失败；不同种子给出不同末解。

## 3. 场景生成器 v1

### 3.1 连续组（`sampling_search_continuous_gen v1`，§2.4）

边界 100 m × 100 m；`edge_check_step = 0.1 m`（写入实例与结果元数据）。

| 实例 id | 类别 | 几何（冻结起点，§2.4） | 起终点 |
| --- | --- | --- | --- |
| `cont-open-100-s0` | 开阔 | 无障碍 | (10,10) → (90,90) |
| `cont-sparse-100-s0` | 稀疏障碍 | 15 个矩形（边长 U[5,12] m，总面积 8%–12%，SplitMix64 种子 0 派生），起终点 1 m 净空 | (10,10) → (90,90) |
| `cont-narrow-100-s0` | 窄道 | 两道平行墙（厚 3 m）形成 2.0 m 宽通道、长 60 m（x∈[20,80]） | (21,50) → (79,50) |
| `cont-trap-100-s0` | 陷阱 | 腔体 [40,60]²，壁厚 3 m，顶壁开口 2.0 m | (15,15) → (50,50) |
| `cont-edge-100-s0` | 贴边 | 沿底边通长墙 y∈[1.5,4]，形成 1.5 m 贴边通道；起终点距边界 0.5 m | (10,0.5) → (90,0.5) |
| `cont-unreachable-100-s0` | 不可达 | 封闭矩形 [80,92]²，**墙厚 0.2 m**，无开口 | (10,10) → (86,86) |

- 有效性语义：点 = 界内且不在任何矩形内（闭区间）；**边 = 精确线段–矩形相交
  （Liang–Barsky，闭区间）**，与沿边步长无关。选精确检查是为可靠探测 0.2 m
  薄壁：按 0.1 m 步长采样仍可能漏检更薄的障碍，精确检查对冻结几何无漏检。
  该选择记录在实例规范 JSON（`edge_check_mode=exact_segment_rectangle`）与
  summary `notes`；`edge_check_step=0.1` 仍显式声明并传给需要显式步长的后端
  （Ch0p1k3 夹取为 ≥1、vss2sn 用其作栅格分辨率）。
- 稀疏生成器自检：保守 1 m 粗网格（单元 9 点全有效才算自由）BFS 必须可达，
  保证"粗网格可达 ⇒ 连续空间可达"；不可达场景必须粗网格不可达（薄壁泄漏
  检查）。生成失败（128 次确定性尝试）报错退出 3。

### 3.2 容量档连续组

| 实例 id | 几何 |
| --- | --- |
| `cont-cap-open-200-s0` | 200 m × 200 m，无障碍，(20,20) → (180,180) |
| `cont-cap-sparse-200-s0` | 200 m × 200 m，60 个矩形（边长 U[5,12] m，总面积 8%–12%），(20,20) → (180,180) |

即"更大边界（2× 线性、4× 面积）+ 更多障碍（15→60）"，单次运行、预算 5 s。

### 3.3 栅格组（`sampling_search_grid_gen v1`，§2.2；暂定实现）

- 规模 128²/512²/1024²（容量档 2048²/4096²），分辨率 0.1 m；单元 row-major、
  **row 0 为底行**（+y 向上）、1 = occupied；四周 1 格边界墙。
- 6 类规则（§2.2 表）：`open`（无内部障碍，对角两点）；`maze`（房间间距
  P = max(8, N/16)，墙厚 2、门洞宽 3，房间生成树随机 DFS 保证连通，起点/终点
  为左下/右上房间中心）；`narrow`（3 格宽通道贯穿约 75% 边长，两墙分别贴
  左/右边界）；`deadend`（入口宽 3、袋深 max(10, N/8)，袋外 → 袋底）；
  `unreachable`（封闭腔，墙厚 1、无开口）；`border_corner`（沿底/左边界
  ≥3 处 1 格宽对角障碍对 + 贴右边界的 1 格通道）。
- **跨族一致性**：`grid_scenario_gen v1` 尚未实现（benchmark_tiers §8.4）；
  本生成器为采样搜索侧的暂定实现，规则按 §2.2 表冻结，实例规范 JSON 标注
  `generator_note=provisional`。后续与栅格族生成器交叉校验（逐实例 SHA256
  比对）在集成阶段登记；不一致时以栅格族生成器为准并登记修订。
- 有效性语义：点 = 所在单元自由（`floor`，越界无效）；边 = 按分辨率沿边
  采样（≥1 单元厚障碍不会被跨越漏检）。vss2sn 适配器对统一回调做**保守
  栅格化**（provenance.md §5），其原生接受 ⇒ 统一复检通过。

### 3.4 实例表与 SHA256

- 每个实例的规范 JSON（键排序、网格单元打包 bits 后 base64）与其 SHA256
  写入 `instances.json`、`instances/<id>.json`、`instances.sha256.txt`。
- 子进程重算同一实例的 SHA256 并写入 trial 记录；驱动逐 trial 比对，
  不一致记 `failed` 并计入 `harness.instance_sha256_mismatches`。
- 规范 JSON 单文件最大约 2.7 MiB（4096²），低于 §3.4 的 10 MiB 归档门槛。

## 4. 正式矩阵与烟测

| 组 | 矩阵 | trial 数 |
| --- | --- | ---: |
| 连续组 | {OMPL RRT*, OMPL Informed（单列）, Ch0p1k3, Klamp't} × 6 场景 × 预算 0.1/0.5/1/5 s × 种子 0–29 | 2,880 |
| 栅格组 | vss2sn × 6 类 × 3 规模（seed 0 几何）× 4 预算 × 30 种子 | 2,160 |
| 容量档 | 连续 2 实例 × 4 目标 × 5 s；栅格 12 实例（2048²/4096² × 6 类）× vss2sn × 5 s；seed 0、单次 | 20 |
| **合计** | | **5,060** |

烟测（`--smoke`）：4 目标 × {open, narrow} × {0.1, 0.5 s} × {0,1}（32）+
vss2sn × {open-128, maze-128} × {0.1, 0.5 s} × {0,1}（8）+ 容量路径
（连续 open × 4 目标、sparse × OMPL、栅格 open-2048 × vss2sn，各 0.1 s、seed 0；6）
= **46 trial**，含每 trial 进程模式与容量档代码路径验证。

排名口径：连续组与栅格组**分开评分**；Informed 单列变体；ai-winter 不入横比
（candidate_registration §4.1/§6.2/§7.2）。

## 5. 指标与口径

- CSV 公共列沿用 tools/README.md §7.2/§9；RRT* 专有列与容量档列按
  benchmark_tiers §5；附加列：`group, instance_id, class, scale, backend,
  variant, edge_check_step, iterations, nodes, first_solution_source,
  native_params, instance_sha256`。缺失值留空（**不用 0 代表未测量**）：
  - vss2sn 的 `collision_checks`/`improvements` 原生未埋点，留空；
  - `nodes` 仅 Klamp't 报告；`expanded_nodes`/`gear_changes`/yaw 误差不适用；
  - `conversion_ms` 未单独埋点（vss2sn 适配器内部转换计入 `wall_ms`），留空。
- 成功率 = success / (success + timeout + no_path + invalid + failed)；
  `cancelled` 与 `interrupted` 单列（§5.2）。超时与失败**不从成功率剔除**。
- 首次解：记录各后端原生观测点（`first_solution_source` 列）：OMPL 中间解
  回调、Ch0p1k3 目标域首个节点、Klamp't 外层 `PlanMore()+IsSolved()` 循环、
  vss2sn 补丁回调。**口径差异未消除**，跨实现比较时必须披露（§5 遗留）。
- 墙钟为驱动在 `plan()` 外测量的值；`cpu_ms` 为子进程 `getrusage` 差值；
  `peak_rss_bytes` 为单 trial 子进程峰值 RSS（采样口径为内核 `ru_maxrss`，
  非 2 s 采样估计）。
- 容量档 RSS 构成：`rss_input_bytes` = `plan()` 前 `VmRSS`（含夹具/实例）；
  `rss_search_bytes` = 峰值 − 输入（含搜索结构 + 适配器转换 + 输出，未再细分）；
  `rss_output_bytes` = 返回路径点字节数。转换成本未单列（见上）。
- 每个 trial `cache_mode=cold`：每 trial 新建后端，无预热（RRT* 无预热，
  §7 计数口径）；`repeat_id` 即采样种子编号 0–29。

## 6. 资源与失败保留

- 构建与运行均经 `tools/task_runner.py exec`（监控门禁、worker 名额）；编译持
  全局编译锁 `flock .git/planner-runtime/build.lock` 且 `--parallel 4`。
- 驱动顺序派发子进程（同一时刻 1 个运行进程）；失败与中断 trial 全部保留在
  CSV/JSONL/`children_failures.log`，重试用新 run ID（§5.4）。

## 7. 证据

| 步骤 | run ID | 结果 |
| --- | --- | --- |
| 构建 1（bench 目标加入后 configure+build，`-j4`、持编译锁） | `20261012T134318949035Z_sampling_search_f1fa2f81` | success（bench 目标链接通过；既有 demo/tests 未受影响） |
| 生成器自检 + 全实例表（38 实例，含 2048²/4096²） | `20261012T134358848849Z_sampling_search_a30fbe74` | success：self-check passed；SHA256 与 Python `hashlib` 独立复算 38/38 一致 |
| 回归测试（既有 34 用例，确认 CMake 增补未破坏组件） | `20261012T134916972893Z_sampling_search_27720a3a` | success：34/34（仅 bench 源文件此后有改动，库/测试源未变） |
| 构建 2（最终二进制：硬超时 kill 回退 + `--help` 退出码修复） | `20261012T135307472551Z_sampling_search_eda2798a` | success |
| 烟测（46 trial，每 trial 独立子进程；含容量档路径） | **`20261012T135320409529Z_sampling_search_f8de5fdc`**（最终；retry_of `20261012T134812496191Z_sampling_search_c59f2a02` → `20261012T134602148847Z_sampling_search_2fd57cb1`；首轮 `20261012T134411652986Z_sampling_search_73b7fb6d` 的 CSV 列数缺陷保留） | success：46/46 子进程、46/46 记录、0 SHA 失配、0 跳过；`harness.ok=true` |
| 进程级种子探针（同种子两次独立进程 vs 异种子） | `20261012T134856320142Z_sampling_search_cec845b1` | 同种子 OMPL 末解逐位一致（113.14062321619376 ×2）；Informed 同种子末解一致；迭代数因墙钟截断有 ±0.1% 波动；异种子末解不同 |

烟测观察（记录，不作结论）：vss2sn 在 128² open/maze、0.1/0.5 s 全部 timeout（性能临界，
provenance.md §8.9 已登记）；Ch0p1k3 在 2.0 m 窄道场景 timeout（其 1 m 保守栅格化
将 2 m 通道两侧单元判为受阻，属原生障碍映射语义，正式矩阵如实记录）；容量档
2048² open 单次墙钟约 0.8 s（含保守栅格化），峰值 RSS 约 62 MiB。

（证据 run 目录：`artifacts/runs/<run_id>/`。本表只覆盖 X-B1 交付与烟测；
正式矩阵与容量档在独占窗口阶段执行后另行报告。）

## 8. 遗留风险

1. **首次解口径**：四后端原生观测点不同（回调 vs 外层循环），跨实现比较须
   披露；若要求严格统一，需在适配层增加统一观测点（改动库，不在 X-B1 范围）。
2. **Klamp't 迭代数**：`iterations` = 外层 `PlanMore()` 调用次数，不是上游
   内部迭代；与 OMPL `numIterations` 不同义，报告须标注。
3. **vss2sn 性能临界**：101² 开阔场景 1 s 预算成功与否随迭代波动
   （provenance.md §8.9）；栅格组正式矩阵须如实记录成功率/超时，不得调参隐藏。
   容量档 4096² 的保守栅格化（约 4097²×17 次回调）可能显著拉长单次墙钟，
   已在硬超时宽限内预留（`--child-grace-s`）。
4. **跨族生成器一致性**：`grid_scenario_gen v1` 未实现，本生成器为暂定实现；
   交叉校验（逐实例 SHA256）在集成阶段完成前，栅格组结果标注"暂定几何"。
5. **0.2 m 薄壁与保守栅格化**：Ch0p1k3/vss2sn 的原生障碍映射为保守近似，
   与精确连续夹具存在理论差异（provenance.md §8.4）；统一复检拒绝的原生解
   按无解处理并保留消息。
6. **连续夹具边检查为精确几何**（非采样）：与统一契约的"沿边采样"表述
   存在形式差异；选择理由（0.2 m 薄壁）与模式已在实例表/summary 记录，
   跨组件比较时须披露。
7. **OMPL 近似解**：适配器只接受精确解，预算末"近似解"按 timeout 处理
   （provenance.md §2），成功率口径不受影响。
