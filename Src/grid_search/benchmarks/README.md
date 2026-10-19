# planner_grid_search 基准工具（X-B1）

本目录是扩展计划 **X-B1 栅格族基准工具**：确定性场景生成器
（`grid_scenario_gen v1`）+ 正式/容量基准运行器（`planner_grid_search_bench`）
+ 独立 Python 参考（`reference/grid_bench_reference.py`）+ 套件编排
（`run_suite.sh`）。契约依据：[benchmark_tiers.md](../../../Docs/expansion/benchmark_tiers.md)
§2.2/§2.3/§3/§5/§6、[组件开发计划](../../../Docs/component_development_plan.md)
§7.2/§7.3/§9、[tools/README.md](../../../tools/README.md) §7.2。

**本轮范围**：harness + 烟测（128² 主烟测、90 实例生成烟测、1 实例容量最小
烟测）。正式全矩阵（90 实例 × 全部后端 × 主组 + 对照组）与容量全矩阵
（2048²/4096² × 6 类 × 3 查询组）属**独占窗口阶段**，未在本轮执行
（见“证据与遗留”）。

## 1. 文件与构建

| 文件 | 职责 |
| --- | --- |
| `scenario_gen.{hpp,cpp}` | `grid_scenario_gen v1` 生成规则 + 独立 BFS 自检 |
| `grid_scenario_gen.cpp` | 生成器入口：实例表/打包文件/SHA256 清单/归档 |
| `grid_search_bench.cpp` | 运行器：逐后端子进程、测量、参考校验、metrics/summary |
| `bench_util.{hpp,cpp}` | 计时、RSS 采样、CSV/文件工具 |
| `sha256.{hpp,cpp}` | 自包含 SHA256（内容寻址与归档） |
| `reference/grid_bench_reference.py` | 独立 Python Dijkstra 参考（标准库） |
| `run_suite.sh` | 编排：生成 → 参考 → 基准（全部证据写 `<output-dir>`） |
| `benchmark_suite.json` | `tools/components.json` 注册片段（本文件不改注册表） |

构建：`Src/grid_search/CMakeLists.txt` 末尾在 `PLANNER_BUILD_BENCHMARKS=ON`
（默认 ON，与根选项一致）时 `add_subdirectory(benchmarks)`。产物：
独立构建 `<build>/benchmarks/planner_grid_{scenario_gen,search_bench}`；
根构建 `<build>/Src/grid_search/benchmarks/...`（`run_suite.sh` 两处都会查找）。
不注册 CTest；开发期经 `tools/task_runner.py exec` 运行。

## 2. grid_scenario_gen v1 生成规则（冻结解释）

统一约定：S×S 栅格，分辨率 **0.1 m/格**；0=空闲、1=占据（打包 uint8，行主序）；
每实例固定 1 对起终点 + 1 个距离场源点；期望 success/no_path 由生成器**独立
BFS**（四/八邻域，八邻域禁穿角）自检。实例 ID：`grid-<class>-<scale>-s<seed>`。

| 类 | 规则（seed 参与方式） | 起终点 | 期望 |
| --- | --- | --- | --- |
| `open` 开阔 | 外围墙 1 格，无内部障碍；边距 m=2+(seed%3) | (m,m) → (S-1-m,S-1-m) | success |
| `maze` 迷宫 | 递归回溯；房间间距 P=max(8,S/16)（n=S/P=16 房间/边）；墙厚 W=1+(seed%2)；房间内部 [riP+W, min(riP+P-1,S-1-W)]；门宽 3；房间访问顺序由 LCG(seed·1000003+ri·131+rj·17) 置换 | 左下房间中心 → 右上房间中心 | success |
| `corridor` 窄道 | 通道中心行 cy=S/2+(seed-2)·max(1,S/64)；两道平行墙行 cy±2、列 [S/8, S-S/8-1]（贯穿 75% 边长）；通道宽 3；两端开阔 | (cy,2) → (cy,S-3) | success |
| `deadend` 死胡同 | 袋深 D=max(10,S/16)+seed（≥10）；袋内部行 [cy-1,cy+1]、列 [S/8, S/8+D-1]；三面墙；入口宽 3 | 袋外 (cy,2) → 袋底 (cy,x1) | success |
| `unreachable` 不可达 | 封闭腔中心 (cr,S/2)，cr=S/2+(seed-2)·max(1,S/32)，半宽 max(4,S/64)；只填腔周界（墙厚 1、无开口） | (2,2) → 腔中心 | no_path |
| `border_corner` 边界斜角 | K=max(3,S/64)+seed 处对角障碍对 base=1+3k：占据 (base,base+1)、(base+1,base)；贴边通道：墙列 S-5、行 [S/8,S-2]，右侧通道宽 3，底部留入口 | (2,2) → (S-3,S-3) | success |

- 迷宫：P=max(8,S/16) 解释为房间间距（pitch）；128²–4096² 均为 16×16 房间；
  墙厚 1–2 格、通道（房间内部与门）宽 ≥3 格，符合 §2.2 冻结要点。
- 矩阵（§2.2）：6 类 × {128²,512²,1024²} × 种子 0–4 = **90 实例**；
  容量档（§3）：6 类 × {2048²,4096²} × 种子 0 = **12 实例**。
- 内容寻址：每个实例打包字节 SHA256 写入 `instances.json` 与
  `instances.sha256`；单文件 >10 MiB（4096² = 16 MiB）按 §3.4 归档到
  `artifacts/data/sha256/<sha256>/<name>`（附 `.sha256`/`.meta.json`），
  运行目录只保留 sidecar 与引用路径。

## 3. 基准协议（planner_grid_search_bench）

- **后端**：全部已编译启用后端；单列配置 `jps3d_jps`（原生允许对角穿角，
  参考用 `eight_corner_cut_allowed`，路径校验不强制禁穿角）、
  `justinhj_astar_nopool`（内存池关）。四邻域能力按探测结果决定
  （`jps3d_jps` 返回 `invalid_input`，跳过并记录）。
- **邻域**：八邻域主组；四邻域对照组同实例、同协议、单列不混排（§2.2）。
- **预算与重复**：点到点 5 s、完整距离场 30 s；预热 5 + 测量 30；
  `--cache-mode warm`（默认，复用后端对象；适配器无状态、每次 search 内部
  转换计入 `wall_ms`）或 `cold`（每 trial 新建后端，`preprocess_ms` 单列）。
- **参考与容差**：独立 Python 参考给出点到点代价/可达性与**逐单元**距离场
  （float64、米、`+inf` 不可达；容差 `1e-8 + 1e-8·|ref|`，计划 §6.2）。
  任一测量 trial 不匹配 → `valid=0`、`failure_reason` 记录、阻止该后端参与
  排名（summary `mismatches`）；退出码 1。
- **计时边界**（§5/§7.3）：`wall_ms`=search 端到端；`cpu_ms`=进程 CPU；
  `postprocess_ms`=结果校验；`conversion_ms`=uint8→GridView 解码（计时外、
  单列）；输入生成/JSON/磁盘在计时外。`peak_rss_bytes` 为 1 ms 采样估计。
- **超时**：状态 `timeout`、`timeout=1`；不从成功率剔除、不自动重跑求成功。
- **进程隔离**：每个后端在独立子进程中运行（RSS/崩溃域隔离），父进程合并
  `benchmark_metrics.csv` 与 `bench_summary.json`；卡死安全网 = metrics 超过
  `4×最大预算+300 s` 无新增写入 → SIGTERM → 5 s → SIGKILL。

### 3.1 metrics.csv 列

固定列（tools/README.md §7.2）：`run_id, case_id, repeat_id, cache_mode,
seed, status, failure_reason, preprocess_ms, wall_ms, cpu_ms, peak_rss_bytes,
expanded_nodes, collision_checks, path_length_m, gear_changes,
endpoint_position_error_m, endpoint_yaw_error_rad, valid`。

§5 栅格专有列：`instance_id, scale, class, neighborhood, pair_id, budget_s,
timeout`；距离场专有列：`source_id, unreachable_cells, max_distance_m,
reference_cost`（距离场行 `reference_cost`=参考最大可达距离，另有
`reference_max_distance_m`/`reference_unreachable_cells` 明示）。

组件附加列：`backend, query, single_list, reference_reachable, cost_m,
cost_error_m, path_cells, field_cells, field_mismatch_cells,
field_max_abs_error_m, reference_max_distance_m, reference_unreachable_cells,
conversion_ms, postprocess_ms, rss_input_bytes, rss_search_bytes,
rss_output_bytes, rss_hwm_bytes`。未测量项留空（不用 0 代表未测量）：
`collision_checks`（所有后端未暴露）、vss2sn 的 `expanded_nodes`（上游不暴露，
provenance 登记）、warm 模式的 `preprocess_ms`。

### 3.2 容量档（--capacity）

- 单次冷启动（warmup 0 + trials 1）；2048²/4096²、种子 0；点到点 5 s、
  距离场 30 s；超时单列。
- 峰值 RSS 构成（采样估计）：`rss_input_bytes`（实例解码后增量）、
  `rss_search_bytes`（搜索期间峰值 − 解码后）、`rss_output_bytes`
  （返回后保留 − 搜索峰值）；`conversion_ms` 单列；实例间 `malloc_trim`。
- 扩重复判据（§3.3，`--expand-repeats`）：仅当 (a) success、(b) 墙钟 ≤50%
  预算、(c) 峰值 RSS ≤50% 内存、(d) 预估组时长 ≤2 h 时扩为“预热 1 + 测量 5”；
  否则保持单次并在 `expansion_notes` 写明原因。容量结果单列“容量/上限”组，
  不混入正式排名。

## 4. 输出与用法

```sh
# 开发期（经入口；占 worker 名额、走监控门禁、编译持全局锁 -j4）：
python3 tools/task_runner.py exec --component grid_search --workers 1 -- \
  bash -lc 'Src/grid_search/scripts/standalone_build.sh /tmp/opencode/gs_bench_build'

python3 tools/task_runner.py exec --component grid_search --workers 1 -- \
  bash -lc 'Src/grid_search/benchmarks/run_suite.sh \
    --build-dir /tmp/opencode/gs_bench_build \
    --output-dir {run_dir}/outputs --smoke --capacity-gen'
```

`run_suite.sh` 在 `<output-dir>` 内产出（不写目录之外）：

```
scenario/instances.json                 实例表（含 SHA256、起终点、源点、期望）
scenario/instances/<id>.u8              打包 uint8（>10 MiB 归档时改为引用）
scenario/instances.sha256               SHA256 清单
scenario/generation_summary.json        生成摘要与自检结果
reference/reference.json                独立参考（代价/可达性/场聚合/场文件引用）
reference/fields/<id>.<neigh>.f64       逐单元 float64 参考场
benchmark_metrics.csv                   每测量 trial 一行（契约文件）
bench_summary.json                      JSON 汇总（P50/P95、成功率、超时、不匹配）
workers/<backend>/{metrics.csv,summary.json}
capacity_scenario/...                   --capacity-gen 的容量输入生成烟测
```

注册片段见 `benchmark_suite.json`：把 `components.grid_search` 条目并入
`tools/components.json` 后，`benchmark --component grid_search --suite v1`
（formal）与 `--suite capacity` 即可在独占窗口运行。

## 5. 证据与烟测发现

- 构建（独立目录 + 新增 benchmarks 目标）：run
  `20261012T135200685859Z_grid_search_55510e83`（首建）、
  `20261012T135327733725Z_grid_search_b96f5b09`（去除未用函数警告后的增量复建）。
- 主烟测（128² × 6 类 × seed 0；`vss2sn_dijkstra` + `justinhj_astar`；1080 测量
  trial、0 超时、0 不匹配、0 后端失败；含 2048²/4096² 生成烟测与 §3.4 归档）：
  run `20261012T135344281135Z_grid_search_7653791b`（exit 0）。该 run 内
  `outputs/benchmark_metrics.csv` 即契约指标文件；并入运行目录 `metrics.csv`
  由注册后的 `benchmark` 子命令执行（exec 不合并，见 tools/task_runner.py）。
- 发现烟测（`vss2sn_astar` 单后端）：run
  `20261012T135407821840Z_grid_search_93a6a3c4`（exit 1）——八邻域
  `grid-maze-128-s0` 与 `grid-border_corner-128-s0` 各 30/30 测量 trial 代价
  不匹配（maze：实测 38.736248 m vs 独立参考 35.631580 m），路径合法但次优；
  四邻域 0 不匹配；同一实例集其余 7 个后端（含 JPS 单列、justinhj 池开/关、
  Smac、libMultiRobot、jps3d A*、vss2sn Dijkstra 与距离场）全部匹配。
  初步定位（未改上游源码验证）：vss2sn A* 的 `compare_cost` 非严格弱序 +
  闭集不复开；provenance §1 登记的“仅影响并列顺序”与实际不符，小规模正确性
  场景未触发。按 §6.2 该后端在修复前阻止参与正式排名；修复属 X-S3
  适配/上游补丁范围（超出本代理文件边界，登记为遗留）。
- 容量档最小烟测（1 实例：deadend-2048-s0 × `vss2sn_dijkstra` × 八邻域，
  单次冷启动 + §3.3 扩重复自动评估）：run
  `20261012T135648813119Z_grid_search_bf545f25`（exit 0）——点到点与距离场
  各扩为预热 1 + 测量 5（12 测量 trial，0 不匹配、0 超时）；RSS 构成
  `rss_input_bytes`≈19.3 MB、`rss_search_bytes`≤49.9 MB、峰值≈70.8 MB、
  `conversion_ms`≈19.3 ms；超时列与单次/扩重复记录见 summary。容量全矩阵
  （2048²/4096² × 6 类 × 3 查询组）仍属独占窗口阶段。
- 正式矩阵生成烟测（6 类 × 128²/512²/1024² × 种子 0–4 = 90 实例，生成 +
  自检，不运行基准）：run `20261012T140020640874Z_grid_search_06c30d15`
  （exit 0）——90/90 自检通过、按规模/类各 30/15、物化 39.8 MiB、无归档。
- 全量 8 后端本地烟测（dev 运行，未封存为正式证据）：3060 测量 trial，
  0 超时，唯一不匹配为上述 vss2sn_astar；jps3d_jps 正确跳过四邻域并匹配
  “允许穿角”参考。
- 正式套件排除 `vss2sn_astar`（X-B1 修正，2026-10-12）：正式运行
  `20261012T153447989346Z_grid_search_1f21bd17`（v1）与
  `20261012T170213402366Z_grid_search_4ea08327`（capacity）因该后端已知次优
  问题（search-001）触发正确性门禁 exit 1：v1 共 900/5400 测量不匹配
  （八邻域 900/2700，四邻域 0/2700 有效），capacity 共 3/114 不匹配
  （八邻域 3/47，四邻域 0/67）。两个失败 run 按 §6.2 保留：四邻域有效数据
  可作辅助证据，八邻域失败数据同样保留于失败 run。按“阻止参与排名、其余
  后端继续”，`tools/components.json` 的 `grid_search.benchmark_suites` 现对
  `v1` 与 `capacity` 显式传入 `--backends` 白名单（其余 7 个后端：
  `smac_2d_astar`、`jps3d_astar`、`jps3d_jps`、`vss2sn_dijkstra`、
  `libmultirobot_astar`、`justinhj_astar`、`justinhj_astar_nopool`），其余
  参数不变；修复并复验通过后恢复。

遗留风险：

1. **vss2sn_astar 八邻域次优路径（烟测实测）**：见上；正式套件已按上述
   `--backends` 白名单排除该后端（X-B1 修正），修复并复验通过后恢复；
   排除期间其正式档结果只作“失败/阻塞”证据。
2. vss2sn 不暴露展开计数（`expanded_nodes` 留空）；`collision_checks`
   所有后端未暴露（留空）；Smac 桩不含足迹/膨胀/costmap 层语义。
3. vss2sn 非方形填充转换成本未测量：本基准全部实例为方形（S×S）。
4. `jps3d_jps` 单列原生穿角语义（参考口径与统一组不同）；不混排名。
   jps3d 上游在闭节点可改进时向 stdout 打印 `ASTAR ERROR!`
   （`graph_search.cpp:203`；烟测出现但结果与参考一致），日志原样保留。
5. 逐单元 Python 参考成本（本机实测）：128² 全部实例 <1 s；2048² 单实例单
   邻域约 10 s（含点到点与完整场）；1024² 全量约数十分钟/约 0.6 GiB 场文件；
   容量档全量 2048²/4096² 参考预计约 10–30 min 与约 1 GiB 场文件。可按需用
   `--filter-class`/`--neighborhoods`/`--backends` 缩小容量参考范围（缩小后
   必须保持原规模标签，不得改写为更小规模）。
6. 逐 trial RSS 为 1 ms 采样估计；亚毫秒搜索峰值≈基线。
7. 后端预算钩子失效时由父进程卡死安全网终止（`4×预算+300 s`）。
8. warm 模式无独立准备接口：转换成本并入 `wall_ms`（登记）。
9. 全量正式矩阵与容量全矩阵未在本轮执行（独占窗口阶段）；容量档生成、
   §3.4 归档与单实例容量搜索/扩重复路径已在烟测 run 中验证（见上）。
