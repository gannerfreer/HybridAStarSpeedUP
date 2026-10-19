# planner_hybrid_astar（Hybrid A* 适配框架与三上游后端）

依据 `Docs/component_development_plan.md` §6.2/§6.3/§7.1/§7.2 交付。当前状态：
统一适配器契约 + 9 个后端（`ros_free`、Smac/KTH/zm0612 各 native + normalized、
用户自研 `user_astar_raw`/`user_astar_normalized`）、v2 夹具 34/34 正确性通过、
`--suite v1` 基准数据与初步选型（严格门槛下仅 `kth_normalized` 合格，性能领先者
`zm0612_normalized` 待路径质量优化；`user_astar_normalized` 经 hybrid-005 开发
优化后质量门槛在开发测量中达标，未重跑正式基准；正式选型窗口由协调者裁定）。

## 目录

| 路径 | 内容 |
| --- | --- |
| `include/planner/hybrid_astar/backend.hpp` | 统一契约：`configure` / `plan` / `reset_cache`，`native`/`normalized` 标签，参数与统计支持表，注册表 |
| `include/planner/hybrid_astar/rs_provider.hpp` | RS 提供者抽象（距离 + 解析连接）；`PlannerRsProvider` 封装正式 `planner::rs` |
| `include/planner/hybrid_astar/ros_free_backend.hpp` | `ros_free` 后端与冻结配置 `RosFreeConfig` |
| `src/` | 搜索实现（`ros_free_backend.cpp`）、目标势能（`grid_potential.*`）、注册表、RS 提供者 |
| `tests/` | 契约测试 + `tests/fixtures/v2/paths/cases.json` 夹具测试（v2 修订 14 例，其余 20 例与 v1 相同） |
| `tests/fixture_scene.hpp` | 测试与基准共享的夹具装载与独立路径校验 |
| `benchmarks/hybrid_astar_bench.cpp` | 基准 `planner_hybrid_astar_bench`（`--suite v1 --output-dir`，指标契约见 tools/README §7.2） |
| `examples/plan_scene_example.cpp` | 固定墙体场景示例与 `--self-check` |
| `docs/upstream_notes.md` | 上游固定清单（commit/许可证）与对照笔记 |

## 适配器契约（§6.3）

- `configure(config)`：解析并冻结参数；JSON 类型/范围错误整体拒绝
  （`accepted=false`）。未知键与后端无法映射的值记录在
  `ConfigReport::unsupported`；只要存在 `unsupported`，`plan()` 返回
  `invalid_input` 并在原因中列明——绝不静默忽略。
- `plan(request, grid, collision_checker, rs_provider)`：校验请求、地图与
  车辆一致性；循环内检查时间预算、节点预算与取消（`request.cancelled`
  与可选 `set_cancel_callback` 钩子）；失败原因与统计完整返回。
- `reset_cache()`：清除与地图/目标/车辆相关的全部缓存（目标势能）。
  热启动仅在 `reset` 周期内有效，通过统计 `cache_mode`（`cold`/`warm`）
  可见。
- `native`（上游复现，仅参考）与 `normalized`（统一契约，参与选型）由
  `BackendKind` 区分并写入统计。
- 换向点契约（hybrid-001，Wave 6 修复）：换向姿态同时作为前段末点与后段
  首点（同 pose、同 s_m，方向字段区分档位），与 serialization README 及
  playback 样例 `path_hybrid_reverse.json` 一致；`PathQuality.
  cusp_poses_are_duplicated_with_zero_gap` 断言 park_116–120、
  deadend_122/125 等场景的 cusp 断口为 0（修复前为 0.3–2.0 m 原语步长）。

## 后端清单（Wave 5）

所有后端共用同一 ROS-free 搜索核心（离散状态 A* + 运动原语 + RS 解析连接，
`planner::rs`/`planner::collision` 正式组件）；native 为按固定上游版本重写的
**结构复现**（未复制上游代码，见 `docs/upstream_notes.md`），仅作参考；
normalized 为冻结参数组、参与选型。

| id | kind | 结构/参数来源 | 关键默认参数 |
| --- | --- | --- | --- |
| `ros_free` | normalized | 自研基线（Wave 4） | 72 bins、2.0/1.5 m、5 转向、RS 启发式+解析连接 |
| `smac_native` | native | nav2 `nav2_smac_planner` @ bcc9c48 | 64 bins、1.415 m 弦长原语、3 转向、reverse 2.1、non-straight 1.2、ratio 3、解析上限 3 m |
| `smac_normalized` | normalized | Smac 结构 + 正式组件 | 同上 + 5 转向、ratio 1、上限 50 m |
| `kth_native` | native | KTH path_planner @ ddf1a3a | 72 bins、0.707 m 原语、R=6 m、3 转向、penalties 1.05/2.0/2.0、30000 迭代上限 |
| `kth_normalized` | normalized | KTH 结构 + 正式组件 | 车辆自身 R、0.5 m 去重网格、5 转向、ratio 1、上限 50 m |
| `zm0612_native` | native | zm0612/Hybrid_A_Star @ cf0cc99 | 72 bins、1.6 m、3 转向、shot ≤5 m、penalties 1.05/2.0/1.5 |
| `zm0612_normalized` | normalized | zm0612 结构 + 正式组件 | 同上 + 9 转向采样、2.0/1.5 m 步长、ratio 1、上限 50 m（Wave 6 冻结配置变更，deadend_124 路径比 1.181→1.000） |
| `user_astar_raw` | native | gannerfreer/HybridAstar @ b6e5383a（vendored，参考） | 72 bins、0.5 m 去重/内部碰撞栅格、1.0 m 步长、5 转向、内置目标容差、无 RS 连接 |
| `user_astar_normalized` | normalized | 同上 + 正式组件 + 势场预检/剪枝（hybrid-005） | 同上 + 公共碰撞检查、RS 目标连接（上限 11.8 m、退化分段清理/拒绝）、势能缓存 |

上游参数名通过别名映射到核心键（如 `angle_quantization_bins→heading_bins`、
`minimum_turning_radius→min_turn_radius_m`、`cellSize→xy_resolution_m`、
`penaltyTurning→turn_penalty`、`reversing_penalty→reverse_cost`、
`steering_change_penalty→steer_change_cost`、`shot_distance→
analytic_shot_distance_m`、`iterations→default_node_budget` 等）；无法对应
的上游键（车辆参数、Dubins 开关、可视化开关等）记入 `unsupported` 并附
原因，设置它们会让 `plan()` 返回 `invalid_input`，绝不静默忽略。已记录的
偏差：KTH 去重网格 0.5 m（上游 cellSize 1.0 m，上游用 tie-breaker 处理同格
后继）、KTH Dubins 前向 shot 以完整 RS 解析连接替代、Smac 回顾代价/二次
代价/原语插值未实现。

## `ros_free` 后端

自行实现的 ROS-free Hybrid A*：离散状态搜索 + 运动原语（用
`planner::rs::integrate_segment` 积分，方向/曲率约定与 RS 一致），启发式
`h = max(RS 距离, 目标势能距离)` 加权；运动原语与 RS 解析连接都做整车
扫掠碰撞检查；解析连接通过后才生成结果；最后重建路径、按换向点切分
`PathSegment` 并再次校验（终点 ≤0.05 m / ≤0.02 rad、曲率限值、整车
无碰撞复检）。后处理耗时计入总耗时。

配置键（默认值见 `RosFreeConfig`）：`xy_resolution_m`(0.5)、
`heading_bins`(72)、`motion_step_m`(2.0)、`reverse_motion_step_m`(1.5)、
`steer_fractions`([-1,-0.5,0,0.5,1])、`allow_reverse`(true)、
`forward_cost`(1.0)、`reverse_cost`(2.0)、`steer_cost`(0.1)、
`gear_change_cost`(2.0)、`heuristic_weight`(1.0)、
`analytic_expansion_ratio`(1)、`analytic_expansion_max_length_m`(50)、
`smooth`（仅 false）、`default_time_budget_s`(5.0)、
`default_node_budget`(1000000)。请求未给预算时使用后两者（§7.2 默认）。

统计：`cache_mode`、`expanded_nodes`、`generated_nodes`、
`closed_states`、`collision_checks`、`motion_rejections`、
`pruned_unreachable_states`、`stale_frontier_entries`、`rs_distance_calls`、
`rs_connection_attempts`、`search_time_ms`、`postprocess_time_ms`、
`total_time_ms`、成功时的 `path_length_m`/`gear_changes`/
`endpoint_position_error_m`/`endpoint_yaw_error_rad`。
`cpu_ms`、`peak_rss_bytes`、`smooth_time_ms` 本后端不测量：结果中不出现，
并在 `unsupported_statistics` 中给出原因。

## `user_astar`（用户自研后端，hybrid-005）

`user_astar_raw`（kNative，参考）与 `user_astar_normalized`（kNormalized，候选）
接入作者本人的 `gannerfreer/HybridAstar`（提交 `b6e5383a…`）vendored 源码，
逐项改写见 `upstream/user_astar/PROVENANCE.md`。raw 保持上游原样行为（内部
栅格碰撞、原始倒车运动学、内置目标容差、无 RS 连接），仅作参考，不参与
选型；normalized 使用修正后的标准倒车运动学 + 公共 `planner::collision`
位姿/运动检查 + 公共 `planner::rs` 目标连接 + 预算/取消 + 成功后端校验。

hybrid-005 定向优化（**开发验证，未重跑正式基准**）：

- **连通性预检与可达性剪枝（复用组件内 `grid_potential.hpp`，不复制算法）**：
  normalized 模式先校验碰撞检查器场景栅格与入参栅格一致（与共享核心同一
  前置条件），再做起/终点公共碰撞检查（非法输入优先），然后在仅自由单元上
  做 8 连通 Dijkstra 目标势场；势场按 grid+goal 缓存（`cache_mode` 区分
  cold/warm，遵循 `reset_cache` 周期）。起点单元在自由栅格松弛中不可达时
  直接 `no_path`（0 扩展、0 原语、仅 2 次位姿检查）；搜索内用同一势场剪枝
  不可达后继（新增统计 `pruned_unreachable_states`）。raw 模式不预检、不剪枝、
  无缓存，行为不变。开发测量（30 次冷启动 × 12 场景，run
  `20261011T024832556803Z_hybrid_astar_50383184`）：`unreachable_127` 从
  **35418 次扩展 / ~2.4 s** 降到 **0 扩展 / ~1 ms**；12 场景冷启动 CPU 合计
  从 ~112.5 s 降到 ~50.6 s（正例合计 ~40.7 s → ~50.6 s，质量修复的代价约
  1.24×；负例从 71.8 s 降到 0.03 s）。
- **目标连接策略（质量调整，不放松任何碰撞/契约门槛）**：RS 连接先剔除
  微米级数值退化分段（`< 1e-3 m`，避免虚假换向与端点噪声）；连接内亚原语
  分段（`< min_connector_segment_m`，默认 0.15 m）若与相邻分段构成换向则
  拒绝该连接并继续搜索；连接长度上限默认从 50 m 收紧到 11.8 m（拒绝
  `park_116/118` 的 12.0–12.65 m 贪心长连接，迫使搜索靠近目标后再连接；
  0 = 不限制，完整性优先）。开发测量（冷启动，对照非正式记录
  `20261010T164056476646Z` 的参考候选值）：`obstacle_106` 换向 **2→0**、
  `obstacle_109` 换向 **5→2**（两者门槛 ≤1 / ≤2），`park_118` 路径
  **23.024→20.501 m**（比值 1.156→**1.030**，门槛 ≤1.10）；其余 8 个正例
  路径比 ≤1.036、换向达标（`park_116` 20.641 m / 换向 1，CPU 约 1.8×）。
- 正确性：34/34 夹具状态与文档期望一致（`unreachable_126/127/129/130` 走
  0 扩展预检；`unreachable_128` 因自由栅格经 1.5 m 缝隙连通仍穷举证明）；
  新增回归用例 `UpstreamBackends.user_astar_normalized_reachability_precheck`、
  `user_astar_tuning_parameters_round_trip` 与
  `user_astar_normalized_rejects_mismatched_checker_grid`；全量测试 **43/43**
  （run `20261011T024739338693Z_hybrid_astar_6f515c08`）。
- **未重跑正式基准的原因**：用户已指示停止重跑（hybrid-005 属非正式记录项），
  本轮只做开发与验证；正式基准窗口与封存由协调者另行安排。默认后端维持
  `kth_normalized` 不变。

normalized 配置键（默认值见适配器；上游别名同样接受）：`xy_resolution_m`(0.5)、
`heading_bins`(72)、`motion_step_m`(1.0)、`steering_samples`(5)、
`allow_reverse`(true)、`forward_cost`(1.0)、`reverse_cost`(2.0)、
`turn_penalty`(1.0)、`switch_cost`(2.0)、`goal_capture_radius_m`(8.0)、
`analytic_expansion_ratio`(3)、`analytic_expansion_max_length_m`(11.8)、
`min_connector_segment_m`(0.15)、`default_time_budget_s`(5.0)、
`default_node_budget`(1000000)。

## 构建与测试（经统一入口）

```sh
python3 tools/task_runner.py build --component hybrid_astar --profile release --parallel 4
python3 tools/task_runner.py test --component hybrid_astar --workers 1
```

测试用例：适配器契约（注册表、配置、unsupported、取消、预算、缓存失效、
输入一致性、统计语义）+ `paths/cases.json` 全部 34 例
（open/obstacle/corridor/park/deadend/unreachable/boundary），成功例
按 §7.1 容限与整车无碰撞校验，负例按文档状态断言。

## 基准与选型（Wave 5）

```sh
python3 tools/task_runner.py benchmark --component hybrid_astar --suite v1
```

`planner_hybrid_astar_bench --suite v1 --output-dir <dir>`：12 个代表性场景
（open/obstacle/corridor/park/deadend 成功例 + `unreachable_127` 负例）×
7 后端；每场景 5 次预热 + 30 次测量（seed 0–29），冷/热分离，预算
默认 5 s（`--time-budget-s` 可调）/ 1e6 节点，后处理计入总耗时；输出
`benchmark_metrics.csv`（固定列 +
组件列，未测量留空）、`plots/latency_*_{p50,p95}_by_backend.svg/.csv` 与
`selection.json`。`selection.json` 按 §7.2 完整规则输出
`qualified_candidates` / `default_backend` / `pending_optimization`，并单独
给出仅作参考的 `fastest_backend`（附参考语义说明）。基准 run（Wave 6）：
`20261010T034348462837Z_hybrid_astar_b2fe2021`（success/0，5040 trials，
约 8.5 分钟，独占窗口）。

冷启动成功样本结果（330 正例 trial/后端；`unreachable_127` 30/30 正确
`no_path` 不计入成功延迟）：

| 后端 | kind | P50 ms | P95 ms | CPU 合计 ms | 路径比最大 | 换向违规 | 合格 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `kth_normalized` | normalized | 12.00 | 264.8 | 22669 | **1.003** | 无 | **✓** |
| `ros_free` | normalized | 8.01 | 168.4 | 13692 | 1.144 (deadend_124) | 无 | ✗ 路径 |
| `zm0612_normalized` | normalized | 9.36 | 192.3 | 22191 | 1.105 (obstacle_109) | 无 | ✗ 路径 |
| `smac_normalized` | normalized | 7.57 | 790.6 | 34979 | 1.324 (corridor_114) | corridor_114 | ✗ 路径+换向 |
| `kth_native`（参考） | native | 8.02 | 205.8 | 15677 | 1.131 | corridor_114 | — |
| `zm0612_native`（参考） | native | 11.43 | 302.7 | 20141 | 1.474 | corridor_114 等 | — |
| `smac_native`（参考） | native | 18.47 | 556.4 | 86677 | 1.295 | 多场景；park_118 30/30 未解 | — |

（路径比 = 每场景成功样本平均路径长 / 该场景合格候选最短；换向违规 =
超过逐场景最少合格候选 +1。Wave 6 的 zm0612 冻结配置变更：9 转向采样、
2.0/1.5 m 步长，deadend_124 路径比 1.181→1.000，代价是 P95 从 121.7 升到
192.3 ms。）

**选型（初步，严格按 §7.2；run `b2fe2021`）**：

- 共同成功场景 11 个；四个 normalized 候选均通过正确性门槛（34/34 用例，
  测试 run `20261010T034237466035Z_hybrid_astar_34910e02`）。
- 冷启动 P95 等权均值：`ros_free` 42.3 ms < `zm0612_normalized` 67.7 <
  `kth_normalized` 69.6 < `smac_normalized` 106.9；相邻差均 >5%，无需进入
  CPU/内存次级比较。
- 合格过滤（正确性 + 路径 ≤110% + 换向 ≤最少+1）：仅 **`kth_normalized`**
  合格；`pending_optimization` = `ros_free`（deadend_124 1.144）、
  `zm0612_normalized`（obstacle_109 1.105）、`smac_normalized`
  （corridor_114 1.324 + 换向）。
- 结论：**`selection.json` 的 `default_backend = kth_normalized`**（唯一
  合格候选）；`fastest_backend = ros_free` 仅作参考，不代表选型结果；
  三个待优化项与其失败原因已结构化输出，正式窗口与最终裁定属协调者。
- 热启动与冷启动耗时接近（本缓存的复用对象是目标势能，搜索仍需重跑），
  已在 `cache_mode` 列区分；内存未逐 trial 测量（列留空，由任务运行器
  采样）。

## 基准套件 v2（统一栅格档实验，2026-10-16）

目的：把四个 normalized 候选放到**统一状态栅格**（cell / 航向 / 步长）上做
公平比较，并把场景量扩到 fixtures v7 的 **36 例（6 类 × 6 变体，统一画布；
基准计时/门槛集合取 34 例，见下）**。
本套件为**实验性质**，不替换 v1 正式选型：`selection.json` 标记
`experimental=true`，qualified / pending / fastest 字段照旧计算但仅供实验
参考。

- 运行：`python3 tools/task_runner.py benchmark --component hybrid_astar --suite v2`
  （`tools/components.json` 的 `benchmark_suites.v2` 命令对齐 v1 形式，
  参数为 `--suite v2 --fixtures-dir {repo_root}/tests/fixtures/v7`；
  输出目录约定与 v1 相同）。
- **时间预算保持默认 5.0 s（门槛/选型用途，以补跑 run `6d2c209d` 为准）**：
  基准新增 CLI 参数 `--time-budget-s <秒>`（double，默认 5.0），但 v2 注册
  命令不降预算。实测 2.0 s 会把 300 个成功样本（墙钟 2.55–3.62 s，含
  `b2-corridor-04` × 3 个 normalized 的诊断样本）转为 timeout 并破坏门槛
  评估，故注册命令保持默认 5.0 s；如需纯速度实验可显式传
  `--time-budget-s 2.0`（结果不用于门槛）。节点预算 1e6 不变，实际预算
  回显在 `benchmark_metrics.csv` 的 `time_budget_s` 列与
  `selection.json.budgets`；v1 正式协议保持 5 s 不变。
- 场景：`tests/fixtures/v7/paths/cases_bench_v2.json`（schema 与 v2 的
  `paths/cases.json` 一致，车辆同为 `vehicles/default.json`）；场景加载/校验
  复用 v1 的 `prepare_scenario` 路径。基准计时/门槛集合为 **34 例 = 30 正例
  + 4 快速可证负例**：全量 36 例中排除穷举型负例 `b2-unreachable-03/06`
  （缺口 1.5/1.6 m、几何不可通过但自由栅格松弛连通，需穷举证明；按 v1
  惯例 `unreachable_128` 类负例不进入基准计时/门槛集合——首轮 5 s 预算下
  4 个 normalized 后端会超时，超时证据保留在 run
  `20261016T113415176181Z_hybrid_astar_a22d3479`；两例仍保留在 fixtures v7
  供正确性覆盖）。烟测 `--suite v2 --smoke` 取 3 例（`b2-open-01`、
  `b2-obstacle-01`、`b2-unreachable-01`），预热 1 + 实测 2。
- **统一栅格 overlay**：`ros_free`、`kth_normalized`、`zm0612_normalized`、
  `smac_normalized` 在 `configure()` 时叠加统一配置
  `{"xy_resolution_m": 0.5, "heading_bins": 72, "motion_step_m": 1.0,
  "reverse_motion_step_m": 1.0}`（四个键对这四个后端均被接受；
  `smac_normalized` 的 heading 由 64 覆盖为 72；代价项保持各自默认）。
  `smac_native`、`kth_native`、`zm0612_native` 保持原配置作参考行；
  user_astar 后端不接受 `reverse_motion_step_m`，不参与本实验。
- 证据：输出契约与 v1 相同（`benchmark_metrics.csv`、`plots/`、
  `selection.json`）；`selection.json` 额外回显 `unified_grid_overlay`、
  `unified_grid_backends`、`reference_backends` 与各后端 `effective_configs`，
  可直接核对 overlay 生效（如 `smac_normalized.heading_bins = 72`）。
- 烟测记录（2026-10-16，经入口 `exec --component hybrid_astar --workers 1`）：
  冻结版 v7（`fixtures verify` 73/73，run `20261016T113345557709Z_tools_eae26c62`）
  上 run `20261016T113351416872Z_hybrid_astar_01591ab6`（`--suite v2 --smoke`）：
  36 例加载成功、7 后端 84 trial、退出码 0；两个正例（`b2-open-01` /
  `b2-obstacle-01`）7/7 后端 success+valid，负例 `b2-unreachable-01`
  7/7 `no_path`；overlay 已生效（`selection.json.effective_configs`：4 个
  统一栅格后端 `heading_bins=72`、`motion_step_m=reverse_motion_step_m=1.0`，
  native 保持 64/1.415、0.7068582、1.6）。首轮烟测
  `20261016T111738697965Z_hybrid_astar_f4a1147f` 针对 v7 早期修订（15/72
  个起/终点足迹越出统一画布，成功例被碰撞检查判 `invalid_input`，保留为
  历史证据），fixtures 修订后重跑通过。另以独立预检（同一 `prepare_case`
  路径 + 正式碰撞检查器）核对修订版全部 36 例的 start/goal 位姿：0 失败
  （run `20261016T113109351710Z_hybrid_astar_5c8b6e08`）。
- 正式运行（2026-10-16，独占窗口，`benchmark --component hybrid_astar --suite v2`）：
  run `20261016T113415176181Z_hybrid_astar_a22d3479`（success，exit 0，墙钟
  95.2 min，15,120 实测 trial = 36×7×30×2，峰值 RSS ≈ 52.4 MiB）。状态分布：
  success 12,360 / no_path 2,280 / timeout 480；`invalid_input` 0、成功路径
  校验失败 0、负例误判成功 0。480 个 timeout 全部来自 `b2-unreachable-03/06`
  × 4 个 normalized 后端（两例缺口 1.5/1.6 m < 2.0 m 包络，几何不可通过，
  需穷举证明；5 s 预算内未完成，native 三后端可证明——与 v1 `unreachable_128`
  同类）。两个正例 `b2-corridor-02/03` 在 `smac_native`/`zm0612_native` 上
  `search space exhausted`（5–6 扩展即耗尽，3 档转向参考后端的已知局限；
  4 个 normalized 全部成功）。因 2 个穷举负例超时，4 个 normalized 的正确性
  门槛未过，本次 `selection.json` **无合格候选**（`default_backend=null`、
  `fastest_backend=null`，`pending_optimization` 列出 4 项），路径/换向门槛
  因参考集合为空而未生效；描述性统计（非门槛）：逐场景平均路径比相对最优
  normalized 的最大值 `ros_free` 1.006、`kth_normalized` 1.398、
  `zm0612_normalized` 1.298、`smac_normalized` 1.767（后三者最差均在
  `b2-corridor-04`），冷启动 P95 等权均值 `ros_free` 111.8 < `kth_normalized`
  152.8 < `zm0612_normalized` 177.1 < `smac_normalized` 193.9 ms。正式选型
  仍以 v1（`kth_normalized`）为准，本实验不替换。
- 正式补跑（2026-10-16，34 场景 = 30 正例 + 4 快速负例，独占窗口）：run
  `20261016T131648767025Z_hybrid_astar_6d2c209d`（success，exit 0，墙钟
  38 min 20 s，14,280 实测 trial = 34×7×30×2，峰值 RSS ≈ 47.2 MiB）。
  timeout 0、`invalid_input` 0、负例误判成功 0、成功路径校验失败 0；4 个
  快速负例全 `no_path`。门槛可评估：**唯一合格候选 `ros_free`**
  （路径最大比 1.006、均值 1.001，无换向违规）→ `default_backend =
  ros_free`、`fastest_backend = ros_free`（实验参考，不替换 v1 的
  `kth_normalized`）；`smac_normalized` / `kth_normalized` /
  `zm0612_normalized` 因 `b2-corridor-04` 路径比（1.767 / 1.398 / 1.298）
  与换向超限进入 `pending_optimization`（zm0612 另有 `b2-deadend-06`
  1.147）。冷启动 P95 等权均值：`ros_free` 112.8 < `kth_normalized` 152.9
  < `zm0612_normalized` 176.6 < `smac_normalized` 195.1 ms。native 参考
  在 `b2-corridor-02/03` 的已知失败不变（各 60 行）。

## 基准套件 v3（停车基准 115 场景实验，2026-10-17）

目的：把 IEEE TITS 2022 论文《Optimization-Based Trajectory Planning for
Autonomous Parking …》的 115 个停车基准场景（ParkingMotionPlanningTITS21
`ParkingBenchmarks/CaseNo_1..115.mat`）接入本组件，对 4 个统一栅格
normalized 后端（`ros_free` / `kth_normalized` / `zm0612_normalized` /
`smac_normalized`）做逐场景对比测试，并输出 Plotly 叠加对比图。

- 夹具：`tests/fixtures/v8/`（v8 = v7 全量复制 + `paths/cases_bench_parking.json`
  + `vehicles/parking_bench.json`；论文画布每边外扩 0.1 m，车辆安全边距 0；
  生成/预检详见该目录 README 与 `tools/parking_fixtures.py`）。
- 运行：`python3 tools/task_runner.py benchmark --component hybrid_astar --suite v3`
  （注册命令：`--cold-only --warmup-runs 0 --measured-runs 1 --time-budget-s 30
  --jobs 8 --vehicle parking_bench.json --export-paths-dir {run_dir}/outputs/paths`）。
- 场景与期望：115 例按文件顺序全量；`expected` 统一 success（论文基线），
  失败只记录（探索性对比，选择字段不替换 v1 正式选型，
  `selection.json.parking_suite=true`、`experimental=true`、`protocol` 回显）。
- 输出：`benchmark_metrics.csv`（460 行）、`paths/<case_id>/<backend>.json`
  （plan_result JSON，逐 trial 导出）、`selection.json`；预检模式
  `--precheck` 输出 `precheck.json`（起终点碰撞，115/115 通过）。
- 对比图：`tools/parking_compare_plots.py`（Plotly，实验辅助，依赖 plotly）→
  逐 case `case_<i>.html` + `index.html`；图集归档
  `artifacts/figures/parking_path_compare_v3_20261016T161016Z_v2/`（含随轨迹
  虚拟车辆框与一键隐藏/显示按钮；首版见同级 `...T161016Z/`）。
- 正式运行（2026-10-16，独占窗口，run `20261016T161016400422Z_hybrid_astar_cc74cfaa`，
  墙钟 102.6 s，8 worker）：**460/460 成功且通过冻结校验**（0 失败/0 超时）；
  成功样本墙钟 P50/P95（ms）：`ros_free` 195/6973、`kth_normalized` 289/9073、
  `zm0612_normalized` 158/6531、`smac_normalized` 302/7004（单次冷启动样本，
  P95 为最近秩）。预运行校准：3 场景 @5 s 有 2 个超时（8 个 success 中），
  @30 s 全部 ≤5.8 s——5 s 预算不足、30 s 充足。报告：
  `Docs/reports/hybrid_astar/20261016T161016400422Z_hybrid_astar_cc74cfaa.md`。
- 扩展（本次新增 CLI）：`--jobs N`（后端×场景组合并行，单进程集中输出；
  单 trial CPU 口径为线程级）、`--export-paths-dir`、`--vehicle`、
  `--warmup-runs/--measured-runs/--cold-only`、`--precheck`。

## 基准套件 v4（ParkBench 受限停车 51 场景实验，2026-10-17）

目的：把 ParkBench（arXiv:2601.22545，CC-BY-NC-4.0，`dqm5rtfg9b-collab/
Constrained_Parking_Scenarios`，commit `20b94635…`）的 51 个受限倒车入库场景
接入本组件，用**其论文车辆**（4.95×2.00 m、轴距 3.0 m、满舵 32°）对 4 个统一
栅格 normalized 后端做逐场景对比测试并输出 Plotly 叠加图。

- 夹具：`tests/fixtures/v9/`（v9 = v8 全量复制 + `paths/cases_bench_parkbench.json`
  + `vehicles/parkbench_vehicle.json`；数据清洗：离群段过滤、目标/起点足迹框
  外扩 0.10 m 裁剪（对齐官方 `utils.py` 语义并扩展到起点框）、线段→厚 0.05 m
  薄凸四边形；逐例审计与许可说明见该目录 README；源 zip 归档于
  `artifacts/data/sha256/5435c67b…/`）。生成/检查：`tools/parkbench_fixtures.py`。
- 运行：`python3 tools/task_runner.py benchmark --component hybrid_astar --suite v4`
  （协议与 v3 相同：冷启动单次、30 s 预算、8 worker、路径导出）。
- 预检：`--precheck` 51/51 通过（结论并入夹具 `self_check`）。首次以 0.05 m
  裁剪余量时 3 例（pb-016/028/029）因 0.1 m 栅格化单元伸入目标足迹被判无效，
  提高到 0.10 m（≥ 栅格半对角 0.0707 m + 裕度）后全过——该结论已写入夹具
  README 与转换器文档。
- 正式运行（独占窗口，run `20261017T024554664760Z_hybrid_astar_45911dbb`，
  墙钟 85.0 s，8 worker）：**204/204 成功且通过冻结校验**（0 失败/0 超时）；
  P50/P95（ms）：`smac_normalized` 849/6937、`kth_normalized` 982/9349、
  `ros_free` 1426/9872、`zm0612_normalized` 1671/16823（单次冷启动样本）。
  实验性描述统计：逐场景路径比最大 `smac_normalized` 1.216（pb-030）、
  `ros_free` 1.401（pb-025）、`kth_normalized` 1.429（pb-025）、
  `zm0612_normalized` 1.456（pb-047）；四后端均未过实验性门槛
  （`default_backend=None`，符合预期——ParkBench 未按 §7.2 门槛设计）。
- 报告：`Docs/reports/hybrid_astar/20261017T024554664760Z_hybrid_astar_45911dbb.md`；
  对比图集：`artifacts/figures/parkbench_compare_v4_20261017T024554Z_v2/`（含随
  轨迹虚拟车辆框与一键隐藏/显示按钮；首版见同级 `...T024554Z/`）。

## 夹具结果（v2，2026-10-10）

测试加载 `tests/fixtures/v2/paths/cases.json`（v2 = v1 全量复制 + 14 例修订；
14 例 `grid.map_version=v2`，其余 20 例仍为 `v1`）。正确性运行对未显式给
预算的用例使用 60 s 预算（后端默认仍为 §7.2 的 5 s / 1e6 节点基准预算）。
34 例实测分类（最终 run `20261009T182750107965Z_hybrid_astar_e5f5f38c`，
断言加强版；首轮 `20261009T182612765664Z_hybrid_astar_71b54272`）：

- **34/34 按文档期望通过**：open 5、obstacle 5、corridor 5、park 5、
  deadend 5、unreachable 5（127 自由栅格松弛断连、128 穷举）、boundary 4
  （起点碰撞/越界→invalid_input、取消→cancelled、8 节点/0.01 s→timeout）。
  成功例均满足终点 ≤0.05 m / ≤0.02 rad、整车路径无碰撞复检、曲率限值，
  换向点保留为 PathSegment 边界；park/deadend 类显式断言无未解决例。
  同一 34/34 亦由 `smac_normalized`、`kth_normalized`、`zm0612_normalized`
  在测试 run `20261010T014213933093Z_hybrid_astar_e025d645` 中通过。
- **0 例 fixture_conflict、0 例 unresolved**（v2 修订后不再出现）。分类
  机制保留：如后续版本再现冲突/未解决，测试会以
  `FIXTURE_CONFLICT` / `UNRESOLVED` 前缀输出明细并使相关断言失败。
- 历史（v1）：run `20261009T175408170666Z_hybrid_astar_28821807` /
  `20261009T174607194255Z_hybrid_astar_5729bd98` 记录的 6 例冲突与 8 例
  未解决是 C 组修订 v2 的依据（见 `tests/fixtures/v2/README.md`）。

## Wave 8 跨组件复核（hybrid-003）

对 Dftpav Wave 7 的三项观察在当前构建（测试 run
`20261010T062941678507Z_hybrid_astar_d3a45417`，36/36 通过）上做最小复现：

- **换向断口（不复现）**：park_116–120、deadend_122/125 在四个 normalized
  后端上逐例测量"前段末点 vs 后段首点"（同 pose、同 `s_m`、方向相反），
  28/28 全部 `max_gap=0.000e+00 m`。Dftpav 的 0.3–2.0 m 观察与
  `hybrid-001` 修复前的构建/输出一致，属陈旧构建或消费端未按冻结契约处理
  换向点重复姿态。
- **小回环（部分复现，已登记残余）**：`obstacle_107/109` 与
  `deadend_122/125` 在部分后端上存在 **1 处**非相邻段横切（collinear=0），
  典型为起点附近"前进-倒车-再前进"折返（obstacle_107：s0 前左弧与 s2
  直线相交）或出位转身折返；均为**无碰撞、契约有效**的小折返，不是闭合圈；
  `park_116/118/119/120` 零自交。修复尝试 1（软回环惩罚 `loop_penalty=15`，
  run `20261010T062737822375Z_hybrid_astar_6e382354`）消除了 ros_free 的
  obstacle 回环，但把 zm0612_normalized 的 obstacle_109 路径比从 1.105
  恶化到 1.245，按 §5.4 回退为默认关闭；`loop_penalty`/`loop_guard_segments`
  保留为可选参数（默认 0/16）。回归护栏
  `CrossComponent.self_intersection_inventory_matches_known_state`：park
  用例必须 0 自交、已知用例 ≤1 且非共线。
- **hybrid-004 定向核查（Dftpav 吸收路线，未闭环）**：上述四个场景 × 4 个
  normalized 后端按完整链路（Hybrid A* → 走廊 → Dftpav → V；局部车道走廊
  回退 0.8 m，同 deadend_124）实测（dftpav test run
  `20261010T182505541183Z_dftpav_2c829c44`，39/39 通过）：**13/16 组合 V
  通过**（3 个死胡同组合被 V 以 `validation_collision` 正确拒绝，属已知
  不可消费组合）；V 通过且初值含折返的 11 个组合中，优化输出**仍保留同一
  折返**——显著非共线横切每例 1 处，横切段长度从初值的 0.35–2.0 m 缩小到
  约 0.07–0.13 m（另有换向尖点处亚厘米原地微环，不计入显著项）。定向尝试
  （走廊模式/间隙、锚定权重、走廊收紧、分段时间、时间权重、走廊分辨率，
  共 3 轮）均未消除折返：**Dftpav 优化/平滑阶段不消除这类小折返**，故
  hybrid-004 保持 open；回归用例 `DftpavE2eChain.SmallFoldSeeds*`
  （dftpav tests）锁定已验证状态（V 门禁 + 折返保留）；消除需路径级修复
  （见下方消费端提示）。
- **hybrid-004 路径级修复评估（§5.4 blocker；结论：不采纳修复）**：对 12 个
  含折返组合做确定性"约束重搜索 + 最小干预选解"原型（工具在仓库外，未入库；
  基线 test run `20261010T195318897522Z_hybrid_astar_1c705850`，40/40 通过）：
  全部组合可得到 0 显著横切，但 `zm0612_normalized × obstacle_109` 的无折返
  最优解约 **25.15 m**（最短目标 2 万扩展/28k 扩展等三次独立测量
  25.142–25.163 m），与质量门 `worst_ratio ≤ 1.15`（kth 参考 20.676 m →
  1.216）不可兼得；该折返是墙 x∈[8,12]、y∈[3.5,6.5] + 地图顶边 y=12 与
  4.9×2.0 整车包络下获取清角偏航的**通过性机动**（三条替代路线分别以 ~6 cm
  擦墙、通道不可穿越、RS 段 ~6 cm 擦墙失败），且现解（含折返，比值 1.105）
  比无折返解更短。搜索期软惩罚、Dftpav 吸收、路径级修复三条路线均不采纳；
  折返清单由 `CrossComponent`/`SmallFoldSeeds*` 回归锁定，`hybrid-004` 按
  "通过性机动、非缺陷"闭环（见 ISSUES.md）。
- **终点贴边（非缺陷）**：obstacle_108 目标足迹在网格内，距上边界
  **0.1059 m**（含 0.1 m 安全裕量的整车足迹；四个后端达成终点一致）。这是
  夹具设计（目标靠近上边沿）而非越界，`check_pose`/路径复检均通过。
- 消费端提示：换向姿态按冻结契约在前段末点与后段首点各出现一次（同
  `s_m`）；消费方应按方向字段分段，避免把重复姿态当作零长段或回环；小
  折返对轨迹优化是可接受的初值（V 可通过）；Dftpav 优化/平滑与路径级修复
  实测均不消除/不采纳（hybrid-004 评估，见上）：折返是部分场景的通过性
  机动，现解通常更短；若未来要消除，需先裁定质量门（`worst_ratio`）与
  通过性机动的关系。

## 限制

- native 后端是按固定上游版本重写的 ROS-free **结构复现**（参考only），
  不是上游代码移植；碰撞/Dubins 由正式组件替代，个别参数（KTH 去重网格、
  Dubins shot、Smac 回顾/二次代价等）已在参数表与 README 中记录为偏差或
  unsupported。`smac_native` 在 park_118 上 30/30 未解（参考结果，不参与
  选型）。
- 所有后端均无额外平滑（`smooth: true` 为 unsupported）。
- 目标势能为栅格中心 Dijkstra（与 Smac 障碍启发式同类），是引导项，
  不作为严格可采纳性证明；RS 距离项是严格下界。
- 夹具场景的栅格采用"格心落入障碍即占据"的最小栅格化（cases.json 只
  固定尺寸/分辨率/原点与障碍几何）；v2 的可行性论证针对障碍几何（含
  整车 4.9 m × 2.0 m 包络），与该方法一致。
- 基准为初步数据：每 trial 内存未测量（列留空）；热启动复用目标势能而
  非搜索结果；`selection.json` 已按 §7.2 输出 `default_backend =
  kth_normalized` 与 `pending_optimization` 三项，正式选型需协调者安排的
  独占窗口与最终裁定。
