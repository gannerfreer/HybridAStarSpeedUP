# planner_grid_search 来源记录（X-S3 栅格 A* + X-S2 栅格 Dijkstra 部分）

- 日期：2026-10-12。执行代理：X-S3/X-S2 栅格部分（本组件独占 `Src/grid_search/**`）。
- 依据：[组件扩展计划](../../Docs/component_expansion_plan.md) §4.2（适配契约）；
  [候选来源登记](../../Docs/expansion/candidate_registration.md) S-02/S-03/S-06/S-07/S-08 与 §6.3；
  家族 A/B/C 探针报告（[a](../../Docs/expansion/probes/family_a_probe.md)、
  [b](../../Docs/expansion/probes/family_b_probe.md)、[c](../../Docs/expansion/probes/family_c_probe.md)）。
- 说明：本文件记录每个来源的固定信息、抽取清单、适配补丁、原生偏差与验证证据。
  接线（根 CMake / `tools/components.json` / `dependencies.lock.json`）由后续单一
  wiring 代理完成；本组件当前通过独立 CMake 构建（见 README.md），构建过程
  **不联网**（只使用已抽取源码与系统头 Eigen3 / Boost）。

## 0. 验证证据（run ID）

| 用途 | run ID | 结果 |
| --- | --- | --- |
| **最终构建 + CTest（Eigen 回退改动后，`grid_search` 标签）** | `20261012T130809893984Z_grid_search_236726db` | 8/8 CTest 通过（29 个 gtest 用例 + 2 个 Python 夹具复现 + 1 个示例自检） |
| **X-B1 主烟测（128² × 6 类；`vss2sn_dijkstra` + `justinhj_astar`；1080 测量 trial）** | `20261012T135344281135Z_grid_search_7653791b` | 0 超时、0 不匹配、0 后端失败（含 2048²/4096² 生成烟测与 §3.4 归档） |
| **X-B1 发现烟测（`vss2sn_astar` 单后端，见 §1.1）** | `20261012T135407821840Z_grid_search_93a6a3c4` | 失败保留（exit 1）：八邻域 60/360 测量不匹配（maze/border_corner 次优路径）；四邻域 0 不匹配 |
| 全新目录独立配置 + 构建 + CTest | `20261012T130415977092Z_grid_search_043254e4` | 8/8 通过（验证“从零独立配置”可用） |
| 适配器开关验证（关闭 justinhj/jps3d/smac 三个开关） | `20261012T130641123485Z_grid_search_a9650d49` | 构建成功；`available_backends()` 仅剩 3 个后端；示例自检通过 |
| 增量构建 + CTest（池关编译期断言修正后） | `20261012T130134341456Z_grid_search_f88db9fc` | 8/8 通过 |
| 前一轮通过（池关补丁生效前） | `20261012T125953515392Z_grid_search_894f5b52` | 8/8 通过（保留；池关配置由编译期断言修正后复跑） |
| 失败保留（编译/测试迭代） | `20261012T124529629965Z_grid_search_3897ceec`、`…_90eb0cab`、`…_3d6c9675`、`…_c32fbe90`、`…_4e3511b0`、`…_bc65d10a`、`…_6e2efeb5` | 见各 run 目录 `stdout.log`；失败原因（头文件路径、Eigen/全局命名空间、参考脚本源点错误、JPS 跳点路径口径、justinhj 池宏、JSON 夹具字段）均已修复并复跑 |

运行证据目录：`artifacts/runs/<run_id>/`（manifest、config、source 快照、日志、封存校验）。
测试用例与夹具复现：`Src/grid_search/tests/`、`Src/grid_search/tests/reference/grid_reference.py`。

## 1. S-06 vss2sn/path_planning（BSD-3-Clause）

| 字段 | 值 |
| --- | --- |
| URL | https://github.com/vss2sn/path_planning |
| commit | `d8f9edaeae064ab4665350bf820d1d5ebbbfcc45`（master HEAD，2022-11-15） |
| 许可证 | BSD-3-Clause；`thirdparty/vss2sn/LICENSE.md`，SHA256 `878e9cb40c3bccfaf4b15a4929a635eb4d725e75638bdaa5f1f77e274f4955ed` |
| 文件级许可 | 三个角色源文件与 `utils`、`planner.hpp` 均无 Copyright/许可头（仅 doxygen `@author`）；根 `LICENSE.md` 覆盖 |
| 获取方式 | 家族 B 探针工作区 clone（`/tmp/opencode/probes/family_b/src/path_planning`，探针阶段一次性获取）；抽取文件复制入 `thirdparty/vss2sn/`，构建不再下载 |
| 角色 | 栅格 Dijkstra（点到点 + 距离场）、栅格 A*（点到点）；四/八邻域 |

抽取清单（上游 SHA256 前 16 位 → 本仓 SHA256 前 16 位；“SAME”=逐字节一致）：

| 文件 | 上游 | 本仓 | 修改 |
| --- | --- | --- | --- |
| `include/path_planning/a_star.hpp` | `d3d9e592…` | `d3d9e592…` | SAME |
| `include/path_planning/dijkstra.hpp` | `0f19c249…` | `ee5f09ac…` | 补丁 P3 |
| `include/path_planning/planner.hpp` | `84f6c90c…` | `112dedee…` | 补丁 P2 |
| `include/utils/utils.hpp` | `b581e395…` | `1190b31e…` | 补丁 P1 |
| `src/a_star.cpp` | `f7f3c473…` | `baca5dc9…` | 补丁 P3/P4 |
| `src/dijkstra.cpp` | `40b62818…` | `c432b59f…` | 补丁 P3 |
| `src/utils.cpp` | `7bdd69f0…` | `347ab925…` | 补丁 P1 |

适配补丁（就地以 `[grid_search 适配补丁 …]` 标注）：

| # | 文件 | 内容 | 依据 |
| --- | --- | --- | --- |
| P1 | `utils.hpp` / `utils.cpp` | 新增 `GetMotion(bool eight_connected)`：八邻域运动原语（对角代价 √2）；`GetMotion()` 默认行为不变 | 上游仅四邻域（对角项被注释，且注释要求先改启发式） |
| P2 | `planner.hpp` | 新增 `SetMotion()`、`SetCancelChecker()`/`ShouldCancel()` 与保护成员（`motion_` 默认 = `GetMotion()`；取消钩子默认空） | 上游无取消/超时钩子；八邻域需要注入运动原语 |
| P3 | `dijkstra.cpp` / `a_star.cpp` | 搜索循环使用 `motion_`；循环头检查取消/超时；边界检查提前到目标判定之前；八邻域禁对角穿角（对角步要求两个正交邻格空闲，且在目标判定前检查） | 统一契约 §4.2；上游目标判定先于边界/障碍检查（越界目标可能被接受） |
| P3 | `dijkstra.hpp` / `dijkstra.cpp` | 新增 `Dijkstra::PlanDistanceField(source)`：完整距离场（与 Plan 相同的运动原语、禁穿角与代价；不可达 `+infinity`；取消命中返回部分场并由适配层丢弃） | 上游无距离场 API（登记 §S-06“Dijkstra 距离场缺口”；本组件选择“适配补丁”路径而非独立参考实现） |
| P4 | `a_star.cpp` | 八邻域（`motion_.size() > 4`）使用 octile 启发式，四邻域保持上游 Manhattan；新增 `<algorithm>` 包含 | 上游启发式为 Manhattan，与对角运动不一致 |

原生偏差（适配层补偿/登记）：

1. `Planner` 假定 **n×n 方形栅格**；非方形输入由适配层按 `max(width,height)` 填充为阻塞（转换成本单列，正确性不受影响）。
2. 上游目标判定先于边界与障碍检查（越界/被占据的目标可能被“找到”）；适配层在调用前拒绝越界/阻塞的起点与终点（统一 `kInvalidInput`）。
3. 上游返回路径为 **goal → start**；适配层反转为统一 `start → goal`。
4. 上游不暴露展开节点计数（`stats.expanded_nodes` 保持 0，`stats.upstream_status` 记录原生成功/失败）。
5. `compare_cost` 非严格弱序 + 闭集不复开（**2026-10-12 按 X-B1 烟测实测修正登记**：原登记“仅影响并列顺序”不成立——八邻域在 `maze`/`border_corner` 返回合法但次优路径，详见 §1.1；**修复前 `vss2sn_astar` 不参与正式排名**，四邻域与 `vss2sn_dijkstra` 不受影响）。
6. 上游无距离场、无取消/超时钩子、无八邻域 → 由 P1–P4 覆盖并全部登记。

### 1.1 vss2sn_astar 八邻域次优发现（X-B1 烟测实测登记，2026-10-12）

证据 run `20261012T135407821840Z_grid_search_93a6a3c4`（`vss2sn_astar` 单后端，
128² × 6 类、独立 Python 参考、预热 5 + 测量 30）：

| 实例（八邻域） | 实测代价 | 独立参考 | 偏差 | 不匹配 |
| --- | ---: | ---: | ---: | --- |
| `grid-maze-128-s0` | 38.736248 m | 35.631580 m | +3.104668 m（+8.7%） | 30/30 测量（另 10 次预热） |
| `grid-border_corner-128-s0` | 23.897056 m | 23.779899 m | +0.117157 m（+0.49%） | 30/30 测量 |

- 合计 **60/360 测量 trial `cost_mismatch`**：路径合法、端点正确，但代价高于独立参考；
  同实例集其余 7 个后端（含 JPS 单列、justinhj 池开/关、Smac、libMultiRobot、jps3d A*、
  `vss2sn_dijkstra` 与距离场）全部匹配。
- 初步定位（未改上游源码验证）：vss2sn A* 的 `compare_cost` 非严格弱序 + 闭集不复开；
  既有 6²–32² 正确性夹具未触发（小规模正确性测试与参考逐单元一致不等于全局最优）。
- 处置：**修复前 `vss2sn_astar` 不参与正式排名**（bench summary `mismatches` 阻止排名且
  exit 1，属门禁预期行为）；修复属 X-S3 适配/上游补丁范围，见 §8 遗留。
- **正式套件排除（X-B1 修正，2026-10-12）**：正式运行
  `20261012T153447989346Z_grid_search_1f21bd17`（v1）与
  `20261012T170213402366Z_grid_search_4ea08327`（capacity）复现该缺陷并触发
  正确性门禁 exit 1（v1：900/5400 测量不匹配，八邻域 900/2700、四邻域
  0/2700；capacity：3/114，八邻域 3/47、四邻域 0/67），两个失败 run 均按
  §6.2 保留。按“阻止参与排名、其余后端继续”，`tools/components.json` 的
  `grid_search.benchmark_suites` 对 `v1`/`capacity` 显式传入 `--backends`
  白名单（其余 7 个后端），其余参数不变；四邻域有效数据（失败 run 内）可作
  辅助证据，八邻域失败数据同样保留于失败 run；修复并复验通过后恢复。

## 2. S-07 Nav2 SmacPlanner2D（Apache-2.0，自 navigation2 抽取）

| 字段 | 值 |
| --- | --- |
| URL | https://github.com/ros-navigation/navigation2（`nav2_smac_planner`） |
| commit | `3286119753ce1fab9d2ff411ea6d439b823fe18d`（main，2026-10-09）；包版本 1.5.0 |
| 许可证 | Apache-2.0（包级 `package.xml`）；仓库 `LICENSE` SHA256 `e71fcb65bd31cdbdcf50cf43ab87a0c600ea3ec407495c2e15d1e68bc9c3bfc1`（随目录保留）；抽取文件保留上游 Apache-2.0 文件头 |
| 获取方式 | 家族 C 探针解耦工程（`artifacts/probes/family_c_20261012T111453Z/nav2_decoupled/`）；抽取文件复制入 `thirdparty/nav2_smac_planner/` |
| 角色 | 栅格 A*（几何口径统一组；代价感知模式单列） |

抽取清单（未修改文件与探针 `nav2_extract_sha256.txt` 一致；桩与探针 `nav2_stubs_sha256.txt` 一致）：

| 文件 | 上游 SHA256 前 16 位 | 本仓 | 说明 |
| --- | --- | --- | --- |
| `a_star.hpp` / `a_star_impl.hpp` | `626491fd…` / `0568baea…` | SAME | `AStarAlgorithm<Node2D>` 核心（未修改） |
| `node_2d.hpp` / `node_basic.hpp` / `goal_manager.hpp` / `constants.hpp` | `6528540c…` / `8b5c85d7…` / `71c392b4…` / `ea355e08…` | SAME | 未修改 |
| `thirdparty/robin_hood.h` | `a1ec2466…` | SAME | vendored 哈希库（未修改） |
| `types.hpp` | `b4305c8b…` | `d6791101…` | 补丁 P4（新增 `use_four_neighborhood` 字段） |
| `src/node_2d.cpp` | `1885bb32…` | `dd4320b6…` | 探针补丁 P1（`<cmath>`，保留）+ 补丁 P2（禁穿角）+ P4（四邻域偏移） |
| `analytic_expansion.hpp` | `f99928ef…`（上游） | `8c85339b…` | **适配桩（非上游）**：Node2D 专用等价替换，去除 OMPL 头依赖 |
| `analytic_expansion_impl.hpp` | `4207ece0…`（上游） | 不纳入 | 随上游 `analytic_expansion.hpp` 一并替换 |

最小桩集（适配层自写，文件头标注“NOT upstream code”，SHA256 与探针一致）：
`nav2_core/planner_exceptions.hpp`、`nav2_costmap_2d/costmap_2d.hpp`、
`nav2_ros_common/{lifecycle_node,node_utils}.hpp`、
`nav2_smac_planner/{collision_checker,node_hybrid,node_lattice}.hpp`。

适配补丁与配置：

| # | 位置 | 内容 |
| --- | --- | --- |
| P1 | `src/node_2d.cpp` | 增加 `#include <cmath>`（探针补丁，保留；上游依赖 ROS 头传递包含） |
| P2 | `src/node_2d.cpp` `getNeighbors` | 八邻域禁对角穿角：对角移动时对两个正交邻格调用同一 `NeighborGetter`/`isNodeValid` 索引路径，任一不可通行则跳过（~20 行，含注释） |
| P3 | 适配层 | `initialize(allow_unknown=false, …, terminal_checking_interval=1)`；`createPath(tolerance=0)`（关闭最接近路径回退，精确最短路口径） |
| P4 | `types.hpp` + `node_2d.cpp` `initMotionModel` | `SearchInfo::use_four_neighborhood`（默认 false = 上游八邻域）选择四邻域偏移；保持 `offsets[3] == +x_size` 布局不变量 |
| — | 适配层 | 取消/超时经 `cancel_checker`（预算/取消标志）与 `max_planning_time`；`PlannerCancelled` 异常由适配层按自身标志区分 `kCancelled`/`kTimeout` |

原生偏差（登记）：

1. 上游八邻域允许对角穿角（P2 改变原生搜索语义；补丁后不再等同上游原生行为——这正是统一契约要求）。
2. `start == goal`：上游 `backtracePath` 因起始节点无父节点返回 false；适配层短路为单单元成功结果。
3. 上游四邻域在 `node_2d.hpp` 文档中声明但实现固定八邻域（P4 补全；仅对照组使用）。
4. 代价感知模式（`cost_penalty > 0` / 逐单元代价层）：单列，不进入统一排名；启发式为几何 Euclidean，对含惩罚代价的最优性不作保证（测试仅验证其绕开高代价带与路径有效）。
5. 未纳入抽取（记录，非阻塞）：插件层 `smac_planner_2d*`、`smoother*`、`costmap_downsampler`、`utils.hpp`、`obstacle_heuristic`/`distance_heuristic`、真实 `costmap_2d`/`GridCollisionChecker` 足迹语义（桩只覆盖 Node2D 索引路径，逐语义复刻上游 `src/collision_checker.cpp` 第 172–189 行）。
6. 上游为滚动 main 分支：已锁定 commit；接线/复验时不得跟随分支移动。

## 3. S-08 jps3d（BSD-3-Clause）

| 字段 | 值 |
| --- | --- |
| URL | https://github.com/KumarRobotics/jps3d |
| commit | `ef65e36a8930395535e4683b1710e514b81d2f43`（master HEAD，2019-03-11） |
| 许可证 | BSD-3-Clause；`thirdparty/jps3d/LICENSE`，SHA256 `cb06c2c22ac7c551c6160706752abc41bae0ea68ce5a6c3bf342f01baa995641` |
| 文件级许可 | 抽取文件无独立 Copyright 头，根 `LICENSE` 覆盖 |
| 获取方式 | 家族 C 探针 clone（`/tmp/opencode/probes/family_c/jps3d`）；抽取文件复制入 `thirdparty/jps3d/` |
| 依赖 | Eigen3 头（系统 `/usr/include/eigen3`）、Boost heap 头（系统；header-only）；不链接 `jps_lib`/yaml-cpp（抽取源码直接编译） |
| 角色 | 普通 A*（`use_jps=false`，统一八/四邻域组）+ JPS 单列变体 |

抽取清单：

| 文件 | 上游 SHA256 前 16 位 | 本仓 | 修改 |
| --- | --- | --- | --- |
| `include/jps_basis/data_type.h` / `data_utils.h` | `db826ed6…` / `d1f36872…` | SAME | 未修改 |
| `include/jps_collision/map_util.h` | `1d7710d3…` | SAME | 未修改 |
| `include/jps_planner/jps_planner/graph_search.h` | `595d93f3…` | `4fc735fe…` | P1/P4 + 终止钩子声明 |
| `include/jps_planner/jps_planner/jps_planner.h` | `367ca491…` | `967620e1…` | P4 + 终止钩子声明 |
| `src/jps_planner/graph_search.cpp` | `326edba6…` | `c292d546…` | P1/P4 + 终止钩子 |
| `src/jps_planner/jps_planner.cpp` | `6d0497f2…` | `da5bd7b4…` | P4 + 终止钩子透传 |

适配补丁与配置：

| # | 内容 |
| --- | --- |
| P1 | `getSucc` 2D：八邻域禁对角穿角（对角步要求两个正交邻格空闲；上游只检查目标格） |
| P2 | unknown 预转换（适配层）：`GridView` unknown/地图外 → Tmap `occupied(1)`（上游 `updateMap` 把 unknown(-1) 当 free，且 start/goal 检查用 `isFree`） |
| P3 | `eps` 固定 1（`JPSPlanner::plan(…, 1.0, …)`；≠1 不得宣称最短路） |
| P4 | `GraphSearch::setFourConnect` + `JPSPlanner::setFourConnect`：普通 A* 模式重建四邻域 `ns_`；JPS 模式保持原生八邻域（jump 规则基于八邻域表，适配层对 JPS + 四邻域返回 `kInvalidInput`） |
| — | 终止钩子：`GraphSearch::setTerminationCheck` + `JPSPlanner::setTerminationCheck`（上游无取消/超时钩子；循环头检查，命中返回 false，适配层区分 `kCancelled`/`kTimeout`） |
| — | 世界↔单元映射：适配层直接构造单元中心坐标 `(cell+0.5)·res`（绕开上游 `floatToInt` 的 `-0.5` 约定偏差） |
| — | 统一输出使用 `getRawPath()`（搜索原始路径）；`getPath()` 的 `removeCornerPts/removeLinePts` 后处理不纳入统一口径；JPS 跳点路径由适配层展开为单位步（直线段，长度不变） |

原生偏差（登记）：

1. 上游 unknown 被搜索视为 free（P2 预转换修复，零源码改动）。
2. 上游八邻域允许对角穿角（P1 修复普通 A*；JPS 变体保持原生语义单列）。
3. `floatToInt` 单元中心约定（适配层绕开）。
4. 上游路径后处理（corner/line 简化）不用于统一输出；简化路径如保留需单列后处理成本（X-B1）。
5. JPS 与普通 A* 同源变体：JPS 结果单列，不充当独立来源、不混入统一排名；其代价与“允许对角穿角”的独立参考一致（测试覆盖）。
6. 排除面（明确不接入）：DMP 距离场规划器、3D 规划器/3D 重载、`create_map`/示例/测试（yaml-cpp 仅配置期依赖，本组件自带最小编译单元，不链接 `jps_lib`）。
7. 上游 2019 年后未更新；接口稳定（正面），无上游修复来源（风险登记）。

## 4. S-02 libMultiRobotPlanning（MIT）

| 字段 | 值 |
| --- | --- |
| URL | https://github.com/whoenig/libMultiRobotPlanning |
| commit | `4c75fa20c435c440d8b6bd6dc81668ddc7296ba0`（main HEAD，2023-07-28；仓库无 tag） |
| 许可证 | MIT；`thirdparty/libMultiRobotPlanning/LICENSE`，SHA256 `bfd2f9d2b3efd2af6b4517c561b52707ad71807944a4bb3b47ba0519e583cacc` |
| 文件级许可 | 三头文件无独立版权头，根 `LICENSE` 覆盖 |
| 获取方式 | 家族 A 探针 clone（`/tmp/opencode/probes/family_a/libMultiRobotPlanning`）；抽取文件复制入 `thirdparty/libMultiRobotPlanning/` |
| 角色 | 栅格 A*（复用图版模板核心） |
| 依赖 | Boost heap 头（系统，header-only） |

抽取清单：`include/libMultiRobotPlanning/{a_star.hpp,neighbor.hpp,planresult.hpp}` — 与上游逐字节一致（`dfb31fd7…`/`d327450b…`/`0a72a150…`）。多机器人算法（CBS/ECBS 等）不纳入。

适配（全部在适配层，无上游修改）：

1. 栅格 `Environment`：四/八邻域、八邻域禁对角穿角、unknown/地图外阻塞；`admissibleHeuristic` 四邻域 Manhattan、八邻域 octile（均可采纳且一致）。
2. 取消/超时：上游无钩子，在 `onExpandNode` 中检查预算/取消并抛异常中止（探针已验证异常穿模板安全）；适配层区分 `kCancelled`/`kTimeout`。
3. 原生偏差：closed set 不重开节点（正确性绑定一致启发式前提，已登记）；`node_budget` 不被解释（忽略，登记）；展开/发现计数经回调统计。
4. 输出 `PlanResult`（states/actions/cost/fmin）转统一路径与代价；`cost` 为 double（单元步长），统一 `cost_m` 按路径重算。

## 5. S-03 justinhj/astar-algorithm-cpp（MIT）

| 字段 | 值 |
| --- | --- |
| URL | https://github.com/justinhj/astar-algorithm-cpp |
| commit | `875bbd8ba4cdf75463cb30424d0ab4b2a8db9653`（tag v1.4.2，2026-09-30） |
| 许可证 | MIT；`thirdparty/justinhj/license.txt`，SHA256 `417efafe07c1723ee6322cf11604cbce87eed6c4eb66fe15b838dfb0a76ebe91`；`stlastar.h`/`fsa.h` 文件头另有自定义宽松署名声明（保留） |
| 获取方式 | 家族 A 探针 clone（`/tmp/opencode/probes/family_a/astar-algorithm-cpp`）；抽取文件复制入 `thirdparty/justinhj/` |
| 角色 | 栅格 A*；内存池开/关为两个配置（单列） |

抽取清单：

| 文件 | 上游 SHA256 前 16 位 | 本仓 | 修改 |
| --- | --- | --- | --- |
| `include/stlastar.h` | `285249de…` | `e7cefecc…` | 补丁 P1：`#ifndef USE_FSA_MEMORY / #define … 1 / #endif`（允许 `-DUSE_FSA_MEMORY=0` 关闭内存池，默认行为不变） |
| `include/fsa.h` | `60653946…` | SAME | 未修改 |
| `license.txt` | `417efafe…` | SAME | 未修改 |

适配（适配层）与原生偏差：

1. 栅格 `UserState`：四/八邻域、禁对角穿角、unknown/地图外阻塞；`GetSuccessors` 经 `AddSuccessor`（形参为非 const 左值引用，适配层使用具名局部变量）；去重经 `Hash`/`IsSameState`。
2. **内存池容量按规模设置**：`MaxNodes = max(1024, width*height + 64)`（上游默认 1000 对 32² 即不足）；耗尽返回 `SEARCH_STATE_OUT_OF_MEMORY`，适配层报告 `kTimeout`（原因含容量）。
3. **池开/关**：两个翻译单元分别以 `USE_FSA_MEMORY=1/0` 编译（补丁 P1 + CMake 源级宏），并以独立 `UserState` 标签类型避免模板实例 ODR 冲突；两个翻译单元含编译期断言（宏值不符即编译失败）。
4. **float32 精度偏差（登记）**：上游 g/h/f 为 float32；统一 `cost_m` 由适配层按返回路径以 double 重算（满足 §6.2 容差），`stats.upstream_cost` 保留上游 float32 自报值（单元步长）。正确性档实测最大偏差 < 1e-3（测试断言；正式分组门槛由 X-B1 冻结）。
5. 取消与“无路径”上游同为 `SEARCH_STATE_FAILED`（适配层用自身取消/超时标志区分）；`m_CancelRequest` 为非原子 bool（单线程使用；跨线程取消需调用方同步，登记）。
6. 分步式 `SearchStep()` 支持步间预算检查（适配层实现）。

## 6. 未接入与排除面（汇总）

| 来源 | 排除内容 | 原因 |
| --- | --- | --- |
| vss2sn | RRT*/其他算法（ant colony、D* Lite、LPA*、JPS、GA、RRT） | 属 X-S4 或不在本轮范围 |
| Nav2 SmacPlanner2D | 插件层、smoother、costmap_downsampler、utils、obstacle/distance heuristic、真实 costmap/足迹语义、OMPL（仅解析、不链接） | 与 Node2D 搜索核心无关；探针 §2.5/2.6 |
| jps3d | DMP、3D、示例/测试、yaml-cpp 链接 | 本轮范围外（登记 §S-08）；最小编译单元不依赖 yaml-cpp |
| libMultiRobotPlanning | 多机器人算法（CBS/ECBS 等）与示例 | 仅接单机器人搜索（登记 §S-02） |

## 7. 依赖与构建登记（供 wiring 代理）

- **不引入任何 FetchContent 下载**：所有上游源码已抽取进 `thirdparty/`；构建只读系统头（jps3d→Eigen3；libMultiRobot→Boost heap）。jps3d 适配器开关 ON 但未找到 Eigen3 时，CMake 打印警告并跳过该适配器（其余后端照常构建）。
- 组件公共目标：`planner_grid_search`（别名 `planner::grid_search`），依赖 `planner::common`。
- 每适配器开关（默认 ON）：`PLANNER_GRID_SEARCH_WITH_VSS2SN`、`..._WITH_SMAC2D`、`..._WITH_JPS3D`、`..._WITH_LIBMULTIROBOT`、`..._WITH_JUSTINHJ`。
- 后端名称（`available_backends()`）：`jps3d_astar`、`jps3d_jps`、`justinhj_astar`、`justinhj_astar_nopool`、`libmultirobot_astar`、`smac_2d_astar`、`vss2sn_astar`、`vss2sn_dijkstra`。
- `dependencies.lock.json` 建议条目（由 wiring 代理写入）：5 个来源的 URL + commit + 许可证 SPDX 与文件路径 + `used_by: grid_search`；**无源码归档 SHA256**（来源为 GitHub clone 抽取，记录 commit 即可；如需归档哈希由 wiring 阶段决定）。本组件不消费锁条目（构建不下载）。
- 建议根选项：`PLANNER_BUILD_GRID_SEARCH`（默认 OFF，与其他可选组件一致）与 `add_subdirectory(Src/grid_search)`；CTest 标签 `grid_search`；`tools/components.json` 条目建议 `kind: cmake`、`target: planner_grid_search`、`test.runner: ctest`、`test.label: grid_search`。

## 8. 未验证项与遗留风险

1. 性能/规模/基准属 X-B1：本轮未测 128²–1024² 正式档、未测容量档；`stats.expanded_nodes` 对 vss2sn 保持 0（上游不暴露）。
2. 正确性档规模较小（6²–32²），未覆盖 2048²/4096²；距离场内存构成留给 X-B1。
3. jps3d JPS 的 `jump` 规则未做禁穿角改造（保持原生单列）；如 X-B1 要求统一禁穿角 JPS，需另立补丁与登记。
4. Smac 桩与真实 costmap 语义差异（足迹、膨胀、层更新）未覆盖；只证明 Node2D 搜索核心可解耦（与探针一致）。
5. vss2sn 方形栅格填充转换成本未测量（单列项）。
6. `node_budget` 仅 Smac 解释；其他后端忽略（如 X-B1 需要统一节点预算，需扩展适配层）。
7. 单线程假设：所有适配器按单线程调用设计（justinhj 取消标志非原子等）。
8. **`vss2sn_astar` 八邻域次优（X-B1 烟测发现，§1.1）**：正式窗口前须修复并复验
   （非严格弱序 + 闭集不复开）；修复前该后端不参与正式排名，正式档结果只作
   “失败/阻塞”证据；正式套件已按 `--backends` 白名单排除（§1.1，X-B1 修正），
   修复并复验通过后恢复；四邻域与 `vss2sn_dijkstra`（点到点 + 距离场）不受影响。
