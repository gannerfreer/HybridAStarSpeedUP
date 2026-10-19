# X-S3（栅格 A* 多实现接入）+ X-S2 栅格部分（栅格 Dijkstra/距离场）报告

- 日期：2026-10-12。执行代理：X-S3/X-S2 栅格部分（独占新建 `Src/grid_search/**`，未 git 提交）。
- 范围：统一栅格契约库 + 5 个来源适配器（8 个后端）+ 正确性档测试 + 来源记录。
  **性能/规模/基准属 X-B1**；本轮只做正确性（无正式基准、无容量档）。
- 状态：**开发完成；正确性档通过**（8/8 CTest，29/29 gtest 用例）；接线（根 CMake /
  `tools/components.json` / `dependencies.lock.json`）留给后续单一 wiring 代理。
- 关键 run ID：`20261012T130809893984Z_grid_search_236726db`（最终构建 + 测试，8/8 通过）；
  全新目录独立配置 `20261012T130415977092Z_grid_search_043254e4`（8/8）；
  开关验证 `20261012T130641123485Z_grid_search_a9650d49`。失败与迭代 run 全部保留
  （见 provenance.md §0）。

## 1. 库结构

```
Src/grid_search/
├── CMakeLists.txt              # 公共目标 planner_grid_search；每适配器开关（默认 ON）；
│                               #   支持独立配置（cmake -S Src/grid_search -B <dir>）
├── README.md / provenance.md / REPORT.md
├── include/planner/grid_search/
│   ├── types.hpp               # 请求/结果/统计/状态区分/路径与代价工具
│   ├── neighborhood.hpp        # 四/八邻域、禁对角穿角、unknown/地图外阻塞
│   ├── distance_field.hpp      # 栅格对齐距离场（米、不可达 +infinity）
│   ├── backend.hpp             # GridSearchBackend 接口 + 注册表
│   └── adapters/adapters.hpp   # 适配器工厂（Smac2dOptions 单列配置）
├── src/                        # 契约实现 + 注册表 + 5 个适配器（+ 内部工具）
├── thirdparty/                 # 抽取上游源码（许可证头保留）+ 适配桩 + 说明
├── examples/grid_search_demo.cpp  # 独立示例（--self-check，注册为 CTest）
├── scripts/standalone_{build,test}.sh  # 开发期入口（flock + -j4）
└── tests/                      # 29 个 gtest 用例 + Python 参考 + 冻结夹具
```

契约要点：只读 `GridView`、单元起终点、四/八邻域分组、八邻域禁对角穿角、unknown
与地图外阻塞；输出状态区分 `success/no_path/timeout/cancelled/invalid_input`
（违约路径 `validation_failed`）；统一 `cost_m` 一律按返回路径以 double 重算；
距离场与栅格逐单元对齐、不可达 `+infinity`。

## 2. 适配器状态与补丁清单

| 后端 | 状态 | 四/八邻域 | 关键补丁/偏差 |
| --- | --- | --- | --- |
| `vss2sn_dijkstra` | 通过 | ✔ / ✔（补丁） | 距离场缺口 → 补丁 `PlanDistanceField`；八邻域 → P1/P3/P4；取消钩子 P2 |
| `vss2sn_astar` | 通过 | ✔ / ✔（补丁） | 八邻域 → P1/P3/P4（含 octile 启发式） |
| `smac_2d_astar` | 通过 | ✔（补丁 P4）/ ✔ | P2 禁穿角；适配层 `allow_unknown=false`+`tolerance=0`；OMPL 以 Node2D 适配桩去除 |
| `jps3d_astar` | 通过 | ✔（补丁 P4）/ ✔（补丁 P1） | P1 禁穿角、P2 unknown 预转换、P3 `eps=1`；附加终止钩子 |
| `jps3d_jps` | 通过（单列） | —（原生仅八邻域） | 保持原生允许穿角；跳点路径展开为单位步；不混排名 |
| `libmultirobot_astar` | 通过 | ✔ / ✔ | 栅格 Environment 在适配层；取消经 `onExpandNode` 异常包装 |
| `justinhj_astar` | 通过 | ✔ / ✔ | 栅格 UserState；内存池开（容量 `max(1024, w*h+64)`） |
| `justinhj_astar_nopool` | 通过（单列） | ✔ / ✔ | 补丁 P1 `USE_FSA_MEMORY` 守卫 + 独立标签类型；编译期断言 |

**Smac 两补丁（重点）**：① `node_2d.cpp::getNeighbors` 禁对角穿角（对角移动对两个
正交邻格走同一 `NeighborGetter/isNodeValid` 索引路径，任一不可通行即跳过）；② 适配层
`initialize(allow_unknown=false, terminal_checking_interval=1)` 与
`createPath(tolerance=0)`（unknown 按阻塞、精确最短路，关闭最接近路径回退）。另有
补丁 P1（`<cmath>`，探针保留）、P4（`SearchInfo::use_four_neighborhood` + 四邻域偏移）；
`analytic_expansion.hpp` 以 Node2D 专用适配桩替换（上游该分支恒返回 nullptr，桩去除
OMPL 头依赖，构建不联网）。Apache-2.0 头与 `LICENSE` 随目录保留，commit 锁定
`3286119753ce1fab9d2ff411ea6d439b823fe18d`。

**vss2sn 两缺口（重点）**：① Dijkstra 无距离场 API → 适配补丁 `Dijkstra::PlanDistanceField`
（与 `Plan` 相同的运动原语/禁穿角/代价语义，完整展开；取消命中返回部分场并由适配层丢弃）；
② A* 仅四邻域 → 补丁 P1（`GetMotion(bool)` 八邻域）+ P4（octile 启发式）+ P3（禁穿角）。

**jps3d 三偏差（重点）**：① 八邻域允许对角穿角 → 补丁 P1（`getSucc` 正交邻格检查，
普通 A* 生效；JPS 保持原生单列）；② unknown(-1) 被搜索视为 free → 适配层预转换
unknown/地图外为 occupied（零源码改动）；③ 启发式权重 `eps` 可 ≠1 → 固定 `eps=1`
（可采纳）。另有 P4 四邻域、终止钩子、`floatToInt` 单元中心约定绕开、统一输出用
`getRawPath()`（不用上游后处理路径）、JPS 跳点路径展开为单位步。

## 3. 测试数与通过数

| 项 | 数量 | 结果 |
| --- | --- | --- |
| CTest 条目（标签 `grid_search`） | 8 | **8/8 通过** |
| gtest 用例 | 29 | **29/29 通过**（邻域 10、代价×参考 3、距离场 6、状态与预算 6、配置与单列 4） |
| 示例自检（`planner_grid_search_demo --self-check`） | 1 | 通过 |
| Python 夹具复现（scenarios/expected） | 2 | 通过 |
| 后端×场景×邻域代价核对（统一组可达 112 + JPS 单列可达 8） | 120 | 全部与独立 Python 参考一致（容差 `1e-8+1e-8·|ref|`） |
| 不可达核对（no_path 且空路径） | 30 | 全部一致 |
| 距离场逐单元比较 | 20 个场、≈5.9k 个可达单元（+ 阻塞/不可达标记） | 全部一致 |
| 后端两两一致（统一组，同场景/邻域） | 420 对 | 全部一致 |
| 非法输入矩阵 | 96（12 类×8 后端） | 全部 `invalid_input` |
| 取消/超时/起终点相同/节点预算 | 各 8 | 状态区分正确 |
| 夹具穿角区分性（八邻域 > 允许穿角） | 4 个场景断言（`corridor_32` 亦成立） | 成立 |
| 适配器独立开关（关闭 justinhj/jps3d/smac） | 1 次配置+构建+示例自检（run `20261012T130641123485Z_grid_search_a9650d49`） | 通过（仅剩 3 个后端） |

关键期望值（独立 Python 参考，米）：`open_16` 四 11.0 / 八 7.778175；`corner_gap_6`
八 6.0（允许穿角 1.414214，全部后端拒绝穿角）；`maze_25` 八 59.556349（允许穿角
51.941125）；`border_corner_16` 八 9.156854；`corridor_32` 四 21.0 / 八 18.217514；
`deadend_24` 八 21.828427；`unreachable_20` 与 `unknown_barrier_16` 全部后端
`no_path`。`jps3d_jps` 与“允许穿角”参考一致（单列），四邻域对照组返回
`invalid_input`（原生仅八邻域）。

上游精度差异登记：justinhj 内部 float32；统一 `cost_m` 由适配层按路径以 double 重算
（满足 §6.2 容差），`stats.upstream_cost` 保留 float32 自报值；实测最大偏差约
`7e-6 m`，小于登记门槛 `1e-3`（正式分组门槛由 X-B1 冻结）。

## 4. provenance 摘要

5 个来源全部抽取进 `thirdparty/`（构建不联网、不 FetchContent）：

| 来源 | commit | SPDX | 抽取/修改 | 验证 |
| --- | --- | --- | --- | --- |
| vss2sn/path_planning | `d8f9eda` | BSD-3-Clause | 7 文件；P1–P4 | run `…236726db`（及 §0 全部） |
| Nav2 SmacPlanner2D | `3286119…`（1.5.0） | Apache-2.0（包级） | 8 上游文件 + 1 适配桩 + 7 桩；P1/P2/P4 | 同上 |
| jps3d | `ef65e36` | BSD-3-Clause | 7 文件；P1–P4 + 终止钩子 | 同上 |
| libMultiRobotPlanning | `4c75fa2` | MIT | 3 头文件（逐字节一致） | 同上 |
| justinhj/astar-algorithm-cpp | `875bbd8`（v1.4.2） | MIT | 2 头文件；P1（宏守卫） | 同上 |

未修改文件与上游 SHA256 一致（含 Smac 桩与探针 `nav2_stubs_sha256.txt` 一致、抽取
文件与 `nav2_extract_sha256.txt` 一致）；全部适配修改以 `[grid_search 适配补丁 …]`
就地标注；许可证全文随目录保留。完整表格见 [provenance.md](provenance.md)。

## 5. 资源与证据

- 构建/测试全部经 `python3 tools/task_runner.py exec --component grid_search --workers 1 -- <命令>`；
  编译在命令内持全局编译锁 `flock .git/planner-runtime/build.lock`，`--parallel 4`；
  监控门禁生效（心跳正常，未触发 90% 阈值，无资源停止）。
- 每次任务封存于 `artifacts/runs/<run_id>/`；失败与迭代 run 保留，未修改历史证据。
- 最终构建 + 测试：`20261012T130809893984Z_grid_search_236726db`；
  全新目录独立配置：`20261012T130415977092Z_grid_search_043254e4`；
  适配器开关验证：`20261012T130641123485Z_grid_search_a9650d49`。

### 自检

- 本代理改动仅限 `Src/grid_search/**`（78 个新文件）；未 git 提交；未修改根
  CMake、`tools/**`、`dependencies.lock.json`、其他 `Src/**` 与 `Docs/**`。
- 工作树中 `Src/map`、`Src/scene`、`Src/service`、`Src/graph_search/`、
  `Src/sampling_search/` 的改动与新增来自并行执行的其他代理（X-M1/X-S1/X-S4 等），
  与本组件无关、未被本代理触碰。

## 6. 遗留风险

1. 性能/规模/容量档未测（X-B1）：128²–1024²、2048²/4096²、展开节点/内存/耗时；vss2sn
   不暴露展开计数（`stats.expanded_nodes=0`）。
2. jps3d JPS 的 `jump` 规则未做禁穿角改造（保持原生单列）；如需统一禁穿角 JPS 需另立补丁。
3. Smac 桩与真实 costmap 语义差异（足迹、膨胀、层更新）未覆盖；只证明 Node2D 搜索核心解耦。
4. `node_budget` 仅 Smac 解释；其他后端忽略（如需统一节点预算需扩展适配层）。
5. 单线程假设（justinhj 取消标志非原子等）；跨线程取消需调用方同步。
6. vss2sn 非方形输入填充转换成本未测量；`node_budget`/方形填充等偏差已在 provenance 登记。
7. 接线前组件未进入根构建与注册表：本轮不改根 CMake / `components.json` /
   `dependencies.lock.json`（由 wiring 代理统一完成；建议条目见 provenance.md §7）。

## 7. 收口补充（2026-10-12）：X-B1 工具就绪与套件注册

- 本组件的 X-B1 基准工具已交付：`benchmarks/`（`grid_scenario_gen v1`、独立
  Python 参考、`planner_grid_search_bench`、`run_suite.sh`），套件 `v1`/`capacity`
  已由收口代理并入 `tools/components.json`（入口 `benchmark --component grid_search
  --suite v1|capacity`）；正式矩阵与容量档属 X-B1 **独占窗口**，未在本轮执行。
- 烟测证据：主烟测 `20261012T135344281135Z_grid_search_7653791b`（1080 测量
  trial、0 超时、0 不匹配）；容量最小烟测 `20261012T135648813119Z_grid_search_bf545f25`；
  发现烟测 `20261012T135407821840Z_grid_search_93a6a3c4`（exit 1，见下）。
- **条件项（`vss2sn_astar` 八邻域次优）**：八邻域在 `maze`/`border_corner` 返回
  合法但次优路径（maze 38.736248 m vs 独立参考 35.631580 m），60/360 测量不匹配；
  **修复前不参与正式排名**。登记与证据：[provenance.md](provenance.md) §1.1 与
  [benchmarks/README.md](benchmarks/README.md) §5。
- 接线状态：根 CMake、`dependencies.lock.json`（5 来源 + 共用条目）与注册表条目
  已随收口批完成（本报告 §6 第 7 条所述边界不再适用）。
