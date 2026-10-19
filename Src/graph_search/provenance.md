# planner_graph_search 来源、适配与验证记录（X-S1 图 A* / X-S2 图 Dijkstra）

- 日期：2026-10-12（UTC）。状态：**正确性档通过（本轮 32/32），性能基准属 X-B1 未执行**。
- 范围：`Src/graph_search/**`（库、五适配器、测试、示例、内部 CMake）；不含根接线
  （根选项 / `tools/components.json` / `dependencies.lock.json` 由后续 wiring 代理统一完成）。
- 依据：[组件扩展计划](../../Docs/component_expansion_plan.md) §4.2、§6.2；[候选来源登记](../../Docs/expansion/candidate_registration.md)
  S-01…S-05 与 §6 预筛；[family_a 探针](../../Docs/expansion/probes/family_a_probe.md)、[family_b 探针](../../Docs/expansion/probes/family_b_probe.md)；
  [基准规模分层](../../Docs/expansion/benchmark_tiers.md) §2.1/§2.3（了解）。
- 验证运行入口：`tools/task_runner.py exec --component graph_search`（监控门禁 + 编译锁 +
  §9 封存）；全部编译持全局锁 `flock -w 1800 .git/planner-runtime/build.lock`，`-j4` 上限。
- 逐来源文件级说明：`src/upstream/{lmrp,justinhj,lemon}/PROVENANCE.md`。

## 1. 总表

| 编号 | 适配器 ID | 来源 / 固定版本 | 许可证 | 获取方式 | 上游类型暴露 |
| --- | --- | --- | --- | --- | --- |
| S-01 | `boost_astar`、`boost_dijkstra`、`boost_distance_field` | Boost Graph，系统 Boost 1.71.0；上游 tag `boost-1.71.0` = `8f5e1f1bf91c7302b8448bdc899dc68312e3acd5` | BSL-1.0 | 系统头文件（`find_package(Boost 1.71)`，不下载） | 无（适配层私有） |
| S-02 | `lmrp_astar` | libMultiRobotPlanning `4c75fa20c435c440d8b6bd6dc81668ddc7296ba0`（main HEAD，无 tag） | MIT | vendored（3 头文件 + LICENSE） | 无 |
| S-03 | `justinhj_astar_pool`、`justinhj_astar_nopool` | justinhj/astar-algorithm-cpp tag `v1.4.2` = `875bbd8ba4cdf75463cb30424d0ab4b2a8db9653` | MIT | vendored（2 头文件 + license + 1 行守卫补丁） | 无 |
| S-04 | `lemon_dijkstra`、`lemon_distance_field` | OpenROAD lemon-graph 副本 `31d79d686f776a62ace1925cb2e40e84a80d1394`；基线 LEMON 1.3.1（官方 tarball SHA256 `71b7c725f4c0b4a8ccb92eb87b208701586cf7a96156ebd821ca3ed855bad3c8`） | BSL-1.0（文件级例外见 §5） | vendored（Dijkstra 传递包含集 22 头 + config.h + LICENSE） | 无 |
| S-05 | `igraph_dijkstra`、`igraph_distance_field` | igraph 官方 1.0.1 发布 tar 包（tag `1.0.1` = `7b4ae766cbdee6b2017aa5b76752457db2a2972f`；探针曾用 main HEAD `5e04f85244b5ec93836ec00110737071e5dded47`） | **GPL-2.0-or-later（隔离组）** | FetchContent `URL_HASH`（发布 tar 包含预生成解析器，无需 flex/bison） | 无（C 包装私有） |

## 2. S-01 Boost Graph

- 系统版本证据：`BOOST_VERSION 107100`（`/usr/include/boost/version.hpp`）；dpkg
  `libboost-graph1.71-dev 1.71.0-6ubuntu6`；Debian 版权文件
  `/usr/share/doc/libboost1.71-dev/copyright` SHA256
  `423890420e5ccbbaaf341421bbb426ef9f5b59ff1ed502b4b6580294c3742060`（含 BSL-1.0 全文）。
- 上游 tag/develop：探针记录 `refs/tags/boost-1.71.0` 与 `boost-1.72.0` 同点
  `8f5e1f1b`（历史压缩/标签重指迹象）；系统头与 tag include 树共享文件逐字节一致
  （family_a §2.1）。本组件只以系统头文件使用 BGL，不 FetchContent。
- 适配要点（`src/adapters/boost_adapter.cpp`）：
  1. `adjacency_list<vecS,vecS,directedS,no_property,property<edge_weight_t,double>>`
     + 非负权 property map；图构建/属性映射成本计入 `Stats.preprocess_ms`。
  2. A* 使用调用方启发式（可采纳性由调用方保证；夹具经 `euclidean_admissible` 校验）；
     零启发式等价 Dijkstra 语义（对照）。
  3. 1.71 的 f 值命名参数为 `rank_map`（无 `cost_map`）；`predecessor_map`/`distance_map`/
     `rank_map`/`color_map` 显式提供。
  4. 早停/取消/超时：`examine_vertex` 中抛异常（goal / cancel / budget），异常安全已验证；
     计数器用指针回写（BGL 按值拷贝 visitor）。
  5. 不可达标记 `numeric_limits<double>::max()`（非 inf）统一归一化为 +inf；
     `boost::negative_edge` 映射为 invalid_input（统一校验为第一道防线）。
- 原生偏差：无内建中断（依赖 visitor 异常，已登记）；不可达哨兵归一化。

## 3. S-02 libMultiRobotPlanning

- 固定 commit `4c75fa20c435c440d8b6bd6dc81668ddc7296ba0`（main HEAD，2023-07-28；仓库无 tag）。
- 许可证：MIT（`LICENSE` SHA256 `bfd2f9d2b3efd2af6b4517c561b52707ad71807944a4bb3b47ba0519e583cacc`）。
- 复制文件（逐字节一致）：`include/libMultiRobotPlanning/{a_star.hpp,neighbor.hpp,planresult.hpp}`；
  文件级 SHA256 与复核方式见 `src/upstream/lmrp/PROVENANCE.md`。
- 适配补丁（适配层，非上游改动）：
  1. **取消/超时异常包装**（登记补丁）：上游无中断钩子；`onExpandNode` 中检查取消标志/
     时间预算并抛出 `Aborted` 中止搜索，`PlanResult` 与容器随栈展开析构。
  2. 图 Environment 提供邻居/代价/启发式/解判定；`PlanResult.states/cost` 转统一路径/代价。
- 原生偏差：closed set 不重开节点（`unordered_set`），正确性绑定一致启发式前提；
  展开计数经 `onExpandNode`（`expanded_nodes`），松弛边数不提供（-1）。

## 4. S-03 justinhj/astar-algorithm-cpp

- 固定 tag `v1.4.2` = `875bbd8ba4cdf75463cb30424d0ab4b2a8db9653`（2026-09-30）；master
  `25b05a33` 相对 tag 仅增基准，核心头一致（family_a §4.1）。
- 许可证：MIT（`license.txt` SHA256 `417efafe07c1723ee6322cf11604cbce87eed6c4eb66fe15b838dfb0a76ebe91`）；
  `stlastar.h`/`fsa.h` 文件头另有自定义宽松声明（并列记录）。
- 复制与补丁：`stlastar.h`（上游 SHA256 `285249de…`，本仓库补丁后 `9d89a19f…`）、
  `fsa.h`（逐字节一致）、`license.txt`、`stlastar_pool_guard.patch`。补丁仅把
  `#define USE_FSA_MEMORY 1` 包进 `#ifndef` 守卫（默认行为不变，允许 `-DUSE_FSA_MEMORY=0`）。
- 适配层行为：池开/关两 TU 位于不同命名空间（避免 `AStarSearch<GraphNode>` 实例化 ODR 冲突），
  同一目标 `planner_graph_search_justinhj` 提供两个工厂；池容量 `max(1000, num_vertices+16)`
  记录于 `EffectiveConfig.options.memory_pool_capacity`。
- **float32 精度（登记条件项，不改核心）**：结果元数据 `cost_precision=float32`；正确性档
  原生代价门槛 `1e-5 + 1e-5*|ref|`；最优性以返回路径 double 重算核对 §6.2。实测（run
  `20261012T130021680328Z_graph_search_48fa514e`）：32 条 √2 边链 expected=45.2548339959、
  native=45.2548408508、**Δ=6.855e-06**、§6.2 容差=4.625e-07（**超严格容差**）、登记门槛
  =4.625e-04（通过）。X-B1 长路径须按规模重新冻结门槛。
- 取消/无路径区分（登记条件项）：上游取消后状态与 FAILED 混淆、取消标志非原子；适配层
  自行记录标志、步间检查时钟并区分 `kCancel`/`kTimeout`/`kNoPath`。

## 5. S-04 LEMON（OpenROAD 副本）

- 副本 commit `31d79d686f776a62ace1925cb2e40e84a80d1394`；基线 = 官方 LEMON 1.3.1
  发布版（2465 文件逐字节相同，family_b §2.1）；HEAD 9 文件 C++20 兼容补丁，
  `dijkstra.h` 及依赖无算法性修改。
- 许可证：整体 BSL-1.0（`LICENSE` SHA256 `0057760df95992e06a0bc78f5632a389a03744f6725e497cedd85bc21e39b28a`）；
  文件级例外：`lemon/bits/enable_if.h`（Boost/Indiana，BSL-1.0，**在包含集内**）、
  `lemon/random.h`（Matsumoto BSD-3 式，**不在包含集内**，未复制）；未发现著佐权污染。
- vendored 内容：`dijkstra.h` 传递包含集 22 头 + 生成 `config.h`（复现探针 LP-off 配置；
  一处登记偏差：`LEMON_USE_PTHREAD` 未定义 → `bits/lock.h` 空实现单线程，包含集不使用跨线程状态）。
- 适配层行为：
  1. 点到点与完整距离场分离；点到点用公开 `init()/addSource()/processNextNode()` 手动循环
     （等价 `run(s,t)` 早停语义）插入逐节点取消/预算检查；两种模式结果不混用。
  2. 不可达用 `reached()` 谓词 → +inf（非数值哨兵）。
  3. `ListDigraph + ArcMap<double>`；建图成本计入 `preprocess_ms`。

## 6. S-05 igraph（GPL-2.0-or-later 隔离组）

- 获取方式：官方 1.0.1 发布 tar 包
  `https://github.com/igraph/igraph/releases/download/1.0.1/igraph-1.0.1.tar.gz`
  （5,390,675 B，SHA256 `969f2d7d22f67e788d8638c9a8c96615f50d7819c08978b3ef4a787bb6daa96c`），
  含预生成解析器，**无需 flex/bison**（探针的 git 检出路线需要本地 flex/bison，已由发布
  tar 包替代并登记）。tag `1.0.1` = `7b4ae766cbdee6b2017aa5b76752457db2a2972f`（2025-12-23）；
  探针 commit `5e04f852…`（main HEAD，1.0.1+87）作为历史预筛记录保留。
- 许可证：GPL-2.0-or-later（`COPYING` SHA256 `b499eddebda05a8859e32b820a64577d91f1de2b52efa2a1575a2cb4000bc259`；
  `include/igraph.h` 文件头明示 or-later）。
- **隔离**：`planner_graph_search_igraph` 独立静态目标，igraph 以 PRIVATE 静态链接；
  `planner_graph_search`、其余适配器、示例与默认交付物不链接 igraph。构建隔离不代替许可证
  核查；对照目标结果不进入默认库/在线服务。
- 适配层行为（C 包装）：`igraph_t`/向量/矩阵/列表 RAII；不可达 `IGRAPH_INFINITY` → +inf；
  取消/预算经公开 `igraph_set_interruption_handler`（线程局部）安装处理器，中断返回
  `IGRAPH_INTERRUPTED` 后按标志区分 `kCancel`/`kTimeout`；不可达的原生警告在调用期间
  抑制（不可达是正常结果，登记偏差）；点到点代价按返回路径 double 求和（igraph 不直接
  返回距离）；距离场不提供前驱（`igraph_distances_dijkstra` 无 parents 输出，留空并记录）。
- 构建配置：Release、静态、GLPK/GraphML/Infomap/OpenMP OFF、无测试/示例/文档；
  系统 BLAS/LAPACK/GMP/ARPACK 复用。

## 7. 适配补丁与原生偏差汇总

| 来源 | 补丁/适配 | 原生偏差 |
| --- | --- | --- |
| S-01 | visitor 异常早停/取消/超时；`rank_map`；max() 归一化；visitor 计数指针回写 | 无内建中断；不可达哨兵为 max() |
| S-02 | `onExpandNode` 异常包装（登记补丁） | closed set 不重开；无松弛边统计 |
| S-03 | `USE_FSA_MEMORY` `#ifndef` 守卫（1 行级，vendored 补丁）；适配层取消/超时标志；池容量按规模 | float32 代价；取消与无路径原生混淆；取消标志非原子 |
| S-04 | 公开 API 手动 `processNextNode` 循环；`reached()` 不可达；vendored config.h 单线程偏差 | 无 visitor 钩子；点到点早停后距离场不完整（不混用） |
| S-05 | C 包装 + 中断处理器 + 警告抑制；代价按路径求和；距离场无前驱 | GPL 隔离组；点到点不直接返回距离；无前驱输出 |

## 8. 验证 run 一览（`artifacts/runs/<run_id>/`）

| run_id | 内容 | 结果 |
| --- | --- | --- |
| `20261012T125315760577Z_graph_search_dfdd056c` | 独立 Python 参考（标准库 Dijkstra）交叉核对 | PASS |
| `20261012T125325856623Z_graph_search_f6e690f6` | 首次配置+编译：`detail/control.hpp` 包含路径缺失 | 失败保留（修复：适配器目标加 `src/` 包含目录） |
| `20261012T125422248707Z_graph_search_8dfc4267` | 编译：LEMON 图/映射不可拷贝（返回局部对象） | 失败保留（修复：就地构建 `LemonGraph(graph)`） |
| `20261012T125450070954Z_graph_search_6145094f` | 全适配器配置+编译（standalone，igraph 发布包） | 成功 |
| `20261012T125502802979Z_graph_search_7cadc689` | ctest 首轮 24/32 | 失败保留（修复：validate_graph 清空 reason；早退 wall_ms 语义；igraph 警告抑制） |
| `20261012T125619098329Z_graph_search_bba18e49` | 修复后增量编译 | 成功 |
| `20261012T125643881116Z_graph_search_de06f851` | ctest 32/32 | **PASS** |
| `20261012T125859070717Z_graph_search_70e34276` | 每适配器独立套件（core/boost/lmrp/justinhj/lemon/igraph）+ demo + 全量 32/32 | PASS |
| `20261012T125958567598Z_graph_search_aad72920` | 直接运行 justinhj 测试二进制路径错误（exit 127） | 失败保留（修复：`tests/` 子目录） |
| `20261012T130021680328Z_graph_search_48fa514e` | justinhj float32 实测 Δ + demo 统计复核（boost 计数修复后 expanded=10/relaxed=10） | PASS |
| `20261012T130057481571Z_graph_search_5610649f` | igraph-only 隔离构建 + 19/19 测试 | 内容通过；退出码被 `ls`+pipefail 污染（保留） |
| `20261012T130154334761Z_graph_search_2b6ed550` | 隔离检查复核：无共享 igraph；默认库 0 igraph 符号；隔离库/测试二进制含 igraph 符号 | PASS |
| `20261012T130203970588Z_graph_search_2703c7f9` | 全部适配器开关 OFF：15/15（core+demo）；未下载 igraph；默认库 0 工厂符号 | PASS |
| `20261012T130231921366Z_graph_search_b515d350` | wired 模拟（/tmp 超级工程，模拟根接线）32/32；未触发 standalone 引导 | PASS |
| `20261012T130512944166Z_graph_search_36e03c13` | 最终复核：增量编译 + ctest 32/32（含最后的 include 整理） | PASS |
| `20261012T130607989885Z_graph_search_bee23bf4` | **最终验证**：示例隔离调整（demo 不链接 igraph 目标）后编译 + ctest 32/32；demo `--self-check` PASS；demo 链接行不含 `planner_graph_search_igraph`，真实 igraph 库符号 0（`nm` 中 "igraph" 匹配均为 LEMON 的 `Digraph` 名称） | **PASS（最终）** |
| `20261012T130657608517Z_graph_search_b25d8974` | lean 路径：`PLANNER_BUILD_TESTS=OFF` + `PLANNER_BUILD_EXAMPLES=OFF` + igraph OFF；不获取 GTest、无测试可执行文件、四个非隔离适配器库构建 | PASS |

测试计数：32 项（`ctest -L graph_search`；每适配器独立可执行文件）。失败 run 全部按
§5.4 保留，重试使用新 run ID 且 `manifest.retry_of` 记录来源。

### 8.1 X-B1 基准工具与烟测（2026-10-12 收口登记）

- 基准工具：`benchmarks/`（`graph_gen v1` 确定性生成 6 实例；`planner_graph_search_bench`
  支持 `--suite v1` 正式矩阵、`--generate-only` 实例物化与 SHA256 归档、`--check`
  一致性门禁、`--smoke` 烟测；输出契约与矩阵见 `benchmarks/README.md`）。注册片段
  已并入 `tools/components.json` → `components.graph_search.benchmark_suites`
  （`v1` + `generate`）；正式入口 `benchmark --component graph_search --suite v1`。
- justinhj float32 **长路径登记门槛**（X-B1 冻结，覆盖 §4 正确性档口径；`edges` =
  返回路径边数，1.2e-7 ≈ 2×2⁻²⁴）：
  `tol(ref, edges) = 1e-5 + max(1e-5, 1.2e-7 × edges) × |ref|`；
  质量对比用 double 重算代价（`recomputed_cost`）。
- 烟测 run `20261012T134601555079Z_graph_search_4b4c41ab`（exit 0）：1k 实例
  `--generate-only`/`--check` PASS（road/rand sha256 回读一致）；`--smoke`
  （boost_astar / boost_dijkstra / boost_distance_field）152 行、144 测量、
  0 invalid、0 门禁失败。
- 正式矩阵（6 实例 × 7 主后端 + 4 零启发式对照 + 3 距离场后端；预热 5 + 测量 30、
  点到点 5 s、距离场 30 s；冷/热分列）属 X-B1 独占窗口，不在本轮执行。

## 9. 依赖锁条目建议（供 wiring 代理）

锁格式见 `dependencies.lock.json` `format_description`（CMake 读取
`dependencies.<key>.{tag,commit,source.url,source.sha256}`）。本组件建议条目：

### 9.1 `igraph`（FetchContent，隔离组）

```json
"igraph": {
  "name": "igraph",
  "version": "1.0.1",
  "tag": "1.0.1",
  "commit": "7b4ae766cbdee6b2017aa5b76752457db2a2972f",
  "status": "locked",
  "source": {
    "url": "https://github.com/igraph/igraph/releases/download/1.0.1/igraph-1.0.1.tar.gz",
    "archive": "igraph-1.0.1.tar.gz",
    "extract_prefix": "igraph-1.0.1/",
    "sha256": "969f2d7d22f67e788d8638c9a8c96615f50d7819c08978b3ef4a787bb6daa96c",
    "sha256_evidence": "2026-10-12 从官方 release asset 下载复算；发布 tar 包含预生成解析器（无需 flex/bison），构建证据见 run 20261012T125450070954Z_graph_search_6145094f / 20261012T130057481571Z_graph_search_5610649f",
    "stability_note": "GitHub release asset 不保证长期字节稳定；首次复核不匹配时按实际字节更新并记录"
  },
  "license": {
    "spdx": "GPL-2.0-or-later",
    "path": "igraph-1.0.1/COPYING",
    "sha256": "b499eddebda05a8859e32b820a64577d91f1de2b52efa2a1575a2cb4000bc259",
    "note": "GPL 隔离组：仅独立目标 planner_graph_search_igraph 静态链接；绝不链入默认库/在线服务；构建隔离不代替许可证核查"
  },
  "used_by": ["Src/graph_search（S-05 Dijkstra 点到点+距离场；GPL-2.0-or-later 隔离）"],
  "fetchcontent": {
    "name": "igraph",
    "cmake_target": "igraph",
    "cmake_options": "BUILD_SHARED_LIBS=OFF（目录作用域普通变量）、BUILD_TESTING=OFF、IGRAPH_GLPK_SUPPORT=OFF、IGRAPH_GRAPHML_SUPPORT=OFF、IGRAPH_INFOMAP_SUPPORT=OFF、IGRAPH_OPENMP_SUPPORT=OFF",
    "component_switch": "PLANNER_GRAPH_SEARCH_WITH_IGRAPH（OFF 不下载/不链接）"
  }
}
```

组件在锁条目缺失时以内嵌同值兜底（`CMakeLists.txt` 注释 + 本文件 §6），wiring 写入后
自动优先使用锁条目。

### 9.2 vendored 三来源（不 FetchContent，仅固定来源）

`lmrp`、`justinhj`、`lemon_graph` 建议按既有 `ego_planner` 模式登记（`fetchcontent`
字段说明不 FetchContent）。归档 SHA256（2026-10-12 下载复算，GitHub 自动归档稳定性
同 ego_planner 注意事项）：

| key | 版本 | commit | 归档 URL / SHA256 | 许可证 |
| --- | --- | --- | --- | --- |
| `lmrp` | main@4c75fa20 | `4c75fa20c435c440d8b6bd6dc81668ddc7296ba0` | `https://github.com/whoenig/libMultiRobotPlanning/archive/4c75fa20….tar.gz` / `081fcba3d37cd221350546f02947a8a5cb18099a5d318f89ca7a036d7ab640a0` | MIT |
| `justinhj` | v1.4.2@875bbd8b | `875bbd8ba4cdf75463cb30424d0ab4b2a8db9653` | `https://github.com/justinhj/astar-algorithm-cpp/archive/875bbd8b….tar.gz` / `54df0eeea1aa921b3bf84a5040d4716e4a1a080b912c7391844b0b173177cb05` | MIT |
| `lemon_graph` | 副本 31d79d68（基线 1.3.1） | `31d79d686f776a62ace1925cb2e40e84a80d1394` | `https://github.com/The-OpenROAD-Project/lemon-graph/archive/31d79d68….tar.gz` / `7c1624db8c1e4078ee25aba9cbacfa6f6abfa5d855602ba5178a3826646008d5` | BSL-1.0 |

文件级 SHA256 与 vendored 清单见 `src/upstream/*/PROVENANCE.md`。

### 9.3 Boost（系统依赖，不新增下载条目）

建议在来源记录/锁说明中登记：系统 Boost 1.71.0（dpkg `1.71.0-6ubuntu6`；上游 tag
`boost-1.71.0` = `8f5e1f1b…`），`find_package(Boost 1.71 REQUIRED)` 复用系统头文件；
版本漂移复核方式见 §2。

## 10. 复核方式（摘要）

```sh
# 独立配置 + 构建 + 测试（监控门禁与编译锁见 README §4）
cmake -S Src/graph_search -B build/graph_search-standalone
cmake --build build/graph_search-standalone --parallel 4
ctest --test-dir build/graph_search-standalone -L graph_search --output-on-failure

# 独立 Python 参考
python3 Src/graph_search/tests/reference/reference.py --json

# 来源复核：各上游 commit/许可证 SHA256 见 §2–§6 与 src/upstream/*/PROVENANCE.md
# 隔离复核：见 run 20261012T130154334761Z_graph_search_2b6ed550 的 nm/ldd 输出
```
