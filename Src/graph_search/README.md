# planner_graph_search（X-S1 图 A* / X-S2 图 Dijkstra）

图搜索统一接口与多实现适配组件（`planner::graph_search`，依赖 `planner::common`）。
本轮交付**正确性**（合成夹具、手算期望、独立 Python 参考核对）；大图性能的基准工具
已就绪（`benchmarks/`：`graph_gen v1` + `planner_graph_search_bench`，用法见
[benchmarks/README.md](benchmarks/README.md)），正式矩阵属 X-B1 独占窗口。
本组件不依赖 Lanelet2、地图服务或显示。

## 1. 目录结构

```
Src/graph_search/
  CMakeLists.txt                独立可配置构建（公共目标 planner_graph_search）
  README.md                     本文件
  provenance.md                 五来源固定信息、适配补丁、原生偏差与验证 run ID
  include/planner/graph_search/
    types.hpp                   统一契约：输入/输出/状态/配置/统计/校验辅助/JSON
    adapters.hpp                适配器工厂声明（按开关以编译定义启用）
  src/
    types.cpp                   契约校验、启发式、代价核对、JSON 导出
    detail/control.hpp          适配器内部：取消/预算检查与计时
    adapters/                   S-01..S-05 适配器实现
    upstream/                   来源说明
      lmrp/                     S-02 vendored 3 头文件 + LICENSE + PROVENANCE.md
      justinhj/                 S-03 vendored stlastar.h/fsa.h + 补丁 + PROVENANCE.md
      lemon/                    S-04 vendored Dijkstra include 集 + PROVENANCE.md
  examples/graph_search_demo.cpp 独立示例（--self-check 注册为 CTest）
  benchmarks/                    X-B1 基准工具（graph_gen v1 + 运行器 + 注册片段）
  tests/
    fixtures.hpp/.cpp           手算夹具 + 共享套件（公共/距离场）
    test_core.cpp               核心契约测试
    test_boost.cpp  test_lmrp.cpp  test_justinhj.cpp
    test_lemon.cpp  test_igraph.cpp
    reference/reference.py      独立 Python 参考（标准库 Dijkstra）
```

## 2. 统一契约（组件扩展计划 §4.2）

- **输入**：`Graph{num_vertices, edges[{from,to,weight}]}`（有向；无向以对称边表示）、
  起点/终点、`Heuristic`（无可靠下界用 `zero_heuristic()`；可采纳性为调用方责任）、
  `Control{budget_s, cancel}`。
- **输出**：`Status`（success / no_path / timeout / cancel / invalid_input）、
  路径（点到点）或距离场（+inf 不可达）、代价、`EffectiveConfig`（精度口径、
  取消/预算能力、适配器选项）与 `Stats`（展开节点、松弛边、预处理/搜索/墙钟）。
- **校验**：负权、越界端点、非有限权值、非法起终点统一返回 `invalid_input`
  （适配器共享 `validate_graph`；Boost 的 `negative_edge` 为二次防线）。
- **比较口径**：代价容差 `1e-8 + 1e-8*|ref|`（`cost_close`）；justinhj 原生
  float32 代价使用登记门槛（见 §5 与 provenance.md）。
- **JSON 导出**：`to_json(Result)` / `to_json(DistanceFieldResult)`，不可达距离为
  `null`，未测量统计为 `null`。

## 3. 适配器

| 编号 | 适配器 ID | CMake 开关（默认 ON） | 目标 | 上游/获取 | 许可证 | 精度 |
| --- | --- | --- | --- | --- | --- | --- |
| S-01 | `boost_astar` / `boost_dijkstra` / `boost_distance_field` | `PLANNER_GRAPH_SEARCH_WITH_BOOST` | `planner_graph_search_boost` | 系统 Boost 1.71.0（header-only） | BSL-1.0 | float64 |
| S-02 | `lmrp_astar` | `PLANNER_GRAPH_SEARCH_WITH_LMRP` | `planner_graph_search_lmrp` | libMultiRobotPlanning `4c75fa20`（vendored 3 头文件） | MIT | float64 |
| S-03 | `justinhj_astar_pool` / `justinhj_astar_nopool` | `PLANNER_GRAPH_SEARCH_WITH_JUSTINHJ` | `planner_graph_search_justinhj` | justinhj v1.4.2 `875bbd8b`（vendored + 1 行池守卫补丁） | MIT | **float32** |
| S-04 | `lemon_dijkstra` / `lemon_distance_field` | `PLANNER_GRAPH_SEARCH_WITH_LEMON` | `planner_graph_search_lemon` | OpenROAD LEMON 副本 `31d79d68`（vendored include 集；基线 1.3.1） | BSL-1.0 | float64 |
| S-05 | `igraph_dijkstra` / `igraph_distance_field` | `PLANNER_GRAPH_SEARCH_WITH_IGRAPH` | `planner_graph_search_igraph`（**隔离**） | igraph 1.0.1 官方发布 tar 包（`969f2d7d…`） | **GPL-2.0-or-later** | float64 |

公共目标：`planner_graph_search`（alias `planner::graph_search`），只含统一接口与
校验/导出，不含任何上游算法。适配器工厂声明见 `adapters.hpp`（按链接目标提供的
`PLANNER_GRAPH_SEARCH_HAS_*` 编译定义启用）。

**GPL 隔离（S-05）**：`planner_graph_search_igraph` 是独立静态目标，igraph 以
PRIVATE 依赖静态链接；`planner_graph_search`、其余适配器目标、示例与默认交付物
均不链接 igraph。构建隔离不代替许可证核查（见 `provenance.md` 与
`src/upstream/` 来源说明）。

## 4. 构建与测试

独立配置（standalone；自动获取锁定依赖与 igraph）：

```sh
cmake -S Src/graph_search -B build/graph_search-standalone
cmake --build build/graph_search-standalone --parallel 4
ctest --test-dir build/graph_search-standalone -L graph_search --output-on-failure
```

经统一入口（构建/测试必须持全局编译锁与监控门禁）：

```sh
# 配置 + 编译（编译命令在命令内持全局编译锁，-j4 上限）
python3 tools/task_runner.py exec --component graph_search --workers 1 -- bash -c \
  'set -o pipefail; { flock -w 1800 {repo_root}/.git/planner-runtime/build.lock \
     cmake -S {repo_root}/Src/graph_search -B {build_dir}/graph_search \
       -DCMAKE_BUILD_TYPE=Release && \
     flock -w 1800 {repo_root}/.git/planner-runtime/build.lock \
     cmake --build {build_dir}/graph_search --parallel 4; } 2>&1 | tee {run_dir}/outputs/build.log'

# 测试
python3 tools/task_runner.py exec --component graph_search --workers 1 -- bash -c \
  'set -o pipefail; ctest --test-dir {build_dir}/graph_search -L graph_search \
     --output-on-failure --parallel 1 2>&1 | tee {run_dir}/outputs/ctest.log'
```

测试可执行文件按适配器拆分（`planner_graph_search_tests_core/_boost/_lmrp/
_justinhj/_lemon/_igraph`），未启用的适配器不构建对应测试；全部注册标签
`graph_search`。`tests/reference/reference.py` 为独立参考（仅标准库）：

```sh
python3 Src/graph_search/tests/reference/reference.py --json
```

## 5. justinhj float32 精度条件项（登记）

上游 g/h/f 与代价均为 `float32`；探针 Δ≈2.8e-6（栅格 8 邻域参考 42.183766）
超 §6.2 容差约 6.5×。本组处理（不改上游核心）：

- 结果元数据标注 `cost_precision=float32`，`EffectiveConfig` 记录该口径；
- 正确性档对原生代价使用**已登记门槛** `1e-5 + 1e-5*|ref|`（覆盖正确性夹具
  路径 ≤64 边的 float32 逐项舍入与累加，约 `2n·2^-24·ref`）；
- 最优性核对以**返回路径的 double 重算代价**为准（§6.2 容差）；
- X-B1 正式比较若使用更长路径，须按规模冻结更宽的门槛并登记，不得直接沿用
  §6.2 严格容差。

## 6. 本轮范围与后续

- 正确性测试 32 项（含每适配器公共套件、距离场、取消/超时、负权拒绝、
  等代价、零权边、起终点相同、断连、无效输入）。
- 取消/超时：全部适配器可区分 `cancel`/`timeout`/`no_path`；S-02 为适配层异常
  包装补丁、S-03 为适配层标志（上游状态混淆）、S-05 经 igraph 中断处理器。
- 基准工具已就绪（`benchmarks/`；套件已并入 `tools/components.json` →
  `components.graph_search.benchmark_suites`（`v1` + `generate`））：
  `--generate-only` 物化 graph_gen v1 六实例（>10 MiB SHA256 寻址归档）、
  `--suite v1` 正式矩阵、`--smoke` 烟测；用法、矩阵与 justinhj float32
  登记门槛见 [benchmarks/README.md](benchmarks/README.md)（烟测 run
  `20261012T134601555079Z_graph_search_4b4c41ab`）。
- 正式矩阵须在 X-B1 独占窗口运行（`benchmark --component graph_search --suite v1`）；
  规模分层、预算与矩阵见 `Docs/expansion/benchmark_tiers.md` §2.1/§2.3。
