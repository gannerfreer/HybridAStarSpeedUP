# planner_grid_search（X-S3 栅格 A* + X-S2 栅格 Dijkstra 部分）

统一栅格搜索契约与多来源适配。只依赖 `planner::common`（`GridView` 类型按冻结
补充暂留 common），不暴露 Lanelet2 或上游类型，不依赖地图服务与显示。

- 来源记录（URL/commit/许可证/补丁/偏差/run ID）：[provenance.md](provenance.md)
- 中文验收报告（本轮正确性档）：[REPORT.md](REPORT.md)
- 上游抽取与适配桩：`thirdparty/`（[说明](thirdparty/README.md)）

## 统一契约（实现于 `include/planner/grid_search/`）

- 输入：只读 `common::GridView`、单元起终点（row/col）、邻域策略
  （`Neighborhood::kFour` / `kEight`）、可选时间/节点预算与取消标志；
- 四/八邻域分组；八邻域禁止对角穿角；未知与地图外按阻塞；
- 输出：`GridSearchResult`（状态 / 路径 / 代价 / 统计），状态区分
  `success`、`no_path`、`timeout`、`cancelled`、`invalid_input`
  （违约路径为 `validation_failed`）；
- 距离场：`DistanceField`（与栅格对齐、米制、不可达 `+infinity`）；
- 代价口径：统一 `cost_m` 一律由适配层按返回路径以 double 重算
  （上游自报值放 `stats.upstream_cost`，可能为 float32）。

## 后端（`available_backends()`）

| 名称 | 角色 | 四邻域 | 八邻域 | 距离场 | 备注 |
| --- | --- | --- | --- | --- | --- |
| `vss2sn_dijkstra` | 栅格 Dijkstra | ✔ | ✔（补丁 P1/P3/P4） | ✔（补丁 P3） | 点到点 + 距离场 |
| `vss2sn_astar` | 栅格 A* | ✔ | ✔（补丁 P1/P3/P4） | — | 八邻域次优发现（修复前不参与正式排名，见基准与 provenance §1.1） |
| `smac_2d_astar` | 栅格 A* | ✔（补丁 P4） | ✔ | — | 代价感知模式经 `Smac2dOptions` 单列 |
| `jps3d_astar` | 栅格 A* | ✔（补丁 P4） | ✔（补丁 P1） | — | unknown 预转换、`eps=1` |
| `jps3d_jps` | JPS 单列变体 | —（原生仅八邻域） | ✔（原生穿角语义） | — | 不参与统一排名 |
| `libmultirobot_astar` | 栅格 A* | ✔ | ✔ | — | 复用图版模板核心；取消经异常包装 |
| `justinhj_astar` | 栅格 A* | ✔ | ✔ | — | 内存池开（容量按规模） |
| `justinhj_astar_nopool` | 栅格 A* | ✔ | ✔ | — | 内存池关（单列配置） |

## 构建

接线后（推荐，由 wiring 代理完成）：

```sh
python3 tools/task_runner.py build --component grid_search --profile release
python3 tools/task_runner.py test  --component grid_search --workers 1
```

当前（未接线）独立构建/测试（开发期入口，编译持全局锁、`-j4`）：

```sh
python3 tools/task_runner.py exec --component grid_search --workers 1 -- \
  bash -lc 'Src/grid_search/scripts/standalone_build.sh <build-dir> \
    && Src/grid_search/scripts/standalone_test.sh <build-dir> 1'
```

每适配器开关（默认 ON）：`PLANNER_GRID_SEARCH_WITH_VSS2SN`、`_WITH_SMAC2D`、
`_WITH_JPS3D`、`_WITH_LIBMULTIROBOT`、`_WITH_JUSTINHJ`。关闭时对应源码不编译、
第三方头不要求、注册表不返回该后端。jps3d 需要系统 Eigen3 头（缺失时警告并
跳过该适配器）；libMultiRobot 需要系统 Boost heap 头。**构建不联网、不下载。**

## 测试

`tests/`（CTest 标签 `grid_search`）：

- `test_neighborhood`：邻域策略、禁穿角、阻塞/边界、路径校验与代价；
- `test_cost_vs_reference`：全部后端 × 10 场景 × 四/八邻域与独立 Python 参考
  核对（容差 `1e-8 + 1e-8·|ref|`）；后端两两一致；夹具穿角区分性；
- `test_distance_field`：逐单元参考比较、源点为零、不可达标记、局部距离关系、
  与点到点一致；
- `test_status_and_budget`：非法输入矩阵、阻塞起终点、取消、超时、节点预算、
  起终点相同、不可达；
- `test_configs_and_single_list`：justinhj 池开/关一致与 float32 偏差登记、
  JPS 单列原生穿角语义、Smac 代价感知模式单列；
- `reference/grid_reference.py` + `fixtures/`：独立 Python 参考与冻结夹具，
  CTest 中复现校验（`*_fixture_check`）。

## 基准（X-B1 工具就绪）

- 基准工具 `benchmarks/`：`grid_scenario_gen v1` 确定性生成（6 类 ×
  128²/512²/1024² × 种子 0–4 = 90 实例；容量档 2048²/4096²）、独立 Python
  参考（逐单元距离场）、`planner_grid_search_bench` 运行器与 `run_suite.sh`
  编排；用法、指标列与证据见 [benchmarks/README.md](benchmarks/README.md)。
- 套件已注册（`tools/components.json` → `components.grid_search.benchmark_suites`）：
  `benchmark --component grid_search --suite v1`（正式）、`--suite capacity`（容量），
  正式运行属 X-B1 独占窗口。
- 烟测：主烟测 `20261012T135344281135Z_grid_search_7653791b`（1080 测量 trial、
  0 超时、0 不匹配）；容量最小烟测 `20261012T135648813119Z_grid_search_bf545f25`。
- **条件项**：`vss2sn_astar` 八邻域次优发现（`maze` 38.736248 m vs 参考
  35.631580 m；run `20261012T135407821840Z_grid_search_93a6a3c4`）——修复前
  不参与正式排名；登记见 [provenance.md](provenance.md) §1.1。
