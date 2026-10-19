# S-02 libMultiRobotPlanning 来源与许可证说明（vendored）

> 本目录是 `planner_graph_search` S-02 适配器（图 A*）所复制的上游头文件。
> 只包含单机器人 A* 模板所需的最小文件集；多机器人算法（CBS/ECBS 等）、
> 示例与仓库 CMake 不接入、不入依赖锁。

## 1. 上游固定信息

| 项 | 值 |
| --- | --- |
| 上游仓库 | <https://github.com/whoenig/libMultiRobotPlanning> |
| 分支 | `main`（仓库无 tag，0 个 refs/tags） |
| 固定 commit | `4c75fa20c435c440d8b6bd6dc81668ddc7296ba0`（2023-07-28，main HEAD，2026-10-12 核实） |
| 许可证 | MIT（`LICENSE`，SHA256 `bfd2f9d2b3efd2af6b4517c561b52707ad71807944a4bb3b47ba0519e583cacc`） |
| 获取方式 | **vendored**（逐文件复制，随组件入 Git；不下载） |

## 2. 复制文件清单（逐字节一致，未修改）

| 本仓库文件 | 上游路径 | SHA256（本仓库副本） |
| --- | --- | --- |
| `include/libMultiRobotPlanning/a_star.hpp` | `include/libMultiRobotPlanning/a_star.hpp` | `dfb31fd79273ffd32f3c1ac6c19847f319ce352e1511c5e8f196c9849d38339f` |
| `include/libMultiRobotPlanning/neighbor.hpp` | `include/libMultiRobotPlanning/neighbor.hpp` | `d327450bf641410753ad11563811d177806bd0bc6c003b6013d1c3b8f936aecc` |
| `include/libMultiRobotPlanning/planresult.hpp` | `include/libMultiRobotPlanning/planresult.hpp` | `0a72a1504b97b77d09e7a90c2eda13444ef33a25880d07873cda6b70272f1bff` |
| `LICENSE` | `LICENSE` | `bfd2f9d2b3efd2af6b4517c561b52707ad71807944a4bb3b47ba0519e583cacc` |

三个头文件无独立版权头，由根 `LICENSE`（MIT，Copyright (c) 2017 whoenig）覆盖。

未复制的上游文件：`include/libMultiRobotPlanning/` 下 CBS/ECBS/PBS 等多机器人算法、
`src/`、`test/`、`example/`、`CMakeLists.txt`（示例依赖 Boost program_options 与
yaml-cpp，本组件不引入）。

## 3. 适配补丁与原生偏差（S-02，X-S1）

上游 `a_star.hpp` **未修改**；以下适配在 `src/adapters/lmrp_adapter.cpp` 内实现，
属适配层行为：

1. **取消/超时（适配补丁，登记于候选登记 §6.3）**：上游搜索循环
   `while (!openSet.empty())` 无外部中断钩子。适配层在 `onExpandNode` 回调中检查
   取消标志与时间预算，超限时抛出适配层异常中止搜索；`astar.search()` 的
   `PlanResult` 与开放集容器随栈展开析构，无泄漏。探针已验证异常可安全穿过模板
   （family_a §3.2）。统一结果据此区分 `kCancel` / `kTimeout` / `kNoPath`。
2. **零启发式**：`admissibleHeuristic` 返回调用方提供的统一启发式；无可靠下界时
   返回 0（语义等价 Dijkstra）。
3. **原生偏差记录**：closed set 不重开节点（`unordered_set`），正确性绑定
   “一致启发式”前提；`PlanResult` 的 `states/actions/cost` 转统一路径与代价。
4. **图构建**：适配层按统一输入构建邻接索引（不计入上游代码）。

## 4. 复核方式

```sh
git clone https://github.com/whoenig/libMultiRobotPlanning /tmp/lmrp
cd /tmp/lmrp && git checkout 4c75fa20c435c440d8b6bd6dc81668ddc7296ba0
sha256sum LICENSE include/libMultiRobotPlanning/{a_star,neighbor,planresult}.hpp
# 与本文件 §2 的 SHA256 对照（a_star/neighbor/planresult 应为上述值）
```
