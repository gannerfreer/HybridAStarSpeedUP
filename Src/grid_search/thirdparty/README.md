# thirdparty：抽取的上游源码与适配桩（X-S3）

本目录只包含 **为适配器抽取的上游最小文件集** 与 **适配层自写的桩/替换文件**。
完整来源记录（URL、commit、许可证、补丁、原生偏差、验证 run ID）见
[../provenance.md](../provenance.md)。

| 目录 | 来源 | commit | 许可证 | 抽取内容 |
| --- | --- | --- | --- | --- |
| `vss2sn/` | [vss2sn/path_planning](https://github.com/vss2sn/path_planning) | `d8f9eda` | BSD-3-Clause（`LICENSE.md`） | 栅格 Dijkstra/A*（`planner/a_star/dijkstra/utils`），4 处适配补丁 |
| `nav2_smac_planner/` | [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)（`nav2_smac_planner`） | `3286119…` | Apache-2.0（`LICENSE`；每个抽取文件保留上游 Apache 头） | `AStarAlgorithm<Node2D>` + `Node2D` 核心 + 最小桩集；2 处补丁；`analytic_expansion.hpp` 为 Node2D 专用适配桩（非上游） |
| `jps3d/` | [KumarRobotics/jps3d](https://github.com/KumarRobotics/jps3d) | `ef65e36` | BSD-3-Clause（`LICENSE`） | 2D 图搜索与 JPS 规划器；4 处适配补丁（含取消钩子） |
| `libMultiRobotPlanning/` | [whoenig/libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning) | `4c75fa2` | MIT（`LICENSE`） | 单机器人 A* 模板 3 头文件，未修改；栅格 Environment 在适配层 |
| `justinhj/` | [justinhj/astar-algorithm-cpp](https://github.com/justinhj/astar-algorithm-cpp) | `875bbd8`（v1.4.2） | MIT（`license.txt`；核心头另带宽松署名声明） | `stlastar.h` + `fsa.h`；1 处补丁（`USE_FSA_MEMORY` 守卫，允许池关配置） |

规则：

* 抽取文件保留上游版权/许可证头（上游没有文件头的来源由根许可证文件覆盖，许可证全文随目录保留）。
* 所有适配修改以 `[grid_search 适配补丁 …]` 注释就地标记；未修改文件与上游逐字节一致（SHA256 对照见 provenance.md）。
* `nav2_smac_planner/stubs/` 与 `nav2_smac_planner/include/nav2_smac_planner/analytic_expansion.hpp`
  是 **适配层自写文件（非上游）**，文件头明确标注；用途与依据见 provenance.md。
* 本目录不参与联网下载：构建只读取这些已抽取文件与系统头（Eigen3 / Boost）。
