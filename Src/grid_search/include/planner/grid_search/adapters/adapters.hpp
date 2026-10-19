#pragma once

// 适配器工厂声明。所有适配器返回统一 GridSearchBackend，公共接口不暴露
// 上游类型；上游源码抽取与补丁见 Src/grid_search/provenance.md。

#include <memory>
#include <vector>

#include "planner/grid_search/backend.hpp"

namespace planner::grid_search {

// vss2sn/path_planning（BSD-3-Clause）：栅格 Dijkstra（点到点 + 距离场）与
// 栅格 A*（点到点）。四/八邻域均支持；八邻域为适配补丁（上游仅四邻域）。
std::unique_ptr<GridSearchBackend> make_vss2sn_dijkstra_backend();
std::unique_ptr<GridSearchBackend> make_vss2sn_astar_backend();

// Nav2 SmacPlanner2D（Apache-2.0，自 navigation2 抽取）：栅格 A*。
// 原生代价感知模式（cost_penalty > 0）单列，不进入统一排名。
struct Smac2dOptions {
  // 0 = 几何代价（统一口径）；>0 启用上游代价感知项（单列）。
  double cost_penalty{0.0};
  // 可选逐单元代价层（尺寸必须等于 width*height，取值 [0, 252]）；
  // 仅对空闲单元生效，用于单列验证代价感知模式。空 = 不启用。
  std::vector<double> cell_costs;
};

std::unique_ptr<GridSearchBackend> make_smac_2d_backend(
    const Smac2dOptions& options = Smac2dOptions{});

// jps3d（BSD-3-Clause）：普通 A*（use_jps=false，适配补丁禁对角穿角）与
// JPS 单列变体（原生语义，仅八邻域，不参与统一排名）。
std::unique_ptr<GridSearchBackend> make_jps3d_astar_backend();
std::unique_ptr<GridSearchBackend> make_jps3d_jps_backend();

// libMultiRobotPlanning（MIT）：栅格 Environment 适配（复用图版 A* 模板核心）。
std::unique_ptr<GridSearchBackend> make_libmultirobot_astar_backend();

// justinhj/astar-algorithm-cpp（MIT）：栅格 UserState。
// 内存池开/关为两个配置（容量按规模设置）。
std::unique_ptr<GridSearchBackend> make_justinhj_astar_backend();         // 池开
std::unique_ptr<GridSearchBackend> make_justinhj_astar_nopool_backend();  // 池关

}  // namespace planner::grid_search
