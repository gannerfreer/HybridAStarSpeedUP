// planner_graph_search 适配器工厂声明（S-01..S-05）。
//
// 声明由对应 CMake 目标以 PUBLIC 编译定义启用：
//   PLANNER_GRAPH_SEARCH_HAS_BOOST          -> boost_astar / boost_dijkstra / boost_distance_field
//   PLANNER_GRAPH_SEARCH_HAS_LMRP           -> lmrp_astar
//   PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_POOL  -> justinhj_astar_pool
//   PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_NOPOOL-> justinhj_astar_nopool
//   PLANNER_GRAPH_SEARCH_HAS_LEMON          -> lemon_dijkstra / lemon_distance_field
//   PLANNER_GRAPH_SEARCH_HAS_IGRAPH         -> igraph_dijkstra / igraph_distance_field
//
// igraph 适配器属 GPL-2.0-or-later 隔离组：仅存在于独立目标
// planner_graph_search_igraph，绝不链入 planner_graph_search。

#pragma once

#include <memory>

#include "planner/graph_search/types.hpp"

namespace planner {
namespace graph_search {

#if defined(PLANNER_GRAPH_SEARCH_HAS_BOOST)
// S-01 Boost Graph（系统 Boost 1.71，header-only）。
std::unique_ptr<GraphSearcher> make_boost_astar();
std::unique_ptr<GraphSearcher> make_boost_dijkstra();
std::unique_ptr<DistanceFieldSearcher> make_boost_distance_field();
#endif

#if defined(PLANNER_GRAPH_SEARCH_HAS_LMRP)
// S-02 libMultiRobotPlanning 单机器人 A*（vendored）。
std::unique_ptr<GraphSearcher> make_lmrp_astar();
#endif

#if defined(PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_POOL)
// S-03 justinhj A*，内存池开（vendored + 守卫补丁）。
std::unique_ptr<GraphSearcher> make_justinhj_astar_pool();
#endif

#if defined(PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_NOPOOL)
// S-03 justinhj A*，内存池关（-DUSE_FSA_MEMORY=0）。
std::unique_ptr<GraphSearcher> make_justinhj_astar_nopool();
#endif

#if defined(PLANNER_GRAPH_SEARCH_HAS_LEMON)
// S-04 LEMON（OpenROAD 副本）Dijkstra。
std::unique_ptr<GraphSearcher> make_lemon_dijkstra();
std::unique_ptr<DistanceFieldSearcher> make_lemon_distance_field();
#endif

#if defined(PLANNER_GRAPH_SEARCH_HAS_IGRAPH)
// S-05 igraph（GPL-2.0-or-later 隔离组）Dijkstra。
std::unique_ptr<GraphSearcher> make_igraph_dijkstra();
std::unique_ptr<DistanceFieldSearcher> make_igraph_distance_field();
#endif

}  // namespace graph_search
}  // namespace planner
