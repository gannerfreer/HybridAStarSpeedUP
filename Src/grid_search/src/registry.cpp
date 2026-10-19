// 后端注册表：按编译开关（PLANNER_GRID_SEARCH_WITH_*）暴露已启用适配器。

#include "planner/grid_search/backend.hpp"

#include <string>
#include <vector>

#include "planner/grid_search/adapters/adapters.hpp"

namespace planner::grid_search {

std::vector<std::string> available_backends() {
  std::vector<std::string> names;
#ifdef PLANNER_GRID_SEARCH_WITH_JPS3D
  names.push_back("jps3d_astar");
  names.push_back("jps3d_jps");
#endif
#ifdef PLANNER_GRID_SEARCH_WITH_JUSTINHJ
  names.push_back("justinhj_astar");
  names.push_back("justinhj_astar_nopool");
#endif
#ifdef PLANNER_GRID_SEARCH_WITH_LIBMULTIROBOT
  names.push_back("libmultirobot_astar");
#endif
#ifdef PLANNER_GRID_SEARCH_WITH_SMAC2D
  names.push_back("smac_2d_astar");
#endif
#ifdef PLANNER_GRID_SEARCH_WITH_VSS2SN
  names.push_back("vss2sn_astar");
  names.push_back("vss2sn_dijkstra");
#endif
  return names;
}

std::unique_ptr<GridSearchBackend> make_backend(const std::string& name) {
#ifdef PLANNER_GRID_SEARCH_WITH_VSS2SN
  if (name == "vss2sn_dijkstra") {
    return make_vss2sn_dijkstra_backend();
  }
  if (name == "vss2sn_astar") {
    return make_vss2sn_astar_backend();
  }
#endif
#ifdef PLANNER_GRID_SEARCH_WITH_SMAC2D
  if (name == "smac_2d_astar") {
    return make_smac_2d_backend();
  }
#endif
#ifdef PLANNER_GRID_SEARCH_WITH_JPS3D
  if (name == "jps3d_astar") {
    return make_jps3d_astar_backend();
  }
  if (name == "jps3d_jps") {
    return make_jps3d_jps_backend();
  }
#endif
#ifdef PLANNER_GRID_SEARCH_WITH_LIBMULTIROBOT
  if (name == "libmultirobot_astar") {
    return make_libmultirobot_astar_backend();
  }
#endif
#ifdef PLANNER_GRID_SEARCH_WITH_JUSTINHJ
  if (name == "justinhj_astar") {
    return make_justinhj_astar_backend();
  }
  if (name == "justinhj_astar_nopool") {
    return make_justinhj_astar_nopool_backend();
  }
#endif
  (void)name;
  return nullptr;
}

}  // namespace planner::grid_search
