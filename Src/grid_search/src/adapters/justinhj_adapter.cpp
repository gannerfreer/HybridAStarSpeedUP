// justinhj A* 适配器：内存池开（USE_FSA_MEMORY=1，上游默认）。

#include "justinhj_common.hpp"

// 编译期配置断言：池开配置必须保持上游默认值（补丁 #ifndef 守卫后仍为 1）。
#if !defined(USE_FSA_MEMORY) || USE_FSA_MEMORY != 1
#error "justinhj_adapter.cpp must keep the upstream default USE_FSA_MEMORY=1"
#endif

namespace planner::grid_search {

std::unique_ptr<GridSearchBackend> make_justinhj_astar_backend() {
  return std::make_unique<justinhj_detail::JustinhjBackend<
      justinhj_detail::PoolTag>>("justinhj_astar");
}

}  // namespace planner::grid_search
