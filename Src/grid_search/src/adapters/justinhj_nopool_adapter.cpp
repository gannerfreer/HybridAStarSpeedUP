// justinhj A* 适配器：内存池关（USE_FSA_MEMORY=0）。
//
// 本翻译单元由 CMake 单独编译并定义 USE_FSA_MEMORY=0（见
// Src/grid_search/CMakeLists.txt）；上游把该宏硬编码为 1，这是 1 行级适配
// 补丁（#ifndef 守卫，默认行为不变，见 provenance.md）。
// 使用独立 UserState 标签避免与池开配置的模板实例发生 ODR 冲突。

#include "justinhj_common.hpp"

// 编译期配置断言：本翻译单元必须以 USE_FSA_MEMORY=0 编译（见 CMake）。
#if !defined(USE_FSA_MEMORY) || USE_FSA_MEMORY != 0
#error "justinhj_nopool_adapter.cpp must be compiled with -DUSE_FSA_MEMORY=0"
#endif

namespace planner::grid_search {

std::unique_ptr<GridSearchBackend> make_justinhj_astar_nopool_backend() {
  return std::make_unique<justinhj_detail::JustinhjBackend<
      justinhj_detail::NoPoolTag>>("justinhj_astar_nopool");
}

}  // namespace planner::grid_search
