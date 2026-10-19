#pragma once

// 对共享夹具加载器（fixture_scene.hpp）的测试侧便捷封装。夹具目录在
// 配置时固化；测试从不依赖工作目录。

#include "fixture_scene.hpp"

#ifndef PLANNER_HYBRID_ASTAR_FIXTURES_DIR
#error "PLANNER_HYBRID_ASTAR_FIXTURES_DIR must be defined by the test CMakeLists"
#endif

namespace planner::hybrid_astar::test_support {

inline std::string fixture_path(const std::string& relative) {
  return std::string(PLANNER_HYBRID_ASTAR_FIXTURES_DIR) + "/" + relative;
}

inline common::VehicleModel load_default_vehicle() {
  return load_default_vehicle_from(fixture_path("vehicles/default.json"));
}

inline std::vector<PathCase> load_path_cases() {
  return load_path_cases_from(fixture_path("paths/cases.json"));
}

}  // namespace planner::hybrid_astar::test_support
