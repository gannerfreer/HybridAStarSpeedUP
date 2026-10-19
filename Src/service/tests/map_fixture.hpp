#pragma once

// 服务单元测试共用的地图夹具辅助：导入只读的 v3 mapping_example 与
// 合成的小窗口地图。每个测试进程导入一次（gtest_discover_tests 为每个
// 测试启动独立进程），不写入任何夹具文件。
//
// 路径与投影原点由 CMake 通过 PLANNER_SERVICE_TEST_MAP_PATH /
// PLANNER_SERVICE_TEST_WINDOW_MAP_PATH / PLANNER_SERVICE_TEST_ORIGIN_* 提供。

#include <memory>
#include <string>

#include "planner/map/map.hpp"

#ifndef PLANNER_SERVICE_TEST_MAP_PATH
#error "PLANNER_SERVICE_TEST_MAP_PATH must be defined by the test CMakeLists"
#endif
#ifndef PLANNER_SERVICE_TEST_WINDOW_MAP_PATH
#error "PLANNER_SERVICE_TEST_WINDOW_MAP_PATH must be defined by the test CMakeLists"
#endif
#ifndef PLANNER_SERVICE_TEST_ORIGIN_LAT
#error "PLANNER_SERVICE_TEST_ORIGIN_LAT must be defined by the test CMakeLists"
#endif
#ifndef PLANNER_SERVICE_TEST_ORIGIN_LON
#error "PLANNER_SERVICE_TEST_ORIGIN_LON must be defined by the test CMakeLists"
#endif
#ifndef PLANNER_SERVICE_TEST_WINDOW_ORIGIN_LAT
#error "PLANNER_SERVICE_TEST_WINDOW_ORIGIN_LAT must be defined by the test CMakeLists"
#endif
#ifndef PLANNER_SERVICE_TEST_WINDOW_ORIGIN_LON
#error "PLANNER_SERVICE_TEST_WINDOW_ORIGIN_LON must be defined by the test CMakeLists"
#endif

namespace planner::service::test {

// CMake 以带引号的字符串宏提供原点（与集成测试 target 的约定一致），
// 这里解析为 double。
inline map::GeoPoint example_origin() {
  return map::GeoPoint{std::stod(PLANNER_SERVICE_TEST_ORIGIN_LAT),
                       std::stod(PLANNER_SERVICE_TEST_ORIGIN_LON), 0.0};
}

inline map::GeoPoint window_origin() {
  return map::GeoPoint{std::stod(PLANNER_SERVICE_TEST_WINDOW_ORIGIN_LAT),
                       std::stod(PLANNER_SERVICE_TEST_WINDOW_ORIGIN_LON), 0.0};
}

inline map::MapLoadOptions example_map_options() {
  map::MapLoadOptions options;
  options.projection = map::ProjectionKind::kUtm;
  options.origin = example_origin();
  return options;
}

inline map::MapLoadOptions window_map_options() {
  map::MapLoadOptions options;
  options.projection = map::ProjectionKind::kUtm;
  options.origin = window_origin();
  return options;
}

// 真实 v3 地图（371 个 lanelet 的完整路网）。
inline map::ImportResult load_example_map() {
  return map::Map::import_osm_file(PLANNER_SERVICE_TEST_MAP_PATH,
                                   example_map_options());
}

// 合成小地图：约 4 m 宽、40 m 长的直线道路 + 约 100 m 外的孤立点
// （用于窗口裁剪与道路外占据测试；见 tests/data/window_map.osm）。
inline map::ImportResult load_window_map() {
  return map::Map::import_osm_file(PLANNER_SERVICE_TEST_WINDOW_MAP_PATH,
                                   window_map_options());
}

}  // namespace planner::service::test
