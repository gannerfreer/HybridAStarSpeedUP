#pragma once

// planner_map 测试套件共用的辅助函数。数据目录由构建系统
// 以 PLANNER_MAP_TEST_DATA_DIR 提供。

#include <memory>
#include <string>

#include <planner/map/map.hpp>

namespace planner::map::test {

inline std::string data_path(const std::string& name) {
  return std::string(PLANNER_MAP_TEST_DATA_DIR) + "/" + name;
}

// 示例地图套件的固定原点：mapping_example.osm 的第一个节点
// （见 tests/data/README.md）。
inline GeoPoint example_origin() { return GeoPoint{49.00345654351, 8.42427590707, 0.0}; }

inline MapLoadOptions example_options(ProjectionKind kind = ProjectionKind::kUtm) {
  MapLoadOptions options;
  options.projection = kind;
  options.origin = example_origin();
  return options;
}

inline ImportResult load_mapping_example(ProjectionKind kind = ProjectionKind::kUtm) {
  return Map::import_osm_file(data_path("mapping_example.osm"), example_options(kind));
}

inline ImportResult load_disconnected() {
  return Map::import_osm_file(data_path("disconnected.osm"), example_options());
}

}  // namespace planner::map::test
