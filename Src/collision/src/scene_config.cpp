/**
 * @file scene_config.cpp
 * @brief 由纯数据场景配置构建碰撞检测场景的具体实现
 */

#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "scene.hpp"

namespace planner::collision {

namespace {

using planner::common::kCellFree;

// 非法输入返回宽高不自洽的空场景，使 Scene::valid() 返回 false
Scene invalid_scene() {
  Scene scene;
  scene.grid.width = 1;
  scene.grid.height = 0;
  return scene;
}

}  // namespace

Scene make_scene(const common::ScenarioConfig& scenario) {
  std::string reason;
  if (!common::scenario_config_valid(scenario, &reason)) {
    return invalid_scene();
  }
  const std::size_t width = static_cast<std::size_t>(scenario.width);
  const std::size_t height = static_cast<std::size_t>(scenario.height);
  if (width > std::numeric_limits<std::size_t>::max() / height) {
    return invalid_scene();
  }

  Scene scene;
  scene.grid.map_id = "scenario";
  scene.grid.map_version = "1";
  scene.grid.width = scenario.width;
  scene.grid.height = scenario.height;
  scene.grid.resolution_m = scenario.resolution_m;
  scene.grid.origin = scenario.origin;
  scene.grid.cells.assign(width * height, kCellFree);

  scene.obstacles.reserve(scenario.obstacles.size());
  for (const common::ObstacleConfig& config : scenario.obstacles) {
    scene.obstacles.push_back(Obstacle::convex_polygon(config.id, config.vertices));
  }
  return scene;
}

}  // namespace planner::collision
