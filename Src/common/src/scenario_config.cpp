/**
 * @file scenario_config.cpp
 * @brief 纯数据场景配置合法性校验的具体实现
 */

#include "planner/common/scenario_config.hpp"

#include <cmath>
#include <string>

#include "planner/math/polygon2d.hpp"

namespace planner::common {

namespace {

bool finite_pose(const math::Pose2d& pose) {
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) && std::isfinite(pose.yaw_rad);
}

}  // namespace

bool scenario_config_valid(const ScenarioConfig& scenario, std::string* reason) {
  if (scenario.width <= 0) {
    if (reason) *reason = "scenario width must be > 0";
    return false;
  }
  if (scenario.height <= 0) {
    if (reason) *reason = "scenario height must be > 0";
    return false;
  }
  if (!std::isfinite(scenario.resolution_m) || !(scenario.resolution_m > 0.0)) {
    if (reason) *reason = "scenario resolution_m must be finite and > 0";
    return false;
  }
  if (!finite_pose(scenario.origin)) {
    if (reason) *reason = "scenario origin pose is not finite";
    return false;
  }
  if (!finite_pose(scenario.start)) {
    if (reason) *reason = "scenario start pose is not finite";
    return false;
  }
  if (!finite_pose(scenario.goal)) {
    if (reason) *reason = "scenario goal pose is not finite";
    return false;
  }

  for (std::size_t i = 0; i < scenario.obstacles.size(); ++i) {
    const ObstacleConfig& obstacle = scenario.obstacles[i];
    const std::string prefix = "scenario obstacle " + std::to_string(i);
    if (obstacle.id.empty()) {
      if (reason) *reason = prefix + " id must not be empty";
      return false;
    }
    if (obstacle.vertices.empty()) {
      if (reason) *reason = prefix + " vertices must not be empty";
      return false;
    }
    std::string polygon_reason;
    if (!math::convex_polygon_valid(obstacle.vertices, &polygon_reason)) {
      if (reason) {
        *reason = prefix + " vertices form an invalid convex polygon: " + polygon_reason;
      }
      return false;
    }
  }
  return true;
}

}  // namespace planner::common
