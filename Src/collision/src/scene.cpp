/**
 * @file scene.cpp
 * @brief 场景环境校验、车体足迹与栅格坐标系变换的具体实现
 */

#include "scene.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

using planner::math::operator+;
using planner::math::operator-;
using planner::math::operator*;

namespace planner::collision {

namespace {

using planner::common::kCellFree;
using planner::common::kCellOccupied;
using planner::common::kCellUnknown;

bool grid_value_allowed(int value) {
  return value == kCellFree || value == kCellOccupied || value == kCellUnknown;
}

}  // namespace

// ---------------------------------------------------------------------------
// 凸多边形障碍物 Obstacle
// ---------------------------------------------------------------------------

Obstacle Obstacle::rectangle(std::string_view id, double x_min_m, double y_min_m, double x_max_m,
                             double y_max_m) {
  Obstacle obstacle;
  obstacle.id = std::string(id);
  std::string reason;
  std::vector<Point2d> vertices =
      math::rectangle_polygon(x_min_m, y_min_m, x_max_m, y_max_m, &reason);
  if (auto polygon = math::Polygon2d::convex(std::move(vertices))) {
    obstacle.polygon = std::move(*polygon);
  }
  return obstacle;  // 输入非法时返回空顶点多边形
}

Obstacle Obstacle::convex_polygon(std::string_view id, std::vector<Point2d> vertices) {
  Obstacle obstacle;
  obstacle.id = std::string(id);
  if (auto polygon = math::Polygon2d::convex(std::move(vertices))) {
    obstacle.polygon = std::move(*polygon);
  }
  return obstacle;
}

bool Obstacle::valid(std::string* reason) const { return polygon.valid(reason); }

// ---------------------------------------------------------------------------
// 场景环境 Scene
// ---------------------------------------------------------------------------

bool Scene::valid(std::string* reason) const {
  const bool no_grid = grid.width == 0 && grid.height == 0;
  if (!no_grid) {
    if (grid.width <= 0 || grid.height <= 0) {
      if (reason) {
        *reason = "grid needs both width and height positive (or both zero)";
      }
      return false;
    }
    if (!(grid.resolution_m > 0.0) || !std::isfinite(grid.resolution_m)) {
      if (reason) *reason = "grid resolution must be finite and positive";
      return false;
    }
    if (!std::isfinite(grid.origin.x_m) || !std::isfinite(grid.origin.y_m) ||
        !std::isfinite(grid.origin.yaw_rad)) {
      if (reason) *reason = "grid origin is not finite";
      return false;
    }
    if (grid.cells.size() !=
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height)) {
      if (reason) *reason = "grid cell count does not match width * height";
      return false;
    }
    for (std::size_t i = 0; i < grid.cells.size(); ++i) {
      if (!grid_value_allowed(grid.cells[i])) {
        if (reason) {
          *reason = "grid cell " + std::to_string(i) + " has a value outside {0,1,2}";
        }
        return false;
      }
    }
  }
  std::string obstacle_reason;
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    if (!obstacles[i].valid(&obstacle_reason)) {
      if (reason) {
        *reason = "obstacle " + std::to_string(i) + ": " + obstacle_reason;
      }
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// 车辆足迹 Footprint
// ---------------------------------------------------------------------------

bool vehicle_footprint_valid(const VehicleModel& vehicle, std::string* reason) {
  return common::vehicle_valid(vehicle, reason);
}

Footprint make_footprint(const VehicleModel& vehicle) noexcept {
  Footprint footprint;
  const double margin = vehicle.safety_margin_m;
  footprint.x_min_m = -(vehicle.rear_overhang_m + margin);
  footprint.x_max_m = vehicle.wheelbase_m + vehicle.front_overhang_m + margin;
  footprint.y_min_m = -(0.5 * vehicle.width_m + margin);
  footprint.y_max_m = 0.5 * vehicle.width_m + margin;
  footprint.half_width_m = 0.5 * (footprint.y_max_m - footprint.y_min_m);
  footprint.half_length_m = 0.5 * (footprint.x_max_m - footprint.x_min_m);
  footprint.center_x_m = 0.5 * (footprint.x_min_m + footprint.x_max_m);
  footprint.corner_radius_m =
      std::hypot(std::max(std::abs(footprint.x_min_m), std::abs(footprint.x_max_m)),
                 std::max(std::abs(footprint.y_min_m), std::abs(footprint.y_max_m)));
  return footprint;
}

}  // namespace planner::collision
