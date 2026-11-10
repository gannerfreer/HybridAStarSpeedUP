/**
 * @file scene.cpp
 * @brief 场景环境校验、车体足迹与栅格坐标系变换的具体实现。
 */

#include "scene.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "types.hpp"

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

Obstacle Obstacle::rectangle(const std::string& id, double x_min_m,
                             double y_min_m, double x_max_m, double y_max_m) {
  Obstacle obstacle;
  obstacle.id = id;
  std::string reason;
  std::vector<Point2d> vertices = math::rectangle_polygon(
      x_min_m, y_min_m, x_max_m, y_max_m, &reason);
  if (auto polygon = math::Polygon2d::convex(std::move(vertices))) {
    obstacle.polygon = std::move(*polygon);
  }
  return obstacle;  // 输入非法时返回空顶点多边形
}

Obstacle Obstacle::convex_polygon(const std::string& id,
                                  std::vector<Point2d> vertices) {
  Obstacle obstacle;
  obstacle.id = id;
  if (auto polygon = math::Polygon2d::convex(std::move(vertices))) {
    obstacle.polygon = std::move(*polygon);
  }
  return obstacle;
}

bool Obstacle::valid(std::string* reason) const {
  return polygon.valid(reason);
}

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
        static_cast<std::size_t>(grid.width) *
            static_cast<std::size_t>(grid.height)) {
      if (reason) *reason = "grid cell count does not match width * height";
      return false;
    }
    for (std::size_t i = 0; i < grid.cells.size(); ++i) {
      if (!grid_value_allowed(grid.cells[i])) {
        if (reason) {
          *reason = "grid cell " + std::to_string(i) +
                    " has a value outside {0,1,2}";
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

bool vehicle_footprint_valid(const VehicleModel& vehicle,
                             std::string* reason) {
  const auto finite_nonneg = [](double v) {
    return std::isfinite(v) && v >= 0.0;
  };
  if (!std::isfinite(vehicle.wheelbase_m) || !(vehicle.wheelbase_m > 0.0)) {
    if (reason) *reason = "vehicle wheelbase must be finite and positive";
    return false;
  }
  if (!std::isfinite(vehicle.width_m) || !(vehicle.width_m > 0.0)) {
    if (reason) *reason = "vehicle width must be finite and positive";
    return false;
  }
  if (!finite_nonneg(vehicle.front_overhang_m)) {
    if (reason) *reason = "vehicle front overhang must be finite and >= 0";
    return false;
  }
  if (!finite_nonneg(vehicle.rear_overhang_m)) {
    if (reason) *reason = "vehicle rear overhang must be finite and >= 0";
    return false;
  }
  if (!finite_nonneg(vehicle.safety_margin_m)) {
    if (reason) *reason = "vehicle safety margin must be finite and >= 0";
    return false;
  }
  return true;
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
      std::hypot(std::max(std::abs(footprint.x_min_m),
                          std::abs(footprint.x_max_m)),
                 std::max(std::abs(footprint.y_min_m),
                          std::abs(footprint.y_max_m)));
  return footprint;
}

// ---------------------------------------------------------------------------
// 栅格坐标系 GridFrame
// ---------------------------------------------------------------------------

GridFrame GridFrame::from_grid(const GridView& grid) noexcept {
  GridFrame frame;
  frame.cos_yaw = std::cos(grid.origin.yaw_rad);
  frame.sin_yaw = std::sin(grid.origin.yaw_rad);
  frame.origin_x = grid.origin.x_m;
  frame.origin_y = grid.origin.y_m;
  frame.width_m = grid.width * grid.resolution_m;
  frame.height_m = grid.height * grid.resolution_m;
  frame.resolution_m = grid.resolution_m;
  return frame;
}

Point2d GridFrame::to_grid_frame(const Point2d& world_point) const noexcept {
  const double dx = world_point.x_m - origin_x;
  const double dy = world_point.y_m - origin_y;
  return Point2d{cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy};
}

bool GridFrame::inside_map(const Point2d& grid_point) const noexcept {
  return grid_point.x_m >= 0.0 && grid_point.x_m <= width_m &&
         grid_point.y_m >= 0.0 && grid_point.y_m <= height_m;
}

double GridFrame::distance_to_boundary(const Point2d& grid_point) const noexcept {
  const double d = std::min(std::min(grid_point.x_m, width_m - grid_point.x_m),
                            std::min(grid_point.y_m, height_m - grid_point.y_m));
  return d;  // 若在地图外部则为负值
}

}  // namespace planner::collision
