/**
 * @file types.cpp
 * @brief Implementations of common utilities, conversions, and operators.
 */

#include "types.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace planner::common {

double normalize_angle_rad(double angle_rad) {
  // Assume the input is a finite floating-point value.
  // Values already in [-pi, pi) pass through directly to preserve idempotence.
  if (angle_rad >= -kPi && angle_rad < kPi) {
    return angle_rad;
  }
  double wrapped = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  double result = wrapped - kPi;
  if (result >= kPi) {
    // Only reachable due to floating-point rounding errors in fmod; maintain half-open interval.
    result = -kPi;
  }
  return result;
}

std::string to_string(Direction direction) {
  switch (direction) {
    case Direction::kForward:
      return "forward";
    case Direction::kReverse:
      return "reverse";
  }
  throw std::invalid_argument("to_string(Direction): invalid enumerator");
}

std::string to_string(Status status) {
  switch (status) {
    case Status::kSuccess:
      return "success";
    case Status::kInvalidInput:
      return "invalid_input";
    case Status::kNoPath:
      return "no_path";
    case Status::kTimeout:
      return "timeout";
    case Status::kCancelled:
      return "cancelled";
    case Status::kOptimizationFailed:
      return "optimization_failed";
    case Status::kValidationFailed:
      return "validation_failed";
  }
  throw std::invalid_argument("to_string(Status): invalid enumerator");
}

int GridView::cell(int row, int col) const {
  if (!in_bounds(row, col)) {
    throw std::out_of_range("GridView::cell: row/col out of bounds");
  }
  const auto index = static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
                     static_cast<std::size_t>(col);
  return cells[index];
}

bool operator==(const Point2d& lhs, const Point2d& rhs) {
  return lhs.x_m == rhs.x_m && lhs.y_m == rhs.y_m;
}

bool operator==(const Pose2d& lhs, const Pose2d& rhs) {
  return lhs.x_m == rhs.x_m && lhs.y_m == rhs.y_m && lhs.yaw_rad == rhs.yaw_rad;
}

bool operator==(const VehicleModel& lhs, const VehicleModel& rhs) {
  return lhs.id == rhs.id && lhs.wheelbase_m == rhs.wheelbase_m && lhs.width_m == rhs.width_m &&
         lhs.front_overhang_m == rhs.front_overhang_m && lhs.rear_overhang_m == rhs.rear_overhang_m &&
         lhs.max_steer_rad == rhs.max_steer_rad && lhs.safety_margin_m == rhs.safety_margin_m;
}

bool operator==(const GridView& lhs, const GridView& rhs) {
  return lhs.map_id == rhs.map_id && lhs.map_version == rhs.map_version && lhs.width == rhs.width &&
         lhs.height == rhs.height && lhs.resolution_m == rhs.resolution_m && lhs.origin == rhs.origin &&
         lhs.cells == rhs.cells;
}

bool operator==(const PathPoint& lhs, const PathPoint& rhs) {
  return lhs.pose == rhs.pose && lhs.s_m == rhs.s_m && lhs.curvature_inv_m == rhs.curvature_inv_m &&
         lhs.direction == rhs.direction;
}

bool operator==(const PathSegment& lhs, const PathSegment& rhs) {
  return lhs.points == rhs.points;
}

}  // namespace planner::common
