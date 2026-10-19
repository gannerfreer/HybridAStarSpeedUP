#include "planner/common/types.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include "planner/common/errors.hpp"

namespace planner::common {

double normalize_angle_rad(double angle_rad) {
  // 假定输入有限；JSON 编解码器在调用前检查有限性。
  // 规范值原样通过，使该函数严格幂等（JSON 编解码器依赖这一点保证往返稳定）。
  if (angle_rad >= -kPi && angle_rad < kPi) {
    return angle_rad;
  }
  double wrapped = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  double result = wrapped - kPi;
  if (result >= kPi) {
    // 只有在 fmod 舍入误差下才会到达；保持半开区间。
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

const char* to_string(ErrorCode code) noexcept {
  switch (code) {
    case ErrorCode::kParseError:
      return "parse_error";
    case ErrorCode::kInvalidDocument:
      return "invalid_document";
    case ErrorCode::kMissingField:
      return "missing_field";
    case ErrorCode::kTypeError:
      return "type_error";
    case ErrorCode::kUnsupportedSchemaVersion:
      return "unsupported_schema_version";
    case ErrorCode::kUnknownType:
      return "unknown_type";
    case ErrorCode::kInvalidValue:
      return "invalid_value";
    case ErrorCode::kUnknownEnumValue:
      return "unknown_enum_value";
    case ErrorCode::kNonFiniteValue:
      return "non_finite_value";
  }
  return "unknown";
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

bool operator==(const TrajectoryPoint& lhs, const TrajectoryPoint& rhs) {
  return lhs.t_s == rhs.t_s && lhs.pose == rhs.pose && lhs.velocity_mps == rhs.velocity_mps &&
         lhs.acceleration_mps2 == rhs.acceleration_mps2 && lhs.curvature_inv_m == rhs.curvature_inv_m;
}

bool operator==(const PlanRequest& lhs, const PlanRequest& rhs) {
  return lhs.request_id == rhs.request_id && lhs.map_id == rhs.map_id && lhs.map_version == rhs.map_version &&
         lhs.vehicle == rhs.vehicle && lhs.start == rhs.start && lhs.goal == rhs.goal &&
         lhs.config == rhs.config && lhs.time_budget_s == rhs.time_budget_s &&
         lhs.node_budget == rhs.node_budget && lhs.cancelled == rhs.cancelled;
}

bool operator==(const PlanResult& lhs, const PlanResult& rhs) {
  return lhs.request_id == rhs.request_id && lhs.status == rhs.status && lhs.reason == rhs.reason &&
         lhs.path == rhs.path && lhs.trajectory == rhs.trajectory && lhs.statistics == rhs.statistics;
}

bool operator==(const CorridorPolygon& lhs, const CorridorPolygon& rhs) {
  return lhs.s_start_m == rhs.s_start_m && lhs.s_end_m == rhs.s_end_m && lhs.vertices == rhs.vertices;
}

bool operator==(const Corridor& lhs, const Corridor& rhs) {
  return lhs.id == rhs.id && lhs.map_id == rhs.map_id && lhs.map_version == rhs.map_version &&
         lhs.frame_id == rhs.frame_id && lhs.polygons == rhs.polygons;
}

}  // namespace planner::common
