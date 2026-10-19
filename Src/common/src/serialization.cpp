#include "planner/common/serialization.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <nlohmann/json.hpp>

// 实现说明：
//
// * 线上契约（字段名、单位、必需/可选状态、枚举值和校验规则）冻结在
//   tests/fixtures/v1/serialization/README.md。编码与解码执行相同的结构
//   和范围校验，因此写入的值总能读回。
// * JSON 无法表示 NaN/Infinity。编码时检查每个 double；解码时，
//   nlohmann/json 对 1e999 这类溢出字面量抛出 out_of_range 异常，
//   本文件将其映射为 kNonFiniteValue。
// * 解码时容忍未知字段，编码时从不输出未知字段；
//   信封字段（schema_version/type）也被各类型的载荷解码器忽略。
// * 诸如 "path_segment.points[2].s_m" 的字段路径字符串只用于错误消息；
//   一致性通过 ErrorCode 断言，而不是文本。

namespace planner::common {
namespace {

using nlohmann::json;

// ---------------------------------------------------------------------------
// 基础辅助函数。`path` 是待校验对象的 JSON 路径，
// 例如 "pose2d" 或 "plan_request.vehicle"（末尾无点号）。
// ---------------------------------------------------------------------------

[[noreturn]] void fail(ErrorCode code, std::string message) {
  throw SerializationError(code, std::move(message));
}

void check_finite(double value, const std::string& path) {
  if (!std::isfinite(value)) {
    fail(ErrorCode::kNonFiniteValue, path + ": NaN/Infinity is not representable in JSON");
  }
}

std::string indexed(const std::string& path, std::size_t index) {
  return path + "[" + std::to_string(index) + "]";
}

const json& require_field(const json& object, const char* name, const std::string& path) {
  const auto it = object.find(name);
  if (it == object.end()) {
    fail(ErrorCode::kMissingField, path + "." + name + ": missing required field");
  }
  return *it;
}

const json* optional_field(const json& object, const char* name) {
  const auto it = object.find(name);
  return it == object.end() ? nullptr : &*it;
}

const json& value_object(const json& value, const std::string& path) {
  if (!value.is_object()) {
    fail(ErrorCode::kTypeError, path + ": must be a JSON object");
  }
  return value;
}

const json& value_array(const json& value, const std::string& path) {
  if (!value.is_array()) {
    fail(ErrorCode::kTypeError, path + ": must be a JSON array");
  }
  return value;
}

double value_double(const json& value, const std::string& path) {
  if (!value.is_number()) {
    fail(ErrorCode::kTypeError, path + ": must be a JSON number");
  }
  const double number = value.get<double>();
  check_finite(number, path);
  return number;
}

std::int64_t value_integer(const json& value, const std::string& path) {
  if (!value.is_number_integer()) {
    fail(ErrorCode::kTypeError, path + ": must be a JSON integer");
  }
  return value.get<std::int64_t>();
}

int value_int32(const json& value, const std::string& path) {
  const std::int64_t number = value_integer(value, path);
  if (number < std::numeric_limits<int>::min() || number > std::numeric_limits<int>::max()) {
    fail(ErrorCode::kInvalidValue, path + ": integer is outside the 32-bit range");
  }
  return static_cast<int>(number);
}

std::string value_string(const json& value, const std::string& path) {
  if (!value.is_string()) {
    fail(ErrorCode::kTypeError, path + ": must be a JSON string");
  }
  return value.get<std::string>();
}

bool value_boolean(const json& value, const std::string& path) {
  if (!value.is_boolean()) {
    fail(ErrorCode::kTypeError, path + ": must be a JSON boolean");
  }
  return value.get<bool>();
}

// 递归拒绝调用方定义的 JSON（config / statistics）中的非有限 double：
// nlohmann/json 会把 NaN/Infinity 静默转储为 null。
void check_json_finite(const json& value, const std::string& path) {
  switch (value.type()) {
    case json::value_t::number_float:
      check_finite(value.get<double>(), path);
      break;
    case json::value_t::object:
      for (auto it = value.begin(); it != value.end(); ++it) {
        check_json_finite(it.value(), path + "." + it.key());
      }
      break;
    case json::value_t::array:
      for (std::size_t i = 0; i < value.size(); ++i) {
        check_json_finite(value[i], indexed(path, i));
      }
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// 枚举名称。
// ---------------------------------------------------------------------------

const char* direction_name(Direction direction) {
  switch (direction) {
    case Direction::kForward:
      return "forward";
    case Direction::kReverse:
      return "reverse";
  }
  fail(ErrorCode::kInvalidValue, "direction: unknown enumerator");
}

const char* status_name(Status status) {
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
  fail(ErrorCode::kInvalidValue, "status: unknown enumerator");
}

Direction direction_from_name(const std::string& name, const std::string& path) {
  if (name == "forward") {
    return Direction::kForward;
  }
  if (name == "reverse") {
    return Direction::kReverse;
  }
  fail(ErrorCode::kUnknownEnumValue, path + ": unknown direction '" + name + "'");
}

Status status_from_name(const std::string& name, const std::string& path) {
  if (name == "success") {
    return Status::kSuccess;
  }
  if (name == "invalid_input") {
    return Status::kInvalidInput;
  }
  if (name == "no_path") {
    return Status::kNoPath;
  }
  if (name == "timeout") {
    return Status::kTimeout;
  }
  if (name == "cancelled") {
    return Status::kCancelled;
  }
  if (name == "optimization_failed") {
    return Status::kOptimizationFailed;
  }
  if (name == "validation_failed") {
    return Status::kValidationFailed;
  }
  fail(ErrorCode::kUnknownEnumValue, path + ": unknown status '" + name + "'");
}

// ---------------------------------------------------------------------------
// Pose2d.
// ---------------------------------------------------------------------------

void check_pose_finite(const Pose2d& pose, const std::string& prefix) {
  check_finite(pose.x_m, prefix + "x_m");
  check_finite(pose.y_m, prefix + "y_m");
  check_finite(pose.yaw_rad, prefix + "yaw_rad");
}

json pose_to_json(const Pose2d& pose, const std::string& path) {
  check_pose_finite(pose, path + ".");
  // yaw_rad 在编码和解码两个方向都会归一化；规范输入保持不变。
  return json{{"x_m", pose.x_m}, {"y_m", pose.y_m}, {"yaw_rad", normalize_angle_rad(pose.yaw_rad)}};
}

Pose2d pose_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  Pose2d pose;
  pose.x_m = value_double(require_field(object, "x_m", path), path + ".x_m");
  pose.y_m = value_double(require_field(object, "y_m", path), path + ".y_m");
  pose.yaw_rad = value_double(require_field(object, "yaw_rad", path), path + ".yaw_rad");
  pose.yaw_rad = normalize_angle_rad(pose.yaw_rad);
  return pose;
}

// ---------------------------------------------------------------------------
// Point2d（走廊多边形的顶点）。
// ---------------------------------------------------------------------------

json point2d_to_json(const Point2d& point, const std::string& path) {
  check_finite(point.x_m, path + ".x_m");
  check_finite(point.y_m, path + ".y_m");
  return json{{"x_m", point.x_m}, {"y_m", point.y_m}};
}

Point2d point2d_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  Point2d point;
  point.x_m = value_double(require_field(object, "x_m", path), path + ".x_m");
  point.y_m = value_double(require_field(object, "y_m", path), path + ".y_m");
  return point;
}

// ---------------------------------------------------------------------------
// VehicleModel.
// ---------------------------------------------------------------------------

void validate_vehicle_model(const VehicleModel& value, const std::string& path) {
  if (value.id.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".id: must not be empty");
  }
  check_finite(value.wheelbase_m, path + ".wheelbase_m");
  if (!(value.wheelbase_m > 0.0)) {
    fail(ErrorCode::kInvalidValue, path + ".wheelbase_m: must be > 0");
  }
  check_finite(value.width_m, path + ".width_m");
  if (!(value.width_m > 0.0)) {
    fail(ErrorCode::kInvalidValue, path + ".width_m: must be > 0");
  }
  check_finite(value.front_overhang_m, path + ".front_overhang_m");
  if (!(value.front_overhang_m >= 0.0)) {
    fail(ErrorCode::kInvalidValue, path + ".front_overhang_m: must be >= 0");
  }
  check_finite(value.rear_overhang_m, path + ".rear_overhang_m");
  if (!(value.rear_overhang_m >= 0.0)) {
    fail(ErrorCode::kInvalidValue, path + ".rear_overhang_m: must be >= 0");
  }
  check_finite(value.max_steer_rad, path + ".max_steer_rad");
  if (!(value.max_steer_rad > 0.0 && value.max_steer_rad < kPi / 2.0)) {
    fail(ErrorCode::kInvalidValue, path + ".max_steer_rad: must be in the open interval (0, pi/2)");
  }
  check_finite(value.safety_margin_m, path + ".safety_margin_m");
  if (!(value.safety_margin_m >= 0.0)) {
    fail(ErrorCode::kInvalidValue, path + ".safety_margin_m: must be >= 0");
  }
}

json vehicle_model_to_json(const VehicleModel& value, const std::string& path) {
  validate_vehicle_model(value, path);
  return json{{"id", value.id},
              {"wheelbase_m", value.wheelbase_m},
              {"width_m", value.width_m},
              {"front_overhang_m", value.front_overhang_m},
              {"rear_overhang_m", value.rear_overhang_m},
              {"max_steer_rad", value.max_steer_rad},
              {"safety_margin_m", value.safety_margin_m}};
}

VehicleModel vehicle_model_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  VehicleModel model;
  model.id = value_string(require_field(object, "id", path), path + ".id");
  model.wheelbase_m = value_double(require_field(object, "wheelbase_m", path), path + ".wheelbase_m");
  model.width_m = value_double(require_field(object, "width_m", path), path + ".width_m");
  model.front_overhang_m =
      value_double(require_field(object, "front_overhang_m", path), path + ".front_overhang_m");
  model.rear_overhang_m =
      value_double(require_field(object, "rear_overhang_m", path), path + ".rear_overhang_m");
  model.max_steer_rad = value_double(require_field(object, "max_steer_rad", path), path + ".max_steer_rad");
  model.safety_margin_m =
      value_double(require_field(object, "safety_margin_m", path), path + ".safety_margin_m");
  validate_vehicle_model(model, path);
  return model;
}

// ---------------------------------------------------------------------------
// GridView.
// ---------------------------------------------------------------------------

void validate_grid_view(const GridView& value, const std::string& path) {
  if (value.map_id.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".map_id: must not be empty");
  }
  if (value.map_version.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".map_version: must not be empty");
  }
  if (value.width < 0) {
    fail(ErrorCode::kInvalidValue, path + ".width: must be >= 0");
  }
  if (value.height < 0) {
    fail(ErrorCode::kInvalidValue, path + ".height: must be >= 0");
  }
  check_finite(value.resolution_m, path + ".resolution_m");
  if (!(value.resolution_m > 0.0)) {
    fail(ErrorCode::kInvalidValue, path + ".resolution_m: must be > 0");
  }
  check_pose_finite(value.origin, path + ".origin.");
  const std::int64_t expected = static_cast<std::int64_t>(value.width) * static_cast<std::int64_t>(value.height);
  if (expected != static_cast<std::int64_t>(value.cells.size())) {
    fail(ErrorCode::kInvalidValue,
         path + ".cells: length " + std::to_string(value.cells.size()) + " must equal width*height " +
             std::to_string(expected));
  }
  for (std::size_t i = 0; i < value.cells.size(); ++i) {
    const int cell = value.cells[i];
    if (cell != kCellFree && cell != kCellOccupied && cell != kCellUnknown) {
      fail(ErrorCode::kInvalidValue,
           indexed(path + ".cells", i) + ": must be 0 (free), 1 (occupied) or 2 (unknown)");
    }
  }
}

json grid_view_to_json(const GridView& value, const std::string& path) {
  validate_grid_view(value, path);
  json cells = json::array();
  for (int cell : value.cells) {
    cells.push_back(cell);
  }
  return json{{"map_id", value.map_id},
              {"map_version", value.map_version},
              {"width", value.width},
              {"height", value.height},
              {"resolution_m", value.resolution_m},
              {"origin", pose_to_json(value.origin, path + ".origin")},
              {"cells", cells}};
}

GridView grid_view_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  GridView grid;
  grid.map_id = value_string(require_field(object, "map_id", path), path + ".map_id");
  grid.map_version = value_string(require_field(object, "map_version", path), path + ".map_version");
  grid.width = value_int32(require_field(object, "width", path), path + ".width");
  grid.height = value_int32(require_field(object, "height", path), path + ".height");
  grid.resolution_m = value_double(require_field(object, "resolution_m", path), path + ".resolution_m");
  grid.origin = pose_from_json(require_field(object, "origin", path), path + ".origin");
  const json& cells = value_array(require_field(object, "cells", path), path + ".cells");
  grid.cells.reserve(cells.size());
  for (std::size_t i = 0; i < cells.size(); ++i) {
    grid.cells.push_back(value_int32(cells[i], indexed(path + ".cells", i)));
  }
  validate_grid_view(grid, path);
  return grid;
}

// ---------------------------------------------------------------------------
// PathPoint / PathSegment.
// ---------------------------------------------------------------------------

void validate_path_point(const PathPoint& value, const std::string& path) {
  check_pose_finite(value.pose, path + ".pose.");
  check_finite(value.s_m, path + ".s_m");
  if (value.s_m < 0.0) {
    fail(ErrorCode::kInvalidValue, path + ".s_m: must be non-negative");
  }
  check_finite(value.curvature_inv_m, path + ".curvature_inv_m");
  static_cast<void>(direction_name(value.direction));
}

json path_point_to_json(const PathPoint& value, const std::string& path) {
  validate_path_point(value, path);
  return json{{"pose", pose_to_json(value.pose, path + ".pose")},
              {"s_m", value.s_m},
              {"curvature_inv_m", value.curvature_inv_m},
              {"direction", direction_name(value.direction)}};
}

PathPoint path_point_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  PathPoint point;
  point.pose = pose_from_json(require_field(object, "pose", path), path + ".pose");
  point.s_m = value_double(require_field(object, "s_m", path), path + ".s_m");
  point.curvature_inv_m =
      value_double(require_field(object, "curvature_inv_m", path), path + ".curvature_inv_m");
  point.direction = direction_from_name(
      value_string(require_field(object, "direction", path), path + ".direction"), path + ".direction");
  validate_path_point(point, path);
  return point;
}

void validate_path_segment(const PathSegment& value, const std::string& path) {
  if (value.points.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".points: must contain at least one point");
  }
  const Direction first_direction = value.points.front().direction;
  for (std::size_t i = 0; i < value.points.size(); ++i) {
    const std::string point_path = indexed(path + ".points", i);
    validate_path_point(value.points[i], point_path);
    if (i > 0) {
      if (!(value.points[i].s_m > value.points[i - 1].s_m)) {
        fail(ErrorCode::kInvalidValue,
             point_path + ".s_m: must be strictly increasing within a segment");
      }
      if (value.points[i].direction != first_direction) {
        fail(ErrorCode::kInvalidValue,
             point_path + ".direction: must match the first point; direction changes start a new segment");
      }
    }
  }
}

json path_segment_to_json(const PathSegment& value, const std::string& path) {
  validate_path_segment(value, path);
  json points = json::array();
  for (std::size_t i = 0; i < value.points.size(); ++i) {
    points.push_back(path_point_to_json(value.points[i], indexed(path + ".points", i)));
  }
  return json{{"points", points}};
}

PathSegment path_segment_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  const json& points = value_array(require_field(object, "points", path), path + ".points");
  PathSegment segment;
  segment.points.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    segment.points.push_back(path_point_from_json(points[i], indexed(path + ".points", i)));
  }
  validate_path_segment(segment, path);
  return segment;
}

// ---------------------------------------------------------------------------
// TrajectoryPoint.
// ---------------------------------------------------------------------------

void validate_trajectory_point(const TrajectoryPoint& value, const std::string& path) {
  check_finite(value.t_s, path + ".t_s");
  if (value.t_s < 0.0) {
    fail(ErrorCode::kInvalidValue, path + ".t_s: must be non-negative");
  }
  check_pose_finite(value.pose, path + ".pose.");
  check_finite(value.velocity_mps, path + ".velocity_mps");
  check_finite(value.acceleration_mps2, path + ".acceleration_mps2");
  check_finite(value.curvature_inv_m, path + ".curvature_inv_m");
}

json trajectory_point_to_json(const TrajectoryPoint& value, const std::string& path) {
  validate_trajectory_point(value, path);
  return json{{"t_s", value.t_s},
              {"pose", pose_to_json(value.pose, path + ".pose")},
              {"velocity_mps", value.velocity_mps},
              {"acceleration_mps2", value.acceleration_mps2},
              {"curvature_inv_m", value.curvature_inv_m}};
}

TrajectoryPoint trajectory_point_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  TrajectoryPoint point;
  point.t_s = value_double(require_field(object, "t_s", path), path + ".t_s");
  point.pose = pose_from_json(require_field(object, "pose", path), path + ".pose");
  point.velocity_mps = value_double(require_field(object, "velocity_mps", path), path + ".velocity_mps");
  point.acceleration_mps2 =
      value_double(require_field(object, "acceleration_mps2", path), path + ".acceleration_mps2");
  point.curvature_inv_m =
      value_double(require_field(object, "curvature_inv_m", path), path + ".curvature_inv_m");
  validate_trajectory_point(point, path);
  return point;
}

// ---------------------------------------------------------------------------
// PlanRequest.
// ---------------------------------------------------------------------------

void validate_plan_request(const PlanRequest& value, const std::string& path) {
  if (value.request_id.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".request_id: must not be empty");
  }
  if (value.map_id.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".map_id: must not be empty");
  }
  if (value.map_version.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".map_version: must not be empty");
  }
  validate_vehicle_model(value.vehicle, path + ".vehicle");
  check_pose_finite(value.start, path + ".start.");
  check_pose_finite(value.goal, path + ".goal.");
  if (!value.config.is_object()) {
    fail(ErrorCode::kTypeError, path + ".config: must be a JSON object");
  }
  check_json_finite(value.config, path + ".config");
  if (value.time_budget_s.has_value()) {
    check_finite(*value.time_budget_s, path + ".time_budget_s");
    if (!(*value.time_budget_s > 0.0)) {
      fail(ErrorCode::kInvalidValue, path + ".time_budget_s: must be > 0");
    }
  }
  if (value.node_budget.has_value() && !(*value.node_budget > 0)) {
    fail(ErrorCode::kInvalidValue, path + ".node_budget: must be > 0");
  }
}

json plan_request_to_json(const PlanRequest& value, const std::string& path) {
  validate_plan_request(value, path);
  json request = json{{"request_id", value.request_id},
                      {"map_id", value.map_id},
                      {"map_version", value.map_version},
                      {"vehicle", vehicle_model_to_json(value.vehicle, path + ".vehicle")},
                      {"start", pose_to_json(value.start, path + ".start")},
                      {"goal", pose_to_json(value.goal, path + ".goal")},
                      {"config", value.config},
                      {"cancelled", value.cancelled}};
  if (value.time_budget_s.has_value()) {
    request["time_budget_s"] = *value.time_budget_s;
  }
  if (value.node_budget.has_value()) {
    request["node_budget"] = *value.node_budget;
  }
  return request;
}

PlanRequest plan_request_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  PlanRequest request;
  request.request_id = value_string(require_field(object, "request_id", path), path + ".request_id");
  request.map_id = value_string(require_field(object, "map_id", path), path + ".map_id");
  request.map_version = value_string(require_field(object, "map_version", path), path + ".map_version");
  request.vehicle = vehicle_model_from_json(require_field(object, "vehicle", path), path + ".vehicle");
  request.start = pose_from_json(require_field(object, "start", path), path + ".start");
  request.goal = pose_from_json(require_field(object, "goal", path), path + ".goal");
  if (const json* config = optional_field(object, "config")) {
    if (config->is_object()) {
      request.config = *config;
    } else {
      fail(ErrorCode::kTypeError, path + ".config: must be a JSON object");
    }
  } else {
    request.config = json::object();
  }
  if (const json* budget = optional_field(object, "time_budget_s")) {
    request.time_budget_s = value_double(*budget, path + ".time_budget_s");
  }
  if (const json* nodes = optional_field(object, "node_budget")) {
    request.node_budget = value_integer(*nodes, path + ".node_budget");
  }
  if (const json* cancelled = optional_field(object, "cancelled")) {
    request.cancelled = value_boolean(*cancelled, path + ".cancelled");
  }
  validate_plan_request(request, path);
  return request;
}

// ---------------------------------------------------------------------------
// PlanResult.
// ---------------------------------------------------------------------------

void validate_plan_result(const PlanResult& value, const std::string& path) {
  if (value.request_id.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".request_id: must not be empty");
  }
  static_cast<void>(status_name(value.status));
  for (std::size_t i = 0; i < value.path.size(); ++i) {
    validate_path_segment(value.path[i], indexed(path + ".path", i));
  }
  for (std::size_t i = 0; i < value.trajectory.size(); ++i) {
    validate_trajectory_point(value.trajectory[i], indexed(path + ".trajectory", i));
    if (i > 0 && !(value.trajectory[i].t_s > value.trajectory[i - 1].t_s)) {
      fail(ErrorCode::kInvalidValue,
           indexed(path + ".trajectory", i) + ".t_s: must be strictly increasing within a trajectory");
    }
  }
  if (!value.statistics.is_object()) {
    fail(ErrorCode::kTypeError, path + ".statistics: must be a JSON object");
  }
  check_json_finite(value.statistics, path + ".statistics");
}

json plan_result_to_json(const PlanResult& value, const std::string& path) {
  validate_plan_result(value, path);
  json path_json = json::array();
  for (std::size_t i = 0; i < value.path.size(); ++i) {
    path_json.push_back(path_segment_to_json(value.path[i], indexed(path + ".path", i)));
  }
  json trajectory_json = json::array();
  for (std::size_t i = 0; i < value.trajectory.size(); ++i) {
    trajectory_json.push_back(trajectory_point_to_json(value.trajectory[i], indexed(path + ".trajectory", i)));
  }
  return json{{"request_id", value.request_id},
              {"status", status_name(value.status)},
              {"reason", value.reason},
              {"path", path_json},
              {"trajectory", trajectory_json},
              {"statistics", value.statistics}};
}

PlanResult plan_result_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  PlanResult result;
  result.request_id = value_string(require_field(object, "request_id", path), path + ".request_id");
  result.status = status_from_name(
      value_string(require_field(object, "status", path), path + ".status"), path + ".status");
  if (const json* reason = optional_field(object, "reason")) {
    result.reason = value_string(*reason, path + ".reason");
  }
  if (const json* path_json = optional_field(object, "path")) {
    const json& segments = value_array(*path_json, path + ".path");
    result.path.reserve(segments.size());
    for (std::size_t i = 0; i < segments.size(); ++i) {
      result.path.push_back(path_segment_from_json(segments[i], indexed(path + ".path", i)));
    }
  }
  if (const json* trajectory_json = optional_field(object, "trajectory")) {
    const json& points = value_array(*trajectory_json, path + ".trajectory");
    result.trajectory.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      result.trajectory.push_back(trajectory_point_from_json(points[i], indexed(path + ".trajectory", i)));
    }
  }
  if (const json* statistics = optional_field(object, "statistics")) {
    if (statistics->is_object()) {
      result.statistics = *statistics;
    } else {
      fail(ErrorCode::kTypeError, path + ".statistics: must be a JSON object");
    }
  } else {
    result.statistics = json::object();
  }
  validate_plan_result(result, path);
  return result;
}

// ---------------------------------------------------------------------------
// Corridor.
// ---------------------------------------------------------------------------

void validate_corridor_polygon(const CorridorPolygon& value, const std::string& path) {
  check_finite(value.s_start_m, path + ".s_start_m");
  if (value.s_start_m < 0.0) {
    fail(ErrorCode::kInvalidValue, path + ".s_start_m: must be non-negative");
  }
  check_finite(value.s_end_m, path + ".s_end_m");
  if (!(value.s_end_m > value.s_start_m)) {
    fail(ErrorCode::kInvalidValue, path + ".s_end_m: must be > s_start_m");
  }
  if (value.vertices.size() < 3) {
    fail(ErrorCode::kInvalidValue,
         path + ".vertices: a convex polygon needs at least 3 vertices (got " +
             std::to_string(value.vertices.size()) + ")");
  }
  for (std::size_t i = 0; i < value.vertices.size(); ++i) {
    check_finite(value.vertices[i].x_m, indexed(path + ".vertices", i) + ".x_m");
    check_finite(value.vertices[i].y_m, indexed(path + ".vertices", i) + ".y_m");
  }
}

void validate_corridor(const Corridor& value, const std::string& path) {
  if (value.id.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".id: must not be empty");
  }
  if (value.map_id.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".map_id: must not be empty");
  }
  if (value.map_version.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".map_version: must not be empty");
  }
  if (value.frame_id.empty()) {
    fail(ErrorCode::kInvalidValue, path + ".frame_id: must not be empty");
  }
  double previous_start = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < value.polygons.size(); ++i) {
    validate_corridor_polygon(value.polygons[i], indexed(path + ".polygons", i));
    if (value.polygons[i].s_start_m < previous_start) {
      fail(ErrorCode::kInvalidValue,
           indexed(path + ".polygons", i) + ".s_start_m: polygons must be ordered by non-decreasing s_start_m");
    }
    previous_start = value.polygons[i].s_start_m;
  }
}

json corridor_polygon_to_json(const CorridorPolygon& value, const std::string& path) {
  validate_corridor_polygon(value, path);
  json vertices = json::array();
  for (std::size_t i = 0; i < value.vertices.size(); ++i) {
    vertices.push_back(point2d_to_json(value.vertices[i], indexed(path + ".vertices", i)));
  }
  return json{{"s_start_m", value.s_start_m}, {"s_end_m", value.s_end_m}, {"vertices", vertices}};
}

CorridorPolygon corridor_polygon_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  CorridorPolygon polygon;
  polygon.s_start_m = value_double(require_field(object, "s_start_m", path), path + ".s_start_m");
  polygon.s_end_m = value_double(require_field(object, "s_end_m", path), path + ".s_end_m");
  const json& vertices = value_array(require_field(object, "vertices", path), path + ".vertices");
  polygon.vertices.reserve(vertices.size());
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    polygon.vertices.push_back(point2d_from_json(vertices[i], indexed(path + ".vertices", i)));
  }
  validate_corridor_polygon(polygon, path);
  return polygon;
}

json corridor_to_json(const Corridor& value, const std::string& path) {
  validate_corridor(value, path);
  json polygons = json::array();
  for (std::size_t i = 0; i < value.polygons.size(); ++i) {
    polygons.push_back(corridor_polygon_to_json(value.polygons[i], indexed(path + ".polygons", i)));
  }
  return json{{"id", value.id},
              {"map_id", value.map_id},
              {"map_version", value.map_version},
              {"frame_id", value.frame_id},
              {"polygons", polygons}};
}

Corridor corridor_from_json(const json& payload, const std::string& path) {
  const json& object = value_object(payload, path);
  Corridor corridor;
  corridor.id = value_string(require_field(object, "id", path), path + ".id");
  corridor.map_id = value_string(require_field(object, "map_id", path), path + ".map_id");
  corridor.map_version = value_string(require_field(object, "map_version", path), path + ".map_version");
  corridor.frame_id = value_string(require_field(object, "frame_id", path), path + ".frame_id");
  const json& polygons = value_array(require_field(object, "polygons", path), path + ".polygons");
  corridor.polygons.reserve(polygons.size());
  for (std::size_t i = 0; i < polygons.size(); ++i) {
    corridor.polygons.push_back(corridor_polygon_from_json(polygons[i], indexed(path + ".polygons", i)));
  }
  validate_corridor(corridor, path);
  return corridor;
}

}  // namespace

// ---------------------------------------------------------------------------
// 严格枚举解析（在公开头文件中声明）。
// ---------------------------------------------------------------------------

Direction direction_from_string(std::string_view text) {
  return direction_from_name(std::string(text), "direction");
}

Status status_from_string(std::string_view text) {
  return status_from_name(std::string(text), "status");
}

// ---------------------------------------------------------------------------
// 编解码器特化。每个特化都用与编码器相同的规则校验其载荷；
// 契约见测试夹具 README。
// ---------------------------------------------------------------------------

template <>
struct json_codec<Pose2d> {
  static constexpr const char* kTypeName = "pose2d";
  static nlohmann::json to_json(const Pose2d& value) { return pose_to_json(value, "pose2d"); }
  static Pose2d from_json(const nlohmann::json& payload) { return pose_from_json(payload, "pose2d"); }
};

template <>
struct json_codec<VehicleModel> {
  static constexpr const char* kTypeName = "vehicle_model";
  static nlohmann::json to_json(const VehicleModel& value) {
    return vehicle_model_to_json(value, "vehicle_model");
  }
  static VehicleModel from_json(const nlohmann::json& payload) {
    return vehicle_model_from_json(payload, "vehicle_model");
  }
};

template <>
struct json_codec<GridView> {
  static constexpr const char* kTypeName = "grid_view";
  static nlohmann::json to_json(const GridView& value) { return grid_view_to_json(value, "grid_view"); }
  static GridView from_json(const nlohmann::json& payload) {
    return grid_view_from_json(payload, "grid_view");
  }
};

template <>
struct json_codec<PathPoint> {
  static constexpr const char* kTypeName = "path_point";
  static nlohmann::json to_json(const PathPoint& value) { return path_point_to_json(value, "path_point"); }
  static PathPoint from_json(const nlohmann::json& payload) {
    return path_point_from_json(payload, "path_point");
  }
};

template <>
struct json_codec<PathSegment> {
  static constexpr const char* kTypeName = "path_segment";
  static nlohmann::json to_json(const PathSegment& value) {
    return path_segment_to_json(value, "path_segment");
  }
  static PathSegment from_json(const nlohmann::json& payload) {
    return path_segment_from_json(payload, "path_segment");
  }
};

template <>
struct json_codec<TrajectoryPoint> {
  static constexpr const char* kTypeName = "trajectory_point";
  static nlohmann::json to_json(const TrajectoryPoint& value) {
    return trajectory_point_to_json(value, "trajectory_point");
  }
  static TrajectoryPoint from_json(const nlohmann::json& payload) {
    return trajectory_point_from_json(payload, "trajectory_point");
  }
};

template <>
struct json_codec<PlanRequest> {
  static constexpr const char* kTypeName = "plan_request";
  static nlohmann::json to_json(const PlanRequest& value) {
    return plan_request_to_json(value, "plan_request");
  }
  static PlanRequest from_json(const nlohmann::json& payload) {
    return plan_request_from_json(payload, "plan_request");
  }
};

template <>
struct json_codec<PlanResult> {
  static constexpr const char* kTypeName = "plan_result";
  static nlohmann::json to_json(const PlanResult& value) {
    return plan_result_to_json(value, "plan_result");
  }
  static PlanResult from_json(const nlohmann::json& payload) {
    return plan_result_from_json(payload, "plan_result");
  }
};

template <>
struct json_codec<Status> {
  static constexpr const char* kTypeName = "status";
  static nlohmann::json to_json(const Status& value) {
    return nlohmann::json{{"status", status_name(value)}};
  }
  static Status from_json(const nlohmann::json& payload) {
    const nlohmann::json& object = value_object(payload, "status");
    return status_from_name(
        value_string(require_field(object, "status", "status"), "status.status"), "status.status");
  }
};

template <>
struct json_codec<Corridor> {
  static constexpr const char* kTypeName = "corridor";
  static nlohmann::json to_json(const Corridor& value) { return corridor_to_json(value, "corridor"); }
  static Corridor from_json(const nlohmann::json& payload) {
    return corridor_from_json(payload, "corridor");
  }
};

// ---------------------------------------------------------------------------
// 信封辅助函数。
// ---------------------------------------------------------------------------

template <typename T>
nlohmann::json encode_payload(const T& value) {
  return json_codec<T>::to_json(value);
}

template <typename T>
T decode_payload(const nlohmann::json& payload) {
  return json_codec<T>::from_json(payload);
}

template <typename T>
std::string encode_document(const T& value, int indent) {
  nlohmann::json document = json_codec<T>::to_json(value);
  if (!document.is_object()) {
    fail(ErrorCode::kInvalidValue, "document: payload must be a JSON object");
  }
  document["schema_version"] = kSchemaVersion;
  document["type"] = json_codec<T>::kTypeName;
  try {
    return document.dump(indent);
  } catch (const nlohmann::json::exception& error) {
    fail(ErrorCode::kInvalidValue, std::string("document: cannot serialize to UTF-8 JSON: ") + error.what());
  }
}

template <typename T>
T decode_document(std::string_view utf8_text) {
  nlohmann::json root;
  try {
    root = nlohmann::json::parse(utf8_text.begin(), utf8_text.end());
  } catch (const nlohmann::json::out_of_range& error) {
    // nlohmann/json 对使 double 溢出的数字字面量（如 1e999）
    // 在 parse 时抛出 out_of_range 来报告。
    fail(ErrorCode::kNonFiniteValue,
         std::string("document: number is not representable as a finite double: ") + error.what());
  } catch (const nlohmann::json::parse_error& error) {
    fail(ErrorCode::kParseError, std::string("document: ") + error.what());
  } catch (const nlohmann::json::exception& error) {
    fail(ErrorCode::kParseError, std::string("document: ") + error.what());
  }
  if (!root.is_object()) {
    fail(ErrorCode::kInvalidDocument, "document: root must be a JSON object");
  }
  const auto version_it = root.find("schema_version");
  if (version_it == root.end()) {
    fail(ErrorCode::kMissingField, "document.schema_version: missing required field");
  }
  if (!version_it->is_number_integer()) {
    fail(ErrorCode::kTypeError, "document.schema_version: must be a JSON integer");
  }
  const std::int64_t version = version_it->get<std::int64_t>();
  if (version != static_cast<std::int64_t>(kSchemaVersion)) {
    fail(ErrorCode::kUnsupportedSchemaVersion,
         "document.schema_version: unsupported major version " + std::to_string(version) +
             " (this build supports 1)");
  }
  const auto type_it = root.find("type");
  if (type_it == root.end()) {
    fail(ErrorCode::kMissingField, "document.type: missing required field");
  }
  if (!type_it->is_string()) {
    fail(ErrorCode::kTypeError, "document.type: must be a JSON string");
  }
  const std::string type = type_it->get<std::string>();
  if (type != json_codec<T>::kTypeName) {
    fail(ErrorCode::kUnknownType,
         "document.type: '" + type + "' cannot be decoded as " + json_codec<T>::kTypeName);
  }
  return json_codec<T>::from_json(root);
}

// ---------------------------------------------------------------------------
// 显式实例化：上面的泛型模板只对本文件列出的第一版公开类型可编译
// （因此 serialize.cpp 必须随契约一起更新）。
// ---------------------------------------------------------------------------

#define PLANNER_COMMON_INSTANTIATE(TYPE)                                  \
  template nlohmann::json encode_payload<TYPE>(const TYPE&);              \
  template TYPE decode_payload<TYPE>(const nlohmann::json&);              \
  template std::string encode_document<TYPE>(const TYPE&, int);           \
  template TYPE decode_document<TYPE>(std::string_view);

PLANNER_COMMON_INSTANTIATE(Pose2d)
PLANNER_COMMON_INSTANTIATE(VehicleModel)
PLANNER_COMMON_INSTANTIATE(GridView)
PLANNER_COMMON_INSTANTIATE(PathPoint)
PLANNER_COMMON_INSTANTIATE(PathSegment)
PLANNER_COMMON_INSTANTIATE(TrajectoryPoint)
PLANNER_COMMON_INSTANTIATE(PlanRequest)
PLANNER_COMMON_INSTANTIATE(PlanResult)
PLANNER_COMMON_INSTANTIATE(Status)
PLANNER_COMMON_INSTANTIATE(Corridor)

#undef PLANNER_COMMON_INSTANTIATE

}  // namespace planner::common
