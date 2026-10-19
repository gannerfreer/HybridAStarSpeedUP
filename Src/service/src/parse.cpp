// POST /plan/submit 的严格请求解析器（规则见 parse.hpp）。

#include "planner/service/parse.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "planner/collision/scene.hpp"
#include "planner/service/pipeline.hpp"

namespace planner::service {
namespace {

using nlohmann::json;

bool finite_number(const json& value) {
  return value.is_number() && std::isfinite(value.get<double>());
}

ParseOutcome bad_field(const std::string& path, const char* expected) {
  ParseOutcome outcome = ParseOutcome::failure("invalid_field");
  outcome.detail = path + " must be " + expected;
  return outcome;
}

// ---------------------------------------------------------------------------
// 基础字段读取器。
// ---------------------------------------------------------------------------

bool read_string(const json& object, const char* key, bool allow_empty,
                 std::string* out, ParseOutcome* outcome) {
  if (!object.contains(key)) {
    *outcome = ParseOutcome::failure("missing_field");
    outcome->detail = std::string("missing field '") + key + "'";
    return false;
  }
  const json& value = object.at(key);
  if (!value.is_string()) {
    *outcome = bad_field(key, "a string");
    return false;
  }
  *out = value.get<std::string>();
  if (!allow_empty && out->empty()) {
    *outcome = bad_field(key, "a non-empty string");
    return false;
  }
  return true;
}

bool read_finite(const json& object, const char* key, double* out,
                 ParseOutcome* outcome) {
  if (!object.contains(key)) {
    *outcome = ParseOutcome::failure("missing_field");
    outcome->detail = std::string("missing field '") + key + "'";
    return false;
  }
  if (!finite_number(object.at(key))) {
    *outcome = bad_field(key, "a finite number");
    return false;
  }
  *out = object.at(key).get<double>();
  return true;
}

bool read_positive(const json& object, const char* key, double* out,
                   ParseOutcome* outcome) {
  if (!read_finite(object, key, out, outcome)) {
    return false;
  }
  if (!(*out > 0.0)) {
    *outcome = bad_field(key, "a positive number");
    return false;
  }
  return true;
}

bool read_non_negative(const json& object, const char* key, double* out,
                       ParseOutcome* outcome) {
  if (!read_finite(object, key, out, outcome)) {
    return false;
  }
  if (*out < 0.0) {
    *outcome = bad_field(key, "a non-negative number");
    return false;
  }
  return true;
}

bool read_int_value(const json& object, const char* key, std::int64_t minimum,
                    std::int64_t* out, ParseOutcome* outcome) {
  if (!object.contains(key)) {
    *outcome = ParseOutcome::failure("missing_field");
    outcome->detail = std::string("missing field '") + key + "'";
    return false;
  }
  const json& value = object.at(key);
  if (!value.is_number() || !std::isfinite(value.get<double>())) {
    *outcome = bad_field(key, "an integer");
    return false;
  }
  const double as_double = value.get<double>();
  if (std::floor(as_double) != as_double ||
      as_double > static_cast<double>(std::numeric_limits<std::int64_t>::max()) ||
      as_double < static_cast<double>(minimum)) {
    *outcome = bad_field(key, "an integer within range");
    return false;
  }
  *out = static_cast<std::int64_t>(as_double);
  return true;
}

bool read_positive_int(const json& object, const char* key, std::int64_t* out,
                       ParseOutcome* outcome) {
  return read_int_value(object, key, 1, out, outcome);
}

bool read_bool(const json& object, const char* key, bool* out,
               ParseOutcome* outcome) {
  if (!object.contains(key)) {
    *outcome = ParseOutcome::failure("missing_field");
    outcome->detail = std::string("missing field '") + key + "'";
    return false;
  }
  if (!object.at(key).is_boolean()) {
    *outcome = bad_field(key, "a boolean");
    return false;
  }
  *out = object.at(key).get<bool>();
  return true;
}

bool read_pose(const json& value, const char* path, common::Pose2d* out,
               ParseOutcome* outcome) {
  if (!value.is_object()) {
    *outcome = bad_field(path, "an object with x_m, y_m and yaw_rad");
    return false;
  }
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  if (!read_finite(value, "x_m", &x_m, outcome) ||
      !read_finite(value, "y_m", &y_m, outcome) ||
      !read_finite(value, "yaw_rad", &yaw_rad, outcome)) {
    outcome->detail = std::string(path) + "." + outcome->detail;
    return false;
  }
  out->x_m = x_m;
  out->y_m = y_m;
  out->yaw_rad = common::normalize_angle_rad(yaw_rad);
  return true;
}

// ---------------------------------------------------------------------------
// 车辆。
// ---------------------------------------------------------------------------

ParseOutcome validate_vehicle(const common::VehicleModel& vehicle) {
  std::string detail;
  if (!collision::vehicle_footprint_valid(vehicle, &detail)) {
    ParseOutcome outcome = ParseOutcome::failure("invalid_vehicle");
    outcome.detail = detail;
    return outcome;
  }
  if (!std::isfinite(vehicle.max_steer_rad) || !(vehicle.max_steer_rad > 0.0) ||
      !(vehicle.max_steer_rad < common::kPi / 2.0)) {
    ParseOutcome outcome = ParseOutcome::failure("invalid_vehicle");
    outcome.detail = "max_steer_rad must be finite and in (0, pi/2)";
    return outcome;
  }
  return ParseOutcome::success();
}

bool read_vehicle_fields(const json& value, common::VehicleModel* out,
                         ParseOutcome* outcome) {
  if (!value.is_object()) {
    *outcome = bad_field("vehicle", "an object");
    return false;
  }
  if (value.contains("id")) {
    if (!value.at("id").is_string()) {
      *outcome = bad_field("vehicle.id", "a string");
      return false;
    }
    out->id = value.at("id").get<std::string>();
  }
  if (!read_finite(value, "wheelbase_m", &out->wheelbase_m, outcome) ||
      !read_finite(value, "width_m", &out->width_m, outcome) ||
      !read_finite(value, "front_overhang_m", &out->front_overhang_m, outcome) ||
      !read_finite(value, "rear_overhang_m", &out->rear_overhang_m, outcome) ||
      !read_finite(value, "max_steer_rad", &out->max_steer_rad, outcome) ||
      !read_finite(value, "safety_margin_m", &out->safety_margin_m, outcome)) {
    outcome->detail = std::string("vehicle.") + outcome->detail;
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// 算法参数（algorithm_params）。
// ---------------------------------------------------------------------------

ParseOutcome unknown_parameter(const std::string& path, const std::string& key) {
  ParseOutcome outcome = ParseOutcome::failure("unknown_algorithm_parameter");
  outcome.detail = path + " key '" + key + "' is not part of the W5 contract";
  return outcome;
}

ParseOutcome parse_corridor_options(const json& value, corridor::CorridorOptions* out) {
  if (!value.is_object()) {
    return bad_field("algorithm_params.corridor", "an object");
  }
  static const std::set<std::string> kAllowed{
      "sample_step_m",      "construction_step_m",     "verification_step_m",
      "clearance_m",        "max_expansion_m",         "perturbation_yaw_rad",
      "perturbation_position_m"};
  ParseOutcome outcome;
  for (const auto& item : value.items()) {
    if (kAllowed.find(item.key()) == kAllowed.end()) {
      return unknown_parameter("algorithm_params.corridor", item.key());
    }
  }
  if (value.contains("sample_step_m") &&
      !read_positive(value, "sample_step_m", &out->sample_step_m, &outcome)) {
    return outcome;
  }
  if (value.contains("construction_step_m") &&
      !read_positive(value, "construction_step_m", &out->construction_step_m, &outcome)) {
    return outcome;
  }
  if (value.contains("verification_step_m") &&
      !read_positive(value, "verification_step_m", &out->verification_step_m, &outcome)) {
    return outcome;
  }
  if (value.contains("clearance_m") &&
      !read_non_negative(value, "clearance_m", &out->clearance_m, &outcome)) {
    return outcome;
  }
  if (value.contains("max_expansion_m") &&
      !read_non_negative(value, "max_expansion_m", &out->max_expansion_m, &outcome)) {
    return outcome;
  }
  if (value.contains("perturbation_yaw_rad") &&
      !read_non_negative(value, "perturbation_yaw_rad", &out->perturbation_yaw_rad, &outcome)) {
    return outcome;
  }
  if (value.contains("perturbation_position_m") &&
      !read_non_negative(value, "perturbation_position_m", &out->perturbation_position_m,
                         &outcome)) {
    return outcome;
  }
  return ParseOutcome::success();
}

ParseOutcome parse_dftpav_limits(const json& value, const common::VehicleModel& vehicle,
                                 dftpav::OptimizeLimits* out) {
  if (!value.is_object()) {
    return bad_field("algorithm_params.dftpav.limits", "an object");
  }
  static const std::set<std::string> kAllowed{
      "max_forward_velocity_mps",     "max_backward_velocity_mps",
      "max_forward_acceleration_mps2", "max_backward_acceleration_mps2",
      "max_forward_curvature_inv_m",  "max_backward_curvature_inv_m",
      "max_lateral_acceleration_mps2", "max_steering_rate_radps"};
  ParseOutcome outcome;
  for (const auto& item : value.items()) {
    if (kAllowed.find(item.key()) == kAllowed.end()) {
      return unknown_parameter("algorithm_params.dftpav.limits", item.key());
    }
  }
  dftpav::OptimizeLimits limits = limits_for_vehicle(vehicle);
  if (value.contains("max_forward_velocity_mps") &&
      !read_positive(value, "max_forward_velocity_mps", &limits.max_forward_velocity_mps,
                     &outcome)) {
    return outcome;
  }
  if (value.contains("max_backward_velocity_mps") &&
      !read_positive(value, "max_backward_velocity_mps", &limits.max_backward_velocity_mps,
                     &outcome)) {
    return outcome;
  }
  if (value.contains("max_forward_acceleration_mps2") &&
      !read_positive(value, "max_forward_acceleration_mps2",
                     &limits.max_forward_acceleration_mps2, &outcome)) {
    return outcome;
  }
  if (value.contains("max_backward_acceleration_mps2") &&
      !read_positive(value, "max_backward_acceleration_mps2",
                     &limits.max_backward_acceleration_mps2, &outcome)) {
    return outcome;
  }
  if (value.contains("max_forward_curvature_inv_m") &&
      !read_positive(value, "max_forward_curvature_inv_m",
                     &limits.max_forward_curvature_inv_m, &outcome)) {
    return outcome;
  }
  if (value.contains("max_backward_curvature_inv_m") &&
      !read_positive(value, "max_backward_curvature_inv_m",
                     &limits.max_backward_curvature_inv_m, &outcome)) {
    return outcome;
  }
  if (value.contains("max_lateral_acceleration_mps2") &&
      !read_positive(value, "max_lateral_acceleration_mps2",
                     &limits.max_lateral_acceleration_mps2, &outcome)) {
    return outcome;
  }
  if (value.contains("max_steering_rate_radps") &&
      !read_positive(value, "max_steering_rate_radps", &limits.max_steering_rate_radps,
                     &outcome)) {
    return outcome;
  }
  *out = limits;
  return ParseOutcome::success();
}

ParseOutcome parse_dftpav_options(const json& value, dftpav::OptimizeOptions* out) {
  if (!value.is_object()) {
    return bad_field("algorithm_params.dftpav.options", "an object");
  }
  static const std::set<std::string> kAllowed{
      "traj_resolution",        "destraj_resolution",
      "static_obstacle_weight", "feasibility_weight",
      "time_weight",            "hull_margin_m",
      "traj_piece_duration_s",  "min_piece_duration_s",
      "lbfgs_memsize",          "lbfgs_past",
      "lbfgs_delta",            "lbfgs_max_iterations",
      "cost_fail_threshold",    "anchor_weight",
      "direction_switch_velocity_mps", "sample_dt_s",
      "constraint_tolerance",   "curvature_report_min_speed_mps",
      "heading_report_min_speed_mps", "provisional_checks",
      "corridor_tightening_m",  "corridor_repair"};
  ParseOutcome outcome;
  for (const auto& item : value.items()) {
    if (kAllowed.find(item.key()) == kAllowed.end()) {
      return unknown_parameter("algorithm_params.dftpav.options", item.key());
    }
  }
  auto positive_int = [&](const char* key, int* target) -> bool {
    std::int64_t parsed = 0;
    if (!read_positive_int(value, key, &parsed, &outcome)) {
      return false;
    }
    if (parsed > std::numeric_limits<int>::max()) {
      outcome = bad_field(key, "an integer within int range");
      return false;
    }
    *target = static_cast<int>(parsed);
    return true;
  };
  if (value.contains("traj_resolution") && !positive_int("traj_resolution", &out->traj_resolution)) {
    return outcome;
  }
  if (value.contains("destraj_resolution") &&
      !positive_int("destraj_resolution", &out->destraj_resolution)) {
    return outcome;
  }
  if (value.contains("static_obstacle_weight") &&
      !read_non_negative(value, "static_obstacle_weight", &out->static_obstacle_weight, &outcome)) {
    return outcome;
  }
  if (value.contains("feasibility_weight") &&
      !read_non_negative(value, "feasibility_weight", &out->feasibility_weight, &outcome)) {
    return outcome;
  }
  if (value.contains("time_weight") &&
      !read_non_negative(value, "time_weight", &out->time_weight, &outcome)) {
    return outcome;
  }
  if (value.contains("hull_margin_m")) {
    double margin = 0.0;
    if (!read_non_negative(value, "hull_margin_m", &margin, &outcome)) {
      return outcome;
    }
    out->hull_margin_m = margin;
  }
  if (value.contains("traj_piece_duration_s") &&
      !read_positive(value, "traj_piece_duration_s", &out->traj_piece_duration_s, &outcome)) {
    return outcome;
  }
  if (value.contains("min_piece_duration_s") &&
      !read_positive(value, "min_piece_duration_s", &out->min_piece_duration_s, &outcome)) {
    return outcome;
  }
  if (value.contains("lbfgs_memsize") && !positive_int("lbfgs_memsize", &out->lbfgs_memsize)) {
    return outcome;
  }
  if (value.contains("lbfgs_past")) {
    std::int64_t parsed = 0;
    if (!read_int_value(value, "lbfgs_past", 0, &parsed, &outcome) ||
        parsed > std::numeric_limits<int>::max()) {
      outcome = bad_field("lbfgs_past", "an integer within int range");
      return outcome;
    }
    out->lbfgs_past = static_cast<int>(parsed);
  }
  if (value.contains("lbfgs_delta") &&
      !read_positive(value, "lbfgs_delta", &out->lbfgs_delta, &outcome)) {
    return outcome;
  }
  if (value.contains("lbfgs_max_iterations") &&
      !positive_int("lbfgs_max_iterations", &out->lbfgs_max_iterations)) {
    return outcome;
  }
  if (value.contains("cost_fail_threshold") &&
      !read_positive(value, "cost_fail_threshold", &out->cost_fail_threshold, &outcome)) {
    return outcome;
  }
  if (value.contains("anchor_weight") &&
      !read_non_negative(value, "anchor_weight", &out->anchor_weight, &outcome)) {
    return outcome;
  }
  if (value.contains("direction_switch_velocity_mps") &&
      !read_non_negative(value, "direction_switch_velocity_mps",
                         &out->direction_switch_velocity_mps, &outcome)) {
    return outcome;
  }
  if (value.contains("sample_dt_s") &&
      !read_positive(value, "sample_dt_s", &out->sample_dt_s, &outcome)) {
    return outcome;
  }
  if (value.contains("constraint_tolerance") &&
      !read_positive(value, "constraint_tolerance", &out->constraint_tolerance, &outcome)) {
    return outcome;
  }
  if (value.contains("curvature_report_min_speed_mps") &&
      !read_non_negative(value, "curvature_report_min_speed_mps",
                         &out->curvature_report_min_speed_mps, &outcome)) {
    return outcome;
  }
  if (value.contains("heading_report_min_speed_mps") &&
      !read_non_negative(value, "heading_report_min_speed_mps",
                         &out->heading_report_min_speed_mps, &outcome)) {
    return outcome;
  }
  if (value.contains("provisional_checks") &&
      !read_bool(value, "provisional_checks", &out->provisional_checks, &outcome)) {
    return outcome;
  }
  if (value.contains("corridor_tightening_m") &&
      !read_non_negative(value, "corridor_tightening_m", &out->corridor_tightening_m, &outcome)) {
    return outcome;
  }
  if (value.contains("corridor_repair") &&
      !read_bool(value, "corridor_repair", &out->corridor_repair, &outcome)) {
    return outcome;
  }
  return ParseOutcome::success();
}

ParseOutcome parse_algorithm_params(const json& value, const common::VehicleModel& vehicle,
                                    PipelineParams* out) {
  if (!value.is_object()) {
    return bad_field("algorithm_params", "an object");
  }
  static const std::set<std::string> kAllowed{
      "hybrid_backend", "hybrid", "time_budget_s", "node_budget",
      "dftpav_time_budget_s", "corridor", "dftpav"};
  ParseOutcome outcome;
  for (const auto& item : value.items()) {
    if (kAllowed.find(item.key()) == kAllowed.end()) {
      return unknown_parameter("algorithm_params", item.key());
    }
  }
  if (value.contains("hybrid_backend")) {
    if (!read_string(value, "hybrid_backend", false, &out->hybrid_backend, &outcome)) {
      return outcome;
    }
  }
  if (value.contains("hybrid")) {
    if (!value.at("hybrid").is_object()) {
      return bad_field("algorithm_params.hybrid", "an object of backend configure keys");
    }
    out->hybrid_config = value.at("hybrid");
  }
  if (value.contains("time_budget_s") &&
      !read_positive(value, "time_budget_s", &out->hybrid_time_budget_s, &outcome)) {
    return outcome;
  }
  if (value.contains("node_budget") &&
      !read_positive_int(value, "node_budget", &out->hybrid_node_budget, &outcome)) {
    return outcome;
  }
  if (value.contains("dftpav_time_budget_s") &&
      !read_positive(value, "dftpav_time_budget_s", &out->dftpav_time_budget_s, &outcome)) {
    return outcome;
  }
  if (value.contains("corridor")) {
    const ParseOutcome parsed = parse_corridor_options(value.at("corridor"), &out->corridor);
    if (!parsed.ok) {
      return parsed;
    }
  }
  if (value.contains("dftpav")) {
    const json& dftpav = value.at("dftpav");
    if (!dftpav.is_object()) {
      return bad_field("algorithm_params.dftpav", "an object");
    }
    static const std::set<std::string> kDftpavKeys{"time_budget_s", "limits", "options"};
    for (const auto& item : dftpav.items()) {
      if (kDftpavKeys.find(item.key()) == kDftpavKeys.end()) {
        return unknown_parameter("algorithm_params.dftpav", item.key());
      }
    }
    if (dftpav.contains("time_budget_s") &&
        !read_positive(dftpav, "time_budget_s", &out->dftpav_time_budget_s, &outcome)) {
      return outcome;
    }
    if (dftpav.contains("limits")) {
      dftpav::OptimizeLimits limits;
      const ParseOutcome parsed =
          parse_dftpav_limits(dftpav.at("limits"), vehicle, &limits);
      if (!parsed.ok) {
        return parsed;
      }
      out->dftpav_limits = limits;
    }
    if (dftpav.contains("options")) {
      const ParseOutcome parsed = parse_dftpav_options(dftpav.at("options"), &out->dftpav_options);
      if (!parsed.ok) {
        return parsed;
      }
    }
  }
  return ParseOutcome::success();
}

}  // namespace

// ---------------------------------------------------------------------------
// 公开入口点。
// ---------------------------------------------------------------------------

ParseOutcome parse_obstacle(const json& value, ServiceObstacle* out) {
  if (!value.is_object()) {
    return bad_field("obstacles[]", "an object");
  }
  ParseOutcome outcome;
  ServiceObstacle obstacle;
  if (!read_string(value, "id", false, &obstacle.id, &outcome)) {
    outcome.detail = std::string("obstacles[].id: ") + outcome.detail;
    return outcome;
  }
  std::string type;
  if (!read_string(value, "type", false, &type, &outcome)) {
    outcome.detail = std::string("obstacles[].") + outcome.detail;
    return outcome;
  }
  if (type == "rectangle") {
    obstacle.kind = ServiceObstacle::Kind::kRectangle;
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    if (!read_finite(value, "x_min_m", &x0, &outcome) ||
        !read_finite(value, "y_min_m", &y0, &outcome) ||
        !read_finite(value, "x_max_m", &x1, &outcome) ||
        !read_finite(value, "y_max_m", &y1, &outcome)) {
      outcome.detail = std::string("obstacles[]: ") + outcome.detail;
      return outcome;
    }
    obstacle.x_min_m = std::min(x0, x1);
    obstacle.x_max_m = std::max(x0, x1);
    obstacle.y_min_m = std::min(y0, y1);
    obstacle.y_max_m = std::max(y0, y1);
    if (!(obstacle.x_min_m < obstacle.x_max_m) || !(obstacle.y_min_m < obstacle.y_max_m)) {
      ParseOutcome failure = ParseOutcome::failure("invalid_obstacle");
      failure.detail = "obstacle '" + obstacle.id + "' is a degenerate rectangle";
      return failure;
    }
  } else if (type == "convex_polygon") {
    obstacle.kind = ServiceObstacle::Kind::kConvexPolygon;
    if (!value.contains("vertices") || !value.at("vertices").is_array()) {
      return bad_field("obstacles[].vertices", "an array of {x_m, y_m}");
    }
    const json& vertices = value.at("vertices");
    if (vertices.size() < 3) {
      ParseOutcome failure = ParseOutcome::failure("invalid_obstacle");
      failure.detail = "obstacle '" + obstacle.id + "' needs at least 3 vertices";
      return failure;
    }
    for (const json& vertex : vertices) {
      if (!vertex.is_object()) {
        return bad_field("obstacles[].vertices[]", "an object with x_m and y_m");
      }
      double x_m = 0.0;
      double y_m = 0.0;
      if (!read_finite(vertex, "x_m", &x_m, &outcome) ||
          !read_finite(vertex, "y_m", &y_m, &outcome)) {
        outcome.detail = std::string("obstacles[].vertices[]: ") + outcome.detail;
        return outcome;
      }
      obstacle.vertices.push_back(common::Point2d{x_m, y_m});
    }
    std::string reason;
    const collision::Obstacle validated =
        collision::Obstacle::convex_polygon(obstacle.id, obstacle.vertices);
    if (!validated.valid(&reason)) {
      ParseOutcome failure = ParseOutcome::failure("invalid_obstacle");
      failure.detail = "obstacle '" + obstacle.id + "': " + reason;
      return failure;
    }
  } else {
    ParseOutcome failure = ParseOutcome::failure("invalid_obstacle");
    failure.detail = "obstacle '" + obstacle.id + "' has unknown type '" + type +
                     "' (expected rectangle or convex_polygon)";
    return failure;
  }
  *out = std::move(obstacle);
  return ParseOutcome::success();
}

ParseOutcome parse_vehicle(const json& value, common::VehicleModel* out) {
  ParseOutcome outcome;
  common::VehicleModel vehicle = default_vehicle();
  vehicle.id.clear();
  if (!read_vehicle_fields(value, &vehicle, &outcome)) {
    return outcome;
  }
  const ParseOutcome validated = validate_vehicle(vehicle);
  if (!validated.ok) {
    return validated;
  }
  *out = std::move(vehicle);
  return ParseOutcome::success();
}

ParseOutcome load_vehicle_file(const std::string& path, common::VehicleModel* out) {
  std::ifstream stream(path);
  if (!stream) {
    ParseOutcome outcome = ParseOutcome::failure("vehicle_ref_error");
    outcome.detail = "cannot open vehicle file: " + path;
    return outcome;
  }
  json document;
  try {
    stream >> document;
  } catch (const std::exception& error) {
    ParseOutcome outcome = ParseOutcome::failure("vehicle_ref_error");
    outcome.detail = std::string("invalid JSON in vehicle file '") + path + "': " + error.what();
    return outcome;
  }
  return parse_vehicle(document, out);
}

ParseOutcome parse_plan_request(const json& body, ServicePlanRequest* out) {
  if (!body.is_object()) {
    return bad_field("request body", "a JSON object");
  }
  ParseOutcome outcome;
  ServicePlanRequest request;

  if (!read_string(body, "request_id", false, &request.request_id, &outcome) ||
      !read_string(body, "map_id", false, &request.map_id, &outcome) ||
      !read_string(body, "map_version", false, &request.map_version, &outcome)) {
    return outcome;
  }

  if (!body.contains("start")) {
    ParseOutcome outcome_missing = ParseOutcome::failure("missing_field");
    outcome_missing.detail = "missing field 'start'";
    return outcome_missing;
  }
  if (!read_pose(body.at("start"), "start", &request.start, &outcome)) {
    return outcome;
  }
  if (!body.contains("goal")) {
    ParseOutcome outcome_missing = ParseOutcome::failure("missing_field");
    outcome_missing.detail = "missing field 'goal'";
    return outcome_missing;
  }
  if (!read_pose(body.at("goal"), "goal", &request.goal, &outcome)) {
    return outcome;
  }

  request.vehicle = default_vehicle();
  if (body.contains("vehicle") && body.contains("vehicle_ref")) {
    return bad_field("vehicle/vehicle_ref", "at most one of them");
  }
  if (body.contains("vehicle")) {
    const ParseOutcome parsed = parse_vehicle(body.at("vehicle"), &request.vehicle);
    if (!parsed.ok) {
      return parsed;
    }
  } else if (body.contains("vehicle_ref")) {
    const json& ref = body.at("vehicle_ref");
    std::string path;
    if (ref.is_string()) {
      path = ref.get<std::string>();
    } else if (ref.is_object()) {
      if (!read_string(ref, "path", false, &path, &outcome)) {
        outcome.detail = std::string("vehicle_ref.") + outcome.detail;
        return outcome;
      }
    } else {
      return bad_field("vehicle_ref", "a path string or an object with 'path'");
    }
    const ParseOutcome loaded = load_vehicle_file(path, &request.vehicle);
    if (!loaded.ok) {
      return loaded;
    }
  }

  if (body.contains("obstacles")) {
    if (!body.at("obstacles").is_array()) {
      return bad_field("obstacles", "an array");
    }
    for (const json& value : body.at("obstacles")) {
      ServiceObstacle obstacle;
      const ParseOutcome parsed = parse_obstacle(value, &obstacle);
      if (!parsed.ok) {
        return parsed;
      }
      request.obstacles.push_back(std::move(obstacle));
    }
  }

  if (body.contains("algorithm_params")) {
    const ParseOutcome parsed =
        parse_algorithm_params(body.at("algorithm_params"), request.vehicle, &request.params);
    if (!parsed.ok) {
      return parsed;
    }
  }

  *out = std::move(request);
  return ParseOutcome::success();
}

}  // namespace planner::service
