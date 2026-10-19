// planner_validation 命令行示例。
//
// 模式：
//   planner_validation_example <request.json>
//       读取一个预构建的验证请求（文档布局参见 examples/README.md），
//       运行 planner::validation::validate_trajectory，并把判定结果作为
//       JSON 结论打印到 stdout。
//   planner_validation_example --self-check
//       在内存中构建一条有效轨迹和一条故意超限的轨迹，检查验证器是否
//       按预期约束接受/拒绝它们，并打印结果。
//
// 退出码：
//   0 判定有效（或所有自检通过），
//   1 验证失败或输入被拒绝（文件可读，判定即为示例的结果），
//   2 用法错误，
//   3 文件或文档错误，
//   4 自检期望不匹配。

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/collision/scene.hpp"
#include "planner/common/serialization.hpp"
#include "planner/common/types.hpp"
#include "planner/validation/validation.hpp"

namespace {

using nlohmann::json;

using planner::common::Corridor;
using planner::common::CorridorPolygon;
using planner::common::GridView;
using planner::common::Point2d;
using planner::common::Pose2d;
using planner::common::TrajectoryPoint;
using planner::common::VehicleModel;
using planner::validation::Constraint;
using planner::validation::ValidationRequest;
using planner::validation::ValidationResult;

const char* kUsage =
    "usage:\n"
    "  planner_validation_example <request.json>\n"
    "  planner_validation_example --self-check\n";

int fail(int code, const std::string& message) {
  std::cerr << "planner_validation_example: " << message << "\n";
  return code;
}

bool is_finite_number(const json& value) {
  return value.is_number() && std::isfinite(value.get<double>());
}

double require_number(const json& object, const char* key) {
  if (!object.contains(key)) {
    throw std::runtime_error(std::string("missing numeric field '") + key + "'");
  }
  const json& value = object.at(key);
  if (!is_finite_number(value)) {
    throw std::runtime_error(std::string("field '") + key +
                             "' must be a finite number");
  }
  return value.get<double>();
}

json number_or_null(double value) {
  if (std::isfinite(value)) {
    return value;
  }
  return nullptr;
}

json violation_to_json(const planner::validation::ValidationViolation& violation) {
  json out;
  out["constraint"] = planner::validation::to_string(violation.constraint);
  out["time_s"] = number_or_null(violation.time_s);
  out["point_index"] =
      violation.point_index >= 0 ? json(violation.point_index) : json(nullptr);
  out["actual"] = number_or_null(violation.actual);
  out["limit"] = number_or_null(violation.limit);
  out["detail"] = violation.detail;
  return out;
}

json result_to_json(const ValidationResult& result) {
  json out;
  out["schema_version"] = 1;
  out["type"] = "validation_result";
  out["valid"] = result.valid;
  out["status"] = planner::common::to_string(result.status);
  out["reason"] = result.reason;
  out["detail"] = result.detail;
  if (result.first_failure.constraint == Constraint::kNone) {
    out["first_failure"] = nullptr;
  } else {
    out["first_failure"] = violation_to_json(result.first_failure);
  }
  json stats;
  stats["trajectory_points"] = result.trajectory_points;
  stats["corridor_samples"] = result.corridor_samples;
  stats["collision_pose_checks"] = result.collision_pose_checks;
  stats["collision_motion_checks"] = result.collision_motion_checks;
  out["stats"] = stats;
  return out;
}

void check_envelope(const json& document) {
  if (!document.is_object()) {
    throw std::runtime_error("document root must be a JSON object");
  }
  if (document.contains("schema_version")) {
    const json& version = document.at("schema_version");
    if (!version.is_number_integer() || version.get<int>() != 1) {
      throw std::runtime_error("schema_version must be the integer 1");
    }
  }
  if (document.contains("type")) {
    const json& type = document.at("type");
    if (!type.is_string() || type.get<std::string>() != "validation_request") {
      throw std::runtime_error("type must be \"validation_request\"");
    }
  }
}

ValidationRequest parse_request(const json& document) {
  check_envelope(document);
  ValidationRequest request;

  if (!document.contains("vehicle")) {
    throw std::runtime_error("missing required object 'vehicle'");
  }
  request.vehicle = planner::common::decode_payload<VehicleModel>(
      document.at("vehicle"));

  if (!document.contains("limits")) {
    throw std::runtime_error("missing required object 'limits'");
  }
  const json& limits = document.at("limits");
  request.limits.max_speed_mps = require_number(limits, "max_speed_mps");
  request.limits.max_acceleration_mps2 =
      require_number(limits, "max_acceleration_mps2");
  request.limits.max_deceleration_mps2 =
      require_number(limits, "max_deceleration_mps2");
  request.limits.max_curvature_inv_m =
      require_number(limits, "max_curvature_inv_m");

  if (!document.contains("goal")) {
    throw std::runtime_error("missing required object 'goal'");
  }
  request.goal = planner::common::decode_payload<Pose2d>(document.at("goal"));

  planner::collision::Scene scene;
  if (document.contains("grid")) {
    scene.grid = planner::common::decode_payload<GridView>(document.at("grid"));
  }
  if (document.contains("obstacles")) {
    const json& obstacles = document.at("obstacles");
    if (!obstacles.is_array()) {
      throw std::runtime_error("'obstacles' must be an array");
    }
    for (const json& entry : obstacles) {
      if (!entry.is_object() || !entry.contains("id") ||
          !entry.contains("vertices")) {
        throw std::runtime_error(
            "each obstacle needs 'id' and 'vertices' fields");
      }
      // Point2d 没有独立的编解码实例化；示例文档把障碍物顶点存为
      // {x_m, y_m} 负载并在此解析。
      std::vector<Point2d> vertices;
      for (const json& vertex : entry.at("vertices")) {
        if (!vertex.is_object()) {
          throw std::runtime_error("obstacle vertices must be objects");
        }
        Point2d vertex_point;
        vertex_point.x_m = require_number(vertex, "x_m");
        vertex_point.y_m = require_number(vertex, "y_m");
        vertices.push_back(vertex_point);
      }
      planner::collision::Obstacle obstacle =
          planner::collision::Obstacle::convex_polygon(
              entry.at("id").get<std::string>(), std::move(vertices));
      std::string reason;
      if (!obstacle.valid(&reason)) {
        throw std::runtime_error("obstacle '" + obstacle.id +
                                 "' is not a valid convex polygon: " + reason);
      }
      scene.obstacles.push_back(std::move(obstacle));
    }
  }
  request.scene = std::move(scene);

  if (!document.contains("corridor")) {
    throw std::runtime_error("missing required object 'corridor'");
  }
  request.corridor =
      planner::common::decode_payload<Corridor>(document.at("corridor"));

  if (!document.contains("trajectory")) {
    throw std::runtime_error("missing required array 'trajectory'");
  }
  const json& trajectory = document.at("trajectory");
  if (!trajectory.is_array()) {
    throw std::runtime_error("'trajectory' must be an array");
  }
  for (const json& point : trajectory) {
    request.trajectory.push_back(
        planner::common::decode_payload<TrajectoryPoint>(point));
  }
  return request;
}

// ---------------------------------------------------------------------------
// 内置自检数据（结构与 examples/data 相同）。
// ---------------------------------------------------------------------------

TrajectoryPoint point(double t_s, double x_m, double y_m, double yaw_rad,
                      double velocity_mps, double acceleration_mps2) {
  TrajectoryPoint point;
  point.t_s = t_s;
  point.pose = Pose2d{x_m, y_m, yaw_rad};
  point.velocity_mps = velocity_mps;
  point.acceleration_mps2 = acceleration_mps2;
  point.curvature_inv_m = 0.0;
  return point;
}

VehicleModel default_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "default_test_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

CorridorPolygon rectangle_polygon(double s_start_m, double s_end_m,
                                  double x_min_m, double y_min_m,
                                  double x_max_m, double y_max_m) {
  CorridorPolygon polygon;
  polygon.s_start_m = s_start_m;
  polygon.s_end_m = s_end_m;
  polygon.vertices = {Point2d{x_min_m, y_min_m}, Point2d{x_max_m, y_min_m},
                      Point2d{x_max_m, y_max_m}, Point2d{x_min_m, y_max_m}};
  return polygon;
}

ValidationRequest straight_request() {
  ValidationRequest request;
  request.vehicle = default_vehicle();
  request.limits.max_speed_mps = 2.0;
  request.limits.max_acceleration_mps2 = 2.0;
  request.limits.max_deceleration_mps2 = 2.0;
  request.limits.max_curvature_inv_m = 0.2;
  request.goal = Pose2d{3.0, 0.0, 0.0};
  request.trajectory = {
      point(0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      point(1.0, 0.5, 0.0, 0.0, 1.0, 0.0),
      point(2.0, 1.5, 0.0, 0.0, 1.0, 0.0),
      point(3.0, 2.5, 0.0, 0.0, 1.0, -1.0),
      point(4.0, 3.0, 0.0, 0.0, 0.0, 0.0),
  };
  Corridor corridor;
  corridor.id = "corridor_example";
  corridor.map_id = "map_example";
  corridor.map_version = "v1";
  corridor.frame_id = "map";
  corridor.polygons.push_back(rectangle_polygon(0.0, 1.5, -2.0, -2.0, 6.0, 2.0));
  corridor.polygons.push_back(rectangle_polygon(1.5, 3.0, 0.0, -2.0, 7.0, 2.0));
  request.corridor = std::move(corridor);
  planner::collision::Scene scene;
  scene.obstacles.push_back(planner::collision::Obstacle::rectangle(
      "wall_beside_path", 0.5, 3.0, 2.0, 4.0));
  request.scene = std::move(scene);
  return request;
}

int self_check() {
  bool ok = true;

  const ValidationRequest valid_request = straight_request();
  const ValidationResult valid_result =
      planner::validation::validate_trajectory(valid_request);
  std::cout << "self-check valid trajectory: valid=" << valid_result.valid
            << " reason=" << valid_result.reason << "\n";
  if (!valid_result.valid || valid_result.status != planner::common::Status::kSuccess) {
    ok = false;
  }

  ValidationRequest speeding = straight_request();
  speeding.trajectory[2].velocity_mps = 3.0;  // 限制为 2.0 m/s
  const ValidationResult speeding_result =
      planner::validation::validate_trajectory(speeding);
  std::cout << "self-check speeding trajectory: valid=" << speeding_result.valid
            << " reason=" << speeding_result.reason
            << " time="
            << (std::isfinite(speeding_result.first_failure.time_s)
                    ? std::to_string(speeding_result.first_failure.time_s)
                    : "n/a")
            << "\n";
  if (speeding_result.valid ||
      speeding_result.first_failure.constraint != Constraint::kSpeedLimit) {
    ok = false;
  }

  if (!ok) {
    std::cerr << "planner_validation_example: self-check mismatch\n";
    return 4;
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--self-check") {
    return self_check();
  }
  if (argc != 2 || std::string(argv[1]) == "--help" ||
      std::string(argv[1]) == "-h") {
    std::cerr << kUsage;
    return 2;
  }

  std::ifstream input(argv[1]);
  if (!input) {
    return fail(3, std::string("cannot open '") + argv[1] + "'");
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  if (!input.good() && !input.eof()) {
    return fail(3, std::string("cannot read '") + argv[1] + "'");
  }

  json document;
  try {
    document = json::parse(buffer.str());
  } catch (const std::exception& error) {
    return fail(3, std::string("invalid JSON: ") + error.what());
  }

  ValidationRequest request;
  try {
    request = parse_request(document);
  } catch (const std::exception& error) {
    return fail(3, std::string("invalid validation request: ") + error.what());
  }

  const ValidationResult result =
      planner::validation::validate_trajectory(request);
  std::cout << result_to_json(result).dump(2) << "\n";
  return result.valid ? 0 : 1;
}
