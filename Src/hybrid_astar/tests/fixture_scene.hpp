#pragma once

// hybrid_astar 测试与基准测试共用的测试夹具加载和独立检查。
//
// 夹具解析直接读取带版本号的 paths/cases.json（该文件遵循自己的 schema，
// 而非公共编解码器）。栅格化与路径分析重新实现消费方所需的少量几何，而不
// 调用规划器内部实现，使验收证据不只是重跑生产者。整车碰撞证据按集成契约
// 要求使用公开的 planner::collision 检测器。
//
// 接收显式路径的函数由测试（宏固化的夹具目录）和基准测试（命令行夹具
// 目录）共用。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/collision/collision_checker.hpp"
#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"
#include "planner/hybrid_astar/ros_free_backend.hpp"

namespace planner::hybrid_astar::test_support {

using common::Direction;

inline nlohmann::json read_json_file(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open fixture file: " + path);
  }
  nlohmann::json document;
  stream >> document;
  return document;
}

inline common::VehicleModel load_default_vehicle_from(const std::string& path) {
  const nlohmann::json document = read_json_file(path);
  common::VehicleModel vehicle;
  vehicle.id = document.at("id").get<std::string>();
  vehicle.wheelbase_m = document.at("wheelbase_m").get<double>();
  vehicle.width_m = document.at("width_m").get<double>();
  vehicle.front_overhang_m = document.at("front_overhang_m").get<double>();
  vehicle.rear_overhang_m = document.at("rear_overhang_m").get<double>();
  vehicle.max_steer_rad = document.at("max_steer_rad").get<double>();
  vehicle.safety_margin_m = document.at("safety_margin_m").get<double>();
  return vehicle;
}

inline common::Pose2d parse_pose(const nlohmann::json& value) {
  return common::Pose2d{value.at("x_m").get<double>(),
                        value.at("y_m").get<double>(),
                        value.at("yaw_rad").get<double>()};
}

struct ObstacleSpec {
  std::string id;
  bool is_polygon{false};
  double x_min_m{0.0};
  double y_min_m{0.0};
  double x_max_m{0.0};
  double y_max_m{0.0};
  std::vector<common::Point2d> vertices;

  std::vector<common::Point2d> corners() const {
    if (!is_polygon) {
      return {common::Point2d{x_min_m, y_min_m},
              common::Point2d{x_max_m, y_min_m},
              common::Point2d{x_max_m, y_max_m},
              common::Point2d{x_min_m, y_max_m}};
    }
    return vertices;
  }
};

struct PathCase {
  std::string id;
  std::string case_class;
  std::string boundary_kind;
  int seed{0};
  common::GridView grid;
  common::Pose2d start;
  common::Pose2d goal;
  std::vector<ObstacleSpec> obstacles;
  std::string expected_status;          // "success" | "failure"
  std::string expected_failure_reason;  // no_path/timeout/invalid_input/cancelled
  std::optional<bool> cancelled;
  std::optional<double> time_budget_s;
  std::optional<std::int64_t> node_budget;
};

inline ObstacleSpec parse_obstacle(const nlohmann::json& value) {
  ObstacleSpec obstacle;
  obstacle.id = value.at("id").get<std::string>();
  const std::string type = value.at("type").get<std::string>();
  if (type == "rectangle") {
    obstacle.is_polygon = false;
    obstacle.x_min_m = value.at("x_min_m").get<double>();
    obstacle.y_min_m = value.at("y_min_m").get<double>();
    obstacle.x_max_m = value.at("x_max_m").get<double>();
    obstacle.y_max_m = value.at("y_max_m").get<double>();
  } else if (type == "convex_polygon") {
    obstacle.is_polygon = true;
    for (const nlohmann::json& vertex : value.at("vertices")) {
      obstacle.vertices.push_back(
          common::Point2d{vertex.at("x_m").get<double>(),
                          vertex.at("y_m").get<double>()});
    }
  } else {
    throw std::runtime_error("unknown obstacle type: " + type);
  }
  return obstacle;
}

inline PathCase parse_path_case(const nlohmann::json& value) {
  PathCase path_case;
  path_case.id = value.at("id").get<std::string>();
  path_case.case_class = value.at("class").get<std::string>();
  if (value.contains("boundary_kind")) {
    path_case.boundary_kind = value.at("boundary_kind").get<std::string>();
  }
  path_case.seed = value.at("seed").get<int>();
  const nlohmann::json& grid = value.at("grid");
  path_case.grid.map_id = grid.at("map_id").get<std::string>();
  path_case.grid.map_version = grid.at("map_version").get<std::string>();
  path_case.grid.resolution_m = grid.at("resolution_m").get<double>();
  path_case.grid.origin = parse_pose(grid.at("origin"));
  path_case.grid.width = grid.at("width").get<int>();
  path_case.grid.height = grid.at("height").get<int>();
  path_case.grid.cells.assign(
      static_cast<std::size_t>(path_case.grid.width) *
          static_cast<std::size_t>(path_case.grid.height),
      common::kCellFree);
  path_case.start = parse_pose(value.at("start"));
  path_case.goal = parse_pose(value.at("goal"));
  for (const nlohmann::json& obstacle : value.at("obstacles")) {
    path_case.obstacles.push_back(parse_obstacle(obstacle));
  }
  const nlohmann::json& expected = value.at("expected");
  path_case.expected_status = expected.at("status").get<std::string>();
  if (expected.contains("failure_reason")) {
    path_case.expected_failure_reason =
        expected.at("failure_reason").get<std::string>();
  }
  if (value.contains("request_overrides")) {
    const nlohmann::json& overrides = value.at("request_overrides");
    if (overrides.contains("cancelled")) {
      path_case.cancelled = overrides.at("cancelled").get<bool>();
    }
    if (overrides.contains("time_budget_s")) {
      path_case.time_budget_s = overrides.at("time_budget_s").get<double>();
    }
    if (overrides.contains("node_budget")) {
      path_case.node_budget =
          overrides.at("node_budget").get<std::int64_t>();
    }
  }
  return path_case;
}

inline std::vector<PathCase> load_path_cases_from(const std::string& path) {
  const nlohmann::json document = read_json_file(path);
  std::vector<PathCase> cases;
  for (const nlohmann::json& value : document.at("cases")) {
    cases.push_back(parse_path_case(value));
  }
  return cases;
}


// ---------------------------------------------------------------------------
// 栅格化与场景构建。
//
// cases.json 固定了栅格元数据（width/height/resolution/origin）和障碍物
// 几何，但不含逐单元数据。单元中心落在封闭障碍物多边形内时该单元标记为
// 占据（接触即算）。这是最小推导：它为目标势场给出有用的阻塞单元，而不
// 添加完整的单元接触光晕，否则会与碰撞组件已施加的车辆安全边距重复计入。
// ---------------------------------------------------------------------------

inline bool point_in_convex_polygon(const std::vector<common::Point2d>& polygon,
                                    const common::Point2d& point) {
  const std::size_t count = polygon.size();
  if (count < 3) {
    return false;
  }
  double signed_area = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d& a = polygon[i];
    const common::Point2d& b = polygon[(i + 1) % count];
    signed_area += a.x_m * b.y_m - b.x_m * a.y_m;
  }
  const bool ccw = signed_area >= 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d& a = polygon[i];
    const common::Point2d& b = polygon[(i + 1) % count];
    const double cross =
        (b.x_m - a.x_m) * (point.y_m - a.y_m) - (b.y_m - a.y_m) * (point.x_m - a.x_m);
    if (ccw ? cross < 0.0 : cross > 0.0) {
      return false;
    }
  }
  return true;
}

inline void rasterize_obstacles(common::GridView* grid,
                                const std::vector<ObstacleSpec>& obstacles) {
  const collision::GridFrame frame = collision::GridFrame::from_grid(*grid);
  for (const ObstacleSpec& obstacle : obstacles) {
    std::vector<common::Point2d> polygon;
    for (const common::Point2d& corner : obstacle.corners()) {
      polygon.push_back(frame.to_grid_frame(corner));
    }
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const common::Point2d& point : polygon) {
      min_x = std::min(min_x, point.x_m);
      max_x = std::max(max_x, point.x_m);
      min_y = std::min(min_y, point.y_m);
      max_y = std::max(max_y, point.y_m);
    }
    const int col_begin =
        std::max(0, static_cast<int>(std::floor(min_x / grid->resolution_m)) - 1);
    const int col_end = std::min(
        grid->width - 1, static_cast<int>(std::ceil(max_x / grid->resolution_m)));
    const int row_begin =
        std::max(0, static_cast<int>(std::floor(min_y / grid->resolution_m)) - 1);
    const int row_end = std::min(
        grid->height - 1, static_cast<int>(std::ceil(max_y / grid->resolution_m)));
    for (int row = row_begin; row <= row_end; ++row) {
      for (int col = col_begin; col <= col_end; ++col) {
        const common::Point2d center{
            (col + 0.5) * grid->resolution_m,
            (row + 0.5) * grid->resolution_m};
        if (point_in_convex_polygon(polygon, center)) {
          grid->cells[static_cast<std::size_t>(row) *
                          static_cast<std::size_t>(grid->width) +
                      static_cast<std::size_t>(col)] = common::kCellOccupied;
        }
      }
    }
  }
}

struct PreparedCase {
  common::GridView grid;
  collision::Scene scene;
  collision::CollisionChecker checker;
  common::PlanRequest request;
  collision::CheckResult start_check;
  collision::CheckResult goal_check;

  // CollisionChecker 没有默认构造函数，因此各成员由 prepare_case() 通过
  // 此构造函数提供；checker 绑定到已构造好的 scene 成员。
  PreparedCase(common::GridView grid_value, collision::Scene scene_value,
               const common::VehicleModel& vehicle)
      : grid(std::move(grid_value)),
        scene(std::move(scene_value)),
        checker(vehicle, scene) {}
};

inline PreparedCase prepare_case(const PathCase& path_case,
                                 const common::VehicleModel& vehicle) {
  common::GridView grid = path_case.grid;
  rasterize_obstacles(&grid, path_case.obstacles);
  collision::Scene scene;
  scene.grid = grid;
  for (const ObstacleSpec& obstacle : path_case.obstacles) {
    if (obstacle.is_polygon) {
      scene.obstacles.push_back(
          collision::Obstacle::convex_polygon(obstacle.id, obstacle.vertices));
    } else {
      scene.obstacles.push_back(collision::Obstacle::rectangle(
          obstacle.id, obstacle.x_min_m, obstacle.y_min_m, obstacle.x_max_m,
          obstacle.y_max_m));
    }
  }
  PreparedCase prepared(std::move(grid), std::move(scene), vehicle);
  prepared.start_check = prepared.checker.check_pose(path_case.start);
  prepared.goal_check = prepared.checker.check_pose(path_case.goal);
  prepared.request.request_id = path_case.id;
  prepared.request.map_id = path_case.grid.map_id;
  prepared.request.map_version = path_case.grid.map_version;
  prepared.request.vehicle = vehicle;
  prepared.request.start = path_case.start;
  prepared.request.goal = path_case.goal;
  prepared.request.cancelled = path_case.cancelled.value_or(false);
  prepared.request.time_budget_s = path_case.time_budget_s;
  prepared.request.node_budget = path_case.node_budget;
  return prepared;
}


// ---------------------------------------------------------------------------
// 独立路径分析。
// ---------------------------------------------------------------------------

struct PathAnalysis {
  bool has_path{false};
  std::size_t point_count{0};
  std::size_t segment_count{0};
  double length_m{0.0};
  std::int64_t gear_changes{0};
  double endpoint_position_error_m{0.0};
  double endpoint_yaw_error_rad{0.0};
  double max_abs_curvature{0.0};
  bool s_strictly_increasing{true};
  bool segment_directions_homogeneous{true};
  bool convex_direction_changes{true};  // 相邻分段方向不同
  bool cusps_duplicated{true};          // 尖点位姿出现在两个相邻分段中
  bool starts_at_request{false};
  bool all_finite{true};
};

inline PathAnalysis analyze_path(const common::PlanResult& result,
                                 const common::Pose2d& start,
                                 const common::Pose2d& goal,
                                 const common::VehicleModel& vehicle) {
  (void)vehicle;
  PathAnalysis analysis;
  analysis.has_path = !result.path.empty();
  double previous_s = -std::numeric_limits<double>::infinity();
  Direction previous_segment_direction = Direction::kForward;
  common::Pose2d previous_segment_last_pose;
  bool have_previous_segment = false;
  for (std::size_t segment_index = 0; segment_index < result.path.size();
       ++segment_index) {
    const common::PathSegment& segment = result.path[segment_index];
    ++analysis.segment_count;
    Direction segment_direction = Direction::kForward;
    bool first_point_in_segment = true;
    for (const common::PathPoint& point : segment.points) {
      ++analysis.point_count;
      if (!std::isfinite(point.pose.x_m) || !std::isfinite(point.pose.y_m) ||
          !std::isfinite(point.pose.yaw_rad) || !std::isfinite(point.s_m) ||
          !std::isfinite(point.curvature_inv_m)) {
        analysis.all_finite = false;
      }
      if (first_point_in_segment && have_previous_segment) {
        // 尖点归属：该位姿被复制为前一分段的最后一个点，因此跨边界时
        // s_m 可以相等。
        if (point.s_m < previous_s) {
          analysis.s_strictly_increasing = false;
        }
        if (std::hypot(point.pose.x_m - previous_segment_last_pose.x_m,
                       point.pose.y_m - previous_segment_last_pose.y_m) > 1e-9 ||
            std::abs(common::normalize_angle_rad(
                point.pose.yaw_rad - previous_segment_last_pose.yaw_rad)) > 1e-9) {
          analysis.cusps_duplicated = false;
        }
      } else if (!(point.s_m > previous_s)) {
        analysis.s_strictly_increasing = false;
      }
      if (first_point_in_segment && have_previous_segment &&
          point.direction == previous_segment_direction) {
        analysis.convex_direction_changes = false;
      }
      if (first_point_in_segment) {
        segment_direction = point.direction;
      } else if (point.direction != segment_direction) {
        analysis.segment_directions_homogeneous = false;
      }
      first_point_in_segment = false;
      previous_s = point.s_m;
      analysis.max_abs_curvature =
          std::max(analysis.max_abs_curvature, std::abs(point.curvature_inv_m));
    }
    previous_segment_direction = segment_direction;
    if (!segment.points.empty()) {
      previous_segment_last_pose = segment.points.back().pose;
    }
    have_previous_segment = true;
  }
  if (analysis.has_path) {
    analysis.length_m = result.path.back().points.back().s_m;
    analysis.gear_changes =
        static_cast<std::int64_t>(result.path.size()) - 1;
    const common::Pose2d& first = result.path.front().points.front().pose;
    const common::Pose2d& last = result.path.back().points.back().pose;
    analysis.starts_at_request =
        std::hypot(first.x_m - start.x_m, first.y_m - start.y_m) <= 1e-6 &&
        std::abs(common::normalize_angle_rad(first.yaw_rad - start.yaw_rad)) <=
            1e-6;
    analysis.endpoint_position_error_m =
        std::hypot(last.x_m - goal.x_m, last.y_m - goal.y_m);
    analysis.endpoint_yaw_error_rad = std::abs(
        common::normalize_angle_rad(last.yaw_rad - goal.yaw_rad));
  }
  return analysis;
}

// 结果路径满足 §7.1 验收条件中无需独立碰撞引擎即可检查的部分时返回空
// 字符串。
inline std::string validation_error(const common::PlanResult& result,
                                    const common::Pose2d& start,
                                    const common::Pose2d& goal,
                                    const common::VehicleModel& vehicle) {
  const PathAnalysis analysis = analyze_path(result, start, goal, vehicle);
  if (!analysis.has_path) {
    return "result has no path";
  }
  if (!analysis.all_finite) {
    return "path contains non-finite values";
  }
  if (!analysis.starts_at_request) {
    return "path does not start at the requested start pose";
  }
  if (!analysis.s_strictly_increasing) {
    return "arc length s_m is not strictly increasing";
  }
  if (!analysis.segment_directions_homogeneous) {
    return "a segment mixes forward and reverse points";
  }
  if (!analysis.convex_direction_changes) {
    return "a direction change did not open a new segment";
  }
  if (!analysis.cusps_duplicated) {
    return "cusp pose is not duplicated as the previous segment's last point";
  }
  const double max_curvature = 1.0 / vehicle.min_turn_radius_m();
  if (analysis.max_abs_curvature > max_curvature + 1e-9) {
    return "curvature exceeds the vehicle limit";
  }
  if (analysis.endpoint_position_error_m > 0.05) {
    return "endpoint position error exceeds 0.05 m";
  }
  if (analysis.endpoint_yaw_error_rad > 0.02) {
    return "endpoint yaw error exceeds 0.02 rad";
  }
  return std::string();
}

inline common::Status expected_status_for(const std::string& reason) {
  if (reason == "no_path") {
    return common::Status::kNoPath;
  }
  if (reason == "timeout") {
    return common::Status::kTimeout;
  }
  if (reason == "invalid_input") {
    return common::Status::kInvalidInput;
  }
  if (reason == "cancelled") {
    return common::Status::kCancelled;
  }
  return common::Status::kSuccess;
}

inline std::string describe_collision(const collision::CheckResult& check) {
  return std::string("collision: ") + collision::to_string(check.reason_code) +
         (check.detail.empty() ? "" : " " + check.detail);
}


}  // namespace planner::hybrid_astar::test_support
