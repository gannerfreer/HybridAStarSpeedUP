#pragma once

// planner_dftpav 的端到端链路夹具：
//   Hybrid A*（归一化后端，默认 ros_free；可显式选择其它 normalized 后端）->
//   planner::corridor -> Dftpav 优化器 -> planner::validation。
//
// tests/fixtures/v2 下的夹具文件是共享的版本化输入源
// （paths/cases.json + vehicles/default.json）。栅格化精确重实现文档给出的
// 推导（"接触即占据"）；场景与走廊障碍物用同样的几何经 planner::collision /
// planner::corridor 的公开构造函数重建。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/collision/collision_checker.hpp"
#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"
#include "planner/corridor/corridor_generator.hpp"
#include "planner/dftpav/optimizer.hpp"
#include "planner/hybrid_astar/backend.hpp"

#include "dftpav_test_support.hpp"  // make_footprint_lane_corridor 回退方案

#ifndef PLANNER_DFTPAV_FIXTURES_DIR
#error "PLANNER_DFTPAV_FIXTURES_DIR must be defined by the test CMakeLists"
#endif

namespace planner::dftpav::e2e {

inline nlohmann::json read_json_file(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open fixture file: " + path);
  }
  nlohmann::json document;
  stream >> document;
  return document;
}

inline std::string fixture_path(const std::string& relative) {
  return std::string(PLANNER_DFTPAV_FIXTURES_DIR) + "/" + relative;
}

inline common::VehicleModel load_default_vehicle() {
  const nlohmann::json document = read_json_file(fixture_path("vehicles/default.json"));
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
  return common::Pose2d{value.at("x_m").get<double>(), value.at("y_m").get<double>(),
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
    if (is_polygon) {
      return vertices;
    }
    return {{x_min_m, y_min_m}, {x_max_m, y_min_m}, {x_max_m, y_max_m}, {x_min_m, y_max_m}};
  }
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
          common::Point2d{vertex.at("x_m").get<double>(), vertex.at("y_m").get<double>()});
    }
  } else {
    throw std::runtime_error("unknown obstacle type: " + type);
  }
  return obstacle;
}

struct PathCase {
  std::string id;
  std::string case_class;
  common::GridView grid;
  common::Pose2d start;
  common::Pose2d goal;
  std::vector<ObstacleSpec> obstacles;
  std::string expected_status;
};

inline PathCase parse_path_case(const nlohmann::json& value) {
  PathCase path_case;
  path_case.id = value.at("id").get<std::string>();
  path_case.case_class = value.at("class").get<std::string>();
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
  path_case.expected_status = value.at("expected").at("status").get<std::string>();
  return path_case;
}

inline PathCase load_path_case(const std::string& id) {
  const nlohmann::json document = read_json_file(fixture_path("paths/cases.json"));
  for (const nlohmann::json& value : document.at("cases")) {
    if (value.at("id").get<std::string>() == id) {
      return parse_path_case(value);
    }
  }
  throw std::runtime_error("path case not found in v2 fixtures: " + id);
}

// ---------------------------------------------------------------------------
// 栅格化（文档给出的推导：单元格中心落在闭合障碍物多边形内即为占据；
// 接触即算）。
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
    const int col_end =
        std::min(grid->width - 1, static_cast<int>(std::ceil(max_x / grid->resolution_m)));
    const int row_begin =
        std::max(0, static_cast<int>(std::floor(min_y / grid->resolution_m)) - 1);
    const int row_end =
        std::min(grid->height - 1, static_cast<int>(std::ceil(max_y / grid->resolution_m)));
    for (int row = row_begin; row <= row_end; ++row) {
      for (int col = col_begin; col <= col_end; ++col) {
        const common::Point2d center{(col + 0.5) * grid->resolution_m,
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

// ---------------------------------------------------------------------------
// 场景 / 走廊转换与链路运行器。
// ---------------------------------------------------------------------------

inline std::vector<corridor::Obstacle> build_corridor_obstacles(const PathCase& path_case) {
  std::vector<corridor::Obstacle> obstacles;
  for (const ObstacleSpec& spec : path_case.obstacles) {
    if (spec.is_polygon) {
      obstacles.push_back(corridor::make_convex_obstacle(spec.id, spec.vertices));
    } else {
      obstacles.push_back(corridor::make_rectangle_obstacle(
          spec.id, spec.x_min_m, spec.y_min_m, spec.x_max_m, spec.y_max_m));
    }
  }
  return obstacles;
}

inline collision::Scene build_scene(const PathCase& path_case) {
  collision::Scene scene;
  scene.grid = path_case.grid;
  rasterize_obstacles(&scene.grid, path_case.obstacles);
  for (const ObstacleSpec& spec : path_case.obstacles) {
    if (spec.is_polygon) {
      scene.obstacles.push_back(collision::Obstacle::convex_polygon(spec.id, spec.vertices));
    } else {
      scene.obstacles.push_back(collision::Obstacle::rectangle(
          spec.id, spec.x_min_m, spec.y_min_m, spec.x_max_m, spec.y_max_m));
    }
  }
  return scene;
}

// 防御性的连接点吸附：当前 Hybrid A* 后端输出的零间隙路径中，换向位姿会在
// 相邻两段中各出现一次（s_m 相同、direction 相反；hybrid-003 证据），
// 因此对它们是空操作。优化器在运行段之间共享一个连接点变量；此辅助函数
// 只是为了让走廊生成 / 车道回退在畸形路径重复尖点且有微小偏移时看到
// 相同的连续几何。
inline std::vector<common::PathSegment> snap_path_junctions(
    std::vector<common::PathSegment> path) {
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (path[i].points.empty() || path[i - 1].points.empty()) {
      continue;
    }
    const common::Pose2d& previous = path[i - 1].points.back().pose;
    common::PathPoint& first = path[i].points.front();
    const double gap = std::hypot(first.pose.x_m - previous.x_m, first.pose.y_m - previous.y_m);
    if (gap > 1e-9 && gap <= 0.75) {
      first.pose.x_m = previous.x_m;
      first.pose.y_m = previous.y_m;
    }
  }
  return path;
}

// 由车辆模型导出的限制：曲率限制是真实的运动学上界
// tan(max_steer) / wheelbase（夹具中的说明相同）。
inline OptimizeLimits limits_for_vehicle(const common::VehicleModel& vehicle) {
  OptimizeLimits limits;
  limits.max_forward_velocity_mps = 5.0;
  limits.max_backward_velocity_mps = 2.0;
  limits.max_forward_acceleration_mps2 = 4.0;
  limits.max_backward_acceleration_mps2 = 4.0;
  const double max_curvature = std::tan(vehicle.max_steer_rad) / vehicle.wheelbase_m;
  limits.max_forward_curvature_inv_m = max_curvature;
  limits.max_backward_curvature_inv_m = max_curvature;
  limits.max_lateral_acceleration_mps2 = 5.0;
  limits.max_steering_rate_radps = 10000.0;
  return limits;
}

struct PreparedCase {
  common::GridView grid;
  collision::Scene scene;
  collision::CollisionChecker checker;
  common::PlanRequest plan_request;

  PreparedCase(common::GridView grid_value, collision::Scene scene_value,
               const common::VehicleModel& vehicle)
      : grid(std::move(grid_value)), scene(std::move(scene_value)), checker(vehicle, scene) {}
};

inline PreparedCase prepare_case(const PathCase& path_case,
                                 const common::VehicleModel& vehicle) {
  common::GridView grid = path_case.grid;
  rasterize_obstacles(&grid, path_case.obstacles);
  PreparedCase prepared(std::move(grid), build_scene(path_case), vehicle);
  prepared.plan_request.request_id = path_case.id;
  prepared.plan_request.map_id = path_case.grid.map_id;
  prepared.plan_request.map_version = path_case.grid.map_version;
  prepared.plan_request.vehicle = vehicle;
  prepared.plan_request.start = path_case.start;
  prepared.plan_request.goal = path_case.goal;
  // 正确性运行使用文档规定的 60 s 预算，使搜索能完成完整
  // 不可行性证明（后端默认仍为 5 s）。
  prepared.plan_request.time_budget_s = 60.0;
  return prepared;
}

struct ChainResult {
  common::PlanResult plan;
  corridor::CorridorResult corridor;
  OptimizeResult optimize;
  bool plan_ok{false};
  bool corridor_ok{false};
  // 当生成的走廊被拒绝（障碍物附近的刀刃间隙）而改用沿同一规划路径的
  // 局部车道走廊时为 true。
  bool corridor_fallback{false};
  // 实际使用的 Hybrid A* 后端 id（诊断用）。
  std::string backend_id;
};

// 为一个夹具用例运行完整链路。`overrides` 在默认值之后作用于优化器请求
// （限制/选项）；`scene_override` 替换优化器的验证场景，而 Hybrid A* 规划
// 与走廊仍使用该用例的几何（供验证拒绝的失败链路使用）；`backend_id` 选择
// Hybrid A* 归一化后端（默认与既有链路一致：ros_free；见
// planner/hybrid_astar/backend.hpp 的 create_backend 注册表）。
inline ChainResult run_chain(const PathCase& path_case,
                             const common::VehicleModel& vehicle,
                             const std::function<void(OptimizeRequest*)>& overrides = {},
                             const collision::Scene* scene_override = nullptr,
                             bool force_lane_corridor = false,
                             double fallback_clearance_m = 0.15,
                             const std::string& backend_id = "ros_free") {
  ChainResult chain;
  chain.backend_id = backend_id;
  PreparedCase prepared = prepare_case(path_case, vehicle);
  if (!prepared.checker.valid()) {
    throw std::runtime_error("invalid collision checker: " + prepared.checker.invalid_reason());
  }

  const std::unique_ptr<hybrid_astar::HybridAStarBackend> backend =
      hybrid_astar::create_backend(backend_id);
  if (backend == nullptr) {
    throw std::runtime_error("unknown hybrid A* backend: " + backend_id);
  }
  const hybrid_astar::ConfigReport report = backend->configure(nlohmann::json::object());
  if (!report.accepted) {
    throw std::runtime_error("hybrid A* configure rejected: " + report.reason);
  }
  hybrid_astar::PlannerRsProvider provider(vehicle);
  chain.plan = backend->plan(prepared.plan_request, prepared.grid, prepared.checker, provider);
  chain.plan_ok = chain.plan.status == common::Status::kSuccess && !chain.plan.path.empty();
  if (!chain.plan_ok) {
    return chain;
  }
  chain.plan.path = snap_path_junctions(chain.plan.path);

  corridor::CorridorRequest corridor_request;
  corridor_request.request_id = path_case.id + "_corridor";
  corridor_request.map_id = path_case.grid.map_id;
  corridor_request.map_version = path_case.grid.map_version;
  corridor_request.vehicle = vehicle;
  corridor_request.seed = chain.plan.path;
  corridor_request.obstacles = build_corridor_obstacles(path_case);
  if (!force_lane_corridor) {
    chain.corridor = corridor::generate_corridor(corridor_request);
    chain.corridor_ok = chain.corridor.ok();
  } else {
    chain.corridor.status = common::Status::kNoPath;
    chain.corridor.reason = "forced_fallback";
    chain.corridor.detail = "lane corridor fallback requested for this case";
  }
  if (!chain.corridor_ok) {
    // 针对刀刃式生成走廊的文档化回退方案（生成器拒绝障碍物附近
    // 低于其最小值的间隙）：沿同一规划路径由保守足迹构建局部车道走廊。
    const std::string generator_failure =
        chain.corridor.reason + " " + chain.corridor.detail;
    common::Corridor fallback = test::make_footprint_lane_corridor(
        chain.plan.path, vehicle, 0.5, fallback_clearance_m);
    if (!fallback.polygons.empty()) {
      fallback.id = path_case.id + "_corridor_fallback";
      fallback.map_id = path_case.grid.map_id;
      fallback.map_version = path_case.grid.map_version;
      fallback.frame_id = "map";
      chain.corridor.corridor = std::move(fallback);
      chain.corridor.status = common::Status::kSuccess;
      chain.corridor.reason.clear();
      chain.corridor.detail = "generator failed (" + generator_failure +
                              "); local lane corridor fallback used";
      chain.corridor_ok = true;
      chain.corridor_fallback = true;
    }
  }
  if (!chain.corridor_ok) {
    return chain;
  }

  OptimizeRequest request;
  request.request_id = path_case.id + "_optimize";
  request.map_id = path_case.grid.map_id;
  request.map_version = path_case.grid.map_version;
  request.vehicle = vehicle;
  request.seed = chain.plan.path;
  request.corridor = chain.corridor.corridor;
  request.goal = path_case.goal;
  request.scene = scene_override != nullptr ? *scene_override : prepared.scene;
  request.limits = limits_for_vehicle(vehicle);
  if (overrides) {
    overrides(&request);
  }
  chain.optimize = optimize_trajectory(request);
  return chain;
}

inline double polyline_length(const std::vector<common::PathSegment>& path) {
  double total = 0.0;
  for (const auto& segment : path) {
    for (std::size_t i = 1; i < segment.points.size(); ++i) {
      total += std::hypot(segment.points[i].pose.x_m - segment.points[i - 1].pose.x_m,
                          segment.points[i].pose.y_m - segment.points[i - 1].pose.y_m);
    }
  }
  return total;
}

inline double trajectory_length(const std::vector<common::TrajectoryPoint>& trajectory) {
  double total = 0.0;
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    total += std::hypot(trajectory[i].pose.x_m - trajectory[i - 1].pose.x_m,
                        trajectory[i].pose.y_m - trajectory[i - 1].pose.y_m);
  }
  return total;
}

// 包含性判定失败时的诊断上下文：失败点附近的轨迹窗口、
// 累计弧长和覆盖的走廊多边形。
inline std::string describe_containment_context(const ChainResult& chain,
                                                const OptimizeResult& result) {
  if (result.validation.valid || result.trajectory.empty()) {
    return {};
  }
  const int idx = result.validation.first_failure.point_index;
  if (idx < 0) {
    return {};
  }
  std::string text = " fallback=" + std::to_string(chain.corridor_fallback ? 1 : 0);
  text += " window=" + test::trajectory_window_diag(result.trajectory, idx);
  double s_cursor = 0.0;
  for (int i = 0; i < idx && i + 1 < static_cast<int>(result.trajectory.size()); ++i) {
    s_cursor += std::hypot(result.trajectory[i + 1].pose.x_m - result.trajectory[i].pose.x_m,
                           result.trajectory[i + 1].pose.y_m - result.trajectory[i].pose.y_m);
  }
  text += " s_cursor=" + std::to_string(s_cursor);
  for (const auto& polygon : chain.corridor.corridor.polygons) {
    if (s_cursor >= polygon.s_start_m - 1e-9 && s_cursor <= polygon.s_end_m + 1e-9) {
      text += " polygon=[" + std::to_string(polygon.s_start_m) + "," +
              std::to_string(polygon.s_end_m) + ":";
      for (const auto& vertex : polygon.vertices) {
        text += " (" + std::to_string(vertex.x_m) + "," + std::to_string(vertex.y_m) + ")";
      }
      text += "]";
      break;
    }
  }
  return text;
}

// 在预制初值上运行走廊 -> 优化器 -> validation 链路
// （用于把优化器/修复与 Hybrid A* 后端隔离）。
inline ChainResult run_prefab_chain(const PathCase& path_case,
                                    const common::VehicleModel& vehicle,
                                    const std::vector<common::PathSegment>& seed,
                                    const std::function<void(OptimizeRequest*)>& overrides = {}) {
  ChainResult chain;
  PreparedCase prepared = prepare_case(path_case, vehicle);
  chain.plan.status = common::Status::kSuccess;
  chain.plan.path = seed;
  chain.plan_ok = true;

  corridor::CorridorRequest corridor_request;
  corridor_request.request_id = path_case.id + "_prefab_corridor";
  corridor_request.map_id = path_case.grid.map_id;
  corridor_request.map_version = path_case.grid.map_version;
  corridor_request.vehicle = vehicle;
  corridor_request.seed = seed;
  corridor_request.obstacles = build_corridor_obstacles(path_case);
  chain.corridor = corridor::generate_corridor(corridor_request);
  chain.corridor_ok = chain.corridor.ok();
  if (!chain.corridor_ok) {
    return chain;
  }

  OptimizeRequest request;
  request.request_id = path_case.id + "_prefab_optimize";
  request.map_id = path_case.grid.map_id;
  request.map_version = path_case.grid.map_version;
  request.vehicle = vehicle;
  request.seed = seed;
  request.corridor = chain.corridor.corridor;
  request.goal = path_case.goal;
  request.scene = prepared.scene;
  request.limits = limits_for_vehicle(vehicle);
  if (overrides) {
    overrides(&request);
  }
  chain.optimize = optimize_trajectory(request);
  return chain;
}

// 列出每个路径段的首/末位姿（用于诊断连接点间隙）。
inline std::string describe_path_junctions(const std::vector<common::PathSegment>& path) {
  std::string text;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (path[i].points.empty()) {
      continue;
    }
    const auto& first = path[i].points.front().pose;
    const auto& last = path[i].points.back().pose;
    text += " seg" + std::to_string(i) + "(" + std::to_string(first.x_m) + "," +
            std::to_string(first.y_m) + ")->(" + std::to_string(last.x_m) + "," +
            std::to_string(last.y_m) + ")";
  }
  return text;
}

inline std::string describe_attempts(const OptimizeResult& result) {
  std::string text;
  for (std::size_t i = 0; i < result.stats.attempt_reasons.size(); ++i) {
    text += " attempt" + std::to_string(i) + "=" + result.stats.attempt_reasons[i];
  }
  return text;
}

inline std::string describe_first_failure(const OptimizeResult& result) {
  if (result.validation.valid) {
    return "validation passed";
  }
  const validation::ValidationViolation& failure = result.validation.first_failure;
  return std::string("validation[") + result.validation.reason + "] at t=" +
         std::to_string(failure.time_s) + " point=" + std::to_string(failure.point_index) +
         " actual=" + std::to_string(failure.actual) + " limit=" +
         std::to_string(failure.limit) + " detail=" + failure.detail;
}

// ---------------------------------------------------------------------------
// 独立自交检查（hybrid-004 核查：Dftpav 优化/平滑阶段对小折返初值的处理）。
//
// 对采样折线做 O(n^2) 闭线段相交检测；相邻段共享端点、连续重复点（原地
// 驻留）不计。与 hybrid_astar 的跨组件自交清单同一语义：交点接触也算，
// 并通过线段叉积区分"非共线横切"（有害）与"共线接触/重叠"（折返痕迹）。
// 非共线横切进一步区分"显著"（两段均长于 kMicroLoopSegmentLengthM）与
// 亚厘米级"原地微环"（换向尖点处的数值回溯）；本检查只消费公开轨迹类型，
// 独立于优化器内部实现。
// ---------------------------------------------------------------------------

// 亚厘米级原地微环的线段长度阈值：小于该值的两段相交视为数值回溯而非
// 几何折返（报告仍计入 total，但不计入 significant）。
inline constexpr double kMicroLoopSegmentLengthM = 0.01;

struct SelfIntersectionCount {
  int total{0};
  int noncollinear{0};
  int collinear{0};
  int significant_noncollinear{0};
  std::size_t first_i{0};
  std::size_t first_j{0};
  double first_x{0.0};
  double first_y{0.0};
  double first_len_i{0.0};
  double first_len_j{0.0};
};

inline double segment_cross(const common::Point2d& a, const common::Point2d& b,
                            const common::Point2d& c) {
  return (b.x_m - a.x_m) * (c.y_m - a.y_m) - (b.y_m - a.y_m) * (c.x_m - a.x_m);
}

inline bool point_within_segment_bounds(const common::Point2d& a, const common::Point2d& b,
                                        const common::Point2d& p, double tolerance) {
  return std::min(a.x_m, b.x_m) - tolerance <= p.x_m &&
         p.x_m <= std::max(a.x_m, b.x_m) + tolerance &&
         std::min(a.y_m, b.y_m) - tolerance <= p.y_m &&
         p.y_m <= std::max(a.y_m, b.y_m) + tolerance;
}

// 闭线段相交（接触即算）。
inline bool closed_segments_intersect(const common::Point2d& a, const common::Point2d& b,
                                      const common::Point2d& c, const common::Point2d& d,
                                      double tolerance) {
  const double o1 = segment_cross(a, b, c);
  const double o2 = segment_cross(a, b, d);
  const double o3 = segment_cross(c, d, a);
  const double o4 = segment_cross(c, d, b);
  const auto sign = [tolerance](double value) {
    return value > tolerance ? 1 : (value < -tolerance ? -1 : 0);
  };
  if (sign(o1) != sign(o2) && sign(o3) != sign(o4)) {
    return true;
  }
  if (sign(o1) == 0 && point_within_segment_bounds(a, b, c, tolerance)) return true;
  if (sign(o2) == 0 && point_within_segment_bounds(a, b, d, tolerance)) return true;
  if (sign(o3) == 0 && point_within_segment_bounds(c, d, a, tolerance)) return true;
  if (sign(o4) == 0 && point_within_segment_bounds(c, d, b, tolerance)) return true;
  return false;
}

// 合并连续重复点（距离 <= tolerance），保持折线没有零长步进。
inline std::vector<common::Point2d> dedupe_polyline(const std::vector<common::Point2d>& points,
                                                    double tolerance = 1e-9) {
  std::vector<common::Point2d> deduped;
  for (const common::Point2d& point : points) {
    if (!deduped.empty() &&
        std::hypot(point.x_m - deduped.back().x_m, point.y_m - deduped.back().y_m) <=
            tolerance) {
      continue;
    }
    deduped.push_back(point);
  }
  return deduped;
}

inline std::vector<common::Point2d> flatten_seed_polyline(
    const std::vector<common::PathSegment>& seed, double tolerance = 1e-9) {
  std::vector<common::Point2d> points;
  for (const common::PathSegment& segment : seed) {
    for (const common::PathPoint& path_point : segment.points) {
      points.push_back({path_point.pose.x_m, path_point.pose.y_m});
    }
  }
  return dedupe_polyline(points, tolerance);
}

inline std::vector<common::Point2d> trajectory_polyline(
    const std::vector<common::TrajectoryPoint>& trajectory, double tolerance = 1e-9) {
  std::vector<common::Point2d> points;
  points.reserve(trajectory.size());
  for (const common::TrajectoryPoint& point : trajectory) {
    points.push_back({point.pose.x_m, point.pose.y_m});
  }
  return dedupe_polyline(points, tolerance);
}

inline SelfIntersectionCount count_self_intersections(const std::vector<common::Point2d>& polyline,
                                                      double tolerance = 1e-9) {
  SelfIntersectionCount count;
  const std::size_t size = polyline.size();
  for (std::size_t i = 0; i + 1 < size; ++i) {
    for (std::size_t j = i + 2; j + 1 < size; ++j) {
      const common::Point2d& a = polyline[i];
      const common::Point2d& b = polyline[i + 1];
      const common::Point2d& c = polyline[j];
      const common::Point2d& d = polyline[j + 1];
      if (!closed_segments_intersect(a, b, c, d, tolerance)) {
        continue;
      }
      const bool collinear = std::abs(segment_cross(a, b, c)) <= tolerance &&
                             std::abs(segment_cross(a, b, d)) <= tolerance;
      const double length_i = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
      const double length_j = std::hypot(d.x_m - c.x_m, d.y_m - c.y_m);
      ++count.total;
      if (collinear) {
        ++count.collinear;
      } else {
        ++count.noncollinear;
        if (std::min(length_i, length_j) > kMicroLoopSegmentLengthM) {
          ++count.significant_noncollinear;
        }
      }
      if (count.total == 1) {
        count.first_i = i;
        count.first_j = j;
        count.first_x = 0.5 * (a.x_m + b.x_m);
        count.first_y = 0.5 * (a.y_m + b.y_m);
        count.first_len_i = length_i;
        count.first_len_j = length_j;
      }
    }
  }
  return count;
}

inline std::string describe_self_intersections(const SelfIntersectionCount& count) {
  std::string text = "intersections=" + std::to_string(count.total) +
                     "(noncollinear=" + std::to_string(count.noncollinear) +
                     ",collinear=" + std::to_string(count.collinear) +
                     ",significant=" + std::to_string(count.significant_noncollinear) + ")";
  if (count.total > 0) {
    text += " first=(i=" + std::to_string(count.first_i) + ",j=" + std::to_string(count.first_j) +
            ",at " + std::to_string(count.first_x) + "," + std::to_string(count.first_y) +
            ",len_i=" + std::to_string(count.first_len_i) +
            ",len_j=" + std::to_string(count.first_len_j) + ")";
  }
  return text;
}

}  // namespace planner::dftpav::e2e
