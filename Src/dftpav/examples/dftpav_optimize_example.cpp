// planner_dftpav 命令行示例。
//
// 场景（均使用第 7.1 节的默认车辆）：
//   straight       直线 10 m 初值、宽松走廊      -> success
//   reverse        前进 6 m 再倒车 3 m           -> success + 停车
//   avoid          在局部车道走廊（公共走廊类型）内绕开障碍物，平滑迂回
//                                               -> success
//   infeasible     L 形窄走廊 + 0.05 1/m 限制    -> 显式失败
//   empty-corridor 无多边形的走廊                -> invalid_input
//
// 用法：
//   planner_dftpav_example [--scenario <name>] [--output result.json]
//   planner_dftpav_example --self-check
//
// 使用 --output 时，结果写为 planner::common::PlanResult JSON 文档
// （schema_version 1）。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "planner/common/serialization.hpp"
#include "planner/common/types.hpp"
#include "planner/corridor/corridor_generator.hpp"
#include "planner/dftpav/optimizer.hpp"

namespace {

using planner::common::Corridor;
using planner::common::PathSegment;
using planner::common::Point2d;
using planner::common::TrajectoryPoint;
using planner::common::VehicleModel;
using planner::dftpav::OptimizeRequest;
using planner::dftpav::OptimizeResult;
using planner::dftpav::optimize_trajectory;

struct Rect {
  double x_min{0.0};
  double y_min{0.0};
  double x_max{0.0};
  double y_max{0.0};
};

VehicleModel default_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "example_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

planner::dftpav::OptimizeLimits default_limits() {
  planner::dftpav::OptimizeLimits limits;
  return limits;
}

PathSegment make_segment(const std::vector<Point2d>& points, bool reverse, double step_m) {
  PathSegment segment;
  double s = 0.0;
  std::vector<Point2d> sampled;
  sampled.push_back(points.front());
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const Point2d& a = points[i];
    const Point2d& b = points[i + 1];
    const double length = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    const int count = std::max(1, static_cast<int>(std::ceil(length / step_m)));
    for (int k = 1; k <= count; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(count);
      sampled.push_back({a.x_m + t * (b.x_m - a.x_m), a.y_m + t * (b.y_m - a.y_m)});
    }
  }
  for (std::size_t i = 0; i < sampled.size(); ++i) {
    if (i > 0) {
      s += std::hypot(sampled[i].x_m - sampled[i - 1].x_m, sampled[i].y_m - sampled[i - 1].y_m);
    }
    planner::common::PathPoint point;
    point.pose.x_m = sampled[i].x_m;
    point.pose.y_m = sampled[i].y_m;
    if (i + 1 < sampled.size()) {
      double yaw = std::atan2(sampled[i + 1].y_m - sampled[i].y_m,
                              sampled[i + 1].x_m - sampled[i].x_m);
      if (reverse) {
        yaw += planner::common::kPi;
      }
      point.pose.yaw_rad = planner::common::normalize_angle_rad(yaw);
    } else {
      point.pose.yaw_rad = segment.points.empty() ? 0.0 : segment.points.back().pose.yaw_rad;
    }
    point.s_m = s;
    point.curvature_inv_m = 0.0;
    point.direction = reverse ? planner::common::Direction::kReverse
                              : planner::common::Direction::kForward;
    segment.points.push_back(point);
  }
  return segment;
}

// 圆角折线初值：直线段之间用相切圆弧连接（与组件测试相同的
// 构造方式；满足运动学、曲率有界）。
PathSegment make_fillet_segment(const std::vector<Point2d>& waypoints, double radius_m,
                                double step_m) {
  std::vector<Point2d> points;
  std::vector<double> yaws;
  auto unit = [](const Point2d& a, const Point2d& b) {
    const double d = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    return Point2d{(b.x_m - a.x_m) / d, (b.y_m - a.y_m) / d};
  };
  auto push_straight = [&](const Point2d& from, const Point2d& to) {
    const double length = std::hypot(to.x_m - from.x_m, to.y_m - from.y_m);
    const int count = std::max(1, static_cast<int>(std::ceil(length / step_m)));
    const double yaw = std::atan2(to.y_m - from.y_m, to.x_m - from.x_m);
    for (int k = 1; k <= count; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(count);
      points.push_back({from.x_m + t * (to.x_m - from.x_m), from.y_m + t * (to.y_m - from.y_m)});
      yaws.push_back(yaw);
    }
  };
  points.push_back(waypoints.front());
  yaws.push_back(std::atan2(waypoints[1].y_m - waypoints[0].y_m,
                            waypoints[1].x_m - waypoints[0].x_m));
  Point2d current = waypoints.front();
  for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
    const Point2d u = unit(waypoints[i - 1], waypoints[i]);
    const Point2d v = unit(waypoints[i], waypoints[i + 1]);
    const double cross = u.x_m * v.y_m - u.y_m * v.x_m;
    const double dot = u.x_m * v.x_m + u.y_m * v.y_m;
    const double phi = std::atan2(cross, dot);
    if (std::abs(phi) < 1e-9) {
      continue;
    }
    const double tangent = radius_m * std::tan(std::abs(phi) / 2.0);
    const Point2d p_in{waypoints[i].x_m - tangent * u.x_m, waypoints[i].y_m - tangent * u.y_m};
    const Point2d p_out{waypoints[i].x_m + tangent * v.x_m, waypoints[i].y_m + tangent * v.y_m};
    push_straight(current, p_in);
    const double sign = phi > 0 ? 1.0 : -1.0;
    const Point2d center{p_in.x_m - sign * radius_m * u.y_m, p_in.y_m + sign * radius_m * u.x_m};
    const double angle_in = std::atan2(p_in.y_m - center.y_m, p_in.x_m - center.x_m);
    const double angle_out = std::atan2(p_out.y_m - center.y_m, p_out.x_m - center.x_m);
    double sweep = angle_out - angle_in;
    while (sweep <= -planner::common::kPi) sweep += 2.0 * planner::common::kPi;
    while (sweep > planner::common::kPi) sweep -= 2.0 * planner::common::kPi;
    const int count =
        std::max(1, static_cast<int>(std::ceil(std::abs(sweep) * radius_m / step_m)));
    for (int k = 1; k <= count; ++k) {
      const double angle = angle_in + sweep * static_cast<double>(k) / count;
      points.push_back(
          {center.x_m + radius_m * std::cos(angle), center.y_m + radius_m * std::sin(angle)});
      yaws.push_back(planner::common::normalize_angle_rad(
          std::atan2(sign * std::cos(angle), -sign * std::sin(angle))));
    }
    current = p_out;
  }
  push_straight(current, waypoints.back());

  PathSegment segment;
  double s = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      s += std::hypot(points[i].x_m - points[i - 1].x_m, points[i].y_m - points[i - 1].y_m);
    }
    planner::common::PathPoint path_point;
    path_point.pose.x_m = points[i].x_m;
    path_point.pose.y_m = points[i].y_m;
    path_point.pose.yaw_rad = yaws[i];
    path_point.s_m = s;
    path_point.curvature_inv_m = 0.0;
    path_point.direction = planner::common::Direction::kForward;
    segment.points.push_back(path_point);
  }
  return segment;
}

// 整车矩形的四个角点：矩形中心位于后轴前方 `d_cr` 处，按 `margin_m` 向外扩张。
std::vector<planner::common::Point2d> vehicle_footprint(
    const VehicleModel& vehicle, const planner::common::Pose2d& pose, double margin_m) {
  const double length = vehicle.wheelbase_m + vehicle.front_overhang_m + vehicle.rear_overhang_m;
  const double d_cr =
      (vehicle.front_overhang_m + vehicle.wheelbase_m - vehicle.rear_overhang_m) / 2.0;
  const double half_length = 0.5 * (length + 2.0 * margin_m);
  const double half_width = 0.5 * (vehicle.width_m + 2.0 * margin_m);
  const double cos_yaw = std::cos(pose.yaw_rad);
  const double sin_yaw = std::sin(pose.yaw_rad);
  const double corners[4][2] = {{d_cr + half_length, half_width},
                                {d_cr + half_length, -half_width},
                                {d_cr - half_length, -half_width},
                                {d_cr - half_length, half_width}};
  std::vector<planner::common::Point2d> footprint;
  for (const auto& corner : corners) {
    footprint.push_back({pose.x_m + cos_yaw * corner[0] - sin_yaw * corner[1],
                         pose.y_m + sin_yaw * corner[0] + cos_yaw * corner[1]});
  }
  return footprint;
}

// 沿初值的局部车道走廊：弧长每 `step_m` 一个矩形，其大小取区间内保守整车
// 足迹的包围盒再外扩 `clearance_m`（与组件测试相同的构造方式）。
Corridor lane_corridor_from_seed(const std::vector<PathSegment>& seed,
                                 const VehicleModel& vehicle, double step_m,
                                 double clearance_m) {
  struct Sample {
    planner::common::Pose2d pose;
    double s{0.0};
  };
  std::vector<Sample> samples;
  double s = 0.0;
  for (const auto& segment : seed) {
    for (const auto& point : segment.points) {
      if (!samples.empty()) {
        s += std::hypot(point.pose.x_m - samples.back().pose.x_m,
                        point.pose.y_m - samples.back().pose.y_m);
      }
      samples.push_back({point.pose, s});
    }
  }
  Corridor corridor;
  corridor.id = "example_lane_corridor";
  corridor.map_id = "example_map";
  corridor.map_version = "v1";
  corridor.frame_id = "map";
  const double total = samples.empty() ? 0.0 : samples.back().s;
  auto pose_at = [&samples](double s_query) {
    if (s_query <= samples.front().s) {
      return samples.front().pose;
    }
    if (s_query >= samples.back().s) {
      return samples.back().pose;
    }
    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
      if (s_query <= samples[i + 1].s) {
        const double ds = samples[i + 1].s - samples[i].s;
        const double t = ds > 1e-12 ? (s_query - samples[i].s) / ds : 0.0;
        planner::common::Pose2d pose;
        pose.x_m = samples[i].pose.x_m +
                   t * (samples[i + 1].pose.x_m - samples[i].pose.x_m);
        pose.y_m = samples[i].pose.y_m +
                   t * (samples[i + 1].pose.y_m - samples[i].pose.y_m);
        const double delta = planner::common::normalize_angle_rad(
            samples[i + 1].pose.yaw_rad - samples[i].pose.yaw_rad);
        pose.yaw_rad =
            planner::common::normalize_angle_rad(samples[i].pose.yaw_rad + t * delta);
        return pose;
      }
    }
    return samples.back().pose;
  };
  double s_begin = 0.0;
  while (s_begin < total - 1e-9) {
    const double s_end = std::min(s_begin + step_m, total);
    double x_min = 1e9;
    double y_min = 1e9;
    double x_max = -1e9;
    double y_max = -1e9;
    for (const double s_station : {s_begin, s_end}) {
      for (const auto& corner :
           vehicle_footprint(vehicle, pose_at(s_station), vehicle.safety_margin_m)) {
        x_min = std::min(x_min, corner.x_m);
        y_min = std::min(y_min, corner.y_m);
        x_max = std::max(x_max, corner.x_m);
        y_max = std::max(y_max, corner.y_m);
      }
    }
    planner::common::CorridorPolygon polygon;
    polygon.s_start_m = s_begin;
    polygon.s_end_m = s_end;
    polygon.vertices = {{x_min - clearance_m, y_min - clearance_m},
                        {x_max + clearance_m, y_min - clearance_m},
                        {x_max + clearance_m, y_max + clearance_m},
                        {x_min - clearance_m, y_max + clearance_m}};
    corridor.polygons.push_back(polygon);
    s_begin = s_end;
  }
  return corridor;
}

Corridor rect_corridor(
    const std::vector<std::pair<double, double>>& intervals,
    const std::vector<Rect>& rectangles) {
  Corridor corridor;
  corridor.id = "example_corridor";
  corridor.map_id = "example_map";
  corridor.map_version = "v1";
  corridor.frame_id = "map";
  for (std::size_t i = 0; i < intervals.size(); ++i) {
    planner::common::CorridorPolygon polygon;
    polygon.s_start_m = intervals[i].first;
    polygon.s_end_m = intervals[i].second;
    polygon.vertices = {{rectangles[i].x_min, rectangles[i].y_min},
                        {rectangles[i].x_max, rectangles[i].y_min},
                        {rectangles[i].x_max, rectangles[i].y_max},
                        {rectangles[i].x_min, rectangles[i].y_max}};
    corridor.polygons.push_back(polygon);
  }
  return corridor;
}

OptimizeRequest make_straight_request() {
  OptimizeRequest request;
  request.request_id = "example-straight";
  request.map_id = "example_map";
  request.map_version = "v1";
  request.vehicle = default_vehicle();
  request.seed = {make_segment({{0.0, 0.0}, {10.0, 0.0}}, false, 1.0)};
  request.corridor = rect_corridor({{0.0, 10.0}}, {Rect{-3.0, -4.0, 15.0, 4.0}});
  request.corridor.map_id = request.map_id;
  request.corridor.map_version = request.map_version;
  request.limits = default_limits();
  return request;
}

OptimizeRequest make_reverse_request() {
  OptimizeRequest request;
  request.request_id = "example-reverse";
  request.map_id = "example_map";
  request.map_version = "v1";
  request.vehicle = default_vehicle();
  request.seed = {make_segment({{0.0, 0.0}, {6.0, 0.0}}, false, 1.0),
                  make_segment({{6.0, 0.0}, {3.0, 0.0}}, true, 1.0)};
  // 车道局部走廊多边形：沿（往返）路径每 0.5 m 一个矩形，其大小覆盖
  // 朝 +x 方向的整车，从而强制产生尖点。
  std::vector<std::pair<double, double>> intervals;
  std::vector<Rect> rectangles;
  const double vehicle_rear = 1.1;
  const double vehicle_front = 3.8;
  for (double s = 0.0; s < 9.0 - 1e-9; s += 0.5) {
    const double s_end = std::min(s + 0.5, 9.0);
    const double x_begin = s <= 6.0 ? s : 6.0 - (s - 6.0);
    const double x_end = s_end <= 6.0 ? s_end : 6.0 - (s_end - 6.0);
    intervals.emplace_back(s, s_end);
    rectangles.push_back(Rect{std::min(x_begin, x_end) - vehicle_rear - 0.05,
                              -1.15,
                              std::max(x_begin, x_end) + vehicle_front + 0.05,
                              1.15});
  }
  request.corridor = lane_corridor_from_seed(request.seed, request.vehicle, 0.5, 0.5);
  request.corridor.map_id = request.map_id;
  request.corridor.map_version = request.map_version;
  request.limits = default_limits();
  request.options.anchor_weight = 10000.0;
  return request;
}

// 避障场景：初值在直线连接上平滑绕过障碍物；走廊是由保守足迹构建的
// 局部车道走廊（公共 planner::common::Corridor 类型），额外留出 0.4 m 间隙。
struct AvoidScenario {
  OptimizeRequest request;
  bool corridor_ok{false};
  std::string failure;
};

AvoidScenario make_avoid_scenario() {
  AvoidScenario scenario;
  scenario.request.request_id = "example-avoid";
  scenario.request.map_id = "example_map";
  scenario.request.map_version = "v1";
  scenario.request.vehicle = default_vehicle();
  scenario.request.seed = {make_fillet_segment(
      {{0.0, 0.0}, {4.0, 0.0}, {16.0, 6.0}, {28.0, 0.0}, {34.0, 0.0}}, 12.0, 0.1)};
  scenario.request.corridor = lane_corridor_from_seed(
      scenario.request.seed, scenario.request.vehicle, 0.5, 0.6);
  scenario.request.limits = default_limits();
  scenario.request.options.anchor_weight = 10000.0;
  scenario.corridor_ok = !scenario.request.corridor.polygons.empty();
  if (!scenario.corridor_ok) {
    scenario.failure = "lane corridor is empty";
  }
  return scenario;
}

OptimizeRequest make_infeasible_request() {
  OptimizeRequest request;
  request.request_id = "example-infeasible";
  request.map_id = "example_map";
  request.map_version = "v1";
  request.vehicle = default_vehicle();
  request.seed = {make_segment({{0.0, 0.0}, {5.0, 0.0}}, false, 1.0),
                  make_segment({{5.0, 0.0}, {5.0, 5.0}}, false, 1.0)};
  request.corridor = rect_corridor(
      {{0.0, 5.0}, {5.0, 10.0}},
      {Rect{-3.0, -1.6, 9.0, 1.6}, Rect{3.4, -3.0, 6.6, 9.0}});
  request.corridor.map_id = request.map_id;
  request.corridor.map_version = request.map_version;
  request.limits = default_limits();
  request.limits.max_forward_curvature_inv_m = 0.05;
  request.limits.max_backward_curvature_inv_m = 0.05;
  return request;
}

OptimizeRequest make_empty_corridor_request() {
  OptimizeRequest request = make_straight_request();
  request.request_id = "example-empty-corridor";
  request.corridor.polygons.clear();
  return request;
}

void print_result(const OptimizeResult& result) {
  std::printf("status=%s reason=%s\n",
              planner::common::to_string(result.status).c_str(), result.reason.c_str());
  if (!result.detail.empty()) {
    std::printf("detail=%s\n", result.detail.c_str());
  }
  std::printf("points=%zu duration=%.3fs solve=%.3fs iterations=%d cost=%.3f\n",
              result.stats.trajectory_points, result.stats.trajectory_duration_s,
              result.stats.solve_time_s, result.stats.lbfgs_iterations,
              result.stats.final_cost);
  if (!result.trajectory.empty()) {
    const TrajectoryPoint& first = result.trajectory.front();
    const TrajectoryPoint& last = result.trajectory.back();
    std::printf("start=(%.3f, %.3f) end=(%.3f, %.3f) v_end=%.6f\n", first.pose.x_m,
                first.pose.y_m, last.pose.x_m, last.pose.y_m, last.velocity_mps);
  }
  for (double switch_time : result.stats.direction_switch_times_s) {
    std::printf("direction_change_t=%.3fs\n", switch_time);
  }
}

bool write_result_json(const std::string& path, const OptimizeResult& result) {
  planner::common::PlanResult plan_result;
  plan_result.request_id = "planner_dftpav_example";
  plan_result.status = result.status;
  plan_result.reason = result.reason;
  plan_result.trajectory = result.trajectory;
  plan_result.statistics = {{"direction_changes", result.stats.direction_changes},
                            {"total_pieces", result.stats.total_pieces},
                            {"lbfgs_iterations", result.stats.lbfgs_iterations},
                            {"lbfgs_result", result.stats.lbfgs_result},
                            {"final_cost", result.stats.final_cost},
                            {"solve_time_s", result.stats.solve_time_s},
                            {"trajectory_duration_s", result.stats.trajectory_duration_s},
                            {"trajectory_points", result.stats.trajectory_points},
                            {"corridor_constraint_points",
                             result.stats.corridor_constraint_points},
                            {"tightening_attempts", result.stats.tightening_attempts},
                            {"max_speed_mps", result.stats.max_speed_mps},
                            {"max_acceleration_mps2", result.stats.max_acceleration_mps2},
                            {"max_curvature_inv_m", result.stats.max_curvature_inv_m},
                            {"max_lateral_acceleration_mps2",
                             result.stats.max_lateral_acceleration_mps2},
                            {"detail", result.detail}};
  try {
    const std::string document = planner::common::encode_document(plan_result);
    if (std::FILE* file = std::fopen(path.c_str(), "wb")) {
      std::fwrite(document.data(), 1, document.size(), file);
      std::fclose(file);
      return true;
    }
  } catch (const std::exception& error) {
    std::fprintf(stderr, "failed to encode result: %s\n", error.what());
  }
  return false;
}

int run_scenario(const std::string& name, const std::string& output) {
  if (name == "straight") {
    const OptimizeResult result = optimize_trajectory(make_straight_request());
    print_result(result);
    if (!output.empty() && !write_result_json(output, result)) {
      return 2;
    }
    return result.ok() ? 0 : 1;
  }
  if (name == "reverse") {
    const OptimizeResult result = optimize_trajectory(make_reverse_request());
    print_result(result);
    if (!output.empty() && !write_result_json(output, result)) {
      return 2;
    }
    return result.ok() ? 0 : 1;
  }
  if (name == "avoid") {
    AvoidScenario scenario = make_avoid_scenario();
    if (!scenario.corridor_ok) {
      std::fprintf(stderr, "corridor generation failed: %s\n", scenario.failure.c_str());
      return 1;
    }
    const OptimizeResult result = optimize_trajectory(scenario.request);
    print_result(result);
    if (!output.empty() && !write_result_json(output, result)) {
      return 2;
    }
    return result.ok() ? 0 : 1;
  }
  if (name == "infeasible") {
    const OptimizeResult result = optimize_trajectory(make_infeasible_request());
    print_result(result);
    if (!output.empty() && !write_result_json(output, result)) {
      return 2;
    }
    return result.ok() ? 1 : 0;
  }
  if (name == "empty-corridor") {
    const OptimizeResult result = optimize_trajectory(make_empty_corridor_request());
    print_result(result);
    if (!output.empty() && !write_result_json(output, result)) {
      return 2;
    }
    return (!result.ok() && result.reason == "empty_corridor") ? 0 : 1;
  }
  std::fprintf(stderr, "unknown scenario: %s\n", name.c_str());
  return 2;
}

// CTest 使用的确定性自检（label dftpav）。
int self_check() {
  int failures = 0;
  struct Case {
    std::string name;
    bool expect_ok;
    std::string expect_reason;
  };
  const std::vector<Case> cases = {
      {"straight", true, ""},
      {"reverse", true, ""},
      {"avoid", true, ""},
      {"infeasible", false, ""},
      {"empty-corridor", false, "empty_corridor"},
  };
  for (const Case& test_case : cases) {
    OptimizeResult result;
    if (test_case.name == "straight") {
      result = optimize_trajectory(make_straight_request());
    } else if (test_case.name == "reverse") {
      result = optimize_trajectory(make_reverse_request());
    } else if (test_case.name == "avoid") {
      AvoidScenario scenario = make_avoid_scenario();
      if (!scenario.corridor_ok) {
        std::printf("FAIL avoid: corridor generation: %s\n", scenario.failure.c_str());
        ++failures;
        continue;
      }
      result = optimize_trajectory(scenario.request);
    } else if (test_case.name == "infeasible") {
      result = optimize_trajectory(make_infeasible_request());
    } else {
      result = optimize_trajectory(make_empty_corridor_request());
    }
    const bool ok_match = (result.ok() == test_case.expect_ok);
    const bool reason_match =
        test_case.expect_reason.empty() || result.reason == test_case.expect_reason;
    if (ok_match && reason_match) {
      std::printf("PASS %-16s status=%s reason=%s points=%zu\n", test_case.name.c_str(),
                  planner::common::to_string(result.status).c_str(), result.reason.c_str(),
                  result.stats.trajectory_points);
    } else {
      std::printf("FAIL %-16s status=%s reason=%s detail=%s\n", test_case.name.c_str(),
                  planner::common::to_string(result.status).c_str(), result.reason.c_str(),
                  result.detail.c_str());
      ++failures;
    }
    if (test_case.name == "reverse") {
      if (result.ok() && !result.stats.direction_switch_times_s.empty()) {
        const double switch_time = result.stats.direction_switch_times_s.front();
        double switch_speed = -1.0;
        for (const auto& point : result.trajectory) {
          if (std::abs(point.t_s - switch_time) <= 1e-9) {
            switch_speed = std::abs(point.velocity_mps);
          }
        }
        if (switch_speed < 0.0 || switch_speed > 1e-3) {
          std::printf("FAIL reverse: |v(switch)|=%.6f\n", switch_speed);
          ++failures;
        }
      }
    }
  }
  std::printf("self-check: %s (%d failure(s))\n", failures == 0 ? "OK" : "FAILED", failures);
  return failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  std::string scenario = "straight";
  std::string output;
  bool self_check_requested = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc) {
      scenario = argv[++i];
    } else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
      output = argv[++i];
    } else if (std::strcmp(argv[i], "--self-check") == 0) {
      self_check_requested = true;
    } else {
      std::fprintf(stderr,
                   "usage: %s [--scenario straight|reverse|avoid|infeasible|empty-corridor] "
                   "[--output <file.json>] [--self-check]\n",
                   argv[0]);
      return 2;
    }
  }
  if (self_check_requested) {
    return self_check();
  }
  return run_scenario(scenario, output);
}
