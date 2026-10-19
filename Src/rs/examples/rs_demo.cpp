// planner_rs_demo：规划一条 Reeds-Shepp 路径并打印各分段与采样点。
// Docs/component_development_plan.md 6.2 的教学示例。
//
// 用法：
//   planner_rs_demo x1 y1 yaw1_deg x2 y2 yaw2_deg [--ds 0.5]
//   planner_rs_demo --self-check
//
// 默认车辆为冻结的基准车辆 (7.1)：轴距 2.8 m，最大转向角 0.5 rad。
// 命令行中的角度以度为单位；输出打印弧度、以米为单位的 s_m 和以 1/m
// 为单位的曲率。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "planner/rs/reeds_shepp.hpp"

namespace {

planner::common::VehicleModel benchmark_vehicle() {
  planner::common::VehicleModel vehicle;
  vehicle.id = "v1_benchmark";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

int self_check() {
  using planner::common::Direction;
  using planner::common::Pose2d;
  const planner::common::VehicleModel vehicle = benchmark_vehicle();
  const double radius = planner::rs::min_turn_radius_m(vehicle);
  int failures = 0;

  const double straight = planner::rs::distance({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, vehicle);
  if (std::fabs(straight - 10.0) > 1e-9) {
    std::cerr << "self-check straight failed: " << straight << "\n";
    ++failures;
  }

  const double quarter = planner::rs::distance(
      {0.0, 0.0, 0.0}, {radius, radius, 0.5 * planner::common::kPi}, vehicle);
  if (std::fabs(quarter - radius * 0.5 * planner::common::kPi) > 1e-9) {
    std::cerr << "self-check quarter turn failed: " << quarter << "\n";
    ++failures;
  }

  const planner::rs::RsPath cusp =
      planner::rs::path({0.0, 0.0, 0.0}, {0.0, 0.0, planner::common::kPi}, vehicle);
  bool has_cusp = false;
  for (std::size_t i = 1; i < cusp.segments.size(); ++i) {
    if (cusp.segments[i].direction != cusp.segments[i - 1].direction) {
      has_cusp = true;
    }
  }
  if (!has_cusp) {
    std::cerr << "self-check cusp failed: no direction change\n";
    ++failures;
  }

  const std::vector<planner::common::PathPoint> points = planner::rs::sample(cusp, 0.25);
  if (points.empty() ||
      std::fabs(points.back().pose.x_m - cusp.goal.x_m) > 1e-9 ||
      std::fabs(points.back().pose.y_m - cusp.goal.y_m) > 1e-9 ||
      std::fabs(points.back().pose.yaw_rad - cusp.goal.yaw_rad) > 1e-9) {
    std::cerr << "self-check sampling failed\n";
    ++failures;
  }
  if (failures == 0) {
    std::cout << "planner_rs self-check: PASS (straight, quarter turn, cusp, sampling)\n";
  }
  return failures == 0 ? 0 : 1;
}

void print_path(const planner::rs::RsPath& p, double ds) {
  std::cout << "length_m=" << std::fixed << std::setprecision(6) << p.length_m
            << " segments=" << p.segments.size() << "\n";
  std::cout << "start: x=" << p.start.x_m << " y=" << p.start.y_m
            << " yaw=" << p.start.yaw_rad << "\n";
  std::cout << "goal : x=" << p.goal.x_m << " y=" << p.goal.y_m
            << " yaw=" << p.goal.yaw_rad << "\n";
  std::cout << "\nsegment | direction | length_m | curvature_inv_m\n";
  for (const planner::rs::RsSegment& segment : p.segments) {
    std::cout << "      - | " << planner::common::to_string(segment.direction) << " | "
              << segment.length_m << " | " << segment.curvature_inv_m << "\n";
  }
  std::cout << "\ns_m | x_m | y_m | yaw_rad | curvature_inv_m | direction\n";
  for (const planner::common::PathPoint& point : planner::rs::sample(p, ds)) {
    std::cout << point.s_m << " | " << point.pose.x_m << " | " << point.pose.y_m
              << " | " << point.pose.yaw_rad << " | " << point.curvature_inv_m
              << " | " << planner::common::to_string(point.direction) << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  for (const std::string& arg : args) {
    if (arg == "--self-check") {
      return self_check();
    }
    if (arg == "--help" || arg == "-h") {
      std::cout << "usage: planner_rs_demo x1 y1 yaw1_deg x2 y2 yaw2_deg [--ds 0.5]\n"
                   "       planner_rs_demo --self-check\n";
      return 0;
    }
  }

  double ds = 0.5;
  std::vector<double> values;
  for (std::size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--ds" && i + 1 < args.size()) {
      ds = std::stod(args[++i]);
      continue;
    }
    try {
      values.push_back(std::stod(args[i]));
    } catch (const std::exception&) {
      std::cerr << "unrecognized argument: " << args[i] << "\n";
      return 2;
    }
  }
  if (values.size() != 6) {
    std::cerr << "expected 6 numbers: x1 y1 yaw1_deg x2 y2 yaw2_deg\n";
    return 2;
  }
  const double deg = planner::common::kPi / 180.0;
  const planner::common::Pose2d start{values[0], values[1], values[2] * deg};
  const planner::common::Pose2d goal{values[3], values[4], values[5] * deg};
  const planner::common::VehicleModel vehicle = benchmark_vehicle();

  const planner::rs::RsPlan result = planner::rs::plan(start, goal, vehicle);
  if (result.status != planner::common::Status::kSuccess) {
    std::cerr << "plan failed: status=" << planner::common::to_string(result.status)
              << " reason=" << result.reason << "\n";
    return 1;
  }
  std::cout << "vehicle: wheelbase=" << vehicle.wheelbase_m
            << " max_steer=" << vehicle.max_steer_rad
            << " min_turn_radius=" << planner::rs::min_turn_radius_m(vehicle) << "\n";
  print_path(result.path, ds);
  return 0;
}
