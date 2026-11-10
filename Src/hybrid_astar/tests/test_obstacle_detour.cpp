#include "user_astar.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

// 使用显式检查，保证 Release 构建也执行验证。
void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void plans_collision_free_detour() {
  using namespace planner;

  common::VehicleModel vehicle;
  vehicle.wheelbase_m = 2.5;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.8;
  vehicle.rear_overhang_m = 0.8;
  vehicle.max_steer_rad = 0.6;
  vehicle.safety_margin_m = 0.1;

  collision::Scene scene;
  auto& grid = scene.grid;
  grid.map_id = "obstacle_detour";
  grid.map_version = "1";
  grid.width = 80;
  grid.height = 80;
  grid.resolution_m = 0.5;
  grid.cells.assign(grid.width * grid.height, common::kCellFree);
  // 40 m × 40 m 地图；矩形障碍物 [19,21] × [8,12] 阻断起终点直线。
  for (int row = 16; row < 24; ++row) {
    for (int col = 38; col < 42; ++col) {
      grid.cells[row * grid.width + col] = common::kCellOccupied;
    }
  }

  collision::CollisionChecker checker(vehicle, scene);
  require(checker.valid(), "invalid test scene: " + checker.invalid_reason());

  common::PlanRequest request;
  request.request_id = "detour";
  request.map_id = grid.map_id;
  request.map_version = grid.map_version;
  request.vehicle = vehicle;
  request.start = {10.0, 10.0, 0.0};
  request.goal = {30.0, 10.0, 0.0};
  request.time_budget_s = 10.0;

  collision::MotionSegment direct;
  direct.start = request.start;
  direct.length_m = 20.0;
  require(checker.check_motion(direct).collided(),
          "test setup must block the direct route");

  hybrid_astar::UserAstar planner;
  const auto result = planner.plan(request, grid, checker);
  require(result.status == common::Status::kSuccess,
          "planning failed: " + common::to_string(result.status) + " " + result.reason);
  require(!result.path.empty(), "successful path must not be empty");
  for (const auto& segment : result.path) {
    require(!segment.points.empty(), "path segment must not be empty");
    for (const auto& point : segment.points) {
      require(std::isfinite(point.s_m) && std::isfinite(point.pose.x_m) &&
                  std::isfinite(point.pose.y_m) && std::isfinite(point.pose.yaw_rad) &&
                  std::isfinite(point.curvature_inv_m),
              "path contains nonfinite values");
      require(std::abs(point.curvature_inv_m) <=
                  1.0 / vehicle.min_turn_radius_m() + 1e-9,
              "path exceeds the vehicle curvature limit");
    }
  }
  const auto& start = result.path.front().points.front().pose;
  const auto& end = result.path.back().points.back();
  require(std::hypot(start.x_m - request.start.x_m, start.y_m - request.start.y_m) < 1e-6 &&
              std::abs(common::normalize_angle_rad(start.yaw_rad - request.start.yaw_rad)) < 1e-6,
          "path must begin at the requested start");
  require(std::hypot(end.pose.x_m - request.goal.x_m, end.pose.y_m - request.goal.y_m) <= 0.05 &&
              std::abs(common::normalize_angle_rad(end.pose.yaw_rad - request.goal.yaw_rad)) <= 0.02,
          "path must reach the requested goal pose");
  require(end.s_m > direct.length_m, "path must detour around the obstacle");
  require(checker.check_path(result.path).is_clear(),
          "continuous collision recheck must pass");
  std::cout << "PASS: obstacle detour, path length = " << end.s_m << " m\n";
}

}  // namespace

int main() {
  try {
    plans_collision_free_detour();
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
