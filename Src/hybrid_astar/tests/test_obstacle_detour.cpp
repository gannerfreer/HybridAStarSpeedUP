#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "planner/common/grid_view.hpp"
#include "planner/common/plan.hpp"
#include "planner/common/status.hpp"
#include "planner/common/vehicle.hpp"
#include "planner/math/kinematics/arc.hpp"
#include "user_astar.hpp"

namespace {
using namespace planner;

// GoogleTest 断言在 Release 构建中同样启用。
TEST(ObstacleDetour, PlansCollisionFreeDetour) {
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
  ASSERT_TRUE(checker.valid()) << "invalid test scene: " << checker.invalid_reason();

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
  EXPECT_TRUE(checker.check_motion(direct).collided()) << "test setup must block the direct route";

  hybrid_astar::UserAstar planner;
  const auto result = planner.plan(request, grid, checker);
  ASSERT_EQ(result.status, common::Status::kSuccess)
      << "planning failed: " << common::to_string(result.status) << " " << result.reason;
  ASSERT_FALSE(result.path.empty()) << "successful path must not be empty";
  ASSERT_FALSE(result.path.front().points.empty()) << "path segment must not be empty";
  ASSERT_FALSE(result.path.back().points.empty()) << "path segment must not be empty";
  for (std::size_t segment_index = 0; segment_index < result.path.size(); ++segment_index) {
    SCOPED_TRACE(segment_index);
    const auto& segment = result.path[segment_index];
    ASSERT_FALSE(segment.points.empty()) << "path segment must not be empty";
    for (std::size_t point_index = 0; point_index < segment.points.size(); ++point_index) {
      SCOPED_TRACE(point_index);
      const auto& point = segment.points[point_index];
      EXPECT_TRUE(std::isfinite(point.s_m) && std::isfinite(point.pose.x_m) &&
                  std::isfinite(point.pose.y_m) && std::isfinite(point.pose.yaw_rad) &&
                  std::isfinite(point.curvature_per_m))
          << "path contains nonfinite values";
      EXPECT_LE(std::abs(point.curvature_per_m), 1.0 / vehicle.min_turn_radius_m() + 1e-9)
          << "path exceeds the vehicle curvature limit";
    }
  }
  const auto& start = result.path.front().points.front().pose;
  const auto& end = result.path.back().points.back();
  EXPECT_TRUE(std::hypot(start.x_m - request.start.x_m, start.y_m - request.start.y_m) < 1e-6 &&
              std::abs(math::normalize_angle_rad(start.yaw_rad - request.start.yaw_rad)) < 1e-6)
      << "path must begin at the requested start";
  EXPECT_TRUE(std::hypot(end.pose.x_m - request.goal.x_m, end.pose.y_m - request.goal.y_m) <=
                  0.05 &&
              std::abs(math::normalize_angle_rad(end.pose.yaw_rad - request.goal.yaw_rad)) <= 0.02)
      << "path must reach the requested goal pose";
  EXPECT_GT(end.s_m, direct.length_m) << "path must detour around the obstacle";
  EXPECT_TRUE(checker.check_path(result.path).is_clear())
      << "continuous collision recheck must pass";
}

}  // namespace
