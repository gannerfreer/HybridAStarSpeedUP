// 正向验证用例：必须被接受的轨迹，以及完整通过的工作量统计。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "planner/common/types.hpp"
#include "planner/validation/validation.hpp"
#include "validation_test_support.hpp"

namespace planner::validation {
namespace {

using common::kPi;
using test_support::base_request;
using test_support::empty_scene;
using test_support::make_corridor;
using test_support::point;
using test_support::rectangle_polygon;

TEST(ValidationValid, accepts_straight_forward_trajectory) {
  const ValidationResult result = validate_trajectory(base_request());
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.status, common::Status::kSuccess);
  EXPECT_EQ(result.reason, "valid");
  EXPECT_EQ(result.first_failure.constraint, Constraint::kNone);
  EXPECT_EQ(result.trajectory_points, 5u);
  EXPECT_EQ(result.collision_pose_checks, 5u);
  EXPECT_EQ(result.collision_motion_checks, 4u);
  EXPECT_GT(result.corridor_samples, 0u);
}

TEST(ValidationValid, accepts_reverse_trajectory_with_turnaround_stop) {
  ValidationRequest request = base_request();
  request.trajectory = {
      point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      point(1.0, 0.5, 0.0, 0.0, 0.5, 0.0),
      point(2.0, 1.0, 0.0, 0.0, 0.0, 0.0),   // 换挡前停车
      point(3.0, 0.5, 0.0, 0.0, -0.5, 0.0),  // 倒车
      point(4.0, 0.0, 0.0, 0.0, 0.0, 0.0),
  };
  request.goal = common::Pose2d{0.0, 0.0, 0.0};
  request.corridor = make_corridor(
      {rectangle_polygon(0.0, 2.0, -2.0, -2.0, 6.0, 2.0)});
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail;
  EXPECT_EQ(result.status, common::Status::kSuccess);
}

TEST(ValidationValid, accepts_curved_trajectory_within_curvature_limit) {
  // 以 (0, 5) 为圆心、半径 5 m 的四分之一圆，按声明的曲率限制
  // 0.2 1/m 左转，每 15 度采样一次。
  ValidationRequest request = base_request();
  request.trajectory.clear();
  const double radius = 5.0;
  const double steps = 6.0;
  for (int k = 0; k <= 6; ++k) {
    const double theta = 0.5 * kPi * static_cast<double>(k) / steps;
    request.trajectory.push_back(
        point(5.0 * theta, radius * std::sin(theta),
              radius * (1.0 - std::cos(theta)), theta, 1.0, 0.0, 0.2));
  }
  request.goal = request.trajectory.back().pose;
  request.corridor = make_corridor({rectangle_polygon(
      0.0, 5.0 * 0.5 * kPi, -4.0, -4.0, 9.0, 9.0)});
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail << " at t="
                            << result.first_failure.time_s;
}

TEST(ValidationValid, accepts_single_point_trajectory_at_goal) {
  ValidationRequest request = base_request();
  request.trajectory = {point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)};
  request.goal = common::Pose2d{0.0, 0.0, 0.0};
  request.corridor = make_corridor(
      {rectangle_polygon(0.0, 1.0, -2.0, -2.0, 6.0, 2.0)});
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail;
  EXPECT_EQ(result.collision_motion_checks, 0u);
}

TEST(ValidationValid, accepts_obstacle_beside_the_swept_path) {
  ValidationRequest request = base_request();
  request.scene = empty_scene();
  request.scene->obstacles.push_back(
      collision::Obstacle::rectangle("wall_beside_path", 0.5, 3.0, 2.0, 4.0));
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail;
}

TEST(ValidationValid, accepts_collision_checker_options_passthrough) {
  ValidationRequest request = base_request();
  ValidationOptions options;
  options.corridor_sample_step_m = 0.05;
  options.containment_tolerance_m = 1e-6;
  options.collision.sweep_precision_m = 1e-3;
  const ValidationResult result = validate_trajectory(request, options);
  EXPECT_TRUE(result.valid) << result.detail;
}

}  // namespace
}  // namespace planner::validation
