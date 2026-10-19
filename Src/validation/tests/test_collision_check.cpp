// 静态碰撞检查：通过 planner::collision 的精确位姿检查与重建运动扫掠，
// 包括恰好位于存储采样点之间的薄障碍物、倒车挡和占据的栅格单元。

#include <gtest/gtest.h>

#include <string>

#include "planner/common/types.hpp"
#include "planner/validation/validation.hpp"
#include "validation_test_support.hpp"

namespace planner::validation {
namespace {

using test_support::base_request;
using test_support::free_grid_scene;
using test_support::make_corridor;
using test_support::point;
using test_support::rectangle_polygon;

TEST(ValidationCollision, rejects_start_pose_inside_obstacle) {
  ValidationRequest request = base_request();
  request.scene->obstacles.push_back(
      collision::Obstacle::rectangle("start_block", -1.0, -1.0, 1.0, 1.0));
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCollision);
  EXPECT_EQ(result.first_failure.point_index, 0);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 0.0);
  EXPECT_NE(result.first_failure.detail.find("point 0"), std::string::npos);
}

TEST(ValidationCollision, detects_thin_obstacle_between_forward_samples) {
  ValidationRequest request = base_request();
  request.trajectory = {
      point(0.0, 0.0, 0.0, 0.0, 2.0, 0.0),
      point(3.0, 6.0, 0.0, 0.0, 2.0, 0.0),
  };
  request.limits.max_speed_mps = 3.0;
  request.goal = common::Pose2d{6.0, 0.0, 0.0};
  request.corridor = make_corridor(
      {rectangle_polygon(0.0, 6.0, -2.0, -2.0, 10.0, 2.0)});
  // 两个存储位姿都不碰撞（位姿 x=0 覆盖 x ∈ [-1.1, 3.8]；位姿 x=6
  // 覆盖 x ∈ [4.9, 9.8]）；只有扫掠运动才能发现这面墙。
  request.scene->obstacles.push_back(
      collision::Obstacle::rectangle("thin_wall", 4.3, -2.0, 4.32, 2.0));
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCollision);
  EXPECT_EQ(result.first_failure.point_index, 0);
  EXPECT_GT(result.first_failure.time_s, 0.0);
  EXPECT_LT(result.first_failure.time_s, 1.0);  // 在 s ~= 0.5 m 处接触
  EXPECT_NE(result.first_failure.detail.find("reconstructed motion"),
            std::string::npos);
  EXPECT_GE(result.collision_motion_checks, 1u);
}

TEST(ValidationCollision, detects_thin_obstacle_during_reverse_motion) {
  ValidationRequest request = base_request();
  request.trajectory = {
      point(0.0, 6.0, 0.0, 0.0, -2.0, 0.0),
      point(3.0, 0.0, 0.0, 0.0, -2.0, 0.0),
  };
  request.limits.max_speed_mps = 3.0;
  request.goal = common::Pose2d{0.0, 0.0, 0.0};
  request.corridor = make_corridor(
      {rectangle_polygon(0.0, 6.0, -2.0, -2.0, 10.0, 2.0)});
  request.scene->obstacles.push_back(
      collision::Obstacle::rectangle("thin_wall", 4.3, -2.0, 4.32, 2.0));
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCollision);
  EXPECT_GT(result.first_failure.time_s, 0.0);
  EXPECT_LT(result.first_failure.time_s, 1.0);
}

TEST(ValidationCollision, rejects_occupied_grid_cell) {
  ValidationRequest request = base_request();
  request.trajectory = {
      point(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
      point(1.0, 1.0, 0.0, 0.0, 1.0, 0.0),
  };
  request.goal = common::Pose2d{1.0, 0.0, 0.0};
  request.corridor = make_corridor(
      {rectangle_polygon(0.0, 1.0, -2.0, -2.0, 6.0, 2.0)});
  // 栅格 x 范围 [-3, 5]、y 范围 [-2, 2]，分辨率 0.1 m；单元 (1.5, 0.5)
  // 被占据。
  request.scene = free_grid_scene(80, 40, 0.1, -3.0, -2.0);
  const int row = 25;   // (0.5 - (-2)) / 0.1
  const int col = 45;   // (1.5 - (-3)) / 0.1
  request.scene->grid.cells[static_cast<std::size_t>(row) * 80 + col] = 1;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCollision);
  EXPECT_EQ(result.first_failure.point_index, 0);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 0.0);
}

TEST(ValidationCollision, accepts_clear_path_past_obstacle) {
  ValidationRequest request = base_request();
  request.scene->obstacles.push_back(
      collision::Obstacle::rectangle("wall_beside_path", 0.5, 3.0, 2.0, 4.0));
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail;
}

TEST(ValidationCollision, counts_pose_and_motion_checks_on_failure) {
  ValidationRequest request = base_request();
  request.scene->obstacles.push_back(
      collision::Obstacle::rectangle("start_block", -1.0, -1.0, 1.0, 1.0));
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCollision);
  EXPECT_EQ(result.collision_pose_checks, 1u);   // 在首次命中处停止
  EXPECT_EQ(result.collision_motion_checks, 0u); // 命中后不再检查运动
}

}  // namespace
}  // namespace planner::validation
