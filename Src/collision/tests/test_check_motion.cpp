// 运动检测测试：首次接触、保守扫掠、薄障碍物、倒挡与精度下限契约。

#include <gtest/gtest.h>

#include "planner/collision/collision_checker.hpp"
#include "test_helpers.hpp"

namespace planner::collision {
namespace {

using test::default_vehicle;
using test::free_grid;
using test::make_sub_precision_tangent_case;
using test::paint_cells;

MotionSegment straight(Pose2d start, double length, Direction direction,
                       double curvature = 0.0) {
  MotionSegment motion;
  motion.start = start;
  motion.length_m = length;
  motion.direction = direction;
  motion.curvature_inv_m = curvature;
  return motion;
}

TEST(CheckMotion, straight_clear_motion) {
  const CheckResult result =
      check_motion(straight(Pose2d{2.0, 2.0, 0.0}, 3.0, Direction::kForward),
                   default_vehicle(), free_grid(20.0, 20.0));
  EXPECT_TRUE(result.is_clear()) << result.detail;
}

TEST(CheckMotion, colliding_start_reports_contact_at_zero) {
  Scene scene;
  scene.obstacles.push_back(Obstacle::rectangle("wall", 0.0, -1.0, 1.0, 1.0));
  const CheckResult result =
      check_motion(straight(Pose2d{0.0, 0.0, 0.0}, 3.0, Direction::kForward),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_DOUBLE_EQ(result.first_s_m, 0.0);
  EXPECT_NE(result.detail.find("colliding start"), std::string::npos);
}

TEST(CheckMotion, straight_obstacle_ahead_reports_first_contact) {
  Scene scene = free_grid(30.0, 10.0);
  // 前保险杠起始于 x = 5.8；到达 8.0 时接触。
  scene.obstacles.push_back(
      Obstacle::rectangle("wall", 8.0, 1.0, 9.0, 4.0));
  const CheckResult result =
      check_motion(straight(Pose2d{2.0, 2.0, 0.0}, 5.0, Direction::kForward),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_NEAR(result.first_s_m, 2.2, 1.2e-3);
  EXPECT_NEAR(result.first_pose.x_m, 4.2, 1.2e-3);
}

TEST(CheckMotion, thin_wall_between_endpoints_is_detected) {
  // 两个端点位姿均无障碍；薄墙严格位于二者之间。
  Scene scene = free_grid(30.0, 10.0);
  scene.obstacles.push_back(
      Obstacle::rectangle("thin_wall", 5.85, 1.0, 5.87, 4.0));
  const CheckResult result =
      check_motion(straight(Pose2d{2.0, 2.0, 0.0}, 5.0, Direction::kForward),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided()) << result.detail;
  EXPECT_NEAR(result.first_s_m, 0.05, 1.2e-3);
}

TEST(CheckMotion, reverse_into_obstacle_is_detected) {
  Scene scene = free_grid(30.0, 10.0);
  // 后保险杠起始于 x = 3.9；倒车到达 2.0 时接触。
  scene.obstacles.push_back(Obstacle::rectangle("wall", 1.5, 1.0, 2.0, 4.0));
  const CheckResult result =
      check_motion(straight(Pose2d{5.0, 2.0, 0.0}, 2.0, Direction::kReverse),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_NEAR(result.first_s_m, 1.9, 1.2e-3);
}

TEST(CheckMotion, reverse_clear_motion) {
  Scene scene = free_grid(30.0, 10.0);
  scene.obstacles.push_back(Obstacle::rectangle("wall", 0.2, 1.0, 0.7, 4.0));
  const CheckResult result =
      check_motion(straight(Pose2d{5.0, 2.0, 0.0}, 2.0, Direction::kReverse),
                   default_vehicle(), scene);
  EXPECT_TRUE(result.is_clear()) << result.detail;
}

TEST(CheckMotion, curved_arc_collides_with_block_on_the_path) {
  Scene scene = free_grid(40.0, 40.0);
  // 从 (10, 10, 0) 左转（kappa = 0.2，半径 5）：圆弧约在 s = 3.5 处经过
  // (13.2, 11.2)。
  scene.obstacles.push_back(
      Obstacle::rectangle("block", 12.0, 10.5, 14.0, 12.5));
  const CheckResult result =
      check_motion(straight(Pose2d{10.0, 10.0, 0.0}, 5.0, Direction::kForward,
                            0.2),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided()) << result.detail;
  EXPECT_LT(result.first_s_m, 5.0);
}

TEST(CheckMotion, curved_arc_clear_far_from_the_turn) {
  Scene scene = free_grid(40.0, 40.0);
  scene.obstacles.push_back(
      Obstacle::rectangle("block", 30.0, 30.0, 33.0, 33.0));
  const CheckResult result =
      check_motion(straight(Pose2d{10.0, 10.0, 0.0}, 5.0, Direction::kForward,
                            0.2),
                   default_vehicle(), scene);
  EXPECT_TRUE(result.is_clear()) << result.detail;
}

TEST(CheckMotion, contact_at_final_pose_is_collision) {
  Scene scene = free_grid(30.0, 10.0);
  // 在 s = 2 处前保险杠恰好位于 x = 7.8。
  scene.obstacles.push_back(Obstacle::rectangle("wall", 7.8, 1.0, 8.8, 4.0));
  const CheckResult result =
      check_motion(straight(Pose2d{2.0, 2.0, 0.0}, 2.0, Direction::kForward),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_NEAR(result.first_s_m, 2.0, 1.2e-3);
}

TEST(CheckMotion, zero_length_checks_the_pose) {
  Scene scene;
  scene.obstacles.push_back(Obstacle::rectangle("wall", 3.0, -1.0, 4.0, 1.0));
  const CheckResult result =
      check_motion(straight(Pose2d{0.0, 0.0, 0.0}, 0.0, Direction::kForward),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_DOUBLE_EQ(result.first_s_m, 0.0);
}

TEST(CheckMotion, reports_the_earliest_of_two_obstacles) {
  Scene scene = free_grid(40.0, 10.0);
  scene.obstacles.push_back(Obstacle::rectangle("near", 8.0, 1.0, 8.5, 4.0));
  scene.obstacles.push_back(Obstacle::rectangle("far", 16.0, 1.0, 17.0, 4.0));
  const CheckResult result =
      check_motion(straight(Pose2d{2.0, 2.0, 0.0}, 10.0, Direction::kForward),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_NEAR(result.first_s_m, 2.2, 1.2e-3);
}

TEST(CheckMotion, unknown_cell_ahead_is_collision) {
  Scene scene = free_grid(30.0, 10.0);
  paint_cells(scene.grid, 8.0, 1.0, 8.2, 4.0, 2);
  const CheckResult result =
      check_motion(straight(Pose2d{2.0, 2.0, 0.0}, 5.0, Direction::kForward),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_EQ(result.reason_code, CollisionReason::kGridUnknown);
}

TEST(CheckMotion, crossing_map_boundary_is_collision) {
  Scene scene = free_grid(20.0, 20.0);
  const CheckResult result =
      check_motion(straight(Pose2d{15.0, 10.0, 0.0}, 5.0,
                            Direction::kForward),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_EQ(result.reason_code, CollisionReason::kOutsideMap);
}

TEST(CheckMotion, sub_precision_tangent_is_conservative_collision) {
  const auto test_case = make_sub_precision_tangent_case();
  const CheckResult result =
      check_motion(test_case.motion, test_case.vehicle, test_case.scene);
  ASSERT_TRUE(result.collided()) << result.detail;
  EXPECT_EQ(result.reason_code, CollisionReason::kPrecisionFloor);
}

TEST(CheckMotion, precise_tangent_is_detected_as_contact) {
  Scene scene = free_grid(30.0, 10.0);
  // 障碍物边缘恰好位于扫掠平面 x = 7.8 上。
  scene.obstacles.push_back(Obstacle::rectangle("wall", 7.8, 1.0, 8.8, 4.0));
  const CheckResult result =
      check_motion(straight(Pose2d{2.0, 2.0, 0.0}, 2.0, Direction::kForward),
                   default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_EQ(result.reason_code, CollisionReason::kObstacle);
}

}  // namespace
}  // namespace planner::collision
