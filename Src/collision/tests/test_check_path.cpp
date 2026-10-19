// 路径检测测试：采样点之间的扫掠、方向分段与首次失败上报。

#include <gtest/gtest.h>

#include "planner/collision/collision_checker.hpp"
#include "test_helpers.hpp"

namespace planner::collision {
namespace {

using test::default_vehicle;
using test::free_grid;
using planner::common::PathPoint;

PathPoint point(double x, double y, double yaw, double s, Direction direction,
                double curvature = 0.0) {
  PathPoint p;
  p.pose = Pose2d{x, y, yaw};
  p.s_m = s;
  p.curvature_inv_m = curvature;
  p.direction = direction;
  return p;
}

TEST(CheckPath, straight_segment_clear) {
  PathSegment segment;
  segment.points = {point(2.0, 2.0, 0.0, 0.0, Direction::kForward),
                    point(4.0, 2.0, 0.0, 2.0, Direction::kForward)};
  const CheckResult result =
      check_path(segment, default_vehicle(), free_grid(30.0, 10.0));
  EXPECT_TRUE(result.is_clear()) << result.detail;
  EXPECT_EQ(result.path_segment_index, -1);
}

TEST(CheckPath, rejects_colliding_start) {
  Scene scene;
  scene.obstacles.push_back(Obstacle::rectangle("wall", 0.0, -1.0, 1.0, 1.0));
  PathSegment segment;
  segment.points = {point(0.0, 0.0, 0.0, 0.0, Direction::kForward),
                    point(2.0, 0.0, 0.0, 2.0, Direction::kForward)};
  const CheckResult result = check_path(segment, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_DOUBLE_EQ(result.first_s_m, 0.0);
  EXPECT_EQ(result.path_segment_index, 0);
}

TEST(CheckPath, thin_wall_between_sample_points_is_detected) {
  // 只有两个采样点；两位姿均无障碍，但薄墙严格位于二者之间：仅检查离散点会
  // 通过。
  Scene scene = free_grid(30.0, 10.0);
  scene.obstacles.push_back(
      Obstacle::rectangle("thin_wall", 5.85, 1.0, 5.87, 4.0));
  PathSegment segment;
  segment.points = {point(2.0, 2.0, 0.0, 0.0, Direction::kForward),
                    point(7.0, 2.0, 0.0, 5.0, Direction::kForward)};
  const CheckResult result = check_path(segment, default_vehicle(), scene);
  ASSERT_TRUE(result.collided()) << result.detail;
  EXPECT_NEAR(result.first_s_m, 0.05, 1.2e-3);
}

TEST(CheckPath, direction_change_second_segment_reports_index) {
  Scene scene = free_grid(30.0, 10.0);
  scene.obstacles.push_back(Obstacle::rectangle("wall", 1.5, 1.0, 2.0, 4.0));
  std::vector<PathSegment> path(2);
  // 分段 0：前进，远离障碍物。
  path[0].points = {point(12.0, 2.0, 0.0, 0.0, Direction::kForward),
                    point(14.0, 2.0, 0.0, 2.0, Direction::kForward)};
  // 分段 1：倒车；后保险杠在 s = 1.9 处到达 x = 2.0。
  path[1].points = {point(5.0, 2.0, 0.0, 0.0, Direction::kReverse),
                    point(3.0, 2.0, 0.0, 2.0, Direction::kReverse)};
  const CheckResult result = check_path(path, default_vehicle(), scene);
  ASSERT_TRUE(result.collided()) << result.detail;
  EXPECT_EQ(result.path_segment_index, 1);
  EXPECT_NEAR(result.first_s_m, 1.9, 1.2e-3);
}

TEST(CheckPath, single_point_segment_checks_the_pose) {
  Scene scene;
  scene.obstacles.push_back(Obstacle::rectangle("wall", 2.0, -1.0, 3.0, 1.0));
  PathSegment segment;
  segment.points = {point(0.0, 0.0, 0.0, 0.0, Direction::kForward)};
  const CheckResult result = check_path(segment, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_DOUBLE_EQ(result.first_s_m, 0.0);
}

TEST(CheckPath, curved_segment_uses_stored_curvature) {
  // 从 (10, 10, 0) 左转 90 度、半径 5：圆弧贴近以 (10, 15) 为圆心的圆。
  // 圆上的障碍块必须被击中。
  Scene scene = free_grid(40.0, 40.0);
  scene.obstacles.push_back(
      Obstacle::rectangle("block", 14.0, 14.0, 15.0, 15.0));
  PathSegment segment;
  const double length = 5.0 * (M_PI / 2.0);  // 四分之一圆
  segment.points = {
      point(10.0, 10.0, 0.0, 0.0, Direction::kForward, 0.2),
      point(15.0, 15.0, M_PI / 2.0, length, Direction::kForward, 0.2)};
  const CheckResult result = check_path(segment, default_vehicle(), scene);
  ASSERT_TRUE(result.collided()) << result.detail;
  EXPECT_LT(result.first_s_m, length);
}

TEST(CheckPath, cumulative_first_s_is_segment_local) {
  Scene scene = free_grid(30.0, 10.0);
  scene.obstacles.push_back(
      Obstacle::rectangle("wall", 12.0, 1.0, 12.5, 4.0));
  std::vector<PathSegment> path(2);
  path[0].points = {point(2.0, 2.0, 0.0, 0.0, Direction::kForward),
                    point(4.0, 2.0, 0.0, 2.0, Direction::kForward)};
  path[1].points = {point(4.0, 2.0, 0.0, 0.0, Direction::kForward),
                    point(9.0, 2.0, 0.0, 5.0, Direction::kForward)};
  const CheckResult result = check_path(path, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_EQ(result.path_segment_index, 1);
  // 分段 1 从 x = 4 开始，此时前保险杠位于 7.8；前进 4.2 m 后在
  // x = 12.0 处接触。
  EXPECT_NEAR(result.first_s_m, 4.2, 1.2e-3);
}

}  // namespace
}  // namespace planner::collision
