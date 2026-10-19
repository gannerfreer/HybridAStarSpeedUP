// 位姿检测测试：接触、间隙、旋转、栅格状态、地图边界以及 v1 测试夹具（默认车辆 +
// 最小栅格）。

#include <gtest/gtest.h>

#include "planner/collision/collision_checker.hpp"
#include "test_helpers.hpp"

namespace planner::collision {
namespace {

using test::default_vehicle;
using test::free_grid;
using test::load_fixture_grid;
using test::load_fixture_vehicle;
using test::paint_cells;
using test::small_vehicle;

TEST(CheckPose, clear_in_empty_scene) {
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), Scene{});
  EXPECT_TRUE(result.is_clear()) << result.detail;
}

TEST(CheckPose, obstacle_ahead_is_clear) {
  Scene scene;
  scene.obstacles.push_back(
      Obstacle::rectangle("wall", 5.0, -1.0, 6.0, 1.0));
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  EXPECT_TRUE(result.is_clear()) << result.detail;
}

TEST(CheckPose, deep_overlap_is_collision) {
  Scene scene;
  scene.obstacles.push_back(
      Obstacle::rectangle("wall", 2.0, -1.0, 3.0, 1.0));
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_EQ(result.reason_code, CollisionReason::kObstacle);
}

TEST(CheckPose, tangent_edge_counts_as_collision) {
  Scene scene;
  // 足迹 x_max = 3.8 恰好触及障碍物边缘。
  scene.obstacles.push_back(
      Obstacle::rectangle("wall", 3.8, -1.0, 4.8, 1.0));
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_EQ(result.reason_code, CollisionReason::kObstacle);
}

TEST(CheckPose, tangent_corner_counts_as_collision) {
  Scene scene;
  // 足迹角点 (3.8, 1.0) 触及障碍物角点。
  scene.obstacles.push_back(
      Obstacle::rectangle("block", 3.8, 1.0, 4.8, 2.0));
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
}

TEST(CheckPose, micro_gap_is_clear) {
  Scene scene;
  scene.obstacles.push_back(
      Obstacle::rectangle("wall", 3.8 + 1e-6, -1.0, 4.8, 1.0));
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  EXPECT_TRUE(result.is_clear()) << result.detail;
}

TEST(CheckPose, safety_margin_is_checked) {
  Scene scene;
  // 位于无余量足迹（3.7）与含余量足迹（3.8）之间。
  scene.obstacles.push_back(
      Obstacle::rectangle("wall", 3.75, -1.0, 5.0, 1.0));
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  EXPECT_TRUE(result.collided()) << result.detail;
}

TEST(CheckPose, rotated_45_degrees_matches_hand_geometry) {
  const VehicleModel vehicle = default_vehicle();
  // 车体左前角点 (3.8, 1.0) 旋转 45 度。
  const Point2d corner_45{3.8 * std::cos(M_PI / 4) - 1.0 * std::sin(M_PI / 4),
                          3.8 * std::sin(M_PI / 4) + 1.0 * std::cos(M_PI / 4)};
  Scene scene;
  scene.obstacles.push_back(Obstacle::rectangle(
      "block", corner_45.x_m - 0.3, corner_45.y_m - 0.3, corner_45.x_m + 0.3,
      corner_45.y_m + 0.3));
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, M_PI / 4.0}, vehicle, scene);
  EXPECT_TRUE(result.collided()) << result.detail;

  Scene far_scene;
  far_scene.obstacles.push_back(Obstacle::rectangle(
      "block", corner_45.x_m + 1.0, corner_45.y_m + 1.0,
      corner_45.x_m + 2.0, corner_45.y_m + 2.0));
  EXPECT_TRUE(
      check_pose(Pose2d{0.0, 0.0, M_PI / 4.0}, vehicle, far_scene).is_clear());
}

TEST(CheckPose, occupied_cell_is_collision) {
  Scene scene = free_grid(20.0, 20.0);
  paint_cells(scene.grid, 1.4, 1.4, 1.6, 1.6, 1);
  const CheckResult result =
      check_pose(Pose2d{1.5, 1.5, 0.0}, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_EQ(result.reason_code, CollisionReason::kGridOccupied);
}

TEST(CheckPose, unknown_cell_is_collision) {
  Scene scene = free_grid(20.0, 20.0);
  paint_cells(scene.grid, 4.0, 3.5, 4.2, 3.7, 2);
  const CheckResult result =
      check_pose(Pose2d{3.5, 3.5, 0.0}, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_EQ(result.reason_code, CollisionReason::kGridUnknown);
}

TEST(CheckPose, outside_map_is_collision) {
  Scene scene = free_grid(20.0, 20.0);
  const CheckResult result =
      check_pose(Pose2d{100.0, 100.0, 0.0}, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_EQ(result.reason_code, CollisionReason::kOutsideMap);
}

TEST(CheckPose, footprint_on_map_boundary_is_clear) {
  Scene scene = free_grid(20.0, 20.0);
  // 足迹跨度为 [0, 4.9] x [0, 2.0]：闭合的地图边界视为内部。
  const CheckResult result =
      check_pose(Pose2d{1.1, 1.0, 0.0}, default_vehicle(), scene);
  EXPECT_TRUE(result.is_clear()) << result.detail;
}

TEST(CheckPose, footprint_crossing_boundary_is_collision) {
  Scene scene = free_grid(20.0, 20.0);
  const CheckResult result =
      check_pose(Pose2d{1.0, 1.0, 0.0}, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_EQ(result.reason_code, CollisionReason::kOutsideMap);
}

TEST(CheckPose, default_vehicle_fixture_matches_constants) {
  const VehicleModel fixture =
      load_fixture_vehicle("vehicles/default.json");
  const VehicleModel expected = default_vehicle();
  EXPECT_EQ(fixture.id, expected.id);
  EXPECT_DOUBLE_EQ(fixture.wheelbase_m, expected.wheelbase_m);
  EXPECT_DOUBLE_EQ(fixture.width_m, expected.width_m);
  EXPECT_DOUBLE_EQ(fixture.front_overhang_m, expected.front_overhang_m);
  EXPECT_DOUBLE_EQ(fixture.rear_overhang_m, expected.rear_overhang_m);
  EXPECT_DOUBLE_EQ(fixture.max_steer_rad, expected.max_steer_rad);
  EXPECT_DOUBLE_EQ(fixture.safety_margin_m, expected.safety_margin_m);
}

TEST(CheckPose, minimal_fixture_grid_reasons) {
  const VehicleModel vehicle = small_vehicle();
  Scene scene;
  scene.grid = load_fixture_grid("grids/minimal.json");
  // 空旷区域：第 1..6 行、第 10..19 列，无障碍。
  EXPECT_TRUE(
      check_pose(Pose2d{1.2, 0.4, 0.0}, vehicle, scene).is_clear());
  // 短墙位于第 5 行、第 4..9 列 -> [0.4,1.0] x [0.5,0.6]。
  const CheckResult wall =
      check_pose(Pose2d{0.3, 0.55, 0.0}, vehicle, scene);
  ASSERT_TRUE(wall.collided());
  EXPECT_EQ(wall.reason_code, CollisionReason::kGridOccupied);
  // 孤立的未知栅格位于第 18 行、第 10 列 -> [1.0,1.1] x [1.8,1.9]。
  const CheckResult unknown =
      check_pose(Pose2d{0.55, 1.75, 0.0}, vehicle, scene);
  ASSERT_TRUE(unknown.collided());
  EXPECT_EQ(unknown.reason_code, CollisionReason::kGridUnknown);
  // 无法放入 2 m 地图的足迹报告 outside_map。
  const CheckResult big =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  ASSERT_TRUE(big.collided());
  EXPECT_EQ(big.reason_code, CollisionReason::kOutsideMap);
}

TEST(CheckPose, first_pose_reports_the_checked_pose) {
  Scene scene;
  scene.obstacles.push_back(Obstacle::rectangle("wall", 0.0, -1.0, 1.0, 1.0));
  const Pose2d pose{0.0, 0.0, 0.25};
  const CheckResult result = check_pose(pose, default_vehicle(), scene);
  ASSERT_TRUE(result.collided());
  EXPECT_DOUBLE_EQ(result.first_pose.x_m, pose.x_m);
  EXPECT_DOUBLE_EQ(result.first_pose.y_m, pose.y_m);
  EXPECT_DOUBLE_EQ(result.first_pose.yaw_rad, pose.yaw_rad);
  EXPECT_DOUBLE_EQ(result.first_s_m, 0.0);
}

}  // namespace
}  // namespace planner::collision
