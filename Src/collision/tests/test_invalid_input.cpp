// 无效输入测试：每种检查都以 kInvalidInput 拒绝损坏的几何，而不是返回判定。

#include <gtest/gtest.h>

#include <limits>

#include "planner/collision/collision_checker.hpp"
#include "test_helpers.hpp"

namespace planner::collision {
namespace {

using test::default_vehicle;
using test::free_grid;
using planner::common::PathPoint;

TEST(CheckInvalidInput, rejects_negative_motion_length) {
  MotionSegment motion;
  motion.start = Pose2d{0.0, 0.0, 0.0};
  motion.length_m = -1.0;
  const CheckResult result =
      check_motion(motion, default_vehicle(), Scene{});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
  EXPECT_FALSE(result.collided());
}

TEST(CheckInvalidInput, rejects_nonfinite_pose) {
  const Pose2d pose{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
  const CheckResult result = check_pose(pose, default_vehicle(), Scene{});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_nonfinite_curvature) {
  MotionSegment motion;
  motion.start = Pose2d{0.0, 0.0, 0.0};
  motion.length_m = 1.0;
  motion.curvature_inv_m = std::numeric_limits<double>::infinity();
  const CheckResult result =
      check_motion(motion, default_vehicle(), Scene{});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_absurd_curvature) {
  MotionSegment motion;
  motion.start = Pose2d{0.0, 0.0, 0.0};
  motion.length_m = 1.0;
  motion.curvature_inv_m = 1e9;
  const CheckResult result =
      check_motion(motion, default_vehicle(), Scene{});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_negative_vehicle_width) {
  VehicleModel vehicle = default_vehicle();
  vehicle.width_m = -1.0;
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, vehicle, Scene{});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_zero_wheelbase) {
  VehicleModel vehicle = default_vehicle();
  vehicle.wheelbase_m = 0.0;
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, vehicle, Scene{});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_obstacle_with_too_few_vertices) {
  Scene scene;
  Obstacle obstacle;
  obstacle.id = "broken";
  obstacle.vertices = {Point2d{0.0, 0.0}, Point2d{1.0, 1.0}};
  scene.obstacles.push_back(obstacle);
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_non_convex_obstacle) {
  Scene scene;
  Obstacle obstacle;
  obstacle.id = "dart";
  obstacle.vertices = {Point2d{0.0, 0.0}, Point2d{2.0, 0.0},
                       Point2d{1.0, 1.0}, Point2d{2.0, 2.0}};
  scene.obstacles.push_back(obstacle);
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_grid_cell_count_mismatch) {
  Scene scene = free_grid(10.0, 10.0);
  scene.grid.cells.pop_back();
  const CheckResult result =
      check_pose(Pose2d{2.0, 2.0, 0.0}, default_vehicle(), scene);
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_grid_cell_value_out_of_range) {
  Scene scene = free_grid(10.0, 10.0);
  scene.grid.cells[0] = 7;
  const CheckResult result =
      check_pose(Pose2d{2.0, 2.0, 0.0}, default_vehicle(), scene);
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_nonpositive_resolution) {
  Scene scene = free_grid(10.0, 10.0);
  scene.grid.resolution_m = 0.0;
  const CheckResult result =
      check_pose(Pose2d{2.0, 2.0, 0.0}, default_vehicle(), scene);
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_partial_grid_dimensions) {
  Scene scene = free_grid(10.0, 10.0);
  scene.grid.width = 0;
  scene.grid.cells.clear();
  const CheckResult result =
      check_pose(Pose2d{2.0, 2.0, 0.0}, default_vehicle(), scene);
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_nonpositive_sweep_precision) {
  CheckOptions options;
  options.sweep_precision_m = 0.0;
  const CheckResult result =
      check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), Scene{}, options);
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_empty_path) {
  const CheckResult result =
      check_path(std::vector<PathSegment>{}, default_vehicle(), Scene{});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_path_segment_without_points) {
  PathSegment segment;
  const CheckResult result = check_path(segment, default_vehicle(), Scene{});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_non_increasing_path_arc_length) {
  PathSegment segment;
  PathPoint first;
  first.pose = Pose2d{0.0, 0.0, 0.0};
  first.s_m = 1.0;
  PathPoint second = first;
  second.pose = Pose2d{1.0, 0.0, 0.0};
  second.s_m = 1.0;
  segment.points = {first, second};
  const CheckResult result =
      check_path(segment, default_vehicle(), Scene{});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, rejects_mixed_direction_path_segment) {
  PathSegment segment;
  PathPoint first;
  first.pose = Pose2d{0.0, 0.0, 0.0};
  first.s_m = 0.0;
  first.direction = Direction::kForward;
  PathPoint second = first;
  second.pose = Pose2d{1.0, 0.0, 0.0};
  second.s_m = 1.0;
  second.direction = Direction::kReverse;
  segment.points = {first, second};
  const CheckResult result =
      check_path(segment, default_vehicle(), Scene{});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

TEST(CheckInvalidInput, checker_reports_the_same_rejection) {
  VehicleModel vehicle = default_vehicle();
  vehicle.safety_margin_m = -0.1;
  CollisionChecker checker(vehicle, Scene{});
  EXPECT_FALSE(checker.valid());
  EXPECT_FALSE(checker.invalid_reason().empty());
  const CheckResult result = checker.check_pose(Pose2d{0.0, 0.0, 0.0});
  EXPECT_EQ(result.status, CheckStatus::kInvalidInput);
}

}  // namespace
}  // namespace planner::collision
