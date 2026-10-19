// 独立参考测试：语义必须与生产契约一致（接触计为碰撞、低于精度下限时为
// inconclusive）。

#include <gtest/gtest.h>

#include "planner/collision/reference.hpp"
#include "test_helpers.hpp"

namespace planner::collision {
namespace {

using test::default_vehicle;
using test::free_grid;
using test::make_sub_precision_tangent_case;
using test::paint_cells;

TEST(Reference, pose_tangent_is_collision) {
  Scene scene;
  scene.obstacles.push_back(Obstacle::rectangle("wall", 3.8, -1.0, 4.8, 1.0));
  const ReferenceResult result =
      reference_check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  EXPECT_EQ(result.verdict, ReferenceVerdict::kCollision);
}

TEST(Reference, pose_micro_gap_is_clear) {
  Scene scene;
  scene.obstacles.push_back(
      Obstacle::rectangle("wall", 3.8 + 1e-6, -1.0, 4.8, 1.0));
  const ReferenceResult result =
      reference_check_pose(Pose2d{0.0, 0.0, 0.0}, default_vehicle(), scene);
  EXPECT_EQ(result.verdict, ReferenceVerdict::kClear);
}

TEST(Reference, straight_motion_first_contact_matches_hand_value) {
  Scene scene = free_grid(30.0, 10.0);
  scene.obstacles.push_back(Obstacle::rectangle("wall", 8.0, 1.0, 9.0, 4.0));
  MotionSegment motion;
  motion.start = Pose2d{2.0, 2.0, 0.0};
  motion.length_m = 5.0;
  motion.direction = Direction::kForward;
  const ReferenceResult result =
      reference_check_motion(motion, default_vehicle(), scene);
  ASSERT_EQ(result.verdict, ReferenceVerdict::kCollision);
  EXPECT_NEAR(result.first_s_m, 2.2, 1.2e-3);
}

TEST(Reference, straight_motion_clear) {
  Scene scene = free_grid(30.0, 10.0);
  MotionSegment motion;
  motion.start = Pose2d{2.0, 2.0, 0.0};
  motion.length_m = 3.0;
  motion.direction = Direction::kForward;
  const ReferenceResult result =
      reference_check_motion(motion, default_vehicle(), scene);
  EXPECT_EQ(result.verdict, ReferenceVerdict::kClear);
}

TEST(Reference, reverse_contact_at_endpoint_is_collision) {
  Scene scene = free_grid(30.0, 10.0);
  scene.obstacles.push_back(Obstacle::rectangle("wall", 1.5, 1.0, 2.0, 4.0));
  MotionSegment motion;
  motion.start = Pose2d{5.0, 2.0, 0.0};
  motion.length_m = 1.9;
  motion.direction = Direction::kReverse;
  const ReferenceResult result =
      reference_check_motion(motion, default_vehicle(), scene);
  EXPECT_EQ(result.verdict, ReferenceVerdict::kCollision);
}

TEST(Reference, thin_wall_crossing_is_collision) {
  Scene scene = free_grid(30.0, 10.0);
  scene.obstacles.push_back(
      Obstacle::rectangle("thin_wall", 5.85, 1.0, 5.87, 4.0));
  MotionSegment motion;
  motion.start = Pose2d{2.0, 2.0, 0.0};
  motion.length_m = 5.0;
  motion.direction = Direction::kForward;
  const ReferenceResult result =
      reference_check_motion(motion, default_vehicle(), scene);
  EXPECT_EQ(result.verdict, ReferenceVerdict::kCollision);
}

TEST(Reference, sub_precision_tangent_is_inconclusive) {
  const auto test_case = make_sub_precision_tangent_case();
  const ReferenceResult result = reference_check_motion(
      test_case.motion, test_case.vehicle, test_case.scene);
  EXPECT_EQ(result.verdict, ReferenceVerdict::kInconclusive) << result.detail;
}

TEST(Reference, unknown_cell_is_collision) {
  Scene scene = free_grid(30.0, 10.0);
  paint_cells(scene.grid, 8.0, 1.0, 8.2, 4.0, 2);
  MotionSegment motion;
  motion.start = Pose2d{2.0, 2.0, 0.0};
  motion.length_m = 5.0;
  motion.direction = Direction::kForward;
  const ReferenceResult result =
      reference_check_motion(motion, default_vehicle(), scene);
  EXPECT_EQ(result.verdict, ReferenceVerdict::kCollision);
}

TEST(Reference, invalid_input_is_rejected) {
  MotionSegment motion;
  motion.start = Pose2d{0.0, 0.0, 0.0};
  motion.length_m = -2.0;
  const ReferenceResult result =
      reference_check_motion(motion, default_vehicle(), Scene{});
  EXPECT_EQ(result.verdict, ReferenceVerdict::kInvalidInput);
}

}  // namespace
}  // namespace planner::collision
