#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include "collision_checker.hpp"
#include "planner/common/grid_view.hpp"
#include "planner/common/path.hpp"
#include "planner/common/vehicle.hpp"
#include "planner/math/angle.hpp"
#include "scene.hpp"

namespace {
using namespace planner;

common::VehicleModel test_vehicle() {
  common::VehicleModel vehicle;
  vehicle.wheelbase_m = 2.5;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.8;
  vehicle.rear_overhang_m = 0.8;
  vehicle.max_steer_rad = 0.6;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

collision::Scene make_scene(int width = 40, int height = 40, double resolution_m = 0.5) {
  collision::Scene scene;
  auto& grid = scene.grid;
  grid.map_id = "collision_test";
  grid.map_version = "1";
  grid.width = width;
  grid.height = height;
  grid.resolution_m = resolution_m;
  grid.cells.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
                    common::kCellFree);
  return scene;
}

common::PathPoint path_point(double x_m, double y_m, double s_m,
                             common::Direction direction = common::Direction::kForward,
                             double curvature_per_m = 0.0) {
  common::PathPoint point;
  point.pose = {x_m, y_m, 0.0};
  point.s_m = s_m;
  point.direction = direction;
  point.curvature_per_m = curvature_per_m;
  return point;
}

TEST(CollisionChecker, PoseClearGridOccupiedAndUnknown) {
  const auto vehicle = test_vehicle();
  auto scene = make_scene(20, 20);
  collision::CollisionChecker clear_checker(vehicle, scene);
  ASSERT_TRUE(clear_checker.valid()) << clear_checker.invalid_reason();
  EXPECT_TRUE(clear_checker.check_pose({5.0, 5.0, 0.0}).is_clear());

  scene.grid.cells[10 * scene.grid.width + 10] = common::kCellOccupied;
  collision::CollisionChecker occupied_checker(vehicle, scene);
  ASSERT_TRUE(occupied_checker.valid());
  const auto occupied = occupied_checker.check_pose({5.25, 5.25, 0.0});
  EXPECT_TRUE(occupied.collided());
  EXPECT_EQ(occupied.reason_code, collision::CollisionReason::kGridOccupied);

  scene.grid.cells[10 * scene.grid.width + 10] = common::kCellUnknown;
  collision::CollisionChecker unknown_checker(vehicle, scene);
  ASSERT_TRUE(unknown_checker.valid());
  const auto unknown = unknown_checker.check_pose({5.25, 5.25, 0.0});
  EXPECT_TRUE(unknown.collided());
  EXPECT_EQ(unknown.reason_code, collision::CollisionReason::kGridUnknown);
}

TEST(CollisionChecker, PoseObstacleOutsideMapAndEmptyGrid) {
  const auto vehicle = test_vehicle();
  auto obstacle_scene = make_scene(40, 40);
  obstacle_scene.obstacles.push_back(collision::Obstacle::rectangle("box", 5.0, 5.0, 7.0, 7.0));
  collision::CollisionChecker obstacle_checker(vehicle, obstacle_scene);
  ASSERT_TRUE(obstacle_checker.valid()) << obstacle_checker.invalid_reason();
  const auto obstacle = obstacle_checker.check_pose({5.5, 5.5, 0.0});
  EXPECT_TRUE(obstacle.collided());
  EXPECT_EQ(obstacle.reason_code, collision::CollisionReason::kObstacle);

  auto map_scene = make_scene(20, 20);
  collision::CollisionChecker map_checker(vehicle, map_scene);
  ASSERT_TRUE(map_checker.valid());
  const auto outside = map_checker.check_pose({0.0, 0.0, 0.0});
  EXPECT_TRUE(outside.collided());
  EXPECT_EQ(outside.reason_code, collision::CollisionReason::kOutsideMap);

  collision::Scene empty_scene;
  collision::CollisionChecker empty_checker(vehicle, empty_scene);
  ASSERT_TRUE(empty_checker.valid()) << empty_checker.invalid_reason();
  EXPECT_TRUE(empty_checker.check_pose({1000.0, -500.0, 1.0}).is_clear());
}

TEST(CollisionChecker, RotatedGridFramePoseChecks) {
  const auto vehicle = test_vehicle();
  auto scene = make_scene(20, 20, 0.5);
  scene.grid.origin = {0.3, -0.4, 0.7};
  collision::CollisionChecker clear_checker(vehicle, scene);
  ASSERT_TRUE(clear_checker.valid()) << clear_checker.invalid_reason();

  // 选取栅格坐标系中心附近的位姿，保证旋转地图内足迹完全位于边界内
  const double c = std::cos(scene.grid.origin.yaw_rad);
  const double s = std::sin(scene.grid.origin.yaw_rad);
  const common::Pose2d pose{scene.grid.origin.x_m + c * 5.25 - s * 5.25,
                            scene.grid.origin.y_m + s * 5.25 + c * 5.25, 0.2};
  EXPECT_TRUE(clear_checker.check_pose(pose).is_clear());

  // 标记足迹前左角附近的内点（车体系 (3.3, 0.9)）所在像元：
  // 若栅格系 OBB 航向构造错误，该角点区域将不再与车体相交，检测会漏报
  const double cp = std::cos(pose.yaw_rad);
  const double sp = std::sin(pose.yaw_rad);
  const common::Point2d near_corner{pose.x_m + cp * 3.3 - sp * 0.9,
                                    pose.y_m + sp * 3.3 + cp * 0.9};
  const common::GridFrame frame = common::GridFrame::from_grid(scene.grid);
  const auto grid_corner = frame.to_grid_frame(near_corner);
  const int col = static_cast<int>(std::floor(grid_corner.x_m / scene.grid.resolution_m));
  const int row = static_cast<int>(std::floor(grid_corner.y_m / scene.grid.resolution_m));
  ASSERT_TRUE(scene.grid.in_bounds(row, col));
  scene.grid.cells[static_cast<std::size_t>(row) * scene.grid.width + col] =
      common::kCellOccupied;

  collision::CollisionChecker occupied_checker(vehicle, scene);
  ASSERT_TRUE(occupied_checker.valid()) << occupied_checker.invalid_reason();
  const auto occupied = occupied_checker.check_pose(pose);
  EXPECT_TRUE(occupied.collided());
  EXPECT_EQ(occupied.reason_code, collision::CollisionReason::kGridOccupied);
}

TEST(CollisionChecker, PoseObstacleToleranceBoundary) {
  const auto vehicle = test_vehicle();
  auto scene = make_scene(20, 20);
  // 足迹前缘位于 pose.x + 3.4 m（轴距 2.5 + 前悬 0.8 + 安全裕量 0.1）
  scene.obstacles.push_back(collision::Obstacle::rectangle("front_touch", 8.4, 4.0, 9.4, 6.0));
  collision::CollisionChecker checker(vehicle, scene);
  ASSERT_TRUE(checker.valid()) << checker.invalid_reason();

  const auto touching = checker.check_pose({5.0, 5.0, 0.0});
  EXPECT_TRUE(touching.collided());
  EXPECT_EQ(touching.reason_code, collision::CollisionReason::kObstacle);

  // 后退 1e-6 m 后间隙超过接触容差，应判定为无碰撞
  EXPECT_TRUE(checker.check_pose({5.0 - 1e-6, 5.0, 0.0}).is_clear());
}

TEST(CollisionChecker, PoseObstacleCornerSliverUsesEuclideanDistance) {
  const auto vehicle = test_vehicle();
  const double tol = 1e-12 * std::max(1.0, 9.4);
  const auto check = [&](double factor) {
    const double delta = factor * tol;
    auto scene = make_scene(20, 20);
    scene.obstacles.push_back(collision::Obstacle::rectangle(
        "corner_sliver", 8.4 + delta, 6.0 + delta, 9.4 + delta, 7.0 + delta));
    collision::CollisionChecker checker(vehicle, scene);
    EXPECT_TRUE(checker.valid()) << checker.invalid_reason();
    return checker.check_pose({5.0, 5.0, 0.0});
  };

  // 两轴间隙均为 0.9*tol，欧氏距离约 1.27*tol：SAT 粗筛通过但精确距离应拒绝
  EXPECT_TRUE(check(0.9).is_clear());
  // 两轴间隙均为 0.5*tol，欧氏距离约 0.71*tol：应判定为碰撞
  const auto touching = check(0.5);
  EXPECT_TRUE(touching.collided());
  EXPECT_EQ(touching.reason_code, collision::CollisionReason::kObstacle);
}

TEST(CollisionChecker, PoseGridCornerSliverUsesEuclideanDistance) {
  const auto vehicle = test_vehicle();
  const auto footprint = collision::make_footprint(vehicle);
  const double res = 0.5;
  const double cell_min = 6.0;
  const double tol = 1e-12 * std::max(1.0, cell_min);
  const auto check = [&](double factor) {
    const double delta = factor * tol;
    auto scene = make_scene(20, 20, res);
    scene.grid.cells[12 * scene.grid.width + 12] = common::kCellOccupied;
    collision::CollisionChecker checker(vehicle, scene);
    EXPECT_TRUE(checker.valid()) << checker.invalid_reason();
    // 使足迹前左角恰好位于阻塞像元左下角 (6,6) 的左上方向 delta 处
    const common::Pose2d pose{cell_min - delta - footprint.x_max_m,
                              cell_min - delta - footprint.y_max_m, 0.0};
    return checker.check_pose(pose);
  };

  // 两轴间隙 0.9*tol，欧氏距离约 1.27*tol：SAT 粗筛通过但精确距离应拒绝
  EXPECT_TRUE(check(0.9).is_clear());
  // 两轴间隙 0.5*tol，欧氏距离约 0.71*tol：应判定为碰撞
  const auto touching = check(0.5);
  EXPECT_TRUE(touching.collided());
  EXPECT_EQ(touching.reason_code, collision::CollisionReason::kGridOccupied);
}

TEST(CollisionChecker, MotionClearAndCollide) {
  const auto vehicle = test_vehicle();
  auto scene = make_scene();
  collision::CollisionChecker clear_checker(vehicle, scene);
  ASSERT_TRUE(clear_checker.valid());
  collision::MotionSegment clear_motion;
  clear_motion.start = {3.0, 5.0, 0.0};
  clear_motion.length_m = 3.0;
  EXPECT_TRUE(clear_checker.check_motion(clear_motion).is_clear());

  scene.obstacles.push_back(collision::Obstacle::rectangle("blocking_box", 10.0, 4.0, 11.0, 6.0));
  collision::CollisionChecker checker(vehicle, scene);
  ASSERT_TRUE(checker.valid());
  collision::MotionSegment blocked_motion;
  blocked_motion.start = {3.0, 5.0, 0.0};
  blocked_motion.length_m = 8.0;
  const auto blocked = checker.check_motion(blocked_motion);
  EXPECT_TRUE(blocked.collided());
  EXPECT_EQ(blocked.reason_code, collision::CollisionReason::kObstacle);
  EXPECT_GT(blocked.first_s_m, 0.0);
  EXPECT_LE(blocked.first_s_m, blocked_motion.length_m);

  collision::MotionSegment reverse_motion;
  reverse_motion.start = {12.0, 5.0, 0.0};
  reverse_motion.length_m = 3.0;
  reverse_motion.direction = common::Direction::kReverse;
  const auto reverse = checker.check_motion(reverse_motion);
  EXPECT_TRUE(reverse.collided());
  EXPECT_EQ(reverse.reason_code, collision::CollisionReason::kObstacle);
}

TEST(CollisionChecker, PathCheckClearAndCollide) {
  const auto vehicle = test_vehicle();
  auto scene = make_scene();
  scene.obstacles.push_back(collision::Obstacle::rectangle("path_box", 10.0, 4.0, 11.0, 6.0));
  collision::CollisionChecker checker(vehicle, scene);
  ASSERT_TRUE(checker.valid());

  common::PathSegment clear_path;
  clear_path.points = {path_point(2.0, 5.0, 0.0), path_point(4.0, 5.0, 2.0),
                       path_point(6.0, 5.0, 4.0)};
  EXPECT_TRUE(checker.check_path(clear_path).is_clear());

  common::PathSegment blocked_path;
  blocked_path.points = {path_point(2.0, 5.0, 0.0), path_point(4.0, 5.0, 2.0),
                         path_point(6.0, 5.0, 4.0), path_point(8.0, 5.0, 6.0)};
  const auto blocked = checker.check_path(blocked_path);
  EXPECT_TRUE(blocked.collided());
  EXPECT_EQ(blocked.path_segment_index, 0);
  EXPECT_GT(blocked.first_s_m, 0.0);

  common::PathSegment first_segment;
  first_segment.points = {path_point(2.0, 5.0, 0.0), path_point(4.0, 5.0, 2.0),
                          path_point(6.0, 5.0, 4.0)};
  common::PathSegment second_segment;
  second_segment.points = {path_point(6.0, 5.0, 0.0), path_point(8.0, 5.0, 2.0)};
  const auto multi =
      checker.check_path(std::vector<common::PathSegment>{first_segment, second_segment});
  EXPECT_TRUE(multi.collided());
  EXPECT_EQ(multi.path_segment_index, 1);
}

TEST(CollisionChecker, InvalidInputAndInvalidConstruction) {
  const auto vehicle = test_vehicle();
  auto scene = make_scene();
  collision::CollisionChecker checker(vehicle, scene);
  ASSERT_TRUE(checker.valid());

  EXPECT_EQ(checker.check_pose({std::numeric_limits<double>::quiet_NaN(), 0, 0}).status,
            collision::CheckStatus::kInvalidInput);

  collision::MotionSegment motion;
  motion.start = {3.0, 5.0, 0.0};
  motion.length_m = -1.0;
  EXPECT_EQ(checker.check_motion(motion).status, collision::CheckStatus::kInvalidInput);
  motion.length_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(checker.check_motion(motion).status, collision::CheckStatus::kInvalidInput);
  motion.length_m = 1.0;
  motion.curvature_per_m = 2.0e6;
  EXPECT_EQ(checker.check_motion(motion).status, collision::CheckStatus::kInvalidInput);

  common::PathSegment empty_path;
  EXPECT_EQ(checker.check_path(empty_path).status, collision::CheckStatus::kInvalidInput);
  EXPECT_EQ(checker.check_path(std::vector<common::PathSegment>{}).status,
            collision::CheckStatus::kInvalidInput);

  common::PathSegment mixed_direction;
  mixed_direction.points = {path_point(2.0, 5.0, 0.0),
                            path_point(3.0, 5.0, 1.0, common::Direction::kReverse)};
  EXPECT_EQ(checker.check_path(mixed_direction).status, collision::CheckStatus::kInvalidInput);

  common::PathSegment decreasing_s;
  decreasing_s.points = {path_point(2.0, 5.0, 1.0), path_point(3.0, 5.0, 1.0)};
  EXPECT_EQ(checker.check_path(decreasing_s).status, collision::CheckStatus::kInvalidInput);

  common::VehicleModel invalid_vehicle = vehicle;
  invalid_vehicle.wheelbase_m = 0.0;
  collision::CollisionChecker invalid_checker(invalid_vehicle, scene);
  EXPECT_FALSE(invalid_checker.valid());
  EXPECT_FALSE(invalid_checker.invalid_reason().empty());
  EXPECT_EQ(invalid_checker.check_pose({5.0, 5.0, 0.0}).status,
            collision::CheckStatus::kInvalidInput);

  collision::CheckOptions invalid_options;
  invalid_options.sweep_precision_m = 0.0;
  collision::CollisionChecker bad_options_checker(vehicle, scene, invalid_options);
  EXPECT_FALSE(bad_options_checker.valid());
}

TEST(CollisionFootprint, BoxGeometryPreservesLegacyCornerValues) {
  const auto vehicle = test_vehicle();
  const auto footprint = collision::make_footprint(vehicle);
  EXPECT_EQ(footprint.body_box().corners(), footprint.body_corners());
  std::mt19937 engine(9);
  std::uniform_real_distribution<double> coordinate(-20.0, 20.0);
  std::uniform_real_distribution<double> angle(-math::kPi, math::kPi);
  for (int i = 0; i < 200; ++i) {
    SCOPED_TRACE(i);
    const common::Pose2d pose{coordinate(engine), coordinate(engine), angle(engine)};
    const auto expected_body = footprint.body_corners();
    const auto expected = footprint.world_corners(pose);
    const auto box_corners = footprint.world_box(pose).corners();
    for (std::size_t k = 0; k < 4; ++k) {
      EXPECT_EQ(expected[k], math::transform_point(pose, expected_body[k]));
      EXPECT_EQ(box_corners[k], expected[k]);
    }
    const auto aabb = footprint.world_box(pose).aabb();
    for (const auto& p : box_corners) {
      EXPECT_TRUE(aabb.contains(p));
    }
  }
}

TEST(CollisionFootprint, ExplicitCornerWorldBoxMatchesGenericAabbPredicate) {
  const auto vehicle = test_vehicle();
  const auto footprint = collision::make_footprint(vehicle);
  std::mt19937 engine(20261113);
  std::uniform_real_distribution<double> coordinate(-20.0, 20.0);
  std::uniform_real_distribution<double> size(0.2, 4.0);
  std::uniform_real_distribution<double> angle(-math::kPi, math::kPi);
  for (int i = 0; i < 200; ++i) {
    SCOPED_TRACE(i);
    const common::Pose2d pose{coordinate(engine), coordinate(engine), angle(engine)};
    const math::Box2d box = footprint.world_box(pose);
    const auto corners = box.corners();
    EXPECT_EQ(corners, footprint.world_corners(pose));
    for (double tol : {0.0, 1e-9, 0.01}) {
      const double x = coordinate(engine);
      const double y = coordinate(engine);
      const double w = size(engine);
      const double h = size(engine);
      const math::Aabb2d probe{x, y, x + w, y + h};
      EXPECT_EQ(box.intersects(probe, tol),
                math::convex_polygon_intersects_aabb(corners.data(), corners.size(), probe, tol));
    }
  }
}

TEST(CollisionObstacle, FactoriesNormalizeAndCacheGeometry) {
  const auto rectangle = collision::Obstacle::rectangle("rect", -1.0, -2.0, 3.0, 4.0);
  ASSERT_TRUE(rectangle.polygon.valid());
  EXPECT_EQ(rectangle.id, "rect");
  EXPECT_EQ(rectangle.polygon.size(), 4u);
  EXPECT_FALSE(rectangle.polygon.empty());
  EXPECT_EQ(rectangle.polygon.data(), rectangle.polygon.vertices().data());
  EXPECT_TRUE(math::is_counter_clockwise(rectangle.polygon.vertices()));
  EXPECT_EQ(rectangle.polygon.vertices().front(), (common::Point2d{-1.0, -2.0}));
  EXPECT_DOUBLE_EQ(rectangle.polygon.aabb().min_x, -1.0);
  EXPECT_DOUBLE_EQ(rectangle.polygon.aabb().min_y, -2.0);
  EXPECT_DOUBLE_EQ(rectangle.polygon.aabb().max_x, 3.0);
  EXPECT_DOUBLE_EQ(rectangle.polygon.aabb().max_y, 4.0);

  const auto clockwise =
      collision::Obstacle::convex_polygon("cw", {{0.0, 0.0}, {0.0, 2.0}, {2.0, 2.0}, {2.0, 0.0}});
  ASSERT_TRUE(clockwise.polygon.valid());
  EXPECT_TRUE(math::is_counter_clockwise(clockwise.polygon.vertices()));
  EXPECT_DOUBLE_EQ(clockwise.polygon.aabb().max_x, 2.0);
  EXPECT_DOUBLE_EQ(clockwise.polygon.aabb().max_y, 2.0);

  const auto invalid =
      collision::Obstacle::convex_polygon("bad", {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  EXPECT_TRUE(invalid.polygon.empty());
  EXPECT_FALSE(invalid.polygon.valid());
  EXPECT_DOUBLE_EQ(invalid.polygon.aabb().min_x, 0.0);
  EXPECT_DOUBLE_EQ(invalid.polygon.aabb().max_x, 0.0);

  const auto inverted = collision::Obstacle::rectangle("inverted", 2.0, 0.0, 1.0, 1.0);
  EXPECT_TRUE(inverted.polygon.empty());
  EXPECT_FALSE(inverted.polygon.valid());
}

TEST(CollisionObstacle, StringViewIdIsCopied) {
  std::string source = "factory_source";
  const auto rectangle =
      collision::Obstacle::rectangle(std::string_view(source), 0.0, 0.0, 2.0, 2.0);
  const auto polygon = collision::Obstacle::convex_polygon(
      std::string_view(source), {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}});
  source.assign("mutated");
  EXPECT_EQ(rectangle.id, "factory_source");
  EXPECT_EQ(polygon.id, "factory_source");
}

TEST(CollisionToString, KnownAndInvalidEnumerators) {
  EXPECT_EQ(collision::to_string(collision::CheckStatus::kClear), std::string_view("clear"));
  EXPECT_EQ(collision::to_string(collision::CollisionReason::kObstacle),
            std::string_view("obstacle"));
  EXPECT_EQ(collision::to_string(static_cast<collision::CheckStatus>(99)),
            std::string_view("unknown"));
  EXPECT_EQ(collision::to_string(static_cast<collision::CollisionReason>(99)),
            std::string_view("unknown"));
}

}  // namespace
