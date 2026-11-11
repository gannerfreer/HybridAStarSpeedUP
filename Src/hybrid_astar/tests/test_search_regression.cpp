#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "planner/common/path.hpp"
#include "planner/common/plan.hpp"
#include "planner/common/status.hpp"
#include "planner/common/vehicle.hpp"
#include "planner/hybrid_astar/detail/search_engine.hpp"
#include "planner/math/angle.hpp"
#include "planner/math/kinematics/arc.hpp"
#include "planner/math/kinematics/bicycle_model.hpp"
#include "reeds_shepp.hpp"
#include "user_astar.hpp"

namespace {
using namespace planner;

common::VehicleModel regression_vehicle() {
  common::VehicleModel vehicle;
  vehicle.wheelbase_m = 2.5;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.8;
  vehicle.rear_overhang_m = 0.8;
  vehicle.max_steer_rad = 0.6;
  return vehicle;
}

// 极小车体和薄障碍物放大旧采样检查的弧长缺口，确保问题不会被车体尺寸掩盖
TEST(SearchRegression, RejectsCollisionInArcTail) {
  common::VehicleModel vehicle;
  vehicle.wheelbase_m = 0.0001;
  vehicle.width_m = 0.0001;
  vehicle.front_overhang_m = 0.00005;
  vehicle.rear_overhang_m = 0.00005;
  vehicle.max_steer_rad = std::atan(vehicle.wheelbase_m);  // R = 1 m
  collision::Scene scene;
  auto& grid = scene.grid;
  grid.map_id = "arc_tail";
  grid.map_version = "1";
  grid.width = grid.height = 20;
  grid.resolution_m = 0.5;
  grid.cells.assign(400, common::kCellFree);
  const common::Pose2d start{5, 5, 0};
  const common::Pose2d goal = math::integrate_arc(start, math::kPi / 2, 1);
  const auto hit = math::integrate_arc(start, 0.498, 1);
  scene.obstacles.push_back(collision::Obstacle::rectangle(
      "thin_obstacle", hit.x_m - 0.00002, hit.y_m - 0.00002, hit.x_m + 0.00002, hit.y_m + 0.00002));
  collision::CheckOptions options;
  options.sweep_precision_m = 1e-6;
  collision::CollisionChecker checker(vehicle, scene, options);
  ASSERT_TRUE(checker.valid()) << "arc tail scene must be valid";
  EXPECT_TRUE(checker.check_pose(start).is_clear() && checker.check_pose(goal).is_clear())
      << "arc tail endpoints must be clear";
  const auto solved = rs::plan(start, goal, vehicle);
  ASSERT_EQ(solved.status, common::Status::kSuccess) << "arc RS solution";
  const auto samples = rs::sample(solved.path, 0.5);
  ASSERT_GE(samples.size(), 2u);
  bool arc_length_detected_collision = false;
  for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
    SCOPED_TRACE(i);
    collision::MotionSegment segment;
    segment.start = samples[i].pose;
    segment.direction = samples[i].direction;
    segment.curvature_per_m = samples[i].curvature_per_m;
    segment.length_m = std::hypot(samples[i + 1].pose.x_m - samples[i].pose.x_m,
                                  samples[i + 1].pose.y_m - samples[i].pose.y_m);
    EXPECT_TRUE(checker.check_motion(segment).is_clear())
        << "regression setup must reproduce the old chord-length miss";
    segment.length_m = samples[i + 1].s_m - samples[i].s_m;
    arc_length_detected_collision |= checker.check_motion(segment).collided();
  }
  EXPECT_TRUE(arc_length_detected_collision) << "full arc must hit the thin obstacle";
  common::PlanRequest request;
  request.map_id = grid.map_id;
  request.map_version = grid.map_version;
  request.vehicle = vehicle;
  request.start = start;
  request.goal = goal;
  request.node_budget = 1;
  hybrid_astar::UserAstar planner;
  const auto result = planner.plan(request, grid, checker);
  // 连接必须在目标钩子中被拒绝，让搜索耗尽节点预算；旧代码误接受后返回 validation_failed
  EXPECT_EQ(result.status, common::Status::kTimeout);
  EXPECT_TRUE(result.path.empty())
      << "colliding RS connector must be rejected before output validation";
}

TEST(SearchEngineRegression, MissingHooksReturnInvalidInput) {
  const common::VehicleModel vehicle = regression_vehicle();
  const common::UserAstarConfig config;
  hybrid_astar::detail::SearchEngine engine(vehicle, config);
  const auto missing_all = engine.search({0, 0, 0}, {1, 0, 0}, 10);
  EXPECT_EQ(missing_all.status, common::Status::kInvalidInput);
  EXPECT_NE(missing_all.reason.find("hook"), std::string::npos);

  engine.set_pose_collision_hook([](const math::Pose2d&) { return false; });
  engine.set_goal_hook([](const hybrid_astar::detail::SearchState&) { return true; });
  const auto missing_motion = engine.search({0, 0, 0}, {1, 0, 0}, 10);
  EXPECT_EQ(missing_motion.status, common::Status::kInvalidInput);
  EXPECT_NE(missing_motion.reason.find("hook"), std::string::npos);
}

TEST(SearchEngineRegression, AbortReturnsCancelled) {
  const common::VehicleModel vehicle = regression_vehicle();
  const common::UserAstarConfig config;
  hybrid_astar::detail::SearchEngine engine(vehicle, config);
  engine.set_pose_collision_hook([](const math::Pose2d&) { return false; });
  engine.set_motion_collision_hook([](const hybrid_astar::detail::SearchState&,
                                      const hybrid_astar::detail::SearchState&) { return false; });
  engine.set_goal_hook([](const hybrid_astar::detail::SearchState&) { return false; });
  engine.set_abort_callback([] { return true; });
  const auto result = engine.search({0, 0, 0}, {10, 0, 0}, 100);
  EXPECT_EQ(result.status, common::Status::kCancelled);
  EXPECT_EQ(result.reason, "aborted");
  EXPECT_EQ(result.iterations, 0);
}

TEST(SearchEngineRegression, NodeBudgetReturnsTimeout) {
  const common::VehicleModel vehicle = regression_vehicle();
  const common::UserAstarConfig config;
  hybrid_astar::detail::SearchEngine engine(vehicle, config);
  engine.set_pose_collision_hook([](const math::Pose2d&) { return false; });
  engine.set_motion_collision_hook([](const hybrid_astar::detail::SearchState&,
                                      const hybrid_astar::detail::SearchState&) { return false; });
  engine.set_goal_hook([](const hybrid_astar::detail::SearchState&) { return false; });
  const auto result = engine.search({0, 0, 0}, {100, 0, 0}, 2);
  EXPECT_EQ(result.status, common::Status::kTimeout);
  EXPECT_EQ(result.iterations, 2);
  EXPECT_NE(result.reason.find("node budget"), std::string::npos);
}

TEST(SearchEngineRegression, RejectedSuccessorsReturnNoPath) {
  const common::VehicleModel vehicle = regression_vehicle();
  const common::UserAstarConfig config;
  hybrid_astar::detail::SearchEngine engine(vehicle, config);
  engine.set_pose_collision_hook([](const math::Pose2d&) { return false; });
  engine.set_motion_collision_hook([](const hybrid_astar::detail::SearchState&,
                                      const hybrid_astar::detail::SearchState&) { return true; });
  engine.set_goal_hook([](const hybrid_astar::detail::SearchState&) { return false; });
  const auto result = engine.search({0, 0, 0}, {10, 0, 0}, 100);
  EXPECT_EQ(result.status, common::Status::kNoPath);
  EXPECT_TRUE(result.path.empty());
}

TEST(SearchEngineRegression, StartPoseCollisionReturnsInvalidInput) {
  const common::VehicleModel vehicle = regression_vehicle();
  const common::UserAstarConfig config;
  hybrid_astar::detail::SearchEngine engine(vehicle, config);
  engine.set_pose_collision_hook([](const math::Pose2d&) { return true; });
  engine.set_motion_collision_hook([](const hybrid_astar::detail::SearchState&,
                                      const hybrid_astar::detail::SearchState&) { return false; });
  engine.set_goal_hook([](const hybrid_astar::detail::SearchState&) { return false; });
  const auto result = engine.search({0, 0, 0}, {10, 0, 0}, 100);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("collision"), std::string::npos);
}

TEST(SearchEngineRegression, ReconstructsMultiNodePathFromOrigin) {
  const common::VehicleModel vehicle = regression_vehicle();
  common::UserAstarConfig config;
  config.motion_step_m = 1.0;
  config.steering_samples = 5;
  hybrid_astar::detail::SearchEngine engine(vehicle, config);
  const math::Pose2d goal{2.0, 0.0, 0.0};
  engine.set_pose_collision_hook([](const math::Pose2d&) { return false; });
  engine.set_motion_collision_hook([](const hybrid_astar::detail::SearchState&,
                                      const hybrid_astar::detail::SearchState&) { return false; });
  engine.set_goal_hook([&](const hybrid_astar::detail::SearchState& state) {
    return std::hypot(state.pose.x_m - goal.x_m, state.pose.y_m - goal.y_m) <= 1.1;
  });
  const auto result = engine.search({0, 0, 0}, goal, 100);
  ASSERT_EQ(result.status, common::Status::kSuccess);
  ASSERT_GE(result.path.size(), 2u);
  EXPECT_NEAR(result.path.front().pose.x_m, 0.0, 1e-12);
  EXPECT_NEAR(result.path.front().pose.y_m, 0.0, 1e-12);
  EXPECT_LE(
      std::hypot(result.path.back().pose.x_m - goal.x_m, result.path.back().pose.y_m - goal.y_m),
      1.1);
}

TEST(SearchEngineRegression, ReuseClearsPreviousSearchState) {
  const common::VehicleModel vehicle = regression_vehicle();
  const common::UserAstarConfig config;
  hybrid_astar::detail::SearchEngine engine(vehicle, config);
  math::Pose2d target{2.0, 0.0, 0.0};
  engine.set_pose_collision_hook([](const math::Pose2d&) { return false; });
  engine.set_motion_collision_hook([](const hybrid_astar::detail::SearchState&,
                                      const hybrid_astar::detail::SearchState&) { return false; });
  engine.set_goal_hook([&](const hybrid_astar::detail::SearchState& state) {
    return std::hypot(state.pose.x_m - target.x_m, state.pose.y_m - target.y_m) <= 1.1;
  });
  const auto first = engine.search({0, 0, 0}, target, 100);
  ASSERT_EQ(first.status, common::Status::kSuccess);
  ASSERT_GE(first.path.size(), 2u);

  target = {3.0, 0.0, 0.0};
  const auto second = engine.search({0, 0, 0}, target, 100);
  EXPECT_EQ(second.status, common::Status::kSuccess);
  EXPECT_GE(second.path.size(), 2u);
  EXPECT_NEAR(second.path.front().pose.x_m, 0.0, 1e-12);
}

TEST(SearchEngineRegression, ReconstructsStartAtNegativePi) {
  const common::VehicleModel vehicle = regression_vehicle();
  const common::UserAstarConfig config;
  hybrid_astar::detail::SearchEngine engine(vehicle, config);
  engine.set_pose_collision_hook([](const math::Pose2d&) { return false; });
  engine.set_motion_collision_hook([](const hybrid_astar::detail::SearchState&,
                                      const hybrid_astar::detail::SearchState&) { return false; });
  engine.set_goal_hook([](const hybrid_astar::detail::SearchState&) { return true; });
  const math::Pose2d boundary{0.0, 0.0, -math::kPi};
  const auto result = engine.search(boundary, boundary, 10);
  ASSERT_EQ(result.status, common::Status::kSuccess);
  ASSERT_FALSE(result.path.empty());
  EXPECT_NEAR(result.path.front().pose.x_m, 0.0, 1e-12);
  EXPECT_NEAR(result.path.front().pose.y_m, 0.0, 1e-12);
  EXPECT_NEAR(std::abs(math::normalize_angle_rad(result.path.front().pose.yaw_rad + math::kPi)),
              0.0, 1e-12);
}

TEST(SearchEngineRegression, MotionHookReceivesSiPrimitive) {
  common::VehicleModel vehicle = regression_vehicle();
  common::UserAstarConfig config;
  config.motion_step_m = 0.75;
  config.steering_samples = 3;
  config.allow_reverse = true;
  hybrid_astar::detail::SearchEngine engine(vehicle, config);
  engine.set_pose_collision_hook([](const math::Pose2d&) { return false; });
  engine.set_goal_hook([](const hybrid_astar::detail::SearchState&) { return false; });
  hybrid_astar::detail::SearchState first_from;
  hybrid_astar::detail::SearchState first_to;
  int calls = 0;
  engine.set_motion_collision_hook([&](const hybrid_astar::detail::SearchState& from,
                                       const hybrid_astar::detail::SearchState& to) {
    if (calls++ == 0) {
      first_from = from;
      first_to = to;
    }
    return true;
  });
  const auto result = engine.search({0, 0, 0}, {10, 0, 0}, 1);
  EXPECT_EQ(result.status, common::Status::kTimeout);
  ASSERT_GT(calls, 0);
  const double expected_curvature =
      math::kinematics::curvature_from_steer(-vehicle.max_steer_rad, vehicle.wheelbase_m);
  EXPECT_NEAR(first_from.curvature_per_m, 0.0, 1e-12);
  EXPECT_NEAR(first_from.arc_length_m, 0.0, 1e-12);
  EXPECT_EQ(first_to.direction, common::Direction::kForward);
  EXPECT_NEAR(first_to.arc_length_m, config.motion_step_m, 1e-12);
  EXPECT_NEAR(first_to.curvature_per_m, expected_curvature, 1e-12);
  const math::Pose2d expected =
      math::integrate_arc({0, 0, 0}, config.motion_step_m, expected_curvature);
  EXPECT_NEAR(first_to.pose.x_m, expected.x_m, 1e-12);
  EXPECT_NEAR(first_to.pose.y_m, expected.y_m, 1e-12);
  EXPECT_NEAR(first_to.pose.yaw_rad, expected.yaw_rad, 1e-12);
}

TEST(UserAstarRegression, HooksReverseCancellationBudgetsPiBoundaryAndRsIntegration) {
  const common::VehicleModel vehicle = regression_vehicle();
  collision::Scene scene;
  auto& grid = scene.grid;
  grid.map_id = "regression";
  grid.map_version = "1";
  grid.width = 80;
  grid.height = 80;
  grid.resolution_m = 0.5;
  grid.cells.assign(6400, 0);
  collision::CollisionChecker checker(vehicle, scene);
  hybrid_astar::UserAstar planner;
  common::PlanRequest request;
  request.map_id = grid.map_id;
  request.map_version = grid.map_version;
  request.vehicle = vehicle;
  request.start = {10, 10, 0};
  request.goal = {6, 10, 0};
  auto result = planner.plan(request, grid, checker);
  ASSERT_EQ(result.status, common::Status::kSuccess) << "reverse planning";
  EXPECT_TRUE(checker.check_path(result.path).is_clear());
  bool reverse = false;
  for (const auto& segment : result.path)
    for (const auto& p : segment.points) reverse |= p.direction == common::Direction::kReverse;
  EXPECT_TRUE(reverse) << "reverse motion required";

  request.cancelled = true;
  EXPECT_EQ(planner.plan(request, grid, checker).status, common::Status::kCancelled)
      << "request cancellation";
  request.cancelled = false;
  planner.set_cancel_callback([] { return true; });
  EXPECT_EQ(planner.plan(request, grid, checker).status, common::Status::kCancelled)
      << "callback cancellation";
  planner.set_cancel_callback({});

  request.goal = {30, 10, 0};
  request.node_budget = 1;
  EXPECT_EQ(planner.plan(request, grid, checker).status, common::Status::kTimeout) << "node budget";
  request.node_budget.reset();
  request.time_budget_s = 1e-12;
  EXPECT_EQ(planner.plan(request, grid, checker).status, common::Status::kTimeout) << "time budget";
  request.time_budget_s.reset();

  request.start = {30, 10, math::kPi};
  request.goal = {10, 10, math::kPi};
  auto plus = planner.plan(request, grid, checker);
  request.start.yaw_rad = -math::kPi;
  request.goal.yaw_rad = -math::kPi;
  auto minus = planner.plan(request, grid, checker);
  ASSERT_EQ(plus.status, common::Status::kSuccess);
  ASSERT_EQ(minus.status, common::Status::kSuccess);
  EXPECT_TRUE(checker.check_path(plus.path).is_clear() && checker.check_path(minus.path).is_clear())
      << "pi boundary collision";
  EXPECT_LT(std::abs(plus.path.back().points.back().s_m - minus.path.back().points.back().s_m),
            1e-9)
      << "pi paths must agree";

  // 对外 RS 积分应使用共享内核；包括微小曲率与倒车
  rs::RsSegment segment;
  segment.length_m = 2;
  segment.curvature_per_m = 1e-12;
  segment.direction = common::Direction::kReverse;
  const auto sampled = rs::integrate_segment({2, 3, 0.7}, segment, 2);
  const auto expected = math::integrate_arc({2, 3, 0.7}, -2, 1e-12);
  EXPECT_LT(std::hypot(sampled.x_m - expected.x_m, sampled.y_m - expected.y_m), 1e-12)
      << "RS integration must share math";
}

}  // namespace
