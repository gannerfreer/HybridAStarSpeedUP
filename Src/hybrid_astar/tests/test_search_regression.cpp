#include "hybrid_astar.h"
#include "planner/math/kinematics/arc.hpp"
#include "reeds_shepp.hpp"
#include "user_astar.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace {
using namespace planner;

// 极小车体和薄障碍物放大旧采样检查的弧长缺口，确保问题不会被车体尺寸掩盖。
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
  const common::Pose2d goal = math::integrate_arc(start, common::kPi / 2, 1);
  const auto hit = math::integrate_arc(start, 0.498, 1);
  scene.obstacles.push_back(collision::Obstacle::rectangle(
      "thin_obstacle", hit.x_m - 0.00002, hit.y_m - 0.00002,
      hit.x_m + 0.00002, hit.y_m + 0.00002));
  collision::CheckOptions options;
  options.sweep_precision_m = 1e-6;
  collision::CollisionChecker checker(vehicle, scene, options);
  ASSERT_TRUE(checker.valid()) << "arc tail scene must be valid";
  EXPECT_TRUE(checker.check_pose(start).is_clear() &&
              checker.check_pose(goal).is_clear())
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
    segment.curvature_inv_m = samples[i].curvature_inv_m;
    segment.length_m =
        std::hypot(samples[i + 1].pose.x_m - samples[i].pose.x_m,
                   samples[i + 1].pose.y_m - samples[i].pose.y_m);
    EXPECT_TRUE(checker.check_motion(segment).is_clear())
        << "regression setup must reproduce the old chord-length miss";
    segment.length_m = samples[i + 1].s_m - samples[i].s_m;
    arc_length_detected_collision |= checker.check_motion(segment).collided();
  }
  EXPECT_TRUE(arc_length_detected_collision)
      << "full arc must hit the thin obstacle";
  common::PlanRequest request;
  request.map_id = grid.map_id;
  request.map_version = grid.map_version;
  request.vehicle = vehicle;
  request.start = start;
  request.goal = goal;
  request.node_budget = 1;
  hybrid_astar::UserAstar planner;
  const auto result = planner.plan(request, grid, checker);
  // 连接必须在目标钩子中被拒绝，让搜索耗尽节点预算；旧代码误接受后返回 validation_failed。
  EXPECT_EQ(result.status, common::Status::kTimeout);
  EXPECT_TRUE(result.path.empty())
      << "colliding RS connector must be rejected before output validation";
}

TEST(SearchRegression, HooksReverseCancellationBudgetsPiBoundaryAndRsIntegration) {
  astar::PlannerConfig config;
  astar::HybridAStar core(config);
  astar::HybridSearchResult raw;
  EXPECT_TRUE(!core.Search({0, 0, 0}, {1, 0, 0}, &raw) &&
              raw.message.find("hook") != std::string::npos)
      << "missing hooks must reject search";
  core.SetPoseCollisionHook([](const astar::Pose2D&) { return false; });
  core.SetGoalHook([](const astar::Pose2D&) { return true; });
  EXPECT_TRUE(!core.Search({0, 0, 0}, {1, 0, 0}, &raw))
      << "missing motion check must reject search";
  core.SetMotionCollisionHook(
      [](const astar::Pose2D&, const astar::Pose2D&) { return false; });
  core.SetAbortCallback([] { return true; });
  EXPECT_TRUE(!core.Search({0, 0, 0}, {1, 0, 0}, &raw) &&
              raw.message == "aborted")
      << "core cancellation";

  common::VehicleModel vehicle;
  vehicle.wheelbase_m = 2.5;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.8;
  vehicle.rear_overhang_m = 0.8;
  vehicle.max_steer_rad = 0.6;
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
    for (const auto& p : segment.points)
      reverse |= p.direction == common::Direction::kReverse;
  EXPECT_TRUE(reverse) << "reverse motion required";

  request.cancelled = true;
  EXPECT_EQ(planner.plan(request, grid, checker).status,
            common::Status::kCancelled)
      << "request cancellation";
  request.cancelled = false;
  planner.set_cancel_callback([] { return true; });
  EXPECT_EQ(planner.plan(request, grid, checker).status,
            common::Status::kCancelled)
      << "callback cancellation";
  planner.set_cancel_callback({});

  request.goal = {30, 10, 0};
  request.node_budget = 1;
  EXPECT_EQ(planner.plan(request, grid, checker).status,
            common::Status::kTimeout)
      << "node budget";
  request.node_budget.reset();
  request.time_budget_s = 1e-12;
  EXPECT_EQ(planner.plan(request, grid, checker).status,
            common::Status::kTimeout)
      << "time budget";
  request.time_budget_s.reset();

  request.start = {30, 10, common::kPi};
  request.goal = {10, 10, common::kPi};
  auto plus = planner.plan(request, grid, checker);
  request.start.yaw_rad = -common::kPi;
  request.goal.yaw_rad = -common::kPi;
  auto minus = planner.plan(request, grid, checker);
  ASSERT_EQ(plus.status, common::Status::kSuccess);
  ASSERT_EQ(minus.status, common::Status::kSuccess);
  EXPECT_TRUE(checker.check_path(plus.path).is_clear() &&
              checker.check_path(minus.path).is_clear())
      << "pi boundary collision";
  EXPECT_LT(std::abs(plus.path.back().points.back().s_m -
                     minus.path.back().points.back().s_m),
            1e-9)
      << "pi paths must agree";

  // 对外 RS 积分应使用共享内核；包括微小曲率与倒车。
  rs::RsSegment segment;
  segment.length_m = 2;
  segment.curvature_inv_m = 1e-12;
  segment.direction = common::Direction::kReverse;
  const auto sampled = rs::integrate_segment({2, 3, 0.7}, segment, 2);
  const auto expected = math::integrate_arc({2, 3, 0.7}, -2, 1e-12);
  EXPECT_LT(std::hypot(sampled.x_m - expected.x_m, sampled.y_m - expected.y_m),
            1e-12)
      << "RS integration must share math";
}

}  // namespace
