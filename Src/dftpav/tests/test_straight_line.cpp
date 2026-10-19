#include <gtest/gtest.h>

#include "dftpav_test_support.hpp"

namespace planner::dftpav::test {
namespace {

OptimizeRequest straight_request() {
  OptimizeRequest request;
  request.request_id = "test-straight";
  request.map_id = "test_map";
  request.map_version = "v1";
  request.vehicle = default_vehicle();
  request.seed = {straight_segment(0.0, 0.0, 10.0, 0.0, 1.0)};
  request.corridor = single_rect_corridor(10.0, Rect{-3.0, -4.0, 15.0, 4.0});
  request.corridor.map_id = request.map_id;
  request.corridor.map_version = request.map_version;
  request.limits = default_limits();
  return request;
}

}  // namespace

TEST(DftpavStraightLine, ConvergesAndMeetsPathTolerances) {
  const OptimizeRequest request = straight_request();
  const OptimizeResult result = optimize_trajectory(request);

  ASSERT_TRUE(result.ok()) << "reason=" << result.reason << " detail=" << result.detail;
  EXPECT_EQ(result.stats.direction_changes, 0);
  EXPECT_GT(result.stats.lbfgs_iterations, 0);
  // 收敛判据：可接受的 L-BFGS 终止码，且有限代价低于配置的失败阈值。
  EXPECT_LT(result.stats.final_cost, request.options.cost_fail_threshold);
  EXPECT_GE(result.stats.lbfgs_result, 0);

  ASSERT_GE(result.trajectory.size(), 3u);
  const common::TrajectoryPoint& first = result.trajectory.front();
  const common::TrajectoryPoint& last = result.trajectory.back();
  // 端点容差遵循路径标准（位置 <= 0.05 m、航向 <= 0.02 rad）。
  EXPECT_NEAR(last.pose.x_m, 10.0, 0.05);
  EXPECT_NEAR(last.pose.y_m, 0.0, 0.05);
  EXPECT_NEAR(last.pose.yaw_rad, 0.0, 0.02);
  EXPECT_NEAR(first.pose.x_m, 0.0, 0.05);
  EXPECT_NEAR(first.pose.y_m, 0.0, 0.05);
  // 两端均为静止。
  EXPECT_NEAR(first.velocity_mps, 0.0, 1e-6);
  EXPECT_NEAR(last.velocity_mps, 0.0, 1e-6);

  // 时间顺序与动力学限制，按第 7.1 节容差
  // 1e-6 * max(1, limit)。
  const CheckResult semantics =
      check_trajectory_semantics(result.trajectory, request.limits, 1e-6, 1);
  EXPECT_TRUE(semantics.ok) << semantics.reason << " at t=" << semantics.value;

  // 使用真实车辆足迹的独立走廊包含性检查。
  const CheckResult containment = check_corridor_and_obstacles(
      result.trajectory, request.vehicle, request.seed, request.corridor, {}, 0.0);
  EXPECT_TRUE(containment.ok) << containment.reason << " at t=" << containment.value;
}

TEST(DftpavStraightLine, IsDeterministic) {
  const OptimizeRequest request = straight_request();
  const OptimizeResult first = optimize_trajectory(request);
  const OptimizeResult second = optimize_trajectory(request);

  ASSERT_TRUE(first.ok()) << first.reason;
  ASSERT_TRUE(second.ok()) << second.reason;
  ASSERT_EQ(first.trajectory.size(), second.trajectory.size());
  for (std::size_t i = 0; i < first.trajectory.size(); ++i) {
    EXPECT_EQ(first.trajectory[i].t_s, second.trajectory[i].t_s);
    EXPECT_EQ(first.trajectory[i].pose.x_m, second.trajectory[i].pose.x_m);
    EXPECT_EQ(first.trajectory[i].pose.y_m, second.trajectory[i].pose.y_m);
    EXPECT_EQ(first.trajectory[i].velocity_mps, second.trajectory[i].velocity_mps);
    EXPECT_EQ(first.trajectory[i].curvature_inv_m, second.trajectory[i].curvature_inv_m);
  }
}

TEST(DftpavStraightLine, OnMapMismatchIsRejected) {
  OptimizeRequest request = straight_request();
  request.corridor.map_version = "v2";
  const OptimizeResult result = optimize_trajectory(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "map_mismatch");
}

}  // namespace planner::dftpav::test
