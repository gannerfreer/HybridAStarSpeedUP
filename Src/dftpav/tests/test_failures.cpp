#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "dftpav_test_support.hpp"

namespace planner::dftpav::test {
namespace {

OptimizeRequest base_request() {
  OptimizeRequest request;
  request.request_id = "test-failure";
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

// L 形、3.2 m 宽的走廊：沿其行驶会强制产生约 90 度转弯，
// 曲率远超配置的 0.05 1/m 限制。
OptimizeRequest sharp_turn_request() {
  OptimizeRequest request = base_request();
  request.request_id = "test-sharp-turn";
  request.seed = {straight_segment(0.0, 0.0, 5.0, 0.0, 1.0),
                  straight_segment(5.0, 0.0, 5.0, 5.0, 1.0)};  request.corridor = make_rect_corridor(
      {{0.0, 5.0}, {5.0, 10.0}},
      {Rect{-3.0, -1.6, 9.0, 1.6}, Rect{3.4, -3.0, 6.6, 9.0}});
  request.corridor.map_id = request.map_id;
  request.corridor.map_version = request.map_version;
  request.limits.max_forward_curvature_inv_m = 0.05;
  request.limits.max_backward_curvature_inv_m = 0.05;
  return request;
}

}  // namespace

TEST(DftpavFailures, EmptySeedIsRejected) {
  OptimizeRequest request = base_request();
  request.seed.clear();
  const OptimizeResult result = optimize_trajectory(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "invalid_seed");
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
}

TEST(DftpavFailures, EmptyCorridorIsRejected) {
  OptimizeRequest request = base_request();
  request.corridor.polygons.clear();
  const OptimizeResult result = optimize_trajectory(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "empty_corridor");
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
}

TEST(DftpavFailures, InvalidLimitsAreRejected) {
  OptimizeRequest request = base_request();
  request.limits.max_forward_velocity_mps = -1.0;
  const OptimizeResult result = optimize_trajectory(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "invalid_limits");
}

TEST(DftpavFailures, InvalidVehicleIsRejected) {
  OptimizeRequest request = base_request();
  request.vehicle.wheelbase_m = 0.0;
  const OptimizeResult result = optimize_trajectory(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "invalid_vehicle");
}

TEST(DftpavFailures, InvalidOptionsAreRejected) {
  OptimizeRequest request = base_request();
  request.options.sample_dt_s = 0.0;
  const OptimizeResult result = optimize_trajectory(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "invalid_options");
}

TEST(DftpavFailures, InvalidCorridorPolygonIsRejected) {
  OptimizeRequest request = base_request();
  request.corridor.polygons.front().vertices = {{0.0, 0.0}, {1.0, 1.0}};
  const OptimizeResult result = optimize_trajectory(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "invalid_corridor");
}

TEST(DftpavFailures, CorridorNotCoveringSeedIsRejected) {
  OptimizeRequest request = base_request();
  request.corridor = single_rect_corridor(5.0, Rect{-3.0, -4.0, 15.0, 4.0});
  request.corridor.map_id = request.map_id;
  request.corridor.map_version = request.map_version;
  const OptimizeResult result = optimize_trajectory(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "corridor_coverage");
}

TEST(DftpavFailures, InfeasibleCurvatureLimitNeverReportsSuccess) {
  const OptimizeRequest request = sharp_turn_request();
  const OptimizeResult result = optimize_trajectory(request);
  // 优化器可能放弃（代价/求解器），也可能收敛到仍违反约束的轨迹；
  // 两种情况下结果都不得声称轨迹有效（第 7.1 节）。
  EXPECT_FALSE(result.ok()) << "sharp turn with a 0.05 1/m limit must not succeed";
  EXPECT_FALSE(result.reason.empty());
  const bool expected_reason =
      result.reason == "validation_curvature_limit" ||
      result.reason == "validation_corridor_containment" ||
      result.reason == "provisional_curvature_limit" ||
      result.reason == "provisional_corridor_containment" ||
      result.reason == "provisional_lateral_acceleration_limit" ||
      result.reason == "cost_too_high" || result.reason == "solver_failed";
  EXPECT_TRUE(expected_reason) << "unexpected reason: " << result.reason;

  // 相同输入下失败状态可复现。
  const OptimizeResult repeat = optimize_trajectory(request);
  EXPECT_EQ(repeat.status, result.status);
  EXPECT_EQ(repeat.reason, result.reason);
  EXPECT_EQ(repeat.stats.final_cost, result.stats.final_cost);
  EXPECT_EQ(repeat.stats.lbfgs_result, result.stats.lbfgs_result);
}

TEST(DftpavFailures, NegativeDirectionSwitchVelocityIsRejected) {
  OptimizeRequest request = base_request();
  request.options.direction_switch_velocity_mps = -0.1;
  const OptimizeResult result = optimize_trajectory(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "invalid_options");
}

}  // namespace planner::dftpav::test
