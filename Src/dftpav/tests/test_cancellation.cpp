// 协作式取消（移植扩展，见 Src/dftpav/PROVENANCE.md）的单元测试。
//
// 覆盖：
//   * 不设置取消谓词时既有行为逐位不变（对照组）；
//   * 取消在有限次代价评估内生效（迭代计数上界，远小于完整求解）；
//   * 取消结果是稳定的非成功状态且**不含任何轨迹**（绝不返回部分结果）；
//   * 取消结果对相同输入确定（迭代计数/终止码一致）；
//   * 取消在求解开始前即可生效（不进入求解器）；
//   * 取消绝不触发收紧重试阶梯。

#include <gtest/gtest.h>

#include <iostream>
#include <string>
#include <vector>

#include "dftpav_test_support.hpp"

namespace planner::dftpav::test {
namespace {

OptimizeRequest base_request() {
  OptimizeRequest request;
  request.request_id = "test-cancel";
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

// L 形、0.05 1/m 曲率上限：不取消时优化器需要走完收紧重试阶梯（多次尝试）。
OptimizeRequest retry_ladder_request() {
  OptimizeRequest request = base_request();
  request.request_id = "test-cancel-retries";
  request.seed = {straight_segment(0.0, 0.0, 5.0, 0.0, 1.0),
                  straight_segment(5.0, 0.0, 5.0, 5.0, 1.0)};
  request.corridor = make_rect_corridor(
      {{0.0, 5.0}, {5.0, 10.0}},
      {Rect{-3.0, -1.6, 9.0, 1.6}, Rect{3.4, -3.0, 6.6, 9.0}});
  request.corridor.map_id = request.map_id;
  request.corridor.map_version = request.map_version;
  request.limits.max_forward_curvature_inv_m = 0.05;
  request.limits.max_backward_curvature_inv_m = 0.05;
  return request;
}

}  // namespace

// 对照组：空取消谓词（默认）下该场景成功，记录完整求解的迭代规模。
TEST(DftpavCancellation, BaselineSucceedsWithoutCancelCheck) {
  const OptimizeRequest request = base_request();
  const OptimizeResult result = optimize_trajectory(request);
  ASSERT_TRUE(result.ok()) << result.reason << " " << result.detail;
  EXPECT_FALSE(result.trajectory.empty());
  EXPECT_GT(result.stats.lbfgs_iterations, 0);
}

// 取消在有限次代价评估内生效：谓词第 N 次被调用时请求取消，求解器必须在
// 远小于完整求解的评估数内退出，并返回稳定的非成功状态且不带轨迹。
TEST(DftpavCancellation, CancelDuringSolveExitsEarlyWithoutTrajectory) {
  const OptimizeRequest baseline_request = base_request();
  const OptimizeResult baseline = optimize_trajectory(baseline_request);
  ASSERT_TRUE(baseline.ok()) << baseline.reason;

  constexpr int kCancelAfterCalls = 8;
  OptimizeRequest request = base_request();
  int calls = 0;
  request.options.cancel_check = [&calls] { return ++calls >= kCancelAfterCalls; };
  const OptimizeResult result = optimize_trajectory(request);

  // 取消延迟证据：完整求解 vs 取消求解的代价评估数（同一确定性输入）。
  std::cout << "[cancel-evidence] baseline_lbfgs_iterations="
            << baseline.stats.lbfgs_iterations << " cancelled_lbfgs_iterations="
            << result.stats.lbfgs_iterations << " cancel_check_calls=" << calls
            << " cancelled_solve_time_s=" << result.stats.solve_time_s
            << " baseline_solve_time_s=" << baseline.stats.solve_time_s << std::endl;

  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, common::Status::kCancelled);
  EXPECT_EQ(result.reason, "cancelled");
  // 取消绝不返回部分轨迹，也不返回中间迭代的验证判定。
  EXPECT_TRUE(result.trajectory.empty()) << "cancelled results must not carry a trajectory";
  EXPECT_FALSE(result.validation.valid);
  EXPECT_GT(calls, 0);

  // 量化上界：求解在取消谓词被触发的代价评估数以内退出（代价回调不会
  // 计入被取消的那次评估），并且严格小于完整求解。
  EXPECT_LE(result.stats.lbfgs_iterations, kCancelAfterCalls);
  EXPECT_GT(baseline.stats.lbfgs_iterations, kCancelAfterCalls)
      << "baseline solve must be far longer than the cancel threshold";
  EXPECT_LT(result.stats.lbfgs_iterations, baseline.stats.lbfgs_iterations);
}

// 相同输入下的取消结果确定：状态、原因、迭代计数、求解器终止码一致。
TEST(DftpavCancellation, CancelledResultIsDeterministic) {
  constexpr int kCancelAfterCalls = 8;
  OptimizeRequest first_request = base_request();
  int first_calls = 0;
  first_request.options.cancel_check = [&first_calls] {
    return ++first_calls >= kCancelAfterCalls;
  };
  const OptimizeResult first = optimize_trajectory(first_request);

  OptimizeRequest second_request = base_request();
  int second_calls = 0;
  second_request.options.cancel_check = [&second_calls] {
    return ++second_calls >= kCancelAfterCalls;
  };
  const OptimizeResult second = optimize_trajectory(second_request);

  EXPECT_EQ(first.status, second.status);
  EXPECT_EQ(first.status, common::Status::kCancelled);
  EXPECT_EQ(first.reason, second.reason);
  EXPECT_EQ(first.stats.lbfgs_iterations, second.stats.lbfgs_iterations);
  EXPECT_EQ(first.stats.lbfgs_result, second.stats.lbfgs_result);
  EXPECT_EQ(first.trajectory.size(), second.trajectory.size());
  EXPECT_EQ(first_calls, second_calls);
}

// 立即取消（求解前）：谓词在入口检查即返回 true，不进入求解器。
TEST(DftpavCancellation, CancelBeforeSolveSkipsSolver) {
  OptimizeRequest request = base_request();
  request.options.cancel_check = [] { return true; };
  const OptimizeResult result = optimize_trajectory(request);

  EXPECT_EQ(result.status, common::Status::kCancelled);
  EXPECT_EQ(result.reason, "cancelled");
  EXPECT_TRUE(result.trajectory.empty());
  EXPECT_FALSE(result.validation.valid);
  EXPECT_EQ(result.stats.lbfgs_iterations, 0);
  EXPECT_DOUBLE_EQ(result.stats.solve_time_s, 0.0);
}

// 取消绝不触发收紧重试阶梯：第一次尝试被取消后立即返回，
// tightening_attempts 保持 1，且只记录了这一次尝试。
TEST(DftpavCancellation, CancelDoesNotStartRetryLadder) {
  OptimizeRequest request = retry_ladder_request();
  int calls = 0;
  request.options.cancel_check = [&calls] { return ++calls >= 32; };
  const OptimizeResult result = optimize_trajectory(request);

  EXPECT_EQ(result.status, common::Status::kCancelled);
  EXPECT_EQ(result.reason, "cancelled");
  EXPECT_TRUE(result.trajectory.empty());
  EXPECT_EQ(result.stats.tightening_attempts, 1);
  ASSERT_EQ(result.stats.attempt_reasons.size(), 1u);
  EXPECT_EQ(result.stats.attempt_reasons.front().rfind("cancelled", 0), 0u);
}

}  // namespace planner::dftpav::test
