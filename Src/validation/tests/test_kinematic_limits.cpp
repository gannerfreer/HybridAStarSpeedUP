// 速度/加速度/减速度/曲率限制检查及其数值容差
// 1e-6 * max(1, limit)。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "planner/common/types.hpp"
#include "planner/validation/validation.hpp"
#include "validation_test_support.hpp"

namespace planner::validation {
namespace {

using test_support::base_request;

TEST(ValidationLimits, rejects_speed_above_limit) {
  ValidationRequest request = base_request();
  request.trajectory[2].velocity_mps = 3.0;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kSpeedLimit);
  EXPECT_EQ(result.first_failure.point_index, 2);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 2.0);
  EXPECT_DOUBLE_EQ(result.first_failure.actual, 3.0);
  EXPECT_DOUBLE_EQ(result.first_failure.limit, 2.0);
}

TEST(ValidationLimits, accepts_speed_at_relative_tolerance_boundary) {
  ValidationRequest request = base_request();
  const double limit = request.limits.max_speed_mps;
  request.trajectory[2].velocity_mps = limit + 1e-6 * limit;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail << " constraint="
                            << to_string(result.first_failure.constraint);
}

TEST(ValidationLimits, rejects_speed_above_relative_tolerance) {
  ValidationRequest request = base_request();
  request.trajectory[2].velocity_mps = 2.0001;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kSpeedLimit);
  EXPECT_DOUBLE_EQ(result.first_failure.actual, 2.0001);
  EXPECT_DOUBLE_EQ(result.first_failure.limit, 2.0);
}

TEST(ValidationLimits, rejects_acceleration_above_limit) {
  ValidationRequest request = base_request();
  request.trajectory[1].acceleration_mps2 = 2.5;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kAccelerationLimit);
  EXPECT_EQ(result.first_failure.point_index, 1);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 1.0);
  EXPECT_DOUBLE_EQ(result.first_failure.actual, 2.5);
  EXPECT_DOUBLE_EQ(result.first_failure.limit, 2.0);
}

TEST(ValidationLimits, accepts_acceleration_at_relative_tolerance_boundary) {
  ValidationRequest request = base_request();
  const double limit = request.limits.max_acceleration_mps2;
  request.trajectory[1].acceleration_mps2 = limit + 1e-6 * limit;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail;
}

TEST(ValidationLimits, rejects_deceleration_below_limit) {
  ValidationRequest request = base_request();
  request.trajectory[3].acceleration_mps2 = -2.5;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kDecelerationLimit);
  EXPECT_EQ(result.first_failure.point_index, 3);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 3.0);
  EXPECT_DOUBLE_EQ(result.first_failure.actual, -2.5);
  EXPECT_DOUBLE_EQ(result.first_failure.limit, -2.0);
}

TEST(ValidationLimits, accepts_deceleration_at_relative_tolerance_boundary) {
  ValidationRequest request = base_request();
  const double limit = request.limits.max_deceleration_mps2;
  request.trajectory[3].acceleration_mps2 = -(limit + 1e-6 * limit);
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail;
}

TEST(ValidationLimits, rejects_curvature_above_limit) {
  ValidationRequest request = base_request();
  request.trajectory[1].curvature_inv_m = 0.5;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCurvatureLimit);
  EXPECT_EQ(result.first_failure.point_index, 1);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 1.0);
  EXPECT_DOUBLE_EQ(result.first_failure.actual, 0.5);
  EXPECT_DOUBLE_EQ(result.first_failure.limit, 0.2);
}

TEST(ValidationLimits, accepts_curvature_at_relative_tolerance_boundary) {
  ValidationRequest request = base_request();
  const double limit = request.limits.max_curvature_inv_m;
  request.trajectory[1].curvature_inv_m = limit + 1e-6 * std::max(1.0, limit);
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail;
}

TEST(ValidationLimits, rejects_negative_reverse_speed_above_limit) {
  ValidationRequest request = base_request();
  request.trajectory[2].velocity_mps = -2.5;  // |v| 超过 2.0 限制
  // 保持换挡有效：点 1 减速经过零。
  request.trajectory[1].velocity_mps = 0.0;
  request.trajectory[3].velocity_mps = -1.0;
  request.trajectory[4].velocity_mps = 0.0;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kSpeedLimit);
  EXPECT_DOUBLE_EQ(result.first_failure.actual, 2.5);
}

}  // namespace
}  // namespace planner::validation
