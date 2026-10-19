// 时间维检查：存储值有限、时间严格递增，以及换向时速度为零。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "planner/common/types.hpp"
#include "planner/validation/validation.hpp"
#include "validation_test_support.hpp"

namespace planner::validation {
namespace {

using test_support::base_request;

TEST(ValidationTemporal, rejects_nan_velocity) {
  ValidationRequest request = base_request();
  request.trajectory[1].velocity_mps = std::numeric_limits<double>::quiet_NaN();
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, common::Status::kValidationFailed);
  EXPECT_EQ(result.reason, "finite_values");
  EXPECT_EQ(result.first_failure.constraint, Constraint::kFiniteValues);
  EXPECT_EQ(result.first_failure.point_index, 1);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 1.0);
  EXPECT_NE(result.first_failure.detail.find("velocity_mps"), std::string::npos);
}

TEST(ValidationTemporal, rejects_nan_time_without_fabricating_a_time) {
  ValidationRequest request = base_request();
  request.trajectory[1].t_s = std::numeric_limits<double>::quiet_NaN();
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kFiniteValues);
  EXPECT_EQ(result.first_failure.point_index, 1);
  EXPECT_TRUE(std::isnan(result.first_failure.time_s));
}

TEST(ValidationTemporal, rejects_infinite_curvature) {
  ValidationRequest request = base_request();
  request.trajectory[3].curvature_inv_m =
      std::numeric_limits<double>::infinity();
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kFiniteValues);
  EXPECT_EQ(result.first_failure.point_index, 3);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 3.0);
  EXPECT_NE(result.first_failure.detail.find("curvature_inv_m"),
            std::string::npos);
}

TEST(ValidationTemporal, rejects_equal_times) {
  ValidationRequest request = base_request();
  request.trajectory[2].t_s = request.trajectory[1].t_s;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kTimeMonotonicity);
  EXPECT_EQ(result.first_failure.point_index, 2);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 1.0);
  EXPECT_DOUBLE_EQ(result.first_failure.actual, 1.0);
  EXPECT_DOUBLE_EQ(result.first_failure.limit, 1.0);
}

TEST(ValidationTemporal, rejects_decreasing_times) {
  ValidationRequest request = base_request();
  request.trajectory[2].t_s = 0.5;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kTimeMonotonicity);
  EXPECT_EQ(result.first_failure.point_index, 2);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 0.5);
}

TEST(ValidationTemporal, rejects_direction_change_without_stop) {
  ValidationRequest request = base_request();
  request.trajectory[2].velocity_mps = -1.0;  // 在 t=1,2 之间 +1.0 -> -1.0
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kDirectionChangeStop);
  EXPECT_EQ(result.first_failure.point_index, 2);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 2.0);
  EXPECT_DOUBLE_EQ(result.first_failure.actual, 1.0);
  EXPECT_DOUBLE_EQ(result.first_failure.limit, 1e-3);
}

TEST(ValidationTemporal, accepts_direction_change_at_zero_speed_tolerance) {
  ValidationRequest request = base_request();
  // |v| = 1e-3 恰好等于容差：换挡被接受。
  request.trajectory[2].velocity_mps = -1e-3;
  request.trajectory[3].velocity_mps = -1.0;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail << " constraint="
                            << to_string(result.first_failure.constraint);
}

TEST(ValidationTemporal, rejects_direction_change_above_zero_speed_tolerance) {
  ValidationRequest request = base_request();
  request.trajectory[2].velocity_mps = -1.1e-3;
  request.trajectory[3].velocity_mps = -1.0;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kDirectionChangeStop);
  EXPECT_NEAR(result.first_failure.actual, 1.1e-3, 1e-12);
  EXPECT_DOUBLE_EQ(result.first_failure.limit, 1e-3);
}

TEST(ValidationTemporal, rejects_strictly_opposite_signs_even_with_near_zero_end) {
  ValidationRequest request = base_request();
  // +1.0 -> -2e-3：新方向采样点位于零速带之外。
  request.trajectory[2].velocity_mps = -2e-3;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kDirectionChangeStop);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 2.0);
}

}  // namespace
}  // namespace planner::validation
