// 端点容差检查与首次失败选择规则（最早的违规时间；完全并列时保留文档
// 中的检查顺序）。

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "planner/common/types.hpp"
#include "planner/validation/validation.hpp"
#include "validation_test_support.hpp"

namespace planner::validation {
namespace {

using test_support::base_request;

TEST(ValidationEndpoint, rejects_endpoint_position_beyond_tolerance) {
  ValidationRequest request = base_request();
  request.goal->x_m = 3.2;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kEndpointPosition);
  EXPECT_EQ(result.first_failure.point_index, 4);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 4.0);
  EXPECT_NEAR(result.first_failure.actual, 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(result.first_failure.limit, 0.05);
}

TEST(ValidationEndpoint, rejects_endpoint_heading_beyond_tolerance) {
  ValidationRequest request = base_request();
  request.goal->yaw_rad = 0.1;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kEndpointHeading);
  EXPECT_NEAR(result.first_failure.actual, 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(result.first_failure.limit, 0.02);
}

TEST(ValidationEndpoint, accepts_endpoint_exactly_at_tolerance) {
  ValidationRequest request = base_request();
  request.goal->x_m = 3.05;  // 0.05 m 位置误差
  request.goal->yaw_rad = 0.02;  // 0.02 rad 航向误差
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail << " constraint="
                            << to_string(result.first_failure.constraint);
}

TEST(ValidationEndpoint, accepts_wrapped_goal_heading) {
  // yaw 2*pi 与 0 的航向相同；归一化不能产生失败。
  ValidationRequest request = base_request();
  request.goal->yaw_rad = 2.0 * common::kPi;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail;
}

TEST(ValidationEndpoint, reports_earliest_violation_by_time) {
  ValidationRequest request = base_request();
  request.trajectory[1].velocity_mps = 3.0;  // 在 t = 1 s 处超速
  request.goal->x_m = 3.2;                   // 在 t = 4 s 处端点违规
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kSpeedLimit);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 1.0);
  EXPECT_NE(result.detail.find("speed"), std::string::npos);
}

TEST(ValidationEndpoint, equal_time_tie_keeps_documented_check_order) {
  // 速度限制（在碰撞之前检查）与碰撞都在 t = 0 失败；按文档顺序报告
  // 速度限制。
  ValidationRequest request = base_request();
  request.trajectory[0].velocity_mps = 3.0;
  request.scene->obstacles.push_back(
      collision::Obstacle::rectangle("start_block", -2.0, -2.0, 2.0, 2.0));
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kSpeedLimit);
  EXPECT_DOUBLE_EQ(result.first_failure.time_s, 0.0);
}

TEST(ValidationEndpoint, collision_before_later_velocity_violation_wins) {
  // t = 0.25 处采样点之间的碰撞必须优先于 t = 4 处的超速被报告
  // （最早时间规则，与检查顺序无关）。
  ValidationRequest request = base_request();
  request.trajectory = {
      test_support::point(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
      test_support::point(3.0, 6.0, 0.0, 0.0, 1.0, 0.0),
      test_support::point(4.0, 7.0, 0.0, 0.0, 3.0, 0.0),  // 超速
  };
  request.limits.max_speed_mps = 2.0;
  request.goal = common::Pose2d{7.0, 0.0, 0.0};
  request.corridor = test_support::make_corridor(
      {test_support::rectangle_polygon(0.0, 7.0, -2.0, -2.0, 11.0, 2.0)});
  request.scene->obstacles.push_back(
      collision::Obstacle::rectangle("thin_wall", 4.3, -2.0, 4.32, 2.0));
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCollision);
  EXPECT_GT(result.first_failure.time_s, 0.0);
  EXPECT_LT(result.first_failure.time_s, 3.0);
}

}  // namespace
}  // namespace planner::validation
