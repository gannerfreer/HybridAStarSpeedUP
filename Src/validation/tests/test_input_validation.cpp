// 缺失或不可用的必需输入必须返回 Status::kInvalidInput，并带有稳定
// 原因码和 Constraint::kInput；不得运行任何轨迹检查。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "planner/common/types.hpp"
#include "planner/validation/validation.hpp"
#include "validation_test_support.hpp"

namespace planner::validation {
namespace {

using test_support::base_request;
using test_support::free_grid_scene;
using test_support::make_corridor;
using test_support::rectangle_polygon;

void expect_invalid_input(const ValidationResult& result,
                          const std::string& reason) {
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_EQ(result.reason, reason) << result.detail;
  EXPECT_EQ(result.first_failure.constraint, Constraint::kInput);
  EXPECT_TRUE(std::isnan(result.first_failure.time_s));
}

TEST(ValidationInput, missing_trajectory_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.trajectory.clear();
  expect_invalid_input(validate_trajectory(request), "missing_trajectory");
}

TEST(ValidationInput, missing_goal_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.goal.reset();
  expect_invalid_input(validate_trajectory(request), "missing_goal");
}

TEST(ValidationInput, missing_scene_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.scene.reset();
  expect_invalid_input(validate_trajectory(request), "missing_scene");
}

TEST(ValidationInput, missing_corridor_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.corridor.reset();
  expect_invalid_input(validate_trajectory(request), "missing_corridor");
}

TEST(ValidationInput, invalid_vehicle_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.vehicle.wheelbase_m = 0.0;
  expect_invalid_input(validate_trajectory(request), "invalid_vehicle");
}

TEST(ValidationInput, non_positive_speed_limit_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.limits.max_speed_mps = 0.0;
  expect_invalid_input(validate_trajectory(request), "invalid_limits");
}

TEST(ValidationInput, non_finite_curvature_limit_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.limits.max_curvature_inv_m =
      std::numeric_limits<double>::quiet_NaN();
  expect_invalid_input(validate_trajectory(request), "invalid_limits");
}

TEST(ValidationInput, non_finite_goal_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.goal->yaw_rad = std::numeric_limits<double>::infinity();
  expect_invalid_input(validate_trajectory(request), "invalid_goal");
}

TEST(ValidationInput, malformed_grid_scene_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.scene = free_grid_scene(5, 0, 0.1, 0.0, 0.0);
  expect_invalid_input(validate_trajectory(request), "invalid_scene");
}

TEST(ValidationInput, clockwise_corridor_polygon_returns_invalid_input) {
  ValidationRequest request = base_request();
  common::CorridorPolygon polygon;
  polygon.s_start_m = 0.0;
  polygon.s_end_m = 3.0;
  polygon.vertices = {common::Point2d{-2.0, -2.0}, common::Point2d{-2.0, 2.0},
                      common::Point2d{6.0, 2.0}, common::Point2d{6.0, -2.0}};
  request.corridor = make_corridor({polygon});
  expect_invalid_input(validate_trajectory(request), "invalid_corridor");
}

TEST(ValidationInput, non_convex_corridor_polygon_returns_invalid_input) {
  ValidationRequest request = base_request();
  common::CorridorPolygon polygon;
  polygon.s_start_m = 0.0;
  polygon.s_end_m = 3.0;
  polygon.vertices = {common::Point2d{0.0, 0.0}, common::Point2d{4.0, 0.0},
                      common::Point2d{4.0, 1.0}, common::Point2d{2.0, 1.0},
                      common::Point2d{2.0, 4.0}, common::Point2d{0.0, 4.0}};
  request.corridor = make_corridor({polygon});
  expect_invalid_input(validate_trajectory(request), "invalid_corridor");
}

TEST(ValidationInput, empty_corridor_interval_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.corridor = make_corridor(
      {rectangle_polygon(1.0, 1.0, -2.0, -2.0, 6.0, 2.0)});
  expect_invalid_input(validate_trajectory(request), "invalid_corridor");
}

TEST(ValidationInput, corridor_map_mismatch_returns_invalid_input) {
  ValidationRequest request = base_request();
  request.scene = free_grid_scene(4, 4, 1.0, -2.0, -2.0, "other_map", "v1");
  request.corridor = make_corridor(
      {rectangle_polygon(0.0, 3.0, -2.0, -2.0, 6.0, 2.0)}, "test_map", "v1");
  expect_invalid_input(validate_trajectory(request), "map_mismatch");
}

TEST(ValidationInput, invalid_options_return_invalid_input) {
  ValidationRequest request = base_request();
  ValidationOptions options;
  options.corridor_sample_step_m = 0.0;
  expect_invalid_input(validate_trajectory(request, options),
                       "invalid_options");
}

TEST(ValidationInput, input_failure_returns_status_invalid_input_not_validation_failed) {
  ValidationRequest request = base_request();
  request.limits.max_acceleration_mps2 = -1.0;
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.status, common::Status::kValidationFailed);
}

}  // namespace
}  // namespace planner::validation
