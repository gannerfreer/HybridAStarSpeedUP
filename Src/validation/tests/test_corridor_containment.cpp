// 走廊包含性：整个车辆足迹必须位于覆盖该采样点的走廊多边形内；仅有
// 中心线永远不够，s 覆盖出现缺口即为失败。

#include <gtest/gtest.h>

#include <string>

#include "planner/common/types.hpp"
#include "planner/validation/validation.hpp"
#include "validation_test_support.hpp"

namespace planner::validation {
namespace {

using test_support::base_request;
using test_support::make_corridor;
using test_support::point;
using test_support::rectangle_polygon;

TEST(ValidationCorridor, rejects_footprint_outside_corridor_despite_centerline) {
  ValidationRequest request = base_request();
  request.trajectory = {
      point(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
      point(1.0, 1.0, 0.0, 0.0, 1.0, 0.0),
  };
  request.goal = common::Pose2d{1.0, 0.0, 0.0};
  // 1.6 m x 1.0 m 的走廊：中心线 y = 0 在其内，2.0 m 宽的车辆包络
  // （|y| <= 1.0）不在。
  request.corridor = make_corridor(
      {rectangle_polygon(0.0, 1.0, -0.3, -0.5, 1.3, 0.5)});
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCorridorContainment);
  EXPECT_GT(result.first_failure.actual, 0.4);
  EXPECT_NE(result.first_failure.detail.find("footprint"), std::string::npos);
}

TEST(ValidationCorridor, rejects_coverage_gap_between_polygons) {
  ValidationRequest request = base_request();
  // 只覆盖 3 m 轨迹的前 1.5 m。
  request.corridor = make_corridor(
      {rectangle_polygon(0.0, 1.5, -2.0, -2.0, 6.0, 2.0)});
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCorridorContainment);
  EXPECT_NE(result.first_failure.detail.find("no corridor polygon covers"),
            std::string::npos);
  EXPECT_GT(result.first_failure.actual, 1.5);
  EXPECT_LT(result.first_failure.actual, 1.6);
}

TEST(ValidationCorridor, rejects_empty_corridor_for_non_empty_trajectory) {
  ValidationRequest request = base_request();
  request.corridor = make_corridor({});
  const ValidationResult result = validate_trajectory(request);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCorridorContainment);
  EXPECT_NE(result.first_failure.detail.find("no corridor polygon covers"),
            std::string::npos);
}

TEST(ValidationCorridor, accepts_footprint_inside_wide_corridor) {
  ValidationRequest request = base_request();
  request.corridor = make_corridor(
      {rectangle_polygon(0.0, 3.0, -2.0, -2.0, 7.0, 2.0)});
  const ValidationResult result = validate_trajectory(request);
  EXPECT_TRUE(result.valid) << result.detail;
}

TEST(ValidationCorridor, reports_earliest_containment_sample_time) {
  ValidationRequest request = base_request();
  request.corridor = make_corridor(
      {rectangle_polygon(0.0, 0.5, -2.0, -2.0, 6.0, 2.0)});
  const ValidationResult result = validate_trajectory(request);
  EXPECT_EQ(result.first_failure.constraint, Constraint::kCorridorContainment);
  // 第一个未覆盖的稠密采样点略晚于 s = 0.5 m，即位于匀速中段略晚于
  // t = 1 s 处。
  EXPECT_GT(result.first_failure.time_s, 1.0);
  EXPECT_LT(result.first_failure.time_s, 2.0);
}

}  // namespace
}  // namespace planner::validation
