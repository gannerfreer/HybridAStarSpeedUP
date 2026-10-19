// 路径结构测试：分段、尖点边界、曲率限制，以及通过积分所报告的分段字段
// 进行的终点重构。

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "planner/rs/reeds_shepp.hpp"
#include "rs_test_util.hpp"

namespace planner::rs {
namespace {

using test::benchmark_vehicle;
using test::deterministic_queries;
using test::endpoint_position_error;
using test::endpoint_yaw_error;
using test::pose;

const common::VehicleModel kVehicle = benchmark_vehicle();

TEST(ReedsSheppPath, segments_tile_total_length) {
  for (const test::Query& query : deterministic_queries()) {
    const RsPath p = path(query.start, query.goal, kVehicle);
    double sum = 0.0;
    for (const RsSegment& segment : p.segments) {
      EXPECT_GT(segment.length_m, 0.0) << query.id;
      EXPECT_TRUE(std::isfinite(segment.length_m)) << query.id;
      sum += segment.length_m;
    }
    EXPECT_NEAR(p.length_m, sum, 1e-9) << query.id;
  }
}

TEST(ReedsSheppPath, endpoint_reconstruction_matches_goal) {
  for (const test::Query& query : deterministic_queries()) {
    const RsPath p = path(query.start, query.goal, kVehicle);
    EXPECT_LE(endpoint_position_error(p), kEndpointPositionToleranceM) << query.id;
    EXPECT_LE(endpoint_yaw_error(p), kEndpointYawToleranceRad) << query.id;
  }
}

TEST(ReedsSheppPath, goal_behind_uses_reverse_segment) {
  const RsPath p = path(pose(0.0, 0.0, 0.0), pose(-5.0, 0.0, 0.0), kVehicle);
  ASSERT_EQ(p.segments.size(), 1u);
  EXPECT_EQ(p.segments[0].direction, common::Direction::kReverse);
  EXPECT_DOUBLE_EQ(p.segments[0].curvature_inv_m, 0.0);
  EXPECT_NEAR(p.segments[0].length_m, 5.0, 1e-9);
}

TEST(ReedsSheppPath, cusp_case_contains_direction_changes) {
  const RsPath p = path(pose(0.0, 0.0, 0.0), pose(0.0, 0.0, common::kPi), kVehicle);
  ASSERT_GE(p.segments.size(), 2u);
  int cusps = 0;
  for (std::size_t i = 1; i < p.segments.size(); ++i) {
    if (p.segments[i].direction != p.segments[i - 1].direction) {
      ++cusps;
    }
  }
  EXPECT_GE(cusps, 1);
  EXPECT_NEAR(p.length_m, min_turn_radius_m(kVehicle) * common::kPi, 1e-9);
  EXPECT_LE(endpoint_position_error(p), kEndpointPositionToleranceM);
  EXPECT_LE(endpoint_yaw_error(p), kEndpointYawToleranceRad);
}

TEST(ReedsSheppPath, adjacent_segments_are_never_equal_strokes) {
  // build_path 会合并挡位与曲率相同的原始段，因此每个相邻对要么是尖点，
  // 要么是曲率变化。
  for (const test::Query& query : deterministic_queries()) {
    const RsPath p = path(query.start, query.goal, kVehicle);
    for (std::size_t i = 1; i < p.segments.size(); ++i) {
      const bool same_stroke =
          p.segments[i].direction == p.segments[i - 1].direction &&
          p.segments[i].curvature_inv_m == p.segments[i - 1].curvature_inv_m;
      EXPECT_FALSE(same_stroke) << query.id << " segment " << i;
    }
  }
}

TEST(ReedsSheppPath, curvature_never_exceeds_min_turn_radius) {
  const double limit = 1.0 / min_turn_radius_m(kVehicle);
  for (const test::Query& query : deterministic_queries()) {
    const RsPath p = path(query.start, query.goal, kVehicle);
    for (const RsSegment& segment : p.segments) {
      EXPECT_LE(std::fabs(segment.curvature_inv_m), limit + 1e-12) << query.id;
      if (segment.curvature_inv_m != 0.0) {
        EXPECT_NEAR(std::fabs(segment.curvature_inv_m), limit, 1e-12) << query.id;
      }
    }
  }
}

TEST(ReedsSheppPath, straight_segment_is_forward_with_zero_curvature) {
  const RsPath p = path(pose(0.0, 0.0, 0.0), pose(12.0, 0.0, 0.0), kVehicle);
  ASSERT_EQ(p.segments.size(), 1u);
  EXPECT_EQ(p.segments[0].direction, common::Direction::kForward);
  EXPECT_DOUBLE_EQ(p.segments[0].curvature_inv_m, 0.0);
  EXPECT_NEAR(p.segments[0].length_m, 12.0, 1e-9);
}

TEST(ReedsSheppPath, quarter_turn_curvature_is_exact_limit) {
  const double radius = min_turn_radius_m(kVehicle);
  const RsPath p =
      path(pose(0.0, 0.0, 0.0), pose(radius, radius, 0.5 * common::kPi), kVehicle);
  ASSERT_EQ(p.segments.size(), 1u);
  EXPECT_EQ(p.segments[0].direction, common::Direction::kForward);
  EXPECT_DOUBLE_EQ(p.segments[0].curvature_inv_m, 1.0 / radius);
  EXPECT_NEAR(p.segments[0].length_m, radius * 0.5 * common::kPi, 1e-9);
}

TEST(ReedsSheppPath, plan_normalizes_returned_angles) {
  const double start_yaw = 350.0 * common::kPi / 180.0;
  const RsPlan result = plan(pose(0.0, 0.0, start_yaw), pose(2.0, 1.0, 6.5), kVehicle);
  ASSERT_EQ(result.status, common::Status::kSuccess) << result.reason;
  // 350 deg 归一化为 -10 deg，误差至多一个 ulp。
  EXPECT_NEAR(result.path.start.yaw_rad, -10.0 * common::kPi / 180.0, 1e-15);
  EXPECT_DOUBLE_EQ(result.path.goal.yaw_rad, common::normalize_angle_rad(6.5));
  EXPECT_LE(endpoint_position_error(result.path), kEndpointPositionToleranceM);
  EXPECT_LE(endpoint_yaw_error(result.path), kEndpointYawToleranceRad);
}

TEST(ReedsSheppPath, plan_reports_invalid_vehicle_without_throwing) {
  common::VehicleModel broken = kVehicle;
  broken.wheelbase_m = 0.0;
  const RsPlan result = plan(pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.0), broken);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_FALSE(result.reason.empty());
}

TEST(ReedsSheppPath, integrate_segment_straight_and_arc) {
  RsSegment straight;
  straight.direction = common::Direction::kForward;
  straight.length_m = 2.0;
  straight.curvature_inv_m = 0.0;
  const common::Pose2d moved = integrate_segment(pose(0.0, 0.0, 0.25), straight, 2.0);
  EXPECT_NEAR(moved.x_m, 2.0 * std::cos(0.25), 1e-12);
  EXPECT_NEAR(moved.y_m, 2.0 * std::sin(0.25), 1e-12);
  EXPECT_DOUBLE_EQ(moved.yaw_rad, 0.25);

  const double radius = min_turn_radius_m(kVehicle);
  RsSegment arc;
  arc.direction = common::Direction::kForward;
  arc.length_m = radius * 0.5 * common::kPi;
  arc.curvature_inv_m = 1.0 / radius;
  const common::Pose2d turned = integrate_segment(pose(0.0, 0.0, 0.0), arc, arc.length_m);
  EXPECT_NEAR(turned.x_m, radius, 1e-9);
  EXPECT_NEAR(turned.y_m, radius, 1e-9);
  EXPECT_NEAR(turned.yaw_rad, 0.5 * common::kPi, 1e-15);
}

TEST(ReedsSheppPath, integrate_segment_rejects_overshoot) {
  RsSegment straight;
  straight.length_m = 1.0;
  straight.curvature_inv_m = 0.0;
  EXPECT_THROW(integrate_segment(pose(0.0, 0.0, 0.0), straight, 1.5),
               std::invalid_argument);
  EXPECT_THROW(integrate_segment(pose(0.0, 0.0, 0.0), straight, -0.1),
               std::invalid_argument);
}

}  // namespace
}  // namespace planner::rs
