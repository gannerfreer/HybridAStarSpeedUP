// 采样测试：弧长网格、尖点边界保持和精确端点
// （Docs/.../6.1 采样契约与 6.2 RS 行）。

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "planner/rs/reeds_shepp.hpp"
#include "rs_test_util.hpp"

namespace planner::rs {
namespace {

using test::benchmark_vehicle;
using test::deterministic_queries;
using test::pose;

const common::VehicleModel kVehicle = benchmark_vehicle();

std::vector<double> segment_boundaries(const RsPath& p) {
  std::vector<double> bounds;
  double s = 0.0;
  for (const RsSegment& segment : p.segments) {
    s += segment.length_m;
    bounds.push_back(s);
  }
  return bounds;
}

TEST(ReedsSheppSampling, starts_at_start_and_ends_at_goal) {
  for (const test::Query& query : deterministic_queries()) {
    const RsPath p = path(query.start, query.goal, kVehicle);
    const std::vector<common::PathPoint> points = sample(p, 0.25);
    ASSERT_FALSE(points.empty()) << query.id;
    EXPECT_DOUBLE_EQ(points.front().s_m, 0.0) << query.id;
    EXPECT_NEAR(points.front().pose.x_m, p.start.x_m, 1e-12) << query.id;
    EXPECT_NEAR(points.front().pose.y_m, p.start.y_m, 1e-12) << query.id;
    EXPECT_NEAR(points.front().pose.yaw_rad, p.start.yaw_rad, 1e-12) << query.id;
    EXPECT_DOUBLE_EQ(points.back().s_m, p.length_m) << query.id;
    EXPECT_NEAR(points.back().pose.x_m, p.goal.x_m, 1e-9) << query.id;
    EXPECT_NEAR(points.back().pose.y_m, p.goal.y_m, 1e-9) << query.id;
    EXPECT_NEAR(points.back().pose.yaw_rad, p.goal.yaw_rad, 1e-9) << query.id;
  }
}

TEST(ReedsSheppSampling, arc_length_is_non_decreasing_and_bounded) {
  for (const test::Query& query : deterministic_queries()) {
    const RsPath p = path(query.start, query.goal, kVehicle);
    const std::vector<common::PathPoint> points = sample(p, 0.5);
    for (std::size_t i = 0; i < points.size(); ++i) {
      EXPECT_GE(points[i].s_m, 0.0) << query.id;
      EXPECT_LE(points[i].s_m, p.length_m + 1e-12) << query.id;
      if (i > 0) {
        EXPECT_GE(points[i].s_m, points[i - 1].s_m) << query.id;
      }
    }
  }
}

TEST(ReedsSheppSampling, spacing_never_exceeds_ds) {
  for (const test::Query& query : deterministic_queries()) {
    const RsPath p = path(query.start, query.goal, kVehicle);
    const double ds = 0.3;
    const std::vector<common::PathPoint> points = sample(p, ds);
    for (std::size_t i = 1; i < points.size(); ++i) {
      EXPECT_LE(points[i].s_m - points[i - 1].s_m, ds + 1e-9) << query.id;
    }
  }
}

TEST(ReedsSheppSampling, cusp_boundaries_are_preserved) {
  const RsPath p = path(pose(0.0, 0.0, 0.0), pose(0.0, 0.0, common::kPi), kVehicle);
  ASSERT_GE(p.segments.size(), 2u);
  const std::vector<double> bounds = segment_boundaries(p);
  const std::vector<common::PathPoint> points = sample(p, 0.4);
  for (const double boundary : bounds) {
    if (boundary >= p.length_m - 1e-12) {
      continue;  // 末边界就是最后一个采样点
    }
    const auto it = std::find_if(points.begin(), points.end(),
                                 [boundary](const common::PathPoint& point) {
                                   return std::fabs(point.s_m - boundary) <= 1e-9;
                                 });
    ASSERT_NE(it, points.end()) << "missing cusp boundary at s=" << boundary;
  }
  // 尖点处的采样点属于从该处开始的分段。
  const std::size_t index = 0;  // 第一个内部边界开启分段 1
  const auto it = std::find_if(points.begin(), points.end(),
                               [&bounds, index](const common::PathPoint& point) {
                                 return std::fabs(point.s_m - bounds[index]) <= 1e-9;
                               });
  ASSERT_NE(it, points.end());
  EXPECT_EQ(it->direction, p.segments[index + 1].direction);
  EXPECT_DOUBLE_EQ(it->curvature_inv_m, p.segments[index + 1].curvature_inv_m);
}

TEST(ReedsSheppSampling, sample_at_s_matches_sampled_points) {
  for (const test::Query& query : deterministic_queries()) {
    const RsPath p = path(query.start, query.goal, kVehicle);
    if (p.length_m <= 0.0) {
      continue;
    }
    for (const common::PathPoint& point : sample(p, 0.5)) {
      const common::PathPoint exact = sample_at_s(p, point.s_m);
      EXPECT_NEAR(exact.pose.x_m, point.pose.x_m, 1e-12) << query.id;
      EXPECT_NEAR(exact.pose.y_m, point.pose.y_m, 1e-12) << query.id;
      EXPECT_NEAR(exact.pose.yaw_rad, point.pose.yaw_rad, 1e-12) << query.id;
      EXPECT_EQ(exact.direction, point.direction) << query.id;
      EXPECT_DOUBLE_EQ(exact.curvature_inv_m, point.curvature_inv_m) << query.id;
    }
  }
}

TEST(ReedsSheppSampling, sample_at_s_hits_exact_endpoints) {
  const RsPath p = path(pose(0.0, 0.0, 0.0), pose(6.0, 1.5, -0.4), kVehicle);
  const common::PathPoint first = sample_at_s(p, 0.0);
  EXPECT_DOUBLE_EQ(first.s_m, 0.0);
  EXPECT_DOUBLE_EQ(first.pose.x_m, p.start.x_m);
  EXPECT_DOUBLE_EQ(first.pose.y_m, p.start.y_m);
  EXPECT_DOUBLE_EQ(first.pose.yaw_rad, p.start.yaw_rad);
  const common::PathPoint last = sample_at_s(p, p.length_m);
  EXPECT_DOUBLE_EQ(last.s_m, p.length_m);
  EXPECT_NEAR(last.pose.x_m, p.goal.x_m, 1e-9);
  EXPECT_NEAR(last.pose.y_m, p.goal.y_m, 1e-9);
  EXPECT_NEAR(last.pose.yaw_rad, p.goal.yaw_rad, 1e-9);
}

TEST(ReedsSheppSampling, sample_rejects_bad_ds) {
  const RsPath p = path(pose(0.0, 0.0, 0.0), pose(3.0, 1.0, 0.2), kVehicle);
  EXPECT_THROW(sample(p, 0.0), std::invalid_argument);
  EXPECT_THROW(sample(p, -0.5), std::invalid_argument);
  EXPECT_THROW(sample(p, std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(sample(p, std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

TEST(ReedsSheppSampling, sample_at_s_rejects_out_of_range) {
  const RsPath p = path(pose(0.0, 0.0, 0.0), pose(3.0, 1.0, 0.2), kVehicle);
  EXPECT_THROW(sample_at_s(p, -0.5), std::out_of_range);
  EXPECT_THROW(sample_at_s(p, p.length_m + 0.5), std::out_of_range);
  EXPECT_THROW(sample_at_s(p, std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

TEST(ReedsSheppSampling, zero_length_path_returns_single_start_point) {
  const RsPath p = path(pose(2.0, -1.0, 0.3), pose(2.0, -1.0, 0.3), kVehicle);
  ASSERT_TRUE(p.segments.empty());
  const std::vector<common::PathPoint> points = sample(p, 0.5);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_DOUBLE_EQ(points[0].s_m, 0.0);
  EXPECT_DOUBLE_EQ(points[0].pose.x_m, p.start.x_m);
  EXPECT_DOUBLE_EQ(points[0].pose.y_m, p.start.y_m);
  EXPECT_DOUBLE_EQ(points[0].pose.yaw_rad, p.start.yaw_rad);
  EXPECT_EQ(points[0].direction, common::Direction::kForward);
  EXPECT_DOUBLE_EQ(points[0].curvature_inv_m, 0.0);
  const common::PathPoint exact = sample_at_s(p, 0.0);
  EXPECT_DOUBLE_EQ(exact.pose.x_m, p.start.x_m);
  EXPECT_THROW(sample_at_s(p, 0.1), std::out_of_range);
}

TEST(ReedsSheppSampling, curvature_stays_within_vehicle_limit) {
  const double limit = 1.0 / min_turn_radius_m(kVehicle);
  for (const test::Query& query : deterministic_queries()) {
    const RsPath p = path(query.start, query.goal, kVehicle);
    for (const common::PathPoint& point : sample(p, 0.5)) {
      EXPECT_LE(std::fabs(point.curvature_inv_m), limit + 1e-12) << query.id;
    }
  }
}

TEST(ReedsSheppSampling, rejects_malformed_path) {
  RsPath p;
  p.length_m = 10.0;
  p.segments.push_back({common::Direction::kForward, 1.0, 0.0});
  EXPECT_THROW(sample(p, 0.5), std::invalid_argument);
  RsPath negative;
  negative.length_m = 1.0;
  negative.segments.push_back({common::Direction::kForward, -1.0, 0.0});
  EXPECT_THROW(sample(negative, 0.5), std::invalid_argument);
  RsPath nonfinite;
  nonfinite.length_m = 1.0;
  nonfinite.segments.push_back({common::Direction::kForward, 1.0,
                                std::numeric_limits<double>::quiet_NaN()});
  EXPECT_THROW(sample(nonfinite, 0.5), std::invalid_argument);
}

TEST(ReedsSheppSampling, rejects_ds_that_would_exceed_sample_limit) {
  RsPath p;
  p.length_m = 1.0e9;
  p.segments.push_back({common::Direction::kForward, 1.0e9, 0.0});
  EXPECT_THROW(sample(p, 1e-3), std::invalid_argument);
}

}  // namespace
}  // namespace planner::rs
