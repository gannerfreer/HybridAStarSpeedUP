// 二维轨迹分段的单元测试：边界条件、局部定义域约束、
// 预分配缓冲区的批处理采样和时间缩放。
// 数值容差遵循第 7.1 节（绝对/相对 1e-6）。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>

#include "planner/common/types.hpp"
#include "planner/polynomial/result.hpp"
#include "planner/polynomial/trajectory_segment.hpp"

namespace planner::polynomial {
namespace {

using planner::common::Point2d;

constexpr double kAbsTol = 1e-6;
constexpr double kRelTol = 1e-6;

void expect_point_close(const Point2d& actual, const Point2d& expected) {
  EXPECT_NEAR(actual.x_m, expected.x_m, kAbsTol + kRelTol * std::fabs(expected.x_m));
  EXPECT_NEAR(actual.y_m, expected.y_m, kAbsTol + kRelTol * std::fabs(expected.y_m));
}

CubicBoundary2d cubic_boundary() {
  return CubicBoundary2d{Point2d{1.0, -2.0}, Point2d{0.5, 1.5}, Point2d{4.0, 2.5},
                         Point2d{0.25, -0.5}};
}

QuinticBoundary2d quintic_boundary() {
  return QuinticBoundary2d{Point2d{0.0, 3.0},  Point2d{1.0, 0.0},  Point2d{0.5, -0.25},
                           Point2d{10.0, -1.0}, Point2d{2.0, -1.0}, Point2d{-0.5, 0.25}};
}

TEST(TrajectorySegmentCubic, satisfies_position_and_velocity_boundaries) {
  const CubicBoundary2d boundary = cubic_boundary();
  const Result<TrajectorySegment> solved = TrajectorySegment::cubic(boundary, 2.0);
  ASSERT_TRUE(solved.ok());
  const TrajectorySegment& segment = solved.value;
  EXPECT_EQ(segment.degree(), 3);
  EXPECT_DOUBLE_EQ(segment.duration_s(), 2.0);
  expect_point_close(segment.evaluate(0.0, 0).value, boundary.p0_m);
  expect_point_close(segment.evaluate(0.0, 1).value, boundary.v0_mps);
  expect_point_close(segment.evaluate(2.0, 0).value, boundary.p1_m);
  expect_point_close(segment.evaluate(2.0, 1).value, boundary.v1_mps);
}

TEST(TrajectorySegmentQuintic, satisfies_position_velocity_acceleration_boundaries) {
  const QuinticBoundary2d boundary = quintic_boundary();
  const Result<TrajectorySegment> solved = TrajectorySegment::quintic(boundary, 3.5);
  ASSERT_TRUE(solved.ok());
  const TrajectorySegment& segment = solved.value;
  EXPECT_EQ(segment.degree(), 5);
  expect_point_close(segment.evaluate(0.0, 0).value, boundary.p0_m);
  expect_point_close(segment.evaluate(0.0, 1).value, boundary.v0_mps);
  expect_point_close(segment.evaluate(0.0, 2).value, boundary.a0_mps2);
  expect_point_close(segment.evaluate(3.5, 0).value, boundary.p1_m);
  expect_point_close(segment.evaluate(3.5, 1).value, boundary.v1_mps);
  expect_point_close(segment.evaluate(3.5, 2).value, boundary.a1_mps2);
}

TEST(TrajectorySegmentQuintic, boundary_residuals_stay_within_abs_rel_tolerance) {
  // 一组确定性边界值，均为 0.25 的倍数。
  for (int k = 0; k < 16; ++k) {
    const double kd = static_cast<double>(k);
    const QuinticBoundary2d boundary{
        Point2d{0.5 * kd, 0.25 * static_cast<double>(k % 8 - 4)},
        Point2d{0.25 * static_cast<double>(k % 8 - 4), 0.5},
        Point2d{0.5, 0.0},
        Point2d{0.5 * kd + 2.0, 0.25 * static_cast<double>(k % 8 - 2)},
        Point2d{0.5, 0.25 * static_cast<double>(k % 4 - 2)},
        Point2d{-0.5, 0.0}};
    const Result<TrajectorySegment> solved = TrajectorySegment::quintic(boundary, 1.0);
    ASSERT_TRUE(solved.ok()) << "boundary " << k;
    for (int order = 0; order <= 2; ++order) {
      const Point2d expected_start = order == 0   ? boundary.p0_m
                                     : order == 1 ? boundary.v0_mps
                                                  : boundary.a0_mps2;
      const Point2d expected_end = order == 0   ? boundary.p1_m
                                   : order == 1 ? boundary.v1_mps
                                                : boundary.a1_mps2;
      expect_point_close(solved.value.evaluate(0.0, order).value, expected_start);
      expect_point_close(solved.value.evaluate(1.0, order).value, expected_end);
    }
  }
}

TEST(TrajectorySegmentDomain, rejects_times_outside_local_domain) {
  const Result<TrajectorySegment> solved = TrajectorySegment::cubic(cubic_boundary(), 2.0);
  ASSERT_TRUE(solved.ok());
  const TrajectorySegment& segment = solved.value;
  EXPECT_EQ(segment.evaluate(-1e-9, 0).code, Code::kOutOfRange);
  EXPECT_EQ(segment.evaluate(2.0 + 1e-9, 0).code, Code::kOutOfRange);
  EXPECT_TRUE(segment.evaluate(0.0, 0).ok());
  EXPECT_TRUE(segment.evaluate(2.0, 0).ok());
}

TEST(TrajectorySegmentDomain, rejects_non_finite_time_and_bad_order) {
  const Result<TrajectorySegment> solved = TrajectorySegment::cubic(cubic_boundary(), 2.0);
  ASSERT_TRUE(solved.ok());
  const TrajectorySegment& segment = solved.value;
  EXPECT_EQ(segment.evaluate(std::nan(""), 0).code, Code::kInvalidInput);
  EXPECT_EQ(segment.evaluate(1.0, -1).code, Code::kInvalidInput);
  EXPECT_EQ(segment.evaluate(1.0, 4).code, Code::kUnsupportedDerivative);
}

TEST(TrajectorySegmentBatch, matches_single_evaluation) {
  const Result<TrajectorySegment> solved = TrajectorySegment::quintic(quintic_boundary(), 2.0);
  ASSERT_TRUE(solved.ok());
  const TrajectorySegment& segment = solved.value;
  const double local_times[] = {0.0, 0.05, 0.5, 1.0, 1.5, 2.0};
  Point2d output[6];
  const BatchResult result = segment.evaluate_batch(local_times, 6, 1, output);
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.processed, 6u);
  for (std::size_t i = 0; i < 6; ++i) {
    expect_point_close(output[i], segment.evaluate(local_times[i], 1).value);
  }
}

TEST(TrajectorySegmentBatch, stops_at_first_out_of_range_time) {
  const Result<TrajectorySegment> solved = TrajectorySegment::cubic(cubic_boundary(), 1.0);
  ASSERT_TRUE(solved.ok());
  const TrajectorySegment& segment = solved.value;
  const double local_times[] = {0.25, 0.5, 1.25, 0.75};
  Point2d output[4];
  const BatchResult result = segment.evaluate_batch(local_times, 4, 0, output);
  EXPECT_EQ(result.code, Code::kOutOfRange);
  EXPECT_EQ(result.processed, 2u);
  EXPECT_EQ(result.first_bad_index, 2u);
  expect_point_close(output[0], segment.evaluate(0.25, 0).value);
  expect_point_close(output[1], segment.evaluate(0.5, 0).value);
}

TEST(TrajectorySegmentBatch, rejects_null_buffers_and_accepts_empty_count) {
  const Result<TrajectorySegment> solved = TrajectorySegment::cubic(cubic_boundary(), 1.0);
  ASSERT_TRUE(solved.ok());
  const TrajectorySegment& segment = solved.value;
  const double time = 0.5;
  Point2d output;
  EXPECT_EQ(segment.evaluate_batch(nullptr, 1, 0, &output).code, Code::kInvalidInput);
  EXPECT_EQ(segment.evaluate_batch(&time, 1, 0, nullptr).code, Code::kInvalidInput);
  EXPECT_TRUE(segment.evaluate_batch(nullptr, 0, 0, nullptr).ok());
}

TEST(TrajectorySegmentTimeScaling, preserves_geometry_and_scales_derivatives) {
  const QuinticBoundary2d boundary = quintic_boundary();
  const Result<TrajectorySegment> solved = TrajectorySegment::quintic(boundary, 2.0);
  ASSERT_TRUE(solved.ok());
  const Result<TrajectorySegment> scaled = solved.value.scaled_time(3.0);
  ASSERT_TRUE(scaled.ok());
  EXPECT_DOUBLE_EQ(scaled.value.duration_s(), 6.0);
  const double local_times[] = {0.0, 1.5, 4.0, 6.0};
  for (double t : local_times) {
    expect_point_close(scaled.value.evaluate(t, 0).value, solved.value.evaluate(t / 3.0, 0).value);
    const Point2d velocity = solved.value.evaluate(t / 3.0, 1).value;
    expect_point_close(scaled.value.evaluate(t, 1).value, Point2d{velocity.x_m / 3.0, velocity.y_m / 3.0});
  }
}

TEST(TrajectorySegmentTimeScaling, rejects_invalid_scale) {
  const Result<TrajectorySegment> solved = TrajectorySegment::cubic(cubic_boundary(), 1.0);
  ASSERT_TRUE(solved.ok());
  EXPECT_EQ(solved.value.scaled_time(0.0).code, Code::kInvalidInput);
  EXPECT_EQ(solved.value.scaled_time(-2.0).code, Code::kInvalidInput);
  EXPECT_EQ(solved.value.scaled_time(std::nan("")).code, Code::kInvalidInput);
}

TEST(TrajectorySegmentFactories, reject_invalid_duration_and_non_finite_boundaries) {
  const double nan = std::nan("");
  const CubicBoundary2d cubic = cubic_boundary();
  EXPECT_EQ(TrajectorySegment::cubic(cubic, 0.0).code, Code::kInvalidInput);
  EXPECT_EQ(TrajectorySegment::cubic(cubic, -1.0).code, Code::kInvalidInput);
  EXPECT_EQ(TrajectorySegment::cubic(cubic, std::nan("")).code, Code::kInvalidInput);
  CubicBoundary2d bad_cubic = cubic;
  bad_cubic.p1_m.x_m = nan;
  EXPECT_EQ(TrajectorySegment::cubic(bad_cubic, 1.0).code, Code::kInvalidInput);

  const QuinticBoundary2d quintic = quintic_boundary();
  EXPECT_EQ(TrajectorySegment::quintic(quintic, 0.0).code, Code::kInvalidInput);
  QuinticBoundary2d bad_quintic = quintic;
  bad_quintic.a0_mps2.y_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(TrajectorySegment::quintic(bad_quintic, 1.0).code, Code::kInvalidInput);
}

TEST(TrajectorySegmentDefault, empty_segment_is_invalid_and_rejects_evaluation) {
  const TrajectorySegment empty;
  EXPECT_FALSE(empty.valid());
  EXPECT_EQ(empty.degree(), 0);
  EXPECT_EQ(empty.evaluate(0.0, 0).code, Code::kInvalidInput);
}

TEST(TrajectorySegmentPolynomials, expose_ascending_coefficient_containers) {
  const Result<TrajectorySegment> solved = TrajectorySegment::cubic(cubic_boundary(), 1.0);
  ASSERT_TRUE(solved.ok());
  EXPECT_EQ(solved.value.x_polynomial().coefficients().size(), 4u);
  EXPECT_EQ(solved.value.y_polynomial().coefficients().size(), 4u);
  EXPECT_EQ(solved.value.x_polynomial().coefficients().front(), cubic_boundary().p0_m.x_m);
  EXPECT_EQ(solved.value.y_polynomial().coefficients().front(), cubic_boundary().p0_m.y_m);
}

}  // namespace
}  // namespace planner::polynomial
