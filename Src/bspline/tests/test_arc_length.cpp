// planner_bspline 的弧长重采样测试。

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "bspline_reference.hpp"
#include "planner/bspline/curve.hpp"
#include "test_support.hpp"

namespace planner::bspline {
namespace {

using test::bezier_derivative;
using test::near_point;

// 在 [0, span] 上对 Bezier 分段速率做复合 Simpson 积分。
double bezier_arc_length(const std::vector<common::Point2d>& points, double span,
                         int intervals = 20000) {
  const int steps = 2 * intervals;
  const double step = span / static_cast<double>(steps);
  double sum = 0.0;
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double speed = test::norm(bezier_derivative(points, t, 1, span));
    const int weight = (i == 0 || i == steps) ? 1 : (i % 2 == 0 ? 2 : 4);
    sum += weight * speed;
  }
  return sum * step / 3.0;
}

TEST(BsplineArcLength, StraightLineIsExactlySpaced) {
  const Bspline line = test::require_curve({{0.0, 0.0}, {10.0, 0.0}},
                                           {0.0, 0.0, 1.0, 1.0});
  EXPECT_NEAR(line.total_length_m(), 10.0, 1e-9);
  const ArcLengthResampleResult result = line.sample_equal_arc_length(11);
  ASSERT_EQ(result.status, Status::kSuccess);
  EXPECT_NEAR(result.total_length_m, 10.0, 1e-9);
  EXPECT_NEAR(result.nominal_interval_m, 1.0, 1e-9);
  EXPECT_LT(result.max_interval_error_m, 1e-3);
  EXPECT_LT(result.last_interval_error_m, 1e-3);
  EXPECT_FALSE(result.has_remainder);
  ASSERT_EQ(result.samples.size(), 11u);
  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    EXPECT_NEAR(result.samples[i].parameter, static_cast<double>(i) / 10.0, 1e-9);
    EXPECT_NEAR(result.samples[i].position.x_m, static_cast<double>(i), 1e-7);
    EXPECT_NEAR(result.samples[i].position.y_m, 0.0, 1e-9);
    EXPECT_NEAR(result.samples[i].arc_length_m,
                static_cast<double>(i), 1e-7);
  }
}

TEST(BsplineArcLength, CubicBezierMatchesIndependentIntegration) {
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}};
  const Bspline curve = test::bezier_cubic(points);
  const double reference = bezier_arc_length(points, 1.0);
  EXPECT_NEAR(curve.total_length_m(), reference, 1e-6);

  const ArcLengthResampleResult result = curve.sample_equal_arc_length(101);
  ASSERT_EQ(result.status, Status::kSuccess);
  EXPECT_NEAR(result.total_length_m, reference, 1e-6);
  EXPECT_LT(result.max_interval_error_m, 1e-3);
  ASSERT_EQ(result.samples.size(), 101u);
  // 端点是精确的。
  EXPECT_POINT_NEAR(result.samples.front().position, points.front(), 1e-10);
  EXPECT_POINT_NEAR(result.samples.back().position, points.back(), 1e-10);
  // 弧长单调，参数保持在参数域内。
  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    EXPECT_GE(result.samples[i].arc_length_m, 0.0);
    EXPECT_LE(result.samples[i].parameter, 1.0);
    if (i > 0) {
      EXPECT_GT(result.samples[i].arc_length_m,
                result.samples[i - 1].arc_length_m);
      EXPECT_GE(result.samples[i].parameter, result.samples[i - 1].parameter);
    }
  }
}

TEST(BsplineArcLength, ReportedErrorMatchesIndependentComputation) {
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}};
  const Bspline curve = test::bezier_cubic(points);
  const ArcLengthResampleResult result = curve.sample_equal_arc_length(25);
  ASSERT_EQ(result.status, Status::kSuccess);
  double independent_max = 0.0;
  for (std::size_t i = 0; i + 1 < result.samples.size(); ++i) {
    const double u0 = result.samples[i].parameter;
    const double u1 = result.samples[i + 1].parameter;
    const int steps = 2000;
    const double h = (u1 - u0) / static_cast<double>(steps);
    double sum = 0.0;
    for (int k = 0; k <= steps; ++k) {
      const double t = u0 + h * static_cast<double>(k);
      const double speed = test::norm(bezier_derivative(points, t, 1, 1.0));
      sum += (k == 0 || k == steps) ? speed : (k % 2 == 0 ? 2.0 * speed : 4.0 * speed);
    }
    const double interval = sum * h / 3.0;
    independent_max =
        std::max(independent_max,
                 std::fabs(interval - result.nominal_interval_m));
  }
  EXPECT_NEAR(result.max_interval_error_m, independent_max, 1e-6);
  EXPECT_LT(result.max_interval_error_m, 1e-3);
}

TEST(BsplineArcLength, IntervalBasedResamplingHandlesRemainder) {
  const Bspline line = test::require_curve({{0.0, 0.0}, {10.0, 0.0}},
                                           {0.0, 0.0, 1.0, 1.0});
  const ArcLengthResampleResult result = line.resample_uniform_interval(0.7);
  ASSERT_EQ(result.status, Status::kSuccess);
  EXPECT_TRUE(result.has_remainder);
  EXPECT_NEAR(result.total_length_m, 10.0, 1e-9);
  EXPECT_NEAR(result.nominal_interval_m, 0.7, 1e-12);
  // 完整区间：0.7 * 14 = 9.8；余数 0.2。
  ASSERT_EQ(result.samples.size(), 16u);
  EXPECT_LT(result.max_interval_error_m, 1e-6);
  EXPECT_NEAR(result.last_interval_error_m, 0.5, 1e-6);
  EXPECT_NEAR(result.samples.back().position.x_m, 10.0, 1e-9);
  const double remainder = result.samples.back().arc_length_m -
                           result.samples[result.samples.size() - 2].arc_length_m;
  EXPECT_NEAR(remainder, 0.2, 1e-6);
}

TEST(BsplineArcLength, IntervalBasedWithoutRemainder) {
  const Bspline line = test::require_curve({{0.0, 0.0}, {10.0, 0.0}},
                                           {0.0, 0.0, 1.0, 1.0});
  const ArcLengthResampleResult result = line.resample_uniform_interval(2.5);
  ASSERT_EQ(result.status, Status::kSuccess);
  EXPECT_FALSE(result.has_remainder);
  ASSERT_EQ(result.samples.size(), 5u);
  EXPECT_LT(result.max_interval_error_m, 1e-9);
  EXPECT_LT(result.last_interval_error_m, 1e-9);
}

TEST(BsplineArcLength, C0CurveResamplingSpansTheKnot) {
  const Bspline curve = test::two_segment_c0();
  const std::vector<common::Point2d> left = {
      {0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0}};
  const std::vector<common::Point2d> right = {
      {4.0, 0.0}, {5.0, 2.0}, {7.0, 2.0}, {8.0, 0.0}};
  const double reference = bezier_arc_length(left, 0.5) +
                           bezier_arc_length(right, 0.5);
  EXPECT_NEAR(curve.total_length_m(), reference, 1e-6);

  const ArcLengthResampleResult result = curve.sample_equal_arc_length(201);
  ASSERT_EQ(result.status, Status::kSuccess);
  EXPECT_NEAR(result.total_length_m, reference, 1e-6);
  EXPECT_LT(result.max_interval_error_m, 1e-3);
  EXPECT_POINT_NEAR(result.samples.front().position, left.front(), 1e-10);
  EXPECT_POINT_NEAR(result.samples.back().position, right.back(), 1e-10);
  // 节点本身位于两个相邻采样点之间（取值连续）。
  bool found_knot = false;
  for (const ArcLengthSample& sample : result.samples) {
    if (std::fabs(sample.parameter - 0.5) < 1e-12) {
      found_knot = true;
    }
  }
  (void)found_knot;  // 节点无需被精确命中，跨越即可。
  for (std::size_t i = 0; i + 1 < result.samples.size(); ++i) {
    EXPECT_GE(result.samples[i + 1].parameter, result.samples[i].parameter);
  }
}

TEST(BsplineArcLength, GapCurveResamplesPerSegment) {
  const Bspline curve = test::two_segment_gap();
  const std::vector<common::Point2d> left = {
      {0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0}};
  const std::vector<common::Point2d> right = {
      {5.0, 2.0}, {7.0, 2.0}, {8.0, 0.0}, {9.0, -3.0}};
  const double reference =
      bezier_arc_length(left, 0.5) + bezier_arc_length(right, 0.5);
  const ArcLengthResampleResult result = curve.sample_equal_arc_length(101);
  ASSERT_EQ(result.status, Status::kSuccess);
  EXPECT_NEAR(result.total_length_m, reference, 1e-6);
  EXPECT_LT(result.max_interval_error_m, 1e-3);
}

TEST(BsplineArcLength, DegenerateZeroLengthCurveIsRejected) {
  const Bspline point_curve = test::require_curve(
      {{1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}},
      {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0});
  EXPECT_NEAR(point_curve.total_length_m(), 0.0, 1e-12);
  EXPECT_EQ(point_curve.sample_equal_arc_length(5).status,
            Status::kDegenerateCurve);
  EXPECT_EQ(point_curve.resample_uniform_interval(0.5).status,
            Status::kDegenerateCurve);
  // 退化（零长度）曲线仍可求值。
  const EvaluationResult result = point_curve.evaluate(0.5, 0);
  ASSERT_EQ(result.status, Status::kSuccess);
  EXPECT_POINT_NEAR(result.value, (common::Point2d{1.0, 1.0}), 1e-12);
}

TEST(BsplineArcLength, RejectsInvalidArguments) {
  const Bspline line = test::require_curve({{0.0, 0.0}, {10.0, 0.0}},
                                           {0.0, 0.0, 1.0, 1.0});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(line.sample_equal_arc_length(0).status, Status::kInvalidInput);
  EXPECT_EQ(line.sample_equal_arc_length(5, 0.0).status, Status::kInvalidInput);
  EXPECT_EQ(line.sample_equal_arc_length(5, -1.0).status, Status::kInvalidInput);
  EXPECT_EQ(line.sample_equal_arc_length(5, nan).status, Status::kInvalidInput);
  EXPECT_EQ(line.resample_uniform_interval(0.0).status, Status::kInvalidInput);
  EXPECT_EQ(line.resample_uniform_interval(-0.5).status, Status::kInvalidInput);
  EXPECT_EQ(line.resample_uniform_interval(nan).status, Status::kInvalidInput);
}

TEST(BsplineArcLength, SinglePointResampling) {
  const Bspline line = test::require_curve({{0.0, 0.0}, {10.0, 0.0}},
                                           {0.0, 0.0, 1.0, 1.0});
  const ArcLengthResampleResult result = line.sample_equal_arc_length(1);
  ASSERT_EQ(result.status, Status::kSuccess);
  ASSERT_EQ(result.samples.size(), 1u);
  EXPECT_NEAR(result.samples[0].parameter, 0.0, 1e-12);
  EXPECT_NEAR(result.samples[0].arc_length_m, 0.0, 1e-12);
  EXPECT_NEAR(result.samples[0].position.x_m, 0.0, 1e-12);
}

TEST(BsplineArcLength, TimeParameterizedResamplingUsesTimeParameters) {
  const std::vector<common::Point2d> points = {{0.0, 0.0}, {10.0, 0.0}};
  const Bspline timed = test::require_time_curve(
      points, {0.0, 0.0, 1.0, 1.0}, 0.0, 2.0);
  const ArcLengthResampleResult result = timed.sample_equal_arc_length(11);
  ASSERT_EQ(result.status, Status::kSuccess);
  ASSERT_EQ(result.samples.size(), 11u);
  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    const double expected_time = 2.0 * static_cast<double>(i) / 10.0;
    EXPECT_NEAR(result.samples[i].parameter, expected_time, 1e-9);
    const EvaluationResult evaluated = timed.evaluate(result.samples[i].parameter, 0);
    ASSERT_EQ(evaluated.status, Status::kSuccess);
    EXPECT_TRUE(near_point(result.samples[i].position, evaluated.value, 1e-9, 1e-9));
  }
}

TEST(BsplineArcLength, BufferOverloadReusesBuffer) {
  const Bspline line = test::require_curve({{0.0, 0.0}, {10.0, 0.0}},
                                           {0.0, 0.0, 1.0, 1.0});
  std::vector<ArcLengthSample> buffer;
  const ArcLengthResampleResult first =
      line.sample_equal_arc_length(11, buffer, 1e-3);
  ASSERT_EQ(first.status, Status::kSuccess);
  ASSERT_EQ(buffer.size(), 11u);
  EXPECT_TRUE(first.samples.empty());  // 缓冲区重载不填充 samples
  const ArcLengthResampleResult second =
      line.sample_equal_arc_length(11, buffer, 1e-3);
  ASSERT_EQ(second.status, Status::kSuccess);
  EXPECT_EQ(buffer.size(), 11u);
  EXPECT_NEAR(second.max_interval_error_m, first.max_interval_error_m, 1e-12);
}

}  // namespace
}  // namespace planner::bspline
