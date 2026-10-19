// Bspline::sample 的批量采样契约。

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "bspline_reference.hpp"
#include "planner/bspline/curve.hpp"
#include "test_support.hpp"

namespace planner::bspline {
namespace {

using test::near_point;
using test::reference_evaluate;

TEST(BsplineSample, MatchesSingleEvaluation) {
  const Bspline curve = test::two_segment_c0();
  std::vector<double> parameters;
  for (int i = 0; i <= 100; ++i) {
    parameters.push_back(static_cast<double>(i) / 100.0);
  }
  for (int order = 0; order <= 3; ++order) {
    std::vector<common::Point2d> buffer;
    ASSERT_EQ(curve.sample(parameters, buffer, order), Status::kSuccess);
    ASSERT_EQ(buffer.size(), parameters.size());
    for (std::size_t i = 0; i < parameters.size(); ++i) {
      const EvaluationResult single = curve.evaluate(parameters[i], order);
      ASSERT_EQ(single.status, Status::kSuccess);
      EXPECT_TRUE(near_point(buffer[i], single.value, 1e-12, 1e-12))
          << "i=" << i << " order=" << order;
    }
  }
}

TEST(BsplineSample, MatchesReferenceOnUniformCubic) {
  const std::vector<common::Point2d> points = {
      {0.0, -1.0}, {0.5, -0.75}, {1.0, -0.5}, {1.5, -0.25},
      {2.0, 0.0},  {2.5, 0.25},  {3.0, 0.5},  {3.5, 0.75}};
  const std::vector<double> knots = {0.0,   0.0,   0.0,   0.0,  0.125, 0.25,
                                     0.375, 0.5,   1.0,   1.0,  1.0,   1.0};
  const Bspline curve = test::require_curve(points, knots);
  std::vector<double> parameters;
  for (int i = 0; i <= 257; ++i) {
    parameters.push_back(static_cast<double>(i) / 257.0);
  }
  for (int order = 0; order <= 3; ++order) {
    std::vector<common::Point2d> buffer;
    ASSERT_EQ(curve.sample(parameters, buffer, order), Status::kSuccess);
    for (std::size_t i = 0; i < parameters.size(); ++i) {
      const common::Point2d expected =
          reference_evaluate(points, knots, 3, parameters[i], order);
      EXPECT_TRUE(near_point(buffer[i], expected, 1e-9, 1e-9))
          << "i=" << i << " order=" << order;
    }
  }
}

TEST(BsplineSample, RightSegmentWinsAtDiscontinuousKnots) {
  const Bspline curve = test::two_segment_c0();
  // 0.5 恰好是 C0 节点；批处理路径必须返回右侧取值。
  const std::vector<double> parameters = {0.25, 0.5, 0.75};
  for (int order = 0; order <= 3; ++order) {
    std::vector<common::Point2d> buffer;
    ASSERT_EQ(curve.sample(parameters, buffer, order), Status::kSuccess);
    for (std::size_t i = 0; i < parameters.size(); ++i) {
      const EvaluationResult single = curve.evaluate(parameters[i], order);
      ASSERT_EQ(single.status, Status::kSuccess);
      EXPECT_TRUE(near_point(buffer[i], single.value, 1e-12, 1e-12));
    }
  }
  // 节点处的取值不得是左侧分段的导数。
  std::vector<common::Point2d> buffer;
  ASSERT_EQ(curve.sample({0.5}, buffer, 1), Status::kSuccess);
  const EvaluationResult single = curve.evaluate(0.5, 1);
  EXPECT_POINT_NEAR(buffer[0], single.value, 1e-12);
  EXPECT_GT(test::norm(test::sub(buffer[0], common::Point2d{6.0, -12.0})), 1.0);
}

TEST(BsplineSample, GapCurveUsesRightSegmentValues) {
  const Bspline curve = test::two_segment_gap();
  const std::vector<double> parameters = {0.49, 0.5, 0.51};
  for (int order = 0; order <= 3; ++order) {
    std::vector<common::Point2d> buffer;
    ASSERT_EQ(curve.sample(parameters, buffer, order), Status::kSuccess);
    for (std::size_t i = 0; i < parameters.size(); ++i) {
      const EvaluationResult single = curve.evaluate(parameters[i], order);
      ASSERT_EQ(single.status, Status::kSuccess);
      EXPECT_TRUE(near_point(buffer[i], single.value, 1e-9, 1e-9));
    }
  }
}

TEST(BsplineSample, BufferIsResizedToParameterCount) {
  const Bspline curve = test::two_segment_c0();
  std::vector<common::Point2d> buffer(1000, {7.0, 7.0});
  ASSERT_EQ(curve.sample({0.0, 0.5, 1.0}, buffer, 0), Status::kSuccess);
  EXPECT_EQ(buffer.size(), 3u);
  EXPECT_POINT_NEAR(buffer[0], (common::Point2d{0.0, 0.0}), 1e-12);
  EXPECT_POINT_NEAR(buffer[2], (common::Point2d{8.0, 0.0}), 1e-12);
}

TEST(BsplineSample, EmptyInputSucceeds) {
  const Bspline curve = test::two_segment_c0();
  std::vector<common::Point2d> buffer;
  EXPECT_EQ(curve.sample({}, buffer, 0), Status::kSuccess);
  EXPECT_TRUE(buffer.empty());
}

TEST(BsplineSample, RejectsInvalidParameterWithoutTouchingBuffer) {
  const Bspline curve = test::two_segment_c0();
  std::vector<common::Point2d> buffer(2, {3.0, 4.0});
  EXPECT_EQ(curve.sample({0.25, 1.5}, buffer, 0), Status::kParameterOutOfDomain);
  ASSERT_EQ(buffer.size(), 2u);
  EXPECT_DOUBLE_EQ(buffer[0].x_m, 3.0);
  EXPECT_DOUBLE_EQ(buffer[1].y_m, 4.0);
}

TEST(BsplineSample, RejectsUnsupportedDerivativeOrder) {
  const Bspline curve = test::two_segment_c0();
  std::vector<common::Point2d> buffer;
  EXPECT_EQ(curve.sample({0.25}, buffer, 4), Status::kDerivativeOrderUnsupported);
  EXPECT_EQ(curve.sample({0.25}, buffer, -1), Status::kDerivativeOrderUnsupported);
}

TEST(BsplineSample, OrderAboveDegreeIsZero) {
  const Bspline line = test::require_curve({{0.0, 0.0}, {10.0, 0.0}},
                                           {0.0, 0.0, 1.0, 1.0});
  std::vector<common::Point2d> buffer;
  ASSERT_EQ(line.sample({0.0, 0.5, 1.0}, buffer, 3), Status::kSuccess);
  for (const common::Point2d& point : buffer) {
    EXPECT_DOUBLE_EQ(point.x_m, 0.0);
    EXPECT_DOUBLE_EQ(point.y_m, 0.0);
  }
}

TEST(BsplineSample, TimeParameterizedCurveMatchesEvaluate) {
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}};
  const Bspline timed =
      test::require_time_curve(points, {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0},
                               0.0, 4.0);
  std::vector<double> parameters;
  for (int i = 0; i <= 64; ++i) {
    parameters.push_back(4.0 * static_cast<double>(i) / 64.0);
  }
  for (int order = 0; order <= 3; ++order) {
    std::vector<common::Point2d> buffer;
    ASSERT_EQ(timed.sample(parameters, buffer, order), Status::kSuccess);
    for (std::size_t i = 0; i < parameters.size(); ++i) {
      const EvaluationResult single = timed.evaluate(parameters[i], order);
      ASSERT_EQ(single.status, Status::kSuccess);
      EXPECT_TRUE(near_point(buffer[i], single.value, 1e-12, 1e-12));
    }
  }
}

}  // namespace
}  // namespace planner::bspline
