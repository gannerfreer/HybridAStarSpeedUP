// Bspline 的连续性与右侧分段约定测试。
//
// 单侧求值点相隔 2*delta，因此连续函数在两者之间的差值为
// |f^{(d+1)}| * 2*delta，而不是零。下面的容差与 delta（1e-6）
// 成正比选取，而被测曲线中的实际跳变超过 10 个单位，
// 因此测试仍能清晰区分。

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

constexpr double kDelta = 1e-6;
// 连续函数的界：|f^{(d+1)}| * 2 * delta。
constexpr double kContinuousTolerance = 2e-3;
// 节点处跳变的右连续性界。
constexpr double kRightTolerance = 1e-3;

double one_sided_distance(const Bspline& curve, double knot, int order) {
  const EvaluationResult left = curve.evaluate(knot - kDelta, order);
  const EvaluationResult right = curve.evaluate(knot + kDelta, order);
  EXPECT_EQ(left.status, Status::kSuccess);
  EXPECT_EQ(right.status, Status::kSuccess);
  return test::norm(test::sub(left.value, right.value));
}

// 节点处的取值是右侧分段的取值：它与右极限一致，
// 且在间断的阶数上与左侧不同。
void expect_right_segment_value(const Bspline& curve, double knot, int order,
                                double jump_threshold) {
  const EvaluationResult at_knot = curve.evaluate(knot, order);
  const EvaluationResult right = curve.evaluate(knot + kDelta, order);
  const EvaluationResult left = curve.evaluate(knot - kDelta, order);
  ASSERT_EQ(at_knot.status, Status::kSuccess);
  ASSERT_EQ(right.status, Status::kSuccess);
  ASSERT_EQ(left.status, Status::kSuccess);
  EXPECT_TRUE(near_point(at_knot.value, right.value, kRightTolerance,
                         kRightTolerance));
  EXPECT_GT(test::norm(test::sub(at_knot.value, left.value)), jump_threshold);
}

TEST(BsplineContinuity, C0JoinIsValueContinuousAndDerivativeJumps) {
  const Bspline curve = test::two_segment_c0();
  // degree 3、重数 3：函数值连续。
  EXPECT_LT(one_sided_distance(curve, 0.5, 0), kContinuousTolerance);
  // 一阶导数跳变（24 个单位）且在节点处使用右侧分段。
  EXPECT_GT(one_sided_distance(curve, 0.5, 1), 10.0);
  expect_right_segment_value(curve, 0.5, 1, 10.0);
  expect_right_segment_value(curve, 0.5, 2, 10.0);
}

TEST(BsplineContinuity, C1JoinKeepsFirstDerivativeContinuous) {
  // degree 3、内部节点重数 2：C1，二阶导数跳变。
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {2.0, 1.0}, {1.0, 3.0}, {5.0, 2.0}, {4.0, 4.0}, {7.0, 0.0}};
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 0.5, 0.5,
                                     1.0, 1.0, 1.0, 1.0};
  const Bspline curve = test::require_curve(points, knots);
  EXPECT_LT(one_sided_distance(curve, 0.5, 0), kContinuousTolerance);
  EXPECT_LT(one_sided_distance(curve, 0.5, 1), kContinuousTolerance);
  // 二阶导数跳变（约 187 个单位），且在节点处右连续。
  EXPECT_GT(one_sided_distance(curve, 0.5, 2), 10.0);
  expect_right_segment_value(curve, 0.5, 2, 10.0);
}

TEST(BsplineContinuity, C2JoinKeepsFirstAndSecondDerivativesContinuous) {
  // degree 3、内部节点重数 1：C2。
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 2.0}, {2.0, 3.0}, {4.0, 3.0}, {6.0, 0.0}};
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 0.5,
                                     1.0, 1.0, 1.0, 1.0};
  const Bspline curve = test::require_curve(points, knots);
  EXPECT_LT(one_sided_distance(curve, 0.5, 0), kContinuousTolerance);
  EXPECT_LT(one_sided_distance(curve, 0.5, 1), kContinuousTolerance);
  EXPECT_LT(one_sided_distance(curve, 0.5, 2), kContinuousTolerance);
  // 三阶导数跳变，且在节点处右连续。
  EXPECT_GT(one_sided_distance(curve, 0.5, 3), 10.0);
  expect_right_segment_value(curve, 0.5, 3, 10.0);
}

TEST(BsplineContinuity, GapKnotIsRightSegmentValue) {
  const Bspline curve = test::two_segment_gap();
  const common::Point2d left_end{4.0, 0.0};
  const common::Point2d right_start{5.0, 2.0};
  EXPECT_POINT_NEAR(curve.evaluate(0.5 - kDelta, 0).value, left_end, 1e-3);
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 0).value, right_start, 1e-12);
  EXPECT_GT(test::norm(test::sub(right_start, left_end)), 1.0);
}

TEST(BsplineContinuity, SimpleKnotSplineHasExpectedContinuity) {
  // 内部节点均为单节点的夹持均匀三次曲线：内部处处 C2。
  const std::vector<common::Point2d> points = {
      {0.0, -1.0}, {0.5, -0.75}, {1.0, -0.5}, {1.5, -0.25},
      {2.0, 0.0},  {2.5, 0.25},  {3.0, 0.5},  {3.5, 0.75}};
  const std::vector<double> knots = {0.0,   0.0,   0.0,   0.0,  0.125, 0.25,
                                     0.375, 0.5,   1.0,   1.0,  1.0,   1.0};
  const Bspline curve = test::require_curve(points, knots);
  for (const double knot : {0.125, 0.25, 0.375, 0.5}) {
    EXPECT_LT(one_sided_distance(curve, knot, 0), kContinuousTolerance);
    EXPECT_LT(one_sided_distance(curve, knot, 1), kContinuousTolerance);
    EXPECT_LT(one_sided_distance(curve, knot, 2), kContinuousTolerance);
    // 三阶导数为分段常数，在每个节点处跳变。
    EXPECT_GT(one_sided_distance(curve, knot, 3), 10.0);
    expect_right_segment_value(curve, knot, 3, 10.0);
  }
}

TEST(BsplineContinuity, ReferenceAgreesWithAnalyticBeziers) {
  // 独立参考实现必须先与解析 Bezier 结果一致，
  // 才能用作样条求值的基准真值。
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}};
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
  for (int i = 0; i <= 16; ++i) {
    const double u = static_cast<double>(i) / 16.0;
    for (int order = 0; order <= 3; ++order) {
      EXPECT_TRUE(near_point(reference_evaluate(points, knots, 3, u, order),
                             test::bezier_derivative(points, u, order, 1.0),
                             1e-10, 1e-10));
    }
  }
  // 在双分段 C0 样条上，参考实现也与右侧分段一致。
  const Bspline c0 = test::two_segment_c0();
  const std::vector<common::Point2d> right = {
      {4.0, 0.0}, {5.0, 2.0}, {7.0, 2.0}, {8.0, 0.0}};
  for (int order = 0; order <= 3; ++order) {
    EXPECT_TRUE(near_point(
        reference_evaluate(
            c0.control_points(), c0.knots(), 3, 0.5, order),
        test::bezier_derivative(right, 0.0, order, 0.5), 1e-10, 1e-10));
  }
}

}  // namespace
}  // namespace planner::bspline
