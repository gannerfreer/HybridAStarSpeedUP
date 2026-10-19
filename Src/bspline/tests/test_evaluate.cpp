// Bspline 的求值、导数与参数域契约。

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
using test::bezier_value;
using test::near_point;
using test::near_value;
using test::reference_evaluate;

TEST(BsplineEvaluate, LinearCurve) {
  const Bspline line = test::require_curve({{0.0, 0.0}, {10.0, 0.0}},
                                           {0.0, 0.0, 1.0, 1.0});
  EXPECT_POINT_NEAR(line.evaluate(0.0, 0).value, (common::Point2d{0.0, 0.0}), 1e-12);
  EXPECT_POINT_NEAR(line.evaluate(0.25, 0).value, (common::Point2d{2.5, 0.0}), 1e-12);
  EXPECT_POINT_NEAR(line.evaluate(1.0, 0).value, (common::Point2d{10.0, 0.0}), 1e-12);
  EXPECT_POINT_NEAR(line.evaluate(0.4, 1).value, (common::Point2d{10.0, 0.0}), 1e-12);
  // 高于次数的求导阶数恒为零。
  const EvaluationResult second = line.evaluate(0.4, 2);
  ASSERT_EQ(second.status, Status::kSuccess);
  EXPECT_DOUBLE_EQ(second.value.x_m, 0.0);
  EXPECT_DOUBLE_EQ(second.value.y_m, 0.0);
}

TEST(BsplineEvaluate, QuadraticBezierAnalytic) {
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 2.0}, {2.0, 0.0}};
  const Bspline curve = test::require_curve(points,
                                            {0.0, 0.0, 0.0, 1.0, 1.0, 1.0});
  for (const double t : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    for (int order = 0; order <= 2; ++order) {
      const EvaluationResult result = curve.evaluate(t, order);
      ASSERT_EQ(result.status, Status::kSuccess);
      const common::Point2d expected = bezier_derivative(points, t, order, 1.0);
      EXPECT_TRUE(near_point(result.value, expected, 1e-10, 1e-10))
          << "t=" << t << " order=" << order;
    }
  }
  // 二次曲线的三阶导数为零。
  EXPECT_DOUBLE_EQ(curve.evaluate(0.3, 3).value.x_m, 0.0);
}

TEST(BsplineEvaluate, CubicBezierAnalytic) {
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}};
  const Bspline curve = test::bezier_cubic(points);
  for (const double t : {0.0, 0.125, 0.25, 0.5, 0.875, 1.0}) {
    for (int order = 0; order <= 3; ++order) {
      const EvaluationResult result = curve.evaluate(t, order);
      ASSERT_EQ(result.status, Status::kSuccess);
      const common::Point2d expected = bezier_derivative(points, t, order, 1.0);
      EXPECT_TRUE(near_point(result.value, expected, 1e-10, 1e-10))
          << "t=" << t << " order=" << order;
    }
  }
}

TEST(BsplineEvaluate, ClampedEndpointsAndEndpointDerivatives) {
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}};
  const Bspline curve = test::bezier_cubic(points);
  EXPECT_POINT_NEAR(curve.evaluate(0.0, 0).value, points.front(), 1e-12);
  EXPECT_POINT_NEAR(curve.evaluate(1.0, 0).value, points.back(), 1e-12);
  // 夹持三次曲线的端点导数：3 * (P1 - P0) / (u_{p+1} - u_0)。
  const common::Point2d start_derivative =
      test::scale(test::sub(points[1], points[0]), 3.0);
  const common::Point2d end_derivative =
      test::scale(test::sub(points[3], points[2]), 3.0);
  EXPECT_POINT_NEAR(curve.evaluate(0.0, 1).value, start_derivative, 1e-10);
  EXPECT_POINT_NEAR(curve.evaluate(1.0, 1).value, end_derivative, 1e-10);
  // 在参数域最后一点处使用最后一个分段。
  const EvaluationResult end_second = curve.evaluate(1.0, 2);
  ASSERT_EQ(end_second.status, Status::kSuccess);
  // 对三次 Bezier，B''(1) = 6 * (P3 - 2 P2 + P1)。
  const common::Point2d expected_second = test::scale(
      test::add(points[3], test::sub(points[1], test::scale(points[2], 2.0))),
      6.0);
  EXPECT_POINT_NEAR(end_second.value, expected_second, 1e-9);
}

TEST(BsplineEvaluate, RejectsDerivativeOrdersOutsideRange) {
  const Bspline curve = test::bezier_cubic(
      {{0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}});
  EXPECT_EQ(curve.evaluate(0.5, 4).status, Status::kDerivativeOrderUnsupported);
  EXPECT_EQ(curve.evaluate(0.5, -1).status, Status::kDerivativeOrderUnsupported);
  EXPECT_EQ(curve.evaluate(0.5, 100).status, Status::kDerivativeOrderUnsupported);
}

TEST(BsplineEvaluate, RejectsParametersOutsideDomain) {
  const Bspline curve = test::bezier_cubic(
      {{0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}});
  EXPECT_EQ(curve.evaluate(-1e-6, 0).status, Status::kParameterOutOfDomain);
  EXPECT_EQ(curve.evaluate(1.0 + 1e-6, 0).status, Status::kParameterOutOfDomain);
  EXPECT_EQ(curve.evaluate(2.0, 1).status, Status::kParameterOutOfDomain);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(curve.evaluate(nan, 0).status, Status::kParameterOutOfDomain);
  EXPECT_EQ(curve.evaluate(inf, 0).status, Status::kParameterOutOfDomain);
}

TEST(BsplineEvaluate, ClampsOnlyRoundingAtDomainBounds) {
  const Bspline curve = test::bezier_cubic(
      {{0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}});
  const EvaluationResult just_above = curve.evaluate(1.0 + 1e-15, 0);
  ASSERT_EQ(just_above.status, Status::kSuccess);
  EXPECT_POINT_NEAR(just_above.value, (common::Point2d{4.0, 0.0}), 1e-12);
  const EvaluationResult just_below = curve.evaluate(-1e-15, 0);
  ASSERT_EQ(just_below.status, Status::kSuccess);
  EXPECT_POINT_NEAR(just_below.value, (common::Point2d{0.0, 0.0}), 1e-12);
}

TEST(BsplineEvaluate, InteriorKnotUsesRightSegment) {
  const Bspline curve = test::two_segment_c0();
  const std::vector<common::Point2d> left = {
      {0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0}};
  const std::vector<common::Point2d> right = {
      {4.0, 0.0}, {5.0, 2.0}, {7.0, 2.0}, {8.0, 0.0}};
  // 在 C0 连接处函数值连续。
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 0).value, (common::Point2d{4.0, 0.0}), 1e-12);
  // 一阶导数：取右侧分段的取值，且与左侧不同。
  const EvaluationResult first = curve.evaluate(0.5, 1);
  ASSERT_EQ(first.status, Status::kSuccess);
  const common::Point2d right_first = bezier_derivative(right, 0.0, 1, 0.5);
  const common::Point2d left_first = bezier_derivative(left, 1.0, 1, 0.5);
  EXPECT_POINT_NEAR(first.value, right_first, 1e-9);
  EXPECT_GT(test::norm(test::sub(right_first, left_first)), 1.0);
  // 二阶与三阶导数遵循相同的右侧分段约定。
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 2).value,
                    bezier_derivative(right, 0.0, 2, 0.5), 1e-9);
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 3).value,
                    bezier_derivative(right, 0.0, 3, 0.5), 1e-9);
}

TEST(BsplineEvaluate, GapKnotUsesRightSegment) {
  const Bspline curve = test::two_segment_gap();
  const std::vector<common::Point2d> left = {
      {0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0}};
  const std::vector<common::Point2d> right = {
      {5.0, 2.0}, {7.0, 2.0}, {8.0, 0.0}, {9.0, -3.0}};
  // 间断点处取值双值；右侧分段优先。
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 0).value, right.front(), 1e-12);
  EXPECT_POINT_NEAR(curve.evaluate(0.5 - 1e-9, 0).value, left.back(), 1e-6);
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 1).value,
                    bezier_derivative(right, 0.0, 1, 0.5), 1e-9);
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 2).value,
                    bezier_derivative(right, 0.0, 2, 0.5), 1e-9);
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 3).value,
                    bezier_derivative(right, 0.0, 3, 0.5), 1e-9);
}

TEST(BsplineEvaluate, RepeatedKnotMultiplicityTwo) {
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 2.0}, {2.0, 3.0}, {4.0, 3.0}, {5.0, 2.0}, {6.0, 0.0}};
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 0.5, 0.5,
                                     1.0, 1.0, 1.0, 1.0};
  const Bspline curve = test::require_curve(points, knots);
  // 函数值与一阶导数与独立的 de Boor 参考实现一致。
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 0).value,
                    reference_evaluate(points, knots, 3, 0.5, 0), 1e-9);
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 1).value,
                    reference_evaluate(points, knots, 3, 0.5, 1), 1e-9);
  EXPECT_POINT_NEAR(curve.evaluate(0.25, 2).value,
                    reference_evaluate(points, knots, 3, 0.25, 2), 1e-9);
}

TEST(BsplineEvaluate, GeneralClampedSplineMatchesReference) {
  // 内部节点为单节点的夹持均匀三次曲线（类似测试夹具）。
  const std::vector<common::Point2d> points = {
      {0.0, -1.0}, {0.5, -0.75}, {1.0, -0.5}, {1.5, -0.25},
      {2.0, 0.0},  {2.5, 0.25},  {3.0, 0.5},  {3.5, 0.75}};
  const std::vector<double> knots = {0.0,   0.0,   0.0,   0.0,  0.125, 0.25,
                                     0.375, 0.5,   1.0,   1.0,  1.0,   1.0};
  const Bspline curve = test::require_curve(points, knots);
  for (int i = 0; i <= 40; ++i) {
    const double u = static_cast<double>(i) / 40.0;
    for (int order = 0; order <= 3; ++order) {
      const EvaluationResult result = curve.evaluate(u, order);
      ASSERT_EQ(result.status, Status::kSuccess);
      const common::Point2d expected =
          reference_evaluate(points, knots, 3, u, order);
      EXPECT_TRUE(near_point(result.value, expected, 1e-9, 1e-9))
          << "u=" << u << " order=" << order;
    }
  }
}

TEST(BsplineEvaluate, OpenKnotVectorUsesStandardDomain) {
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 2.0}, {2.0, 0.0}};
  const std::vector<double> knots = {0.0, 0.1, 0.2, 0.6, 0.9, 1.0};
  const Bspline curve = test::require_curve(points, knots);
  const ParameterDomain domain = curve.parameter_domain();
  EXPECT_DOUBLE_EQ(domain.min_parameter, 0.2);
  EXPECT_DOUBLE_EQ(domain.max_parameter, 0.6);
  EXPECT_EQ(curve.evaluate(0.1, 0).status, Status::kParameterOutOfDomain);
  EXPECT_EQ(curve.evaluate(0.7, 0).status, Status::kParameterOutOfDomain);
  for (int i = 0; i <= 20; ++i) {
    const double u = 0.2 + 0.4 * static_cast<double>(i) / 20.0;
    EXPECT_POINT_NEAR(curve.evaluate(u, 0).value,
                      reference_evaluate(points, knots, 2, u, 0), 1e-10);
    EXPECT_POINT_NEAR(curve.evaluate(u, 1).value,
                      reference_evaluate(points, knots, 2, u, 1), 1e-9);
  }
}

TEST(BsplineEvaluate, TimeParameterizationDistinguishesTimeFromGeometry) {
  const std::vector<common::Point2d> points = {{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<double> knots = {0.0, 0.0, 1.0, 1.0};
  const Bspline geometric = test::require_curve(points, knots);
  const Bspline timed =
      test::require_time_curve(points, knots, 0.0, 2.0);

  // 几何参数：参数 1.0 是曲线的终点。
  EXPECT_POINT_NEAR(geometric.evaluate(1.0, 0).value,
                    (common::Point2d{10.0, 0.0}), 1e-12);
  EXPECT_POINT_NEAR(geometric.evaluate(1.0, 1).value,
                    (common::Point2d{10.0, 0.0}), 1e-12);

  // 时间参数：参数 1.0 s 是中点；du/dt = 0.5。
  EXPECT_POINT_NEAR(timed.evaluate(1.0, 0).value,
                    (common::Point2d{5.0, 0.0}), 1e-12);
  EXPECT_POINT_NEAR(timed.evaluate(1.0, 1).value,
                    (common::Point2d{5.0, 0.0}), 1e-12);
  EXPECT_POINT_NEAR(timed.evaluate(2.0, 0).value,
                    (common::Point2d{10.0, 0.0}), 1e-12);
  EXPECT_DOUBLE_EQ(timed.evaluate(1.0, 2).value.x_m, 0.0);
  EXPECT_EQ(timed.evaluate(2.5, 0).status, Status::kParameterOutOfDomain);
  EXPECT_EQ(timed.evaluate(-0.1, 0).status, Status::kParameterOutOfDomain);
}

TEST(BsplineEvaluate, TimeParameterizationScalesHigherDerivatives) {
  // 节点域 [0, 1] 上的三次曲线映射到 [0, 4] s。
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}};
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
  const Bspline timed = test::require_time_curve(points, knots, 0.0, 4.0);
  const double t = 1.0;  // u = 0.25
  for (int order = 0; order <= 3; ++order) {
    const common::Point2d expected =
        test::scale(bezier_derivative(points, 0.25, order, 1.0),
                    std::pow(0.25, order));
    EXPECT_POINT_NEAR(timed.evaluate(t, order).value, expected, 1e-9);
  }
}

}  // namespace
}  // namespace planner::bspline
