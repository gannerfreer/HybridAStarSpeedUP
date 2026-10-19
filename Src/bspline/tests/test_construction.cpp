// planner::bspline::Bspline 的构造验证。

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "planner/bspline/curve.hpp"
#include "test_support.hpp"

namespace planner::bspline {
namespace {

TEST(BsplineConstruction, AcceptsClampedCubic) {
  std::string message;
  const std::vector<common::Point2d> points = {
      {0.0, 0.0}, {1.0, 3.0}, {3.0, 3.0}, {4.0, 0.0}};
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
  const std::optional<Bspline> curve =
      Bspline::create(points, knots, &message);
  ASSERT_TRUE(curve.has_value()) << message;
  EXPECT_EQ(curve->degree(), 3);
  EXPECT_EQ(curve->control_point_count(), 4u);
  EXPECT_EQ(curve->knots(), knots);
  ASSERT_EQ(curve->control_points().size(), points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_DOUBLE_EQ(curve->control_points()[i].x_m, points[i].x_m);
    EXPECT_DOUBLE_EQ(curve->control_points()[i].y_m, points[i].y_m);
  }
  const ParameterDomain domain = curve->parameter_domain();
  EXPECT_EQ(domain.kind, ParameterKind::kGeometric);
  EXPECT_DOUBLE_EQ(domain.min_parameter, 0.0);
  EXPECT_DOUBLE_EQ(domain.max_parameter, 1.0);
  EXPECT_TRUE(curve->is_geometric_parameter());
}

TEST(BsplineConstruction, AcceptsLinearAndQuadratic) {
  std::string message;
  const std::optional<Bspline> line = Bspline::create(
      {{0.0, 0.0}, {10.0, 0.0}}, {0.0, 0.0, 1.0, 1.0}, &message);
  ASSERT_TRUE(line.has_value()) << message;
  EXPECT_EQ(line->degree(), 1);

  const std::optional<Bspline> quadratic =
      Bspline::create({{0.0, 0.0}, {1.0, 2.0}, {2.0, 0.0}},
                      {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}, &message);
  ASSERT_TRUE(quadratic.has_value()) << message;
  EXPECT_EQ(quadratic->degree(), 2);
}

TEST(BsplineConstruction, RejectsTooFewControlPoints) {
  std::string message;
  EXPECT_FALSE(Bspline::create({{0.0, 0.0}}, {0.0, 0.0}, nullptr).has_value());
  // 仅有 2 个控制点却要求 degree 3（节点数量蕴含 degree 3）。
  const auto too_few = Bspline::create(
      {{0.0, 0.0}, {1.0, 1.0}},
      {0.0, 0.0, 0.0, 0.0, 1.0, 1.0}, &message);
  EXPECT_FALSE(too_few.has_value());
  EXPECT_FALSE(message.empty());
}

TEST(BsplineConstruction, RejectsDegreeZero) {
  std::string message;
  const auto curve = Bspline::create({{0.0, 0.0}, {1.0, 0.0}},
                                     {0.0, 1.0, 2.0}, &message);
  EXPECT_FALSE(curve.has_value());
  EXPECT_FALSE(message.empty());
}

TEST(BsplineConstruction, RejectsDecreasingKnots) {
  std::string message;
  const auto curve = Bspline::create(
      {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 1.0}},
      {0.0, 0.0, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0}, &message);
  EXPECT_FALSE(curve.has_value());
  EXPECT_FALSE(message.empty());
}

TEST(BsplineConstruction, RejectsExcessiveMultiplicity) {
  std::string message;
  const auto curve = Bspline::create(
      {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 1.0}},
      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0}, &message);
  EXPECT_FALSE(curve.has_value());
  EXPECT_FALSE(message.empty());
}

TEST(BsplineConstruction, RejectsNonFiniteValues) {
  std::string message;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(Bspline::create({{0.0, 0.0}, {nan, 1.0}, {2.0, 0.0}, {3.0, 1.0}},
                               {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0},
                               &message)
                   .has_value());
  EXPECT_FALSE(Bspline::create({{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 1.0}},
                               {0.0, 0.0, 0.0, 0.0, inf, 1.0, 1.0, 1.0},
                               &message)
                   .has_value());
  EXPECT_FALSE(Bspline::create({{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, inf}},
                               {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0},
                               &message)
                   .has_value());
}

TEST(BsplineConstruction, RejectsZeroWidthDomain) {
  std::string message;
  const auto curve = Bspline::create(
      {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 1.0}},
      {0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0}, &message);
  EXPECT_FALSE(curve.has_value());
  EXPECT_FALSE(message.empty());
}

TEST(BsplineConstruction, TimeParameterizedCurve) {
  std::string message;
  const std::optional<Bspline> curve = Bspline::create_time_parameterized(
      {{0.0, 0.0}, {10.0, 0.0}}, {0.0, 0.0, 1.0, 1.0}, 0.0, 2.0, &message);
  ASSERT_TRUE(curve.has_value()) << message;
  EXPECT_FALSE(curve->is_geometric_parameter());
  const ParameterDomain domain = curve->parameter_domain();
  EXPECT_EQ(domain.kind, ParameterKind::kTime);
  EXPECT_DOUBLE_EQ(domain.min_parameter, 0.0);
  EXPECT_DOUBLE_EQ(domain.max_parameter, 2.0);
}

TEST(BsplineConstruction, TimeParameterizedRejectsBadIntervals) {
  std::string message;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(Bspline::create_time_parameterized(
                   {{0.0, 0.0}, {10.0, 0.0}}, {0.0, 0.0, 1.0, 1.0}, 2.0, 1.0,
                   &message)
                   .has_value());
  EXPECT_FALSE(Bspline::create_time_parameterized(
                   {{0.0, 0.0}, {10.0, 0.0}}, {0.0, 0.0, 1.0, 1.0}, 1.0, 1.0,
                   &message)
                   .has_value());
  EXPECT_FALSE(Bspline::create_time_parameterized(
                   {{0.0, 0.0}, {10.0, 0.0}}, {0.0, 0.0, 1.0, 1.0}, nan, 3.0,
                   &message)
                   .has_value());
}

TEST(BsplineConstruction, CopyIsDeepAndIndependent) {
  const Bspline original = test::two_segment_c0();
  Bspline copy = original;
  EXPECT_EQ(copy.degree(), original.degree());
  EXPECT_EQ(copy.knots(), original.knots());
  const EvaluationResult mid = copy.evaluate(0.25, 0);
  ASSERT_EQ(mid.status, Status::kSuccess);
  // 第一段三次 Bezier 分段的解析中点。
  EXPECT_POINT_NEAR(mid.value, (common::Point2d{2.0, 1.5}), 1e-9);
}

TEST(BsplineConstruction, ClampedUniformKnots) {
  EXPECT_EQ(clamped_uniform_knots(4, 3),
            (std::vector<double>{0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}));
  EXPECT_EQ(clamped_uniform_knots(5, 3),
            (std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0}));
  EXPECT_EQ(clamped_uniform_knots(3, 2),
            (std::vector<double>{0.0, 0.0, 0.0, 1.0, 1.0, 1.0}));
  EXPECT_TRUE(clamped_uniform_knots(2, 3).empty());   // 控制点过少
  EXPECT_TRUE(clamped_uniform_knots(4, 0).empty());   // degree 为 0
  EXPECT_TRUE(clamped_uniform_knots(4, 3, 1.0, 1.0).empty());
}

TEST(BsplineConstruction, StatusNamesAreStable) {
  EXPECT_STREQ(to_string(Status::kSuccess), "success");
  EXPECT_STREQ(to_string(Status::kInvalidInput), "invalid_input");
  EXPECT_STREQ(to_string(Status::kParameterOutOfDomain),
               "parameter_out_of_domain");
  EXPECT_STREQ(to_string(Status::kDerivativeOrderUnsupported),
               "derivative_order_unsupported");
  EXPECT_STREQ(to_string(Status::kDegenerateCurve), "degenerate_curve");
  EXPECT_STREQ(to_string(Status::kArcLengthToleranceExceeded),
               "arc_length_tolerance_exceeded");
  EXPECT_STREQ(to_string(Status::kInternalError), "internal_error");
}

}  // namespace
}  // namespace planner::bspline
