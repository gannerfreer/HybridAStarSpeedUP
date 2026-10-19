// 内部选型对照测试：TinySpline 与 Eigen Splines 后端
// （Docs/component_development_plan.md §3.3 的“数值与性能对照”）。
//
// 目标：
//   * 两个后端对相同控制点/节点给出相同的值、导数与批量结果；
//   * Eigen 后端与测试内的独立解析参考（Bezier/de Boor 实现）一致；
//   * 右侧分段、域拒绝、时间参数化与弧长重采样的公开语义一致。

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "bspline_reference.hpp"
#include "test_support.hpp"

namespace planner::bspline {
namespace {

using common::Point2d;

Bspline require_curve_with_backend(std::vector<Point2d> cps,
                                   std::vector<double> knots,
                                   BackendKind backend) {
  std::string message;
  std::optional<Bspline> curve =
      Bspline::create(std::move(cps), std::move(knots), backend, &message);
  if (!curve.has_value()) {
    throw std::runtime_error("curve construction failed: " + message);
  }
  return std::move(*curve);
}

Bspline cubic_bezier(BackendKind backend) {
  return require_curve_with_backend({{0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0}},
                                    {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0},
                                    backend);
}

// 两段三次 Bezier 共享 P3，0.5 处 C0 连接（重数 3）。
Bspline two_segment_c0(BackendKind backend) {
  return require_curve_with_backend(
      {{0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0},
       {5.0, 2.0}, {7.0, 2.0}, {8.0, 0.0}},
      {0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0}, backend);
}

// 两段独立三次 Bezier，0.5 处位置间断（重数 4）。
Bspline two_segment_gap(BackendKind backend) {
  return require_curve_with_backend(
      {{0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0},
       {5.0, 2.0}, {7.0, 2.0}, {8.0, 0.0}, {9.0, -3.0}},
      {0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0},
      backend);
}

const std::vector<double>& comparison_parameters() {
  static const std::vector<double> parameters = {
      0.0,          1e-12,        0.125, 0.25,
      0.5 - 1e-9,   0.5,          0.5 + 1e-9,
      0.75,         1.0 - 1e-9,   1.0};
  return parameters;
}

void expect_backends_agree(const Bspline& baseline, const Bspline& candidate,
                           double tolerance) {
  ASSERT_EQ(baseline.backend(), BackendKind::kTinyspline);
  ASSERT_EQ(candidate.backend(), BackendKind::kEigenSplines);
  for (const double parameter : comparison_parameters()) {
    for (int order = 0; order <= 3; ++order) {
      SCOPED_TRACE("parameter=" + std::to_string(parameter) +
                   " order=" + std::to_string(order));
      const EvaluationResult expected = baseline.evaluate(parameter, order);
      const EvaluationResult actual = candidate.evaluate(parameter, order);
      ASSERT_EQ(expected.status, Status::kSuccess);
      ASSERT_EQ(actual.status, Status::kSuccess);
      EXPECT_NEAR(actual.value.x_m, expected.value.x_m, tolerance);
      EXPECT_NEAR(actual.value.y_m, expected.value.y_m, tolerance);
    }
  }
}

TEST(BsplineBackends, CubicBezierValuesAndDerivativesAgree) {
  expect_backends_agree(cubic_bezier(BackendKind::kTinyspline),
                        cubic_bezier(BackendKind::kEigenSplines), 1e-9);
}

TEST(BsplineBackends, C0CurvesAgree) {
  expect_backends_agree(two_segment_c0(BackendKind::kTinyspline),
                        two_segment_c0(BackendKind::kEigenSplines), 1e-9);
}

TEST(BsplineBackends, GapCurvesUseRightSegmentOnBothBackends) {
  expect_backends_agree(two_segment_gap(BackendKind::kTinyspline),
                        two_segment_gap(BackendKind::kEigenSplines), 1e-9);
}

TEST(BsplineBackends, EigenMatchesIndependentBezierReference) {
  const std::vector<Point2d> control_points = {
      {0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0}};
  const Bspline curve = cubic_bezier(BackendKind::kEigenSplines);
  for (const double t : comparison_parameters()) {
    SCOPED_TRACE("t=" + std::to_string(t));
    const EvaluationResult value = curve.evaluate(t, 0);
    ASSERT_EQ(value.status, Status::kSuccess);
    EXPECT_TRUE(test::near_point(value.value,
                                 test::bezier_value(control_points, t),
                                 1e-10, 1e-10));
    for (int order = 1; order <= 3; ++order) {
      const EvaluationResult derivative = curve.evaluate(t, order);
      ASSERT_EQ(derivative.status, Status::kSuccess);
      const Point2d expected =
          test::bezier_derivative(control_points, t, order, 1.0);
      EXPECT_NEAR(derivative.value.x_m, expected.x_m, 1e-9);
      EXPECT_NEAR(derivative.value.y_m, expected.y_m, 1e-9);
    }
  }
}

TEST(BsplineBackends, EigenGapKnotUsesRightSegment) {
  const Bspline curve = two_segment_gap(BackendKind::kEigenSplines);
  // 右侧分段起点 (5, 2)；该分段参数跨度为 0.5，
  // 参数域一阶导数 = 3 / 0.5 * (7-5, 2-2) = (12, 0)。
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 0).value, (Point2d{5.0, 2.0}), 1e-12);
  EXPECT_POINT_NEAR(curve.evaluate(0.5, 1).value, (Point2d{12.0, 0.0}), 1e-9);
  // 域末端取最后分段终点。
  EXPECT_POINT_NEAR(curve.evaluate(1.0, 0).value, (Point2d{9.0, -3.0}), 1e-12);
}

TEST(BsplineBackends, EigenRejectsOutOfDomainParameters) {
  const Bspline curve = cubic_bezier(BackendKind::kEigenSplines);
  EXPECT_EQ(curve.evaluate(-1e-6, 0).status,
            Status::kParameterOutOfDomain);
  EXPECT_EQ(curve.evaluate(1.0 + 1e-6, 0).status,
            Status::kParameterOutOfDomain);
  // 舍入容差内的边界值被夹到域端点。
  EXPECT_EQ(curve.evaluate(1.0 + 1e-13, 0).status, Status::kSuccess);
  EXPECT_EQ(curve.evaluate(-1e-13, 0).status, Status::kSuccess);
}

TEST(BsplineBackends, EigenSampleMatchesSingleEvaluation) {
  const Bspline curve = two_segment_c0(BackendKind::kEigenSplines);
  const std::vector<double>& parameters = comparison_parameters();
  for (int order = 0; order <= 3; ++order) {
    SCOPED_TRACE("order=" + std::to_string(order));
    std::vector<Point2d> buffer;
    ASSERT_EQ(curve.sample(parameters, buffer, order), Status::kSuccess);
    ASSERT_EQ(buffer.size(), parameters.size());
    for (std::size_t i = 0; i < parameters.size(); ++i) {
      const EvaluationResult single = curve.evaluate(parameters[i], order);
      ASSERT_EQ(single.status, Status::kSuccess);
      EXPECT_NEAR(buffer[i].x_m, single.value.x_m, 1e-12);
      EXPECT_NEAR(buffer[i].y_m, single.value.y_m, 1e-12);
    }
  }
}

TEST(BsplineBackends, EigenTimeParameterizedDerivativesMatchBaseline) {
  const std::vector<Point2d> control_points = {
      {0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0}};
  const std::vector<double> knots = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
  std::string message;
  std::optional<Bspline> baseline = Bspline::create_time_parameterized(
      control_points, knots, 2.0, 6.0, BackendKind::kTinyspline, &message);
  ASSERT_TRUE(baseline.has_value()) << message;
  std::optional<Bspline> candidate = Bspline::create_time_parameterized(
      control_points, knots, 2.0, 6.0, BackendKind::kEigenSplines, &message);
  ASSERT_TRUE(candidate.has_value()) << message;
  for (const double time : {2.0, 2.5, 4.0, 5.5, 6.0}) {
    for (int order = 0; order <= 2; ++order) {
      const EvaluationResult a = baseline->evaluate(time, order);
      const EvaluationResult b = candidate->evaluate(time, order);
      ASSERT_EQ(a.status, Status::kSuccess);
      ASSERT_EQ(b.status, Status::kSuccess);
      EXPECT_NEAR(b.value.x_m, a.value.x_m, 1e-9);
      EXPECT_NEAR(b.value.y_m, a.value.y_m, 1e-9);
    }
  }
}

TEST(BsplineBackends, EigenArcLengthMatchesBaselineAndTolerance) {
  const Bspline tiny = two_segment_c0(BackendKind::kTinyspline);
  const Bspline eigen = two_segment_c0(BackendKind::kEigenSplines);
  EXPECT_NEAR(eigen.total_length_m(), tiny.total_length_m(), 1e-6);

  const ArcLengthResampleResult tiny_result =
      tiny.sample_equal_arc_length(101);
  const ArcLengthResampleResult eigen_result =
      eigen.sample_equal_arc_length(101);
  ASSERT_EQ(tiny_result.status, Status::kSuccess);
  ASSERT_EQ(eigen_result.status, Status::kSuccess);
  EXPECT_LT(eigen_result.max_interval_error_m, 1e-3);
  ASSERT_EQ(eigen_result.samples.size(), tiny_result.samples.size());
  for (std::size_t i = 0; i < eigen_result.samples.size(); ++i) {
    EXPECT_NEAR(eigen_result.samples[i].position.x_m,
                tiny_result.samples[i].position.x_m, 1e-6);
    EXPECT_NEAR(eigen_result.samples[i].position.y_m,
                tiny_result.samples[i].position.y_m, 1e-6);
    EXPECT_NEAR(eigen_result.samples[i].arc_length_m,
                tiny_result.samples[i].arc_length_m, 1e-6);
  }
}

TEST(BsplineBackends, EigenConstructionRejectionMatchesBaseline) {
  const std::vector<Point2d> control_points = {{0.0, 0.0}, {1.0, 0.0}};
  const std::vector<double> too_short = {0.0, 1.0};
  std::string tiny_message;
  std::string eigen_message;
  EXPECT_FALSE(Bspline::create(control_points, too_short,
                               BackendKind::kTinyspline, &tiny_message)
                   .has_value());
  EXPECT_FALSE(Bspline::create(control_points, too_short,
                               BackendKind::kEigenSplines, &eigen_message)
                   .has_value());
  EXPECT_EQ(tiny_message, eigen_message);
}

TEST(BsplineBackends, DefaultBackendIsReported) {
  const Bspline curve = cubic_bezier(BackendKind::kTinyspline);
  EXPECT_EQ(curve.backend(), BackendKind::kTinyspline);
  EXPECT_STREQ(curve.backend_name(), "tinyspline");
  const Bspline eigen = cubic_bezier(BackendKind::kEigenSplines);
  EXPECT_EQ(eigen.backend(), BackendKind::kEigenSplines);
  EXPECT_STREQ(eigen.backend_name(), "eigen_splines");
  EXPECT_EQ(kDefaultBackend, BackendKind::kTinyspline);
}

}  // namespace
}  // namespace planner::bspline
