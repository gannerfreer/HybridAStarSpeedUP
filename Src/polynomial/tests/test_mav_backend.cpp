// 多项式内部对照测试：
//   * planner_polynomial（闭式边界求解 + Horner 求值）与
//     mav_trajectory_generation 求值核心（计划 §3.4 的“分段多项式表达、
//     求值与连续性处理参考”）在共享系数上的数值一致性；
//   * 与独立数值参考的对照：测试内用带主元的高斯消元独立求解三次/五次
//     边界系统（对应 PythonRobotics 五次公式“公式与结果参照”的角色）。

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mav_segment_evaluator.hpp"
#include "planner/polynomial/polynomial.hpp"
#include "planner/polynomial/trajectory_segment.hpp"

namespace planner::polynomial {
namespace {

using common::Point2d;
using compare::MavSegmentEvaluator;

double factorial_ratio(int j, int r) {
  // j! / (j - r)!
  double value = 1.0;
  for (int i = 0; i < r; ++i) {
    value *= static_cast<double>(j - i);
  }
  return value;
}

// 带主元的高斯消元（测试内独立实现，不复用生产闭式公式）。
std::vector<double> solve_linear_system(
    std::vector<std::vector<double>> matrix, std::vector<double> rhs) {
  const std::size_t n = rhs.size();
  for (std::size_t col = 0; col < n; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < n; ++row) {
      if (std::fabs(matrix[row][col]) > std::fabs(matrix[pivot][col])) {
        pivot = row;
      }
    }
    std::swap(matrix[pivot], matrix[col]);
    std::swap(rhs[pivot], rhs[col]);
    const double diagonal = matrix[col][col];
    for (std::size_t c = col; c < n; ++c) {
      matrix[col][c] /= diagonal;
    }
    rhs[col] /= diagonal;
    for (std::size_t row = 0; row < n; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = matrix[row][col];
      for (std::size_t c = col; c < n; ++c) {
        matrix[row][c] -= factor * matrix[col][c];
      }
      rhs[row] -= factor * rhs[col];
    }
  }
  return rhs;
}

std::vector<double> cubic_reference(const CubicBoundary& boundary,
                                    double duration_s) {
  std::vector<std::vector<double>> matrix(4, std::vector<double>(4, 0.0));
  std::vector<double> rhs(4);
  matrix[0][0] = 1.0;
  rhs[0] = boundary.p0_m;
  matrix[1][1] = 1.0;
  rhs[1] = boundary.v0_mps;
  for (int j = 0; j < 4; ++j) {
    matrix[2][static_cast<std::size_t>(j)] = std::pow(duration_s, j);
    matrix[3][static_cast<std::size_t>(j)] =
        j == 0 ? 0.0 : j * std::pow(duration_s, j - 1);
  }
  rhs[2] = boundary.p1_m;
  rhs[3] = boundary.v1_mps;
  return solve_linear_system(std::move(matrix), std::move(rhs));
}

std::vector<double> quintic_reference(const QuinticBoundary& boundary,
                                      double duration_s) {
  std::vector<std::vector<double>> matrix(6, std::vector<double>(6, 0.0));
  std::vector<double> rhs(6);
  for (int r = 0; r < 3; ++r) {
    matrix[static_cast<std::size_t>(r)][static_cast<std::size_t>(r)] =
        factorial_ratio(r, r);
  }
  rhs[0] = boundary.p0_m;
  rhs[1] = boundary.v0_mps;
  rhs[2] = boundary.a0_mps2;
  for (int j = 0; j < 6; ++j) {
    matrix[3][static_cast<std::size_t>(j)] = std::pow(duration_s, j);
    matrix[4][static_cast<std::size_t>(j)] =
        j == 0 ? 0.0 : factorial_ratio(j, 1) * std::pow(duration_s, j - 1);
    matrix[5][static_cast<std::size_t>(j)] =
        j < 2 ? 0.0 : factorial_ratio(j, 2) * std::pow(duration_s, j - 2);
  }
  rhs[3] = boundary.p1_m;
  rhs[4] = boundary.v1_mps;
  rhs[5] = boundary.a1_mps2;
  return solve_linear_system(std::move(matrix), std::move(rhs));
}

CubicBoundary cubic_boundary(std::size_t k) {
  const double kd = static_cast<double>(k);
  return CubicBoundary{0.5 * kd, 1.0, 0.5 * kd + 1.0 + 0.1 * kd, 0.75};
}

QuinticBoundary quintic_boundary(std::size_t k) {
  const double kd = static_cast<double>(k);
  return QuinticBoundary{0.5 * kd, 0.25, 0.5, 0.5 * kd + 2.0, -0.5,
                         0.25 * kd - 0.5};
}

void expect_candidate_matches_segment(const TrajectorySegment& segment,
                                      const MavSegmentEvaluator& evaluator,
                                      double tolerance) {
  const double duration = segment.duration_s();
  ASSERT_EQ(evaluator.valid(), true);
  for (const double fraction : {0.0, 0.1, 0.5, 0.9, 1.0}) {
    const double t = fraction * duration;
    for (int order = 0; order <= 3; ++order) {
      SCOPED_TRACE("t=" + std::to_string(t) + " order=" + std::to_string(order));
      const Result<Point2d> expected = segment.evaluate(t, order);
      ASSERT_TRUE(expected.ok());
      Point2d actual;
      ASSERT_TRUE(evaluator.evaluate(t, order, &actual));
      EXPECT_NEAR(actual.x_m, expected.value.x_m, tolerance);
      EXPECT_NEAR(actual.y_m, expected.value.y_m, tolerance);
    }
  }
}

TEST(PolynomialMavBackend, CubicEvaluationMatchesBaseline) {
  for (std::size_t k = 0; k < 8; ++k) {
    const CubicBoundary2d boundary{
        Point2d{cubic_boundary(k).p0_m, -0.25 * static_cast<double>(k)},
        Point2d{1.0, 0.5},
        Point2d{cubic_boundary(k).p1_m, 0.25},
        Point2d{0.75, -0.5}};
    const Result<TrajectorySegment> segment =
        TrajectorySegment::cubic(boundary, 1.0);
    ASSERT_TRUE(segment.ok());
    MavSegmentEvaluator evaluator;
    ASSERT_TRUE(MavSegmentEvaluator::create(
        segment.value.x_polynomial().coefficients(),
        segment.value.y_polynomial().coefficients(), segment.value.duration_s(),
        &evaluator));
    expect_candidate_matches_segment(segment.value, evaluator, 1e-9);
  }
}

TEST(PolynomialMavBackend, QuinticEvaluationMatchesBaseline) {
  for (std::size_t k = 0; k < 8; ++k) {
    const QuinticBoundary base = quintic_boundary(k);
    const QuinticBoundary2d boundary{
        Point2d{base.p0_m, -0.5 * static_cast<double>(k)},
        Point2d{base.v0_mps, 0.25},
        Point2d{base.a0_mps2, 0.0},
        Point2d{base.p1_m, 0.5},
        Point2d{base.v1_mps, -0.25},
        Point2d{base.a1_mps2, 0.0}};
    const Result<TrajectorySegment> segment =
        TrajectorySegment::quintic(boundary, 2.0);
    ASSERT_TRUE(segment.ok());
    MavSegmentEvaluator evaluator;
    ASSERT_TRUE(MavSegmentEvaluator::create(
        segment.value.x_polynomial().coefficients(),
        segment.value.y_polynomial().coefficients(), segment.value.duration_s(),
        &evaluator));
    expect_candidate_matches_segment(segment.value, evaluator, 1e-9);
  }
}

TEST(PolynomialMavBackend, BatchMatchesSingleEvaluation) {
  const QuinticBoundary base = quintic_boundary(3);
  const QuinticBoundary2d boundary{
      Point2d{base.p0_m, 0.0}, Point2d{base.v0_mps, 0.5},
      Point2d{base.a0_mps2, 0.0}, Point2d{base.p1_m, 0.0},
      Point2d{base.v1_mps, -0.5}, Point2d{base.a1_mps2, 0.0}};
  const Result<TrajectorySegment> segment = TrajectorySegment::quintic(boundary, 1.5);
  ASSERT_TRUE(segment.ok());
  MavSegmentEvaluator evaluator;
  ASSERT_TRUE(MavSegmentEvaluator::create(
      segment.value.x_polynomial().coefficients(),
      segment.value.y_polynomial().coefficients(), segment.value.duration_s(),
      &evaluator));
  const std::vector<double> times = {0.0, 0.25, 0.5, 1.0, 1.5};
  for (int order = 0; order <= 3; ++order) {
    std::vector<Point2d> buffer(times.size());
    ASSERT_TRUE(evaluator.evaluate_batch(times.data(), times.size(), order,
                                         buffer.data()));
    for (std::size_t i = 0; i < times.size(); ++i) {
      Point2d single;
      ASSERT_TRUE(evaluator.evaluate(times[i], order, &single));
      EXPECT_NEAR(buffer[i].x_m, single.x_m, 1e-12);
      EXPECT_NEAR(buffer[i].y_m, single.y_m, 1e-12);
    }
  }
}

TEST(PolynomialMavBackend, RejectsOutOfRangeAndUnsupportedOrder) {
  MavSegmentEvaluator evaluator;
  ASSERT_TRUE(MavSegmentEvaluator::create({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, 1.0,
                                          &evaluator));
  Point2d value;
  EXPECT_FALSE(evaluator.evaluate(-1e-9, 0, &value));
  EXPECT_FALSE(evaluator.evaluate(1.0 + 1e-9, 0, &value));
  EXPECT_FALSE(evaluator.evaluate(0.5, 4, &value));
  EXPECT_FALSE(evaluator.evaluate(0.5, -1, &value));
  EXPECT_TRUE(evaluator.evaluate(0.5, 3, &value));  // 次数之上为零向量
  EXPECT_NEAR(value.x_m, 0.0, 1e-12);
  EXPECT_FALSE(MavSegmentEvaluator::create({1.0}, {1.0}, 1.0, &evaluator));
  EXPECT_FALSE(MavSegmentEvaluator::create({1.0, 2.0}, {1.0, 2.0}, 0.0,
                                           &evaluator));
}

TEST(PolynomialSolver, QuinticMatchesIndependentGaussianSolve) {
  const QuinticBoundary boundary{1.0, 0.5, 0.2, 3.5, -0.25, -0.3};
  const double duration_s = 1.7;
  const Result<Polynomial> solved = Polynomial::quintic(boundary, duration_s);
  ASSERT_TRUE(solved.ok());
  const std::vector<double> reference =
      quintic_reference(boundary, duration_s);
  ASSERT_EQ(solved.value.coefficients().size(), reference.size());
  for (std::size_t i = 0; i < reference.size(); ++i) {
    EXPECT_NEAR(solved.value.coefficients()[i], reference[i],
                1e-9 * (1.0 + std::fabs(reference[i])));
  }
  // 解析参考的求值交叉验证。
  for (const double t : {0.0, 0.5, 1.0, 1.7}) {
    double expected = 0.0;
    for (std::size_t i = 0; i < reference.size(); ++i) {
      expected += reference[i] * std::pow(t, static_cast<double>(i));
    }
    EXPECT_NEAR(solved.value.evaluate(t), expected,
                1e-9 * (1.0 + std::fabs(expected)));
  }
}

TEST(PolynomialSolver, CubicMatchesIndependentGaussianSolve) {
  const CubicBoundary boundary{0.5, 1.25, 4.0, -0.5};
  const double duration_s = 0.8;
  const Result<Polynomial> solved = Polynomial::cubic(boundary, duration_s);
  ASSERT_TRUE(solved.ok());
  const std::vector<double> reference = cubic_reference(boundary, duration_s);
  ASSERT_EQ(solved.value.coefficients().size(), reference.size());
  for (std::size_t i = 0; i < reference.size(); ++i) {
    EXPECT_NEAR(solved.value.coefficients()[i], reference[i],
                1e-9 * (1.0 + std::fabs(reference[i])));
  }
}

}  // namespace
}  // namespace planner::polynomial
