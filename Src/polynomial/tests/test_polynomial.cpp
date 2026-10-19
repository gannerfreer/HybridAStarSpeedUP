// 一维系数容器、三次/五次边界求解器、Horner 求值、解析导数和
// 时间缩放的单元测试。
//
// 数值比较遵循第 7.1 节：绝对/相对容差 1e-6
// （tolerance = 1e-6 + 1e-6 * |expected|）；闭式解校验使用夹具算例
// 和额外的手推案例。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "planner/polynomial/polynomial.hpp"
#include "planner/polynomial/result.hpp"

namespace planner::polynomial {
namespace {

constexpr double kAbsTol = 1e-6;
constexpr double kRelTol = 1e-6;

void expect_close(double actual, double expected) {
  EXPECT_NEAR(actual, expected, kAbsTol + kRelTol * std::fabs(expected))
      << "expected: " << expected << " actual: " << actual;
}

void expect_coefficients_close(const std::vector<double>& actual,
                               const std::vector<double>& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    SCOPED_TRACE(::testing::Message() << "coefficient index " << i);
    expect_close(actual[i], expected[i]);
  }
}

TEST(PolynomialCoefficients, stores_ascending_and_reports_degree) {
  const std::vector<double> coefficients{1.0, -2.0, 0.5, 3.0};
  const Polynomial polynomial(coefficients);
  EXPECT_EQ(polynomial.degree(), 3u);
  EXPECT_EQ(polynomial.coefficients(), coefficients);
}

TEST(PolynomialCoefficients, empty_input_becomes_zero_polynomial) {
  const Polynomial zero((std::vector<double>{}));
  EXPECT_EQ(zero.degree(), 0u);
  EXPECT_EQ(zero.coefficients().size(), 1u);
  EXPECT_EQ(zero.evaluate(123.0), 0.0);
  const Result<Polynomial> derivative = zero.derivative(1);
  ASSERT_TRUE(derivative.ok());
  EXPECT_EQ(derivative.value.evaluate(7.0), 0.0);
}

TEST(PolynomialCubic, matches_closed_form_worked_example_from_fixture) {
  // tests/fixtures/v1/benchmarks/polynomial/inputs.json：
  //   p0=0, v0=1, p1=1, v1=0 -> p(u) = u + u^2 - u^3.
  const Result<Polynomial> solved =
      Polynomial::cubic(CubicBoundary{0.0, 1.0, 1.0, 0.0}, 1.0);
  ASSERT_TRUE(solved.ok());
  expect_coefficients_close(solved.value.coefficients(), {0.0, 1.0, 1.0, -1.0});
}

TEST(PolynomialCubic, matches_closed_form_for_general_duration) {
  // [0, 2] 上 p(t) = t - t^2/4：p(0)=0, p'(0)=1, p(2)=1, p'(2)=0。
  const Result<Polynomial> solved =
      Polynomial::cubic(CubicBoundary{0.0, 1.0, 1.0, 0.0}, 2.0);
  ASSERT_TRUE(solved.ok());
  expect_coefficients_close(solved.value.coefficients(), {0.0, 1.0, -0.25, 0.0});
  expect_close(solved.value.evaluate(2.0), 1.0);
  expect_close(solved.value.evaluate_derivative(2.0, 1).value, 0.0);
}

TEST(PolynomialCubic, satisfies_boundary_conditions_and_residuals) {
  const CubicBoundary boundary{2.5, -1.25, -3.75, 4.5};
  const Result<Polynomial> solved = Polynomial::cubic(boundary, 3.5);
  ASSERT_TRUE(solved.ok());
  const Polynomial& p = solved.value;
  expect_close(p.evaluate(0.0), boundary.p0_m);
  expect_close(p.evaluate_derivative(0.0, 1).value, boundary.v0_mps);
  expect_close(p.evaluate(3.5), boundary.p1_m);
  expect_close(p.evaluate_derivative(3.5, 1).value, boundary.v1_mps);
}

TEST(PolynomialCubic, constant_velocity_reduces_to_affine_polynomial) {
  const Result<Polynomial> solved =
      Polynomial::cubic(CubicBoundary{1.0, 2.0, 7.0, 2.0}, 3.0);
  ASSERT_TRUE(solved.ok());
  expect_coefficients_close(solved.value.coefficients(), {1.0, 2.0, 0.0, 0.0});
}

TEST(PolynomialQuintic, matches_smoothstep_worked_example_from_fixture) {
  // p0=0, v0=0, a0=0, p1=1, v1=0, a1=0 -> 10u^3 - 15u^4 + 6u^5。
  const Result<Polynomial> solved =
      Polynomial::quintic(QuinticBoundary{0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, 1.0);
  ASSERT_TRUE(solved.ok());
  expect_coefficients_close(solved.value.coefficients(), {0.0, 0.0, 0.0, 10.0, -15.0, 6.0});
}

TEST(PolynomialQuintic, matches_general_duration_scaling_of_smoothstep) {
  // smoothstep 拉伸到 T=4 后系数为 10/T^3, -15/T^4, 6/T^5。
  const Result<Polynomial> solved =
      Polynomial::quintic(QuinticBoundary{0.0, 0.0, 0.0, 3.0, 0.0, 0.0}, 4.0);
  ASSERT_TRUE(solved.ok());
  const double t = 4.0;
  expect_coefficients_close(solved.value.coefficients(),
                            {0.0, 0.0, 0.0, 30.0 / (t * t * t), -45.0 / (t * t * t * t),
                             18.0 / (t * t * t * t * t)});
}

TEST(PolynomialQuintic, matches_closed_form_with_nonzero_end_acceleration) {
  // [0,1] 上 p0=v0=a0=0, p1=v1=0, a1=1 -> 0.5t^3 - t^4 + 0.5t^5。
  const Result<Polynomial> solved =
      Polynomial::quintic(QuinticBoundary{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, 1.0);
  ASSERT_TRUE(solved.ok());
  expect_coefficients_close(solved.value.coefficients(), {0.0, 0.0, 0.0, 0.5, -1.0, 0.5});
  expect_close(solved.value.evaluate_derivative(1.0, 1).value, 0.0);
  expect_close(solved.value.evaluate_derivative(1.0, 2).value, 1.0);
}

TEST(PolynomialQuintic, satisfies_position_velocity_acceleration_boundaries) {
  const QuinticBoundary boundary{1.5, 0.75, -0.5, -2.25, 1.5, 0.25};
  const Result<Polynomial> solved = Polynomial::quintic(boundary, 2.5);
  ASSERT_TRUE(solved.ok());
  const Polynomial& p = solved.value;
  expect_close(p.evaluate(0.0), boundary.p0_m);
  expect_close(p.evaluate_derivative(0.0, 1).value, boundary.v0_mps);
  expect_close(p.evaluate_derivative(0.0, 2).value, boundary.a0_mps2);
  expect_close(p.evaluate(2.5), boundary.p1_m);
  expect_close(p.evaluate_derivative(2.5, 1).value, boundary.v1_mps);
  expect_close(p.evaluate_derivative(2.5, 2).value, boundary.a1_mps2);
}

TEST(PolynomialEvaluation, horner_matches_naive_power_sum) {
  const Polynomial polynomial(std::vector<double>{1.0, -2.0, 0.5, 3.0, -0.25});
  const double times[] = {-2.0, -0.5, 0.0, 0.25, 1.0, 3.5};
  for (double t : times) {
    double naive = 0.0;
    double power = 1.0;
    for (double coefficient : polynomial.coefficients()) {
      naive += coefficient * power;
      power *= t;
    }
    expect_close(polynomial.evaluate(t), naive);
  }
}

TEST(PolynomialDerivative, analytic_derivative_coefficients) {
  const Polynomial polynomial(std::vector<double>{0.0, 1.0, 1.0, -1.0});  // u + u^2 - u^3
  const Result<Polynomial> first = polynomial.derivative(1);
  ASSERT_TRUE(first.ok());
  expect_coefficients_close(first.value.coefficients(), {1.0, 2.0, -3.0});
  const Result<Polynomial> second = first.value.derivative(1);
  ASSERT_TRUE(second.ok());
  expect_coefficients_close(second.value.coefficients(), {2.0, -6.0});
  const Result<Polynomial> third = second.value.derivative(1);
  ASSERT_TRUE(third.ok());
  expect_coefficients_close(third.value.coefficients(), {-6.0});
}

TEST(PolynomialDerivative, evaluate_derivative_matches_derivative_polynomial) {
  const Polynomial polynomial(std::vector<double>{2.0, -1.0, 0.5, 4.0, -2.0, 1.0});
  const double times[] = {-1.5, 0.0, 0.5, 2.0};
  for (int order = 0; order <= 3; ++order) {
    const Result<Polynomial> derived = polynomial.derivative(order);
    ASSERT_TRUE(derived.ok());
    for (double t : times) {
      expect_close(polynomial.evaluate_derivative(t, order).value, derived.value.evaluate(t));
    }
  }
}

TEST(PolynomialDerivative, higher_order_derivatives_of_low_degree_are_zero) {
  const Polynomial quadratic(std::vector<double>{1.0, 2.0, 3.0});
  const Result<double> third = quadratic.evaluate_derivative(1.0, 3);
  ASSERT_TRUE(third.ok());
  EXPECT_EQ(third.value, 0.0);
}

TEST(PolynomialDerivative, rejects_unsupported_and_negative_orders) {
  const Polynomial polynomial(std::vector<double>{1.0, 2.0});
  EXPECT_EQ(polynomial.derivative(4).code, Code::kUnsupportedDerivative);
  EXPECT_EQ(polynomial.derivative(-1).code, Code::kInvalidInput);
  EXPECT_EQ(polynomial.evaluate_derivative(0.0, 4).code, Code::kUnsupportedDerivative);
  EXPECT_EQ(polynomial.evaluate_derivative(0.0, -2).code, Code::kInvalidInput);
}

TEST(PolynomialDerivative, rejects_non_finite_time) {
  const Polynomial polynomial(std::vector<double>{1.0, 2.0});
  const double nan = std::nan("");
  EXPECT_EQ(polynomial.evaluate_derivative(nan, 0).code, Code::kInvalidInput);
  EXPECT_EQ(polynomial.evaluate_derivative(0.0, 0).code, Code::kOk);
}

TEST(PolynomialBatch, preallocated_buffer_matches_single_evaluation) {
  const Polynomial polynomial(std::vector<double>{1.0, -2.0, 0.5, 3.0});
  const double times[] = {0.0, 0.1, 0.25, 0.5, 1.0, 2.5};
  double output[6] = {};
  const BatchResult result = polynomial.evaluate_batch(times, 6, 2, output);
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.processed, 6u);
  for (std::size_t i = 0; i < 6; ++i) {
    expect_close(output[i], polynomial.evaluate_derivative(times[i], 2).value);
  }
}

TEST(PolynomialBatch, stops_at_first_non_finite_time) {
  const Polynomial polynomial(std::vector<double>{1.0, 2.0});
  const double times[] = {0.0, 1.0, std::nan(""), 2.0};
  double output[4] = {};
  const BatchResult result = polynomial.evaluate_batch(times, 4, 0, output);
  EXPECT_EQ(result.code, Code::kInvalidInput);
  EXPECT_EQ(result.processed, 2u);
  EXPECT_EQ(result.first_bad_index, 2u);
  EXPECT_EQ(output[0], 1.0);
  EXPECT_EQ(output[1], 3.0);
}

TEST(PolynomialBatch, rejects_null_buffer_and_accepts_empty_count) {
  const Polynomial polynomial(std::vector<double>{1.0});
  const double time = 0.5;
  double output = 0.0;
  EXPECT_EQ(polynomial.evaluate_batch(nullptr, 1, 0, &output).code, Code::kInvalidInput);
  EXPECT_EQ(polynomial.evaluate_batch(&time, 1, 0, nullptr).code, Code::kInvalidInput);
  const BatchResult empty = polynomial.evaluate_batch(nullptr, 0, 0, nullptr);
  EXPECT_TRUE(empty.ok());
  EXPECT_EQ(empty.processed, 0u);
}

TEST(PolynomialTimeScaling, preserves_values_at_scaled_times) {
  const Result<Polynomial> solved =
      Polynomial::quintic(QuinticBoundary{0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, 1.0);
  ASSERT_TRUE(solved.ok());
  const Result<Polynomial> scaled = solved.value.scaled_time(2.5);
  ASSERT_TRUE(scaled.ok());
  const double times[] = {0.0, 1.25, 5.0};
  for (double t : times) {
    expect_close(scaled.value.evaluate(t), solved.value.evaluate(t / 2.5));
    expect_close(scaled.value.evaluate_derivative(t, 1).value,
                 solved.value.evaluate_derivative(t / 2.5, 1).value / 2.5);
    expect_close(scaled.value.evaluate_derivative(t, 2).value,
                 solved.value.evaluate_derivative(t / 2.5, 2).value / (2.5 * 2.5));
  }
}

TEST(PolynomialTimeScaling, rejects_non_positive_and_non_finite_scale) {
  const Polynomial polynomial(std::vector<double>{1.0, 2.0});
  EXPECT_EQ(polynomial.scaled_time(0.0).code, Code::kInvalidInput);
  EXPECT_EQ(polynomial.scaled_time(-1.0).code, Code::kInvalidInput);
  EXPECT_EQ(polynomial.scaled_time(std::nan("")).code, Code::kInvalidInput);
  EXPECT_EQ(polynomial.scaled_time(std::numeric_limits<double>::infinity()).code,
            Code::kInvalidInput);
}

TEST(PolynomialSolvers, reject_non_positive_or_non_finite_duration) {
  EXPECT_EQ(Polynomial::cubic(CubicBoundary{0.0, 1.0, 1.0, 0.0}, 0.0).code, Code::kInvalidInput);
  EXPECT_EQ(Polynomial::cubic(CubicBoundary{0.0, 1.0, 1.0, 0.0}, -2.0).code, Code::kInvalidInput);
  EXPECT_EQ(Polynomial::cubic(CubicBoundary{0.0, 1.0, 1.0, 0.0}, std::nan("")).code,
            Code::kInvalidInput);
  EXPECT_EQ(Polynomial::quintic(QuinticBoundary{0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, 0.0).code,
            Code::kInvalidInput);
  EXPECT_EQ(
      Polynomial::quintic(QuinticBoundary{0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                          std::numeric_limits<double>::infinity())
          .code,
      Code::kInvalidInput);
}

TEST(PolynomialSolvers, reject_non_finite_boundary_values) {
  const double nan = std::nan("");
  EXPECT_EQ(Polynomial::cubic(CubicBoundary{nan, 1.0, 1.0, 0.0}, 1.0).code,
            Code::kInvalidInput);
  EXPECT_EQ(Polynomial::cubic(CubicBoundary{0.0, 1.0, 1.0, std::numeric_limits<double>::infinity()},
                              1.0)
                .code,
            Code::kInvalidInput);
  EXPECT_EQ(Polynomial::quintic(QuinticBoundary{0.0, 0.0, nan, 1.0, 0.0, 0.0}, 1.0).code,
            Code::kInvalidInput);
}

TEST(PolynomialStatus, maps_to_common_status_and_stable_names) {
  EXPECT_STREQ(to_string(Code::kOk), "ok");
  EXPECT_STREQ(to_string(Code::kOutOfRange), "out_of_range");
  EXPECT_STREQ(to_string(Code::kUnsupportedDerivative), "unsupported_derivative");
  EXPECT_STREQ(to_string(Code::kInvalidInput), "invalid_input");
  EXPECT_EQ(to_common_status(Code::kOk), planner::common::Status::kSuccess);
  EXPECT_EQ(to_common_status(Code::kOutOfRange), planner::common::Status::kInvalidInput);
  EXPECT_EQ(to_common_status(Code::kUnsupportedDerivative),
            planner::common::Status::kInvalidInput);
  EXPECT_EQ(to_common_status(Code::kInvalidInput), planner::common::Status::kInvalidInput);
}

}  // namespace
}  // namespace planner::polynomial
