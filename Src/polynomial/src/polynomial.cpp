// planner_polynomial 核心：系数容器、三次/五次边界求解器、
// Horner 求值、解析导数和时间缩放。

#include "planner/polynomial/polynomial.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace planner::polynomial {
namespace {

bool all_finite(const double* values, std::size_t count) noexcept {
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(values[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

const char* to_string(Code code) noexcept {
  switch (code) {
    case Code::kOk:
      return "ok";
    case Code::kOutOfRange:
      return "out_of_range";
    case Code::kUnsupportedDerivative:
      return "unsupported_derivative";
    case Code::kInvalidInput:
      return "invalid_input";
  }
  return "invalid_input";
}

planner::common::Status to_common_status(Code code) noexcept {
  switch (code) {
    case Code::kOk:
      return planner::common::Status::kSuccess;
    case Code::kOutOfRange:
    case Code::kUnsupportedDerivative:
    case Code::kInvalidInput:
      return planner::common::Status::kInvalidInput;
  }
  return planner::common::Status::kInvalidInput;
}

Polynomial::Polynomial(std::vector<double> coefficients_ascending) {
  if (coefficients_ascending.empty()) {
    coefficients_ascending.push_back(0.0);
  }
  coefficients_ = std::move(coefficients_ascending);
}

Result<Polynomial> Polynomial::cubic(const CubicBoundary& boundary, double duration_s) {
  if (!std::isfinite(duration_s) || duration_s <= 0.0) {
    return Result<Polynomial>::failure(Code::kInvalidInput);
  }
  const double values[] = {boundary.p0_m, boundary.v0_mps, boundary.p1_m, boundary.v1_mps};
  if (!all_finite(values, 4)) {
    return Result<Polynomial>::failure(Code::kInvalidInput);
  }

  // 闭式解（参见测试夹具算例
  // tests/fixtures/v1/benchmarks/polynomial/inputs.json）：
  //   c0 = p0, c1 = v0,
  //   c2 = (3*(p1-p0) - (2*v0 + v1)*T) / T^2,
  //   c3 = (2*(p0-p1) + (v0 + v1)*T) / T^3.
  const double t = duration_s;
  const double inv_t = 1.0 / t;
  const double inv_t2 = inv_t * inv_t;
  const double inv_t3 = inv_t2 * inv_t;
  const double delta_p = boundary.p1_m - boundary.p0_m;
  std::vector<double> coefficients(4);
  coefficients[0] = boundary.p0_m;
  coefficients[1] = boundary.v0_mps;
  coefficients[2] = (3.0 * delta_p - (2.0 * boundary.v0_mps + boundary.v1_mps) * t) * inv_t2;
  coefficients[3] =
      (2.0 * (boundary.p0_m - boundary.p1_m) + (boundary.v0_mps + boundary.v1_mps) * t) *
      inv_t3;
  if (!all_finite(coefficients.data(), coefficients.size())) {
    return Result<Polynomial>::failure(Code::kInvalidInput);
  }
  return Result<Polynomial>::success(Polynomial(std::move(coefficients)));
}

Result<Polynomial> Polynomial::quintic(const QuinticBoundary& boundary, double duration_s) {
  if (!std::isfinite(duration_s) || duration_s <= 0.0) {
    return Result<Polynomial>::failure(Code::kInvalidInput);
  }
  const double values[] = {boundary.p0_m, boundary.v0_mps, boundary.a0_mps2,
                           boundary.p1_m, boundary.v1_mps, boundary.a1_mps2};
  if (!all_finite(values, 6)) {
    return Result<Polynomial>::failure(Code::kInvalidInput);
  }

  // c0 = p0, c1 = v0, c2 = a0/2。对 c3..c5，把 [0, T] 上的残差方程组
  // 写为 V(T) * [c3, c4, c5]^T = [A, B, C]^T：
  //   A = p1 - p0 - v0*T - a0*T^2/2,
  //   B = v1 - v0 - a0*T,
  //   C = a1 - a0,
  // 其中 V(T) = [[T^3, T^4, T^5], [3T^2, 4T^3, 5T^4], [6T, 12T^2, 20T^3]]。
  // 按 T 的幂缩放可得单位时长逆解（已用 smoothstep 算例
  // 10u^3 - 15u^4 + 6u^5 校验）：以缩放未知量
  // x1 = c3, x2 = c4*T, x3 = c5*T^2 表示，T=1 的解为
  // x1 = 10*A - 4*B + 0.5*C, x2 = -15*A + 7*B - C, x3 = 6*A - 3*B + 0.5*C，
  // 再作用于 A/T^3, B/T^2, C/T，因此
  //   c3 =  10*A/T^3 - 4*B/T^2 + 0.5*C/T,
  //   c4 = -15*A/T^4 + 7*B/T^3 -     C/T^2,
  //   c5 =   6*A/T^5 - 3*B/T^4 + 0.5*C/T^3.
  const double t = duration_s;
  const double inv_t = 1.0 / t;
  const double inv_t2 = inv_t * inv_t;
  const double inv_t3 = inv_t2 * inv_t;
  const double inv_t4 = inv_t3 * inv_t;
  const double inv_t5 = inv_t4 * inv_t;
  const double a = boundary.p1_m - boundary.p0_m - boundary.v0_mps * t -
                   0.5 * boundary.a0_mps2 * t * t;
  const double b = boundary.v1_mps - boundary.v0_mps - boundary.a0_mps2 * t;
  const double c = boundary.a1_mps2 - boundary.a0_mps2;
  std::vector<double> coefficients(6);
  coefficients[0] = boundary.p0_m;
  coefficients[1] = boundary.v0_mps;
  coefficients[2] = 0.5 * boundary.a0_mps2;
  coefficients[3] = 10.0 * a * inv_t3 - 4.0 * b * inv_t2 + 0.5 * c * inv_t;
  coefficients[4] = -15.0 * a * inv_t4 + 7.0 * b * inv_t3 - c * inv_t2;
  coefficients[5] = 6.0 * a * inv_t5 - 3.0 * b * inv_t4 + 0.5 * c * inv_t3;
  if (!all_finite(coefficients.data(), coefficients.size())) {
    return Result<Polynomial>::failure(Code::kInvalidInput);
  }
  return Result<Polynomial>::success(Polynomial(std::move(coefficients)));
}

std::size_t Polynomial::degree() const noexcept {
  return coefficients_.empty() ? 0 : coefficients_.size() - 1;
}

const std::vector<double>& Polynomial::coefficients() const noexcept {
  return coefficients_;
}

bool Polynomial::is_finite() const noexcept {
  return all_finite(coefficients_.data(), coefficients_.size());
}

double Polynomial::evaluate(double t_s) const noexcept {
  // 对升幂系数使用 Horner 法则。
  double result = 0.0;
  for (std::size_t i = coefficients_.size(); i-- > 0;) {
    result = result * t_s + coefficients_[i];
  }
  return result;
}

Result<Polynomial> Polynomial::derivative(int order) const {
  if (order < 0) {
    return Result<Polynomial>::failure(Code::kInvalidInput);
  }
  if (order > kMaxDerivativeOrder) {
    return Result<Polynomial>::failure(Code::kUnsupportedDerivative);
  }
  if (order == 0) {
    return Result<Polynomial>::success(*this);
  }
  if (static_cast<std::size_t>(order) >= coefficients_.size()) {
    return Result<Polynomial>::success(Polynomial(std::vector<double>{0.0}));
  }
  const std::size_t count = coefficients_.size() - static_cast<std::size_t>(order);
  std::vector<double> derived(count);
  for (std::size_t i = 0; i < count; ++i) {
    // (i + order)! / i! = j 从 1 到 order 的 (i + j) 连乘。
    double factor = 1.0;
    for (int j = 1; j <= order; ++j) {
      factor *= static_cast<double>(i) + static_cast<double>(j);
    }
    derived[i] = coefficients_[i + static_cast<std::size_t>(order)] * factor;
  }
  if (!all_finite(derived.data(), derived.size())) {
    return Result<Polynomial>::failure(Code::kInvalidInput);
  }
  return Result<Polynomial>::success(Polynomial(std::move(derived)));
}

Result<double> Polynomial::evaluate_derivative(double t_s, int order) const noexcept {
  if (!std::isfinite(t_s) || order < 0) {
    return Result<double>::failure(Code::kInvalidInput);
  }
  if (order > kMaxDerivativeOrder) {
    return Result<double>::failure(Code::kUnsupportedDerivative);
  }
  if (static_cast<std::size_t>(order) >= coefficients_.size()) {
    return Result<double>::success(0.0);
  }
  // 对解析导数系数做 Horner 求值，系数即时计算（无分配）：
  // d_j = (j+order)!/j! * c_{j+order}。从最高系数向下迭代 i 时，
  // j = i - order，且 (j+order)!/j! = (i-order+1) * ... * i。
  double result = 0.0;
  for (std::size_t i = coefficients_.size(); i-- > static_cast<std::size_t>(order);) {
    double factor = 1.0;
    for (int r = 1; r <= order; ++r) {
      factor *= static_cast<double>(i) - static_cast<double>(order) + static_cast<double>(r);
    }
    result = result * t_s + coefficients_[i] * factor;
  }
  return Result<double>::success(result);
}

BatchResult Polynomial::evaluate_batch(const double* times_s, std::size_t count,
                                       int derivative_order, double* output) const noexcept {
  BatchResult result;
  if (count == 0) {
    return result;
  }
  if (times_s == nullptr || output == nullptr) {
    result.code = Code::kInvalidInput;
    return result;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const Result<double> value = evaluate_derivative(times_s[i], derivative_order);
    if (!value.ok()) {
      result.code = value.code;
      result.first_bad_index = i;
      return result;
    }
    output[i] = value.value;
    result.processed = i + 1;
  }
  return result;
}

Result<Polynomial> Polynomial::scaled_time(double time_scale) const {
  if (!std::isfinite(time_scale) || time_scale <= 0.0) {
    return Result<Polynomial>::failure(Code::kInvalidInput);
  }
  std::vector<double> scaled(coefficients_.size());
  double factor = 1.0;
  for (std::size_t i = 0; i < coefficients_.size(); ++i) {
    scaled[i] = coefficients_[i] / factor;
    factor *= time_scale;
  }
  if (!all_finite(scaled.data(), scaled.size())) {
    return Result<Polynomial>::failure(Code::kInvalidInput);
  }
  return Result<Polynomial>::success(Polynomial(std::move(scaled)));
}

}  // namespace planner::polynomial
