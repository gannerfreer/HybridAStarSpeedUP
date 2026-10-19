// MavSegmentEvaluator 实现：包装 mav_trajectory_generation::Polynomial
// 的逐点求值（内部对照候选；上游类型只出现在本文件）。

#include "mav_segment_evaluator.hpp"

#include <cmath>
#include <utility>

#include <mav_trajectory_generation/polynomial.h>

namespace planner::polynomial::compare {

namespace {

// mav_trajectory_generation 的 CHECK 宏要求 glog；保持上游表达式不变，
// 仅在此处把系数向量转换为上游类型。
mav_trajectory_generation::Polynomial to_mav_polynomial(
    const std::vector<double>& coefficients_ascending) {
  Eigen::VectorXd coefficients(
      static_cast<Eigen::Index>(coefficients_ascending.size()));
  for (std::size_t i = 0; i < coefficients_ascending.size(); ++i) {
    coefficients(static_cast<Eigen::Index>(i)) = coefficients_ascending[i];
  }
  return mav_trajectory_generation::Polynomial(
      static_cast<int>(coefficients_ascending.size()), coefficients);
}

bool is_finite_vector(const std::vector<double>& values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

}  // namespace

struct MavSegmentEvaluator::Impl {
  Impl(mav_trajectory_generation::Polynomial x_polynomial,
       mav_trajectory_generation::Polynomial y_polynomial, double duration,
       int count)
      : x(std::move(x_polynomial)),
        y(std::move(y_polynomial)),
        duration_s(duration),
        coefficient_count(count) {}

  mav_trajectory_generation::Polynomial x;
  mav_trajectory_generation::Polynomial y;
  double duration_s{0.0};
  int coefficient_count{0};
};

bool MavSegmentEvaluator::create(
    const std::vector<double>& x_coefficients_ascending,
    const std::vector<double>& y_coefficients_ascending, double duration_s,
    MavSegmentEvaluator* out) {
  if (out == nullptr || x_coefficients_ascending.size() < 2 ||
      x_coefficients_ascending.size() != y_coefficients_ascending.size() ||
      !std::isfinite(duration_s) || !(duration_s > 0.0) ||
      !is_finite_vector(x_coefficients_ascending) ||
      !is_finite_vector(y_coefficients_ascending)) {
    return false;
  }
  auto impl = std::make_shared<Impl>(
      to_mav_polynomial(x_coefficients_ascending),
      to_mav_polynomial(y_coefficients_ascending), duration_s,
      static_cast<int>(x_coefficients_ascending.size()));
  out->impl_ = std::move(impl);
  return true;
}

bool MavSegmentEvaluator::valid() const noexcept {
  return impl_ != nullptr;
}

double MavSegmentEvaluator::duration_s() const noexcept {
  return impl_ != nullptr ? impl_->duration_s : 0.0;
}

bool MavSegmentEvaluator::evaluate(double t_s, int derivative_order,
                                   planner::common::Point2d* out) const {
  if (impl_ == nullptr || out == nullptr || !std::isfinite(t_s) ||
      derivative_order < 0 || derivative_order > 3) {
    return false;
  }
  if (t_s < 0.0 || t_s > impl_->duration_s) {
    return false;
  }
  if (derivative_order >= impl_->coefficient_count) {
    *out = planner::common::Point2d{0.0, 0.0};
    return true;
  }
  out->x_m = impl_->x.evaluate(t_s, derivative_order);
  out->y_m = impl_->y.evaluate(t_s, derivative_order);
  return true;
}

bool MavSegmentEvaluator::evaluate_batch(const double* times_s, std::size_t count,
                                         int derivative_order,
                                         planner::common::Point2d* out) const {
  if (impl_ == nullptr || (count > 0 && (times_s == nullptr || out == nullptr))) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    if (!evaluate(times_s[i], derivative_order, &out[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace planner::polynomial::compare
