// 由三次/五次边界条件构建的二维轨迹分段，
// 带局部定义域约束和批处理采样。

#include "planner/polynomial/trajectory_segment.hpp"

#include <cmath>
#include <cstddef>
#include <utility>

namespace planner::polynomial {
namespace {

using planner::common::Point2d;

bool point_finite(const Point2d& point) noexcept {
  return std::isfinite(point.x_m) && std::isfinite(point.y_m);
}

}  // namespace

TrajectorySegment::TrajectorySegment(Polynomial x, Polynomial y, double duration_s, int degree)
    : x_(std::move(x)), y_(std::move(y)), duration_s_(duration_s), degree_(degree) {}

Result<TrajectorySegment> TrajectorySegment::cubic(const CubicBoundary2d& boundary,
                                                   double duration_s) {
  if (!point_finite(boundary.p0_m) || !point_finite(boundary.v0_mps) ||
      !point_finite(boundary.p1_m) || !point_finite(boundary.v1_mps)) {
    return Result<TrajectorySegment>::failure(Code::kInvalidInput);
  }
  const Result<Polynomial> x =
      Polynomial::cubic(CubicBoundary{boundary.p0_m.x_m, boundary.v0_mps.x_m,
                                      boundary.p1_m.x_m, boundary.v1_mps.x_m},
                        duration_s);
  if (!x.ok()) {
    return Result<TrajectorySegment>::failure(x.code);
  }
  const Result<Polynomial> y =
      Polynomial::cubic(CubicBoundary{boundary.p0_m.y_m, boundary.v0_mps.y_m,
                                      boundary.p1_m.y_m, boundary.v1_mps.y_m},
                        duration_s);
  if (!y.ok()) {
    return Result<TrajectorySegment>::failure(y.code);
  }
  return Result<TrajectorySegment>::success(
      TrajectorySegment(x.value, y.value, duration_s, 3));
}

Result<TrajectorySegment> TrajectorySegment::quintic(const QuinticBoundary2d& boundary,
                                                     double duration_s) {
  if (!point_finite(boundary.p0_m) || !point_finite(boundary.v0_mps) ||
      !point_finite(boundary.a0_mps2) || !point_finite(boundary.p1_m) ||
      !point_finite(boundary.v1_mps) || !point_finite(boundary.a1_mps2)) {
    return Result<TrajectorySegment>::failure(Code::kInvalidInput);
  }
  const Result<Polynomial> x = Polynomial::quintic(
      QuinticBoundary{boundary.p0_m.x_m, boundary.v0_mps.x_m, boundary.a0_mps2.x_m,
                      boundary.p1_m.x_m, boundary.v1_mps.x_m, boundary.a1_mps2.x_m},
      duration_s);
  if (!x.ok()) {
    return Result<TrajectorySegment>::failure(x.code);
  }
  const Result<Polynomial> y = Polynomial::quintic(
      QuinticBoundary{boundary.p0_m.y_m, boundary.v0_mps.y_m, boundary.a0_mps2.y_m,
                      boundary.p1_m.y_m, boundary.v1_mps.y_m, boundary.a1_mps2.y_m},
      duration_s);
  if (!y.ok()) {
    return Result<TrajectorySegment>::failure(y.code);
  }
  return Result<TrajectorySegment>::success(
      TrajectorySegment(x.value, y.value, duration_s, 5));
}

bool TrajectorySegment::valid() const noexcept {
  return std::isfinite(duration_s_) && duration_s_ > 0.0 && degree_ > 0 &&
         x_.is_finite() && y_.is_finite();
}

double TrajectorySegment::duration_s() const noexcept {
  return duration_s_;
}

int TrajectorySegment::degree() const noexcept {
  return degree_;
}

const Polynomial& TrajectorySegment::x_polynomial() const noexcept {
  return x_;
}

const Polynomial& TrajectorySegment::y_polynomial() const noexcept {
  return y_;
}

Result<Point2d> TrajectorySegment::evaluate(double local_time_s,
                                            int derivative_order) const noexcept {
  if (!valid()) {
    return Result<Point2d>::failure(Code::kInvalidInput);
  }
  if (!std::isfinite(local_time_s)) {
    return Result<Point2d>::failure(Code::kInvalidInput);
  }
  if (local_time_s < 0.0 || local_time_s > duration_s_) {
    return Result<Point2d>::failure(Code::kOutOfRange);
  }
  const Result<double> x = x_.evaluate_derivative(local_time_s, derivative_order);
  if (!x.ok()) {
    return Result<Point2d>::failure(x.code);
  }
  const Result<double> y = y_.evaluate_derivative(local_time_s, derivative_order);
  if (!y.ok()) {
    return Result<Point2d>::failure(y.code);
  }
  return Result<Point2d>::success(Point2d{x.value, y.value});
}

BatchResult TrajectorySegment::evaluate_batch(const double* local_times_s, std::size_t count,
                                              int derivative_order,
                                              Point2d* output) const noexcept {
  BatchResult result;
  if (count == 0) {
    return result;
  }
  if (local_times_s == nullptr || output == nullptr) {
    result.code = Code::kInvalidInput;
    return result;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const Result<Point2d> value = evaluate(local_times_s[i], derivative_order);
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

Result<TrajectorySegment> TrajectorySegment::scaled_time(double time_scale) const {
  if (!valid()) {
    return Result<TrajectorySegment>::failure(Code::kInvalidInput);
  }
  if (!std::isfinite(time_scale) || time_scale <= 0.0) {
    return Result<TrajectorySegment>::failure(Code::kInvalidInput);
  }
  const double scaled_duration = duration_s_ * time_scale;
  if (!std::isfinite(scaled_duration) || scaled_duration <= 0.0) {
    return Result<TrajectorySegment>::failure(Code::kInvalidInput);
  }
  const Result<Polynomial> x = x_.scaled_time(time_scale);
  if (!x.ok()) {
    return Result<TrajectorySegment>::failure(x.code);
  }
  const Result<Polynomial> y = y_.scaled_time(time_scale);
  if (!y.ok()) {
    return Result<TrajectorySegment>::failure(y.code);
  }
  return Result<TrajectorySegment>::success(
      TrajectorySegment(x.value, y.value, scaled_duration, degree_));
}

}  // namespace planner::polynomial
