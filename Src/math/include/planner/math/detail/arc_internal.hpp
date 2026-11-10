#pragma once
#include <cmath>
#include "planner/math/pose2d.hpp"
namespace planner::math::internal {
constexpr double kSincSeriesBound = 1e-8;
inline double sinc(double x) noexcept {
  if (std::abs(x) < kSincSeriesBound) {
    const double x2 = x * x;
    return 1.0 - x2 / 6.0 + x2 * x2 / 120.0;
  }
  return std::sin(x) / x;
}
inline Pose2d advance_arc_trig_unchecked(const Pose2d& start, double s, double kappa,
                        double cy, double sy) noexcept {
  const double b = kappa * s;
  const double limit = 0.5 * b;
  const double sb = sinc(b);
  const double half = limit * sinc(limit) * sinc(limit);
  const double dsin = cy * sb - sy * half;   // (sin(y+b) - sin y) / b
  const double dcos = -sy * sb - cy * half;  // (cos(y+b) - cos y) / b
  Pose2d out;
  out.x_m = start.x_m + s * dsin;
  out.y_m = start.y_m - s * dcos;
  out.yaw_rad = start.yaw_rad + b;
  return out;
}
}  // namespace planner::math::internal
