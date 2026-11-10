#include "planner/math/angle.hpp"

#include <cmath>
#include <limits>

namespace planner::math {

double normalize_angle_rad(double angle_rad) {
  if (!std::isfinite(angle_rad)) return std::numeric_limits<double>::quiet_NaN();
  // 若已落在 [-pi, pi) 区间内，直接返回以确保数值幂等性。
  if (angle_rad >= -kPi && angle_rad < kPi) {
    return angle_rad;
  }
  double wrapped = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  double result = wrapped - kPi;
  if (result >= kPi) {
    // 仅在 fmod 发生浮点舍入极值时可能触发；强制约束在半开区间内。
    result = -kPi;
  }
  return result;
}

double angle_diff_rad(double a_rad, double b_rad) noexcept {
  return normalize_angle_rad(a_rad - b_rad);
}

}  // namespace planner::math
