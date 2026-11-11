/**
 * @file geometry_utils.hpp
 * @brief 多个几何基元翻译单元共享的数值辅助函数与容差常量
 */

#pragma once

#include <cmath>

namespace planner::math::internal {

constexpr double kEpsScale = 1e-12;

// 未归一化平方域 SAT 的安全取值边界，确保投影与间隙平方乘积不发生浮点溢出
inline constexpr double kFastMaxAbsCoordinate = 1.0e75;
inline constexpr double kFastMaxAxisLenSq = 1.0e300;

// 快速两点欧氏距离
// - 常规取值区间 |dx|, |dy| 处于 [0.0, 1e150) 时，dx^2 + dy^2 无上溢风险，
// - 允许 2.225e-308 下溢到 0.0，
//   范围内直接调用 std::sqrt 否则使用 std::hypot
inline double hypot_fast(double dx, double dy) noexcept {
  constexpr double kHypotFastMax = 1.0e150;  // (1e150)^2 = 1e300
  const double ax = std::abs(dx);
  const double ay = std::abs(dy);
  // (0, 0) 是碰撞检测热点路径中最常见的退化输入（两点重叠或间距为零）
  if (ax == 0.0 && ay == 0.0) {
    return 0.0;
  }
  if (ax >= kHypotFastMax || ay >= kHypotFastMax) {
    return std::hypot(dx, dy);
  }
  return std::sqrt(dx * dx + dy * dy);
}

inline bool coordinates_in_fast_domain(double x, double y) noexcept {
  return std::isfinite(x) && std::isfinite(y) && std::abs(x) <= kFastMaxAbsCoordinate &&
         std::abs(y) <= kFastMaxAbsCoordinate;
}

}  // namespace planner::math::internal
