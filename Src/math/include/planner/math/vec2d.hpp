#pragma once

namespace planner::math {

/**
 * @brief 二维笛卡尔平面坐标点。
 */
struct Vec2d {
  double x_m{0.0};  ///< X 坐标（米）。
  double y_m{0.0};  ///< Y 坐标（米）。
};

using Point2d = Vec2d;

inline constexpr bool operator==(const Point2d& lhs, const Point2d& rhs) {
  return lhs.x_m == rhs.x_m && lhs.y_m == rhs.y_m;
}

}  // namespace planner::math
