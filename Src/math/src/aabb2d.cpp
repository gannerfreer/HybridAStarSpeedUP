/**
 * @file aabb2d.cpp
 * @brief 二维轴对齐包围盒聚合与自由函数接口实现。
 */

#include "planner/math/aabb2d.hpp"

#include <algorithm>
#include <cmath>

#include "planner/math/detail/geometry_utils.hpp"

namespace planner::math {

Aabb2d aabb_of_raw(const Point2d* points, std::size_t n) noexcept {
  Aabb2d box;
  if (n == 0) {
    return box;
  }
  box.min_x = box.max_x = points[0].x_m;
  box.min_y = box.max_y = points[0].y_m;
  for (std::size_t i = 1; i < n; ++i) {
    box.merge_point(points[i]);
  }
  return box;
}

Aabb2d aabb_of(const std::vector<Point2d>& points) noexcept {
  return aabb_of_raw(points.data(), points.size());
}

double aabb_distance(const Aabb2d& a, const Aabb2d& b) noexcept {
  return internal::hypot_fast(a.gap_x(b), a.gap_y(b));
}

bool aabb_within(const Aabb2d& a, const Aabb2d& b, double reach) noexcept {
  const double dx = a.gap_x(b);
  const double dy = a.gap_y(b);
  // 逐轴快速上界判别：若任一单轴投影间隙已超过 reach，则空间总距离必然超过 reach
  if (dx > reach || dy > reach) {
    return false;
  }
  const double distance_sq = dx * dx + dy * dy;
  const double reach_sq = reach * reach;
  if (!std::isfinite(distance_sq) || !std::isfinite(reach_sq)) {
    // 针对极值或 inf/nan 等异常输入保留回退行为
    return aabb_distance(a, b) <= reach;
  }
  return distance_sq <= reach_sq;
}

double point_aabb_distance(const Point2d& p, const Aabb2d& box) noexcept {
  return box.distance_to(p);
}

double point_aabb_distance_sq(const Point2d& p, const Aabb2d& box) noexcept {
  return box.distance_sq_to(p);
}

}  // namespace planner::math
