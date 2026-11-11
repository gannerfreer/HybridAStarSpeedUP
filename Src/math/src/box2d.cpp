/**
 * @file box2d.cpp
 * @brief 二维有向包围盒查询实现；相交判定统一委托至凸多边形 SAT 算法
 */

#include "planner/math/box2d.hpp"

#include <cmath>

#include "planner/math/polygon2d.hpp"

namespace planner::math {

Box2d::Box2d(const Vec2d& center, double length_m, double width_m, double heading_rad) noexcept
    : Box2d(center, length_m, width_m, heading_rad, std::cos(heading_rad), std::sin(heading_rad)) {}

bool Box2d::intersects(const Aabb2d& box, double tol) const noexcept {
  const std::array<Point2d, 4> c = corners();
  return convex_polygon_intersects_aabb(c.data(), c.size(), box, tol);
}

bool Box2d::intersects(const Box2d& other, double tol) const noexcept {
  const std::array<Point2d, 4> c = corners();
  const std::array<Point2d, 4> o = other.corners();
  return convex_polygons_intersect(c.data(), c.size(), o.data(), o.size(), tol);
}

double Box2d::distance_sq_to(const Point2d& p) const noexcept {
  const std::array<Point2d, 4> c = corners();
  return point_polygon_distance_sq(p, c.data(), c.size());
}

}  // namespace planner::math
