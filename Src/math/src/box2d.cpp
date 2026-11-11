/**
 * @file box2d.cpp
 * @brief 二维有向包围盒查询实现；AABB 相交采用专用 4 轴 SAT 快速路径并在必要时回退至通用算法
 */

#include "planner/math/box2d.hpp"

#include <cmath>

#include "planner/math/polygon2d.hpp"

namespace planner::math {

namespace {

/// 专用快速路径允许的最大绝对坐标（防止投影与外包盒计算发生浮点溢出）
constexpr double kFastMaxAbsCoordinate = 1.0e150;

/// 检查标量是否有限且位于快速路径的无溢出坐标域内
bool in_fast_domain(double value) noexcept {
  return std::isfinite(value) && std::abs(value) <= kFastMaxAbsCoordinate;
}

}  // namespace

Box2d::Box2d(const Vec2d& center, double length_m, double width_m, double heading_rad) noexcept
    : Box2d(center, length_m, width_m, heading_rad, std::cos(heading_rad), std::sin(heading_rad)) {}

bool Box2d::intersects(const Aabb2d& box, double tol) const noexcept {
  const auto generic = [this, &box, tol]() noexcept {
    const std::array<Point2d, 4> c = corners();
    return convex_polygon_intersects_aabb(c.data(), c.size(), box, tol);
  };
  if (!(tol >= 0.0) || !std::isfinite(tol) || !(box.min_x <= box.max_x) ||
      !(box.min_y <= box.max_y) || !(length_m_ >= 0.0) || !(width_m_ >= 0.0) ||
      !(half_length_m_ >= 0.0) || !(half_width_m_ >= 0.0) || !std::isfinite(cos_heading_) ||
      !std::isfinite(sin_heading_) || !in_fast_domain(center_.x_m) ||
      !in_fast_domain(center_.y_m) || !in_fast_domain(length_m_) ||
      !in_fast_domain(width_m_) || !in_fast_domain(box.min_x) || !in_fast_domain(box.max_x) ||
      !in_fast_domain(box.min_y) || !in_fast_domain(box.max_y)) {
    return generic();
  }

  const double c = cos_heading_;
  const double s = sin_heading_;
  const double abs_c = std::abs(c);
  const double abs_s = std::abs(s);

  // 世界 X/Y 轴：用 OBB 的世界系外包半宽直接比较间隙（轴为单位向量，无需除法）
  const double ex = half_length_m_ * abs_c + half_width_m_ * abs_s;
  const double ey = half_length_m_ * abs_s + half_width_m_ * abs_c;
  if (center_.x_m + ex < box.min_x - tol || center_.x_m - ex > box.max_x + tol ||
      center_.y_m + ey < box.min_y - tol || center_.y_m - ey > box.max_y + tol) {
    return false;
  }

  // 包围盒局部 X/Y 轴：AABB 中心相对 OBB 中心的投影 vs 双方半宽之和
  const double dx = 0.5 * (box.min_x + box.max_x) - center_.x_m;
  const double dy = 0.5 * (box.min_y + box.max_y) - center_.y_m;
  const double proj_x = c * dx + s * dy;
  const double proj_y = -s * dx + c * dy;
  const double hx = 0.5 * (box.max_x - box.min_x);
  const double hy = 0.5 * (box.max_y - box.min_y);
  const double radius_x = hx * abs_c + hy * abs_s;
  const double radius_y = hx * abs_s + hy * abs_c;
  if (std::abs(proj_x) > half_length_m_ + radius_x + tol ||
      std::abs(proj_y) > half_width_m_ + radius_y + tol) {
    return false;
  }
  return true;
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
