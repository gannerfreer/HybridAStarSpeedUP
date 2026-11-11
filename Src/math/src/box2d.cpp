/**
 * @file box2d.cpp
 * @brief 二维有向包围盒查询实现；AABB 相交采用专用 4 轴 SAT 快速路径并在必要时回退至通用算法
 */

#include "planner/math/box2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "planner/math/polygon2d.hpp"

namespace planner::math {

namespace {

/// 专用快速路径允许的最大绝对坐标（防止投影与外包盒计算发生浮点溢出）
constexpr double kFastMaxAbsCoordinate = 1.0e150;

/// 分离边界附近的 ULP 舍入松弛因子：仅用于判定何时回退通用 SAT，不扩大接触容差
constexpr double kUlpBoundarySlackFactor = 16.0;

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
      !in_fast_domain(center_.y_m) || !in_fast_domain(length_m_) || !in_fast_domain(width_m_) ||
      !in_fast_domain(box.min_x) || !in_fast_domain(box.max_x) || !in_fast_domain(box.min_y) ||
      !in_fast_domain(box.max_y)) {
    return generic();
  }

  const double c = cos_heading_;
  const double s = sin_heading_;
  const double abs_c = std::abs(c);
  const double abs_s = std::abs(s);

  // ULP 边界回退是用于保持闭集语义的舍入保护，而非扩大接触容差：
  // 投影与外扩经过多次浮点运算，精确相切可能被舍入成微小分离；此时交由通用 SAT 判定
  const auto full_slack = [this, &box, tol]() noexcept {
    const double base_scale =
        std::max(std::max(std::abs(center_.x_m), std::abs(center_.y_m)),
                 std::max(std::max(half_length_m_, half_width_m_), std::max(1.0, std::abs(tol))));
    const double scale = std::max(base_scale, box.max_abs_coordinate());
    return kUlpBoundarySlackFactor * std::numeric_limits<double>::epsilon() * scale;
  };

  // 世界 X/Y 轴：用 OBB 的世界系外包半宽比较间隙
  const double ex = half_length_m_ * abs_c + half_width_m_ * abs_s;
  const double ey = half_length_m_ * abs_s + half_width_m_ * abs_c;
  const double obb_max_x = center_.x_m + ex;
  const double obb_min_x = center_.x_m - ex;
  const double world_min_x = box.min_x - tol;
  const double world_max_x = box.max_x + tol;
  if (obb_max_x < world_min_x) {
    const double slack = full_slack();
    return (obb_max_x < world_min_x - slack) ? false : generic();
  }
  if (obb_min_x > world_max_x) {
    const double slack = full_slack();
    return (obb_min_x > world_max_x + slack) ? false : generic();
  }
  const double obb_max_y = center_.y_m + ey;
  const double obb_min_y = center_.y_m - ey;
  const double world_min_y = box.min_y - tol;
  const double world_max_y = box.max_y + tol;
  if (obb_max_y < world_min_y) {
    const double slack = full_slack();
    return (obb_max_y < world_min_y - slack) ? false : generic();
  }
  if (obb_min_y > world_max_y) {
    const double slack = full_slack();
    return (obb_min_y > world_max_y + slack) ? false : generic();
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
  const double abs_proj_x = std::abs(proj_x);
  const double limit_x = half_length_m_ + radius_x + tol;
  if (abs_proj_x > limit_x) {
    const double slack = full_slack();
    return (abs_proj_x > limit_x + slack) ? false : generic();
  }
  const double abs_proj_y = std::abs(proj_y);
  const double limit_y = half_width_m_ + radius_y + tol;
  if (abs_proj_y > limit_y) {
    const double slack = full_slack();
    return (abs_proj_y > limit_y + slack) ? false : generic();
  }

  // 快速路径未发现分离：还需排除投影恰在边界附近的舍入差异，统一回退通用 SAT
  const double slack = full_slack();
  const double gap_world_x = std::min(obb_max_x - world_min_x, world_max_x - obb_min_x);
  if (gap_world_x <= slack) {
    return generic();
  }
  const double gap_world_y = std::min(obb_max_y - world_min_y, world_max_y - obb_min_y);
  if (gap_world_y <= slack) {
    return generic();
  }
  if (limit_x - abs_proj_x <= slack || limit_y - abs_proj_y <= slack) {
    return generic();
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
