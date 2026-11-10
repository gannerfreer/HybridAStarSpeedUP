/**
 * @file segment2d.hpp
 * @brief 二维闭合线段基元及点/线段间精确欧氏距离查询。
 */

#pragma once

#include <algorithm>
#include <cmath>

#include "planner/math/detail/geometry_utils.hpp"
#include "planner/math/vec2d.hpp"

namespace planner::math {

/**
 * @name 精确几何欧氏距离计算
 * @{
 */

/**
 * @brief 二维有向线段（起点 start、终点 end），几何上按闭集处理。
 */
struct Segment2d {
  Vec2d start;  ///< 线段起点。
  Vec2d end;    ///< 线段终点。

  Segment2d() = default;
  Segment2d(const Vec2d& start_point, const Vec2d& end_point) noexcept
      : start(start_point), end(end_point) {}

  /**
   * @brief 计算线段欧氏长度。
   * @return 线段长度（米）。
   */
  double length() const noexcept;

  /**
   * @brief 计算线段长度的平方。
   * @return 线段长度平方（m^2）。
   */
  double length_sq() const noexcept;

  /**
   * @brief 计算线段单位方向向量；退化（零长）线段返回零向量。
   * @return 单位方向向量或零向量。
   */
  Vec2d unit_direction() const noexcept;

  /**
   * @brief 计算线段上距离点 p 的最近点（对参数做 [0, 1] 截断）。
   * @param[in] p 待测点。
   * @return 线段上的最近点。
   */
  Vec2d closest_point(const Vec2d& p) const noexcept {
    const Vec2d ab = end - start;
    const double ab2 = ab.norm_sq();
    if (!(ab2 > 0.0)) {
      return start;
    }
    double t = (p - start).dot(ab) / ab2;
    t = std::max(0.0, std::min(1.0, t));
    return Vec2d{start.x_m + t * ab.x_m, start.y_m + t * ab.y_m};
  }

  /**
   * @brief 计算点到线段 [start, end] 的最小欧氏距离。
   * @param[in] p 待测点。
   * @return 最小欧氏距离（米）。
   */
  double distance_to(const Vec2d& p) const noexcept {
    return (p - closest_point(p)).norm();
  }

  /**
   * @brief 计算点到线段 [start, end] 的最小欧氏距离的平方。
   * @param[in] p 待测点。
   * @return 平方距离（m^2）。
   */
  double distance_sq_to(const Vec2d& p) const noexcept {
    const Vec2d proj = closest_point(p);
    const double dx = p.x_m - proj.x_m;
    const double dy = p.y_m - proj.y_m;
    const double result = dx * dx + dy * dy;
    if (std::isfinite(result)) {
      return result;
    }
    const double d = (p - proj).norm();
    return d * d;
  }

  /**
   * @brief 判定两闭线段是否相交或接触（端点接触与共线重叠均视为相交）。
   * @param[in] other 另一条线段。
   * @return 若相交或接触返回 true。
   */
  bool intersects(const Segment2d& other) const noexcept {
    // 标准线段与线段相交测试（包含端点接触和相切）
    const double d1 = (end - start).cross(other.start - start);
    const double d2 = (end - start).cross(other.end - start);
    const double d3 = (other.end - other.start).cross(start - other.start);
    const double d4 = (other.end - other.start).cross(end - other.start);
    if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
        ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))) {
      return true;
    }
    if (d1 == 0.0 && distance_to(other.start) == 0.0) return true;
    if (d2 == 0.0 && distance_to(other.end) == 0.0) return true;
    if (d3 == 0.0 && other.distance_to(start) == 0.0) return true;
    if (d4 == 0.0 && other.distance_to(end) == 0.0) return true;
    return false;
  }

  /**
   * @brief 计算两线段 [start, end] 与 [other.start, other.end] 之间的最小欧氏距离。
   * @param[in] other 另一条线段。
   * @return 最小欧氏距离（米）。
   */
  double distance_to(const Segment2d& other) const noexcept {
    if (intersects(other)) {
      return 0.0;
    }
    return std::min(
        std::min(other.distance_to(start), other.distance_to(end)),
        std::min(distance_to(other.start), distance_to(other.end)));
  }

  /**
   * @brief 计算两线段之间的最小欧氏距离的平方。
   * @param[in] other 另一条线段。
   * @return 平方距离（m^2）。
   */
  double distance_sq_to(const Segment2d& other) const noexcept {
    if (!internal::coordinates_in_fast_domain(start.x_m, start.y_m) ||
        !internal::coordinates_in_fast_domain(end.x_m, end.y_m) ||
        !internal::coordinates_in_fast_domain(other.start.x_m,
                                             other.start.y_m) ||
        !internal::coordinates_in_fast_domain(other.end.x_m, other.end.y_m)) {
      const double d = distance_to(other);
      return d * d;
    }
    // 平方域线段相交测试，逻辑与归一化版本完全对应
    const double d1 = (end - start).cross(other.start - start);
    const double d2 = (end - start).cross(other.end - start);
    const double d3 = (other.end - other.start).cross(start - other.start);
    const double d4 = (other.end - other.start).cross(end - other.start);
    if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
        ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))) {
      return 0.0;
    }
    if (d1 == 0.0 && distance_sq_to(other.start) == 0.0) return 0.0;
    if (d2 == 0.0 && distance_sq_to(other.end) == 0.0) return 0.0;
    if (d3 == 0.0 && other.distance_sq_to(start) == 0.0) return 0.0;
    if (d4 == 0.0 && other.distance_sq_to(end) == 0.0) return 0.0;
    return std::min(
        std::min(other.distance_sq_to(start), other.distance_sq_to(end)),
        std::min(distance_sq_to(other.start), distance_sq_to(other.end)));
  }
};

/// @}

}  // namespace planner::math
