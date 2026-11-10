/**
 * @file vec2d.hpp
 * @brief 二维笛卡尔向量/点基元与基础向量代数。
 */

#pragma once

#include <cmath>

#include "planner/math/detail/geometry_utils.hpp"

namespace planner::math {

/**
 * @name 二维基础向量代数
 * @{
 */

/**
 * @brief 二维笛卡尔平面坐标点。
 */
struct Vec2d {
  double x_m{0.0};  ///< X 坐标（米）。
  double y_m{0.0};  ///< Y 坐标（米）。

  /**
   * @brief 计算二维向量点积标量：\f$ a_x b_x + a_y b_y \f$。
   * @param[in] other 第二个二维向量。
   * @return 点积标量值。
   */
  double dot(const Vec2d& other) const noexcept { return x_m * other.x_m + y_m * other.y_m; }

  /**
   * @brief 计算二维向量叉积标量（对应三维叉积的 Z 分量）：\f$ a_x b_y - a_y b_x \f$。
   * @param[in] other 第二个二维向量。
   * @return 叉积标量值。
   */
  double cross(const Vec2d& other) const noexcept { return x_m * other.y_m - y_m * other.x_m; }

  /**
   * @brief 计算二维向量欧氏范数的平方。
   * @return 向量模长平方（m^2）。
   */
  double norm_sq() const noexcept { return dot(*this); }

  /**
   * @brief 计算二维点/向量的欧氏范数（模长）。
   * @return 向量模长。
   */
  double norm() const noexcept { return internal::hypot_fast(x_m, y_m); }

  /**
   * @brief 计算单位方向向量；零向量输入返回零向量。
   * @return 模长为 1 的单位向量，或零向量。
   */
  Vec2d normalized_or_zero() const noexcept {
    const double len = norm();
    if (!(len > 0.0) || !std::isfinite(len)) {
      return Vec2d{};
    }
    return Vec2d{x_m / len, y_m / len};
  }

  /**
   * @brief 将向量绕原点逆时针旋转指定角度。
   * @param[in] angle_rad 旋转角（弧度）。
   * @return 旋转后的二维向量。
   */
  Vec2d rotated(double angle_rad) const noexcept {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return Vec2d{c * x_m - s * y_m, s * x_m + c * y_m};
  }

  /**
   * @brief 将向量投影到指定轴方向上。
   * @param[in] axis 投影轴方向向量。
   * @return axis 方向上的投影向量；axis 为零向量时返回零向量。
   */
  Vec2d projected_on(const Vec2d& axis) const noexcept {
    const double axis_len_sq = axis.norm_sq();
    if (!(axis_len_sq > 0.0) || !std::isfinite(axis_len_sq)) {
      return Vec2d{};
    }
    const double scale = dot(axis) / axis_len_sq;
    return Vec2d{axis.x_m * scale, axis.y_m * scale};
  }
};

using Point2d = Vec2d;

inline constexpr bool operator==(const Point2d& lhs, const Point2d& rhs) {
  return lhs.x_m == rhs.x_m && lhs.y_m == rhs.y_m;
}

/**
 * @brief 计算二维向量叉积标量（对应三维叉积的 Z 分量）：\f$ a_x b_y - a_y b_x \f$。
 * @param[in] a 第一个二维向量。
 * @param[in] b 第二个二维向量。
 * @return 叉积标量值。
 */
double cross(const Point2d& a, const Point2d& b) noexcept;

/**
 * @brief 计算二维向量点积标量：\f$ a_x b_x + a_y b_y \f$。
 * @param[in] a 第一个二维向量。
 * @param[in] b 第二个二维向量。
 * @return 点积标量值。
 */
double dot(const Point2d& a, const Point2d& b) noexcept;

inline Point2d operator+(const Point2d& a, const Point2d& b) noexcept {
  return Point2d{a.x_m + b.x_m, a.y_m + b.y_m};
}

inline Point2d operator-(const Point2d& a, const Point2d& b) noexcept {
  return Point2d{a.x_m - b.x_m, a.y_m - b.y_m};
}

inline Point2d operator*(const Point2d& a, double s) noexcept {
  return Point2d{a.x_m * s, a.y_m * s};
}

/**
 * @brief 计算二维点/向量的欧氏范数（模长）。
 * @param[in] a 输入二维向量。
 * @return 向量模长。
 */
double norm(const Point2d& a) noexcept;

/**
 * @brief 计算两二维点之间的欧氏距离。
 * @param[in] a 第一个点。
 * @param[in] b 第二个点。
 * @return 空间距离（米）。
 */
double distance(const Point2d& a, const Point2d& b) noexcept;
/// @}

}  // namespace planner::math
