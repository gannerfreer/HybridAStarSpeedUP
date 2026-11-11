/**
 * @file aabb2d.hpp
 * @brief 二维轴对齐包围盒 (AABB) 及精确欧氏距离查询
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "planner/math/detail/geometry_utils.hpp"
#include "planner/math/vec2d.hpp"

namespace planner::math {

/**
 * @name 二维轴对齐包围盒 (AABB)
 * @{
 */

/**
 * @brief 二维轴对齐矩形包围盒
 */
struct Aabb2d {
  double min_x{0.0};  ///< X 轴坐标下界（米）
  double min_y{0.0};  ///< Y 轴坐标下界（米）
  double max_x{0.0};  ///< X 轴坐标上界（米）
  double max_y{0.0};  ///< Y 轴坐标上界（米）

  /**
   * @brief 判定点 p 是否位于闭 AABB 内部或其边界上
   * @param[in] p 待测点
   * @return 若点在盒内或边上返回 true
   */
  bool contains(const Point2d& p) const noexcept {
    return p.x_m >= min_x && p.x_m <= max_x && p.y_m >= min_y && p.y_m <= max_y;
  }

  /**
   * @brief 判定两个闭 AABB 是否相交或接触（边界相碰判定为相交）
   * @param[in] other 另一个 AABB
   * @return 若相交或接触返回 true
   */
  bool intersects(const Aabb2d& other) const noexcept {
    return !(other.min_x > max_x || other.max_x < min_x || other.min_y > max_y ||
             other.max_y < min_y);
  }

  /**
   * @brief 四周对称向外扩展包围盒边界
   * @param[in] margin 向外扩充的外扩距离（米）
   */
  void expand(double margin) noexcept {
    min_x -= margin;
    min_y -= margin;
    max_x += margin;
    max_y += margin;
  }

  /**
   * @brief 扩展本包围盒以紧密包络新增的点 p
   * @param[in] p 待合并的点
   */
  void merge_point(const Point2d& p) noexcept {
    min_x = std::min(min_x, p.x_m);
    min_y = std::min(min_y, p.y_m);
    max_x = std::max(max_x, p.x_m);
    max_y = std::max(max_y, p.y_m);
  }

  /**
   * @brief 计算包围盒各角点坐标的最大绝对值尺度
   * @return 最大绝对值坐标（米）
   */
  double max_abs_coordinate() const noexcept {
    return std::max(std::max(std::abs(min_x), std::abs(max_x)),
                    std::max(std::abs(min_y), std::abs(max_y)));
  }

  /**
   * @brief 计算点到闭 AABB 的最小欧氏距离（点在盒内或边界上时为 0）
   * @param[in] p 待测点
   * @return 最小欧氏距离（米）
   */
  double distance_to(const Point2d& p) const noexcept {
    const double dx = std::max(std::max(min_x - p.x_m, 0.0), p.x_m - max_x);
    const double dy = std::max(std::max(min_y - p.y_m, 0.0), p.y_m - max_y);
    return internal::hypot_fast(dx, dy);
  }

  /**
   * @brief 计算点到闭 AABB 的最小欧氏距离的平方
   * @param[in] p 待测点
   * @return 平方距离（m^2）
   */
  double distance_sq_to(const Point2d& p) const noexcept {
    const double dx = std::max(std::max(min_x - p.x_m, 0.0), p.x_m - max_x);
    const double dy = std::max(std::max(min_y - p.y_m, 0.0), p.y_m - max_y);
    const double result = dx * dx + dy * dy;
    if (!std::isfinite(result)) {
      const double d = internal::hypot_fast(dx, dy);
      return d * d;
    }
    return result;
  }

  /**
   * @brief 计算本包围盒与另一 AABB 在 X 轴上的投影间隙（重叠时为 0）
   * @param[in] other 另一个 AABB
   * @return 单轴间隙（米）
   */
  double gap_x(const Aabb2d& other) const noexcept {
    return std::max(0.0, std::max(min_x - other.max_x, other.min_x - max_x));
  }

  /**
   * @brief 计算本包围盒与另一 AABB 在 Y 轴上的投影间隙（重叠时为 0）
   * @param[in] other 另一个 AABB
   * @return 单轴间隙（米）
   */
  double gap_y(const Aabb2d& other) const noexcept {
    return std::max(0.0, std::max(min_y - other.max_y, other.min_y - max_y));
  }
};

namespace internal {

/**
 * @brief 判定 AABB 的全部边界坐标是否均位于未归一化快速 SAT 的无溢出安全域内
 * @param[in] box 待检查的轴对齐包围盒
 * @return 若所有坐标有限且绝对值不超过安全上限则返回 true
 */
inline bool box_in_fast_domain(const Aabb2d& box) noexcept {
  return std::isfinite(box.min_x) && std::isfinite(box.max_x) && std::isfinite(box.min_y) &&
         std::isfinite(box.max_y) && std::abs(box.min_x) <= kFastMaxAbsCoordinate &&
         std::abs(box.max_x) <= kFastMaxAbsCoordinate &&
         std::abs(box.min_y) <= kFastMaxAbsCoordinate &&
         std::abs(box.max_y) <= kFastMaxAbsCoordinate;
}

}  // namespace internal

/**
 * @brief 计算包含点集的紧凑二维轴对齐包围盒
 * @param[in] points 点集数组
 * @return 最小外包 Aabb2d
 */
Aabb2d aabb_of(const std::vector<Point2d>& points) noexcept;

/**
 * @brief 计算两个闭 AABB 之间的最小欧氏距离（重叠或接触时返回 0）
 * @param[in] a 第一个 AABB
 * @param[in] b 第二个 AABB
 * @return 最小欧氏距离（米）
 */
double aabb_distance(const Aabb2d& a, const Aabb2d& b) noexcept;

/**
 * @brief 平方距离域下的两 AABB 快速可达性粗筛
 * @details 数学上完全等价于 `aabb_distance(a, b) <= reach`，但省去开方计算开销
 * @param[in] a 第一个 AABB
 * @param[in] b 第二个 AABB
 * @param[in] reach 判定距离阈值（米）
 * @return 若间距小于等于 reach 则返回 true
 */
bool aabb_within(const Aabb2d& a, const Aabb2d& b, double reach) noexcept;

/**
 * @brief 计算点到闭 AABB 的最小欧氏距离（点在盒内或边界上时为 0）
 * @param[in] p 待测点
 * @param[in] box 目标 AABB
 * @return 最小欧氏距离（米）
 */
double point_aabb_distance(const Point2d& p, const Aabb2d& box) noexcept;

/**
 * @brief 计算点到闭 AABB 的最小欧氏距离的平方
 * @param[in] p 待测点
 * @param[in] box 目标 AABB
 * @return 平方距离（m^2）
 */
double point_aabb_distance_sq(const Point2d& p, const Aabb2d& box) noexcept;

// 零动态分配的 AABB 入口；n 为输入点数量
Aabb2d aabb_of_raw(const Point2d* points, std::size_t n) noexcept;
/// @}

}  // namespace planner::math
