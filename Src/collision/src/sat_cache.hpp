/**
 * @file sat_cache.hpp
 * @brief 碰撞检测底层几何快速路径数据结构与 SAT 预计算缓存
 * @details 碰撞检测查询热点路径的核心优化机制：
 *          - 采用未归一化的边法向投影轴，并在投影间隙平方域进行比较：
 *            `d > 0 && d * d > tol * tol * axis_len_sq`，彻底消除逐轴计算模长开方与浮点除法开销
 *          - 当前仅由扫掠包络的保守快速路径使用：针对单次检测中固定的扫掠包络多边形，
 *            预先计算并缓存其在各边法向轴上的投影极值区间
 *          - 若坐标极值或轴长平方超出安全无溢出范围（|coord| > 1e75, len_sq > 1e300），则自动平滑回退至标准归一化 SAT 路径
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "planner/math/aabb2d.hpp"
#include "planner/math/polygon2d.hpp"
#include "planner/math/vec2d.hpp"

namespace planner::collision::internal {

using math::Aabb2d;
using math::Point2d;

/// 快速未归一化平方距离域下的坐标与轴长安全上限（防止乘法浮点溢出）；
/// 与 math 层快速域上限保持一致，若 math 侧调整必须同步更新
constexpr double kFastMaxAbsCoordinate = 1.0e75;
constexpr double kFastMaxAxisLenSq = 1.0e300;

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

/**
 * @brief 凸多边形与 AABB 相交测试的分离轴定理 (SAT) 预计算缓存结构
 * @tparam Capacity 固定栈内存支持的最大多边形顶点容量
 */
template <std::size_t Capacity>
struct PolygonSatCache {
  static constexpr std::size_t kCapacity = Capacity;

  /// 快速未归一化平方距离域下的坐标与轴长安全上限（防止乘法浮点溢出）
  static constexpr double kFastMaxAbsCoordinate = internal::kFastMaxAbsCoordinate;
  static constexpr double kFastMaxAxisLenSq = internal::kFastMaxAxisLenSq;

  const Point2d* points{nullptr};  ///< 指向多边形外部顶点缓冲区的无所有权指针
  std::size_t count{0};            ///< 多边形顶点总数
  bool usable{false};  ///< 容量是否足够（count <= Capacity）；若超出则置 false 触发回退
  bool fast_domain{false};  ///< 所有坐标与轴长是否均在免溢出的快速安全域内
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  std::array<Point2d, Capacity> raw_axes{};    ///< 边 (i, i+1) 的未归一化外法向轴
  std::array<double, Capacity> axis_len_sq{};  ///< 投影轴模长平方 |axis|^2
  std::array<double, Capacity> poly_lo{};  ///< 多边形在 raw_axes[i] 上的投影标量最小值
  std::array<double, Capacity> poly_hi{};  ///< 多边形在 raw_axes[i] 上的投影标量最大值
  std::array<std::uint8_t, Capacity> axis_ok{};  ///< 1 表示有效非退化分离轴，0 表示退化重合轴

  /**
   * @brief 一次性预计算多边形外包盒、未归一化边法向轴、模长平方与投影极值区间
   * @param[in] polygon 凸多边形顶点指针
   * @param[in] n 顶点个数
   */
  void build(const Point2d* polygon, std::size_t n) noexcept {
    points = polygon;
    count = n;
    usable = n <= Capacity;
    fast_domain = false;
    if (!usable || n == 0) {
      return;  // 空多边形合法（恒不相交）；超容量则标记并在查询时触发回退
    }
    min_x = std::numeric_limits<double>::infinity();
    max_x = -std::numeric_limits<double>::infinity();
    min_y = std::numeric_limits<double>::infinity();
    max_y = -std::numeric_limits<double>::infinity();
    bool coordinates_finite = true;
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p = polygon[i];
      if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) ||
          std::abs(p.x_m) > kFastMaxAbsCoordinate || std::abs(p.y_m) > kFastMaxAbsCoordinate) {
        coordinates_finite = false;
      }
      min_x = std::min(min_x, p.x_m);
      max_x = std::max(max_x, p.x_m);
      min_y = std::min(min_y, p.y_m);
      max_y = std::max(max_y, p.y_m);
    }
    fast_domain = coordinates_finite;
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p0 = polygon[i];
      const Point2d& p1 = polygon[(i + 1) % n];
      const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
      const double len_sq = axis.x_m * axis.x_m + axis.y_m * axis.y_m;
      if (!(len_sq > 0.0)) {
        axis_ok[i] = 0;  // 顶点重合产生的退化轴：无法作为有效分离轴
        raw_axes[i] = Point2d{0.0, 0.0};
        axis_len_sq[i] = 0.0;
        poly_lo[i] = 0.0;
        poly_hi[i] = 0.0;
        continue;
      }
      if (!std::isfinite(len_sq) || len_sq > kFastMaxAxisLenSq) {
        fast_domain = false;
      }
      axis_ok[i] = 1;
      raw_axes[i] = axis;
      axis_len_sq[i] = len_sq;
      double lo = std::numeric_limits<double>::infinity();
      double hi = -std::numeric_limits<double>::infinity();
      for (std::size_t j = 0; j < n; ++j) {
        const double v = polygon[j].x_m * axis.x_m + polygon[j].y_m * axis.y_m;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
      }
      poly_lo[i] = lo;
      poly_hi[i] = hi;
    }
  }

  /**
   * @brief 利用预计算缓存对输入 AABB 执行快速闭集相交测试
   * @param[in] box 目标 AABB
   * @param[in] tol 距离容差
   * @return 若在容差范围内无法排除相交则返回 true
   */
  bool maybe_intersects(const Aabb2d& box, double tol) const noexcept {
    if (count == 0) {
      return false;
    }
    if (!usable) {
      return true;  // 容量不足时保守返回 true，由调用方执行精确距离检验以避免漏判
    }
    if (!fast_domain || !box_in_fast_domain(box) || !(tol >= 0.0) || !std::isfinite(tol)) {
      return math::convex_polygon_intersects_aabb(points, count, box, tol);
    }
    // 快速外包盒拒斥粗筛
    if (max_x < box.min_x - tol || min_x > box.max_x + tol || max_y < box.min_y - tol ||
        min_y > box.max_y + tol) {
      return false;
    }
    const double tol_sq = tol * tol;
    for (std::size_t i = 0; i < count; ++i) {
      if (axis_ok[i] == 0) {
        continue;
      }
      const Point2d& axis = raw_axes[i];
      const double p00 = box.min_x * axis.x_m + box.min_y * axis.y_m;
      const double p10 = box.max_x * axis.x_m + box.min_y * axis.y_m;
      const double p01 = box.min_x * axis.x_m + box.max_y * axis.y_m;
      const double p11 = box.max_x * axis.x_m + box.max_y * axis.y_m;
      const double box_lo = std::min(std::min(p00, p10), std::min(p01, p11));
      const double box_hi = std::max(std::max(p00, p10), std::max(p01, p11));
      const double gap_above = box_lo - poly_hi[i];
      if (gap_above > 0.0 && gap_above * gap_above > tol_sq * axis_len_sq[i]) {
        return false;
      }
      const double gap_below = poly_lo[i] - box_hi;
      if (gap_below > 0.0 && gap_below * gap_below > tol_sq * axis_len_sq[i]) {
        return false;
      }
    }
    return true;
  }
};

using PolygonSatCache8 = PolygonSatCache<8>;

}  // namespace planner::collision::internal
