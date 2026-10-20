#pragma once

// 碰撞组件内部的几何快速路径（不对外安装；公共头文件仅新增平方域
// 辅助函数，不改变已有函数签名）。
//
// 这里只放“与公共函数判定等价、但把与查询单元无关的量预计算一次”的
// 辅助结构。使用者：collision_checker.cpp 的单元扫描热路径。
//
// 决策路径的平方距离化：
//   * 边法线不再归一化，直接使用原始轴；投影间隙 d 与容差 tol 的比较改为
//     `d > 0 && d*d > tol*tol*axis_len_sq`，省去逐轴 hypot/除法；
//   * 多边形的投影区间只依赖多边形与轴，故随轴一起缓存（归约顺序与原实现
//     相同：按顶点升序取 min/max），逐格只计算盒子四角投影；
//   * 坐标/轴长度平方超出安全域（|坐标| > 1e75、axis_len_sq 非有限或
//     > 1e300、tol < 0/非有限）时回退到 geometry.cpp 中保留的归一化实现，
//     使 inf/nan/极大值输入的行为不劣化。
//
// 与公共实现的一致性由随机点级对照保证；常规域内二者只在浮点边界
// （ulp 级）可能给出不同的“可证明更远”判定，而所有过滤只跳过必然不接触
// 的候选，最终判定仍由距离比较完成。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "planner/collision/geometry.hpp"

namespace planner::collision::internal {

// 归一化回退实现（定义在 geometry.cpp）。语义与平方化之前的公共函数
// 逐位一致，供极端输入域与顶点数超过缓存容量的多边形使用。
bool convex_polygon_intersects_aabb_normalized(const Point2d* polygon,
                                               std::size_t count,
                                               const Aabb2d& box,
                                               double tol) noexcept;

// 预计算的“凸多边形 vs AABB”分离轴缓存。语义与
// `convex_polygon_intersects_aabb(polygon, box, tol)` 等价（含空多边形
// 返回 false、退化边跳过、容差内接触计为相交）。
template <std::size_t Capacity>
struct PolygonSatCache {
  static constexpr std::size_t kCapacity = Capacity;

  // 平方化路径的安全域：坐标绝对值上限与轴长度平方上限。1e75 坐标配合
  // 1e75 轴分量时投影约 1e150，间隙平方约 1e300，仍在 double 有限域内；
  // 超出后回退归一化实现，避免 inf/inf 比较掩盖真实分离。
  static constexpr double kFastMaxAbsCoordinate = 1.0e75;
  static constexpr double kFastMaxAxisLenSq = 1.0e300;

  const Point2d* points{nullptr};
  std::size_t count{0};
  bool usable{false};  // count > Capacity 时为 false（调用方回退公共函数）
  bool fast_domain{false};  // 坐标/轴长度是否处于平方化安全域
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  std::array<Point2d, Capacity> raw_axes{};       // 与边 (i, i+1) 对应
  std::array<double, Capacity> axis_len_sq{};     // |axis|²（未归一化）
  std::array<double, Capacity> poly_lo{};         // 多边形在原始轴上的投影
  std::array<double, Capacity> poly_hi{};
  std::array<std::uint8_t, Capacity> axis_ok{};   // 边法线非退化时为 1

  // 预计算包围盒、原始边法线、轴长度平方与多边形投影区间。
  void build(const Point2d* polygon, std::size_t n) noexcept {
    points = polygon;
    count = n;
    usable = n <= Capacity;
    fast_domain = false;
    if (!usable || n == 0) {
      return;  // 空多边形合法（恒不相交）；超容量由调用方回退
    }
    min_x = std::numeric_limits<double>::infinity();
    max_x = -std::numeric_limits<double>::infinity();
    min_y = std::numeric_limits<double>::infinity();
    max_y = -std::numeric_limits<double>::infinity();
    bool coordinates_finite = true;
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p = polygon[i];
      if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) ||
          std::abs(p.x_m) > kFastMaxAbsCoordinate ||
          std::abs(p.y_m) > kFastMaxAbsCoordinate) {
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
        axis_ok[i] = 0;  // 退化轴：无法分离（与原实现一致）
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
        const double v =
            polygon[j].x_m * axis.x_m + polygon[j].y_m * axis.y_m;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
      }
      poly_lo[i] = lo;
      poly_hi[i] = hi;
    }
  }

  // 凸多边形与轴对齐包围盒的闭集相交测试（分离轴）。与归一化公共实现
  // 数学等价；常规域内只省去开方/除法，极端域回退。
  bool maybe_intersects(const Aabb2d& box, double tol) const noexcept {
    if (count == 0) {
      return false;
    }
    if (!usable) {
      return true;  // 保守方向：调用方自行回退完整判定
    }
    if (!fast_domain || !box_in_fast_domain(box) || !(tol >= 0.0) ||
        !std::isfinite(tol)) {
      return convex_polygon_intersects_aabb_normalized(points, count, box, tol);
    }
    // 与归一化实现相同的包围盒快速拒绝。
    if (max_x < box.min_x - tol || min_x > box.max_x + tol ||
        max_y < box.min_y - tol || min_y > box.max_y + tol) {
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
      if (gap_above > 0.0 &&
          gap_above * gap_above > tol_sq * axis_len_sq[i]) {
        return false;
      }
      const double gap_below = poly_lo[i] - box_hi;
      if (gap_below > 0.0 &&
          gap_below * gap_below > tol_sq * axis_len_sq[i]) {
        return false;
      }
    }
    return true;
  }

 private:
  static bool box_in_fast_domain(const Aabb2d& box) noexcept {
    return std::isfinite(box.min_x) && std::isfinite(box.max_x) &&
           std::isfinite(box.min_y) && std::isfinite(box.max_y) &&
           std::abs(box.min_x) <= kFastMaxAbsCoordinate &&
           std::abs(box.max_x) <= kFastMaxAbsCoordinate &&
           std::abs(box.min_y) <= kFastMaxAbsCoordinate &&
           std::abs(box.max_y) <= kFastMaxAbsCoordinate;
  }
};

using PolygonSatCache8 = PolygonSatCache<8>;

}  // namespace planner::collision::internal
