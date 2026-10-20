#pragma once

// 碰撞组件内部的几何快速路径（不对外安装、不改变公共头文件 API）。
//
// 这里只放“与公共函数判定逐位一致、但把与查询单元无关的量预计算一次”的
// 辅助结构。使用者：collision_checker.cpp 的单元扫描热路径。
//
// 与公共实现的一致性由以下方式保证：
//   * `PolygonSatCache<8>` 的表达式与 geometry.cpp 中
//     `convex_polygon_intersects_aabb` 的逐单元循环逐字相同（同一包围盒
//     min/max、同一边法线构造、同一次斜边/除法、同一投影与比较表达
//     式），因此对任意输入二者返回相同的布尔值；
//   * geometry.cpp 中顶点数 <= Capacity 的多边形直接走本结构（单一实现），
//     顶点数更多的多边形走原循环（本结构标记为不可用）；
//   * notes/sat_cache_pointwise_check.cpp 对随机多边形/包围盒/容差做位级
//     一致性对照（results/evidence/sat_cache_pointwise_check.txt）。

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "planner/collision/geometry.hpp"

namespace planner::collision::internal {

// 预计算的“凸多边形 vs AABB”分离轴缓存。语义与
// `convex_polygon_intersects_aabb(polygon, box, tol)` 完全一致（含空多边形
// 返回 false、退化边跳过、容差内接触计为相交）。
template <std::size_t Capacity>
struct PolygonSatCache {
  static constexpr std::size_t kCapacity = Capacity;

  const Point2d* points{nullptr};
  std::size_t count{0};
  bool usable{false};  // count > Capacity 时为 false（回退公共函数）
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  std::array<Point2d, Capacity> unit_axes{};      // 与边 (i, i+1) 对应
  std::array<std::uint8_t, Capacity> axis_ok{};   // 边法线非退化时为 1

  // 预计算包围盒与逐边单位法线。表达式与原实现逐字相同。
  void build(const Point2d* polygon, std::size_t n) noexcept {
    points = polygon;
    count = n;
    usable = n <= Capacity;
    if (!usable || n == 0) {
      return;  // 空多边形合法（恒不相交）；超容量由调用方回退公共函数
    }
    min_x = std::numeric_limits<double>::infinity();
    max_x = -std::numeric_limits<double>::infinity();
    min_y = std::numeric_limits<double>::infinity();
    max_y = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
      min_x = std::min(min_x, polygon[i].x_m);
      max_x = std::max(max_x, polygon[i].x_m);
      min_y = std::min(min_y, polygon[i].y_m);
      max_y = std::max(max_y, polygon[i].y_m);
    }
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p0 = polygon[i];
      const Point2d& p1 = polygon[(i + 1) % n];
      const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
      const double axis_len = norm(axis);
      if (!(axis_len > 0.0)) {
        axis_ok[i] = 0;  // 退化轴：无法分离（与原实现一致）
        unit_axes[i] = Point2d{0.0, 0.0};
        continue;
      }
      axis_ok[i] = 1;
      unit_axes[i] = Point2d{axis.x_m / axis_len, axis.y_m / axis_len};
    }
  }

  // 凸多边形与轴对齐包围盒的闭集相交测试（分离轴）。返回值与逐单元调用
  // `convex_polygon_intersects_aabb` 逐位一致。
  bool maybe_intersects(const Aabb2d& box, double tol) const noexcept {
    if (count == 0) {
      return false;
    }
    if (!usable) {
      return true;  // 保守方向：调用方自行回退完整判定
    }
    // 与公共实现相同的包围盒快速拒绝。
    if (max_x < box.min_x - tol || min_x > box.max_x + tol ||
        max_y < box.min_y - tol || min_y > box.max_y + tol) {
      return false;
    }
    for (std::size_t i = 0; i < count; ++i) {
      if (axis_ok[i] == 0) {
        continue;
      }
      const Point2d& unit = unit_axes[i];
      double poly_lo = std::numeric_limits<double>::infinity();
      double poly_hi = -std::numeric_limits<double>::infinity();
      for (std::size_t j = 0; j < count; ++j) {
        const double v = points[j].x_m * unit.x_m + points[j].y_m * unit.y_m;
        poly_lo = std::min(poly_lo, v);
        poly_hi = std::max(poly_hi, v);
      }
      const double box_lo = std::min(
          std::min(box.min_x * unit.x_m + box.min_y * unit.y_m,
                   box.max_x * unit.x_m + box.min_y * unit.y_m),
          std::min(box.min_x * unit.x_m + box.max_y * unit.y_m,
                   box.max_x * unit.x_m + box.max_y * unit.y_m));
      const double box_hi = std::max(
          std::max(box.min_x * unit.x_m + box.min_y * unit.y_m,
                   box.max_x * unit.x_m + box.min_y * unit.y_m),
          std::max(box.min_x * unit.x_m + box.max_y * unit.y_m,
                   box.max_x * unit.x_m + box.max_y * unit.y_m));
      if (poly_hi < box_lo - tol || box_hi < poly_lo - tol) {
        return false;
      }
    }
    return true;
  }
};

using PolygonSatCache8 = PolygonSatCache<8>;

}  // namespace planner::collision::internal
