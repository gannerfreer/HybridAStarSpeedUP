/**
 * @file polygon2d.cpp
 * @brief 二维凸多边形基元、投影分离轴与欧氏距离实现
 * @note 按照双精度浮点设计！！！
 */

#include "planner/math/polygon2d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "planner/math/detail/geometry_utils.hpp"
#include "planner/math/detail/sat_cache.hpp"
#include "planner/math/segment2d.hpp"

namespace planner::math {
namespace {
std::size_t convex_hull_impl(Point2d* points, std::size_t n, Point2d* chain,
                             Point2d* out) noexcept {
  if (n <= 2) {
    for (std::size_t i = 0; i < n; ++i) out[i] = points[i];
    return n;
  }
  std::sort(points, points + n, [](const Point2d& a, const Point2d& b) {
    if (a.x_m != b.x_m) return a.x_m < b.x_m;
    return a.y_m < b.y_m;
  });
  // 剔除重复顶点
  std::size_t m = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (m > 0 && points[i].x_m == points[m - 1].x_m && points[i].y_m == points[m - 1].y_m) {
      continue;
    }
    points[m++] = points[i];
  }
  if (m <= 2) {
    for (std::size_t i = 0; i < m; ++i) out[i] = points[i];
    return m;
  }
  std::size_t k = 0;
  for (std::size_t i = 0; i < m; ++i) {
    while (k >= 2 && (chain[k - 1] - chain[k - 2]).cross(points[i] - chain[k - 1]) <= 0.0) {
      --k;
    }
    chain[k++] = points[i];
  }
  const std::size_t lower = k;
  for (std::size_t i = m - 1; i-- > 0;) {
    while (k > lower && (chain[k - 1] - chain[k - 2]).cross(points[i] - chain[k - 1]) <= 0.0) {
      --k;
    }
    chain[k++] = points[i];
  }
  // chain 中包含闭合凸包，丢弃重复的闭合点
  --k;
  for (std::size_t i = 0; i < k; ++i) out[i] = chain[i];
  return k;
}
}  // namespace
std::size_t convex_hull_small(std::array<Point2d, 8> points, std::size_t n,
                              std::array<Point2d, 8>& out) noexcept {
  std::array<Point2d, 16> chain{};
  return convex_hull_impl(points.data(), n, chain.data(), out.data());
}

std::optional<Polygon2d> Polygon2d::convex(std::vector<Point2d> vertices, std::string* reason) {
  if (!convex_polygon_valid(vertices, reason)) {
    return std::nullopt;
  }
  normalize_counter_clockwise(vertices);
  Polygon2d polygon;
  polygon.vertices_ = std::move(vertices);
  polygon.aabb_ = aabb_of(polygon.vertices_);
  return polygon;
}

bool Polygon2d::valid(std::string* reason) const { return convex_polygon_valid(vertices_, reason); }

namespace {

double abs_coordinate_scale(const Point2d* points, std::size_t n) {
  double scale = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    scale = std::max(scale, std::abs(points[i].x_m));
    scale = std::max(scale, std::abs(points[i].y_m));
  }
  return scale;
}

// 未归一化平方域 SAT 的安全取值边界，确保投影与间隙平方乘积不发生浮点溢出
constexpr double kFastMaxAxisLenSq = internal::kFastMaxAxisLenSq;

bool polygon_in_fast_domain(const Point2d* points, std::size_t n) noexcept {
  for (std::size_t i = 0; i < n; ++i) {
    if (!internal::coordinates_in_fast_domain(points[i].x_m, points[i].y_m)) {
      return false;
    }
  }
  return true;
}

// 检查多边形所有有效外法向轴模长是否均落在快速安全域内
bool polygon_axes_in_fast_domain(const Point2d* polygon, std::size_t n) noexcept {
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& p0 = polygon[i];
    const Point2d& p1 = polygon[(i + 1) % n];
    const double ax = -(p1.y_m - p0.y_m);
    const double ay = p1.x_m - p0.x_m;
    const double len_sq = ax * ax + ay * ay;
    if (!(len_sq > 0.0)) {
      continue;  // 顶点重合产生的退化轴跳过
    }
    if (!std::isfinite(len_sq) || len_sq > kFastMaxAxisLenSq) {
      return false;
    }
  }
  return true;
}

// 计算点 p 在指定投影轴 axis 上的标量投影值
double project(const Point2d& p, const Point2d& axis) noexcept {
  return p.x_m * axis.x_m + p.y_m * axis.y_m;
}

bool separating_axis_normalized(const Point2d* a, std::size_t na, const Point2d* b, std::size_t nb,
                                const Point2d& axis, double tol) noexcept {
  const double axis_len = std::sqrt(axis.x_m * axis.x_m + axis.y_m * axis.y_m);
  if (!(axis_len > 0.0) || !std::isfinite(axis_len)) {
    return false;  // 退化轴：无法分离两形状
  }
  const Point2d unit{axis.x_m / axis_len, axis.y_m / axis_len};
  double min_a = std::numeric_limits<double>::infinity();
  double max_a = -std::numeric_limits<double>::infinity();
  double min_b = std::numeric_limits<double>::infinity();
  double max_b = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < na; ++i) {
    const double v = project(a[i], unit);
    min_a = std::min(min_a, v);
    max_a = std::max(max_a, v);
  }
  for (std::size_t i = 0; i < nb; ++i) {
    const double v = project(b[i], unit);
    min_b = std::min(min_b, v);
    max_b = std::max(max_b, v);
  }
  return max_a < min_b - tol || max_b < min_a - tol;
}

// 未归一化分离轴判定：在投影间隙平方域比较，彻底免除开方与除法计算
// 调用方需保证 tol >= 0 且几何数据落在无溢出安全域内
bool separating_axis_unnormalized(const Point2d* a, std::size_t na, const Point2d* b,
                                  std::size_t nb, const Point2d& axis, double tol_sq) noexcept {
  const double axis_len_sq = axis.x_m * axis.x_m + axis.y_m * axis.y_m;
  if (!(axis_len_sq > 0.0)) {
    return false;  // 退化轴：无法构成有效分离轴
  }
  double min_a = std::numeric_limits<double>::infinity();
  double max_a = -std::numeric_limits<double>::infinity();
  double min_b = std::numeric_limits<double>::infinity();
  double max_b = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < na; ++i) {
    const double v = project(a[i], axis);
    min_a = std::min(min_a, v);
    max_a = std::max(max_a, v);
  }
  for (std::size_t i = 0; i < nb; ++i) {
    const double v = project(b[i], axis);
    min_b = std::min(min_b, v);
    max_b = std::max(max_b, v);
  }
  const double threshold = tol_sq * axis_len_sq;
  const double gap_above = min_b - max_a;
  if (gap_above > 0.0 && gap_above * gap_above > threshold) {
    return true;
  }
  const double gap_below = min_a - max_b;
  if (gap_below > 0.0 && gap_below * gap_below > threshold) {
    return true;
  }
  return false;
}

// 归一化 SAT 回退实现：与标准分离轴测试在数值上逐位等价
bool convex_polygons_intersect_normalized(const Point2d* a, std::size_t na, const Point2d* b,
                                          std::size_t nb, double tol) noexcept {
  if (na < 3 || nb < 3) {
    return convex_polygon_distance(a, na, b, nb) <= tol;
  }
  for (int which = 0; which < 2; ++which) {
    const Point2d* polygon = which == 0 ? a : b;
    const std::size_t n = which == 0 ? na : nb;
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p0 = polygon[i];
      const Point2d& p1 = polygon[(i + 1) % n];
      const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
      if (separating_axis_normalized(a, na, b, nb, axis, tol)) {
        return false;
      }
    }
  }
  return true;
}

// 通用凸多边形 vs AABB 未归一化相交检测（用于多边形顶点数超过固定栈缓存容量时的快速遍历）
bool convex_polygon_intersects_aabb_unnormalized(const Point2d* polygon, std::size_t n,
                                                 const Aabb2d& box, double tol) noexcept {
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    min_x = std::min(min_x, polygon[i].x_m);
    max_x = std::max(max_x, polygon[i].x_m);
    min_y = std::min(min_y, polygon[i].y_m);
    max_y = std::max(max_y, polygon[i].y_m);
  }
  if (max_x < box.min_x - tol || min_x > box.max_x + tol || max_y < box.min_y - tol ||
      min_y > box.max_y + tol) {
    return false;
  }
  const double tol_sq = tol * tol;
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& p0 = polygon[i];
    const Point2d& p1 = polygon[(i + 1) % n];
    const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
    const double axis_len_sq = axis.x_m * axis.x_m + axis.y_m * axis.y_m;
    if (!(axis_len_sq > 0.0)) {
      continue;
    }
    double poly_lo = std::numeric_limits<double>::infinity();
    double poly_hi = -std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < n; ++j) {
      const double v = project(polygon[j], axis);
      poly_lo = std::min(poly_lo, v);
      poly_hi = std::max(poly_hi, v);
    }
    const double p00 = box.min_x * axis.x_m + box.min_y * axis.y_m;
    const double p10 = box.max_x * axis.x_m + box.min_y * axis.y_m;
    const double p01 = box.min_x * axis.x_m + box.max_y * axis.y_m;
    const double p11 = box.max_x * axis.x_m + box.max_y * axis.y_m;
    const double box_lo = std::min(std::min(p00, p10), std::min(p01, p11));
    const double box_hi = std::max(std::max(p00, p10), std::max(p01, p11));
    const double threshold = tol_sq * axis_len_sq;
    const double gap_above = box_lo - poly_hi;
    if (gap_above > 0.0 && gap_above * gap_above > threshold) {
      return false;
    }
    const double gap_below = poly_lo - box_hi;
    if (gap_below > 0.0 && gap_below * gap_below > threshold) {
      return false;
    }
  }
  return true;
}

double signed_area_twice_raw(const Point2d* polygon, std::size_t n) noexcept {
  double area2 = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& a = polygon[i];
    const Point2d& b = polygon[(i + 1) % n];
    area2 += a.x_m * b.y_m - b.x_m * a.y_m;
  }
  return area2;
}

// 多边形内点判断；reverse 为 true 时按反向索引访问以保持 CCW 语义
bool point_in_ccw_polygon(const Point2d& p, const Point2d* polygon, std::size_t n,
                          bool reverse) noexcept {
  const double area_scale = abs_coordinate_scale(polygon, n) + 1.0;
  // 面积容差 (8 * epsilon * scale^2)
  const double tol = 8.0 * std::numeric_limits<double>::epsilon() * area_scale * area_scale;
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& a = polygon[reverse ? n - 1 - i : i];
    const Point2d& b = polygon[reverse ? n - 1 - ((i + 1) % n) : (i + 1) % n];
    const double c = (b - a).cross(p - a);
    if (c < -tol) {
      return false;
    }
  }
  return true;
}

double point_polygon_distance_raw(const Point2d& p, const Point2d* polygon,
                                  std::size_t n) noexcept {
  if (n == 0) {
    return std::numeric_limits<double>::infinity();
  }
  if (n == 1) {
    return (p - polygon[0]).norm();
  }
  if (n >= 3) {
    const bool ccw = signed_area_twice_raw(polygon, n) > 0.0;
    if (point_in_ccw_polygon(p, polygon, n, !ccw)) {
      return 0.0;
    }
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    best = std::min(best, Segment2d{polygon[i], polygon[(i + 1) % n]}.distance_to(p));
  }
  return best;
}

double point_polygon_distance_sq_raw(const Point2d& p, const Point2d* polygon,
                                     std::size_t n) noexcept {
  if (n == 0) {
    return std::numeric_limits<double>::infinity();
  }
  if (n == 1) {
    const double dx = p.x_m - polygon[0].x_m;
    const double dy = p.y_m - polygon[0].y_m;
    const double result = dx * dx + dy * dy;
    if (std::isfinite(result)) {
      return result;
    }
    const double d = (p - polygon[0]).norm();
    return d * d;
  }
  if (n >= 3) {
    const bool ccw = signed_area_twice_raw(polygon, n) > 0.0;
    if (point_in_ccw_polygon(p, polygon, n, !ccw)) {
      return 0.0;
    }
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    best = std::min(best, Segment2d{polygon[i], polygon[(i + 1) % n]}.distance_sq_to(p));
  }
  return best;
}

std::size_t polygon_edge_count(std::size_t n) noexcept { return n == 1 ? 1 : n; }

Segment2d polygon_edge(const Point2d* polygon, std::size_t n, std::size_t i) noexcept {
  if (n == 1) {
    return Segment2d{polygon[0], polygon[0]};
  }
  return Segment2d{polygon[i], polygon[(i + 1) % n]};
}

}  // namespace

namespace internal {

// 归一化 SAT 回退实现：单位外法向轴结合显式模长与除法运算
bool convex_polygon_intersects_aabb_normalized(const Point2d* polygon, std::size_t count,
                                               const Aabb2d& box, double tol) noexcept {
  if (count == 0) {
    return false;
  }
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < count; ++i) {
    min_x = std::min(min_x, polygon[i].x_m);
    max_x = std::max(max_x, polygon[i].x_m);
    min_y = std::min(min_y, polygon[i].y_m);
    max_y = std::max(max_y, polygon[i].y_m);
  }
  if (max_x < box.min_x - tol || min_x > box.max_x + tol || max_y < box.min_y - tol ||
      min_y > box.max_y + tol) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const Point2d& p0 = polygon[i];
    const Point2d& p1 = polygon[(i + 1) % count];
    const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
    const double axis_len = axis.norm();
    if (!(axis_len > 0.0)) {
      continue;
    }
    const Point2d unit{axis.x_m / axis_len, axis.y_m / axis_len};
    double poly_lo = std::numeric_limits<double>::infinity();
    double poly_hi = -std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j < count; ++j) {
      const double v = project(polygon[j], unit);
      poly_lo = std::min(poly_lo, v);
      poly_hi = std::max(poly_hi, v);
    }
    const double p00 = box.min_x * unit.x_m + box.min_y * unit.y_m;
    const double p10 = box.max_x * unit.x_m + box.min_y * unit.y_m;
    const double p01 = box.min_x * unit.x_m + box.max_y * unit.y_m;
    const double p11 = box.max_x * unit.x_m + box.max_y * unit.y_m;
    const double box_lo = std::min(std::min(p00, p10), std::min(p01, p11));
    const double box_hi = std::max(std::max(p00, p10), std::max(p01, p11));
    if (poly_hi < box_lo - tol || box_hi < poly_lo - tol) {
      return false;
    }
  }
  return true;
}

}  // namespace internal

// ---------------------------------------------------------------------------
// 凸多边形几何算法
// ---------------------------------------------------------------------------

double signed_area_twice(const std::vector<Point2d>& polygon) noexcept {
  return signed_area_twice_raw(polygon.data(), polygon.size());
}

bool is_counter_clockwise(const std::vector<Point2d>& polygon) noexcept {
  return signed_area_twice(polygon) > 0.0;
}

void normalize_counter_clockwise(std::vector<Point2d>& polygon) {
  if (!polygon.empty() && signed_area_twice(polygon) < 0.0) {
    std::reverse(polygon.begin(), polygon.end());
  }
}

bool convex_polygon_valid(const std::vector<Point2d>& polygon, std::string* reason) {
  // 去除相邻连续重复顶点及首尾闭合多余顶点
  std::vector<Point2d> clean;
  clean.reserve(polygon.size());
  for (const Point2d& p : polygon) {
    if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m)) {
      if (reason) *reason = "polygon vertex is not finite";
      return false;
    }
    if (clean.empty() || (clean.back() - p).norm() > 0.0) {
      clean.push_back(p);
    }
  }
  if (clean.size() >= 2 && (clean.front() - clean.back()).norm() == 0.0) {
    clean.pop_back();
  }
  if (clean.size() < 3) {
    if (reason) *reason = "polygon needs at least 3 distinct vertices";
    return false;
  }
  const double scale = abs_coordinate_scale(clean.data(), clean.size()) + 1.0;
  const double area_tol = internal::kEpsScale * scale * scale * 8.0;
  if (std::abs(signed_area_twice(clean)) <= area_tol) {
    if (reason) *reason = "polygon is degenerate (zero area)";
    return false;
  }
  // 凸性检验
  int sign = 0;
  const std::size_t n = clean.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& a = clean[i];
    const Point2d& b = clean[(i + 1) % n];
    const Point2d& c = clean[(i + 2) % n];
    const double cr = (b - a).cross(c - b);
    if (cr > area_tol) {
      if (sign < 0) {
        if (reason) *reason = "polygon is not convex";
        return false;
      }
      sign = 1;
    } else if (cr < -area_tol) {
      if (sign > 0) {
        if (reason) *reason = "polygon is not convex";
        return false;
      }
      sign = -1;
    }
  }
  return true;
}

std::vector<Point2d> rectangle_polygon(double x_min, double y_min, double x_max, double y_max,
                                       std::string* reason) {
  if (!std::isfinite(x_min) || !std::isfinite(y_min) || !std::isfinite(x_max) ||
      !std::isfinite(y_max)) {
    if (reason) *reason = "rectangle coordinates are not finite";
    return {};
  }
  if (x_min > x_max || y_min > y_max) {
    if (reason) *reason = "rectangle has inverted extents";
    return {};
  }
  return {Point2d{x_min, y_min}, Point2d{x_max, y_min}, Point2d{x_max, y_max},
          Point2d{x_min, y_max}};
}

std::vector<Point2d> convex_hull(std::vector<Point2d> points) {
  std::vector<Point2d> chain(2 * points.size());
  std::vector<Point2d> hull(points.size());
  const auto count = convex_hull_impl(points.data(), points.size(), chain.data(), hull.data());
  hull.resize(count);
  return hull;
}

bool convex_polygons_intersect(const Point2d* a, std::size_t na, const Point2d* b, std::size_t nb,
                               double tol) noexcept {
  if (na < 3 || nb < 3) {
    // 退化几何输入：回退至线段/点欧氏距离判定
    return convex_polygon_distance(a, na, b, nb) <= tol;
  }
  if (!(tol >= 0.0) || !std::isfinite(tol) || !polygon_in_fast_domain(a, na) ||
      !polygon_in_fast_domain(b, nb) || !polygon_axes_in_fast_domain(a, na) ||
      !polygon_axes_in_fast_domain(b, nb)) {
    return convex_polygons_intersect_normalized(a, na, b, nb, tol);
  }
  const double tol_sq = tol * tol;
  for (int which = 0; which < 2; ++which) {
    const Point2d* polygon = which == 0 ? a : b;
    const std::size_t n = which == 0 ? na : nb;
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p0 = polygon[i];
      const Point2d& p1 = polygon[(i + 1) % n];
      // 未归一化外法向量作为投影轴（投影区间能够覆盖正反两个方向）
      const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
      if (separating_axis_unnormalized(a, na, b, nb, axis, tol_sq)) {
        return false;
      }
    }
  }
  return true;
}

bool convex_polygons_intersect(const std::vector<Point2d>& a, const std::vector<Point2d>& b,
                               double tol) noexcept {
  return convex_polygons_intersect(a.data(), a.size(), b.data(), b.size(), tol);
}

double point_polygon_distance(const Point2d& p, const Point2d* polygon, std::size_t n) noexcept {
  return point_polygon_distance_raw(p, polygon, n);
}

double point_polygon_distance(const Point2d& p, const std::vector<Point2d>& polygon) noexcept {
  return point_polygon_distance(p, polygon.data(), polygon.size());
}

double point_polygon_distance_sq(const Point2d& p, const Point2d* polygon, std::size_t n) noexcept {
  return point_polygon_distance_sq_raw(p, polygon, n);
}

double point_polygon_distance_sq(const Point2d& p, const std::vector<Point2d>& polygon) noexcept {
  return point_polygon_distance_sq(p, polygon.data(), polygon.size());
}

bool convex_polygon_intersects_aabb(const Point2d* polygon, std::size_t n, const Aabb2d& box,
                                    double tol) noexcept {
  if (n == 0 || polygon == nullptr) {
    return false;
  }
  // 针对顶点数 <= 8 的多边形（如车体足迹与 8 顶点扫掠包络）走栈缓存快速路径
  if (n <= internal::PolygonSatCache8::kCapacity) {
    internal::PolygonSatCache8 cache;
    cache.build(polygon, n);
    return cache.maybe_intersects(box, tol);
  }
  if (!(tol >= 0.0) || !std::isfinite(tol) || !polygon_in_fast_domain(polygon, n) ||
      !internal::box_in_fast_domain(box) || !polygon_axes_in_fast_domain(polygon, n)) {
    return internal::convex_polygon_intersects_aabb_normalized(polygon, n, box, tol);
  }
  return convex_polygon_intersects_aabb_unnormalized(polygon, n, box, tol);
}

bool convex_polygon_intersects_aabb(const std::vector<Point2d>& polygon, const Aabb2d& box,
                                    double tol) noexcept {
  return convex_polygon_intersects_aabb(polygon.data(), polygon.size(), box, tol);
}

double convex_polygon_distance(const Point2d* a, std::size_t na, const Point2d* b,
                               std::size_t nb) noexcept {
  if (na == 0 || nb == 0 || a == nullptr || b == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  const bool a_degenerate = na < 3 || signed_area_twice_raw(a, na) == 0.0;
  const bool b_degenerate = nb < 3 || signed_area_twice_raw(b, nb) == 0.0;
  if (a_degenerate || b_degenerate) {
    double best = std::numeric_limits<double>::infinity();
    const std::size_t edge_count_a = polygon_edge_count(na);
    const std::size_t edge_count_b = polygon_edge_count(nb);
    for (std::size_t i = 0; i < edge_count_a; ++i) {
      const Segment2d ea = polygon_edge(a, na, i);
      for (std::size_t j = 0; j < edge_count_b; ++j) {
        best = std::min(best, ea.distance_to(polygon_edge(b, nb, j)));
      }
    }
    return best;
  }
  if (convex_polygons_intersect_normalized(a, na, b, nb, 0.0)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < na; ++i) {
    best = std::min(best, point_polygon_distance_raw(a[i], b, nb));
  }
  for (std::size_t i = 0; i < nb; ++i) {
    best = std::min(best, point_polygon_distance_raw(b[i], a, na));
  }
  return best;
}

double convex_polygon_distance(const std::vector<Point2d>& a,
                               const std::vector<Point2d>& b) noexcept {
  return convex_polygon_distance(a.data(), a.size(), b.data(), b.size());
}

double convex_polygon_distance_sq(const Point2d* a, std::size_t na, const Point2d* b,
                                  std::size_t nb) noexcept {
  if (na == 0 || nb == 0 || a == nullptr || b == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  // 坐标超出快速安全域时回退至标准欧氏距离平方
  if (!polygon_in_fast_domain(a, na) || !polygon_in_fast_domain(b, nb)) {
    const double d = convex_polygon_distance(a, na, b, nb);
    return d * d;
  }
  const bool a_degenerate = na < 3 || signed_area_twice_raw(a, na) == 0.0;
  const bool b_degenerate = nb < 3 || signed_area_twice_raw(b, nb) == 0.0;
  if (a_degenerate || b_degenerate) {
    double best = std::numeric_limits<double>::infinity();
    const std::size_t edge_count_a = polygon_edge_count(na);
    const std::size_t edge_count_b = polygon_edge_count(nb);
    for (std::size_t i = 0; i < edge_count_a; ++i) {
      const Segment2d ea = polygon_edge(a, na, i);
      for (std::size_t j = 0; j < edge_count_b; ++j) {
        best = std::min(best, ea.distance_sq_to(polygon_edge(b, nb, j)));
      }
    }
    return best;
  }
  if (convex_polygons_intersect(a, na, b, nb, 0.0)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < na; ++i) {
    best = std::min(best, point_polygon_distance_sq_raw(a[i], b, nb));
  }
  for (std::size_t i = 0; i < nb; ++i) {
    best = std::min(best, point_polygon_distance_sq_raw(b[i], a, na));
  }
  return best;
}

double convex_polygon_distance_sq(const std::vector<Point2d>& a,
                                  const std::vector<Point2d>& b) noexcept {
  return convex_polygon_distance_sq(a.data(), a.size(), b.data(), b.size());
}

double polygon_aabb_distance(const Point2d* polygon, std::size_t n, const Aabb2d& box) noexcept {
  if (n == 0 || polygon == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    best = std::min(best, box.distance_to(polygon[i]));
    if (best == 0.0) {
      return 0.0;
    }
  }
  const std::array<Point2d, 4> corners{Point2d{box.min_x, box.min_y}, Point2d{box.max_x, box.min_y},
                                       Point2d{box.max_x, box.max_y},
                                       Point2d{box.min_x, box.max_y}};
  if (internal::convex_polygon_intersects_aabb_normalized(polygon, n, box, 0.0)) {
    return 0.0;
  }
  for (const Point2d& c : corners) {
    best = std::min(best, point_polygon_distance(c, polygon, n));
  }
  return best;
}

double polygon_aabb_distance(const std::vector<Point2d>& polygon, const Aabb2d& box) noexcept {
  return polygon_aabb_distance(polygon.data(), polygon.size(), box);
}

double polygon_aabb_distance_sq(const Point2d* polygon, std::size_t n, const Aabb2d& box) noexcept {
  if (n == 0 || polygon == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < n; ++i) {
    best = std::min(best, box.distance_sq_to(polygon[i]));
    if (best == 0.0) {
      return 0.0;
    }
  }
  const std::array<Point2d, 4> corners{Point2d{box.min_x, box.min_y}, Point2d{box.max_x, box.min_y},
                                       Point2d{box.max_x, box.max_y},
                                       Point2d{box.min_x, box.max_y}};
  if (convex_polygon_intersects_aabb(polygon, n, box, 0.0)) {
    return 0.0;
  }
  for (const Point2d& c : corners) {
    best = std::min(best, point_polygon_distance_sq(c, polygon, n));
  }
  return best;
}

double polygon_aabb_distance_sq(const std::vector<Point2d>& polygon, const Aabb2d& box) noexcept {
  return polygon_aabb_distance_sq(polygon.data(), polygon.size(), box);
}

}  // namespace planner::math
