/**
 * @file geometry.cpp
 * @brief 二维计算几何基元、投影分离轴判定与精确欧氏距离函数的具体实现。
 * @details 采用标准双精度浮点运算。所有几何闭集的边界接触在碰撞检测中均保守归类为相交，
 *          确保浮点截断或舍入误差绝不漏判接触碰撞。
 */

#include "geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "geometry_internal.hpp"

namespace planner::collision {

namespace {

constexpr double kEpsScale = 1e-12;

// 快速两点欧氏距离 / 范数计算实现：
// - 常规取值区间 |dx|, |dy| 处于 [1e-150, 1e150) 时，dx^2 + dy^2 绝无溢出或下溢风险，
//   直接调用 std::sqrt 可获得符合 IEEE 754 规范的高性能严格舍入结果。
// - 若超出该安全区间（如极大数/无穷/NaN），平滑回退至标准库 std::hypot。
// - 针对 (0, 0) 输入（相交接触或重合时的极高频输入）提供优先短路分支。
constexpr double kHypotFastMax = 1.0e150;   // (1e150)^2 = 1e300
constexpr double kHypotFastMin = 1.0e-150;  // (1e-150)^2 = 1e-300

double hypot_fast(double dx, double dy) noexcept {
  const double ax = std::abs(dx);
  const double ay = std::abs(dy);
  // (0, 0) 是碰撞检测热点路径中最常见的退化输入（两点重叠或间距为零）
  if (ax == 0.0 && ay == 0.0) {
    return 0.0;
  }
  if (ax >= kHypotFastMax || ay >= kHypotFastMax ||
      (ax < kHypotFastMin && ay < kHypotFastMin)) {
    return std::hypot(dx, dy);
  }
  return std::sqrt(dx * dx + dy * dy);
}

double abs_coordinate_scale(const std::vector<Point2d>& points) {
  double scale = 0.0;
  for (const Point2d& p : points) {
    scale = std::max(scale, std::abs(p.x_m));
    scale = std::max(scale, std::abs(p.y_m));
  }
  return scale;
}

// 未归一化平方域 SAT 的安全取值边界，确保投影与间隙平方乘积不发生浮点溢出。
constexpr double kFastMaxAbsCoordinate =
    internal::PolygonSatCache8::kFastMaxAbsCoordinate;
constexpr double kFastMaxAxisLenSq =
    internal::PolygonSatCache8::kFastMaxAxisLenSq;

bool point_in_fast_domain(const Point2d& p) noexcept {
  return std::isfinite(p.x_m) && std::isfinite(p.y_m) &&
         std::abs(p.x_m) <= kFastMaxAbsCoordinate &&
         std::abs(p.y_m) <= kFastMaxAbsCoordinate;
}

bool polygon_in_fast_domain(const std::vector<Point2d>& points) noexcept {
  for (const Point2d& p : points) {
    if (!point_in_fast_domain(p)) {
      return false;
    }
  }
  return true;
}

bool box_in_fast_domain(const Aabb2d& box) noexcept {
  return std::isfinite(box.min_x) && std::isfinite(box.max_x) &&
         std::isfinite(box.min_y) && std::isfinite(box.max_y) &&
         std::abs(box.min_x) <= kFastMaxAbsCoordinate &&
         std::abs(box.max_x) <= kFastMaxAbsCoordinate &&
         std::abs(box.min_y) <= kFastMaxAbsCoordinate &&
         std::abs(box.max_y) <= kFastMaxAbsCoordinate;
}

// 检查多边形所有有效外法向轴模长是否均落在快速安全域内
bool polygon_axes_in_fast_domain(
    const std::vector<Point2d>& polygon) noexcept {
  const std::size_t n = polygon.size();
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

// 计算点 p 在指定投影轴上的标量投影值
double project(const Point2d& p, const Point2d& axis) noexcept {
  return p.x_m * axis.x_m + p.y_m * axis.y_m;
}

bool separating_axis_normalized(const std::vector<Point2d>& a,
                                const std::vector<Point2d>& b,
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
  for (const Point2d& p : a) {
    const double v = project(p, unit);
    min_a = std::min(min_a, v);
    max_a = std::max(max_a, v);
  }
  for (const Point2d& p : b) {
    const double v = project(p, unit);
    min_b = std::min(min_b, v);
    max_b = std::max(max_b, v);
  }
  return max_a < min_b - tol || max_b < min_a - tol;
}

// 未归一化分离轴判定：在投影间隙平方域比较，彻底免除开方与除法计算。
// 调用方需保证 tol >= 0 且几何数据落在无溢出安全域内。
bool separating_axis_unnormalized(const std::vector<Point2d>& a,
                                  const std::vector<Point2d>& b,
                                  const Point2d& axis,
                                  double tol_sq) noexcept {
  const double axis_len_sq =
      axis.x_m * axis.x_m + axis.y_m * axis.y_m;
  if (!(axis_len_sq > 0.0)) {
    return false;  // 退化轴：无法构成有效分离轴
  }
  double min_a = std::numeric_limits<double>::infinity();
  double max_a = -std::numeric_limits<double>::infinity();
  double min_b = std::numeric_limits<double>::infinity();
  double max_b = -std::numeric_limits<double>::infinity();
  for (const Point2d& p : a) {
    const double v = project(p, axis);
    min_a = std::min(min_a, v);
    max_a = std::max(max_a, v);
  }
  for (const Point2d& p : b) {
    const double v = project(p, axis);
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
bool convex_polygons_intersect_normalized(const std::vector<Point2d>& a,
                                          const std::vector<Point2d>& b,
                                          double tol) noexcept {
  if (a.size() < 3 || b.size() < 3) {
    return convex_polygon_distance(a, b) <= tol;
  }
  for (const auto* polygon : {&a, &b}) {
    const std::size_t n = polygon->size();
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p0 = (*polygon)[i];
      const Point2d& p1 = (*polygon)[(i + 1) % n];
      const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
      if (separating_axis_normalized(a, b, axis, tol)) {
        return false;
      }
    }
  }
  return true;
}

// 通用凸多边形 vs AABB 未归一化相交检测（用于多边形顶点数超过固定栈缓存容量时的快速遍历）
bool convex_polygon_intersects_aabb_unnormalized(
    const Point2d* polygon, std::size_t n, const Aabb2d& box,
    double tol) noexcept {
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
  if (max_x < box.min_x - tol || min_x > box.max_x + tol ||
      max_y < box.min_y - tol || min_y > box.max_y + tol) {
    return false;
  }
  const double tol_sq = tol * tol;
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& p0 = polygon[i];
    const Point2d& p1 = polygon[(i + 1) % n];
    const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
    const double axis_len_sq =
        axis.x_m * axis.x_m + axis.y_m * axis.y_m;
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

// 判定点 p 是否严格位于逆时针凸多边形内部或其边界上。
// 面积容差 (8 * epsilon * scale^2) 用于吸收浮点投影噪声。
bool point_in_ccw_polygon(const Point2d& p,
                          const std::vector<Point2d>& polygon) noexcept {
  const double area_scale = abs_coordinate_scale(polygon) + 1.0;
  const double tol =
      8.0 * std::numeric_limits<double>::epsilon() * area_scale * area_scale;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Point2d& a = polygon[i];
    const Point2d& b = polygon[(i + 1) % polygon.size()];
    const double c = cross(b - a, p - a);
    if (c < -tol) {
      return false;
    }
  }
  return true;
}

}  // namespace

namespace internal {

// 归一化 SAT 回退实现：单位外法向轴结合显式模长与除法运算。
bool convex_polygon_intersects_aabb_normalized(const Point2d* polygon,
                                               std::size_t count,
                                               const Aabb2d& box,
                                               double tol) noexcept {
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
  if (max_x < box.min_x - tol || min_x > box.max_x + tol ||
      max_y < box.min_y - tol || min_y > box.max_y + tol) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const Point2d& p0 = polygon[i];
    const Point2d& p1 = polygon[(i + 1) % count];
    const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
    const double axis_len = norm(axis);
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
// 二维基础向量代数
// ---------------------------------------------------------------------------

double cross(const Point2d& a, const Point2d& b) noexcept {
  return a.x_m * b.y_m - a.y_m * b.x_m;
}

double dot(const Point2d& a, const Point2d& b) noexcept {
  return a.x_m * b.x_m + a.y_m * b.y_m;
}

Point2d operator+(const Point2d& a, const Point2d& b) noexcept {
  return Point2d{a.x_m + b.x_m, a.y_m + b.y_m};
}

Point2d operator-(const Point2d& a, const Point2d& b) noexcept {
  return Point2d{a.x_m - b.x_m, a.y_m - b.y_m};
}

Point2d operator*(const Point2d& a, double s) noexcept {
  return Point2d{a.x_m * s, a.y_m * s};
}

double norm(const Point2d& a) noexcept { return hypot_fast(a.x_m, a.y_m); }

double distance(const Point2d& a, const Point2d& b) noexcept {
  return hypot_fast(a.x_m - b.x_m, a.y_m - b.y_m);
}

double point_segment_distance(const Point2d& p, const Point2d& a,
                              const Point2d& b) noexcept {
  const Point2d ab = b - a;
  const double ab2 = dot(ab, ab);
  if (!(ab2 > 0.0)) {
    return distance(p, a);
  }
  double t = dot(p - a, ab) / ab2;
  t = std::max(0.0, std::min(1.0, t));
  const Point2d proj{a.x_m + t * ab.x_m, a.y_m + t * ab.y_m};
  return distance(p, proj);
}

double point_segment_distance_sq(const Point2d& p, const Point2d& a,
                                 const Point2d& b) noexcept {
  const Point2d ab = b - a;
  const double ab2 = dot(ab, ab);
  if (!(ab2 > 0.0)) {
    const double dx = p.x_m - a.x_m;
    const double dy = p.y_m - a.y_m;
    const double result = dx * dx + dy * dy;
    if (std::isfinite(result)) {
      return result;
    }
    const double d = distance(p, a);
    return d * d;
  }
  double t = dot(p - a, ab) / ab2;
  t = std::max(0.0, std::min(1.0, t));
  const Point2d proj{a.x_m + t * ab.x_m, a.y_m + t * ab.y_m};
  const double dx = p.x_m - proj.x_m;
  const double dy = p.y_m - proj.y_m;
  const double result = dx * dx + dy * dy;
  if (std::isfinite(result)) {
    return result;
  }
  const double d = distance(p, proj);
  return d * d;
}

Point2d transform_point(const Pose2d& pose, const Point2d& body_point) noexcept {
  const double c = std::cos(pose.yaw_rad);
  const double s = std::sin(pose.yaw_rad);
  return Point2d{pose.x_m + c * body_point.x_m - s * body_point.y_m,
                 pose.y_m + s * body_point.x_m + c * body_point.y_m};
}

Point2d to_body_frame(const Pose2d& pose, const Point2d& world_point) noexcept {
  const double c = std::cos(pose.yaw_rad);
  const double s = std::sin(pose.yaw_rad);
  const double dx = world_point.x_m - pose.x_m;
  const double dy = world_point.y_m - pose.y_m;
  return Point2d{c * dx + s * dy, -s * dx + c * dy};
}

// ---------------------------------------------------------------------------
// 尺度相关数值容差
// ---------------------------------------------------------------------------

double contact_tolerance(double scale_m) noexcept {
  return kEpsScale * std::max(1.0, scale_m);
}

// ---------------------------------------------------------------------------
// 二维轴对齐包围盒 (AABB)
// ---------------------------------------------------------------------------

bool Aabb2d::contains(const Point2d& p) const noexcept {
  return p.x_m >= min_x && p.x_m <= max_x && p.y_m >= min_y && p.y_m <= max_y;
}

bool Aabb2d::intersects(const Aabb2d& other) const noexcept {
  return !(other.min_x > max_x || other.max_x < min_x || other.min_y > max_y ||
           other.max_y < min_y);
}

void Aabb2d::expand(double margin) noexcept {
  min_x -= margin;
  min_y -= margin;
  max_x += margin;
  max_y += margin;
}

void Aabb2d::merge_point(const Point2d& p) noexcept {
  min_x = std::min(min_x, p.x_m);
  min_y = std::min(min_y, p.y_m);
  max_x = std::max(max_x, p.x_m);
  max_y = std::max(max_y, p.y_m);
}

double Aabb2d::max_abs_coordinate() const noexcept {
  return std::max(std::max(std::abs(min_x), std::abs(max_x)),
                  std::max(std::abs(min_y), std::abs(max_y)));
}

Aabb2d aabb_of(const std::vector<Point2d>& points) noexcept {
  Aabb2d box;
  if (points.empty()) {
    return box;
  }
  box.min_x = box.max_x = points.front().x_m;
  box.min_y = box.max_y = points.front().y_m;
  for (const Point2d& p : points) {
    box.merge_point(p);
  }
  return box;
}

double aabb_distance(const Aabb2d& a, const Aabb2d& b) noexcept {
  const double dx = std::max(0.0, std::max(a.min_x - b.max_x, b.min_x - a.max_x));
  const double dy = std::max(0.0, std::max(a.min_y - b.max_y, b.min_y - a.max_y));
  return hypot_fast(dx, dy);
}

bool aabb_within(const Aabb2d& a, const Aabb2d& b, double reach) noexcept {
  const double dx = std::max(0.0, std::max(a.min_x - b.max_x, b.min_x - a.max_x));
  const double dy = std::max(0.0, std::max(a.min_y - b.max_y, b.min_y - a.max_y));
  // 逐轴快速上界判别：若任一单轴投影间隙已超过 reach，则空间总距离必然超过 reach
  if (dx > reach || dy > reach) {
    return false;
  }
  const double distance_sq = dx * dx + dy * dy;
  const double reach_sq = reach * reach;
  if (!std::isfinite(distance_sq) || !std::isfinite(reach_sq)) {
    // 针对极值或 inf/nan 等异常输入保留回退行为
    return aabb_distance(a, b) <= reach;
  }
  return distance_sq <= reach_sq;
}

double point_aabb_distance(const Point2d& p, const Aabb2d& box) noexcept {
  const double dx =
      std::max(std::max(box.min_x - p.x_m, 0.0), p.x_m - box.max_x);
  const double dy =
      std::max(std::max(box.min_y - p.y_m, 0.0), p.y_m - box.max_y);
  return hypot_fast(dx, dy);
}

double point_aabb_distance_sq(const Point2d& p, const Aabb2d& box) noexcept {
  const double dx =
      std::max(std::max(box.min_x - p.x_m, 0.0), p.x_m - box.max_x);
  const double dy =
      std::max(std::max(box.min_y - p.y_m, 0.0), p.y_m - box.max_y);
  const double result = dx * dx + dy * dy;
  if (!std::isfinite(result)) {
    const double d = hypot_fast(dx, dy);
    return d * d;
  }
  return result;
}

// ---------------------------------------------------------------------------
// 凸多边形几何算法
// ---------------------------------------------------------------------------

double signed_area_twice(const std::vector<Point2d>& polygon) noexcept {
  double area2 = 0.0;
  const std::size_t n = polygon.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& a = polygon[i];
    const Point2d& b = polygon[(i + 1) % n];
    area2 += a.x_m * b.y_m - b.x_m * a.y_m;
  }
  return area2;
}

bool is_counter_clockwise(const std::vector<Point2d>& polygon) noexcept {
  return signed_area_twice(polygon) > 0.0;
}

void normalize_counter_clockwise(std::vector<Point2d>& polygon) {
  if (!polygon.empty() && signed_area_twice(polygon) < 0.0) {
    std::reverse(polygon.begin(), polygon.end());
  }
}

bool convex_polygon_valid(const std::vector<Point2d>& polygon,
                          std::string* reason) {
  // 去除相邻连续重复顶点及首尾闭合多余顶点
  std::vector<Point2d> clean;
  clean.reserve(polygon.size());
  for (const Point2d& p : polygon) {
    if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m)) {
      if (reason) *reason = "polygon vertex is not finite";
      return false;
    }
    if (clean.empty() || distance(clean.back(), p) > 0.0) {
      clean.push_back(p);
    }
  }
  if (clean.size() >= 2 && distance(clean.front(), clean.back()) == 0.0) {
    clean.pop_back();
  }
  if (clean.size() < 3) {
    if (reason) *reason = "polygon needs at least 3 distinct vertices";
    return false;
  }
  const double scale = abs_coordinate_scale(clean) + 1.0;
  const double area_tol = kEpsScale * scale * scale * 8.0;
  if (std::abs(signed_area_twice(clean)) <= area_tol) {
    if (reason) *reason = "polygon is degenerate (zero area)";
    return false;
  }
  // 凸性检验：转向叉积符号不允许发生翻转（允许共线顶点）
  int sign = 0;
  const std::size_t n = clean.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& a = clean[i];
    const Point2d& b = clean[(i + 1) % n];
    const Point2d& c = clean[(i + 2) % n];
    const double cr = cross(b - a, c - b);
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

std::vector<Point2d> rectangle_polygon(double x_min, double y_min,
                                       double x_max, double y_max,
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
  if (points.size() <= 2) {
    return points;
  }
  std::sort(points.begin(), points.end(), [](const Point2d& a, const Point2d& b) {
    if (a.x_m != b.x_m) return a.x_m < b.x_m;
    return a.y_m < b.y_m;
  });
  points.erase(std::unique(points.begin(), points.end(),
                           [](const Point2d& a, const Point2d& b) {
                             return a.x_m == b.x_m && a.y_m == b.y_m;
                           }),
               points.end());
  if (points.size() <= 2) {
    return points;
  }
  std::vector<Point2d> hull;
  hull.reserve(points.size() + 1);
  const auto cross3 = [](const Point2d& o, const Point2d& a,
                         const Point2d& b) {
    return (a.x_m - o.x_m) * (b.y_m - o.y_m) -
           (a.y_m - o.y_m) * (b.x_m - o.x_m);
  };
  for (const Point2d& p : points) {
    while (hull.size() >= 2 &&
            cross3(hull[hull.size() - 2], hull.back(), p) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(p);
  }
  const std::size_t lower = hull.size();
  for (std::size_t i = points.size() - 1; i-- > 0;) {
    while (hull.size() > lower &&
            cross3(hull[hull.size() - 2], hull.back(), points[i]) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(points[i]);
  }
  hull.pop_back();  // 移除重复的首个起点
  return hull;      // 逆时针凸包
}

bool convex_polygons_intersect(const std::vector<Point2d>& a,
                               const std::vector<Point2d>& b,
                               double tol) noexcept {
  if (a.size() < 3 || b.size() < 3) {
    // 退化几何输入：回退至线段/点欧氏距离判定
    return convex_polygon_distance(a, b) <= tol;
  }
  if (!(tol >= 0.0) || !std::isfinite(tol) || !polygon_in_fast_domain(a) ||
      !polygon_in_fast_domain(b) || !polygon_axes_in_fast_domain(a) ||
      !polygon_axes_in_fast_domain(b)) {
    return convex_polygons_intersect_normalized(a, b, tol);
  }
  const double tol_sq = tol * tol;
  for (const auto* polygon : {&a, &b}) {
    const std::size_t n = polygon->size();
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p0 = (*polygon)[i];
      const Point2d& p1 = (*polygon)[(i + 1) % n];
      // 未归一化外法向量作为投影轴（投影区间能够覆盖正反两个方向）
      const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
      if (separating_axis_unnormalized(a, b, axis, tol_sq)) {
        return false;
      }
    }
  }
  return true;
}

double point_polygon_distance(const Point2d& p,
                              const std::vector<Point2d>& polygon) noexcept {
  if (polygon.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (polygon.size() == 1) {
    return distance(p, polygon.front());
  }
  if (polygon.size() >= 3) {
    bool ccw = is_counter_clockwise(polygon);
    std::vector<Point2d> tmp;
    const std::vector<Point2d>* poly = &polygon;
    if (!ccw) {
      tmp = polygon;
      std::reverse(tmp.begin(), tmp.end());
      poly = &tmp;
    }
    if (point_in_ccw_polygon(p, *poly)) {
      return 0.0;
    }
  }
  double best = std::numeric_limits<double>::infinity();
  const std::size_t n = polygon.size();
  for (std::size_t i = 0; i < n; ++i) {
    best = std::min(best,
                    point_segment_distance(p, polygon[i],
                                           polygon[(i + 1) % n]));
  }
  return best;
}

double point_polygon_distance_sq(const Point2d& p,
                                 const std::vector<Point2d>& polygon) noexcept {
  if (polygon.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (polygon.size() == 1) {
    const double dx = p.x_m - polygon.front().x_m;
    const double dy = p.y_m - polygon.front().y_m;
    const double result = dx * dx + dy * dy;
    if (std::isfinite(result)) {
      return result;
    }
    const double d = distance(p, polygon.front());
    return d * d;
  }
  if (polygon.size() >= 3) {
    bool ccw = is_counter_clockwise(polygon);
    std::vector<Point2d> tmp;
    const std::vector<Point2d>* poly = &polygon;
    if (!ccw) {
      tmp = polygon;
      std::reverse(tmp.begin(), tmp.end());
      poly = &tmp;
    }
    if (point_in_ccw_polygon(p, *poly)) {
      return 0.0;
    }
  }
  double best = std::numeric_limits<double>::infinity();
  const std::size_t n = polygon.size();
  for (std::size_t i = 0; i < n; ++i) {
    best = std::min(best,
                    point_segment_distance_sq(p, polygon[i],
                                              polygon[(i + 1) % n]));
  }
  return best;
}

bool convex_polygon_intersects_aabb(const std::vector<Point2d>& polygon,
                                    const Aabb2d& box, double tol) noexcept {
  if (polygon.empty()) {
    return false;
  }
  // 针对顶点数 <= 8 的多边形（如车体足迹与 8 顶点扫掠包络）走栈缓存快速路径
  if (polygon.size() <= internal::PolygonSatCache8::kCapacity) {
    internal::PolygonSatCache8 cache;
    cache.build(polygon.data(), polygon.size());
    return cache.maybe_intersects(box, tol);
  }
  if (!(tol >= 0.0) || !std::isfinite(tol) ||
      !polygon_in_fast_domain(polygon) || !box_in_fast_domain(box) ||
      !polygon_axes_in_fast_domain(polygon)) {
    return internal::convex_polygon_intersects_aabb_normalized(
        polygon.data(), polygon.size(), box, tol);
  }
  return convex_polygon_intersects_aabb_unnormalized(
      polygon.data(), polygon.size(), box, tol);
}

double segment_segment_distance(const Point2d& a0, const Point2d& a1,
                                const Point2d& b0, const Point2d& b1) noexcept {
  // 标准线段与线段相交测试（包含端点接触和相切）
  const double d1 = cross(a1 - a0, b0 - a0);
  const double d2 = cross(a1 - a0, b1 - a0);
  const double d3 = cross(b1 - b0, a0 - b0);
  const double d4 = cross(b1 - b0, a1 - b0);
  if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
      ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))) {
    return 0.0;
  }
  if (d1 == 0.0 && point_segment_distance(b0, a0, a1) == 0.0) return 0.0;
  if (d2 == 0.0 && point_segment_distance(b1, a0, a1) == 0.0) return 0.0;
  if (d3 == 0.0 && point_segment_distance(a0, b0, b1) == 0.0) return 0.0;
  if (d4 == 0.0 && point_segment_distance(a1, b0, b1) == 0.0) return 0.0;
  return std::min(std::min(point_segment_distance(a0, b0, b1),
                           point_segment_distance(a1, b0, b1)),
                  std::min(point_segment_distance(b0, a0, a1),
                           point_segment_distance(b1, a0, a1)));
}

double segment_segment_distance_sq(const Point2d& a0, const Point2d& a1,
                                   const Point2d& b0,
                                   const Point2d& b1) noexcept {
  if (!point_in_fast_domain(a0) || !point_in_fast_domain(a1) ||
      !point_in_fast_domain(b0) || !point_in_fast_domain(b1)) {
    const double d = segment_segment_distance(a0, a1, b0, b1);
    return d * d;
  }
  // 平方域线段相交测试，逻辑与归一化版本完全对应
  const double d1 = cross(a1 - a0, b0 - a0);
  const double d2 = cross(a1 - a0, b1 - a0);
  const double d3 = cross(b1 - b0, a0 - b0);
  const double d4 = cross(b1 - b0, a1 - b0);
  if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
      ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))) {
    return 0.0;
  }
  if (d1 == 0.0 && point_segment_distance_sq(b0, a0, a1) == 0.0) return 0.0;
  if (d2 == 0.0 && point_segment_distance_sq(b1, a0, a1) == 0.0) return 0.0;
  if (d3 == 0.0 && point_segment_distance_sq(a0, b0, b1) == 0.0) return 0.0;
  if (d4 == 0.0 && point_segment_distance_sq(a1, b0, b1) == 0.0) return 0.0;
  return std::min(std::min(point_segment_distance_sq(a0, b0, b1),
                           point_segment_distance_sq(a1, b0, b1)),
                  std::min(point_segment_distance_sq(b0, a0, a1),
                           point_segment_distance_sq(b1, a0, a1)));
}

double convex_polygon_distance(const std::vector<Point2d>& a,
                               const std::vector<Point2d>& b) noexcept {
  if (a.empty() || b.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const bool a_degenerate = a.size() < 3 || signed_area_twice(a) == 0.0;
  const bool b_degenerate = b.size() < 3 || signed_area_twice(b) == 0.0;
  if (a_degenerate || b_degenerate) {
    double best = std::numeric_limits<double>::infinity();
    const auto edges = [](const std::vector<Point2d>& poly) {
      std::vector<std::pair<Point2d, Point2d>> result;
      if (poly.size() == 1) {
        result.emplace_back(poly.front(), poly.front());
      } else {
        for (std::size_t i = 0; i < poly.size(); ++i) {
          result.emplace_back(poly[i], poly[(i + 1) % poly.size()]);
        }
      }
      return result;
    };
    for (const auto& ea : edges(a)) {
      for (const auto& eb : edges(b)) {
        best = std::min(best, segment_segment_distance(ea.first, ea.second,
                                                       eb.first, eb.second));
      }
    }
    return best;
  }
  if (convex_polygons_intersect_normalized(a, b, 0.0)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  for (const Point2d& p : a) {
    best = std::min(best, point_polygon_distance(p, b));
  }
  for (const Point2d& p : b) {
    best = std::min(best, point_polygon_distance(p, a));
  }
  return best;
}

double convex_polygon_distance_sq(const std::vector<Point2d>& a,
                                  const std::vector<Point2d>& b) noexcept {
  if (a.empty() || b.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  // 坐标超出快速安全域时回退至标准欧氏距离平方
  if (!polygon_in_fast_domain(a) || !polygon_in_fast_domain(b)) {
    const double d = convex_polygon_distance(a, b);
    return d * d;
  }
  const bool a_degenerate = a.size() < 3 || signed_area_twice(a) == 0.0;
  const bool b_degenerate = b.size() < 3 || signed_area_twice(b) == 0.0;
  if (a_degenerate || b_degenerate) {
    double best = std::numeric_limits<double>::infinity();
    const auto edges = [](const std::vector<Point2d>& poly) {
      std::vector<std::pair<Point2d, Point2d>> result;
      if (poly.size() == 1) {
        result.emplace_back(poly.front(), poly.front());
      } else {
        for (std::size_t i = 0; i < poly.size(); ++i) {
          result.emplace_back(poly[i], poly[(i + 1) % poly.size()]);
        }
      }
      return result;
    };
    for (const auto& ea : edges(a)) {
      for (const auto& eb : edges(b)) {
        best = std::min(best, segment_segment_distance_sq(
                                  ea.first, ea.second, eb.first, eb.second));
      }
    }
    return best;
  }
  if (convex_polygons_intersect(a, b, 0.0)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  for (const Point2d& p : a) {
    best = std::min(best, point_polygon_distance_sq(p, b));
  }
  for (const Point2d& p : b) {
    best = std::min(best, point_polygon_distance_sq(p, a));
  }
  return best;
}

double polygon_aabb_distance(const std::vector<Point2d>& polygon,
                             const Aabb2d& box) noexcept {
  double best = std::numeric_limits<double>::infinity();
  for (const Point2d& p : polygon) {
    best = std::min(best, point_aabb_distance(p, box));
    if (best == 0.0) {
      return 0.0;
    }
  }
  const std::array<Point2d, 4> corners{
      Point2d{box.min_x, box.min_y}, Point2d{box.max_x, box.min_y},
      Point2d{box.max_x, box.max_y}, Point2d{box.min_x, box.max_y}};
  if (internal::convex_polygon_intersects_aabb_normalized(
          polygon.data(), polygon.size(), box, 0.0)) {
    return 0.0;
  }
  for (const Point2d& c : corners) {
    best = std::min(best, point_polygon_distance(c, polygon));
  }
  return best;
}

double polygon_aabb_distance_sq(const std::vector<Point2d>& polygon,
                                const Aabb2d& box) noexcept {
  if (polygon.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double best = std::numeric_limits<double>::infinity();
  for (const Point2d& p : polygon) {
    best = std::min(best, point_aabb_distance_sq(p, box));
    if (best == 0.0) {
      return 0.0;
    }
  }
  const std::array<Point2d, 4> corners{
      Point2d{box.min_x, box.min_y}, Point2d{box.max_x, box.min_y},
      Point2d{box.max_x, box.max_y}, Point2d{box.min_x, box.max_y}};
  if (convex_polygon_intersects_aabb(polygon, box, 0.0)) {
    return 0.0;
  }
  for (const Point2d& c : corners) {
    best = std::min(best, point_polygon_distance_sq(c, polygon));
  }
  return best;
}

}  // namespace planner::collision
