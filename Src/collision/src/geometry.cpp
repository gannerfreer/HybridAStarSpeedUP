// 碰撞组件的 2D 几何基础。约定见 geometry.hpp。此处全部是普通 double
// 运算；接触处理有意偏向“重叠”，确保舍入绝不会掩盖接触。

#include "planner/collision/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace planner::collision {

namespace {

constexpr double kEpsScale = 1e-12;

// 快速斜边（热路径上 std::hypot 占比高：callgrind 显示 case_39 约 22%，
// 其中 90% 以上来自这里替换的四个距离函数）。取值范围：
//   * 常规域 |dx|,|dy| ∈ [1e-150, 1e150)：dx²+dy² 无溢出（< 2e300 <
//     DBL_MAX）且不会被舍入到 0（≥ 1e-300 ≫ 最小次正规 4.9e-324），
//     sqrt 为 IEEE 正确舍入，与 glibc hypot 的差异仅在最后一位舍入
//     （hypot 在 glibc 2.31 不保证正确舍入），位级对照见
//     results/evidence/hypot_fast_domain_check.txt；
//   * 该域之外（inf/nan/极大值/两分量同为极小值）一律回退 std::hypot，
//     因此极端输入的返回值与标准库逐位一致。
// 碰撞判定的所有比较阈值 ≥ 1e-12 * scale，ulp 级差值不改变判定方向
// （差分验收 2000/2000 与全量组件测试见 A_B_RESULTS.md）。
constexpr double kHypotFastMax = 1.0e150;   // (1e150)² = 1e300
constexpr double kHypotFastMin = 1.0e-150;  // (1e-150)² = 1e-300

double hypot_fast(double dx, double dy) noexcept {
  const double ax = std::abs(dx);
  const double ay = std::abs(dy);
  // (0,0) 是热路径上最常见的退化输入（相交/包含判定直接得到 0 间隙），
  // callgrind 显示不加此分支时约占 case_39 剩余 hypot 回退的 99.7%。
  // std::hypot(±0,±0) 返回 +0，直接返回与其逐位一致。
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

// p 在 axis 上投影的平方长度（axis 未归一化）。
double project(const Point2d& p, const Point2d& axis) noexcept {
  return p.x_m * axis.x_m + p.y_m * axis.y_m;
}

bool separating_axis(const std::vector<Point2d>& a,
                     const std::vector<Point2d>& b, const Point2d& axis,
                     double tol) noexcept {
  const double axis_len = std::sqrt(axis.x_m * axis.x_m + axis.y_m * axis.y_m);
  if (!(axis_len > 0.0) || !std::isfinite(axis_len)) {
    return false;  // 退化轴：无法分离
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

// 当 `p` 严格位于逆时针凸多边形内部或边界上时返回 true。该容差是纯
// 浮点舍入界（8 * DBL_EPSILON * scale^2，面积单位）：它吸收投影舍入
// 误差，但远低于碰撞接触容差，因此对于位于外部且超出舍入噪声的点，
// `point_polygon_distance` 返回真实欧氏距离。
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

// ---------------------------------------------------------------------------
// 基础向量代数
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
// 容差
// ---------------------------------------------------------------------------

double contact_tolerance(double scale_m) noexcept {
  return kEpsScale * std::max(1.0, scale_m);
}

// ---------------------------------------------------------------------------
// 轴对齐包围盒
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

double point_aabb_distance(const Point2d& p, const Aabb2d& box) noexcept {
  const double dx =
      std::max(std::max(box.min_x - p.x_m, 0.0), p.x_m - box.max_x);
  const double dy =
      std::max(std::max(box.min_y - p.y_m, 0.0), p.y_m - box.max_y);
  return hypot_fast(dx, dy);
}

// ---------------------------------------------------------------------------
// 凸多边形
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
  // 去掉连续重复点（以及重复的闭合顶点）。
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
  // 凸性：转向方向不得反转。允许共线顶点。
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
  hull.pop_back();  // 去掉重复的首点
  return hull;      // 逆时针
}

bool convex_polygons_intersect(const std::vector<Point2d>& a,
                               const std::vector<Point2d>& b,
                               double tol) noexcept {
  if (a.size() < 3 || b.size() < 3) {
    // 退化输入：回退到线段/点距离。
    return convex_polygon_distance(a, b) <= tol;
  }
  for (const auto* polygon : {&a, &b}) {
    const std::size_t n = polygon->size();
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p0 = (*polygon)[i];
      const Point2d& p1 = (*polygon)[(i + 1) % n];
      // 外法线候选：投影区间时两个符号都已覆盖，因此边方向本身即为有效
      // 的分离轴。
      const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
      if (separating_axis(a, b, axis, tol)) {
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

bool convex_polygon_intersects_aabb(const std::vector<Point2d>& polygon,
                                    const Aabb2d& box, double tol) noexcept {
  if (polygon.empty()) {
    return false;
  }
  // 包围盒轴。
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const Point2d& p : polygon) {
    min_x = std::min(min_x, p.x_m);
    max_x = std::max(max_x, p.x_m);
    min_y = std::min(min_y, p.y_m);
    max_y = std::max(max_y, p.y_m);
  }
  if (max_x < box.min_x - tol || min_x > box.max_x + tol ||
      max_y < box.min_y - tol || min_y > box.max_y + tol) {
    return false;
  }
  // 多边形边法线。
  const std::size_t n = polygon.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& p0 = polygon[i];
    const Point2d& p1 = polygon[(i + 1) % n];
    const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
    const double axis_len = norm(axis);
    if (!(axis_len > 0.0)) {
      continue;
    }
    const Point2d unit{axis.x_m / axis_len, axis.y_m / axis_len};
    double poly_lo = std::numeric_limits<double>::infinity();
    double poly_hi = -std::numeric_limits<double>::infinity();
    for (const Point2d& p : polygon) {
      const double v = project(p, unit);
      poly_lo = std::min(poly_lo, v);
      poly_hi = std::max(poly_hi, v);
    }
    const double box_lo = std::min(
        std::min(project(Point2d{box.min_x, box.min_y}, unit),
                 project(Point2d{box.max_x, box.min_y}, unit)),
        std::min(project(Point2d{box.min_x, box.max_y}, unit),
                 project(Point2d{box.max_x, box.max_y}, unit)));
    const double box_hi = std::max(
        std::max(project(Point2d{box.min_x, box.min_y}, unit),
                 project(Point2d{box.max_x, box.min_y}, unit)),
        std::max(project(Point2d{box.min_x, box.max_y}, unit),
                 project(Point2d{box.max_x, box.max_y}, unit)));
    if (poly_hi < box_lo - tol || box_hi < poly_lo - tol) {
      return false;
    }
  }
  return true;
}

double segment_segment_distance(const Point2d& a0, const Point2d& a1,
                                const Point2d& b0, const Point2d& b1) noexcept {
  // 标准线段相交测试（含相切）。
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
  if (convex_polygons_intersect(a, b, 0.0)) {
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

}  // namespace planner::collision
