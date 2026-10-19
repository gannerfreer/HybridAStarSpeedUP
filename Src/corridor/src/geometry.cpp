// 安全走廊组件的凸几何辅助函数。

#include "planner/corridor/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace planner::corridor {
namespace {

constexpr double kClipEps = 1e-12;

// 去除闭合环中连续重复点与近似共线的顶点。输入环可以首尾重复；输出
// 末尾绝不重复首点。
std::vector<common::Point2d> clean_ring(const std::vector<common::Point2d>& ring,
                                        double eps) {
  std::vector<common::Point2d> points;
  for (const common::Point2d& point : ring) {
    if (!points.empty() && distance(points.back(), point) <= eps) {
      continue;
    }
    points.push_back(point);
  }
  if (points.size() >= 2 && distance(points.front(), points.back()) <= eps) {
    points.pop_back();
  }
  bool changed = true;
  while (changed && points.size() >= 3) {
    changed = false;
    std::vector<common::Point2d> next;
    const std::size_t count = points.size();
    next.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      const common::Point2d& a = points[(i + count - 1) % count];
      const common::Point2d& b = points[i];
      const common::Point2d& c = points[(i + 1) % count];
      const common::Point2d e1 = subtract(b, a);
      const common::Point2d e2 = subtract(c, b);
      const double scale = norm(e1) + norm(e2) + 1.0;
      if (std::fabs(cross(e1, e2)) <= eps * scale) {
        changed = true;
        continue;
      }
      next.push_back(b);
    }
    points.swap(next);
  }
  return points;
}

// CCW 环中边 a -> b 的单位外法向。
common::Point2d outward_normal(const common::Point2d& a, const common::Point2d& b) {
  const common::Point2d edge = subtract(b, a);
  const double length = norm(edge);
  if (!(length > kClipEps)) {
    return common::Point2d{0.0, 0.0};
  }
  return common::Point2d{edge.y_m / length, -edge.x_m / length};
}

// 用 Sutherland-Hodgman 裁剪将 `ring` 对单个半平面裁剪。
std::vector<common::Point2d> clip_ring(const std::vector<common::Point2d>& ring,
                                       const HalfPlane& plane) {
  std::vector<common::Point2d> output;
  if (ring.empty()) {
    return output;
  }
  output.reserve(ring.size() + 1);
  const std::size_t count = ring.size();
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d& p = ring[i];
    const common::Point2d& q = ring[(i + 1) % count];
    const double dp = dot(plane.normal, p) - plane.offset;
    const double dq = dot(plane.normal, q) - plane.offset;
    const bool p_inside = dp <= kClipEps;
    const bool q_inside = dq <= kClipEps;
    if (p_inside) {
      output.push_back(p);
    }
    if (p_inside != q_inside) {
      const double denominator = dp - dq;
      if (std::fabs(denominator) > kClipEps) {
        const double t = dp / denominator;
        output.push_back(add(p, multiply(subtract(q, p), t)));
      }
    }
  }
  return output;
}

}  // namespace

double dot(const common::Point2d& a, const common::Point2d& b) {
  return a.x_m * b.x_m + a.y_m * b.y_m;
}

double cross(const common::Point2d& a, const common::Point2d& b) {
  return a.x_m * b.y_m - a.y_m * b.x_m;
}

common::Point2d add(const common::Point2d& a, const common::Point2d& b) {
  return common::Point2d{a.x_m + b.x_m, a.y_m + b.y_m};
}

common::Point2d subtract(const common::Point2d& a, const common::Point2d& b) {
  return common::Point2d{a.x_m - b.x_m, a.y_m - b.y_m};
}

common::Point2d multiply(const common::Point2d& a, double scalar) {
  return common::Point2d{a.x_m * scalar, a.y_m * scalar};
}

double norm(const common::Point2d& a) { return std::hypot(a.x_m, a.y_m); }

double distance(const common::Point2d& a, const common::Point2d& b) {
  return norm(subtract(a, b));
}

bool is_finite(const common::Point2d& p) {
  return std::isfinite(p.x_m) && std::isfinite(p.y_m);
}

double polygon_signed_area(const std::vector<common::Point2d>& polygon) {
  if (polygon.size() < 3) {
    return 0.0;
  }
  double sum = 0.0;
  const std::size_t count = polygon.size();
  for (std::size_t i = 0; i < count; ++i) {
    sum += cross(polygon[i], polygon[(i + 1) % count]);
  }
  return 0.5 * sum;
}

bool is_convex_ccw_polygon(const std::vector<common::Point2d>& polygon, double eps) {
  const std::size_t count = polygon.size();
  if (count < 3) {
    return false;
  }
  for (const common::Point2d& point : polygon) {
    if (!is_finite(point)) {
      return false;
    }
  }
  if (!(polygon_signed_area(polygon) > eps)) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d e1 = subtract(polygon[(i + 1) % count], polygon[i]);
    const common::Point2d e2 = subtract(polygon[(i + 2) % count], polygon[(i + 1) % count]);
    const double scale = norm(e1) * norm(e2);
    if (!(scale > eps * eps)) {
      return false;
    }
    if (!(cross(e1, e2) > eps * scale)) {
      return false;
    }
  }
  return true;
}

std::vector<common::Point2d> convex_hull(std::vector<common::Point2d> points, double eps) {
  std::sort(points.begin(), points.end(),
            [](const common::Point2d& a, const common::Point2d& b) {
              if (a.x_m != b.x_m) {
                return a.x_m < b.x_m;
              }
              return a.y_m < b.y_m;
            });
  points.erase(std::unique(points.begin(), points.end(),
                           [eps](const common::Point2d& a, const common::Point2d& b) {
                             return distance(a, b) <= eps;
                           }),
               points.end());
  const std::size_t count = points.size();
  if (count < 3) {
    return points;
  }
  std::vector<common::Point2d> hull(2 * count);
  std::size_t k = 0;
  for (std::size_t i = 0; i < count; ++i) {
    while (k >= 2 &&
           cross(subtract(hull[k - 1], hull[k - 2]), subtract(points[i], hull[k - 1])) <= 0.0) {
      --k;
    }
    hull[k++] = points[i];
  }
  const std::size_t lower = k + 1;
  for (std::size_t i = count - 1; i-- > 0;) {
    while (k >= lower &&
           cross(subtract(hull[k - 1], hull[k - 2]), subtract(points[i], hull[k - 1])) <= 0.0) {
      --k;
    }
    hull[k++] = points[i];
  }
  hull.resize(k - 1);
  return hull;
}

bool point_in_convex_polygon_ccw(const common::Point2d& point,
                                 const std::vector<common::Point2d>& polygon_ccw,
                                 double eps) {
  const std::size_t count = polygon_ccw.size();
  if (count < 3 || !is_finite(point)) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d& a = polygon_ccw[i];
    const common::Point2d& b = polygon_ccw[(i + 1) % count];
    const double value = cross(subtract(b, a), subtract(point, a));
    const double scale = std::max(1.0, norm(subtract(b, a)));
    if (value < -eps * scale) {
      return false;
    }
  }
  return true;
}

double convex_polygons_gap(const std::vector<common::Point2d>& a_ccw,
                           const std::vector<common::Point2d>& b_ccw) {
  if (a_ccw.size() < 3 || b_ccw.size() < 3) {
    return -std::numeric_limits<double>::infinity();
  }
  double best = -std::numeric_limits<double>::infinity();
  auto consider_axis = [&](const common::Point2d& normal) {
    if (!(norm(normal) > 0.5)) {
      return;
    }
    double min_a = std::numeric_limits<double>::infinity();
    double max_a = -std::numeric_limits<double>::infinity();
    double min_b = std::numeric_limits<double>::infinity();
    double max_b = -std::numeric_limits<double>::infinity();
    for (const common::Point2d& p : a_ccw) {
      const double value = dot(normal, p);
      min_a = std::min(min_a, value);
      max_a = std::max(max_a, value);
    }
    for (const common::Point2d& p : b_ccw) {
      const double value = dot(normal, p);
      min_b = std::min(min_b, value);
      max_b = std::max(max_b, value);
    }
    best = std::max(best, std::max(min_a - max_b, min_b - max_a));
  };
  for (std::size_t i = 0; i < a_ccw.size(); ++i) {
    consider_axis(outward_normal(a_ccw[i], a_ccw[(i + 1) % a_ccw.size()]));
  }
  for (std::size_t i = 0; i < b_ccw.size(); ++i) {
    consider_axis(outward_normal(b_ccw[i], b_ccw[(i + 1) % b_ccw.size()]));
  }
  return best;
}

bool convex_polygon_contains(const std::vector<common::Point2d>& outer_ccw,
                             const std::vector<common::Point2d>& inner_ccw,
                             double eps) {
  if (outer_ccw.size() < 3) {
    return false;
  }
  for (std::size_t i = 0; i < outer_ccw.size(); ++i) {
    const common::Point2d& a = outer_ccw[i];
    const common::Point2d& b = outer_ccw[(i + 1) % outer_ccw.size()];
    const common::Point2d normal = outward_normal(a, b);
    if (!(norm(normal) > 0.5)) {
      continue;
    }
    const double offset = dot(normal, a);
    for (const common::Point2d& p : inner_ccw) {
      if (dot(normal, p) > offset + eps) {
        return false;
      }
    }
  }
  return true;
}

double convex_polygon_intersection_area(const std::vector<common::Point2d>& a_ccw,
                                        const std::vector<common::Point2d>& b_ccw) {
  if (a_ccw.size() < 3 || b_ccw.size() < 3) {
    return 0.0;
  }
  std::vector<common::Point2d> current = a_ccw;
  for (const HalfPlane& plane : polygon_half_planes(b_ccw)) {
    current = clean_ring(clip_ring(current, plane), kClipEps);
    if (current.size() < 3) {
      return 0.0;
    }
  }
  return std::max(0.0, polygon_signed_area(current));
}

double convex_polygon_min_width(const std::vector<common::Point2d>& polygon_ccw) {
  const std::size_t count = polygon_ccw.size();
  if (count < 3 || !(polygon_signed_area(polygon_ccw) > kGeometryEps)) {
    return std::numeric_limits<double>::infinity();
  }
  auto width_along = [&](const common::Point2d& normal) {
    const double length = norm(normal);
    if (!(length > kClipEps)) {
      return std::numeric_limits<double>::infinity();
    }
    const common::Point2d unit{normal.x_m / length, normal.y_m / length};
    double min_value = std::numeric_limits<double>::infinity();
    double max_value = -std::numeric_limits<double>::infinity();
    for (const common::Point2d& p : polygon_ccw) {
      const double value = dot(unit, p);
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
    }
    return max_value - min_value;
  };
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d e = subtract(polygon_ccw[(i + 1) % count], polygon_ccw[i]);
    best = std::min(best, width_along(common::Point2d{e.y_m, -e.x_m}));
  }
  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t j = i + 1; j < count; ++j) {
      const common::Point2d e = subtract(polygon_ccw[j], polygon_ccw[i]);
      best = std::min(best, width_along(common::Point2d{e.y_m, -e.x_m}));
    }
  }
  return best;
}

std::vector<HalfPlane> polygon_half_planes(const std::vector<common::Point2d>& polygon_ccw) {
  std::vector<HalfPlane> planes;
  planes.reserve(polygon_ccw.size());
  const std::size_t count = polygon_ccw.size();
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d& a = polygon_ccw[i];
    const common::Point2d& b = polygon_ccw[(i + 1) % count];
    const common::Point2d normal = outward_normal(a, b);
    if (!(norm(normal) > 0.5)) {
      continue;
    }
    planes.push_back(HalfPlane{normal, dot(normal, a)});
  }
  return planes;
}

bool intersect_half_planes(const std::vector<HalfPlane>& half_planes,
                           const std::vector<common::Point2d>& bounds_ccw,
                           std::vector<common::Point2d>* out) {
  if (out == nullptr || bounds_ccw.size() < 3) {
    return false;
  }
  std::vector<common::Point2d> current = bounds_ccw;
  for (const HalfPlane& plane : half_planes) {
    current = clip_ring(current, plane);
    current = clean_ring(current, kClipEps);
    if (current.size() < 3) {
      return false;
    }
  }
  current = clean_ring(current, kGeometryEps);
  if (!is_convex_ccw_polygon(current)) {
    return false;
  }
  *out = std::move(current);
  return true;
}

bool offset_convex_polygon(const std::vector<common::Point2d>& polygon_ccw,
                           double offset_m, std::vector<common::Point2d>* out) {
  if (out == nullptr || !(offset_m >= 0.0) || polygon_ccw.size() < 3 ||
      !(polygon_signed_area(polygon_ccw) > kGeometryEps)) {
    return false;
  }
  for (const common::Point2d& point : polygon_ccw) {
    if (!is_finite(point)) {
      return false;
    }
  }
  std::vector<HalfPlane> planes = polygon_half_planes(polygon_ccw);
  if (planes.size() < 3) {
    return false;
  }
  for (HalfPlane& plane : planes) {
    plane.offset += offset_m;
  }
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const common::Point2d& point : polygon_ccw) {
    min_x = std::min(min_x, point.x_m);
    min_y = std::min(min_y, point.y_m);
    max_x = std::max(max_x, point.x_m);
    max_y = std::max(max_y, point.y_m);
  }
  const double inflation = offset_m + 1.0;
  const std::vector<common::Point2d> bounds{
      common::Point2d{min_x - inflation, min_y - inflation},
      common::Point2d{max_x + inflation, min_y - inflation},
      common::Point2d{max_x + inflation, max_y + inflation},
      common::Point2d{min_x - inflation, max_y + inflation}};
  return intersect_half_planes(planes, bounds, out);
}

}  // namespace planner::corridor
