#pragma once

// 安全走廊测试共用的构建函数与独立（测试本地）几何检查器。
//
// 该检查器有意重新实现足迹构造、点在多边形内、多边形方向和线段相交，
// 而不调用组件辅助函数，从而用不共享生成器内部实现的几何来验证生成的
// 走廊。这里的距离容差（1e-6 m）是远低于安全裕度的浮点保护量。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "planner/common/types.hpp"
#include "planner/corridor/corridor_generator.hpp"

namespace planner::corridor {
namespace corridor_test {

using common::Corridor;
using common::CorridorPolygon;
using common::Direction;
using common::PathPoint;
using common::PathSegment;
using common::Point2d;
using common::Pose2d;
using common::VehicleModel;

constexpr double kTolM = 1e-6;

inline VehicleModel test_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "default_test_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

inline PathPoint make_point(double x_m, double y_m, double yaw_rad, double s_m,
                            Direction direction = Direction::kForward) {
  PathPoint point;
  point.pose = Pose2d{x_m, y_m, yaw_rad};
  point.s_m = s_m;
  point.curvature_inv_m = 0.0;
  point.direction = direction;
  return point;
}

// 沿 +x 方向的直线种子，航向恒定。
inline PathSegment straight_seed(double x_start, double x_end, double y_m, double yaw_rad,
                                 double step_m = 0.5) {
  PathSegment segment;
  const double length = std::fabs(x_end - x_start);
  std::size_t steps = static_cast<std::size_t>(std::ceil(length / step_m));
  if (steps < 1) {
    steps = 1;
  }
  for (std::size_t k = 0; k <= steps; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(steps);
    segment.points.push_back(make_point(x_start + (x_end - x_start) * t, y_m, yaw_rad,
                                        length * t));
  }
  return segment;
}

// 以原点为中心的圆弧；正扫角为逆时针，航向沿切线方向。
inline PathSegment arc_seed(double radius_m, double start_angle_rad, double sweep_rad,
                            double step_m = 0.25) {
  PathSegment segment;
  const double arc_length = radius_m * std::fabs(sweep_rad);
  std::size_t steps = static_cast<std::size_t>(std::ceil(arc_length / step_m));
  if (steps < 1) {
    steps = 1;
  }
  for (std::size_t k = 0; k <= steps; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(steps);
    const double phi = start_angle_rad + sweep_rad * t;
    const double tangent = sweep_rad >= 0.0 ? phi + 0.5 * common::kPi : phi - 0.5 * common::kPi;
    segment.points.push_back(make_point(radius_m * std::cos(phi), radius_m * std::sin(phi),
                                        common::normalize_angle_rad(tangent), arc_length * t));
  }
  return segment;
}

// 经过 `waypoints` 的圆弧圆角折线，镜像 Dftpav 避障测试夹具
// （Src/dftpav/tests/test_corridor_avoidance.cpp），使走廊回归测试能重放
// 精确的绕行种子。这是测试本地副本；共享夹具位于 Dftpav 组件。
inline PathSegment fillet_seed(const std::vector<Point2d>& waypoints, double radius_m,
                               double step_m = 0.1) {
  PathSegment segment;
  std::vector<Point2d> points;
  std::vector<double> yaws;
  auto push = [&](const Point2d& point, double yaw) {
    points.push_back(point);
    yaws.push_back(common::normalize_angle_rad(yaw));
  };
  auto unit = [](const Point2d& a, const Point2d& b) {
    const double length = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    return Point2d{(b.x_m - a.x_m) / length, (b.y_m - a.y_m) / length};
  };
  auto push_straight = [&](const Point2d& from, const Point2d& to) {
    const double length = std::hypot(to.x_m - from.x_m, to.y_m - from.y_m);
    const int count = std::max(1, static_cast<int>(std::ceil(length / step_m)));
    const double yaw = std::atan2(to.y_m - from.y_m, to.x_m - from.x_m);
    for (int k = 1; k <= count; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(count);
      push(Point2d{from.x_m + t * (to.x_m - from.x_m), from.y_m + t * (to.y_m - from.y_m)},
           yaw);
    }
  };
  push(waypoints.front(), 0.0);
  Point2d current = waypoints.front();
  for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
    const Point2d& previous = waypoints[i - 1];
    const Point2d& corner = waypoints[i];
    const Point2d& next = waypoints[i + 1];
    const Point2d u = unit(previous, corner);
    const Point2d v = unit(corner, next);
    const double cross_value = u.x_m * v.y_m - u.y_m * v.x_m;
    const double dot_value = u.x_m * v.x_m + u.y_m * v.y_m;
    const double phi = std::atan2(cross_value, dot_value);
    if (std::fabs(phi) < 1e-9) {
      continue;
    }
    const double tangent = radius_m * std::tan(std::fabs(phi) / 2.0);
    const Point2d p_in{corner.x_m - tangent * u.x_m, corner.y_m - tangent * u.y_m};
    const Point2d p_out{corner.x_m + tangent * v.x_m, corner.y_m + tangent * v.y_m};
    push_straight(current, p_in);
    const double sign = phi > 0.0 ? 1.0 : -1.0;
    const Point2d center{p_in.x_m - sign * radius_m * u.y_m,
                         p_in.y_m + sign * radius_m * u.x_m};
    const double angle_in = std::atan2(p_in.y_m - center.y_m, p_in.x_m - center.x_m);
    const double angle_out = std::atan2(p_out.y_m - center.y_m, p_out.x_m - center.x_m);
    double sweep = angle_out - angle_in;
    while (sweep <= -common::kPi) {
      sweep += 2.0 * common::kPi;
    }
    while (sweep > common::kPi) {
      sweep -= 2.0 * common::kPi;
    }
    const int count =
        std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) * radius_m / step_m)));
    for (int k = 1; k <= count; ++k) {
      const double angle =
          angle_in + sweep * (static_cast<double>(k) / static_cast<double>(count));
      push(Point2d{center.x_m + radius_m * std::cos(angle),
                   center.y_m + radius_m * std::sin(angle)},
           std::atan2(sign * std::cos(angle), -sign * std::sin(angle)));
    }
    current = p_out;
  }
  push_straight(current, waypoints.back());

  double s_m = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      s_m += std::hypot(points[i].x_m - points[i - 1].x_m, points[i].y_m - points[i - 1].y_m);
    }
    segment.points.push_back(make_point(points[i].x_m, points[i].y_m, yaws[i], s_m));
  }
  return segment;
}

inline CorridorRequest make_request(std::vector<PathSegment> seed,
                                    std::vector<Obstacle> obstacles) {
  CorridorRequest request;
  request.request_id = "test";
  request.map_id = "test_map";
  request.map_version = "v1";
  request.frame_id = "map";
  request.vehicle = test_vehicle();
  request.seed = std::move(seed);
  request.obstacles = std::move(obstacles);
  return request;
}

// ---------------------------------------------------------------------------
// 独立几何
// ---------------------------------------------------------------------------

struct LocalFootprint {
  double rear_m{0.0};
  double front_m{0.0};
  double half_width_m{0.0};
};

inline LocalFootprint local_footprint(const VehicleModel& vehicle) {
  LocalFootprint footprint;
  footprint.rear_m = vehicle.rear_overhang_m + vehicle.safety_margin_m;
  footprint.front_m = vehicle.wheelbase_m + vehicle.front_overhang_m + vehicle.safety_margin_m;
  footprint.half_width_m = 0.5 * vehicle.width_m + vehicle.safety_margin_m;
  return footprint;
}

inline std::array<Point2d, 4> local_footprint_corners(const Pose2d& pose,
                                                      const LocalFootprint& footprint) {
  const double body_x[4] = {footprint.front_m, footprint.front_m, -footprint.rear_m,
                            -footprint.rear_m};
  const double body_y[4] = {-footprint.half_width_m, footprint.half_width_m,
                            footprint.half_width_m, -footprint.half_width_m};
  const double cosine = std::cos(pose.yaw_rad);
  const double sine = std::sin(pose.yaw_rad);
  std::array<Point2d, 4> corners;
  for (std::size_t i = 0; i < 4; ++i) {
    corners[i] = Point2d{pose.x_m + cosine * body_x[i] - sine * body_y[i],
                         pose.y_m + sine * body_x[i] + cosine * body_y[i]};
  }
  return corners;
}

inline double local_cross(const Point2d& a, const Point2d& b) {
  return a.x_m * b.y_m - a.y_m * b.x_m;
}

inline Point2d local_subtract(const Point2d& a, const Point2d& b) {
  return Point2d{a.x_m - b.x_m, a.y_m - b.y_m};
}

inline double local_area(const std::vector<Point2d>& polygon) {
  double sum = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    sum += local_cross(polygon[i], polygon[(i + 1) % polygon.size()]);
  }
  return 0.5 * sum;
}

inline bool local_inside_or_on(const std::vector<Point2d>& polygon_ccw, const Point2d& point,
                               double tolerance) {
  for (std::size_t i = 0; i < polygon_ccw.size(); ++i) {
    const Point2d& a = polygon_ccw[i];
    const Point2d& b = polygon_ccw[(i + 1) % polygon_ccw.size()];
    if (local_cross(local_subtract(b, a), local_subtract(point, a)) < -tolerance) {
      return false;
    }
  }
  return true;
}

inline bool local_strictly_inside(const std::vector<Point2d>& polygon_ccw, const Point2d& point,
                                  double tolerance) {
  for (std::size_t i = 0; i < polygon_ccw.size(); ++i) {
    const Point2d& a = polygon_ccw[i];
    const Point2d& b = polygon_ccw[(i + 1) % polygon_ccw.size()];
    if (local_cross(local_subtract(b, a), local_subtract(point, a)) <= tolerance) {
      return false;
    }
  }
  return true;
}

inline bool local_segments_cross(const Point2d& a, const Point2d& b, const Point2d& c,
                                 const Point2d& d) {
  const double d1 = local_cross(local_subtract(b, a), local_subtract(c, a));
  const double d2 = local_cross(local_subtract(b, a), local_subtract(d, a));
  const double d3 = local_cross(local_subtract(d, c), local_subtract(a, c));
  const double d4 = local_cross(local_subtract(d, c), local_subtract(b, c));
  return ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
         ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
}

// 点位于凸 CCW 多边形外的有符号距离（在内部或边界上时为 0）。对 CCW
// 多边形，内部位于每条有向边的左侧，因此叉积为正表示“在内部”。
inline double local_outside_deficit(const std::vector<Point2d>& polygon_ccw,
                                    const Point2d& point) {
  double worst = 0.0;
  const std::size_t count = polygon_ccw.size();
  for (std::size_t i = 0; i < count; ++i) {
    const Point2d& a = polygon_ccw[i];
    const Point2d& b = polygon_ccw[(i + 1) % count];
    const Point2d edge = local_subtract(b, a);
    const double length = std::hypot(edge.x_m, edge.y_m);
    if (!(length > 1e-12)) {
      continue;
    }
    const double inside_distance = local_cross(edge, local_subtract(point, a)) / length;
    worst = std::max(worst, -inside_distance);
  }
  return worst;
}

// 凸多边形在探测线 x = x_m 处的 y 范围（裁剪切片）。
struct LocalYRange {
  bool exists{false};
  double min_y{0.0};
  double max_y{0.0};
};

inline LocalYRange local_y_range_at_x(const std::vector<Point2d>& polygon_ccw, double x_m,
                                      double eps = 1e-9) {
  LocalYRange range;
  std::vector<Point2d> current = polygon_ccw;
  const double offsets[2] = {-(x_m - eps), x_m + eps};
  const Point2d normals[2] = {Point2d{-1.0, 0.0}, Point2d{1.0, 0.0}};
  for (int plane = 0; plane < 2 && !current.empty(); ++plane) {
    std::vector<Point2d> next;
    const std::size_t count = current.size();
    for (std::size_t i = 0; i < count; ++i) {
      const Point2d& p = current[i];
      const Point2d& q = current[(i + 1) % count];
      const double dp =
          normals[plane].x_m * p.x_m + normals[plane].y_m * p.y_m - offsets[plane];
      const double dq =
          normals[plane].x_m * q.x_m + normals[plane].y_m * q.y_m - offsets[plane];
      const bool p_inside = dp <= 1e-12;
      const bool q_inside = dq <= 1e-12;
      if (p_inside) {
        next.push_back(p);
      }
      if (p_inside != q_inside) {
        const double denominator = dp - dq;
        if (std::fabs(denominator) > 1e-15) {
          const double t = dp / denominator;
          next.push_back(Point2d{p.x_m + t * (q.x_m - p.x_m), p.y_m + t * (q.y_m - p.y_m)});
        }
      }
    }
    current = std::move(next);
  }
  if (current.empty()) {
    return range;
  }
  range.exists = true;
  range.min_y = current.front().y_m;
  range.max_y = current.front().y_m;
  for (const Point2d& point : current) {
    range.min_y = std::min(range.min_y, point.y_m);
    range.max_y = std::max(range.max_y, point.y_m);
  }
  return range;
}

struct FlatPath {
  std::vector<double> s;
  std::vector<Pose2d> poses;
};

inline FlatPath flatten_seed(const std::vector<PathSegment>& seed) {
  FlatPath flat;
  for (const PathSegment& segment : seed) {
    for (const PathPoint& point : segment.points) {
      if (!flat.s.empty()) {
        const bool duplicate_s = std::fabs(point.s_m - flat.s.back()) <= 1e-9;
        const double position_error =
            std::hypot(point.pose.x_m - flat.poses.back().x_m,
                       point.pose.y_m - flat.poses.back().y_m);
        if (duplicate_s && position_error <= 1e-6) {
          continue;
        }
      }
      flat.s.push_back(point.s_m);
      flat.poses.push_back(point.pose);
    }
  }
  return flat;
}

inline Pose2d local_pose_at(const FlatPath& flat, double s) {
  if (s <= flat.s.front()) {
    return flat.poses.front();
  }
  if (s >= flat.s.back()) {
    return flat.poses.back();
  }
  std::size_t index = 0;
  while (index + 1 < flat.s.size() && flat.s[index + 1] < s) {
    ++index;
  }
  const double span = flat.s[index + 1] - flat.s[index];
  const double t = span > 0.0 ? (s - flat.s[index]) / span : 0.0;
  const Pose2d& a = flat.poses[index];
  const Pose2d& b = flat.poses[index + 1];
  double delta_yaw = b.yaw_rad - a.yaw_rad;
  while (delta_yaw >= common::kPi) {
    delta_yaw -= 2.0 * common::kPi;
  }
  while (delta_yaw < -common::kPi) {
    delta_yaw += 2.0 * common::kPi;
  }
  Pose2d pose;
  pose.x_m = a.x_m + (b.x_m - a.x_m) * t;
  pose.y_m = a.y_m + (b.y_m - a.y_m) * t;
  pose.yaw_rad = a.yaw_rad + delta_yaw * t;
  return pose;
}

// ---------------------------------------------------------------------------
// 组合独立检查
// ---------------------------------------------------------------------------

struct LocalCheckResult {
  bool ok{true};
  std::string message;
};

inline LocalCheckResult check_polygons_well_formed(const Corridor& corridor) {
  LocalCheckResult result;
  if (corridor.polygons.empty()) {
    result.ok = false;
    result.message = "corridor has no polygons";
    return result;
  }
  double previous_start = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < corridor.polygons.size(); ++i) {
    const CorridorPolygon& polygon = corridor.polygons[i];
    if (polygon.vertices.size() < 3) {
      result.ok = false;
      result.message = "polygon " + std::to_string(i) + " has fewer than three vertices";
      return result;
    }
    if (!(local_area(polygon.vertices) > 1e-9)) {
      result.ok = false;
      result.message = "polygon " + std::to_string(i) + " is not counter-clockwise";
      return result;
    }
    if (!(polygon.s_end_m > polygon.s_start_m)) {
      result.ok = false;
      result.message = "polygon " + std::to_string(i) + " has an empty s interval";
      return result;
    }
    if (polygon.s_start_m < previous_start - kTolM) {
      result.ok = false;
      result.message = "polygon " + std::to_string(i) + " s order decreased";
      return result;
    }
    previous_start = polygon.s_start_m;
  }
  return result;
}

inline LocalCheckResult check_footprints_covered(const Corridor& corridor,
                                                 const std::vector<PathSegment>& seed,
                                                 const VehicleModel& vehicle,
                                                 double step_m = 0.05) {
  LocalCheckResult result;
  const FlatPath flat = flatten_seed(seed);
  if (flat.s.size() < 2) {
    result.ok = false;
    result.message = "no usable seed points";
    return result;
  }
  const LocalFootprint footprint = local_footprint(vehicle);
  for (std::size_t i = 0; i + 1 < flat.s.size(); ++i) {
    const double span = flat.s[i + 1] - flat.s[i];
    std::size_t substeps = static_cast<std::size_t>(std::ceil(span / step_m));
    if (substeps < 1) {
      substeps = 1;
    }
    for (std::size_t k = 0; k <= substeps; ++k) {
      const double s = (k == substeps)
                           ? flat.s[i + 1]
                           : flat.s[i] + span * (static_cast<double>(k) /
                                                 static_cast<double>(substeps));
      const std::array<Point2d, 4> corners =
          local_footprint_corners(local_pose_at(flat, s), footprint);
      bool covered = false;
      for (const CorridorPolygon& polygon : corridor.polygons) {
        if (s < polygon.s_start_m - kTolM || s > polygon.s_end_m + kTolM) {
          continue;
        }
        bool inside = true;
        for (const Point2d& corner : corners) {
          if (!local_inside_or_on(polygon.vertices, corner, kTolM)) {
            inside = false;
            break;
          }
        }
        if (inside) {
          covered = true;
          break;
        }
      }
      if (!covered) {
        result.ok = false;
        result.message = "footprint at s = " + std::to_string(s) +
                         " m is not contained in any covering polygon";
        return result;
      }
    }
  }
  return result;
}

inline LocalCheckResult check_obstacles_outside(const Corridor& corridor,
                                                const std::vector<Obstacle>& obstacles) {
  LocalCheckResult result;
  for (std::size_t i = 0; i < corridor.polygons.size(); ++i) {
    const std::vector<Point2d>& vertices = corridor.polygons[i].vertices;
    for (const Obstacle& obstacle : obstacles) {
      for (const Point2d& vertex : obstacle.vertices) {
        if (local_strictly_inside(vertices, vertex, 1e-9)) {
          result.ok = false;
          result.message = "obstacle '" + obstacle.id + "' has a vertex inside polygon " +
                           std::to_string(i);
          return result;
        }
      }
      for (std::size_t e = 0; e < vertices.size(); ++e) {
        const Point2d& a = vertices[e];
        const Point2d& b = vertices[(e + 1) % vertices.size()];
        for (std::size_t o = 0; o < obstacle.vertices.size(); ++o) {
          const Point2d& c = obstacle.vertices[o];
          const Point2d& d = obstacle.vertices[(o + 1) % obstacle.vertices.size()];
          if (local_segments_cross(a, b, c, d)) {
            result.ok = false;
            result.message = "obstacle '" + obstacle.id + "' edge crosses polygon " +
                             std::to_string(i) + " edge";
            return result;
          }
        }
      }
    }
  }
  return result;
}

inline LocalCheckResult check_adjacent_polygons_overlap(const Corridor& corridor,
                                                        const std::vector<PathSegment>& seed) {
  LocalCheckResult result;
  const FlatPath flat = flatten_seed(seed);
  for (std::size_t i = 0; i + 1 < corridor.polygons.size(); ++i) {
    const CorridorPolygon& first = corridor.polygons[i];
    const CorridorPolygon& second = corridor.polygons[i + 1];
    if (second.s_start_m > first.s_end_m + kTolM) {
      result.ok = false;
      result.message = "polygon intervals are not contiguous";
      return result;
    }
    const double shared_s = std::min(first.s_end_m, second.s_end_m);
    const Pose2d pose = local_pose_at(flat, shared_s);
    const Point2d point{pose.x_m, pose.y_m};
    if (!local_strictly_inside(first.vertices, point, 1e-9) ||
        !local_strictly_inside(second.vertices, point, 1e-9)) {
      result.ok = false;
      result.message = "adjacent polygons do not overlap around their shared boundary";
      return result;
    }
  }
  return result;
}

inline LocalCheckResult check_corridor(const Corridor& corridor,
                                       const std::vector<PathSegment>& seed,
                                       const std::vector<Obstacle>& obstacles,
                                       const VehicleModel& vehicle) {
  LocalCheckResult result = check_polygons_well_formed(corridor);
  if (!result.ok) {
    return result;
  }
  result = check_footprints_covered(corridor, seed, vehicle);
  if (!result.ok) {
    return result;
  }
  result = check_obstacles_outside(corridor, obstacles);
  if (!result.ok) {
    return result;
  }
  return check_adjacent_polygons_overlap(corridor, seed);
}

// 独立鲁棒性检查（corridor-002）：种子位姿按航向 `+/- yaw_delta_rad` 和
// 位置 `+/- position_delta_m`（x 与 y）扰动后，其每个保守足迹都必须包含
// 在 s 区间覆盖该位姿的走廊多边形内。该检查只使用公开走廊几何和测试
// 本地足迹。
inline LocalCheckResult check_perturbed_footprints(const Corridor& corridor,
                                                   const std::vector<PathSegment>& seed,
                                                   const VehicleModel& vehicle,
                                                   double yaw_delta_rad,
                                                   double position_delta_m,
                                                   double step_m = 0.05) {
  LocalCheckResult result;
  const FlatPath flat = flatten_seed(seed);
  if (flat.s.size() < 2) {
    result.ok = false;
    result.message = "no usable seed points";
    return result;
  }
  const LocalFootprint footprint = local_footprint(vehicle);
  const double yaw_offsets[3] = {-yaw_delta_rad, 0.0, yaw_delta_rad};
  const double position_offsets[3] = {-position_delta_m, 0.0, position_delta_m};
  double worst_deficit = 0.0;
  std::string worst_message = "no perturbation checked";
  for (std::size_t i = 0; i + 1 < flat.s.size(); ++i) {
    const double span = flat.s[i + 1] - flat.s[i];
    std::size_t substeps = static_cast<std::size_t>(std::ceil(span / step_m));
    if (substeps < 1) {
      substeps = 1;
    }
    for (std::size_t k = 0; k <= substeps; ++k) {
      const double s = (k == substeps)
                           ? flat.s[i + 1]
                           : flat.s[i] + span * (static_cast<double>(k) /
                                                 static_cast<double>(substeps));
      const Pose2d base = local_pose_at(flat, s);
      for (const double yaw_offset : yaw_offsets) {
        for (const double x_offset : position_offsets) {
          for (const double y_offset : position_offsets) {
            Pose2d perturbed;
            perturbed.x_m = base.x_m + x_offset;
            perturbed.y_m = base.y_m + y_offset;
            perturbed.yaw_rad = base.yaw_rad + yaw_offset;
            const std::array<Point2d, 4> corners =
                local_footprint_corners(perturbed, footprint);
            double best_deficit = std::numeric_limits<double>::infinity();
            for (const CorridorPolygon& polygon : corridor.polygons) {
              if (s < polygon.s_start_m - kTolM || s > polygon.s_end_m + kTolM) {
                continue;
              }
              double deficit = 0.0;
              for (const Point2d& corner : corners) {
                deficit = std::max(deficit, local_outside_deficit(polygon.vertices, corner));
              }
              best_deficit = std::min(best_deficit, deficit);
            }
            if (!std::isfinite(best_deficit)) {
              result.ok = false;
              result.message = "no corridor polygon covers s = " + std::to_string(s) +
                               " for a perturbed footprint";
              return result;
            }
            if (best_deficit > worst_deficit) {
              worst_deficit = best_deficit;
              worst_message = "worst perturbed footprint deficit " +
                              std::to_string(best_deficit) + " m at s = " + std::to_string(s) +
                              " (dyaw " + std::to_string(yaw_offset) + " rad, dx " +
                              std::to_string(x_offset) + " m, dy " + std::to_string(y_offset) +
                              " m)";
            }
          }
        }
      }
    }
  }
  if (worst_deficit > kTolM) {
    result.ok = false;
    result.message = worst_message;
  }
  return result;
}

}  // namespace corridor_test
}  // namespace planner::corridor
