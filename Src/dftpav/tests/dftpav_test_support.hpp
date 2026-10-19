#pragma once

// planner_dftpav 测试的共享夹具与独立检查。
//
// 本文件中的几何与轨迹检查有意独立于优化器内部：测试只消费公开 API 类型
// （planner::common::TrajectoryPoint / Corridor / VehicleModel）和公开的
// 走廊生成器，因此能发现优化器/流水线中的错误。

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "planner/common/types.hpp"
#include "planner/corridor/corridor_generator.hpp"
#include "planner/dftpav/optimizer.hpp"

namespace planner::dftpav::test {

// ---------------------------------------------------------------------------
// 夹具。
// ---------------------------------------------------------------------------

// Docs/component_development_plan.md 第 7.1 节的默认测试车辆：
// 轴距 2.8 m、宽度 1.8 m、前/后悬 0.9/1.0 m、
// 最大转向角 0.5 rad、安全余量 0.1 m。
inline common::VehicleModel default_vehicle() {
  common::VehicleModel vehicle;
  vehicle.id = "test_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

inline OptimizeLimits default_limits() {
  OptimizeLimits limits;
  limits.max_forward_velocity_mps = 5.0;
  limits.max_backward_velocity_mps = 2.0;
  limits.max_forward_acceleration_mps2 = 4.0;
  limits.max_backward_acceleration_mps2 = 4.0;
  limits.max_forward_curvature_inv_m = 1.0;
  limits.max_backward_curvature_inv_m = 1.0;
  limits.max_lateral_acceleration_mps2 = 5.0;
  limits.max_steering_rate_radps = 10000.0;
  return limits;
}

struct Rect {
  double x_min{0.0};
  double y_min{0.0};
  double x_max{0.0};
  double y_max{0.0};
};

inline std::vector<common::Point2d> rect_vertices_ccw(const Rect& rect) {
  return {{rect.x_min, rect.y_min},
          {rect.x_max, rect.y_min},
          {rect.x_max, rect.y_max},
          {rect.x_min, rect.y_max}};
}

// 折线 -> PathSegment。点沿行驶方向排序；对于后退段，航向为弦方向加 pi
// （RS 约定）。弧长为累计的二维长度。
inline common::PathSegment make_segment(const std::vector<common::Point2d>& points,
                                        common::Direction direction) {
  common::PathSegment segment;
  double s = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      s += std::hypot(points[i].x_m - points[i - 1].x_m, points[i].y_m - points[i - 1].y_m);
    }
    common::PathPoint path_point;
    path_point.pose.x_m = points[i].x_m;
    path_point.pose.y_m = points[i].y_m;
    if (i + 1 < points.size()) {
      double yaw = std::atan2(points[i + 1].y_m - points[i].y_m,
                              points[i + 1].x_m - points[i].x_m);
      if (direction == common::Direction::kReverse) {
        yaw += common::kPi;
      }
      path_point.pose.yaw_rad = common::normalize_angle_rad(yaw);
    } else {
      path_point.pose.yaw_rad =
          segment.points.empty() ? 0.0 : segment.points.back().pose.yaw_rad;
    }
    path_point.s_m = s;
    path_point.curvature_inv_m = 0.0;
    path_point.direction = direction;
    segment.points.push_back(path_point);
  }
  return segment;
}

// 圆角折线：直线段之间用半径为 `radius_m` 的相切圆弧连接（曲率为 1/radius
// 的运动学初值）。`waypoints` 的间距必须足够容纳圆角；这只是测试夹具，
// 不是通用几何算法。
inline common::PathSegment make_fillet_segment(const std::vector<common::Point2d>& waypoints,
                                               double radius_m, double step_m = 0.1) {
  std::vector<common::Point2d> points;
  std::vector<double> yaws;
  auto push = [&](const common::Point2d& p, double yaw) {
    points.push_back(p);
    yaws.push_back(common::normalize_angle_rad(yaw));
  };
  auto unit = [](const common::Point2d& a, const common::Point2d& b) {
    const double d = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    return common::Point2d{(b.x_m - a.x_m) / d, (b.y_m - a.y_m) / d};
  };
  auto push_straight = [&](const common::Point2d& from, const common::Point2d& to) {
    const double length = std::hypot(to.x_m - from.x_m, to.y_m - from.y_m);
    const int count = std::max(1, static_cast<int>(std::ceil(length / step_m)));
    const double yaw = std::atan2(to.y_m - from.y_m, to.x_m - from.x_m);
    for (int k = 1; k <= count; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(count);
      push({from.x_m + t * (to.x_m - from.x_m), from.y_m + t * (to.y_m - from.y_m)}, yaw);
    }
  };

  push(waypoints.front(),
       std::atan2(waypoints[1].y_m - waypoints[0].y_m,
                  waypoints[1].x_m - waypoints[0].x_m));
  common::Point2d current = waypoints.front();
  double current_yaw = 0.0;
  (void)current_yaw;
  for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
    const common::Point2d& previous = waypoints[i - 1];
    const common::Point2d& corner = waypoints[i];
    const common::Point2d& next = waypoints[i + 1];
    const common::Point2d u = unit(previous, corner);
    const common::Point2d v = unit(corner, next);
    const double cross = u.x_m * v.y_m - u.y_m * v.x_m;
    const double dot = u.x_m * v.x_m + u.y_m * v.y_m;
    const double phi = std::atan2(cross, dot);
    if (std::abs(phi) < 1e-9) {
      continue;
    }
    const double tangent = radius_m * std::tan(std::abs(phi) / 2.0);
    const common::Point2d p_in{corner.x_m - tangent * u.x_m, corner.y_m - tangent * u.y_m};
    const common::Point2d p_out{corner.x_m + tangent * v.x_m, corner.y_m + tangent * v.y_m};
    push_straight(current, p_in);
    const double sign = phi > 0 ? 1.0 : -1.0;
    const common::Point2d center{p_in.x_m - sign * radius_m * u.y_m,
                                 p_in.y_m + sign * radius_m * u.x_m};
    const double angle_in = std::atan2(p_in.y_m - center.y_m, p_in.x_m - center.x_m);
    const double angle_out = std::atan2(p_out.y_m - center.y_m, p_out.x_m - center.x_m);
    double sweep = angle_out - angle_in;
    while (sweep <= -common::kPi) sweep += 2.0 * common::kPi;
    while (sweep > common::kPi) sweep -= 2.0 * common::kPi;
    const int count =
        std::max(1, static_cast<int>(std::ceil(std::abs(sweep) * radius_m / step_m)));
    for (int k = 1; k <= count; ++k) {
      const double angle = angle_in + sweep * static_cast<double>(k) / count;
      push({center.x_m + radius_m * std::cos(angle), center.y_m + radius_m * std::sin(angle)},
           std::atan2(sign * std::cos(angle), -sign * std::sin(angle)));
    }
    current = p_out;
  }
  push_straight(current, waypoints.back());

  common::PathSegment segment;
  double s = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      s += std::hypot(points[i].x_m - points[i - 1].x_m, points[i].y_m - points[i - 1].y_m);
    }
    common::PathPoint path_point;
    path_point.pose.x_m = points[i].x_m;
    path_point.pose.y_m = points[i].y_m;
    path_point.pose.yaw_rad = yaws[i];
    path_point.s_m = s;
    path_point.curvature_inv_m = 0.0;
    path_point.direction = common::Direction::kForward;
    segment.points.push_back(path_point);
  }
  return segment;
}

// 折线的密集采样（默认步长 0.1 m），用于喂给期待合理采样密度的
// 走廊生成器。
inline std::vector<common::Point2d> resample_polyline(const std::vector<common::Point2d>& waypoints,
                                                       double step_m = 0.1) {
  std::vector<common::Point2d> points;
  if (waypoints.empty()) {
    return points;
  }
  points.push_back(waypoints.front());
  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const common::Point2d& a = waypoints[i];
    const common::Point2d& b = waypoints[i + 1];
    const double length = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    const int count = std::max(1, static_cast<int>(std::ceil(length / step_m)));
    for (int k = 1; k <= count; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(count);
      points.push_back({a.x_m + t * (b.x_m - a.x_m), a.y_m + t * (b.y_m - a.y_m)});
    }
  }
  return points;
}

// 从 (x0, y0) 到 (x1, y1) 的直线段（单一方向），按 `step_m` 采样。
inline common::PathSegment straight_segment(double x0, double y0, double x1, double y1,
                                            double step_m = 1.0) {
  const double length = std::hypot(x1 - x0, y1 - y0);
  const int count = std::max(1, static_cast<int>(std::ceil(length / step_m)));
  std::vector<common::Point2d> points;
  for (int k = 0; k <= count; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(count);
    points.push_back({x0 + t * (x1 - x0), y0 + t * (y1 - y0)});
  }
  return make_segment(points, common::Direction::kForward);
}

// 每个 (s_start, s_end, rectangle) 条目对应一个凸多边形的走廊；
// 多边形按给定顺序返回。
inline common::Corridor make_rect_corridor(
    const std::vector<std::pair<double, double>>& s_intervals,
    const std::vector<Rect>& rectangles) {
  common::Corridor corridor;
  corridor.id = "test_corridor";
  corridor.map_id = "test_map";
  corridor.map_version = "v1";
  corridor.frame_id = "map";
  for (std::size_t i = 0; i < s_intervals.size(); ++i) {
    common::CorridorPolygon polygon;
    polygon.s_start_m = s_intervals[i].first;
    polygon.s_end_m = s_intervals[i].second;
    polygon.vertices = rect_vertices_ccw(rectangles[i]);
    corridor.polygons.push_back(polygon);
  }
  return corridor;
}

inline common::Corridor single_rect_corridor(double s_end, const Rect& rect) {
  return make_rect_corridor({{0.0, s_end}}, {rect});
}

// 跟随初值的局部走廊：弧长每 `step_m` 一个轴对齐矩形，其大小取区间端点处
// 保守整车足迹的包围盒（沿初值插值，因此多边形覆盖其 s 区间内的每个站点）
// 再外扩 `clearance_m`。模拟走廊生成器的局部多边形。
std::vector<common::Point2d> vehicle_footprint(const common::VehicleModel& vehicle,
                                               const common::TrajectoryPoint& point,
                                               double margin_m);
inline common::Corridor make_footprint_lane_corridor(
    const std::vector<common::PathSegment>& seed, const common::VehicleModel& vehicle,
    double step_m, double clearance_m = 0.05) {
  struct Sample {
    common::Pose2d pose;
    double s{0.0};
  };
  std::vector<Sample> samples;
  double s = 0.0;
  for (const auto& segment : seed) {
    for (const auto& point : segment.points) {
      if (!samples.empty()) {
        const auto& previous = samples.back().pose;
        s += std::hypot(point.pose.x_m - previous.x_m, point.pose.y_m - previous.y_m);
      }
      samples.push_back({point.pose, s});
    }
  }
  common::Corridor corridor;
  corridor.id = "test_lane_corridor";
  corridor.map_id = "test_map";
  corridor.map_version = "v1";
  corridor.frame_id = "map";
  const double total = samples.empty() ? 0.0 : samples.back().s;
  auto pose_at = [&samples](double s_query) {
    if (s_query <= samples.front().s) {
      return samples.front().pose;
    }
    if (s_query >= samples.back().s) {
      return samples.back().pose;
    }
    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
      if (s_query <= samples[i + 1].s) {
        const double ds = samples[i + 1].s - samples[i].s;
        const double t = ds > 1e-12 ? (s_query - samples[i].s) / ds : 0.0;
        common::Pose2d pose;
        pose.x_m = samples[i].pose.x_m +
                   t * (samples[i + 1].pose.x_m - samples[i].pose.x_m);
        pose.y_m = samples[i].pose.y_m +
                   t * (samples[i + 1].pose.y_m - samples[i].pose.y_m);
        const double delta = common::normalize_angle_rad(samples[i + 1].pose.yaw_rad -
                                                         samples[i].pose.yaw_rad);
        pose.yaw_rad = common::normalize_angle_rad(samples[i].pose.yaw_rad + t * delta);
        return pose;
      }
    }
    return samples.back().pose;
  };
  double s_begin = 0.0;
  while (s_begin < total - 1e-9) {
    const double s_end = std::min(s_begin + step_m, total);
    double x_min = std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();
    for (const double s_station : {s_begin, s_end}) {
      common::TrajectoryPoint pose_point;
      pose_point.pose = pose_at(s_station);
      for (const auto& corner :
           vehicle_footprint(vehicle, pose_point, vehicle.safety_margin_m)) {
        x_min = std::min(x_min, corner.x_m);
        y_min = std::min(y_min, corner.y_m);
        x_max = std::max(x_max, corner.x_m);
        y_max = std::max(y_max, corner.y_m);
      }
    }
    common::CorridorPolygon polygon;
    polygon.s_start_m = s_begin;
    polygon.s_end_m = s_end;
    polygon.vertices = {{x_min - clearance_m, y_min - clearance_m},
                        {x_max + clearance_m, y_min - clearance_m},
                        {x_max + clearance_m, y_max + clearance_m},
                        {x_min - clearance_m, y_max + clearance_m}};
    corridor.polygons.push_back(polygon);
    s_begin = s_end;
  }
  return corridor;
}

// ---------------------------------------------------------------------------
// 独立几何检查。
// ---------------------------------------------------------------------------

inline double cross(const common::Point2d& a, const common::Point2d& b,
                    const common::Point2d& c) {
  return (b.x_m - a.x_m) * (c.y_m - a.y_m) - (b.y_m - a.y_m) * (c.x_m - a.x_m);
}

// 两个凸多边形之间的分离轴测试。接触即算相交（计划第 7.1 节：接触即碰撞）。
inline bool convex_polygons_intersect(const std::vector<common::Point2d>& a,
                                      const std::vector<common::Point2d>& b,
                                      double tolerance_m = 0.0) {
  auto separated = [&tolerance_m](const std::vector<common::Point2d>& p,
                                  const std::vector<common::Point2d>& q) {
    for (std::size_t i = 0; i < p.size(); ++i) {
      const common::Point2d& p0 = p[i];
      const common::Point2d& p1 = p[(i + 1) % p.size()];
      const double nx = -(p1.y_m - p0.y_m);
      const double ny = p1.x_m - p0.x_m;
      const double length = std::hypot(nx, ny);
      if (length <= 1e-12) {
        continue;
      }
      double min_p = std::numeric_limits<double>::infinity();
      double max_p = -std::numeric_limits<double>::infinity();
      double min_q = std::numeric_limits<double>::infinity();
      double max_q = -std::numeric_limits<double>::infinity();
      for (const auto& vertex : p) {
        const double projection = (nx * vertex.x_m + ny * vertex.y_m) / length;
        min_p = std::min(min_p, projection);
        max_p = std::max(max_p, projection);
      }
      for (const auto& vertex : q) {
        const double projection = (nx * vertex.x_m + ny * vertex.y_m) / length;
        min_q = std::min(min_q, projection);
        max_q = std::max(max_q, projection);
      }
      if (max_p < min_q - tolerance_m || max_q < min_p - tolerance_m) {
        return true;
      }
    }
    return false;
  };
  return !separated(a, b) && !separated(b, a);
}

inline bool point_in_convex_ccw(const std::vector<common::Point2d>& polygon,
                                const common::Point2d& point, double tolerance_m = 0.0) {
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const common::Point2d& a = polygon[i];
    const common::Point2d& b = polygon[(i + 1) % polygon.size()];
    const double ex = b.x_m - a.x_m;
    const double ey = b.y_m - a.y_m;
    const double length = std::hypot(ex, ey);
    if (length <= 1e-12) {
      return false;
    }
    const double signed_distance = (ey * (point.x_m - a.x_m) - ex * (point.y_m - a.y_m)) / length;
    if (signed_distance > tolerance_m) {
      return false;
    }
  }
  return true;
}

inline double d_cr_of(const common::VehicleModel& vehicle) {
  return (vehicle.front_overhang_m + vehicle.wheelbase_m - vehicle.rear_overhang_m) / 2.0;
}

inline double vehicle_length_of(const common::VehicleModel& vehicle) {
  return vehicle.wheelbase_m + vehicle.front_overhang_m + vehicle.rear_overhang_m;
}

// 矩形足迹的四个角点：中心位于后轴前方 `d_cr` 处，每边按 `margin_m` 扩张。
inline std::vector<common::Point2d> vehicle_footprint(const common::VehicleModel& vehicle,
                                                      const common::TrajectoryPoint& point,
                                                      double margin_m = 0.0) {
  const double half_length = 0.5 * (vehicle_length_of(vehicle) + 2.0 * margin_m);
  const double half_width = 0.5 * (vehicle.width_m + 2.0 * margin_m);
  const double d_cr = d_cr_of(vehicle);
  const double cos_yaw = std::cos(point.pose.yaw_rad);
  const double sin_yaw = std::sin(point.pose.yaw_rad);
  const double corners[4][2] = {{d_cr + half_length, half_width},
                                {d_cr + half_length, -half_width},
                                {d_cr - half_length, -half_width},
                                {d_cr - half_length, half_width}};
  std::vector<common::Point2d> footprint;
  for (const auto& corner : corners) {
    footprint.push_back(
        {point.pose.x_m + cos_yaw * corner[0] - sin_yaw * corner[1],
         point.pose.y_m + sin_yaw * corner[0] + cos_yaw * corner[1]});
  }
  return footprint;
}

// ---------------------------------------------------------------------------
// 轨迹检查。
// ---------------------------------------------------------------------------

struct CheckResult {
  bool ok{true};
  std::string reason;
  double value{0.0};
};

inline CheckResult check_trajectory_semantics(
    const std::vector<common::TrajectoryPoint>& trajectory, const OptimizeLimits& limits,
    double tolerance_factor, int forward_sign = 1) {
  CheckResult result;
  if (trajectory.size() < 2) {
    return {false, "trajectory has fewer than 2 points", 0.0};
  }
  const double max_velocity =
      forward_sign > 0 ? limits.max_forward_velocity_mps : limits.max_backward_velocity_mps;
  const double max_acceleration = forward_sign > 0 ? limits.max_forward_acceleration_mps2
                                                   : limits.max_backward_acceleration_mps2;
  const double max_curvature = forward_sign > 0 ? limits.max_forward_curvature_inv_m
                                                : limits.max_backward_curvature_inv_m;
  const double velocity_tolerance = tolerance_factor * std::max(1.0, max_velocity);
  const double acceleration_tolerance = tolerance_factor * std::max(1.0, max_acceleration);
  const double curvature_tolerance = tolerance_factor * std::max(1.0, max_curvature);
  const double lateral_tolerance =
      tolerance_factor * std::max(1.0, limits.max_lateral_acceleration_mps2);
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const common::TrajectoryPoint& point = trajectory[i];
    if (!(std::isfinite(point.t_s) && std::isfinite(point.pose.x_m) &&
          std::isfinite(point.pose.y_m) && std::isfinite(point.pose.yaw_rad) &&
          std::isfinite(point.velocity_mps) && std::isfinite(point.acceleration_mps2) &&
          std::isfinite(point.curvature_inv_m))) {
      return {false, "non-finite value", point.t_s};
    }
    if (i > 0 && !(point.t_s > trajectory[i - 1].t_s)) {
      return {false, "time not strictly increasing", point.t_s};
    }
    // 每个采样点必须满足*前进*限制集：测试场景都是同向运行段，
    // 混合轨迹由调用方通过 forward_sign 逐采样检查。
    if (std::abs(point.velocity_mps) > max_velocity + velocity_tolerance) {
      return {false, "velocity limit", point.t_s};
    }
    if (std::abs(point.acceleration_mps2) > max_acceleration + acceleration_tolerance) {
      return {false, "acceleration limit", point.t_s};
    }
    if (std::abs(point.velocity_mps) >= 0.1 &&
        std::abs(point.curvature_inv_m) > max_curvature + curvature_tolerance) {
      return {false, "curvature limit", point.t_s};
    }
    const double lateral = point.velocity_mps * point.velocity_mps * point.curvature_inv_m;
    if (std::abs(lateral) > limits.max_lateral_acceleration_mps2 + lateral_tolerance) {
      return {false, "lateral acceleration limit", point.t_s};
    }
  }
  return result;
}

// 把 `points` 投影到初值折线上（与适配器相同的约定）。
inline std::vector<common::Point2d> flatten_seed(const std::vector<common::PathSegment>& seed) {
  std::vector<common::Point2d> polyline;
  for (const auto& segment : seed) {
    for (const auto& point : segment.points) {
      if (polyline.empty() ||
          std::hypot(point.pose.x_m - polyline.back().x_m, point.pose.y_m - polyline.back().y_m) >
              1e-12) {
        polyline.push_back({point.pose.x_m, point.pose.y_m});
      }
    }
  }
  return polyline;
}

inline double project_arc_length(const std::vector<common::Point2d>& polyline,
                                 const common::Point2d& query) {
  double best = std::numeric_limits<double>::infinity();
  double best_s = 0.0;
  double s = 0.0;
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    const common::Point2d& a = polyline[i];
    const common::Point2d& b = polyline[i + 1];
    const double ex = b.x_m - a.x_m;
    const double ey = b.y_m - a.y_m;
    const double length = std::hypot(ex, ey);
    if (length <= 1e-12) {
      continue;
    }
    double t = ((query.x_m - a.x_m) * ex + (query.y_m - a.y_m) * ey) / (length * length);
    t = std::min(1.0, std::max(0.0, t));
    const double px = a.x_m + t * ex;
    const double py = a.y_m + t * ey;
    const double distance = std::hypot(query.x_m - px, query.y_m - py);
    if (distance < best) {
      best = distance;
      best_s = s + t * length;
    }
    s += length;
  }
  return best_s;
}

// 每个采样足迹（余量 0）必须位于走廊并集内——某个覆盖多边形在
// `containment_tolerance_m` 内包含它（组件对软约束残差的临时性容差）——
// 且不得与任何障碍物多边形相交（精确检查）。
inline CheckResult check_corridor_and_obstacles(
    const std::vector<common::TrajectoryPoint>& trajectory, const common::VehicleModel& vehicle,
    const std::vector<common::PathSegment>& seed, const common::Corridor& corridor,
    const std::vector<std::vector<common::Point2d>>& obstacles, double margin_m = 0.0,
    double containment_tolerance_m = 0.05) {
  const std::vector<common::Point2d> polyline = flatten_seed(seed);
  for (const auto& point : trajectory) {
    const double s = project_arc_length(polyline, {point.pose.x_m, point.pose.y_m});
    bool contained = false;
    for (const auto& polygon_entry : corridor.polygons) {
      if (s < polygon_entry.s_start_m - 1e-6 || s > polygon_entry.s_end_m + 1e-6) {
        continue;
      }
      bool polygon_contains = true;
      for (const auto& corner : vehicle_footprint(vehicle, point, margin_m)) {
        if (!point_in_convex_ccw(polygon_entry.vertices, corner, containment_tolerance_m)) {
          polygon_contains = false;
          break;
        }
      }
      if (polygon_contains) {
        contained = true;
        break;
      }
    }
    if (!contained) {
      return {false, "footprint outside every covering corridor polygon at s=" +
                         std::to_string(s),
              point.t_s};
    }
    const auto footprint_with_margin = vehicle_footprint(vehicle, point, margin_m);
    for (const auto& obstacle : obstacles) {
      if (convex_polygons_intersect(footprint_with_margin, obstacle, 1e-9)) {
        return {false, "footprint intersects obstacle", point.t_s};
      }
    }
  }
  return {};
}

// 找到最接近 `t_s` 的采样点（1e-9 以内）。
inline const common::TrajectoryPoint* sample_at(
    const std::vector<common::TrajectoryPoint>& trajectory, double t_s) {
  for (const auto& point : trajectory) {
    if (std::abs(point.t_s - t_s) <= 1e-9) {
      return &point;
    }
  }
  return nullptr;
}

inline double trajectory_length(const std::vector<common::TrajectoryPoint>& trajectory) {
  double total = 0.0;
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    total += std::hypot(trajectory[i].pose.x_m - trajectory[i - 1].pose.x_m,
                        trajectory[i].pose.y_m - trajectory[i - 1].pose.y_m);
  }
  return total;
}

inline double polyline_length(const std::vector<common::PathSegment>& path) {
  double total = 0.0;
  for (const auto& segment : path) {
    for (std::size_t i = 1; i < segment.points.size(); ++i) {
      total += std::hypot(segment.points[i].pose.x_m - segment.points[i - 1].pose.x_m,
                          segment.points[i].pose.y_m - segment.points[i - 1].pose.y_m);
    }
  }
  return total;
}

// 轨迹索引附近的诊断窗口，用于测试失败消息。
inline std::string trajectory_window_diag(
    const std::vector<common::TrajectoryPoint>& trajectory, int center, int radius = 2) {
  std::string text;
  const int begin = std::max(0, center - radius);
  const int end = std::min<int>(static_cast<int>(trajectory.size()) - 1, center + radius);
  for (int i = begin; i <= end && i >= 0; ++i) {
    const common::TrajectoryPoint& point = trajectory[static_cast<std::size_t>(i)];
    text += " [i=" + std::to_string(i) + " t=" + std::to_string(point.t_s) +
            " x=" + std::to_string(point.pose.x_m) + " y=" + std::to_string(point.pose.y_m) +
            " yaw=" + std::to_string(point.pose.yaw_rad) +
            " v=" + std::to_string(point.velocity_mps) +
            " k=" + std::to_string(point.curvature_inv_m) + "]";
  }
  return text;
}

}  // namespace planner::dftpav::test
