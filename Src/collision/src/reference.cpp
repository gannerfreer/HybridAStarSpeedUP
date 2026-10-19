// 独立参考实现：直接曲线检查加保守区间细分。本文件有意不与
// src/collision_checker.cpp 共享任何仿真逻辑：多边形重叠使用边相交/包含
// 测试，凸包用本地例程重建，且全程不使用任何加速索引。只有数据类型和
// 容差策略来自 common/scene 头文件。

#include "planner/collision/reference.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "planner/common/types.hpp"

namespace planner::collision {

namespace {

using planner::common::kCellFree;
using planner::common::kCellOccupied;
using planner::common::kCellUnknown;
using planner::common::kPi;
using planner::common::normalize_angle_rad;

constexpr double kRefSeriesBound = 1e-8;
constexpr double kRefMaxAbsCurvature_inv_m = 1.0e6;
constexpr double kRefSafetyFactor = 1.0 + 1e-12;
constexpr double kRefSafetyOffset_m = 1e-12;

bool finite_point(const Point2d& p) {
  return std::isfinite(p.x_m) && std::isfinite(p.y_m);
}

bool finite_pose(const Pose2d& p) {
  return finite_point(Point2d{p.x_m, p.y_m}) && std::isfinite(p.yaw_rad);
}

double ref_sinc(double x) {
  if (std::abs(x) < kRefSeriesBound) {
    const double x2 = x * x;
    return 1.0 - x2 / 6.0 + x2 * x2 / 120.0;
  }
  return std::sin(x) / x;
}

// 自有的圆弧积分：带符号弧长 s 之后的世界位姿。
Pose2d ref_pose_on_arc(const Pose2d& start, double s, double kappa) {
  const double b = kappa * s;
  const double cy = std::cos(start.yaw_rad);
  const double sy = std::sin(start.yaw_rad);
  const double q = 0.5 * b;
  const double quartic = q * ref_sinc(q) * ref_sinc(q);
  const double dsin = cy * ref_sinc(b) - sy * quartic;
  const double dcos = -sy * ref_sinc(b) - cy * quartic;
  Pose2d out;
  out.x_m = start.x_m + s * dsin;
  out.y_m = start.y_m - s * dcos;
  out.yaw_rad = normalize_angle_rad(start.yaw_rad + b);
  return out;
}

// 世界 -> 栅格原点坐标系（自有的一份坐标变换副本）。
Point2d ref_to_grid_frame(const GridFrame& frame, const Point2d& p) {
  const double dx = p.x_m - frame.origin_x;
  const double dy = p.y_m - frame.origin_y;
  return Point2d{frame.cos_yaw * dx + frame.sin_yaw * dy,
                 -frame.sin_yaw * dx + frame.cos_yaw * dy};
}

std::vector<Point2d> ref_grid_corners(const Footprint& footprint,
                                      const GridFrame& frame,
                                      const Pose2d& pose) {
  const std::array<Point2d, 4> world = footprint.world_corners(pose);
  std::vector<Point2d> out;
  out.reserve(4);
  for (const Point2d& p : world) {
    out.push_back(ref_to_grid_frame(frame, p));
  }
  return out;
}

// 点到线段的距离（自有实现）。
double ref_point_segment_distance(const Point2d& p, const Point2d& a,
                                  const Point2d& b) {
  const double abx = b.x_m - a.x_m;
  const double aby = b.y_m - a.y_m;
  const double length2 = abx * abx + aby * aby;
  double t = 0.0;
  if (length2 > 0.0) {
    t = ((p.x_m - a.x_m) * abx + (p.y_m - a.y_m) * aby) / length2;
    t = std::max(0.0, std::min(1.0, t));
  }
  const double qx = a.x_m + t * abx;
  const double qy = a.y_m + t * aby;
  return std::hypot(p.x_m - qx, p.y_m - qy);
}

double ref_segment_distance(const Point2d& a0, const Point2d& a1,
                            const Point2d& b0, const Point2d& b1) {
  // 围绕四个线段端点的方向测试。
  const auto side = [](const Point2d& p, const Point2d& q, const Point2d& r) {
    return (q.x_m - p.x_m) * (r.y_m - p.y_m) -
           (q.y_m - p.y_m) * (r.x_m - p.x_m);
  };
  const double d1 = side(a0, a1, b0);
  const double d2 = side(a0, a1, b1);
  const double d3 = side(b0, b1, a0);
  const double d4 = side(b0, b1, a1);
  const bool proper_crossing =
      ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
      ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
  if (proper_crossing) {
    return 0.0;
  }
  // 共线相切的情况落到端点距离，这也覆盖所有不相交的构型。
  double best = std::min(ref_point_segment_distance(b0, a0, a1),
                         ref_point_segment_distance(b1, a0, a1));
  best = std::min(best, ref_point_segment_distance(a0, b0, b1));
  best = std::min(best, ref_point_segment_distance(a1, b0, b1));
  return best;
}

// 支持两种绕向的严格点在凸多边形内测试。该容差是纯浮点舍入界
// （8 * DBL_EPSILON * scale）；接触容差改为施加在形状间的欧氏距离上，
// 因此角点外的点绝不会因逐边放大的容差被归为“内部”。
bool ref_point_in_polygon(const Point2d& p, const std::vector<Point2d>& poly) {
  if (poly.size() < 3) {
    return false;
  }
  double scale = std::abs(p.x_m);
  scale = std::max(scale, std::abs(p.y_m));
  for (const Point2d& vertex : poly) {
    scale = std::max(scale, std::abs(vertex.x_m));
    scale = std::max(scale, std::abs(vertex.y_m));
  }
  scale += 1.0;
  const double round_tol =
      8.0 * std::numeric_limits<double>::epsilon() * scale;
  std::vector<Point2d> tmp;
  const std::vector<Point2d>* vertices = &poly;
  if (signed_area_twice(poly) < 0.0) {
    tmp = poly;
    std::reverse(tmp.begin(), tmp.end());
    vertices = &tmp;
  }
  const std::size_t n = vertices->size();
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& a = (*vertices)[i];
    const Point2d& b = (*vertices)[(i + 1) % n];
    const double ex = b.x_m - a.x_m;
    const double ey = b.y_m - a.y_m;
    const double edge_len = std::hypot(ex, ey);
    if (!(edge_len > 0.0)) {
      continue;
    }
    const double cr = ex * (p.y_m - a.y_m) - ey * (p.x_m - a.x_m);
    // 到边所在直线的带符号距离；负值表示在外部。
    if (cr / edge_len < -round_tol) {
      return false;
    }
  }
  return true;
}

// 组件接触策略下的多边形重叠：最小欧氏边界距离 <= tol（接触计为
// 碰撞），或一个多边形严格包含另一个多边形的某个顶点。
bool ref_polygons_overlap(const std::vector<Point2d>& a,
                          const std::vector<Point2d>& b, double tol);
double ref_polygon_distance(const std::vector<Point2d>& a,
                            const std::vector<Point2d>& b) {
  if (a.empty() || b.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (a.size() >= 3 && b.size() >= 3 && ref_polygons_overlap(a, b, 0.0)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  const auto edges = [](const std::vector<Point2d>& poly) {
    std::vector<std::pair<Point2d, Point2d>> result;
    if (poly.size() == 1) {
      result.emplace_back(poly[0], poly[0]);
    } else {
      for (std::size_t i = 0; i < poly.size(); ++i) {
        result.emplace_back(poly[i], poly[(i + 1) % poly.size()]);
      }
    }
    return result;
  };
  const auto edges_a = edges(a);
  const auto edges_b = edges(b);
  for (const auto& ea : edges_a) {
    for (const auto& eb : edges_b) {
      best = std::min(best,
                      ref_segment_distance(ea.first, ea.second, eb.first,
                                           eb.second));
    }
  }
  return best;
}

// 含接触的多边形重叠：边界距离 <= tol，或一个多边形包含另一个多边形
// 的某个顶点。
bool ref_polygons_overlap(const std::vector<Point2d>& a,
                          const std::vector<Point2d>& b, double tol) {
  if (a.size() < 3 || b.size() < 3) {
    return ref_polygon_distance(a, b) <= tol;
  }
  const std::size_t na = a.size();
  const std::size_t nb = b.size();
  for (std::size_t i = 0; i < na; ++i) {
    for (std::size_t j = 0; j < nb; ++j) {
      if (ref_segment_distance(a[i], a[(i + 1) % na], b[j],
                               b[(j + 1) % nb]) <= tol) {
        return true;
      }
    }
  }
  if (ref_point_in_polygon(a[0], b) || ref_point_in_polygon(b[0], a)) {
    return true;
  }
  return false;
}

// 本地单调链凸包（自有实现，输入不产生额外分配）。
std::vector<Point2d> ref_convex_hull(std::vector<Point2d> points) {
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
  const auto cross3 = [](const Point2d& o, const Point2d& a,
                         const Point2d& b) {
    return (a.x_m - o.x_m) * (b.y_m - o.y_m) -
           (a.y_m - o.y_m) * (b.x_m - o.x_m);
  };
  std::vector<Point2d> hull;
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
  hull.pop_back();
  return hull;
}

double ref_tolerance(double scale) { return contact_tolerance(scale); }

struct RefPoseHit {
  bool hit{false};
  std::string detail;
};

// 针对地图矩形、障碍物与被阻塞单元的精确位姿检查。
RefPoseHit ref_pose_hit(const Footprint& footprint, const Scene& scene,
                        const GridFrame& frame, const Pose2d& pose) {
  RefPoseHit result;
  const std::vector<Point2d> grid_corners =
      ref_grid_corners(footprint, frame, pose);
  if (scene.has_grid()) {
    for (const Point2d& p : grid_corners) {
      if (!frame.inside_map(p)) {
        result.hit = true;
        result.detail = "footprint leaves the map rectangle";
        return result;
      }
    }
  }
  const std::array<Point2d, 4> world = footprint.world_corners(pose);
  const std::vector<Point2d> world_polygon(world.begin(), world.end());
  for (const Obstacle& obstacle : scene.obstacles) {
    const double tol = ref_tolerance(
        std::max(aabb_of(world_polygon).max_abs_coordinate(),
                 obstacle.aabb().max_abs_coordinate()));
    if (ref_polygons_overlap(world_polygon, obstacle.vertices, tol)) {
      result.hit = true;
      result.detail = "obstacle '" + obstacle.id + "' overlaps the footprint";
      return result;
    }
  }
  if (scene.has_grid()) {
    const GridView& grid = scene.grid;
    const double res = frame.resolution_m;
    double min_x = grid_corners[0].x_m;
    double max_x = grid_corners[0].x_m;
    double min_y = grid_corners[0].y_m;
    double max_y = grid_corners[0].y_m;
    for (const Point2d& p : grid_corners) {
      min_x = std::min(min_x, p.x_m);
      max_x = std::max(max_x, p.x_m);
      min_y = std::min(min_y, p.y_m);
      max_y = std::max(max_y, p.y_m);
    }
    int col0 = static_cast<int>(std::floor(min_x / res)) - 1;
    int col1 = static_cast<int>(std::floor(max_x / res)) + 1;
    int row0 = static_cast<int>(std::floor(min_y / res)) - 1;
    int row1 = static_cast<int>(std::floor(max_y / res)) + 1;
    col0 = std::max(col0, 0);
    row0 = std::max(row0, 0);
    col1 = std::min(col1, grid.width - 1);
    row1 = std::min(row1, grid.height - 1);
    const double scale =
        std::max(std::max(std::abs(min_x), std::abs(max_x)),
                 std::max(std::abs(min_y), std::abs(max_y)));
    const double tol = ref_tolerance(scale);
    for (int row = row0; row <= row1; ++row) {
      for (int col = col0; col <= col1; ++col) {
        const int value =
            grid.cells[static_cast<std::size_t>(row) * grid.width + col];
        if (value == kCellFree) {
          continue;
        }
        const std::vector<Point2d> cell{
            Point2d{col * res, row * res}, Point2d{(col + 1) * res, row * res},
            Point2d{(col + 1) * res, (row + 1) * res},
            Point2d{col * res, (row + 1) * res}};
        if (ref_polygons_overlap(grid_corners, cell, tol)) {
          result.hit = true;
          result.detail = "grid cell (" + std::to_string(row) + "," +
                          std::to_string(col) + ") " +
                          (value == kCellOccupied ? "occupied" : "unknown") +
                          " overlaps the footprint";
          return result;
        }
      }
    }
  }
  return result;
}

// 保守的扫掠区域覆盖：八个端点角点的凸包加上精确的角点圆弧矢高。自有
// 实现。
struct RefEnvelope {
  std::vector<Point2d> hull;
  double sagitta_m{0.0};
};

RefEnvelope ref_build_envelope(const Footprint& footprint, const Pose2d& p0,
                               const Pose2d& p1, double delta_yaw, double kappa) {
  RefEnvelope envelope;
  const std::array<Point2d, 4> c0 = footprint.world_corners(p0);
  const std::array<Point2d, 4> c1 = footprint.world_corners(p1);
  const double abs_yaw = std::abs(delta_yaw);
  if (abs_yaw >= kPi) {
    const double radius = (kappa != 0.0 ? std::abs(1.0 / kappa) : 0.0) +
                          footprint.corner_radius_m;
    envelope.sagitta_m = 2.0 * radius * kRefSafetyFactor + kRefSafetyOffset_m;
  } else if (delta_yaw != 0.0) {
    const double tangent = std::abs(std::tan(0.25 * delta_yaw));
    for (std::size_t i = 0; i < 4; ++i) {
      const double chord =
          std::hypot(c0[i].x_m - c1[i].x_m, c0[i].y_m - c1[i].y_m);
      envelope.sagitta_m = std::max(envelope.sagitta_m, 0.5 * chord * tangent);
    }
    envelope.sagitta_m =
        envelope.sagitta_m * kRefSafetyFactor + kRefSafetyOffset_m;
  }
  std::vector<Point2d> points;
  points.reserve(8);
  for (const Point2d& p : c0) points.push_back(p);
  for (const Point2d& p : c1) points.push_back(p);
  envelope.hull = ref_convex_hull(std::move(points));
  return envelope;
}

bool ref_envelope_clear(const Footprint& footprint, const Scene& scene,
                        const GridFrame& frame, const RefEnvelope& envelope) {
  (void)footprint;  // 为与生产路径保持签名一致而保留
  if (envelope.hull.empty()) {
    return false;
  }
  const Aabb2d hull_box = aabb_of(envelope.hull);
  const double tol = ref_tolerance(hull_box.max_abs_coordinate() + 1.0);
  const double reach = envelope.sagitta_m + tol;
  if (scene.has_grid()) {
    std::vector<Point2d> grid_hull;
    grid_hull.reserve(envelope.hull.size());
    for (const Point2d& p : envelope.hull) {
      grid_hull.push_back(ref_to_grid_frame(frame, p));
    }
    for (const Point2d& p : grid_hull) {
      if (frame.distance_to_boundary(p) <= reach) {
        return false;
      }
    }
    const GridView& grid = scene.grid;
    const double res = frame.resolution_m;
    const Aabb2d grid_box = aabb_of(grid_hull);
    int col0 = static_cast<int>(std::floor((grid_box.min_x - reach) / res));
    int col1 = static_cast<int>(std::ceil((grid_box.max_x + reach) / res));
    int row0 = static_cast<int>(std::floor((grid_box.min_y - reach) / res));
    int row1 = static_cast<int>(std::ceil((grid_box.max_y + reach) / res));
    col0 = std::max(col0, 0);
    row0 = std::max(row0, 0);
    col1 = std::min(col1, grid.width);
    row1 = std::min(row1, grid.height);
    for (int row = row0; row < row1; ++row) {
      for (int col = col0; col < col1; ++col) {
        const int value =
            grid.cells[static_cast<std::size_t>(row) * grid.width + col];
        if (value == kCellFree) {
          continue;
        }
        const std::vector<Point2d> cell{
            Point2d{col * res, row * res}, Point2d{(col + 1) * res, row * res},
            Point2d{(col + 1) * res, (row + 1) * res},
            Point2d{col * res, (row + 1) * res}};
        // 凸包到单元的精确距离；判定前用真实线段距离细化粗过滤。
        if (ref_polygon_distance(grid_hull, cell) <= reach) {
          return false;
        }
      }
    }
  }
  for (const Obstacle& obstacle : scene.obstacles) {
    if (ref_polygon_distance(envelope.hull, obstacle.vertices) <= reach) {
      return false;
    }
  }
  return true;
}

struct RefSweepContext {
  const Footprint* footprint{nullptr};
  const Scene* scene{nullptr};
  const GridFrame* frame{nullptr};
  Pose2d start;
  double kappa{0.0};
  double precision_m{1e-3};
};

double ref_interval_displacement(const RefSweepContext& ctx, const Pose2d& p0,
                                 const Pose2d& p1, double delta_yaw) {
  const std::array<Point2d, 4> c0 = ctx.footprint->world_corners(p0);
  const std::array<Point2d, 4> c1 = ctx.footprint->world_corners(p1);
  double max_chord = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    max_chord = std::max(max_chord, std::hypot(c0[i].x_m - c1[i].x_m,
                                               c0[i].y_m - c1[i].y_m));
  }
  const double abs_yaw = std::abs(delta_yaw);
  if (abs_yaw >= kPi) {
    const double radius = (ctx.kappa != 0.0 ? std::abs(1.0 / ctx.kappa) : 0.0) +
                          ctx.footprint->corner_radius_m;
    return abs_yaw * radius;
  }
  if (abs_yaw == 0.0) {
    return max_chord;
  }
  const double half = 0.5 * abs_yaw;
  return max_chord * (half / std::sin(half));
}

struct RefDecision {
  enum class Kind { kClear, kCollision, kInconclusive };
  Kind kind{Kind::kClear};
  double s{0.0};
  Pose2d pose;
  std::string detail;
};

RefDecision ref_resolve(const RefSweepContext& ctx, double s0, const Pose2d& p0,
                        double s1) {
  const Pose2d p1 = ref_pose_on_arc(ctx.start, s1, ctx.kappa);
  const RefPoseHit hit1 = ref_pose_hit(*ctx.footprint, *ctx.scene, *ctx.frame, p1);
  const double delta_yaw = ctx.kappa * (s1 - s0);
  const double displacement = ref_interval_displacement(ctx, p0, p1, delta_yaw);
  const bool at_floor = displacement <= ctx.precision_m;

  if (hit1.hit) {
    if (at_floor) {
      RefDecision decision;
      decision.kind = RefDecision::Kind::kCollision;
      decision.s = s1;
      decision.pose = p1;
      decision.detail = hit1.detail;
      return decision;
    }
    const double sm = 0.5 * (s0 + s1);
    const Pose2d pm = ref_pose_on_arc(ctx.start, sm, ctx.kappa);
    if (ref_pose_hit(*ctx.footprint, *ctx.scene, *ctx.frame, pm).hit) {
      return ref_resolve(ctx, s0, p0, sm);
    }
    RefDecision left = ref_resolve(ctx, s0, p0, sm);
    if (left.kind != RefDecision::Kind::kClear) {
      return left;
    }
    return ref_resolve(ctx, sm, pm, s1);
  }

  const RefEnvelope envelope =
      ref_build_envelope(*ctx.footprint, p0, p1, delta_yaw, ctx.kappa);
  if (ref_envelope_clear(*ctx.footprint, *ctx.scene, *ctx.frame, envelope)) {
    return RefDecision{};
  }
  if (at_floor) {
    RefDecision decision;
    decision.kind = RefDecision::Kind::kInconclusive;
    decision.s = s0;
    decision.pose = p0;
    decision.detail =
        "conservative interval subdivision cannot decide at the 1e-3 m floor";
    return decision;
  }
  const double sm = 0.5 * (s0 + s1);
  const Pose2d pm = ref_pose_on_arc(ctx.start, sm, ctx.kappa);
  if (ref_pose_hit(*ctx.footprint, *ctx.scene, *ctx.frame, pm).hit) {
    return ref_resolve(ctx, s0, p0, sm);
  }
  RefDecision left = ref_resolve(ctx, s0, p0, sm);
  if (left.kind != RefDecision::Kind::kClear) {
    return left;
  }
  return ref_resolve(ctx, sm, pm, s1);
}

ReferenceResult invalid_reference(const std::string& detail) {
  ReferenceResult result;
  result.verdict = ReferenceVerdict::kInvalidInput;
  result.detail = detail;
  return result;
}

bool ref_scene_usable(const Scene& scene, std::string* reason) {
  if (!scene.valid(reason)) {
    return false;
  }
  return true;
}

}  // namespace

ReferenceResult reference_check_pose(const Pose2d& pose,
                                     const VehicleModel& vehicle,
                                     const Scene& scene) {
  if (!finite_pose(pose)) {
    return invalid_reference("pose is not finite");
  }
  std::string reason;
  if (!vehicle_footprint_valid(vehicle, &reason)) {
    return invalid_reference(reason);
  }
  if (!ref_scene_usable(scene, &reason)) {
    return invalid_reference(reason);
  }
  const Footprint footprint = make_footprint(vehicle);
  const GridFrame frame =
      scene.has_grid() ? GridFrame::from_grid(scene.grid) : GridFrame{};
  const RefPoseHit hit = ref_pose_hit(footprint, scene, frame, pose);
  ReferenceResult result;
  if (!hit.hit) {
    result.verdict = ReferenceVerdict::kClear;
    return result;
  }
  result.verdict = ReferenceVerdict::kCollision;
  result.first_pose = pose;
  result.first_s_m = 0.0;
  result.detail = hit.detail;
  return result;
}

ReferenceResult reference_check_motion(const MotionSegment& segment,
                                       const VehicleModel& vehicle,
                                       const Scene& scene) {
  if (!finite_pose(segment.start)) {
    return invalid_reference("motion start pose is not finite");
  }
  if (!std::isfinite(segment.length_m) || segment.length_m < 0.0) {
    return invalid_reference("motion length must be finite and non-negative");
  }
  if (!std::isfinite(segment.curvature_inv_m) ||
      std::abs(segment.curvature_inv_m) > kRefMaxAbsCurvature_inv_m) {
    return invalid_reference("motion curvature is not finite or out of range");
  }
  std::string reason;
  if (!vehicle_footprint_valid(vehicle, &reason)) {
    return invalid_reference(reason);
  }
  if (!ref_scene_usable(scene, &reason)) {
    return invalid_reference(reason);
  }
  const Footprint footprint = make_footprint(vehicle);
  const GridFrame frame =
      scene.has_grid() ? GridFrame::from_grid(scene.grid) : GridFrame{};
  const RefPoseHit start_hit = ref_pose_hit(footprint, scene, frame, segment.start);
  if (start_hit.hit) {
    ReferenceResult result;
    result.verdict = ReferenceVerdict::kCollision;
    result.first_pose = segment.start;
    result.first_s_m = 0.0;
    result.detail = "colliding start: " + start_hit.detail;
    return result;
  }
  if (segment.length_m == 0.0) {
    ReferenceResult result;
    result.verdict = ReferenceVerdict::kClear;
    return result;
  }
  RefSweepContext ctx;
  ctx.footprint = &footprint;
  ctx.scene = &scene;
  ctx.frame = &frame;
  ctx.start = segment.start;
  ctx.kappa = segment.curvature_inv_m;
  ctx.precision_m = 1e-3;
  const double sign = segment.direction == planner::common::Direction::kForward
                          ? 1.0
                          : -1.0;
  const RefDecision decision =
      ref_resolve(ctx, 0.0, segment.start, sign * segment.length_m);
  ReferenceResult result;
  switch (decision.kind) {
    case RefDecision::Kind::kClear:
      result.verdict = ReferenceVerdict::kClear;
      break;
    case RefDecision::Kind::kCollision:
      result.verdict = ReferenceVerdict::kCollision;
      result.first_pose = decision.pose;
      result.first_s_m = std::abs(decision.s);
      result.detail = decision.detail;
      break;
    case RefDecision::Kind::kInconclusive:
      result.verdict = ReferenceVerdict::kInconclusive;
      result.first_pose = decision.pose;
      result.first_s_m = std::abs(decision.s);
      result.detail = decision.detail;
      break;
  }
  return result;
}

ReferenceResult reference_check_path(const std::vector<PathSegment>& path,
                                     const VehicleModel& vehicle,
                                     const Scene& scene) {
  if (path.empty()) {
    return invalid_reference("path is empty");
  }
  std::string reason;
  if (!vehicle_footprint_valid(vehicle, &reason)) {
    return invalid_reference(reason);
  }
  if (!ref_scene_usable(scene, &reason)) {
    return invalid_reference(reason);
  }
  for (std::size_t seg = 0; seg < path.size(); ++seg) {
    const PathSegment& segment = path[seg];
    if (segment.points.empty()) {
      return invalid_reference("path segment " + std::to_string(seg) +
                               " has no points");
    }
    ReferenceResult first =
        reference_check_pose(segment.points.front().pose, vehicle, scene);
    if (first.verdict != ReferenceVerdict::kClear) {
      return first;
    }
    for (std::size_t i = 0; i + 1 < segment.points.size(); ++i) {
      MotionSegment motion;
      motion.start = segment.points[i].pose;
      motion.length_m = segment.points[i + 1].s_m - segment.points[i].s_m;
      motion.direction = segment.points[i].direction;
      motion.curvature_inv_m = segment.points[i].curvature_inv_m;
      if (!(motion.length_m > 0.0)) {
        continue;
      }
      ReferenceResult step = reference_check_motion(motion, vehicle, scene);
      if (step.verdict != ReferenceVerdict::kClear) {
        return step;
      }
    }
  }
  ReferenceResult result;
  result.verdict = ReferenceVerdict::kClear;
  return result;
}

}  // namespace planner::collision
