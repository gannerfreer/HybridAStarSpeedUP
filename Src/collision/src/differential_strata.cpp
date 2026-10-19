// 差分生成器的集中对抗构造：角点擦碰、精确接触、薄障碍物、倒车运动与
// 地图边缘。每个构造在接受前都用生成器自身的测量辅助函数对照其设计窗口
// 进行验证；失败尝试回退到保证成功的构造。

#include "differential_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "planner/common/types.hpp"

namespace planner::collision::differential::internal {

namespace {

using planner::common::kCellFree;
using planner::common::kPi;
using planner::common::normalize_angle_rad;

constexpr double kResolution = 0.1;
constexpr double kFeatureTolerance_m = 5e-7;
constexpr int kInternalTries = 25;

std::vector<Point2d> footprint_polygon(const Footprint& footprint,
                                       const Pose2d& pose) {
  const std::array<Point2d, 4> corners = footprint.world_corners(pose);
  return {corners[0], corners[1], corners[2], corners[3]};
}

bool verify_pose_gap(const Footprint& footprint, const Pose2d& pose,
                     const Obstacle& feature, double gap) {
  const double measured =
      convex_polygon_distance(footprint_polygon(footprint, pose),
                              feature.vertices);
  if (gap >= 0.0) {
    return std::abs(measured - gap) <= 1e-9;
  }
  return measured == 0.0;
}

bool verify_sweep_gap(const Footprint& footprint, const MotionSegment& motion,
                      const Obstacle& feature, double gap) {
  const std::vector<Obstacle> obstacles{feature};
  const double measured = sweep_min_clearance(footprint, motion, obstacles);
  if (gap >= 0.0) {
    return measured >= gap - kFeatureTolerance_m &&
           measured <= gap + kFeatureTolerance_m;
  }
  return measured == 0.0;
}

Obstacle place_at_edge(const Pose2d& pose, const Footprint& footprint,
                       int edge, double gap, double t0, double t1, double depth,
                       const std::string& id) {
  const std::array<Point2d, 4> body = footprint.body_corners();
  const Point2d a = transform_point(pose, body[static_cast<std::size_t>(edge)]);
  const Point2d b =
      transform_point(pose, body[static_cast<std::size_t>((edge + 1) % 4)]);
  const Point2d direction = b - a;
  const double length = norm(direction);
  const Point2d u{direction.x_m / length, direction.y_m / length};
  const Point2d n{u.y_m, -u.x_m};  // 逆时针角点顺序下指向外侧
  const Point2d near_a = a + u * t0 + n * gap;
  const Point2d near_b = a + u * t1 + n * gap;
  std::vector<Point2d> vertices{near_a, near_a + n * depth,
                                near_b + n * depth, near_b};
  return Obstacle::convex_polygon(id, vertices);
}

Obstacle place_at_corner(const Pose2d& pose, const Footprint& footprint,
                         int corner, double gap, double depth, double width,
                         double tangent_sign, const std::string& id) {
  const std::array<Point2d, 4> body = footprint.body_corners();
  const std::size_t previous = static_cast<std::size_t>((corner + 3) % 4);
  const std::size_t current = static_cast<std::size_t>(corner);
  const auto outward_normal = [&](std::size_t edge) {
    const Point2d a = transform_point(pose, body[edge]);
    const Point2d b = transform_point(pose, body[(edge + 1) % 4]);
    const Point2d d = b - a;
    const double len = norm(d);
    const Point2d u{d.x_m / len, d.y_m / len};
    return Point2d{u.y_m, -u.x_m};
  };
  const Point2d n_prev = outward_normal(previous);
  const Point2d n_next = outward_normal(current);
  Point2d n{n_prev.x_m + n_next.x_m, n_prev.y_m + n_next.y_m};
  const double n_len = norm(n);
  n = Point2d{n.x_m / n_len, n.y_m / n_len};
  const Point2d t{n.y_m * tangent_sign, -n.x_m * tangent_sign};
  const Point2d c = transform_point(pose, body[current]);
  const Point2d q = c + n * gap;
  std::vector<Point2d> vertices{q, q + n * depth, q + n * depth + t * width,
                                q + t * width};
  return Obstacle::convex_polygon(id, vertices);
}

// 沿扫掠采样的足迹角点 AABB（散置障碍物/未知块的保护区域）。
Aabb2d sweep_protection(const Footprint& footprint, const MotionSegment& motion,
                        double margin) {
  const double sign =
      motion.direction == Direction::kForward ? 1.0 : -1.0;
  std::vector<Point2d> points;
  const int samples = 17;
  for (int i = 0; i <= samples; ++i) {
    const double s = motion.length_m * static_cast<double>(i) / samples;
    const Pose2d pose =
        pose_on_arc(motion.start, sign * s, motion.curvature_inv_m);
    const std::array<Point2d, 4> corners = footprint.world_corners(pose);
    for (const Point2d& p : corners) {
      points.push_back(p);
    }
  }
  return protection_box(points, margin);
}

// 对扫掠足迹与地图矩形进行采样检查。
void sample_map_margins(const Footprint& footprint, const MotionSegment& motion,
                        double width_m, double height_m, bool* all_inside,
                        bool* any_outside, double* min_margin) {
  const double sign =
      motion.direction == Direction::kForward ? 1.0 : -1.0;
  *all_inside = true;
  *any_outside = false;
  *min_margin = std::numeric_limits<double>::infinity();
  const int samples = 33;
  for (int i = 0; i <= samples; ++i) {
    const double s = motion.length_m * static_cast<double>(i) / samples;
    const Pose2d pose =
        pose_on_arc(motion.start, sign * s, motion.curvature_inv_m);
    const std::array<Point2d, 4> corners = footprint.world_corners(pose);
    for (const Point2d& p : corners) {
      const double margin =
          std::min(std::min(p.x_m, width_m - p.x_m),
                   std::min(p.y_m, height_m - p.y_m));
      if (margin < 0.0) {
        *any_outside = true;
        *all_inside = false;
      }
      *min_margin = std::min(*min_margin, margin);
    }
  }
}

// 构建栅格，在保护区域之外放置散置障碍物与未知块（集中用例中栅格坐标
// 系 == 世界坐标系）。
void build_concentrated_grid(DifferentialCase& test_case, Rng& rng, int width_m,
                             int height_m, const Aabb2d* protection,
                             bool has_protection, bool add_boundary_unknown) {
  const int width = width_m * 10;
  const int height = height_m * 10;
  const double world_width = width * kResolution;
  const double world_height = height * kResolution;
  std::vector<int> cells(static_cast<std::size_t>(width) * height, kCellFree);
  const Aabb2d map_box{0.6, 0.6, world_width - 0.6, world_height - 0.6};
  const int obstacle_count = rng.uniform_int(0, 16);
  for (int i = 0; i < obstacle_count; ++i) {
    for (int attempt = 0; attempt < 10; ++attempt) {
      Obstacle obstacle =
          random_obstacle(rng, map_box, "obs_" + std::to_string(i));
      if (!obstacle.valid()) {
        continue;
      }
      if (has_protection && protection->intersects(obstacle.aabb())) {
        continue;
      }
      rasterize_obstacle(cells, width, height, kResolution, obstacle.vertices,
                         protection, has_protection);
      break;
    }
  }
  const double unknown_target = rng.uniform(0.0, 0.2) * width * height;
  double unknown_cells = 0.0;
  const double max_side = std::min(world_width, world_height);
  for (int attempt = 0; attempt < 6 && unknown_cells < unknown_target;
       ++attempt) {
    const double side = max_side * rng.uniform(0.05, 0.3);
    Aabb2d block;
    block.min_x = rng.uniform(0.0, std::max(0.0, world_width - side));
    block.min_y = rng.uniform(0.0, std::max(0.0, world_height - side));
    block.max_x = std::min(world_width, block.min_x + side);
    block.max_y = std::min(world_height, block.min_y + side);
    if (has_protection && protection->intersects(block)) {
      continue;
    }
    unknown_cells += static_cast<double>(rasterize_unknown_block(
        cells, width, height, kResolution, block, protection, has_protection));
  }
  if (add_boundary_unknown && rng.coin(0.6)) {
    // 接触随机地图边界的未知斑块。
    const double side = max_side * rng.uniform(0.05, 0.2);
    Aabb2d block;
    switch (rng.uniform_int(0, 3)) {
      case 0:
        block = Aabb2d{0.0, rng.uniform(0.0, world_height - side), side,
                       rng.uniform(0.0, world_height - side) + side};
        break;
      case 1:
        block = Aabb2d{world_width - side,
                       rng.uniform(0.0, world_height - side), world_width,
                       rng.uniform(0.0, world_height - side) + side};
        break;
      case 2:
        block = Aabb2d{rng.uniform(0.0, world_width - side), 0.0,
                       rng.uniform(0.0, world_width - side) + side, side};
        break;
      default:
        block = Aabb2d{rng.uniform(0.0, world_width - side),
                       world_height - side,
                       rng.uniform(0.0, world_width - side) + side,
                       world_height};
        break;
    }
    if (!(has_protection && protection->intersects(block))) {
      rasterize_unknown_block(cells, width, height, kResolution, block,
                              protection, has_protection);
    }
  }
  Pose2d origin;
  origin.x_m = 0.0;
  origin.y_m = 0.0;
  origin.yaw_rad = 0.0;
  test_case.scene.grid = make_grid(width, height, kResolution, origin,
                                   std::move(cells), test_case.case_id);
}

Pose2d random_interior_pose(Rng& rng, double width_m, double height_m,
                            double margin) {
  Pose2d pose;
  pose.x_m = rng.uniform(margin, width_m - margin);
  pose.y_m = rng.uniform(margin, height_m - margin);
  pose.yaw_rad = rng.uniform(-kPi, kPi);
  return pose;
}

void set_motion(DifferentialCase& out, Direction direction, double length,
                double curvature) {
  out.direction = direction;
  out.length_m = length;
  out.curvature_inv_m = curvature;
}

// ---------------------------------------------------------------------------
// corner_graze / exact_contact / thin_obstacle
// ---------------------------------------------------------------------------

void generate_feature_pose(DifferentialCase& out, Rng& rng) {
  const Footprint footprint = make_footprint(out.vehicle);
  const bool thin = out.stratum == Stratum::kThinObstacle;
  for (int attempt = 0; attempt < kInternalTries; ++attempt) {
    const int width_m = rng.uniform_int(20, 30);
    const int height_m = rng.uniform_int(20, 30);
    const Pose2d pose =
        random_interior_pose(rng, width_m, height_m, 7.0);
    const double gap = out.stratum == Stratum::kExactContact
                           ? rng.uniform(-1e-9, 1e-9)
                           : rng.uniform(-0.05, 0.05);
    Obstacle feature;
    bool constructed = false;
    if (thin) {
      const int edge = rng.uniform_int(0, 3);
      const double edge_length =
          (edge % 2 == 0) ? (footprint.x_max_m - footprint.x_min_m)
                          : (footprint.y_max_m - footprint.y_min_m);
      const double thickness = rng.uniform(0.02, 0.05);
      const double span =
          rng.uniform(0.4, std::max(0.5, 0.45 * edge_length));
      const double t0 = rng.uniform(0.05, std::max(0.05, 0.3 * edge_length));
      const double t1 = std::min(t0 + span, edge_length - 0.05);
      if (t1 - t0 < 0.3) {
        continue;
      }
      feature = place_at_edge(pose, footprint, edge, gap, t0, t1, thickness,
                              "feature_thin");
      constructed = verify_pose_gap(footprint, pose, feature, gap);
    } else if (rng.coin(0.5)) {
      const int corner = rng.uniform_int(0, 3);
      const double depth = rng.uniform(0.4, 2.0);
      const double width = rng.uniform(0.3, 1.5);
      const double tangent_sign = rng.coin() ? 1.0 : -1.0;
      feature = place_at_corner(pose, footprint, corner, gap, depth, width,
                                tangent_sign, "feature_corner");
      constructed = verify_pose_gap(footprint, pose, feature, gap);
    } else {
      const int edge = rng.uniform_int(0, 3);
      const double edge_length =
          (edge % 2 == 0) ? (footprint.x_max_m - footprint.x_min_m)
                          : (footprint.y_max_m - footprint.y_min_m);
      const double depth = rng.uniform(0.4, 2.0);
      const double span =
          rng.uniform(0.4, std::max(0.5, 0.5 * edge_length));
      const double t0 = rng.uniform(0.05, std::max(0.05, 0.3 * edge_length));
      const double t1 = std::min(t0 + span, edge_length - 0.05);
      if (t1 - t0 < 0.3) {
        continue;
      }
      feature = place_at_edge(pose, footprint, edge, gap, t0, t1, depth,
                              "feature_edge");
      constructed = verify_pose_gap(footprint, pose, feature, gap);
    }
    if (!constructed) {
      continue;
    }
    out.start = pose;
    out.scene.obstacles.push_back(std::move(feature));
    out.design_gap_m = gap;
    out.measured_clearance_m = std::abs(convex_polygon_distance(
        footprint_polygon(footprint, pose), out.scene.obstacles.back().vertices));
    const std::vector<Point2d> corridor =
        footprint_polygon(footprint, pose);
    const Aabb2d protection = protection_box(corridor, 0.5);
    build_concentrated_grid(out, rng, width_m, height_m, &protection, true,
                            false);
    return;
  }
  // 保证成功的回退：沿直边放置，远离地图边界。
  const int width_m = 30;
  const int height_m = 20;
  Pose2d pose;
  pose.x_m = width_m * 0.5;
  pose.y_m = height_m * 0.5;
  pose.yaw_rad = 0.0;
  const double gap = out.stratum == Stratum::kExactContact ? 0.0 : 0.02;
  const double edge_length = footprint.y_max_m - footprint.y_min_m;
  Obstacle feature = place_at_edge(pose, footprint, 1, gap, 0.25 * edge_length,
                                   0.75 * edge_length,
                                   thin ? 0.03 : 0.8, "feature_fallback");
  out.start = pose;
  out.scene.obstacles.push_back(std::move(feature));
  out.design_gap_m = gap;
  out.measured_clearance_m =
      std::abs(convex_polygon_distance(footprint_polygon(footprint, pose),
                                       out.scene.obstacles.back().vertices));
  const Aabb2d protection =
      protection_box(footprint_polygon(footprint, pose), 0.5);
  build_concentrated_grid(out, rng, width_m, height_m, &protection, true,
                          false);
}

void generate_feature_motion(DifferentialCase& out, Rng& rng) {
  const Footprint footprint = make_footprint(out.vehicle);
  const bool thin = out.stratum == Stratum::kThinObstacle;
  const bool exact = out.stratum == Stratum::kExactContact;
  for (int attempt = 0; attempt < kInternalTries; ++attempt) {
    const int width_m = rng.uniform_int(22, 34);
    const int height_m = rng.uniform_int(22, 34);
    const Pose2d start = random_interior_pose(rng, width_m, height_m, 8.0);
    const Direction direction =
        rng.coin() ? Direction::kForward : Direction::kReverse;
    const double length = rng.uniform(0.5, 5.0);
    const bool interior = rng.coin(0.5);
    const double curvature =
        interior ? 0.0 : (rng.coin(0.5) ? 0.0 : rng.uniform(-0.03, 0.03));
    const double gap =
        exact ? rng.uniform(-1e-9, 0.0) : rng.uniform(-0.05, 0.05);
    double design_gap = gap;
    const double sign = direction == Direction::kForward ? 1.0 : -1.0;
    MotionSegment motion;
    motion.start = start;
    motion.length_m = length;
    motion.direction = direction;
    motion.curvature_inv_m = curvature;
    Obstacle feature;
    bool constructed = false;
    if (thin && interior) {
      // 在首次二分中点处横穿路径的薄墙：该点处的设计间隙为精确接触
      // （0）。
      design_gap = 0.0;
      const Pose2d mid =
          pose_on_arc(start, sign * 0.5 * length, curvature);
      const double edge_length = footprint.y_max_m - footprint.y_min_m;
      const double thickness = rng.uniform(0.02, 0.05);
      feature = place_at_edge(mid, footprint, 1, 0.0, -0.6,
                              edge_length + 0.6, thickness, "feature_thin_cross");
      constructed = verify_sweep_gap(footprint, motion, feature, 0.0);
    } else if (interior) {
      // 直线扫掠中点处的侧面擦碰：间隙恒定。
      const Pose2d mid =
          pose_on_arc(start, sign * 0.5 * length, curvature);
      const int edge = rng.coin() ? 0 : 2;
      const double edge_length =
          (edge % 2 == 0) ? (footprint.x_max_m - footprint.x_min_m)
                          : (footprint.y_max_m - footprint.y_min_m);
      const double span = rng.uniform(0.4, std::max(0.5, edge_length - 0.3));
      const double t0 = rng.uniform(0.1, std::max(0.1, edge_length - span - 0.1));
      const double depth = rng.uniform(0.4, 1.5);
      feature = place_at_edge(mid, footprint, edge, gap, t0, t0 + span, depth,
                              "feature_side");
      constructed = verify_sweep_gap(footprint, motion, feature, gap);
    } else {
      // 接近构造：终点处前缘前方的障碍物。
      const Pose2d end = pose_on_arc(start, sign * length, curvature);
      const int leading_edge =
          direction == Direction::kForward ? 1 : 3;
      const double edge_length = footprint.y_max_m - footprint.y_min_m;
      const double depth = thin ? rng.uniform(0.02, 0.05) : rng.uniform(0.4, 2.0);
      feature = place_at_edge(end, footprint, leading_edge, gap,
                              0.2 * edge_length, 0.8 * edge_length, depth,
                              "feature_ahead");
      constructed = verify_sweep_gap(footprint, motion, feature, gap);
    }
    if (!constructed) {
      continue;
    }
    out.start = start;
    set_motion(out, direction, length, curvature);
    out.scene.obstacles.push_back(std::move(feature));
    out.design_gap_m = design_gap;
    out.measured_clearance_m =
        sweep_min_clearance(footprint, motion, out.scene.obstacles);
    const Aabb2d protection = sweep_protection(footprint, motion, 0.5);
    build_concentrated_grid(out, rng, width_m, height_m, &protection, true,
                            false);
    return;
  }
  // 保证成功的回退：沿 +x 方向以 2 m/s 直线接近。
  const int width_m = 34;
  const int height_m = 22;
  Pose2d start;
  start.x_m = 10.0;
  start.y_m = height_m * 0.5;
  start.yaw_rad = 0.0;
  MotionSegment motion;
  motion.start = start;
  motion.length_m = 2.0;
  motion.direction = Direction::kForward;
  motion.curvature_inv_m = 0.0;
  const double gap = exact ? 0.0 : 0.02;
  const double edge_length = footprint.y_max_m - footprint.y_min_m;
  const Pose2d end = pose_on_arc(start, motion.length_m, 0.0);
  Obstacle feature = place_at_edge(end, footprint, 1, gap, 0.2 * edge_length,
                                   0.8 * edge_length, thin ? 0.03 : 0.8,
                                   "feature_fallback");
  out.start = start;
  set_motion(out, Direction::kForward, motion.length_m, 0.0);
  out.scene.obstacles.push_back(std::move(feature));
  out.design_gap_m = gap;
  out.measured_clearance_m =
      sweep_min_clearance(footprint, motion, out.scene.obstacles);
  const Aabb2d protection = sweep_protection(footprint, motion, 0.5);
  build_concentrated_grid(out, rng, width_m, height_m, &protection, true,
                          false);
}

// ---------------------------------------------------------------------------
// reverse_motion
// ---------------------------------------------------------------------------

void generate_reverse_motion(DifferentialCase& out, Rng& rng) {
  const Footprint footprint = make_footprint(out.vehicle);
  for (int attempt = 0; attempt < kInternalTries; ++attempt) {
    const int width_m = rng.uniform_int(20, 36);
    const int height_m = rng.uniform_int(20, 36);
    const Pose2d start = random_interior_pose(rng, width_m, height_m, 7.0);
    const double length = rng.uniform(0.5, 5.0);
    const double curvature =
        rng.coin(0.3) ? 0.0 : rng.uniform(-0.05, 0.05);
    MotionSegment motion;
    motion.start = start;
    motion.length_m = length;
    motion.direction = Direction::kReverse;
    motion.curvature_inv_m = curvature;
    // 一半样本在车辆后方放置特征物；其余依赖散置场景。
    Obstacle feature;
    bool has_feature = rng.coin(0.5);
    double gap = 0.0;
    if (has_feature) {
      const Pose2d end = pose_on_arc(start, -length, curvature);
      gap = rng.uniform(-0.05, 1.5);
      if (gap > 0.0 && gap < 1e-6) {
        gap = 1e-6;  // 避开低于精度的区间
      }
      const double edge_length = footprint.y_max_m - footprint.y_min_m;
      feature = place_at_edge(end, footprint, 3, gap, 0.2 * edge_length,
                              0.8 * edge_length, rng.uniform(0.4, 1.5),
                              "feature_behind");
      if (!verify_sweep_gap(footprint, motion, feature, gap)) {
        continue;
      }
    }
    out.start = start;
    set_motion(out, Direction::kReverse, length, curvature);
    if (has_feature) {
      out.scene.obstacles.push_back(std::move(feature));
      out.measured_clearance_m =
          sweep_min_clearance(footprint, motion, out.scene.obstacles);
      out.design_gap_m = gap;
    } else {
      // 无特征物：实测间隙按构造未定义。
      out.measured_clearance_m =
          std::numeric_limits<double>::quiet_NaN();
      out.design_gap_m = std::numeric_limits<double>::quiet_NaN();
    }
    const Aabb2d protection = sweep_protection(footprint, motion, 0.5);
    // 散置障碍物被栅格化（多边形列表仅保留特征物）。
    build_concentrated_grid(out, rng, width_m, height_m, &protection, true,
                            false);
    return;
  }
  // 保证成功的回退：在空旷区域中做直线倒车运动。
  const int width_m = 30;
  const int height_m = 20;
  Pose2d start;
  start.x_m = 15.0;
  start.y_m = 10.0;
  start.yaw_rad = 0.0;
  set_motion(out, Direction::kReverse, 2.0, 0.0);
  out.start = start;
  out.design_gap_m = std::numeric_limits<double>::quiet_NaN();
  out.measured_clearance_m = std::numeric_limits<double>::quiet_NaN();
  MotionSegment motion{start, 2.0, Direction::kReverse, 0.0};
  const Aabb2d protection = sweep_protection(footprint, motion, 0.5);
  build_concentrated_grid(out, rng, width_m, height_m, &protection, true,
                          false);
}

// ---------------------------------------------------------------------------
// map_edge
// ---------------------------------------------------------------------------

struct AxisAlignedFootprint {
  Aabb2d relative;  // 相对后轴的足迹 AABB
};

AxisAlignedFootprint axis_aligned_footprint(const Footprint& footprint,
                                            double yaw_quarter) {
  const Pose2d pose{0.0, 0.0, yaw_quarter * 0.5 * kPi};
  return AxisAlignedFootprint{aabb_of(std::vector<Point2d>{
      footprint.world_corners(pose)[0], footprint.world_corners(pose)[1],
      footprint.world_corners(pose)[2], footprint.world_corners(pose)[3]})};
}

// 沿一条地图边界的带符号放置：地图过小时返回 false。`delta` 是足迹到
// 边界的距离，在内为正、相切为零、在外为负；足迹保持与边界平行（轴对齐
// 偏航角）。
bool place_near_border(const AxisAlignedFootprint& shape, int side,
                       double delta, double along, double width_m,
                       double height_m, Pose2d* pose) {
  const Aabb2d& rel = shape.relative;
  Pose2d result;
  switch (side) {
    case 0:  // 左边界，delta 相对 x=0
      result.x_m = delta - rel.min_x;
      result.y_m = along;
      break;
    case 1:  // 右边界，delta 相对 x=width
      result.x_m = width_m - delta - rel.max_x;
      result.y_m = along;
      break;
    case 2:  // 下边界
      result.x_m = along;
      result.y_m = delta - rel.min_y;
      break;
    default:  // 上边界
      result.x_m = along;
      result.y_m = height_m - delta - rel.max_y;
      break;
  }
  // 使足迹远离垂直方向的边界；位于内部的放置还必须保持在地图内。
  const double x0 = rel.min_x + result.x_m;
  const double x1 = rel.max_x + result.x_m;
  const double y0 = rel.min_y + result.y_m;
  const double y1 = rel.max_y + result.y_m;
  if (side == 0 || side == 1) {
    if (y0 < 0.05 || y1 > height_m - 0.05) {
      return false;
    }
    if (delta >= 0.0 && (x0 < 0.05 || x1 > width_m - 0.05)) {
      return false;
    }
  } else {
    if (x0 < 0.05 || x1 > width_m - 0.05) {
      return false;
    }
    if (delta >= 0.0 && (y0 < 0.05 || y1 > height_m - 0.05)) {
      return false;
    }
  }
  *pose = result;
  return true;
}

void generate_map_edge(DifferentialCase& out, Rng& rng) {
  const Footprint footprint = make_footprint(out.vehicle);
  for (int attempt = 0; attempt < kInternalTries; ++attempt) {
    const int width_m = rng.uniform_int(14, 60);
    const int height_m = rng.uniform_int(14, 60);
    const int yaw_quarter = rng.uniform_int(0, 3);
    const double yaw = yaw_quarter * 0.5 * kPi;
    const AxisAlignedFootprint shape = axis_aligned_footprint(footprint,
                                                              yaw_quarter);
    const Aabb2d& rel = shape.relative;
    const int side = rng.uniform_int(0, 3);
    const double half_x = 0.5 * (rel.max_x - rel.min_x);
    const double half_y = 0.5 * (rel.max_y - rel.min_y);
    if (out.check_kind != "motion") {
      int mode = rng.uniform_int(0, 99);
      double delta = 0.0;
      bool outside_fully = false;
      if (mode < 35) {
        delta = rng.uniform(0.0, 0.05);  // 内侧带，含相切
      } else if (mode < 60) {
        delta = rng.uniform(-0.5, -1e-3);  // 穿越
      } else {
        outside_fully = true;  // 完全在外部
      }
      const double along_min =
          (side < 2 ? half_y + 0.3 : half_x + 0.3);
      const double along_max =
          (side < 2 ? height_m - half_y - 0.3 : width_m - half_x - 0.3);
      if (along_min >= along_max) {
        continue;
      }
      const double along = rng.uniform(along_min, along_max);
      Pose2d pose;
      pose.yaw_rad = yaw;
      if (outside_fully) {
        const double separation = rng.uniform(0.2, 3.0);
        // 将足迹近侧边界置于外部 `separation` 处。
        double signed_delta = 0.0;
        switch (side) {
          case 0:
            signed_delta = -separation - (rel.max_x - rel.min_x);
            break;
          case 1:
            signed_delta = -separation - (rel.max_x - rel.min_x);
            break;
          case 2:
            signed_delta = -separation - (rel.max_y - rel.min_y);
            break;
          default:
            signed_delta = -separation - (rel.max_y - rel.min_y);
            break;
        }
        if (!place_near_border(shape, side, signed_delta, along, width_m,
                               height_m, &pose)) {
          continue;
        }
      } else {
        if (!place_near_border(shape, side, delta, along, width_m, height_m,
                               &pose)) {
          continue;
        }
      }
      out.start = pose;
      out.design_gap_m = std::numeric_limits<double>::quiet_NaN();
      out.measured_clearance_m = std::numeric_limits<double>::quiet_NaN();
      const std::vector<Point2d> corners = footprint_polygon(footprint, pose);
      const Aabb2d protection = protection_box(corners, 0.5);
      build_concentrated_grid(out, rng, width_m, height_m, &protection, true,
                              true);
      return;
    }

    // 运动变体。
    const int variant = rng.uniform_int(0, 99);
    MotionSegment motion;
    motion.curvature_inv_m = 0.0;
    Pose2d pose;
    pose.yaw_rad = yaw;
    if (variant < 40) {
      // 在内侧带中平行于边界。
      const double delta = rng.uniform(1e-3, 0.05);
      // 沿边界行驶：偏航角旋转 90 度。
      const int travel = rng.uniform_int(0, 1);
      const double travel_yaw =
          normalize_angle_rad(yaw + (travel == 0 ? 0.5 * kPi : -0.5 * kPi));
      Pose2d travel_shape_pose{0.0, 0.0, travel_yaw};
      const std::array<Point2d, 4> tc =
          footprint.world_corners(travel_shape_pose);
      const Aabb2d travel_shape =
          aabb_of(std::vector<Point2d>{tc[0], tc[1], tc[2], tc[3]});
      const double along_limit = side < 2 ? height_m : width_m;
      const double half_along =
          side < 2 ? 0.5 * (travel_shape.max_y - travel_shape.min_y)
                   : 0.5 * (travel_shape.max_x - travel_shape.min_x);
      const double max_length =
          std::min(5.0, along_limit - 2.0 * half_along - 1.0);
      if (max_length < 0.5) {
        continue;
      }
      const double length = rng.uniform(0.5, max_length);
      const double along_min = half_along + 0.5;
      const double along_max = along_limit - half_along - 0.5 - length;
      if (along_min >= along_max) {
        continue;
      }
      const double along = rng.uniform(along_min, along_max);
      // 放置足迹，使其到边界侧的距离等于 delta。
      switch (side) {
        case 0:
          pose.x_m = delta - travel_shape.min_x;
          pose.y_m = along;
          break;
        case 1:
          pose.x_m = width_m - delta - travel_shape.max_x;
          pose.y_m = along;
          break;
        case 2:
          pose.x_m = along;
          pose.y_m = delta - travel_shape.min_y;
          break;
        default:
          pose.x_m = along;
          pose.y_m = height_m - delta - travel_shape.max_y;
          break;
      }
      pose.yaw_rad = travel_yaw;
      motion.start = pose;
      motion.length_m = length;
      motion.direction = Direction::kForward;
      motion.curvature_inv_m = 0.0;
      bool all_inside = false;
      bool any_outside = false;
      double min_margin = 0.0;
      sample_map_margins(footprint, motion, width_m, height_m, &all_inside,
                         &any_outside, &min_margin);
      if (!all_inside || min_margin < 0.5e-3) {
        continue;
      }
    } else if (variant < 75) {
      // 朝向边界并穿越它。
      const double delta_in = rng.uniform(0.8, 2.0);
      const bool reverse_gear = rng.coin();
      double outward_yaw = 0.0;
      switch (side) {
        case 0:
          outward_yaw = kPi;  // -x
          break;
        case 1:
          outward_yaw = 0.0;  // +x
          break;
        case 2:
          outward_yaw = -0.5 * kPi;  // -y
          break;
        default:
          outward_yaw = 0.5 * kPi;  // +y
          break;
      }
      const double travel_yaw =
          reverse_gear ? normalize_angle_rad(outward_yaw + kPi) : outward_yaw;
      Pose2d travel_shape_pose{0.0, 0.0, travel_yaw};
      const std::array<Point2d, 4> tc =
          footprint.world_corners(travel_shape_pose);
      const Aabb2d travel_shape =
          aabb_of(std::vector<Point2d>{tc[0], tc[1], tc[2], tc[3]});
      const double half_extent_along_border =
          side < 2 ? 0.5 * (travel_shape.max_y - travel_shape.min_y)
                   : 0.5 * (travel_shape.max_x - travel_shape.min_x);
      const double along_limit = side < 2 ? height_m : width_m;
      const double along_min = half_extent_along_border + 0.5;
      const double along_max = along_limit - half_extent_along_border - 0.5;
      if (along_min >= along_max) {
        continue;
      }
      const double along = rng.uniform(along_min, along_max);
      const double length = std::min(5.0, delta_in + rng.uniform(0.5, 1.5));
      switch (side) {
        case 0:
          pose.x_m = delta_in - travel_shape.min_x;
          pose.y_m = along;
          break;
        case 1:
          pose.x_m = width_m - delta_in - travel_shape.max_x;
          pose.y_m = along;
          break;
        case 2:
          pose.x_m = along;
          pose.y_m = delta_in - travel_shape.min_y;
          break;
        default:
          pose.x_m = along;
          pose.y_m = height_m - delta_in - travel_shape.max_y;
          break;
      }
      pose.yaw_rad = normalize_angle_rad(travel_yaw);
      motion.start = pose;
      motion.length_m = length;
      motion.direction = reverse_gear ? Direction::kReverse : Direction::kForward;
      motion.curvature_inv_m = 0.0;
      bool all_inside = false;
      bool any_outside = false;
      double min_margin = 0.0;
      sample_map_margins(footprint, motion, width_m, height_m, &all_inside,
                         &any_outside, &min_margin);
      if (!any_outside) {
        continue;
      }
    } else {
      // 起点完全在外部。
      const double separation = rng.uniform(0.3, 2.0);
      double delta = 0.0;
      switch (side) {
        case 0:
          delta = -separation - (rel.max_x - rel.min_x);
          break;
        case 1:
          delta = -separation - (rel.max_x - rel.min_x);
          break;
        case 2:
          delta = -separation - (rel.max_y - rel.min_y);
          break;
        default:
          delta = -separation - (rel.max_y - rel.min_y);
          break;
      }
      const double along_min = half_x + 0.5;
      const double along_max =
          (side < 2 ? height_m - half_y - 0.5 : width_m - half_x - 0.5);
      if (along_min >= along_max) {
        continue;
      }
      const double along = rng.uniform(along_min, along_max);
      if (!place_near_border(shape, side, delta, along, width_m, height_m,
                             &pose)) {
        continue;
      }
      pose.yaw_rad = yaw;
      motion.start = pose;
      motion.length_m = rng.uniform(0.5, 3.0);
      motion.direction =
          rng.coin() ? Direction::kForward : Direction::kReverse;
      motion.curvature_inv_m = rng.uniform(-0.05, 0.05);
    }
    out.start = motion.start;
    set_motion(out, motion.direction, motion.length_m,
               motion.curvature_inv_m);
    out.design_gap_m = std::numeric_limits<double>::quiet_NaN();
    out.measured_clearance_m = std::numeric_limits<double>::quiet_NaN();
    const Aabb2d protection = sweep_protection(footprint, motion, 0.5);
    build_concentrated_grid(out, rng, width_m, height_m, &protection, true,
                            true);
    return;
  }
  // 保证成功的回退：在左边界内侧 0.02 m 处平行运动。
  const int width_m = 30;
  const int height_m = 20;
  const AxisAlignedFootprint shape = axis_aligned_footprint(footprint, 1);
  Pose2d pose;
  pose.yaw_rad = 0.5 * kPi;  // 沿 +y 行驶
  pose.x_m = 0.02 - shape.relative.min_x;
  pose.y_m = 5.0;
  set_motion(out, Direction::kForward, 3.0, 0.0);
  out.start = pose;
  out.design_gap_m = std::numeric_limits<double>::quiet_NaN();
  out.measured_clearance_m = std::numeric_limits<double>::quiet_NaN();
  MotionSegment motion{pose, 3.0, Direction::kForward, 0.0};
  const Aabb2d protection = sweep_protection(footprint, motion, 0.5);
  build_concentrated_grid(out, rng, width_m, height_m, &protection, true,
                          true);
}

}  // namespace

void generate_concentrated(DifferentialCase& out, Rng& rng) {
  switch (out.stratum) {
    case Stratum::kCornerGraze:
    case Stratum::kExactContact:
    case Stratum::kThinObstacle:
      if (out.check_kind == "motion") {
        generate_feature_motion(out, rng);
      } else {
        generate_feature_pose(out, rng);
      }
      break;
    case Stratum::kReverseMotion:
      generate_reverse_motion(out, rng);
      break;
    case Stratum::kMapEdge:
      generate_map_edge(out, rng);
      break;
    case Stratum::kWideRandom:
      // 宽范围随机在 differential.cpp 中生成。
      break;
  }
}

}  // namespace planner::collision::differential::internal
