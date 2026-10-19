// 差分生成器：regime 规格、确定性分配、共享构造辅助函数以及宽范围随机
// 分层。

#include "differential_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "planner/common/types.hpp"

namespace planner::collision::differential {

using planner::common::kCellFree;
using planner::common::kCellOccupied;
using planner::common::kCellUnknown;
using planner::common::kPi;
using planner::common::normalize_angle_rad;

namespace {

constexpr double kMaxAbsCurvature_inv_m = 0.195108;  // tan(0.5)/2.8 四舍五入
constexpr double kPi_ = 3.14159265358979323846;

std::uint64_t mix64(std::uint64_t z) noexcept {
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}

}  // namespace

namespace internal {

std::uint64_t mixed_seed(int seed, int ordinal, int attempt) noexcept {
  std::uint64_t h = 0x9E3779B97F4A7C15ull;
  h ^= mix64(static_cast<std::uint64_t>(static_cast<std::uint32_t>(seed)) +
             0x100000001ull);
  h = mix64(h ^ static_cast<std::uint64_t>(ordinal));
  h = mix64(h ^ (static_cast<std::uint64_t>(attempt) << 32));
  return h;
}

Point2d grid_to_world(const GridFrame& frame, const Point2d& p) noexcept {
  return Point2d{frame.origin_x + frame.cos_yaw * p.x_m - frame.sin_yaw * p.y_m,
                 frame.origin_y + frame.sin_yaw * p.x_m + frame.cos_yaw * p.y_m};
}

Pose2d grid_to_world_pose(const GridFrame& frame, const Pose2d& pose) noexcept {
  const Point2d p = grid_to_world(frame, Point2d{pose.x_m, pose.y_m});
  Pose2d out;
  out.x_m = p.x_m;
  out.y_m = p.y_m;
  out.yaw_rad = normalize_angle_rad(
      pose.yaw_rad + std::atan2(frame.sin_yaw, frame.cos_yaw));
  return out;
}

}  // namespace internal

namespace {

using internal::grid_to_world_pose;
using internal::mixed_seed;
using internal::Rng;

double sinc(double x) noexcept {
  if (std::abs(x) < 1e-8) {
    const double x2 = x * x;
    return 1.0 - x2 / 6.0 + x2 * x2 / 120.0;
  }
  return std::sin(x) / x;
}

}  // namespace

namespace internal {

Pose2d pose_on_arc(const Pose2d& start, double s, double kappa) noexcept {
  const double b = kappa * s;
  const double cy = std::cos(start.yaw_rad);
  const double sy = std::sin(start.yaw_rad);
  const double q = 0.5 * b;
  const double quartic = q * sinc(q) * sinc(q);
  const double dsin = cy * sinc(b) - sy * quartic;
  const double dcos = -sy * sinc(b) - cy * quartic;
  Pose2d out;
  out.x_m = start.x_m + s * dsin;
  out.y_m = start.y_m - s * dcos;
  out.yaw_rad = normalize_angle_rad(start.yaw_rad + b);
  return out;
}

GridView make_grid(int width, int height, double resolution_m,
                   const Pose2d& origin, std::vector<int> cells,
                   const std::string& case_family) {
  GridView grid;
  grid.map_id = "grid_diff_" + case_family;
  grid.map_version = "v1";
  grid.width = width;
  grid.height = height;
  grid.resolution_m = resolution_m;
  grid.origin = origin;
  grid.cells = std::move(cells);
  return grid;
}

void rasterize_obstacle(std::vector<int>& cells, int width, int height,
                        double resolution_m,
                        const std::vector<Point2d>& polygon_grid,
                        const Aabb2d* protection, bool has_protection) {
  if (polygon_grid.empty()) {
    return;
  }
  const Aabb2d box = aabb_of(polygon_grid);
  int col0 = static_cast<int>(std::floor(box.min_x / resolution_m));
  int col1 = static_cast<int>(std::floor(box.max_x / resolution_m));
  int row0 = static_cast<int>(std::floor(box.min_y / resolution_m));
  int row1 = static_cast<int>(std::floor(box.max_y / resolution_m));
  col0 = std::max(col0, 0);
  row0 = std::max(row0, 0);
  col1 = std::min(col1, width - 1);
  row1 = std::min(row1, height - 1);
  const double tol = contact_tolerance(box.max_abs_coordinate() + 1.0);
  for (int row = row0; row <= row1; ++row) {
    for (int col = col0; col <= col1; ++col) {
      const Aabb2d cell{col * resolution_m, row * resolution_m,
                        (col + 1) * resolution_m, (row + 1) * resolution_m};
      if (has_protection && protection->intersects(cell)) {
        continue;
      }
      if (!convex_polygon_intersects_aabb(polygon_grid, cell, tol)) {
        continue;
      }
      const std::size_t idx =
          static_cast<std::size_t>(row) * width + static_cast<std::size_t>(col);
      if (cells[idx] == kCellFree) {
        cells[idx] = kCellOccupied;
      }
    }
  }
}

std::size_t rasterize_unknown_block(std::vector<int>& cells, int width,
                                    int height, double resolution_m,
                                    const Aabb2d& block_grid,
                                    const Aabb2d* protection,
                                    bool has_protection) {
  int col0 = static_cast<int>(std::floor(block_grid.min_x / resolution_m));
  int col1 = static_cast<int>(std::ceil(block_grid.max_x / resolution_m)) - 1;
  int row0 = static_cast<int>(std::floor(block_grid.min_y / resolution_m));
  int row1 = static_cast<int>(std::ceil(block_grid.max_y / resolution_m)) - 1;
  col0 = std::max(col0, 0);
  row0 = std::max(row0, 0);
  col1 = std::min(col1, width - 1);
  row1 = std::min(row1, height - 1);
  std::size_t marked = 0;
  for (int row = row0; row <= row1; ++row) {
    for (int col = col0; col <= col1; ++col) {
      const Aabb2d cell{col * resolution_m, row * resolution_m,
                        (col + 1) * resolution_m, (row + 1) * resolution_m};
      if (has_protection && protection->intersects(cell)) {
        continue;
      }
      const std::size_t idx =
          static_cast<std::size_t>(row) * width + static_cast<std::size_t>(col);
      if (cells[idx] == kCellFree) {
        cells[idx] = kCellUnknown;
        ++marked;
      }
    }
  }
  return marked;
}

Obstacle random_obstacle(Rng& rng, const Aabb2d& map_box,
                         const std::string& id) {
  const double available_x = map_box.max_x - map_box.min_x;
  const double available_y = map_box.max_y - map_box.min_y;
  const double max_side = std::min(8.0, std::min(available_x, available_y));
  if (max_side < 0.25 || rng.coin(0.5)) {
    const double w = std::min(max_side, rng.uniform(0.2, 8.0));
    const double h = std::min(max_side, rng.uniform(0.2, 8.0));
    const double cx = rng.uniform(map_box.min_x + 0.5 * w,
                                  map_box.max_x - 0.5 * w);
    const double cy = rng.uniform(map_box.min_y + 0.5 * h,
                                  map_box.max_y - 0.5 * h);
    return Obstacle::rectangle(id, cx - 0.5 * w, cy - 0.5 * h, cx + 0.5 * w,
                               cy + 0.5 * h);
  }
  const int vertices = rng.uniform_int(3, 6);
  const double side = std::min(max_side, rng.uniform(0.2, 8.0));
  const double cx = rng.uniform(map_box.min_x + 0.5 * side,
                                map_box.max_x - 0.5 * side);
  const double cy = rng.uniform(map_box.min_y + 0.5 * side,
                                map_box.max_y - 0.5 * side);
  std::vector<Point2d> points;
  points.reserve(static_cast<std::size_t>(vertices));
  for (int i = 0; i < vertices; ++i) {
    points.push_back(Point2d{rng.uniform(cx - 0.5 * side, cx + 0.5 * side),
                             rng.uniform(cy - 0.5 * side, cy + 0.5 * side)});
  }
  std::vector<Point2d> hull = convex_hull(points);
  if (hull.size() < 3 || !convex_polygon_valid(hull)) {
    return Obstacle::rectangle(id, cx - 0.5 * side, cy - 0.5 * side,
                               cx + 0.5 * side, cy + 0.5 * side);
  }
  return Obstacle::convex_polygon(id, std::move(hull));
}

Aabb2d protection_box(const std::vector<Point2d>& points, double margin) {
  Aabb2d box = aabb_of(points);
  box.expand(margin);
  return box;
}

double clearance_to_obstacles(const Footprint& footprint, const Pose2d& pose,
                              const std::vector<Obstacle>& obstacles) {
  const std::array<Point2d, 4> corners = footprint.world_corners(pose);
  const std::vector<Point2d> polygon(corners.begin(), corners.end());
  double best = std::numeric_limits<double>::infinity();
  for (const Obstacle& obstacle : obstacles) {
    best = std::min(best, convex_polygon_distance(polygon, obstacle.vertices));
  }
  return best;
}

double sweep_min_clearance(const Footprint& footprint,
                           const MotionSegment& motion,
                           const std::vector<Obstacle>& obstacles) {
  const double sign =
      motion.direction == Direction::kForward ? 1.0 : -1.0;
  const double kappa = motion.curvature_inv_m;
  const double rate = 1.0 + std::abs(kappa) * footprint.corner_radius_m;
  const double length = motion.length_m;
  const auto distance_at = [&](double arc_s) {
    const Pose2d pose = pose_on_arc(motion.start, sign * arc_s, kappa);
    return clearance_to_obstacles(footprint, pose, obstacles);
  };
  const int coarse_steps =
      std::max(2, static_cast<int>(std::ceil(length / (1e-2 / rate))));
  double best_s = 0.0;
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i <= coarse_steps; ++i) {
    const double s = length * static_cast<double>(i) / coarse_steps;
    const double d = distance_at(s);
    if (d < best) {
      best = d;
      best_s = s;
    }
  }
  const double window = 2.0 * length / coarse_steps;
  const double fine_step = 2.5e-4 / rate;
  const double lo = std::max(0.0, best_s - window);
  const double hi = std::min(length, best_s + window);
  const int fine_steps =
      std::max(2, static_cast<int>(std::ceil((hi - lo) / fine_step)));
  for (int i = 0; i <= fine_steps; ++i) {
    const double s = lo + (hi - lo) * static_cast<double>(i) / fine_steps;
    best = std::min(best, distance_at(s));
  }
  best = std::min(best, distance_at(length));
  return best;
}

}  // namespace internal

// ---------------------------------------------------------------------------
// Regime 与分配
// ---------------------------------------------------------------------------

const VehicleModel& default_test_vehicle() {
  static const VehicleModel vehicle = [] {
    VehicleModel v;
    v.id = "default_test_vehicle";
    v.wheelbase_m = 2.8;
    v.width_m = 1.8;
    v.front_overhang_m = 0.9;
    v.rear_overhang_m = 1.0;
    v.max_steer_rad = 0.5;
    v.safety_margin_m = 0.1;
    return v;
  }();
  return vehicle;
}

RegimeSpec dev_regime_spec() {
  RegimeSpec spec;
  spec.name = "dev_regression";
  spec.id_prefix = "cr-dev";
  spec.seed = 1000;
  spec.pose_checks = 5000;
  spec.motion_checks = 5000;
  return spec;
}

RegimeSpec acceptance_regime_spec(int seed) {
  RegimeSpec spec;
  spec.name = "acceptance";
  spec.id_prefix = "cr-acc";
  spec.seed = seed;
  spec.pose_checks = 500;
  spec.motion_checks = 500;
  return spec;
}

const char* to_string(Stratum stratum) noexcept {
  switch (stratum) {
    case Stratum::kWideRandom:
      return "wide_random";
    case Stratum::kCornerGraze:
      return "corner_graze";
    case Stratum::kExactContact:
      return "exact_contact";
    case Stratum::kThinObstacle:
      return "thin_obstacle";
    case Stratum::kReverseMotion:
      return "reverse_motion";
    case Stratum::kMapEdge:
      return "map_edge";
  }
  return "unknown";
}

std::vector<CaseAllocation> build_regime_allocation(const RegimeSpec& regime) {
  std::vector<CaseAllocation> allocation;
  allocation.reserve(static_cast<std::size_t>(regime.pose_checks +
                                              regime.motion_checks));
  const auto append = [&allocation](bool motion, Stratum stratum, int count) {
    for (int i = 0; i < count; ++i) {
      allocation.push_back(CaseAllocation{motion, stratum});
    }
  };
  const auto distribute = [&append](bool motion, int count,
                                    std::vector<Stratum> strata) {
    if (strata.empty()) {
      append(motion, Stratum::kWideRandom, count);
      return;
    }
    const int base = count / static_cast<int>(strata.size());
    const int remainder = count % static_cast<int>(strata.size());
    for (std::size_t i = 0; i < strata.size(); ++i) {
      append(motion, strata[i],
             base + (static_cast<int>(i) < remainder ? 1 : 0));
    }
  };
  // 位姿检查：一半宽范围随机，一半集中。按 random_spec.json，
  // reverse_motion 是仅用于运动的分层。
  const int pose_wide = regime.pose_checks / 2;
  const int pose_concentrated = regime.pose_checks - pose_wide;
  const int motion_wide = regime.motion_checks / 2;
  const int motion_concentrated = regime.motion_checks - motion_wide;
  append(false, Stratum::kWideRandom, pose_wide);
  distribute(false, pose_concentrated,
             {Stratum::kCornerGraze, Stratum::kExactContact,
              Stratum::kThinObstacle, Stratum::kMapEdge});
  append(true, Stratum::kWideRandom, motion_wide);
  distribute(true, motion_concentrated,
             {Stratum::kCornerGraze, Stratum::kExactContact,
              Stratum::kThinObstacle, Stratum::kReverseMotion,
              Stratum::kMapEdge});
  return allocation;
}

std::string format_case_id(const RegimeSpec& regime, int sequence,
                           bool motion) {
  char buffer[80];
  const int seed_width = regime.seed >= 1000 ? 4 : (regime.seed >= 100 ? 3 : 2);
  const int seq_width = regime.name == "acceptance" ? 4 : 5;
  std::snprintf(buffer, sizeof(buffer), "%s-s%0*d-%0*d-%s",
                regime.id_prefix.c_str(), seed_width, regime.seed, seq_width,
                sequence, motion ? "motion" : "pose");
  return std::string(buffer);
}

// ---------------------------------------------------------------------------
// 用例生成
// ---------------------------------------------------------------------------

namespace {

void generate_wide_random(DifferentialCase& test_case, Rng& rng) {
  constexpr double resolution = 0.1;
  const int width_m = rng.uniform_int(10, 120);
  const int height_m = rng.uniform_int(10, 120);
  const int width = width_m * 10;
  const int height = height_m * 10;
  Pose2d origin;
  origin.x_m = rng.uniform(-30.0, 30.0);
  origin.y_m = rng.uniform(-30.0, 30.0);
  origin.yaw_rad = rng.uniform(-kPi_, kPi_);
  const double world_width = width * resolution;
  const double world_height = height * resolution;

  std::vector<int> cells(static_cast<std::size_t>(width) * height, kCellFree);
  const Aabb2d map_box{0.6, 0.6, world_width - 0.6, world_height - 0.6};
  const int obstacle_count = rng.uniform_int(0, 16);
  std::vector<Obstacle> obstacles;
  for (int i = 0; i < obstacle_count; ++i) {
    Obstacle obstacle =
        internal::random_obstacle(rng, map_box, "obs_" + std::to_string(i));
    if (!obstacle.valid()) {
      continue;
    }
    internal::rasterize_obstacle(cells, width, height, resolution,
                                 obstacle.vertices, nullptr, false);
    obstacles.push_back(std::move(obstacle));
  }

  const double unknown_target = rng.uniform(0.0, 0.2);
  const double target_cells = unknown_target * width * height;
  double unknown_cells = 0.0;
  const double max_side = std::min(world_width, world_height);
  for (int attempt = 0; attempt < 8 && unknown_cells < target_cells;
       ++attempt) {
    const double side = max_side * rng.uniform(0.05, 0.3);
    Aabb2d block;
    block.min_x = rng.uniform(0.0, std::max(0.0, world_width - side));
    block.min_y = rng.uniform(0.0, std::max(0.0, world_height - side));
    block.max_x = std::min(world_width, block.min_x + side);
    block.max_y = std::min(world_height, block.min_y + side);
    unknown_cells += static_cast<double>(
        internal::rasterize_unknown_block(cells, width, height, resolution,
                                          block, nullptr, false));
  }

  test_case.scene.grid = internal::make_grid(width, height, resolution, origin,
                                             std::move(cells), test_case.case_id);
  const GridFrame frame = GridFrame::from_grid(test_case.scene.grid);
  for (Obstacle& obstacle : obstacles) {
    for (Point2d& vertex : obstacle.vertices) {
      vertex = internal::grid_to_world(frame, vertex);
    }
    test_case.scene.obstacles.push_back(std::move(obstacle));
  }

  Pose2d start_grid;
  start_grid.x_m = rng.uniform(-2.0, world_width + 2.0);
  start_grid.y_m = rng.uniform(-2.0, world_height + 2.0);
  start_grid.yaw_rad = rng.uniform(-kPi_, kPi_);
  test_case.start = internal::grid_to_world_pose(frame, start_grid);
  if (test_case.check_kind == "motion") {
    test_case.direction =
        rng.coin() ? Direction::kForward : Direction::kReverse;
    test_case.length_m = rng.uniform(0.05, 5.0);
    test_case.curvature_inv_m =
        rng.coin(0.2) ? 0.0
                      : rng.uniform(-kMaxAbsCurvature_inv_m,
                                    kMaxAbsCurvature_inv_m);
  }
}

}  // namespace

DifferentialCase generate_case(const RegimeSpec& regime,
                               const CaseAllocation& slot, int ordinal,
                               int attempt,
                               const std::string& replacement_of) {
  DifferentialCase test_case;
  test_case.case_id = format_case_id(regime, ordinal, slot.motion);
  test_case.seed = regime.seed;
  test_case.regime = regime.name;
  test_case.check_kind = slot.motion ? "motion" : "pose";
  test_case.stratum = slot.stratum;
  test_case.vehicle = default_test_vehicle();
  test_case.replacement_of = replacement_of;
  Rng rng(mixed_seed(regime.seed, ordinal, attempt));
  if (slot.stratum == Stratum::kWideRandom) {
    generate_wide_random(test_case, rng);
  } else {
    internal::generate_concentrated(test_case, rng);
  }
  return test_case;
}

CheckResult production_check(const DifferentialCase& test_case,
                             const CheckOptions& options) {
  CollisionChecker checker(test_case.vehicle, test_case.scene, options);
  if (test_case.check_kind == "pose") {
    return checker.check_pose(test_case.start);
  }
  MotionSegment motion;
  motion.start = test_case.start;
  motion.length_m = test_case.length_m;
  motion.direction = test_case.direction;
  motion.curvature_inv_m = test_case.curvature_inv_m;
  return checker.check_motion(motion);
}

ReferenceResult reference_check(const DifferentialCase& test_case) {
  if (test_case.check_kind == "pose") {
    return reference_check_pose(test_case.start, test_case.vehicle,
                                test_case.scene);
  }
  MotionSegment motion;
  motion.start = test_case.start;
  motion.length_m = test_case.length_m;
  motion.direction = test_case.direction;
  motion.curvature_inv_m = test_case.curvature_inv_m;
  return reference_check_motion(motion, test_case.vehicle, test_case.scene);
}

}  // namespace planner::collision::differential
