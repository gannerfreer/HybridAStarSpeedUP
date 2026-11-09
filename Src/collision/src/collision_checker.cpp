/**
 * @file collision_checker.cpp
 * @brief Implementation of exact pose checking and adaptive swept-motion collision detection.
 * @details
 * Algorithm Overview:
 * - Pose checking: tests the vehicle footprint polygon against convex obstacle polygons
 *   (using SAT with contact counting as collision), map rectangular boundaries, and occupied grid cells.
 * - Motion checking: recursive interval bisection. For each interval, the swept footprint is conservatively
 *   bounded by the convex hull of the 8 endpoint corners expanded by circular arc sagitta. When provably
 *   clear, the interval is pruned; otherwise it is bisected until maximum point displacement <= sweep_precision_m
 *   (default 1e-3 m). Unresolved intervals at precision floor are conservatively reported as collision.
 */

#include "collision_checker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_internal.hpp"
#include "types.hpp"

namespace planner::collision {

namespace {

using planner::common::kCellFree;
using planner::common::kCellOccupied;
using planner::common::kCellUnknown;
using planner::common::kPi;
using planner::common::normalize_angle_rad;

constexpr double kMaxAbsCurvature_inv_m = 1.0e6;  // Turning radius >= 1 um
constexpr double kSincSeriesBound = 1e-8;
constexpr int kGridChunkCells = 8;

// Multi-resolution prefix pyramid levels: block side length at level l is 2^l (1, 2, 4...).
// Configured via PLANNER_COLLISION_PREFIX_LEVELS; 0 disables this acceleration.
#ifndef PLANNER_COLLISION_PREFIX_LEVELS
#define PLANNER_COLLISION_PREFIX_LEVELS 2
#endif
constexpr int kPrefixLevels =
    PLANNER_COLLISION_PREFIX_LEVELS > 0 ? PLANNER_COLLISION_PREFIX_LEVELS : 0;
constexpr double kObstacleHashCell_m = 4.0;
constexpr double kEnvelopeSafetyFactor = 1.0 + 1e-12;
constexpr double kEnvelopeSafetyOffset_m = 1e-12;

// Numerically stable sinc(x) = sin(x)/x with series expansion near zero.
double sinc(double x) noexcept {
  if (std::abs(x) < kSincSeriesBound) {
    const double x2 = x * x;
    return 1.0 - x2 / 6.0 + x2 * x2 / 120.0;
  }
  return std::sin(x) / x;
}

// Evaluates pose along a constant-curvature circular arc at signed arc length s (negative for reverse).
// Start yaw cos/sin are precalculated by the caller to avoid repeated trigonometry in recursion.
Pose2d pose_on_arc_trig(const Pose2d& start, double s, double kappa,
                        double cy, double sy) noexcept {
  const double b = kappa * s;
  const double limit = 0.5 * b;
  const double sb = sinc(b);
  const double half = limit * sinc(limit) * sinc(limit);
  const double dsin = cy * sb - sy * half;   // (sin(y+b) - sin y) / b
  const double dcos = -sy * sb - cy * half;  // (cos(y+b) - cos y) / b
  Pose2d out;
  out.x_m = start.x_m + s * dsin;
  out.y_m = start.y_m - s * dcos;
  out.yaw_rad = normalize_angle_rad(start.yaw_rad + b);
  return out;
}

bool pose_finite(const Pose2d& pose) noexcept {
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

Aabb2d aabb_of_raw(const Point2d* points, std::size_t n) noexcept {
  Aabb2d box;
  if (n == 0) {
    return box;
  }
  box.min_x = box.max_x = points[0].x_m;
  box.min_y = box.max_y = points[0].y_m;
  for (std::size_t i = 1; i < n; ++i) {
    box.merge_point(points[i]);
  }
  return box;
}

// Monotone chain 2D convex hull for up to 8 points with zero dynamic memory allocations.
std::size_t convex_hull_small(std::array<Point2d, 8> points, std::size_t n,
                              std::array<Point2d, 8>& out) noexcept {
  if (n <= 2) {
    for (std::size_t i = 0; i < n; ++i) out[i] = points[i];
    return n;
  }
  std::sort(points.begin(), points.begin() + n,
            [](const Point2d& a, const Point2d& b) {
              if (a.x_m != b.x_m) return a.x_m < b.x_m;
              return a.y_m < b.y_m;
            });
  // Remove duplicate points.
  std::size_t m = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (m > 0 && points[i].x_m == points[m - 1].x_m &&
        points[i].y_m == points[m - 1].y_m) {
      continue;
    }
    points[m++] = points[i];
  }
  if (m <= 2) {
    for (std::size_t i = 0; i < m; ++i) out[i] = points[i];
    return m;
  }
  std::array<Point2d, 16> chain{};
  std::size_t k = 0;
  for (std::size_t i = 0; i < m; ++i) {
    while (k >= 2 && cross(chain[k - 1] - chain[k - 2], points[i] - chain[k - 1]) <= 0.0) {
      --k;
    }
    chain[k++] = points[i];
  }
  const std::size_t lower = k;
  for (std::size_t i = m - 1; i-- > 0;) {
    while (k > lower && cross(chain[k - 1] - chain[k - 2], points[i] - chain[k - 1]) <= 0.0) {
      --k;
    }
    chain[k++] = points[i];
  }
  // chain holds closed convex hull; drop duplicate closing point.
  --k;
  for (std::size_t i = 0; i < k; ++i) out[i] = chain[i];
  return k;
}

}  // namespace

// ---------------------------------------------------------------------------
// CollisionCheckerImpl
// ---------------------------------------------------------------------------

struct CollisionCheckerImpl {
  VehicleModel vehicle;
  Scene scene;
  CheckOptions options;
  Footprint footprint;
  GridFrame frame;
  bool valid{false};
  std::string invalid_reason;

  std::vector<Aabb2d> obstacle_boxes;
  std::vector<int> obstacle_order;  ///< Obstacle indices sorted by aabb.min_x for acceleration.
  double obstacle_scale_m{0.0};     ///< Maximum absolute coordinate across obstacle bounding boxes.
  bool use_obstacle_order{false};

  int chunk_size{kGridChunkCells};
  int chunks_x{0};
  int chunks_y{0};
  std::vector<std::uint8_t> chunk_blocked;
  bool use_chunk_index{false};

  /// Exact row index of non-free grid cells (independent of enable_acceleration;
  /// only alters candidate enumeration order, yielding identical decisions to full scan):
  ///   * blocked_offsets[r] .. blocked_offsets[r+1] are column indices (in ascending order)
  ///     of blocked cells in row r; total memory O(#blocked).
  bool blocked_index_valid{false};
  std::vector<int> blocked_offsets;
  std::vector<int> blocked_cols;

  /// Multi-resolution block count 2D prefix sums ("emptiness pyramid").
  /// Level l has block edge length block = 2^l; rows/cols are block-level grid dimensions;
  /// sum is an integral image of size (rows+1) x (cols+1), where each element represents
  /// the count of occupied/unknown cells (cells != kCellFree).
  /// Any rectangular query can be evaluated in O(1) via 4 table lookups.
  struct PrefixLevel {
    int block{1};
    int rows{0};
    int cols{0};
    std::vector<std::uint32_t> sum;
  };
  std::vector<PrefixLevel> prefix_levels;
};

namespace {

/// Builds multi-resolution emptiness prefix pyramid across all levels.
/// In-place 2D prefix sums allow O(1) verification of whether a bounding box is clear.
void build_prefix_pyramid(const GridView& grid,
                          std::vector<CollisionCheckerImpl::PrefixLevel>& levels) {
  levels.clear();
  levels.resize(static_cast<std::size_t>(kPrefixLevels));
  for (int level = 0; level < kPrefixLevels; ++level) {
    CollisionCheckerImpl::PrefixLevel& pyramid = levels[static_cast<std::size_t>(level)];
    pyramid.block = 1 << level;
    const int block = pyramid.block;
    pyramid.rows = (grid.height + block - 1) / block;
    pyramid.cols = (grid.width + block - 1) / block;
    const std::size_t stride = static_cast<std::size_t>(pyramid.cols) + 1;
    pyramid.sum.assign(static_cast<std::size_t>(pyramid.rows + 1) * stride, 0);
    for (int row = 0; row < grid.height; ++row) {
      const int block_row = row / block;
      for (int col = 0; col < grid.width; ++col) {
        if (grid.cells[static_cast<std::size_t>(row) * grid.width + col] ==
            kCellFree) {
          continue;
        }
        ++pyramid.sum[static_cast<std::size_t>(block_row + 1) * stride +
                      static_cast<std::size_t>(col / block + 1)];
      }
    }
    // In-place 2D prefix sum: sum[r][c] = rectangular sum of block counts in [0..r-1, 0..c-1].
    for (int r = 1; r <= pyramid.rows; ++r) {
      for (int c = 1; c <= pyramid.cols; ++c) {
        pyramid.sum[static_cast<std::size_t>(r) * stride +
                    static_cast<std::size_t>(c)] +=
            pyramid.sum[static_cast<std::size_t>(r - 1) * stride +
                        static_cast<std::size_t>(c)] +
            pyramid.sum[static_cast<std::size_t>(r) * stride +
                        static_cast<std::size_t>(c - 1)] -
            pyramid.sum[static_cast<std::size_t>(r - 1) * stride +
                        static_cast<std::size_t>(c - 1)];
      }
    }
  }
}

/// Checks if the closed cell range [row0, row1] x [col0, col1] contains no occupied or unknown cells.
/// The range is first expanded outward to block boundaries and clamped to the block grid;
/// if the 4-lookup 2D prefix sum is 0, the superset is guaranteed free of blocked cells.
/// Empty or invalid ranges are treated as clear (matching zero evaluated cells).
bool prefix_box_empty(const CollisionCheckerImpl& st, int level, int row0,
                      int row1, int col0, int col1) noexcept {
  if (row1 < row0 || col1 < col0) {
    return true;
  }
  if (row1 < 0 || col1 < 0) {
    return true;
  }
  if (level < 0 || level >= static_cast<int>(st.prefix_levels.size())) {
    return false;
  }
  const CollisionCheckerImpl::PrefixLevel& pyramid =
      st.prefix_levels[static_cast<std::size_t>(level)];
  if (pyramid.rows <= 0 || pyramid.cols <= 0) {
    return true;
  }
  const int block = pyramid.block;
  row0 = std::max(row0, 0);
  col0 = std::max(col0, 0);
  int block_row0 = row0 / block;
  int block_row1 = row1 / block;
  int block_col0 = col0 / block;
  int block_col1 = col1 / block;
  block_row0 = std::min(block_row0, pyramid.rows - 1);
  block_row1 = std::min(block_row1, pyramid.rows - 1);
  block_col0 = std::min(block_col0, pyramid.cols - 1);
  block_col1 = std::min(block_col1, pyramid.cols - 1);
  if (block_row1 < block_row0 || block_col1 < block_col0) {
    return true;
  }
  const std::size_t stride = static_cast<std::size_t>(pyramid.cols) + 1;
  const auto at = [&](int r, int c) noexcept -> std::uint32_t {
    return pyramid.sum[static_cast<std::size_t>(r) * stride +
                       static_cast<std::size_t>(c)];
  };
  const std::uint32_t total = at(block_row1 + 1, block_col1 + 1) -
                              at(block_row0, block_col1 + 1) -
                              at(block_row1 + 1, block_col0) +
                              at(block_row0, block_col0);
  return total == 0;
}

/// Tests pyramid levels from coarse to fine; returns true if any level proves the bounding box contains no blocked cells.
bool prefix_pyramid_empty(const CollisionCheckerImpl& st, int row0, int row1,
                          int col0, int col1) noexcept {
  for (int level = static_cast<int>(st.prefix_levels.size()) - 1; level >= 0;
       --level) {
    if (prefix_box_empty(st, level, row0, row1, col0, col1)) {
      return true;
    }
  }
  return false;
}

/// Enumerates blocked cells in [row0, row1] x [col0, col1] in exact row-major order.
/// Free cells return immediately in test_cell, so skipping them does not alter decisions.
/// Enumeration matches full scanning order (row ascending, col ascending) and terminates
/// as soon as fn(row, col) returns false.
template <typename Fn>
void for_each_blocked_cell_in(const CollisionCheckerImpl& st, int row0, int row1,
                              int col0, int col1, Fn&& fn) {
  const int* cols = st.blocked_cols.data();
  for (int row = row0; row <= row1; ++row) {
    const int* begin = cols + st.blocked_offsets[static_cast<std::size_t>(row)];
    const int* end = cols + st.blocked_offsets[static_cast<std::size_t>(row) + 1];
    const int* it = std::lower_bound(begin, end, col0);
    for (; it != end && *it <= col1; ++it) {
      if (!fn(row, *it)) {
        return;
      }
    }
  }
}

struct PoseHit {
  bool hit{false};
  CollisionReason reason{CollisionReason::kNone};
  std::string detail;
};

std::array<Point2d, 4> grid_corners_from_world(
    const CollisionCheckerImpl& st, const std::array<Point2d, 4>& world) {
  std::array<Point2d, 4> out;
  for (std::size_t i = 0; i < 4; ++i) {
    out[i] = st.frame.to_grid_frame(world[i]);
  }
  return out;
}

std::string format_double(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.12g", value);
  return std::string(buffer);
}

/// Traverses indices of obstacles that may lie within `reach` distance of `query`.
/// Acceleration reorders enumeration without altering candidate tests, ensuring identical decisions.
template <typename Fn>
void for_each_obstacle_near(const CollisionCheckerImpl& st, const Aabb2d& query, double reach,
                            Fn&& fn) {
  if (st.obstacle_boxes.empty()) {
    return;
  }
  const auto close_enough = [&](const Aabb2d& box) {
    return aabb_within(box, query, reach);
  };
  if (st.use_obstacle_order) {
    for (const int idx : st.obstacle_order) {
      const Aabb2d& box = st.obstacle_boxes[static_cast<std::size_t>(idx)];
      if (box.min_x > query.max_x + reach) {
        break;  // Sorted by min_x
      }
      if (close_enough(box)) {
        fn(idx);
      }
    }
  } else {
    for (std::size_t i = 0; i < st.obstacle_boxes.size(); ++i) {
      if (close_enough(st.obstacle_boxes[i])) {
        fn(static_cast<int>(i));
      }
    }
  }
}

PoseHit evaluate_pose(const CollisionCheckerImpl& st, const Pose2d& pose) {
  PoseHit result;
  const std::array<Point2d, 4> world = st.footprint.world_corners(pose);
  // Reuse world corners: transform directly to grid frame to avoid redundant trigonometric calls.
  const std::array<Point2d, 4> grid = grid_corners_from_world(st, world);
  const Aabb2d world_box = aabb_of_raw(world.data(), world.size());
  const std::vector<Point2d> footprint_world(world.begin(), world.end());

  // 1) Out-of-bounds check: any point outside the closed map rectangle is considered occupied.
  if (st.scene.has_grid()) {
    for (const Point2d& p : grid) {
      if (!st.frame.inside_map(p)) {
        result.hit = true;
        result.reason = CollisionReason::kOutsideMap;
        result.detail = "footprint leaves the map rectangle";
        return result;
      }
    }
  }

  // 2) Polygon obstacles (world frame): contact occurs if minimum Euclidean distance <= scale-dependent tolerance.
  if (!st.obstacle_boxes.empty()) {
    const double footprint_scale = world_box.max_abs_coordinate();
    const double filter_tol =
        contact_tolerance(std::max(footprint_scale, st.obstacle_scale_m)) * 2.0;
    PoseHit obstacle_hit;
    for_each_obstacle_near(st, world_box, filter_tol, [&](int idx) {
      if (obstacle_hit.hit) {
        return;
      }
      const Obstacle& obstacle = st.scene.obstacles[static_cast<std::size_t>(idx)];
      const double tol =
          contact_tolerance(std::max(footprint_scale,
                                     obstacle.aabb().max_abs_coordinate()));
      // O(1) coarse filter: if the separating axis projection gap exceeds tol,
      // minimum Euclidean distance d >= gap > tol; polygon cannot touch, safely skipping exact distance.
      if (!convex_polygons_intersect(footprint_world, obstacle.vertices,
                                     tol)) {
        return;
      }
      if (convex_polygon_distance_sq(footprint_world, obstacle.vertices) <=
          tol * tol) {
        obstacle_hit.hit = true;
        obstacle_hit.reason = CollisionReason::kObstacle;
        obstacle_hit.detail = "obstacle '" + obstacle.id +
                              "' overlaps the vehicle footprint";
      }
    });
    if (obstacle_hit.hit) {
      return obstacle_hit;
    }
  }

  // 3) Blocked grid cells (grid coordinate frame, cells represented as axis-aligned bounding boxes).
  if (st.scene.has_grid()) {
    const GridView& grid_view = st.scene.grid;
    const double res = st.frame.resolution_m;
    double min_x = grid[0].x_m, max_x = grid[0].x_m;
    double min_y = grid[0].y_m, max_y = grid[0].y_m;
    for (const Point2d& p : grid) {
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
    col1 = std::min(col1, grid_view.width - 1);
    row1 = std::min(row1, grid_view.height - 1);
    // Multi-resolution prefix emptiness check: if any level proves the bounding box
    // contains no blocked cells, return early without cell scanning.
    if (prefix_pyramid_empty(st, row0, row1, col0, col1)) {
      return PoseHit{};
    }
    const double scale =
        std::max(std::max(std::abs(min_x), std::abs(max_x)),
                 std::max(std::abs(min_y), std::abs(max_y)));
    const double tol = contact_tolerance(scale);
    // SAT precomputation for cell scanning: the footprint polygon is fixed within evaluate_pose,
    // so bounding box and edge normals are computed once and reused across cells.
    internal::PolygonSatCache8 cell_sat;
    cell_sat.build(grid.data(), grid.size());
    const std::vector<Point2d> footprint_grid_polygon(grid.begin(), grid.end());
    const double cell_tol_sq = tol * tol;

    PoseHit cell_hit;
    const auto test_cell = [&](int row, int col) {
      if (cell_hit.hit) {
        return;
      }
      const int value =
          grid_view.cells[static_cast<std::size_t>(row) * grid_view.width + col];
      if (value == kCellFree) {
        return;
      }
      const Aabb2d cell_box{col * res, row * res, (col + 1) * res,
                            (row + 1) * res};
      // O(1) coarse filter: if separating axis projection gap exceeds tolerance,
      // minimum distance > tol, so contact is impossible.
      if (!cell_sat.maybe_intersects(cell_box, tol)) {
        return;
      }
      // Exact Euclidean distance check in squared domain.
      if (polygon_aabb_distance_sq(footprint_grid_polygon, cell_box) >
          cell_tol_sq) {
        return;
      }
      cell_hit.hit = true;
      cell_hit.reason = (value == kCellOccupied) ? CollisionReason::kGridOccupied
                                                 : CollisionReason::kGridUnknown;
      cell_hit.detail = "grid cell (" + std::to_string(row) + "," +
                        std::to_string(col) + ") " +
                        (value == kCellOccupied ? "occupied" : "unknown") +
                        " overlaps the vehicle footprint";
    };

    if (st.blocked_index_valid) {
      // Exact candidate enumeration: iterate only over blocked cells in row-major order.
      for_each_blocked_cell_in(st, row0, row1, col0, col1,
                               [&](int row, int col) {
                                 test_cell(row, col);
                                 return !cell_hit.hit;
                               });
    } else if (st.use_chunk_index && !st.chunk_blocked.empty()) {
      const int chunk = st.chunk_size;
      const int c0 = col0 / chunk;
      const int c1 = col1 / chunk;
      const int r0 = row0 / chunk;
      const int r1 = row1 / chunk;
      // Skip empty chunks; maintain row-major traversal order.
      for (int cr = r0; cr <= r1 && !cell_hit.hit; ++cr) {
        const int rr0 = std::max(row0, cr * chunk);
        const int rr1 = std::min(row1, (cr + 1) * chunk - 1);
        for (int row = rr0; row <= rr1 && !cell_hit.hit; ++row) {
          for (int cc = c0; cc <= c1 && !cell_hit.hit; ++cc) {
            if (st.chunk_blocked[static_cast<std::size_t>(cr) * st.chunks_x + cc] == 0) {
              continue;
            }
            const int cc0 = std::max(col0, cc * chunk);
            const int cc1 = std::min(col1, (cc + 1) * chunk - 1);
            for (int col = cc0; col <= cc1 && !cell_hit.hit; ++col) {
              test_cell(row, col);
            }
          }
        }
      }
    } else {
      for (int row = row0; row <= row1 && !cell_hit.hit; ++row) {
        for (int col = col0; col <= col1 && !cell_hit.hit; ++col) {
          test_cell(row, col);
        }
      }
    }
    if (cell_hit.hit) {
      return cell_hit;
    }
  }
  return result;
}

// ---------------------------------------------------------------------------
// Swept motion engine
// ---------------------------------------------------------------------------

struct Envelope {
  std::vector<Point2d> hull;  ///< Convex hull in world coordinates.
  double sagitta_m{0.0};      ///< Maximum sagitta deviation of corner arcs from chords.
};

/// Conservative bounding envelope of swept footprint across an interval:
/// hull(corners(s0), corners(s1)) expanded by corner arc sagitta.
Envelope build_envelope(const CollisionCheckerImpl& st, const Pose2d& p0, const Pose2d& p1,
                        double delta_yaw, double kappa) {
  Envelope envelope;
  const std::array<Point2d, 4> c0 = st.footprint.world_corners(p0);
  const std::array<Point2d, 4> c1 = st.footprint.world_corners(p1);
  const double abs_yaw = std::abs(delta_yaw);
  if (abs_yaw >= kPi) {
    // Pathological multi-turn interval: fall back to conservative bound
    // "distance to chord <= 2 * arc_radius", forcing further bisection.
    const double arc_radius =
        (kappa != 0.0 ? std::abs(1.0 / kappa) : 0.0) +
        st.footprint.corner_radius_m;
    envelope.sagitta_m = 2.0 * arc_radius * kEnvelopeSafetyFactor +
                         kEnvelopeSafetyOffset_m;
  } else if (delta_yaw != 0.0) {
    const double tan_quarter = std::abs(std::tan(0.25 * delta_yaw));
    for (std::size_t i = 0; i < 4; ++i) {
      const double chord = distance(c0[i], c1[i]);
      envelope.sagitta_m =
          std::max(envelope.sagitta_m, 0.5 * chord * tan_quarter);
    }
    envelope.sagitta_m =
        envelope.sagitta_m * kEnvelopeSafetyFactor + kEnvelopeSafetyOffset_m;
  }
  std::array<Point2d, 8> points{};
  std::size_t n = 0;
  for (const Point2d& p : c0) points[n++] = p;
  for (const Point2d& p : c1) points[n++] = p;
  std::array<Point2d, 8> hull{};
  const std::size_t hull_n = convex_hull_small(points, n, hull);
  envelope.hull.assign(hull.begin(), hull.begin() + hull_n);
  return envelope;
}

/// Returns true if the swept envelope is provably disjoint from all obstacles
/// and blocked cells, and remains strictly inside the map boundary.
bool envelope_provably_clear(const CollisionCheckerImpl& st, const Envelope& envelope) {
  if (envelope.hull.empty()) {
    return false;
  }
  const Aabb2d hull_box = aabb_of(envelope.hull);
  const double scale = hull_box.max_abs_coordinate() + 1.0;
  const double tol = contact_tolerance(scale);
  const double reach = envelope.sagitta_m + tol;
  // Obstacles may reside at larger coordinates; tolerance must match pairwise policy.
  const double obstacle_reach =
      envelope.sagitta_m +
      contact_tolerance(std::max(scale, st.obstacle_scale_m));

  // Grid check: map boundary inclusion plus blocked cells.
  if (st.scene.has_grid()) {
    std::vector<Point2d> grid_hull;
    grid_hull.reserve(envelope.hull.size());
    for (const Point2d& p : envelope.hull) {
      grid_hull.push_back(st.frame.to_grid_frame(p));
    }
    const Aabb2d hull_grid_box = aabb_of(grid_hull);
    for (const Point2d& p : grid_hull) {
      // distance_to_boundary <= reach covers both boundary proximity and outside map.
      if (st.frame.distance_to_boundary(p) <= reach) {
        return false;
      }
    }
    const GridView& grid_view = st.scene.grid;
    const double res = st.frame.resolution_m;
    Aabb2d query_box = hull_grid_box;
    query_box.expand(reach);
    int col0 = static_cast<int>(std::floor(query_box.min_x / res));
    int col1 = static_cast<int>(std::ceil(query_box.max_x / res));
    int row0 = static_cast<int>(std::floor(query_box.min_y / res));
    int row1 = static_cast<int>(std::ceil(query_box.max_y / res));
    col0 = std::max(col0, 0);
    row0 = std::max(row0, 0);
    col1 = std::min(col1, grid_view.width);
    row1 = std::min(row1, grid_view.height);
    // Multi-resolution prefix emptiness check on closed cell range [row0, row1-1] x [col0, col1-1].
    // If any level proves no blocked cells exist, cell scanning can be safely skipped.
    const bool grid_empty =
        prefix_pyramid_empty(st, row0, row1 - 1, col0, col1 - 1);
    bool clear = true;
    // Precompute SAT cache for the envelope hull.
    internal::PolygonSatCache8 hull_sat;
    hull_sat.build(grid_hull.data(), grid_hull.size());
    const auto test_cell = [&](int row, int col) {
      const int value =
          grid_view.cells[static_cast<std::size_t>(row) * grid_view.width + col];
      if (value == kCellFree) {
        return;
      }
      const Aabb2d cell_box{col * res, row * res, (col + 1) * res,
                            (row + 1) * res};
      if (!aabb_within(cell_box, hull_grid_box, reach)) {
        return;  // AABB filtering: aabb_dist <= polygon_dist
      }
      // O(1) coarse filter: if separating axis projection gap > reach, distance > reach.
      if (!hull_sat.maybe_intersects(cell_box, reach)) {
        return;
      }
      if (polygon_aabb_distance_sq(grid_hull, cell_box) <= reach * reach) {
        clear = false;
      }
    };
    if (!grid_empty) {
      if (st.blocked_index_valid) {
        // Exact candidate enumeration: iterate only over blocked cells in row-major order.
        for_each_blocked_cell_in(st, row0, row1 - 1, col0, col1 - 1,
                                 [&](int row, int col) {
                                   test_cell(row, col);
                                   return clear;
                                 });
      } else if (st.use_chunk_index && !st.chunk_blocked.empty()) {
        const int chunk = st.chunk_size;
        const int c0 = col0 / chunk;
        const int c1 = col1 / chunk;
        const int r0 = row0 / chunk;
        const int r1 = row1 / chunk;
        for (int cr = r0; cr <= r1 && clear; ++cr) {
          for (int cc = c0; cc <= c1 && clear; ++cc) {
            if (st.chunk_blocked[static_cast<std::size_t>(cr) * st.chunks_x + cc] == 0) {
              continue;
            }
            const int cc0 = std::max(col0, cc * chunk);
            const int cc1 = std::min(col1, (cc + 1) * chunk);
            const int rr0 = std::max(row0, cr * chunk);
            const int rr1 = std::min(row1, (cr + 1) * chunk);
            for (int row = rr0; row < rr1 && clear; ++row) {
              for (int col = cc0; col < cc1 && clear; ++col) {
                test_cell(row, col);
              }
            }
          }
        }
      } else {
        for (int row = row0; row < row1 && clear; ++row) {
          for (int col = col0; col < col1 && clear; ++col) {
            test_cell(row, col);
          }
        }
      }
    }
    if (!clear) {
      return false;
    }
  }

  // Obstacle checks.
  bool clear = true;
  for_each_obstacle_near(st, hull_box, obstacle_reach, [&](int idx) {
    if (!clear) {
      return;
    }
    const Obstacle& obstacle = st.scene.obstacles[static_cast<std::size_t>(idx)];
    // O(1) SAT coarse filter: skip exact distance when separated by more than obstacle_reach.
    if (!convex_polygons_intersect(envelope.hull, obstacle.vertices,
                                   obstacle_reach)) {
      return;
    }
    const double d_sq =
        convex_polygon_distance_sq(envelope.hull, obstacle.vertices);
    if (d_sq <= obstacle_reach * obstacle_reach) {
      clear = false;
    }
  });
  return clear;
}

struct SweepContext {
  const CollisionCheckerImpl* state{nullptr};
  Pose2d start;
  double kappa{0.0};
  double precision_m{1e-3};
  // Cosine and sine of start yaw (constant throughout swept motion check; precomputed to avoid repeated trig).
  double cos_yaw{1.0};
  double sin_yaw{0.0};
};

struct Decision {
  enum class Kind { kClear, kCollision, kUncertain };
  Kind kind{Kind::kClear};
  double s{0.0};
  Pose2d pose;
  CollisionReason reason{CollisionReason::kNone};
  std::string detail;
};

double interval_displacement(const SweepContext& ctx, const Pose2d& p0,
                             const Pose2d& p1, double delta_yaw) {
  const std::array<Point2d, 4> c0 = ctx.state->footprint.world_corners(p0);
  const std::array<Point2d, 4> c1 = ctx.state->footprint.world_corners(p1);
  double max_chord = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    max_chord = std::max(max_chord, distance(c0[i], c1[i]));
  }
  const double abs_yaw = std::abs(delta_yaw);
  if (abs_yaw >= kPi) {
    // Conservative bound: displacement <= |delta_yaw| * (|1/kappa| + corner_radius).
    const double arc_radius =
        (ctx.kappa != 0.0 ? std::abs(1.0 / ctx.kappa) : 0.0) +
        ctx.state->footprint.corner_radius_m;
    return abs_yaw * arc_radius;
  }
  if (abs_yaw == 0.0) {
    return max_chord;
  }
  const double half = 0.5 * abs_yaw;
  const double ratio = half / std::sin(half);
  return max_chord * ratio;
}

/// Resolves swept interval (s0, s1]; caller guarantees pose(s0) is collision-free.
/// Returns the first collision (in traversal order), first uncertain interval, or clear.
Decision resolve(const SweepContext& ctx, double s0, const Pose2d& p0,
                 double s1) {
  const CollisionCheckerImpl& st = *ctx.state;
  const Pose2d p1 =
      pose_on_arc_trig(ctx.start, s1, ctx.kappa, ctx.cos_yaw, ctx.sin_yaw);
  const PoseHit hit1 = evaluate_pose(st, p1);
  const double delta_yaw = ctx.kappa * (s1 - s0);
  const bool at_floor =
      interval_displacement(ctx, p0, p1, delta_yaw) <= ctx.precision_m;

  if (hit1.hit) {
    // Collision detected in (s0, s1]. Continue bisection to report the earliest collision.
    if (at_floor) {
      Decision decision;
      decision.kind = Decision::Kind::kCollision;
      decision.s = s1;
      decision.pose = p1;
      decision.reason = hit1.reason;
      decision.detail = hit1.detail;
      return decision;
    }
    const double sm = 0.5 * (s0 + s1);
    const Pose2d pm =
        pose_on_arc_trig(ctx.start, sm, ctx.kappa, ctx.cos_yaw, ctx.sin_yaw);
    const PoseHit hitm = evaluate_pose(st, pm);
    if (hitm.hit) {
      return resolve(ctx, s0, p0, sm);
    }
    Decision left = resolve(ctx, s0, p0, sm);
    if (left.kind != Decision::Kind::kClear) {
      return left;
    }
    return resolve(ctx, sm, pm, s1);
  }

  const Envelope envelope = build_envelope(st, p0, p1, delta_yaw, ctx.kappa);
  if (envelope_provably_clear(st, envelope)) {
    return Decision{};
  }
  if (at_floor) {
    Decision decision;
    decision.kind = Decision::Kind::kUncertain;
    decision.s = s0;
    decision.pose = p0;
    decision.reason = CollisionReason::kPrecisionFloor;
    decision.detail = "swept interval unresolved at the precision floor (" +
                      format_double(ctx.precision_m) +
                      " m); treated as collision";
    return decision;
  }
  const double sm = 0.5 * (s0 + s1);
  const Pose2d pm =
      pose_on_arc_trig(ctx.start, sm, ctx.kappa, ctx.cos_yaw, ctx.sin_yaw);
  const PoseHit hitm = evaluate_pose(st, pm);
  if (hitm.hit) {
    return resolve(ctx, s0, p0, sm);
  }
  Decision left = resolve(ctx, s0, p0, sm);
  if (left.kind != Decision::Kind::kClear) {
    return left;
  }
  return resolve(ctx, sm, pm, s1);
}

// ---------------------------------------------------------------------------
// Input validation helpers
// ---------------------------------------------------------------------------

CheckResult invalid_result(const std::string& detail) {
  CheckResult result;
  result.status = CheckStatus::kInvalidInput;
  result.reason_code = CollisionReason::kInvalidInput;
  result.detail = detail;
  return result;
}

bool validate_motion_segment(const MotionSegment& segment,
                             std::string* reason) {
  if (!pose_finite(segment.start)) {
    *reason = "motion start pose is not finite";
    return false;
  }
  if (!std::isfinite(segment.length_m) || segment.length_m < 0.0) {
    *reason = "motion length must be finite and non-negative";
    return false;
  }
  if (!std::isfinite(segment.curvature_inv_m)) {
    *reason = "motion curvature is not finite";
    return false;
  }
  if (std::abs(segment.curvature_inv_m) > kMaxAbsCurvature_inv_m) {
    *reason = "motion curvature magnitude exceeds the supported limit";
    return false;
  }
  return true;
}

CheckResult result_from_decision(const Decision& decision) {
  CheckResult result;
  if (decision.kind == Decision::Kind::kClear) {
    result.status = CheckStatus::kClear;
    return result;
  }
  result.status = CheckStatus::kCollision;
  result.reason_code = decision.reason;
  result.detail = decision.detail;
  result.first_pose = decision.pose;
  result.first_s_m = std::abs(decision.s);
  return result;
}

CheckResult check_motion_impl(const CollisionCheckerImpl& st, const MotionSegment& segment) {
  std::string reason;
  if (!validate_motion_segment(segment, &reason)) {
    return invalid_result(reason);
  }
  const PoseHit start_hit = evaluate_pose(st, segment.start);
  if (start_hit.hit) {
    CheckResult result;
    result.status = CheckStatus::kCollision;
    result.reason_code = start_hit.reason;
    result.detail = "colliding start: " + start_hit.detail;
    result.first_pose = segment.start;
    result.first_s_m = 0.0;
    return result;
  }
  if (segment.length_m == 0.0) {
    CheckResult result;
    result.status = CheckStatus::kClear;
    return result;
  }
  SweepContext ctx;
  ctx.state = &st;
  ctx.start = segment.start;
  ctx.kappa = segment.curvature_inv_m;
  ctx.precision_m = st.options.sweep_precision_m;
  ctx.cos_yaw = std::cos(segment.start.yaw_rad);
  ctx.sin_yaw = std::sin(segment.start.yaw_rad);
  const double sign = segment.direction == Direction::kForward ? 1.0 : -1.0;
  const double sigma = sign * segment.length_m;
  return result_from_decision(resolve(ctx, 0.0, segment.start, sigma));
}

CheckResult check_pose_impl(const CollisionCheckerImpl& st, const Pose2d& pose) {
  if (!pose_finite(pose)) {
    return invalid_result("pose is not finite");
  }
  const PoseHit hit = evaluate_pose(st, pose);
  CheckResult result;
  if (!hit.hit) {
    result.status = CheckStatus::kClear;
    return result;
  }
  result.status = CheckStatus::kCollision;
  result.reason_code = hit.reason;
  result.detail = hit.detail;
  result.first_pose = pose;
  result.first_s_m = 0.0;
  return result;
}

bool validate_path_segment(const PathSegment& segment, std::string* reason) {
  if (segment.points.empty()) {
    *reason = "path segment has no points";
    return false;
  }
  const Direction direction = segment.points.front().direction;
  double previous_s = -1.0;
  for (std::size_t i = 0; i < segment.points.size(); ++i) {
    const auto& point = segment.points[i];
    if (!pose_finite(point.pose)) {
      *reason = "path point " + std::to_string(i) + " pose is not finite";
      return false;
    }
    if (!std::isfinite(point.s_m) || point.s_m < 0.0) {
      *reason = "path point " + std::to_string(i) +
                " arc length must be finite and non-negative";
      return false;
    }
    if (i > 0 && !(point.s_m > previous_s)) {
      *reason = "path point arc lengths must be strictly increasing";
      return false;
    }
    if (point.direction != direction) {
      *reason = "path segment mixes travel directions; gear changes must "
                "start a new segment";
      return false;
    }
    if (!std::isfinite(point.curvature_inv_m) ||
        std::abs(point.curvature_inv_m) > kMaxAbsCurvature_inv_m) {
      *reason = "path point " + std::to_string(i) +
                " curvature is not finite or out of range";
      return false;
    }
    previous_s = point.s_m;
  }
  return true;
}

CheckResult check_path_impl(const CollisionCheckerImpl& st,
                            const std::vector<PathSegment>& path) {
  if (path.empty()) {
    return invalid_result("path is empty");
  }
  for (std::size_t seg = 0; seg < path.size(); ++seg) {
    std::string reason;
    if (!validate_path_segment(path[seg], &reason)) {
      return invalid_result("path segment " + std::to_string(seg) + ": " +
                            reason);
    }
    const auto& points = path[seg].points;
    const double base_s = points.front().s_m;
    CheckResult first = check_pose_impl(st, points.front().pose);
    if (first.collided()) {
      first.path_segment_index = static_cast<int>(seg);
      first.first_s_m = 0.0;
      return first;
    }
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      MotionSegment motion;
      motion.start = points[i].pose;
      motion.length_m = points[i + 1].s_m - points[i].s_m;
      motion.direction = points[i].direction;
      motion.curvature_inv_m = points[i].curvature_inv_m;
      if (motion.length_m == 0.0) {
        continue;  // Duplicate waypoint
      }
      CheckResult step = check_motion_impl(st, motion);
      if (step.collided()) {
        step.path_segment_index = static_cast<int>(seg);
        step.first_s_m = (points[i].s_m - base_s) + step.first_s_m;
        return step;
      }
    }
    // Always explicitly check the stored end pose, even if arc reconstruction slightly drifts.
    CheckResult last = check_pose_impl(st, points.back().pose);
    if (last.collided()) {
      last.path_segment_index = static_cast<int>(seg);
      last.first_s_m = points.back().s_m - base_s;
      return last;
    }
  }
  CheckResult result;
  result.status = CheckStatus::kClear;
  return result;
}

}  // namespace

// ---------------------------------------------------------------------------
// CollisionChecker
// ---------------------------------------------------------------------------

CollisionChecker::CollisionChecker(const VehicleModel& vehicle,
                                   const Scene& scene, CheckOptions options) {
  auto state = std::make_shared<CollisionCheckerImpl>();
  state->vehicle = vehicle;
  state->scene = scene;
  state->options = options;
  std::string reason;
  if (!vehicle_footprint_valid(vehicle, &reason)) {
    state->valid = false;
  } else if (!scene.valid(&reason)) {
    state->valid = false;
  } else if (!std::isfinite(options.sweep_precision_m) ||
             !(options.sweep_precision_m > 0.0)) {
    reason = "sweep precision must be finite and positive";
    state->valid = false;
  } else {
    state->valid = true;
  }
  state->invalid_reason = reason;
  state->footprint = make_footprint(vehicle);
  if (scene.has_grid()) {
    state->frame = GridFrame::from_grid(scene.grid);
  }

  if (state->valid) {
    for (Obstacle& obstacle : state->scene.obstacles) {
      normalize_counter_clockwise(obstacle.vertices);
    }
    state->obstacle_boxes.reserve(state->scene.obstacles.size());
    for (const Obstacle& obstacle : state->scene.obstacles) {
      state->obstacle_boxes.push_back(obstacle.aabb());
      state->obstacle_scale_m =
          std::max(state->obstacle_scale_m,
                   state->obstacle_boxes.back().max_abs_coordinate());
    }
    if (scene.has_grid()) {
      // Exact row index of blocked cells (always built; independent of enable_acceleration,
      // changing only candidate enumeration while maintaining identical decisions to full scan).
      // Two-pass construction: first count per row to establish offsets, then fill columns in ascending order.
      const int height = scene.grid.height;
      const int width = scene.grid.width;
      state->blocked_offsets.assign(static_cast<std::size_t>(height) + 1, 0);
      std::size_t total = 0;
      for (int row = 0; row < height; ++row) {
        int count = 0;
        for (int col = 0; col < width; ++col) {
          const int value = scene.grid.cells[static_cast<std::size_t>(row) *
                                                 width +
                                             col];
          if (value != kCellFree) {
            ++count;
          }
        }
        state->blocked_offsets[static_cast<std::size_t>(row) + 1] =
            state->blocked_offsets[static_cast<std::size_t>(row)] + count;
        total += static_cast<std::size_t>(count);
      }
      // Fall back to direct scanning if the blocked set exceeds memory threshold (8M entries = 32 MB).
      constexpr std::size_t kMaxBlockedIndexEntries = 8u << 20;
      if (total <= kMaxBlockedIndexEntries) {
        state->blocked_cols.resize(total);
        std::size_t cursor = 0;
        for (int row = 0; row < height; ++row) {
          for (int col = 0; col < width; ++col) {
            const int value = scene.grid.cells[static_cast<std::size_t>(row) *
                                                   width +
                                               col];
            if (value != kCellFree) {
              state->blocked_cols[cursor++] = col;
            }
          }
        }
        state->blocked_index_valid = true;
      }
    }
    if (scene.has_grid() && kPrefixLevels > 0) {
      build_prefix_pyramid(state->scene.grid, state->prefix_levels);
    }
    if (options.enable_acceleration) {
      state->use_obstacle_order = true;
      state->obstacle_order.resize(state->scene.obstacles.size());
      for (std::size_t i = 0; i < state->obstacle_order.size(); ++i) {
        state->obstacle_order[i] = static_cast<int>(i);
      }
      std::sort(state->obstacle_order.begin(), state->obstacle_order.end(),
                [&](int a, int b) {
                  return state->obstacle_boxes[static_cast<std::size_t>(a)].min_x <
                         state->obstacle_boxes[static_cast<std::size_t>(b)].min_x;
                });
      if (scene.has_grid()) {
        state->use_chunk_index = true;
        state->chunks_x =
            (scene.grid.width + kGridChunkCells - 1) / kGridChunkCells;
        state->chunks_y =
            (scene.grid.height + kGridChunkCells - 1) / kGridChunkCells;
        state->chunk_blocked.assign(
            static_cast<std::size_t>(state->chunks_x) * state->chunks_y, 0);
        for (int row = 0; row < scene.grid.height; ++row) {
          for (int col = 0; col < scene.grid.width; ++col) {
            const int value = scene.grid.cells[static_cast<std::size_t>(row) *
                                                   scene.grid.width +
                                               col];
            if (value != kCellFree) {
              const std::size_t idx =
                  static_cast<std::size_t>(row / kGridChunkCells) *
                      state->chunks_x +
                  static_cast<std::size_t>(col / kGridChunkCells);
              state->chunk_blocked[idx] = 1;
            }
          }
        }
      }
    }
  }
  state_ = std::move(state);
}

bool CollisionChecker::valid() const noexcept { return state_->valid; }

const std::string& CollisionChecker::invalid_reason() const noexcept {
  return state_->invalid_reason;
}

const VehicleModel& CollisionChecker::vehicle() const noexcept {
  return state_->vehicle;
}

const Scene& CollisionChecker::scene() const noexcept { return state_->scene; }

const CheckOptions& CollisionChecker::options() const noexcept {
  return state_->options;
}

CheckResult CollisionChecker::check_pose(const Pose2d& pose) const {
  if (!state_->valid) {
    return invalid_result(state_->invalid_reason);
  }
  return check_pose_impl(*state_, pose);
}

CheckResult CollisionChecker::check_motion(const MotionSegment& segment) const {
  if (!state_->valid) {
    return invalid_result(state_->invalid_reason);
  }
  return check_motion_impl(*state_, segment);
}

CheckResult CollisionChecker::check_path(const PathSegment& segment) const {
  if (!state_->valid) {
    return invalid_result(state_->invalid_reason);
  }
  return check_path_impl(*state_, std::vector<PathSegment>{segment});
}

CheckResult CollisionChecker::check_path(
    const std::vector<PathSegment>& path) const {
  if (!state_->valid) {
    return invalid_result(state_->invalid_reason);
  }
  return check_path_impl(*state_, path);
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

CheckResult check_pose(const Pose2d& pose, const VehicleModel& vehicle,
                       const Scene& scene, const CheckOptions& options) {
  CollisionChecker checker(vehicle, scene, options);
  return checker.check_pose(pose);
}

CheckResult check_motion(const MotionSegment& segment, const VehicleModel& vehicle,
                         const Scene& scene, const CheckOptions& options) {
  CollisionChecker checker(vehicle, scene, options);
  return checker.check_motion(segment);
}

CheckResult check_path(const PathSegment& segment, const VehicleModel& vehicle,
                       const Scene& scene, const CheckOptions& options) {
  CollisionChecker checker(vehicle, scene, options);
  return checker.check_path(segment);
}

CheckResult check_path(const std::vector<PathSegment>& path,
                       const VehicleModel& vehicle, const Scene& scene,
                       const CheckOptions& options) {
  CollisionChecker checker(vehicle, scene, options);
  return checker.check_path(path);
}

const char* to_string(CheckStatus status) noexcept {
  switch (status) {
    case CheckStatus::kClear:
      return "clear";
    case CheckStatus::kCollision:
      return "collision";
    case CheckStatus::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

const char* to_string(CollisionReason reason) noexcept {
  switch (reason) {
    case CollisionReason::kNone:
      return "none";
    case CollisionReason::kObstacle:
      return "obstacle";
    case CollisionReason::kGridOccupied:
      return "grid_occupied";
    case CollisionReason::kGridUnknown:
      return "grid_unknown";
    case CollisionReason::kOutsideMap:
      return "outside_map";
    case CollisionReason::kPrecisionFloor:
      return "precision_floor";
    case CollisionReason::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

}  // namespace planner::collision
