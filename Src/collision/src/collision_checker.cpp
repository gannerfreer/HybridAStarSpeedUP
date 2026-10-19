// 生产碰撞检测：精确位姿检查（障碍物 + 栅格）与带自适应细分的保守扫掠
// 运动检查。
//
// 算法概要（见 collision_checker.hpp 与 Docs §6.2/§7.1）：
//   * 位姿检查：足迹多边形对凸多边形障碍物（分离轴，接触计为碰撞）以及
//     对地图矩形 / 被阻塞栅格单元；
//   * 运动检查：递归区间细分。对每个区间，扫掠区域由八个端点角点位置的
//     凸包外扩精确的圆弧矢高覆盖，这是扫掠足迹的保守超集。当该包络可证明
//     无碰撞时丢弃该区间；否则继续细分，直到车辆任一点的最大位移 <=
//     sweep_precision_m（默认 1e-3 m）。在精度下限处仍未解决的区间报告为
//     碰撞（保守方向，不漏检接触）。

#include "planner/collision/collision_checker.hpp"

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

#include "planner/common/types.hpp"

namespace planner::collision {

namespace {

using planner::common::kCellFree;
using planner::common::kCellOccupied;
using planner::common::kCellUnknown;
using planner::common::kPi;
using planner::common::normalize_angle_rad;

constexpr double kMaxAbsCurvature_inv_m = 1.0e6;  // 半径 >= 1 um
constexpr double kSincSeriesBound = 1e-8;
constexpr int kGridChunkCells = 8;
constexpr double kObstacleHashCell_m = 4.0;
constexpr double kEnvelopeSafetyFactor = 1.0 + 1e-12;
constexpr double kEnvelopeSafetyOffset_m = 1e-12;

// 稳定的 sinc：接近零时使用级数展开。
double sinc(double x) noexcept {
  if (std::abs(x) < kSincSeriesBound) {
    const double x2 = x * x;
    return 1.0 - x2 / 6.0 + x2 * x2 / 120.0;
  }
  return std::sin(x) / x;
}

// 从 `start` 出发、沿常曲率圆弧走过带符号弧长 `s` 后的位姿（倒车时为
// 负）。yaw_change = kappa * s。
Pose2d pose_on_arc(const Pose2d& start, double s, double kappa) noexcept {
  const double b = kappa * s;
  const double cy = std::cos(start.yaw_rad);
  const double sy = std::sin(start.yaw_rad);
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

// 凸多边形与轴对齐包围盒之间的距离（相交或相切时为 0）。
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
  if (convex_polygon_intersects_aabb(polygon, box, 0.0)) {
    return 0.0;
  }
  for (const Point2d& c : corners) {
    best = std::min(best, point_polygon_distance(c, polygon));
  }
  return best;
}

// 最多 8 个点的凸包（单调链），无内存分配。
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
  // 去除重复点。
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
  // chain 保存闭合凸包；去掉重复的末点。
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
  std::vector<int> obstacle_order;  // 按 aabb.min_x 排序（加速）
  double obstacle_scale_m{0.0};     // 障碍物包围盒的最大 |坐标|
  bool use_obstacle_order{false};

  int chunk_size{kGridChunkCells};
  int chunks_x{0};
  int chunks_y{0};
  std::vector<std::uint8_t> chunk_blocked;
  bool use_chunk_index{false};
};

namespace {

struct PoseHit {
  bool hit{false};
  CollisionReason reason{CollisionReason::kNone};
  std::string detail;
};

std::array<Point2d, 4> grid_corners(const CollisionCheckerImpl& st, const Pose2d& pose) {
  const std::array<Point2d, 4> world = st.footprint.world_corners(pose);
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

// 遍历可能位于 `query` 的 `reach` 范围内的障碍物索引。
// 加速只改变候选的枚举方式；两种模式下逐候选测试完全相同，因此判定
// 不会改变。
template <typename Fn>
void for_each_obstacle_near(const CollisionCheckerImpl& st, const Aabb2d& query, double reach,
                            Fn&& fn) {
  if (st.obstacle_boxes.empty()) {
    return;
  }
  const auto close_enough = [&](const Aabb2d& box) {
    return aabb_distance(box, query) <= reach;
  };
  if (st.use_obstacle_order) {
    for (const int idx : st.obstacle_order) {
      const Aabb2d& box = st.obstacle_boxes[static_cast<std::size_t>(idx)];
      if (box.min_x > query.max_x + reach) {
        break;  // 已按 min_x 排序
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
  const std::array<Point2d, 4> grid = grid_corners(st, pose);
  const Aabb2d world_box = aabb_of_raw(world.data(), world.size());
  const std::vector<Point2d> footprint_world(world.begin(), world.end());

  // 1) 闭地图矩形之外的一切区域均为占据（仅在存在栅格——因而存在地图
  // 矩形——时适用）。
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

  // 2) 多边形障碍物（世界坐标系）：最小欧氏距离在尺度相关容差内即接触
  // （接触计为碰撞）。
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
      if (convex_polygon_distance(footprint_world, obstacle.vertices) <= tol) {
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

  // 3) 被阻塞的栅格单元（原点坐标系，单元为轴对齐包围盒）。
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
    const double scale =
        std::max(std::max(std::abs(min_x), std::abs(max_x)),
                 std::max(std::abs(min_y), std::abs(max_y)));
    const double tol = contact_tolerance(scale);
    const std::vector<Point2d> footprint_grid_polygon(grid.begin(), grid.end());

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
      // 最小欧氏距离在容差内即接触（与独立参考实现的策略一致）。
      if (polygon_aabb_distance(footprint_grid_polygon, cell_box) > tol) {
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

    if (st.use_chunk_index && !st.chunk_blocked.empty()) {
      const int chunk = st.chunk_size;
      const int c0 = col0 / chunk;
      const int c1 = col1 / chunk;
      const int r0 = row0 / chunk;
      const int r1 = row1 / chunk;
      // 分块整体跳过，但行仍为外层循环，因此报告的首次阻塞单元与纯路径
      // 一致。
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
// 扫掠运动引擎
// ---------------------------------------------------------------------------

struct Envelope {
  std::vector<Point2d> hull;  // 世界坐标系，凸
  double sagitta_m{0.0};      // 角点圆弧偏离其弦的最大偏差
};

// 区间内扫掠足迹的保守覆盖：
//   hull(corners(s0), corners(s1)) 外扩角点圆弧矢高。
Envelope build_envelope(const CollisionCheckerImpl& st, const Pose2d& p0, const Pose2d& p1,
                        double delta_yaw, double kappa) {
  Envelope envelope;
  const std::array<Point2d, 4> c0 = st.footprint.world_corners(p0);
  const std::array<Point2d, 4> c1 = st.footprint.world_corners(p1);
  const double abs_yaw = std::abs(delta_yaw);
  if (abs_yaw >= kPi) {
    // 病态的多圈区间：回退到保守界“到弦的距离 <= 2 * 圆弧半径”，其中
    // 圆弧半径受 |1/kappa| + 最大角点半径约束。这会强制细分。
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

// 当包络可证明与所有阻塞区域（障碍物、占据/未知单元）不相交且可证明
// 位于地图内时返回 true。
bool envelope_provably_clear(const CollisionCheckerImpl& st, const Envelope& envelope) {
  if (envelope.hull.empty()) {
    return false;
  }
  const Aabb2d hull_box = aabb_of(envelope.hull);
  const double scale = hull_box.max_abs_coordinate() + 1.0;
  const double tol = contact_tolerance(scale);
  const double reach = envelope.sagitta_m + tol;
  // 障碍物可能位于比凸包更大的坐标处；其容差不得小于成对策略容差。
  const double obstacle_reach =
      envelope.sagitta_m +
      contact_tolerance(std::max(scale, st.obstacle_scale_m));

  // 栅格：地图矩形包含性加被阻塞单元。
  if (st.scene.has_grid()) {
    std::vector<Point2d> grid_hull;
    grid_hull.reserve(envelope.hull.size());
    for (const Point2d& p : envelope.hull) {
      grid_hull.push_back(st.frame.to_grid_frame(p));
    }
    const Aabb2d hull_grid_box = aabb_of(grid_hull);
    for (const Point2d& p : grid_hull) {
      // distance_to_boundary <= reach 同时覆盖“距边缘过近”与“位于地图
      // 之外”。
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
    bool clear = true;
    const auto test_cell = [&](int row, int col) {
      const int value =
          grid_view.cells[static_cast<std::size_t>(row) * grid_view.width + col];
      if (value == kCellFree) {
        return;
      }
      const Aabb2d cell_box{col * res, row * res, (col + 1) * res,
                            (row + 1) * res};
      if (aabb_distance(cell_box, hull_grid_box) > reach) {
        return;  // 安全过滤：aabb 距离 <= 多边形距离
      }
      if (polygon_aabb_distance(grid_hull, cell_box) <= reach) {
        clear = false;
      }
    };
    if (st.use_chunk_index && !st.chunk_blocked.empty()) {
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
    if (!clear) {
      return false;
    }
  }

  // 障碍物。
  bool clear = true;
  for_each_obstacle_near(st, hull_box, obstacle_reach, [&](int idx) {
    if (!clear) {
      return;
    }
    const Obstacle& obstacle = st.scene.obstacles[static_cast<std::size_t>(idx)];
    const double d = convex_polygon_distance(envelope.hull, obstacle.vertices);
    if (d <= obstacle_reach) {
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
    // 保守界：车体偏移处点的位移 <= |delta_yaw| * (|1/kappa| + 角点半径)。
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

// 解决区间 (s0, s1]；调用方保证 pose(s0) 无碰撞。返回首个碰撞（按遍历
// 顺序）、首个不确定区间，或 clear。
Decision resolve(const SweepContext& ctx, double s0, const Pose2d& p0,
                 double s1) {
  const CollisionCheckerImpl& st = *ctx.state;
  const Pose2d p1 = pose_on_arc(ctx.start, s1, ctx.kappa);
  const PoseHit hit1 = evaluate_pose(st, p1);
  const double delta_yaw = ctx.kappa * (s1 - s0);
  const bool at_floor =
      interval_displacement(ctx, p0, p1, delta_yaw) <= ctx.precision_m;

  if (hit1.hit) {
    // (s0, s1] 内存在碰撞。继续细化以报告最早出现的一个。
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
    const Pose2d pm = pose_on_arc(ctx.start, sm, ctx.kappa);
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
  const Pose2d pm = pose_on_arc(ctx.start, sm, ctx.kappa);
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
// 输入验证辅助函数
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
        continue;  // 重复点
      }
      CheckResult step = check_motion_impl(st, motion);
      if (step.collided()) {
        step.path_segment_index = static_cast<int>(seg);
        step.first_s_m = (points[i].s_m - base_s) + step.first_s_m;
        return step;
      }
    }
    // 始终精确检查存储的终点位姿，即使重建圆弧的终点与之略有偏差。
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
// 自由函数
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
