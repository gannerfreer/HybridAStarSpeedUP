/**
 * @file collision_checker.cpp
 * @brief 精确位姿碰撞检测与自适应二分扫掠运动碰撞检测的实现文件。
 * @details
 * 算法原理概述：
 * - 位姿检测（Pose checking）：将车辆多边形轮廓与凸多边形障碍物进行相交测试（采用分离轴定理 SAT，
 *   严格将接触判定为碰撞）、地图矩形边界判定以及栅格被占用/未知像元碰撞测试。
 * - 运动检测（Motion checking）：递归区间自适应二分法。对于每个时间/弧长区间，车辆扫掠轮廓
 *   保守外包于首尾两端 8 个顶点构成的凸包并外扩圆弧矢高（Sagitta）。当数学上严格证明无碰撞时，
 *   剪枝跳过该区间；否则持续二分直至区间内最大位移 <= sweep_precision_m（默认 1e-3 m）。
 *   若达到精度下限仍未证伪碰撞，则保守报告为碰撞。
 */

#include "planner/math/kinematics/arc.hpp"
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

#include "planner/common/grid_view.hpp"
#include "planner/common/path.hpp"
#include "planner/common/vehicle.hpp"
#include "planner/math/aabb2d.hpp"
#include "planner/math/detail/geometry_utils.hpp"
#include "planner/math/detail/sat_cache.hpp"
#include "planner/math/polygon2d.hpp"
#include "planner/math/vec2d.hpp"

using planner::math::operator+;
using planner::math::operator-;
using planner::math::operator*;

namespace planner::collision {

namespace {

using planner::common::kCellFree;
using planner::common::kCellOccupied;
using planner::common::kCellUnknown;
using planner::math::kPi;
using planner::math::normalize_angle_rad;

constexpr double kMaxAbsCurvature_inv_m = 1.0e6;  ///< 最大绝对曲率限制（对应转弯半径 >= 1 um）
constexpr int kGridChunkCells = 8;                ///< 栅格分块加速的像元块尺寸

// 多分辨率前缀和金字塔层数：第 l 层的块边长为 2^l（1, 2, 4...）。
// 可通过宏 PLANNER_COLLISION_PREFIX_LEVELS 外部配置；设为 0 时禁用该加速优化。
#ifndef PLANNER_COLLISION_PREFIX_LEVELS
#define PLANNER_COLLISION_PREFIX_LEVELS 2
#endif
constexpr int kPrefixLevels =
    PLANNER_COLLISION_PREFIX_LEVELS > 0 ? PLANNER_COLLISION_PREFIX_LEVELS : 0;
constexpr double kObstacleHashCell_m = 4.0;
constexpr double kEnvelopeSafetyFactor = 1.0 + 1e-12;
constexpr double kEnvelopeSafetyOffset_m = 1e-12;

/// 检查位姿各坐标分量是否均为有限浮点数。
bool pose_finite(const Pose2d& pose) noexcept {
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

}  // namespace

// ---------------------------------------------------------------------------
// CollisionCheckerImpl
// ---------------------------------------------------------------------------

/// 碰撞检测器内部实现结构体，持有预计算几何索引与场景缓存
struct CollisionCheckerImpl {
  VehicleModel vehicle;
  Scene scene;
  CheckOptions options;
  Footprint footprint;
  common::GridFrame frame;
  bool valid{false};
  std::string invalid_reason;

  std::vector<math::Aabb2d> obstacle_boxes;
  std::vector<int> obstacle_order;  ///< 按 aabb.min_x 升序排列的障碍物索引，用于快速剪枝与遍历加速。
  double obstacle_scale_m{0.0};     ///< 所有障碍物包围盒顶点坐标的最大绝对值，用于计算数值容差。
  bool use_obstacle_order{false};

  int chunk_size{kGridChunkCells};
  int chunks_x{0};
  int chunks_y{0};
  std::vector<std::uint8_t> chunk_blocked;
  bool use_chunk_index{false};

  /// 非空闲栅格单元的行索引结构（独立于 enable_acceleration；
  /// 仅改变候选像元的遍历顺序，检测决策结果与全局全量扫描完全一致）：
  ///   * blocked_offsets[r] .. blocked_offsets[r+1] 记录第 r 行中所有被阻挡单元的列索引（按升序排列）；
  ///     总内存复杂度仅为 O(被阻挡单元数量)。
  bool blocked_index_valid{false};
  std::vector<int> blocked_offsets;
  std::vector<int> blocked_cols;

  /// 多分辨率块计数二维前缀和（“判空金字塔”）。
  /// 第 l 层的块边长为 block = 2^l；rows/cols 为块级栅格尺寸；
  /// sum 为大小为 (rows+1) x (cols+1) 的积分图，每个元素记录对应区域内被阻挡/未知像元（即 cells != kCellFree）的数量。
  /// 任意矩形区域的被阻挡像元计数均可在 O(1) 时间内通过 4 次查表完成。
  struct PrefixLevel {
    int block{1};
    int rows{0};
    int cols{0};
    std::vector<std::uint32_t> sum;
  };
  std::vector<PrefixLevel> prefix_levels;
};

namespace {

/// 构建跨所有层级的多分辨率判空前缀和金字塔。
/// 利用原位 2D 积分图，支持以 O(1) 复杂度快速验证包围盒内是否完全处于空闲状态。
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
    // 原位 2D 前缀和：sum[r][c] 表示区间 [0..r-1, 0..c-1] 内的阻挡像元总计数
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

/// 检查闭区间像元范围 [row0, row1] x [col0, col1] 内是否不含任何被占用或未知单元。
/// 该范围首先向外扩充至所在块边界并截断至栅格内；若 4 点查表的 2D 前缀和为 0，
/// 则该外包超集保证不含任何阻挡像元。空范围或非法范围视为无障碍。
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

/// 从粗到细依次测试金字塔各层；若任意层级证明该包围盒不含被阻挡单元，则返回 true。
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

/// 按严格的行主序（Row-major）枚举 [row0, row1] x [col0, col1] 范围内的被阻挡像元。
/// 空闲像元在 test_cell 中会立即跳过，因此跳过它们绝不会改变检测决策。
/// 遍历顺序与全扫描顺序一致（行递增、列递增），当 fn(row, col) 返回 false 时提前终止。
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

/// 遍历所有可能位于 query 轴对齐包围盒 reach 距离内的障碍物索引。
/// 加速模式下通过基于 min_x 的排序快速进行范围剪枝，遍历剪枝完全不影响候选判定，保证检测决策与全局扫描完全一致。
template <typename Fn>
void for_each_obstacle_near(const CollisionCheckerImpl& st, const math::Aabb2d& query, double reach,
                            Fn&& fn) {
  if (st.obstacle_boxes.empty()) {
    return;
  }
  const auto close_enough = [&](const math::Aabb2d& box) {
    return math::aabb_within(box, query, reach);
  };
  if (st.use_obstacle_order) {
    for (const int idx : st.obstacle_order) {
      const math::Aabb2d& box = st.obstacle_boxes[static_cast<std::size_t>(idx)];
      if (box.min_x > query.max_x + reach) {
        break;  // 按 min_x 升序排列，后续必定超界
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
  const std::array<Point2d, 4> world = st.footprint.world_box(pose).corners();
  // 复用世界坐标系顶点：直接变换至栅格坐标系，避免重复调用昂贵的三角函数。
  const std::array<Point2d, 4> grid = grid_corners_from_world(st, world);
  const math::Aabb2d world_box = math::aabb_of_raw(world.data(), world.size());

  // 1) 地图边界外判定：车辆轮廓中若有任何点落在闭合地图矩形外部，则直接判定为碰撞。
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

  // 2) 凸多边形障碍物碰撞检测（世界坐标系）：当两多边形间的最小欧氏距离 <= 依赖于几何尺度的接触容差时，判定为碰撞。
  if (!st.obstacle_boxes.empty()) {
    const double footprint_scale = world_box.max_abs_coordinate();
    const double filter_tol =
        math::contact_tolerance(std::max(footprint_scale, st.obstacle_scale_m)) * 2.0;
    PoseHit obstacle_hit;
    for_each_obstacle_near(st, world_box, filter_tol, [&](int idx) {
      if (obstacle_hit.hit) {
        return;
      }
      const Obstacle& obstacle = st.scene.obstacles[static_cast<std::size_t>(idx)];
      const double tol =
          math::contact_tolerance(std::max(footprint_scale,
                                     obstacle.aabb().max_abs_coordinate()));
      // O(1) 粗筛：若分离轴投影间隙大于容差 tol，则两多边形最小欧氏距离 d >= gap > tol，
      // 两者绝无接触可能，可安全跳过精确距离计算。
      if (!math::convex_polygons_intersect(world.data(), world.size(),
                                           obstacle.data(), obstacle.size(),
                                           tol)) {
        return;
      }
      if (math::convex_polygon_distance_sq(world.data(), world.size(),
                                           obstacle.data(), obstacle.size()) <=
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

  // 3) 被阻挡栅格单元碰撞检测（栅格坐标系，每个单元视为轴对齐包围盒 AABB）。
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
    // 多分辨率前缀和快速判空：若任意层级证明该包围盒不含被阻挡单元，则无需扫描直接判定通过。
    if (prefix_pyramid_empty(st, row0, row1, col0, col1)) {
      return PoseHit{};
    }
    const double scale =
        std::max(std::max(std::abs(min_x), std::abs(max_x)),
                 std::max(std::abs(min_y), std::abs(max_y)));
    const double tol = math::contact_tolerance(scale);
    // 栅格单元分离轴定理（SAT）预计算缓存：在 evaluate_pose 内部车辆多边形保持固定，
    // 故包围盒与边法向量仅计算一次，在各个候选单元间复用。
    math::internal::PolygonSatCache8 cell_sat;
    cell_sat.build(grid.data(), grid.size());
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
      const math::Aabb2d cell_box{col * res, row * res, (col + 1) * res,
                            (row + 1) * res};
      // O(1) 粗筛：若分离轴投影间隙超过容差，则最小距离必大于 tol，两者不可能发生接触。
      if (!cell_sat.maybe_intersects(cell_box, tol)) {
        return;
      }
      // 在平方距离域执行精确欧氏距离检验。
      if (math::polygon_aabb_distance_sq(grid.data(), grid.size(), cell_box) >
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
      // 精确候选枚举：按行主序仅迭代被阻挡单元。
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
      // 跳过空闲块；维持行主序遍历顺序。
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
// 扫掠运动碰撞检测引擎
// ---------------------------------------------------------------------------

struct Envelope {
  std::array<Point2d, 8> hull{};  ///< 世界坐标系下的外包凸包顶点（固定容量栈存储）。
  std::size_t hull_size{0};       ///< 凸包有效顶点数量。
  double sagitta_m{0.0};          ///< 各顶点圆弧运动相对于弦的最大矢高（Sagitta）偏差。
};

/// 构建区间内车辆扫掠轮廓的保守外包络（Envelope）：
/// 由首尾两端顶点凸包 hull(corners(s0), corners(s1)) 并在径向外扩各顶点圆弧运动产生的最大矢高（Sagitta）。
Envelope build_envelope(const CollisionCheckerImpl& st, const Pose2d& p0, const Pose2d& p1,
                        double delta_yaw, double kappa) {
  Envelope envelope;
  const std::array<Point2d, 4> c0 = st.footprint.world_corners(p0);
  const std::array<Point2d, 4> c1 = st.footprint.world_corners(p1);
  const double abs_yaw = std::abs(delta_yaw);
  if (abs_yaw >= kPi) {
    // 病态多圈旋转区间：回退至保守上界“到弦的距离 <= 2 * 旋转半径”，迫使算法进行进一步二分。
    const double arc_radius =
        (kappa != 0.0 ? std::abs(1.0 / kappa) : 0.0) +
        st.footprint.corner_radius_m;
    envelope.sagitta_m = 2.0 * arc_radius * kEnvelopeSafetyFactor +
                         kEnvelopeSafetyOffset_m;
  } else if (delta_yaw != 0.0) {
    const double tan_quarter = std::abs(std::tan(0.25 * delta_yaw));
    for (std::size_t i = 0; i < 4; ++i) {
      const double chord = math::distance(c0[i], c1[i]);
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
  envelope.hull_size =
      math::convex_hull_small(points, n, envelope.hull);
  return envelope;
}

/// 判定扫掠包络是否严格与所有障碍物及阻挡栅格单元无干涉，且完全位于地图矩形边界之内。
bool envelope_provably_clear(const CollisionCheckerImpl& st, const Envelope& envelope) {
  if (envelope.hull_size == 0) {
    return false;
  }
  const math::Aabb2d hull_box =
      math::aabb_of_raw(envelope.hull.data(), envelope.hull_size);
  const double scale = hull_box.max_abs_coordinate() + 1.0;
  const double tol = math::contact_tolerance(scale);
  const double reach = envelope.sagitta_m + tol;
  // 障碍物可能位于较大坐标处；数值容差必须与双边距离策略严格匹配
  const double obstacle_reach =
      envelope.sagitta_m +
      math::contact_tolerance(std::max(scale, st.obstacle_scale_m));

  // 栅格检测：地图边界包含性与阻挡像元相交性
  if (st.scene.has_grid()) {
    std::array<Point2d, 8> grid_hull{};
    std::size_t grid_hull_size = 0;
    for (std::size_t i = 0; i < envelope.hull_size; ++i) {
      grid_hull[grid_hull_size++] = st.frame.to_grid_frame(envelope.hull[i]);
    }
    const math::Aabb2d hull_grid_box =
        math::aabb_of_raw(grid_hull.data(), grid_hull_size);
    for (std::size_t i = 0; i < grid_hull_size; ++i) {
      // distance_to_boundary <= reach 同时涵盖了靠近地图边界与出界两种情形
      if (st.frame.distance_to_boundary(grid_hull[i]) <= reach) {
        return false;
      }
    }
    const GridView& grid_view = st.scene.grid;
    const double res = st.frame.resolution_m;
    math::Aabb2d query_box = hull_grid_box;
    query_box.expand(reach);
    int col0 = static_cast<int>(std::floor(query_box.min_x / res));
    int col1 = static_cast<int>(std::ceil(query_box.max_x / res));
    int row0 = static_cast<int>(std::floor(query_box.min_y / res));
    int row1 = static_cast<int>(std::ceil(query_box.max_y / res));
    col0 = std::max(col0, 0);
    row0 = std::max(row0, 0);
    col1 = std::min(col1, grid_view.width);
    row1 = std::min(row1, grid_view.height);
    // 在闭区间像元范围 [row0, row1-1] x [col0, col1-1] 上进行多分辨率前缀和快速判空。
    // 若任意层级证明不存在被阻挡单元，则可安全跳过像元细筛扫描。
    const bool grid_empty =
        prefix_pyramid_empty(st, row0, row1 - 1, col0, col1 - 1);
    bool clear = true;
    // 预先为外包凸包构建 SAT 分离轴缓存
    math::internal::PolygonSatCache8 hull_sat;
    hull_sat.build(grid_hull.data(), grid_hull_size);
    const auto test_cell = [&](int row, int col) {
      const int value =
          grid_view.cells[static_cast<std::size_t>(row) * grid_view.width + col];
      if (value == kCellFree) {
        return;
      }
      const math::Aabb2d cell_box{col * res, row * res, (col + 1) * res,
                            (row + 1) * res};
      if (!math::aabb_within(cell_box, hull_grid_box, reach)) {
        return;  // AABB 包围盒粗筛：aabb_dist <= polygon_dist
      }
      // O(1) 粗筛：若分离轴投影间隙 > reach，则最小几何距离必大于 reach
      if (!hull_sat.maybe_intersects(cell_box, reach)) {
        return;
      }
      if (math::polygon_aabb_distance_sq(grid_hull.data(), grid_hull_size,
                                         cell_box) <= reach * reach) {
        clear = false;
      }
    };
    if (!grid_empty) {
      if (st.blocked_index_valid) {
        // 精确候选枚举：按行主序仅迭代被阻挡单元
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

  // 凸多边形障碍物碰撞检测
  bool clear = true;
  for_each_obstacle_near(st, hull_box, obstacle_reach, [&](int idx) {
    if (!clear) {
      return;
    }
    const Obstacle& obstacle = st.scene.obstacles[static_cast<std::size_t>(idx)];
    // O(1) SAT 粗筛：当分离轴间隙大于 obstacle_reach 时跳过精确多边形距离计算
    if (!math::convex_polygons_intersect(envelope.hull.data(), envelope.hull_size,
                                         obstacle.data(), obstacle.size(),
                                         obstacle_reach)) {
      return;
    }
    const double d_sq = math::convex_polygon_distance_sq(
        envelope.hull.data(), envelope.hull_size, obstacle.data(),
        obstacle.size());
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
  // 起始航向角的正弦和余弦值（在扫掠检测全程保持恒定；预先计算以避免在递归二分中重复调用三角函数）
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
    max_chord = std::max(max_chord, math::distance(c0[i], c1[i]));
  }
  const double abs_yaw = std::abs(delta_yaw);
  if (abs_yaw >= kPi) {
    // 保守上界：弧长位移 <= |delta_yaw| * (|1/kappa| + corner_radius)
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

/// 求解扫掠区间 (s0, s1] 内的碰撞状态；调用方保证起始位姿 pose(s0) 严格无碰撞。
/// 按轨迹遍历顺序返回首个确定的碰撞、首个达到精度下限的不确定区间，或全部安全通过。
Decision resolve(const SweepContext& ctx, double s0, const Pose2d& p0,
                 double s1) {
  const CollisionCheckerImpl& st = *ctx.state;
  const Pose2d p1 =
      math::integrate_arc_trig(ctx.start, s1, ctx.kappa, ctx.cos_yaw, ctx.sin_yaw);
  const PoseHit hit1 = evaluate_pose(st, p1);
  const double delta_yaw = ctx.kappa * (s1 - s0);
  const bool at_floor =
      interval_displacement(ctx, p0, p1, delta_yaw) <= ctx.precision_m;

  if (hit1.hit) {
    // 在区间 (s0, s1] 中检测到碰撞，继续二分以报告最早发生的碰撞点
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
        math::integrate_arc_trig(ctx.start, sm, ctx.kappa, ctx.cos_yaw, ctx.sin_yaw);
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
      math::integrate_arc_trig(ctx.start, sm, ctx.kappa, ctx.cos_yaw, ctx.sin_yaw);
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
// 输入合法性校验辅助函数
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
        continue;  // 重复航向点，直接跳过
      }
      CheckResult step = check_motion_impl(st, motion);
      if (step.collided()) {
        step.path_segment_index = static_cast<int>(seg);
        step.first_s_m = (points[i].s_m - base_s) + step.first_s_m;
        return step;
      }
    }
    // 始终显式检测存储的终点位姿，消除圆弧积分重建带来的微小漂移。
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
    state->frame = common::GridFrame::from_grid(scene.grid);
  }

  if (state->valid) {
    state->obstacle_boxes.reserve(state->scene.obstacles.size());
    for (const Obstacle& obstacle : state->scene.obstacles) {
      state->obstacle_boxes.push_back(obstacle.aabb());
      state->obstacle_scale_m =
          std::max(state->obstacle_scale_m,
                   state->obstacle_boxes.back().max_abs_coordinate());
    }
    if (scene.has_grid()) {
      // 构建非空闲栅格单元的精确行索引结构（始终构建；独立于 enable_acceleration，
      // 仅改变候选枚举顺序，决策结果与全扫描完全一致）。
      // 两遍扫描构建：第一遍统计每行数量以确定偏移量，第二遍按升序填充列索引。
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
      // 若阻挡单元总数超过内存阈值（8M 项 = 32 MB），则回退至常规扫描。
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
// 独立接口函数
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
