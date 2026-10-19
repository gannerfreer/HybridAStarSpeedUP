#pragma once

// 由 src/differential.cpp（regime/分配与宽范围随机生成）和
// src/differential_strata.cpp（集中对抗构造）共享的内部辅助函数。非公开
// 头文件。

#include <cstdint>
#include <string>
#include <vector>

#include "planner/collision/differential.hpp"

namespace planner::collision::differential::internal {

// 基于确定性 xorshift/splitmix 的 RNG。与 std::uniform_real_
// distribution 不同，其映射完全确定，因此用例生成在每个平台都可复现。
class Rng {
 public:
  explicit Rng(std::uint64_t seed) : state_(seed == 0 ? 0x9E3779B97F4A7C15ull
                                                      : seed) {}

  std::uint64_t next_u64() noexcept {
    // splitmix64
    std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }

  double uniform() noexcept {
    return static_cast<double>(next_u64() >> 11) * (1.0 / 9007199254740992.0);
  }

  double uniform(double lo, double hi) noexcept {
    return lo + (hi - lo) * uniform();
  }

  int uniform_int(int lo, int hi) noexcept {  // 含两端
    if (hi <= lo) return lo;
    const double u = uniform();
    const int value = lo + static_cast<int>(u * (hi - lo + 1));
    return value > hi ? hi : value;
  }

  bool coin(double probability = 0.5) noexcept { return uniform() < probability; }

  std::uint64_t state() const noexcept { return state_; }

 private:
  std::uint64_t state_;
};

std::uint64_t mixed_seed(int seed, int ordinal, int attempt) noexcept;

// 栅格原点位姿下的栅格坐标系 <-> 世界坐标转换。
Point2d grid_to_world(const GridFrame& frame, const Point2d& p) noexcept;
Pose2d grid_to_world_pose(const GridFrame& frame, const Pose2d& pose) noexcept;

// 稳定的常曲率积分（与检查器相同的约定）。
Pose2d pose_on_arc(const Pose2d& start, double s, double kappa) noexcept;

// 构建 GridView 文档（行主序单元），map_id/map_version 由用例 id 前缀
// 确定。
GridView make_grid(int width, int height, double resolution_m,
                   const Pose2d& origin, std::vector<int> cells,
                   const std::string& case_family);

// 将与障碍物多边形接触的每个单元标记为占据（接触计为碰撞），可选跳过
// 与 `protection` 相交的单元。
void rasterize_obstacle(std::vector<int>& cells, int width, int height,
                        double resolution_m,
                        const std::vector<Point2d>& polygon_grid,
                        const Aabb2d* protection, bool has_protection);

// 将块内的空闲单元标记为未知，跳过受保护单元。返回新标记的未知单元
// 数量。
std::size_t rasterize_unknown_block(std::vector<int>& cells, int width,
                                    int height, double resolution_m,
                                    const Aabb2d& block_grid,
                                    const Aabb2d* protection,
                                    bool has_protection);

// 矩形 / 随机凸包障碍物，世界尺寸在 [0.2, 8] m 内，用栅格坐标系表示，
// 完全位于 `map_box`（已收缩）内。
Obstacle random_obstacle(Rng& rng, const Aabb2d& map_box,
                         const std::string& id);

// 保护盒 = 给定世界点的 AABB 外扩 margin。
Aabb2d protection_box(const std::vector<Point2d>& points, double margin);

// `pose` 处的足迹到给定障碍物的最小距离（世界坐标系）。重叠时返回 0。
double clearance_to_obstacles(const Footprint& footprint, const Pose2d& pose,
                              const std::vector<Obstacle>& obstacles);

// `motion` 的扫掠足迹到给定障碍物的最小距离，采样时点的最大位移 <=
// 2.5e-4 m（粗扫描加局部细化）。任一样本处重叠即返回 0。
double sweep_min_clearance(const Footprint& footprint,
                           const MotionSegment& motion,
                           const std::vector<Obstacle>& obstacles);

// 集中构造；填充 `out` 的 scene/start/motion 字段。
void generate_concentrated(DifferentialCase& out, Rng& rng);

}  // namespace planner::collision::differential::internal
