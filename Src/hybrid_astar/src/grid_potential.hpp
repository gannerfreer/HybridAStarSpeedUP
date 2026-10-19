#pragma once

// 内部使用（不安装）：GridView 空闲单元上的目标势场。
//
// 该势场是仅在值为 kCellFree 的单元上（占据、未知和地图外单元均被阻塞）
// 计算、单元中心之间 8 连通的 Dijkstra 距离。它用于两个目的：
//   * 感知障碍物的启发式项（与 RS 距离组合），以及
//   * 可靠的连通性松弛：整车路径意味着后轴轨迹严格位于封闭占据单元之外，
//     因此在该单元图中不连通的起点/目标对，碰撞检测器也不可能连通。
//     松弛模型包含的自由空间从不超过碰撞组件。
//
// 与 Smac 的动态规划障碍启发式类似，该单元中心度量只是引导，并非连续
// 意义下的可采纳性证明；RS 项是严格下界，搜索保持完备。

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include "planner/common/types.hpp"

namespace planner::hybrid_astar::internal {

inline constexpr float kPotentialInfinity = std::numeric_limits<float>::infinity();

struct GridPotential {
  int width{0};
  int height{0};
  double resolution_m{0.0};
  std::vector<float> distance_m;  // 每单元一个值，行主序；infinity = 不可达
  bool aborted{false};            // 计算期间触达截止时间
};

// 覆盖栅格标识、几何、所有单元值以及目标单元的缓存键。两个相等的键描述
// 同一个势场。
std::uint64_t grid_cache_key(const common::GridView& grid, int goal_row, int goal_col);

// 计算势场；`deadline_exceeded` 可以为空。截止时间触发时结果带有
// `aborted=true`，不得缓存或使用。
GridPotential compute_grid_potential(const common::GridView& grid, int goal_row,
                                     int goal_col,
                                     const std::function<bool()>& deadline_exceeded);

}  // namespace planner::hybrid_astar::internal
