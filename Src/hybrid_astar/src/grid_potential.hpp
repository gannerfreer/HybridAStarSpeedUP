/**
 * @file grid_potential.hpp
 * @brief 二维栅格地图上 8 连通 Dijkstra 势场计算的内部接口头文件。
 *
 * 基于目标栅格单元，在空闲单元（cells == kCellFree）上计算以欧氏距离为权重的最短路径距离场。
 * 被占用、未知以及地图外部的单元均被视为障碍阻挡。
 * 该势场与 Reeds-Shepp 解析连接展开结合使用，作为感知障碍物拓扑的下界启发式估计，
 * 并用作连通性过滤器，在搜索前置阶段快速检验目标的可达性。
 */

#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include "planner/common/grid_view.hpp"

namespace planner::hybrid_astar::internal {

/// 表示不可达栅格单元的无穷大势能值。
inline constexpr float kPotentialInfinity = std::numeric_limits<float>::infinity();

/**
 * @brief 二维势场距离图结构体，存储各栅格到目标单元的最短路径距离。
 */
struct GridPotential {
  int width{0};                   ///< 栅格宽度（列数）。
  int height{0};                  ///< 栅格高度（行数）。
  double resolution_m{0.0};       ///< 栅格像元分辨率（米，m）。
  std::vector<float> distance_m;  ///< 行主序存储的距离场（米，m）；不可达像元为无穷大（infinity）。
  bool aborted{false};            ///< 是否因超时或外部终止而提前中止计算。
};

/**
 * @brief 计算覆盖栅格内容、几何尺寸与目标单元坐标的 64 位哈希键。
 * @param[in] grid 包含栅格占据状态与尺寸信息的视图。
 * @param[in] goal_row 目标单元的行索引。
 * @param[in] goal_col 目标单元的列索引。
 * @return 64 位缓存哈希键。
 */
std::uint64_t grid_cache_key(const common::GridView& grid, int goal_row, int goal_col);

/**
 * @brief 从目标单元出发，在所有连通空闲像元上计算 8 连通 Dijkstra 势场。
 * @param[in] grid 栅格占据地图视图。
 * @param[in] goal_row 目标单元的行索引。
 * @param[in] goal_col 目标单元的列索引。
 * @param[in] deadline_exceeded 可选的超时检查回调，若超时返回 true 则提前中止计算。
 * @return 计算得到的势场数据；若被中止，结构体内的 aborted 标志置为 true。
 */
GridPotential compute_grid_potential(const common::GridView& grid, int goal_row,
                                     int goal_col,
                                     const std::function<bool()>& deadline_exceeded);

}  // namespace planner::hybrid_astar::internal
