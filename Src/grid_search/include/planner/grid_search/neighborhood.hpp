#pragma once

// 邻域策略（四/八邻域 + 禁对角穿角 + unknown/地图外阻塞）的共享实现。
//
// 规则（冻结契约，所有后端一致）：
//   * 四邻域：正交四步，步代价 1；
//   * 八邻域：正交四步 + 对角四步，对角步代价 sqrt(2)；
//   * 八邻域禁止对角穿角：对角移动 (dr,dc) 要求 (row, col+dc) 与 (row+dr, col)
//     两个正交邻格都不阻塞；
//   * 阻塞 = 占据(1)、未知(2) 或地图外。

#include <vector>

#include "planner/common/types.hpp"
#include "planner/grid_search/types.hpp"

namespace planner::grid_search {

// 单元是否按阻塞处理（占据、未知、地图外）。
bool cell_blocked(const common::GridView& grid, int row, int col) noexcept;

// 单个邻域步长。`cost` 为单元步长（正交 1、对角 sqrt(2)）。
struct NeighborStep {
  int d_row{0};
  int d_col{0};
  double cost{1.0};
  bool diagonal{false};
};

// 邻域步长表（固定顺序：正交在前、对角在后，保证确定性）。
const std::vector<NeighborStep>& neighborhood_steps(Neighborhood neighborhood);

// 对角穿角检查：仅对角步有意义；正交步恒为 true。
bool diagonal_corner_allowed(const common::GridView& grid, int row, int col,
                             const NeighborStep& step) noexcept;

// (row,col) 的可行邻居（在界内、不阻塞、符合禁穿角规则）。
std::vector<GridCell> neighbors(const common::GridView& grid, int row, int col,
                                Neighborhood neighborhood);

}  // namespace planner::grid_search
