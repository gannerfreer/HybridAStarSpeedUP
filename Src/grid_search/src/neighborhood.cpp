// 邻域策略实现（四/八邻域、禁对角穿角、unknown/地图外阻塞）。

#include "planner/grid_search/neighborhood.hpp"

#include <cmath>

namespace planner::grid_search {
namespace {

const std::vector<NeighborStep>& four_steps() {
  static const std::vector<NeighborStep> steps = {
      {0, 1, 1.0, false},
      {1, 0, 1.0, false},
      {0, -1, 1.0, false},
      {-1, 0, 1.0, false},
  };
  return steps;
}

const std::vector<NeighborStep>& eight_steps() {
  static const std::vector<NeighborStep> steps = {
      {0, 1, 1.0, false},
      {1, 0, 1.0, false},
      {0, -1, 1.0, false},
      {-1, 0, 1.0, false},
      {-1, 1, std::sqrt(2.0), true},
      {-1, -1, std::sqrt(2.0), true},
      {1, 1, std::sqrt(2.0), true},
      {1, -1, std::sqrt(2.0), true},
  };
  return steps;
}

}  // namespace

bool cell_blocked(const common::GridView& grid, int row, int col) noexcept {
  if (!grid.in_bounds(row, col)) {
    return true;
  }
  return grid.cell(row, col) != common::kCellFree;
}

const std::vector<NeighborStep>& neighborhood_steps(
    Neighborhood neighborhood) {
  return neighborhood == Neighborhood::kFour ? four_steps() : eight_steps();
}

bool diagonal_corner_allowed(const common::GridView& grid, int row, int col,
                             const NeighborStep& step) noexcept {
  if (!step.diagonal) {
    return true;
  }
  return !cell_blocked(grid, row, col + step.d_col) &&
         !cell_blocked(grid, row + step.d_row, col);
}

std::vector<GridCell> neighbors(const common::GridView& grid, int row, int col,
                                Neighborhood neighborhood) {
  std::vector<GridCell> result;
  for (const NeighborStep& step : neighborhood_steps(neighborhood)) {
    const int next_row = row + step.d_row;
    const int next_col = col + step.d_col;
    if (cell_blocked(grid, next_row, next_col)) {
      continue;
    }
    if (!diagonal_corner_allowed(grid, row, col, step)) {
      continue;
    }
    result.push_back(GridCell{next_row, next_col});
  }
  return result;
}

}  // namespace planner::grid_search
