// 邻域策略与统一契约工具的正确性测试（无上游依赖）。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "planner/grid_search/neighborhood.hpp"
#include "planner/grid_search/types.hpp"

namespace planner::grid_search {
namespace {

common::GridView make_grid(int width, int height,
                           const std::vector<int>& cells) {
  common::GridView grid;
  grid.map_id = "unit";
  grid.map_version = "unit";
  grid.width = width;
  grid.height = height;
  grid.resolution_m = 1.0;
  grid.cells = cells;
  return grid;
}

// 6x6：中心 (2,2) 空闲；四周可单独设置。
common::GridView corner_grid(int occupied_a_row, int occupied_a_col,
                             int occupied_b_row, int occupied_b_col) {
  std::vector<int> cells(36, common::kCellFree);
  const auto index = [](int row, int col) { return row * 6 + col; };
  cells[index(occupied_a_row, occupied_a_col)] = common::kCellOccupied;
  cells[index(occupied_b_row, occupied_b_col)] = common::kCellOccupied;
  return make_grid(6, 6, cells);
}

TEST(Neighborhood, StepsMatchContract) {
  const auto& four = neighborhood_steps(Neighborhood::kFour);
  const auto& eight = neighborhood_steps(Neighborhood::kEight);
  ASSERT_EQ(four.size(), 4u);
  ASSERT_EQ(eight.size(), 8u);
  for (const NeighborStep& step : four) {
    EXPECT_FALSE(step.diagonal);
    EXPECT_DOUBLE_EQ(step.cost, 1.0);
  }
  int diagonal = 0;
  for (const NeighborStep& step : eight) {
    if (step.diagonal) {
      ++diagonal;
      EXPECT_DOUBLE_EQ(step.cost, std::sqrt(2.0));
    } else {
      EXPECT_DOUBLE_EQ(step.cost, 1.0);
    }
  }
  EXPECT_EQ(diagonal, 4);
}

TEST(Neighborhood, BlockedStatesAndOutside) {
  common::GridView grid = make_grid(4, 4, std::vector<int>(16, common::kCellFree));
  grid.cells[5] = common::kCellOccupied;   // (1,1)
  grid.cells[6] = common::kCellUnknown;    // (1,2)
  EXPECT_FALSE(cell_blocked(grid, 0, 0));
  EXPECT_TRUE(cell_blocked(grid, 1, 1));
  EXPECT_TRUE(cell_blocked(grid, 1, 2));
  EXPECT_TRUE(cell_blocked(grid, -1, 0));
  EXPECT_TRUE(cell_blocked(grid, 0, 4));
}

TEST(Neighborhood, DiagonalCornerCutIsForbidden) {
  // (2,3) 与 (3,2) 占据：从 (2,2) 到 (3,3) 的对角移动被禁止。
  const common::GridView grid = corner_grid(2, 3, 3, 2);
  const std::vector<GridCell> cells = neighbors(grid, 2, 2, Neighborhood::kEight);
  bool diagonal_to_goal = false;
  for (const GridCell& cell : cells) {
    if (cell.row == 3 && cell.col == 3) {
      diagonal_to_goal = true;
    }
  }
  EXPECT_FALSE(diagonal_to_goal);
  // 只阻塞一个正交邻格同样禁止。
  const common::GridView single = corner_grid(2, 3, 5, 5);
  const std::vector<GridCell> single_cells =
      neighbors(single, 2, 2, Neighborhood::kEight);
  for (const GridCell& cell : single_cells) {
    EXPECT_FALSE(cell.row == 3 && cell.col == 3);
  }
  // 四邻域不受影响（本来就没有对角步）：(1,2) 与 (2,1) 被占据后只剩 2 个。
  const std::vector<GridCell> four_cells =
      neighbors(grid, 2, 2, Neighborhood::kFour);
  EXPECT_EQ(four_cells.size(), 2u);
}

TEST(Neighborhood, DiagonalAllowedWhenBothOrthogonalsFree) {
  const common::GridView grid = corner_grid(5, 5, 5, 5);  // 无阻碍
  const std::vector<GridCell> cells = neighbors(grid, 2, 2, Neighborhood::kEight);
  bool diagonal_to_goal = false;
  for (const GridCell& cell : cells) {
    if (cell.row == 3 && cell.col == 3) {
      diagonal_to_goal = true;
    }
  }
  EXPECT_TRUE(diagonal_to_goal);
}

TEST(Neighborhood, BoundaryDoesNotWrap) {
  const common::GridView grid = make_grid(3, 3, std::vector<int>(9, common::kCellFree));
  const std::vector<GridCell> cells = neighbors(grid, 0, 0, Neighborhood::kEight);
  for (const GridCell& cell : cells) {
    EXPECT_GE(cell.row, 0);
    EXPECT_GE(cell.col, 0);
    EXPECT_LT(cell.row, 3);
    EXPECT_LT(cell.col, 3);
  }
  EXPECT_EQ(cells.size(), 3u);  // (0,1) (1,0) (1,1)
}

TEST(Types, NeighborhoodRoundTrip) {
  EXPECT_EQ(to_string(Neighborhood::kFour), "four");
  EXPECT_EQ(to_string(Neighborhood::kEight), "eight");
  EXPECT_EQ(neighborhood_from_string("four"), Neighborhood::kFour);
  EXPECT_EQ(neighborhood_from_string("eight"), Neighborhood::kEight);
  EXPECT_FALSE(neighborhood_from_string("octile").has_value());
}

TEST(Types, CostToleranceMatchesContract) {
  EXPECT_TRUE(costs_match(1.0, 1.0));
  EXPECT_TRUE(costs_match(42.1837661841 + 4.2e-7, 42.1837661841));
  EXPECT_FALSE(costs_match(42.1837661841 + 5e-7, 42.1837661841));
}

TEST(Types, PathCostUsesNeighborhoodStepCosts) {
  common::GridView grid =
      make_grid(4, 4, std::vector<int>(16, common::kCellFree));
  grid.resolution_m = 0.5;
  const std::vector<GridCell> path = {
      GridCell{0, 0}, GridCell{0, 1}, GridCell{1, 1}, GridCell{2, 2}};
  const double expected = (2.0 + std::sqrt(2.0)) * 0.5;
  EXPECT_DOUBLE_EQ(path_cost_m(grid, Neighborhood::kEight, path), expected);
}

TEST(Types, ValidatePathRejectsContractViolations) {
  common::GridView grid =
      make_grid(4, 4, std::vector<int>(16, common::kCellFree));
  grid.cells[1 * 4 + 1] = common::kCellOccupied;  // (1,1)
  GridSearchRequest request;
  request.grid = &grid;
  request.start = GridCell{0, 0};
  request.goal = GridCell{0, 2};
  request.neighborhood = Neighborhood::kEight;

  EXPECT_EQ(validate_path(grid, request,
                          {GridCell{0, 0}, GridCell{0, 1}, GridCell{0, 2}}),
            "");
  // 端点不匹配。
  EXPECT_NE(validate_path(grid, request,
                          {GridCell{0, 0}, GridCell{0, 2}}),
            "");
  // 经过阻塞单元。
  EXPECT_NE(validate_path(grid, request,
                          {GridCell{0, 0}, GridCell{1, 1}, GridCell{0, 2}}),
            "");
  // 对角穿角：路径 (0,1)->(1,2) 的对角步要求 (0,2) 与 (1,1) 均可通行，
  // 而 (1,1) 被占据。
  request.goal = GridCell{1, 2};
  const std::string problem = validate_path(
      grid, request, {GridCell{0, 0}, GridCell{0, 1}, GridCell{1, 2}});
  EXPECT_NE(problem.find("corner"), std::string::npos) << problem;
  // 四邻域请求不接受对角步。
  request.neighborhood = Neighborhood::kFour;
  EXPECT_NE(validate_path(grid, request,
                          {GridCell{0, 0}, GridCell{0, 1}, GridCell{1, 2}}),
            "");
}

TEST(Types, ValidateRequestRejectsBlockedAndOutOfRange) {
  common::GridView grid =
      make_grid(4, 4, std::vector<int>(16, common::kCellFree));
  grid.cells[0] = common::kCellOccupied;  // (0,0)
  grid.cells[5] = common::kCellUnknown;   // (1,1)
  GridSearchRequest request;
  request.grid = &grid;
  request.start = GridCell{1, 2};
  request.goal = GridCell{2, 2};
  std::string reason;
  EXPECT_EQ(validate_request(request, &reason), common::Status::kSuccess);

  request.start = GridCell{0, 0};
  EXPECT_EQ(validate_request(request, &reason), common::Status::kInvalidInput);
  request.start = GridCell{1, 2};
  request.goal = GridCell{1, 1};  // unknown 阻塞
  EXPECT_EQ(validate_request(request, &reason), common::Status::kInvalidInput);
  request.goal = GridCell{-1, 0};
  EXPECT_EQ(validate_request(request, &reason), common::Status::kInvalidInput);
  request.goal = GridCell{2, 2};
  request.time_budget_s = -1.0;
  EXPECT_EQ(validate_request(request, &reason), common::Status::kInvalidInput);
  request.time_budget_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validate_request(request, &reason), common::Status::kInvalidInput);
  request.time_budget_s.reset();
  request.node_budget = 0;
  EXPECT_EQ(validate_request(request, &reason), common::Status::kInvalidInput);
}

}  // namespace
}  // namespace planner::grid_search
