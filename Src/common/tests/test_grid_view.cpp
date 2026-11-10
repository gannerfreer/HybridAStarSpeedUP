#include "scene.hpp"
#include "types.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace planner;

TEST(GridView, InBoundsAndCellAccess) {
  common::GridView grid;
  EXPECT_FALSE(grid.in_bounds(0, 0));
  EXPECT_FALSE(grid.in_bounds(-1, 0));
  EXPECT_THROW(grid.cell(0, 0), std::out_of_range);

  grid.width = 3;
  grid.height = 2;
  grid.resolution_m = 0.5;
  grid.cells = {0, 1, 2, 3, 4, 5};
  EXPECT_TRUE(grid.in_bounds(0, 0));
  EXPECT_TRUE(grid.in_bounds(1, 2));
  EXPECT_FALSE(grid.in_bounds(-1, 0));
  EXPECT_FALSE(grid.in_bounds(0, -1));
  EXPECT_FALSE(grid.in_bounds(2, 0));
  EXPECT_FALSE(grid.in_bounds(0, 3));
  EXPECT_EQ(grid.cell(0, 0), 0);
  EXPECT_EQ(grid.cell(0, 2), 2);
  EXPECT_EQ(grid.cell(1, 0), 3);
  EXPECT_EQ(grid.cell(1, 2), 5);
  EXPECT_THROW(grid.cell(2, 0), std::out_of_range);
  EXPECT_THROW(grid.cell(0, 3), std::out_of_range);
  EXPECT_THROW(grid.cell(-1, 0), std::out_of_range);
  EXPECT_THROW(grid.cell(0, -1), std::out_of_range);
}

TEST(GridView, EmptyAndMismatchedCellsBehavior) {
  collision::Scene empty_scene;
  std::string reason;
  EXPECT_TRUE(empty_scene.valid(&reason)) << reason;

  collision::Scene missing_cells;
  missing_cells.grid.width = 2;
  missing_cells.grid.height = 3;
  missing_cells.grid.resolution_m = 0.5;
  reason.clear();
  EXPECT_FALSE(missing_cells.valid(&reason));
  EXPECT_NE(reason.find("does not match"), std::string::npos);

  collision::Scene extra_cells = missing_cells;
  extra_cells.grid.cells.assign(7, common::kCellFree);
  reason.clear();
  EXPECT_FALSE(extra_cells.valid(&reason));
  EXPECT_NE(reason.find("does not match"), std::string::npos);

  collision::Scene valid_scene;
  valid_scene.grid.width = 2;
  valid_scene.grid.height = 3;
  valid_scene.grid.resolution_m = 0.5;
  valid_scene.grid.cells = {common::kCellFree,   common::kCellOccupied,
                            common::kCellUnknown, common::kCellFree,
                            common::kCellFree,   common::kCellFree};
  reason.clear();
  EXPECT_TRUE(valid_scene.valid(&reason)) << reason;
}

}  // namespace
