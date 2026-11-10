#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "planner/common/grid_view.hpp"
#include "planner/math/angle.hpp"
#include "scene.hpp"

namespace {
using namespace planner;

TEST(GridView, InBoundsAndCellAccess) {
  common::GridView grid;
  EXPECT_FALSE(grid.in_bounds(0, 0));
  EXPECT_FALSE(grid.in_bounds(-1, 0));
  EXPECT_THROW((void)grid.cell(0, 0), std::out_of_range);

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
  EXPECT_THROW((void)grid.cell(2, 0), std::out_of_range);
  EXPECT_THROW((void)grid.cell(0, 3), std::out_of_range);
  EXPECT_THROW((void)grid.cell(-1, 0), std::out_of_range);
  EXPECT_THROW((void)grid.cell(0, -1), std::out_of_range);
}

TEST(GridView, TryCellBounds) {
  common::GridView grid;
  EXPECT_FALSE(grid.try_cell(0, 0).has_value());

  grid.width = 3;
  grid.height = 2;
  grid.cells = {0, 1, 2, 3, 4, 5};
  EXPECT_EQ(grid.try_cell(0, 0), std::optional<int>{0});
  EXPECT_EQ(grid.try_cell(1, 2), std::optional<int>{5});
  EXPECT_FALSE(grid.try_cell(-1, 0).has_value());
  EXPECT_FALSE(grid.try_cell(0, -1).has_value());
  EXPECT_FALSE(grid.try_cell(2, 0).has_value());
  EXPECT_FALSE(grid.try_cell(0, 3).has_value());
}

TEST(GridFrame, TransformsMatchOriginYaw) {
  common::GridView grid;
  grid.width = 4;
  grid.height = 2;
  grid.resolution_m = 0.5;
  grid.origin = {1.0, 2.0, math::kPi / 2.0};
  const common::GridFrame frame = common::GridFrame::from_grid(grid);
  EXPECT_DOUBLE_EQ(frame.resolution_m, 0.5);
  EXPECT_DOUBLE_EQ(frame.width_m, 2.0);
  EXPECT_DOUBLE_EQ(frame.height_m, 1.0);
  EXPECT_NEAR(frame.cos_yaw, 0.0, 1e-12);
  EXPECT_NEAR(frame.sin_yaw, 1.0, 1e-12);

  const common::Point2d local = frame.to_grid_frame({1.0, 2.5});
  EXPECT_NEAR(local.x_m, 0.5, 1e-12);
  EXPECT_NEAR(local.y_m, 0.0, 1e-12);
  EXPECT_TRUE(frame.inside_map(local));
  EXPECT_FALSE(frame.inside_map({-0.1, 0.0}));
  EXPECT_DOUBLE_EQ(frame.distance_to_boundary({0.5, 0.5}), 0.5);
  EXPECT_DOUBLE_EQ(frame.distance_to_boundary({-0.2, 0.5}), -0.2);
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
  valid_scene.grid.cells = {common::kCellFree, common::kCellOccupied, common::kCellUnknown,
                            common::kCellFree, common::kCellFree,     common::kCellFree};
  reason.clear();
  EXPECT_TRUE(valid_scene.valid(&reason)) << reason;
}

}  // namespace
