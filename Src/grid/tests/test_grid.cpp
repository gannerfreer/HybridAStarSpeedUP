// planner_grid 单元测试：坐标换算、窗口单元数/预算降级数学与栅格数据校验。
//
// 期望值全部手工推导（见各用例注释），不依赖显示或地图组件。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include "planner/common/types.hpp"
#include "planner/grid/grid.hpp"

namespace planner::grid {
namespace {

common::GridView make_grid(int width, int height, double resolution_m,
                           double origin_x, double origin_y, double yaw_rad) {
  common::GridView grid;
  grid.width = width;
  grid.height = height;
  grid.resolution_m = resolution_m;
  grid.origin = common::Pose2d{origin_x, origin_y, yaw_rad};
  grid.cells.assign(static_cast<std::size_t>(width) *
                        static_cast<std::size_t>(height),
                    common::kCellFree);
  return grid;
}

TEST(GridCellStateTest, AcceptsOnlyThreeStateValues) {
  EXPECT_TRUE(cell_state_valid(common::kCellFree));
  EXPECT_TRUE(cell_state_valid(common::kCellOccupied));
  EXPECT_TRUE(cell_state_valid(common::kCellUnknown));
  EXPECT_FALSE(cell_state_valid(-1));
  EXPECT_FALSE(cell_state_valid(3));
  EXPECT_FALSE(cell_state_valid(42));
}

TEST(GridValidGridTest, AcceptsEmptyAndWellFormedGrids) {
  std::string reason;
  common::GridView empty;
  EXPECT_TRUE(valid_grid(empty, &reason)) << reason;

  common::GridView grid = make_grid(2, 2, 0.5, 0.0, 0.0, 0.0);
  grid.cells = {common::kCellFree, common::kCellOccupied, common::kCellUnknown,
                common::kCellFree};
  EXPECT_TRUE(valid_grid(grid, &reason)) << reason;
}

TEST(GridValidGridTest, RejectsLengthMismatchAndBadValues) {
  std::string reason;
  common::GridView grid = make_grid(2, 2, 0.5, 0.0, 0.0, 0.0);
  grid.cells.pop_back();
  EXPECT_FALSE(valid_grid(grid, &reason));
  EXPECT_FALSE(reason.empty());

  grid = make_grid(2, 2, 0.5, 0.0, 0.0, 0.0);
  grid.cells[3] = 7;
  EXPECT_FALSE(valid_grid(grid, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(GridValidGridTest, RejectsBadShapeResolutionAndOrigin) {
  std::string reason;
  common::GridView grid = make_grid(2, 2, 0.5, 0.0, 0.0, 0.0);
  grid.width = 0;
  EXPECT_FALSE(valid_grid(grid, &reason));

  grid = make_grid(2, 2, 0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(valid_grid(grid, &reason));

  grid = make_grid(2, 2, 0.5, std::numeric_limits<double>::quiet_NaN(), 0.0,
                   0.0);
  EXPECT_FALSE(valid_grid(grid, &reason));
}

TEST(GridCeilCellCountTest, CeilSemanticsAndInvalidInput) {
  EXPECT_EQ(ceil_cell_count(1.0, 0.1), 10);
  EXPECT_EQ(ceil_cell_count(1.05, 0.1), 11);
  EXPECT_EQ(ceil_cell_count(0.05, 0.1), 1);
  EXPECT_EQ(ceil_cell_count(0.0, 0.1), 0);
  EXPECT_EQ(ceil_cell_count(-1.0, 0.1), 0);
  EXPECT_EQ(ceil_cell_count(1.0, 0.0), 0);
  EXPECT_EQ(ceil_cell_count(1.0, -0.1), 0);
  EXPECT_EQ(ceil_cell_count(std::numeric_limits<double>::infinity(), 0.1), 0);
  // 超出 int 可表示范围：返回 0，由调用方按预算拒绝。
  EXPECT_EQ(ceil_cell_count(1e300, 0.1), 0);
}

TEST(GridWindowSpecTest, ComputesCeilDimensionsAndCellCount) {
  const auto spec = window_grid_spec(10.0, 5.0, 0.1);
  ASSERT_TRUE(spec.has_value());
  EXPECT_EQ(spec->width, 100);
  EXPECT_EQ(spec->height, 50);
  EXPECT_EQ(spec->cells, 5000u);

  const auto ceil_spec = window_grid_spec(1.05, 0.05, 0.1);
  ASSERT_TRUE(ceil_spec.has_value());
  EXPECT_EQ(ceil_spec->width, 11);
  EXPECT_EQ(ceil_spec->height, 1);
  EXPECT_EQ(ceil_spec->cells, 11u);

  EXPECT_FALSE(window_grid_spec(10.0, 5.0, 0.0).has_value());
  EXPECT_FALSE(window_grid_spec(0.0, 5.0, 0.1).has_value());
  EXPECT_FALSE(window_grid_spec(1e300, 5.0, 0.1).has_value());
}

TEST(GridBudgetFitTest, DoublesResolutionUntilWithinBudget) {
  // 20 m x 20 m @ 0.1 m = 40000 个单元；预算 1000：
  // 0.2 -> 100x100=10000、0.4 -> 50x50=2500、0.8 -> 25x25=625 <= 1000。
  const CellBudgetFit fit =
      fit_resolution_to_cell_budget(20.0, 20.0, 0.1, 1000);
  EXPECT_TRUE(fit.within_budget);
  EXPECT_DOUBLE_EQ(fit.resolution_m, 0.8);
  EXPECT_EQ(fit.cells, 625u);

  const CellBudgetFit exact =
      fit_resolution_to_cell_budget(20.0, 20.0, 0.1, 40000);
  EXPECT_TRUE(exact.within_budget);
  EXPECT_DOUBLE_EQ(exact.resolution_m, 0.1);
  EXPECT_EQ(exact.cells, 40000u);
}

TEST(GridBudgetFitTest, ZeroBudgetBehavesAsOne) {
  const CellBudgetFit fit = fit_resolution_to_cell_budget(20.0, 20.0, 0.1, 0);
  EXPECT_TRUE(fit.within_budget);
  // 需要 ceil(20/res) <= 1，即 res >= 20：0.1 * 2^8 = 25.6。
  EXPECT_DOUBLE_EQ(fit.resolution_m, 25.6);
  EXPECT_EQ(fit.cells, 1u);
}

TEST(GridBudgetFitTest, ReportsFailureWhenGuardIsExhausted) {
  // 每边 1e9 m @ 1e-300：32 次翻倍后仍远超预算（且单元数溢出为 inf）。
  const CellBudgetFit fit =
      fit_resolution_to_cell_budget(1e9, 1e9, 1e-300, 16u * 1024u * 1024u);
  EXPECT_FALSE(fit.within_budget);
  EXPECT_DOUBLE_EQ(fit.resolution_m, std::ldexp(1e-300, 32));
}

TEST(GridBudgetFitTest, InvalidInputsAreNotWithinBudget) {
  const CellBudgetFit zero =
      fit_resolution_to_cell_budget(20.0, 20.0, 0.0, 1000);
  EXPECT_FALSE(zero.within_budget);
  EXPECT_DOUBLE_EQ(zero.resolution_m, 0.0);

  const CellBudgetFit negative =
      fit_resolution_to_cell_budget(-1.0, 20.0, 0.1, 1000);
  EXPECT_FALSE(negative.within_budget);
  EXPECT_DOUBLE_EQ(negative.resolution_m, 0.1);
}

TEST(GridWorldToCellTest, FloorSemanticsWithAxisAlignedOrigin) {
  const common::GridView grid = make_grid(4, 3, 0.5, 1.0, 2.0, 0.0);
  // (1.6, 2.7)：局部 (0.6, 0.7) -> col floor(1.2)=1、row floor(1.4)=1。
  const auto cell = world_to_cell(grid, 1.6, 2.7);
  ASSERT_TRUE(cell.has_value());
  EXPECT_EQ(cell->col, 1);
  EXPECT_EQ(cell->row, 1);

  // 边界点属于上侧单元：局部 (0.5, 0.5) -> col 1、row 1。
  const auto boundary = world_to_cell(grid, 1.5, 2.5);
  ASSERT_TRUE(boundary.has_value());
  EXPECT_EQ(boundary->col, 1);
  EXPECT_EQ(boundary->row, 1);

  // 栅格矩形之外允许负索引（调用方自行裁剪）。
  const auto outside = world_to_cell(grid, -1.0, -1.0);
  ASSERT_TRUE(outside.has_value());
  EXPECT_EQ(outside->col, -4);
  EXPECT_EQ(outside->row, -6);

  EXPECT_FALSE(world_to_cell(grid, std::numeric_limits<double>::quiet_NaN(), 2.0)
                   .has_value());
  EXPECT_FALSE(world_to_cell(make_grid(0, 0, 0.5, 0.0, 0.0, 0.0), 0.0, 0.0)
                   .has_value());
}

TEST(GridWorldToCellTest, RespectsOriginYaw) {
  const double half_pi = std::acos(-1.0) / 2.0;
  const common::GridView grid = make_grid(4, 4, 1.0, 0.0, 0.0, half_pi);
  // 世界 (0, 2)：局部 = R(-pi/2) * (0, 2) = (2, 0) -> col 2、row 0。
  const auto cell = world_to_cell(grid, 0.0, 2.0);
  ASSERT_TRUE(cell.has_value());
  EXPECT_EQ(cell->col, 2);
  EXPECT_EQ(cell->row, 0);

  // 单元中心往返：cell (0, 2) 中心 = R(pi/2) * (2.5, 0.5) = (-0.5, 2.5)。
  const common::Point2d center = cell_center(grid, 0, 2);
  EXPECT_NEAR(center.x_m, -0.5, 1e-12);
  EXPECT_NEAR(center.y_m, 2.5, 1e-12);
  const auto roundtrip = world_to_cell(grid, center.x_m, center.y_m);
  ASSERT_TRUE(roundtrip.has_value());
  EXPECT_EQ(roundtrip->col, 2);
  EXPECT_EQ(roundtrip->row, 0);
}

TEST(GridCellCenterTest, AxisAlignedCenter) {
  const common::GridView grid = make_grid(10, 10, 0.5, 1.0, 2.0, 0.0);
  const common::Point2d center = cell_center(grid, 3, 4);
  EXPECT_DOUBLE_EQ(center.x_m, 1.0 + 4.5 * 0.5);
  EXPECT_DOUBLE_EQ(center.y_m, 2.0 + 3.5 * 0.5);
}

TEST(GridWorldToCellClampedTest, ClampsToGridRange) {
  const common::GridView grid = make_grid(4, 3, 0.5, 0.0, 0.0, 0.0);
  const CellIndex far = world_to_cell_clamped(grid, 100.0, 100.0);
  EXPECT_EQ(far.col, 3);
  EXPECT_EQ(far.row, 2);
  const CellIndex low = world_to_cell_clamped(grid, -100.0, -100.0);
  EXPECT_EQ(low.col, 0);
  EXPECT_EQ(low.row, 0);
  // 不可映射输入（NaN）返回 {0,0}；调用方负责先做有限性校验。
  const CellIndex nan_cell =
      world_to_cell_clamped(grid, std::numeric_limits<double>::quiet_NaN(), 0.0);
  EXPECT_EQ(nan_cell.col, 0);
  EXPECT_EQ(nan_cell.row, 0);
}

TEST(GridCellIndexTest, EqualityOperators) {
  EXPECT_TRUE((CellIndex{1, 2}) == (CellIndex{1, 2}));
  EXPECT_TRUE((CellIndex{1, 2}) != (CellIndex{2, 1}));
}

}  // namespace
}  // namespace planner::grid
