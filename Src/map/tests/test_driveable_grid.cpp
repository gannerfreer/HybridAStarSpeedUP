// Map::export_driveable_grid 的单元测试（W5 服务接入）：
// 已知 lanelet 内部单元 free、道路外单元 occupied、栅格约定、确定性、
// 参数校验与窗口边界情形。
//
// 使用只读的 v3 mapping_example 夹具（test_support.hpp 的固定原点），
// 不写入任何夹具文件。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace planner::map {
namespace {

class DriveableGridTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto result = test::load_mapping_example();
    ASSERT_TRUE(result.ok()) << result.reason;
    map_ = std::move(result.map);
  }

  // 单元 (row, col) 索引：GridView 约定，单元覆盖 [origin + c*res, ...]。
  static int row_of(const common::GridView& grid, double y_m) {
    return static_cast<int>(std::floor((y_m - grid.origin.y_m) / grid.resolution_m));
  }
  static int col_of(const common::GridView& grid, double x_m) {
    return static_cast<int>(std::floor((x_m - grid.origin.x_m) / grid.resolution_m));
  }

  static std::uint64_t fnv1a(const std::vector<int>& cells) {
    std::uint64_t hash = 1469598103934665603ull;
    for (int cell : cells) {
      hash ^= static_cast<std::uint64_t>(cell & 0xff);
      hash *= 1099511628211ull;
    }
    return hash;
  }

  std::unique_ptr<Map> map_;
};

TEST_F(DriveableGridTest, KnownLaneletInteriorIsFreeAndRoadOutsideIsOccupied) {
  // 窗口覆盖 (2,5)-(18,5) 路段及其周边空地。
  const Bounds2d window{-5.0, -5.0, 25.0, 15.0};
  const DriveableGridResult result = map_->export_driveable_grid(window, 0.1);
  ASSERT_TRUE(result.ok()) << result.reason;
  const common::GridView& grid = result.grid;

  // 栅格身份与约定：origin 为窗口最小角、yaw 0、ceil 维度、行主序。
  EXPECT_EQ(grid.map_id, map_->info().map_id);
  EXPECT_EQ(grid.map_version, map_->info().map_version);
  EXPECT_DOUBLE_EQ(grid.resolution_m, 0.1);
  EXPECT_DOUBLE_EQ(grid.origin.x_m, -5.0);
  EXPECT_DOUBLE_EQ(grid.origin.y_m, -5.0);
  EXPECT_DOUBLE_EQ(grid.origin.yaw_rad, 0.0);
  EXPECT_EQ(grid.width, 300);
  EXPECT_EQ(grid.height, 200);
  EXPECT_EQ(grid.cells.size(), 60000u);

  // 已知 lanelet 覆盖点：单元 free（(2,5)/(10,5)/(18,5) 分别位于
  // lanelet 3670769534662493708 / 7711382928694550045 / 2981562299451081503）。
  EXPECT_EQ(grid.cell(row_of(grid, 5.0), col_of(grid, 2.0)), common::kCellFree);
  EXPECT_EQ(grid.cell(row_of(grid, 5.0), col_of(grid, 10.0)), common::kCellFree);
  EXPECT_EQ(grid.cell(row_of(grid, 5.0), col_of(grid, 18.0)), common::kCellFree);

  // 道路外点（距最近 lanelet >= 1.7 m）：单元 occupied。
  EXPECT_EQ(grid.cell(row_of(grid, -4.0), col_of(grid, -4.0)), common::kCellOccupied);
  EXPECT_EQ(grid.cell(row_of(grid, -4.0), col_of(grid, 8.0)), common::kCellOccupied);
  EXPECT_EQ(grid.cell(row_of(grid, 14.0), col_of(grid, 24.0)), common::kCellOccupied);

  // 导出从不产生未知单元，且 free/occupied 两类都出现。
  bool saw_free = false;
  bool saw_occupied = false;
  for (int cell : grid.cells) {
    ASSERT_TRUE(cell == common::kCellFree || cell == common::kCellOccupied);
    saw_free = saw_free || cell == common::kCellFree;
    saw_occupied = saw_occupied || cell == common::kCellOccupied;
  }
  EXPECT_TRUE(saw_free);
  EXPECT_TRUE(saw_occupied);
}

TEST_F(DriveableGridTest, ExportIsDeterministicForIdenticalInput) {
  const Bounds2d window{-5.0, -5.0, 25.0, 15.0};
  const DriveableGridResult first = map_->export_driveable_grid(window, 0.1);
  const DriveableGridResult second = map_->export_driveable_grid(window, 0.1);
  ASSERT_TRUE(first.ok()) << first.reason;
  ASSERT_TRUE(second.ok()) << second.reason;
  EXPECT_EQ(first.grid, second.grid);
  EXPECT_EQ(fnv1a(first.grid.cells), fnv1a(second.grid.cells));
}

TEST_F(DriveableGridTest, RejectsInvalidResolution) {
  const Bounds2d window{0.0, 0.0, 10.0, 10.0};
  const double invalid[] = {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()};
  for (double resolution : invalid) {
    const DriveableGridResult result = map_->export_driveable_grid(window, resolution);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.reason, "invalid_resolution");
    EXPECT_TRUE(result.grid.cells.empty());
    EXPECT_EQ(result.grid.width, 0);
  }
}

TEST_F(DriveableGridTest, RejectsInvalidAndEmptyWindow) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  DriveableGridResult result =
      map_->export_driveable_grid(Bounds2d{0.0, 0.0, nan, 10.0}, 0.1);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "invalid_window");

  result = map_->export_driveable_grid(Bounds2d{0.0, 0.0, 10.0,
                                                std::numeric_limits<double>::infinity()},
                                       0.1);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "invalid_window");

  // 空窗口：零面积与反向边界都拒绝。
  result = map_->export_driveable_grid(Bounds2d{5.0, 0.0, 5.0, 10.0}, 0.1);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "empty_window");
  result = map_->export_driveable_grid(Bounds2d{0.0, 10.0, 10.0, 0.0}, 0.1);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "empty_window");
}

TEST_F(DriveableGridTest, WindowOutsideMapIsFullyOccupied) {
  // 窗口与所有车道不相交：合法结果，全占据，不报错。
  const DriveableGridResult result =
      map_->export_driveable_grid(Bounds2d{10000.0, 10000.0, 10010.0, 10010.0}, 0.1);
  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_EQ(result.grid.width, 100);
  EXPECT_EQ(result.grid.height, 100);
  for (int cell : result.grid.cells) {
    EXPECT_EQ(cell, common::kCellOccupied);
  }
}

TEST_F(DriveableGridTest, TinyWindowProducesSingleCell) {
  // 极小窗口：仍产生 1x1 栅格；单元内容按闭集相交语义判定。
  const DriveableGridResult on_road =
      map_->export_driveable_grid(Bounds2d{1.95, 4.95, 2.0, 5.0}, 0.1);
  ASSERT_TRUE(on_road.ok()) << on_road.reason;
  EXPECT_EQ(on_road.grid.width, 1);
  EXPECT_EQ(on_road.grid.height, 1);
  EXPECT_EQ(on_road.grid.cells.front(), common::kCellFree);

  const DriveableGridResult off_road =
      map_->export_driveable_grid(Bounds2d{7.95, -4.05, 8.0, -4.0}, 0.1);
  ASSERT_TRUE(off_road.ok()) << off_road.reason;
  EXPECT_EQ(off_road.grid.width, 1);
  EXPECT_EQ(off_road.grid.height, 1);
  EXPECT_EQ(off_road.grid.cells.front(), common::kCellOccupied);
}

TEST_F(DriveableGridTest, RejectsOversizedGridBeforeAllocation) {
  // 1000 m x 1000 m @ 0.001 m = 1e12 单元 > 16M 上限。
  const DriveableGridResult result =
      map_->export_driveable_grid(Bounds2d{0.0, 0.0, 1000.0, 1000.0}, 0.001);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "grid_too_large");
  EXPECT_TRUE(result.grid.cells.empty());
}

TEST_F(DriveableGridTest, PartiallyOverlappingWindowClipsAtWindowEdge) {
  // 窗口只覆盖路段的一部分：边界单元仍按闭集相交判定，
  // 窗口之外的区域不参与（尺寸只由窗口决定）。
  const Bounds2d window{1.0, 4.0, 3.0, 6.0};
  const DriveableGridResult result = map_->export_driveable_grid(window, 0.05);
  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_EQ(result.grid.width, 40);
  EXPECT_EQ(result.grid.height, 40);
  EXPECT_DOUBLE_EQ(result.grid.origin.x_m, 1.0);
  EXPECT_DOUBLE_EQ(result.grid.origin.y_m, 4.0);
  EXPECT_EQ(result.grid.cell(row_of(result.grid, 5.0), col_of(result.grid, 2.0)),
            common::kCellFree);
}

}  // namespace
}  // namespace planner::map
