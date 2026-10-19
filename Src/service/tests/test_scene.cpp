// 本地规划场景构建器的单元测试（不启动服务进程）。
//
// W5 增强：场景域基础为地图 lanelet 可行驶域（Map::export_driveable_grid），
// 请求障碍叠加为占据。测试使用只读的 v3 mapping_example 夹具验证
// "道路外单元被占据" 与 "起点/终点必须在可行驶域内"；窗口裁剪与分辨率
// 上调使用合成小地图（tests/data/window_map.osm）。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "map_fixture.hpp"
#include "planner/service/scene.hpp"
#include "planner/service/types.hpp"

namespace planner::service {
namespace {

class PlanSceneTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto result = test::load_example_map();
    ASSERT_TRUE(result.ok()) << result.reason;
    map_ = std::move(result.map);
  }

  static int cell_at(const common::GridView& grid, double x_m, double y_m) {
    const int col =
        static_cast<int>(std::floor((x_m - grid.origin.x_m) / grid.resolution_m));
    const int row =
        static_cast<int>(std::floor((y_m - grid.origin.y_m) / grid.resolution_m));
    return grid.cell(row, col);
  }

  std::unique_ptr<map::Map> map_;
};

class PlanSceneSmallMapTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto result = test::load_window_map();
    ASSERT_TRUE(result.ok()) << result.reason;
    map_ = std::move(result.map);
  }

  static int cell_at(const common::GridView& grid, double x_m, double y_m) {
    const int col =
        static_cast<int>(std::floor((x_m - grid.origin.x_m) / grid.resolution_m));
    const int row =
        static_cast<int>(std::floor((y_m - grid.origin.y_m) / grid.resolution_m));
    return grid.cell(row, col);
  }

  std::unique_ptr<map::Map> map_;
};

TEST_F(PlanSceneTest, BuildsDriveableWindowCoveringStartGoalAndObstacles) {
  const SceneBuildResult scene = build_scene(
      *map_, default_vehicle(), common::Pose2d{2.0, 5.0, 0.0},
      common::Pose2d{18.0, 5.0, 0.0}, {});
  ASSERT_TRUE(scene.ok) << scene.reason << " " << scene.detail;
  EXPECT_EQ(scene.grid.map_id, map_->info().map_id);
  EXPECT_EQ(scene.grid.map_version, map_->info().map_version);
  EXPECT_DOUBLE_EQ(scene.grid.resolution_m, 0.1);
  EXPECT_GT(scene.grid.width, 0);
  EXPECT_GT(scene.grid.height, 0);
  EXPECT_EQ(scene.grid.cells.size(),
            static_cast<std::size_t>(scene.grid.width) *
                static_cast<std::size_t>(scene.grid.height));
  // 起点/终点的后轴点位于网格矩形内。
  EXPECT_GE(2.0, scene.grid.origin.x_m);
  EXPECT_LE(18.0, scene.grid.origin.x_m + scene.grid.width * scene.grid.resolution_m);
  EXPECT_GE(5.0, scene.grid.origin.y_m);
  EXPECT_LE(5.0, scene.grid.origin.y_m + scene.grid.height * scene.grid.resolution_m);

  // 可行驶域基础：起点/终点/途经点在 lanelet 内 → free；
  // 道路外单元（距最近 lanelet >= 1.4 m，且在窗口内）→ occupied。
  EXPECT_EQ(cell_at(scene.grid, 2.0, 5.0), common::kCellFree);
  EXPECT_EQ(cell_at(scene.grid, 10.0, 5.0), common::kCellFree);
  EXPECT_EQ(cell_at(scene.grid, 18.0, 5.0), common::kCellFree);
  EXPECT_EQ(cell_at(scene.grid, 2.0, -3.5), common::kCellOccupied);
  EXPECT_EQ(cell_at(scene.grid, 25.0, 13.0), common::kCellOccupied);

  EXPECT_TRUE(scene.scene.obstacles.empty());
  EXPECT_TRUE(scene.scene.has_grid());
}

TEST_F(PlanSceneTest, RasterizesObstaclesOnTopOfDriveableDomain) {
  ServiceObstacle obstacle;
  obstacle.kind = ServiceObstacle::Kind::kRectangle;
  obstacle.id = "obs-1";
  obstacle.x_min_m = 9.0;
  obstacle.y_min_m = 4.0;
  obstacle.x_max_m = 11.0;
  obstacle.y_max_m = 6.0;

  const SceneBuildResult scene = build_scene(
      *map_, default_vehicle(), common::Pose2d{2.0, 5.0, 0.0},
      common::Pose2d{18.0, 5.0, 0.0}, {obstacle});
  ASSERT_TRUE(scene.ok) << scene.reason << " " << scene.detail;
  ASSERT_EQ(scene.scene.obstacles.size(), 1u);
  EXPECT_EQ(scene.scene.obstacles.front().id, "obs-1");

  // 障碍物叠加占据；可行驶域内的其余单元仍 free。
  EXPECT_EQ(cell_at(scene.grid, 10.0, 5.0), common::kCellOccupied);
  EXPECT_EQ(cell_at(scene.grid, 9.0 + 0.05, 4.0 + 0.05), common::kCellOccupied);
  EXPECT_EQ(cell_at(scene.grid, 2.0, 5.0), common::kCellFree);
  EXPECT_EQ(cell_at(scene.grid, 12.0, 5.0), common::kCellFree);
}

TEST_F(PlanSceneTest, RejectsOffRoadStartOrGoal) {
  // (5,-5) 与 (8,-4) 距最近 lanelet 约 5.2 m / 3.7 m，均在窗口内但不可行驶。
  SceneBuildResult scene = build_scene(
      *map_, default_vehicle(), common::Pose2d{5.0, -5.0, 0.0},
      common::Pose2d{18.0, 5.0, 0.0}, {});
  EXPECT_FALSE(scene.ok);
  EXPECT_EQ(scene.reason, "start_outside_driveable");

  scene = build_scene(*map_, default_vehicle(), common::Pose2d{2.0, 5.0, 0.0},
                      common::Pose2d{8.0, -4.0, 0.0}, {});
  EXPECT_FALSE(scene.ok);
  EXPECT_EQ(scene.reason, "goal_outside_driveable");
}

TEST_F(PlanSceneTest, RejectsStartOrGoalOutsideMapBounds) {
  SceneBuildResult scene = build_scene(
      *map_, default_vehicle(), common::Pose2d{10000.0, 5.0, 0.0},
      common::Pose2d{18.0, 5.0, 0.0}, {});
  EXPECT_FALSE(scene.ok);
  EXPECT_EQ(scene.reason, "start_outside_map");

  scene = build_scene(*map_, default_vehicle(), common::Pose2d{2.0, 5.0, 0.0},
                      common::Pose2d{18.0, 10000.0, 0.0}, {});
  EXPECT_FALSE(scene.ok);
  EXPECT_EQ(scene.reason, "goal_outside_map");
}

TEST_F(PlanSceneSmallMapTest, ClampsWindowToMapBoundsAndMarksOffRoadOccupied) {
  const map::Bounds2d& bounds = map_->info().bounds_m;
  const SceneBuildResult scene = build_scene(
      *map_, default_vehicle(), common::Pose2d{0.0, 5.0, 0.0},
      common::Pose2d{0.0, 35.0, 0.0}, {});
  ASSERT_TRUE(scene.ok) << scene.reason << " " << scene.detail;
  EXPECT_DOUBLE_EQ(scene.grid.resolution_m, 0.1);
  // 窗口被裁剪到地图边界（西/南/北三侧），东侧未被裁剪；栅格上界按 ceil
  // 维度最多超出窗口（即地图边界）不足一个单元。
  EXPECT_NEAR(scene.grid.origin.x_m, bounds.min_x_m, 1e-6);
  EXPECT_NEAR(scene.grid.origin.y_m, bounds.min_y_m, 1e-6);
  EXPECT_LE(scene.grid.origin.x_m + scene.grid.width * scene.grid.resolution_m,
            bounds.max_x_m + scene.grid.resolution_m + 1e-9);
  EXPECT_LE(scene.grid.origin.y_m + scene.grid.height * scene.grid.resolution_m,
            bounds.max_y_m + scene.grid.resolution_m + 1e-9);
  EXPECT_GE(scene.grid.origin.x_m + scene.grid.width * scene.grid.resolution_m,
            bounds.min_x_m);

  // 道路中心线（x≈0）free；道路外（x≈+5 m，仍在窗口内）occupied。
  EXPECT_EQ(cell_at(scene.grid, 0.0, 5.0), common::kCellFree);
  EXPECT_EQ(cell_at(scene.grid, 0.0, 35.0), common::kCellFree);
  EXPECT_EQ(cell_at(scene.grid, 5.0, 5.0), common::kCellOccupied);
}

TEST_F(PlanSceneSmallMapTest, RejectsOffRoadStart) {
  const SceneBuildResult scene = build_scene(
      *map_, default_vehicle(), common::Pose2d{5.0, 5.0, 0.0},
      common::Pose2d{0.0, 35.0, 0.0}, {});
  EXPECT_FALSE(scene.ok);
  EXPECT_EQ(scene.reason, "start_outside_driveable");
}

TEST_F(PlanSceneSmallMapTest, RaisesResolutionWhenWindowExceedsCellBudget) {
  SceneBuildOptions options;
  options.max_cells = 1000;
  const SceneBuildResult scene = build_scene(
      *map_, default_vehicle(), common::Pose2d{0.0, 5.0, 0.0},
      common::Pose2d{0.0, 35.0, 0.0}, {}, options);
  ASSERT_TRUE(scene.ok) << scene.reason << " " << scene.detail;
  EXPECT_GT(scene.grid.resolution_m, 0.1);
  const std::size_t cells = static_cast<std::size_t>(scene.grid.width) *
                            static_cast<std::size_t>(scene.grid.height);
  EXPECT_LE(cells, options.max_cells);
}

}  // namespace
}  // namespace planner::service
