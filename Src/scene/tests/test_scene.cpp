// planner_scene 单元测试：窗口推导、可行驶域基础、障碍叠加、预算降级与
// 稳定错误码。使用只读的 v3 mapping_example 夹具（与 W5 服务场景测试相同的
// 地图与投影原点），因此期望值可以直接对照迁移前的服务行为。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "planner/collision/scene.hpp"
#include "planner/map/map.hpp"
#include "planner/scene/scene.hpp"

#ifndef PLANNER_SCENE_TEST_MAP_PATH
#error "PLANNER_SCENE_TEST_MAP_PATH must be defined by the test CMakeLists"
#endif
#ifndef PLANNER_SCENE_TEST_ORIGIN_LAT
#error "PLANNER_SCENE_TEST_ORIGIN_LAT must be defined by the test CMakeLists"
#endif
#ifndef PLANNER_SCENE_TEST_ORIGIN_LON
#error "PLANNER_SCENE_TEST_ORIGIN_LON must be defined by the test CMakeLists"
#endif

namespace planner::scene {
namespace {

// 与 planner::service::default_vehicle 相同的 v1 默认测试车辆。
common::VehicleModel default_vehicle() {
  common::VehicleModel vehicle;
  vehicle.id = "default_test_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

class SceneTest : public ::testing::Test {
 protected:
  void SetUp() override {
    map::MapLoadOptions options;
    options.projection = map::ProjectionKind::kUtm;
    options.origin = map::GeoPoint{std::stod(PLANNER_SCENE_TEST_ORIGIN_LAT),
                                   std::stod(PLANNER_SCENE_TEST_ORIGIN_LON),
                                   0.0};
    auto result = map::Map::import_osm_file(PLANNER_SCENE_TEST_MAP_PATH, options);
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

TEST_F(SceneTest, BuildsDriveableWindowCoveringStartGoalAndObstacles) {
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

TEST_F(SceneTest, RasterizesObstaclesOnTopOfDriveableDomain) {
  const collision::Obstacle obstacle =
      collision::Obstacle::rectangle("obs-1", 9.0, 4.0, 11.0, 6.0);

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

TEST_F(SceneTest, RejectsInvalidObstacleWithStableCode) {
  // 反向矩形 → 顶点为空；栅格化跳过，碰撞场景校验失败。
  const collision::Obstacle invalid =
      collision::Obstacle::rectangle("bad", 1.0, 1.0, -1.0, -1.0);
  const SceneBuildResult scene = build_scene(
      *map_, default_vehicle(), common::Pose2d{2.0, 5.0, 0.0},
      common::Pose2d{18.0, 5.0, 0.0}, {invalid});
  EXPECT_FALSE(scene.ok);
  EXPECT_EQ(scene.reason, "invalid_scene");
  EXPECT_FALSE(scene.detail.empty());
}

TEST_F(SceneTest, RejectsOffRoadStartOrGoal) {
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

TEST_F(SceneTest, RejectsStartOrGoalOutsideMapBounds) {
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

TEST_F(SceneTest, RaisesResolutionWhenWindowExceedsCellBudget) {
  SceneBuildOptions options;
  options.max_cells = 10000;
  const SceneBuildResult scene = build_scene(
      *map_, default_vehicle(), common::Pose2d{2.0, 5.0, 0.0},
      common::Pose2d{18.0, 5.0, 0.0}, {}, options);
  ASSERT_TRUE(scene.ok) << scene.reason << " " << scene.detail;
  EXPECT_GT(scene.grid.resolution_m, 0.1);
  const std::size_t cells = static_cast<std::size_t>(scene.grid.width) *
                            static_cast<std::size_t>(scene.grid.height);
  EXPECT_LE(cells, options.max_cells);
}

}  // namespace
}  // namespace planner::scene
