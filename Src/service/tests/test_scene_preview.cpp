// X-M1 单元测试：planner::scene 预览入口与 build_scene 的跨入口一致性
// （协议 §3.3/§5；P11 库级验证），以及窗口裁剪、预算与稳定错误码。
// 不启动服务进程；使用只读 v3 地图夹具。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "map_fixture.hpp"
#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"
#include "planner/map/map.hpp"
#include "planner/scene/content_id.hpp"
#include "planner/scene/scene.hpp"
#include "planner/service/scene.hpp"
#include "planner/service/types.hpp"

namespace planner::service {
namespace {

using planner::collision::Obstacle;
using planner::common::Point2d;
using planner::common::Pose2d;
using planner::map::Bounds2d;

// 与场景库相同的"单元中心位于闭合凸多边形内即占据"规则（独立复算用）。
bool center_inside(const std::vector<Point2d>& polygon, const Point2d& point) {
  const std::size_t count = polygon.size();
  if (count < 3U) {
    return false;
  }
  double signed_area = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    signed_area += polygon[i].x_m * polygon[(i + 1) % count].y_m -
                   polygon[(i + 1) % count].x_m * polygon[i].y_m;
  }
  const bool ccw = signed_area >= 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const Point2d& a = polygon[i];
    const Point2d& b = polygon[(i + 1) % count];
    const double cross =
        (b.x_m - a.x_m) * (point.y_m - a.y_m) - (b.y_m - a.y_m) * (point.x_m - a.x_m);
    if (ccw ? cross < 0.0 : cross > 0.0) {
      return false;
    }
  }
  return true;
}

class ScenePreviewTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto result = test::load_example_map();
    ASSERT_TRUE(result.ok()) << result.reason;
    map_ = std::move(result.map);
  }

  std::unique_ptr<map::Map> map_;
};

TEST_F(ScenePreviewTest, BuildsExplicitWindowWithSceneId) {
  scene::ScenePreviewOptions options;
  options.window = Bounds2d{0.0, 0.0, 6.0, 6.0};
  options.resolution_m = 0.1;
  const std::vector<Obstacle> obstacles{
      Obstacle::rectangle("obs-1", 1.0, 1.0, 2.0, 2.0)};

  const scene::SceneBuildResult preview =
      scene::build_scene_preview(*map_, obstacles, options);
  ASSERT_TRUE(preview.ok) << preview.reason << " " << preview.detail;
  EXPECT_EQ(preview.grid.width, 60);
  EXPECT_EQ(preview.grid.height, 60);
  EXPECT_DOUBLE_EQ(preview.grid.resolution_m, 0.1);
  EXPECT_DOUBLE_EQ(preview.grid.origin.x_m, 0.0);
  EXPECT_DOUBLE_EQ(preview.grid.origin.y_m, 0.0);
  EXPECT_DOUBLE_EQ(preview.requested_resolution_m, 0.1);
  EXPECT_DOUBLE_EQ(preview.effective_resolution_m, 0.1);
  EXPECT_EQ(preview.scene_version, "scene_v1");
  EXPECT_EQ(preview.scene_id.size(), 64U);
  EXPECT_EQ(preview.scene_id,
            scene::scene_content_id(map_->info().map_id, map_->info().map_version,
                                    preview.window, preview.effective_resolution_m,
                                    preview.grid.width, preview.grid.height,
                                    preview.grid.origin, preview.scene.obstacles));
  // 障碍单元中心 (1.5, 1.5) 位于闭合矩形内 → 占据。
  const int col = static_cast<int>(std::floor(1.5 / 0.1));
  const int row = static_cast<int>(std::floor(1.5 / 0.1));
  EXPECT_EQ(preview.grid.cell(row, col), common::kCellOccupied);
}

TEST_F(ScenePreviewTest, MatchesV6P11FrozenSceneIdAndIndependentCells) {
  // v6 夹具 P11：窗口 (0,0)-(20,20)、0.1 m、obs-a 矩形 + obs-b 三角形。
  scene::ScenePreviewOptions options;
  options.window = Bounds2d{0.0, 0.0, 20.0, 20.0};
  options.resolution_m = 0.1;
  const std::vector<Obstacle> obstacles{
      Obstacle::rectangle("obs-a", 2.0, 2.0, 4.0, 4.0),
      Obstacle::convex_polygon("obs-b", {Point2d{10.0, 10.0}, Point2d{12.0, 10.0},
                                         Point2d{11.0, 13.0}})};

  const scene::SceneBuildResult preview =
      scene::build_scene_preview(*map_, obstacles, options);
  ASSERT_TRUE(preview.ok) << preview.reason << " " << preview.detail;
  EXPECT_EQ(preview.grid.width, 200);
  EXPECT_EQ(preview.grid.height, 200);
  EXPECT_EQ(preview.scene_id,
            "044007e558d42fbe4ff7cafc2a239834cf75ceefc6fbc464425233e799ee22d8");

  // 独立复算：可行驶域直接导出 + 手写中心点包含规则叠加。
  const map::DriveableGridResult base =
      map_->export_driveable_grid(preview.window, preview.effective_resolution_m);
  ASSERT_TRUE(base.ok()) << base.reason;
  ASSERT_EQ(base.grid.cells.size(), preview.grid.cells.size());
  std::size_t mismatches = 0;
  for (int row = 0; row < preview.grid.height; ++row) {
    for (int col = 0; col < preview.grid.width; ++col) {
      int expected = base.grid.cell(row, col);
      const Point2d center{preview.grid.origin.x_m + (col + 0.5) * preview.grid.resolution_m,
                           preview.grid.origin.y_m + (row + 0.5) * preview.grid.resolution_m};
      for (const Obstacle& obstacle : obstacles) {
        if (center_inside(obstacle.vertices, center)) {
          expected = common::kCellOccupied;
        }
      }
      if (preview.grid.cell(row, col) != expected) {
        ++mismatches;
      }
    }
  }
  EXPECT_EQ(mismatches, 0u);
}

TEST_F(ScenePreviewTest, PreviewAndPlanSceneAreCellIdenticalForSameEffectiveInput) {
  // P11 核心（库级）：同一地图版本/有效窗口/分辨率/障碍/规则下，
  // build_scene（规划）与 build_scene_preview（预览）逐格一致、标识相等。
  const std::vector<Obstacle> obstacles{
      Obstacle::rectangle("obs-1", 9.0, 4.0, 11.0, 6.0)};
  const scene::SceneBuildResult plan =
      scene::build_scene(*map_, default_vehicle(), Pose2d{2.0, 5.0, 0.0},
                         Pose2d{18.0, 5.0, 0.0}, obstacles);
  ASSERT_TRUE(plan.ok) << plan.reason << " " << plan.detail;

  scene::ScenePreviewOptions options;
  options.window = plan.window;
  options.resolution_m = plan.effective_resolution_m;
  const scene::SceneBuildResult preview =
      scene::build_scene_preview(*map_, obstacles, options);
  ASSERT_TRUE(preview.ok) << preview.reason << " " << preview.detail;

  EXPECT_EQ(preview.scene_id, plan.scene_id);
  EXPECT_EQ(preview.scene_version, plan.scene_version);
  EXPECT_EQ(preview.grid.cells, plan.grid.cells);
  EXPECT_EQ(preview.grid.width, plan.grid.width);
  EXPECT_EQ(preview.grid.height, plan.grid.height);
  EXPECT_DOUBLE_EQ(preview.grid.resolution_m, plan.grid.resolution_m);
  EXPECT_EQ(preview.grid.origin.x_m, plan.grid.origin.x_m);
  EXPECT_EQ(preview.grid.origin.y_m, plan.grid.origin.y_m);
  EXPECT_EQ(preview.grid.origin.yaw_rad, plan.grid.origin.yaw_rad);
  EXPECT_EQ(preview.window.min_x_m, plan.window.min_x_m);
  EXPECT_EQ(preview.window.max_y_m, plan.window.max_y_m);
}

TEST_F(ScenePreviewTest, ClipsRequestedWindowToMapBounds) {
  const map::Bounds2d& bounds = map_->info().bounds_m;
  scene::ScenePreviewOptions options;
  options.window = Bounds2d{bounds.min_x_m - 500.0, bounds.min_y_m - 500.0, 5.0, 5.0};
  options.resolution_m = 0.1;
  const scene::SceneBuildResult preview = scene::build_scene_preview(*map_, {}, options);
  ASSERT_TRUE(preview.ok) << preview.reason << " " << preview.detail;
  // 服务回显量化后的有效窗口（协议 §2.4/§5.1，步长 1e-9）。
  EXPECT_NEAR(preview.window.min_x_m, bounds.min_x_m, 1e-9);
  EXPECT_NEAR(preview.window.min_y_m, bounds.min_y_m, 1e-9);
  EXPECT_DOUBLE_EQ(preview.window.max_x_m, 5.0);
  EXPECT_DOUBLE_EQ(preview.window.max_y_m, 5.0);
  EXPECT_DOUBLE_EQ(preview.grid.origin.x_m, preview.window.min_x_m);
  EXPECT_DOUBLE_EQ(preview.grid.origin.y_m, preview.window.min_y_m);
}

TEST_F(ScenePreviewTest, QuantizesWindowAndResolution) {
  scene::ScenePreviewOptions options;
  // 低于 1e-9 的差异被量化归并（协议 §5.1 规则 4）。
  options.window = Bounds2d{0.0, 0.0, 6.0000000004, 6.0};
  options.resolution_m = 0.1000000004;
  const scene::SceneBuildResult preview = scene::build_scene_preview(*map_, {}, options);
  ASSERT_TRUE(preview.ok) << preview.reason << " " << preview.detail;
  EXPECT_EQ(preview.window.max_x_m, 6.0);
  EXPECT_EQ(preview.effective_resolution_m, 0.1);
  EXPECT_EQ(preview.requested_resolution_m, 0.1);

  scene::ScenePreviewOptions baseline;
  baseline.window = Bounds2d{0.0, 0.0, 6.0, 6.0};
  baseline.resolution_m = 0.1;
  const scene::SceneBuildResult reference =
      scene::build_scene_preview(*map_, {}, baseline);
  ASSERT_TRUE(reference.ok);
  EXPECT_EQ(preview.scene_id, reference.scene_id);
}

TEST_F(ScenePreviewTest, RejectsInvalidWindowsAndBudgetAndObstacles) {
  scene::ScenePreviewOptions options;
  options.window = Bounds2d{0.0, 0.0, 6.0, 6.0};

  // 窗口在地图外（与地图边界不相交）。
  scene::ScenePreviewOptions outside = options;
  outside.window = Bounds2d{1e6, 1e6, 1e6 + 10.0, 1e6 + 10.0};
  scene::SceneBuildResult preview = scene::build_scene_preview(*map_, {}, outside);
  EXPECT_FALSE(preview.ok);
  EXPECT_EQ(preview.reason, "window_outside_map");

  // 空窗口。
  scene::ScenePreviewOptions empty = options;
  empty.window = Bounds2d{0.0, 0.0, 0.0, 1.0};
  preview = scene::build_scene_preview(*map_, {}, empty);
  EXPECT_FALSE(preview.ok);
  EXPECT_EQ(preview.reason, "empty_window");

  // 非有限窗口。
  scene::ScenePreviewOptions non_finite = options;
  non_finite.window = Bounds2d{0.0, 0.0, std::numeric_limits<double>::infinity(), 1.0};
  preview = scene::build_scene_preview(*map_, {}, non_finite);
  EXPECT_FALSE(preview.ok);
  EXPECT_EQ(preview.reason, "invalid_window");

  // 预算耗尽：极小分辨率 + 1 格预算 → 32 次倍增后仍超限（guard 耗尽）。
  // 端点路径在小地图上因窗口先裁剪到地图边界而不可达（协议 §3.3 语义 1、
  // fixtures_v6_plan §8 待核实 3），此处覆盖同一 fit 路径。
  scene::ScenePreviewOptions tiny_budget = options;
  tiny_budget.resolution_m = 1e-9;
  tiny_budget.max_cells = 1;
  preview = scene::build_scene_preview(*map_, {}, tiny_budget);
  EXPECT_FALSE(preview.ok);
  EXPECT_EQ(preview.reason, "window_too_large");

  // 退化矩形（collision 语义：顶点为空 → invalid_scene）。
  const std::vector<Obstacle> invalid{
      Obstacle::rectangle("bad", 1.0, 1.0, -1.0, -1.0)};
  preview = scene::build_scene_preview(*map_, invalid, options);
  EXPECT_FALSE(preview.ok);
  EXPECT_EQ(preview.reason, "invalid_scene");

  // 非凸多边形。
  const std::vector<Obstacle> non_convex{Obstacle::convex_polygon(
      "bad-poly", {Point2d{0.0, 0.0}, Point2d{2.0, 0.0}, Point2d{1.0, 0.5},
                   Point2d{2.0, 2.0}})};
  preview = scene::build_scene_preview(*map_, non_convex, options);
  EXPECT_FALSE(preview.ok);
  EXPECT_EQ(preview.reason, "invalid_scene");
}

TEST_F(ScenePreviewTest, BuildSceneResultCarriesSceneIdForPlanService) {
  const scene::SceneBuildResult plan = scene::build_scene(
      *map_, default_vehicle(), Pose2d{2.0, 5.0, 0.0}, Pose2d{18.0, 5.0, 0.0}, {});
  ASSERT_TRUE(plan.ok) << plan.reason << " " << plan.detail;
  EXPECT_EQ(plan.scene_version, "scene_v1");
  EXPECT_EQ(plan.scene_id.size(), 64U);
  EXPECT_EQ(plan.requested_resolution_m, 0.1);
  EXPECT_GT(plan.window.max_x_m - plan.window.min_x_m, 0.0);
  EXPECT_EQ(plan.scene_id,
            scene::scene_content_id(map_->info().map_id, map_->info().map_version,
                                    plan.window, plan.effective_resolution_m,
                                    plan.grid.width, plan.grid.height, plan.grid.origin,
                                    plan.scene.obstacles));
}

}  // namespace
}  // namespace planner::service
