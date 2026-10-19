// X-M1 单元测试：规划输入快照 LRU 存储、scene_id 规范序列化冻结值、
// 定点格式化与 grid_snapshot 组装（不启动服务进程）。

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"
#include "planner/scene/content_id.hpp"
#include "planner/scene/snapshot.hpp"
#include "planner/service/snapshot_store.hpp"

namespace planner::service {
namespace {

constexpr const char* kExampleMapVersion =
    "11561c763ed50322185139ec2dd041d6da72e1bd4e5fc3bb66f6b340d92c088f";

// ---------------------------------------------------------------------------
// LRU 存储（协议 §3.4：容量 16、按 scene_id 去重、LRU 淘汰）。
// ---------------------------------------------------------------------------

TEST(SceneSnapshotStore, DefaultCapacityMatchesProtocol) {
  SceneSnapshotStore store;
  EXPECT_EQ(store.capacity(), 16u);
  EXPECT_EQ(kDefaultSceneSnapshotCapacity, 16u);
  EXPECT_EQ(store.size(), 0u);
}

TEST(SceneSnapshotStore, DeduplicatesBySceneIdAndRefreshesContent) {
  SceneSnapshotStore store;
  EXPECT_TRUE(store.put("a", "first"));
  EXPECT_FALSE(store.put("a", "second"));
  EXPECT_EQ(store.size(), 1u);
  ASSERT_TRUE(store.get("a").has_value());
  EXPECT_EQ(*store.get("a"), "second");
}

TEST(SceneSnapshotStore, EvictsLeastRecentlyUsed) {
  SceneSnapshotStore store(3);
  store.put("a", "A");
  store.put("b", "B");
  store.put("c", "C");
  store.put("d", "D");  // a 被淘汰
  EXPECT_EQ(store.size(), 3u);
  EXPECT_FALSE(store.get("a").has_value());
  EXPECT_TRUE(store.get("b").has_value());
  EXPECT_TRUE(store.get("d").has_value());
}

TEST(SceneSnapshotStore, GetRefreshesRecency) {
  SceneSnapshotStore store(3);
  store.put("a", "A");
  store.put("b", "B");
  store.put("c", "C");
  ASSERT_TRUE(store.get("a").has_value());  // a 变为最近使用
  store.put("d", "D");                      // b 被淘汰
  EXPECT_TRUE(store.get("a").has_value());
  EXPECT_FALSE(store.get("b").has_value());
  EXPECT_TRUE(store.get("c").has_value());
  EXPECT_TRUE(store.get("d").has_value());
}

TEST(SceneSnapshotStore, SeventeenDistinctScenesEvictOldest) {
  SceneSnapshotStore store;  // 容量 16
  for (int i = 0; i < 17; ++i) {
    store.put("scene-" + std::to_string(i), "snapshot-" + std::to_string(i));
  }
  EXPECT_EQ(store.size(), 16u);
  EXPECT_FALSE(store.get("scene-0").has_value());
  EXPECT_TRUE(store.get("scene-1").has_value());
  EXPECT_TRUE(store.get("scene-16").has_value());
}

TEST(SceneSnapshotStore, UnknownSceneAndZeroCapacity) {
  SceneSnapshotStore store(0);
  EXPECT_FALSE(store.put("a", "A"));
  EXPECT_FALSE(store.get("a").has_value());
  SceneSnapshotStore normal;
  EXPECT_FALSE(normal.get("missing").has_value());
}

TEST(SceneSnapshotStore, SnapshotCellCapMatchesOnlineAcceptanceRange) {
  // 在线验收 ≤1024²；容量档（2048²/4096²）不进快照（扩展计划 §3.2/§6.1）。
  EXPECT_EQ(kMaxSnapshotCells, 1024u * 1024u);
  EXPECT_LT(kMaxSnapshotCells, 2048u * 2048u);
}

// ---------------------------------------------------------------------------
// scene_id 规范序列化（协议 §5；与 v6 夹具冻结值逐字节一致）。
// ---------------------------------------------------------------------------

TEST(SceneContentId, MatchesProtocolExampleHash) {
  const std::string id = scene::scene_content_id(
      "mapping_example", kExampleMapVersion, map::Bounds2d{0.0, 0.0, 0.6, 0.4}, 0.2, 3,
      2, common::Pose2d{0.0, 0.0, 0.0},
      {collision::Obstacle::rectangle("obs-1", 0.2, 0.0, 0.4, 0.2)});
  EXPECT_EQ(id,
            "20821dc86d1519b59d252e17f34a3ccf35dcccb1f7a0f876eedd108718e4e776");
}

TEST(SceneContentId, MatchesV6ProtocolCaseHashes) {
  // v6 P11：窗口 (0,0)-(20,20)、0.1 m、obs-a 矩形 + obs-b 三角形。
  const std::string p11 = scene::scene_content_id(
      "mapping_example", kExampleMapVersion, map::Bounds2d{0.0, 0.0, 20.0, 20.0}, 0.1,
      200, 200, common::Pose2d{0.0, 0.0, 0.0},
      {collision::Obstacle::rectangle("obs-a", 2.0, 2.0, 4.0, 4.0),
       collision::Obstacle::convex_polygon(
           "obs-b", {common::Point2d{10.0, 10.0}, common::Point2d{12.0, 10.0},
                     common::Point2d{11.0, 13.0}})});
  EXPECT_EQ(p11,
            "044007e558d42fbe4ff7cafc2a239834cf75ceefc6fbc464425233e799ee22d8");

  // v6 P13(b)：同一窗口下两个不同障碍 → 两个不同 scene_id。
  const std::string preview = scene::scene_content_id(
      "mapping_example", kExampleMapVersion, map::Bounds2d{0.0, 0.0, 6.0, 6.0}, 0.1, 60,
      60, common::Pose2d{0.0, 0.0, 0.0},
      {collision::Obstacle::rectangle("obs-preview", 1.0, 1.0, 2.0, 2.0)});
  const std::string plan = scene::scene_content_id(
      "mapping_example", kExampleMapVersion, map::Bounds2d{0.0, 0.0, 6.0, 6.0}, 0.1, 60,
      60, common::Pose2d{0.0, 0.0, 0.0},
      {collision::Obstacle::rectangle("obs-plan", 3.0, 3.0, 4.0, 4.0)});
  EXPECT_EQ(preview,
            "f903719dba895138267c04fb1d7aa2588ab0420e25e5c1e60fee4c536c01d766");
  EXPECT_EQ(plan,
            "2ae0202fb955d8d9e0cf64d8a15f022d3d8ec18b295a91a266e2a7987d3b78b7");
  EXPECT_NE(preview, plan);
}

TEST(SceneContentId, IsIdempotentAndSensitiveToInputs) {
  const map::Bounds2d window{0.0, 0.0, 6.0, 6.0};
  const auto base = scene::scene_content_id(
      "mapping_example", kExampleMapVersion, window, 0.1, 60, 60,
      common::Pose2d{0.0, 0.0, 0.0},
      {collision::Obstacle::rectangle("obs", 1.0, 1.0, 2.0, 2.0)});
  const auto same = scene::scene_content_id(
      "mapping_example", kExampleMapVersion, window, 0.1, 60, 60,
      common::Pose2d{0.0, 0.0, 0.0},
      {collision::Obstacle::rectangle("obs", 1.0, 1.0, 2.0, 2.0)});
  EXPECT_EQ(base, same);
  // 障碍改名（几何不变）改变标识。
  const auto renamed = scene::scene_content_id(
      "mapping_example", kExampleMapVersion, window, 0.1, 60, 60,
      common::Pose2d{0.0, 0.0, 0.0},
      {collision::Obstacle::rectangle("obs-2", 1.0, 1.0, 2.0, 2.0)});
  EXPECT_NE(base, renamed);
  // 分辨率变化改变标识。
  const auto coarser = scene::scene_content_id(
      "mapping_example", kExampleMapVersion, window, 0.2, 30, 30,
      common::Pose2d{0.0, 0.0, 0.0},
      {collision::Obstacle::rectangle("obs", 1.0, 1.0, 2.0, 2.0)});
  EXPECT_NE(base, coarser);
}

// ---------------------------------------------------------------------------
// 定点格式（协议 §5.1 规则 4）。
// ---------------------------------------------------------------------------

TEST(SceneFixedFormat, RoundsHalfAwayFromZeroWithNineDecimals) {
  EXPECT_EQ(scene::format_scene_fixed(0.1), "0.100000000");
  EXPECT_EQ(scene::format_scene_fixed(-0.0), "0.000000000");
  EXPECT_EQ(scene::format_scene_fixed(1.0000000005), "1.000000001");
  EXPECT_EQ(scene::format_scene_fixed(-1.0000000005), "-1.000000001");
  EXPECT_EQ(scene::format_scene_fixed(0.0000000004), "0.000000000");
  EXPECT_EQ(scene::format_scene_fixed(-2.5), "-2.500000000");
  EXPECT_EQ(scene::format_scene_fixed(20.0), "20.000000000");
  EXPECT_EQ(scene::quantize_scene_value(0.1000000004), 0.1);
  EXPECT_EQ(scene::quantize_scene_value(-0.0), 0.0);
}

// ---------------------------------------------------------------------------
// base64_u8 与 grid_snapshot 组装（协议 §2）。
// ---------------------------------------------------------------------------

TEST(GridSnapshotBase64, EncodesTriStateBytes) {
  EXPECT_EQ(scene::encode_cells_base64(std::vector<int>{0, 1, 2}), "AAEC");
  EXPECT_EQ(scene::encode_cells_base64(std::vector<int>{0, 1}), "AAE=");
  EXPECT_EQ(scene::encode_cells_base64(std::vector<int>{2}), "Ag==");
  EXPECT_EQ(scene::encode_cells_base64(std::vector<int>{0, 1, 0, 0, 0, 2}), "AAEAAAAC");
  EXPECT_THROW(scene::encode_cells_base64(std::vector<int>{3}), std::invalid_argument);
  EXPECT_THROW(scene::encode_cells_base64(std::vector<int>{-1}), std::invalid_argument);
}

common::GridView make_example_grid() {
  common::GridView grid;
  grid.map_id = "mapping_example";
  grid.map_version = kExampleMapVersion;
  grid.width = 3;
  grid.height = 2;
  grid.resolution_m = 0.2;
  grid.origin = common::Pose2d{0.0, 0.0, 0.0};
  grid.cells = {0, 1, 0, 0, 0, 0};
  return grid;
}

TEST(GridSnapshotJson, BuildsDriveableDocumentWithoutSceneFields) {
  scene::GridSnapshotOptions options;
  options.source = "driveable";
  options.window = map::Bounds2d{0.0, 0.0, 0.6, 0.4};
  options.requested_resolution_m = 0.2;
  options.budget_cells = 16777216;
  options.request_seq = 11;
  const nlohmann::json doc =
      scene::make_grid_snapshot_json(make_example_grid(), options);
  EXPECT_EQ(doc["schema_version"], 1);
  EXPECT_EQ(doc["type"], "grid_snapshot");
  EXPECT_EQ(doc["source"], "driveable");
  EXPECT_FALSE(doc.contains("scene_id"));
  EXPECT_FALSE(doc.contains("scene_version"));
  EXPECT_EQ(doc["map_id"], "mapping_example");
  EXPECT_EQ(doc["map_version"], kExampleMapVersion);
  EXPECT_EQ(doc["width"], 3);
  EXPECT_EQ(doc["height"], 2);
  EXPECT_DOUBLE_EQ(doc["requested_resolution_m"].get<double>(), 0.2);
  EXPECT_DOUBLE_EQ(doc["effective_resolution_m"].get<double>(), 0.2);
  EXPECT_DOUBLE_EQ(doc["resolution_m"].get<double>(), 0.2);
  EXPECT_FALSE(doc["degraded"].get<bool>());
  EXPECT_EQ(doc["encoding"], "base64_u8");
  EXPECT_EQ(doc["cells"], "AAEAAAAA");
  EXPECT_EQ(doc["request_seq"], 11);
  EXPECT_EQ(doc["budget_cells"], 16777216);
  EXPECT_DOUBLE_EQ(doc["window"]["max_y_m"].get<double>(), 0.4);
  EXPECT_DOUBLE_EQ(doc["origin"]["yaw_rad"].get<double>(), 0.0);
}

TEST(GridSnapshotJson, PlanningSceneDocumentCarriesSceneIdAndDegradedFlag) {
  common::GridView grid = make_example_grid();
  grid.resolution_m = 0.4;  // 降级后
  scene::GridSnapshotOptions options;
  options.source = "planning_scene";
  options.scene_id = std::string(64, 'a');
  options.scene_version = "scene_v1";
  options.window = map::Bounds2d{0.0, 0.0, 0.6, 0.4};
  options.requested_resolution_m = 0.2;
  options.budget_cells = 16777216;
  const nlohmann::json doc =
      scene::make_grid_snapshot_json(grid, options);
  EXPECT_EQ(doc["scene_id"], std::string(64, 'a'));
  EXPECT_EQ(doc["scene_version"], "scene_v1");
  EXPECT_TRUE(doc["degraded"].get<bool>());
  EXPECT_DOUBLE_EQ(doc["effective_resolution_m"].get<double>(), 0.4);
  EXPECT_FALSE(doc.contains("request_seq"));
}

}  // namespace
}  // namespace planner::service
