// 对象、边界、拓扑与路由查询测试（M2/M3）。

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <set>
#include <string>

#include "test_support.hpp"

namespace planner::map {
namespace {

bool numeric_less(const ObjectId& lhs, const ObjectId& rhs) {
  return std::stoll(lhs) < std::stoll(rhs);
}

class MapQueryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto result = test::load_mapping_example();
    ASSERT_TRUE(result.ok()) << result.reason;
    map_ = std::move(result.map);
  }

  std::unique_ptr<Map> map_;
};

TEST_F(MapQueryTest, lanelet_ids_are_complete_sorted_and_unique) {
  const auto ids = map_->lanelet_ids();
  EXPECT_EQ(ids.size(), map_->info().counts.lanelets);
  EXPECT_TRUE(std::is_sorted(ids.begin(), ids.end(), numeric_less));
  EXPECT_EQ(std::adjacent_find(ids.begin(), ids.end()), ids.end());
}

TEST_F(MapQueryTest, lanelet_geometry_preserves_ids_and_attributes) {
  const auto ids = map_->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto geometry = map_->lanelet_geometry(ids.front());
  ASSERT_TRUE(geometry.has_value());
  EXPECT_EQ(geometry->id, ids.front());
  ASSERT_GE(geometry->left.size(), 2u);
  ASSERT_GE(geometry->right.size(), 2u);
  ASSERT_GE(geometry->centerline.size(), 2u);
  for (const auto& point : geometry->left) {
    EXPECT_FALSE(point.id.empty());
  }
  for (const auto& point : geometry->right) {
    EXPECT_FALSE(point.id.empty());
  }
  EXPECT_TRUE(geometry->attributes.is_object());
  ASSERT_TRUE(geometry->attributes.contains("type"));
  EXPECT_EQ(geometry->attributes["type"], "lanelet");
}

TEST_F(MapQueryTest, unknown_or_invalid_object_ids_are_rejected) {
  EXPECT_FALSE(map_->lanelet_geometry("999999999").has_value());
  EXPECT_FALSE(map_->lanelet_geometry("not-an-id").has_value());
  EXPECT_FALSE(map_->lanelet_geometry("").has_value());
  EXPECT_FALSE(map_->area_geometry("999999999").has_value());
}

TEST_F(MapQueryTest, area_geometry_matches_count) {
  const auto ids = map_->area_ids();
  EXPECT_EQ(ids.size(), map_->info().counts.areas);
  if (!ids.empty()) {
    const auto area = map_->area_geometry(ids.front());
    ASSERT_TRUE(area.has_value());
    EXPECT_GE(area->outer.size(), 2u);
    EXPECT_TRUE(area->attributes.is_object());
  }
}

TEST_F(MapQueryTest, points_in_bounds_full_map_matches_count) {
  const auto points = map_->points_in_bounds(map_->info().bounds_m);
  EXPECT_EQ(points.size(), map_->info().counts.points);
  std::set<ObjectId> seen;
  for (const auto& point : points) {
    EXPECT_FALSE(point.id.empty());
    EXPECT_TRUE(seen.insert(point.id).second);
  }
}

TEST_F(MapQueryTest, window_queries_return_referenced_objects) {
  const auto ids = map_->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto geometry = map_->lanelet_geometry(ids.front());
  ASSERT_TRUE(geometry.has_value());
  ASSERT_GE(geometry->left.size(), 2u);
  const auto& anchor = geometry->left[geometry->left.size() / 2];
  const Bounds2d window{anchor.x_m - 15.0, anchor.y_m - 15.0, anchor.x_m + 15.0,
                        anchor.y_m + 15.0};
  const auto points = map_->points_in_bounds(window);
  EXPECT_FALSE(points.empty());
  for (const auto& point : points) {
    EXPECT_TRUE(window.contains(point.x_m, point.y_m));
  }
  const auto lanelets = map_->lanelets_in_bounds(window);
  EXPECT_NE(std::find(lanelets.begin(), lanelets.end(), ids.front()), lanelets.end());
  const auto full = map_->lanelets_in_bounds(map_->info().bounds_m);
  EXPECT_EQ(full, ids);
}

TEST_F(MapQueryTest, follower_connections_are_classified) {
  // 直接后继在此地图上分为两个合法类别：
  //  * 按存储方向的连接：preceding(next) 会报告回链；
  //  * 反向进入的连接：后继是通过其反向顶点到达的，因此 preceding(next)
  //    与 route(id, next) 都不能仅凭 id 访问到它。
  // 封装层必须为两者都报告 id；此地图上必须至少存在一些正常的回链。
  std::size_t followers = 0;
  std::size_t back_links = 0;
  std::size_t lanelets_scanned = 0;
  for (const auto& id : map_->lanelet_ids()) {
    const auto following = map_->following(id);
    ASSERT_EQ(following.status, common::Status::kSuccess) << following.reason;
    for (const auto& next : following.ids) {
      ++followers;
      const auto preceding = map_->preceding(next);
      ASSERT_EQ(preceding.status, common::Status::kSuccess) << preceding.reason;
      if (std::find(preceding.ids.begin(), preceding.ids.end(), id) != preceding.ids.end()) {
        ++back_links;
      }
    }
    if (++lanelets_scanned >= 100u) {
      break;
    }
  }
  EXPECT_GT(followers, 0u);
  EXPECT_GT(back_links, 0u);
}

TEST_F(MapQueryTest, directional_lanelets_pin_inversion_semantics) {
  // 单向路由语义的冻结地图回归测试：lanelet 45312（单行道）的直接后继
  // 是 45318，但 45318 是从另一端进入的，因此按存储方向的前驱查询不会
  // 报告回链，getRoute 也只访问存储的顶点
  // （Lanelet2 为反向进入保留了一个反向顶点）。
  const auto following = map_->following("45312");
  ASSERT_EQ(following.status, common::Status::kSuccess) << following.reason;
  EXPECT_NE(std::find(following.ids.begin(), following.ids.end(), "45318"),
            following.ids.end());

  const auto preceding = map_->preceding("45318");
  ASSERT_EQ(preceding.status, common::Status::kSuccess) << preceding.reason;
  EXPECT_EQ(std::find(preceding.ids.begin(), preceding.ids.end(), "45312"),
            preceding.ids.end());

  const auto route = map_->route("45312", "45318");
  EXPECT_EQ(route.status, common::Status::kNoPath) << route.reason;
}

TEST_F(MapQueryTest, topology_queries_reject_unknown_and_invalid_ids) {
  const auto unknown = map_->following("999999999");
  EXPECT_EQ(unknown.status, common::Status::kInvalidInput);
  EXPECT_NE(unknown.reason.find("unknown lanelet id"), std::string::npos) << unknown.reason;

  const auto invalid = map_->preceding("abc");
  EXPECT_EQ(invalid.status, common::Status::kInvalidInput);
  EXPECT_NE(invalid.reason.find("invalid lanelet id"), std::string::npos) << invalid.reason;

  const auto adjacent = map_->adjacent("", NeighborSide::kLeft);
  EXPECT_EQ(adjacent.status, common::Status::kInvalidInput);
}

TEST_F(MapQueryTest, route_from_lanelet_to_itself_succeeds) {
  const auto ids = map_->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  for (const auto& id : ids) {
    const auto route = map_->route(id, id);
    if (route.status == common::Status::kSuccess) {
      ASSERT_EQ(route.lanelet_ids.size(), 1u);
      EXPECT_EQ(route.lanelet_ids.front(), id);
      EXPECT_GE(route.length_m, 0.0);
      return;
    }
  }
  FAIL() << "no lanelet accepted a self route";
}

TEST_F(MapQueryTest, route_to_direct_follower_succeeds) {
  std::size_t routable = 0;
  bool enough = false;
  for (const auto& id : map_->lanelet_ids()) {
    const auto following = map_->following(id);
    if (following.status != common::Status::kSuccess) {
      continue;
    }
    for (const auto& next : following.ids) {
      const auto route = map_->route(id, next);
      if (route.status != common::Status::kSuccess) {
        continue;  // 反向进入的连接：无法仅凭存储 id 访问
      }
      ASSERT_GE(route.lanelet_ids.size(), 2u);
      EXPECT_EQ(route.lanelet_ids.front(), id);
      EXPECT_EQ(route.lanelet_ids.back(), next);
      EXPECT_GE(route.length_m, 0.0);
      ++routable;
      if (routable >= 5u) {
        enough = true;
        break;
      }
    }
    if (enough) {
      break;
    }
  }
  EXPECT_GT(routable, 0u) << "no direct follower was routable in the example map";
}

TEST_F(MapQueryTest, route_rejects_unknown_ids) {
  const auto ids = map_->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto result = map_->route(ids.front(), "999999999");
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("unknown lanelet id"), std::string::npos) << result.reason;
}

TEST(MapQuery, disconnected_map_reports_no_path_and_no_neighbors) {
  auto result = test::load_disconnected();
  ASSERT_TRUE(result.ok()) << result.reason;
  const Map& map = *result.map;
  const auto ids = map.lanelet_ids();
  ASSERT_EQ(ids.size(), 2u);

  const auto route = map.route(ids[0], ids[1]);
  EXPECT_EQ(route.status, common::Status::kNoPath) << route.reason;
  EXPECT_TRUE(route.lanelet_ids.empty());

  const auto left = map.adjacent(ids[0], NeighborSide::kLeft);
  ASSERT_EQ(left.status, common::Status::kSuccess) << left.reason;
  EXPECT_TRUE(left.ids.empty());

  const auto following = map.following(ids[0]);
  ASSERT_EQ(following.status, common::Status::kSuccess) << following.reason;
  EXPECT_TRUE(following.ids.empty());
}

}  // namespace
}  // namespace planner::map
