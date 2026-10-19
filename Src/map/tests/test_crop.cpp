// 局部窗口裁剪测试（M3）：id 保留、裁剪边语义、
// 窗口局部变换与拓扑隔离。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace planner::map {
namespace {

bool same_point(const MapPoint& a, const MapPoint& b) {
  if (!a.id.empty() && !b.id.empty()) {
    return a.id == b.id;
  }
  return std::abs(a.x_m - b.x_m) <= 1e-9 && std::abs(a.y_m - b.y_m) <= 1e-9;
}

std::set<ObjectId> collect_ids(const std::vector<MapPoint>& points) {
  std::set<ObjectId> ids;
  for (const auto& point : points) {
    if (!point.id.empty()) {
      ids.insert(point.id);
    }
  }
  return ids;
}

class MapCropTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto result = test::load_mapping_example();
    ASSERT_TRUE(result.ok()) << result.reason;
    map_ = std::move(result.map);
  }

  std::unique_ptr<Map> map_;
};

TEST_F(MapCropTest, crop_keeps_object_ids_and_clip_flags_are_consistent) {
  const auto ids = map_->lanelet_ids();
  ASSERT_FALSE(ids.empty());

  // 选一个边界较长的 lanelet，让窗口确实能切断它。
  ObjectId chosen;
  LaneletGeometry geometry;
  for (const auto& id : ids) {
    auto candidate = map_->lanelet_geometry(id);
    if (!candidate || candidate->left.size() < 3) {
      continue;
    }
    const double dx = candidate->left.back().x_m - candidate->left.front().x_m;
    const double dy = candidate->left.back().y_m - candidate->left.front().y_m;
    if (std::hypot(dx, dy) > 20.0) {
      chosen = id;
      geometry = *candidate;
      break;
    }
  }
  ASSERT_FALSE(chosen.empty()) << "no long lanelet found for the crop test";

  const auto& anchor = geometry.left[geometry.left.size() / 2];
  const Bounds2d window{anchor.x_m - 4.0, anchor.y_m - 4.0, anchor.x_m + 4.0, anchor.y_m + 4.0};
  const auto crop = map_->crop_to_bounds(window);

  EXPECT_NE(std::find(crop.lanelet_ids.begin(), crop.lanelet_ids.end(), chosen),
            crop.lanelet_ids.end());

  const std::set<ObjectId> original_left = collect_ids(geometry.left);
  const std::set<ObjectId> original_right = collect_ids(geometry.right);
  const std::set<ObjectId> original_centerline = collect_ids(geometry.centerline);

  std::size_t chosen_paths = 0;
  std::size_t clip_edges = 0;
  for (const auto& path : crop.paths) {
    if (path.lanelet_id != chosen) {
      continue;
    }
    ++chosen_paths;
    clip_edges += (path.start_clip_edge ? 1U : 0U) + (path.end_clip_edge ? 1U : 0U);
    ASSERT_FALSE(path.points.empty());

    const std::vector<MapPoint>* original = nullptr;
    if (path.role == "left") {
      original = &geometry.left;
    } else if (path.role == "right") {
      original = &geometry.right;
    } else if (path.role == "centerline") {
      original = &geometry.centerline;
    }
    ASSERT_NE(original, nullptr) << "unexpected crop role: " << path.role;
    ASSERT_GE(original->size(), 2u);

    // 每个点都位于窗口内；原始 id 被保留，合成的裁剪点不带 id。
    for (const auto& point : path.points) {
      EXPECT_GE(point.x_m, window.min_x_m - 1e-9);
      EXPECT_LE(point.x_m, window.max_x_m + 1e-9);
      EXPECT_GE(point.y_m, window.min_y_m - 1e-9);
      EXPECT_LE(point.y_m, window.max_y_m + 1e-9);
      const auto& source_ids = path.role == "left"     ? original_left
                               : path.role == "right"  ? original_right
                                                       : original_centerline;
      if (!point.id.empty()) {
        EXPECT_NE(source_ids.find(point.id), source_ids.end())
            << "cropped point id " << point.id << " is not part of the original " << path.role;
      }
    }

    // 裁剪边恰好表示“该端不是原始折线的端点”。
    EXPECT_EQ(path.start_clip_edge, !same_point(path.points.front(), original->front()));
    EXPECT_EQ(path.end_clip_edge, !same_point(path.points.back(), original->back()));
  }
  EXPECT_GT(chosen_paths, 0u);
  EXPECT_GT(clip_edges, 0u) << "the 4 m window should cut the long boundary";
}

TEST_F(MapCropTest, crop_of_full_bounds_keeps_original_geometry) {
  const auto ids = map_->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto geometry = map_->lanelet_geometry(ids.front());
  ASSERT_TRUE(geometry.has_value());

  const auto crop = map_->crop_to_bounds(map_->info().bounds_m);
  EXPECT_EQ(crop.lanelet_ids, ids);

  std::vector<const ClippedPath*> paths;
  for (const auto& path : crop.paths) {
    if (path.lanelet_id == ids.front()) {
      paths.push_back(&path);
    }
  }
  ASSERT_EQ(paths.size(), 3u);  // left、right、centerline，各一段
  for (const auto* path : paths) {
    EXPECT_FALSE(path->start_clip_edge);
    EXPECT_FALSE(path->end_clip_edge);
  }
  const auto* left_path = paths[0];
  EXPECT_EQ(left_path->role, "left");
  ASSERT_EQ(left_path->points.size(), geometry->left.size());
  for (std::size_t i = 0; i < left_path->points.size(); ++i) {
    EXPECT_TRUE(same_point(left_path->points[i], geometry->left[i]));
  }
}

TEST_F(MapCropTest, window_transform_round_trip) {
  const Bounds2d window{-120.5, 40.25, -90.5, 70.25};
  const auto crop = map_->crop_to_bounds(window);
  EXPECT_DOUBLE_EQ(crop.window_frame.x_m, window.min_x_m);
  EXPECT_DOUBLE_EQ(crop.window_frame.y_m, window.min_y_m);
  EXPECT_DOUBLE_EQ(crop.window_frame.yaw_rad, 0.0);

  for (const common::Point2d local : {common::Point2d{0.0, 0.0}, common::Point2d{3.5, -2.0},
                                      common::Point2d{30.0, 30.0}}) {
    const auto in_map = crop.local_to_map(local);
    const auto back = crop.map_to_local(in_map);
    EXPECT_NEAR(back.x_m, local.x_m, 1e-9);
    EXPECT_NEAR(back.y_m, local.y_m, 1e-9);
  }
  const auto corner = crop.local_to_map(common::Point2d{0.0, 0.0});
  EXPECT_DOUBLE_EQ(corner.x_m, window.min_x_m);
  EXPECT_DOUBLE_EQ(corner.y_m, window.min_y_m);
}

TEST_F(MapCropTest, crop_does_not_change_map_or_topology) {
  const auto ids = map_->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const ObjectId id = ids.front();
  const auto before = map_->lanelet_geometry(id);
  ASSERT_TRUE(before.has_value());

  const Bounds2d window{map_->info().bounds_m.min_x_m, map_->info().bounds_m.min_y_m,
                        map_->info().bounds_m.min_x_m + 50.0,
                        map_->info().bounds_m.min_y_m + 50.0};
  const auto crop = map_->crop_to_bounds(window);
  (void)crop;

  const auto after = map_->lanelet_geometry(id);
  ASSERT_TRUE(after.has_value());
  ASSERT_EQ(before->left.size(), after->left.size());
  ASSERT_EQ(before->right.size(), after->right.size());
  for (std::size_t i = 0; i < before->left.size(); ++i) {
    EXPECT_EQ(before->left[i].id, after->left[i].id);
  }
  EXPECT_EQ(map_->lanelet_ids(), ids);

  const auto route = map_->route(id, id);
  EXPECT_EQ(route.status, common::Status::kSuccess) << route.reason;
}

TEST_F(MapCropTest, empty_and_invalid_windows_return_empty_results) {
  const auto far_away = map_->crop_to_bounds(Bounds2d{1000000.0, 1000000.0, 1000010.0,
                                                      1000010.0});
  EXPECT_TRUE(far_away.empty());
  EXPECT_TRUE(far_away.paths.empty());
  EXPECT_TRUE(far_away.points.empty());

  const auto invalid = map_->crop_to_bounds(Bounds2d{10.0, 10.0, -10.0, -10.0});
  EXPECT_TRUE(invalid.empty());
}

}  // namespace
}  // namespace planner::map
