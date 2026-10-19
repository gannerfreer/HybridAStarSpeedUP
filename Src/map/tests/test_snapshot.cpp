// 公开 JSON 地图快照测试（M4）与独立投影辅助函数。

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "test_support.hpp"

namespace planner::map {
namespace {

class MapSnapshotTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto result = test::load_mapping_example();
    ASSERT_TRUE(result.ok()) << result.reason;
    map_ = std::move(result.map);
    snapshot_ = map_->to_snapshot_json();
  }

  std::unique_ptr<Map> map_;
  nlohmann::json snapshot_;
};

TEST_F(MapSnapshotTest, envelope_and_metadata_match_the_map) {
  EXPECT_EQ(snapshot_["schema_version"], 1);
  EXPECT_EQ(snapshot_["type"], "map_snapshot");
  const auto& info = map_->info();
  EXPECT_EQ(snapshot_["map_id"], info.map_id);
  EXPECT_EQ(snapshot_["map_version"], info.map_version);
  EXPECT_EQ(snapshot_["coordinate_system"]["projection"], "utm");
  EXPECT_EQ(snapshot_["coordinate_system"]["frame_id"], "map");
  EXPECT_DOUBLE_EQ(snapshot_["coordinate_system"]["origin"]["lat_deg"].get<double>(),
                   info.origin.lat_deg);
  EXPECT_DOUBLE_EQ(snapshot_["coordinate_system"]["origin"]["lon_deg"].get<double>(),
                   info.origin.lon_deg);
  EXPECT_EQ(snapshot_["counts"]["points"], info.counts.points);
  EXPECT_EQ(snapshot_["counts"]["lanelets"], info.counts.lanelets);
  EXPECT_EQ(snapshot_["counts"]["areas"], info.counts.areas);
  EXPECT_EQ(snapshot_["lanelets"].size(), info.counts.lanelets);
  EXPECT_EQ(snapshot_["areas"].size(), info.counts.areas);
  EXPECT_EQ(snapshot_["regulatory_elements"].size(), info.counts.regulatory_elements);
  EXPECT_DOUBLE_EQ(snapshot_["bounds_m"]["min_x_m"].get<double>(), info.bounds_m.min_x_m);
  EXPECT_DOUBLE_EQ(snapshot_["bounds_m"]["max_y_m"].get<double>(), info.bounds_m.max_y_m);
}

TEST_F(MapSnapshotTest, lanelet_geometry_preserves_ids_and_matches_query_api) {
  const auto ids = map_->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto first = map_->lanelet_geometry(ids.front());
  ASSERT_TRUE(first.has_value());

  const nlohmann::json* entry = nullptr;
  for (const auto& candidate : snapshot_["lanelets"]) {
    if (candidate["id"] == ids.front()) {
      entry = &candidate;
      break;
    }
  }
  ASSERT_NE(entry, nullptr);
  EXPECT_TRUE((*entry)["id"].is_string());
  EXPECT_EQ((*entry)["attributes"]["type"], "lanelet");
  ASSERT_EQ((*entry)["left"].size(), first->left.size());
  ASSERT_EQ((*entry)["right"].size(), first->right.size());
  for (std::size_t i = 0; i < first->left.size(); ++i) {
    EXPECT_EQ((*entry)["left"][i]["id"], first->left[i].id);
    EXPECT_DOUBLE_EQ((*entry)["left"][i]["x_m"].get<double>(), first->left[i].x_m);
    EXPECT_DOUBLE_EQ((*entry)["left"][i]["y_m"].get<double>(), first->left[i].y_m);
  }
  EXPECT_EQ((*entry)["centerline"].size(), first->centerline.size());
}

TEST_F(MapSnapshotTest, every_number_is_finite_and_ids_are_strings) {
  const std::function<void(const nlohmann::json&)> walk = [&](const nlohmann::json& node) {
    if (node.is_number_float()) {
      EXPECT_TRUE(std::isfinite(node.get<double>()));
    } else if (node.is_object()) {
      for (auto it = node.begin(); it != node.end(); ++it) {
        walk(it.value());
      }
    } else if (node.is_array()) {
      for (const auto& child : node) {
        walk(child);
      }
    }
  };
  walk(snapshot_);

  for (const auto& lanelet : snapshot_["lanelets"]) {
    EXPECT_TRUE(lanelet["id"].is_string());
    EXPECT_TRUE(lanelet["attributes"].is_object());
    for (const auto& point : lanelet["left"]) {
      EXPECT_TRUE(point["id"].is_string());
    }
  }
  for (const auto& area : snapshot_["areas"]) {
    EXPECT_TRUE(area["id"].is_string());
  }
  for (const auto& element : snapshot_["regulatory_elements"]) {
    EXPECT_TRUE(element["id"].is_string());
    EXPECT_TRUE(element["roles"].is_object());
    for (auto it = element["roles"].begin(); it != element["roles"].end(); ++it) {
      EXPECT_TRUE(it.value().is_array());
      for (const auto& id : it.value()) {
        EXPECT_TRUE(id.is_string());
      }
    }
  }
}

TEST_F(MapSnapshotTest, dump_parse_round_trip_is_stable) {
  const std::string text = snapshot_.dump();
  const auto reparsed = nlohmann::json::parse(text);
  EXPECT_EQ(reparsed, snapshot_);
}

TEST_F(MapSnapshotTest, content_options_are_respected) {
  SnapshotOptions options;
  options.include_centerline = false;
  options.include_areas = false;
  options.include_regulatory_elements = false;
  const auto trimmed = map_->to_snapshot_json(options);
  EXPECT_FALSE(trimmed.contains("areas"));
  EXPECT_FALSE(trimmed.contains("regulatory_elements"));
  EXPECT_EQ(trimmed["lanelets"].size(), map_->info().counts.lanelets);
  for (const auto& lanelet : trimmed["lanelets"]) {
    EXPECT_FALSE(lanelet.contains("centerline"));
  }
}

TEST(MapProjectionStatics, standalone_helpers_match_instance_methods) {
  auto result = test::load_mapping_example();
  ASSERT_TRUE(result.ok()) << result.reason;
  const Map& map = *result.map;
  const auto info = map.info();
  for (const GeoPoint sample :
       {GeoPoint{49.0034, 8.4242, 0.0}, GeoPoint{49.0040, 8.4300, 0.0}}) {
    const auto local = map.to_local(sample);
    ASSERT_TRUE(local.has_value());
    const auto standalone = Map::project_to_local(sample, info.projection, info.origin);
    ASSERT_TRUE(standalone.has_value());
    EXPECT_NEAR(standalone->x_m, local->x_m, 1e-9);
    EXPECT_NEAR(standalone->y_m, local->y_m, 1e-9);

    const auto gps = Map::project_to_gps(*local, info.projection, info.origin);
    ASSERT_TRUE(gps.has_value());
    EXPECT_NEAR(gps->lat_deg, sample.lat_deg, 1e-8);
    EXPECT_NEAR(gps->lon_deg, sample.lon_deg, 1e-8);
  }

  EXPECT_FALSE(
      Map::project_to_local(GeoPoint{95.0, 8.4, 0.0}, info.projection, info.origin).has_value());
  EXPECT_FALSE(Map::project_to_gps(common::Point2d{std::nan(""), 0.0}, info.projection,
                                   info.origin)
                   .has_value());
  EXPECT_FALSE(
      Map::project_to_gps(common::Point2d{0.0, 0.0}, info.projection, GeoPoint{91.0, 8.4, 0.0})
          .has_value());
}

TEST(MapProjectionStatics, standalone_round_trip_for_each_projection) {
  const GeoPoint origin{49.0, 8.4, 0.0};
  for (const ProjectionKind kind : {ProjectionKind::kUtm, ProjectionKind::kMercator,
                                    ProjectionKind::kSphericalMercator}) {
    for (double x = -300.0; x <= 300.0; x += 150.0) {
      for (double y = -300.0; y <= 300.0; y += 150.0) {
        const auto gps = Map::project_to_gps(common::Point2d{x, y}, kind, origin);
        ASSERT_TRUE(gps.has_value());
        const auto back = Map::project_to_local(*gps, kind, origin);
        ASSERT_TRUE(back.has_value());
        EXPECT_LE(std::hypot(back->x_m - x, back->y_m - y), 1e-3)
            << to_string(kind) << " at " << x << ", " << y;
      }
    }
  }
}

}  // namespace
}  // namespace planner::map
