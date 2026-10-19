// 导入诊断与地图元数据测试（M1/M2）。

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "test_support.hpp"

namespace planner::map {
namespace {

TEST(MapImport, loads_official_example_map) {
  auto result = test::load_mapping_example();
  ASSERT_TRUE(result.ok()) << result.reason;
  const auto& info = result.map->info();
  EXPECT_EQ(info.map_id, "mapping_example");
  EXPECT_EQ(info.projection, ProjectionKind::kUtm);
  EXPECT_DOUBLE_EQ(info.origin.lat_deg, 49.00345654351);
  EXPECT_DOUBLE_EQ(info.origin.lon_deg, 8.42427590707);
  EXPECT_EQ(info.traffic_rules_location, "de");
  EXPECT_EQ(info.traffic_rules_participant, "vehicle");
  // 固定到冻结的工作副本（tests/data/README.md）：上游文件有 2258 个节点 /
  // 1141 条 way / 456 条关系；Lanelet2 加载的线串比原始 way 数少一个
  // （有一条 way 的可用节点 < 2），另含 371 个 lanelet 关系、76 个
  // multipolygon 区域和 9 个 regulatory element。
  EXPECT_EQ(info.counts.points, 2258u);
  EXPECT_EQ(info.counts.line_strings, 1140u);
  EXPECT_EQ(info.counts.polygons, 0u);
  EXPECT_EQ(info.counts.lanelets, 371u);
  EXPECT_EQ(info.counts.areas, 76u);
  EXPECT_EQ(info.counts.regulatory_elements, 9u);
  ASSERT_FALSE(info.bounds_m.empty());
  EXPECT_LT(info.bounds_m.min_x_m, info.bounds_m.max_x_m);
  EXPECT_LT(info.bounds_m.min_y_m, info.bounds_m.max_y_m);
  EXPECT_GT(info.validation.checks_run, 0u);
  // 验证发现会被报告，但不会使导入失败。
  for (const auto& entry : result.diagnostics) {
    EXPECT_FALSE(entry.empty());
  }
}

TEST(MapImport, map_version_is_sha256_of_source_bytes) {
  auto result = test::load_mapping_example();
  ASSERT_TRUE(result.ok()) << result.reason;
  const auto& info = result.map->info();
  EXPECT_EQ(info.map_version,
            "11561c763ed50322185139ec2dd041d6da72e1bd4e5fc3bb66f6b340d92c088f");
  EXPECT_EQ(info.source_bytes,
            static_cast<std::size_t>(
                std::filesystem::file_size(test::data_path("mapping_example.osm"))));
  EXPECT_EQ(info.source_path, test::data_path("mapping_example.osm"));
}

TEST(MapImport, validation_can_be_disabled) {
  auto options = test::example_options();
  options.validate_on_import = false;
  auto result = Map::import_osm_file(test::data_path("mapping_example.osm"), options);
  ASSERT_TRUE(result.ok()) << result.reason;
  EXPECT_EQ(result.map->info().validation.checks_run, 0u);
  EXPECT_EQ(result.map->info().validation.errors, 0u);
  EXPECT_EQ(result.map->info().validation.warnings, 0u);
}

TEST(MapImport, rejects_missing_file) {
  auto result = Map::import_osm_file(test::data_path("does_not_exist.osm"),
                                     test::example_options());
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("not found"), std::string::npos) << result.reason;
}

TEST(MapImport, rejects_unsupported_extension) {
  auto result = Map::import_osm_file(test::data_path("README.md"), test::example_options());
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.reason.find("unsupported map extension"), std::string::npos)
      << result.reason;
}

TEST(MapImport, requires_explicit_projection_origin) {
  MapLoadOptions options;  // 未配置原点
  auto result = Map::import_osm_file(test::data_path("mapping_example.osm"), options);
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.reason.find("origin is required"), std::string::npos) << result.reason;
}

TEST(MapImport, rejects_invalid_projection_origin) {
  auto options = test::example_options();
  options.origin->lat_deg = 91.0;
  auto result = Map::import_osm_file(test::data_path("mapping_example.osm"), options);
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.reason.find("invalid projection origin"), std::string::npos)
      << result.reason;
}

TEST(MapImport, rejects_invalid_traffic_rules) {
  auto options = test::example_options();
  options.traffic_rules_location = "xx";
  auto result = Map::import_osm_file(test::data_path("mapping_example.osm"), options);
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.reason.find("traffic rules"), std::string::npos) << result.reason;
}

TEST(MapImport, rejects_malformed_xml_with_parse_error) {
  auto result = Map::import_osm_file(test::data_path("invalid_xml.osm"), test::example_options());
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  // XML 格式错误不可恢复：解析器报告 "Start-end tags mismatch"，
  // 导入以显式的 OSM 加载错误失败。
  EXPECT_NE(result.reason.find("OSM load failed"), std::string::npos) << result.reason;
}

TEST(MapImport, accepts_map_without_lanelets_with_warning) {
  auto result = Map::import_osm_file(test::data_path("no_lanelets.osm"), test::example_options());
  ASSERT_TRUE(result.ok()) << result.reason;
  const auto& info = result.map->info();
  EXPECT_EQ(info.counts.lanelets, 0u);
  EXPECT_EQ(info.counts.points, 3u);
  EXPECT_EQ(info.counts.line_strings, 1u);
  bool warned = false;
  for (const auto& entry : result.diagnostics) {
    warned = warned || entry.find("no lanelets") != std::string::npos;
  }
  EXPECT_TRUE(warned);
}

TEST(MapImport, projection_kind_names_round_trip) {
  for (const ProjectionKind kind : {ProjectionKind::kUtm, ProjectionKind::kMercator,
                                    ProjectionKind::kSphericalMercator}) {
    const auto parsed = projection_kind_from_string(to_string(kind));
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(*parsed, kind);
  }
  EXPECT_FALSE(projection_kind_from_string("web_mercator").has_value());
}

}  // namespace
}  // namespace planner::map
