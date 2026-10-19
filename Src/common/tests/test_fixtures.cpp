// 测试夹具驱动的契约测试：tests/fixtures/v1/serialization/{valid,invalid}
// 下的每个文件都通过公开编解码器执行。夹具的绝对目录在配置时注入。

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "planner/common/errors.hpp"
#include "planner/common/serialization.hpp"
#include "planner/common/types.hpp"

#ifndef PLANNER_COMMON_FIXTURES_DIR
#error "PLANNER_COMMON_FIXTURES_DIR must be defined by the build (absolute path to tests/fixtures/v1/serialization)"
#endif

namespace planner::common {
namespace {

std::string read_text_file(const std::string& relative_path) {
  const std::string path = std::string(PLANNER_COMMON_FIXTURES_DIR) + "/" + relative_path;
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open fixture: " + path);
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

template <typename T>
void expect_valid_fixture(const std::string& file, const std::string& type_name) {
  const std::string text = read_text_file("valid/" + file);
  const T value = decode_document<T>(text);

  // 解码 -> 编码 -> 解码必须还原出相同的值（包括精确的 double），
  // 且输出的文档必须携带冻结的信封。
  const std::string reencoded = encode_document(value);
  const T again = decode_document<T>(reencoded);
  EXPECT_EQ(value, again) << file;

  const nlohmann::json document = nlohmann::json::parse(text);
  ASSERT_EQ(document.at("schema_version").get<int>(), 1) << file;
  ASSERT_EQ(document.at("type").get<std::string>(), type_name) << file;
  const nlohmann::json emitted = nlohmann::json::parse(reencoded);
  ASSERT_EQ(emitted.at("schema_version").get<int>(), 1) << file;
  ASSERT_EQ(emitted.at("type").get<std::string>(), type_name) << file;
}

template <typename T>
void expect_invalid_fixture(const std::string& file, ErrorCode expected) {
  const std::string text = read_text_file("invalid/" + file);
  try {
    (void)decode_document<T>(text);
    FAIL() << file << ": decode_document must reject this fixture";
  } catch (const SerializationError& error) {
    EXPECT_EQ(error.code(), expected) << file << ": " << error.what();
  }
}

TEST(SerializationFixtures, accepts_valid_fixtures) {
  expect_valid_fixture<Pose2d>("pose2d.json", "pose2d");
  expect_valid_fixture<VehicleModel>("vehicle_model.json", "vehicle_model");
  expect_valid_fixture<GridView>("grid_view.json", "grid_view");
  expect_valid_fixture<PathPoint>("path_point.json", "path_point");
  expect_valid_fixture<PathSegment>("path_segment.json", "path_segment");
  expect_valid_fixture<TrajectoryPoint>("trajectory_point.json", "trajectory_point");
  expect_valid_fixture<PlanRequest>("plan_request.json", "plan_request");
  expect_valid_fixture<PlanResult>("plan_result.json", "plan_result");
  expect_valid_fixture<Status>("status.json", "status");
  expect_valid_fixture<Corridor>("corridor.json", "corridor");
}

TEST(SerializationFixtures, rejects_invalid_fixtures) {
  expect_invalid_fixture<Pose2d>("schema_version_unknown.json", ErrorCode::kUnsupportedSchemaVersion);
  expect_invalid_fixture<Pose2d>("schema_version_missing.json", ErrorCode::kMissingField);
  expect_invalid_fixture<Pose2d>("schema_version_wrong_type.json", ErrorCode::kTypeError);
  expect_invalid_fixture<Pose2d>("type_unknown.json", ErrorCode::kUnknownType);
  expect_invalid_fixture<Pose2d>("type_missing.json", ErrorCode::kMissingField);
  expect_invalid_fixture<Pose2d>("malformed_json.json", ErrorCode::kParseError);
  expect_invalid_fixture<Pose2d>("nan_literal.json", ErrorCode::kParseError);
  expect_invalid_fixture<Pose2d>("nonfinite_overflow.json", ErrorCode::kNonFiniteValue);
  expect_invalid_fixture<Pose2d>("pose_wrong_type.json", ErrorCode::kTypeError);

  expect_invalid_fixture<VehicleModel>("vehicle_missing_required.json", ErrorCode::kMissingField);
  expect_invalid_fixture<VehicleModel>("vehicle_numeric_id.json", ErrorCode::kTypeError);
  expect_invalid_fixture<VehicleModel>("vehicle_invalid_wheelbase.json", ErrorCode::kInvalidValue);

  expect_invalid_fixture<GridView>("grid_cells_length_mismatch.json", ErrorCode::kInvalidValue);
  expect_invalid_fixture<GridView>("grid_invalid_cell_value.json", ErrorCode::kInvalidValue);
  expect_invalid_fixture<GridView>("grid_resolution_not_positive.json", ErrorCode::kInvalidValue);

  expect_invalid_fixture<PathPoint>("path_point_unknown_direction.json", ErrorCode::kUnknownEnumValue);
  expect_invalid_fixture<PathSegment>("path_segment_direction_mixed.json", ErrorCode::kInvalidValue);
  expect_invalid_fixture<PathSegment>("path_segment_s_not_increasing.json", ErrorCode::kInvalidValue);

  expect_invalid_fixture<PlanRequest>("plan_request_budget_wrong_type.json", ErrorCode::kTypeError);
  expect_invalid_fixture<PlanResult>("plan_result_unknown_status.json", ErrorCode::kUnknownEnumValue);
  expect_invalid_fixture<PlanResult>("plan_result_trajectory_time_not_increasing.json",
                                     ErrorCode::kInvalidValue);

  expect_invalid_fixture<Corridor>("corridor_polygon_too_few_vertices.json", ErrorCode::kInvalidValue);
  expect_invalid_fixture<Corridor>("corridor_s_interval_empty.json", ErrorCode::kInvalidValue);
  expect_invalid_fixture<Corridor>("corridor_polygons_unordered.json", ErrorCode::kInvalidValue);
}

// 共享的 v1 车辆/栅格夹具由夹具工作包负责
// （tests/fixtures/v1/{vehicles,grids}）；在此加载它们可验证
// 冻结的 common 契约能原样消费共享夹具（E0-5）。
TEST(SerializationFixtures, loads_shared_v1_vehicle_and_grid_fixtures) {
  const VehicleModel vehicle = decode_document<VehicleModel>(read_text_file("../vehicles/default.json"));
  EXPECT_EQ(vehicle.id, "default_test_vehicle");
  EXPECT_DOUBLE_EQ(vehicle.wheelbase_m, 2.8);
  EXPECT_DOUBLE_EQ(vehicle.width_m, 1.8);
  EXPECT_DOUBLE_EQ(vehicle.front_overhang_m, 0.9);
  EXPECT_DOUBLE_EQ(vehicle.rear_overhang_m, 1.0);
  EXPECT_DOUBLE_EQ(vehicle.max_steer_rad, 0.5);
  EXPECT_DOUBLE_EQ(vehicle.safety_margin_m, 0.1);

  const GridView grid = decode_document<GridView>(read_text_file("../grids/minimal.json"));
  EXPECT_EQ(grid.map_id, "grid_minimal_20x20");
  EXPECT_EQ(grid.map_version, "v1");
  EXPECT_EQ(grid.width, 20);
  EXPECT_EQ(grid.height, 20);
  EXPECT_DOUBLE_EQ(grid.resolution_m, 0.1);
  ASSERT_EQ(grid.cells.size(), 400U);
  EXPECT_EQ(grid.cell(0, 0), kCellOccupied);
  EXPECT_EQ(grid.cell(0, 19), kCellOccupied);
  EXPECT_EQ(grid.cell(19, 19), kCellOccupied);
  EXPECT_EQ(grid.cell(19, 0), kCellUnknown);
}

}  // namespace
}  // namespace planner::common
