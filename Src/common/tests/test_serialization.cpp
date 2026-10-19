// planner_common 的编解码器单元测试（不涉及夹具文件）。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include <nlohmann/json.hpp>

#include "planner/common/errors.hpp"
#include "planner/common/serialization.hpp"
#include "planner/common/types.hpp"

namespace planner::common {
namespace {

using nlohmann::json;

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

VehicleModel test_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "default_test_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

PathPoint test_path_point(double s_m, Direction direction) {
  PathPoint point;
  point.pose = Pose2d{s_m, 0.0, 0.0};
  point.s_m = s_m;
  point.curvature_inv_m = 0.0;
  point.direction = direction;
  return point;
}

PathSegment test_segment(Direction direction) {
  PathSegment segment;
  segment.points.push_back(test_path_point(0.0, direction));
  segment.points.push_back(test_path_point(1.0, direction));
  return segment;
}

TrajectoryPoint test_trajectory_point(double t_s) {
  TrajectoryPoint point;
  point.t_s = t_s;
  point.pose = Pose2d{t_s, 0.0, 0.0};
  point.velocity_mps = t_s;
  point.acceleration_mps2 = 0.0;
  point.curvature_inv_m = 0.0;
  return point;
}

GridView test_grid_view() {
  GridView grid;
  grid.map_id = "test_grid";
  grid.map_version = "v1";
  grid.width = 3;
  grid.height = 2;
  grid.resolution_m = 0.1;
  grid.origin = Pose2d{1.0, 2.0, 0.0};
  grid.cells = {0, 1, 2, 0, 0, 1};
  return grid;
}

Corridor test_corridor() {
  Corridor corridor;
  corridor.id = "corridor_1";
  corridor.map_id = "test_map";
  corridor.map_version = "v1";
  corridor.frame_id = "map";
  CorridorPolygon polygon;
  polygon.s_start_m = 0.0;
  polygon.s_end_m = 2.0;
  polygon.vertices = {Point2d{0.0, 0.0}, Point2d{2.0, 0.0}, Point2d{2.0, 2.0}, Point2d{0.0, 2.0}};
  corridor.polygons.push_back(polygon);
  return corridor;
}

template <typename T>
void expect_document_error(const std::string& text, ErrorCode expected) {
  try {
    (void)decode_document<T>(text);
    FAIL() << "decode_document accepted a document it should reject";
  } catch (const SerializationError& error) {
    EXPECT_EQ(error.code(), expected) << "message: " << error.what();
  }
}

template <typename T>
void expect_encode_error(const T& value, ErrorCode expected) {
  try {
    (void)encode_document(value);
    FAIL() << "encode_document accepted a value it should reject";
  } catch (const SerializationError& error) {
    EXPECT_EQ(error.code(), expected) << "message: " << error.what();
  }
}

TEST(Serialization, round_trips_pose2d_exactly) {
  const Pose2d input{0.1, -2.8, 1.0 / 3.0};
  const std::string text = encode_document(input);
  const Pose2d output = decode_document<Pose2d>(text);
  EXPECT_EQ(output, input);
  EXPECT_EQ(output.x_m, 0.1);
  EXPECT_EQ(output.y_m, -2.8);
  EXPECT_EQ(output.yaw_rad, 1.0 / 3.0);
}

TEST(Serialization, normalizes_yaw_on_encode_and_decode) {
  const Pose2d input{1.0, 2.0, 1.5 * kPi};
  const Pose2d output = decode_document<Pose2d>(encode_document(input));
  EXPECT_NEAR(output.yaw_rad, -0.5 * kPi, 1e-12);
  EXPECT_GE(output.yaw_rad, -kPi);
  EXPECT_LT(output.yaw_rad, kPi);

  const json document = {{"schema_version", 1},
                         {"type", "pose2d"},
                         {"x_m", 0.0},
                         {"y_m", 0.0},
                         {"yaw_rad", 2.5 * kPi}};
  const Pose2d decoded = decode_document<Pose2d>(document.dump());
  EXPECT_NEAR(decoded.yaw_rad, 0.5 * kPi, 1e-12);
}

TEST(Serialization, round_trips_vehicle_model) {
  const VehicleModel input = test_vehicle();
  const VehicleModel output = decode_document<VehicleModel>(encode_document(input));
  EXPECT_EQ(output, input);
  EXPECT_DOUBLE_EQ(output.wheelbase_m, 2.8);
}

TEST(Serialization, round_trips_grid_view) {
  const GridView input = test_grid_view();
  const GridView output = decode_document<GridView>(encode_document(input));
  EXPECT_EQ(output, input);
  EXPECT_EQ(output.cells, input.cells);
}

TEST(Serialization, round_trips_path_point_and_segment) {
  const PathPoint point = test_path_point(1.5, Direction::kReverse);
  EXPECT_EQ(decode_document<PathPoint>(encode_document(point)), point);

  PathSegment segment;
  segment.points.push_back(test_path_point(0.0, Direction::kForward));
  segment.points.push_back(test_path_point(1.0, Direction::kForward));
  segment.points.push_back(test_path_point(2.5, Direction::kForward));
  const PathSegment output = decode_document<PathSegment>(encode_document(segment));
  EXPECT_EQ(output, segment);
}

TEST(Serialization, round_trips_trajectory_point) {
  const TrajectoryPoint input = test_trajectory_point(0.5);
  const TrajectoryPoint output = decode_document<TrajectoryPoint>(encode_document(input));
  EXPECT_EQ(output, input);
}

TEST(Serialization, round_trips_plan_request_with_verbatim_config) {
  PlanRequest request;
  request.request_id = "req_1";
  request.map_id = "map_1";
  request.map_version = "v1";
  request.vehicle = test_vehicle();
  request.start = Pose2d{0.0, 0.0, 0.0};
  request.goal = Pose2d{10.0, 5.0, 0.5};
  request.config = json{{"planner", {{"heuristic", "euclidean"}, {"weight", 1.5}}},
                        {"flags", json::array({true, false})}};
  request.time_budget_s = 5.0;
  request.node_budget = 1000000;
  request.cancelled = true;

  const PlanRequest output = decode_document<PlanRequest>(encode_document(request));
  EXPECT_EQ(output, request);
  EXPECT_EQ(output.config, request.config);
}

TEST(Serialization, round_trips_plan_result) {
  PlanResult result;
  result.request_id = "req_42";
  result.status = Status::kSuccess;
  result.reason = "goal_reached";
  result.path.push_back(test_segment(Direction::kForward));
  result.path.push_back(test_segment(Direction::kReverse));
  result.trajectory.push_back(test_trajectory_point(0.0));
  result.trajectory.push_back(test_trajectory_point(0.5));
  result.statistics = json{{"expanded_nodes", 7}, {"planning_time_s", 0.125}};

  const PlanResult output = decode_document<PlanResult>(encode_document(result));
  EXPECT_EQ(output, result);
  EXPECT_EQ(output.statistics, result.statistics);
}

TEST(Serialization, round_trips_all_status_values) {
  const Status statuses[] = {Status::kSuccess,   Status::kInvalidInput, Status::kNoPath,
                             Status::kTimeout,   Status::kCancelled,    Status::kOptimizationFailed,
                             Status::kValidationFailed};
  for (Status status : statuses) {
    EXPECT_EQ(decode_document<Status>(encode_document(status)), status);
  }
}

TEST(Serialization, parses_enum_strings_strictly) {
  EXPECT_EQ(direction_from_string("forward"), Direction::kForward);
  EXPECT_EQ(direction_from_string("reverse"), Direction::kReverse);
  EXPECT_EQ(status_from_string("success"), Status::kSuccess);
  EXPECT_EQ(status_from_string("invalid_input"), Status::kInvalidInput);
  EXPECT_EQ(status_from_string("no_path"), Status::kNoPath);
  EXPECT_EQ(status_from_string("timeout"), Status::kTimeout);
  EXPECT_EQ(status_from_string("cancelled"), Status::kCancelled);
  EXPECT_EQ(status_from_string("optimization_failed"), Status::kOptimizationFailed);
  EXPECT_EQ(status_from_string("validation_failed"), Status::kValidationFailed);
  EXPECT_THROW(direction_from_string("diagonal"), SerializationError);
  EXPECT_THROW(status_from_string("done"), SerializationError);
}

TEST(Serialization, round_trips_corridor) {
  const Corridor input = test_corridor();
  const Corridor output = decode_document<Corridor>(encode_document(input));
  EXPECT_EQ(output, input);
}

TEST(Serialization, preserves_double_precision_for_representative_values) {
  const double values[] = {0.0, 0.1, 1.0 / 3.0, 2.8, -9.87654321e-7, 1.2345678901234567, 1e-15, 1e15};
  for (double value : values) {
    const Pose2d input{value, -value, 0.0};
    const Pose2d output = decode_document<Pose2d>(encode_document(input));
    EXPECT_EQ(output.x_m, value) << "value: " << value;
    EXPECT_EQ(output.y_m, -value) << "value: " << value;
  }
}

TEST(Serialization, accepts_unknown_optional_fields) {
  const std::string text = R"json({
    "schema_version": 1,
    "type": "vehicle_model",
    "id": "vehicle_a",
    "wheelbase_m": 2.8,
    "width_m": 1.8,
    "front_overhang_m": 0.9,
    "rear_overhang_m": 1.0,
    "max_steer_rad": 0.5,
    "safety_margin_m": 0.1,
    "footprint_shape": "rectangle",
    "future": {"nested": [1, 2, 3], "flag": true}
  })json";
  const VehicleModel vehicle = decode_document<VehicleModel>(text);
  EXPECT_EQ(vehicle.id, "vehicle_a");
  EXPECT_DOUBLE_EQ(vehicle.wheelbase_m, 2.8);

  const Pose2d pose = decode_document<Pose2d>(
      R"json({"schema_version": 1, "type": "pose2d", "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.5, "extra": null})json");
  EXPECT_EQ(pose, (Pose2d{1.0, 2.0, 0.5}));
}

TEST(Serialization, plan_result_defaults_optional_fields_when_absent) {
  const std::string text =
      R"json({"schema_version": 1, "type": "plan_result", "request_id": "req_1", "status": "no_path"})json";
  const PlanResult result = decode_document<PlanResult>(text);
  EXPECT_EQ(result.request_id, "req_1");
  EXPECT_EQ(result.status, Status::kNoPath);
  EXPECT_TRUE(result.reason.empty());
  EXPECT_TRUE(result.path.empty());
  EXPECT_TRUE(result.trajectory.empty());
  EXPECT_TRUE(result.statistics.is_object());
  EXPECT_TRUE(result.statistics.empty());
}

TEST(Serialization, encode_document_emits_envelope_and_known_fields_only) {
  const std::string text = encode_document(Pose2d{1.0, 2.0, 0.5});
  const json document = json::parse(text);
  EXPECT_EQ(document.at("schema_version").get<int>(), 1);
  EXPECT_EQ(document.at("type").get<std::string>(), "pose2d");
  EXPECT_DOUBLE_EQ(document.at("x_m").get<double>(), 1.0);
  EXPECT_DOUBLE_EQ(document.at("yaw_rad").get<double>(), 0.5);
  EXPECT_EQ(document.size(), 5U);  // 3 个载荷字段 + schema_version + type
}

TEST(Serialization, payload_helpers_work_without_envelope) {
  const json payload = encode_payload(Pose2d{1.0, 2.0, 0.25});
  EXPECT_TRUE(payload.is_object());
  EXPECT_FALSE(payload.contains("schema_version"));
  EXPECT_FALSE(payload.contains("type"));
  EXPECT_DOUBLE_EQ(payload.at("x_m").get<double>(), 1.0);
  const Pose2d decoded = decode_payload<Pose2d>(payload);
  EXPECT_EQ(decoded, (Pose2d{1.0, 2.0, 0.25}));
}

TEST(SerializationRejection, rejects_malformed_json) {
  expect_document_error<Pose2d>("{", ErrorCode::kParseError);
  expect_document_error<Pose2d>("not json", ErrorCode::kParseError);
  expect_document_error<Pose2d>(R"json({"schema_version": 1, "type": "pose2d", "x_m": NaN})json",
                                ErrorCode::kParseError);
}

TEST(SerializationRejection, rejects_non_object_root) {
  expect_document_error<Pose2d>("[]", ErrorCode::kInvalidDocument);
  expect_document_error<Pose2d>("42", ErrorCode::kInvalidDocument);
  expect_document_error<Pose2d>("\"text\"", ErrorCode::kInvalidDocument);
}

TEST(SerializationRejection, rejects_missing_schema_version) {
  expect_document_error<Pose2d>(
      R"json({"type": "pose2d", "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0})json", ErrorCode::kMissingField);
}

TEST(SerializationRejection, rejects_unknown_schema_version) {
  expect_document_error<Pose2d>(
      R"json({"schema_version": 2, "type": "pose2d", "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0})json",
      ErrorCode::kUnsupportedSchemaVersion);
  expect_document_error<Pose2d>(
      R"json({"schema_version": 0, "type": "pose2d", "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0})json",
      ErrorCode::kUnsupportedSchemaVersion);
}

TEST(SerializationRejection, rejects_wrong_schema_version_type) {
  expect_document_error<Pose2d>(
      R"json({"schema_version": "1", "type": "pose2d", "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0})json",
      ErrorCode::kTypeError);
  expect_document_error<Pose2d>(
      R"json({"schema_version": 1.0, "type": "pose2d", "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0})json",
      ErrorCode::kTypeError);
}

TEST(SerializationRejection, rejects_unknown_or_missing_type) {
  expect_document_error<Pose2d>(
      R"json({"schema_version": 1, "type": "route", "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0})json",
      ErrorCode::kUnknownType);
  expect_document_error<Pose2d>(
      R"json({"schema_version": 1, "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.0})json", ErrorCode::kMissingField);
  // 另一种类型的合法文档不能解码为 Pose2d。
  expect_document_error<Pose2d>(encode_document(test_vehicle()), ErrorCode::kUnknownType);
}

TEST(SerializationRejection, rejects_missing_required_field) {
  expect_document_error<Pose2d>(R"json({"schema_version": 1, "type": "pose2d", "x_m": 1.0, "y_m": 2.0})json",
                                ErrorCode::kMissingField);
  expect_document_error<VehicleModel>(
      R"json({"schema_version": 1, "type": "vehicle_model", "id": "v", "width_m": 1.8,
              "front_overhang_m": 0.9, "rear_overhang_m": 1.0, "max_steer_rad": 0.5,
              "safety_margin_m": 0.1})json",
      ErrorCode::kMissingField);
  expect_document_error<PlanResult>(
      R"json({"schema_version": 1, "type": "plan_result", "status": "success"})json", ErrorCode::kMissingField);
}

TEST(SerializationRejection, rejects_wrong_field_type) {
  expect_document_error<Pose2d>(
      R"json({"schema_version": 1, "type": "pose2d", "x_m": "1.0", "y_m": 2.0, "yaw_rad": 0.0})json",
      ErrorCode::kTypeError);
  expect_document_error<VehicleModel>(
      R"json({"schema_version": 1, "type": "vehicle_model", "id": 12345, "wheelbase_m": 2.8,
              "width_m": 1.8, "front_overhang_m": 0.9, "rear_overhang_m": 1.0,
              "max_steer_rad": 0.5, "safety_margin_m": 0.1})json",
      ErrorCode::kTypeError);
  expect_document_error<GridView>(
      R"json({"schema_version": 1, "type": "grid_view", "map_id": "m", "map_version": "v1",
              "width": "4", "height": 3, "resolution_m": 0.1,
              "origin": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
              "cells": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]})json",
      ErrorCode::kTypeError);
  expect_document_error<PlanRequest>(
      R"json({"schema_version": 1, "type": "plan_request", "request_id": "r", "map_id": "m",
              "map_version": "v1",
              "vehicle": {"id": "v", "wheelbase_m": 2.8, "width_m": 1.8, "front_overhang_m": 0.9,
                          "rear_overhang_m": 1.0, "max_steer_rad": 0.5, "safety_margin_m": 0.1},
              "start": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
              "goal": {"x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.0},
              "time_budget_s": "5s"})json",
      ErrorCode::kTypeError);
}

TEST(SerializationRejection, rejects_unknown_enum_values) {
  expect_document_error<PathPoint>(
      R"json({"schema_version": 1, "type": "path_point",
              "pose": {"x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.0},
              "s_m": 0.0, "curvature_inv_m": 0.0, "direction": "backward"})json",
      ErrorCode::kUnknownEnumValue);
  expect_document_error<PlanResult>(
      R"json({"schema_version": 1, "type": "plan_result", "request_id": "r", "status": "ok"})json",
      ErrorCode::kUnknownEnumValue);
  expect_document_error<Status>(R"json({"schema_version": 1, "type": "status", "status": "done"})json",
                                ErrorCode::kUnknownEnumValue);
}

TEST(SerializationRejection, rejects_nonfinite_numbers_on_decode) {
  expect_document_error<Pose2d>(
      R"json({"schema_version": 1, "type": "pose2d", "x_m": 1e999, "y_m": 0.0, "yaw_rad": 0.0})json",
      ErrorCode::kNonFiniteValue);
}

TEST(SerializationRejection, rejects_nonfinite_numbers_on_encode) {
  expect_encode_error(Pose2d{kNaN, 0.0, 0.0}, ErrorCode::kNonFiniteValue);
  expect_encode_error(Pose2d{0.0, 0.0, kInf}, ErrorCode::kNonFiniteValue);

  VehicleModel vehicle = test_vehicle();
  vehicle.wheelbase_m = kNaN;
  expect_encode_error(vehicle, ErrorCode::kNonFiniteValue);

  GridView grid = test_grid_view();
  grid.resolution_m = kNaN;
  expect_encode_error(grid, ErrorCode::kNonFiniteValue);

  PathPoint point = test_path_point(0.0, Direction::kForward);
  point.s_m = kNaN;
  expect_encode_error(point, ErrorCode::kNonFiniteValue);

  TrajectoryPoint trajectory_point = test_trajectory_point(0.0);
  trajectory_point.velocity_mps = kInf;
  expect_encode_error(trajectory_point, ErrorCode::kNonFiniteValue);

  PlanRequest request;
  request.request_id = "r";
  request.map_id = "m";
  request.map_version = "v1";
  request.vehicle = test_vehicle();
  request.start = Pose2d{0.0, 0.0, 0.0};
  request.goal = Pose2d{1.0, 0.0, 0.0};
  request.time_budget_s = kNaN;
  expect_encode_error(request, ErrorCode::kNonFiniteValue);

  request.time_budget_s = 1.0;
  request.config = json{{"bad", kNaN}};
  expect_encode_error(request, ErrorCode::kNonFiniteValue);

  PlanResult result;
  result.request_id = "r";
  result.status = Status::kSuccess;
  result.statistics["bad"] = kNaN;
  expect_encode_error(result, ErrorCode::kNonFiniteValue);

  Corridor corridor = test_corridor();
  corridor.polygons[0].vertices[0].x_m = kInf;
  expect_encode_error(corridor, ErrorCode::kNonFiniteValue);
}

TEST(SerializationRejection, rejects_invalid_grid_invariants) {
  GridView grid = test_grid_view();
  grid.cells.pop_back();
  expect_encode_error(grid, ErrorCode::kInvalidValue);

  grid = test_grid_view();
  grid.cells[2] = 5;
  expect_encode_error(grid, ErrorCode::kInvalidValue);

  grid = test_grid_view();
  grid.resolution_m = 0.0;
  expect_encode_error(grid, ErrorCode::kInvalidValue);

  grid = test_grid_view();
  grid.map_id.clear();
  expect_encode_error(grid, ErrorCode::kInvalidValue);
}

TEST(SerializationRejection, rejects_invalid_path_segment_invariants) {
  PathSegment segment = test_segment(Direction::kForward);
  segment.points[1].s_m = 0.0;
  expect_encode_error(segment, ErrorCode::kInvalidValue);

  segment = test_segment(Direction::kForward);
  segment.points[1].direction = Direction::kReverse;
  expect_encode_error(segment, ErrorCode::kInvalidValue);

  const PathSegment empty;
  expect_encode_error(empty, ErrorCode::kInvalidValue);
}

TEST(SerializationRejection, rejects_invalid_trajectory_time_order) {
  PlanResult result;
  result.request_id = "r";
  result.status = Status::kSuccess;
  result.trajectory.push_back(test_trajectory_point(0.0));
  result.trajectory.push_back(test_trajectory_point(0.0));
  expect_encode_error(result, ErrorCode::kInvalidValue);
}

TEST(SerializationRejection, rejects_invalid_corridor_invariants) {
  Corridor corridor = test_corridor();
  corridor.polygons[0].vertices.resize(2);
  expect_encode_error(corridor, ErrorCode::kInvalidValue);

  corridor = test_corridor();
  corridor.polygons[0].s_end_m = corridor.polygons[0].s_start_m;
  expect_encode_error(corridor, ErrorCode::kInvalidValue);

  corridor = test_corridor();
  corridor.frame_id.clear();
  expect_encode_error(corridor, ErrorCode::kInvalidValue);

  corridor = test_corridor();
  CorridorPolygon later;
  later.s_start_m = 3.0;
  later.s_end_m = 4.0;
  later.vertices = {Point2d{3.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 1.0}};
  corridor.polygons.insert(corridor.polygons.begin(), later);  // 第一个从 3.0 开始，第二个从 0.0 开始
  expect_encode_error(corridor, ErrorCode::kInvalidValue);
}

TEST(SerializationRejection, rejects_empty_identifiers) {
  VehicleModel vehicle = test_vehicle();
  vehicle.id.clear();
  expect_encode_error(vehicle, ErrorCode::kInvalidValue);

  PlanRequest request;
  request.request_id.clear();
  request.map_id = "m";
  request.map_version = "v1";
  request.vehicle = test_vehicle();
  request.start = Pose2d{0.0, 0.0, 0.0};
  request.goal = Pose2d{1.0, 0.0, 0.0};
  expect_encode_error(request, ErrorCode::kInvalidValue);
}

}  // namespace
}  // namespace planner::common
