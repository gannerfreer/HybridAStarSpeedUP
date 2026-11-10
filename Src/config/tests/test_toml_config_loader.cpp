/**
 * @file test_toml_config_loader.cpp
 * @brief TOML 配置加载、严格键校验与端到端规划流程测试。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "collision_checker.hpp"
#include "planner/common/plan.hpp"
#include "planner/common/planner_config.hpp"
#include "planner/common/scenario_config.hpp"
#include "planner/common/status.hpp"
#include "planner/common/vehicle.hpp"
#include "planner/config/config_loader.hpp"
#include "planner/math/pose2d.hpp"
#include "scene.hpp"
#include "user_astar.hpp"

namespace {

using namespace planner;

std::string data_path(const std::string& name) {
  return std::string(PLANNER_CONFIG_TEST_DATA_DIR) + "/" + name;
}

std::string example_path(const std::string& name) {
  return std::string(PLANNER_CONFIG_EXAMPLE_DIR) + "/" + name;
}

TEST(VehicleConfig, LoadsValidValues) {
  const auto result = config::load_vehicle_config(data_path("valid_vehicle.toml"));
  ASSERT_TRUE(result) << result.error.message;
  EXPECT_EQ(result.value->id, "test_sedan");
  EXPECT_DOUBLE_EQ(result.value->wheelbase_m, 2.7);
  EXPECT_DOUBLE_EQ(result.value->width_m, 1.85);
  EXPECT_DOUBLE_EQ(result.value->front_overhang_m, 0.9);
  EXPECT_DOUBLE_EQ(result.value->rear_overhang_m, 0.85);
  EXPECT_DOUBLE_EQ(result.value->max_steer_rad, 0.55);
  EXPECT_DOUBLE_EQ(result.value->safety_margin_m, 0.12);
}

TEST(PlannerConfig, LoadsNestedTables) {
  const auto result = config::load_planner_config(data_path("valid_planner.toml"));
  ASSERT_TRUE(result) << result.error.message;
  EXPECT_DOUBLE_EQ(result.value->xy_resolution_m, 0.4);
  EXPECT_EQ(result.value->heading_bins, 96);
  EXPECT_DOUBLE_EQ(result.value->motion_step_m, 0.8);
  EXPECT_EQ(result.value->steering_samples, 7);
  EXPECT_FALSE(result.value->allow_reverse);
  EXPECT_DOUBLE_EQ(result.value->backward_search_range_m, 3.5);
  EXPECT_DOUBLE_EQ(result.value->linear_preferred_distance_m2, 4.0);
  EXPECT_DOUBLE_EQ(result.value->goal_capture_radius_m, 6.0);
  EXPECT_DOUBLE_EQ(result.value->forward_cost, 1.2);
  EXPECT_DOUBLE_EQ(result.value->reverse_cost, 2.5);
  EXPECT_DOUBLE_EQ(result.value->turn_penalty, 1.5);
  EXPECT_DOUBLE_EQ(result.value->switch_cost, 3.0);
  EXPECT_EQ(result.value->analytic_expansion_ratio, 4);
  EXPECT_DOUBLE_EQ(result.value->analytic_shot_distance_m, 10.5);
  EXPECT_DOUBLE_EQ(result.value->analytic_expansion_max_length_m, 15.0);
  EXPECT_DOUBLE_EQ(result.value->min_connector_segment_m, 0.2);
  EXPECT_DOUBLE_EQ(result.value->default_time_budget_s, 2.5);
  EXPECT_EQ(result.value->default_node_budget, 1000000);
}

TEST(PlannerConfig, UsesDefaultsWhenFieldsAreOmitted) {
  const common::UserAstarConfig defaults;
  const auto result = config::load_planner_config(data_path("valid_planner_minimal.toml"));
  ASSERT_TRUE(result) << result.error.message;
  EXPECT_DOUBLE_EQ(result.value->xy_resolution_m, defaults.xy_resolution_m);
  EXPECT_EQ(result.value->heading_bins, defaults.heading_bins);
  EXPECT_DOUBLE_EQ(result.value->motion_step_m, defaults.motion_step_m);
  EXPECT_EQ(result.value->steering_samples, defaults.steering_samples);
  EXPECT_EQ(result.value->allow_reverse, defaults.allow_reverse);
  EXPECT_DOUBLE_EQ(result.value->goal_capture_radius_m, defaults.goal_capture_radius_m);
  EXPECT_DOUBLE_EQ(result.value->default_time_budget_s, defaults.default_time_budget_s);
  EXPECT_EQ(result.value->default_node_budget, defaults.default_node_budget);
}

TEST(ScenarioConfig, LoadsMapPosesAndObstacles) {
  const auto result = config::load_scenario_config(data_path("valid_scenario.toml"));
  ASSERT_TRUE(result) << result.error.message;
  EXPECT_EQ(result.value->width, 40);
  EXPECT_EQ(result.value->height, 30);
  EXPECT_DOUBLE_EQ(result.value->resolution_m, 0.25);
  EXPECT_DOUBLE_EQ(result.value->origin.x_m, 1.5);
  EXPECT_DOUBLE_EQ(result.value->origin.y_m, -2.0);
  EXPECT_DOUBLE_EQ(result.value->origin.yaw_rad, 0.1);
  EXPECT_DOUBLE_EQ(result.value->start.x_m, 4.0);
  EXPECT_DOUBLE_EQ(result.value->start.y_m, 5.0);
  EXPECT_DOUBLE_EQ(result.value->start.yaw_rad, 0.2);
  EXPECT_DOUBLE_EQ(result.value->goal.x_m, 8.0);
  EXPECT_DOUBLE_EQ(result.value->goal.y_m, 9.0);
  EXPECT_DOUBLE_EQ(result.value->goal.yaw_rad, -0.3);
  ASSERT_EQ(result.value->obstacles.size(), 2u);
  EXPECT_EQ(result.value->obstacles[0].id, "box");
  ASSERT_EQ(result.value->obstacles[0].vertices.size(), 4u);
  EXPECT_DOUBLE_EQ(result.value->obstacles[0].vertices[0].x_m, 2.0);
  EXPECT_DOUBLE_EQ(result.value->obstacles[0].vertices[2].y_m, 3.0);
  EXPECT_EQ(result.value->obstacles[1].id, "triangle");
  ASSERT_EQ(result.value->obstacles[1].vertices.size(), 3u);
}

TEST(ConfigErrors, MissingFileAndSyntaxError) {
  EXPECT_NO_THROW({
    const auto missing = config::load_vehicle_config(data_path("no_such.toml"));
    EXPECT_FALSE(missing);
    EXPECT_EQ(missing.error.file, data_path("no_such.toml"));
    EXPECT_FALSE(missing.error.message.empty());
  });

  const auto syntax = config::load_vehicle_config(data_path("syntax_error.toml"));
  ASSERT_FALSE(syntax);
  EXPECT_EQ(syntax.error.status, common::Status::kInvalidInput);
  EXPECT_EQ(syntax.error.line, 2);
  EXPECT_GT(syntax.error.column, 0);
  EXPECT_TRUE(syntax.error.key_path.empty());
  EXPECT_NE(syntax.error.message.find("parse error"), std::string::npos);
  EXPECT_NE(syntax.error.message.find("syntax_error.toml"), std::string::npos);
}

TEST(ConfigErrors, UnknownKeysAreRejectedByDefault) {
  const auto result = config::load_vehicle_config(data_path("unknown_key.toml"));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error.status, common::Status::kInvalidInput);
  EXPECT_EQ(result.error.key_path, "bogus_key");
  EXPECT_EQ(result.error.line, 8);
  EXPECT_EQ(result.error.column, 1);
  EXPECT_NE(result.error.message.find("unknown key"), std::string::npos);
  EXPECT_NE(result.error.message.find("bogus_key"), std::string::npos);

  const auto nested = config::load_planner_config(data_path("unknown_nested_key_planner.toml"));
  ASSERT_FALSE(nested);
  EXPECT_EQ(nested.error.key_path, "search.bogus");
  EXPECT_EQ(nested.error.line, 5);
  EXPECT_EQ(nested.error.column, 1);
}

TEST(ConfigOptions, AllowUnknownKeysSkipsUnknownEntries) {
  config::ConfigLoadOptions options;
  options.allow_unknown_keys = true;

  const auto vehicle = config::load_vehicle_config(data_path("unknown_key.toml"), options);
  ASSERT_TRUE(vehicle) << vehicle.error.message;
  EXPECT_EQ(vehicle.value->id, "unknown");

  const auto planner =
      config::load_planner_config(data_path("unknown_nested_key_planner.toml"), options);
  ASSERT_TRUE(planner) << planner.error.message;
  EXPECT_DOUBLE_EQ(planner.value->xy_resolution_m, 0.5);

  const auto strict = config::load_vehicle_config(data_path("unknown_key.toml"));
  EXPECT_FALSE(strict);
}

TEST(ConfigErrors, MissingRequiredAndTypeErrors) {
  const auto missing = config::load_vehicle_config(data_path("missing_vehicle_field.toml"));
  ASSERT_FALSE(missing);
  EXPECT_EQ(missing.error.key_path, "wheelbase_m");
  EXPECT_GT(missing.error.line, 0);
  EXPECT_NE(missing.error.message.find("missing required key"), std::string::npos);

  const auto typed = config::load_vehicle_config(data_path("type_error_vehicle.toml"));
  ASSERT_FALSE(typed);
  EXPECT_EQ(typed.error.key_path, "wheelbase_m");
  EXPECT_EQ(typed.error.line, 3);
  EXPECT_GT(typed.error.column, 0);
  EXPECT_NE(typed.error.message.find("expected a float or integer"), std::string::npos);
}

TEST(ConfigErrors, StrictBooleanStringAndIntegerTypes) {
  const auto bool_error = config::load_vehicle_config(data_path("type_error_bool.toml"));
  ASSERT_FALSE(bool_error);
  EXPECT_EQ(bool_error.error.key_path, "width_m");
  EXPECT_EQ(bool_error.error.line, 4);
  EXPECT_NE(bool_error.error.message.find("expected a float or integer"), std::string::npos);

  const auto int_error = config::load_planner_config(data_path("type_error_int.toml"));
  ASSERT_FALSE(int_error);
  EXPECT_EQ(int_error.error.key_path, "search.heading_bins");
  EXPECT_EQ(int_error.error.line, 4);
  EXPECT_NE(int_error.error.message.find("expected an integer"), std::string::npos);

  const auto string_error = config::load_vehicle_config(data_path("type_error_string.toml"));
  ASSERT_FALSE(string_error);
  EXPECT_EQ(string_error.error.key_path, "id");
  EXPECT_EQ(string_error.error.line, 2);
  EXPECT_NE(string_error.error.message.find("expected a string"), std::string::npos);
}

TEST(ConfigErrors, DuplicateKeysAreRejectedByParser) {
  const auto result = config::load_vehicle_config(data_path("duplicate_key.toml"));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error.line, 2);
  EXPECT_NE(result.error.message.find("parse error"), std::string::npos);
}

TEST(ConfigErrors, SchemaVersionIsRequiredAndMustBeOne) {
  const auto missing = config::load_vehicle_config(data_path("schema_version_missing.toml"));
  ASSERT_FALSE(missing);
  EXPECT_EQ(missing.error.key_path, "schema_version");
  EXPECT_GT(missing.error.line, 0);

  const auto future = config::load_vehicle_config(data_path("schema_version_two.toml"));
  ASSERT_FALSE(future);
  EXPECT_EQ(future.error.key_path, "schema_version");
  EXPECT_EQ(future.error.line, 1);
  EXPECT_NE(future.error.message.find("schema_version must be 1"), std::string::npos);
}

TEST(ConfigErrors, RangeAndFiniteValidation) {
  const auto range = config::load_vehicle_config(data_path("range_vehicle.toml"));
  ASSERT_FALSE(range);
  EXPECT_EQ(range.error.status, common::Status::kInvalidInput);
  EXPECT_EQ(range.error.key_path, "max_steer_rad");
  EXPECT_EQ(range.error.line, 7);

  const auto nan = config::load_vehicle_config(data_path("nonfinite_vehicle.toml"));
  ASSERT_FALSE(nan);
  EXPECT_EQ(nan.error.key_path, "wheelbase_m");
  EXPECT_EQ(nan.error.line, 3);

  const auto inf = config::load_planner_config(data_path("nonfinite_planner.toml"));
  ASSERT_FALSE(inf);
  EXPECT_EQ(inf.error.key_path, "search.xy_resolution_m");
  EXPECT_EQ(inf.error.line, 4);
  EXPECT_NE(inf.error.message.find("finite"), std::string::npos);
}

TEST(ConfigErrors, NonFiniteScenarioValuesPointToMemberNodes) {
  struct Case {
    const char* file;
    const char* key_path;
    int line;
    int column;
  };
  const Case cases[] = {
      {"nonfinite_scenario_origin.toml", "map.origin.x_m", 9, 7},
      {"nonfinite_scenario_start.toml", "start.y_m", 10, 7},
      {"nonfinite_scenario_goal.toml", "goal.yaw_rad", 15, 11},
      {"nonfinite_scenario_vertex.toml", "obstacles[0].vertices[2].x_m", 21, 11},
  };
  for (const Case& test_case : cases) {
    SCOPED_TRACE(test_case.file);
    const auto result = config::load_scenario_config(data_path(test_case.file));
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error.key_path, test_case.key_path);
    EXPECT_EQ(result.error.line, test_case.line);
    EXPECT_EQ(result.error.column, test_case.column);
    EXPECT_NE(result.error.message.find("finite"), std::string::npos);
  }
}

TEST(ConfigOptions, AllowUnknownKeysSkipsNestedScenarioEntries) {
  const auto strict_obstacle =
      config::load_scenario_config(data_path("unknown_nested_scenario_obstacle.toml"));
  ASSERT_FALSE(strict_obstacle);
  EXPECT_EQ(strict_obstacle.error.key_path, "obstacles[0].bogus");
  EXPECT_EQ(strict_obstacle.error.line, 18);

  const auto strict_vertex =
      config::load_scenario_config(data_path("unknown_nested_scenario_vertex.toml"));
  ASSERT_FALSE(strict_vertex);
  EXPECT_EQ(strict_vertex.error.key_path, "obstacles[0].vertices[0].z_m");
  EXPECT_EQ(strict_vertex.error.line, 19);

  config::ConfigLoadOptions options;
  options.allow_unknown_keys = true;
  const auto obstacle =
      config::load_scenario_config(data_path("unknown_nested_scenario_obstacle.toml"), options);
  ASSERT_TRUE(obstacle) << obstacle.error.message;
  ASSERT_EQ(obstacle.value->obstacles.size(), 1u);
  EXPECT_EQ(obstacle.value->obstacles[0].id, "box");

  const auto vertex =
      config::load_scenario_config(data_path("unknown_nested_scenario_vertex.toml"), options);
  ASSERT_TRUE(vertex) << vertex.error.message;
  ASSERT_EQ(vertex.value->obstacles.size(), 1u);
  EXPECT_EQ(vertex.value->obstacles[0].vertices.size(), 4u);
}

TEST(ConfigErrors, ScenarioDimensionsAndObstaclesAreValidated) {
  const auto dimensions =
      config::load_scenario_config(data_path("invalid_scenario_dimensions.toml"));
  ASSERT_FALSE(dimensions);
  EXPECT_EQ(dimensions.error.key_path, "map.width");
  EXPECT_EQ(dimensions.error.line, 4);
  EXPECT_NE(dimensions.error.message.find("must be > 0"), std::string::npos);

  const auto obstacle = config::load_scenario_config(data_path("invalid_scenario_obstacle.toml"));
  ASSERT_FALSE(obstacle);
  EXPECT_EQ(obstacle.error.key_path, "obstacles[0].vertices");
  EXPECT_GT(obstacle.error.line, 0);
  EXPECT_NE(obstacle.error.message.find("invalid convex polygon"), std::string::npos);
}

TEST(ScenarioConfigValidation, CentralizesRangePoseAndObstacleChecks) {
  common::ScenarioConfig scenario;
  scenario.width = 10;
  scenario.height = 10;
  scenario.resolution_m = 0.5;
  scenario.start = math::Pose2d{1.0, 1.0, 0.0};
  scenario.goal = math::Pose2d{5.0, 5.0, 0.0};
  common::ObstacleConfig obstacle;
  obstacle.id = "box";
  obstacle.vertices = {math::Point2d{2.0, 2.0}, math::Point2d{3.0, 2.0}, math::Point2d{3.0, 3.0},
                       math::Point2d{2.0, 3.0}};
  scenario.obstacles.push_back(obstacle);
  EXPECT_TRUE(common::scenario_config_valid(scenario));

  scenario.width = 0;
  EXPECT_FALSE(common::scenario_config_valid(scenario));
  scenario.width = 10;
  scenario.height = 0;
  EXPECT_FALSE(common::scenario_config_valid(scenario));
  scenario.height = 10;
  scenario.resolution_m = 0.0;
  EXPECT_FALSE(common::scenario_config_valid(scenario));
  scenario.resolution_m = 0.5;
  scenario.origin.x_m = std::nan("");
  EXPECT_FALSE(common::scenario_config_valid(scenario));
  scenario.origin = math::Pose2d{};
  scenario.start.y_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(common::scenario_config_valid(scenario));
  scenario.start = math::Pose2d{1.0, 1.0, 0.0};
  scenario.goal.yaw_rad = std::nan("");
  EXPECT_FALSE(common::scenario_config_valid(scenario));
  scenario.goal = math::Pose2d{5.0, 5.0, 0.0};

  scenario.obstacles[0].id.clear();
  EXPECT_FALSE(common::scenario_config_valid(scenario));
  scenario.obstacles[0].id = "box";
  scenario.obstacles[0].vertices.clear();
  EXPECT_FALSE(common::scenario_config_valid(scenario));
  scenario.obstacles[0].vertices = {math::Point2d{0.0, 0.0}, math::Point2d{1.0, 1.0},
                                    math::Point2d{2.0, 2.0}};
  EXPECT_FALSE(common::scenario_config_valid(scenario));
}

TEST(SceneConfig, MakeSceneBuildsGridAndObstacles) {
  const auto scenario = config::load_scenario_config(data_path("valid_scenario.toml"));
  ASSERT_TRUE(scenario) << scenario.error.message;
  collision::Scene scene = collision::make_scene(*scenario.value);
  ASSERT_TRUE(scene.valid());
  EXPECT_EQ(scene.grid.map_id, "scenario");
  EXPECT_EQ(scene.grid.map_version, "1");
  EXPECT_EQ(scene.grid.width, 40);
  EXPECT_EQ(scene.grid.height, 30);
  EXPECT_DOUBLE_EQ(scene.grid.resolution_m, 0.25);
  EXPECT_EQ(scene.grid.origin, scenario.value->origin);
  EXPECT_EQ(scene.grid.cells.size(), 40u * 30u);
  for (int cell : scene.grid.cells) {
    EXPECT_EQ(cell, common::kCellFree);
  }
  ASSERT_EQ(scene.obstacles.size(), 2u);
  EXPECT_EQ(scene.obstacles[0].id, "box");
  EXPECT_EQ(scene.obstacles[1].id, "triangle");
}

TEST(SceneConfig, MakeSceneRejectsInvalidInput) {
  common::ScenarioConfig scenario;
  scenario.width = 0;
  scenario.height = 10;
  scenario.resolution_m = 0.5;
  EXPECT_FALSE(collision::make_scene(scenario).valid());

  scenario.width = 10;
  scenario.resolution_m = 0.0;
  EXPECT_FALSE(collision::make_scene(scenario).valid());

  scenario.resolution_m = 0.5;
  scenario.start.x_m = std::nan("");
  EXPECT_FALSE(collision::make_scene(scenario).valid());

  scenario.start = math::Pose2d{};
  common::ObstacleConfig obstacle;
  obstacle.id = "bad";
  obstacle.vertices = {math::Point2d{0.0, 0.0}, math::Point2d{1.0, 1.0}, math::Point2d{2.0, 2.0}};
  scenario.obstacles.push_back(obstacle);
  EXPECT_FALSE(collision::make_scene(scenario).valid());
}

TEST(ConfigPipeline, ExampleConfigsPlanSuccessfully) {
  const auto vehicle = config::load_vehicle_config(example_path("vehicles/standard_sedan.toml"));
  ASSERT_TRUE(vehicle) << vehicle.error.message;
  const auto planner_config = config::load_planner_config(example_path("planner/default.toml"));
  ASSERT_TRUE(planner_config) << planner_config.error.message;
  const auto scenario = config::load_scenario_config(example_path("scenarios/narrow_parking.toml"));
  ASSERT_TRUE(scenario) << scenario.error.message;

  collision::Scene scene = collision::make_scene(*scenario.value);
  ASSERT_TRUE(scene.valid());
  collision::CollisionChecker checker(*vehicle.value, scene);
  ASSERT_TRUE(checker.valid()) << checker.invalid_reason();

  common::PlanRequest request;
  request.request_id = "config_pipeline";
  request.map_id = scene.grid.map_id;
  request.map_version = scene.grid.map_version;
  request.vehicle = *vehicle.value;
  request.start = scenario.value->start;
  request.goal = scenario.value->goal;

  hybrid_astar::UserAstar planner(*planner_config.value);
  const common::PlanResult result = planner.plan(request, scene.grid, checker);
  ASSERT_EQ(result.status, common::Status::kSuccess)
      << common::to_string(result.status) << ": " << result.reason;
  ASSERT_FALSE(result.path.empty());
  ASSERT_FALSE(result.path.front().points.empty());
  ASSERT_FALSE(result.path.back().points.empty());
  EXPECT_TRUE(checker.check_path(result.path).is_clear());

  const double direct_distance =
      std::hypot(request.goal.x_m - request.start.x_m, request.goal.y_m - request.start.y_m);
  EXPECT_GT(result.path.back().points.back().s_m, direct_distance);
  EXPECT_LT(std::hypot(result.path.back().points.back().pose.x_m - request.goal.x_m,
                       result.path.back().points.back().pose.y_m - request.goal.y_m),
            0.05);
}

}  // namespace
