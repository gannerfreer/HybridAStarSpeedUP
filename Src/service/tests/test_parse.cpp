// POST /plan/submit 解析器的单元测试（不启动服务进程）。

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <nlohmann/json.hpp>

#include "planner/service/parse.hpp"
#include "planner/service/types.hpp"

#ifndef PLANNER_SERVICE_TEST_VEHICLE_PATH
#error "PLANNER_SERVICE_TEST_VEHICLE_PATH must be defined by the test CMakeLists"
#endif

namespace planner::service {
namespace {

using nlohmann::json;

json minimal_request() {
  return json{{"request_id", "req-1"},
              {"map_id", "mapping_example"},
              {"map_version", "abc"},
              {"start", json{{"x_m", 2.0}, {"y_m", 5.0}, {"yaw_rad", 0.0}}},
              {"goal", json{{"x_m", 18.0}, {"y_m", 5.0}, {"yaw_rad", 0.0}}}};
}

TEST(PlanRequestParse, MinimalRequestUsesSection71Defaults) {
  ServicePlanRequest request;
  const ParseOutcome outcome = parse_plan_request(minimal_request(), &request);
  ASSERT_TRUE(outcome.ok) << outcome.detail;
  EXPECT_EQ(request.request_id, "req-1");
  EXPECT_EQ(request.map_id, "mapping_example");
  EXPECT_EQ(request.map_version, "abc");
  EXPECT_EQ(request.vehicle, default_vehicle());
  EXPECT_EQ(request.params.hybrid_backend, "kth_normalized");
  EXPECT_DOUBLE_EQ(request.params.hybrid_time_budget_s, 5.0);
  EXPECT_EQ(request.params.hybrid_node_budget, 1000000);
  EXPECT_DOUBLE_EQ(request.params.dftpav_time_budget_s, 10.0);
  EXPECT_FALSE(request.params.dftpav_limits.has_value());
  EXPECT_TRUE(request.obstacles.empty());
  EXPECT_EQ(request.start.x_m, 2.0);
  EXPECT_EQ(request.goal.x_m, 18.0);
}

TEST(PlanRequestParse, NormalizesYawIntoPiRange) {
  json body = minimal_request();
  body["start"]["yaw_rad"] = 7.0;
  ServicePlanRequest request;
  const ParseOutcome outcome = parse_plan_request(body, &request);
  ASSERT_TRUE(outcome.ok) << outcome.detail;
  EXPECT_NEAR(request.start.yaw_rad, 7.0 - 2.0 * common::kPi, 1e-12);
}

TEST(PlanRequestParse, RejectsMissingOrInvalidRequiredFields) {
  ServicePlanRequest request;
  json body = minimal_request();
  body.erase("request_id");
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["request_id"] = "";
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["map_version"] = 3;
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body.erase("goal");
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["start"]["x_m"] = "two";
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["start"]["y_m"] = nullptr;
  EXPECT_FALSE(parse_plan_request(body, &request).ok);
}

TEST(PlanRequestParse, ToleratesUnknownTopLevelFields) {
  json body = minimal_request();
  body["future_field"] = json::object();
  ServicePlanRequest request;
  EXPECT_TRUE(parse_plan_request(body, &request).ok);
}

TEST(PlanRequestParse, RejectsUnknownAlgorithmParameters) {
  ServicePlanRequest request;
  json body = minimal_request();
  body["algorithm_params"] = json{{"bogus", 1}};
  const ParseOutcome outcome = parse_plan_request(body, &request);
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.reason, "unknown_algorithm_parameter");

  body = minimal_request();
  body["algorithm_params"] = json{{"corridor", json{{"bogus", 1}}}};
  EXPECT_EQ(parse_plan_request(body, &request).reason, "unknown_algorithm_parameter");

  body = minimal_request();
  body["algorithm_params"] = json{{"dftpav", json{{"bogus", 1}}}};
  EXPECT_EQ(parse_plan_request(body, &request).reason, "unknown_algorithm_parameter");

  body = minimal_request();
  body["algorithm_params"] =
      json{{"dftpav", json{{"options", json{{"bogus", 1}}}}}};
  EXPECT_EQ(parse_plan_request(body, &request).reason, "unknown_algorithm_parameter");
}

TEST(PlanRequestParse, ParsesAlgorithmParameters) {
  json body = minimal_request();
  body["algorithm_params"] = json{
      {"hybrid_backend", "ros_free"},
      {"hybrid", json{{"heading_bins", 36}}},
      {"time_budget_s", 2.5},
      {"node_budget", 1000},
      {"corridor", json{{"sample_step_m", 0.5}, {"clearance_m", 0.1}}},
      {"dftpav",
       json{{"time_budget_s", 7.0},
            {"limits", json{{"max_forward_velocity_mps", 3.0}}},
            {"options", json{{"sample_dt_s", 0.1}, {"corridor_repair", false}}}}}};
  ServicePlanRequest request;
  const ParseOutcome outcome = parse_plan_request(body, &request);
  ASSERT_TRUE(outcome.ok) << outcome.detail;
  EXPECT_EQ(request.params.hybrid_backend, "ros_free");
  EXPECT_EQ(request.params.hybrid_config.value("heading_bins", 0), 36);
  EXPECT_DOUBLE_EQ(request.params.hybrid_time_budget_s, 2.5);
  EXPECT_EQ(request.params.hybrid_node_budget, 1000);
  EXPECT_DOUBLE_EQ(request.params.corridor.sample_step_m, 0.5);
  EXPECT_DOUBLE_EQ(request.params.corridor.clearance_m, 0.1);
  EXPECT_DOUBLE_EQ(request.params.dftpav_time_budget_s, 7.0);
  ASSERT_TRUE(request.params.dftpav_limits.has_value());
  EXPECT_DOUBLE_EQ(request.params.dftpav_limits->max_forward_velocity_mps, 3.0);
  // 未指定的限制保留由车辆推导的默认值。
  EXPECT_NEAR(request.params.dftpav_limits->max_forward_curvature_inv_m,
              std::tan(0.5) / 2.8, 1e-12);
  EXPECT_DOUBLE_EQ(request.params.dftpav_options.sample_dt_s, 0.1);
  EXPECT_FALSE(request.params.dftpav_options.corridor_repair);
}

TEST(PlanRequestParse, RejectsInvalidAlgorithmParameterValues) {
  ServicePlanRequest request;
  json body = minimal_request();
  body["algorithm_params"] = json{{"time_budget_s", 0.0}};
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["algorithm_params"] = json{{"node_budget", 0}};
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["algorithm_params"] = json{{"node_budget", 1.5}};
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["algorithm_params"] = json{{"hybrid_backend", ""}};
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["algorithm_params"] = json{{"corridor", json{{"sample_step_m", -1.0}}}};
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["algorithm_params"] = json{{"dftpav", json{{"limits", json{{"max_forward_velocity_mps", 0.0}}}}}};
  EXPECT_FALSE(parse_plan_request(body, &request).ok);
}

TEST(PlanRequestParse, NormalizesRectanglesAndValidatesPolygons) {
  json body = minimal_request();
  body["obstacles"] = json::array(
      {json{{"id", "obs-1"},
            {"type", "rectangle"},
            {"x_min_m", 11.0},
            {"y_min_m", 6.0},
            {"x_max_m", 9.0},
            {"y_max_m", 4.0}},
       json{{"id", "obs-2"},
            {"type", "convex_polygon"},
            {"vertices", json::array({json{{"x_m", 9.5}, {"y_m", 3.5}},
                                      json{{"x_m", 12.5}, {"y_m", 4.5}},
                                      json{{"x_m", 12.0}, {"y_m", 7.5}},
                                      json{{"x_m", 9.0}, {"y_m", 6.5}}})}}});
  ServicePlanRequest request;
  const ParseOutcome outcome = parse_plan_request(body, &request);
  ASSERT_TRUE(outcome.ok) << outcome.detail;
  ASSERT_EQ(request.obstacles.size(), 2u);
  EXPECT_EQ(request.obstacles[0].kind, ServiceObstacle::Kind::kRectangle);
  EXPECT_DOUBLE_EQ(request.obstacles[0].x_min_m, 9.0);
  EXPECT_DOUBLE_EQ(request.obstacles[0].x_max_m, 11.0);
  EXPECT_DOUBLE_EQ(request.obstacles[0].y_min_m, 4.0);
  EXPECT_EQ(request.obstacles[1].kind, ServiceObstacle::Kind::kConvexPolygon);
  EXPECT_EQ(request.obstacles[1].vertices.size(), 4u);
}

TEST(PlanRequestParse, RejectsInvalidObstacles) {
  ServicePlanRequest request;

  json body = minimal_request();
  body["obstacles"] = json::array(
      {json{{"id", "obs-1"},
            {"type", "circle"},
            {"x_min_m", 1.0},
            {"y_min_m", 1.0},
            {"x_max_m", 2.0},
            {"y_max_m", 2.0}}});
  EXPECT_EQ(parse_plan_request(body, &request).reason, "invalid_obstacle");

  body = minimal_request();
  body["obstacles"] = json::array(
      {json{{"id", "obs-1"},
            {"type", "rectangle"},
            {"x_min_m", 1.0},
            {"y_min_m", 1.0},
            {"x_max_m", 1.0},
            {"y_max_m", 2.0}}});
  EXPECT_EQ(parse_plan_request(body, &request).reason, "invalid_obstacle");

  body = minimal_request();
  body["obstacles"] = json::array(
      {json{{"id", "obs-1"},
            {"type", "convex_polygon"},
            {"vertices", json::array({json{{"x_m", 0.0}, {"y_m", 0.0}},
                                      json{{"x_m", 1.0}, {"y_m", 0.0}}})}}});
  EXPECT_EQ(parse_plan_request(body, &request).reason, "invalid_obstacle");

  // 非凸四边形。
  body = minimal_request();
  body["obstacles"] = json::array(
      {json{{"id", "obs-1"},
            {"type", "convex_polygon"},
            {"vertices", json::array({json{{"x_m", 0.0}, {"y_m", 0.0}},
                                      json{{"x_m", 2.0}, {"y_m", 0.0}},
                                      json{{"x_m", 0.0}, {"y_m", 2.0}},
                                      json{{"x_m", 2.0}, {"y_m", 2.0}}})}}});
  EXPECT_EQ(parse_plan_request(body, &request).reason, "invalid_obstacle");

  body = minimal_request();
  body["obstacles"] = json{{"id", "obs-1"}};
  EXPECT_FALSE(parse_plan_request(body, &request).ok);
}

TEST(PlanRequestParse, LoadsVehicleRefFromFile) {
  ServicePlanRequest request;
  json body = minimal_request();
  body["vehicle_ref"] = PLANNER_SERVICE_TEST_VEHICLE_PATH;
  const ParseOutcome outcome = parse_plan_request(body, &request);
  ASSERT_TRUE(outcome.ok) << outcome.detail;
  EXPECT_EQ(request.vehicle, default_vehicle());

  body = minimal_request();
  body["vehicle_ref"] = json{{"ref", "default_test_vehicle"},
                             {"path", PLANNER_SERVICE_TEST_VEHICLE_PATH}};
  EXPECT_TRUE(parse_plan_request(body, &request).ok);
}

TEST(PlanRequestParse, RejectsVehicleRefWithoutPath) {
  ServicePlanRequest request;
  json body = minimal_request();
  body["vehicle_ref"] = json{{"ref", "default_test_vehicle"}};
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["vehicle_ref"] = 42;
  EXPECT_FALSE(parse_plan_request(body, &request).ok);

  body = minimal_request();
  body["vehicle_ref"] = "/nonexistent/vehicle.json";
  const ParseOutcome outcome = parse_plan_request(body, &request);
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.reason, "vehicle_ref_error");
}

TEST(PlanRequestParse, RejectsInvalidInlineVehicle) {
  ServicePlanRequest request;
  json body = minimal_request();
  body["vehicle"] = json{{"wheelbase_m", 0.0},
                         {"width_m", 1.8},
                         {"front_overhang_m", 0.9},
                         {"rear_overhang_m", 1.0},
                         {"max_steer_rad", 0.5},
                         {"safety_margin_m", 0.1}};
  const ParseOutcome outcome = parse_plan_request(body, &request);
  EXPECT_FALSE(outcome.ok);
  EXPECT_EQ(outcome.reason, "invalid_vehicle");

  body["vehicle"] = json{{"wheelbase_m", 2.8},
                         {"width_m", 1.8},
                         {"front_overhang_m", 0.9},
                         {"rear_overhang_m", 1.0},
                         {"max_steer_rad", 2.0},
                         {"safety_margin_m", 0.1}};
  EXPECT_EQ(parse_plan_request(body, &request).reason, "invalid_vehicle");
}

}  // namespace
}  // namespace planner::service
