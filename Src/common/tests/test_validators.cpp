/**
 * @file test_validators.cpp
 * @brief 车辆模型与 Hybrid A* 规划器配置合法性校验单元测试。
 */

#include "planner/common/planner_config.hpp"
#include "planner/common/vehicle.hpp"
#include "planner/math/angle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace {

using namespace planner;

common::VehicleModel valid_vehicle() {
  common::VehicleModel vehicle;
  vehicle.id = "test_vehicle";
  vehicle.wheelbase_m = 2.5;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.8;
  vehicle.rear_overhang_m = 0.8;
  vehicle.max_steer_rad = 0.6;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

TEST(VehicleValid, AcceptsValidModel) {
  std::string reason = "unchanged";
  EXPECT_TRUE(common::vehicle_valid(valid_vehicle(), &reason));
  EXPECT_EQ(reason, "unchanged");
  EXPECT_TRUE(common::vehicle_valid(valid_vehicle(), nullptr));
}

TEST(VehicleValid, RejectsInvalidWheelbaseAndWidth) {
  std::string reason;
  common::VehicleModel vehicle = valid_vehicle();
  vehicle.wheelbase_m = 0.0;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));
  EXPECT_FALSE(reason.empty());

  vehicle = valid_vehicle();
  vehicle.wheelbase_m = -1.0;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.wheelbase_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.width_m = 0.0;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.width_m = -1.0;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.width_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));
}

TEST(VehicleValid, RejectsNegativeOverhangsAndMargin) {
  std::string reason;
  common::VehicleModel vehicle = valid_vehicle();
  vehicle.front_overhang_m = -0.01;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));
  EXPECT_FALSE(reason.empty());

  vehicle = valid_vehicle();
  vehicle.rear_overhang_m = -0.01;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.front_overhang_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.safety_margin_m = -0.01;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.safety_margin_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));
}

TEST(VehicleValid, RejectsInvalidMaxSteer) {
  std::string reason;
  common::VehicleModel vehicle = valid_vehicle();
  vehicle.max_steer_rad = 0.0;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));
  EXPECT_FALSE(reason.empty());

  vehicle = valid_vehicle();
  vehicle.max_steer_rad = -0.1;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.max_steer_rad = math::kPi / 2.0;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.max_steer_rad = math::kPi / 2.0 + 0.1;
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.max_steer_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));

  vehicle = valid_vehicle();
  vehicle.max_steer_rad = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(common::vehicle_valid(vehicle, &reason));
}

TEST(PlannerConfigValid, AcceptsDefaults) {
  const common::UserAstarConfig config;
  std::string reason = "unchanged";
  EXPECT_TRUE(common::planner_config_valid(config, &reason));
  EXPECT_EQ(reason, "unchanged");
  EXPECT_TRUE(common::planner_config_valid(config, nullptr));
}

TEST(PlannerConfigValid, RejectsInvalidResolutionStepAndCaptureRadius) {
  std::string reason;
  common::UserAstarConfig config;
  config.xy_resolution_m = 0.0;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));
  EXPECT_FALSE(reason.empty());

  config = common::UserAstarConfig{};
  config.xy_resolution_m = -1.0;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.xy_resolution_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.motion_step_m = 0.0;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.motion_step_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.goal_capture_radius_m = -1.0;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));
}

TEST(PlannerConfigValid, RejectsInvalidTimeBudget) {
  std::string reason;
  common::UserAstarConfig config;
  config.default_time_budget_s = 0.0;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.default_time_budget_s = -1.0;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.default_time_budget_s = 1e9 + 1.0;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.default_time_budget_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(common::planner_config_valid(config, &reason));
}

TEST(PlannerConfigValid, RejectsInvalidCostsAndRanges) {
  std::string reason;
  common::UserAstarConfig config;
  config.forward_cost = -0.1;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.reverse_cost = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.turn_penalty = -0.1;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.switch_cost = -0.1;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.backward_search_range_m = -0.1;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.linear_preferred_distance_square = -0.1;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.analytic_shot_distance_m = -0.1;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.analytic_expansion_max_length_m = -0.1;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.min_connector_segment_m = -0.1;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));
}

TEST(PlannerConfigValid, RejectsInvalidDiscreteSearchParameters) {
  std::string reason;
  common::UserAstarConfig config;
  config.heading_bins = 7;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));
  EXPECT_FALSE(reason.empty());

  config = common::UserAstarConfig{};
  config.steering_samples = 1;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.analytic_expansion_ratio = 0;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));

  config = common::UserAstarConfig{};
  config.default_node_budget = 0;
  EXPECT_FALSE(common::planner_config_valid(config, &reason));
}

}  // namespace
