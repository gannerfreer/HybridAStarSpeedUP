// 无效输入与数值边界处理：plan() 必须报告 kInvalidInput，抛异常的包装
// 必须拒绝，绝不静默产出路径（Docs/.../5.4 硬性安全原则）。

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "planner/rs/reeds_shepp.hpp"
#include "rs_test_util.hpp"

namespace planner::rs {
namespace {

using test::benchmark_vehicle;
using test::pose;

const common::VehicleModel kVehicle = benchmark_vehicle();
const common::Pose2d kStart = pose(0.0, 0.0, 0.0);
const common::Pose2d kGoal = pose(5.0, 2.0, 0.4);

bool rejected_by_plan(const common::Pose2d& start, const common::Pose2d& goal,
                      const common::VehicleModel& vehicle) {
  const RsPlan result = plan(start, goal, vehicle);
  return result.status == common::Status::kInvalidInput && !result.reason.empty();
}

TEST(ReedsSheppInvalidInput, rejects_zero_and_negative_wheelbase) {
  common::VehicleModel vehicle = kVehicle;
  vehicle.wheelbase_m = 0.0;
  EXPECT_TRUE(rejected_by_plan(kStart, kGoal, vehicle));
  vehicle.wheelbase_m = -2.8;
  EXPECT_TRUE(rejected_by_plan(kStart, kGoal, vehicle));
}

TEST(ReedsSheppInvalidInput, rejects_zero_and_obtuse_max_steer) {
  common::VehicleModel vehicle = kVehicle;
  vehicle.max_steer_rad = 0.0;
  EXPECT_TRUE(rejected_by_plan(kStart, kGoal, vehicle));
  vehicle.max_steer_rad = 0.5 * common::kPi;
  EXPECT_TRUE(rejected_by_plan(kStart, kGoal, vehicle));
  vehicle.max_steer_rad = 1.9;
  EXPECT_TRUE(rejected_by_plan(kStart, kGoal, vehicle));
}

TEST(ReedsSheppInvalidInput, rejects_non_finite_vehicle_parameters) {
  common::VehicleModel vehicle = kVehicle;
  vehicle.wheelbase_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(rejected_by_plan(kStart, kGoal, vehicle));
  vehicle = kVehicle;
  vehicle.max_steer_rad = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(rejected_by_plan(kStart, kGoal, vehicle));
}

TEST(ReedsSheppInvalidInput, rejects_non_finite_pose_fields) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(rejected_by_plan(pose(nan, 0.0, 0.0), kGoal, kVehicle));
  EXPECT_TRUE(rejected_by_plan(pose(0.0, inf, 0.0), kGoal, kVehicle));
  EXPECT_TRUE(rejected_by_plan(kStart, pose(0.0, 0.0, nan), kVehicle));
  EXPECT_TRUE(rejected_by_plan(kStart, pose(inf, 0.0, 0.0), kVehicle));
}

TEST(ReedsSheppInvalidInput, distance_and_path_throw_invalid_argument) {
  common::VehicleModel vehicle = kVehicle;
  vehicle.wheelbase_m = 0.0;
  EXPECT_THROW(distance(kStart, kGoal, vehicle), std::invalid_argument);
  EXPECT_THROW(path(kStart, kGoal, vehicle), std::invalid_argument);
  EXPECT_THROW(min_turn_radius_m(vehicle), std::invalid_argument);
}

TEST(ReedsSheppInvalidInput, minimal_valid_vehicle_is_accepted) {
  common::VehicleModel vehicle;
  vehicle.id = "minimal";
  vehicle.wheelbase_m = 0.5;
  vehicle.max_steer_rad = 0.01;
  vehicle.width_m = 0.3;
  const RsPlan result = plan(kStart, kGoal, vehicle);
  EXPECT_EQ(result.status, common::Status::kSuccess) << result.reason;
  EXPECT_NEAR(result.path.length_m, distance(kStart, kGoal, vehicle), 1e-12);
}

TEST(ReedsSheppInvalidInput, shortest_and_longest_valid_wheelbases) {
  common::VehicleModel tiny = kVehicle;
  tiny.wheelbase_m = 1e-3;
  EXPECT_EQ(plan(kStart, kGoal, tiny).status, common::Status::kSuccess);
  common::VehicleModel huge = kVehicle;
  huge.wheelbase_m = 1e6;
  EXPECT_EQ(plan(kStart, kGoal, huge).status, common::Status::kSuccess);
}

TEST(ReedsSheppInvalidInput, nearly_straight_steering_is_accepted) {
  common::VehicleModel vehicle = kVehicle;
  vehicle.max_steer_rad = 1e-6;  // 最小半径约 2.8e6 m
  const RsPlan result = plan(kStart, kGoal, vehicle);
  EXPECT_EQ(result.status, common::Status::kSuccess) << result.reason;
  EXPECT_TRUE(std::isfinite(result.path.length_m));
}

TEST(ReedsSheppInvalidInput, is_valid_query_matches_plan_acceptance) {
  EXPECT_TRUE(is_valid_query(kStart, kGoal, kVehicle));
  common::VehicleModel vehicle = kVehicle;
  vehicle.max_steer_rad = 2.0;
  EXPECT_FALSE(is_valid_query(kStart, kGoal, vehicle));
  EXPECT_FALSE(is_valid_query(pose(0.0, 0.0, std::nan("")), kGoal, kVehicle));
}

TEST(ReedsSheppInvalidInput, extreme_but_finite_poses_stay_well_defined) {
  const common::Pose2d far_start = pose(1e6, -1e6, 3.0);
  const common::Pose2d far_goal = pose(-1e6, 1e6, -3.0);
  const RsPlan result = plan(far_start, far_goal, kVehicle);
  EXPECT_EQ(result.status, common::Status::kSuccess) << result.reason;
  EXPECT_TRUE(std::isfinite(result.path.length_m));
}

}  // namespace
}  // namespace planner::rs
