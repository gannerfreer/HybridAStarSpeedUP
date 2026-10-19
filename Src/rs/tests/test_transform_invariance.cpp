// 刚体变换不变性：RS 距离与路径终点不得依赖全局坐标系
// （3.7 要求的独立验证）。

#include <cmath>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "planner/rs/reeds_shepp.hpp"
#include "rs_test_util.hpp"

namespace planner::rs {
namespace {

using test::benchmark_vehicle;
using test::deterministic_queries;
using test::endpoint_position_error;
using test::endpoint_yaw_error;
using test::pose;
using test::yaw_error;

const common::VehicleModel kVehicle = benchmark_vehicle();

common::Pose2d transform_pose(const common::Pose2d& pose, double tx, double ty,
                              double theta) {
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  return {tx + c * pose.x_m - s * pose.y_m, ty + s * pose.x_m + c * pose.y_m,
          common::normalize_angle_rad(pose.yaw_rad + theta)};
}

TEST(ReedsSheppTransform, distance_invariant_under_rigid_transforms) {
  const std::vector<std::pair<std::pair<double, double>, double>> transforms = {
      {{0.0, 0.0}, 0.0},       {{10.0, -4.0}, 0.0},
      {{-7.5, 3.25}, 1.0},     {{2.0, 2.0}, -2.1},
      {{100.0, 50.0}, common::kPi - 0.01},
  };
  for (const test::Query& query : deterministic_queries()) {
    const double base = distance(query.start, query.goal, kVehicle);
    for (const auto& transform : transforms) {
      const double tx = transform.first.first;
      const double ty = transform.first.second;
      const double theta = transform.second;
      const double moved =
          distance(transform_pose(query.start, tx, ty, theta),
                   transform_pose(query.goal, tx, ty, theta), kVehicle);
      EXPECT_NEAR(moved, base, 1e-9 + 1e-9 * std::fabs(base))
          << query.id << " transform " << tx << "," << ty << "," << theta;
    }
  }
}

TEST(ReedsSheppTransform, path_endpoint_maps_to_transformed_goal) {
  const double tx = -3.5;
  const double ty = 8.25;
  const double theta = 0.77;
  for (const test::Query& query : deterministic_queries()) {
    const common::Pose2d start = transform_pose(query.start, tx, ty, theta);
    const common::Pose2d goal = transform_pose(query.goal, tx, ty, theta);
    const RsPath p = path(start, goal, kVehicle);
    EXPECT_LE(endpoint_position_error(p), kEndpointPositionToleranceM) << query.id;
    EXPECT_LE(endpoint_yaw_error(p), kEndpointYawToleranceRad) << query.id;
    EXPECT_NEAR(p.goal.x_m, goal.x_m, 1e-12) << query.id;
    EXPECT_NEAR(p.goal.y_m, goal.y_m, 1e-12) << query.id;
    EXPECT_LE(yaw_error(p.goal.yaw_rad, goal.yaw_rad), 1e-12) << query.id;
  }
}

TEST(ReedsSheppTransform, distance_invariant_under_full_turn_offsets) {
  for (const test::Query& query : deterministic_queries()) {
    const double base = distance(query.start, query.goal, kVehicle);
    common::Pose2d start = query.start;
    common::Pose2d goal = query.goal;
    start.yaw_rad += 4.0 * common::kPi;
    goal.yaw_rad -= 6.0 * common::kPi;
    EXPECT_NEAR(distance(start, goal, kVehicle), base,
                1e-9 + 1e-9 * std::fabs(base))
        << query.id;
  }
}

TEST(ReedsSheppTransform, translate_only_reverses_are_equivalent) {
  // 航向相同的纯平移不得改变路径长度。
  const RsPath p1 = path(pose(0.0, 0.0, 0.0), pose(4.0, 2.0, 0.0), kVehicle);
  const RsPath p2 = path(pose(20.0, -9.0, 0.0), pose(24.0, -7.0, 0.0), kVehicle);
  EXPECT_NEAR(p1.length_m, p2.length_m, 1e-9);
}

}  // namespace
}  // namespace planner::rs
