// planner::rs 的解析距离预期值（Docs/.../6.2 RS 行）。
//
// 此处每个用例都对照闭式 Reeds-Shepp 长度或由公开 VehicleModel 推导的
// 最小转弯半径界进行检验；均不依赖 OMPL。

#include <cmath>

#include <gtest/gtest.h>

#include "planner/rs/reeds_shepp.hpp"
#include "rs_test_util.hpp"

namespace planner::rs {
namespace {

using test::benchmark_vehicle;
using test::deterministic_queries;
using test::pose;
using test::vehicle_with_radius;

const common::VehicleModel kVehicle = benchmark_vehicle();

TEST(ReedsSheppDistance, identical_poses_have_zero_distance) {
  const double d = distance(pose(3.0, -2.0, 0.3), pose(3.0, -2.0, 0.3), kVehicle);
  EXPECT_DOUBLE_EQ(d, 0.0);
}

TEST(ReedsSheppDistance, straight_forward_matches_euclidean) {
  for (const double d : {0.5, 5.0, 23.75}) {
    EXPECT_NEAR(distance(pose(0.0, 0.0, 0.0), pose(d, 0.0, 0.0), kVehicle), d, 1e-9);
  }
}

TEST(ReedsSheppDistance, straight_reverse_matches_euclidean) {
  for (const double d : {0.5, 5.0, 23.75}) {
    EXPECT_NEAR(distance(pose(0.0, 0.0, 0.0), pose(-d, 0.0, 0.0), kVehicle), d, 1e-9);
  }
}

TEST(ReedsSheppDistance, quarter_turn_matches_min_radius_arc) {
  const double radius = min_turn_radius_m(kVehicle);
  // 从 (0,0,0) 出发的左四分之一圆：圆心 (0, R)，终点 (R, R, pi/2)。
  EXPECT_NEAR(distance(pose(0.0, 0.0, 0.0), pose(radius, radius, 0.5 * common::kPi), kVehicle),
              radius * 0.5 * common::kPi, 1e-9);
  // 右四分之一圆：圆心 (0, R)，镜像后终点 (-R, R, -pi/2)。
  EXPECT_NEAR(distance(pose(0.0, 0.0, 0.0), pose(-radius, radius, -0.5 * common::kPi), kVehicle),
              radius * 0.5 * common::kPi, 1e-9);
}

TEST(ReedsSheppDistance, uturn_at_min_radius_is_pi_times_radius) {
  const double radius = min_turn_radius_m(kVehicle);
  // 半圆：任何路径都必须累计 pi 的航向变化，且 |d yaw / ds| <= 1/R。
  EXPECT_NEAR(distance(pose(0.0, 0.0, 0.0), pose(0.0, 2.0 * radius, common::kPi), kVehicle),
              radius * common::kPi, 1e-9);
}

TEST(ReedsSheppDistance, cusp_case_matches_pi_times_radius) {
  const double radius = min_turn_radius_m(kVehicle);
  // 相同位置、相反航向：航向变化 pi 需要长度 >= pi*R。
  EXPECT_NEAR(distance(pose(0.0, 0.0, 0.0), pose(0.0, 0.0, common::kPi), kVehicle),
              radius * common::kPi, 1e-9);
}

TEST(ReedsSheppDistance, angle_wrap_350_degrees_to_minus_10_is_straight) {
  const double deg = common::kPi / 180.0;
  const double start_yaw = 350.0 * deg;  // 归一化为 -10 deg
  const double goal_yaw = -10.0 * deg;
  // 目标点位于（环绕后的）-10 deg 航向上：一条直线。
  const common::Pose2d goal{5.0 * std::cos(goal_yaw), 5.0 * std::sin(goal_yaw),
                            goal_yaw};
  EXPECT_NEAR(distance(pose(0.0, 0.0, start_yaw), goal, kVehicle), 5.0, 1e-9);
  // 归一化后位姿相同：即使原始 yaw 相差一整圈，距离也为零（归一化值可能
  // 相差一个 ulp，因此用小容差检验才是诚实的做法）。
  EXPECT_NEAR(distance(pose(0.0, 0.0, start_yaw), pose(0.0, 0.0, goal_yaw), kVehicle),
              0.0, 1e-9);
}

TEST(ReedsSheppDistance, full_turn_goal_yaw_is_equivalent) {
  const double base = distance(pose(0.0, 0.0, 0.0), pose(6.0, 1.5, -0.4), kVehicle);
  const double wrapped =
      distance(pose(0.0, 0.0, 0.0), pose(6.0, 1.5, 2.0 * common::kPi - 0.4), kVehicle);
  const double shifted =
      distance(pose(0.0, 0.0, 2.0 * common::kPi), pose(6.0, 1.5, 2.0 * common::kPi - 0.4), kVehicle);
  EXPECT_NEAR(wrapped, base, 1e-9);
  EXPECT_NEAR(shifted, base, 1e-9);
}

TEST(ReedsSheppDistance, symmetric_under_query_swap) {
  for (const test::Query& query : deterministic_queries()) {
    const double forward = distance(query.start, query.goal, kVehicle);
    const double backward = distance(query.goal, query.start, kVehicle);
    EXPECT_NEAR(backward, forward, 1e-9 + 1e-9 * std::fabs(forward)) << query.id;
  }
}

TEST(ReedsSheppDistance, scales_linearly_with_turning_radius) {
  // 只有同一*归一化*查询下，长度才与 R 成线性：第二辆车将归一化目标偏移
  // 翻倍（且其 R = 2R）。
  const common::VehicleModel small = vehicle_with_radius(3.0);
  const common::VehicleModel large = vehicle_with_radius(6.0);
  struct NormalizedQuery {
    double x;
    double y;
    double phi;
  };
  const std::vector<NormalizedQuery> queries = {
      {1.0, 0.0, 0.0}, {1.0, 0.5, 0.3}, {-2.0, 1.0, -1.0},
      {0.5, -1.5, 2.5}, {0.0, 0.0, 0.0}, {-1.0, -0.5, 0.2},
  };
  for (const NormalizedQuery& query : queries) {
    const common::Pose2d start = pose(0.0, 0.0, 0.0);
    const double d_small =
        distance(start, pose(query.x * 3.0, query.y * 3.0, query.phi), small);
    const double d_large =
        distance(start, pose(query.x * 6.0, query.y * 6.0, query.phi), large);
    EXPECT_NEAR(d_large, 2.0 * d_small, 1e-9 + 1e-9 * d_large)
        << query.x << "," << query.y << "," << query.phi;
  }
}

TEST(ReedsSheppDistance, never_below_euclidean_distance) {
  for (const test::Query& query : deterministic_queries()) {
    const double d = distance(query.start, query.goal, kVehicle);
    const double euclidean = std::hypot(query.goal.x_m - query.start.x_m,
                                        query.goal.y_m - query.start.y_m);
    EXPECT_GE(d + 1e-9, euclidean) << query.id;
  }
}

TEST(ReedsSheppDistance, matches_path_length_for_all_queries) {
  for (const test::Query& query : deterministic_queries()) {
    const RsPath p = path(query.start, query.goal, kVehicle);
    double sum = 0.0;
    for (const RsSegment& segment : p.segments) {
      sum += segment.length_m;
    }
    EXPECT_NEAR(p.length_m, sum, 1e-9) << query.id;
    EXPECT_NEAR(distance(query.start, query.goal, kVehicle), p.length_m, 1e-9)
        << query.id;
  }
}

}  // namespace
}  // namespace planner::rs
