#include "motion.hpp"
#include "reeds_shepp.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {
using namespace planner;

common::VehicleModel test_vehicle() {
  common::VehicleModel vehicle;
  vehicle.wheelbase_m = 2.5;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.8;
  vehicle.rear_overhang_m = 0.8;
  vehicle.max_steer_rad = 0.6;
  return vehicle;
}

const std::vector<std::pair<common::Pose2d, common::Pose2d>>& known_queries() {
  static const std::vector<std::pair<common::Pose2d, common::Pose2d>> queries = {
      {{0.0, 0.0, 0.0}, {5.0, 3.0, 0.7}},
      {{-2.0, 1.0, 0.3}, {2.0, -3.0, -1.2}},
      {{1.0, 0.5, -0.4}, {-1.0, 2.0, 2.6}},
  };
  return queries;
}

TEST(ReedsShepp, PlansKnownQueriesAndClosesEndpoint) {
  const common::VehicleModel vehicle = test_vehicle();
  const auto& queries = known_queries();
  const double max_curvature = 1.0 / rs::min_turn_radius_m(vehicle);
  for (std::size_t query_index = 0; query_index < queries.size();
       ++query_index) {
    SCOPED_TRACE(query_index);
    const auto& start = queries[query_index].first;
    const auto& goal = queries[query_index].second;
    const auto solved = rs::plan(start, goal, vehicle);
    ASSERT_EQ(solved.status, common::Status::kSuccess) << solved.reason;
    EXPECT_GT(solved.path.length_m, 0.0);
    ASSERT_FALSE(solved.path.segments.empty());
    common::Pose2d cursor = solved.path.start;
    double total_length = 0.0;
    for (const auto& segment : solved.path.segments) {
      EXPECT_TRUE(std::isfinite(segment.length_m));
      EXPECT_GE(segment.length_m, 0.0);
      EXPECT_TRUE(std::isfinite(segment.curvature_inv_m));
      EXPECT_LE(std::abs(segment.curvature_inv_m), max_curvature + 1e-12)
          << "segment exceeds the minimum turn radius bound";
      cursor = rs::integrate_segment(cursor, segment, segment.length_m);
      total_length += segment.length_m;
    }
    EXPECT_NEAR(total_length, solved.path.length_m, 1e-9);
    EXPECT_LT(std::hypot(cursor.x_m - goal.x_m, cursor.y_m - goal.y_m),
              rs::kEndpointPositionToleranceM);
    EXPECT_LT(std::abs(math::normalize_angle_rad(cursor.yaw_rad - goal.yaw_rad)),
              rs::kEndpointYawToleranceRad);
  }
}

TEST(ReedsShepp, SampleIsMonotonicAndEndsAtGoal) {
  const double ds_m = 0.25;
  const common::VehicleModel vehicle = test_vehicle();
  const auto& queries = known_queries();
  for (std::size_t query_index = 0; query_index < queries.size();
       ++query_index) {
    SCOPED_TRACE(query_index);
    const auto solved = rs::plan(queries[query_index].first,
                                 queries[query_index].second, vehicle);
    ASSERT_EQ(solved.status, common::Status::kSuccess) << solved.reason;
    const auto samples = rs::sample(solved.path, ds_m);
    ASSERT_GE(samples.size(), 2u);
    EXPECT_EQ(samples.front().s_m, 0.0);
    EXPECT_EQ(samples.back().s_m, solved.path.length_m);
    EXPECT_EQ(samples.front().pose.x_m, solved.path.start.x_m);
    EXPECT_EQ(samples.front().pose.y_m, solved.path.start.y_m);
    EXPECT_LT(std::hypot(samples.back().pose.x_m - solved.path.goal.x_m,
                         samples.back().pose.y_m - solved.path.goal.y_m),
              1e-9);
    for (std::size_t i = 1; i < samples.size(); ++i) {
      SCOPED_TRACE(i);
      EXPECT_GT(samples[i].s_m, samples[i - 1].s_m);
      EXPECT_LE(samples[i].s_m - samples[i - 1].s_m, ds_m + 1e-12);
      EXPECT_TRUE(std::isfinite(samples[i].pose.x_m));
      EXPECT_TRUE(std::isfinite(samples[i].pose.y_m));
      EXPECT_TRUE(std::isfinite(samples[i].pose.yaw_rad));
    }
  }
}

TEST(ReedsShepp, ReverseSegmentIntegrationMatchesSharedArcKernel) {
  const common::Pose2d start{2.0, 3.0, 0.7};
  rs::RsSegment segment;
  segment.length_m = 2.0;
  segment.curvature_inv_m = 1e-12;
  segment.direction = common::Direction::kReverse;

  const auto full = rs::integrate_segment(start, segment, segment.length_m);
  const auto expected_full =
      math::integrate_arc(start, -segment.length_m, segment.curvature_inv_m);
  EXPECT_LT(std::hypot(full.x_m - expected_full.x_m,
                       full.y_m - expected_full.y_m),
            1e-12);
  EXPECT_LT(std::abs(math::normalize_angle_rad(full.yaw_rad -
                                               expected_full.yaw_rad)),
            1e-12);

  const double partial_length_m = 0.75;
  const auto partial = rs::integrate_segment(start, segment, partial_length_m);
  const auto expected_partial =
      math::integrate_arc(start, -partial_length_m, segment.curvature_inv_m);
  EXPECT_LT(std::hypot(partial.x_m - expected_partial.x_m,
                       partial.y_m - expected_partial.y_m),
            1e-12);

  rs::RsSegment straight;
  straight.length_m = 2.0;
  straight.curvature_inv_m = 0.0;
  straight.direction = common::Direction::kForward;
  const auto forward = rs::integrate_segment(start, straight, 2.0);
  const auto expected_forward = math::integrate_arc(start, 2.0, 0.0);
  EXPECT_LT(std::hypot(forward.x_m - expected_forward.x_m,
                       forward.y_m - expected_forward.y_m),
            1e-15);
  EXPECT_THROW(rs::integrate_segment(start, segment, 2.5),
               std::invalid_argument);
}

}  // namespace
