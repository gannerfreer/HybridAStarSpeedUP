// 分段二维轨迹的单元测试：衔接点连续性（三次为 C1，
// 五次为 C2）、右侧分段边界规则、定义域拒绝、
// 批处理采样和时间缩放。容差遵循第 7.1 节。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "planner/common/types.hpp"
#include "planner/polynomial/piecewise_trajectory.hpp"
#include "planner/polynomial/result.hpp"
#include "planner/polynomial/trajectory_segment.hpp"

namespace planner::polynomial {
namespace {

using planner::common::Point2d;

constexpr double kTolerance = 1e-6;
constexpr double kAbsTol = 1e-6;
constexpr double kRelTol = 1e-6;

void expect_point_close(const Point2d& actual, const Point2d& expected) {
  EXPECT_NEAR(actual.x_m, expected.x_m, kAbsTol + kRelTol * std::fabs(expected.x_m));
  EXPECT_NEAR(actual.y_m, expected.y_m, kAbsTol + kRelTol * std::fabs(expected.y_m));
}

TrajectorySegment cubic_segment(const Point2d& p0, const Point2d& v0, const Point2d& p1,
                                const Point2d& v1, double duration_s) {
  const Result<TrajectorySegment> solved =
      TrajectorySegment::cubic(CubicBoundary2d{p0, v0, p1, v1}, duration_s);
  EXPECT_TRUE(solved.ok());
  return solved.value;
}

TrajectorySegment quintic_segment(const Point2d& p0, const Point2d& v0, const Point2d& a0,
                                  const Point2d& p1, const Point2d& v1, const Point2d& a1,
                                  double duration_s) {
  const Result<TrajectorySegment> solved =
      TrajectorySegment::quintic(QuinticBoundary2d{p0, v0, a0, p1, v1, a1}, duration_s);
  EXPECT_TRUE(solved.ok());
  return solved.value;
}

// 三个三次分段，衔接点速度匹配。在第一个衔接点加速度发生
// 跳变（0.0 -> -1.0），右侧分段选择测试正是观察这一点。
std::vector<TrajectorySegment> cubic_chain() {
  std::vector<TrajectorySegment> segments;
  segments.push_back(cubic_segment(Point2d{0.0, 0.0}, Point2d{1.0, 0.0}, Point2d{1.0, 0.5},
                                   Point2d{1.0, 0.0}, 1.0));
  segments.push_back(cubic_segment(Point2d{1.0, 0.5}, Point2d{1.0, 0.0}, Point2d{2.0, 0.5},
                                   Point2d{1.5, 0.0}, 2.0));
  segments.push_back(cubic_segment(Point2d{2.0, 0.5}, Point2d{1.5, 0.0}, Point2d{4.0, 0.0},
                                   Point2d{0.0, 0.0}, 1.5));
  return segments;
}

// 四个五次分段，衔接点的位置/速度/加速度均匹配。
std::vector<TrajectorySegment> quintic_chain() {
  std::vector<TrajectorySegment> segments;
  segments.push_back(quintic_segment(Point2d{0.0, 0.0}, Point2d{2.0, 0.0}, Point2d{0.0, 0.5},
                                     Point2d{4.0, 1.0}, Point2d{2.0, 0.5}, Point2d{0.0, -0.25},
                                     2.0));
  segments.push_back(quintic_segment(Point2d{4.0, 1.0}, Point2d{2.0, 0.5}, Point2d{0.0, -0.25},
                                     Point2d{7.0, 1.5}, Point2d{1.0, 0.5}, Point2d{-0.1, 0.05},
                                     1.5));
  segments.push_back(quintic_segment(Point2d{7.0, 1.5}, Point2d{1.0, 0.5}, Point2d{-0.1, 0.05},
                                     Point2d{9.0, 1.0}, Point2d{0.5, -0.5}, Point2d{0.0, 0.0},
                                     3.0));
  segments.push_back(quintic_segment(Point2d{9.0, 1.0}, Point2d{0.5, -0.5}, Point2d{0.0, 0.0},
                                     Point2d{12.0, 0.0}, Point2d{0.0, 0.0}, Point2d{0.0, 0.0},
                                     2.5));
  return segments;
}

TEST(PiecewiseCreate, rejects_empty_segment_list) {
  const Result<PiecewiseTrajectory2d> created = PiecewiseTrajectory2d::create({});
  EXPECT_EQ(created.code, Code::kInvalidInput);
}

TEST(PiecewiseCreate, rejects_invalid_segments) {
  std::vector<TrajectorySegment> segments;
  segments.push_back(cubic_segment(Point2d{0.0, 0.0}, Point2d{0.0, 0.0}, Point2d{1.0, 0.0},
                                   Point2d{0.0, 0.0}, 1.0));
  segments.push_back(TrajectorySegment{});  // 默认构造：无效
  EXPECT_EQ(PiecewiseTrajectory2d::create(std::move(segments)).code, Code::kInvalidInput);
}

TEST(PiecewiseCreate, rejects_non_finite_tolerance) {
  EXPECT_EQ(PiecewiseTrajectory2d::create(cubic_chain(), ContinuityPolicy::kAutomatic,
                                          std::nan(""))
                .code,
            Code::kInvalidInput);
  EXPECT_EQ(PiecewiseTrajectory2d::create(cubic_chain(), ContinuityPolicy::kAutomatic, -1.0).code,
            Code::kInvalidInput);
}

TEST(PiecewiseCubic, is_c1_at_every_internal_junction) {
  const Result<PiecewiseTrajectory2d> created = PiecewiseTrajectory2d::create(cubic_chain());
  ASSERT_TRUE(created.ok());
  const PiecewiseTrajectory2d& trajectory = created.value;
  ASSERT_EQ(trajectory.segment_count(), 3u);
  EXPECT_DOUBLE_EQ(trajectory.duration_s(), 4.5);
  for (std::size_t i = 0; i + 1 < trajectory.segment_count(); ++i) {
    const double junction_s = trajectory.segment_start_time_s(i + 1);
    for (int order = 0; order <= 1; ++order) {
      const Point2d left = trajectory.segment(i).evaluate(trajectory.segment(i).duration_s(), order).value;
      const Point2d right = trajectory.segment(i + 1).evaluate(0.0, order).value;
      EXPECT_NEAR(right.x_m, left.x_m, kTolerance) << "junction " << i << " order " << order;
      EXPECT_NEAR(right.y_m, left.y_m, kTolerance) << "junction " << i << " order " << order;
      // 分段容器返回相同的连续值。
      expect_point_close(trajectory.evaluate(junction_s, order).value, right);
    }
  }
}

TEST(PiecewiseQuintic, is_c2_at_every_internal_junction) {
  const Result<PiecewiseTrajectory2d> created = PiecewiseTrajectory2d::create(quintic_chain());
  ASSERT_TRUE(created.ok());
  const PiecewiseTrajectory2d& trajectory = created.value;
  EXPECT_DOUBLE_EQ(trajectory.duration_s(), 9.0);
  for (std::size_t i = 0; i + 1 < trajectory.segment_count(); ++i) {
    const double junction_s = trajectory.segment_start_time_s(i + 1);
    for (int order = 0; order <= 2; ++order) {
      const Point2d left = trajectory.segment(i).evaluate(trajectory.segment(i).duration_s(), order).value;
      const Point2d right = trajectory.segment(i + 1).evaluate(0.0, order).value;
      EXPECT_NEAR(right.x_m, left.x_m, kTolerance) << "junction " << i << " order " << order;
      EXPECT_NEAR(right.y_m, left.y_m, kTolerance) << "junction " << i << " order " << order;
      expect_point_close(trajectory.evaluate(junction_s, order).value, right);
    }
  }
}

TEST(PiecewiseCreate, rejects_velocity_mismatch_between_cubics) {
  std::vector<TrajectorySegment> segments;
  segments.push_back(cubic_segment(Point2d{0.0, 0.0}, Point2d{1.0, 0.0}, Point2d{1.0, 0.0},
                                   Point2d{1.0, 0.0}, 1.0));
  segments.push_back(cubic_segment(Point2d{1.0, 0.0}, Point2d{1.1, 0.0}, Point2d{2.0, 0.0},
                                   Point2d{1.0, 0.0}, 1.0));
  EXPECT_EQ(PiecewiseTrajectory2d::create(std::move(segments)).code, Code::kInvalidInput);
}

TEST(PiecewiseCreate, rejects_acceleration_mismatch_between_quintics) {
  std::vector<TrajectorySegment> segments;
  segments.push_back(quintic_segment(Point2d{0.0, 0.0}, Point2d{1.0, 0.0}, Point2d{0.0, 0.0},
                                     Point2d{1.0, 0.0}, Point2d{1.0, 0.0}, Point2d{1.0, 0.0},
                                     1.0));
  segments.push_back(quintic_segment(Point2d{1.0, 0.0}, Point2d{1.0, 0.0}, Point2d{2.0, 0.0},
                                     Point2d{2.0, 0.0}, Point2d{1.0, 0.0}, Point2d{1.0, 0.0},
                                     1.0));
  EXPECT_EQ(PiecewiseTrajectory2d::create(std::move(segments)).code, Code::kInvalidInput);
}

TEST(PiecewiseCreate, rejects_position_mismatch_even_with_position_only_policy) {
  std::vector<TrajectorySegment> segments;
  segments.push_back(cubic_segment(Point2d{0.0, 0.0}, Point2d{1.0, 0.0}, Point2d{1.0, 0.0},
                                   Point2d{0.0, 0.0}, 1.0));
  segments.push_back(cubic_segment(Point2d{1.5, 0.0}, Point2d{0.0, 0.0}, Point2d{2.0, 0.0},
                                   Point2d{1.0, 0.0}, 1.0));
  EXPECT_EQ(PiecewiseTrajectory2d::create(std::move(segments), ContinuityPolicy::kPositionOnly).code,
            Code::kInvalidInput);
}

TEST(PiecewiseCreate, position_only_policy_allows_derivative_jumps) {
  std::vector<TrajectorySegment> segments;
  segments.push_back(cubic_segment(Point2d{0.0, 0.0}, Point2d{1.0, 0.0}, Point2d{1.0, 0.0},
                                   Point2d{1.0, 0.0}, 1.0));
  segments.push_back(cubic_segment(Point2d{1.0, 0.0}, Point2d{2.0, 0.0}, Point2d{2.0, 0.0},
                                   Point2d{1.0, 0.0}, 1.0));
  const Result<PiecewiseTrajectory2d> created =
      PiecewiseTrajectory2d::create(std::move(segments), ContinuityPolicy::kPositionOnly);
  ASSERT_TRUE(created.ok());
  EXPECT_DOUBLE_EQ(created.value.evaluate(1.0, 1).value.x_m, 2.0);
}

TEST(PiecewiseBoundaryRule, internal_junction_selects_right_segment) {
  const Result<PiecewiseTrajectory2d> created = PiecewiseTrajectory2d::create(cubic_chain());
  ASSERT_TRUE(created.ok());
  const PiecewiseTrajectory2d& trajectory = created.value;
  // 第一个衔接点在 t = 1.0：分段 0 在其末端的加速度为 0，
  // 分段 1 在其起点的加速度为 -1。
  const double junction_s = trajectory.segment_start_time_s(1);
  EXPECT_DOUBLE_EQ(junction_s, 1.0);
  const Point2d right = trajectory.segment(1).evaluate(0.0, 2).value;
  const Point2d left = trajectory.segment(0).evaluate(1.0, 2).value;
  EXPECT_NE(right.x_m, left.x_m);
  const Point2d selected = trajectory.evaluate(junction_s, 2).value;
  expect_point_close(selected, right);
}

TEST(PiecewiseBoundaryRule, final_endpoint_selects_last_segment) {
  const Result<PiecewiseTrajectory2d> created = PiecewiseTrajectory2d::create(cubic_chain());
  ASSERT_TRUE(created.ok());
  const PiecewiseTrajectory2d& trajectory = created.value;
  const double total = trajectory.duration_s();
  const Point2d selected = trajectory.evaluate(total, 2).value;
  const Point2d last = trajectory.segment(trajectory.segment_count() - 1)
                           .evaluate(trajectory.segment(trajectory.segment_count() - 1).duration_s(), 2)
                           .value;
  expect_point_close(selected, last);
}

TEST(PiecewiseDomain, rejects_times_outside_the_trajectory) {
  const Result<PiecewiseTrajectory2d> created = PiecewiseTrajectory2d::create(cubic_chain());
  ASSERT_TRUE(created.ok());
  const PiecewiseTrajectory2d& trajectory = created.value;
  EXPECT_EQ(trajectory.evaluate(-1e-9, 0).code, Code::kOutOfRange);
  EXPECT_EQ(trajectory.evaluate(trajectory.duration_s() + 1e-9, 0).code, Code::kOutOfRange);
  EXPECT_TRUE(trajectory.evaluate(0.0, 0).ok());
  EXPECT_TRUE(trajectory.evaluate(trajectory.duration_s(), 0).ok());
  EXPECT_EQ(trajectory.evaluate(std::nan(""), 0).code, Code::kInvalidInput);
  EXPECT_EQ(trajectory.evaluate(1.0, 4).code, Code::kUnsupportedDerivative);
}

TEST(PiecewiseSampling, batch_into_preallocated_buffer_matches_single) {
  const Result<PiecewiseTrajectory2d> created = PiecewiseTrajectory2d::create(quintic_chain());
  ASSERT_TRUE(created.ok());
  const PiecewiseTrajectory2d& trajectory = created.value;
  const double times[] = {0.0, 0.5, 1.25, 4.0, 8.5, 9.0};
  Point2d output[6];
  const BatchResult result = trajectory.evaluate_batch(times, 6, 0, output);
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.processed, 6u);
  for (std::size_t i = 0; i < 6; ++i) {
    expect_point_close(output[i], trajectory.evaluate(times[i], 0).value);
  }
  const double bad_times[] = {0.5, -1.0};
  const BatchResult bad = trajectory.evaluate_batch(bad_times, 2, 0, output);
  EXPECT_EQ(bad.code, Code::kOutOfRange);
  EXPECT_EQ(bad.processed, 1u);
  EXPECT_EQ(bad.first_bad_index, 1u);
}

TEST(PiecewiseTimeScaling, preserves_positions_and_scales_duration) {
  const Result<PiecewiseTrajectory2d> created = PiecewiseTrajectory2d::create(quintic_chain());
  ASSERT_TRUE(created.ok());
  const PiecewiseTrajectory2d& trajectory = created.value;
  const Result<PiecewiseTrajectory2d> scaled = trajectory.scaled_time(2.0);
  ASSERT_TRUE(scaled.ok());
  EXPECT_DOUBLE_EQ(scaled.value.duration_s(), 2.0 * trajectory.duration_s());
  const double sample_times[] = {0.0, 3.0, 9.0, 18.0};
  for (double t : sample_times) {
    expect_point_close(scaled.value.evaluate(t, 0).value, trajectory.evaluate(t / 2.0, 0).value);
    const Point2d velocity = trajectory.evaluate(t / 2.0, 1).value;
    expect_point_close(scaled.value.evaluate(t, 1).value,
                       Point2d{velocity.x_m / 2.0, velocity.y_m / 2.0});
  }
}

TEST(PiecewiseTimeScaling, rejects_invalid_scale) {
  const Result<PiecewiseTrajectory2d> created = PiecewiseTrajectory2d::create(cubic_chain());
  ASSERT_TRUE(created.ok());
  EXPECT_EQ(created.value.scaled_time(0.0).code, Code::kInvalidInput);
  EXPECT_EQ(created.value.scaled_time(-1.0).code, Code::kInvalidInput);
  EXPECT_EQ(created.value.scaled_time(std::numeric_limits<double>::infinity()).code,
            Code::kInvalidInput);
}

TEST(PiecewiseAccessors, start_times_are_cumulative_and_indexed_access_throws) {
  const Result<PiecewiseTrajectory2d> created = PiecewiseTrajectory2d::create(cubic_chain());
  ASSERT_TRUE(created.ok());
  const PiecewiseTrajectory2d& trajectory = created.value;
  EXPECT_DOUBLE_EQ(trajectory.segment_start_time_s(0), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.segment_start_time_s(1), 1.0);
  EXPECT_DOUBLE_EQ(trajectory.segment_start_time_s(2), 3.0);
  EXPECT_EQ(trajectory.segment(1).duration_s(), 2.0);
  EXPECT_THROW(trajectory.segment(3), std::out_of_range);
  EXPECT_THROW(trajectory.segment_start_time_s(3), std::out_of_range);
}

}  // namespace
}  // namespace planner::polynomial
