// 测试夹具驱动的测试：实现必须复现
// tests/fixtures/v1/benchmarks/polynomial/inputs.json 的边界条件集合、
// 算例和批处理规模（第 7.3 节输入）。
//
// 夹具以文字描述边界规则；本测试实现这些文档化的规则，
// 并交叉校验手推的抽样值以及每个分段的端点残差。
// 数值容差遵循第 7.1 节。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/common/types.hpp"
#include "planner/polynomial/polynomial.hpp"
#include "planner/polynomial/result.hpp"
#include "planner/polynomial/trajectory_segment.hpp"

#ifndef PLANNER_POLYNOMIAL_FIXTURES_DIR
#error "PLANNER_POLYNOMIAL_FIXTURES_DIR must be defined by CMake"
#endif

namespace planner::polynomial {
namespace {

using planner::common::Point2d;
using Json = nlohmann::json;

constexpr double kAbsTol = 1e-6;
constexpr double kRelTol = 1e-6;

Json load_fixture() {
  const std::string path = std::string(PLANNER_POLYNOMIAL_FIXTURES_DIR) + "/inputs.json";
  std::ifstream stream(path);
  EXPECT_TRUE(stream.good()) << "cannot open fixture: " << path;
  Json document;
  stream >> document;
  return document;
}

void expect_close(double actual, double expected) {
  EXPECT_NEAR(actual, expected, kAbsTol + kRelTol * std::fabs(expected))
      << "expected: " << expected << " actual: " << actual;
}

// 来自 boundary_condition_sets.cubic 的文档化三次规则。
CubicBoundary2d cubic_boundary(std::size_t k) {
  const double kd = static_cast<double>(k);
  const int m8 = static_cast<int>(k % 8);
  const int m5 = static_cast<int>(k % 5);
  const int m3 = static_cast<int>(k % 3);
  const int m2 = static_cast<int>(k % 2);
  CubicBoundary2d boundary;
  boundary.p0_m = Point2d{0.5 * kd, 0.25 * static_cast<double>(m8 - 4)};
  boundary.v0_mps = Point2d{1.0, 0.5};
  boundary.p1_m = Point2d{0.5 * kd + 1.0 + 0.25 * static_cast<double>(m5),
                          0.25 * static_cast<double>(m8 - 2)};
  boundary.v1_mps = Point2d{1.0 + 0.25 * static_cast<double>(m3),
                            0.5 + 0.25 * static_cast<double>(m2)};
  return boundary;
}

// 来自 boundary_condition_sets.quintic 的文档化五次规则。
QuinticBoundary2d quintic_boundary(std::size_t k) {
  const double kd = static_cast<double>(k);
  const int m8 = static_cast<int>(k % 8);
  const int m4 = static_cast<int>(k % 4);
  QuinticBoundary2d boundary;
  boundary.p0_m = Point2d{0.5 * kd, 0.25 * static_cast<double>(m8 - 4)};
  boundary.v0_mps = Point2d{0.25 * static_cast<double>(m8 - 4), 0.5};
  boundary.a0_mps2 = Point2d{0.5, 0.0};
  boundary.p1_m = Point2d{0.5 * kd + 2.0, 0.25 * static_cast<double>(m8 - 2)};
  boundary.v1_mps = Point2d{0.5, 0.25 * static_cast<double>(m4 - 2)};
  boundary.a1_mps2 = Point2d{-0.5, 0.0};
  return boundary;
}

TEST(PolynomialFixture, declares_expected_batch_sizes_and_timing_boundaries) {
  const Json document = load_fixture();
  ASSERT_TRUE(document.is_object());
  EXPECT_EQ(document.at("schema_version").get<int>(), 1);
  EXPECT_EQ(document.at("type").get<std::string>(), "polynomial_benchmark_inputs");
  EXPECT_EQ(document.at("fixture_version").get<std::string>(), "v1");
  EXPECT_EQ(document.at("dimension").get<int>(), 2);

  const Json& sets = document.at("boundary_condition_sets");
  EXPECT_EQ(sets.at("cubic").at("segment_count").get<std::size_t>(), 1000u);
  EXPECT_EQ(sets.at("quintic").at("segment_count").get<std::size_t>(), 1000u);
  EXPECT_EQ(document.at("construct_batch").at("cubic").at("segments_per_batch").get<std::size_t>(),
            1000u);
  EXPECT_EQ(
      document.at("construct_batch").at("quintic").at("segments_per_batch").get<std::size_t>(),
      1000u);
  EXPECT_EQ(document.at("eval_batch").at("cubic").at("time_point_count").get<std::size_t>(),
            10000u);
  EXPECT_EQ(document.at("eval_batch").at("quintic").at("time_point_count").get<std::size_t>(),
            10000u);
  const std::vector<int> orders =
      document.at("eval_batch").at("cubic").at("derivative_orders").get<std::vector<int>>();
  EXPECT_EQ(orders, (std::vector<int>{0, 1, 2, 3}));

  const Json& timing = document.at("timing_boundaries");
  EXPECT_EQ(timing.at("warmup_batches").get<int>(), 5);
  EXPECT_EQ(timing.at("measured_batches").get<int>(), 30);
}

TEST(PolynomialFixture, validation_examples_reproduce_expected_coefficients) {
  const Json document = load_fixture();
  for (const Json& example : document.at("validation_examples")) {
    const Json& boundary = example.at("boundary");
    const std::vector<double> expected =
        example.at("expected_coefficients_ascending").get<std::vector<double>>();
    Result<Polynomial> solved = Result<Polynomial>::failure(Code::kInvalidInput);
    if (example.at("degree").get<int>() == 3) {
      solved = Polynomial::cubic(CubicBoundary{boundary.at("p0").get<double>(),
                                               boundary.at("v0").get<double>(),
                                               boundary.at("p1").get<double>(),
                                               boundary.at("v1").get<double>()},
                                 1.0);
    } else {
      solved = Polynomial::quintic(QuinticBoundary{boundary.at("p0").get<double>(),
                                                   boundary.at("v0").get<double>(),
                                                   boundary.at("a0").get<double>(),
                                                   boundary.at("p1").get<double>(),
                                                   boundary.at("v1").get<double>(),
                                                   boundary.at("a1").get<double>()},
                                   1.0);
    }
    ASSERT_TRUE(solved.ok()) << example.at("id").get<std::string>();
    ASSERT_EQ(solved.value.coefficients().size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
      SCOPED_TRACE(::testing::Message() << "index " << i);
      expect_close(solved.value.coefficients()[i], expected[i]);
    }
  }
}

TEST(PolynomialFixture, cubic_rule_matches_hand_derived_spot_values) {
  struct Expected {
    std::size_t k;
    CubicBoundary2d boundary;
  };
  // 由夹具规则手推得到（k mod 8/5/3/2）。
  const std::vector<Expected> expected{
      {0, CubicBoundary2d{Point2d{0.0, -1.0}, Point2d{1.0, 0.5}, Point2d{1.0, -0.5}, Point2d{1.0, 0.5}}},
      {1, CubicBoundary2d{Point2d{0.5, -0.75}, Point2d{1.0, 0.5}, Point2d{1.75, -0.25}, Point2d{1.25, 0.75}}},
      {5, CubicBoundary2d{Point2d{2.5, 0.25}, Point2d{1.0, 0.5}, Point2d{3.5, 0.75}, Point2d{1.5, 0.75}}},
      {8, CubicBoundary2d{Point2d{4.0, -1.0}, Point2d{1.0, 0.5}, Point2d{5.75, -0.5}, Point2d{1.5, 0.5}}},
      {999, CubicBoundary2d{Point2d{499.5, 0.75}, Point2d{1.0, 0.5}, Point2d{501.5, 1.25}, Point2d{1.0, 0.75}}},
  };
  for (const Expected& item : expected) {
    const CubicBoundary2d actual = cubic_boundary(item.k);
    expect_close(actual.p0_m.x_m, item.boundary.p0_m.x_m);
    expect_close(actual.p0_m.y_m, item.boundary.p0_m.y_m);
    expect_close(actual.v0_mps.x_m, item.boundary.v0_mps.x_m);
    expect_close(actual.v0_mps.y_m, item.boundary.v0_mps.y_m);
    expect_close(actual.p1_m.x_m, item.boundary.p1_m.x_m);
    expect_close(actual.p1_m.y_m, item.boundary.p1_m.y_m);
    expect_close(actual.v1_mps.x_m, item.boundary.v1_mps.x_m);
    expect_close(actual.v1_mps.y_m, item.boundary.v1_mps.y_m);
  }
}

TEST(PolynomialFixture, quintic_rule_matches_hand_derived_spot_values) {
  struct Expected {
    std::size_t k;
    QuinticBoundary2d boundary;
  };
  const std::vector<Expected> expected{
      {0, QuinticBoundary2d{Point2d{0.0, -1.0}, Point2d{-1.0, 0.5}, Point2d{0.5, 0.0},
                            Point2d{2.0, -0.5}, Point2d{0.5, -0.5}, Point2d{-0.5, 0.0}}},
      {3, QuinticBoundary2d{Point2d{1.5, -0.25}, Point2d{-0.25, 0.5}, Point2d{0.5, 0.0},
                            Point2d{3.5, 0.25}, Point2d{0.5, 0.25}, Point2d{-0.5, 0.0}}},
      {10, QuinticBoundary2d{Point2d{5.0, -0.5}, Point2d{-0.5, 0.5}, Point2d{0.5, 0.0},
                             Point2d{7.0, 0.0}, Point2d{0.5, 0.0}, Point2d{-0.5, 0.0}}},
      {999, QuinticBoundary2d{Point2d{499.5, 0.75}, Point2d{0.75, 0.5}, Point2d{0.5, 0.0},
                              Point2d{501.5, 1.25}, Point2d{0.5, 0.25}, Point2d{-0.5, 0.0}}},
  };
  for (const Expected& item : expected) {
    const QuinticBoundary2d actual = quintic_boundary(item.k);
    expect_close(actual.p0_m.x_m, item.boundary.p0_m.x_m);
    expect_close(actual.p0_m.y_m, item.boundary.p0_m.y_m);
    expect_close(actual.v0_mps.x_m, item.boundary.v0_mps.x_m);
    expect_close(actual.v0_mps.y_m, item.boundary.v0_mps.y_m);
    expect_close(actual.a0_mps2.x_m, item.boundary.a0_mps2.x_m);
    expect_close(actual.a0_mps2.y_m, item.boundary.a0_mps2.y_m);
    expect_close(actual.p1_m.x_m, item.boundary.p1_m.x_m);
    expect_close(actual.p1_m.y_m, item.boundary.p1_m.y_m);
    expect_close(actual.v1_mps.x_m, item.boundary.v1_mps.x_m);
    expect_close(actual.v1_mps.y_m, item.boundary.v1_mps.y_m);
    expect_close(actual.a1_mps2.x_m, item.boundary.a1_mps2.x_m);
    expect_close(actual.a1_mps2.y_m, item.boundary.a1_mps2.y_m);
  }
}

TEST(PolynomialFixture, all_thousand_cubic_segments_satisfy_boundary_residuals) {
  const Json document = load_fixture();
  const std::size_t count =
      document.at("boundary_condition_sets").at("cubic").at("segment_count").get<std::size_t>();
  ASSERT_EQ(count, 1000u);
  for (std::size_t k = 0; k < count; ++k) {
    const CubicBoundary2d boundary = cubic_boundary(k);
    const Result<TrajectorySegment> solved = TrajectorySegment::cubic(boundary, 1.0);
    ASSERT_TRUE(solved.ok()) << "segment " << k;
    const TrajectorySegment& segment = solved.value;
    expect_close(segment.evaluate(0.0, 0).value.x_m, boundary.p0_m.x_m);
    expect_close(segment.evaluate(0.0, 0).value.y_m, boundary.p0_m.y_m);
    expect_close(segment.evaluate(0.0, 1).value.x_m, boundary.v0_mps.x_m);
    expect_close(segment.evaluate(0.0, 1).value.y_m, boundary.v0_mps.y_m);
    expect_close(segment.evaluate(1.0, 0).value.x_m, boundary.p1_m.x_m);
    expect_close(segment.evaluate(1.0, 0).value.y_m, boundary.p1_m.y_m);
    expect_close(segment.evaluate(1.0, 1).value.x_m, boundary.v1_mps.x_m);
    expect_close(segment.evaluate(1.0, 1).value.y_m, boundary.v1_mps.y_m);
  }
}

TEST(PolynomialFixture, all_thousand_quintic_segments_satisfy_boundary_residuals) {
  const Json document = load_fixture();
  const std::size_t count =
      document.at("boundary_condition_sets").at("quintic").at("segment_count").get<std::size_t>();
  ASSERT_EQ(count, 1000u);
  for (std::size_t k = 0; k < count; ++k) {
    const QuinticBoundary2d boundary = quintic_boundary(k);
    const Result<TrajectorySegment> solved = TrajectorySegment::quintic(boundary, 1.0);
    ASSERT_TRUE(solved.ok()) << "segment " << k;
    const TrajectorySegment& segment = solved.value;
    for (int order = 0; order <= 2; ++order) {
      const Point2d expected_start =
          order == 0 ? boundary.p0_m : order == 1 ? boundary.v0_mps : boundary.a0_mps2;
      const Point2d expected_end =
          order == 0 ? boundary.p1_m : order == 1 ? boundary.v1_mps : boundary.a1_mps2;
      expect_close(segment.evaluate(0.0, order).value.x_m, expected_start.x_m);
      expect_close(segment.evaluate(0.0, order).value.y_m, expected_start.y_m);
      expect_close(segment.evaluate(1.0, order).value.x_m, expected_end.x_m);
      expect_close(segment.evaluate(1.0, order).value.y_m, expected_end.y_m);
    }
  }
}

TEST(PolynomialFixture, eval_time_points_are_interior_batch_positions) {
  const Json document = load_fixture();
  const std::size_t segments =
      document.at("boundary_condition_sets").at("cubic").at("segment_count").get<std::size_t>();
  const std::size_t points = document.at("eval_batch").at("cubic").at("time_point_count").get<std::size_t>();
  ASSERT_EQ(points % segments, 0u);
  const std::size_t per_segment = points / segments;
  EXPECT_EQ(per_segment, 10u);
  const Result<TrajectorySegment> solved =
      TrajectorySegment::cubic(cubic_boundary(7), 1.0);
  ASSERT_TRUE(solved.ok());
  std::vector<double> local_times(per_segment);
  for (std::size_t j = 0; j < per_segment; ++j) {
    local_times[j] = (static_cast<double>(j) + 0.5) / static_cast<double>(per_segment);
    EXPECT_GT(local_times[j], 0.0);
    EXPECT_LT(local_times[j], 1.0);
  }
  std::vector<Point2d> output(per_segment);
  const BatchResult batch = solved.value.evaluate_batch(local_times.data(), per_segment, 0,
                                                        output.data());
  EXPECT_TRUE(batch.ok());
  EXPECT_EQ(batch.processed, per_segment);
  for (std::size_t j = 0; j < per_segment; ++j) {
    expect_close(output[j].x_m, solved.value.evaluate(local_times[j], 0).value.x_m);
    expect_close(output[j].y_m, solved.value.evaluate(local_times[j], 0).value.y_m);
  }
}

}  // namespace
}  // namespace planner::polynomial
