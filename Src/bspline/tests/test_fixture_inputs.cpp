// 针对 tests/fixtures/v1/benchmarks/bspline/inputs.json 的测试夹具驱动检查。
//
// 该夹具是 v1 微基准测试的权威共享输入；
// 本库必须能使用同样的显式节点与控制点。

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "bspline_reference.hpp"
#include "planner/bspline/curve.hpp"
#include "test_support.hpp"

namespace planner::bspline {
namespace {

using test::near_point;
using test::reference_evaluate;

struct FixtureGroup {
  std::string name;
  std::vector<double> knots;
  std::vector<common::Point2d> control_points;
};

std::vector<FixtureGroup> load_groups() {
  std::ifstream stream(PLANNER_BSPLINE_BENCH_INPUTS);
  if (!stream.is_open()) {
    throw std::runtime_error(std::string("cannot open fixture: ") +
                             PLANNER_BSPLINE_BENCH_INPUTS);
  }
  nlohmann::json document;
  stream >> document;
  if (document.at("schema_version").get<int>() != 1 ||
      document.at("type").get<std::string>() != "bspline_benchmark_inputs") {
    throw std::runtime_error("unexpected fixture schema");
  }
  if (document.at("dimension").get<int>() != 2 ||
      document.at("degree").get<int>() != 3) {
    throw std::runtime_error("fixture must describe cubic 2D splines");
  }
  std::vector<FixtureGroup> groups;
  for (const nlohmann::json& group : document.at("control_groups")) {
    FixtureGroup parsed;
    parsed.name = group.at("name").get<std::string>();
    parsed.knots = group.at("knots").get<std::vector<double>>();
    for (const nlohmann::json& point : group.at("control_points")) {
      parsed.control_points.push_back(
          common::Point2d{point.at("x_m").get<double>(),
                          point.at("y_m").get<double>()});
    }
    if (parsed.knots.size() != group.at("knot_count").get<std::size_t>() ||
        parsed.control_points.size() !=
            group.at("control_point_count").get<std::size_t>()) {
      throw std::runtime_error("fixture group count mismatch");
    }
    groups.push_back(std::move(parsed));
  }
  return groups;
}

TEST(BsplineFixtureInputs, ControlGroupsMatchDocumentedCounts) {
  const std::vector<FixtureGroup> groups = load_groups();
  ASSERT_EQ(groups.size(), 3u);
  EXPECT_EQ(groups[0].name, "cp8");
  EXPECT_EQ(groups[1].name, "cp32");
  EXPECT_EQ(groups[2].name, "cp128");
  EXPECT_EQ(groups[0].control_points.size(), 8u);
  EXPECT_EQ(groups[1].control_points.size(), 32u);
  EXPECT_EQ(groups[2].control_points.size(), 128u);
}

TEST(BsplineFixtureInputs, CurvesEvaluateAgainstReference) {
  for (const FixtureGroup& group : load_groups()) {
    const std::string context = "group=" + group.name;
    const Bspline curve =
        test::require_curve(group.control_points, group.knots);
    EXPECT_EQ(curve.degree(), 3) << context;
    EXPECT_EQ(curve.control_point_count(), group.control_points.size())
        << context;
    const ParameterDomain domain = curve.parameter_domain();
    EXPECT_DOUBLE_EQ(domain.min_parameter, 0.0) << context;
    EXPECT_DOUBLE_EQ(domain.max_parameter, 1.0) << context;
    // 夹持端插值首尾控制点。
    EXPECT_TRUE(near_point(curve.evaluate(0.0, 0).value,
                           group.control_points.front(), 1e-9, 1e-9))
        << context;
    EXPECT_TRUE(near_point(curve.evaluate(1.0, 0).value,
                           group.control_points.back(), 1e-9, 1e-9))
        << context;

    std::vector<double> parameters;
    const int steps = 129;
    for (int i = 0; i <= steps; ++i) {
      parameters.push_back(static_cast<double>(i) / steps);
    }
    for (int order = 0; order <= 3; ++order) {
      std::vector<common::Point2d> buffer;
      ASSERT_EQ(curve.sample(parameters, buffer, order), Status::kSuccess)
          << context;
      for (std::size_t i = 0; i < parameters.size(); ++i) {
        const common::Point2d expected =
            reference_evaluate(group.control_points, group.knots, 3,
                               parameters[i], order);
        // 第 7.1 节容差：每个物理单位 1e-6 绝对/相对。
        EXPECT_TRUE(near_point(buffer[i], expected, 1e-6, 1e-6))
            << context << " u=" << parameters[i] << " order=" << order;
      }
    }
  }
}

TEST(BsplineFixtureInputs, ArcLengthResamplingMeetsV1Tolerance) {
  for (const FixtureGroup& group : load_groups()) {
    const std::string context = "group=" + group.name;
    const Bspline curve =
        test::require_curve(group.control_points, group.knots);
    EXPECT_GT(curve.total_length_m(), 0.0) << context;
    const ArcLengthResampleResult result = curve.sample_equal_arc_length(101);
    ASSERT_EQ(result.status, Status::kSuccess) << context;
    EXPECT_LE(result.max_interval_error_m, 1e-3) << context;
    EXPECT_LE(result.last_interval_error_m, 1e-3) << context;
    EXPECT_TRUE(near_point(result.samples.front().position,
                           group.control_points.front(), 1e-9, 1e-9))
        << context;
    EXPECT_TRUE(near_point(result.samples.back().position,
                           group.control_points.back(), 1e-9, 1e-9))
        << context;
  }
}

TEST(BsplineFixtureInputs, ShiftedConstructionVariantsWork) {
  const std::vector<FixtureGroup> groups = load_groups();
  const FixtureGroup& base = groups.front();
  for (int c = 0; c < 4; ++c) {
    std::vector<common::Point2d> shifted = base.control_points;
    for (common::Point2d& point : shifted) {
      point.x_m += 0.25 * c;
      point.y_m -= 0.125 * c;
    }
    const Bspline curve = test::require_curve(shifted, base.knots);
    const EvaluationResult result = curve.evaluate(0.5, 0);
    ASSERT_EQ(result.status, Status::kSuccess);
    // v1 基准测试的构造规则是纯平移。
    const EvaluationResult base_result = test::require_curve(
        base.control_points, base.knots).evaluate(0.5, 0);
    EXPECT_NEAR(result.value.x_m - base_result.value.x_m, 0.25 * c, 1e-9);
    EXPECT_NEAR(result.value.y_m - base_result.value.y_m, -0.125 * c, 1e-9);
  }
}

}  // namespace
}  // namespace planner::bspline
