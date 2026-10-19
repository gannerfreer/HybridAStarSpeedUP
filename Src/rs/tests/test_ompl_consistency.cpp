// OMPL 1.6.0 封装一致性测试。
//
// 重要（Docs/component_development_plan.md 3.7）：交付的原生求解器与 OMPL
// 实现同一 48 曲线族，因此下面的比较是相对锁定参照的*封装一致性测试*，
// 而非独立的双重验证。独立证据来自解析直线/圆弧用例、刚体变换不变性
// 和终点重构测试。未构建 OMPL 参照适配器时
// （PLANNER_RS_WITH_OMPL=OFF），这些测试会被跳过。

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "planner/rs/reeds_shepp.hpp"
#include "rs_test_util.hpp"

#ifdef PLANNER_RS_HAS_OMPL
#include "ompl_reference.hpp"
#endif

namespace planner::rs {
namespace {

using test::benchmark_vehicle;
using test::deterministic_queries;
using test::endpoint_position_error;
using test::endpoint_yaw_error;

TEST(ReedsSheppOmplConsistency, wrapper_consistency_distance_matches_reference) {
#ifndef PLANNER_RS_HAS_OMPL
  GTEST_SKIP() << "OMPL reference adapter is not built (PLANNER_RS_WITH_OMPL=OFF)";
#else
  const common::VehicleModel vehicle = benchmark_vehicle();
  double worst_abs = 0.0;
  double worst_rel = 0.0;
  double worst_value = 0.0;
  std::string worst_id;
  for (const test::Query& query : deterministic_queries()) {
    const double native = distance(query.start, query.goal, vehicle);
    const double reference =
        ompl_reference::distance(query.start, query.goal, vehicle);
    const double tolerance = 1e-6 + 1e-6 * std::fabs(reference);
    const double error = std::fabs(native - reference);
    EXPECT_LE(error, tolerance) << query.id << " native=" << native
                                << " reference=" << reference;
    const double relative = error / std::max(1.0, std::fabs(reference));
    if (error > worst_abs) {
      worst_abs = error;
      worst_rel = relative;
      worst_value = reference;
      worst_id = query.id;
    }
  }
  SCOPED_TRACE("worst case: " + worst_id + " reference=" +
               std::to_string(worst_value) + " abs_error=" +
               std::to_string(worst_abs) + " rel_error=" +
               std::to_string(worst_rel));
  EXPECT_LE(worst_abs, 1e-6 + 1e-6 * std::fabs(worst_value));
#endif
}

TEST(ReedsSheppOmplConsistency, wrapper_consistency_reference_path_hits_goal) {
#ifndef PLANNER_RS_HAS_OMPL
  GTEST_SKIP() << "OMPL reference adapter is not built (PLANNER_RS_WITH_OMPL=OFF)";
#else
  const common::VehicleModel vehicle = benchmark_vehicle();
  for (const test::Query& query : deterministic_queries()) {
    const RsPath reference = ompl_reference::path(query.start, query.goal, vehicle);
    EXPECT_LE(endpoint_position_error(reference), kEndpointPositionToleranceM)
        << query.id;
    EXPECT_LE(endpoint_yaw_error(reference), kEndpointYawToleranceRad) << query.id;
    double sum = 0.0;
    for (const RsSegment& segment : reference.segments) {
      sum += segment.length_m;
    }
    EXPECT_NEAR(reference.length_m, sum, 1e-9) << query.id;
    const double native = distance(query.start, query.goal, vehicle);
    EXPECT_NEAR(reference.length_m, native,
                1e-6 + 1e-6 * std::fabs(native))
        << query.id;
  }
#endif
}

TEST(ReedsSheppOmplConsistency, wrapper_consistency_rejects_invalid_input_like_component) {
#ifndef PLANNER_RS_HAS_OMPL
  GTEST_SKIP() << "OMPL reference adapter is not built (PLANNER_RS_WITH_OMPL=OFF)";
#else
  common::VehicleModel broken = benchmark_vehicle();
  broken.wheelbase_m = 0.0;
  EXPECT_THROW(ompl_reference::distance({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, broken),
               std::invalid_argument);
  EXPECT_THROW(ompl_reference::path({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, broken),
               std::invalid_argument);
#endif
}

}  // namespace
}  // namespace planner::rs
