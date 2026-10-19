// Ch0p1k3 适配器专项测试（连续空间、保守栅格化、种子钩子与原生参数披露）。

#include <gtest/gtest.h>

#include <string>

#include "fixture.hpp"
#include "planner/sampling_search/planner.hpp"
#include "planner/sampling_search/types.hpp"

namespace planner::sampling_search {
namespace {

using test::three_wall_fixture;

TEST(SamplingSearchCh0p1k3, SolvesProbeScenarioWithValidEdges) {
  if (!ch0p1k3_rrtstar_available()) {
    GTEST_SKIP() << "Ch0p1k3 adapter not compiled";
  }
  const auto planner = make_ch0p1k3_rrtstar();
  const auto fixture = three_wall_fixture(1.0);
  const Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5, 0);
  const Result result = planner->plan(problem, nullptr);
  ASSERT_EQ(result.status, Status::kSuccess) << result.message;
  EXPECT_TRUE(path_edges_valid(problem, result.path));
  EXPECT_NEAR(result.path.front().x_m, 10.0, 1e-6);
  EXPECT_NEAR(result.path.front().y_m, 10.0, 1e-6);
  EXPECT_NEAR(result.path.back().x_m, 90.0, 1e-6);
  EXPECT_NEAR(result.path.back().y_m, 90.0, 1e-6);
  EXPECT_GT(result.stats.iterations, 0u);
  EXPECT_GE(result.stats.first_solution_s, 0.0);
}

TEST(SamplingSearchCh0p1k3, NativeParamsDiscloseRasterizationAndSeedHook) {
  if (!ch0p1k3_rrtstar_available()) {
    GTEST_SKIP() << "Ch0p1k3 adapter not compiled";
  }
  const auto planner = make_ch0p1k3_rrtstar();
  const auto fixture = three_wall_fixture(1.0);
  const Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.3, 0);
  const Result result = planner->plan(problem, nullptr);
  EXPECT_NE(result.stats.native_params.find("stepsize="), std::string::npos);
  EXPECT_NE(result.stats.native_params.find("raster=conservative"),
            std::string::npos);
  EXPECT_NE(result.stats.native_params.find("seed_hook"), std::string::npos);
}

TEST(SamplingSearchCh0p1k3, RejectsNonIntegralBounds) {
  if (!ch0p1k3_rrtstar_available()) {
    GTEST_SKIP() << "Ch0p1k3 adapter not compiled";
  }
  const auto planner = make_ch0p1k3_rrtstar();
  const auto fixture = three_wall_fixture(1.0);
  Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5, 0);
  problem.bounds.max_x = 100.5;
  const Result result = planner->plan(problem, nullptr);
  EXPECT_EQ(result.status, Status::kInvalidInput);
  EXPECT_FALSE(result.message.empty());
}

TEST(SamplingSearchCh0p1k3, CancelledRunDiscardsNativeResult) {
  if (!ch0p1k3_rrtstar_available()) {
    GTEST_SKIP() << "Ch0p1k3 adapter not compiled";
  }
  const auto planner = make_ch0p1k3_rrtstar();
  const auto fixture = three_wall_fixture(1.0);
  const Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 5.0, 0);
  CancelToken cancel;
  cancel.cancel();
  const Result result = planner->plan(problem, &cancel);
  EXPECT_EQ(result.status, Status::kCancelled);
  EXPECT_TRUE(result.path.empty());
}

}  // namespace
}  // namespace planner::sampling_search
