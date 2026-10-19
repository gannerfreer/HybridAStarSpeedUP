// Klamp't/KrisLibrary 适配器专项测试（原生 rrt*、RRG* 语义披露、预算行为）。

#include <gtest/gtest.h>

#include <string>

#include "fixture.hpp"
#include "planner/sampling_search/planner.hpp"
#include "planner/sampling_search/types.hpp"

namespace planner::sampling_search {
namespace {

using test::three_wall_fixture;

TEST(SamplingSearchKlampt, SolvesProbeScenarioWithValidEdges) {
  if (!klampt_rrtstar_available()) {
    GTEST_SKIP() << "Klamp't adapter not compiled";
  }
  const auto planner = make_klampt_rrtstar();
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
  EXPECT_GT(result.stats.nodes, 0u);
  EXPECT_GE(result.stats.first_solution_s, 0.0);
}

TEST(SamplingSearchKlampt, NativeParamsDiscloseRrgSemanticsAndBidirectionalOff) {
  if (!klampt_rrtstar_available()) {
    GTEST_SKIP() << "Klamp't adapter not compiled";
  }
  const auto planner = make_klampt_rrtstar();
  const auto fixture = three_wall_fixture(1.0);
  const Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.3, 0);
  const Result result = planner->plan(problem, nullptr);
  EXPECT_NE(result.stats.native_params.find("rrg=true"), std::string::npos);
  EXPECT_NE(result.stats.native_params.find("bidirectional=false"),
            std::string::npos);
  EXPECT_NE(result.stats.native_params.find("diameter="), std::string::npos);
}

TEST(SamplingSearchKlampt, BudgetStopsLoopAndReportsWallTime) {
  if (!klampt_rrtstar_available()) {
    GTEST_SKIP() << "Klamp't adapter not compiled";
  }
  const auto planner = make_klampt_rrtstar();
  const auto fixture = test::unreachable_fixture(0.5);
  const Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{86.0, 86.0}, 0.4, 0);
  const Result result = planner->plan(problem, nullptr);
  EXPECT_EQ(result.status, Status::kTimeout);
  EXPECT_TRUE(result.path.empty());
  EXPECT_LT(result.stats.wall_s, 2.0);
}

}  // namespace
}  // namespace planner::sampling_search
