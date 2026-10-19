// OMPL 适配器专项测试（RRT* 与 Informed RRT* 变体单列、长度目标提前结束）。

#include <gtest/gtest.h>

#include <string>

#include "fixture.hpp"
#include "planner/sampling_search/planner.hpp"
#include "planner/sampling_search/types.hpp"

namespace planner::sampling_search {
namespace {

using test::open_fixture;
using test::three_wall_fixture;

TEST(SamplingSearchOmpl, BothVariantsSolveAndReportVariantNames) {
  if (!ompl_rrtstar_available()) {
    GTEST_SKIP() << "OMPL adapter not compiled";
  }
  const auto rrtstar = make_ompl_rrtstar();
  const auto informed = make_ompl_informed_rrtstar();
  ASSERT_NE(rrtstar, nullptr);
  ASSERT_NE(informed, nullptr);
  EXPECT_STREQ(rrtstar->name(), "ompl");
  EXPECT_STREQ(informed->name(), "ompl");

  const auto fixture = three_wall_fixture(1.0);
  const Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5, 0);
  const Result a = rrtstar->plan(problem, nullptr);
  const Result b = informed->plan(problem, nullptr);
  EXPECT_EQ(a.status, Status::kSuccess) << a.message;
  EXPECT_EQ(b.status, Status::kSuccess) << b.message;
  EXPECT_EQ(a.stats.variant, "rrtstar");
  EXPECT_EQ(b.stats.variant, "informed_rrtstar");
  EXPECT_TRUE(path_edges_valid(problem, a.path));
  EXPECT_TRUE(path_edges_valid(problem, b.path));
}

TEST(SamplingSearchOmpl, LengthTargetTriggersEarlyStop) {
  if (!ompl_rrtstar_available()) {
    GTEST_SKIP() << "OMPL adapter not compiled";
  }
  const auto planner = make_ompl_rrtstar();
  const auto fixture = open_fixture(100.0, 1.0);
  // 开阔场景直线代价 ≈ 113.14；长度目标 120 应在预算内提前满足。
  const Problem problem = fixture.make_problem(Point2d{10.0, 10.0},
                                               Point2d{90.0, 90.0}, 5.0, 0, 120.0);
  const Result result = planner->plan(problem, nullptr);
  ASSERT_EQ(result.status, Status::kSuccess) << result.message;
  EXPECT_LE(result.cost, 120.0 + 1e-6);
  EXPECT_LT(result.stats.wall_s, 4.0)
      << "length target should stop well before the 5 s budget";
}

TEST(SamplingSearchOmpl, UnreachableBudgetReportsTimeout) {
  if (!ompl_rrtstar_available()) {
    GTEST_SKIP() << "OMPL adapter not compiled";
  }
  const auto planner = make_ompl_informed_rrtstar();
  const auto fixture = test::unreachable_fixture(0.5);
  const Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{86.0, 86.0}, 0.3, 0);
  const Result result = planner->plan(problem, nullptr);
  EXPECT_EQ(result.status, Status::kTimeout);
  EXPECT_TRUE(result.path.empty());
}

}  // namespace
}  // namespace planner::sampling_search
