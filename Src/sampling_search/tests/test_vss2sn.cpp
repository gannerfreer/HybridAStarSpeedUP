// vss2sn 栅格 RRT* 适配器专项测试（栅格采样组；种子/预算/取消补丁；
// 方阵栅格与分辨率限制）。

#include <gtest/gtest.h>

#include <string>

#include "fixture.hpp"
#include "planner/sampling_search/planner.hpp"
#include "planner/sampling_search/types.hpp"

namespace planner::sampling_search {
namespace {

using test::small_fixture;

TEST(SamplingSearchVss2sn, SolvesLatticeAlignedScenarioWithValidEdges) {
  if (!vss2sn_rrtstar_available()) {
    GTEST_SKIP() << "vss2sn adapter not compiled";
  }
  const auto planner = make_vss2sn_rrtstar();
  const auto fixture = small_fixture(1.0);
  const Problem problem =
      fixture.make_problem(Point2d{5.0, 5.0}, Point2d{45.0, 45.0}, 0.5, 0);
  const Result result = planner->plan(problem, nullptr);
  ASSERT_EQ(result.status, Status::kSuccess) << result.message;
  EXPECT_TRUE(path_edges_valid(problem, result.path));
  // 对齐输入：端点精确落在格点上。
  EXPECT_DOUBLE_EQ(result.path.front().x_m, 5.0);
  EXPECT_DOUBLE_EQ(result.path.front().y_m, 5.0);
  EXPECT_DOUBLE_EQ(result.path.back().x_m, 45.0);
  EXPECT_DOUBLE_EQ(result.path.back().y_m, 45.0);
  EXPECT_GT(result.stats.iterations, 0u);
}

TEST(SamplingSearchVss2sn, NativeParamsRecordGridGroupConfiguration) {
  if (!vss2sn_rrtstar_available()) {
    GTEST_SKIP() << "vss2sn adapter not compiled";
  }
  const auto planner = make_vss2sn_rrtstar();
  const auto fixture = small_fixture(1.0);
  const Problem problem =
      fixture.make_problem(Point2d{5.0, 5.0}, Point2d{45.0, 45.0}, 0.3, 0);
  const Result result = planner->plan(problem, nullptr);
  EXPECT_NE(result.stats.native_params.find("resolution=1.000000"),
            std::string::npos);
  EXPECT_NE(result.stats.native_params.find("grid=51x51"),
            std::string::npos);
  EXPECT_NE(result.stats.native_params.find("patches=seed"), std::string::npos);
}

TEST(SamplingSearchVss2sn, RejectsNonSquareBounds) {
  if (!vss2sn_rrtstar_available()) {
    GTEST_SKIP() << "vss2sn adapter not compiled";
  }
  const auto planner = make_vss2sn_rrtstar();
  const auto fixture = small_fixture(1.0);
  Problem problem =
      fixture.make_problem(Point2d{5.0, 5.0}, Point2d{45.0, 45.0}, 0.5, 0);
  problem.bounds.max_y = 40.0;  // 50 × 40：非方阵
  const Result result = planner->plan(problem, nullptr);
  EXPECT_EQ(result.status, Status::kInvalidInput);
}

TEST(SamplingSearchVss2sn, RejectsNonMultipleBounds) {
  if (!vss2sn_rrtstar_available()) {
    GTEST_SKIP() << "vss2sn adapter not compiled";
  }
  const auto planner = make_vss2sn_rrtstar();
  const auto fixture = small_fixture(1.0);
  Problem problem =
      fixture.make_problem(Point2d{5.0, 5.0}, Point2d{45.0, 45.0}, 0.5, 0);
  problem.bounds.max_x = 50.5;
  problem.bounds.max_y = 50.5;
  const Result result = planner->plan(problem, nullptr);
  EXPECT_EQ(result.status, Status::kInvalidInput);
}

TEST(SamplingSearchVss2sn, CancelledRunDiscardsNativeResult) {
  if (!vss2sn_rrtstar_available()) {
    GTEST_SKIP() << "vss2sn adapter not compiled";
  }
  const auto planner = make_vss2sn_rrtstar();
  const auto fixture = small_fixture(1.0);
  const Problem problem =
      fixture.make_problem(Point2d{5.0, 5.0}, Point2d{45.0, 45.0}, 5.0, 0);
  CancelToken cancel;
  cancel.cancel();
  const Result result = planner->plan(problem, &cancel);
  EXPECT_EQ(result.status, Status::kCancelled);
  EXPECT_TRUE(result.path.empty());
}

}  // namespace
}  // namespace planner::sampling_search
