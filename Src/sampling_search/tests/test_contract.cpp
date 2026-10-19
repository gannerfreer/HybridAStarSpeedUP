// X-S4 统一契约测试：对全部已编译适配器核对返回边有效性、端点容差、
// 取消/超时语义、预算耗尽不报不可达、多种子成功率烟测与无效输入。
//
// 场景分组：连续空间组（ompl / ch0p1k3 / klampt）使用三墙场景；
// 栅格采样组（vss2sn）使用对齐小场景（与连续组分开评分）。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fixture.hpp"
#include "planner/sampling_search/planner.hpp"
#include "planner/sampling_search/types.hpp"

namespace planner::sampling_search {
namespace {

using test::open_fixture;
using test::RectFixture;
using test::small_fixture;
using test::three_wall_fixture;
using test::unreachable_fixture;

bool is_grid_backend(const std::string& name) {
  return name == "vss2sn";
}

// 统一契约测试使用对齐小场景（50×50；连续与栅格组都可运行）。
// 100×100 三墙探针场景只在连续组测试中使用（栅格组分开评分）。
RectFixture fixture_for(const std::string& /*name*/) {
  return small_fixture(1.0);
}

void expect_success_invariants(const Result& result, const Problem& problem,
                               double endpoint_tolerance = 1e-6) {
  ASSERT_EQ(result.status, Status::kSuccess) << result.message;
  ASSERT_FALSE(result.path.empty());
  ASSERT_GE(result.path.size(), 2u);
  EXPECT_TRUE(path_edges_valid(problem, result.path));
  EXPECT_NEAR(result.path.front().x_m, problem.start.x_m, endpoint_tolerance);
  EXPECT_NEAR(result.path.front().y_m, problem.start.y_m, endpoint_tolerance);
  EXPECT_NEAR(result.path.back().x_m, problem.goal.x_m, endpoint_tolerance);
  EXPECT_NEAR(result.path.back().y_m, problem.goal.y_m, endpoint_tolerance);
  EXPECT_NEAR(result.cost, polyline_length(result.path), 1e-9);
  EXPECT_GE(result.stats.first_solution_s, 0.0);
}

TEST(SamplingSearchContract, SolvesScenarioAndReturnsValidEdges) {
  for (auto& planner : make_available_planners()) {
    SCOPED_TRACE(planner->name());
    const RectFixture fixture = fixture_for(planner->name());
    const Problem problem =
        fixture.make_problem(Point2d{5.0, 5.0}, Point2d{45.0, 45.0}, 1.0, 0);
    const Result result = planner->plan(problem, nullptr);
    expect_success_invariants(result, problem);
    EXPECT_EQ(result.stats.backend, std::string(planner->name()));
    EXPECT_GT(result.stats.budget_s, 0.0);
    EXPECT_GE(result.stats.wall_s, 0.0);
    EXPECT_FALSE(result.stats.variant.empty());
  }
}

TEST(SamplingSearchContract, ProbeScenarioAlsoSolvesForContinuousGroup) {
  for (auto& planner : make_available_planners()) {
    if (is_grid_backend(planner->name())) {
      continue;  // 栅格组不参与 100×100 连续探针场景比较
    }
    SCOPED_TRACE(planner->name());
    const RectFixture fixture = three_wall_fixture(1.0);
    const Problem problem =
        fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5, 0);
    const Result result = planner->plan(problem, nullptr);
    expect_success_invariants(result, problem);
  }
}

TEST(SamplingSearchContract, MultiSeedSmokeSuccessRate) {
  for (auto& planner : make_available_planners()) {
    SCOPED_TRACE(planner->name());
    const RectFixture fixture = fixture_for(planner->name());
    int successes = 0;
    int trials = 5;
    for (std::uint64_t seed = 0; seed < static_cast<std::uint64_t>(trials);
         ++seed) {
      const Problem problem = fixture.make_problem(Point2d{5.0, 5.0},
                                                   Point2d{45.0, 45.0}, 0.5, seed);
      const Result result = planner->plan(problem, nullptr);
      if (result.status == Status::kSuccess) {
        ++successes;
        EXPECT_TRUE(path_edges_valid(problem, result.path))
            << "seed=" << seed << " " << result.message;
      }
    }
    EXPECT_GE(successes, 4) << planner->name() << " solved " << successes << "/"
                            << trials << " seeds";
  }
}

TEST(SamplingSearchContract, UnreachableGoalTimesOutNotNoPath) {
  for (auto& planner : make_available_planners()) {
    SCOPED_TRACE(planner->name());
    const double step = is_grid_backend(planner->name()) ? 1.0 : 0.5;
    const RectFixture fixture = unreachable_fixture(step);
    const Problem problem =
        fixture.make_problem(Point2d{10.0, 10.0}, Point2d{86.0, 86.0}, 0.3, 0);
    const Result result = planner->plan(problem, nullptr);
    // 预算耗尽无解不是不可达证明：必须报 kTimeout（而非 kNoPath），且无部分路径。
    EXPECT_EQ(result.status, Status::kTimeout) << result.message;
    EXPECT_TRUE(result.path.empty());
    EXPECT_NE(result.status, Status::kNoPath);
  }
}

TEST(SamplingSearchContract, CancelBeforeStartReturnsCancelled) {
  for (auto& planner : make_available_planners()) {
    SCOPED_TRACE(planner->name());
    const double step = is_grid_backend(planner->name()) ? 1.0 : 0.5;
    const RectFixture fixture = unreachable_fixture(step);
    const Problem problem =
        fixture.make_problem(Point2d{10.0, 10.0}, Point2d{86.0, 86.0}, 5.0, 0);
    CancelToken cancel;
    cancel.cancel();
    const Result result = planner->plan(problem, &cancel);
    EXPECT_EQ(result.status, Status::kCancelled) << result.message;
    EXPECT_TRUE(result.path.empty());
  }
}

TEST(SamplingSearchContract, ThreadedCancelDiscardsPartialResult) {
  for (auto& planner : make_available_planners()) {
    SCOPED_TRACE(planner->name());
    const double step = is_grid_backend(planner->name()) ? 1.0 : 0.5;
    const RectFixture fixture = unreachable_fixture(step);
    const Problem problem =
        fixture.make_problem(Point2d{10.0, 10.0}, Point2d{86.0, 86.0}, 5.0, 0);
    CancelToken cancel;
    std::thread canceller([&cancel]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      cancel.cancel();
    });
    const Result result = planner->plan(problem, &cancel);
    canceller.join();
    EXPECT_EQ(result.status, Status::kCancelled) << result.message;
    EXPECT_TRUE(result.path.empty());
    EXPECT_LT(result.stats.wall_s, 4.0);
  }
}

TEST(SamplingSearchContract, InvalidInputIsRejectedByAllBackends) {
  for (auto& planner : make_available_planners()) {
    SCOPED_TRACE(planner->name());
    const RectFixture fixture = open_fixture();
    const Problem valid =
        fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5, 0);

    Problem bad = valid;
    bad.budget_s = 0.0;
    EXPECT_EQ(planner->plan(bad, nullptr).status, Status::kInvalidInput);

    bad = valid;
    bad.edge_check_step = 0.0;
    EXPECT_EQ(planner->plan(bad, nullptr).status, Status::kInvalidInput);

    bad = valid;
    bad.start = Point2d{-5.0, 10.0};
    EXPECT_EQ(planner->plan(bad, nullptr).status, Status::kInvalidInput);

    const RectFixture walls = three_wall_fixture();
    bad = walls.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5, 0);
    bad.goal = Point2d{35.0, 30.0};  // 位于三墙场景障碍内
    EXPECT_EQ(planner->plan(bad, nullptr).status, Status::kInvalidInput);

    bad = valid;
    bad.point_valid = nullptr;
    EXPECT_EQ(planner->plan(bad, nullptr).status, Status::kInvalidInput);

    bad = valid;
    bad.edge_valid = nullptr;
    EXPECT_EQ(planner->plan(bad, nullptr).status, Status::kInvalidInput);
  }
}

TEST(SamplingSearchContract, StartEqualsGoalSucceedsWithZeroCost) {
  for (auto& planner : make_available_planners()) {
    SCOPED_TRACE(planner->name());
    const RectFixture fixture = open_fixture();
    const Problem problem =
        fixture.make_problem(Point2d{10.0, 10.0}, Point2d{10.0, 10.0}, 1.0, 0);
    const Result result = planner->plan(problem, nullptr);
    ASSERT_EQ(result.status, Status::kSuccess) << result.message;
    ASSERT_FALSE(result.path.empty());
    EXPECT_NEAR(result.path.front().x_m, 10.0, 1e-6);
    EXPECT_NEAR(result.path.front().y_m, 10.0, 1e-6);
    EXPECT_NEAR(result.path.back().x_m, 10.0, 1e-6);
    EXPECT_NEAR(result.path.back().y_m, 10.0, 1e-6);
    EXPECT_NEAR(result.cost, 0.0, 1e-6);
    // 平凡情形由公共助手统一处理（不调用后端）。
    EXPECT_NE(result.stats.native_params.find("trivial"), std::string::npos);
  }
}

TEST(SamplingSearchContract, StatsAreRecordedOnEveryStatus) {
  for (auto& planner : make_available_planners()) {
    SCOPED_TRACE(planner->name());
    const RectFixture fixture = fixture_for(planner->name());
    const Problem problem =
        fixture.make_problem(Point2d{5.0, 5.0}, Point2d{45.0, 45.0}, 0.5, 1);
    const Result result = planner->plan(problem, nullptr);
    EXPECT_EQ(result.stats.backend, std::string(planner->name()));
    EXPECT_DOUBLE_EQ(result.stats.budget_s, 0.5);
    EXPECT_FALSE(result.stats.native_params.empty());
  }
}

}  // namespace
}  // namespace planner::sampling_search
