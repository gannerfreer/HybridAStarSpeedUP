// 状态区分与预算：invalid_input / no_path / timeout / cancelled 明确分离，
// 覆盖全部已启用后端（适配层预校验，不依赖上游错误语义）。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "planner/grid_search/backend.hpp"
#include "planner/grid_search/types.hpp"

#include "scenarios.hpp"

namespace planner::grid_search {
namespace {

using test::Scenario;

const Scenario& representative_scenario() {
  static const Scenario scenario = [] {
    const std::vector<Scenario> scenarios = test::load_scenarios();
    for (const Scenario& candidate : scenarios) {
      if (candidate.id == "maze_25") {
        return candidate;
      }
    }
    return scenarios.front();
  }();
  return scenario;
}

GridSearchRequest base_request(const Scenario& scenario) {
  return test::make_request(scenario, Neighborhood::kEight);
}

TEST(StatusAndBudget, InvalidInputsAreRejectedByEveryBackend) {
  const Scenario& scenario = representative_scenario();
  for (const std::string& name : available_backends()) {
    std::unique_ptr<GridSearchBackend> backend = make_backend(name);
    ASSERT_NE(backend, nullptr) << name;

    // 空栅格指针。
    GridSearchRequest request = base_request(scenario);
    request.grid = nullptr;
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;

    // 零尺寸栅格。
    common::GridView empty;
    empty.width = 0;
    empty.height = 0;
    empty.resolution_m = 1.0;
    request = base_request(scenario);
    request.grid = &empty;
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;

    // 单元长度不匹配。
    common::GridView broken = scenario.grid;
    broken.cells.pop_back();
    request = base_request(scenario);
    request.grid = &broken;
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;

    // 非法三态取值。
    broken = scenario.grid;
    broken.cells[0] = 7;
    request = base_request(scenario);
    request.grid = &broken;
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;

    // 非正分辨率。
    common::GridView bad_resolution = scenario.grid;
    bad_resolution.resolution_m = 0.0;
    request = base_request(scenario);
    request.grid = &bad_resolution;
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;

    // 起终点越界。
    request = base_request(scenario);
    request.start = GridCell{-1, 0};
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;
    request = base_request(scenario);
    request.goal = GridCell{scenario.grid.height, 0};
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;

    // 起点/终点被阻塞（占据/未知）。
    common::GridView blocked = scenario.grid;
    blocked.cells[static_cast<std::size_t>(scenario.start.row) *
                      static_cast<std::size_t>(blocked.width) +
                  static_cast<std::size_t>(scenario.start.col)] =
        common::kCellOccupied;
    request = base_request(scenario);
    request.grid = &blocked;
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;
    blocked = scenario.grid;
    blocked.cells[static_cast<std::size_t>(scenario.goal.row) *
                      static_cast<std::size_t>(blocked.width) +
                  static_cast<std::size_t>(scenario.goal.col)] =
        common::kCellUnknown;
    request = base_request(scenario);
    request.grid = &blocked;
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;

    // 非法预算。
    request = base_request(scenario);
    request.time_budget_s = -1.0;
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;
    request.time_budget_s = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;
    request.time_budget_s.reset();
    request.node_budget = 0;
    EXPECT_EQ(backend->search(request).status, common::Status::kInvalidInput)
        << name;
  }
}

TEST(StatusAndBudget, PreCancelledRequestsAreCancelled) {
  const Scenario& scenario = representative_scenario();
  for (const std::string& name : available_backends()) {
    std::unique_ptr<GridSearchBackend> backend = make_backend(name);
    GridSearchRequest request = base_request(scenario);
    request.cancelled = true;
    const GridSearchResult result = backend->search(request);
    EXPECT_EQ(result.status, common::Status::kCancelled) << name;
    EXPECT_FALSE(result.reason.empty()) << name;
    EXPECT_TRUE(result.path.empty()) << name;
  }
}

TEST(StatusAndBudget, TinyTimeBudgetIsTimeoutNotNoPath) {
  const Scenario& scenario = representative_scenario();
  for (const std::string& name : available_backends()) {
    std::unique_ptr<GridSearchBackend> backend = make_backend(name);
    GridSearchRequest request = base_request(scenario);
    request.time_budget_s = 1e-9;
    const GridSearchResult result = backend->search(request);
    EXPECT_EQ(result.status, common::Status::kTimeout)
        << name << " reason=" << result.reason;
    EXPECT_FALSE(result.reason.empty()) << name;
  }
}

TEST(StatusAndBudget, StartEqualsGoalIsASingleCellSuccess) {
  const Scenario& scenario = representative_scenario();
  for (const std::string& name : available_backends()) {
    std::unique_ptr<GridSearchBackend> backend = make_backend(name);
    GridSearchRequest request = base_request(scenario);
    request.goal = request.start;
    const GridSearchResult result = backend->search(request);
    ASSERT_EQ(result.status, common::Status::kSuccess)
        << name << " reason=" << result.reason;
    ASSERT_EQ(result.path.size(), 1u) << name;
    EXPECT_EQ(result.path.front(), request.start) << name;
    EXPECT_DOUBLE_EQ(result.cost_m, 0.0) << name;
  }
}

TEST(StatusAndBudget, NodeBudgetHonoredBySmacAndIgnoredElsewhere) {
  const Scenario& scenario = representative_scenario();
  for (const std::string& name : available_backends()) {
    std::unique_ptr<GridSearchBackend> backend = make_backend(name);
    GridSearchRequest request = base_request(scenario);
    request.node_budget = 1;
    const GridSearchResult result = backend->search(request);
    if (name == "smac_2d_astar") {
      EXPECT_EQ(result.status, common::Status::kTimeout) << name;
      EXPECT_NE(result.reason.find("node budget"), std::string::npos)
          << result.reason;
    } else {
      // 其他后端不解释 node_budget（provenance.md 登记）；搜索正常完成。
      EXPECT_EQ(result.status, common::Status::kSuccess)
          << name << " reason=" << result.reason;
    }
  }
}

TEST(StatusAndBudget, UnreachableIsNoPathNotInvalidInput) {
  const std::vector<Scenario> scenarios = test::load_scenarios();
  const Scenario* unreachable = nullptr;
  for (const Scenario& scenario : scenarios) {
    if (scenario.id == "unreachable_20") {
      unreachable = &scenario;
    }
  }
  ASSERT_NE(unreachable, nullptr);
  for (const std::string& name : available_backends()) {
    std::unique_ptr<GridSearchBackend> backend = make_backend(name);
    for (Neighborhood neighborhood :
         {Neighborhood::kFour, Neighborhood::kEight}) {
      GridSearchRequest request =
          test::make_request(*unreachable, neighborhood);
      const GridSearchResult result = backend->search(request);
      if (name == "jps3d_jps" && neighborhood == Neighborhood::kFour) {
        EXPECT_EQ(result.status, common::Status::kInvalidInput) << name;
        continue;
      }
      EXPECT_EQ(result.status, common::Status::kNoPath)
          << name << " " << to_string(neighborhood);
      EXPECT_TRUE(result.path.empty()) << name;
    }
  }
}

}  // namespace
}  // namespace planner::grid_search
