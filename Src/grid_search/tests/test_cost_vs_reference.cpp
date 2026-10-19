// 栅格正确性档：全部后端 × 10 场景 × 四/八邻域与独立 Python 参考核对代价。
//
// 统一口径（计划 §6.2）：
//   * 精确最短路组比较代价，容差 1e-8 + 1e-8 * |reference|；
//   * 八邻域禁止对角穿角（jps3d JPS 单列变体保持原生允许穿角语义）；
//   * 不可达场景要求 no_path 且不返回路径；
//   * 所有返回路径必须通过 validate_path（端点、界内、禁穿角、邻域一致）。

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

using test::ExpectedNeighborhood;
using test::Scenario;

const std::vector<Neighborhood>& neighborhoods() {
  static const std::vector<Neighborhood> values = {Neighborhood::kFour,
                                                   Neighborhood::kEight};
  return values;
}

bool is_jps_variant(const std::string& backend) {
  return backend == "jps3d_jps";
}

TEST(CostVsReference, AllBackendsMatchIndependentPythonReference) {
  const std::vector<Scenario> scenarios = test::load_scenarios();
  const auto expected = test::load_expected();
  ASSERT_FALSE(scenarios.empty());
  ASSERT_FALSE(expected.empty());

  int checked = 0;
  for (const std::string& name : available_backends()) {
    std::unique_ptr<GridSearchBackend> backend = make_backend(name);
    ASSERT_NE(backend, nullptr) << name;
    for (const Scenario& scenario : scenarios) {
      const ExpectedNeighborhood& expected_eight_cut =
          expected.at(scenario.id).eight_corner_cut_allowed;
      for (Neighborhood neighborhood : neighborhoods()) {
        const std::string context =
            test::context_of(name, scenario, neighborhood);
        if (is_jps_variant(name) && neighborhood == Neighborhood::kFour) {
          GridSearchResult result =
              backend->search(test::make_request(scenario, neighborhood));
          EXPECT_EQ(result.status, common::Status::kInvalidInput) << context;
          continue;
        }
        const ExpectedNeighborhood& reference =
            is_jps_variant(name)
                ? expected_eight_cut
                : test::expected_for(expected.at(scenario.id), neighborhood);
        GridSearchResult result =
            backend->search(test::make_request(scenario, neighborhood));
        if (!reference.reachable) {
          EXPECT_EQ(result.status, common::Status::kNoPath) << context;
          EXPECT_TRUE(result.path.empty()) << context;
          EXPECT_TRUE(std::isnan(result.cost_m) || result.cost_m == 0.0)
              << context;
          continue;
        }
        ASSERT_EQ(result.status, common::Status::kSuccess) << context
                                                           << " reason="
                                                           << result.reason;
        EXPECT_TRUE(costs_match(result.cost_m, reference.cost_m))
            << context << " actual=" << result.cost_m
            << " reference=" << reference.cost_m;
        if (!is_jps_variant(name)) {
          // JPS 单列变体保持原生允许对角穿角语义，不适用统一禁穿角校验。
          EXPECT_EQ(validate_path(scenario.grid,
                                  test::make_request(scenario, neighborhood),
                                  result.path),
                    "")
              << context;
        } else {
          EXPECT_EQ(validate_path(scenario.grid,
                                  test::make_request(scenario, neighborhood),
                                  result.path,
                                  /*enforce_corner_rule=*/false),
                    "")
              << context;
        }
        ++checked;
      }
    }
  }
  EXPECT_GT(checked, 0);
}

TEST(CostVsReference, UnifiedBackendsAgreeWithEachOther) {
  const std::vector<Scenario> scenarios = test::load_scenarios();
  for (const Scenario& scenario : scenarios) {
    for (Neighborhood neighborhood : neighborhoods()) {
      std::vector<std::pair<std::string, double>> costs;
      for (const std::string& name : available_backends()) {
        if (is_jps_variant(name)) {
          continue;  // 单列变体不参与统一排名/一致性口径
        }
        std::unique_ptr<GridSearchBackend> backend = make_backend(name);
        const GridSearchResult result =
            backend->search(test::make_request(scenario, neighborhood));
        if (result.status == common::Status::kSuccess) {
          costs.emplace_back(name, result.cost_m);
        } else {
          EXPECT_EQ(result.status, common::Status::kNoPath)
              << test::context_of(name, scenario, neighborhood);
        }
      }
      for (const auto& entry : costs) {
        for (const auto& other : costs) {
          EXPECT_TRUE(costs_match(entry.second, other.second))
              << scenario.id << " " << to_string(neighborhood) << " "
              << entry.first << "=" << entry.second << " vs " << other.first
              << "=" << other.second;
        }
      }
    }
  }
}

// 夹具必须真实区分“禁穿角”与“允许穿角”，否则穿角规则一致性无从验证。
TEST(CostVsReference, FixtureExercisesCornerCutDifference) {
  const auto expected = test::load_expected();
  const std::vector<std::string> corner_sensitive = {
      "corner_gap_6", "border_corner_16", "maze_25", "deadend_24"};
  for (const std::string& id : corner_sensitive) {
    const auto iterator = expected.find(id);
    ASSERT_NE(iterator, expected.end()) << id;
    const ExpectedNeighborhood& eight = iterator->second.eight;
    const ExpectedNeighborhood& cut = iterator->second.eight_corner_cut_allowed;
    ASSERT_TRUE(eight.reachable) << id;
    ASSERT_TRUE(cut.reachable) << id;
    EXPECT_GT(eight.cost_m, cut.cost_m) << id;
  }
}

}  // namespace
}  // namespace planner::grid_search
