// 配置单列与单列变体：
//   * justinhj 内存池开/关两组配置结果一致；上游 float32 代价偏差登记；
//   * jps3d JPS 单列变体（原生八邻域允许对角穿角，不混入统一排名）；
//   * Nav2 SmacPlanner2D 代价感知模式单列（cost_penalty / 代价层）。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "planner/grid_search/adapters/adapters.hpp"
#include "planner/grid_search/backend.hpp"
#include "planner/grid_search/types.hpp"

#include "scenarios.hpp"

namespace planner::grid_search {
namespace {

using test::Scenario;

TEST(Configs, JustinhjMemoryPoolOnAndOffAgree) {
  std::unique_ptr<GridSearchBackend> pooled = make_backend("justinhj_astar");
  std::unique_ptr<GridSearchBackend> unpooled =
      make_backend("justinhj_astar_nopool");
  if (pooled == nullptr || unpooled == nullptr) {
    GTEST_SKIP() << "justinhj adapters not enabled";
  }
  for (const Scenario& scenario : test::load_scenarios()) {
    for (Neighborhood neighborhood :
         {Neighborhood::kFour, Neighborhood::kEight}) {
      const GridSearchRequest request =
          test::make_request(scenario, neighborhood);
      const GridSearchResult with_pool = pooled->search(request);
      const GridSearchResult without_pool = unpooled->search(request);
      const std::string context =
          test::context_of("justinhj", scenario, neighborhood);
      EXPECT_EQ(with_pool.status, without_pool.status) << context;
      EXPECT_TRUE(costs_match(with_pool.cost_m, without_pool.cost_m))
          << context;
      ASSERT_EQ(with_pool.path.size(), without_pool.path.size()) << context;
      for (std::size_t index = 0; index < with_pool.path.size(); ++index) {
        EXPECT_EQ(with_pool.path[index], without_pool.path[index])
            << context << " index=" << index;
      }
    }
  }
}

// 上游精度差异登记（计划 §6.2：先登记后冻结分组门槛）：
// justinhj 内部 g/h/f 为 float32；统一 cost_m 由适配层按返回路径以 double
// 重算并满足 §6.2 容差；stats.upstream_cost 保留上游 float32 自报值作为证据。
TEST(Configs, JustinhjFloat32DeviationIsRegistered) {
  std::unique_ptr<GridSearchBackend> backend = make_backend("justinhj_astar");
  if (backend == nullptr) {
    GTEST_SKIP() << "justinhj adapter not enabled";
  }
  const auto expected = test::load_expected();
  double maximum_deviation = 0.0;
  int samples = 0;
  for (const Scenario& scenario : test::load_scenarios()) {
    for (Neighborhood neighborhood :
         {Neighborhood::kFour, Neighborhood::kEight}) {
      const auto iterator = expected.find(scenario.id);
      ASSERT_NE(iterator, expected.end());
      const auto& reference = test::expected_for(iterator->second, neighborhood);
      if (!reference.reachable) {
        continue;
      }
      const GridSearchResult result =
          backend->search(test::make_request(scenario, neighborhood));
      ASSERT_EQ(result.status, common::Status::kSuccess) << scenario.id;
      EXPECT_TRUE(costs_match(result.cost_m, reference.cost_m)) << scenario.id;
      const double upstream = result.stats.upstream_cost;
      ASSERT_TRUE(std::isfinite(upstream)) << scenario.id;
      // 上游自报代价为单元步长（float32）；换算到米后与参考比较（登记偏差）。
      const double deviation =
          std::fabs(upstream * scenario.grid.resolution_m - reference.cost_m);
      maximum_deviation = std::max(maximum_deviation, deviation);
      ++samples;
    }
  }
  EXPECT_GT(samples, 0);
  // 登记门槛：float32 精度偏差 < 1e-3（小规模正确性档；正式档门槛由 X-B1
  // 依据本节证据冻结）。
  EXPECT_LT(maximum_deviation, 1e-3) << "max deviation=" << maximum_deviation;
}

TEST(SingleList, JpsVariantKeepsNativeCornerSemantics) {
  std::unique_ptr<GridSearchBackend> jps = make_backend("jps3d_jps");
  if (jps == nullptr) {
    GTEST_SKIP() << "jps3d adapters not enabled";
  }
  const auto expected = test::load_expected();
  for (const Scenario& scenario : test::load_scenarios()) {
    const auto iterator = expected.find(scenario.id);
    ASSERT_NE(iterator, expected.end());
    const auto& reference = iterator->second.eight_corner_cut_allowed;
    const GridSearchResult result =
        jps->search(test::make_request(scenario, Neighborhood::kEight));
    const std::string context =
        test::context_of("jps3d_jps", scenario, Neighborhood::kEight);
    if (!reference.reachable) {
      EXPECT_EQ(result.status, common::Status::kNoPath) << context;
      continue;
    }
    ASSERT_EQ(result.status, common::Status::kSuccess)
        << context << " reason=" << result.reason;
    // 单列口径：与“允许对角穿角”的独立参考一致（原生语义），不与统一
    // 禁穿角排名混排。
    EXPECT_TRUE(costs_match(result.cost_m, reference.cost_m))
        << context << " actual=" << result.cost_m
        << " cut_allowed_reference=" << reference.cost_m;
  }
  // 四邻域对照组不适用于 JPS 变体。
  const Scenario& scenario = test::load_scenarios().front();
  const GridSearchResult four =
      jps->search(test::make_request(scenario, Neighborhood::kFour));
  EXPECT_EQ(four.status, common::Status::kInvalidInput);
}

TEST(SingleList, SmacCostAwareModeDetoursHighCostBand) {
  // 16x16 全自由栅格；col 7、rows 3..12 为高代价带（252）。
  common::GridView grid;
  grid.map_id = "cost_aware";
  grid.map_version = "unit";
  grid.width = 16;
  grid.height = 16;
  grid.resolution_m = 1.0;
  grid.cells.assign(16 * 16, common::kCellFree);
  std::vector<double> cell_costs(16 * 16, 0.0);
  for (int row = 3; row <= 12; ++row) {
    grid.cells[static_cast<std::size_t>(row) * 16 + 7] = common::kCellFree;
    cell_costs[static_cast<std::size_t>(row) * 16 + 7] = 252.0;
  }
  GridSearchRequest request;
  request.grid = &grid;
  request.start = GridCell{8, 1};
  request.goal = GridCell{8, 14};
  request.neighborhood = Neighborhood::kEight;

  std::unique_ptr<GridSearchBackend> geometric = make_backend("smac_2d_astar");
  if (geometric == nullptr) {
    GTEST_SKIP() << "smac adapter not enabled";
  }
  const GridSearchResult default_result = geometric->search(request);
  ASSERT_EQ(default_result.status, common::Status::kSuccess);
  EXPECT_TRUE(costs_match(default_result.cost_m, 13.0))
      << "actual=" << default_result.cost_m;
  bool default_crosses_band = false;
  for (const GridCell& cell : default_result.path) {
    if (cell_costs[static_cast<std::size_t>(cell.row) * 16 +
                   static_cast<std::size_t>(cell.col)] > 0.0) {
      default_crosses_band = true;
    }
  }
  EXPECT_TRUE(default_crosses_band);

  Smac2dOptions options;
  options.cost_penalty = 100.0;
  options.cell_costs = cell_costs;
  std::unique_ptr<GridSearchBackend> cost_aware =
      make_smac_2d_backend(options);
  const GridSearchResult aware_result = cost_aware->search(request);
  ASSERT_EQ(aware_result.status, common::Status::kSuccess)
      << " reason=" << aware_result.reason;
  // 单列口径：几何代价变长（绕行），但路径完全避开高代价带。
  EXPECT_GT(aware_result.cost_m, default_result.cost_m);
  for (const GridCell& cell : aware_result.path) {
    EXPECT_EQ(cell_costs[static_cast<std::size_t>(cell.row) * 16 +
                         static_cast<std::size_t>(cell.col)],
              0.0)
        << "cost-aware path crosses the high-cost band at (" << cell.row << ","
        << cell.col << ")";
  }
  EXPECT_EQ(validate_path(grid, request, aware_result.path), "");
}

}  // namespace
}  // namespace planner::grid_search
