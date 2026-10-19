// 距离场正确性：与独立 Python 参考逐单元核对 + 语义（源点为零、不可达标记、
// 局部距离关系）与点到点一致性。
//
// 距离场角色当前由 vss2sn Dijkstra 适配补丁 P3 提供（上游无此 API）；未启用
// 该开关时测试跳过。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "planner/grid_search/backend.hpp"
#include "planner/grid_search/distance_field.hpp"
#include "planner/grid_search/neighborhood.hpp"
#include "planner/grid_search/types.hpp"

#include "scenarios.hpp"

namespace planner::grid_search {
namespace {

using test::ExpectedNeighborhood;
using test::Scenario;

constexpr const char* kDistanceFieldBackend = "vss2sn_dijkstra";

std::unique_ptr<GridSearchBackend> distance_field_backend() {
  for (const std::string& name : available_backends()) {
    if (name == kDistanceFieldBackend) {
      return make_backend(name);
    }
  }
  return nullptr;
}

DistanceFieldRequest make_field_request(const Scenario& scenario,
                                        Neighborhood neighborhood) {
  DistanceFieldRequest request;
  request.grid = &scenario.grid;
  request.source = scenario.distance_source;
  request.neighborhood = neighborhood;
  return request;
}

TEST(DistanceField, MatchesIndependentPythonReference) {
  std::unique_ptr<GridSearchBackend> backend = distance_field_backend();
  if (backend == nullptr) {
    GTEST_SKIP() << "distance-field backend not enabled";
  }
  ASSERT_TRUE(backend->supports_distance_field());
  const std::vector<Scenario> scenarios = test::load_scenarios();
  const auto expected = test::load_expected();
  int cells_checked = 0;
  for (const Scenario& scenario : scenarios) {
    for (Neighborhood neighborhood :
         {Neighborhood::kFour, Neighborhood::kEight}) {
      const ExpectedNeighborhood& reference =
          test::expected_for(expected.at(scenario.id), neighborhood);
      ASSERT_EQ(reference.distance_field_m.size(),
                static_cast<std::size_t>(scenario.grid.width) *
                    static_cast<std::size_t>(scenario.grid.height));
      const DistanceFieldResult result =
          backend->distance_field(make_field_request(scenario, neighborhood));
      const std::string context =
          test::context_of(kDistanceFieldBackend, scenario, neighborhood);
      ASSERT_EQ(result.status, common::Status::kSuccess)
          << context << " reason=" << result.reason;
      EXPECT_EQ(result.field.width, scenario.grid.width) << context;
      EXPECT_EQ(result.field.height, scenario.grid.height) << context;

      std::int64_t unreachable = 0;
      for (int row = 0; row < scenario.grid.height; ++row) {
        for (int col = 0; col < scenario.grid.width; ++col) {
          const std::size_t index =
              static_cast<std::size_t>(row) *
                  static_cast<std::size_t>(scenario.grid.width) +
              static_cast<std::size_t>(col);
          const double actual = result.field.at(row, col);
          const double reference_value = reference.distance_field_m[index];
          if (std::isinf(reference_value)) {
            EXPECT_TRUE(std::isinf(actual))
                << context << " cell(" << row << "," << col << ")";
            ++unreachable;
          } else {
            EXPECT_TRUE(costs_match(actual, reference_value))
                << context << " cell(" << row << "," << col
                << ") actual=" << actual << " reference=" << reference_value;
            ++cells_checked;
          }
        }
      }
      EXPECT_EQ(result.field.unreachable_count(), unreachable) << context;
      EXPECT_EQ(unreachable, reference.unreachable_cells) << context;
      EXPECT_TRUE(costs_match(result.field.max_reachable_m(),
                              reference.max_distance_m))
          << context;
    }
  }
  EXPECT_GT(cells_checked, 0);
}

TEST(DistanceField, SourceZeroAndBlockedUnreachable) {
  std::unique_ptr<GridSearchBackend> backend = distance_field_backend();
  if (backend == nullptr) {
    GTEST_SKIP() << "distance-field backend not enabled";
  }
  for (const Scenario& scenario : test::load_scenarios()) {
    for (Neighborhood neighborhood :
         {Neighborhood::kFour, Neighborhood::kEight}) {
      const DistanceFieldResult result =
          backend->distance_field(make_field_request(scenario, neighborhood));
      ASSERT_EQ(result.status, common::Status::kSuccess) << scenario.id;
      EXPECT_DOUBLE_EQ(
          result.field.at(scenario.distance_source.row,
                          scenario.distance_source.col),
          0.0)
          << scenario.id;
      for (int row = 0; row < scenario.grid.height; ++row) {
        for (int col = 0; col < scenario.grid.width; ++col) {
          if (cell_blocked(scenario.grid, row, col)) {
            EXPECT_FALSE(result.field.reachable(row, col))
                << scenario.id << " blocked cell(" << row << "," << col << ")";
          } else {
            EXPECT_GE(result.field.at(row, col), 0.0) << scenario.id;
          }
        }
      }
    }
  }
}

TEST(DistanceField, LocalTriangleRelations) {
  std::unique_ptr<GridSearchBackend> backend = distance_field_backend();
  if (backend == nullptr) {
    GTEST_SKIP() << "distance-field backend not enabled";
  }
  for (const Scenario& scenario : test::load_scenarios()) {
    for (Neighborhood neighborhood :
         {Neighborhood::kFour, Neighborhood::kEight}) {
      const DistanceFieldResult result =
          backend->distance_field(make_field_request(scenario, neighborhood));
      ASSERT_EQ(result.status, common::Status::kSuccess) << scenario.id;
      const double tolerance = 1e-9;
      for (int row = 0; row < scenario.grid.height; ++row) {
        for (int col = 0; col < scenario.grid.width; ++col) {
          if (cell_blocked(scenario.grid, row, col)) {
            continue;
          }
          const double here = result.field.at(row, col);
          if (!std::isfinite(here)) {
            // 封闭区域内的自由单元可以不可达（其正确性由与独立参考的
            // 逐单元比较覆盖）；此处只检查局部关系。
            continue;
          }
          if (row == scenario.distance_source.row &&
              col == scenario.distance_source.col) {
            EXPECT_DOUBLE_EQ(here, 0.0);
            continue;
          }
          // 每个可达非源单元必须有一个距离严格更小的允许邻居（正步长最短路
          // 场的局部关系），且任何允许邻居满足三角不等式。
          bool has_predecessor = false;
          for (const NeighborStep& step : neighborhood_steps(neighborhood)) {
            const int next_row = row + step.d_row;
            const int next_col = col + step.d_col;
            if (cell_blocked(scenario.grid, next_row, next_col)) {
              continue;
            }
            if (!diagonal_corner_allowed(scenario.grid, row, col, step)) {
              continue;
            }
            const double there = result.field.at(next_row, next_col);
            ASSERT_TRUE(std::isfinite(there)) << scenario.id;
            const double step_m = step.cost * scenario.grid.resolution_m;
            EXPECT_LE(std::fabs(here - there), step_m + tolerance)
                << scenario.id << " (" << row << "," << col << ")";
            if (there + step_m <= here + tolerance) {
              has_predecessor = true;
            }
          }
          EXPECT_TRUE(has_predecessor)
              << scenario.id << " cell(" << row << "," << col
              << ") has no decreasing neighbor";
        }
      }
    }
  }
}

TEST(DistanceField, PointToPointCostMatchesFieldAtGoal) {
  std::unique_ptr<GridSearchBackend> backend = distance_field_backend();
  if (backend == nullptr) {
    GTEST_SKIP() << "distance-field backend not enabled";
  }
  const auto expected = test::load_expected();
  for (const Scenario& scenario : test::load_scenarios()) {
    for (Neighborhood neighborhood :
         {Neighborhood::kFour, Neighborhood::kEight}) {
      const ExpectedNeighborhood& reference =
          test::expected_for(expected.at(scenario.id), neighborhood);
      if (!reference.reachable) {
        continue;
      }
      const DistanceFieldResult field =
          backend->distance_field(make_field_request(scenario, neighborhood));
      ASSERT_EQ(field.status, common::Status::kSuccess) << scenario.id;
      // 点到点搜索必须与距离场同源（场源 = distance_source）。
      GridSearchRequest request = test::make_request(scenario, neighborhood);
      request.start = scenario.distance_source;
      const GridSearchResult search = backend->search(request);
      ASSERT_EQ(search.status, common::Status::kSuccess) << scenario.id;
      EXPECT_TRUE(costs_match(field.field.at(scenario.goal.row,
                                             scenario.goal.col),
                              search.cost_m))
          << scenario.id;
    }
  }
}

TEST(DistanceField, InvalidSourceAndCancelAreDistinguished) {
  std::unique_ptr<GridSearchBackend> backend = distance_field_backend();
  if (backend == nullptr) {
    GTEST_SKIP() << "distance-field backend not enabled";
  }
  const std::vector<Scenario> scenarios = test::load_scenarios();
  const Scenario& scenario = scenarios.front();
  DistanceFieldRequest request =
      make_field_request(scenario, Neighborhood::kEight);
  request.source = GridCell{-1, 0};
  DistanceFieldResult result = backend->distance_field(request);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);

  request = make_field_request(scenario, Neighborhood::kEight);
  request.cancelled = true;
  result = backend->distance_field(request);
  EXPECT_EQ(result.status, common::Status::kCancelled);

  request = make_field_request(scenario, Neighborhood::kEight);
  request.time_budget_s = 1e-9;
  result = backend->distance_field(request);
  EXPECT_EQ(result.status, common::Status::kTimeout);
}

TEST(DistanceField, OtherBackendsReportUnsupported) {
  for (const std::string& name : available_backends()) {
    std::unique_ptr<GridSearchBackend> backend = make_backend(name);
    if (backend->supports_distance_field()) {
      EXPECT_EQ(name, kDistanceFieldBackend);
      continue;
    }
    const std::vector<Scenario> scenarios = test::load_scenarios();
    const DistanceFieldResult result =
        backend->distance_field(make_field_request(scenarios.front(),
                                                   Neighborhood::kEight));
    EXPECT_EQ(result.status, common::Status::kInvalidInput) << name;
    EXPECT_NE(result.reason.find("distance-field"), std::string::npos) << name;
  }
}

}  // namespace
}  // namespace planner::grid_search
