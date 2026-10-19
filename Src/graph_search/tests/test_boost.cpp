// S-01 Boost Graph 适配器测试：公共套件 + 距离场 + 夹具可采纳性。

#include <gtest/gtest.h>

#include "fixtures.hpp"
#include "planner/graph_search/adapters.hpp"

namespace planner {
namespace graph_search {
namespace {

TEST(BoostGraphAdapter, AStarCommonSuite) {
  const auto searcher = make_boost_astar();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().adapter, "boost_astar");
  EXPECT_EQ(searcher->config().cost_precision, "float64");
  EXPECT_TRUE(searcher->config().uses_heuristic);
  EXPECT_EQ(searcher->config().options.at("f_value_map"), "rank_map");
  test::run_common_suite(*searcher);
}

TEST(BoostGraphAdapter, DijkstraCommonSuite) {
  const auto searcher = make_boost_dijkstra();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().adapter, "boost_dijkstra");
  EXPECT_EQ(searcher->config().cost_precision, "float64");
  EXPECT_FALSE(searcher->config().uses_heuristic);
  test::run_common_suite(*searcher);
}

TEST(BoostGraphAdapter, DistanceFieldSuite) {
  const auto searcher = make_boost_distance_field();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().adapter, "boost_distance_field");
  EXPECT_FALSE(searcher->config().uses_heuristic);
  test::run_distance_field_suite(*searcher);
}

TEST(BoostGraphAdapter, FixtureEuclideanHeuristicIsAdmissible) {
  EXPECT_TRUE(euclidean_admissible(test::fixture_a_directed(),
                                   test::fixture_a_xs(), test::fixture_a_ys()));
}

}  // namespace
}  // namespace graph_search
}  // namespace planner
