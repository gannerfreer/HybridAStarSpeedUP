// S-04 LEMON（OpenROAD 副本）适配器测试：点到点 + 距离场 + 跨角色一致性。

#include <gtest/gtest.h>

#include "fixtures.hpp"
#include "planner/graph_search/adapters.hpp"

namespace planner {
namespace graph_search {
namespace {

TEST(LemonAdapter, DijkstraCommonSuite) {
  const auto searcher = make_lemon_dijkstra();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().adapter, "lemon_dijkstra");
  EXPECT_EQ(searcher->config().cost_precision, "float64");
  EXPECT_FALSE(searcher->config().uses_heuristic);
  EXPECT_EQ(searcher->config().options.at("upstream_commit"),
            "31d79d686f776a62ace1925cb2e40e84a80d1394");
  EXPECT_EQ(searcher->config().options.at("upstream_baseline"), "LEMON 1.3.1");
  test::run_common_suite(*searcher);
}

TEST(LemonAdapter, DistanceFieldSuite) {
  const auto searcher = make_lemon_distance_field();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().adapter, "lemon_distance_field");
  test::run_distance_field_suite(*searcher);
}

// 点到点与完整距离场两种调用不得混用（早停后未处理节点距离不完整）；
// 两者在可达终点的距离上必须一致。
TEST(LemonAdapter, PointToPointMatchesDistanceField) {
  const auto p2p = make_lemon_dijkstra();
  const auto field = make_lemon_distance_field();
  ASSERT_NE(p2p, nullptr);
  ASSERT_NE(field, nullptr);
  Control control;
  const Result result =
      p2p->search(test::fixture_a_directed(), 0, 9, zero_heuristic(), control);
  const DistanceFieldResult distances =
      field->search(test::fixture_a_directed(), 0, control);
  ASSERT_EQ(result.status, Status::kSuccess);
  ASSERT_EQ(distances.status, Status::kSuccess);
  EXPECT_TRUE(cost_close(result.cost, distances.distances[9]));
}

}  // namespace
}  // namespace graph_search
}  // namespace planner
