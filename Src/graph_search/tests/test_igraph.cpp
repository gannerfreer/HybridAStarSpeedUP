// S-05 igraph 适配器测试：点到点 + 距离场 + GPL 隔离元数据。
// 本可执行文件链接 GPL-2.0-or-later 隔离目标 planner_graph_search_igraph，
// 属组合边界内测试；不进入默认交付库。

#include <gtest/gtest.h>

#include "fixtures.hpp"
#include "planner/graph_search/adapters.hpp"

namespace planner {
namespace graph_search {
namespace {

TEST(IgraphAdapter, DijkstraCommonSuite) {
  const auto searcher = make_igraph_dijkstra();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().adapter, "igraph_dijkstra");
  EXPECT_EQ(searcher->config().cost_precision, "float64");
  EXPECT_FALSE(searcher->config().uses_heuristic);
  EXPECT_EQ(searcher->config().options.at("upstream_version"), "1.0.1");
  test::run_common_suite(*searcher);
}

TEST(IgraphAdapter, DistanceFieldSuite) {
  const auto searcher = make_igraph_distance_field();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().adapter, "igraph_distance_field");
  test::run_distance_field_suite(*searcher);
}

TEST(IgraphAdapter, IsolationMetadataIsRecorded) {
  const auto searcher = make_igraph_dijkstra();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().options.at("license"),
            "GPL-2.0-or-later (isolation group)");
  EXPECT_NE(searcher->config().options.at("isolation").find("never linked"),
            std::string::npos);
  EXPECT_EQ(searcher->config().options.at("cost_source"),
            "sum along returned igraph path (double)");
}

// 距离场不提供前驱（igraph_distances_dijkstra 无 parents 输出），已记录。
TEST(IgraphAdapter, DistanceFieldHasNoPredecessors) {
  const auto searcher = make_igraph_distance_field();
  ASSERT_NE(searcher, nullptr);
  Control control;
  const DistanceFieldResult result =
      searcher->search(test::fixture_a_directed(), 0, control);
  ASSERT_EQ(result.status, Status::kSuccess);
  EXPECT_TRUE(result.predecessors.empty());
  EXPECT_EQ(result.config.options.at("predecessors"),
            "not provided by igraph_distances_dijkstra");
}

}  // namespace
}  // namespace graph_search
}  // namespace planner
