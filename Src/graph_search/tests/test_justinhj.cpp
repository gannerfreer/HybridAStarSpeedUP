// S-03 justinhj 适配器测试：池开/关两组配置 + float32 精度条件项。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

#include "fixtures.hpp"
#include "planner/graph_search/adapters.hpp"

namespace planner {
namespace graph_search {
namespace {

TEST(JustinhjAdapter, PoolCommonSuite) {
  const auto searcher = make_justinhj_astar_pool();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().adapter, "justinhj_astar_pool");
  EXPECT_EQ(searcher->config().cost_precision, "float32");
  EXPECT_EQ(searcher->config().options.at("memory_pool"), "on");
  test::run_common_suite(*searcher);
}

TEST(JustinhjAdapter, NoPoolCommonSuite) {
  const auto searcher = make_justinhj_astar_nopool();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().adapter, "justinhj_astar_nopool");
  EXPECT_EQ(searcher->config().cost_precision, "float32");
  EXPECT_EQ(searcher->config().options.at("memory_pool"), "off");
  test::run_common_suite(*searcher);
}

TEST(JustinhjAdapter, PoolConfigurationsAgree) {
  const auto pool = make_justinhj_astar_pool();
  const auto nopool = make_justinhj_astar_nopool();
  ASSERT_NE(pool, nullptr);
  ASSERT_NE(nopool, nullptr);
  Control control;
  const Result with_pool = pool->search(test::fixture_a_directed(), 0, 9,
                                        zero_heuristic(), control);
  const Result without_pool = nopool->search(test::fixture_a_directed(), 0, 9,
                                             zero_heuristic(), control);
  EXPECT_EQ(with_pool.status, Status::kSuccess);
  EXPECT_EQ(without_pool.status, Status::kSuccess);
  EXPECT_EQ(with_pool.path, without_pool.path);
  EXPECT_EQ(with_pool.cost, without_pool.cost);
  // 池容量按规模设置并记录（num_vertices=12 -> max(1000, 28) = 1000）。
  EXPECT_EQ(with_pool.config.options.at("memory_pool_capacity"), "1000");
  EXPECT_EQ(without_pool.config.options.at("memory_pool_capacity"), "1000");
}

// float32 条件项：长链上原生代价使用已登记门槛，路径按 double 重算满足 §6.2。
TEST(JustinhjAdapter, Float32NativeCostRegisteredThreshold) {
  const auto searcher = make_justinhj_astar_pool();
  ASSERT_NE(searcher, nullptr);
  const Graph graph = test::fixture_f_precision_chain(32);
  Control control;
  const Result result = searcher->search(graph, 0, 32, zero_heuristic(), control);
  ASSERT_EQ(result.status, Status::kSuccess) << result.message;
  const double expected = 32 * 1.4142135623730951;
  EXPECT_TRUE(cost_close(path_cost(graph, result.path), expected));
  EXPECT_LE(std::fabs(result.cost - expected),
            test::float32_native_tolerance(expected));
  // 精度口径必须标注在结果元数据中。
  EXPECT_EQ(result.config.cost_precision, "float32");
  // 记录测量到的 float32 原生偏差（供 provenance/报告登记门槛复核）。
  const double delta = std::fabs(result.cost - expected);
  std::printf(
      "[justinhj float32] expected=%.10f native=%.10f delta=%.3e "
      "strict_tol=%.3e exceeds_strict=%s registered_tol=%.3e\n",
      expected, result.cost, delta, cost_tolerance(expected),
      cost_close(result.cost, expected) ? "no" : "yes",
      test::float32_native_tolerance(expected));
}

}  // namespace
}  // namespace graph_search
}  // namespace planner
