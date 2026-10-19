// S-02 libMultiRobotPlanning 适配器测试：公共套件 + 取消补丁记录。

#include <gtest/gtest.h>

#include "fixtures.hpp"
#include "planner/graph_search/adapters.hpp"

namespace planner {
namespace graph_search {
namespace {

TEST(LmrpAdapter, AStarCommonSuite) {
  const auto searcher = make_lmrp_astar();
  ASSERT_NE(searcher, nullptr);
  EXPECT_EQ(searcher->config().adapter, "lmrp_astar");
  EXPECT_EQ(searcher->config().cost_precision, "float64");
  EXPECT_TRUE(searcher->config().uses_heuristic);
  test::run_common_suite(*searcher);
}

TEST(LmrpAdapter, AdapterPatchIsRecorded) {
  const auto searcher = make_lmrp_astar();
  ASSERT_NE(searcher, nullptr);
  // 上游无取消钩子：适配层 onExpandNode 异常包装为登记补丁。
  EXPECT_EQ(searcher->config().options.at("cancel"),
            "exception-adapter-patch");
  EXPECT_EQ(searcher->config().options.at("upstream_commit"),
            "4c75fa20c435c440d8b6bd6dc81668ddc7296ba0");
}

}  // namespace
}  // namespace graph_search
}  // namespace planner
