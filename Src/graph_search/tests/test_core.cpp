// 核心库单元测试：统一契约校验、启发式、代价核对、路径工具与 JSON 导出。

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>

#include "fixtures.hpp"
#include "planner/graph_search/types.hpp"

namespace planner {
namespace graph_search {
namespace {

TEST(CoreTypes, StatusStringsAreStable) {
  EXPECT_STREQ(status_to_string(Status::kSuccess), "success");
  EXPECT_STREQ(status_to_string(Status::kNoPath), "no_path");
  EXPECT_STREQ(status_to_string(Status::kTimeout), "timeout");
  EXPECT_STREQ(status_to_string(Status::kCancel), "cancel");
  EXPECT_STREQ(status_to_string(Status::kInvalidInput), "invalid_input");
}

TEST(CoreTypes, ValidateGraphAcceptsHandFixtures) {
  std::string reason = "unset";
  EXPECT_TRUE(validate_graph(test::fixture_a_directed(), &reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_TRUE(validate_graph(test::fixture_b_undirected(), &reason));
  EXPECT_TRUE(validate_graph(test::fixture_c_zero_weight(), &reason));
}

TEST(CoreTypes, ValidateGraphRejectsNegativeWeight) {
  std::string reason;
  EXPECT_FALSE(validate_graph(test::fixture_d_negative(), &reason));
  EXPECT_NE(reason.find("negative"), std::string::npos);
}

TEST(CoreTypes, ValidateGraphRejectsOutOfRangeEndpoint) {
  std::string reason;
  EXPECT_FALSE(validate_graph(test::fixture_e_edge_out_of_range(), &reason));
  EXPECT_NE(reason.find("out of range"), std::string::npos);
}

TEST(CoreTypes, ValidateGraphRejectsNonFiniteWeight) {
  std::string reason;
  EXPECT_FALSE(validate_graph(test::fixture_e_nan_weight(), &reason));
  EXPECT_FALSE(validate_graph(test::fixture_e_infinite_weight(), &reason));
}

TEST(CoreTypes, ValidVertexBounds) {
  const Graph graph = test::fixture_a_directed();
  EXPECT_TRUE(valid_vertex(graph, 0));
  EXPECT_TRUE(valid_vertex(graph, 11));
  EXPECT_FALSE(valid_vertex(graph, 12));
  EXPECT_FALSE(valid_vertex(graph, -1));
}

TEST(CoreTypes, ZeroHeuristicIsZero) {
  const Heuristic heuristic = zero_heuristic();
  EXPECT_EQ(heuristic(0), 0.0);
  EXPECT_EQ(heuristic(42), 0.0);
}

TEST(CoreTypes, EuclideanHeuristicValues) {
  EuclideanHeuristic heuristic(test::fixture_a_xs(), test::fixture_a_ys(), 9);
  ASSERT_TRUE(heuristic.valid());
  EXPECT_DOUBLE_EQ(heuristic(9), 0.0);
  EXPECT_DOUBLE_EQ(heuristic(8), 1.0);
  EXPECT_DOUBLE_EQ(heuristic(0), std::hypot(4.0, 2.0));
  // 非法构造：坐标大小不一致。
  EuclideanHeuristic invalid({0.0, 1.0}, {0.0}, 0);
  EXPECT_FALSE(invalid.valid());
  EXPECT_EQ(invalid(0), 0.0);  // 保守返回 0（可采纳）
}

TEST(CoreTypes, EuclideanAdmissibleFixtureCheck) {
  EXPECT_TRUE(euclidean_admissible(test::fixture_a_directed(),
                                   test::fixture_a_xs(), test::fixture_a_ys()));
  std::vector<double> xs = test::fixture_a_xs();
  std::vector<double> ys = test::fixture_a_ys();
  xs[9] = 100.0;  // 破坏边 8->9：euclid(8,9)=97 > w=1.0
  EXPECT_FALSE(euclidean_admissible(test::fixture_a_directed(), xs, ys));
}

TEST(CoreTypes, CostToleranceMatchesContract) {
  EXPECT_DOUBLE_EQ(cost_tolerance(0.0), 1e-8);
  EXPECT_DOUBLE_EQ(cost_tolerance(100.0), 1e-8 + 1e-6);
  EXPECT_TRUE(cost_close(1.0, 1.0));
  EXPECT_TRUE(cost_close(1.0 + 1e-8 + 1e-12, 1.0));
  EXPECT_FALSE(cost_close(1.0 + 1e-6, 1.0));
  EXPECT_FALSE(cost_close(std::numeric_limits<double>::infinity(), 1.0));
  EXPECT_FALSE(cost_close(std::numeric_limits<double>::quiet_NaN(), 1.0));
}

TEST(CoreTypes, PathCostAndValidity) {
  const Graph graph = test::fixture_a_directed();
  EXPECT_DOUBLE_EQ(path_cost(graph, {0, 4, 5, 6, 8, 9}), 5.5);
  EXPECT_TRUE(path_is_valid(graph, {0, 4, 5, 6, 8, 9}, 0, 9));
  EXPECT_FALSE(path_is_valid(graph, {0, 4, 5, 6, 8}, 0, 9));   // 终点不符
  EXPECT_FALSE(path_is_valid(graph, {0, 4, 5, 6, 8, 10}, 0, 10));  // 8->10 无
  EXPECT_TRUE(std::isinf(path_cost(graph, {})));
  EXPECT_TRUE(std::isinf(path_cost(graph, {0, 9})));  // 无边
}

TEST(CoreTypes, PathCostUsesMinimumParallelEdge) {
  Graph graph;
  graph.num_vertices = 2;
  graph.edges = {{0, 1, 2.0}, {0, 1, 1.0}};
  EXPECT_DOUBLE_EQ(path_cost(graph, {0, 1}), 1.0);
}

TEST(CoreTypes, ResultJsonExport) {
  Result result;
  result.status = Status::kSuccess;
  result.path = {0, 1};
  result.cost = 1.5;
  result.config.adapter = "unit_test";
  result.config.cost_precision = "float64";
  result.config.uses_heuristic = true;
  result.stats.expanded_nodes = 2;
  result.stats.wall_ms = 0.5;
  const nlohmann::json json = nlohmann::json::parse(to_json(result));
  EXPECT_EQ(json.at("status"), "success");
  EXPECT_EQ(json.at("path"), nlohmann::json({0, 1}));
  EXPECT_DOUBLE_EQ(json.at("cost").get<double>(), 1.5);
  EXPECT_EQ(json.at("config").at("adapter"), "unit_test");
  EXPECT_EQ(json.at("config").at("uses_heuristic").get<bool>(), true);
  EXPECT_EQ(json.at("stats").at("expanded_nodes").get<long>(), 2);
  EXPECT_TRUE(json.at("stats").at("relaxed_edges").is_null());
}

TEST(CoreTypes, DistanceFieldJsonUsesNullForUnreachable) {
  DistanceFieldResult result;
  result.status = Status::kSuccess;
  result.distances = {0.0, std::numeric_limits<double>::infinity()};
  result.predecessors = {-1, -1};
  result.config.adapter = "unit_test";
  const nlohmann::json json = nlohmann::json::parse(to_json(result));
  EXPECT_DOUBLE_EQ(json.at("distances").at(0).get<double>(), 0.0);
  EXPECT_TRUE(json.at("distances").at(1).is_null());
  EXPECT_EQ(json.at("predecessors"), nlohmann::json({-1, -1}));
}

}  // namespace
}  // namespace graph_search
}  // namespace planner
