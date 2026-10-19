// planner_graph_search 测试夹具与共享套件（gtest）。
//
// 夹具为小型合成图，期望值来自手算并另由 tests/reference/reference.py
// （标准库独立 Dijkstra）交叉核对；容差按组件扩展计划 §6.2
// （1e-8 + 1e-8*|ref|）。justinhj 的原生 float32 代价使用已登记的
// float32 门槛（见 provenance.md §5）。

#pragma once

#include <string>
#include <vector>

#include "planner/graph_search/types.hpp"

namespace planner {
namespace graph_search {
namespace test {

struct Case {
  std::string name;
  Graph graph;
  int start = 0;
  int goal = 0;
  bool use_euclidean = false;
  std::vector<double> xs;
  std::vector<double> ys;
  Status expected_status = Status::kSuccess;
  double expected_cost = 0.0;
  bool unique_path = false;
  std::vector<int> expected_path;
  bool check_native_cost = true;
};

struct FieldCase {
  std::string name;
  Graph graph;
  int source = 0;
  std::vector<double> expected_distances;  // inf = unreachable
};

// 手算夹具（见 reference.py 的交叉核对输出）。
Graph fixture_a_directed();
std::vector<double> fixture_a_xs();
std::vector<double> fixture_a_ys();
Graph fixture_b_undirected();
Graph fixture_c_zero_weight();
Graph fixture_d_negative();
Graph fixture_e_edge_out_of_range();
Graph fixture_e_nan_weight();
Graph fixture_e_infinite_weight();
Graph fixture_f_precision_chain(int num_edges);

std::vector<Case> common_cases();
std::vector<FieldCase> field_cases();

// justinhj 原生 float32 代价的登记门槛（正确性档夹具，路径边数 <= 64）。
double float32_native_tolerance(double reference_cost);

// 共享套件：每个适配器测试文件调用一次；断言失败带用例名。
void run_common_suite(const GraphSearcher& searcher);
void run_distance_field_suite(const DistanceFieldSearcher& searcher);

}  // namespace test
}  // namespace graph_search
}  // namespace planner
