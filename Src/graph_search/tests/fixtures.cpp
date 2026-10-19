// planner_graph_search 测试夹具与共享套件实现。
//
// 期望值来源：手算（各用例注释给出推导）+ tests/reference/reference.py 的
// 独立 Python 参考交叉核对（证据 run 输出）。共享套件对任何 GraphSearcher /
// DistanceFieldSearcher 适配器运行同一批用例。

#include "fixtures.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

namespace planner {
namespace graph_search {
namespace test {

namespace {

constexpr double kSqrt2 = 1.4142135623730951;

}  // namespace

Graph fixture_a_directed() {
  Graph g;
  g.num_vertices = 12;
  g.edges = {
      {0, 1, 1.0}, {1, 2, 1.0}, {2, 3, 1.0}, {3, 6, 1.0},
      {0, 4, 1.5}, {4, 5, 1.0}, {1, 5, 1.6}, {2, 5, 1.0},
      {5, 6, 1.0}, {5, 7, 1.2}, {7, 8, 1.0}, {8, 9, 1.0},
      {6, 8, 1.0}, {10, 11, 1.0},
  };
  return g;
}

std::vector<double> fixture_a_xs() {
  return {0, 1, 2, 3, 1, 2, 3, 2, 3, 4, 10, 11};
}

std::vector<double> fixture_a_ys() {
  return {0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 10, 10};
}

Graph fixture_b_undirected() {
  // 无向边以对称有向边表示；节点 6 孤立。
  const double undirected[9][3] = {
      {0, 1, 2.0}, {0, 2, 5.0}, {1, 2, 1.0}, {1, 3, 4.0}, {2, 3, 2.0},
      {2, 4, 7.0}, {3, 4, 1.0}, {3, 5, 3.0}, {4, 5, 2.0},
  };
  Graph g;
  g.num_vertices = 7;
  for (const auto& e : undirected) {
    g.edges.push_back({static_cast<int>(e[0]), static_cast<int>(e[1]), e[2]});
    g.edges.push_back({static_cast<int>(e[1]), static_cast<int>(e[0]), e[2]});
  }
  return g;
}

Graph fixture_c_zero_weight() {
  Graph g;
  g.num_vertices = 5;
  g.edges = {
      {0, 1, 0.0}, {1, 2, 0.0}, {0, 2, 1.0}, {2, 3, 1.0},
      {3, 4, 0.5}, {2, 4, 3.0}, {2, 0, 0.0}, {1, 0, 0.0},
  };
  return g;
}

Graph fixture_d_negative() {
  Graph g = fixture_a_directed();
  g.edges.push_back({2, 3, -1.0});
  return g;
}

Graph fixture_e_edge_out_of_range() {
  Graph g = fixture_a_directed();
  g.edges.push_back({0, 12, 1.0});
  return g;
}

Graph fixture_e_nan_weight() {
  Graph g = fixture_a_directed();
  g.edges.push_back({0, 1, std::numeric_limits<double>::quiet_NaN()});
  return g;
}

Graph fixture_e_infinite_weight() {
  Graph g = fixture_a_directed();
  g.edges.push_back({0, 1, std::numeric_limits<double>::infinity()});
  return g;
}

Graph fixture_f_precision_chain(int num_edges) {
  Graph g;
  g.num_vertices = num_edges + 1;
  for (int i = 0; i < num_edges; ++i) {
    g.edges.push_back({i, i + 1, kSqrt2});
  }
  return g;
}

double float32_native_tolerance(double reference_cost) {
  // 登记门槛（见 provenance.md §5）：正确性档夹具路径 <= 64 边，
  // 覆盖 float32 逐项舍入 + 累加误差（约 2n*2^-24*ref）。
  return 1e-5 + 1e-5 * std::fabs(reference_cost);
}

std::vector<Case> common_cases() {
  std::vector<Case> cases;
  const Graph a = fixture_a_directed();
  const std::vector<double> ax = fixture_a_xs();
  const std::vector<double> ay = fixture_a_ys();
  const Graph b = fixture_b_undirected();
  const Graph c = fixture_c_zero_weight();

  Case case_a_euclid;
  case_a_euclid.name = "a_directed_euclid_0_to_9";
  case_a_euclid.graph = a;
  case_a_euclid.start = 0;
  case_a_euclid.goal = 9;
  case_a_euclid.use_euclidean = true;
  case_a_euclid.xs = ax;
  case_a_euclid.ys = ay;
  case_a_euclid.expected_status = Status::kSuccess;
  case_a_euclid.expected_cost = 5.5;
  case_a_euclid.unique_path = true;
  case_a_euclid.expected_path = {0, 4, 5, 6, 8, 9};
  cases.push_back(case_a_euclid);

  Case case_a_zero = case_a_euclid;
  case_a_zero.name = "a_directed_zero_heuristic_0_to_9";
  case_a_zero.use_euclidean = false;
  cases.push_back(case_a_zero);

  Case case_a_no_path;
  case_a_no_path.name = "a_directed_no_path_0_to_11";
  case_a_no_path.graph = a;
  case_a_no_path.start = 0;
  case_a_no_path.goal = 11;
  case_a_no_path.expected_status = Status::kNoPath;
  cases.push_back(case_a_no_path);

  Case case_a_other_component;
  case_a_other_component.name = "a_directed_10_to_11";
  case_a_other_component.graph = a;
  case_a_other_component.start = 10;
  case_a_other_component.goal = 11;
  case_a_other_component.expected_status = Status::kSuccess;
  case_a_other_component.expected_cost = 1.0;
  case_a_other_component.unique_path = true;
  case_a_other_component.expected_path = {10, 11};
  cases.push_back(case_a_other_component);

  Case case_a_same;
  case_a_same.name = "a_directed_start_equals_goal_0_to_0";
  case_a_same.graph = a;
  case_a_same.start = 0;
  case_a_same.goal = 0;
  case_a_same.expected_status = Status::kSuccess;
  case_a_same.expected_cost = 0.0;
  case_a_same.unique_path = true;
  case_a_same.expected_path = {0};
  cases.push_back(case_a_same);

  Case case_b_equal;
  case_b_equal.name = "b_undirected_equal_cost_0_to_5";
  case_b_equal.graph = b;
  case_b_equal.start = 0;
  case_b_equal.goal = 5;
  case_b_equal.expected_status = Status::kSuccess;
  case_b_equal.expected_cost = 8.0;  // 0-1-2-3-5 与 0-1-2-3-4-5 同为 8
  cases.push_back(case_b_equal);

  Case case_b_isolated_same;
  case_b_isolated_same.name = "b_undirected_isolated_start_equals_goal_6";
  case_b_isolated_same.graph = b;
  case_b_isolated_same.start = 6;
  case_b_isolated_same.goal = 6;
  case_b_isolated_same.expected_status = Status::kSuccess;
  case_b_isolated_same.expected_cost = 0.0;
  case_b_isolated_same.unique_path = true;
  case_b_isolated_same.expected_path = {6};
  cases.push_back(case_b_isolated_same);

  Case case_b_no_path;
  case_b_no_path.name = "b_undirected_no_path_0_to_6";
  case_b_no_path.graph = b;
  case_b_no_path.start = 0;
  case_b_no_path.goal = 6;
  case_b_no_path.expected_status = Status::kNoPath;
  cases.push_back(case_b_no_path);

  Case case_c_zero;
  case_c_zero.name = "c_zero_weight_0_to_4";
  case_c_zero.graph = c;
  case_c_zero.start = 0;
  case_c_zero.goal = 4;
  case_c_zero.expected_status = Status::kSuccess;
  case_c_zero.expected_cost = 1.5;  // 0-1-2 (0) + 2-3 (1) + 3-4 (0.5)
  cases.push_back(case_c_zero);

  Case case_c_zero_direct;
  case_c_zero_direct.name = "c_zero_weight_0_to_2";
  case_c_zero_direct.graph = c;
  case_c_zero_direct.start = 0;
  case_c_zero_direct.goal = 2;
  case_c_zero_direct.expected_status = Status::kSuccess;
  case_c_zero_direct.expected_cost = 0.0;  // 0-1-2 全零权
  cases.push_back(case_c_zero_direct);

  Case case_d_negative;
  case_d_negative.name = "d_negative_weight_rejected";
  case_d_negative.graph = fixture_d_negative();
  case_d_negative.start = 0;
  case_d_negative.goal = 9;
  case_d_negative.expected_status = Status::kInvalidInput;
  cases.push_back(case_d_negative);

  Case case_e_range;
  case_e_range.name = "e_edge_endpoint_out_of_range_rejected";
  case_e_range.graph = fixture_e_edge_out_of_range();
  case_e_range.start = 0;
  case_e_range.goal = 9;
  case_e_range.expected_status = Status::kInvalidInput;
  cases.push_back(case_e_range);

  Case case_e_nan;
  case_e_nan.name = "e_nan_weight_rejected";
  case_e_nan.graph = fixture_e_nan_weight();
  case_e_nan.start = 0;
  case_e_nan.goal = 9;
  case_e_nan.expected_status = Status::kInvalidInput;
  cases.push_back(case_e_nan);

  Case case_e_inf;
  case_e_inf.name = "e_infinite_weight_rejected";
  case_e_inf.graph = fixture_e_infinite_weight();
  case_e_inf.start = 0;
  case_e_inf.goal = 9;
  case_e_inf.expected_status = Status::kInvalidInput;
  cases.push_back(case_e_inf);

  Case case_e_start;
  case_e_start.name = "e_start_out_of_range_rejected";
  case_e_start.graph = a;
  case_e_start.start = 12;
  case_e_start.goal = 9;
  case_e_start.expected_status = Status::kInvalidInput;
  cases.push_back(case_e_start);

  Case case_e_goal;
  case_e_goal.name = "e_goal_out_of_range_rejected";
  case_e_goal.graph = a;
  case_e_goal.start = 0;
  case_e_goal.goal = -1;
  case_e_goal.expected_status = Status::kInvalidInput;
  cases.push_back(case_e_goal);

  Case case_f_precision;
  case_f_precision.name = "f_precision_chain_32_edges";
  case_f_precision.graph = fixture_f_precision_chain(32);
  case_f_precision.start = 0;
  case_f_precision.goal = 32;
  case_f_precision.use_euclidean = true;
  case_f_precision.xs.resize(33);
  case_f_precision.ys.assign(33, 0.0);
  for (int i = 0; i <= 32; ++i) {
    case_f_precision.xs[static_cast<std::size_t>(i)] = i * kSqrt2;
  }
  case_f_precision.expected_status = Status::kSuccess;
  case_f_precision.expected_cost = 32 * kSqrt2;
  case_f_precision.unique_path = true;
  case_f_precision.expected_path.resize(33);
  for (int i = 0; i <= 32; ++i) {
    case_f_precision.expected_path[static_cast<std::size_t>(i)] = i;
  }
  cases.push_back(case_f_precision);

  return cases;
}

std::vector<FieldCase> field_cases() {
  std::vector<FieldCase> cases;
  const double inf = std::numeric_limits<double>::infinity();

  FieldCase a;
  a.name = "a_directed_field_from_0";
  a.graph = fixture_a_directed();
  a.source = 0;
  // 0:0, 1:1, 2:2, 3:3, 4:1.5, 5:2.5, 6:3.5, 7:3.7, 8:4.5, 9:5.5,
  // 10/11 不可达。
  a.expected_distances = {0.0, 1.0, 2.0, 3.0, 1.5, 2.5, 3.5, 3.7, 4.5, 5.5,
                          inf, inf};
  cases.push_back(a);

  FieldCase b;
  b.name = "b_undirected_field_from_0";
  b.graph = fixture_b_undirected();
  b.source = 0;
  // 0:0, 1:2, 2:3, 3:5, 4:6, 5:8, 6 不可达。
  b.expected_distances = {0.0, 2.0, 3.0, 5.0, 6.0, 8.0, inf};
  cases.push_back(b);

  FieldCase c;
  c.name = "c_zero_weight_field_from_0";
  c.graph = fixture_c_zero_weight();
  c.source = 0;
  // 0:0, 1:0, 2:0, 3:1, 4:1.5。
  c.expected_distances = {0.0, 0.0, 0.0, 1.0, 1.5};
  cases.push_back(c);

  return cases;
}

void run_common_suite(const GraphSearcher& searcher) {
  const EffectiveConfig& config = searcher.config();
  ASSERT_FALSE(config.adapter.empty());

  for (const Case& test_case : common_cases()) {
    SCOPED_TRACE("case=" + test_case.name + " adapter=" + config.adapter);
    Heuristic heuristic = test_case.use_euclidean
                              ? Heuristic(EuclideanHeuristic(
                                    test_case.xs, test_case.ys, test_case.goal))
                              : zero_heuristic();
    Control control;
    const Result result =
        searcher.search(test_case.graph, test_case.start, test_case.goal,
                        heuristic, control);
    EXPECT_EQ(result.status, test_case.expected_status)
        << "message=" << result.message;
    EXPECT_EQ(result.config.adapter, config.adapter);
    if (test_case.expected_status == Status::kSuccess ||
        test_case.expected_status == Status::kNoPath) {
      // 实际执行了搜索的用例必须给出测量值；校验拒绝等早退保持 -1。
      EXPECT_GE(result.stats.wall_ms, 0.0);
    }

    if (test_case.expected_status != Status::kSuccess) {
      EXPECT_TRUE(result.path.empty());
      if (test_case.expected_status == Status::kInvalidInput) {
        EXPECT_FALSE(result.message.empty());
      }
      continue;
    }

    ASSERT_TRUE(path_is_valid(test_case.graph, result.path, test_case.start,
                              test_case.goal))
        << "path invalid: " << result.message;
    const double recomputed = path_cost(test_case.graph, result.path);
    EXPECT_TRUE(cost_close(recomputed, test_case.expected_cost))
        << "recomputed=" << recomputed << " expected=" << test_case.expected_cost;
    if (test_case.unique_path) {
      EXPECT_EQ(result.path, test_case.expected_path);
    }
    if (test_case.check_native_cost) {
      if (config.cost_precision == "float32") {
        EXPECT_LE(std::fabs(result.cost - test_case.expected_cost),
                  float32_native_tolerance(test_case.expected_cost))
            << "native float32 cost=" << result.cost
            << " expected=" << test_case.expected_cost;
      } else {
        EXPECT_TRUE(cost_close(result.cost, test_case.expected_cost))
            << "native cost=" << result.cost
            << " expected=" << test_case.expected_cost;
      }
    }
  }

  // 取消：搜索前已置位 -> kCancel（所有适配器统一预检查）。
  {
    SCOPED_TRACE("case=cancel_before_search adapter=" + config.adapter);
    std::atomic<bool> cancel{true};
    Control control;
    control.cancel = &cancel;
    const Result result = searcher.search(
        fixture_a_directed(), 0, 9, zero_heuristic(), control);
    EXPECT_EQ(result.status, Status::kCancel);
    EXPECT_TRUE(result.path.empty());
  }

  // 超时：预算在预检查时已耗尽 -> kTimeout。
  {
    SCOPED_TRACE("case=tiny_budget_before_search adapter=" + config.adapter);
    Control control;
    control.budget_s = 1e-9;
    const Result result = searcher.search(
        fixture_a_directed(), 0, 9, zero_heuristic(), control);
    EXPECT_EQ(result.status, Status::kTimeout);
    EXPECT_TRUE(result.path.empty());
  }

  // 搜索中取消：启发式回调内置位取消标志（仅 A* 类适配器调用启发式）。
  if (config.uses_heuristic) {
    SCOPED_TRACE("case=cancel_during_search adapter=" + config.adapter);
    std::atomic<bool> cancel{false};
    int calls = 0;
    Heuristic heuristic = [&cancel, &calls](int) {
      if (++calls == 1) {
        cancel.store(true);
      }
      return 0.0;
    };
    Control control;
    control.cancel = &cancel;
    const Result result =
        searcher.search(fixture_a_directed(), 0, 9, heuristic, control);
    EXPECT_EQ(result.status, Status::kCancel);
    EXPECT_TRUE(result.path.empty());
  }

  // 搜索中超时：启发式首次调用耗时超过预算（仅 A* 类适配器）。
  if (config.uses_heuristic) {
    SCOPED_TRACE("case=timeout_during_search adapter=" + config.adapter);
    int calls = 0;
    Heuristic heuristic = [&calls](int) {
      if (++calls == 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      return 0.0;
    };
    Control control;
    control.budget_s = 0.001;  // 1 ms < 5 ms 睡眠
    const Result result =
        searcher.search(fixture_a_directed(), 0, 9, heuristic, control);
    EXPECT_EQ(result.status, Status::kTimeout);
    EXPECT_TRUE(result.path.empty());
  }
}

void run_distance_field_suite(const DistanceFieldSearcher& searcher) {
  const EffectiveConfig& config = searcher.config();
  ASSERT_FALSE(config.adapter.empty());

  for (const FieldCase& test_case : field_cases()) {
    SCOPED_TRACE("case=" + test_case.name + " adapter=" + config.adapter);
    Control control;
    const DistanceFieldResult result =
        searcher.search(test_case.graph, test_case.source, control);
    ASSERT_EQ(result.status, Status::kSuccess) << result.message;
    const std::size_t n =
        static_cast<std::size_t>(test_case.graph.num_vertices);
    ASSERT_EQ(result.distances.size(), n);
    EXPECT_GE(result.stats.wall_ms, 0.0);

    EXPECT_EQ(result.distances[static_cast<std::size_t>(test_case.source)], 0.0);
    for (std::size_t v = 0; v < n; ++v) {
      const double expected = test_case.expected_distances[v];
      if (std::isinf(expected)) {
        EXPECT_TRUE(std::isinf(result.distances[v]))
            << "v=" << v << " got=" << result.distances[v];
      } else {
        EXPECT_TRUE(cost_close(result.distances[v], expected))
            << "v=" << v << " got=" << result.distances[v]
            << " expected=" << expected;
      }
    }

    // 局部距离关系（§6.2）：对每条边 dist[to] <= dist[from] + w。
    for (const Edge& e : test_case.graph.edges) {
      const double du = result.distances[static_cast<std::size_t>(e.from)];
      const double dv = result.distances[static_cast<std::size_t>(e.to)];
      if (!std::isinf(du)) {
        EXPECT_LE(dv, du + e.weight + cost_tolerance(du + e.weight))
            << "edge " << e.from << "->" << e.to;
      }
    }

    // 前驱（若适配器提供）：源点为 -1；可达节点前驱有效且满足
    // dist[v] == dist[pred] + w(pred,v)（并行边取最小）。
    if (!result.predecessors.empty()) {
      ASSERT_EQ(result.predecessors.size(), n);
      EXPECT_EQ(result.predecessors[static_cast<std::size_t>(test_case.source)],
                -1);
      for (std::size_t v = 0; v < n; ++v) {
        if (v == static_cast<std::size_t>(test_case.source) ||
            std::isinf(result.distances[v])) {
          continue;
        }
        const int pred = result.predecessors[v];
        ASSERT_TRUE(pred >= 0 && pred < test_case.graph.num_vertices)
            << "v=" << v;
        const double edge_cost =
            path_cost(test_case.graph, {pred, static_cast<int>(v)});
        ASSERT_TRUE(std::isfinite(edge_cost));
        EXPECT_TRUE(cost_close(result.distances[v],
                               result.distances[static_cast<std::size_t>(pred)] +
                                   edge_cost))
            << "v=" << v << " pred=" << pred;
      }
    }
  }

  // 取消：搜索前已置位 -> kCancel。
  {
    SCOPED_TRACE("case=field_cancel_before_search adapter=" + config.adapter);
    std::atomic<bool> cancel{true};
    Control control;
    control.cancel = &cancel;
    const DistanceFieldResult result =
        searcher.search(fixture_a_directed(), 0, control);
    EXPECT_EQ(result.status, Status::kCancel);
  }

  // 超时：预算在预检查时已耗尽 -> kTimeout。
  {
    SCOPED_TRACE("case=field_tiny_budget adapter=" + config.adapter);
    Control control;
    control.budget_s = 1e-9;
    const DistanceFieldResult result =
        searcher.search(fixture_a_directed(), 0, control);
    EXPECT_EQ(result.status, Status::kTimeout);
  }
}

}  // namespace test
}  // namespace graph_search
}  // namespace planner
