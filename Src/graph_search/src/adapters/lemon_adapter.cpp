// S-04 LEMON（OpenROAD 副本）适配器：Dijkstra 点到点 + 完整距离场。
//
// 上游：vendored include 集（副本 commit 31d79d68，基线 LEMON 1.3.1；
// 见 src/upstream/lemon/PROVENANCE.md）。
// 适配要点（family_b 探针 §2.4）：
//  * 点到点与完整距离场分离；点到点用公开的 init/addSource/processNextNode
//    手动循环（等价 run(s,t) 早停语义）以插入预算/取消检查；
//  * 不可达用 reached() 谓词判定，统一输出 +inf，不使用数值哨兵；
//  * ListDigraph + ArcMap<double>；建图成本计入 preprocess_ms。

#include <lemon/dijkstra.h>
#include <lemon/list_graph.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "detail/control.hpp"
#include "planner/graph_search/adapters.hpp"

namespace planner {
namespace graph_search {
namespace {

using LGraph = lemon::ListDigraph;
using LWeightMap = LGraph::ArcMap<double>;
using LDijkstra = lemon::Dijkstra<LGraph, LWeightMap>;

struct LemonGraph {
  // LEMON 图/映射不可拷贝，按输入就地构建。
  explicit LemonGraph(const Graph& input) : weights(graph), ids(graph) {
    nodes.reserve(static_cast<std::size_t>(input.num_vertices));
    for (int i = 0; i < input.num_vertices; ++i) {
      nodes.push_back(graph.addNode());
    }
    for (int i = 0; i < input.num_vertices; ++i) {
      ids[nodes[static_cast<std::size_t>(i)]] = i;
    }
    for (const Edge& e : input.edges) {
      LGraph::Arc arc = graph.addArc(
          nodes[static_cast<std::size_t>(e.from)],
          nodes[static_cast<std::size_t>(e.to)]);
      weights[arc] = e.weight;
    }
  }

  LGraph graph;
  std::vector<LGraph::Node> nodes;
  LWeightMap weights;
  LGraph::NodeMap<int> ids;
};

EffectiveConfig lemon_config(const char* adapter, bool distance_field) {
  EffectiveConfig config;
  config.adapter = adapter;
  config.cost_precision = "float64";
  config.supports_budget = true;
  config.supports_cancel = true;
  config.has_early_stop = !distance_field;
  config.uses_heuristic = false;
  config.options["upstream_commit"] =
      "31d79d686f776a62ace1925cb2e40e84a80d1394";
  config.options["upstream_baseline"] = "LEMON 1.3.1";
  config.options["copy"] = "OpenROAD lemon-graph";
  config.options["graph"] = "ListDigraph + ArcMap<double>";
  config.options["cancel"] = "per-node-check (public processNextNode loop)";
  config.options["unreachable"] = "reached() predicate -> +inf";
  config.options["mode"] = distance_field ? "full-distance-field"
                                          : "point-to-point-early-stop";
  return config;
}

class LemonDijkstraSearcher : public GraphSearcher {
 public:
  LemonDijkstraSearcher() : config_(lemon_config("lemon_dijkstra", false)) {}

  const EffectiveConfig& config() const override { return config_; }

  Result search(const Graph& graph, int start, int goal,
                const Heuristic& heuristic,
                const Control& control) const override {
    (void)heuristic;  // Dijkstra 语义：忽略启发式（等价零启发式）
    Result result;
    result.config = config_;

    std::string reason;
    if (!validate_graph(graph, &reason)) {
      result.status = Status::kInvalidInput;
      result.message = reason;
      return result;
    }
    if (!valid_vertex(graph, start) || !valid_vertex(graph, goal)) {
      result.status = Status::kInvalidInput;
      result.message = "start or goal vertex out of range";
      return result;
    }

    const detail::Clock::time_point t0 = detail::Clock::now();
    const Status pre = detail::precheck_control(control, t0);
    if (pre != Status::kSuccess) {
      result.status = pre;
      result.message = pre == Status::kCancel ? "cancel requested before search"
                                              : "budget exhausted before search";
      return result;
    }

    LemonGraph lemon(graph);
    const double preprocess_ms = detail::elapsed_ms(t0);

    LDijkstra dijkstra(lemon.graph, lemon.weights);
    dijkstra.init();
    dijkstra.addSource(lemon.nodes[static_cast<std::size_t>(start)]);

    const detail::Clock::time_point t_search = detail::Clock::now();
    long expanded = 0;
    bool found = false;
    while (!dijkstra.emptyQueue()) {
      if (detail::cancel_requested(control)) {
        result.status = Status::kCancel;
        result.message = "cancelled via per-node budget/cancel check";
        break;
      }
      if (detail::budget_exceeded(control, t0)) {
        result.status = Status::kTimeout;
        result.message = "time budget exceeded via per-node check";
        break;
      }
      LGraph::Node node = dijkstra.processNextNode();
      ++expanded;
      if (node == lemon.nodes[static_cast<std::size_t>(goal)]) {
        found = true;
        break;
      }
    }

    result.stats.expanded_nodes = expanded;
    result.stats.preprocess_ms = preprocess_ms;
    result.stats.search_ms = detail::elapsed_ms(t_search);
    result.stats.wall_ms = detail::elapsed_ms(t0);
    if (result.status == Status::kCancel || result.status == Status::kTimeout) {
      return result;
    }
    if (!found) {
      result.status = Status::kNoPath;
      return result;
    }

    result.status = Status::kSuccess;
    result.cost = dijkstra.dist(lemon.nodes[static_cast<std::size_t>(goal)]);
    std::vector<int> reversed;
    for (LGraph::Node node = lemon.nodes[static_cast<std::size_t>(goal)];
         node != lemon::INVALID; node = dijkstra.predNode(node)) {
      reversed.push_back(lemon.ids[node]);
      if (node == lemon.nodes[static_cast<std::size_t>(start)]) {
        break;
      }
    }
    std::reverse(reversed.begin(), reversed.end());
    result.path = std::move(reversed);
    return result;
  }

 private:
  EffectiveConfig config_;
};

class LemonDistanceFieldSearcher : public DistanceFieldSearcher {
 public:
  LemonDistanceFieldSearcher()
      : config_(lemon_config("lemon_distance_field", true)) {}

  const EffectiveConfig& config() const override { return config_; }

  DistanceFieldResult search(const Graph& graph, int source,
                             const Control& control) const override {
    DistanceFieldResult result;
    result.config = config_;

    std::string reason;
    if (!validate_graph(graph, &reason)) {
      result.status = Status::kInvalidInput;
      result.message = reason;
      return result;
    }
    if (!valid_vertex(graph, source)) {
      result.status = Status::kInvalidInput;
      result.message = "source vertex out of range";
      return result;
    }

    const detail::Clock::time_point t0 = detail::Clock::now();
    const Status pre = detail::precheck_control(control, t0);
    if (pre != Status::kSuccess) {
      result.status = pre;
      result.message = pre == Status::kCancel ? "cancel requested before search"
                                              : "budget exhausted before search";
      return result;
    }

    LemonGraph lemon(graph);
    const double preprocess_ms = detail::elapsed_ms(t0);

    LDijkstra dijkstra(lemon.graph, lemon.weights);
    dijkstra.init();
    dijkstra.addSource(lemon.nodes[static_cast<std::size_t>(source)]);

    const detail::Clock::time_point t_search = detail::Clock::now();
    long expanded = 0;
    while (!dijkstra.emptyQueue()) {
      if (detail::cancel_requested(control)) {
        result.status = Status::kCancel;
        result.message = "cancelled via per-node budget/cancel check";
        break;
      }
      if (detail::budget_exceeded(control, t0)) {
        result.status = Status::kTimeout;
        result.message = "time budget exceeded via per-node check";
        break;
      }
      dijkstra.processNextNode();
      ++expanded;
    }

    result.stats.expanded_nodes = expanded;
    result.stats.preprocess_ms = preprocess_ms;
    result.stats.search_ms = detail::elapsed_ms(t_search);
    result.stats.wall_ms = detail::elapsed_ms(t0);
    if (result.status == Status::kCancel || result.status == Status::kTimeout) {
      return result;
    }

    result.status = Status::kSuccess;
    const int n = graph.num_vertices;
    result.distances.assign(static_cast<std::size_t>(n),
                            std::numeric_limits<double>::infinity());
    result.predecessors.assign(static_cast<std::size_t>(n), -1);
    for (int v = 0; v < n; ++v) {
      LGraph::Node node = lemon.nodes[static_cast<std::size_t>(v)];
      if (!dijkstra.reached(node)) {
        continue;
      }
      result.distances[static_cast<std::size_t>(v)] = dijkstra.dist(node);
      if (v != source) {
        LGraph::Node pred = dijkstra.predNode(node);
        if (pred != lemon::INVALID) {
          result.predecessors[static_cast<std::size_t>(v)] = lemon.ids[pred];
        }
      }
    }
    return result;
  }

 private:
  EffectiveConfig config_;
};

}  // namespace

std::unique_ptr<GraphSearcher> make_lemon_dijkstra() {
  return std::unique_ptr<GraphSearcher>(new LemonDijkstraSearcher());
}

std::unique_ptr<DistanceFieldSearcher> make_lemon_distance_field() {
  return std::unique_ptr<DistanceFieldSearcher>(
      new LemonDistanceFieldSearcher());
}

}  // namespace graph_search
}  // namespace planner
