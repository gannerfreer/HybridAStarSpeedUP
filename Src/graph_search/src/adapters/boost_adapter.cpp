// S-01 Boost Graph 适配器：图 A*、图 Dijkstra（点到点）与完整距离场。
//
// 上游：系统 Boost 1.71.0（header-only；来源与版本记录见 provenance.md）。
// 适配要点（family_a 探针 §2）：
//  * 不可达标记为 numeric_limits<double>::max()（非 inf），统一归一化为 +inf；
//  * Boost 1.71 的 A* f 值命名参数为 rank_map（无 cost_map）；
//  * 早停/取消/超时通过 visitor 抛异常实现（无内建中断），异常安全已验证；
//  * 负权由统一校验先行拒绝；boost::negative_edge 作为二次防线映射为
//    invalid_input。

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/exception.hpp>
#include <boost/version.hpp>

#include <algorithm>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#include "detail/control.hpp"
#include "planner/graph_search/adapters.hpp"

namespace planner {
namespace graph_search {
namespace {

using BoostGraph =
    boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                          boost::no_property,
                          boost::property<boost::edge_weight_t, double>>;
using Vertex = boost::graph_traits<BoostGraph>::vertex_descriptor;

enum class Mode { kAStar, kDijkstra, kDistanceField };

struct FoundGoal {};
struct Cancelled {};
struct TimedOut {};

struct BoostHeuristic : boost::astar_heuristic<BoostGraph, double> {
  const Heuristic* fn = nullptr;
  double operator()(Vertex v) const {
    return (*fn)(static_cast<int>(v));
  }
};

// examine_vertex 中检查目标/取消/预算；抛异常穿过 BGL 模板（栈上容器安全）。
// 计数器用指针：BGL 按值拷贝 visitor，成员变量不会回写调用方。
struct ControlVisitor : boost::default_astar_visitor {
  Vertex goal = 0;
  bool stop_at_goal = true;
  const Control* control = nullptr;
  detail::Clock::time_point start{};
  long* expanded = nullptr;
  long* relaxed = nullptr;

  void examine_vertex(Vertex u, const BoostGraph&) {
    ++*expanded;
    if (stop_at_goal && u == goal) {
      throw FoundGoal{};
    }
    if (detail::cancel_requested(*control)) {
      throw Cancelled{};
    }
    if (detail::budget_exceeded(*control, start)) {
      throw TimedOut{};
    }
  }

  template <class Edge, class Graph>
  void edge_relaxed(Edge, const Graph&) {
    ++*relaxed;
  }
};

std::string boost_version_string() {
  return std::to_string(BOOST_VERSION / 100000) + "." +
         std::to_string((BOOST_VERSION / 100) % 1000) + "." +
         std::to_string(BOOST_VERSION % 100);
}

struct RunOutcome {
  Status status = Status::kInvalidInput;
  std::vector<int> path;
  double cost = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> distances;
  std::vector<int> predecessors;
  long expanded = -1;
  long relaxed = -1;
  std::string message;
};

RunOutcome run_boost(const Graph& input, int start, int goal,
                     const Heuristic& heuristic, const Control& control,
                     Mode mode, double* preprocess_ms, double* search_ms) {
  RunOutcome outcome;
  const int n = input.num_vertices;

  const detail::Clock::time_point t_build = detail::Clock::now();
  BoostGraph g(n);
  for (const Edge& e : input.edges) {
    boost::add_edge(static_cast<Vertex>(e.from), static_cast<Vertex>(e.to),
                    e.weight, g);
  }
  *preprocess_ms = detail::elapsed_ms(t_build);

  std::vector<Vertex> pred(n);
  std::vector<double> dist(n);
  std::vector<double> rank(n);
  std::vector<boost::default_color_type> color(n);

  ControlVisitor visitor;
  visitor.goal = static_cast<Vertex>(goal);
  visitor.stop_at_goal = mode != Mode::kDistanceField;
  visitor.control = &control;
  visitor.start = detail::Clock::now();
  long expanded_count = 0;
  long relaxed_count = 0;
  visitor.expanded = &expanded_count;
  visitor.relaxed = &relaxed_count;

  bool found_goal = false;
  const detail::Clock::time_point t_search = detail::Clock::now();
  try {
    if (mode == Mode::kAStar) {
      BoostHeuristic h;
      h.fn = &heuristic;
      boost::astar_search(
          g, static_cast<Vertex>(start), h,
          boost::visitor(visitor)
              .predecessor_map(&pred[0])
              .distance_map(&dist[0])
              .rank_map(&rank[0])
              .color_map(&color[0]));
    } else {
      boost::dijkstra_shortest_paths(
          g, static_cast<Vertex>(start),
          boost::visitor(visitor)
              .predecessor_map(&pred[0])
              .distance_map(&dist[0])
              .color_map(&color[0]));
    }
  } catch (const FoundGoal&) {
    found_goal = true;
  } catch (const Cancelled&) {
    outcome.status = Status::kCancel;
    outcome.message = "cancelled via visitor exception";
    *search_ms = detail::elapsed_ms(t_search);
    return outcome;
  } catch (const TimedOut&) {
    outcome.status = Status::kTimeout;
    outcome.message = "time budget exceeded via visitor exception";
    *search_ms = detail::elapsed_ms(t_search);
    return outcome;
  } catch (const boost::negative_edge& e) {
    outcome.status = Status::kInvalidInput;
    outcome.message = std::string("boost::negative_edge: ") + e.what();
    *search_ms = detail::elapsed_ms(t_search);
    return outcome;
  } catch (const std::exception& e) {
    outcome.status = Status::kInvalidInput;
    outcome.message = std::string("boost exception: ") + e.what();
    *search_ms = detail::elapsed_ms(t_search);
    return outcome;
  }
  *search_ms = detail::elapsed_ms(t_search);
  outcome.expanded = expanded_count;
  outcome.relaxed = relaxed_count;

  const double big = (std::numeric_limits<double>::max)();

  if (mode == Mode::kDistanceField) {
    outcome.status = Status::kSuccess;
    outcome.distances.assign(n, std::numeric_limits<double>::infinity());
    outcome.predecessors.assign(n, -1);
    for (int v = 0; v < n; ++v) {
      if (dist[static_cast<std::size_t>(v)] >= big * 0.5) {
        continue;  // 不可达（BGL 哨兵 max()）归一化为 +inf
      }
      outcome.distances[static_cast<std::size_t>(v)] =
          dist[static_cast<std::size_t>(v)];
      if (v != start) {
        outcome.predecessors[static_cast<std::size_t>(v)] =
            static_cast<int>(pred[static_cast<std::size_t>(v)]);
      }
    }
    return outcome;
  }

  if (!found_goal) {
    outcome.status = Status::kNoPath;
    return outcome;
  }

  outcome.cost = dist[static_cast<std::size_t>(goal)];
  // 前驱链回溯（BGL 将源点前驱设为自身；防御性限制步数）。
  std::vector<int> reversed;
  Vertex v = static_cast<Vertex>(goal);
  for (int steps = 0; steps <= n; ++steps) {
    reversed.push_back(static_cast<int>(v));
    if (static_cast<int>(v) == start) {
      break;
    }
    const Vertex next = pred[static_cast<std::size_t>(v)];
    if (static_cast<int>(next) == static_cast<int>(v) ||
        steps == n) {
      outcome.status = Status::kNoPath;
      outcome.message = "predecessor chain did not reach the start vertex";
      return outcome;
    }
    v = next;
  }
  std::reverse(reversed.begin(), reversed.end());
  outcome.status = Status::kSuccess;
  outcome.path = std::move(reversed);
  return outcome;
}

class BoostSearcher : public GraphSearcher {
 public:
  BoostSearcher(std::string name, Mode mode) : mode_(mode) {
    config_.adapter = std::move(name);
    config_.cost_precision = "float64";
    config_.supports_budget = true;
    config_.supports_cancel = true;
    config_.has_early_stop = true;
    config_.uses_heuristic = (mode == Mode::kAStar);
    config_.options["boost_version"] = boost_version_string();
    config_.options["f_value_map"] = "rank_map";
    config_.options["early_stop"] = "goal-visitor-exception";
    config_.options["cancel"] = "visitor-exception";
    config_.options["unreachable_marker"] = "max() normalized to inf";
    config_.options["algorithm"] =
        mode == Mode::kAStar ? "astar_search" : "dijkstra_shortest_paths";
  }

  const EffectiveConfig& config() const override { return config_; }

  Result search(const Graph& graph, int start, int goal,
                const Heuristic& heuristic, const Control& control) const override {
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

    double preprocess_ms = 0.0;
    double search_ms = 0.0;
    RunOutcome outcome = run_boost(graph, start, goal, heuristic, control, mode_,
                                   &preprocess_ms, &search_ms);
    result.status = outcome.status;
    result.path = std::move(outcome.path);
    result.cost = outcome.cost;
    result.message = std::move(outcome.message);
    result.stats.expanded_nodes = outcome.expanded;
    result.stats.relaxed_edges = outcome.relaxed;
    result.stats.preprocess_ms = preprocess_ms;
    result.stats.search_ms = search_ms;
    result.stats.wall_ms = detail::elapsed_ms(t0);
    return result;
  }

 private:
  Mode mode_;
  EffectiveConfig config_;
};

class BoostDistanceFieldSearcher : public DistanceFieldSearcher {
 public:
  BoostDistanceFieldSearcher() {
    config_.adapter = "boost_distance_field";
    config_.cost_precision = "float64";
    config_.supports_budget = true;
    config_.supports_cancel = true;
    config_.has_early_stop = false;
    config_.uses_heuristic = false;
    config_.options["boost_version"] = boost_version_string();
    config_.options["algorithm"] = "dijkstra_shortest_paths";
    config_.options["mode"] = "full-distance-field";
    config_.options["cancel"] = "visitor-exception";
    config_.options["unreachable_marker"] = "max() normalized to inf";
  }

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

    double preprocess_ms = 0.0;
    double search_ms = 0.0;
    RunOutcome outcome = run_boost(graph, source, 0, zero_heuristic(), control,
                                   Mode::kDistanceField, &preprocess_ms,
                                   &search_ms);
    result.status = outcome.status;
    result.distances = std::move(outcome.distances);
    result.predecessors = std::move(outcome.predecessors);
    result.message = std::move(outcome.message);
    result.stats.expanded_nodes = outcome.expanded;
    result.stats.relaxed_edges = outcome.relaxed;
    result.stats.preprocess_ms = preprocess_ms;
    result.stats.search_ms = search_ms;
    result.stats.wall_ms = detail::elapsed_ms(t0);
    return result;
  }

 private:
  EffectiveConfig config_;
};

}  // namespace

std::unique_ptr<GraphSearcher> make_boost_astar() {
  return std::unique_ptr<GraphSearcher>(
      new BoostSearcher("boost_astar", Mode::kAStar));
}

std::unique_ptr<GraphSearcher> make_boost_dijkstra() {
  return std::unique_ptr<GraphSearcher>(
      new BoostSearcher("boost_dijkstra", Mode::kDijkstra));
}

std::unique_ptr<DistanceFieldSearcher> make_boost_distance_field() {
  return std::unique_ptr<DistanceFieldSearcher>(
      new BoostDistanceFieldSearcher());
}

}  // namespace graph_search
}  // namespace planner
