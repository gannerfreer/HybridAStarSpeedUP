// S-02 libMultiRobotPlanning 适配器：图 A*（单机器人模板）。
//
// 上游：vendored 三头文件，commit 4c75fa20（MIT；见 src/upstream/lmrp/PROVENANCE.md）。
// 适配要点（family_a 探针 §3）：
//  * Environment 提供邻居/代价/启发式/解判定；无内建取消钩子，适配层在
//    onExpandNode 中检查取消标志与预算并抛出异常中止搜索（登记补丁）；
//  * closed set 不重开节点（上游语义），正确性绑定一致启发式前提；
//  * PlanResult.states/cost 转统一路径与代价。

#include <algorithm>
#include <exception>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "detail/control.hpp"
#include "libMultiRobotPlanning/a_star.hpp"
#include "planner/graph_search/adapters.hpp"

namespace planner {
namespace graph_search {
namespace {

using libMultiRobotPlanning::AStar;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

struct State {
  int id = 0;
};

inline bool operator==(const State& a, const State& b) { return a.id == b.id; }

struct StateHasher {
  std::size_t operator()(const State& s) const {
    return std::hash<int>()(s.id);
  }
};

using Adjacency = std::vector<std::vector<Neighbor<State, int, double>>>;

class GraphEnvironment {
 public:
  const Adjacency* adjacency = nullptr;
  const Heuristic* heuristic = nullptr;
  int goal = 0;
  const Control* control = nullptr;
  detail::Clock::time_point start{};
  long expansions = 0;

  struct Aborted {
    Status status;
  };

  double admissibleHeuristic(const State& s) {
    return (*heuristic)(s.id);
  }

  bool isSolution(const State& s) { return s.id == goal; }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, int, double>>& out) {
    for (const auto& neighbor : (*adjacency)[static_cast<std::size_t>(s.id)]) {
      out.push_back(neighbor);
    }
  }

  // 适配补丁：上游无取消/超时钩子；此处抛出以中止搜索（异常安全已验证）。
  void onExpandNode(const State&, double, double) {
    ++expansions;
    if (detail::cancel_requested(*control)) {
      throw Aborted{Status::kCancel};
    }
    if (detail::budget_exceeded(*control, start)) {
      throw Aborted{Status::kTimeout};
    }
  }

  void onDiscover(const State&, double, double) {}
};

class LmrpAStarSearcher : public GraphSearcher {
 public:
  LmrpAStarSearcher() {
    config_.adapter = "lmrp_astar";
    config_.cost_precision = "float64";
    config_.supports_budget = true;
    config_.supports_cancel = true;
    config_.has_early_stop = true;
    config_.uses_heuristic = true;
    config_.options["upstream_commit"] = "4c75fa20c435c440d8b6bd6dc81668ddc7296ba0";
    config_.options["cancel"] = "exception-adapter-patch";
    config_.options["dedup"] = "closed-set-no-reopen";
    config_.options["heap"] = "boost d_ary_heap";
  }

  const EffectiveConfig& config() const override { return config_; }

  Result search(const Graph& graph, int start, int goal,
                const Heuristic& heuristic,
                const Control& control) const override {
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

    // 邻接索引（转换成本单列）。
    Adjacency adjacency(static_cast<std::size_t>(graph.num_vertices));
    for (const Edge& e : graph.edges) {
      adjacency[static_cast<std::size_t>(e.from)].emplace_back(
          State{e.to}, e.to, e.weight);
    }
    const double preprocess_ms = detail::elapsed_ms(t0);

    GraphEnvironment environment;
    environment.adjacency = &adjacency;
    environment.heuristic = &heuristic;
    environment.goal = goal;
    environment.control = &control;
    environment.start = detail::Clock::now();

    AStar<State, int, double, GraphEnvironment, StateHasher> astar(environment);
    PlanResult<State, int, double> solution;

    const detail::Clock::time_point t_search = detail::Clock::now();
    bool ok = false;
    try {
      ok = astar.search(State{start}, solution);
    } catch (const GraphEnvironment::Aborted& aborted) {
      result.status = aborted.status;
      result.message = aborted.status == Status::kCancel
                           ? "cancelled via onExpandNode adapter patch"
                           : "time budget exceeded via onExpandNode adapter patch";
      result.stats.expanded_nodes = environment.expansions;
      result.stats.preprocess_ms = preprocess_ms;
      result.stats.search_ms = detail::elapsed_ms(t_search);
      result.stats.wall_ms = detail::elapsed_ms(t0);
      return result;
    } catch (const std::exception& e) {
      result.status = Status::kInvalidInput;
      result.message = std::string("libMultiRobotPlanning exception: ") +
                       e.what();
      result.stats.preprocess_ms = preprocess_ms;
      result.stats.search_ms = detail::elapsed_ms(t_search);
      result.stats.wall_ms = detail::elapsed_ms(t0);
      return result;
    }

    result.stats.expanded_nodes = environment.expansions;
    result.stats.preprocess_ms = preprocess_ms;
    result.stats.search_ms = detail::elapsed_ms(t_search);
    result.stats.wall_ms = detail::elapsed_ms(t0);

    if (!ok) {
      result.status = Status::kNoPath;
      return result;
    }

    result.status = Status::kSuccess;
    result.path.reserve(solution.states.size());
    for (const auto& state : solution.states) {
      result.path.push_back(state.first.id);
    }
    result.cost = solution.cost;
    return result;
  }

 private:
  EffectiveConfig config_;
};

}  // namespace

std::unique_ptr<GraphSearcher> make_lmrp_astar() {
  return std::unique_ptr<GraphSearcher>(new LmrpAStarSearcher());
}

}  // namespace graph_search
}  // namespace planner
