// S-03 justinhj A* 适配器实现（模板文件）。
//
// 由 justinhj_adapter_pool.cpp / justinhj_adapter_nopool.cpp 分别包含，
// 两个 TU 位于不同命名空间以避免 AStarSearch<GraphNode> 实例化的 ODR 冲突：
//   * pool    : USE_FSA_MEMORY=1（上游默认；内存池）
//   * nopool  : USE_FSA_MEMORY=0（1 行守卫补丁关闭内存池，见 PROVENANCE）
//
// 调用前必须已定义：USE_FSA_MEMORY、PLANNER_JUSTINHJ_NAMESPACE、
// PLANNER_JUSTINHJ_FACTORY。

#include "stlastar.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "detail/control.hpp"
#include "planner/graph_search/adapters.hpp"

namespace planner {
namespace graph_search {
namespace PLANNER_JUSTINHJ_NAMESPACE {

#if USE_FSA_MEMORY
constexpr const char* kAdapterName = "justinhj_astar_pool";
constexpr const char* kPoolState = "on";
#else
constexpr const char* kAdapterName = "justinhj_astar_nopool";
constexpr const char* kPoolState = "off";
#endif

struct SearchContext {
  const std::vector<std::vector<std::pair<int, double>>>* adjacency = nullptr;
  const Heuristic* heuristic = nullptr;
};

// 上游 UserState 接口：GoalDistanceEstimate（float）、IsGoal、GetSuccessors
// （AddSuccessor 形参为非 const 左值引用）、GetCost、IsSameState、Hash。
class GraphNode {
 public:
  int id = 0;
  const SearchContext* ctx = nullptr;

  GraphNode() = default;
  GraphNode(int id_in, const SearchContext* ctx_in) : id(id_in), ctx(ctx_in) {}

  float GoalDistanceEstimate(GraphNode& goal) {
    (void)goal;  // 统一启发式已绑定终点；调用方保证可采纳
    return static_cast<float>((*ctx->heuristic)(id));
  }

  bool IsGoal(GraphNode& goal) { return id == goal.id; }

  bool GetSuccessors(AStarSearch<GraphNode>* astar, GraphNode* parent) {
    (void)parent;
    bool ok = true;
    for (const auto& edge : (*ctx->adjacency)[static_cast<std::size_t>(id)]) {
      GraphNode successor(edge.first, ctx);
      ok = astar->AddSuccessor(successor) && ok;
    }
    return ok;
  }

  float GetCost(GraphNode& successor) {
    for (const auto& edge : (*ctx->adjacency)[static_cast<std::size_t>(id)]) {
      if (edge.first == successor.id) {
        return static_cast<float>(edge.second);
      }
    }
    return 0.0f;  // 上游接口要求：非邻居返回 0（不会发生）
  }

  bool IsSameState(GraphNode& rhs) { return id == rhs.id; }

  std::size_t Hash() { return std::hash<int>()(id); }
};

using GraphSearch = AStarSearch<GraphNode>;

class JustinhjSearcher : public GraphSearcher {
 public:
  JustinhjSearcher() {
    config_.adapter = kAdapterName;
    config_.cost_precision = "float32";
    config_.supports_budget = true;
    config_.supports_cancel = true;
    config_.has_early_stop = true;
    config_.uses_heuristic = true;
    config_.options["upstream_commit"] =
        "875bbd8ba4cdf75463cb30424d0ab4b2a8db9653";
    config_.options["upstream_tag"] = "v1.4.2";
    config_.options["memory_pool"] = kPoolState;
    config_.options["cancel"] = "adapter-flags-stepwise";
    config_.options["cancel_thread_safety"] =
        "native flag is non-atomic; adapter observes between steps";
    config_.options["dedup"] = "hash-no-reopen";
    config_.options["cost_note"] =
        "native cost is float32; verify optimality with double path recompute";
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

    std::vector<std::vector<std::pair<int, double>>> adjacency(
        static_cast<std::size_t>(graph.num_vertices));
    for (const Edge& e : graph.edges) {
      adjacency[static_cast<std::size_t>(e.from)].emplace_back(e.to, e.weight);
    }
    const double preprocess_ms = detail::elapsed_ms(t0);

    SearchContext context;
    context.adjacency = &adjacency;
    context.heuristic = &heuristic;

    // 池容量按规模设置：唯一状态 <= num_vertices（+ 起终点与余量）。
    const int capacity = std::max(1000, graph.num_vertices + 16);
    result.config.options["memory_pool_capacity"] = std::to_string(capacity);

    GraphSearch astar(capacity);
    GraphNode start_node(start, &context);
    GraphNode goal_node(goal, &context);
    astar.SetStartAndGoalStates(start_node, goal_node);

    const detail::Clock::time_point t_search = detail::Clock::now();
    bool cancel_observed = false;
    bool timeout_observed = false;
    unsigned int state = GraphSearch::SEARCH_STATE_SEARCHING;
    while (true) {
      state = astar.SearchStep();
      if (state != GraphSearch::SEARCH_STATE_SEARCHING) {
        break;
      }
      if (detail::cancel_requested(control)) {
        astar.CancelSearch();
        cancel_observed = true;
        state = astar.SearchStep();  // 协作标志：下一次步进落到 FAILED
        break;
      }
      if (detail::budget_exceeded(control, t0)) {
        astar.CancelSearch();
        timeout_observed = true;
        state = astar.SearchStep();
        break;
      }
    }
    const double search_ms = detail::elapsed_ms(t_search);

    result.stats.expanded_nodes = astar.GetStepCount();
    result.stats.preprocess_ms = preprocess_ms;
    result.stats.search_ms = search_ms;
    result.stats.wall_ms = detail::elapsed_ms(t0);

    if (cancel_observed) {
      result.status = Status::kCancel;
      result.message =
          "cancelled via adapter flags (native state conflates FAILED)";
      return result;
    }
    if (timeout_observed) {
      result.status = Status::kTimeout;
      result.message =
          "time budget exceeded via adapter flags (native state conflates FAILED)";
      return result;
    }

    if (state == GraphSearch::SEARCH_STATE_SUCCEEDED) {
      result.status = Status::kSuccess;
      for (GraphNode* node = astar.GetSolutionStart(); node != nullptr;
           node = astar.GetSolutionNext()) {
        result.path.push_back(node->id);
      }
      result.cost = static_cast<double>(astar.GetSolutionCost());
      return result;
    }

    if (state == GraphSearch::SEARCH_STATE_OUT_OF_MEMORY) {
      result.status = Status::kInvalidInput;
      result.message = "justinhj memory pool exhausted (MaxNodes=" +
                       std::to_string(capacity) + ")";
      return result;
    }

    result.status = Status::kNoPath;
    return result;
  }

 private:
  EffectiveConfig config_;
};

}  // namespace PLANNER_JUSTINHJ_NAMESPACE

std::unique_ptr<GraphSearcher> PLANNER_JUSTINHJ_FACTORY() {
  return std::unique_ptr<GraphSearcher>(
      new PLANNER_JUSTINHJ_NAMESPACE::JustinhjSearcher());
}

}  // namespace graph_search
}  // namespace planner
