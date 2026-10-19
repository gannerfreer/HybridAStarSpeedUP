// planner_graph_search 核心实现：统一契约校验、启发式、代价核对与 JSON 导出。
// 不含任何搜索算法；算法全部由适配器提供。

#include "planner/graph_search/types.hpp"

#include <nlohmann/json.hpp>

#include <utility>
#include <unordered_map>

namespace planner {
namespace graph_search {

const char* status_to_string(Status status) {
  switch (status) {
    case Status::kSuccess:
      return "success";
    case Status::kNoPath:
      return "no_path";
    case Status::kTimeout:
      return "timeout";
    case Status::kCancel:
      return "cancel";
    case Status::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

bool valid_vertex(const Graph& graph, int vertex) {
  return vertex >= 0 && vertex < graph.num_vertices;
}

bool validate_graph(const Graph& graph, std::string* reason) {
  const auto fail = [reason](const std::string& text) {
    if (reason != nullptr) {
      *reason = text;
    }
    return false;
  };
  if (reason != nullptr) {
    reason->clear();
  }
  if (graph.num_vertices < 0) {
    return fail("num_vertices is negative");
  }
  for (std::size_t i = 0; i < graph.edges.size(); ++i) {
    const Edge& e = graph.edges[i];
    if (!valid_vertex(graph, e.from) || !valid_vertex(graph, e.to)) {
      return fail("edge " + std::to_string(i) + " endpoint out of range [" +
                  std::to_string(e.from) + " -> " + std::to_string(e.to) +
                  "]");
    }
    if (!std::isfinite(e.weight)) {
      return fail("edge " + std::to_string(i) + " weight is not finite");
    }
    if (e.weight < 0.0) {
      return fail("edge " + std::to_string(i) + " has negative weight " +
                  std::to_string(e.weight));
    }
  }
  return true;
}

Heuristic zero_heuristic() {
  return [](int) { return 0.0; };
}

EuclideanHeuristic::EuclideanHeuristic(std::vector<double> xs,
                                       std::vector<double> ys, int goal)
    : xs_(std::move(xs)), ys_(std::move(ys)), goal_(goal) {
  valid_ = goal_ >= 0 && static_cast<std::size_t>(goal_) < xs_.size() &&
           xs_.size() == ys_.size();
}

double EuclideanHeuristic::operator()(int vertex) const {
  if (!valid_ || vertex < 0 || static_cast<std::size_t>(vertex) >= xs_.size()) {
    return 0.0;  // 非法顶点由统一校验先行拒绝；此处保守返回 0（可采纳）。
  }
  const double dx = xs_[static_cast<std::size_t>(vertex)] -
                    xs_[static_cast<std::size_t>(goal_)];
  const double dy = ys_[static_cast<std::size_t>(vertex)] -
                    ys_[static_cast<std::size_t>(goal_)];
  return std::hypot(dx, dy);
}

bool euclidean_admissible(const Graph& graph, const std::vector<double>& xs,
                          const std::vector<double>& ys) {
  if (xs.size() != ys.size() ||
      xs.size() < static_cast<std::size_t>(graph.num_vertices)) {
    return false;
  }
  for (const Edge& e : graph.edges) {
    const double dx = xs[static_cast<std::size_t>(e.from)] -
                      xs[static_cast<std::size_t>(e.to)];
    const double dy = ys[static_cast<std::size_t>(e.from)] -
                      ys[static_cast<std::size_t>(e.to)];
    if (e.weight + 1e-12 < std::hypot(dx, dy)) {
      return false;
    }
  }
  return true;
}

double cost_tolerance(double reference_cost) {
  return 1e-8 + 1e-8 * std::fabs(reference_cost);
}

bool cost_close(double measured, double reference_cost) {
  if (!std::isfinite(measured) || !std::isfinite(reference_cost)) {
    return false;
  }
  return std::fabs(measured - reference_cost) <= cost_tolerance(reference_cost);
}

namespace {

// (from, to) -> 最小并行边权。
std::unordered_map<long long, double> edge_index(const Graph& graph) {
  std::unordered_map<long long, double> index;
  index.reserve(graph.edges.size() * 2);
  for (const Edge& e : graph.edges) {
    const long long key = (static_cast<long long>(e.from) << 32) |
                          static_cast<unsigned int>(e.to);
    auto it = index.find(key);
    if (it == index.end() || e.weight < it->second) {
      index[key] = e.weight;
    }
  }
  return index;
}

}  // namespace

double path_cost(const Graph& graph, const std::vector<int>& path) {
  if (path.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto index = edge_index(graph);
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const int from = path[i - 1];
    const int to = path[i];
    if (!valid_vertex(graph, from) || !valid_vertex(graph, to)) {
      return std::numeric_limits<double>::infinity();
    }
    const long long key =
        (static_cast<long long>(from) << 32) | static_cast<unsigned int>(to);
    auto it = index.find(key);
    if (it == index.end()) {
      return std::numeric_limits<double>::infinity();
    }
    total += it->second;
  }
  return total;
}

bool path_is_valid(const Graph& graph, const std::vector<int>& path, int start,
                   int goal) {
  if (path.empty() || path.front() != start || path.back() != goal) {
    return false;
  }
  return std::isfinite(path_cost(graph, path));
}

namespace {

nlohmann::json config_json(const EffectiveConfig& config) {
  nlohmann::json out;
  out["adapter"] = config.adapter;
  out["cost_precision"] = config.cost_precision;
  out["supports_budget"] = config.supports_budget;
  out["supports_cancel"] = config.supports_cancel;
  out["has_early_stop"] = config.has_early_stop;
  out["uses_heuristic"] = config.uses_heuristic;
  out["options"] = config.options;
  return out;
}

nlohmann::json stats_json(const Stats& stats) {
  nlohmann::json out;
  out["expanded_nodes"] =
      stats.expanded_nodes >= 0 ? nlohmann::json(stats.expanded_nodes)
                                : nlohmann::json(nullptr);
  out["relaxed_edges"] =
      stats.relaxed_edges >= 0 ? nlohmann::json(stats.relaxed_edges)
                               : nlohmann::json(nullptr);
  out["preprocess_ms"] =
      stats.preprocess_ms >= 0.0 ? nlohmann::json(stats.preprocess_ms)
                                 : nlohmann::json(nullptr);
  out["search_ms"] = stats.search_ms >= 0.0
                         ? nlohmann::json(stats.search_ms)
                         : nlohmann::json(nullptr);
  out["wall_ms"] = stats.wall_ms >= 0.0 ? nlohmann::json(stats.wall_ms)
                                        : nlohmann::json(nullptr);
  return out;
}

nlohmann::json finite_or_null(double value) {
  if (std::isfinite(value)) {
    return nlohmann::json(value);
  }
  return nlohmann::json(nullptr);
}

}  // namespace

std::string to_json(const Result& result) {
  nlohmann::json out;
  out["status"] = status_to_string(result.status);
  out["path"] = result.path;
  out["cost"] = finite_or_null(result.cost);
  out["config"] = config_json(result.config);
  out["stats"] = stats_json(result.stats);
  out["message"] = result.message;
  return out.dump(2);
}

std::string to_json(const DistanceFieldResult& result) {
  nlohmann::json out;
  out["status"] = status_to_string(result.status);
  nlohmann::json distances = nlohmann::json::array();
  for (double d : result.distances) {
    distances.push_back(finite_or_null(d));
  }
  out["distances"] = std::move(distances);
  out["predecessors"] = result.predecessors;
  out["config"] = config_json(result.config);
  out["stats"] = stats_json(result.stats);
  out["message"] = result.message;
  return out.dump(2);
}

}  // namespace graph_search
}  // namespace planner
