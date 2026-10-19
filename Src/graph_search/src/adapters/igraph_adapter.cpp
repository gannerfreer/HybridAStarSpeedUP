// S-05 igraph 适配器：Dijkstra 点到点 + 完整距离场（C 包装）。
//
// GPL-2.0-or-later 隔离组：本文件只在独立目标 planner_graph_search_igraph 中
// 编译（链接 igraph 静态库），绝不链入 planner_graph_search / 默认交付库。
// 来源与获取方式见 provenance.md（官方 1.0.1 发布 tar 包，预生成解析器）。
//
// 适配要点（family_b 探针 §3.3）：
//  * C API 包装：igraph_t / igraph_vector_int_t / igraph_matrix_t /
//    igraph_vector_int_list_t 的 RAII 转换；
//  * 不可达 = IGRAPH_INFINITY，统一归一化为 +inf；
//  * 取消/预算：通过公开的 igraph_set_interruption_handler 安装线程局部
//    中断处理器（igraph 在 Dijkstra 主循环调用 IGRAPH_ALLOW_INTERRUPTION），
//    中断时返回 IGRAPH_INTERRUPTED，适配层据标志区分 cancel/timeout；
//  * 点到点代价：igraph 不直接返回距离，按返回路径求和（double）；
//  * 距离场不返回前驱（igraph_distances_dijkstra 无 parents 输出），
//    predecessors 留空并在 EffectiveConfig 记录。

extern "C" {
#include <igraph.h>
}

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "detail/control.hpp"
#include "planner/graph_search/adapters.hpp"

namespace planner {
namespace graph_search {
namespace {

// ---------------------------------------------------------------------------
// igraph 对象 RAII 包装
// ---------------------------------------------------------------------------

struct IgraphVectorInt {
  IgraphVectorInt() { igraph_vector_int_init(&value, 0); }
  ~IgraphVectorInt() { igraph_vector_int_destroy(&value); }
  igraph_vector_int_t value;
};

struct IgraphVector {
  IgraphVector() { igraph_vector_init(&value, 0); }
  ~IgraphVector() { igraph_vector_destroy(&value); }
  igraph_vector_t value;
};

struct IgraphMatrix {
  IgraphMatrix() { igraph_matrix_init(&value, 0, 0); }
  ~IgraphMatrix() { igraph_matrix_destroy(&value); }
  igraph_matrix_t value;
};

struct IgraphVectorIntList {
  IgraphVectorIntList() { igraph_vector_int_list_init(&value, 0); }
  ~IgraphVectorIntList() { igraph_vector_int_list_destroy(&value); }
  igraph_vector_int_list_t value;
};

struct IgraphGraph {
  IgraphGraph() = default;
  ~IgraphGraph() {
    if (initialized) {
      igraph_destroy(&value);
    }
  }
  igraph_t value{};
  bool initialized = false;
};

// ---------------------------------------------------------------------------
// 中断处理器（线程局部状态；igraph 的中断处理器变量本身为 TLS）
// ---------------------------------------------------------------------------

struct InterruptState {
  const Control* control = nullptr;
  detail::Clock::time_point start{};
};

thread_local InterruptState* g_interrupt_state = nullptr;

igraph_bool_t interrupt_check() {
  const InterruptState* state = g_interrupt_state;
  if (state == nullptr) {
    return false;
  }
  if (detail::cancel_requested(*state->control)) {
    return true;
  }
  if (detail::budget_exceeded(*state->control, state->start)) {
    return true;
  }
  return false;
}

class ScopedInterruptHandler {
 public:
  explicit ScopedInterruptHandler(InterruptState* state) {
    g_interrupt_state = state;
    previous_ = igraph_set_interruption_handler(&interrupt_check);
  }
  ~ScopedInterruptHandler() {
    igraph_set_interruption_handler(previous_);
    g_interrupt_state = nullptr;
  }
  ScopedInterruptHandler(const ScopedInterruptHandler&) = delete;
  ScopedInterruptHandler& operator=(const ScopedInterruptHandler&) = delete;

 private:
  igraph_interruption_handler_t* previous_ = nullptr;
};

// 不可达目标会触发 igraph 原生警告（"Couldn't reach some vertices."），
// 而不可达在统一契约中是正常结果（kNoPath）；适配层在调用期间抑制警告并
// 记录该偏差。警告处理器为线程局部，与中断处理器一致。
class ScopedWarningSuppressor {
 public:
  ScopedWarningSuppressor() {
    previous_ = igraph_set_warning_handler(&igraph_warning_handler_ignore);
  }
  ~ScopedWarningSuppressor() { igraph_set_warning_handler(previous_); }
  ScopedWarningSuppressor(const ScopedWarningSuppressor&) = delete;
  ScopedWarningSuppressor& operator=(const ScopedWarningSuppressor&) = delete;

 private:
  igraph_warning_handler_t* previous_ = nullptr;
};

// ---------------------------------------------------------------------------
// 图构建与错误映射
// ---------------------------------------------------------------------------

igraph_error_t build_igraph(const Graph& input, IgraphGraph* out) {
  IgraphVectorInt edges;
  IgraphVector weights;
  igraph_error_t rc = igraph_vector_int_resize(
      &edges.value, static_cast<igraph_int_t>(input.edges.size() * 2));
  if (rc != IGRAPH_SUCCESS) {
    return rc;
  }
  rc = igraph_vector_resize(&weights.value,
                            static_cast<igraph_int_t>(input.edges.size()));
  if (rc != IGRAPH_SUCCESS) {
    return rc;
  }
  for (std::size_t i = 0; i < input.edges.size(); ++i) {
    VECTOR(edges.value)[2 * i] = input.edges[i].from;
    VECTOR(edges.value)[2 * i + 1] = input.edges[i].to;
    VECTOR(weights.value)[i] = input.edges[i].weight;
  }
  rc = igraph_create(&out->value, &edges.value, input.num_vertices,
                     IGRAPH_DIRECTED);
  out->initialized = (rc == IGRAPH_SUCCESS);
  return rc;
}

Status map_interrupt(const Control& control, detail::Clock::time_point start) {
  if (detail::cancel_requested(control)) {
    return Status::kCancel;
  }
  if (detail::budget_exceeded(control, start)) {
    return Status::kTimeout;
  }
  return Status::kTimeout;  // 中断但标志不可见：按超时处理（防御）
}

EffectiveConfig igraph_config(const char* adapter, bool distance_field) {
  EffectiveConfig config;
  config.adapter = adapter;
  config.cost_precision = "float64";
  config.supports_budget = true;
  config.supports_cancel = true;
  config.has_early_stop = !distance_field;
  config.uses_heuristic = false;
  config.options["upstream_version"] = "1.0.1";
  config.options["upstream_commit"] = "7b4ae766cbdee6b2017aa5b76752457db2a2972f";
  config.options["license"] = "GPL-2.0-or-later (isolation group)";
  config.options["acquisition"] =
      "release tarball igraph-1.0.1.tar.gz (pre-generated parsers)";
  config.options["isolation"] =
      "planner_graph_search_igraph only; never linked into the default library";
  config.options["cancel"] = "igraph_set_interruption_handler (TLS)";
  config.options["unreachable"] = "IGRAPH_INFINITY normalized to +inf";
  config.options["warnings"] =
      "native warnings suppressed around the call (unreachable is normal)";
  config.options["mode"] =
      distance_field ? "full-distance-field" : "point-to-point";
  if (distance_field) {
    config.options["predecessors"] = "not provided by igraph_distances_dijkstra";
  } else {
    config.options["cost_source"] = "sum along returned igraph path (double)";
  }
  return config;
}

// ---------------------------------------------------------------------------
// 适配器
// ---------------------------------------------------------------------------

class IgraphDijkstraSearcher : public GraphSearcher {
 public:
  IgraphDijkstraSearcher() : config_(igraph_config("igraph_dijkstra", false)) {}

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

    IgraphGraph igraph;
    const igraph_error_t build_rc = build_igraph(graph, &igraph);
    const double preprocess_ms = detail::elapsed_ms(t0);
    if (build_rc != IGRAPH_SUCCESS) {
      result.status = Status::kInvalidInput;
      result.message =
          std::string("igraph_create failed: ") + igraph_strerror(build_rc);
      result.stats.preprocess_ms = preprocess_ms;
      result.stats.wall_ms = detail::elapsed_ms(t0);
      return result;
    }

    IgraphVector weights;
    igraph_vector_resize(&weights.value,
                         static_cast<igraph_int_t>(graph.edges.size()));
    for (std::size_t i = 0; i < graph.edges.size(); ++i) {
      VECTOR(weights.value)[i] = graph.edges[i].weight;
    }

    IgraphVectorIntList paths;
    InterruptState interrupt{&control, t0};
    ScopedInterruptHandler handler(&interrupt);
    ScopedWarningSuppressor warning_suppressor;

    const detail::Clock::time_point t_search = detail::Clock::now();
    const igraph_error_t rc = igraph_get_shortest_paths_dijkstra(
        &igraph.value, &paths.value, nullptr, start, igraph_vss_1(goal),
        &weights.value, IGRAPH_OUT, nullptr, nullptr);
    result.stats.preprocess_ms = preprocess_ms;
    result.stats.search_ms = detail::elapsed_ms(t_search);
    result.stats.wall_ms = detail::elapsed_ms(t0);

    if (rc == IGRAPH_INTERRUPTED) {
      result.status = map_interrupt(control, t0);
      result.message = "interrupted via igraph interruption handler";
      return result;
    }
    if (rc != IGRAPH_SUCCESS) {
      result.status = Status::kInvalidInput;
      result.message =
          std::string("igraph_get_shortest_paths_dijkstra failed: ") +
          igraph_strerror(rc);
      return result;
    }

    const igraph_vector_int_t* path =
        igraph_vector_int_list_get_ptr(&paths.value, 0);
    const igraph_int_t path_size = igraph_vector_int_size(path);
    if (path_size == 0) {
      result.status = Status::kNoPath;
      return result;
    }

    result.status = Status::kSuccess;
    result.path.reserve(static_cast<std::size_t>(path_size));
    for (igraph_int_t i = 0; i < path_size; ++i) {
      result.path.push_back(static_cast<int>(VECTOR(*path)[i]));
    }
    result.cost = path_cost(graph, result.path);
    return result;
  }

 private:
  EffectiveConfig config_;
};

class IgraphDistanceFieldSearcher : public DistanceFieldSearcher {
 public:
  IgraphDistanceFieldSearcher()
      : config_(igraph_config("igraph_distance_field", true)) {}

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

    IgraphGraph igraph;
    const igraph_error_t build_rc = build_igraph(graph, &igraph);
    const double preprocess_ms = detail::elapsed_ms(t0);
    if (build_rc != IGRAPH_SUCCESS) {
      result.status = Status::kInvalidInput;
      result.message =
          std::string("igraph_create failed: ") + igraph_strerror(build_rc);
      result.stats.preprocess_ms = preprocess_ms;
      result.stats.wall_ms = detail::elapsed_ms(t0);
      return result;
    }

    IgraphVector weights;
    igraph_vector_resize(&weights.value,
                         static_cast<igraph_int_t>(graph.edges.size()));
    for (std::size_t i = 0; i < graph.edges.size(); ++i) {
      VECTOR(weights.value)[i] = graph.edges[i].weight;
    }

    IgraphMatrix distances;
    InterruptState interrupt{&control, t0};
    ScopedInterruptHandler handler(&interrupt);
    ScopedWarningSuppressor warning_suppressor;

    const detail::Clock::time_point t_search = detail::Clock::now();
    const igraph_error_t rc = igraph_distances_dijkstra(
        &igraph.value, &distances.value, igraph_vss_1(source), igraph_vss_all(),
        &weights.value, IGRAPH_OUT);
    result.stats.preprocess_ms = preprocess_ms;
    result.stats.search_ms = detail::elapsed_ms(t_search);
    result.stats.wall_ms = detail::elapsed_ms(t0);

    if (rc == IGRAPH_INTERRUPTED) {
      result.status = map_interrupt(control, t0);
      result.message = "interrupted via igraph interruption handler";
      return result;
    }
    if (rc != IGRAPH_SUCCESS) {
      result.status = Status::kInvalidInput;
      result.message = std::string("igraph_distances_dijkstra failed: ") +
                       igraph_strerror(rc);
      return result;
    }

    result.status = Status::kSuccess;
    const int n = graph.num_vertices;
    result.distances.resize(static_cast<std::size_t>(n));
    for (int v = 0; v < n; ++v) {
      const double d = MATRIX(distances.value, 0, v);
      result.distances[static_cast<std::size_t>(v)] =
          (d == IGRAPH_INFINITY) ? std::numeric_limits<double>::infinity() : d;
    }
    // igraph_distances_dijkstra 不输出前驱；留空（EffectiveConfig 已记录）。
    return result;
  }

 private:
  EffectiveConfig config_;
};

}  // namespace

std::unique_ptr<GraphSearcher> make_igraph_dijkstra() {
  return std::unique_ptr<GraphSearcher>(new IgraphDijkstraSearcher());
}

std::unique_ptr<DistanceFieldSearcher> make_igraph_distance_field() {
  return std::unique_ptr<DistanceFieldSearcher>(
      new IgraphDistanceFieldSearcher());
}

}  // namespace graph_search
}  // namespace planner
