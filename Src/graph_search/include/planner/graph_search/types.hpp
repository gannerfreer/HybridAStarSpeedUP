// planner_graph_search 统一公共接口（X-S1 图 A* / X-S2 图 Dijkstra）。
//
// 契约（组件扩展计划 §4.2）：
//  * 输入：顶点数 + 有向边（from/to/非负边权）、起点、终点与启发式
//    （无可靠下界时使用零启发式；可采纳性是调用方责任）；
//  * 输出：状态、路径/距离、代价、有效配置与统计；
//  * 状态区分 success / no_path / timeout / cancel / invalid_input；
//  * 精确最短路比较口径为代价，容差 `1e-8 + 1e-8 * abs(reference_cost)`。
//
// 本头文件不暴露任何上游库类型；适配器通过 adapters.hpp 的工厂创建。

#pragma once

#include <atomic>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace planner {
namespace graph_search {

// ---------------------------------------------------------------------------
// 状态
// ---------------------------------------------------------------------------

enum class Status {
  kSuccess,      // 找到路径 / 距离场完成
  kNoPath,       // 起点到终点不可达（预算内搜索正常结束）
  kTimeout,      // 超出时间预算；结果被丢弃
  kCancel,       // 取消标志触发；结果被丢弃
  kInvalidInput  // 输入违反统一契约（含负权、越界、非有限权值）
};

// 稳定字符串（用于证据 JSON 与测试）。
const char* status_to_string(Status status);

// ---------------------------------------------------------------------------
// 输入
// ---------------------------------------------------------------------------

struct Edge {
  int from = 0;
  int to = 0;
  double weight = 0.0;
};

// 有向图输入。无向图以对称边列表表示（每条无向边两条有向边）。
struct Graph {
  int num_vertices = 0;
  std::vector<Edge> edges;
};

// 统一契约校验：顶点数非负、边端点均在 [0, num_vertices)、边权有限且 >= 0。
// 通过返回 true 且不写 reason；失败返回 false 并写入可读原因。
bool validate_graph(const Graph& graph, std::string* reason);

// 顶点 id 是否合法。
bool valid_vertex(const Graph& graph, int vertex);

// ---------------------------------------------------------------------------
// 启发式
// ---------------------------------------------------------------------------

// 顶点 -> 到终点的下界估计。适配器只接受可采纳启发式（不得高估）；
// 无可靠下界时使用 zero_heuristic()。Dijkstra 系列适配器忽略该参数
// （等价零启发式），并在 EffectiveConfig 中记录。
using Heuristic = std::function<double(int)>;

Heuristic zero_heuristic();

// 欧氏启发式。仅当每条边的权重 >= 端点欧氏距离时可采纳；
// 用 euclidean_admissible() 校验夹具（测试与生成器使用）。
class EuclideanHeuristic {
 public:
  EuclideanHeuristic(std::vector<double> xs, std::vector<double> ys, int goal);

  double operator()(int vertex) const;

  bool valid() const { return valid_; }
  int goal() const { return goal_; }

 private:
  std::vector<double> xs_;
  std::vector<double> ys_;
  int goal_ = 0;
  bool valid_ = false;
};

// 校验“所有边权 >= 端点欧氏距离（含 1e-12 松弛）”，即欧氏启发式可采纳。
bool euclidean_admissible(const Graph& graph, const std::vector<double>& xs,
                          const std::vector<double>& ys);

// ---------------------------------------------------------------------------
// 控制（预算/取消）
// ---------------------------------------------------------------------------

struct Control {
  // <= 0 表示无时间预算；否则为秒。
  double budget_s = 0.0;
  // 非空时按需轮询；nullptr 表示无外部取消。
  const std::atomic<bool>* cancel = nullptr;
};

// ---------------------------------------------------------------------------
// 有效配置与统计
// ---------------------------------------------------------------------------

// 适配器实际生效的配置。用于分组/门槛登记与结果解释。
struct EffectiveConfig {
  std::string adapter;               // 稳定适配器 ID（如 "boost_astar"）
  std::string cost_precision;        // "float64" | "float32"
  bool supports_budget = true;       // 搜索过程中可检查时间预算
  bool supports_cancel = true;       // 搜索过程中可响应取消
  bool has_early_stop = true;        // 点到点搜索可在终点处理时早停
  bool uses_heuristic = true;        // false：Dijkstra 语义（忽略启发式）
  std::map<std::string, std::string> options;  // 其余适配器细节
};

struct Stats {
  long expanded_nodes = -1;   // -1 = 上游未提供
  long relaxed_edges = -1;    // -1 = 上游未提供
  double preprocess_ms = -1;  // 图/邻接/属性映射构建（含统一输入转换）
  double search_ms = -1;      // 算法核心
  double wall_ms = -1;        // 适配器调用总耗时
  // 约定：未实际执行搜索的早退（invalid_input 校验拒绝、预检查
  // cancel/timeout）保持 -1（未测量），不得用 0 冒充测量值。
};

// ---------------------------------------------------------------------------
// 输出
// ---------------------------------------------------------------------------

struct Result {
  Status status = Status::kInvalidInput;
  std::vector<int> path;  // 仅 kSuccess 非空；含起点与终点
  double cost = std::numeric_limits<double>::quiet_NaN();  // 原生上报代价
  EffectiveConfig config;
  Stats stats;
  std::string message;  // 诊断信息（invalid_input 原因、上游警告等）
};

// 完整距离场（不可达标记为 +inf；前驱 -1 表示无前驱/源点）。
struct DistanceFieldResult {
  Status status = Status::kInvalidInput;
  std::vector<double> distances;  // 大小 = num_vertices（成功时）
  std::vector<int> predecessors;  // 大小 = num_vertices（成功时）
  EffectiveConfig config;
  Stats stats;
  std::string message;
};

// ---------------------------------------------------------------------------
// 适配器接口
// ---------------------------------------------------------------------------

// 点到点搜索（A* 或 Dijkstra）。实现必须无状态（search 可并发调用不同实例）。
class GraphSearcher {
 public:
  virtual ~GraphSearcher() = default;
  virtual const EffectiveConfig& config() const = 0;
  virtual Result search(const Graph& graph, int start, int goal,
                        const Heuristic& heuristic,
                        const Control& control) const = 0;
};

// 完整距离场搜索（Dijkstra）。
class DistanceFieldSearcher {
 public:
  virtual ~DistanceFieldSearcher() = default;
  virtual const EffectiveConfig& config() const = 0;
  virtual DistanceFieldResult search(const Graph& graph, int source,
                                     const Control& control) const = 0;
};

// ---------------------------------------------------------------------------
// 校验辅助（测试与证据工具共用；不参与算法排名）
// ---------------------------------------------------------------------------

// §6.2 容差：1e-8 + 1e-8 * abs(reference)。
double cost_tolerance(double reference_cost);

// 代价比较（a 为实测，b 为参考）。
bool cost_close(double measured, double reference_cost);

// 返回路径按输入图重算的 double 代价（并行边取最小权；缺边返回 +inf）。
double path_cost(const Graph& graph, const std::vector<int>& path);

// 路径是否为合法起终路径（端点、逐跳边存在、无越界）。
bool path_is_valid(const Graph& graph, const std::vector<int>& path, int start,
                   int goal);

// ---------------------------------------------------------------------------
// JSON 导出（统一输出，字段稳定；不可达距离用 null 表示）
// ---------------------------------------------------------------------------

std::string to_json(const Result& result);
std::string to_json(const DistanceFieldResult& result);

}  // namespace graph_search
}  // namespace planner
