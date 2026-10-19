// graph_gen v1 实现（X-B1）。冻结规则与文件格式见 graph_gen.hpp 与
// benchmarks/README.md；本文件不做任何性能测量。

#include "graph_gen.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <unordered_set>

namespace planner {
namespace graph_search {
namespace bench {

namespace {

// 单流 SplitMix64；消费顺序是 v1 规则的一部分（改动即新版本）。
class SplitMix64 {
 public:
  explicit SplitMix64(std::uint64_t seed) : state_(seed) {}

  std::uint64_t next() {
    state_ += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }

  // [0, 1) 的均匀 double（53 位尾数）。
  double unit() {
    return static_cast<double>(next() >> 11) *
           (1.0 / 9007199254740992.0);
  }

  // [0, bound) 的均匀整数；bound > 0。
  std::uint64_t below(std::uint64_t bound) { return next() % bound; }

 private:
  std::uint64_t state_;
};

std::uint64_t position_key(double x, double y) {
  std::uint64_t xb = 0;
  std::uint64_t yb = 0;
  static_assert(sizeof(double) == sizeof(std::uint64_t), "double is 64-bit");
  std::memcpy(&xb, &x, sizeof(xb));
  std::memcpy(&yb, &y, sizeof(yb));
  // 64 位混合；仅用于精确重复检测（相等坐标 -> 相等键）。
  std::uint64_t h = xb * 0x9E3779B97F4A7C15ULL;
  h ^= yb + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2);
  return h;
}

double euclid(const std::vector<double>& xs, const std::vector<double>& ys,
              int a, int b) {
  return std::hypot(xs[static_cast<std::size_t>(a)] -
                        xs[static_cast<std::size_t>(b)],
                    ys[static_cast<std::size_t>(a)] -
                        ys[static_cast<std::size_t>(b)]);
}

// 从已构建图按 (hops, vertex) 排序选取 8 个可达分位对。
bool select_pairs(const Graph& graph, int num_vertices,
                  std::vector<PairSpec>* pairs, std::string* error) {
  const std::vector<int> hops = bfs_hops(graph, 0);
  std::vector<std::pair<int, int>> reached;  // (hops, vertex)
  for (int v = 1; v < num_vertices; ++v) {
    if (hops[static_cast<std::size_t>(v)] >= 0) {
      reached.push_back({hops[static_cast<std::size_t>(v)], v});
    }
  }
  if (reached.size() < 8) {
    if (error != nullptr) {
      *error = "fewer than 8 reachable vertices from source 0";
    }
    return false;
  }
  std::sort(reached.begin(), reached.end());
  const double quantiles[8] = {0.05, 0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 0.99};
  std::set<int> chosen;
  for (int k = 0; k < 8; ++k) {
    const long long last = static_cast<long long>(reached.size()) - 1;
    long long index = static_cast<long long>(
        std::llround(quantiles[k] * static_cast<double>(last)));
    index = std::max(0LL, std::min(last, index));
    while (chosen.count(reached[static_cast<std::size_t>(index)].second) != 0) {
      index = (index + 1 <= last) ? index + 1 : 0;
    }
    const int goal = reached[static_cast<std::size_t>(index)].second;
    const int hop_count = reached[static_cast<std::size_t>(index)].first;
    chosen.insert(goal);
    PairSpec pair;
    pair.pair_id = "r0" + std::to_string(k + 1);
    pair.start = 0;
    pair.goal = goal;
    pair.hops = hop_count;
    pair.expected = "reachable";
    pairs->push_back(pair);
  }
  PairSpec disconnected;
  disconnected.pair_id = "disconnected";
  disconnected.start = 0;
  disconnected.goal = num_vertices - 1;  // 孤立顶点，生成规则保证不可达
  disconnected.hops = -1;
  disconnected.expected = "unreachable";
  pairs->push_back(disconnected);

  PairSpec same;
  same.pair_id = "same_point";
  same.start = 0;
  same.goal = 0;
  same.hops = 0;
  same.expected = "zero_cost";
  pairs->push_back(same);
  return true;
}

// road：链 + 支路，双向边；权重 = 欧氏距离。
bool generate_road(const InstanceSpec& spec, Instance* instance,
                   std::string* error) {
  const int n = spec.num_vertices;
  const int active = n - 1;  // 顶点 n-1 孤立
  const double pitch = 10.0;
  SplitMix64 rng(spec.seed);

  const int c = std::max(4, static_cast<int>(std::llround(0.6 * active)));
  std::vector<double> xs(static_cast<std::size_t>(n), 0.0);
  std::vector<double> ys(static_cast<std::size_t>(n), 0.0);
  for (int i = 0; i < c; ++i) {
    xs[static_cast<std::size_t>(i)] = i * pitch;
    ys[static_cast<std::size_t>(i)] =
        40.0 * std::sin(0.05 * i) + 1.5 * (rng.unit() - 0.5);
  }

  const int r = active - c;
  const int b = std::max(1, c / 8);
  std::vector<int> branch_len(static_cast<std::size_t>(b), r / b);
  const int extra = r % b;
  for (int j = 0; j < extra; ++j) {
    ++branch_len[static_cast<std::size_t>(rng.below(static_cast<std::uint64_t>(b)))];
  }

  std::vector<Edge> edges;
  auto add_two_way = [&](int u, int v) {
    const double w = euclid(xs, ys, u, v);
    Edge forward;
    forward.from = u;
    forward.to = v;
    forward.weight = w;
    Edge backward;
    backward.from = v;
    backward.to = u;
    backward.weight = w;
    edges.push_back(forward);
    edges.push_back(backward);
  };
  for (int i = 0; i + 1 < c; ++i) {
    add_two_way(i, i + 1);
  }

  int next_id = c;
  for (int j = 0; j < b; ++j) {
    const int attach = (7 * j + 3) % c;
    double tx = 0.0;
    double ty = 0.0;
    if (attach + 1 < c) {
      tx = xs[static_cast<std::size_t>(attach + 1)] -
           xs[static_cast<std::size_t>(attach)];
      ty = ys[static_cast<std::size_t>(attach + 1)] -
           ys[static_cast<std::size_t>(attach)];
    } else {
      tx = xs[static_cast<std::size_t>(attach)] -
           xs[static_cast<std::size_t>(attach - 1)];
      ty = ys[static_cast<std::size_t>(attach)] -
           ys[static_cast<std::size_t>(attach - 1)];
    }
    const double tangent_length = std::hypot(tx, ty);
    tx /= tangent_length;
    ty /= tangent_length;
    const double px = -ty;
    const double py = tx;
    int previous = attach;
    for (int k = 0; k < branch_len[static_cast<std::size_t>(j)]; ++k) {
      const double along = pitch * 0.15 * (rng.unit() - 0.5);
      xs[static_cast<std::size_t>(next_id)] =
          xs[static_cast<std::size_t>(attach)] + px * pitch * (k + 1) + tx * along;
      ys[static_cast<std::size_t>(next_id)] =
          ys[static_cast<std::size_t>(attach)] + py * pitch * (k + 1) + ty * along;
      add_two_way(previous, next_id);
      previous = next_id;
      ++next_id;
    }
  }
  if (next_id != active) {
    if (error != nullptr) {
      *error = "road generator placed " + std::to_string(next_id - c) +
               " branch vertices but expected " + std::to_string(r);
    }
    return false;
  }

  instance->spec = spec;
  instance->graph.num_vertices = n;
  instance->graph.edges = std::move(edges);
  instance->xs = std::move(xs);
  instance->ys = std::move(ys);
  instance->isolated_vertex = n - 1;
  instance->undirected_edges = static_cast<long long>(instance->graph.edges.size() / 2);
  if (!select_pairs(instance->graph, n, &instance->pairs, error)) {
    return false;
  }
  return true;
}

// rand：随机生成树（0 为根）+ 随机补边到 m = 2n；权重 = 欧氏距离 × (1+u)。
bool generate_rand(const InstanceSpec& spec, Instance* instance,
                   std::string* error) {
  const int n = spec.num_vertices;
  const int active = n - 1;  // 顶点 n-1 孤立
  SplitMix64 rng(spec.seed);

  std::vector<double> xs(static_cast<std::size_t>(n), 0.0);
  std::vector<double> ys(static_cast<std::size_t>(n), 0.0);
  std::unordered_set<std::uint64_t> seen_positions;
  seen_positions.reserve(static_cast<std::size_t>(active) * 2);
  for (int i = 0; i < active; ++i) {
    for (;;) {
      const double x = rng.unit() * 1000.0;
      const double y = rng.unit() * 1000.0;
      if (seen_positions.insert(position_key(x, y)).second) {
        xs[static_cast<std::size_t>(i)] = x;
        ys[static_cast<std::size_t>(i)] = y;
        break;
      }
    }
  }

  std::vector<Edge> edges;
  edges.reserve(static_cast<std::size_t>(2 * n));
  std::unordered_set<std::uint64_t> used;
  used.reserve(static_cast<std::size_t>(2 * n) * 2);
  auto add_edge = [&](int u, int v) {
    const double w = euclid(xs, ys, u, v) * (1.0 + rng.unit());
    Edge edge;
    edge.from = u;
    edge.to = v;
    edge.weight = w;
    edges.push_back(edge);
    used.insert(edge_key(u, v));
  };
  // 随机生成树：parent ∈ [0, i)，方向 parent -> i（0 可达全部活动顶点）。
  for (int i = 1; i < active; ++i) {
    const int parent = static_cast<int>(rng.below(static_cast<std::uint64_t>(i)));
    add_edge(parent, i);
  }
  // 补边到 m = 2n。
  const std::size_t target_m = static_cast<std::size_t>(2) * static_cast<std::size_t>(n);
  const std::size_t extra_needed = target_m - edges.size();
  std::size_t added = 0;
  std::size_t attempts = 0;
  const std::size_t max_attempts = 64 * extra_needed + 1024;
  while (added < extra_needed && attempts < max_attempts) {
    ++attempts;
    const int u = static_cast<int>(rng.below(static_cast<std::uint64_t>(active)));
    const int v = static_cast<int>(rng.below(static_cast<std::uint64_t>(active)));
    if (u == v) {
      continue;
    }
    if (used.count(edge_key(u, v)) != 0) {
      continue;
    }
    add_edge(u, v);
    ++added;
  }
  if (added != extra_needed) {
    if (error != nullptr) {
      *error = "rand generator could not reach m = 2n unique edges (added " +
               std::to_string(edges.size()) + " of " + std::to_string(target_m) +
               ")";
    }
    return false;
  }

  instance->spec = spec;
  instance->graph.num_vertices = n;
  instance->graph.edges = std::move(edges);
  instance->xs = std::move(xs);
  instance->ys = std::move(ys);
  instance->isolated_vertex = n - 1;
  instance->undirected_edges = -1;  // 有向随机图不按无向边计数
  if (!select_pairs(instance->graph, n, &instance->pairs, error)) {
    return false;
  }
  return true;
}

}  // namespace

std::vector<InstanceSpec> default_instance_specs() {
  return {
      {"graph-road-1k", "road", "1k", 1000, 101},
      {"graph-road-10k", "road", "10k", 10000, 102},
      {"graph-road-100k", "road", "100k", 100000, 103},
      {"graph-rand-1k", "rand", "1k", 1000, 201},
      {"graph-rand-10k", "rand", "10k", 10000, 202},
      {"graph-rand-100k", "rand", "100k", 100000, 203},
  };
}

const PairSpec* Instance::find_pair(const std::string& pair_id) const {
  for (const PairSpec& pair : pairs) {
    if (pair.pair_id == pair_id) {
      return &pair;
    }
  }
  return nullptr;
}

bool generate_instance(const InstanceSpec& spec, Instance* instance,
                       std::string* error) {
  if (instance == nullptr) {
    if (error != nullptr) {
      *error = "instance pointer is null";
    }
    return false;
  }
  if (spec.num_vertices < 64) {
    if (error != nullptr) {
      *error = "graph_gen v1 requires num_vertices >= 64";
    }
    return false;
  }
  instance->pairs.clear();
  if (spec.graph_class == "road") {
    return generate_road(spec, instance, error);
  }
  if (spec.graph_class == "rand") {
    return generate_rand(spec, instance, error);
  }
  if (error != nullptr) {
    *error = "unknown graph class: " + spec.graph_class;
  }
  return false;
}

std::string serialize_instance(const Instance& instance) {
  std::ostringstream out;
  out << "graph_gen v" << kGraphGenVersion << "\n";
  out << "instance " << instance.spec.instance_id << "\n";
  out << "class " << instance.spec.graph_class << "\n";
  out << "scale " << instance.spec.scale << "\n";
  out << "seed " << instance.spec.seed << "\n";
  out << "vertices " << instance.graph.num_vertices << "\n";
  out << "edges " << instance.graph.edges.size() << "\n";
  out << "isolated_vertex " << instance.isolated_vertex << "\n";
  out << "pairs " << instance.pairs.size() << "\n";
  out << std::setprecision(17);
  for (int i = 0; i < instance.graph.num_vertices; ++i) {
    out << "V " << i << " " << instance.xs[static_cast<std::size_t>(i)] << " "
        << instance.ys[static_cast<std::size_t>(i)] << "\n";
  }
  for (const Edge& edge : instance.graph.edges) {
    out << "E " << edge.from << " " << edge.to << " " << edge.weight << "\n";
  }
  for (const PairSpec& pair : instance.pairs) {
    out << "P " << pair.pair_id << " " << pair.start << " " << pair.goal << " "
        << pair.hops << " " << pair.expected << "\n";
  }
  return out.str();
}

ReferenceField reference_dijkstra(const Graph& graph, int source) {
  ReferenceField field;
  if (!valid_vertex(graph, source)) {
    return field;
  }
  const int n = graph.num_vertices;
  std::vector<std::vector<std::pair<int, double>>> adjacency(
      static_cast<std::size_t>(n));
  for (const Edge& edge : graph.edges) {
    adjacency[static_cast<std::size_t>(edge.from)].push_back(
        {edge.to, edge.weight});
  }
  field.dist.assign(static_cast<std::size_t>(n),
                    std::numeric_limits<double>::infinity());
  field.parent.assign(static_cast<std::size_t>(n), -1);
  using QueueEntry = std::pair<double, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                      std::greater<QueueEntry>>
      queue;
  field.dist[static_cast<std::size_t>(source)] = 0.0;
  queue.push({0.0, source});
  while (!queue.empty()) {
    const auto [distance, vertex] = queue.top();
    queue.pop();
    if (distance > field.dist[static_cast<std::size_t>(vertex)]) {
      continue;
    }
    for (const auto& [next, weight] : adjacency[static_cast<std::size_t>(vertex)]) {
      const double candidate = distance + weight;
      if (candidate < field.dist[static_cast<std::size_t>(next)]) {
        field.dist[static_cast<std::size_t>(next)] = candidate;
        field.parent[static_cast<std::size_t>(next)] = vertex;
        queue.push({candidate, next});
      }
    }
  }
  field.unreachable = 0;
  field.max_finite = 0.0;
  for (int v = 0; v < n; ++v) {
    const double distance = field.dist[static_cast<std::size_t>(v)];
    if (!std::isfinite(distance)) {
      ++field.unreachable;
    } else {
      field.max_finite = std::max(field.max_finite, distance);
    }
  }
  field.ok = true;
  return field;
}

std::vector<int> bfs_hops(const Graph& graph, int source) {
  const int n = graph.num_vertices;
  std::vector<int> hops(static_cast<std::size_t>(n), -1);
  if (!valid_vertex(graph, source)) {
    return hops;
  }
  std::vector<std::vector<int>> adjacency(static_cast<std::size_t>(n));
  for (const Edge& edge : graph.edges) {
    adjacency[static_cast<std::size_t>(edge.from)].push_back(edge.to);
  }
  std::queue<int> queue;
  hops[static_cast<std::size_t>(source)] = 0;
  queue.push(source);
  while (!queue.empty()) {
    const int vertex = queue.front();
    queue.pop();
    for (const int next : adjacency[static_cast<std::size_t>(vertex)]) {
      if (hops[static_cast<std::size_t>(next)] < 0) {
        hops[static_cast<std::size_t>(next)] =
            hops[static_cast<std::size_t>(vertex)] + 1;
        queue.push(next);
      }
    }
  }
  return hops;
}

std::uint64_t edge_key(int from, int to) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(from)) << 32) |
         static_cast<std::uint32_t>(to);
}

EdgeIndex::EdgeIndex(const Graph& graph) {
  min_weight_.reserve(graph.edges.size() * 2);
  for (const Edge& edge : graph.edges) {
    const std::uint64_t key = edge_key(edge.from, edge.to);
    auto found = min_weight_.find(key);
    if (found == min_weight_.end() || edge.weight < found->second) {
      min_weight_[key] = edge.weight;
    }
  }
}

double EdgeIndex::path_cost(const std::vector<int>& path,
                            bool* edge_missing) const {
  if (edge_missing != nullptr) {
    *edge_missing = false;
  }
  if (path.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto found = min_weight_.find(edge_key(path[i - 1], path[i]));
    if (found == min_weight_.end()) {
      if (edge_missing != nullptr) {
        *edge_missing = true;
      }
      return std::numeric_limits<double>::infinity();
    }
    total += found->second;
  }
  return total;
}

bool EdgeIndex::path_is_valid(const Graph& graph, const std::vector<int>& path,
                              int start, int goal) const {
  if (path.empty() || path.front() != start || path.back() != goal) {
    return false;
  }
  for (const int vertex : path) {
    if (!valid_vertex(graph, vertex)) {
      return false;
    }
  }
  bool missing = false;
  const double cost = path_cost(path, &missing);
  return !missing && std::isfinite(cost);
}

bool check_instance(const Instance& instance, std::vector<std::string>* issues) {
  auto report = [&](const std::string& text) {
    if (issues != nullptr) {
      issues->push_back(instance.spec.instance_id + ": " + text);
    }
  };
  const Graph& graph = instance.graph;
  const int n = graph.num_vertices;
  if (instance.spec.instance_id.empty() || instance.spec.scale.empty()) {
    report("spec fields are empty");
  }
  if (static_cast<int>(instance.xs.size()) != n ||
      static_cast<int>(instance.ys.size()) != n) {
    report("coordinate arrays do not match num_vertices");
    return false;
  }
  std::string reason;
  if (!validate_graph(graph, &reason)) {
    report("validate_graph failed: " + reason);
    return false;
  }
  long long zero_weights = 0;
  long long inadmissible = 0;
  long long self_loops = 0;
  long long isolated_incidents = 0;
  for (const Edge& edge : graph.edges) {
    if (edge.weight == 0.0) {
      ++zero_weights;
    }
    if (edge.from == edge.to) {
      ++self_loops;
    }
    if (edge.weight + 1e-12 < euclid(instance.xs, instance.ys, edge.from, edge.to)) {
      ++inadmissible;
    }
    if (edge.from == instance.isolated_vertex ||
        edge.to == instance.isolated_vertex) {
      ++isolated_incidents;
    }
  }
  if (zero_weights != 0) {
    report("zero-weight edges: " + std::to_string(zero_weights));
  }
  if (inadmissible != 0) {
    report("edges with weight < euclidean distance: " +
           std::to_string(inadmissible));
  }
  if (self_loops != 0) {
    report("self loops: " + std::to_string(self_loops));
  }
  if (isolated_incidents != 0) {
    report("isolated vertex has incident edges: " +
           std::to_string(isolated_incidents));
  }
  const double average_out_degree =
      n > 0 ? static_cast<double>(graph.edges.size()) / static_cast<double>(n)
            : 0.0;
  if (average_out_degree < 1.9 || average_out_degree > 2.1) {
    std::ostringstream text;
    text << "average out-degree outside [1.9, 2.1]: " << average_out_degree;
    report(text.str());
  }
  if (instance.spec.graph_class == "road" &&
      static_cast<long long>(graph.edges.size()) != 2LL * (n - 2)) {
    report("road edge count != 2*(n-2)");
  }
  if (instance.spec.graph_class == "rand" &&
      static_cast<long long>(graph.edges.size()) != 2LL * n) {
    report("rand edge count != 2*n");
  }

  if (instance.pairs.size() != 10) {
    report("pair count != 10");
  }
  const PairSpec* disconnected = instance.find_pair("disconnected");
  const PairSpec* same = instance.find_pair("same_point");
  std::set<int> reachable_goals;
  int reachable_count = 0;
  for (const PairSpec& pair : instance.pairs) {
    if (pair.expected == "reachable") {
      ++reachable_count;
      if (!reachable_goals.insert(pair.goal).second) {
        report("duplicate reachable goal " + std::to_string(pair.goal));
      }
      if (pair.start != 0) {
        report("reachable pair start != 0: " + pair.pair_id);
      }
    }
  }
  if (reachable_count != 8) {
    report("reachable pair count != 8: " + std::to_string(reachable_count));
  }
  if (disconnected == nullptr || same == nullptr) {
    report("disconnected/same_point pair missing");
    return issues == nullptr || issues->empty();
  }

  const std::vector<int> hops = bfs_hops(graph, 0);
  for (const PairSpec& pair : instance.pairs) {
    if (pair.expected != "reachable") {
      continue;
    }
    if (hops[static_cast<std::size_t>(pair.goal)] != pair.hops) {
      report("pair " + pair.pair_id + " hop count mismatch (recorded " +
             std::to_string(pair.hops) + ", bfs " +
             std::to_string(hops[static_cast<std::size_t>(pair.goal)]) + ")");
    }
  }

  const ReferenceField reference = reference_dijkstra(graph, 0);
  if (!reference.ok) {
    report("reference dijkstra failed");
    return issues == nullptr || issues->empty();
  }
  for (const PairSpec& pair : instance.pairs) {
    const double distance = reference.dist[static_cast<std::size_t>(pair.goal)];
    if (pair.expected == "reachable" && !std::isfinite(distance)) {
      report("reachable pair " + pair.pair_id + " is unreachable in reference");
    }
    if (pair.expected == "unreachable" && std::isfinite(distance)) {
      report("disconnected pair is reachable in reference");
    }
    if (pair.expected == "zero_cost" &&
        (pair.start != pair.goal || distance != 0.0)) {
      report("same_point pair is not the zero-cost source pair");
    }
  }
  if (disconnected->goal != instance.isolated_vertex) {
    report("disconnected goal is not the isolated vertex");
  }
  if (!std::isinf(reference.dist[static_cast<std::size_t>(disconnected->goal)])) {
    report("isolated vertex is reachable");
  }
  return issues == nullptr || issues->empty();
}

double float32_registered_tolerance(double reference_cost,
                                    std::size_t path_edges) {
  const double per_edge = 1.2e-7;  // ≈ 2 × 2^-24，逐边 float32 累加允许量
  const double relative =
      std::max(1e-5, per_edge * static_cast<double>(path_edges));
  return 1e-5 + relative * std::fabs(reference_cost);
}

}  // namespace bench
}  // namespace graph_search
}  // namespace planner
