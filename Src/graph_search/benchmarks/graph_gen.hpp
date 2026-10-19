// graph_gen v1：图搜索正式基准的确定性实例生成器（X-B1）。
//
// 冻结规则（Docs/expansion/benchmark_tiers.md §2.1）：
//  * 类别 road（道路状稀疏）：链 + 支路，双向边，平均出度 ≈ 2，权重 = 端点
//    欧氏距离（几何权重），种子派生链侧向抖动与支路长度；
//  * 类别 rand（固定随机稀疏）：G(n, m)，m = 2n 有向边；先以 0 为根生成随机
//    生成树（父 -> 子，保证弱连通与 0 可达），再补随机有向边到 m；权重 =
//    欧氏距离 × (1 + u)，u ∈ [0, 1) 由种子派生；
//  * 顶点 n-1 为孤立顶点（无任何关联边），断连对 (0, n-1) 由生成规则保证
//    不可达；起终点相同对 (0, 0)；
//  * 8 个可达对按源点 0 的 BFS 跳数分布取最近秩分位点
//    q = {0.05, 0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 0.99}，覆盖短/中/长；
//  * 非负权、无零权边；所有权重 ≥ 端点欧氏距离（欧氏启发式可采纳）；
//  * 全部随机数来自单流 SplitMix64，消费顺序固定；实例完全确定。
//
// 生成器规则、文件格式与门槛的完整说明见 benchmarks/README.md。

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "planner/graph_search/types.hpp"

namespace planner {
namespace graph_search {
namespace bench {

constexpr int kGraphGenVersion = 1;

// 正式档 6 个冻结实例：1k/10k/100k × road/rand，种子 101–103 / 201–203。
struct InstanceSpec {
  std::string instance_id;
  std::string graph_class;  // "road" | "rand"
  std::string scale;        // "1k" | "10k" | "100k"
  int num_vertices = 0;
  std::uint64_t seed = 0;
};

std::vector<InstanceSpec> default_instance_specs();

// 起终点对（8 可达 + 1 断连 + 1 同点）。
struct PairSpec {
  std::string pair_id;   // "r01".."r08" / "disconnected" / "same_point"
  int start = 0;
  int goal = 0;
  int hops = -1;         // BFS 跳数（可达对）；-1 表示断连/不适用
  std::string expected;  // "reachable" | "unreachable" | "zero_cost"
};

struct Instance {
  InstanceSpec spec;
  Graph graph;
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<PairSpec> pairs;
  int isolated_vertex = -1;
  long long undirected_edges = 0;

  const PairSpec* find_pair(const std::string& pair_id) const;
};

// 生成一个实例。失败（顶点数过小、补边次数超限等）时返回 false 并写入
// error；正常输入不会失败。
bool generate_instance(const InstanceSpec& spec, Instance* instance,
                       std::string* error);

// v1 文本序列化（确定性；%.17g 精确往返）：
//   头 + "V <i> <x> <y>" × n + "E <from> <to> <weight>" × m
//   + "P <pair_id> <start> <goal> <hops> <expected>" × 10
std::string serialize_instance(const Instance& instance);

// 独立参考 Dijkstra（标准库优先队列；double）。与适配器实现无关，用于
// 正确性门禁、参考代价与距离场核对；不参与性能测量。
struct ReferenceField {
  std::vector<double> dist;   // 大小 n；不可达 = +inf
  std::vector<int> parent;    // -1 = 源点/不可达
  long long unreachable = 0;
  double max_finite = 0.0;
  bool ok = false;
};

ReferenceField reference_dijkstra(const Graph& graph, int source);

// BFS 跳数；不可达 = -1。
std::vector<int> bfs_hops(const Graph& graph, int source);

// 预构建有向边索引（(from,to) -> 最小权），用于逐 trial 快速重算路径代价。
class EdgeIndex {
 public:
  explicit EdgeIndex(const Graph& graph);

  // 路径按索引重算的 double 代价；缺边时返回 +inf 并置 *edge_missing。
  double path_cost(const std::vector<int>& path, bool* edge_missing) const;

  bool path_is_valid(const Graph& graph, const std::vector<int>& path, int start,
                     int goal) const;

 private:
  std::unordered_map<std::uint64_t, double> min_weight_;
};

std::uint64_t edge_key(int from, int to);

// 结构自检（README §"生成器自检"）：契约校验、无零权边、权重 ≥ 欧氏距离、
// 孤立顶点无关联边、对可达性/代价与参考一致、平均出度 ≈ 2、跳数一致。
bool check_instance(const Instance& instance, std::vector<std::string>* issues);

// justinhj 原生 float32 代价在正式档长路径上的已登记门槛（X-B1 冻结）：
//   tol = 1e-5 + max(1e-5, 1.2e-7 × path_edges) × |reference|
// 见 benchmarks/README.md §"justinhj float32 口径"。非 justinhj 后端使用
// 组件统一容差 cost_tolerance（§6.2）。
double float32_registered_tolerance(double reference_cost,
                                    std::size_t path_edges);

}  // namespace bench
}  // namespace graph_search
}  // namespace planner
