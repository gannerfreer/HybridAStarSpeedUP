// planner_graph_search 独立示例：在小型合成图上运行所有已编译适配器，
// 打印统一结果（JSON），并可用 --self-check 做手算期望自检（CTest 注册）。
//
// 运行：
//   planner_graph_search_demo            # 打印各适配器结果
//   planner_graph_search_demo --self-check
//
// 夹具与 tests/fixtures.cpp 的 fixture A 相同（12 节点有向图；手算
// 0->9 最优代价 5.5，路径 0-4-5-6-8-9；0->11 不可达）。

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "planner/graph_search/adapters.hpp"
#include "planner/graph_search/types.hpp"

namespace {

using planner::graph_search::Control;
using planner::graph_search::DistanceFieldSearcher;
using planner::graph_search::Edge;
using planner::graph_search::Graph;
using planner::graph_search::GraphSearcher;
using planner::graph_search::Heuristic;
using planner::graph_search::Result;
using planner::graph_search::Status;
using planner::graph_search::cost_close;
using planner::graph_search::path_cost;
using planner::graph_search::to_json;
using planner::graph_search::zero_heuristic;

Graph demo_graph() {
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

struct SearcherEntry {
  std::string name;
  std::unique_ptr<GraphSearcher> searcher;
};

struct FieldEntry {
  std::string name;
  std::unique_ptr<DistanceFieldSearcher> searcher;
};

std::vector<SearcherEntry> make_searchers() {
  std::vector<SearcherEntry> entries;
#ifdef PLANNER_GRAPH_SEARCH_HAS_BOOST
  entries.push_back({"boost_astar", planner::graph_search::make_boost_astar()});
  entries.push_back(
      {"boost_dijkstra", planner::graph_search::make_boost_dijkstra()});
#endif
#ifdef PLANNER_GRAPH_SEARCH_HAS_LMRP
  entries.push_back({"lmrp_astar", planner::graph_search::make_lmrp_astar()});
#endif
#ifdef PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_POOL
  entries.push_back({"justinhj_astar_pool",
                     planner::graph_search::make_justinhj_astar_pool()});
#endif
#ifdef PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_NOPOOL
  entries.push_back({"justinhj_astar_nopool",
                     planner::graph_search::make_justinhj_astar_nopool()});
#endif
#ifdef PLANNER_GRAPH_SEARCH_HAS_LEMON
  entries.push_back(
      {"lemon_dijkstra", planner::graph_search::make_lemon_dijkstra()});
#endif
  // igraph（GPL-2.0-or-later 隔离组）不在示例中链接；其适配器由专用测试
  // 可执行文件 planner_graph_search_tests_igraph 覆盖。
  return entries;
}

std::vector<FieldEntry> make_field_searchers() {
  std::vector<FieldEntry> entries;
#ifdef PLANNER_GRAPH_SEARCH_HAS_BOOST
  entries.push_back({"boost_distance_field",
                     planner::graph_search::make_boost_distance_field()});
#endif
#ifdef PLANNER_GRAPH_SEARCH_HAS_LEMON
  entries.push_back({"lemon_distance_field",
                     planner::graph_search::make_lemon_distance_field()});
#endif
  // igraph 距离场同样由隔离测试覆盖，不在示例中链接。
  return entries;
}

int run(bool self_check) {
  const Graph graph = demo_graph();
  const Heuristic heuristic = zero_heuristic();
  Control control;

  int failures = 0;
  for (auto& entry : make_searchers()) {
    const Result result = entry.searcher->search(graph, 0, 9, heuristic, control);
    const bool ok = result.status == Status::kSuccess &&
                    cost_close(result.cost, 5.5) &&
                    path_cost(graph, result.path) == 5.5;
    if (!ok) {
      ++failures;
    }
    if (self_check) {
      std::printf("%-24s %s (cost=%.10f)\n", entry.name.c_str(),
                  ok ? "OK" : "FAIL", result.cost);
    } else {
      std::printf("== %s ==\n%s\n", entry.name.c_str(),
                  to_json(result).c_str());
    }
  }

  for (auto& entry : make_field_searchers()) {
    const auto result = entry.searcher->search(graph, 0, control);
    const bool ok = result.status == Status::kSuccess &&
                    result.distances.size() == 12 &&
                    result.distances[9] == 5.5;
    if (!ok) {
      ++failures;
    }
    if (self_check) {
      std::printf("%-24s %s (dist[9]=%.10f)\n", entry.name.c_str(),
                  ok ? "OK" : "FAIL", result.distances.empty()
                                         ? -1.0
                                         : result.distances[9]);
    } else {
      std::printf("== %s ==\n%s\n", entry.name.c_str(),
                  to_json(result).c_str());
    }
  }

  if (self_check) {
    std::printf("self-check: %s (%d failures)\n",
                failures == 0 ? "PASS" : "FAIL", failures);
  }
  return failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  bool self_check = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--self-check") == 0) {
      self_check = true;
    } else {
      std::fprintf(stderr, "usage: %s [--self-check]\n", argv[0]);
      return 2;
    }
  }
  return run(self_check);
}
