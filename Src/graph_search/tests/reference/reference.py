#!/usr/bin/env python3
"""planner_graph_search 独立 Python 参考（仅标准库，CPython 3.8+）。

对 C++ 测试夹具 A（12 节点有向）、B（无向对称）、C（零权边）运行标准库
Dijkstra，输出期望代价/距离场，并与 C++ 测试中手算的常量交叉核对；不一致时
以非零退出码报告。本脚本不依赖任何仓库组件，属独立参考实现（组件扩展计划
§6.2“小图采用可手算期望或独立社区参考核对”）。

用法：
    python3 reference.py            # 打印报告并自检
    python3 reference.py --json     # 额外输出机器可读 JSON
"""

import argparse
import heapq
import json
import math
import sys

INF = float("inf")


def dijkstra(num_vertices, edges, source):
    adjacency = [[] for _ in range(num_vertices)]
    for src, dst, weight in edges:
        adjacency[src].append((dst, weight))
    distances = [INF] * num_vertices
    predecessors = [-1] * num_vertices
    distances[source] = 0.0
    heap = [(0.0, source)]
    settled = [False] * num_vertices
    while heap:
        dist, vertex = heapq.heappop(heap)
        if settled[vertex]:
            continue
        settled[vertex] = True
        for dst, weight in adjacency[vertex]:
            candidate = dist + weight
            if candidate < distances[dst] - 1e-12:
                distances[dst] = candidate
                predecessors[dst] = vertex
                heapq.heappush(heap, (candidate, dst))
    return distances, predecessors


def path_from_predecessors(predecessors, source, goal):
    if source == goal:
        return [source]
    path = []
    vertex = goal
    while vertex != -1:
        path.append(vertex)
        if vertex == source:
            return list(reversed(path))
        vertex = predecessors[vertex]
    return []


FIXTURE_A_EDGES = [
    (0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (3, 6, 1.0),
    (0, 4, 1.5), (4, 5, 1.0), (1, 5, 1.6), (2, 5, 1.0),
    (5, 6, 1.0), (5, 7, 1.2), (7, 8, 1.0), (8, 9, 1.0),
    (6, 8, 1.0), (10, 11, 1.0),
]
FIXTURE_B_UNDIRECTED = [
    (0, 1, 2.0), (0, 2, 5.0), (1, 2, 1.0), (1, 3, 4.0), (2, 3, 2.0),
    (2, 4, 7.0), (3, 4, 1.0), (3, 5, 3.0), (4, 5, 2.0),
]
FIXTURE_C_EDGES = [
    (0, 1, 0.0), (1, 2, 0.0), (0, 2, 1.0), (2, 3, 1.0),
    (3, 4, 0.5), (2, 4, 3.0), (2, 0, 0.0), (1, 0, 0.0),
]


def fixture_b_symmetric():
    edges = []
    for src, dst, weight in FIXTURE_B_UNDIRECTED:
        edges.append((src, dst, weight))
        edges.append((dst, src, weight))
    return edges


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true",
                        help="额外输出机器可读 JSON")
    args = parser.parse_args()

    report = {}
    failures = []

    def check(name, actual, expected, tolerance=1e-9):
        if expected == INF:
            ok = actual == INF
        else:
            ok = abs(actual - expected) <= tolerance + tolerance * abs(expected)
        if not ok:
            failures.append("%s: actual=%r expected=%r" % (name, actual, expected))
        return ok

    # Fixture A：0->9 代价 5.5 路径 0-4-5-6-8-9；0->11 不可达；距离场。
    a_dist, a_pred = dijkstra(12, FIXTURE_A_EDGES, 0)
    check("A.dist[9]", a_dist[9], 5.5)
    check("A.dist[11]", a_dist[11], INF)
    a_path = path_from_predecessors(a_pred, 0, 9)
    check("A.path", float(len(a_path)), 6.0)
    if a_path != [0, 4, 5, 6, 8, 9]:
        failures.append("A.path: %r" % (a_path,))
    a_expected = [0.0, 1.0, 2.0, 3.0, 1.5, 2.5, 3.5, 3.7, 4.5, 5.5, INF, INF]
    for v in range(12):
        check("A.dist[%d]" % v, a_dist[v], a_expected[v])
    report["fixture_a_distances"] = [
        None if d == INF else d for d in a_dist]
    report["fixture_a_path_0_9"] = a_path

    # Fixture B：0->5 代价 8（两条等代价路线）；6 孤立。
    b_edges = fixture_b_symmetric()
    b_dist, b_pred = dijkstra(7, b_edges, 0)
    b_expected = [0.0, 2.0, 3.0, 5.0, 6.0, 8.0, INF]
    for v in range(7):
        check("B.dist[%d]" % v, b_dist[v], b_expected[v])
    b_path = path_from_predecessors(b_pred, 0, 5)
    check("B.path_cost", sum(
        next(w for s, d, w in b_edges if (s, d) == (b_path[i], b_path[i + 1]))
        for i in range(len(b_path) - 1)), 8.0)
    report["fixture_b_distances"] = [None if d == INF else d for d in b_dist]

    # Fixture C：零权边；0->4 代价 1.5；0->2 代价 0。
    c_dist, _ = dijkstra(5, FIXTURE_C_EDGES, 0)
    c_expected = [0.0, 0.0, 0.0, 1.0, 1.5]
    for v in range(5):
        check("C.dist[%d]" % v, c_dist[v], c_expected[v])
    report["fixture_c_distances"] = c_dist

    # Fixture F：32 条 sqrt(2) 边链，代价 32*sqrt(2)。
    report["fixture_f_expected_cost"] = 32 * math.sqrt(2.0)

    print("planner_graph_search independent reference (Python stdlib Dijkstra)")
    print(json.dumps(report, indent=2, sort_keys=True))
    if failures:
        print("MISMATCHES:", file=sys.stderr)
        for failure in failures:
            print("  - " + failure, file=sys.stderr)
        return 1
    print("all hand-computed expectations verified: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
