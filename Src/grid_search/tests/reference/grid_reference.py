#!/usr/bin/env python3
"""独立 Python 参考实现（X-S3 栅格正确性档夹具）。

只使用标准库（CPython 3.8–3.13）。职责：

* 确定性生成 10 个正确性场景（开阔/迷宫/窄道/死胡同/不可达/边界斜角/
  对角缝隙/unknown 阻塞/unknown 通道/无围墙边界），冻结为 scenarios.json；
* 用独立 Dijkstra 计算每个场景在四邻域、八邻域（禁对角穿角，统一契约）与
  八邻域允许穿角（仅用于 jps3d JPS 单列变体的原生语义核对）下的
  最优代价与完整距离场，冻结为 expected.json；
* ``--check`` 模式重新生成并与冻结文件比较（CTest 夹具复现测试）。

单位：距离场与代价以米为单位（单元步长正交 1、对角 sqrt(2)，乘以分辨率）；
不可达为 JSON null。本参考实现与 C++ 适配器相互独立（不同语言、不同作者），
用于 §6.2 “与独立 Python 参考核对代价”。

用法：
  grid_reference.py --write-scenarios <path>     # 生成场景夹具
  grid_reference.py --write-expected <path>      # 由场景夹具生成期望值
  grid_reference.py --check-scenarios <path>     # 校验冻结场景夹具
  grid_reference.py --check-expected <path>      # 校验冻结期望值
"""

import argparse
import heapq
import json
import math
import sys

FIXTURE_NAME = "grid_search_scenarios_v1"
FIXTURE_VERSION = 1
GENERATOR = "grid_reference.py v1 (deterministic, stdlib only)"
CHECK_TOLERANCE = 1e-9


# ---------------------------------------------------------------------------
# 确定性伪随机（自带 LCG，跨 Python 版本稳定）
# ---------------------------------------------------------------------------

class Lcg:
    def __init__(self, seed):
        self.state = (seed * 2654435761 + 1013904223) & 0x7FFFFFFF

    def next(self):
        self.state = (1103515245 * self.state + 12345) & 0x7FFFFFFF
        return self.state

    def permutation(self, items):
        result = list(items)
        for index in range(len(result) - 1, 0, -1):
            swap = self.next() % (index + 1)
            result[index], result[swap] = result[swap], result[index]
        return result


# ---------------------------------------------------------------------------
# 场景定义（全部确定性；无外部输入）
# ---------------------------------------------------------------------------

def blank(width, height, border=True):
    cells = [["." for _ in range(width)] for _ in range(height)]
    if border:
        for col in range(width):
            cells[0][col] = "#"
            cells[height - 1][col] = "#"
        for row in range(height):
            cells[row][0] = "#"
            cells[row][width - 1] = "#"
    return cells


def fill_rect(cells, row0, col0, row1, col1, value="#"):
    for row in range(row0, row1 + 1):
        for col in range(col0, col1 + 1):
            cells[row][col] = value


def maze_cells(size=25, seed=0):
    """递归回溯迷宫：房间 2x2、墙厚 1（通道宽 2），确定性 LCG 顺序。"""
    rooms = (size - 1) // 3
    cells = [["#"] * size for _ in range(size)]
    for ri in range(rooms):
        for rj in range(rooms):
            fill_rect(cells, 3 * ri + 1, 3 * rj + 1, 3 * ri + 2, 3 * rj + 2, ".")
    visited = [[False] * rooms for _ in range(rooms)]
    stack = [(0, 0)]
    visited[0][0] = True
    while stack:
        ri, rj = stack[-1]
        directions = Lcg(seed + ri * 131 + rj * 17).permutation(
            [(0, 1), (1, 0), (0, -1), (-1, 0)])
        moved = False
        for dr, dc in directions:
            nr, nc = ri + dr, rj + dc
            if not (0 <= nr < rooms and 0 <= nc < rooms) or visited[nr][nc]:
                continue
            if dr == 1:
                fill_rect(cells, 3 * ri + 3, 3 * rj + 1, 3 * ri + 3,
                          3 * rj + 2, ".")
            elif dr == -1:
                fill_rect(cells, 3 * ri, 3 * rj + 1, 3 * ri, 3 * rj + 2, ".")
            elif dc == 1:
                fill_rect(cells, 3 * ri + 1, 3 * rj + 3, 3 * ri + 2,
                          3 * rj + 3, ".")
            else:
                fill_rect(cells, 3 * ri + 1, 3 * rj, 3 * ri + 2, 3 * rj, ".")
            visited[nr][nc] = True
            stack.append((nr, nc))
            moved = True
            break
        if not moved:
            stack.pop()
    return cells


def build_scenarios():
    scenarios = []

    # 1) 开阔：仅外围墙，对角区域两点；分辨率 0.5 覆盖米制换算。
    cells = blank(16, 16)
    scenarios.append(dict(id="open_16", width=16, height=16, resolution_m=0.5,
                          cells=cells, start=(2, 2), goal=(13, 13),
                          distance_source=(2, 2)))

    # 2) 迷宫：2x2 房间、墙厚 1、通道宽 2；左下房间 -> 右上房间。
    cells = maze_cells(25, seed=0)
    scenarios.append(dict(id="maze_25", width=25, height=25, resolution_m=1.0,
                          cells=cells, start=(1, 1), goal=(22, 22),
                          distance_source=(1, 1)))

    # 3) 窄道：两道平行墙形成 3 格宽 S 形通道，贯穿约 75% 边长。
    cells = blank(32, 32)
    fill_rect(cells, 10, 1, 10, 24)   # 上墙：cols 1..24
    fill_rect(cells, 14, 7, 14, 30)   # 下墙：cols 7..30
    scenarios.append(dict(id="corridor_32", width=32, height=32,
                          resolution_m=0.25, cells=cells, start=(4, 4),
                          goal=(27, 27), distance_source=(12, 2)))

    # 4) 死胡同：入口宽 3、袋深 14 的 U 形袋。
    cells = blank(24, 24)
    fill_rect(cells, 6, 11, 6, 15)    # 袋顶
    fill_rect(cells, 7, 11, 20, 11)   # 左壁
    fill_rect(cells, 7, 15, 20, 15)   # 右壁
    scenarios.append(dict(id="deadend_24", width=24, height=24,
                          resolution_m=1.0, cells=cells, start=(22, 4),
                          goal=(8, 13), distance_source=(22, 4)))

    # 5) 不可达：终点在封闭腔（墙厚 1、无开口）。
    cells = blank(20, 20)
    fill_rect(cells, 7, 7, 7, 13)
    fill_rect(cells, 13, 7, 13, 13)
    fill_rect(cells, 7, 7, 13, 7)
    fill_rect(cells, 7, 13, 13, 13)
    scenarios.append(dict(id="unreachable_20", width=20, height=20,
                          resolution_m=1.0, cells=cells, start=(2, 2),
                          goal=(10, 10), distance_source=(2, 2)))

    # 6) 边界斜角：沿相邻边界 3 组对角障碍对 + 贴边通道。
    cells = blank(16, 16)
    for k in range(3):
        base = 1 + 3 * k
        cells[base][base + 1] = "#"
        cells[base + 1][base] = "#"
    fill_rect(cells, 1, 13, 12, 13)   # 贴边通道（col 14 与边界之间）
    scenarios.append(dict(id="border_corner_16", width=16, height=16,
                          resolution_m=0.5, cells=cells, start=(2, 2),
                          goal=(13, 14), distance_source=(2, 2)))

    # 7) 对角缝隙（微用例）：对角目标被两个正交障碍夹住。
    cells = blank(6, 6)
    cells[2][3] = "#"
    cells[3][2] = "#"
    scenarios.append(dict(id="corner_gap_6", width=6, height=6,
                          resolution_m=1.0, cells=cells, start=(2, 2),
                          goal=(3, 3), distance_source=(2, 2)))

    # 8) unknown 阻塞：unknown 列完全隔断。
    cells = blank(16, 16)
    for row in range(1, 15):
        cells[row][8] = "?"
    scenarios.append(dict(id="unknown_barrier_16", width=16, height=16,
                          resolution_m=1.0, cells=cells, start=(8, 2),
                          goal=(8, 12), distance_source=(8, 2)))

    # 9) unknown 通道：unknown 列中留 3 格缺口（只允许从缺口通过）。
    cells = blank(16, 16)
    for row in range(1, 15):
        cells[row][8] = "?"
    fill_rect(cells, 7, 8, 9, 8, ".")
    scenarios.append(dict(id="unknown_gap_16", width=16, height=16,
                          resolution_m=1.0, cells=cells, start=(8, 2),
                          goal=(8, 12), distance_source=(8, 2)))

    # 10) 无围墙边界：整幅自由栅格，起终点在角上（边界/地图外按阻塞）。
    cells = blank(16, 16, border=False)
    scenarios.append(dict(id="edge_open_16", width=16, height=16,
                          resolution_m=2.0, cells=cells, start=(0, 0),
                          goal=(15, 15), distance_source=(0, 0)))

    for scenario in scenarios:
        assert len(scenario["cells"]) == scenario["height"]
        assert all(len(row) == scenario["width"] for row in scenario["cells"])
        for (row, col) in (scenario["start"], scenario["goal"],
                           scenario["distance_source"]):
            assert scenario["cells"][row][col] == ".", scenario["id"]
    return scenarios


# ---------------------------------------------------------------------------
# 独立参考 Dijkstra
# ---------------------------------------------------------------------------

def motion_steps(eight, corner_cut_allowed):
    steps = [(0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0)]
    if eight:
        root2 = math.sqrt(2.0)
        steps += [(-1, 1, root2), (-1, -1, root2), (1, 1, root2),
                  (1, -1, root2)]
    return steps


def dijkstra_field(scenario, neighborhood, corner_cut_allowed=False,
                   source=None):
    """返回行主序距离场（米）；不可达为 None。"""
    width = scenario["width"]
    height = scenario["height"]
    resolution = scenario["resolution_m"]
    cells = scenario["cells"]
    eight = neighborhood == "eight"
    steps = motion_steps(eight, corner_cut_allowed)
    src = source if source is not None else scenario["distance_source"]
    field = [[None] * width for _ in range(height)]
    if cells[src[0]][src[1]] != ".":
        return field
    field[src[0]][src[1]] = 0.0
    heap = [(0.0, src[0], src[1])]
    while heap:
        distance, row, col = heapq.heappop(heap)
        if distance > field[row][col]:
            continue
        for dr, dc, step_cost in steps:
            nr, nc = row + dr, col + dc
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            if cells[nr][nc] != ".":
                continue
            if dr != 0 and dc != 0 and not corner_cut_allowed:
                if cells[row][col + dc] != "." or cells[row + dr][col] != ".":
                    continue
            candidate = distance + step_cost
            if field[nr][nc] is None or candidate < field[nr][nc]:
                field[nr][nc] = candidate
                heapq.heappush(heap, (candidate, nr, nc))
    return field


def expected_for(scenario):
    goal = scenario["goal"]
    result = {"width": scenario["width"], "height": scenario["height"],
              "resolution_m": scenario["resolution_m"],
              "start": list(scenario["start"]), "goal": list(goal),
              "distance_source": list(scenario["distance_source"]),
              "neighborhoods": {}}
    for neighborhood in ("four", "eight"):
        # 点到点代价：从场景 start 出发（与 C++ 请求一致）。
        cost_field = dijkstra_field(scenario, neighborhood,
                                    source=scenario["start"])
        cost = cost_field[goal[0]][goal[1]]
        # 距离场：从场景 distance_source 出发（与 C++ 距离场请求一致）。
        field = dijkstra_field(scenario, neighborhood)
        entry = {
            "cost_m": None if cost is None else cost * scenario["resolution_m"],
            "reachable": cost is not None,
            "distance_field_m": [
                [None if value is None else value * scenario["resolution_m"]
                 for value in row]
                for row in field],
            "unreachable_cells": sum(
                1 for row in field for value in row if value is None),
            "max_distance_m": 0.0 if not any(
                value is not None for row in field for value in row) else max(
                value for row in field for value in row if value is not None)
                * scenario["resolution_m"],
        }
        result["neighborhoods"][neighborhood] = entry
    # 仅用于 jps3d JPS 单列变体原生语义核对（八邻域允许对角穿角）。
    cut_field = dijkstra_field(scenario, "eight", corner_cut_allowed=True,
                               source=scenario["start"])
    cut_cost = cut_field[goal[0]][goal[1]]
    result["neighborhoods"]["eight_corner_cut_allowed"] = {
        "cost_m": None if cut_cost is None else cut_cost
        * scenario["resolution_m"],
        "reachable": cut_cost is not None,
    }
    return result


# ---------------------------------------------------------------------------
# 序列化与校验
# ---------------------------------------------------------------------------

def scenarios_document():
    return {
        "fixture": FIXTURE_NAME,
        "fixture_version": FIXTURE_VERSION,
        "generator": GENERATOR,
        "unit": "meters (cell steps: orthogonal 1, diagonal sqrt(2), times resolution_m)",
        "scenarios": [
            {
                "id": scenario["id"],
                "width": scenario["width"],
                "height": scenario["height"],
                "resolution_m": scenario["resolution_m"],
                "cells": ["".join(row) for row in scenario["cells"]],
                "start": list(scenario["start"]),
                "goal": list(scenario["goal"]),
                "distance_source": list(scenario["distance_source"]),
            }
            for scenario in build_scenarios()
        ],
    }


def expected_document():
    return {
        "fixture": FIXTURE_NAME,
        "fixture_version": FIXTURE_VERSION,
        "generator": GENERATOR,
        "tolerance_note": "compare with 1e-8 + 1e-8 * abs(reference) (plan §6.2)",
        "scenarios": {
            scenario["id"]: expected_for(scenario)
            for scenario in build_scenarios()
        },
    }


def write_json(path, document):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2, sort_keys=True)
        handle.write("\n")


def load_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def diff_documents(actual, expected, path="", problems=None):
    """结构 + 数值比较（距离容差 1e-9，其余精确）。"""
    if problems is None:
        problems = []
    if isinstance(expected, dict):
        if not isinstance(actual, dict):
            problems.append("%s: expected object" % (path or "/"))
            return problems
        for key in expected:
            if key not in actual:
                problems.append("%s.%s: missing" % (path, key))
            else:
                diff_documents(actual[key], expected[key],
                               "%s.%s" % (path, key), problems)
        for key in actual:
            if key not in expected:
                problems.append("%s.%s: unexpected" % (path, key))
        return problems
    if isinstance(expected, list):
        if not isinstance(actual, list) or len(actual) != len(expected):
            problems.append("%s: list length mismatch" % path)
            return problems
        for index, (left, right) in enumerate(zip(actual, expected)):
            diff_documents(left, right, "%s[%d]" % (path, index), problems)
        return problems
    if isinstance(expected, float) or isinstance(actual, float):
        if expected is None or actual is None:
            if expected != actual:
                problems.append("%s: null mismatch" % path)
            return problems
        if abs(float(actual) - float(expected)) > CHECK_TOLERANCE:
            problems.append("%s: %r != %r" % (path, actual, expected))
        return problems
    if actual != expected:
        problems.append("%s: %r != %r" % (path, actual, expected))
    return problems


def check(path, document):
    frozen = load_json(path)
    problems = diff_documents(frozen, document)
    if problems:
        for problem in problems[:20]:
            sys.stderr.write("  %s\n" % problem)
        return 1
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-scenarios", metavar="PATH")
    parser.add_argument("--write-expected", metavar="PATH")
    parser.add_argument("--check-scenarios", metavar="PATH")
    parser.add_argument("--check-expected", metavar="PATH")
    parser.add_argument("--scenarios", metavar="PATH",
                        help="场景夹具路径（--write-expected 使用；默认内置生成）")
    args = parser.parse_args(argv)

    status = 0
    if args.write_scenarios:
        write_json(args.write_scenarios, scenarios_document())
        print("wrote scenarios: %s" % args.write_scenarios)
    if args.write_expected:
        if args.scenarios:
            # 由冻结的场景夹具重新构造内部场景，保证与 C++ 输入一致。
            frozen = load_json(args.scenarios)
            scenarios = []
            for item in frozen["scenarios"]:
                scenarios.append(dict(
                    id=item["id"], width=item["width"], height=item["height"],
                    resolution_m=item["resolution_m"],
                    cells=[list(row) for row in item["cells"]],
                    start=tuple(item["start"]), goal=tuple(item["goal"]),
                    distance_source=tuple(item["distance_source"])))
            document = {
                "fixture": FIXTURE_NAME,
                "fixture_version": FIXTURE_VERSION,
                "generator": GENERATOR,
                "tolerance_note": "compare with 1e-8 + 1e-8 * abs(reference) (plan §6.2)",
                "scenarios": {scenario["id"]: expected_for(scenario)
                              for scenario in scenarios},
            }
        else:
            document = expected_document()
        write_json(args.write_expected, document)
        print("wrote expected: %s" % args.write_expected)
    if args.check_scenarios:
        status |= check(args.check_scenarios, scenarios_document())
        if status == 0:
            print("scenarios fixture check OK: %s" % args.check_scenarios)
    if args.check_expected:
        if args.scenarios:
            frozen = load_json(args.scenarios)
            scenarios = []
            for item in frozen["scenarios"]:
                scenarios.append(dict(
                    id=item["id"], width=item["width"], height=item["height"],
                    resolution_m=item["resolution_m"],
                    cells=[list(row) for row in item["cells"]],
                    start=tuple(item["start"]), goal=tuple(item["goal"]),
                    distance_source=tuple(item["distance_source"])))
            document = {
                "fixture": FIXTURE_NAME,
                "fixture_version": FIXTURE_VERSION,
                "generator": GENERATOR,
                "tolerance_note": "compare with 1e-8 + 1e-8 * abs(reference) (plan §6.2)",
                "scenarios": {scenario["id"]: expected_for(scenario)
                              for scenario in scenarios},
            }
        else:
            document = expected_document()
        status |= check(args.check_expected, document)
        if status == 0:
            print("expected fixture check OK: %s" % args.check_expected)
    if not any((args.write_scenarios, args.write_expected,
                args.check_scenarios, args.check_expected)):
        document = expected_document()
        for scenario_id, entry in sorted(document["scenarios"].items()):
            costs = ", ".join(
                "%s=%s" % (name, "unreachable" if not data["reachable"]
                           else "%.6f" % data["cost_m"])
                for name, data in sorted(entry["neighborhoods"].items()))
            print("%-22s %s" % (scenario_id, costs))
    return status


if __name__ == "__main__":
    sys.exit(main())
