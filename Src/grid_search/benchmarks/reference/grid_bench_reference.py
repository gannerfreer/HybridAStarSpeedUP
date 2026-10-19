#!/usr/bin/env python3
"""grid_bench_reference v1：X-B1 栅格基准的独立 Python 参考（仅标准库）。

职责（benchmark_tiers.md §2.2、§5；组件开发计划 §7.3）：

* 读取 grid_scenario_gen v1 生成的实例表（instances.json）与打包 uint8
  栅格文件（0=空闲、1=占据，行主序；>10 MiB 走 SHA256 寻址归档）；
* 用独立 Dijkstra（本文件实现，不复用任何 C++ 适配器）计算：
    - 点到点参考代价（从 start 出发、goal 提前终止）与可达性；
    - 完整距离场参考（从 distance_source 出发，逐单元、米制、不可达 +inf），
      以 little-endian float64 二进制写出（fields/<id>.<neighborhood>.f64）；
    - 八邻域允许对角穿角的参考代价（--corner-cut；仅 jps3d_jps 单列变体）；
* 参考代价容差口径：1e-8 + 1e-8*abs(reference)（计划 §6.2）。

用法：
  grid_bench_reference.py --instances <instances.json> --output <reference.json>
      [--fields-dir <dir>] [--neighborhoods eight,four] [--corner-cut]
      [--filter-class a,b] [--filter-scale 128] [--filter-seed 0]
      [--repo-root <dir>]

退出码：0 成功；1 参考计算失败（证据保留）；2 用法错误。
"""

import argparse
import array
import hashlib
import heapq
import json
import math
import os
import sys
import time

REFERENCE_NAME = "grid_bench_reference.py v1"
TOLERANCE_NOTE = "1e-8 + 1e-8*abs(reference)"
ROOT2 = math.sqrt(2.0)

FOUR_STEPS = ((0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0))
EIGHT_STEPS = FOUR_STEPS + (
    (-1, 1, ROOT2), (-1, -1, ROOT2), (1, 1, ROOT2), (1, -1, ROOT2))


def dijkstra(cells, width, height, source, eight, corner_cut_allowed=False,
             goal=None):
    """独立 Dijkstra；返回 array('d') 距离（单元步长；不可达 +inf）。

    goal 非空时在弹出 goal 后提前终止（点到点参考）。
    """
    distances = array.array("d", [math.inf]) * (width * height)
    source_row, source_col = source
    distances[source_row * width + source_col] = 0.0
    heap = [(0.0, source_row, source_col)]
    steps = EIGHT_STEPS if eight else FOUR_STEPS
    while heap:
        distance, row, col = heapq.heappop(heap)
        index = row * width + col
        if distance > distances[index]:
            continue
        if goal is not None and (row, col) == goal:
            break
        for d_row, d_col, step_cost in steps:
            n_row = row + d_row
            n_col = col + d_col
            if n_row < 0 or n_row >= height or n_col < 0 or n_col >= width:
                continue
            n_index = n_row * width + n_col
            if cells[n_index] != 0:
                continue
            if d_row != 0 and d_col != 0 and not corner_cut_allowed:
                if cells[row * width + n_col] != 0:
                    continue
                if cells[n_row * width + col] != 0:
                    continue
            candidate = distance + step_cost
            if candidate < distances[n_index]:
                distances[n_index] = candidate
                heapq.heappush(heap, (candidate, n_row, n_col))
    return distances


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_cells_path(entry, instances_dir, repo_root):
    cells = entry["cells"]
    if cells.get("archived", False):
        return os.path.join(repo_root, cells["archive_relative_path"])
    return os.path.join(instances_dir, cells["local_path"])


def field_stats(distances_cells, resolution):
    unreachable = 0
    reachable = 0
    maximum = 0.0
    for value in distances_cells:
        if math.isinf(value):
            unreachable += 1
        else:
            reachable += 1
            if value > maximum:
                maximum = value
    return {
        "unreachable_cells": unreachable,
        "reachable_cells": reachable,
        "max_distance_m": maximum * resolution,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instances", required=True,
                        help="grid_scenario_gen v1 的 instances.json")
    parser.add_argument("--output", required=True, help="reference.json 路径")
    parser.add_argument("--fields-dir",
                        help="逐单元参考场目录（默认 <output 目录>/fields）")
    parser.add_argument("--neighborhoods", default="eight,four")
    parser.add_argument("--corner-cut", action="store_true",
                        help="额外计算八邻域允许对角穿角的参考代价")
    parser.add_argument("--filter-class", default="")
    parser.add_argument("--filter-scale", default="")
    parser.add_argument("--filter-seed", default="")
    parser.add_argument("--repo-root", default=os.getcwd())
    args = parser.parse_args(argv)

    output_path = os.path.abspath(args.output)
    output_dir = os.path.dirname(output_path)
    fields_dir = args.fields_dir or os.path.join(output_dir, "fields")
    os.makedirs(fields_dir, exist_ok=True)

    with open(args.instances, "r", encoding="utf-8") as handle:
        document = json.load(handle)
    instances_dir = os.path.dirname(os.path.abspath(args.instances))

    filter_classes = [item for item in args.filter_class.split(",") if item]
    filter_scales = [item for item in args.filter_scale.split(",") if item]
    filter_seeds = [item for item in args.filter_seed.split(",") if item]
    neighborhoods = [item for item in args.neighborhoods.split(",") if item]
    for name in neighborhoods:
        if name not in ("eight", "four"):
            parser.error("unknown neighborhood: %s" % name)

    references = {}
    started = time.time()
    for entry in document["instances"]:
        instance_id = entry["instance_id"]
        if filter_classes and entry["class"] not in filter_classes:
            continue
        if filter_scales and str(entry["scale"]) not in filter_scales:
            continue
        if filter_seeds and str(entry["seed"]) not in filter_seeds:
            continue
        cells_path = resolve_cells_path(entry, instances_dir, args.repo_root)
        with open(cells_path, "rb") as handle:
            packed = handle.read()
        if len(packed) != entry["width"] * entry["height"]:
            sys.stderr.write("size mismatch for %s: %s\n"
                             % (instance_id, cells_path))
            return 1
        actual_sha = hashlib.sha256(packed).hexdigest()
        if actual_sha != entry["cells"]["sha256"]:
            sys.stderr.write("SHA256 mismatch for %s: %s\n"
                             % (instance_id, cells_path))
            return 1
        width = entry["width"]
        height = entry["height"]
        resolution = entry["resolution_m"]
        start = tuple(entry["start"])
        goal = tuple(entry["goal"])
        source = tuple(entry["distance_source"])
        instance_reference = {
            "width": width,
            "height": height,
            "resolution_m": resolution,
            "cells_sha256": actual_sha,
        }
        for neighborhood in neighborhoods:
            eight = neighborhood == "eight"
            t0 = time.time()
            cost_field = dijkstra(packed, width, height, start, eight,
                                  goal=goal)
            goal_cost = cost_field[goal[0] * width + goal[1]]
            cost_m = None if math.isinf(goal_cost) else goal_cost * resolution
            field_cells = dijkstra(packed, width, height, source, eight)
            stats = field_stats(field_cells, resolution)
            field_name = "%s.%s.f64" % (instance_id, neighborhood)
            field_path = os.path.join(fields_dir, field_name)
            meters = array.array(
                "d", (value * resolution for value in field_cells))
            with open(field_path, "wb") as handle:
                meters.tofile(handle)
            instance_reference[neighborhood] = {
                "cost_m": cost_m,
                "reachable": cost_m is not None,
                "unreachable_cells": stats["unreachable_cells"],
                "reachable_cells": stats["reachable_cells"],
                "max_distance_m": stats["max_distance_m"],
                "field_path": os.path.relpath(field_path, output_dir),
                "field_sha256": sha256_file(field_path),
            }
            elapsed = time.time() - t0
            sys.stdout.write(
                "reference %-26s %-5s cost=%-12s unreachable=%-9d %.2fs\n"
                % (instance_id, neighborhood,
                   "unreachable" if cost_m is None else "%.6f" % cost_m,
                   stats["unreachable_cells"], elapsed))
            sys.stdout.flush()
        if args.corner_cut:
            t0 = time.time()
            cut_field = dijkstra(packed, width, height, start, True,
                                 corner_cut_allowed=True, goal=goal)
            cut_cost = cut_field[goal[0] * width + goal[1]]
            instance_reference["eight_corner_cut_allowed"] = {
                "cost_m": None if math.isinf(cut_cost)
                else cut_cost * resolution,
                "reachable": not math.isinf(cut_cost),
            }
            sys.stdout.write("reference %-26s %-5s cost=%-12s %.2fs\n"
                             % (instance_id, "cut",
                                "unreachable" if math.isinf(cut_cost)
                                else "%.6f" % (cut_cost * resolution),
                                time.time() - t0))
        references[instance_id] = instance_reference

    output = {
        "type": "grid_bench_reference",
        "schema_version": 1,
        "reference": REFERENCE_NAME,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "run_id": os.environ.get("PLANNER_RUN_ID", "unmonitored"),
        "tolerance_note": TOLERANCE_NOTE,
        "instances_path": os.path.abspath(args.instances),
        "neighborhoods": neighborhoods,
        "corner_cut_allowed_eight": bool(args.corner_cut),
        "field_encoding": "little-endian float64, row-major, meters, "
                          "+inf = unreachable",
        "instances": references,
    }
    with open(output_path, "w", encoding="utf-8") as handle:
        json.dump(output, handle, indent=2, sort_keys=True)
        handle.write("\n")
    sys.stdout.write("reference written: %s (%d instances, %.1fs)\n"
                     % (output_path, len(references), time.time() - started))
    return 0


if __name__ == "__main__":
    sys.exit(main())
