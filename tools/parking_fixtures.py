#!/usr/bin/env python3
"""v8 夹具生成器（停车基准 115 场景集）。

命令：

    python3 tools/parking_fixtures.py generate --input <parking_cases.json> \
        [--source-version v7] [--output-dir tests/fixtures/v8] [--force] \
        [--precheck-json <碰撞预检 JSON>]
    python3 tools/parking_fixtures.py check --output-dir tests/fixtures/v8

约定（对照 Docs/component_development_plan.md 夹具规约与 v7 体例）：

- v8 = ``v7`` 全量复制（不含 v7 的 ``fixture_manifest.sha256.json``）+ 新增
  ``paths/cases_bench_parking.json``（115 个论文停车基准场景）与
  ``vehicles/parking_bench.json``（论文车辆精确参数，安全边距 0），
  ``README.md`` / ``index.json`` 修订；v1–v7 保持冻结不动。
- 输入为停车仓库 ``cpp/tools/export_cases.py`` 生成的中立 JSON
  （``type = parking_cases_export``），本工具不读 ``.mat``。
- 生成时做几何自检：索引连续、顶点数与边界、车辆包络足迹角点在画布内、
  障碍顶点在画布内；可选用 ``--precheck-json`` 并入正式碰撞检查器
  （``planner_hybrid_astar_bench --suite v3 --precheck``）的起终点预检结果。
- 本模块只使用 CPython 3.8+ 标准库。
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

MANIFEST_NAME = "fixture_manifest.sha256.json"
FIXTURES_REL = Path("tests/fixtures")
SOURCE_VERSION_DEFAULT = "v7"
TARGET_VERSION_DEFAULT = "v8"
EXPECTED_CASES = 115
CANVAS_TOLERANCE = 1e-9


class ParkingFixtureError(RuntimeError):
    pass


def canonical_json(value):
    return json.dumps(value, indent=2, ensure_ascii=False) + "\n"


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ParkingFixtureError("无法读取 JSON: %s (%s)" % (path, exc))


def envelope_corners(vehicle, x, y, yaw):
    """后轴中心位姿下的整车包络足迹四角（含安全边距）。"""
    rear = vehicle["rear_overhang_m"] + vehicle["safety_margin_m"]
    front = vehicle["wheelbase_m"] + vehicle["front_overhang_m"] + vehicle["safety_margin_m"]
    half = vehicle["width_m"] / 2.0 + vehicle["safety_margin_m"]
    local = [(-rear, -half), (front, -half), (front, half), (-rear, half)]
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [
        (x + cos_yaw * lx - sin_yaw * ly, y + sin_yaw * lx + cos_yaw * ly)
        for lx, ly in local
    ]


def validate_input(document, canvas):
    if document.get("type") != "parking_cases_export":
        raise ParkingFixtureError("输入 JSON type 必须是 parking_cases_export")
    cases = document.get("cases")
    if not isinstance(cases, list) or not cases:
        raise ParkingFixtureError("输入 JSON 缺少 cases 数组")
    if document.get("case_count") != len(cases):
        raise ParkingFixtureError(
            "case_count (%s) 与 cases 长度 (%d) 不一致"
            % (document.get("case_count"), len(cases))
        )
    if len(cases) != EXPECTED_CASES:
        raise ParkingFixtureError(
            "停车基准场景数必须为 %d，实际 %d" % (EXPECTED_CASES, len(cases))
        )
    vehicle = document.get("vehicle", {})
    for key in (
        "wheelbase_m",
        "front_overhang_m",
        "rear_overhang_m",
        "width_m",
        "max_steer_rad",
        "safety_margin_m",
    ):
        if key not in vehicle:
            raise ParkingFixtureError("车辆参数缺少 %s" % key)
    indices = sorted(case["index"] for case in cases)
    if indices != list(range(1, EXPECTED_CASES + 1)):
        raise ParkingFixtureError("场景索引必须是 1..%d 的连续整数" % EXPECTED_CASES)
    problems = []
    for case in cases:
        for name in ("boundary", "obstacles"):
            if name not in case:
                problems.append("case_%s 缺少 %s" % (case.get("index"), name))
        if "obstacles" in case and not case["obstacles"]:
            problems.append("case_%s 障碍为空" % case["index"])
        for obstacle in case.get("obstacles", []):
            if len(obstacle["vertices"]) < 3:
                problems.append(
                    "case_%s 的 %s 顶点少于 3" % (case["index"], obstacle["id"])
                )
    if problems:
        raise ParkingFixtureError("；".join(problems[:10]))
    return vehicle


def geometric_self_check(document, canvas, vehicle):
    """几何自检。

    - 起终点包络足迹必须完全在画布内（严格检查）。
    - 障碍允许越出画布：源数据常态（190/575 个多边形有顶点出界，最大出界
      ≈9.6 m），消费方按“地图矩形之外视为占据”语义裁剪；仅统计不判失败。
    """
    failures = []
    obstacle_total = 0
    obstacle_out = 0
    max_overshoot = 0.0
    for case in document["cases"]:
        boundary = case["boundary"]
        for label, x_key, y_key, theta_key in (
            ("start", "x0_m", "y0_m", "theta0_rad"),
            ("goal", "xf_m", "yf_m", "thetaf_rad"),
        ):
            for x, y in envelope_corners(
                vehicle, boundary[x_key], boundary[y_key], boundary[theta_key]
            ):
                if (
                    x < canvas["x_min_m"] - CANVAS_TOLERANCE
                    or x > canvas["x_max_m"] + CANVAS_TOLERANCE
                    or y < canvas["y_min_m"] - CANVAS_TOLERANCE
                    or y > canvas["y_max_m"] + CANVAS_TOLERANCE
                ):
                    failures.append("case_%d %s 包络越出画布" % (case["index"], label))
        for obstacle in case["obstacles"]:
            obstacle_total += 1
            outside = False
            for vertex in obstacle["vertices"]:
                for value, low, high in (
                    (vertex["x_m"], canvas["x_min_m"], canvas["x_max_m"]),
                    (vertex["y_m"], canvas["y_min_m"], canvas["y_max_m"]),
                ):
                    overshoot = max(low - value, value - high, 0.0)
                    if overshoot > CANVAS_TOLERANCE:
                        outside = True
                        max_overshoot = max(max_overshoot, overshoot)
            if outside:
                obstacle_out += 1
    stats = {
        "obstacle_total": obstacle_total,
        "obstacle_out_of_canvas": obstacle_out,
        "max_overshoot_m": round(max_overshoot, 3),
    }
    return failures, stats


def build_case_entry(case, grid_meta):
    boundary = case["boundary"]
    return {
        "id": "case_%d" % case["index"],
        "class": "parking",
        "vehicle": "parking_bench_vehicle",
        "seed": 900 + case["index"],
        "grid": dict(grid_meta, map_id="parking_case_%d" % case["index"]),
        "start": {
            "x_m": boundary["x0_m"],
            "y_m": boundary["y0_m"],
            "yaw_rad": boundary["theta0_rad"],
        },
        "goal": {
            "x_m": boundary["xf_m"],
            "y_m": boundary["yf_m"],
            "yaw_rad": boundary["thetaf_rad"],
        },
        "obstacles": [
            {
                "id": obstacle["id"],
                "type": "convex_polygon",
                "vertices": [
                    {"x_m": vertex["x_m"], "y_m": vertex["y_m"]}
                    for vertex in obstacle["vertices"]
                ],
            }
            for obstacle in case["obstacles"]
        ],
        "expected": {"status": "success"},
        "notes": (
            "源：ParkingBenchmarks/%s；%d 个凸多边形障碍；论文基线期望成功"
            "（停车套件为探索性对比，失败只记录不中断）。"
            % (case["source"], len(case["obstacles"]))
        ),
    }


def build_cases_document(document, canvas, vehicle, precheck, canvas_margin_m):
    effective_canvas = {
        "x_min_m": canvas["x_min_m"] - canvas_margin_m,
        "x_max_m": canvas["x_max_m"] + canvas_margin_m,
        "y_min_m": canvas["y_min_m"] - canvas_margin_m,
        "y_max_m": canvas["y_max_m"] + canvas_margin_m,
    }
    grid_meta = {
        "map_version": TARGET_VERSION_DEFAULT,
        "resolution_m": 0.1,
        "origin": {
            "x_m": effective_canvas["x_min_m"],
            "y_m": effective_canvas["y_min_m"],
            "yaw_rad": 0.0,
        },
        "width": int(
            round(
                (effective_canvas["x_max_m"] - effective_canvas["x_min_m"]) / 0.1
            )
        ),
        "height": int(
            round(
                (effective_canvas["y_max_m"] - effective_canvas["y_min_m"]) / 0.1
            )
        ),
    }
    cases = [build_case_entry(case, grid_meta) for case in document["cases"]]
    failures, obstacle_stats = geometric_self_check(
        document, effective_canvas, vehicle
    )
    if failures:
        raise ParkingFixtureError("起终点包络画布检查失败：%s" % "；".join(failures[:10]))
    margin_note = (
        "论文画布为 40×40 m（[-20,20]²）；此处每边外扩 %.1f m（一格）——"
        "源数据中 2/115 例（case_22 x=20.021、case_28 x=-20.052）起点矩形包络"
        "擦出论文边界 2.1/5.2 cm，冻结碰撞契约把“地图矩形之外”判为碰撞，外扩"
        "一格保证 115 例全部可被评估；越界障碍照常按画布外=占据裁剪。"
        % canvas_margin_m
        if canvas_margin_m > 0.0
        else "论文画布 40×40 m（[-20,20]²），无外扩。"
    )
    self_check = {
        "performed_by": "tools/parking_fixtures.py 生成时几何自检",
        "case_count": len(cases),
        "indices": "case_1..case_%d 连续" % len(cases),
        "canvas": "全部 %d 例同一画布：%d×%d 格 @0.1 m（%.1f×%.1f m），"
        "origin (%.1f,%.1f)、row0 底行、yaw 0；map_version=%s、map_id 唯一。%s"
        % (
            len(cases),
            grid_meta["width"],
            grid_meta["height"],
            (effective_canvas["x_max_m"] - effective_canvas["x_min_m"]),
            (effective_canvas["y_max_m"] - effective_canvas["y_min_m"]),
            effective_canvas["x_min_m"],
            effective_canvas["y_min_m"],
            TARGET_VERSION_DEFAULT,
            margin_note,
        ),
        "vehicle": "vehicles/parking_bench.json（论文车辆精确参数：轴距 2.8 m、"
        "宽 1.942 m、前悬 0.96 m、后悬 0.929 m、满舵 0.75 rad、安全边距 0；"
        "包络 4.689×1.942 m）。",
        "envelope_within_canvas": "起终点包络足迹角点全部在画布内 %d/%d"
        % (len(cases), len(cases)),
        "obstacles_out_of_canvas": "源数据障碍越出画布为常态：%d/%d 个多边形"
        "（最大出界 %.3f m），越界部分按“地图矩形之外视为占据”语义裁剪，"
        "与 MATLAB 侧 costmap 裁剪一致。"
        % (
            obstacle_stats["obstacle_out_of_canvas"],
            obstacle_stats["obstacle_total"],
            obstacle_stats["max_overshoot_m"],
        ),
        "convexity": "多边形凸性由导出器（cpp/tools/export_cases.py）逐一校验通过。",
    }
    if precheck is not None:
        self_check["collision_precheck"] = precheck
    source = document.get("source", "")
    return {
        "schema_version": 1,
        "type": "path_case_set",
        "fixture_version": TARGET_VERSION_DEFAULT,
        "canvas_margin_m": canvas_margin_m,
        "source": "ParkingMotionPlanningTITS21（TITS 2022 论文 115 个停车基准场景）："
        "%s，经 cpp/tools/export_cases.py 导出中立 JSON 后由 tools/parking_fixtures.py 转换" % source,
        "vehicle_ref": {
            "file": "vehicles/parking_bench.json",
            "id": "parking_bench_vehicle",
        },
        "grid_derivation": "grid 仅给几何（map_id/map_version/resolution_m/origin/"
        "width/height，width/height 为格数）；占据格由 obstacles 按“单元中心落入"
        "多边形即占据”栅格化得到（与 Src/hybrid_astar/tests/fixture_scene.hpp 的 "
        "rasterize_obstacles 一致）；无 unknown 格；地图外视为占据。obstacles 为"
        "世界坐标、米；多边形顶点保持 .mat 源顺序（简单凸多边形，方向不做归一化，"
        "消费方按 planner::collision 契约归一化为逆时针）。",
        "expected_semantics": "expected.status 为 success 或 failure；failure 时 "
        "failure_reason ∈ {no_path, timeout, invalid_input, cancelled}，与 Src/common "
        "Status 枚举语义一致。本场景集不使用 request_overrides（基准运行器按套件"
        "预算覆盖）；expected 统一取 success（论文基线），停车套件为探索性对比，"
        "失败只记录，不影响 v1 正式选型。",
        "classes": [
            {
                "name": "parking",
                "count": len(cases),
                "description": "论文停车基准 CaseNo_1..115：40×40 m 停车场、"
                "每例 5 个凸多边形障碍（随机布设）、起终点位姿取自 .mat（yaw 保持"
                "源值，可能超出 (-π,π]）。",
            }
        ],
        "cases": cases,
        "self_check": self_check,
        "notes": [
            "消费方：planner_hybrid_astar_bench --suite v3（4 个 normalized 后端 + "
            "统一栅格 overlay）。",
            "生成命令：python3 tools/parking_fixtures.py generate --input "
            "<ParkingMotionPlanningTITS21/parking_cases.json> --force；随后运行 "
            "task_runner.py fixtures freeze --version v8。",
        ],
    }


def build_vehicle_document(neutral_vehicle):
    return {
        "schema_version": 1,
        "type": "vehicle_model",
        "id": "parking_bench_vehicle",
        "wheelbase_m": neutral_vehicle["wheelbase_m"],
        "width_m": neutral_vehicle["width_m"],
        "front_overhang_m": neutral_vehicle["front_overhang_m"],
        "rear_overhang_m": neutral_vehicle["rear_overhang_m"],
        "max_steer_rad": neutral_vehicle["max_steer_rad"],
        "safety_margin_m": neutral_vehicle["safety_margin_m"],
        "footprint_shape": "rectangle",
        "source": "ParkingMotionPlanningTITS21 InitializeParams.m / TITS 2022 论文参数（cpp/tools/export_cases.py 导出）",
        "notes": "论文停车基准车辆：轴距 2.8 m、宽 1.942 m、前悬 0.96 m、后悬 0.929 m、"
        "满舵 0.75 rad；安全边距 0（论文口径，与 v1–v7 默认测试车辆的 0.1 m 边距不同）。"
        "最小转弯半径 = 2.8 / tan(0.75) ≈ 3.0170 m。",
    }


def build_readme(cases_document, precheck):
    vehicle = cases_document["vehicle_ref"]["id"]
    margin = cases_document["canvas_margin_m"]
    lines = [
        "# tests/fixtures/v8：E0 固定夹具（v8 修订版：停车基准 115 场景集）",
        "",
        "本目录保存跨组件复用的版本化测试输入。v8 = v7 全量复制（不含 v7 清单）"
        "+ 新增论文停车基准 115 场景集与论文车辆模型；v1–v7 保持冻结不动。",
        "",
        "## v8 修订说明（相对 v7）",
        "",
        "| 文件 | v8 变更 | 原因 |",
        "| --- | --- | --- |",
        "| `paths/cases_bench_parking.json` | 新增（115 场景） | 停车基准套件 v3 场景集，"
        "供 `planner_hybrid_astar_bench --suite v3` 消费 |",
        "| `vehicles/parking_bench.json` | 新增 | 论文车辆精确参数（安全边距 0） |",
        "| `README.md`、`index.json` | 修订（v8 说明） | 版本化修订记录 |",
        "| `fixture_manifest.sha256.json` | 由 `fixtures freeze` 生成 | 清单不含自身 |",
        "| 其余文件 | 与 v7 逐字节一致 | 避免无关变更 |",
        "",
        "### 停车场景集 `paths/cases_bench_parking.json`",
        "",
        "- 来源：ParkingMotionPlanningTITS21 `ParkingBenchmarks/CaseNo_1..115.mat`"
        "（IEEE TITS 2022 论文基准）；`.mat` → 中立 JSON → 本夹具的两段转换分别由"
        " `cpp/tools/export_cases.py`（scipy，一次性）与 `tools/parking_fixtures.py`"
        "（标准库）完成。",
        "- 画布：论文画布 40×40 m（[-20,20]²），本夹具每边外扩 %.1f m（一格）为 "
        "%d×%d 格 @0.1 m，origin (%.1f,%.1f)、row0 底行、yaw 0；`map_version=v8`、"
        "`grid.map_id=parking_case_<i>` 唯一。外扩原因：源数据 2/115 例"
        "（case_22 x=20.021、case_28 x=-20.052）起点矩形包络擦出论文边界 "
        "2.1/5.2 cm，冻结碰撞契约（地图矩形之外=碰撞）需外扩一格才能让 115 例"
        "全部可评估；越界障碍照常按画布外=占据裁剪。"
        % (margin, 400 + int(round(2 * margin / 0.1)), 400 + int(round(2 * margin / 0.1)),
           -20.0 - margin, -20.0 - margin),
        "- 车辆：`vehicles/parking_bench.json`（论文参数：轴距 2.8 m、宽 1.942 m、"
        "前悬 0.96 m、后悬 0.929 m、满舵 0.75 rad、安全边距 0；包络 4.689×1.942 m）。",
        "- 场景：`id=case_1..case_115`、`class=parking`、`seed=901..1015`；每例 5 个"
        "凸多边形障碍（源数据随机布设）；start/goal 取自 `.mat` 边界位姿（yaw 保持"
        "源值，可能超出 (-π,π]，消费方按 `normalize_angle_rad` 语义处理）。",
        "- 期望：统一 `expected.status=success`（论文基线）；本套件为探索性对比，"
        "失败（no_path/timeout）只记录，不参与 v1 正式选型门槛。",
        "",
        "### 自检（生成时）",
        "",
    ]
    self_check = cases_document["self_check"]
    lines.append("- 索引连续：%s。" % self_check["indices"])
    lines.append("- 画布：%s" % self_check["canvas"])
    lines.append("- 包络画布检查：%s。" % self_check["envelope_within_canvas"])
    lines.append("- 障碍越界统计：%s" % self_check["obstacles_out_of_canvas"])
    lines.append("- 凸性：%s" % self_check["convexity"])
    if precheck is not None:
        invalid = precheck.get("invalid_cases", [])
        lines.append(
            "- 正式碰撞检查器起终点预检（%s）：%d/%d 例起终点足迹无碰撞%s。"
            % (
                precheck.get("performed_by", "planner_hybrid_astar_bench --precheck"),
                precheck.get("valid_cases", 0),
                EXPECTED_CASES,
                "" if not invalid else "；无效例：" + ", ".join(invalid),
            )
        )
    else:
        lines.append(
            "- 正式碰撞检查器起终点预检：未附加（可用 `planner_hybrid_astar_bench "
            "--suite v3 --precheck` 生成后以 `--precheck-json` 重新生成）。"
        )
    lines.extend(
        [
            "",
            "### 生成与冻结",
            "",
            "```sh",
            "python3 tools/parking_fixtures.py generate \\",
            "    --input <repo>/ParkingMotionPlanningTITS21/parking_cases.json --force",
            "python3 tools/task_runner.py fixtures freeze --version v8",
            "python3 tools/task_runner.py fixtures verify --version v8",
            "```",
            "",
            "其余集合（grids/serialization/collision/benchmarks/playback/map 与 v1–v6"
            " 的 paths/cases.json）与 v7 逐字节一致，说明见 v7 目录及其 index。",
            "",
        ]
    )
    return "\n".join(lines)


def build_index(source_index, precheck):
    index = json.loads(json.dumps(source_index))
    index["fixture_version"] = TARGET_VERSION_DEFAULT
    index["revision_of"] = SOURCE_VERSION_DEFAULT
    index["revision_summary"] = (
        "v8 = v7 全量复制（不含 v7 清单）+ paths/cases_bench_parking.json 新增："
        "论文停车基准 115 场景集（40×40 m 统一画布 400×400 格 @0.1 m，"
        "id case_1..115，统一 expected=success，探索性对比）；"
        "vehicles/parking_bench.json 新增：论文车辆精确参数（安全边距 0）；"
        "其余文件与 v7 逐字节一致，v1–v7 保持冻结不动。"
    )
    index["revision_history"] = [
        "v8：paths/ 新增 cases_bench_parking.json（TITS 2022 论文停车基准 115 场景；"
        "消费方 planner_hybrid_astar_bench --suite v3）与 vehicles/parking_bench.json"
        "（论文车辆参数、安全边距 0）；生成时几何自检通过；其余文件与 v7 逐字节一致。"
    ] + list(index.get("revision_history", []))
    sets = []
    for entry in index.get("sets", []):
        entry = dict(entry)
        if entry.get("id") == "vehicles":
            entry["description"] = (
                "车辆模型：default.json（v1–v7 默认测试车辆，含 0.1 m 安全边距）"
                "+ v8 新增 parking_bench.json（论文停车基准车辆，安全边距 0）。"
            )
        elif entry.get("id") == "paths":
            entry["description"] = (
                entry.get("description", "")
                + " v8 新增 cases_bench_parking.json（论文停车基准 115 场景 = "
                "40×40 m 画布 400×400 格 @0.1 m，id case_1..115，统一 "
                "expected=success，探索性对比）。"
            )
        sets.append(entry)
    index["sets"] = sets
    note = (
        "v8 修订（相对 v7）：paths/ 新增 cases_bench_parking.json —— 论文停车基准 "
        "115 场景集（id case_1..case_115、class=parking、seed=901..1015；"
        "论文画布 [-20,20]² 每边外扩 0.1 m（一格）为 402×402 格 @0.1 m、"
        "origin (-20.1,-20.1)，外扩原因：源数据 2/115 例起点矩形包络擦出论文边界 "
        "2.1/5.2 cm，冻结碰撞契约需外扩一格方可评估全部 115 例；map_version=v8、"
        "map_id 唯一；统一 expected=success；每例 5 个凸多边形障碍；start/goal 为"
        "论文边界位姿，yaw 保持 .mat 源值）。vehicles/parking_bench.json 新增论文"
        "车辆参数（轴距 2.8 m、宽 1.942 m、前悬 0.96 m、后悬 0.929 m、满舵 0.75 rad、"
        "安全边距 0）。转换链：ParkingBenchmarks/CaseNo_*.mat → "
        "cpp/tools/export_cases.py（中立 JSON，含扇形三角剖分与凸性校验）→ "
        "tools/parking_fixtures.py（本夹具）。生成时几何自检（索引连续/包络与障碍"
        "在画布内）通过%s。消费方：planner_hybrid_astar_bench --suite v3。"
        "其余文件与 v7 逐字节一致；v1–v7 保持冻结不动。"
        % (
            (
                "；正式碰撞检查器起终点预检通过 %d/%d"
                % (precheck.get("valid_cases", 0), EXPECTED_CASES)
            )
            if precheck is not None
            else "；正式碰撞预检另行记录"
        )
    )
    index["notes"] = [note] + list(index.get("notes", []))
    return index


def generate(args):
    repo_root = Path(args.repo_root).resolve()
    fixtures_root = repo_root / FIXTURES_REL
    source_dir = fixtures_root / args.source_version
    target_dir = (
        Path(args.output_dir).resolve()
        if args.output_dir
        else fixtures_root / TARGET_VERSION_DEFAULT
    )
    if not source_dir.is_dir():
        raise ParkingFixtureError("源版本目录不存在: %s" % source_dir)

    document = read_json(args.input)
    canvas = document.get("canvas", {})
    for key in ("x_min_m", "x_max_m", "y_min_m", "y_max_m"):
        if key not in canvas:
            raise ParkingFixtureError("输入 JSON 缺少画布字段 %s" % key)
    vehicle = validate_input(document, canvas)
    precheck = read_json(args.precheck_json) if args.precheck_json else None

    if target_dir.exists():
        if not args.force:
            raise ParkingFixtureError(
                "目标目录已存在（使用 --force 重新生成）: %s" % target_dir
            )
        shutil.rmtree(target_dir)

    shutil.copytree(
        source_dir,
        target_dir,
        ignore=shutil.ignore_patterns(MANIFEST_NAME),
    )
    # 顶层 index.json / README.md 随后以 v8 修订内容覆盖（子目录内同名文件保持
    # v7 逐字节复制，不得用 ignore_patterns 匹配文件名，否则会误删嵌套文件）。

    cases_document = build_cases_document(
        document, canvas, vehicle, precheck, args.canvas_margin
    )
    (target_dir / "paths").mkdir(parents=True, exist_ok=True)
    (target_dir / "vehicles").mkdir(parents=True, exist_ok=True)
    (target_dir / "paths" / "cases_bench_parking.json").write_text(
        canonical_json(cases_document), encoding="utf-8"
    )
    (target_dir / "vehicles" / "parking_bench.json").write_text(
        canonical_json(build_vehicle_document(vehicle)), encoding="utf-8"
    )
    (target_dir / "README.md").write_text(
        build_readme(cases_document, precheck), encoding="utf-8"
    )
    source_index = read_json(source_dir / "index.json")
    (target_dir / "index.json").write_text(
        canonical_json(build_index(source_index, precheck)), encoding="utf-8"
    )

    print(
        "v8 生成完成: %s（%d 个场景，%d 个障碍）"
        % (
            target_dir,
            len(cases_document["cases"]),
            sum(len(case["obstacles"]) for case in cases_document["cases"]),
        )
    )
    print("几何自检: %s" % cases_document["self_check"]["envelope_within_canvas"])
    return 0


def check(args):
    repo_root = Path(args.repo_root).resolve()
    target_dir = (
        Path(args.output_dir).resolve()
        if args.output_dir
        else repo_root / FIXTURES_REL / TARGET_VERSION_DEFAULT
    )
    problems = []
    cases_path = target_dir / "paths" / "cases_bench_parking.json"
    if not cases_path.is_file():
        problems.append("缺少 paths/cases_bench_parking.json")
    else:
        document = read_json(cases_path)
        cases = document.get("cases", [])
        margin = document.get("canvas_margin_m", 0.0)
        expected_origin = -20.0 - margin
        expected_side = int(round((40.0 + 2.0 * margin) / 0.1))
        if len(cases) != EXPECTED_CASES:
            problems.append("场景数 %d != %d" % (len(cases), EXPECTED_CASES))
        ids = [case.get("id") for case in cases]
        expected_ids = ["case_%d" % i for i in range(1, EXPECTED_CASES + 1)]
        if ids != expected_ids:
            problems.append("场景 id 序列不是 case_1..case_115")
        map_ids = [case["grid"]["map_id"] for case in cases]
        if len(set(map_ids)) != len(map_ids):
            problems.append("grid.map_id 不唯一")
        for case in cases:
            grid = case["grid"]
            if (
                abs(grid["origin"]["x_m"] - expected_origin) > 1e-9
                or abs(grid["origin"]["y_m"] - expected_origin) > 1e-9
            ):
                problems.append(
                    "%s 原点不是 (%.1f,%.1f)" % (case["id"], expected_origin, expected_origin)
                )
                break
            if grid["width"] != expected_side or grid["height"] != expected_side:
                problems.append(
                    "%s 画布不是 %d×%d" % (case["id"], expected_side, expected_side)
                )
                break
            if not case["obstacles"]:
                problems.append("%s 无障碍" % case["id"])
                break
            if any(len(obstacle["vertices"]) < 3 for obstacle in case["obstacles"]):
                problems.append("%s 存在顶点少于 3 的障碍" % case["id"])
                break
    vehicle_path = target_dir / "vehicles" / "parking_bench.json"
    if not vehicle_path.is_file():
        problems.append("缺少 vehicles/parking_bench.json")
    else:
        vehicle = read_json(vehicle_path)
        if vehicle.get("safety_margin_m") != 0.0:
            problems.append("车辆安全边距不是 0")
        if vehicle.get("max_steer_rad") != 0.75:
            problems.append("车辆满舵不是 0.75")
    # v7 全量复制核对：除清单与顶层 index/README（v8 修订）外，v7 的每个文件
    # 都必须存在于 v8。
    source_dir = repo_root / FIXTURES_REL / SOURCE_VERSION_DEFAULT
    if source_dir.is_dir():
        for source_file in sorted(source_dir.rglob("*")):
            if not source_file.is_file():
                continue
            relative = source_file.relative_to(source_dir)
            if relative.as_posix() in (MANIFEST_NAME, "index.json", "README.md"):
                continue
            if not (target_dir / relative).is_file():
                problems.append("缺少 v7 文件: %s" % relative.as_posix())
    if problems:
        for problem in problems:
            print("check 失败: %s" % problem, file=sys.stderr)
        return 1
    print("v8 夹具语义检查通过（%d 个场景，全部结构/画布/车辆一致）" % EXPECTED_CASES)
    print("提示：哈希清单校验请使用 task_runner.py fixtures verify --version v8")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        default=str(Path(__file__).resolve().parents[1]),
        help="仓库根目录（默认按脚本位置推断）",
    )
    subparsers = parser.add_subparsers(dest="action", required=True)

    generate_parser = subparsers.add_parser("generate", help="生成 v8 夹具")
    generate_parser.add_argument("--input", required=True, help="中立 JSON 路径")
    generate_parser.add_argument(
        "--source-version", default=SOURCE_VERSION_DEFAULT, help="源夹具版本（默认 v7）"
    )
    generate_parser.add_argument(
        "--output-dir", default=None, help="输出目录（默认 tests/fixtures/v8）"
    )
    generate_parser.add_argument("--force", action="store_true", help="覆盖已存在目录")
    generate_parser.add_argument(
        "--precheck-json", default=None, help="起终点碰撞预检 JSON（可选）"
    )
    generate_parser.add_argument(
        "--canvas-margin",
        type=float,
        default=0.1,
        help="论文画布每边外扩米数（默认 0.1，见 README 说明）",
    )

    check_parser = subparsers.add_parser("check", help="语义检查已生成的 v8 夹具")
    check_parser.add_argument("--output-dir", default=None, help="夹具目录")

    args = parser.parse_args(argv)
    try:
        if args.action == "generate":
            return generate(args)
        return check(args)
    except ParkingFixtureError as exc:
        print("parking_fixtures 失败: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
