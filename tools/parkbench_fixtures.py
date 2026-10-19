#!/usr/bin/env python3
"""v9 停车夹具生成器（ParkBench 51 个受限倒车入库场景）。

命令：

    python3 tools/parkbench_fixtures.py generate \
        [--source-zip artifacts/data/sha256/<sha>/Constrained_Parking_Scenarios-main.zip] \
        [--expected-sha256 <sha|"">] [--source-version v8] \
        [--output-dir tests/fixtures/v9] [--force]
    python3 tools/parkbench_fixtures.py check [--output-dir tests/fixtures/v9]

约定（对照 v8 体例）：

- v9 = ``v8`` 全量复制（不含 v8 的 ``fixture_manifest.sha256.json``）+ 新增
  ``paths/cases_bench_parkbench.json``（ParkBench 51 场景）与
  ``vehicles/parkbench_vehicle.json``（论文车辆 4.95×2.00 m、轴距 3.0 m、
  满舵 32°，后轴中心参考的自行车模型）。
- 源数据：``Constrained_Parking_Scenarios``（CC-BY-NC-4.0，学术/非商用），按
  仓库 SHA256 寻址归档；zip 内 ``rear_in/*.json``。
- 清洗（逐例计数写入场景文件 notes 与夹具 README）：
  1. 离群段过滤：到起/终点最近距离（两端点取小）≥50 m 的线段丢弃；
  2. 目标框/起点框裁剪：论文车辆足迹外扩 ``BOX_CLEARANCE_M`` = 0.10 m 的矩形
     做 Liang-Barsky 裁剪，框内部分剔除（对齐官方 utils.py“滤除目标框内障碍点”
     的语义，并扩展到起点框；未裁剪时的冲突清单见夹具 README）。余量取 0.10 m
     是为覆盖 0.1 m 栅格化量化：墙体单元中心可偏离墙中线 ±0.025 m、单元角点
     再延伸半对角 0.0707 m，总伸入 ≤0.096 m——0.05 m 余量实测会让 3 例
     （pb-016/028/029）的目标足迹与栅格单元相交，故提高到 0.10 m；
  3. 剩余片段 → 薄凸四边形（默认厚 0.05 m，逆时针）。
- 本模块只使用 CPython 3.8+ 标准库。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import sys
import zipfile
from pathlib import Path

MANIFEST_NAME = "fixture_manifest.sha256.json"
FIXTURES_REL = Path("tests/fixtures")
SOURCE_VERSION_DEFAULT = "v8"
TARGET_VERSION_DEFAULT = "v9"
EXPECTED_CASES = 51
PARKBENCH_SHA256 = "5435c67b096b66c7cbf2d7106a2b7c34003b4c774d1c76c041a118a56d542659"
ZIP_NAME = "Constrained_Parking_Scenarios-main.zip"
ZIP_INNER_ROOT = "Constrained_Parking_Scenarios-main/rear_in/"
BOX_CLEARANCE_M = 0.10
WALL_THICKNESS_M = 0.05
OUTLIER_RADIUS_M = 50.0
CANVAS_MARGIN_M = 1.0
RESOLUTION_M = 0.1
MIN_FRAGMENT_M = 1e-3
T_PARALLEL_EPS = 1e-9

# ParkBench 论文车辆（Appendix B Table 2；后轴中心参考）。
VEHICLE = {
    "wheelbase_m": 3.0,
    "width_m": 2.0,
    "front_overhang_m": 0.925,
    "rear_overhang_m": 1.025,
    "max_steer_rad": 32.0 * math.pi / 180.0,
    "safety_margin_m": 0.0,
}


class ParkbenchFixtureError(RuntimeError):
    pass


def canonical_json(value):
    return json.dumps(value, indent=2, ensure_ascii=False) + "\n"


def footprint_local_box(vehicle, clearance=0.0):
    """后轴坐标系下的足迹矩形 (xmin, xmax, ymin, ymax)。"""
    return (
        -(vehicle["rear_overhang_m"] + vehicle["safety_margin_m"] + clearance),
        vehicle["wheelbase_m"] + vehicle["front_overhang_m"]
        + vehicle["safety_margin_m"] + clearance,
        -(vehicle["width_m"] / 2.0 + vehicle["safety_margin_m"] + clearance),
        vehicle["width_m"] / 2.0 + vehicle["safety_margin_m"] + clearance,
    )


def to_local(point, x, y, theta):
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    dx, dy = point[0] - x, point[1] - y
    return (cos_t * dx + sin_t * dy, -sin_t * dx + cos_t * dy)


def to_world(point, x, y, theta):
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return (x + cos_t * point[0] - sin_t * point[1],
            y + sin_t * point[0] + cos_t * point[1])


def footprint_corners(pose, vehicle):
    xmin, xmax, ymin, ymax = footprint_local_box(vehicle)
    local = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
    return [to_world(p, pose[0], pose[1], pose[2]) for p in local]


def wall_quad(a, b, thickness=WALL_THICKNESS_M):
    """线段 → 薄凸四边形（逆时针）；过短返回 None。"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length < MIN_FRAGMENT_M:
        return None
    nx, ny = -dy / length * thickness / 2.0, dx / length * thickness / 2.0
    quad = [
        (a[0] + nx, a[1] + ny),
        (b[0] + nx, b[1] + ny),
        (b[0] - nx, b[1] - ny),
        (a[0] - nx, a[1] - ny),
    ]
    signed = sum(
        quad[i][0] * quad[(i + 1) % 4][1] - quad[(i + 1) % 4][0] * quad[i][1]
        for i in range(4)
    )
    if signed < 0:
        quad.reverse()
    return quad


def clip_segment(a, b, x, y, theta, box):
    """Liang-Barsky：返回 (框外片段列表, 状态)。

    状态：0 = 完全在框外（原样返回）；1 = 部分裁剪（1~2 段）；
    2 = 完全在框内（空列表）。
    """
    la = to_local(a, x, y, theta)
    lb = to_local(b, x, y, theta)
    xmin, xmax, ymin, ymax = box
    t0, t1 = 0.0, 1.0
    dx, dy = lb[0] - la[0], lb[1] - la[1]
    for p, q in ((-dx, la[0] - xmin), (dx, xmax - la[0]),
                 (-dy, la[1] - ymin), (dy, ymax - la[1])):
        if abs(p) < T_PARALLEL_EPS:
            if q < 0:
                return [(a, b)], 0
            continue
        r = q / p
        if p < 0:
            if r > t1:
                return [(a, b)], 0
            if r > t0:
                t0 = r
        else:
            if r < t0:
                return [(a, b)], 0
            if r < t1:
                t1 = r
    if t1 - t0 <= 1e-12:
        return [(a, b)], 0
    if t0 <= T_PARALLEL_EPS and t1 >= 1.0 - T_PARALLEL_EPS:
        return [], 2

    def point(t):
        return to_world((la[0] + t * dx, la[1] + t * dy), x, y, theta)

    pieces = []
    if t0 > T_PARALLEL_EPS:
        pieces.append((a, point(t0)))
    if t1 < 1.0 - T_PARALLEL_EPS:
        pieces.append((point(t1), b))
    return pieces, 1


def convex_polygons_overlap(first, second):
    """凸多边形相交测试（分离轴定理，含接触即算相交）。"""
    for polygon in (first, second):
        count = len(polygon)
        for i in range(count):
            ax, ay = polygon[i]
            bx, by = polygon[(i + 1) % count]
            nx, ny = -(by - ay), bx - ax
            min_a = min_b = float("inf")
            max_a = max_b = float("-inf")
            for px, py in first:
                value = nx * (px - ax) + ny * (py - ay)
                min_a = min(min_a, value)
                max_a = max(max_a, value)
            for px, py in second:
                value = nx * (px - ax) + ny * (py - ay)
                min_b = min(min_b, value)
                max_b = max(max_b, value)
            if max_a < min_b - 1e-9 or max_b < min_a - 1e-9:
                return False
    return True


def parse_case(document, name):
    """解析单个 ParkBench 场景：返回 (start, target, segments)。"""
    frames = document.get("Frames")
    if not isinstance(frames, dict) or not frames:
        raise ParkbenchFixtureError("%s 缺 Frames" % name)
    frame = frames.get("0")
    if frame is None:
        if len(frames) != 1:
            raise ParkbenchFixtureError("%s 的 Frames 键不是 '0'" % name)
        frame = next(iter(frames.values()))
    request = frame["PlanningRequest"]
    start = [float(v) for v in request["m_startPosture"]["m_pose"][:3]]
    area = request.get("m_targetArea")
    if area is None:
        areas = request["m_targetAreas"]
        target_pose = areas["m_targetPosture"][0]["m_pose"]
    else:
        target_pose = area["m_targetPosture"]["m_pose"]
    target = [float(v) for v in target_pose[:3]]
    segments = []
    for obj in frame["NfmAggregatedPolygonObjects"]:
        nodes = obj.get("nfmPolygonObjectNodes", [])
        for first, second in zip(nodes[:-1], nodes[1:]):
            segments.append(
                ((float(first["m_x"]), float(first["m_y"])),
                 (float(second["m_x"]), float(second["m_y"])))
            )
    if not segments:
        raise ParkbenchFixtureError("%s 没有任何线段" % name)
    return start, target, segments


def clean_case(start, target, segments, vehicle):
    """按文档约定的三步清洗；返回 (quads, stats)。"""
    stats = {"segments": len(segments), "outliers": 0,
             "target_removed": 0, "target_partial": 0,
             "start_removed": 0, "start_partial": 0, "quads": 0}
    target_box = footprint_local_box(vehicle, BOX_CLEARANCE_M)
    start_box = footprint_local_box(vehicle, BOX_CLEARANCE_M)
    kept = []
    for a, b in segments:
        nearest = min(
            math.hypot(a[0] - start[0], a[1] - start[1]),
            math.hypot(a[0] - target[0], a[1] - target[1]),
            math.hypot(b[0] - start[0], b[1] - start[1]),
            math.hypot(b[0] - target[0], b[1] - target[1]),
        )
        if nearest >= OUTLIER_RADIUS_M:
            stats["outliers"] += 1
            continue
        pieces, status = clip_segment(a, b, target[0], target[1], target[2],
                                      target_box)
        if status == 2:
            stats["target_removed"] += 1
            continue
        if status == 1:
            stats["target_partial"] += 1
        surviving = []
        for c, d in pieces:
            sub, sub_status = clip_segment(c, d, start[0], start[1], start[2],
                                           start_box)
            if sub_status == 2:
                stats["start_removed"] += 1
                continue
            if sub_status == 1:
                stats["start_partial"] += 1
            surviving.extend(sub)
        kept.extend(surviving)
    quads = []
    for a, b in kept:
        quad = wall_quad(a, b)
        if quad is not None:
            quads.append(quad)
    stats["quads"] = len(quads)
    return quads, stats


def canvas_for(quads, start, target, vehicle):
    """稳健画布：墙体 + 起终点足迹框 + 1 m 边距，按 0.1 m 向外取整。"""
    points = [p for quad in quads for p in quad]
    points.extend(footprint_corners(start, vehicle))
    points.extend(footprint_corners(target, vehicle))
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    xmin = math.floor((min(xs) - CANVAS_MARGIN_M) / RESOLUTION_M) * RESOLUTION_M
    ymin = math.floor((min(ys) - CANVAS_MARGIN_M) / RESOLUTION_M) * RESOLUTION_M
    xmax = math.ceil((max(xs) + CANVAS_MARGIN_M) / RESOLUTION_M) * RESOLUTION_M
    ymax = math.ceil((max(ys) + CANVAS_MARGIN_M) / RESOLUTION_M) * RESOLUTION_M
    width = int(round((xmax - xmin) / RESOLUTION_M))
    height = int(round((ymax - ymin) / RESOLUTION_M))
    return xmin, ymin, width, height


def build_case_entry(index, source_name, start, target, quads, stats,
                     canvas):
    xmin, ymin, width, height = canvas
    identifier = "pb-%03d" % index
    return {
        "id": identifier,
        "class": "parkbench",
        "vehicle": "parkbench_vehicle",
        "seed": 1200 + index,
        "grid": {
            "map_id": "pb_%03d" % index,
            "map_version": TARGET_VERSION_DEFAULT,
            "resolution_m": RESOLUTION_M,
            "origin": {"x_m": xmin, "y_m": ymin, "yaw_rad": 0.0},
            "width": width,
            "height": height,
        },
        "start": {"x_m": start[0], "y_m": start[1], "yaw_rad": start[2]},
        "goal": {"x_m": target[0], "y_m": target[1], "yaw_rad": target[2]},
        "obstacles": [
            {
                "id": "wall_%03d" % quad_index,
                "type": "convex_polygon",
                "vertices": [{"x_m": p[0], "y_m": p[1]} for p in quad],
            }
            for quad_index, quad in enumerate(quads)
        ],
        "expected": {"status": "success"},
        "notes": (
            "源：rear_in/%s；线段 %d → 四边形 %d（离群丢弃 %d；目标框整段移除 %d/"
            "部分裁剪 %d；起点框整段移除 %d/部分裁剪 %d）。薄墙厚 %.2f m，"
            "裁剪含 %.2f m 外扩余量；未裁剪版冲突清单见夹具 README。"
            % (source_name, stats["segments"], stats["quads"], stats["outliers"],
               stats["target_removed"], stats["target_partial"],
               stats["start_removed"], stats["start_partial"],
               WALL_THICKNESS_M, BOX_CLEARANCE_M)
        ),
    }


def build_cases_document(cases, audit, vehicle, precheck=None):
    self_check = {
        "performed_by": "tools/parkbench_fixtures.py 生成时自检",
        "case_count": len(cases),
        "source_sha256": PARKBENCH_SHA256,
        "cleaning_audit": audit,
        "wall_thickness_m": WALL_THICKNESS_M,
        "box_clearance_m": BOX_CLEARANCE_M,
        "outlier_radius_m": OUTLIER_RADIUS_M,
        "postcheck": "生成后逐例断言：四边形凸且逆时针、面积>0、与起终点足迹"
        "框（无余量）不相交、画布包含全部几何。",
    }
    if precheck is not None:
        self_check["collision_precheck"] = precheck
    return {
        "schema_version": 1,
        "type": "path_case_set",
        "fixture_version": TARGET_VERSION_DEFAULT,
        "canvas_margin_m": CANVAS_MARGIN_M,
        "source": "Constrained_Parking_Scenarios（ParkBench，arXiv:2601.22545，"
        "CC-BY-NC-4.0）：rear_in/*.json 51 例；归档 "
        "artifacts/data/sha256/%s/%s；经 tools/parkbench_fixtures.py 转换"
        % (PARKBENCH_SHA256, ZIP_NAME),
        "vehicle_ref": {
            "file": "vehicles/parkbench_vehicle.json",
            "id": "parkbench_vehicle",
        },
        "grid_derivation": "grid 仅给几何（map_id/map_version/resolution_m/origin/"
        "width/height，width/height 为格数）；占据格由 obstacles 按“单元中心落入"
        "多边形即占据”栅格化得到；无 unknown 格；地图外视为占据。obstacles 为"
        "世界坐标、米；每个薄凸四边形对应源数据的一条边界线段（厚 %.2f m、"
        "逆时针；消费方按 planner::collision 契约处理）。" % WALL_THICKNESS_M,
        "expected_semantics": "expected.status 为 success 或 failure；本场景集"
        "统一取 success（ParkBench 为原论文可达基准），探索性对比下失败只记录、"
        "不影响 v1 正式选型；不使用 request_overrides。",
        "classes": [
            {
                "name": "parkbench",
                "count": len(cases),
                "description": "ParkBench 受限倒车入库 51 例：细线段墙体/车位分隔线；"
                "论文车辆 4.95×2.00 m（轴距 3.0 m、满舵 32°、后轴中心参考）；"
                "目标/起点足迹框内线段已按官方 utils.py 语义裁剪。",
            }
        ],
        "cases": cases,
        "self_check": self_check,
        "notes": [
            "消费方：planner_hybrid_astar_bench --suite v4（4 个 normalized 后端 + "
            "统一栅格 overlay）。",
            "生成命令：python3 tools/parkbench_fixtures.py generate --force；随后 "
            "task_runner.py fixtures freeze --version v9。",
        ],
    }


def build_vehicle_document():
    return {
        "schema_version": 1,
        "type": "vehicle_model",
        "id": "parkbench_vehicle",
        "wheelbase_m": VEHICLE["wheelbase_m"],
        "width_m": VEHICLE["width_m"],
        "front_overhang_m": VEHICLE["front_overhang_m"],
        "rear_overhang_m": VEHICLE["rear_overhang_m"],
        "max_steer_rad": VEHICLE["max_steer_rad"],
        "safety_margin_m": VEHICLE["safety_margin_m"],
        "footprint_shape": "rectangle",
        "source": "ParkBench 论文（arXiv:2601.22545）Appendix B Table 2",
        "notes": "中大型乘用车：长 4.95 m、宽 2.00 m、轴距 3.00 m、前悬 0.925 m、"
        "后悬 1.025 m、最大前轮等效转角 ±32°（0.5585 rad）、安全边距 0（论文口径）。"
        "最小转弯半径 = 3.0 / tan(32°) ≈ 4.801 m。运动学自行车模型、后轴中心参考。",
    }


def build_readme(audit, cases_document):
    totals = audit["totals"]
    lines = [
        "# tests/fixtures/v9：E0 固定夹具（v9 修订版：ParkBench 51 场景集）",
        "",
        "本目录保存跨组件复用的版本化测试输入。v9 = v8 全量复制（不含 v8 清单）"
        "+ 新增 ParkBench 受限倒车入库 51 场景与论文车辆模型；v1–v8 保持冻结不动。",
        "",
        "## v9 修订说明（相对 v8）",
        "",
        "| 文件 | v9 变更 | 原因 |",
        "| --- | --- | --- |",
        "| `paths/cases_bench_parkbench.json` | 新增（51 场景） | 停车基准套件 v4 消费 |",
        "| `vehicles/parkbench_vehicle.json` | 新增 | ParkBench 论文车辆（4.95×2.00 m） |",
        "| `README.md`、`index.json` | 修订（v9 说明） | 版本化修订记录 |",
        "| `fixture_manifest.sha256.json` | 由 `fixtures freeze` 生成 | 清单不含自身 |",
        "| 其余文件 | 与 v8 逐字节一致 | 避免无关变更 |",
        "",
        "## ParkBench 场景集（来源、许可与引用）",
        "",
        "- 来源仓库：https://github.com/dqm5rtfg9b-collab/Constrained_Parking_Scenarios"
        "（commit `20b94635bb134918651fe0e5e117cd94a5b81037`，2026-02-03）；",
        "  原始 zip 按仓库 SHA256 寻址归档：`artifacts/data/sha256/%s/%s`"
        "（含 `.sha256` 与 `.meta.json`）。" % (PARKBENCH_SHA256, ZIP_NAME),
        "- **许可：CC-BY-NC-4.0（数据集，仅学术/非商用）**，代码 MIT；引用：",
        "  Feng Tao et al., “Adapting Reinforcement Learning for Path Planning in",
        "  Constrained Parking Scenarios”, arXiv:2601.22545, 2026。",
        "- 场景：`rear_in/*.json` 51 例（受限倒车入库）；障碍为“细折线段”式墙体/"
        "车位分隔线；起终点为后轴中心位姿（容差 lateral/longitudinal 0.05 m、"
        "orientation 0.01 rad）。",
        "",
        "## 转换与清洗（tools/parkbench_fixtures.py，逐例计数见场景文件 notes）",
        "",
        "1. 离群段过滤：到起/终点最近距离（两端点取小）≥ %0.0f m 的线段丢弃"
        "（源数据含 km 级离群点）。" % OUTLIER_RADIUS_M,
        "2. 目标框/起点框裁剪（对齐官方 `utils.py` “滤除目标框内障碍点”语义并"
        "扩展到起点框）：以论文车辆足迹**外扩 %0.2f m** 的矩形做 Liang-Barsky"
        "裁剪，框内部分剔除（保证起终点足迹与墙体不接触、不重叠）。" % BOX_CLEARANCE_M,
        "3. 剩余片段 → 薄凸四边形（厚 %0.2f m、逆时针）；每例独立画布"
        "（0.1 m 分辨率、1.0 m 边距）。" % WALL_THICKNESS_M,
        "",
        "清洗审计（总计）：线段 %d → 四边形 %d；离群丢弃 %d（%d 例）；目标框"
        "整段移除 %d / 部分裁剪 %d（%d 例）；起点框整段移除 %d / 部分裁剪 %d"
        "（%d 例）。上述即源数据与“起终点位姿合法”约束之间的全部冲突，逐例"
        "明细见下表。"
        % (totals["segments"], totals["quads"], totals["outliers"],
           sum(1 for row in audit["per_case"] if row["outliers"] > 0),
           totals["target_removed"], totals["target_partial"],
           sum(1 for row in audit["per_case"]
               if row["target_removed"] + row["target_partial"] > 0),
           totals["start_removed"], totals["start_partial"],
           sum(1 for row in audit["per_case"]
               if row["start_removed"] + row["start_partial"] > 0)),
        "",
        "## 逐例审计",
        "",
        "| case | 源文件 | 线段 | 离群 | 目标框移除/裁剪 | 起点框移除/裁剪 | 四边形 | 画布(m) |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in audit["per_case"]:
        lines.append(
            "| %s | `%s` | %d | %d | %d/%d | %d/%d | %d | %.1f×%.1f |"
            % (row["id"], row["source"], row["segments"], row["outliers"],
               row["target_removed"], row["target_partial"],
               row["start_removed"], row["start_partial"], row["quads"],
               row["canvas_w"] * RESOLUTION_M, row["canvas_h"] * RESOLUTION_M)
        )
    precheck = cases_document["self_check"].get("collision_precheck")
    lines.extend(["", "## 碰撞预检", ""])
    if precheck is not None:
        lines.append(
            "- 正式碰撞检查器起终点预检（%s）：%d/%d 例有效%s。"
            % (precheck.get("performed_by", "planner_hybrid_astar_bench --precheck"),
               precheck.get("valid_cases", 0), EXPECTED_CASES,
               "" if not precheck.get("invalid_cases")
               else "；无效例：" + ", ".join(precheck["invalid_cases"]))
        )
    else:
        lines.append(
            "- 未附加（可用 `planner_hybrid_astar_bench --suite v4 --precheck` "
            "生成后以 `--precheck-json` 重新生成）。"
        )
    lines.extend([
        "",
        "## 车辆模型",
        "",
        "`vehicles/parkbench_vehicle.json`（ParkBench 论文 Appendix B Table 2）："
        "长 4.95 m、宽 2.00 m、轴距 3.00 m、前悬 0.925 m、后悬 1.025 m、"
        "满舵 32°（0.5585 rad）、安全边距 0；最小转弯半径 ≈ 4.801 m；"
        "运动学自行车模型、后轴中心参考。",
        "",
        "## 生成与冻结",
        "",
        "```sh",
        "python3 tools/parkbench_fixtures.py generate --force",
        "python3 tools/parkbench_fixtures.py check",
        "python3 tools/task_runner.py fixtures freeze --version v9",
        "python3 tools/task_runner.py fixtures verify --version v9",
        "```",
        "",
        "消费方：`planner_hybrid_astar_bench --suite v4`（见 tools/README.md §7.5）。",
        "其余集合与 v8 逐字节一致，说明见 v8 目录。",
        "",
    ])
    return "\n".join(lines)


def build_index(source_index):
    index = json.loads(json.dumps(source_index))
    index["fixture_version"] = TARGET_VERSION_DEFAULT
    index["revision_of"] = SOURCE_VERSION_DEFAULT
    index["revision_summary"] = (
        "v9 = v8 全量复制（不含 v8 清单）+ paths/cases_bench_parkbench.json 新增："
        "ParkBench 受限倒车入库 51 场景（细线段墙体→薄凸四边形，目标/起点框裁剪）"
        "+ vehicles/parkbench_vehicle.json 新增：ParkBench 论文车辆（4.95×2.00 m、"
        "轴距 3.0 m、满舵 32°）；其余文件与 v8 逐字节一致，v1–v8 保持冻结不动。"
    )
    index["revision_history"] = [
        "v9：paths/ 新增 cases_bench_parkbench.json（ParkBench 51 场景，"
        "CC-BY-NC-4.0，源 zip SHA256 %s，见 README 审计表）与 "
        "vehicles/parkbench_vehicle.json（论文车辆）；消费方 suite v4；"
        "其余文件与 v8 逐字节一致。" % PARKBENCH_SHA256
    ] + list(index.get("revision_history", []))
    sets = []
    for entry in index.get("sets", []):
        entry = dict(entry)
        if entry.get("id") == "vehicles":
            entry["description"] = (
                "车辆模型：default.json（0.1 m 边距基准车）、parking_bench.json"
                "（TITS21 论文车、边距 0）、v9 新增 parkbench_vehicle.json"
                "（ParkBench 论文车 4.95×2.00 m、边距 0）。"
            )
        elif entry.get("id") == "paths":
            entry["description"] = (
                entry.get("description", "")
                + " v9 新增 cases_bench_parkbench.json（ParkBench 受限倒车入库 "
                "51 场景；细线段墙体→薄凸四边形、目标/起点框裁剪；论文车辆）。"
            )
        sets.append(entry)
    index["sets"] = sets
    index["notes"] = [
        "v9 修订（相对 v8）：paths/ 新增 cases_bench_parkbench.json —— ParkBench"
        "（arXiv:2601.22545，CC-BY-NC-4.0）51 个受限倒车入库场景；障碍为细线段"
        "墙体，经离群过滤与目标/起点框裁剪后转为厚 0.05 m 薄凸四边形；每例独立"
        "画布（0.1 m）；vehicles/ 新增论文车辆（4.95×2.00 m、轴距 3.0 m、满舵 32°）。"
        "源 zip（SHA256 %s）归档于 artifacts/data/sha256/。消费方：suite v4。"
        "其余文件与 v8 逐字节一致；v1–v8 保持冻结不动。" % PARKBENCH_SHA256
    ] + list(index.get("notes", []))
    return index


def quad_is_valid(quad, tolerance=1e-9):
    """凸、逆时针、面积>0 检查。"""
    if len(quad) != 4:
        return False
    crosses = []
    for i in range(4):
        ax, ay = quad[i]
        bx, by = quad[(i + 1) % 4]
        cx, cy = quad[(i + 2) % 4]
        crosses.append((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
    if not all(cross >= -tolerance for cross in crosses):
        return False
    area = sum(
        quad[i][0] * quad[(i + 1) % 4][1] - quad[(i + 1) % 4][0] * quad[i][1]
        for i in range(4)
    )
    return area > tolerance


def postcheck_case(entry, vehicle):
    obstacles = entry["obstacles"]
    for obstacle in obstacles:
        quad = [(v["x_m"], v["y_m"]) for v in obstacle["vertices"]]
        if not quad_is_valid(quad):
            raise ParkbenchFixtureError(
                "%s 的 %s 不是有效凸四边形" % (entry["id"], obstacle["id"])
            )
        for label in ("start", "goal"):
            pose = entry[label]
            box = footprint_corners(
                (pose["x_m"], pose["y_m"], pose["yaw_rad"]), vehicle
            )
            if convex_polygons_overlap(quad, box):
                raise ParkbenchFixtureError(
                    "%s 的 %s 与 %s 足迹框相交" % (entry["id"], obstacle["id"], label)
                )
    grid = entry["grid"]
    for obstacle in obstacles:
        for vertex in obstacle["vertices"]:
            x = (vertex["x_m"] - grid["origin"]["x_m"]) / RESOLUTION_M
            y = (vertex["y_m"] - grid["origin"]["y_m"]) / RESOLUTION_M
            if not (-1e-6 <= x <= grid["width"] + 1e-6
                    and -1e-6 <= y <= grid["height"] + 1e-6):
                raise ParkbenchFixtureError(
                    "%s 的 %s 顶点越出画布" % (entry["id"], obstacle["id"])
                )


def load_parkbench_cases(zip_path, expected_sha256):
    data = zip_path.read_bytes()
    actual = hashlib.sha256(data).hexdigest()
    if expected_sha256 and actual != expected_sha256:
        raise ParkbenchFixtureError(
            "源 zip SHA256 不符：期望 %s，实际 %s" % (expected_sha256, actual)
        )
    with zipfile.ZipFile(zip_path) as archive:
        names = sorted(
            name for name in archive.namelist()
            if name.startswith(ZIP_INNER_ROOT) and name.endswith(".json")
        )
        if len(names) != EXPECTED_CASES:
            raise ParkbenchFixtureError(
                "期望 %d 个场景，zip 内实际 %d" % (EXPECTED_CASES, len(names))
            )
        loaded = []
        for name in names:
            document = json.loads(archive.read(name).decode("utf-8"))
            loaded.append((Path(name).name, document))
    return actual, loaded


def generate(args):
    repo_root = Path(args.repo_root).resolve()
    fixtures_root = repo_root / FIXTURES_REL
    source_dir = fixtures_root / args.source_version
    target_dir = (
        Path(args.output_dir).resolve()
        if args.output_dir
        else fixtures_root / TARGET_VERSION_DEFAULT
    )
    zip_path = (
        Path(args.source_zip).resolve()
        if args.source_zip
        else repo_root / "artifacts" / "data" / "sha256" / PARKBENCH_SHA256 / ZIP_NAME
    )
    if not source_dir.is_dir():
        raise ParkbenchFixtureError("源版本目录不存在: %s" % source_dir)
    if not zip_path.is_file():
        raise ParkbenchFixtureError("源 zip 不存在: %s" % zip_path)

    actual_sha, loaded = load_parkbench_cases(zip_path, args.expected_sha256)
    if target_dir.exists():
        if not args.force:
            raise ParkbenchFixtureError("目标目录已存在（--force 覆盖）: %s" % target_dir)
        shutil.rmtree(target_dir)
    shutil.copytree(source_dir, target_dir,
                    ignore=shutil.ignore_patterns(MANIFEST_NAME))

    cases = []
    per_case = []
    totals = {"segments": 0, "outliers": 0, "target_removed": 0,
              "target_partial": 0, "start_removed": 0, "start_partial": 0,
              "quads": 0}
    for index, (source_name, document) in enumerate(loaded, start=1):
        start, target, segments = parse_case(document, source_name)
        quads, stats = clean_case(start, target, segments, VEHICLE)
        canvas = canvas_for(quads, start, target, VEHICLE)
        entry = build_case_entry(index, source_name, start, target, quads,
                                 stats, canvas)
        postcheck_case(entry, VEHICLE)
        cases.append(entry)
        per_case.append({
            "id": entry["id"], "source": source_name, "canvas_w": canvas[2],
            "canvas_h": canvas[3], **stats,
        })
        for key in totals:
            totals[key] += stats[key]

    audit = {"totals": totals, "per_case": per_case}
    precheck = None
    if args.precheck_json:
        precheck = json.loads(
            Path(args.precheck_json).read_text(encoding="utf-8"))
    cases_document = build_cases_document(cases, audit, VEHICLE, precheck)
    (target_dir / "paths").mkdir(parents=True, exist_ok=True)
    (target_dir / "vehicles").mkdir(parents=True, exist_ok=True)
    (target_dir / "paths" / "cases_bench_parkbench.json").write_text(
        canonical_json(cases_document), encoding="utf-8")
    (target_dir / "vehicles" / "parkbench_vehicle.json").write_text(
        canonical_json(build_vehicle_document()), encoding="utf-8")
    (target_dir / "README.md").write_text(
        build_readme(audit, cases_document), encoding="utf-8")
    source_index = json.loads((source_dir / "index.json").read_text(encoding="utf-8"))
    (target_dir / "index.json").write_text(
        canonical_json(build_index(source_index)), encoding="utf-8")

    print("v9 生成完成: %s（%d 场景，四边形 %d，源 SHA256 %s）"
          % (target_dir, len(cases), totals["quads"], actual_sha))
    print("清洗总计: 线段 %d → 离群 %d，目标框移除/裁剪 %d/%d，起点框移除/裁剪 %d/%d"
          % (totals["segments"], totals["outliers"], totals["target_removed"],
             totals["target_partial"], totals["start_removed"],
             totals["start_partial"]))
    return 0


def check(args):
    repo_root = Path(args.repo_root).resolve()
    target_dir = (
        Path(args.output_dir).resolve()
        if args.output_dir
        else repo_root / FIXTURES_REL / TARGET_VERSION_DEFAULT
    )
    problems = []
    cases_path = target_dir / "paths" / "cases_bench_parkbench.json"
    if not cases_path.is_file():
        problems.append("缺少 paths/cases_bench_parkbench.json")
    else:
        document = json.loads(cases_path.read_text(encoding="utf-8"))
        cases = document.get("cases", [])
        if len(cases) != EXPECTED_CASES:
            problems.append("场景数 %d != %d" % (len(cases), EXPECTED_CASES))
        expected_ids = ["pb-%03d" % i for i in range(1, EXPECTED_CASES + 1)]
        if [case.get("id") for case in cases] != expected_ids:
            problems.append("场景 id 序列不是 pb-001..pb-051")
        for case in cases:
            try:
                postcheck_case(case, VEHICLE)
            except ParkbenchFixtureError as exc:
                problems.append(str(exc))
                break
            grid = case["grid"]
            if grid["resolution_m"] != RESOLUTION_M:
                problems.append("%s 分辨率不是 0.1" % case["id"])
                break
            if not case["obstacles"]:
                problems.append("%s 无障碍" % case["id"])
                break
    vehicle_path = target_dir / "vehicles" / "parkbench_vehicle.json"
    if not vehicle_path.is_file():
        problems.append("缺少 vehicles/parkbench_vehicle.json")
    else:
        vehicle = json.loads(vehicle_path.read_text(encoding="utf-8"))
        if vehicle.get("id") != "parkbench_vehicle":
            problems.append("车辆 id 不是 parkbench_vehicle")
        if abs(vehicle.get("wheelbase_m", 0.0) - 3.0) > 1e-9:
            problems.append("车辆轴距不是 3.0")
        if abs(vehicle.get("width_m", 0.0) - 2.0) > 1e-9:
            problems.append("车辆宽度不是 2.0")
    if problems:
        for problem in problems:
            print("check 失败: %s" % problem, file=sys.stderr)
        return 1
    print("v9 夹具语义检查通过（%d 场景：凸四边形/起终点无交/画布一致/论文车辆）"
          % EXPECTED_CASES)
    print("提示：哈希清单校验请使用 task_runner.py fixtures verify --version v9")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        default=str(Path(__file__).resolve().parents[1]),
        help="仓库根目录（默认按脚本位置推断）",
    )
    subparsers = parser.add_subparsers(dest="action", required=True)

    generate_parser = subparsers.add_parser("generate", help="生成 v9 夹具")
    generate_parser.add_argument("--source-zip", default=None,
                                 help="ParkBench 源 zip（默认按归档路径）")
    generate_parser.add_argument(
        "--expected-sha256", default=PARKBENCH_SHA256,
        help="源 zip 期望 SHA256（空字符串跳过校验，供单测）")
    generate_parser.add_argument("--source-version", default=SOURCE_VERSION_DEFAULT)
    generate_parser.add_argument("--output-dir", default=None)
    generate_parser.add_argument("--force", action="store_true")
    generate_parser.add_argument("--precheck-json", default=None,
                                 help="起终点碰撞预检 JSON（可选，并入自检）")

    check_parser = subparsers.add_parser("check", help="语义检查已生成的 v9 夹具")
    check_parser.add_argument("--output-dir", default=None)

    args = parser.parse_args(argv)
    try:
        if args.action == "generate":
            return generate(args)
        return check(args)
    except ParkbenchFixtureError as exc:
        print("parkbench_fixtures 失败: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
