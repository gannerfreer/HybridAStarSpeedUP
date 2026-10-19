#!/usr/bin/env python3
"""v6 栅格夹具生成器（X-W1 交付）。

命令：

    python3 tools/grid_fixtures.py generate --version v6 [--output-dir DIR] [--force]
    python3 tools/grid_fixtures.py check    --version v6      # 亦支持 --check
    python3 tools/grid_fixtures.py validate --version v6

约定（见 ``Docs/expansion/fixtures_v6_plan.md``、``Docs/expansion/grid_protocol.md``）：

- v6 = v5 全量复制（不含 v5 的 ``fixture_manifest.sha256.json``）+ ``grids/`` 新增与
  修订；本工具只读 ``tests/fixtures/v5``，绝不写入 v5。
- 显示档由 ``splitmix64_thresholds_v1`` 确定性生成；生成与校验均从算法复算，
  不抄录文档数值；``display_spec.json`` 的 sha256 与 spot_checks 生成时回填并强制复验。
- 手算夹具（G1–G8）与协议用例（P1–P13）的期望值在生成时独立复算核对。
- 本模块只使用 CPython 3.8+ 标准库；JSON 文本布局不是契约（v5 README 第 91–93 行）。

已知计划文本偏差（生成时处理并记录）：

- G5 ``obs-d`` 三角形计划顶点 ``(0,2.5)/(1.5,4.0)/(3.0,2.5)`` 的底边与第 2 行单元
  中心共线；按协议 §3.3 闭合边界语义会额外占据 ``(2,0)``，与计划期望
  （8 格占据、``obs-d`` 仅 ``(3,1)``）不符。生成器以计划期望 cells 为准，将底边
  上移 0.1 m 为 ``y=2.6``，使闭合语义与期望值自洽（见 ``G5_OBSTACLES`` 注释）。
"""

from __future__ import annotations

import argparse
import base64
import binascii
import hashlib
import json
import math
import sys
import tempfile
from pathlib import Path

MANIFEST_NAME = "fixture_manifest.sha256.json"
FIXTURES_REL = "tests/fixtures"
SUPPORTED_VERSIONS = ("v6",)
DEFAULT_BUDGET_CELLS = 16 * 1024 * 1024
MAX_DOUBLINGS = 32
GRID_PROTOCOL_VERSION = "1.0"
SCENE_VERSION = "scene_v1"

MASK64 = (1 << 64) - 1
SPLITMIX_GAMMA = 0x9E3779B97F4A7C15
SPLITMIX_MIX1 = 0xBF58476D1CE4E5B9
SPLITMIX_MIX2 = 0x94D049BB133111EB
SPLITMIX_SHIFTS = (30, 27, 31)
THRESHOLD_MODULUS = 100
FREE_MAX = 82  # v < 82 -> free
OCCUPIED_MAX = 94  # v < 94 -> occupied；否则 unknown
CELL_ENCODING = {"0": "free", "1": "occupied", "2": "unknown"}

BASE64_ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

DISPLAY_TIERS = (
    {"name": "display_256", "width": 256, "height": 256, "resolution_m": 0.1, "seed": 20261012},
    {"name": "display_512", "width": 512, "height": 512, "resolution_m": 0.1, "seed": 20261013},
    {"name": "display_1024", "width": 1024, "height": 1024, "resolution_m": 0.1, "seed": 20261014},
)

DISPLAY_SPOT_CHECKS = (
    ("display_256", 0, 0),
    ("display_256", 1, 0),
    ("display_256", 255, 255),
    ("display_512", 0, 0),
    ("display_512", 1, 0),
    ("display_512", 511, 511),
    ("display_1024", 0, 0),
    ("display_1024", 1, 0),
    ("display_1024", 1023, 1023),
)

# ---------------------------------------------------------------------------
# splitmix64_thresholds_v1（生成器与校验脚本逐字一致；协议见计划 §4）
# ---------------------------------------------------------------------------


def splitmix64(x):
    """splitmix64 单步；x 先加黄金 gamma 再两次混合（64 位掩码）。"""
    x = (int(x) + SPLITMIX_GAMMA) & MASK64
    z = x
    z = ((z ^ (z >> 30)) * SPLITMIX_MIX1) & MASK64
    z = ((z ^ (z >> 27)) * SPLITMIX_MIX2) & MASK64
    return (z ^ (z >> 31)) & MASK64


def threshold_state(h):
    """h % 100 的三态阈值映射：<82 空闲、<94 占据、否则未知。"""
    value = int(h) % THRESHOLD_MODULUS
    if value < FREE_MAX:
        return 0
    if value < OCCUPIED_MAX:
        return 1
    return 2


def display_cell_hash(seed, width, row, col):
    """行主序索引 = seed + r*width + c（r=0 为底行）。"""
    return splitmix64(int(seed) + int(row) * int(width) + int(col))


def generate_display_states(seed, width, height):
    """按 splitmix64_thresholds_v1 生成 width×height 三态字节。"""
    states = bytearray(width * height)
    for row in range(height):
        base = int(seed) + row * width
        for col in range(width):
            states[row * width + col] = threshold_state(splitmix64(base + col))
    return states


# ---------------------------------------------------------------------------
# base64_u8 编解码（RFC 4648 §4；无空白；规范填充）
# ---------------------------------------------------------------------------


class FixtureError(RuntimeError):
    pass


def encode_cells(states):
    """三态字节 → 规范 base64（标准字母表、`=` 填充、无空白）。"""
    return base64.b64encode(bytes(states)).decode("ascii")


def decode_cells(text, width, height):
    """规范 base64 → 三态列表；长度与取值不合法时抛 FixtureError。"""
    if not isinstance(text, str) or text == "":
        raise FixtureError("cells 必须是规范 base64 字符串")
    if len(text) % 4 != 0:
        raise FixtureError("base64 长度不是 4 的倍数")
    for char in text:
        if char not in BASE64_ALPHABET and char != "=":
            raise FixtureError("base64 含非法字符: %r" % char)
    if "=" in text:
        if text.rstrip("=").find("=") != -1:
            raise FixtureError("base64 填充只允许出现在末尾")
        if len(text) - len(text.rstrip("=")) > 2:
            raise FixtureError("base64 填充最多 2 个")
    try:
        raw = base64.b64decode(text, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise FixtureError("base64 解码失败: %s" % exc)
    if base64.b64encode(raw).decode("ascii") != text:
        raise FixtureError("base64 不是规范编码（填充位非零或非规范）")
    if width < 1 or height < 1:
        raise FixtureError("width/height 必须 >= 1")
    if len(raw) != width * height:
        raise FixtureError("解码长度 %d != width×height %d" % (len(raw), width * height))
    for value in raw:
        if value not in (0, 1, 2):
            raise FixtureError("单元取值非法: %d" % value)
    return list(raw)


# ---------------------------------------------------------------------------
# 栅格坐标换算（cell (0,0) 左下角，origin 位姿含旋转）
# ---------------------------------------------------------------------------


def _rotation(origin):
    yaw = float(origin["yaw_rad"])
    return math.cos(yaw), math.sin(yaw)


def cell_center(origin, resolution_m, row, col):
    """cell (r,c) 中心 = origin + R(yaw)·((c+0.5)·res, (r+0.5)·res)。"""
    cos_yaw, sin_yaw = _rotation(origin)
    dx = (col + 0.5) * resolution_m
    dy = (row + 0.5) * resolution_m
    return (
        origin["x_m"] + cos_yaw * dx - sin_yaw * dy,
        origin["y_m"] + sin_yaw * dx + cos_yaw * dy,
    )


def cell_corners(origin, resolution_m, row, col):
    """cell 四角（世界坐标），逆时针（按 origin 局部系）。"""
    cos_yaw, sin_yaw = _rotation(origin)
    corners = []
    for local_x, local_y in (
        (col * resolution_m, row * resolution_m),
        ((col + 1) * resolution_m, row * resolution_m),
        ((col + 1) * resolution_m, (row + 1) * resolution_m),
        (col * resolution_m, (row + 1) * resolution_m),
    ):
        corners.append((
            origin["x_m"] + cos_yaw * local_x - sin_yaw * local_y,
            origin["y_m"] + sin_yaw * local_x + cos_yaw * local_y,
        ))
    return corners


def world_to_cell(origin, resolution_m, x, y):
    """世界点 → (row, col)；col = floor(局部 x/res)、row = floor(局部 y/res)。"""
    cos_yaw, sin_yaw = _rotation(origin)
    dx = x - origin["x_m"]
    dy = y - origin["y_m"]
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    return (math.floor(local_y / resolution_m), math.floor(local_x / resolution_m))


def in_bounds(width, height, row, col):
    return 0 <= row < height and 0 <= col < width


def point_in_obstacle(obstacle, x, y, eps=1e-9):
    """单元中心是否位于闭合障碍内（矩形含边界；凸多边形含边界）。"""
    if obstacle["type"] == "rectangle":
        return (
            obstacle["x_min_m"] - eps <= x <= obstacle["x_max_m"] + eps
            and obstacle["y_min_m"] - eps <= y <= obstacle["y_max_m"] + eps
        )
    vertices = obstacle["vertices"]
    if len(vertices) < 3:
        raise FixtureError("凸多边形顶点数 < 3")
    sign = 0
    count = len(vertices)
    for index in range(count):
        x0, y0 = vertices[index]["x_m"], vertices[index]["y_m"]
        x1, y1 = vertices[(index + 1) % count]["x_m"], vertices[(index + 1) % count]["y_m"]
        cross = (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0)
        if abs(cross) <= eps:
            continue
        current = 1 if cross > 0 else -1
        if sign == 0:
            sign = current
        elif sign != current:
            return False
    return True


def rasterize_obstacles(obstacles, width, height, resolution_m, origin):
    """按闭合边界规则栅格化障碍（协议 §3.3 语义 3）。"""
    cells = []
    for row in range(height):
        for col in range(width):
            x, y = cell_center(origin, resolution_m, row, col)
            occupied = 0
            for obstacle in obstacles:
                if point_in_obstacle(obstacle, x, y):
                    occupied = 1
                    break
            cells.append(occupied)
    return cells


# ---------------------------------------------------------------------------
# 单元预算与降级（协议 §4）
# ---------------------------------------------------------------------------


def degrade_resolution(window, requested_resolution_m, budget_cells=DEFAULT_BUDGET_CELLS,
                       max_doublings=MAX_DOUBLINGS):
    """按预算降级：返回结果 dict；guard 耗尽仍超限时返回 None（拒绝）。"""
    requested = float(requested_resolution_m)
    if not math.isfinite(requested) or requested <= 0:
        raise FixtureError("请求分辨率必须有限且 > 0")
    span_x = float(window["max_x_m"]) - float(window["min_x_m"])
    span_y = float(window["max_y_m"]) - float(window["min_y_m"])
    if not (span_x > 0 and span_y > 0):
        raise FixtureError("窗口必须 min < max")
    resolution = requested
    doublings = 0
    while True:
        width = int(math.ceil(span_x / resolution))
        height = int(math.ceil(span_y / resolution))
        count = width * height
        if count <= budget_cells:
            return {
                "requested_resolution_m": requested,
                "effective_resolution_m": resolution,
                "degraded": resolution > requested,
                "width": width,
                "height": height,
                "cells": count,
                "budget_cells": budget_cells,
                "doublings": doublings,
            }
        if doublings >= max_doublings:
            return None
        resolution *= 2.0
        doublings += 1


# ---------------------------------------------------------------------------
# scene_id 规范序列化（协议 §5；仅用于夹具期望值与自检）
# ---------------------------------------------------------------------------


def quantize_fixed(value):
    """四舍五入（round half away from zero）到 1e-9 的整数倍，返回纳单位整数。"""
    value = float(value)
    if not math.isfinite(value):
        raise FixtureError("非有限数值不能进入规范序列化: %r" % (value,))
    scaled = value * 1e9
    if scaled >= 0:
        nanos = math.floor(scaled + 0.5)
    else:
        nanos = -math.floor(-scaled + 0.5)
    return int(nanos)


def format_fixed(value):
    """定点格式：恰好 9 位小数；-0.000000000 归一化为 0.000000000。"""
    nanos = quantize_fixed(value)
    if nanos == 0:
        return "0.000000000"
    sign = "-" if nanos < 0 else ""
    magnitude = abs(nanos)
    return "%s%d.%09d" % (sign, magnitude // 1000000000, magnitude % 1000000000)


def canonical_json(value):
    """规范 JSON 文本：键按 Unicode 码点升序、无多余空白、数值定点 9 位小数。"""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return format_fixed(value)
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, dict):
        parts = []
        for key in sorted(value.keys()):
            parts.append(json.dumps(key, ensure_ascii=False) + ":" + canonical_json(value[key]))
        return "{" + ",".join(parts) + "}"
    if isinstance(value, (list, tuple)):
        return "[" + ",".join(canonical_json(item) for item in value) + "]"
    raise FixtureError("规范序列化不支持类型: %r" % (type(value),))


def canonical_obstacle(obstacle):
    if obstacle["type"] == "rectangle":
        return {
            "id": obstacle["id"],
            "type": "rectangle",
            "x_max_m": float(obstacle["x_max_m"]),
            "x_min_m": float(obstacle["x_min_m"]),
            "y_max_m": float(obstacle["y_max_m"]),
            "y_min_m": float(obstacle["y_min_m"]),
        }
    if obstacle["type"] == "convex_polygon":
        return {
            "id": obstacle["id"],
            "type": "convex_polygon",
            "vertices": [
                {"x_m": float(vertex["x_m"]), "y_m": float(vertex["y_m"])}
                for vertex in obstacle["vertices"]
            ],
        }
    raise FixtureError("未知障碍类型: %r" % (obstacle.get("type"),))


def scene_canonical_input(map_id, map_version, window, resolution_m, width, height,
                          origin, obstacles):
    """协议 §5.2 规范输入对象（字段集合固定）。"""
    return {
        "domain": "planner.scene.content.v1",
        "scene_version": SCENE_VERSION,
        "map_id": map_id,
        "map_version": map_version,
        "window": {
            "min_x_m": float(window["min_x_m"]),
            "min_y_m": float(window["min_y_m"]),
            "max_x_m": float(window["max_x_m"]),
            "max_y_m": float(window["max_y_m"]),
        },
        "resolution_m": float(resolution_m),
        "width": int(width),
        "height": int(height),
        "origin": {
            "x_m": float(origin["x_m"]),
            "y_m": float(origin["y_m"]),
            "yaw_rad": float(origin["yaw_rad"]),
        },
        "obstacles": [canonical_obstacle(item) for item in sorted(obstacles, key=lambda item: item["id"])],
    }


def scene_id_of(map_id, map_version, window, resolution_m, width, height, origin, obstacles):
    text = canonical_json(scene_canonical_input(
        map_id, map_version, window, resolution_m, width, height, origin, obstacles))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


# 协议 §5.3 冻结示例（conformance 断言；生成器必须复算出同一文本与 SHA-256）
SCENE_ID_EXAMPLE_CANONICAL = (
    '{"domain":"planner.scene.content.v1","height":2,"map_id":"mapping_example",'
    '"map_version":"11561c763ed50322185139ec2dd041d6da72e1bd4e5fc3bb66f6b340d92c088f",'
    '"obstacles":[{"id":"obs-1","type":"rectangle","x_max_m":0.400000000,"x_min_m":0.200000000,'
    '"y_max_m":0.200000000,"y_min_m":0.000000000}],'
    '"origin":{"x_m":0.000000000,"y_m":0.000000000,"yaw_rad":0.000000000},'
    '"resolution_m":0.200000000,"scene_version":"scene_v1","width":3,'
    '"window":{"max_x_m":0.600000000,"max_y_m":0.400000000,"min_x_m":0.000000000,"min_y_m":0.000000000}}'
)
SCENE_ID_EXAMPLE_SHA256 = "20821dc86d1519b59d252e17f34a3ccf35dcccb1f7a0f876eedd108718e4e776"


def verify_scene_id_example():
    """自检规范序列化与协议 §5.3 冻结示例一致；不一致即中止生成。"""
    text = canonical_json(scene_canonical_input(
        "mapping_example",
        "11561c763ed50322185139ec2dd041d6da72e1bd4e5fc3bb66f6b340d92c088f",
        {"min_x_m": 0.0, "min_y_m": 0.0, "max_x_m": 0.6, "max_y_m": 0.4},
        0.2, 3, 2, {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        [{"id": "obs-1", "type": "rectangle", "x_min_m": 0.2, "y_min_m": 0.0,
          "x_max_m": 0.4, "y_max_m": 0.2}],
    ))
    if text != SCENE_ID_EXAMPLE_CANONICAL:
        raise FixtureError(
            "scene_id 规范序列化与协议 §5.3 示例不一致:\n  实际: %s\n  期望: %s"
            % (text, SCENE_ID_EXAMPLE_CANONICAL))
    digest = hashlib.sha256(text.encode("utf-8")).hexdigest()
    if digest != SCENE_ID_EXAMPLE_SHA256:
        raise FixtureError("scene_id 示例 SHA-256 不一致: %s" % digest)
    return digest


# ---------------------------------------------------------------------------
# JSON 文本渲染（确定性；grid_view 的 cells 按行排版）
# ---------------------------------------------------------------------------


def render_object(fields, cells=None, width=0, height=0, trailing=None):
    """按给定字段顺序渲染 JSON 对象；cells 为整数数组时按 width 分行。

    fields: [(key, value)]；cells: 整数列表或 None；trailing: [(key, value)]（cells 之后）。
    """
    lines = ["{"]
    entries = list(fields)
    if cells is None:
        for index, (key, value) in enumerate(entries + list(trailing or [])):
            comma = "," if index < len(entries) + len(trailing or []) - 1 else ""
            lines.append('  "%s": %s%s' % (key, json.dumps(value, ensure_ascii=False), comma))
        lines.append("}")
        return "\n".join(lines) + "\n"
    for key, value in entries:
        lines.append('  "%s": %s,' % (key, json.dumps(value, ensure_ascii=False)))
    lines.append('  "cells": [')
    for row in range(height):
        row_cells = cells[row * width:(row + 1) * width]
        comma = "," if row < height - 1 else ""
        lines.append("    " + ",".join(str(int(value)) for value in row_cells) + comma)
    lines.append("  ]" + ("," if trailing else ""))
    for index, (key, value) in enumerate(trailing or []):
        comma = "," if index < len(trailing or []) - 1 else ""
        lines.append('  "%s": %s%s' % (key, json.dumps(value, ensure_ascii=False), comma))
    lines.append("}")
    return "\n".join(lines) + "\n"


def dump_json(document):
    return json.dumps(document, ensure_ascii=False, indent=2, sort_keys=False) + "\n"


def read_json(path):
    with open(str(path), "r", encoding="utf-8") as handle:
        return json.load(handle)


# ---------------------------------------------------------------------------
# 手算夹具 G1–G8（期望值在生成时独立复算）
# ---------------------------------------------------------------------------

G5_OBSTACLES = (
    # 计划 §3.4 文本：obs-a (1,1)-(3,3)、obs-b (4,0)-(6,1)、obs-c (5.0,2.0)-(5.5,3.0)、
    # obs-d 三角形 (0,2.5)/(1.5,4.0)/(3.0,2.5)。obs-d 底边与第 2 行中心共线，闭合语义
    # 会多占 (2,0)；以计划期望 cells（8 格、obs-d 仅 (3,1)）为准，底边上移 0.1 m。
    {"id": "obs-a", "type": "rectangle", "x_min_m": 1.0, "y_min_m": 1.0, "x_max_m": 3.0, "y_max_m": 3.0},
    {"id": "obs-b", "type": "rectangle", "x_min_m": 4.0, "y_min_m": 0.0, "x_max_m": 6.0, "y_max_m": 1.0},
    {"id": "obs-c", "type": "rectangle", "x_min_m": 5.0, "y_min_m": 2.0, "x_max_m": 5.5, "y_max_m": 3.0},
    {"id": "obs-d", "type": "convex_polygon", "vertices": [
        {"x_m": 0.0, "y_m": 2.6}, {"x_m": 1.5, "y_m": 4.0}, {"x_m": 3.0, "y_m": 2.6},
    ]},
)


def _grid_view_document(map_id, width, height, resolution_m, origin, cells, notes):
    return render_object(
        fields=[
            ("schema_version", 1),
            ("type", "grid_view"),
            ("map_id", map_id),
            ("map_version", "v6"),
            ("width", width),
            ("height", height),
            ("resolution_m", resolution_m),
            ("origin", origin),
            ("cell_encoding", CELL_ENCODING),
        ],
        cells=cells,
        width=width,
        height=height,
        trailing=[("notes", notes)],
    )


def build_g1():
    """G1：旋转原点 CCW（π/2）。"""
    origin = {"x_m": 10.0, "y_m": -5.0, "yaw_rad": math.pi / 2.0}
    cells = [
        1, 0, 0, 0,
        0, 0, 2, 0,
        0, 0, 0, 0,
        0, 0, 0, 1,
    ]
    notes = (
        "4×4、0.5 m、origin=(10.0,-5.0,π/2)。cell (0,0) 中心 (9.75,-4.75) occupied、"
        "(1,2) (9.25,-3.75) unknown、(3,3) (8.25,-3.25) occupied；cell (0,0) 四角 "
        "(10,-5)/(10,-4.5)/(9.5,-4.5)/(9.5,-5)。旋转原点按 origin 位姿换算，"
        "不得按轴对齐假设。"
    )
    return "grids/rotated_origin_ccw.json", _grid_view_document(
        "grid_rotated_origin_ccw", 4, 4, 0.5, origin, cells, notes)


def build_g2():
    """G2：旋转原点 CW（-π/2）。"""
    origin = {"x_m": -3.0, "y_m": 2.0, "yaw_rad": -math.pi / 2.0}
    cells = [
        0, 0, 0, 1,
        0, 0, 0, 0,
        0, 2, 0, 0,
        1, 0, 0, 0,
    ]
    notes = (
        "4×4、0.5 m、origin=(-3.0,2.0,-π/2)。cell (0,0) 中心 (-2.75,1.75) free、"
        "(0,3) (-2.75,0.25) occupied、(2,1) (-1.75,1.25) unknown、(3,0) (-1.25,1.75) "
        "occupied、(3,3) (-1.25,0.25) free；yaw 在 [-π,π)。"
    )
    return "grids/rotated_origin_cw.json", _grid_view_document(
        "grid_rotated_origin_cw", 4, 4, 0.5, origin, cells, notes)


def build_g3():
    """G3：边缘单元与越界。"""
    origin = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
    cells = [
        1, 0, 0, 1,
        0, 0, 0, 0,
        0, 0, 0, 0,
        1, 0, 0, 1,
    ]
    notes = (
        "4×4、1.0 m、origin=(0,0,0)；仅四角 occupied。(0,0) 中心 (0.5,0.5)、(3,3) "
        "(3.5,3.5)。window 精确 [0,4]×[0,4] 时 width=height=4（无多余单元）；"
        "in_bounds 对 (-1,0)/(0,-1)/(4,0)/(0,4) 为 false；地图外与 unknown 按占据。"
    )
    return "grids/edge_cells.json", _grid_view_document(
        "grid_edge_cells", 4, 4, 1.0, origin, cells, notes)


def build_g4():
    """G4：unknown 与三态。"""
    origin = {"x_m": -2.0, "y_m": -2.0, "yaw_rad": 0.0}
    cells = [
        0, 0, 0, 0, 2,
        0, 2, 2, 2, 0,
        0, 2, 2, 2, 0,
        0, 0, 0, 1, 1,
        2, 0, 0, 0, 0,
    ]
    notes = (
        "5×5、0.5 m、origin=(-2,-2,0)。unknown 块 = 第 1–2 行第 1–3 列（6 格）；"
        "孤立 unknown (0,4) 与 (4,0)；occupied (3,3)、(3,4)。抽查中心：(1,1) "
        "(-1.25,-1.25)、(0,4) (0.25,-1.75)、(4,0) (-1.75,0.25)、(3,3) (-0.25,-0.25)。"
        "三态齐全；规划消费方把 unknown 视为占据；显示端第三态图例可区分。"
    )
    return "grids/unknown_edges.json", _grid_view_document(
        "grid_unknown_edges", 5, 5, 0.5, origin, cells, notes)


def build_g5():
    """G5：障碍叠加（矩形 + 凸多边形；闭合边界语义）。"""
    origin = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
    width, height, resolution = 6, 4, 1.0
    cells = rasterize_obstacles(G5_OBSTACLES, width, height, resolution, origin)
    notes = (
        "6×4、1.0 m、origin=(0,0,0)；基础域全空闲，叠加 4 个障碍（obs-a 矩形 "
        "(1,1)-(3,3)；obs-b 矩形 (4,0)-(6,1)；obs-c 矩形 (5.0,2.0)-(5.5,3.0) 中心 "
        "x=5.5 在闭合边上；obs-d 凸三角形，底边 y=2.6——计划文本 2.5 与第 2 行中心"
        "共线会按闭合语义多占 (2,0)，与计划期望 8 格不符，生成器以期望 cells 为准）。"
        "共 8 格占据、其余空闲、不含 unknown；边界接触即占据。"
    )
    return "grids/obstacle_overlay.json", _grid_view_document(
        "grid_obstacle_overlay", width, height, resolution, origin, cells, notes)


def build_g6():
    """G6：行列 ↔ 世界坐标。"""
    origin = {"x_m": 1.5, "y_m": -2.0, "yaw_rad": 0.0}
    cells = [
        1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1,
    ]
    notes = (
        "6×5、0.25 m、origin=(1.5,-2.0,0)。正向：(0,0)→(1.625,-1.875)、(0,5)→"
        "(2.875,-1.875)、(4,0)→(1.625,-0.875)、(4,5)→(2.875,-0.875)、(2,2)→"
        "(2.125,-1.375)。反向 col=floor((x-1.5)/0.25)、row=floor((y+2)/0.25)："
        "(1.5,-2.0)→(0,0)；(1.7499,-1.7501)→(0,0)；(1.75,-1.75)→(1,1)；"
        "(2.9999,-0.7501)→(4,5)；(3.0,-0.75) 出界（col=6/row=5，不静默夹取）。"
    )
    return "grids/coord_conversion.json", _grid_view_document(
        "grid_coord_conversion", 6, 5, 0.25, origin, cells, notes)


def build_g7():
    """G7：小型 grid_snapshot 解码样例。"""
    document = {
        "schema_version": 1,
        "type": "grid_snapshot",
        "source": "fixture",
        "map_id": "grid_snapshot_small",
        "map_version": "v6",
        "window": {"min_x_m": 0.0, "min_y_m": 0.0, "max_x_m": 0.3, "max_y_m": 0.2},
        "requested_resolution_m": 0.1,
        "effective_resolution_m": 0.1,
        "resolution_m": 0.1,
        "degraded": False,
        "width": 3,
        "height": 2,
        "origin": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        "encoding": "base64_u8",
        "cells": "AAEAAAAC",
        "notes": (
            "3×2、0.1 m、origin=(0,0,0)；解码字节 [0,1,0,0,0,2]，长度 = 3×2 = 6；"
            "cell (0,0)=0 中心 (0.05,0.05)、(0,1)=1 中心 (0.15,0.05)、(1,2)=2 中心 "
            "(0.25,0.15)。base64 解码路径、三态、长度自检、行主序与左下角约定。"
        ),
    }
    return "grids/snapshot_small.json", dump_json(document)


def build_g8():
    """G8：64² 正确性小图（边界 → 对角 → %5 优先级）。"""
    width = height = 64
    cells = []
    for row in range(height):
        for col in range(width):
            if row == 0 or row == height - 1 or col == 0 or col == width - 1:
                cells.append(1)
            elif row == col:
                cells.append(2)
            elif (row + col) % 5 == 0:
                cells.append(1)
            else:
                cells.append(0)
    notes = (
        "64×64、0.1 m、origin=(0,0,0)、window=[0,6.4]²。规则（自上而下首个命中）："
        "边界（r∈{0,63} 或 c∈{0,63}）→ 占据；r==c → 未知；(r+c)%5==0 → 占据；"
        "其余空闲。总计 4096 格 = 空闲 3026、占据 1008、未知 62。抽查：(0,0)=1、"
        "(1,1)=2、(1,2)=0、(1,4)=1、(2,3)=1、(31,31)=2、(31,32)=0、(62,62)=2、"
        "(62,60)=0、(63,63)=1。"
    )
    return "grids/pattern_64.json", _grid_view_document(
        "grid_pattern_64", width, height, 0.1, {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        cells, notes)


HAND_FIXTURE_BUILDERS = (build_g1, build_g2, build_g3, build_g4, build_g5,
                         build_g6, build_g7, build_g8)


def g8_expected_counts():
    """从规则函数复算 G8 三态计数（生成与校验共用）。"""
    counts = {0: 0, 1: 0, 2: 0}
    width = height = 64
    for row in range(height):
        for col in range(width):
            if row == 0 or row == height - 1 or col == 0 or col == width - 1:
                state = 1
            elif row == col:
                state = 2
            elif (row + col) % 5 == 0:
                state = 1
            else:
                state = 0
            counts[state] += 1
    return counts


# ---------------------------------------------------------------------------
# 显示档（display_256/512/1024 + display_spec）
# ---------------------------------------------------------------------------


def display_window(width, height, resolution_m):
    return {
        "min_x_m": 0.0,
        "min_y_m": 0.0,
        "max_x_m": round(width * resolution_m, 9),
        "max_y_m": round(height * resolution_m, 9),
    }


def build_display_document(tier, cells_b64):
    width = tier["width"]
    height = tier["height"]
    resolution = tier["resolution_m"]
    document = {
        "schema_version": 1,
        "type": "grid_snapshot",
        "source": "fixture",
        "map_id": "grid_display_%d" % width,
        "map_version": "v6",
        "window": display_window(width, height, resolution),
        "requested_resolution_m": resolution,
        "effective_resolution_m": resolution,
        "resolution_m": resolution,
        "degraded": False,
        "width": width,
        "height": height,
        "origin": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        "encoding": "base64_u8",
        "cells": cells_b64,
        "budget_cells": DEFAULT_BUDGET_CELLS,
        "notes": (
            "合成显示夹具（source=fixture），非算法正确性证据。splitmix64_thresholds_v1、"
            "seed=%d、行主序（r=0 为底行）；三态阈值 <82 free、<94 occupied、其余 unknown。"
            "文件 sha256 与 spot_checks 见 display_spec.json（生成时回填、强制复算）。"
            % (tier["seed"],)
        ),
    }
    return dump_json(document)


def build_display_documents():
    """生成三档显示快照；返回 [{rel, text, tier, sha256, state_counts}]。"""
    results = []
    for tier in DISPLAY_TIERS:
        states = generate_display_states(tier["seed"], tier["width"], tier["height"])
        cells_b64 = encode_cells(states)
        text = build_display_document(tier, cells_b64)
        sha256_hex = hashlib.sha256(text.encode("utf-8")).hexdigest()
        counts = {0: 0, 1: 0, 2: 0}
        for state in states:
            counts[state] += 1
        results.append({
            "rel": "grids/%s.json" % tier["name"],
            "text": text,
            "tier": tier,
            "sha256": sha256_hex,
            "state_counts": counts,
        })
    return results


def build_display_spec(display_results):
    tiers = []
    for result in display_results:
        tier = result["tier"]
        tiers.append({
            "name": tier["name"],
            "file": result["rel"],
            "map_id": "grid_display_%d" % tier["width"],
            "width": tier["width"],
            "height": tier["height"],
            "resolution_m": tier["resolution_m"],
            "window": display_window(tier["width"], tier["height"], tier["resolution_m"]),
            "seed": tier["seed"],
            "sha256": result["sha256"],
            "state_counts": {
                "free": result["state_counts"][0],
                "occupied": result["state_counts"][1],
                "unknown": result["state_counts"][2],
            },
        })
    by_name = {result["tier"]["name"]: result for result in display_results}
    spot_checks = []
    for name, row, col in DISPLAY_SPOT_CHECKS:
        result = by_name[name]
        tier = result["tier"]
        h = display_cell_hash(tier["seed"], tier["width"], row, col)
        spot_checks.append({
            "tier": name,
            "row": row,
            "col": col,
            "h": str(h),
            "v": h % THRESHOLD_MODULUS,
            "state": threshold_state(h),
        })
    document = {
        "schema_version": 1,
        "type": "grid_display_spec",
        "fixture_version": "v6",
        "pattern": {
            "name": "splitmix64_thresholds_v1",
            "mask": "2^64-1",
            "splitmix64": {
                "gamma": "0x9E3779B97F4A7C15",
                "mix1": "0xBF58476D1CE4E5B9",
                "mix2": "0x94D049BB133111EB",
                "shifts": list(SPLITMIX_SHIFTS),
            },
            "cell_order": "row-major; r=0 is bottom row; index = seed + r*width + c",
            "thresholds": {
                "modulus": THRESHOLD_MODULUS,
                "free": "v < %d" % FREE_MAX,
                "occupied": "v < %d" % OCCUPIED_MAX,
                "unknown": "otherwise",
            },
            "state_values": CELL_ENCODING,
        },
        "tiers": tiers,
        "spot_checks": spot_checks,
        "notes": (
            "显示档为 grid_snapshot（source=fixture、encoding=base64_u8）；sha256 为文件"
            "字节哈希，生成时回填并由生成器/校验脚本强制复算（与 map bench archive.sha256 "
            "同风格）。spot_checks 的 h/v/state 由 splitmix64_thresholds_v1 独立复算。"
        ),
    }
    return dump_json(document)


# ---------------------------------------------------------------------------
# 协议用例 P1–P13
# ---------------------------------------------------------------------------

DRIVEABLE_ENDPOINT = "/map/grid/driveable"
PREVIEW_ENDPOINT = "/map/scene/preview"


def _error_case(case_id, name, endpoint, request, reason, http=400, status="invalid_input", extra=None):
    expected = {"http": http, "status": status, "reason": reason}
    if extra:
        expected.update(extra)
    return {
        "id": case_id,
        "name": name,
        "kind": "error",
        "endpoint": endpoint,
        "request": request,
        "expected": expected,
    }


def _window(min_x, min_y, max_x, max_y):
    return {"min_x_m": min_x, "min_y_m": min_y, "max_x_m": max_x, "max_y_m": max_y}


def build_protocol_cases(repo_root):
    """P1–P13；地图身份与边界从 v3 夹具读取，期望值独立复算。"""
    repo_root = Path(repo_root)
    v3_root = repo_root / FIXTURES_REL / "v3" / "map"
    osm_path = v3_root / "mapping_example.osm"
    snapshot_path = v3_root / "mapping_example.snapshot.json"
    if not osm_path.is_file() or not snapshot_path.is_file():
        raise FixtureError("缺少 v3 地图夹具（P7/P11/P13 需要）: %s" % v3_root)
    map_version = hashlib.sha256(osm_path.read_bytes()).hexdigest()
    snapshot = read_json(snapshot_path)
    if snapshot.get("map_version") != map_version:
        raise FixtureError("v3 地图快照 map_version 与 OSM 字节 SHA-256 不一致")
    map_id = snapshot["map_id"]
    bounds = snapshot["bounds_m"]

    cases = []

    cases.append(_error_case(
        "P1", "empty_window_x", DRIVEABLE_ENDPOINT,
        {"window": _window(0.0, 0.0, 0.0, 1.0), "resolution_m": 0.1, "request_seq": 1},
        "empty_window"))
    cases.append(_error_case(
        "P2", "empty_window_y", DRIVEABLE_ENDPOINT,
        {"window": _window(0.0, 0.0, 1.0, 0.0), "resolution_m": 0.1, "request_seq": 2},
        "empty_window"))
    cases.append(_error_case(
        "P3", "inverted_window", DRIVEABLE_ENDPOINT,
        {"window": _window(1.0, 1.0, 0.0, 0.0), "resolution_m": 0.1, "request_seq": 3},
        "empty_window"))
    cases.append(_error_case(
        "P4", "resolution_zero", DRIVEABLE_ENDPOINT,
        {"window": _window(0.0, 0.0, 1.0, 1.0), "resolution_m": 0.0, "request_seq": 4},
        "invalid_resolution"))
    cases.append(_error_case(
        "P5", "resolution_negative", DRIVEABLE_ENDPOINT,
        {"window": _window(0.0, 0.0, 1.0, 1.0), "resolution_m": -0.1, "request_seq": 5},
        "invalid_resolution"))
    cases.append(_error_case(
        "P6", "non_finite", DRIVEABLE_ENDPOINT,
        {"raw_body": (
            '{"window":{"min_x_m":0.0,"min_y_m":0.0,"max_x_m":1.0,"max_y_m":1.0},'
            '"resolution_m":1e999}')},
        "non_finite_value",
        extra={"subcases": [
            {"token": "1e999", "expected": {"http": 400, "status": "invalid_input",
                                            "reason": "non_finite_value"}},
            {"token": "NaN（裸 NaN，非法 JSON）", "expected": {"http": 400,
                                                              "status": "invalid_input",
                                                              "reason": "parse_error"}},
        ]}))

    outside_window = _window(1e6, 1e6, 1e6 + 10.0, 1e6 + 10.0)
    if not (outside_window["min_x_m"] > bounds["max_x_m"] or outside_window["max_y_m"] > bounds["max_y_m"]):
        raise FixtureError("P7 窗口必须完全在地图外")
    cases.append(_error_case(
        "P7", "window_outside_map", PREVIEW_ENDPOINT,
        {"window": outside_window, "resolution_m": 0.1, "request_seq": 6},
        "window_outside_map"))

    def _degrade_case(case_id, name, window, expected_resolution, expected_width,
                      expected_height, expected_doublings):
        result = degrade_resolution(window, 0.1)
        if result is None:
            raise FixtureError("%s 期望降级成功，但 guard 耗尽" % case_id)
        if (result["effective_resolution_m"] != expected_resolution
                or result["width"] != expected_width
                or result["height"] != expected_height
                or result["doublings"] != expected_doublings
                or not result["degraded"]):
            raise FixtureError("%s 降级复算不一致: %r" % (case_id, result))
        return {
            "id": case_id,
            "name": name,
            "kind": "degradation",
            "endpoint": DRIVEABLE_ENDPOINT,
            "request": {"window": window, "resolution_m": 0.1, "request_seq": 7},
            "expected": {
                "http": 200,
                "requested_resolution_m": 0.1,
                "effective_resolution_m": result["effective_resolution_m"],
                "degraded": True,
                "width": result["width"],
                "height": result["height"],
                "cells": result["cells"],
                "budget_cells": result["budget_cells"],
                "guard_doublings": result["doublings"],
            },
        }

    cases.append(_degrade_case("P8", "degrade_k2", _window(0.0, 0.0, 1600.0, 1600.0),
                               0.4, 4000, 4000, 2))
    cases.append(_degrade_case("P9", "degrade_k3", _window(0.0, 0.0, 2000.0, 2000.0),
                               0.8, 2500, 2500, 3))

    guard_window = _window(0.0, 0.0, 2e12, 2e12)
    guard_result = degrade_resolution(guard_window, 0.1)
    if guard_result is not None:
        raise FixtureError("P10 期望 guard 耗尽拒绝，实际降级成功: %r" % guard_result)
    cases.append(_error_case(
        "P10", "reject_guard", DRIVEABLE_ENDPOINT,
        {"window": guard_window, "resolution_m": 0.1, "request_seq": 8},
        "grid_too_large",
        extra={"guard_doublings": MAX_DOUBLINGS, "budget_cells": DEFAULT_BUDGET_CELLS}))

    # P11：预览 vs 规划输入快照逐格一致（窗口与障碍固定于 v3 地图内）
    p11_window = _window(0.0, 0.0, 20.0, 20.0)
    p11_resolution = 0.1
    p11_obstacles = [
        {"id": "obs-a", "type": "rectangle", "x_min_m": 2.0, "y_min_m": 2.0,
         "x_max_m": 4.0, "y_max_m": 4.0},
        {"id": "obs-b", "type": "convex_polygon", "vertices": [
            {"x_m": 10.0, "y_m": 10.0}, {"x_m": 12.0, "y_m": 10.0}, {"x_m": 11.0, "y_m": 13.0},
        ]},
    ]
    for obstacle in p11_obstacles:
        if obstacle["type"] == "rectangle":
            points = [(obstacle["x_min_m"], obstacle["y_min_m"]),
                      (obstacle["x_max_m"], obstacle["y_max_m"])]
        else:
            points = [(vertex["x_m"], vertex["y_m"]) for vertex in obstacle["vertices"]]
        for x, y in points:
            if not (bounds["min_x_m"] <= x <= bounds["max_x_m"]
                    and bounds["min_y_m"] <= y <= bounds["max_y_m"]):
                raise FixtureError("P11 障碍超出 v3 地图边界: %s" % obstacle["id"])
            if not (p11_window["min_x_m"] <= x <= p11_window["max_x_m"]
                    and p11_window["min_y_m"] <= y <= p11_window["max_y_m"]):
                raise FixtureError("P11 障碍超出窗口: %s" % obstacle["id"])
    p11_degraded = degrade_resolution(p11_window, p11_resolution)
    if p11_degraded is None or p11_degraded["degraded"]:
        raise FixtureError("P11 期望不降级: %r" % p11_degraded)
    p11_origin = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
    p11_scene_id = scene_id_of(map_id, map_version, p11_window, p11_resolution,
                               p11_degraded["width"], p11_degraded["height"],
                               p11_origin, p11_obstacles)
    cases.append({
        "id": "P11",
        "name": "preview_plan_consistency",
        "kind": "consistency",
        "endpoints": {"preview": PREVIEW_ENDPOINT, "plan": "/plan/scene/{scene_id}"},
        "request": {
            "map_id": map_id,
            "map_version": map_version,
            "window": p11_window,
            "resolution_m": p11_resolution,
            "obstacles": p11_obstacles,
            "request_seq": 9,
        },
        "expected": {
            "clipped_window": p11_window,
            "source": "planning_scene",
            "scene_version": SCENE_VERSION,
            "scene_id": p11_scene_id,
            "width": p11_degraded["width"],
            "height": p11_degraded["height"],
            "resolution_m": p11_resolution,
            "origin": p11_origin,
            "degraded": False,
            "cells_equal": True,
            "scene_id_equal": True,
        },
        "notes": (
            "地图服务预览与规划服务输入快照必须对同一有效输入逐格一致（协议 §5 跨服务"
            "一致）；窗口 (0,0)-(20,20)、障碍 obs-a/obs-b 均在 v3 地图边界内，"
            "scene_id 由规范序列化独立复算。"
        ),
    })

    # P12：请求乱序陈旧拒绝（协议 §6 规则 1–2）
    cases.append({
        "id": "P12",
        "name": "stale_seq_order",
        "kind": "stale",
        "client": {"source": "planning_scene", "initial_latest_seq": 0},
        "events": [
            {"step": 1, "action": "request", "source": "planning_scene",
             "request_seq": 1, "window": _window(0.0, 0.0, 3.0, 3.0)},
            {"step": 2, "action": "request", "source": "planning_scene",
             "request_seq": 2, "window": _window(0.0, 0.0, 6.0, 6.0)},
            {"step": 3, "action": "response", "source": "planning_scene",
             "request_seq": 2, "window": _window(0.0, 0.0, 6.0, 6.0)},
            {"step": 4, "action": "response", "source": "planning_scene",
             "request_seq": 1, "window": _window(0.0, 0.0, 3.0, 3.0)},
        ],
        "expected": {
            "latest_seq": 2,
            "applied_request_seq": 2,
            "applied_window": _window(0.0, 0.0, 6.0, 6.0),
            "stale_ignored": 1,
            "discarded_request_seq": [1],
        },
        "notes": "B(seq=2) 先到应用；A(seq=1) 后到视为陈旧丢弃并计数（协议 §6 规则 1–2）。",
    })

    # P13：地图版本与场景标识陈旧（协议 §6 规则 3–4）
    p13_preview_window = _window(0.0, 0.0, 6.0, 6.0)
    p13_preview_obstacles = [
        {"id": "obs-preview", "type": "rectangle", "x_min_m": 1.0, "y_min_m": 1.0,
         "x_max_m": 2.0, "y_max_m": 2.0},
    ]
    p13_plan_obstacles = [
        {"id": "obs-plan", "type": "rectangle", "x_min_m": 3.0, "y_min_m": 3.0,
         "x_max_m": 4.0, "y_max_m": 4.0},
    ]
    p13_preview_id = scene_id_of(map_id, map_version, p13_preview_window, 0.1, 60, 60,
                                 p11_origin, p13_preview_obstacles)
    p13_plan_id = scene_id_of(map_id, map_version, p13_preview_window, 0.1, 60, 60,
                              p11_origin, p13_plan_obstacles)
    if p13_preview_id == p13_plan_id:
        raise FixtureError("P13 子例 (b) 需要两个不同 scene_id")
    cases.append({
        "id": "P13",
        "name": "stale_map_and_scene",
        "kind": "stale",
        "subcases": [
            {
                "name": "stale_map_version",
                "kind": "map_version_mismatch",
                "current_map": {"map_id": map_id, "map_version": map_version},
                "response": {
                    "map_id": map_id,
                    "map_version": "0" * 64,
                    "request_seq": 7,
                    "window": _window(0.0, 0.0, 3.0, 3.0),
                },
                "expected": {
                    "discard": True,
                    "reason": "map_version_mismatch",
                    "applied": False,
                },
            },
            {
                "name": "stale_scene_id",
                "kind": "scene_id_mismatch",
                "current_preview": {
                    "scene_id": p13_preview_id,
                    "scene_version": SCENE_VERSION,
                    "window": p13_preview_window,
                    "obstacles": p13_preview_obstacles,
                },
                "plan_result": {
                    "scene_id": p13_plan_id,
                    "scene_version": SCENE_VERSION,
                    "window": p13_preview_window,
                    "obstacles": p13_plan_obstacles,
                },
                "expected": {
                    "overwrite_preview": False,
                    "label": "规划实际场景",
                    "preview_scene_id": p13_preview_id,
                    "plan_scene_id": p13_plan_id,
                },
            },
        ],
        "notes": (
            "(a) map_version 不等时即使 seq 最新也丢弃（协议 §6 规则 3）；(b) 历史 "
            "scene_id 只进入“规划实际场景”面板，不覆盖当前预览（协议 §6 规则 4）。"
        ),
    })

    document = {
        "schema_version": 1,
        "type": "grid_protocol_cases",
        "fixture_version": "v6",
        "grid_protocol_version": GRID_PROTOCOL_VERSION,
        "scene_rules_version": SCENE_VERSION,
        "map": {"map_id": map_id, "map_version": map_version},
        "cases": cases,
        "notes": (
            "P1–P13 共 13 个协议用例；错误码取协议 §8 稳定字符串。P8/P9/P10 的降级与"
            "guard 结果、P11/P13 的 scene_id 由生成器从算法独立复算，不抄录文档数值。"
            "服务侧语义复核属 X-M1。"
        ),
    }
    return dump_json(document)


# ---------------------------------------------------------------------------
# v6 README / index
# ---------------------------------------------------------------------------

README_TEXT = """# tests/fixtures/v6：E0 固定夹具（v6 修订版：栅格协议夹具与显示档）

本目录保存跨组件复用的版本化测试输入。v6 在 v5 基础上新增/修订 `grids/` 集合（栅格协议
夹具、手算三态夹具与显示档快照）；其余文件与 v5 逐字节一致（不含 v5 清单）。本目录由
`tools/grid_fixtures.py` 确定性生成并自检；`fixtures freeze/verify`（哈希清单校验）经统一
入口执行。手算数值与显示档 sha256/spot_checks 均由生成器独立复算核对，未抄录文档数值。

## v6 修订说明（相对 v5，2026-10-12）

v6 = v5 全量复制（不含 v5 的 `fixture_manifest.sha256.json`，由 v6 freeze 重新生成）
+ 本表修订；v1–v5 保持冻结不动。

| 文件 | v6 变更 | 原因 |
| --- | --- | --- |
| `grids/minimal.json` | 与 v5 逐字节一致（复制） | 保持既有公共契约测试与消费方路径 |
| `grids/rotated_origin_ccw.json`、`rotated_origin_cw.json`、`edge_cells.json`、`unknown_edges.json`、`obstacle_overlay.json`、`coord_conversion.json`、`pattern_64.json` | 新增（手算 `grid_view` 夹具 G1–G8 中的 7 个） | 旋转原点、边缘单元、未知、障碍叠加、行列↔世界坐标 |
| `grids/snapshot_small.json` | 新增（小型 `grid_snapshot`，G7） | 协议解码黄金样例（base64 三态 + 长度自检） |
| `grids/protocol_cases.json` | 新增（P1–P13） | 空窗口、超限/降级、预览 vs 规划逐格一致、请求乱序与陈旧拒绝 |
| `grids/display_spec.json`、`display_256.json`、`display_512.json`、`display_1024.json` | 新增（显示档 + 生成规范） | 显示性能档位 256²/512²/1024² |
| `index.json`、`README.md` | 修订（v6 说明、`grids` 集合描述、revision_history/notes） | 版本化修订记录（v5 体例） |
| `fixture_manifest.sha256.json` | 由 freeze 重新生成 | 清单不含自身；verify 拒绝清单外文件 |
| 其余文件 | 与 v5 逐字节一致 | 避免无关变更 |

### SHA256 寻址归档约定（§9，v5 起）

- 单文件 >10 MiB 的生成输入存放于 `artifacts/data/sha256/<sha256>/`（不入 Git）：
  `<scale>.osm`、`<scale>.sha256`（sha256sum 格式）、`<scale>.meta.json`（生成规则、
  spec archive 记录与校验结果）；`specs.json` 的 `archive.relative_path` 以仓库根为基准，
  `archive.sha256` 为文件字节哈希。
- 离线归档必须包含被引用文件；生成器每次生成后校验生成文件 SHA256 与 `archive.sha256`
  一致（不一致时报错退出，不静默继续）。
- v6 显示档最大约 1.4 MiB（1024²），直接入库，无归档需求。

消费方迁移：

| 消费方 | v6 使用 | 回退/说明 |
| --- | --- | --- |
| X-W1 固定夹具渲染 | `grids/display_*`、`snapshot_small.json`、手算夹具 | 回退 v5 `grids/minimal.json`；不依赖任何服务 |
| X-M1/X-W2 协议联调 | `grids/protocol_cases.json`、`snapshot_small.json` + v3 地图夹具 | 地图仍用 `tests/fixtures/v3/map/mapping_example.osm` |
| 公共契约测试 | 现有版本路径 | v6 `minimal.json` 与 v5 逐字节一致，不要求迁移 |
| `planner_map_bench` | 维持 v5 `benchmarks/map/specs.json` | v6 benchmarks 与 v5 逐字节一致，优先级不变 |
| Web W4 回放 | 维持 v5 `playback/` | v6 逐字节复制 |

## 目录结构

| 路径 | 内容 | 状态 |
| --- | --- | --- |
| `index.json` | 夹具索引：集合清单与说明；不内嵌哈希 | v6 修订 |
| `grids/minimal.json` | GridView 最小样例（20×20、0.1 m，三态 + 边界） | v5 复制 |
| `grids/rotated_origin_ccw.json` | G1 旋转原点（CCW，π/2，4×4、0.5 m） | v6 新增 |
| `grids/rotated_origin_cw.json` | G2 旋转原点（CW，-π/2，4×4、0.5 m） | v6 新增 |
| `grids/edge_cells.json` | G3 边缘单元与越界（4×4、1.0 m） | v6 新增 |
| `grids/unknown_edges.json` | G4 unknown 与三态（5×5、0.5 m） | v6 新增 |
| `grids/obstacle_overlay.json` | G5 障碍叠加（6×4、1.0 m，矩形 + 凸多边形） | v6 新增 |
| `grids/coord_conversion.json` | G6 行列↔世界坐标（6×5、0.25 m） | v6 新增 |
| `grids/snapshot_small.json` | G7 小型 `grid_snapshot`（3×2、0.1 m，base64 三态） | v6 新增 |
| `grids/pattern_64.json` | G8 64² 正确性小图（0.1 m，三态 + 边界 + %5 规则） | v6 新增 |
| `grids/protocol_cases.json` | P1–P13 协议用例（错误/降级/一致性/陈旧） | v6 新增 |
| `grids/display_spec.json` | 显示档生成规范（splitmix64_thresholds_v1 + sha256 + spot_checks） | v6 新增 |
| `grids/display_256.json` | 显示档 256²、0.1 m、seed 20261012 | v6 新增 |
| `grids/display_512.json` | 显示档 512²、0.1 m、seed 20261013 | v6 新增 |
| `grids/display_1024.json` | 显示档 1024²、0.1 m、seed 20261014 | v6 新增 |
| `vehicles/ paths/ collision/ serialization/ benchmarks/ playback/ map/` | 与 v5 逐字节一致 | v5 复制 |

## JSON 顶层约定

- 每个 JSON 文件为 UTF-8；顶层含整数 `schema_version: 1` 与字符串 `type`。
- 对象 ID 一律使用字符串；枚举使用字符串；拒绝 NaN/Infinity；数值保存足以往返 double 的精度。
- 单位后缀约定：`_m`（米）、`_rad`（弧度）、`_s`（秒）、`_mps`、`_mps2`、`_inv_m`（1/米，曲率）。
- 必填字段缺失或主版本未知应报错；未知可选字段允许读取，但不得改变已有字段含义。
- 引用其他夹具文件时，除非文件内另有说明，使用以 `tests/fixtures/v6/` 为根的相对路径；
  `playback/index.json` 中的文件名相对 `playback/` 目录。
- `playback/` 内文件哈希对保存后的原始文件字节计算（sha256sum 格式小写十六进制），
  不要求不同语言生成相同文本布局。
- 栅格：`grid_view` 用整数数组（行主序，0=free、1=occupied、2=unknown）；`grid_snapshot`
  用 `base64_u8`（协议 §2.3）。`origin` 为 cell (0,0) 左下角位姿（ROS OccupancyGrid 约定），
  cell 中心 = `origin + R(yaw)·((c+0.5)·res, (r+0.5)·res)`；unknown 与地图外视为占据。

## 状态与待补

- `grids/`：G1–G8 与 P1–P13 由生成器输出后通过冻结字段校验与独立复算；显示档
  sha256/spot_checks 强制复验；X-W1 渲染验收抽查 `snapshot_small.json` 与手算夹具
  （图例/坐标/点击查询），X-M1 复核 P1–P13 服务侧语义。
- 计划文本偏差（已在生成器与夹具 notes 记录）：G5 `obs-d` 三角形计划底边 y=2.5 与
  第 2 行单元中心共线，按闭合边界语义会多占 (2,0)，与计划期望 8 格不符；生成器以
  计划期望 cells 为准，底边上移 0.1 m 为 y=2.6。
- `paths/`、`collision/`、`benchmarks/`、`playback/`、`map/`：与 v5 逐字节一致，状态
  沿用 v5 README。
- 本 README 描述的“生成”“自检”均为夹具生成状态；不表示任何服务已加载或算法已验证。

## 合成数据声明

`grids/` 中显示档与手算夹具为合成显示数据（`source: "fixture"` 或 `grid_view`），仅作
显示/解码/协议用例测试，不能作为算法正确性证据，也不代表任何上游实现的实际输出。
`playback/` 下全部文件标记 `synthetic: true`；`paths/`、`benchmarks/`、`collision/` 中的
数据为确定性测试输入或生成规范，同样不构成算法正确性结论。
"""


def build_readme():
    return README_TEXT


def build_index(v5_index):
    index = {
        "schema_version": 1,
        "type": "fixture_index",
        "fixture_version": "v6",
        "revision_of": "v5",
        "revision_summary": (
            "v6 = v5 全量复制（不含 v5 清单）+ grids/ 集合新增：手算 grid_view 夹具 "
            "G1–G8（含旋转原点/边缘/未知/障碍叠加/坐标换算）、协议用例 P1–P13、"
            "小型 grid_snapshot 解码样例与显示档 256²/512²/1024²（splitmix64_thresholds_v1，"
            "seed 20261012/20261013/20261014）；其余文件与 v5 逐字节一致，v1–v5 保持冻结不动。"
        ),
        "revision_history": [
            "v6：grids/ 集合新增（X-W1 生成）：手算 G1–G8、协议用例 P1–P13、"
            "snapshot_small 解码样例、display_spec 与 display_256/512/1024 显示档；"
            "显示档 sha256 与 spot_checks 生成时回填并强制复算；其余文件与 v5 逐字节一致。",
        ],
        "sets": [],
        "notes": [],
    }
    for entry in v5_index.get("revision_history", []):
        index["revision_history"].append(entry)
    for entry in v5_index.get("sets", []):
        item = dict(entry)
        if item.get("id") == "grids":
            item["description"] = (
                "栅格夹具：v5 minimal.json（逐字节复制）+ 手算三态 grid_view（旋转原点/"
                "边缘/未知/障碍叠加/坐标换算/64² 正确性小图）+ grid_snapshot 解码样例 + "
                "协议用例 P1–P13 + 显示档 256²/512²/1024² 与生成规范。"
            )
            item["kind"] = "grid_fixtures"
        index["sets"].append(item)
    index["notes"] = [
        (
            "v6 修订（相对 v5）：grids/ 新增由 tools/grid_fixtures.py 确定性生成 —— 手算 "
            "G1–G8（rotated_origin_ccw/cw、edge_cells、unknown_edges、obstacle_overlay、"
            "coord_conversion、snapshot_small、pattern_64）、协议用例 protocol_cases（P1–P13）、"
            "显示档 display_256/512/1024 与 display_spec（splitmix64_thresholds_v1，seed "
            "20261012/20261013/20261014，sha256/spot_checks 强制复算）。grids/minimal.json "
            "与 v5 逐字节一致；其余集合与 v5 逐字节一致。消费方：X-W1 固定夹具渲染、"
            "X-M1/X-W2 协议联调（地图仍用 v3 夹具）。"
        ),
        (
            "v6 生成器已知计划偏差：G5 obs-d 三角形计划底边 y=2.5 与第 2 行单元中心共线，"
            "按闭合边界语义会多占 (2,0)，与计划期望 8 格不符；生成器以计划期望 cells 为准，"
            "底边上移 0.1 m 为 y=2.6（详见夹具 notes 与 tools/grid_fixtures.py 文档）。"
        ),
    ]
    for entry in v5_index.get("notes", []):
        index["notes"].append(entry)
    return index


# ---------------------------------------------------------------------------
# 文档集合构建、写入与校验
# ---------------------------------------------------------------------------


def build_grid_documents(repo_root):
    """构建 grids/ 下全部文档；返回 [(rel, text)]（含 minimal 之外的全部新增/修订）。"""
    repo_root = Path(repo_root)
    v5_grids = repo_root / FIXTURES_REL / "v5" / "grids"
    documents = []
    # minimal.json 由 v5 复制（在 build_documents 中统一处理），此处不重复生成。
    for builder in HAND_FIXTURE_BUILDERS:
        rel, text = builder()
        documents.append((rel, text))
    protocol = build_protocol_cases(repo_root)
    documents.append(("grids/protocol_cases.json", protocol))
    display_results = build_display_documents()
    for result in display_results:
        documents.append((result["rel"], result["text"]))
    documents.append(("grids/display_spec.json", build_display_spec(display_results)))
    # 稳定性检查：v5 minimal 必须存在且将逐字节复制
    if not (v5_grids / "minimal.json").is_file():
        raise FixtureError("缺少 v5 grids/minimal.json")
    return documents


def build_documents(repo_root, use_cache=True):
    """v6 全量文档：v5 复制（除清单/README/index）+ grids 新增 + 修订 README/index。

    同一进程内重复调用（测试/校验）命中缓存；字节内容不可变，缓存安全。
    """
    repo_root = Path(repo_root)
    cache_key = str(repo_root.resolve())
    if use_cache and cache_key in _BUILD_CACHE:
        return _BUILD_CACHE[cache_key]
    v5_root = repo_root / FIXTURES_REL / "v5"
    if not v5_root.is_dir():
        raise FixtureError("缺少 v5 夹具目录: %s" % v5_root)
    documents = []
    for full in sorted(v5_root.rglob("*")):
        if not full.is_file():
            continue
        rel = full.relative_to(v5_root).as_posix()
        if rel in (MANIFEST_NAME, "README.md", "index.json"):
            continue
        documents.append((rel, full.read_bytes()))
    for rel, text in build_grid_documents(repo_root):
        documents.append((rel, text.encode("utf-8")))
    v5_index = read_json(v5_root / "index.json")
    documents.append(("README.md", build_readme().encode("utf-8")))
    documents.append(("index.json", dump_json(build_index(v5_index)).encode("utf-8")))
    documents.sort(key=lambda item: item[0])
    if use_cache:
        _BUILD_CACHE[cache_key] = documents
    return documents


_BUILD_CACHE = {}


def _write_documents(root, documents, force=False):
    root = Path(root)
    expected = {rel for rel, _ in documents}
    if root.exists() and any(root.iterdir()):
        if not force:
            raise FixtureError(
                "目标目录已存在且非空: %s（如需覆盖请加 --force；清单 %s 不受影响）"
                % (root, MANIFEST_NAME))
        for full in sorted(root.rglob("*")):
            if not full.is_file():
                continue
            rel = full.relative_to(root).as_posix()
            if rel == MANIFEST_NAME or rel in expected:
                continue
            full.unlink()
    root.mkdir(parents=True, exist_ok=True)
    for rel, data in documents:
        target = root / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)


def compare_tree(root, documents, repo_root):
    """与预期文档逐字节比较；返回 problems 列表。"""
    root = Path(root)
    problems = []
    expected = dict(documents)
    if not root.is_dir():
        return ["目标目录不存在: %s" % root]
    for rel, data in documents:
        full = root / rel
        if not full.is_file():
            problems.append("缺少文件: %s" % rel)
            continue
        actual = full.read_bytes()
        if actual != data:
            problems.append(
                "内容不一致: %s（期望 %d B sha256 %s / 实际 %d B sha256 %s）"
                % (rel, len(data), hashlib.sha256(data).hexdigest(),
                   len(actual), hashlib.sha256(actual).hexdigest()))
    on_disk = []
    for full in sorted(root.rglob("*")):
        if full.is_file():
            rel = full.relative_to(root).as_posix()
            if rel != MANIFEST_NAME:
                on_disk.append(rel)
    for rel in on_disk:
        if rel not in expected:
            problems.append("清单外额外文件: %s" % rel)
    problems.extend(revalidate_grids(root, repo_root))
    return problems


def revalidate_grids(root, repo_root):
    """对落盘 grids/ 文档做独立复算校验（不复用内存文档）。"""
    root = Path(root)
    problems = []
    grids = root / "grids"

    def load(rel):
        path = grids / rel
        if not path.is_file():
            problems.append("缺少 grids/%s" % rel)
            return None
        try:
            return read_json(path)
        except ValueError as exc:
            problems.append("grids/%s 不是有效 JSON: %s" % (rel, exc))
            return None

    def check(condition, message):
        if not condition:
            problems.append(message)

    # minimal 与 v5 逐字节一致
    v5_minimal = Path(repo_root) / FIXTURES_REL / "v5" / "grids" / "minimal.json"
    if v5_minimal.is_file() and (grids / "minimal.json").is_file():
        check(v5_minimal.read_bytes() == (grids / "minimal.json").read_bytes(),
              "grids/minimal.json 与 v5 不一致")

    # G1 / G2 旋转原点
    for rel, expected_checks in (
        ("rotated_origin_ccw.json", (
            ("grid_rotated_origin_ccw", 4, 4, 0.5, math.pi / 2.0),
            ((0, 0), 1, (9.75, -4.75)),
            ((1, 2), 2, (9.25, -3.75)),
            ((3, 3), 1, (8.25, -3.25)),
        )),
        ("rotated_origin_cw.json", (
            ("grid_rotated_origin_cw", 4, 4, 0.5, -math.pi / 2.0),
            ((0, 0), 0, (-2.75, 1.75)),
            ((0, 3), 1, (-2.75, 0.25)),
            ((2, 1), 2, (-1.75, 1.25)),
            ((3, 0), 1, (-1.25, 1.75)),
            ((3, 3), 0, (-1.25, 0.25)),
        )),
    ):
        doc = load(rel)
        if doc is None:
            continue
        header = expected_checks[0]
        check(doc.get("type") == "grid_view" and doc.get("map_id") == header[0]
              and doc.get("width") == header[1] and doc.get("height") == header[2]
              and doc.get("resolution_m") == header[3], "grids/%s 头部字段不一致" % rel)
        origin = doc["origin"]
        check(abs(abs(origin["yaw_rad"]) - math.pi / 2.0) < 1e-12, "grids/%s yaw 非 ±π/2" % rel)
        cells = doc["cells"]
        check(len(cells) == doc["width"] * doc["height"], "grids/%s cells 长度不一致" % rel)
        for (row, col), state, center in expected_checks[1:]:
            check(cells[row * doc["width"] + col] == state,
                  "grids/%s cell (%d,%d) 状态不一致" % (rel, row, col))
            actual = cell_center(origin, doc["resolution_m"], row, col)
            check(abs(actual[0] - center[0]) < 1e-9 and abs(actual[1] - center[1]) < 1e-9,
                  "grids/%s cell (%d,%d) 中心不一致: %r" % (rel, row, col, actual))
        if rel == "rotated_origin_ccw.json":
            corners = cell_corners(origin, doc["resolution_m"], 0, 0)
            expected_corners = [(10.0, -5.0), (10.0, -4.5), (9.5, -4.5), (9.5, -5.0)]
            for actual, wanted in zip(corners, expected_corners):
                check(abs(actual[0] - wanted[0]) < 1e-9 and abs(actual[1] - wanted[1]) < 1e-9,
                      "grids/%s (0,0) 四角不一致: %r" % (rel, corners))

    # G3 边缘单元
    doc = load("edge_cells.json")
    if doc is not None:
        cells = doc["cells"]
        width = doc["width"]
        for row, col in ((0, 0), (0, 3), (3, 0), (3, 3)):
            check(cells[row * width + col] == 1, "edge_cells 角单元 (%d,%d) 非占据" % (row, col))
        check(sum(cells) == 4, "edge_cells 占据数 != 4")
        for row, col in ((-1, 0), (0, -1), (4, 0), (0, 4)):
            check(not in_bounds(width, doc["height"], row, col),
                  "edge_cells in_bounds(%d,%d) 应为 false" % (row, col))

    # G4 unknown 三态
    doc = load("unknown_edges.json")
    if doc is not None:
        cells = doc["cells"]
        width = doc["width"]
        check(cells.count(2) == 8, "unknown_edges unknown 数 != 8")
        check(cells.count(1) == 2, "unknown_edges occupied 数 != 2")
        check(cells[0 * width + 4] == 2 and cells[4 * width + 0] == 2, "unknown_edges 孤立 unknown 不一致")
        for (row, col), center in (((1, 1), (-1.25, -1.25)), ((0, 4), (0.25, -1.75)),
                                   ((4, 0), (-1.75, 0.25)), ((3, 3), (-0.25, -0.25))):
            actual = cell_center(doc["origin"], doc["resolution_m"], row, col)
            check(abs(actual[0] - center[0]) < 1e-9 and abs(actual[1] - center[1]) < 1e-9,
                  "unknown_edges (%d,%d) 中心不一致: %r" % (row, col, actual))

    # G5 障碍叠加：从障碍几何独立复算 cells
    doc = load("obstacle_overlay.json")
    if doc is not None:
        recomputed = rasterize_obstacles(G5_OBSTACLES, doc["width"], doc["height"],
                                         doc["resolution_m"], doc["origin"])
        check(recomputed == doc["cells"], "obstacle_overlay cells 与障碍复算不一致")
        check(sum(recomputed) == 8, "obstacle_overlay 占据数 != 8")
        for row, col in ((0, 4), (0, 5), (1, 1), (1, 2), (2, 1), (2, 2), (2, 5), (3, 1)):
            check(recomputed[row * doc["width"] + col] == 1,
                  "obstacle_overlay 期望占据 (%d,%d) 未占据" % (row, col))
        check(2 not in recomputed, "obstacle_overlay 不应含 unknown")

    # G6 坐标换算
    doc = load("coord_conversion.json")
    if doc is not None:
        origin = doc["origin"]
        resolution = doc["resolution_m"]
        forward = {
            (0, 0): (1.625, -1.875), (0, 5): (2.875, -1.875),
            (4, 0): (1.625, -0.875), (4, 5): (2.875, -0.875),
            (2, 2): (2.125, -1.375),
        }
        for (row, col), center in forward.items():
            actual = cell_center(origin, resolution, row, col)
            check(abs(actual[0] - center[0]) < 1e-9 and abs(actual[1] - center[1]) < 1e-9,
                  "coord_conversion 正向 (%d,%d) 不一致: %r" % (row, col, actual))
            back = world_to_cell(origin, resolution, center[0], center[1])
            check(back == (row, col), "coord_conversion 往返 (%d,%d) -> %r" % (row, col, back))
        inverse = {
            (1.5, -2.0): (0, 0),
            (1.7499, -1.7501): (0, 0),
            (1.75, -1.75): (1, 1),
            (2.9999, -0.7501): (4, 5),
        }
        for point, expected in inverse.items():
            actual = world_to_cell(origin, resolution, point[0], point[1])
            check(actual == expected, "coord_conversion 反向 %r -> %r != %r" % (point, actual, expected))
        check(world_to_cell(origin, resolution, 3.0, -0.75) == (5, 6),
              "coord_conversion (3.0,-0.75) 应出界为 (5,6)")
        check(not in_bounds(doc["width"], doc["height"], *world_to_cell(origin, resolution, 3.0, -0.75)),
              "coord_conversion 出界判定失败")

    # G7 快照解码
    doc = load("snapshot_small.json")
    if doc is not None:
        check(doc.get("type") == "grid_snapshot" and doc.get("source") == "fixture",
              "snapshot_small 类型/来源不一致")
        check(doc.get("encoding") == "base64_u8", "snapshot_small encoding 不一致")
        check(doc.get("map_id") == "grid_snapshot_small" and doc.get("map_version") == "v6",
              "snapshot_small map 身份不一致")
        try:
            decoded = decode_cells(doc["cells"], doc["width"], doc["height"])
            check(decoded == [0, 1, 0, 0, 0, 2], "snapshot_small 解码字节不一致: %r" % decoded)
        except FixtureError as exc:
            problems.append("snapshot_small 解码失败: %s" % exc)
        check(doc["window"] == {"min_x_m": 0.0, "min_y_m": 0.0, "max_x_m": 0.3, "max_y_m": 0.2},
              "snapshot_small window 不一致")
        check(doc["requested_resolution_m"] == 0.1 and doc["effective_resolution_m"] == 0.1
              and doc["resolution_m"] == 0.1 and doc["degraded"] is False,
              "snapshot_small 分辨率/降级字段不一致")

    # G8 规则与计数
    doc = load("pattern_64.json")
    if doc is not None:
        width = doc["width"]
        cells = doc["cells"]
        counts = {0: 0, 1: 0, 2: 0}
        for index, state in enumerate(cells):
            row, col = divmod(index, width)
            if row == 0 or row == width - 1 or col == 0 or col == width - 1:
                expected_state = 1
            elif row == col:
                expected_state = 2
            elif (row + col) % 5 == 0:
                expected_state = 1
            else:
                expected_state = 0
            check(state == expected_state,
                  "pattern_64 (%d,%d) 状态 %d != 规则 %d" % (row, col, state, expected_state))
            counts[state] += 1
        check(counts == {0: 3026, 1: 1008, 2: 62},
              "pattern_64 计数不一致: %r" % counts)
        spots = {(0, 0): 1, (1, 1): 2, (1, 2): 0, (1, 4): 1, (2, 3): 1,
                 (31, 31): 2, (31, 32): 0, (62, 62): 2, (62, 60): 0, (63, 63): 1}
        for (row, col), state in spots.items():
            check(cells[row * width + col] == state,
                  "pattern_64 抽查 (%d,%d) 不一致" % (row, col))

    # 显示档：全量复算 states、sha256 与 spot_checks
    spec = load("display_spec.json")
    if spec is not None:
        check(spec.get("type") == "grid_display_spec", "display_spec 类型不一致")
        tiers = {tier["name"]: tier for tier in spec.get("tiers", [])}
        for tier in DISPLAY_TIERS:
            entry = tiers.get(tier["name"])
            if entry is None:
                problems.append("display_spec 缺少档位: %s" % tier["name"])
                continue
            rel = "%s.json" % tier["name"]
            doc = load(rel)
            if doc is None:
                continue
            try:
                decoded = decode_cells(doc["cells"], doc["width"], doc["height"])
            except FixtureError as exc:
                problems.append("grids/%s 解码失败: %s" % (rel, exc))
                continue
            states = generate_display_states(tier["seed"], tier["width"], tier["height"])
            check(decoded == list(states), "grids/%s 全量三态与算法复算不一致" % rel)
            actual_sha = hashlib.sha256((grids / rel).read_bytes()).hexdigest()
            check(actual_sha == entry["sha256"],
                  "grids/%s sha256 与 display_spec 不一致" % rel)
            check(doc["map_id"] == entry["map_id"],
                  "grids/%s map_id 与 display_spec 不一致" % rel)
            counts = {0: 0, 1: 0, 2: 0}
            for state in states:
                counts[state] += 1
            check(entry["state_counts"] == {
                "free": counts[0], "occupied": counts[1], "unknown": counts[2]},
                "display_spec %s state_counts 不一致" % tier["name"])
            check(doc["window"] == entry["window"], "grids/%s window 不一致" % rel)
            check(doc["requested_resolution_m"] == tier["resolution_m"]
                  and doc["effective_resolution_m"] == tier["resolution_m"]
                  and doc["degraded"] is False, "grids/%s 分辨率/降级不一致" % rel)
            check(doc.get("budget_cells") == DEFAULT_BUDGET_CELLS,
                  "grids/%s budget_cells 不一致" % rel)
        for spot in spec.get("spot_checks", []):
            name = spot["tier"]
            tier = next((item for item in DISPLAY_TIERS if item["name"] == name), None)
            if tier is None:
                problems.append("display_spec spot_check 档位未知: %s" % name)
                continue
            h = display_cell_hash(tier["seed"], tier["width"], spot["row"], spot["col"])
            check(str(h) == spot["h"] and h % THRESHOLD_MODULUS == spot["v"]
                  and threshold_state(h) == spot["state"],
                  "display_spec spot_check 复算不一致: %r" % spot)

    # 协议用例：降级与 scene_id 复算
    doc = load("protocol_cases.json")
    if doc is not None:
        check(doc.get("type") == "grid_protocol_cases"
              and doc.get("fixture_version") == "v6", "protocol_cases 类型/版本不一致")
        cases = {case["id"]: case for case in doc.get("cases", [])}
        check(len(cases) == 13, "protocol_cases 用例数 != 13")
        for case_id in ("P8", "P9"):
            case = cases.get(case_id)
            if case is None:
                continue
            result = degrade_resolution(case["request"]["window"],
                                        case["request"]["resolution_m"])
            expected = case["expected"]
            check(result is not None
                  and result["effective_resolution_m"] == expected["effective_resolution_m"]
                  and result["width"] == expected["width"]
                  and result["height"] == expected["height"]
                  and result["cells"] == expected["cells"]
                  and result["doublings"] == expected["guard_doublings"]
                  and expected["degraded"] is True,
                  "%s 降级复算不一致" % case_id)
        case = cases.get("P10")
        if case is not None:
            check(degrade_resolution(case["request"]["window"],
                                     case["request"]["resolution_m"]) is None,
                  "P10 guard 复算未拒绝")
        case = cases.get("P11")
        if case is not None:
            request = case["request"]
            recomputed = scene_id_of(
                request["map_id"], request["map_version"], request["window"],
                request["resolution_m"], case["expected"]["width"],
                case["expected"]["height"], case["expected"]["origin"],
                request["obstacles"])
            check(recomputed == case["expected"]["scene_id"],
                  "P11 scene_id 复算不一致: %s" % recomputed)
        case = cases.get("P13")
        if case is not None:
            subcases = {item["name"]: item for item in case["subcases"]}
            subcase = subcases.get("stale_scene_id")
            if subcase is not None:
                preview = subcase["current_preview"]
                plan = subcase["plan_result"]
                check(preview["scene_id"] == scene_id_of(
                    doc["map"]["map_id"], doc["map"]["map_version"], preview["window"],
                    0.1, 60, 60, {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
                    preview["obstacles"]), "P13 预览 scene_id 复算不一致")
                check(plan["scene_id"] == scene_id_of(
                    doc["map"]["map_id"], doc["map"]["map_version"], plan["window"],
                    0.1, 60, 60, {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
                    plan["obstacles"]), "P13 规划 scene_id 复算不一致")
                check(preview["scene_id"] != plan["scene_id"], "P13 两个 scene_id 必须不同")

    return problems


# ---------------------------------------------------------------------------
# 命令
# ---------------------------------------------------------------------------


def generate(version, output_root, repo_root, force=False, validate=True):
    if version not in SUPPORTED_VERSIONS:
        raise FixtureError("不支持的版本: %s（支持: %s）" % (version, ", ".join(SUPPORTED_VERSIONS)))
    verify_scene_id_example()
    documents = build_documents(repo_root)
    _write_documents(output_root, documents, force=force)
    summary = {
        "version": version,
        "output_root": str(output_root),
        "files": len(documents),
        "grid_files": sum(1 for rel, _ in documents if rel.startswith("grids/")),
    }
    if validate:
        problems = compare_tree(output_root, documents, repo_root)
        if problems:
            raise FixtureError("生成后自检失败（%d 项）:\n  - %s"
                               % (len(problems), "\n  - ".join(problems)))
        summary["validated"] = True
    return summary


def check(version, repo_root):
    if version not in SUPPORTED_VERSIONS:
        raise FixtureError("不支持的版本: %s" % version)
    verify_scene_id_example()
    root = Path(repo_root) / FIXTURES_REL / version
    if not root.is_dir():
        return False, ["目标目录不存在: %s" % root]
    documents = build_documents(repo_root)
    with tempfile.TemporaryDirectory(prefix="grid-fixtures-check-") as tmp:
        temp_root = Path(tmp) / version
        _write_documents(temp_root, documents, force=True)
        problems = compare_tree(root, documents, repo_root)
        if problems:
            return False, problems
        return True, []


def validate(version, repo_root):
    root = Path(repo_root) / FIXTURES_REL / version
    if not root.is_dir():
        return False, ["目标目录不存在: %s" % root]
    verify_scene_id_example()
    documents = build_documents(repo_root)
    problems = compare_tree(root, documents, repo_root)
    return (not problems), problems


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="grid_fixtures.py",
        description="v6 栅格夹具生成器（手算 G1–G8、协议用例 P1–P13、显示档 256²/512²/1024²）")
    parser.add_argument("command", nargs="?", choices=("generate", "check", "validate"),
                        default=None, help="generate / check / validate（缺省由 --check 决定）")
    parser.add_argument("--check", action="store_true",
                        help="等价于 check：临时目录重新生成并与仓库 v6 逐字节比较")
    parser.add_argument("--version", default="v6", help="夹具版本（当前仅 v6）")
    parser.add_argument("--output-dir", default=None, help="生成目标目录（默认 tests/fixtures/<version>）")
    parser.add_argument("--force", action="store_true", help="目标非空时覆盖（保留清单文件）")
    args = parser.parse_args(argv)

    repo_root = Path(__file__).resolve().parents[1]
    version = args.version
    command = args.command
    if command is None:
        command = "check" if args.check else "generate"

    try:
        if command == "generate":
            output_root = Path(args.output_dir) if args.output_dir else (
                repo_root / FIXTURES_REL / version)
            summary = generate(version, output_root, repo_root, force=args.force)
            print("grid_fixtures generate 完成: version=%s files=%d grids=%d -> %s"
                  % (version, summary["files"], summary["grid_files"], summary["output_root"]))
            if summary.get("validated"):
                print("生成后自检通过（独立复算：手算用例 + 显示档全量 + scene_id 示例）。")
            return 0
        if command == "check":
            ok, problems = check(version, repo_root)
            if ok:
                print("grid_fixtures check 通过: %s 与重新生成逐字节一致（不含清单）"
                      % (repo_root / FIXTURES_REL / version))
                return 0
            sys.stderr.write("grid_fixtures check 失败（%d 项）:\n" % len(problems))
            for problem in problems:
                sys.stderr.write("  - %s\n" % problem)
            return 1
        ok, problems = validate(version, repo_root)
        if ok:
            print("grid_fixtures validate 通过: %s（独立复算）"
                  % (repo_root / FIXTURES_REL / version))
            return 0
        sys.stderr.write("grid_fixtures validate 失败（%d 项）:\n" % len(problems))
        for problem in problems:
            sys.stderr.write("  - %s\n" % problem)
        return 1
    except FixtureError as exc:
        sys.stderr.write("grid_fixtures 错误: %s\n" % exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
