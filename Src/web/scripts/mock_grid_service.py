#!/usr/bin/env python3
"""X-W2 临时 mock 地图服务（planner_map_service 栅格端点契约的确定性替身；仅标准库）。

用途：CDP 集成检查中复现真实服务难以构造的场景：
  - 乱序响应（`--delay-first-grid-s` 延迟首个请求，验证晚到响应不覆盖新请求）；
  - 协议主版本不匹配（`--grid-protocol-version 2.0`，验证图层禁用可见状态）；
  - 服务端降级（`--max-cells` 小于窗口单元数，degraded=true 显示实际分辨率）。
**不是算法正确性证据**；正式验收以真实服务（Src/map）为准。

实现端点（与协议 §3.2/§3.3/§3.5 同形状）：
  GET  /health              含 grid_protocol_version / scene_rules_version
  GET  /map/info            与 v3 夹具同 map_id/map_version（供在线状态判定）
  POST /map/grid/driveable  source=driveable；回显 request_seq；单元 (0,0)=seq%3
  POST /map/scene/preview   source=planning_scene；窗口裁剪到地图边界；scene_id=SHA256

用法：
  python3 Src/web/scripts/mock_grid_service.py --port 8865 --map-version <hash> \
      [--grid-protocol-version 1.0] [--scene-rules-version scene_v1] \
      [--delay-first-grid-s 1.2] [--max-cells 4096]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# 与 tests/fixtures/v3/map/mapping_example.snapshot.json 一致的包围盒（mock 固定值）。
MAP_BOUNDS = {
    "min_x_m": -899.4944769139984,
    "min_y_m": -185.262257614173,
    "max_x_m": 2526.1362360325875,
    "max_y_m": 855.8350301543251,
}

_state = {
    "map_id": "mapping_example",
    "map_version": "",
    "grid_protocol_version": "1.0",
    "scene_rules_version": "scene_v1",
    "delay_first_grid_s": 0.0,
    "max_cells": 1024 * 1024,
}
_lock = threading.Lock()


def _envelope(doc_type: str) -> dict:
    return {"schema_version": 1, "type": doc_type}


def _error(status: int, code: str, reason: str) -> tuple[int, dict]:
    return status, {"schema_version": 1, "type": "error", "status": code, "reason": reason}


def _parse_body(raw: bytes) -> tuple[dict | None, tuple[int, dict] | None]:
    if not raw:
        return None, _error(400, "invalid_input", "parse_error")
    try:
        body = json.loads(raw.decode("utf-8"))
    except Exception:  # noqa: BLE001
        return None, _error(400, "invalid_input", "parse_error")
    if not isinstance(body, dict):
        return None, _error(400, "invalid_input", "parse_error")
    return body, None


def _parse_window(body: dict) -> tuple[dict | None, tuple[int, dict] | None]:
    window = body.get("window")
    if not isinstance(window, dict):
        return None, _error(400, "invalid_input", "missing_field")
    try:
        values = {key: float(window[key]) for key in ("min_x_m", "min_y_m", "max_x_m", "max_y_m")}
    except (KeyError, TypeError, ValueError):
        return None, _error(400, "invalid_input", "invalid_window")
    if not all(math.isfinite(value) for value in values.values()):
        return None, _error(400, "invalid_input", "non_finite_value")
    if not (values["min_x_m"] < values["max_x_m"] and values["min_y_m"] < values["max_y_m"]):
        return None, _error(400, "invalid_input", "empty_window")
    return values, None


def _parse_resolution(body: dict) -> tuple[float | None, tuple[int, dict] | None]:
    value = body.get("resolution_m")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None, _error(400, "invalid_input", "missing_field")
    resolution = float(value)
    if not math.isfinite(resolution):
        return None, _error(400, "invalid_input", "non_finite_value")
    if not resolution > 0:
        return None, _error(400, "invalid_input", "invalid_resolution")
    return resolution, None


def _parse_request_seq(body: dict) -> tuple[int | None, tuple[int, dict] | None]:
    value = body.get("request_seq")
    if value is None:
        return None, None
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None, _error(400, "invalid_input", "type_error")
    return value, None


def _fit_resolution(window: dict, requested: float) -> tuple[float, bool]:
    resolution = requested
    for _ in range(32):
        cols = math.ceil((window["max_x_m"] - window["min_x_m"]) / resolution)
        rows = math.ceil((window["max_y_m"] - window["min_y_m"]) / resolution)
        if cols * rows <= _state["max_cells"]:
            return resolution, resolution > requested
        resolution *= 2
    return resolution, True


def _make_cells(width: int, height: int, request_seq: int | None) -> str:
    import base64

    cells = bytearray(width * height)
    for row in range(height):
        for col in range(width):
            # 确定性占据图案（每 7 格一条对角线），便于 CDP 像素断言。
            if (row + col) % 7 == 0:
                cells[row * width + col] = 1
    if cells and request_seq is not None:
        # 把 request_seq 编码进单元 (0,0)：乱序用例验证最终展示的是最新 seq。
        cells[0] = request_seq % 3
    return base64.b64encode(bytes(cells)).decode("ascii")


def _snapshot_document(
    *,
    source: str,
    window: dict,
    requested_resolution: float,
    request_seq: int | None,
    scene_id: str | None,
) -> dict:
    effective, degraded = _fit_resolution(window, requested_resolution)
    width = max(1, math.ceil((window["max_x_m"] - window["min_x_m"]) / effective))
    height = max(1, math.ceil((window["max_y_m"] - window["min_y_m"]) / effective))
    doc = _envelope("grid_snapshot")
    doc.update(
        {
            "source": source,
            "map_id": _state["map_id"],
            "map_version": _state["map_version"],
            "window": {
                "min_x_m": window["min_x_m"],
                "min_y_m": window["min_y_m"],
                "max_x_m": window["max_x_m"],
                "max_y_m": window["max_y_m"],
            },
            "requested_resolution_m": requested_resolution,
            "effective_resolution_m": effective,
            "resolution_m": effective,
            "degraded": degraded,
            "width": width,
            "height": height,
            "origin": {"x_m": window["min_x_m"], "y_m": window["min_y_m"], "yaw_rad": 0.0},
            "encoding": "base64_u8",
            "cells": _make_cells(width, height, request_seq),
            "budget_cells": _state["max_cells"],
        }
    )
    if scene_id is not None:
        doc["scene_id"] = scene_id
        doc["scene_version"] = _state["scene_rules_version"]
    if request_seq is not None:
        doc["request_seq"] = request_seq
    return doc


def _scene_id(window: dict, resolution: float, obstacles: list) -> str:
    canonical = json.dumps(
        {
            "map_id": _state["map_id"],
            "map_version": _state["map_version"],
            "window": window,
            "resolution_m": resolution,
            "obstacles": obstacles,
            "scene_version": _state["scene_rules_version"],
        },
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?")[0]
        if path == "/health":
            doc = _envelope("health")
            doc.update(
                {
                    "status": "ok",
                    "map_id": _state["map_id"],
                    "map_version": _state["map_version"],
                    "uptime_s": 0.0,
                    "grid_protocol_version": _state["grid_protocol_version"],
                    "scene_rules_version": _state["scene_rules_version"],
                }
            )
            return self._send(200, doc)
        if path == "/map/info":
            doc = _envelope("map_info")
            doc.update(
                {
                    "map_id": _state["map_id"],
                    "map_version": _state["map_version"],
                    "counts": {"lanelets": 371, "areas": 76},
                    "bounds_m": dict(MAP_BOUNDS),
                }
            )
            return self._send(200, doc)
        return self._send(*_error(404, "invalid_input", f"no handler for GET {path}"))

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?")[0]
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length > 0 else b""
        body, failure = _parse_body(raw)
        if failure:
            return self._send(*failure)
        assert body is not None

        if path not in ("/map/grid/driveable", "/map/scene/preview"):
            return self._send(*_error(404, "invalid_input", f"no handler for POST {path}"))

        window, failure = _parse_window(body)
        if failure:
            return self._send(*failure)
        resolution, failure = _parse_resolution(body)
        if failure:
            return self._send(*failure)
        request_seq, failure = _parse_request_seq(body)
        if failure:
            return self._send(*failure)
        assert window is not None and resolution is not None

        # 乱序用例：延迟首个请求（request_seq==1）的响应，让更晚请求先返回。
        if request_seq == 1 and _state["delay_first_grid_s"] > 0:
            time.sleep(_state["delay_first_grid_s"])

        if path == "/map/grid/driveable":
            return self._send(
                200,
                _snapshot_document(
                    source="driveable",
                    window=window,
                    requested_resolution=resolution,
                    request_seq=request_seq,
                    scene_id=None,
                ),
            )

        # /map/scene/preview：窗口裁剪到地图边界（协议 §3.3 语义 1）。
        cropped = {
            "min_x_m": max(window["min_x_m"], MAP_BOUNDS["min_x_m"]),
            "min_y_m": max(window["min_y_m"], MAP_BOUNDS["min_y_m"]),
            "max_x_m": min(window["max_x_m"], MAP_BOUNDS["max_x_m"]),
            "max_y_m": min(window["max_y_m"], MAP_BOUNDS["max_y_m"]),
        }
        if not (cropped["min_x_m"] < cropped["max_x_m"] and cropped["min_y_m"] < cropped["max_y_m"]):
            return self._send(*_error(400, "invalid_input", "window_outside_map"))
        obstacles = body.get("obstacles") or []
        return self._send(
            200,
            _snapshot_document(
                source="planning_scene",
                window=cropped,
                requested_resolution=resolution,
                request_seq=request_seq,
                scene_id=_scene_id(cropped, resolution, obstacles),
            ),
        )

    def _send(self, status: int, body: dict) -> None:
        data = json.dumps(body).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args) -> None:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="X-W2 临时 mock 地图服务（非算法证据）")
    parser.add_argument("--port", type=int, default=8865)
    parser.add_argument("--map-id", default="mapping_example")
    parser.add_argument("--map-version", default="", help="当前地图版本（与快照一致才能通过客户端校验）")
    parser.add_argument("--grid-protocol-version", default="1.0")
    parser.add_argument("--scene-rules-version", default="scene_v1")
    parser.add_argument("--delay-first-grid-s", type=float, default=0.0)
    parser.add_argument("--max-cells", type=int, default=1024 * 1024)
    args = parser.parse_args()
    _state.update(
        {
            "map_id": args.map_id,
            "map_version": args.map_version,
            "grid_protocol_version": args.grid_protocol_version,
            "scene_rules_version": args.scene_rules_version,
            "delay_first_grid_s": args.delay_first_grid_s,
            "max_cells": args.max_cells,
        }
    )
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(
        f"mock_grid_service listening on 127.0.0.1:{args.port} "
        f"(map_version={args.map_version!r}, protocol={args.grid_protocol_version}, "
        f"scene_rules={args.scene_rules_version}, delay_first={args.delay_first_grid_s}s, "
        f"max_cells={args.max_cells})",
        flush=True,
    )
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
