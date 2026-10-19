#!/usr/bin/env python3
"""W5 临时 mock 规划服务（planner_plan_service 契约的确定性替身；仅标准库）。

用途：真实规划服务（Src/service/）就绪前，验证浏览器端提交/取消/状态/结果与
陈旧防护的完整流程。**不是算法正确性证据**；集成验收以真实服务为准。

实现端点（与契约一致）：
  GET  /health
  POST /plan/submit          校验必填字段与 algorithm_params 未知键；
                             地图版本不匹配 → 409 stale_map
  POST /plan/cancel
  GET  /plan/status/{job_id} 按时间推进 queued→running(stage)→succeeded
  GET  /plan/result/{job_id} 由请求起终位姿派生确定性几何（路径/走廊/轨迹/校验/统计）

用法：
  python3 Src/web/scripts/mock_plan_service.py --port 8766 --map-version <当前地图版本>
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

KNOWN_ALGORITHM_KEYS = {"hybrid_backend", "hybrid", "time_budget_s", "node_budget", "dftpav_time_budget_s", "corridor", "dftpav"}
STAGES = ["hybrid", "corridor", "optimize", "validate"]
STAGE_DURATION_S = 0.7
RUNNING_DELAY_S = 0.6
GRID_PROTOCOL_VERSION = "1.0"
SCENE_RULES_VERSION = "scene_v1"

_lock = threading.Lock()
_jobs: dict[str, dict] = {}
_job_seq = 0
_current_map_version = ""


def _error(status: int, code: str, reason: str) -> tuple[int, dict]:
    return status, {"schema_version": 1, "type": "error", "status": code, "reason": reason}


def _scene_id_for(request: dict) -> str:
    canonical = json.dumps(
        {
            "map_id": request.get("map_id"),
            "map_version": request.get("map_version"),
            "start": request.get("start"),
            "goal": request.get("goal"),
            "obstacles": request.get("obstacles") or [],
            "scene_version": SCENE_RULES_VERSION,
        },
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _scene_snapshot(job: dict) -> dict:
    """该次规划"实际使用"的输入快照（mock 确定性构造；形状与协议 §2 一致）。"""
    request = job["request"]
    start = request["start"]
    window = {
        "min_x_m": start["x_m"] - 4.0,
        "min_y_m": start["y_m"] - 3.0,
        "max_x_m": start["x_m"] + 4.0,
        "max_y_m": start["y_m"] + 3.0,
    }
    resolution = 1.0
    width, height = 8, 6
    cells = bytearray(width * height)
    for row in range(height):
        for col in range(width):
            if (row + col) % 5 == 0:
                cells[row * width + col] = 1
    return {
        "schema_version": 1,
        "type": "grid_snapshot",
        "source": "planning_scene",
        "scene_id": job["scene_id"],
        "scene_version": SCENE_RULES_VERSION,
        "map_id": request["map_id"],
        "map_version": request["map_version"],
        "window": window,
        "requested_resolution_m": resolution,
        "effective_resolution_m": resolution,
        "resolution_m": resolution,
        "degraded": False,
        "width": width,
        "height": height,
        "origin": {"x_m": window["min_x_m"], "y_m": window["min_y_m"], "yaw_rad": 0.0},
        "encoding": "base64_u8",
        "cells": base64.b64encode(bytes(cells)).decode("ascii"),
        "budget_cells": 16777216,
        "mock": True,
    }


def _validate_submit(body: dict) -> tuple[int, dict] | None:
    for field in ("request_id", "map_id", "map_version", "start", "goal", "vehicle_ref", "obstacles"):
        if field not in body:
            return _error(400, "invalid_input", f"missing_field: {field}")
    if body["map_version"] != _current_map_version:
        return _error(
            409,
            "stale_map",
            f"map_version mismatch: request {body['map_version']} != current {_current_map_version}",
        )
    params = body.get("algorithm_params") or {}
    if not isinstance(params, dict):
        return _error(400, "invalid_input", "invalid_field: algorithm_params")
    unknown = sorted(set(params.keys()) - KNOWN_ALGORITHM_KEYS)
    if unknown:
        return _error(400, "unknown_algorithm_parameter", f"unknown algorithm_params keys: {', '.join(unknown)}")
    start, goal = body["start"], body["goal"]
    for pose in (start, goal):
        if not isinstance(pose, dict) or not all(isinstance(pose.get(k), (int, float)) for k in ("x_m", "y_m", "yaw_rad")):
            return _error(400, "invalid_input", "invalid_field: start/goal pose")
    return None


def _build_result(job: dict) -> dict:
    request = job["request"]
    start = request["start"]
    goal = request["goal"]
    dx = goal["x_m"] - start["x_m"]
    dy = goal["y_m"] - start["y_m"]
    length = math.hypot(dx, dy) or 1.0
    ux, uy = dx / length, dy / length
    yaw = math.atan2(dy, dx)

    forward_end = 0.6 * length
    points_forward = []
    for index in range(5):
        fraction = index / 4
        s = forward_end * fraction
        points_forward.append(
            {
                "pose": {"x_m": start["x_m"] + ux * s, "y_m": start["y_m"] + uy * s, "yaw_rad": yaw},
                "s_m": s,
                "curvature_inv_m": 0.0,
                "direction": "forward",
            }
        )
    reverse_points = []
    for index in range(1, 4):
        s = forward_end + 0.1 * length * index
        reverse_points.append(
            {
                "pose": {
                    "x_m": start["x_m"] + ux * (forward_end - 0.1 * length * index),
                    "y_m": start["y_m"] + uy * (forward_end - 0.1 * length * index),
                    "yaw_rad": yaw,
                },
                "s_m": s,
                "curvature_inv_m": 0.0,
                "direction": "reverse",
            }
        )

    corridor = []
    for index in range(2):
        s0 = index * (length / 2)
        s1 = (index + 1) * (length / 2)
        base = start["x_m"] + ux * s0
        base_y = start["y_m"] + uy * s0
        end = start["x_m"] + ux * s1
        end_y = start["y_m"] + uy * s1
        corridor.append(
            {
                "s_start_m": s0,
                "s_end_m": s1,
                "vertices": [
                    {"x_m": base - uy * 2.0, "y_m": base_y + ux * 2.0},
                    {"x_m": end - uy * 2.0, "y_m": end_y + ux * 2.0},
                    {"x_m": end + uy * 2.0, "y_m": end_y - ux * 2.0},
                    {"x_m": base + uy * 2.0, "y_m": base_y - ux * 2.0},
                ],
            }
        )

    trajectory = []
    duration = 6.0
    for index in range(13):
        t = duration * index / 12
        if t <= 3.0:
            s = forward_end * (t / 3.0)
            velocity = 2.0 * min(t, 1.0, 3.0 - t) if 0 < t < 3.0 else 0.0
            direction_x = ux
            direction_y = uy
        else:
            back = 0.2 * length * ((t - 3.0) / 3.0)
            s = forward_end - back
            velocity = -1.0 * min(t - 3.0, 1.0, 6.0 - t) if 3.0 < t < 6.0 else 0.0
            direction_x = ux
            direction_y = uy
        trajectory.append(
            {
                "t_s": t,
                "pose": {"x_m": start["x_m"] + direction_x * s, "y_m": start["y_m"] + direction_y * s, "yaw_rad": yaw},
                "velocity_mps": velocity,
                "acceleration_mps2": 0.0,
                "curvature_inv_m": 0.0,
            }
        )

    def _polyline_length(points: list[dict]) -> float:
        total = 0.0
        for previous, current in zip(points, points[1:]):
            total += math.hypot(
                current["pose"]["x_m"] - previous["pose"]["x_m"],
                current["pose"]["y_m"] - previous["pose"]["y_m"],
            )
        return total

    path_length = _polyline_length(points_forward) + _polyline_length([points_forward[-1], *reverse_points])
    trajectory_length = _polyline_length(trajectory)
    params = request.get("algorithm_params") or {}
    backend = str(params.get("hybrid_backend", "kth_normalized"))

    # 与真实 planner_plan_service 一致的响应形状（hybrid.segments / statistics.hybrid 等）。
    response = {
        "schema_version": 1,
        "type": "plan_result",
        "job_id": job["job_id"],
        "job_seq": job["seq"],
        "request_id": request["request_id"],
        "map_id": request["map_id"],
        "map_version": request["map_version"],
        "status": "succeeded",
        "stage": "done",
        "reason": "",
        "hybrid": {
            "backend": backend,
            "segments": [{"points": points_forward}, {"points": reverse_points}],
            "statistics": {
                "backend_id": backend,
                "expanded_nodes": 2048,
                "collision_checks": 16384,
                "path_length_m": round(path_length, 6),
                "search_time_ms": 31.0,
                "postprocess_time_ms": 2.0,
            },
        },
        "corridor": {
            "id": "mock-corridor",
            "map_id": request["map_id"],
            "map_version": request["map_version"],
            "frame_id": "map",
            "polygons": corridor,
            "statistics": {"polygons": len(corridor), "construction_time_s": 0.012},
        },
        "trajectory": trajectory,
        "validation": {
            "valid": True,
            "reason": "",
            "detail": "",
            "first_failure": None,
            "checks": {
                "trajectory_points": len(trajectory),
                "corridor_samples": len(corridor),
                "collision_pose_checks": len(trajectory),
                "collision_motion_checks": max(0, len(trajectory) - 1),
            },
        },
        "statistics": {
            "hybrid_backend": backend,
            "stage": "done",
            "timings_s": {"hybrid": 0.031, "corridor": 0.012, "optimize": 0.044, "validate": 0.008},
            "hybrid": {
                "backend_id": backend,
                "expanded_nodes": 2048,
                "collision_checks": 16384,
                "path_length_m": round(path_length, 6),
                "search_time_ms": 31.0,
            },
            "corridor": {"polygons": len(corridor)},
            "optimize": {"iterations": 17},
            "validation": {"valid": True},
            "path_length_m": round(path_length, 6),
            "trajectory_length_m": round(trajectory_length, 6),
            "trajectory_points": len(trajectory),
            "hybrid_budget_s": params.get("time_budget_s"),
            "hybrid_node_budget": params.get("node_budget"),
        },
        "mock": True,
        "mock_note": "W5 临时 mock 服务结果（真实服务响应形状），仅用于显示流程验证，非算法正确性证据",
    }
    # 协议 §3.4：场景已构建即出现（成功与失败都出现）。
    response["scene_id"] = job["scene_id"]
    response["scene_version"] = SCENE_RULES_VERSION
    return response


def _job_view(job: dict) -> tuple[str, str]:
    if job["cancelled"]:
        return "cancelled", ""
    elapsed = time.time() - job["submitted_at"]
    if elapsed < RUNNING_DELAY_S:
        return "queued", ""
    stage_index = int((elapsed - RUNNING_DELAY_S) / STAGE_DURATION_S)
    if stage_index >= len(STAGES):
        return "succeeded", "done"
    return "running", STAGES[stage_index]


class Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?")[0]
        if path == "/health":
            return self._send(
                200,
                {
                    "schema_version": 1,
                    "type": "health",
                    "status": "ok",
                    "grid_protocol_version": GRID_PROTOCOL_VERSION,
                    "scene_rules_version": SCENE_RULES_VERSION,
                },
            )
        if path.startswith("/plan/scene/"):
            scene_id = path[len("/plan/scene/"):]
            with _lock:
                job = next((entry for entry in _jobs.values() if entry.get("scene_id") == scene_id), None)
            if not job:
                return self._send(*_error(404, "not_found", "unknown_scene"))
            return self._send(200, _scene_snapshot(job))
        if path.startswith("/plan/status/"):
            job_id = path[len("/plan/status/"):]
            with _lock:
                job = _jobs.get(job_id)
            if not job:
                return self._send(*_error(404, "invalid_input", f"unknown job {job_id}"))
            status, stage = _job_view(job)
            body = {"schema_version": 1, "type": "job_status", "job_id": job_id, "status": status, "stage": stage, "reason": job.get("reason", ""), "job_seq": job["seq"]}
            # 协议 §3.4：map 阶段（场景构建）完成后出现；之前缺省。
            if status != "queued":
                body["scene_id"] = job["scene_id"]
                body["scene_version"] = SCENE_RULES_VERSION
            return self._send(200, body)
        if path.startswith("/plan/result/"):
            job_id = path[len("/plan/result/"):]
            with _lock:
                job = _jobs.get(job_id)
            if not job:
                return self._send(*_error(404, "invalid_input", f"unknown job {job_id}"))
            status, _ = _job_view(job)
            if status != "succeeded":
                return self._send(*_error(409, "invalid_input", f"job not terminal: {status}"))
            return self._send(200, _build_result(job))
        return self._send(*_error(404, "invalid_input", f"no handler for GET {path}"))

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?")[0]
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length).decode("utf-8")) if length else {}
        except Exception as error:  # noqa: BLE001
            return self._send(*_error(400, "invalid_input", f"invalid JSON body: {error}"))
        if path == "/plan/submit":
            failure = _validate_submit(body)
            if failure:
                return self._send(*failure)
            global _job_seq
            with _lock:
                _job_seq += 1
                job_id = f"mock-job-{_job_seq:03d}"
                _jobs[job_id] = {
                    "job_id": job_id,
                    "seq": _job_seq,
                    "request": body,
                    "submitted_at": time.time(),
                    "cancelled": False,
                    "reason": "",
                    "scene_id": _scene_id_for(body),
                }
            return self._send(
                200,
                {
                    "schema_version": 1,
                    "type": "plan_job",
                    "job_id": job_id,
                    "job_seq": _job_seq,
                    "request_id": body["request_id"],
                    "map_id": body["map_id"],
                    "map_version": body["map_version"],
                    "status": "queued",
                },
            )
        if path == "/plan/cancel":
            job_id = body.get("job_id")
            with _lock:
                job = _jobs.get(str(job_id))
                if job:
                    job["cancelled"] = True
            if not job:
                return self._send(*_error(404, "invalid_input", f"unknown job {job_id}"))
            return self._send(200, {"schema_version": 1, "type": "job_status", "job_id": job_id, "status": "cancelled", "job_seq": job["seq"]})
        return self._send(*_error(404, "invalid_input", f"no handler for POST {path}"))

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
    parser = argparse.ArgumentParser(description="W5 临时 mock 规划服务（非算法证据）")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--map-version", default="", help="当前地图版本（提交版本不一致时返回 409 stale_map）")
    args = parser.parse_args()
    global _current_map_version
    _current_map_version = args.map_version
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"mock_plan_service listening on 127.0.0.1:{args.port} (map_version={args.map_version!r})", flush=True)
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
