#!/usr/bin/env python3
"""W2 集成冒烟：对比 planner_map_service 响应与 v3 地图快照（只读，仅标准库）。

检查项：
  1. /health 与 /map/info 包络、map_id/map_version 与对象计数；
  2. 抽样 lanelet 几何（属性精确相等；左右边界/中心线点数与坐标 ≤1e-9）；
  3. 点查询（中心线中点命中自身 lanelet）；
  4. 拓扑查询（ids ⊆ 快照 lanelet 集合）；
  5. 路由（本地找连通对，验证 lanelet_ids 包含起终且长度 > 0）；
  6. 窗口裁剪（window_m 回显、window_frame 原点=窗口最小角、路径点位于窗口局部系、
     lanelet_ids ⊆ 快照并包含目标 lanelet）；
  7. 错误语义（未知 ID → 404 invalid_input；缺字段 → 400）；
  8. /map/snapshot 计数与抽样几何一致性。

用法（经统一入口 exec 运行）：
  python3 Src/web/scripts/service_compare.py --base-url http://127.0.0.1:8765 \
    --snapshot tests/fixtures/v3/map/mapping_example.snapshot.json --output-dir <dir>
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path


def http_json(method: str, url: str, body: dict | None = None, timeout: float = 20.0):
    data = json.dumps(body).encode("utf-8") if body is not None else None
    headers = {"Accept": "application/json"}
    if data is not None:
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(url, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        raw = error.read()
        try:
            return error.code, json.loads(raw.decode("utf-8"))
        except Exception:
            return error.code, None


def close_enough(a: float, b: float, tolerance: float = 1e-9) -> bool:
    return abs(a - b) <= tolerance + tolerance * max(abs(a), abs(b))


def points_equal(actual: list, expected: list, tolerance: float = 1e-9) -> bool:
    if len(actual) != len(expected):
        return False
    for a, e in zip(actual, expected):
        if a.get("id", "") != e.get("id", ""):
            return False
        if not close_enough(a["x_m"], e["x_m"], tolerance) or not close_enough(a["y_m"], e["y_m"], tolerance):
            return False
    return True


def centerline_endpoints(lanelet: dict):
    centerline = lanelet.get("centerline") or []
    if len(centerline) < 2:
        return None, None
    return centerline[0], centerline[-1]


def find_connected_pair(lanelets: list[dict]):
    by_id = {item["id"]: item for item in lanelets}
    pairs: list[tuple[str, str]] = []
    for a in lanelets:
        _, a_end = centerline_endpoints(a)
        if a_end is None:
            continue
        for b in lanelets:
            if b["id"] == a["id"]:
                continue
            b_start, _ = centerline_endpoints(b)
            if b_start is None:
                continue
            if math.hypot(b_start["x_m"] - a_end["x_m"], b_start["y_m"] - a_end["y_m"]) < 0.5:
                pairs.append((a["id"], b["id"]))
                break
    return pairs, by_id


def main() -> int:
    parser = argparse.ArgumentParser(description="W2 服务/快照一致性对比")
    parser.add_argument("--base-url", default="http://127.0.0.1:8765")
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    base = args.base_url.rstrip("/")
    snapshot_path = Path(args.snapshot)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    snapshot_bytes = snapshot_path.read_bytes()
    snapshot = json.loads(snapshot_bytes.decode("utf-8"))
    lanelets = snapshot["lanelets"]
    lanelet_ids = {item["id"] for item in lanelets}
    areas = snapshot.get("areas", [])

    report: dict = {
        "schema_version": 1,
        "type": "web_service_compare",
        "base_url": base,
        "snapshot": {
            "path": str(snapshot_path),
            "sha256": hashlib.sha256(snapshot_bytes).hexdigest(),
            "map_id": snapshot["map_id"],
            "map_version": snapshot["map_version"],
        },
        "checks": {},
        "failures": [],
    }
    checks = report["checks"]

    def check(name: str, ok: bool, detail=None) -> None:
        checks[name] = {"ok": bool(ok)}
        if detail is not None:
            checks[name]["detail"] = detail
        if not ok:
            report["failures"].append(name)

    # 0) 等待服务（最多 30 s）
    healthy = False
    deadline = time.time() + 30
    while time.time() < deadline:
        status, health = http_json("GET", f"{base}/health", timeout=3)
        if status == 200 and isinstance(health, dict) and health.get("status") == "ok":
            healthy = True
            break
        time.sleep(0.5)
    check("health_ok", healthy)
    if not healthy:
        (output_dir / "service_compare.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print("[compare] 服务未就绪", file=sys.stderr)
        return 1

    # 1) /map/info
    status, info = http_json("GET", f"{base}/map/info")
    check(
        "info_envelope",
        status == 200 and isinstance(info, dict) and info.get("schema_version") == 1 and info.get("type") == "map_info",
    )
    if isinstance(info, dict):
        check("info_map_id", info.get("map_id") == snapshot["map_id"], info.get("map_id"))
        check("info_map_version", info.get("map_version") == snapshot["map_version"], info.get("map_version"))
        counts = info.get("counts", {})
        check("info_counts_lanelets", counts.get("lanelets") == snapshot["counts"]["lanelets"])
        check("info_counts_areas", counts.get("areas") == snapshot["counts"]["areas"])
        check("info_counts_points", counts.get("points") == snapshot["counts"]["points"])
        check("info_counts_regulatory", counts.get("regulatory_elements") == snapshot["counts"]["regulatory_elements"])
        bounds = info.get("bounds_m", {})
        check(
            "info_bounds",
            close_enough(bounds.get("min_x_m", 0), snapshot["bounds_m"]["min_x_m"], 1e-6)
            and close_enough(bounds.get("max_y_m", 0), snapshot["bounds_m"]["max_y_m"], 1e-6),
        )

    # 2) 抽样 lanelet 几何
    sample_indices = sorted({0, len(lanelets) // 4, len(lanelets) // 2, (3 * len(lanelets)) // 4, len(lanelets) - 1})
    sample_ids = [lanelets[index]["id"] for index in sample_indices]
    geometry_ok = True
    for lanelet_id in sample_ids:
        status, geometry = http_json("GET", f"{base}/map/lanelet/{lanelet_id}")
        expected = next(item for item in lanelets if item["id"] == lanelet_id)
        ok = (
            status == 200
            and isinstance(geometry, dict)
            and geometry.get("id") == lanelet_id
            and geometry.get("attributes") == expected["attributes"]
            and points_equal(geometry.get("left", []), expected["left"])
            and points_equal(geometry.get("right", []), expected["right"])
            and points_equal(geometry.get("centerline", []), expected.get("centerline", []))
        )
        if not ok:
            geometry_ok = False
            checks[f"lanelet_{lanelet_id}_geometry"] = {"ok": False}
    check("lanelet_sample_geometry", geometry_ok, {"sample_ids": sample_ids})

    # 3) 点查询：中心线中点命中自身
    point_ok = True
    for lanelet_id in sample_ids:
        expected = next(item for item in lanelets if item["id"] == lanelet_id)
        centerline = expected.get("centerline") or expected["left"]
        mid = centerline[len(centerline) // 2]
        status, result = http_json(
            "POST", f"{base}/map/query/point", {"x_m": mid["x_m"], "y_m": mid["y_m"]}
        )
        ids = [item["id"] for item in (result or {}).get("lanelets", [])]
        if not (status == 200 and lanelet_id in ids):
            point_ok = False
            checks[f"point_{lanelet_id}"] = {"ok": False, "ids": ids}
    check("point_query_sample", point_ok, {"sample_ids": sample_ids})

    # 4) 拓扑：ids ⊆ 快照
    topology_ok = True
    topology_detail: dict[str, list[str]] = {}
    for lanelet_id in sample_ids[:3]:
        for relation in ("following", "preceding"):
            status, result = http_json("GET", f"{base}/map/lanelet/{lanelet_id}/{relation}")
            ids = (result or {}).get("ids", [])
            topology_detail[f"{lanelet_id}/{relation}"] = ids
            if not (status == 200 and all(item in lanelet_ids for item in ids)):
                topology_ok = False
        status, result = http_json("GET", f"{base}/map/lanelet/{lanelet_id}/adjacent?side=left")
        ids = (result or {}).get("ids", [])
        topology_detail[f"{lanelet_id}/adjacent_left"] = ids
        if not (status == 200 and all(item in lanelet_ids for item in ids)):
            topology_ok = False
    check("topology_ids_valid", topology_ok, topology_detail)

    # 5) 路由：本地连通对
    pairs, by_id = find_connected_pair(lanelets)
    route_ok = False
    route_detail = None
    for from_id, to_id in pairs[:5]:
        status, result = http_json("POST", f"{base}/map/route", {"from_id": from_id, "to_id": to_id})
        ids = (result or {}).get("lanelet_ids", [])
        length = (result or {}).get("length_m", 0)
        route_detail = {"pair": [from_id, to_id], "status": status, "ids": ids, "length_m": length}
        if (
            status == 200
            and result.get("status") == "success"
            and from_id in ids
            and to_id in ids
            and length > 0
            and all(item in lanelet_ids for item in ids)
        ):
            route_ok = True
            break
    check("route_connected_pair", route_ok, route_detail)

    # 6) 窗口裁剪：以目标 lanelet 的包围盒外扩 10 m
    window_ok = False
    window_detail = None
    if pairs:
        target = by_id[pairs[0][0]]
        points = target["left"] + target["right"] + target.get("centerline", [])
        pad = 10.0
        bounds = {
            "min_x_m": min(p["x_m"] for p in points) - pad,
            "min_y_m": min(p["y_m"] for p in points) - pad,
            "max_x_m": max(p["x_m"] for p in points) + pad,
            "max_y_m": max(p["y_m"] for p in points) + pad,
        }
        status, crop = http_json("POST", f"{base}/map/query/window", {"bounds": bounds})
        window_detail = {"bounds": bounds, "status": status}
        if status == 200 and isinstance(crop, dict):
            frame = crop.get("window_frame", {})
            width = bounds["max_x_m"] - bounds["min_x_m"]
            height = bounds["max_y_m"] - bounds["min_y_m"]
            paths = crop.get("paths", [])
            points_local = crop.get("points", [])
            # 裁剪结果的路径/点位于地图坐标系（与地图组件 test_crop 一致）：
            # 所有坐标应落在窗口包围盒内。
            in_window_map = all(
                bounds["min_x_m"] - 1e-6 <= p["x_m"] <= bounds["max_x_m"] + 1e-6
                and bounds["min_y_m"] - 1e-6 <= p["y_m"] <= bounds["max_y_m"] + 1e-6
                for path in paths
                for p in path.get("points", [])
            ) and all(
                bounds["min_x_m"] - 1e-6 <= p["x_m"] <= bounds["max_x_m"] + 1e-6
                and bounds["min_y_m"] - 1e-6 <= p["y_m"] <= bounds["max_y_m"] + 1e-6
                for p in points_local
            )
            # 变换说明（window_frame 描述窗口局部系，yaw=0 时）：局部 = 地图 - 原点，
            # 换算后的局部坐标应落在 [0,width]×[0,height]。
            transform_ok = all(
                0 - 1e-6 <= p["x_m"] - frame.get("x_m", 0.0) <= width + 1e-6
                and 0 - 1e-6 <= p["y_m"] - frame.get("y_m", 0.0) <= height + 1e-6
                for path in paths
                for p in path.get("points", [])
            )
            window_ok = (
                close_enough(crop.get("window_m", {}).get("min_x_m", 1e18), bounds["min_x_m"], 1e-9)
                and close_enough(frame.get("x_m", 1e18), bounds["min_x_m"], 1e-9)
                and close_enough(frame.get("y_m", 1e18), bounds["min_y_m"], 1e-9)
                and close_enough(frame.get("yaw_rad", 1e18), 0.0, 1e-12)
                and len(paths) > 0
                and in_window_map
                and transform_ok
                and target["id"] in crop.get("lanelet_ids", [])
                and all(item in lanelet_ids for item in crop.get("lanelet_ids", []))
            )
            window_detail.update(
                {
                    "window_m": crop.get("window_m"),
                    "window_frame": frame,
                    "paths": len(paths),
                    "points": len(points_local),
                    "lanelet_ids": crop.get("lanelet_ids", []),
                    "all_points_in_window_map": in_window_map,
                    "window_local_transform_ok": transform_ok,
                }
            )
    check("window_crop_consistent", window_ok, window_detail)

    # 7) 错误语义
    status, error = http_json("GET", f"{base}/map/lanelet/999999999")
    check(
        "error_unknown_lanelet",
        status == 404 and isinstance(error, dict) and error.get("status") == "invalid_input",
        {"status": status, "body": error},
    )
    status, error = http_json("POST", f"{base}/map/route", {"from_id": "999999999", "to_id": "999999999"})
    check("error_unknown_route", status == 404, {"status": status})
    status, error = http_json("POST", f"{base}/map/query/point", {"x_m": 0.0})
    check("error_missing_field", status == 400, {"status": status})
    status, error = http_json(
        "POST",
        f"{base}/map/query/window",
        {"bounds": {"min_x_m": 10, "min_y_m": 0, "max_x_m": 0, "max_y_m": 10}},
    )
    check("error_empty_window", status == 400, {"status": status})

    # 8) /map/snapshot 一致性（计数 + 抽样几何）
    status, served = http_json("GET", f"{base}/map/snapshot")
    snapshot_ok = (
        status == 200
        and isinstance(served, dict)
        and served.get("schema_version") == 1
        and served.get("type") == "map_snapshot"
        and served.get("map_version") == snapshot["map_version"]
        and len(served.get("lanelets", [])) == len(lanelets)
        and len(served.get("areas", [])) == len(areas)
    )
    check("snapshot_counts_match", snapshot_ok)
    if snapshot_ok:
        served_by_id = {item["id"]: item for item in served["lanelets"]}
        probe = sample_ids[0]
        check(
            "snapshot_sample_geometry",
            points_equal(served_by_id[probe]["left"], by_id[probe]["left"])
            and served_by_id[probe]["attributes"] == by_id[probe]["attributes"],
            {"probe": probe},
        )

    report["ok"] = not report["failures"]
    (output_dir / "service_compare.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    print(f"[compare] checks: {len(checks)}, failures: {len(report['failures'])}")
    for name, result in checks.items():
        if not result["ok"]:
            print(f"[compare] FAIL {name}: {result.get('detail')}")
    print(f"[compare] report: {output_dir / 'service_compare.json'}")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
