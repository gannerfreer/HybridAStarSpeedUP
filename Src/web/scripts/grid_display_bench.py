#!/usr/bin/env python3
"""显示档性能首次实测（X-W1）：按 benchmark_tiers §4.1 协议测 256²/512²/1024²。

测量项与门槛（`Docs/expansion/benchmark_tiers.md` §4.2，X-E1 冻结初值）：

  尺寸     传输（未压缩 base64 JSON）  解析（冷，单次）  纹理生成（单次重建）  帧耗时（热缓存，P95）  堆内存增量
  256²     ≤ 96 KiB                  ≤ 10 ms          ≤ 5 ms                ≤ 16.7 ms            ≤ 2 MiB
  512²     ≤ 384 KiB                 ≤ 30 ms          ≤ 15 ms               ≤ 16.7 ms            ≤ 8 MiB
  1024²    ≤ 1.5 MiB                 ≤ 100 ms         ≤ 60 ms               ≤ 33.3 ms            ≤ 32 MiB

偏差按 §4.3 处理（≤20% 登记并做一次定向优化；>20% 或 2× 登记 ISSUES 待协调者）。

输入：`tests/fixtures/v6/grids/display_{256,512,1024}.json`（合成显示夹具）。
说明：benchmark_tiers §4.1 要求“每尺寸 2 个固定快照（开阔、迷宫）”，但
`grid_scenario_gen v1` 尚未实现（该文档 §8.4），本 run 以 v6 显示档夹具为输入并在
报告中记录该偏差；门槛按单夹具口径评估。

运行（经统一入口）：

  python3 tools/task_runner.py exec --component web --workers 2 -- \
    python3 Src/web/scripts/grid_display_bench.py
"""

from __future__ import annotations

import argparse
import functools
import gzip
import hashlib
import http.server
import json
import math
import os
import platform
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import browser_cdp_check as cdp  # noqa: E402

VIEWPORT_WIDTH = 1440
VIEWPORT_HEIGHT = 900
DPR_MAIN = 1
DPR_CHECK = 2
PORT = 18769

TIERS = (
    {"name": "display_256", "file": "display_256.json", "size": 256},
    {"name": "display_512", "file": "display_512.json", "size": 512},
    {"name": "display_1024", "file": "display_1024.json", "size": 1024},
)

# benchmark_tiers §4.2 冻结门槛（字节 / 毫秒 / 字节）。
THRESHOLDS = {
    256: {"transport_bytes": 96 * 1024, "parse_ms": 10.0, "texture_ms": 5.0,
          "frame_p95_ms": 16.7, "heap_delta_bytes": 2 * 1024 * 1024},
    512: {"transport_bytes": 384 * 1024, "parse_ms": 30.0, "texture_ms": 15.0,
          "frame_p95_ms": 16.7, "heap_delta_bytes": 8 * 1024 * 1024},
    1024: {"transport_bytes": int(1.5 * 1024 * 1024), "parse_ms": 100.0,
           "texture_ms": 60.0, "frame_p95_ms": 33.3,
           "heap_delta_bytes": 32 * 1024 * 1024},
}


def nearest_rank(values, quantile):
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(quantile * len(ordered))) - 1))
    return ordered[index]


def summarize(values):
    if not values:
        return {"count": 0}
    ordered = [float(value) for value in values]
    return {
        "count": len(ordered),
        "min_ms": min(ordered),
        "p50_ms": nearest_rank(ordered, 0.5),
        "p95_ms": nearest_rank(ordered, 0.95),
        "max_ms": max(ordered),
        "mean_ms": sum(ordered) / len(ordered),
    }


def median(values):
    ordered = sorted(float(value) for value in values)
    middle = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def verdict(measured, threshold):
    """§4.3：≤门槛 pass；≤1.2× deviation_minor；否则 deviation_major。

    1e-6 相对容差吸收浮点噪声（如 60 Hz 帧间隔 16.7 ms 的表示误差）。
    """
    if measured is None:
        return "missing"
    if measured <= threshold * (1 + 1e-6):
        return "pass"
    ratio = measured / threshold
    if ratio <= 1.2:
        return "deviation_minor"
    return "deviation_major"


def evaluate_async(pipe, session, expression, timeout=180.0):
    result = pipe.call(
        "Runtime.evaluate",
        {"expression": expression, "returnByValue": True, "awaitPromise": True},
        session,
        timeout,
    )
    if "exceptionDetails" in result:
        raise cdp.CdpError(f"页面脚本异常: {result['exceptionDetails']}")
    return result.get("result", {}).get("value")


def collect_gc(pipe, session):
    pipe.call("HeapProfiler.enable", {}, session)
    pipe.call("HeapProfiler.collectGarbage", {}, session)


def heap_used(pipe, session):
    metrics = pipe.call("Performance.getMetrics", {}, session)
    for item in metrics.get("metrics", []):
        if item.get("name") == "JSHeapUsedSize":
            return float(item.get("value"))
    return None


def heap_after_gc(pipe, session):
    collect_gc(pipe, session)
    time.sleep(0.2)
    return heap_used(pipe, session)


def memory_total_bytes():
    try:
        with open("/proc/meminfo", "r", encoding="utf-8") as handle:
            for line in handle:
                if line.startswith("MemTotal:"):
                    return int(line.split()[1]) * 1024
    except OSError:
        pass
    return None


def wait_bench_ready(pipe, session):
    ready = cdp.wait_for_condition(
        pipe, session, "window.__plannerGridBench && window.__plannerGridBench.ready === true", timeout=20.0
    )
    if not ready:
        raise cdp.CdpError("基准钩子未就绪（?gridBench=1）")


def run_tier(pipe, session, base_url, tier, out_dir, fixture_dir, reps, interact_ms, run_index):
    file = tier["file"]
    size = tier["size"]
    record = {"size": size, "file": file}

    pipe.call("Page.navigate", {"url": f"{base_url}?gridBench=1&grid=1&benchRun={run_index}"}, session)
    wait_bench_ready(pipe, session)
    time.sleep(0.6)

    # 内存基线（无栅格；强制 GC 后 3 次读数取中位数）
    baseline_readings = [heap_after_gc(pipe, session) for _ in range(3)]
    baseline = median([value for value in baseline_readings if value is not None])
    record["heap_baseline_bytes"] = baseline
    record["heap_baseline_readings"] = baseline_readings

    # 冷启动：文件读取 → 栅格首帧绘制完成（新页面上下文）
    cold = evaluate_async(
        pipe, session, f"window.__plannerGridBench.loadFixture({json.dumps(file)})", timeout=60.0
    )
    record["cold_start"] = cold
    cdp.wait_for_condition(
        pipe, session, "window.__plannerDebug.getState().grid.status === 'ready'", timeout=20.0
    )
    time.sleep(0.3)

    # 热重绘：请求重绘 → 绘制完成（复用已解码 cells 与纹理缓存）
    hot = evaluate_async(pipe, session, f"window.__plannerGridBench.hotRedraw({reps})", timeout=180.0)
    record["hot_redraw"] = summarize(hot.get("samples_ms") or [])

    # 帧耗时：连续缩放/平移（全部帧）
    frames = evaluate_async(
        pipe, session, f"window.__plannerGridBench.interact({interact_ms})", timeout=interact_ms / 1000.0 + 60.0
    )
    record["frame"] = {
        "frames": frames.get("frames"),
        "p50_ms": frames.get("frame_p50_ms"),
        "p95_ms": frames.get("frame_p95_ms"),
        "long_frames": frames.get("long_frames"),
        "duration_ms": frames.get("duration_ms"),
    }

    # 内存稳定态（栅格层驻留；3 次读数取中位数）
    after_readings = [heap_after_gc(pipe, session) for _ in range(3)]
    after = median([value for value in after_readings if value is not None])
    record["heap_after_bytes"] = after
    record["heap_after_readings"] = after_readings
    record["heap_delta_bytes"] = None if (after is None or baseline is None) else after - baseline
    record["canvas_backing_bytes_estimate"] = size * size * 4
    record["heap_scope_note"] = (
        "JSHeapUsedSize（CDP Performance.getMetrics）为主口径；Canvas 后备存储不计入 JS 堆，"
        "以 canvas_backing_bytes_estimate 单列参考。"
    )

    # 解析/纹理（每尺寸 ≥30 次冷单次）
    pipeline = evaluate_async(
        pipe, session, f"window.__plannerGridBench.runPipeline({json.dumps(file)}, {reps})", timeout=600.0
    )
    per_rep = pipeline.get("per_rep") or []
    record["fetch"] = summarize([item["fetch_ms"] for item in per_rep])
    record["parse"] = summarize([item["parse_ms"] for item in per_rep])
    record["texture"] = summarize([item["texture_ms"] for item in per_rep])
    record["pipeline_reps"] = len(per_rep)

    # 传输：文件字节 + gzip（含 JSON 头与 base64 表示）
    raw = (fixture_dir / file).read_bytes()
    record["transport_bytes"] = len(raw)
    record["gzip_bytes"] = len(gzip.compress(raw, 9))
    record["snapshot_sha256"] = hashlib.sha256(raw).hexdigest()

    # 纹理缓存复用（热重绘 + 交互期间重建次数）
    grid_state = evaluate_async(
        pipe,
        session,
        "JSON.stringify(window.__plannerDebug.getState().grid)",
        timeout=30.0,
    )
    state = json.loads(grid_state) if grid_state else {}
    record["texture_rebuilds"] = state.get("texture_rebuilds")
    record["degraded"] = state.get("degraded")
    record["map_id"] = state.get("map_id")

    screenshot = out_dir / f"grid_display_{size}.png"
    pipe.screenshot(session, screenshot)
    record["screenshot"] = screenshot.name

    # 门槛对照（§4.2 / §4.3）
    thresholds = THRESHOLDS[size]
    measured = {
        "transport_bytes": record["transport_bytes"],
        "parse_ms": record["parse"]["p95_ms"] if record["parse"].get("count") else None,
        "texture_ms": record["texture"]["p95_ms"] if record["texture"].get("count") else None,
        "frame_p95_ms": record["frame"].get("p95_ms"),
        "heap_delta_bytes": record["heap_delta_bytes"],
    }
    record["thresholds"] = thresholds
    record["verdicts"] = {
        key: verdict(measured[key], thresholds[key]) for key in thresholds
    }
    record["measured"] = measured
    return record


def run_dpr2_check(pipe, session, base_url, fixture_dir, out_dir, report):
    pipe.call(
        "Emulation.setDeviceMetricsOverride",
        {"width": VIEWPORT_WIDTH, "height": VIEWPORT_HEIGHT, "deviceScaleFactor": DPR_CHECK, "mobile": False},
        session,
    )
    pipe.call("Page.navigate", {"url": f"{base_url}?gridBench=1&grid=1&benchRun=dpr2"}, session)
    wait_bench_ready(pipe, session)
    time.sleep(0.6)
    evaluate_async(
        pipe, session, "window.__plannerGridBench.loadFixture('display_1024.json')", timeout=60.0
    )
    cdp.wait_for_condition(
        pipe, session, "window.__plannerDebug.getState().grid.status === 'ready'", timeout=20.0
    )
    time.sleep(0.4)
    bitmap = evaluate_async(
        pipe,
        session,
        "(() => { const canvas = document.querySelector('canvas.map-canvas');"
        " const rect = canvas.getBoundingClientRect();"
        " return { canvas: [canvas.width, canvas.height], css: [Math.round(rect.width), Math.round(rect.height)],"
        " dpr: window.devicePixelRatio }; })()",
        timeout=30.0,
    )
    ok = (
        isinstance(bitmap, dict)
        and bitmap.get("canvas")
        and bitmap.get("css")
        and bitmap["canvas"][0] == bitmap["css"][0] * DPR_CHECK
        and bitmap["canvas"][1] == bitmap["css"][1] * DPR_CHECK
    )
    pipe.screenshot(session, out_dir / "grid_display_dpr2_1024.png")
    report["dpr2_check"] = {
        "bitmap": bitmap,
        "expected_scale": DPR_CHECK,
        "ok": bool(ok),
        "screenshot": "grid_display_dpr2_1024.png",
        "note": "DPR=2 仅作位图尺寸检查（非门槛复验）。",
    }
    pipe.call(
        "Emulation.setDeviceMetricsOverride",
        {"width": VIEWPORT_WIDTH, "height": VIEWPORT_HEIGHT, "deviceScaleFactor": DPR_MAIN, "mobile": False},
        session,
    )
    return bool(ok)


def write_csv(path, tiers):
    header = [
        "size", "file", "snapshot_sha256", "transport_bytes", "gzip_bytes",
        "parse_ms", "texture_ms", "cold_first_frame_ms", "hot_redraw_p95_ms",
        "frame_p50_ms", "frame_p95_ms", "long_frames", "frames",
        "heap_delta_bytes", "viewport", "dpr",
    ]
    lines = [",".join(header)]
    for tier in tiers:
        row = [
            str(tier["size"]),
            tier["file"],
            tier.get("snapshot_sha256", ""),
            str(tier.get("transport_bytes", "")),
            str(tier.get("gzip_bytes", "")),
            "%.3f" % tier["parse"]["p95_ms"] if tier.get("parse", {}).get("count") else "",
            "%.3f" % tier["texture"]["p95_ms"] if tier.get("texture", {}).get("count") else "",
            "%.3f" % tier["cold_start"]["first_frame_ms"] if tier.get("cold_start") else "",
            "%.3f" % tier["hot_redraw"]["p95_ms"] if tier.get("hot_redraw", {}).get("count") else "",
            "%.3f" % tier["frame"]["p50_ms"] if tier.get("frame", {}).get("p50_ms") is not None else "",
            "%.3f" % tier["frame"]["p95_ms"] if tier.get("frame", {}).get("p95_ms") is not None else "",
            str(tier.get("frame", {}).get("long_frames", "")),
            str(tier.get("frame", {}).get("frames", "")),
            str(tier.get("heap_delta_bytes", "")),
            "1440x900",
            str(DPR_MAIN),
        ]
        lines.append(",".join(row))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="栅格显示档性能首次实测（CDP 无头 Chrome）")
    parser.add_argument("--output-dir", default=None, help="产物目录（默认 $PLANNER_RUN_DIR/outputs/grid_display_bench）")
    parser.add_argument("--reps", type=int, default=30, help="解析/纹理/热重绘重复次数（默认 30）")
    parser.add_argument("--interact-ms", type=int, default=11000, help="帧耗时交互窗口（默认 11000，≥10 s 且 ≥600 帧）")
    parser.add_argument("--url", default=None, help="页面 URL（默认由脚本自建静态服务）")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[3]
    dist_dir = repo_root / "Src" / "web" / "dist"
    fixture_dir = repo_root / "tests" / "fixtures" / "v6" / "grids"
    run_dir = os.environ.get("PLANNER_RUN_DIR")
    out_dir = Path(args.output_dir) if args.output_dir else (
        Path(run_dir) / "outputs" / "grid_display_bench" if run_dir else Path("/tmp/grid-display-bench")
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    if not (dist_dir / "index.html").is_file():
        print("[bench] dist/index.html 缺失；先执行 build --component web", file=sys.stderr)
        return 66
    for tier in TIERS:
        if not (fixture_dir / tier["file"]).is_file():
            print(f"[bench] 缺少夹具 {tier['file']}（tests/fixtures/v6/grids）", file=sys.stderr)
            return 66

    try:
        chrome_bin = cdp.find_chrome()
    except cdp.CdpError as error:
        print(f"[bench] {error}", file=sys.stderr)
        return 69

    cdp.StaticAndProxyHandler.api_target = None
    cdp.StaticAndProxyHandler.plan_api_target = None
    handler = functools.partial(cdp.StaticAndProxyHandler, directory=str(dist_dir))
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", PORT), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base_url = args.url or f"http://127.0.0.1:{PORT}/"

    report = {
        "schema_version": 1,
        "type": "grid_display_bench",
        "protocol": "Docs/expansion/benchmark_tiers.md §4.1（X-E1 冻结）；偏差按 §4.3",
        "fixture_version": "v6",
        "fixture_note": (
            "输入为 tests/fixtures/v6/grids/display_{256,512,1024}.json（splitmix64_thresholds_v1）；"
            "§4.1 要求每尺寸开阔/迷宫两个快照，但 grid_scenario_gen v1 尚未实现"
            "（benchmark_tiers §8.4），本 run 按单夹具口径评估并在报告中登记该偏差。"
        ),
        "environment": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "machine": platform.machine(),
            "cpu_count": os.cpu_count(),
            "mem_total_bytes": memory_total_bytes(),
            "viewport": [VIEWPORT_WIDTH, VIEWPORT_HEIGHT],
            "dpr": DPR_MAIN,
            "url": base_url,
        },
        "thresholds": {str(size): THRESHOLDS[size] for size in THRESHOLDS},
        "tiers": [],
        "notes": [],
    }

    pipe = None
    try:
        time.sleep(0.3)
        pipe = cdp.ChromePipe(chrome_bin, out_dir / "cdp_chrome.log")
        version = pipe.call("Browser.getVersion", {})
        report["environment"]["chrome"] = {
            "product": version.get("product"),
            "revision": version.get("revision"),
            "userAgent": version.get("userAgent"),
            "jsVersion": version.get("jsVersion"),
        }
        target_id = pipe.call("Target.createTarget", {"url": "about:blank"})["targetId"]
        session = pipe.call("Target.attachToTarget", {"targetId": target_id, "flatten": True})["sessionId"]
        pipe.call("Page.enable", session=session)
        pipe.call("Runtime.enable", session=session)
        pipe.call("Performance.enable", {}, session)
        pipe.call(
            "Emulation.setDeviceMetricsOverride",
            {"width": VIEWPORT_WIDTH, "height": VIEWPORT_HEIGHT, "deviceScaleFactor": DPR_MAIN, "mobile": False},
            session,
        )
        run_index = 0
        for tier in TIERS:
            run_index += 1
            print(f"[bench] {tier['name']}: 冷启动/热重绘/帧耗时/内存/解析/纹理 …")
            record = run_tier(
                pipe, session, base_url, tier, out_dir, fixture_dir,
                args.reps, args.interact_ms, run_index,
            )
            report["tiers"].append(record)
            print(
                f"[bench] {tier['name']}: transport={record['transport_bytes']} B "
                f"parse_p95={record['parse'].get('p95_ms'):.2f} ms "
                f"texture_p95={record['texture'].get('p95_ms'):.2f} ms "
                f"frame_p95={record['frame'].get('p95_ms'):.2f} ms "
                f"heap_delta={record['heap_delta_bytes']} B "
                f"verdicts={record['verdicts']}"
            )
        dpr2_ok = run_dpr2_check(pipe, session, base_url, fixture_dir, out_dir, report)
        report["notes"].append(f"DPR=2 位图检查: {'ok' if dpr2_ok else 'failed'}")
        if dpr2_ok:
            report["notes"].append("DPR=2 位图尺寸 = CSS 尺寸 × 2（与门槛无关的记录项）。")
        report["notes"].append(
            "帧耗时采集 requestAnimationFrame 帧间隔（全部帧，nearest-rank P50/P95）；"
            "热重绘复用已解码 cells 与纹理缓存（texture_rebuilds 记录）。"
        )
    except cdp.CdpError as error:
        print(f"[bench] 失败: {error}", file=sys.stderr)
        report["failure"] = str(error)
        (out_dir / "grid_display_bench.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        return 1
    finally:
        if pipe is not None:
            pipe.close()
        httpd.shutdown()
        httpd.server_close()

    write_csv(out_dir / "grid_display_metrics.csv", report["tiers"])
    (out_dir / "grid_display_bench.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    print("[bench] 门槛对照（§4.2）：")
    overall = "pass"
    for tier in report["tiers"]:
        for key, verdict_value in tier["verdicts"].items():
            marker = "OK" if verdict_value == "pass" else verdict_value.upper()
            print(f"  {tier['size']}² {key}: measured={tier['measured'][key]} "
                  f"threshold={tier['thresholds'][key]} -> {marker}")
            if verdict_value == "deviation_major":
                overall = "deviation_major"
            elif verdict_value == "deviation_minor" and overall == "pass":
                overall = "deviation_minor"
    print(f"[bench] 结论: {overall}")
    print(f"[bench] 产物: {out_dir / 'grid_display_bench.json'} / grid_display_metrics.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
