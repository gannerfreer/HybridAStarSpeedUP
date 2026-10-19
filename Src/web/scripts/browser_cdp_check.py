#!/usr/bin/env python3
"""W1/W2 浏览器交互检查：Chrome DevTools Protocol（pipe 传输，仅标准库）。

与 `browser_smoke.sh`（`--screenshot` 一次性捕获，存在无头窗口尺寸偏差）不同，
本脚本通过 `Emulation.setDeviceMetricsOverride` 固定视口，等待画布尺寸稳定后再截图，
并在真实浏览器中执行交互检查：

  W1：画布尺寸一致、滚轮光标中心缩放、拖拽平移、图层开关、重置视图、DPR=2；
  W2（--api-target 时）：/api 反向代理到 planner_map_service，执行
      点击选择（对象 ID 与高亮像素）、拓扑查询、起终位姿 + 路由、窗口拖拽查询。

产物（截图 PNG + 检查报告 JSON）写入 $PLANNER_RUN_DIR/outputs/browser_cdp/。
经统一入口运行：

  python3 tools/task_runner.py exec --component web --workers 2 -- \
    python3 Src/web/scripts/browser_cdp_check.py
"""

from __future__ import annotations

import argparse
import base64
import functools
import http.server
import json
import math
import os
import select
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

VIEWPORT_WIDTH = 1440
VIEWPORT_HEIGHT = 900


class CdpError(RuntimeError):
    pass


def find_chrome() -> str:
    for candidate in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser"):
        path = shutil.which(candidate)
        if path:
            return path
    raise CdpError("未找到 chrome/chromium 可执行文件")


def _dup_high(fd: int) -> int:
    """把 fd 复制到编号 > 4 的新 fd（必要时临时占用低位 fd 抬高编号）。"""
    holders = []
    try:
        while True:
            new_fd = os.dup(fd)
            if new_fd > 4:
                return new_fd
            holders.append(new_fd)
    finally:
        for holder in holders:
            os.close(holder)


class ChromePipe:
    """Chrome CDP over pipe：命令写 fd3，响应读 fd4，消息以 NUL 分隔。"""

    def __init__(self, chrome_bin: str, log_path: Path) -> None:
        # 先备份进程内可能占用的 fd 3/4（例如同进程 HTTP 服务的监听套接字）：
        # Chrome remote-debugging-pipe 需要 3/4，但映射不能破坏其他资源。
        saved: dict[int, int | None] = {}
        for fd in (3, 4):
            try:
                saved[fd] = _dup_high(fd)
            except OSError:
                saved[fd] = None

        cmd_r, cmd_w = os.pipe()
        out_r, out_w = os.pipe()
        # Chrome remote-debugging-pipe 约定子进程 fd3 读命令、fd4 写响应。
        # 先把四个 pipe 端搬到 >4 的高位，再映射 3/4，并用 pass_fds=(3,4)
        # 交给子进程（CPython 会在 preexec 之后关闭不在 pass_fds 中的 fd，
        # 因此不能依赖 preexec 内的 dup2）。
        t_cmd_r = _dup_high(cmd_r)
        t_cmd_w = _dup_high(cmd_w)
        t_out_r = _dup_high(out_r)
        t_out_w = _dup_high(out_w)
        for fd in (cmd_r, cmd_w, out_r, out_w):
            os.close(fd)
        os.dup2(t_cmd_r, 3)
        os.dup2(t_out_w, 4)
        os.close(t_cmd_r)
        os.close(t_out_w)
        self.cmd_w = t_cmd_w
        self.out_r = t_out_r

        self.log = open(log_path, "wb")
        self.profile_dir = tempfile.mkdtemp(prefix="web-cdp-")
        argv = [
            chrome_bin,
            "--headless=new",
            "--remote-debugging-pipe",
            "--no-sandbox",
            "--disable-gpu",
            "--disable-extensions",
            "--disable-background-networking",
            "--no-first-run",
            "--no-default-browser-check",
            "--no-proxy-server",
            f"--user-data-dir={self.profile_dir}",
        ]
        try:
            self.proc = subprocess.Popen(
                argv,
                pass_fds=(3, 4),
                stdout=self.log,
                stderr=self.log,
            )
        finally:
            os.close(3)
            os.close(4)
            for fd, backup in saved.items():
                if backup is not None:
                    os.dup2(backup, fd)
                    os.close(backup)
        self.buffer = b""
        self.next_id = 1
        # 诊断：保留最近的 CDP 事件（Runtime.exceptionThrown / console）供失败报告。
        self.events: list[dict] = []

    def _write(self, message: dict) -> int:
        os.write(self.cmd_w, json.dumps(message).encode("utf-8") + b"\0")
        return message["id"]

    def _read_message(self, timeout: float) -> dict:
        deadline = time.time() + timeout
        while b"\0" not in self.buffer:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise CdpError("等待 CDP 消息超时")
            ready, _, _ = select.select([self.out_r], [], [], remaining)
            if not ready:
                raise CdpError("等待 CDP 消息超时")
            chunk = os.read(self.out_r, 65536)
            if not chunk:
                raise CdpError("chrome pipe 已关闭")
            self.buffer += chunk
        raw, self.buffer = self.buffer.split(b"\0", 1)
        return json.loads(raw.decode("utf-8"))

    def call(self, method: str, params: dict | None = None, session: str | None = None, timeout: float = 20.0) -> dict:
        message = {"id": self.next_id, "method": method}
        self.next_id += 1
        if params:
            message["params"] = params
        if session:
            message["sessionId"] = session
        target_id = self._write(message)
        deadline = time.time() + timeout
        while True:
            incoming = self._read_message(max(0.1, deadline - time.time()))
            if incoming.get("id") == target_id:
                if "error" in incoming:
                    raise CdpError(f"{method} 失败: {incoming['error']}")
                return incoming.get("result", {})
            # 忽略事件消息（保留最近事件用于失败诊断）。
            self.events.append(incoming)
            if len(self.events) > 200:
                del self.events[:100]

    def recent_exceptions(self, limit: int = 5) -> list[str]:
        found: list[str] = []
        for event in reversed(self.events):
            method = event.get("method")
            if method == "Runtime.exceptionThrown":
                details = (event.get("params") or {}).get("exceptionDetails") or {}
                text = details.get("exception", {}).get("description") or details.get("text")
                if text:
                    found.append(str(text).splitlines()[0])
            elif method == "Runtime.consoleAPICalled" and (event.get("params") or {}).get("type") == "error":
                args = (event.get("params") or {}).get("args") or []
                rendered = ", ".join(str(arg.get("value") or arg.get("description") or "") for arg in args)
                if rendered:
                    found.append(f"console.error: {rendered}")
            if len(found) >= limit:
                break
        return found

    def evaluate(self, expression: str, session: str, timeout: float = 20.0):
        result = self.call(
            "Runtime.evaluate",
            {"expression": expression, "returnByValue": True},
            session,
            timeout,
        )
        if "exceptionDetails" in result:
            details = result["exceptionDetails"]
            description = (details.get("exception") or {}).get("description") or details.get("text")
            snippet = expression if len(expression) <= 200 else expression[:200] + "…"
            raise CdpError(f"页面脚本异常: {description}\n表达式: {snippet}")
        return result.get("result", {}).get("value")

    def screenshot(self, session: str, path: Path) -> None:
        result = self.call("Page.captureScreenshot", {"format": "png"}, session)
        path.write_bytes(base64.b64decode(result["data"]))

    def close(self) -> None:
        try:
            self.call("Browser.close", timeout=5.0)
        except Exception:
            pass
        try:
            self.proc.wait(timeout=5)
        except Exception:
            self.proc.kill()
        for fd in (self.cmd_w, self.out_r):
            try:
                os.close(fd)
            except OSError:
                pass
        self.log.close()
        shutil.rmtree(self.profile_dir, ignore_errors=True)


class StaticAndProxyHandler(http.server.SimpleHTTPRequestHandler):
    """静态文件 + `/api`、`/plan-api` 反向代理（模拟 Vite dev 代理；仅本地检查使用）。"""

    api_target: str | None = None
    plan_api_target: str | None = None

    def do_GET(self) -> None:  # noqa: N802 (http.server 命名约定)
        self._dispatch("GET")

    def do_POST(self) -> None:  # noqa: N802
        self._dispatch("POST")

    def _dispatch(self, method: str) -> None:
        if self.path.startswith("/plan-api/"):
            self._proxy(method, "/plan-api", self.plan_api_target)
            return
        if self.path.startswith("/api/"):
            self._proxy(method, "/api", self.api_target)
            return
        if method == "GET":
            super().do_GET()
        else:
            self.send_error(405, "method not allowed")

    def _proxy(self, method: str, prefix: str, target: str | None) -> None:
        if not target:
            self.send_error(502, f"{prefix} target not configured")
            return
        url = target.rstrip("/") + self.path[len(prefix):]
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length) if length > 0 else None
        headers = {"Accept": "application/json"}
        if body is not None:
            headers["Content-Type"] = self.headers.get("Content-Type", "application/json")
        request = urllib.request.Request(url, data=body, method=method, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                data = response.read()
                status = response.status
                content_type = response.headers.get("Content-Type", "application/json")
        except urllib.error.HTTPError as error:
            data = error.read()
            status = error.code
            content_type = "application/json"
        except Exception as error:  # noqa: BLE001
            self.send_error(502, f"proxy error: {error}")
            return
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args) -> None:  # 静默访问日志
        pass


STATE_EXPR = r"""(() => {
  const canvas = document.querySelector('canvas.map-canvas');
  const status = document.querySelector('.statusbar');
  const readout = document.querySelector('.readout');
  if (!canvas || !status) return JSON.stringify({ready: false});
  const host = canvas.parentElement;
  const rect = host.getBoundingClientRect();
  const ctx = canvas.getContext('2d');
  const data = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
  let orange = 0, nonBackground = 0, selection = 0, footprint = 0;
  const bg = [11, 18, 32];
  for (let i = 0; i < data.length; i += 4) {
    const r = data[i], g = data[i + 1], b = data[i + 2];
    if (Math.abs(r - 245) < 40 && Math.abs(g - 166) < 40 && Math.abs(b - 35) < 40) orange += 1;
    if (Math.abs(r - 34) < 40 && Math.abs(g - 211) < 40 && Math.abs(b - 238) < 40) selection += 1;
    // 起点足迹填充 rgba(34,197,94,0.20)：叠加背景 (16,54,44)、叠加区域填充 (24,75,84)。
    if (
      (Math.abs(r - 16) < 10 && Math.abs(g - 54) < 10 && Math.abs(b - 44) < 10) ||
      (Math.abs(r - 24) < 10 && Math.abs(g - 75) < 10 && Math.abs(b - 84) < 10)
    ) footprint += 1;
    if (Math.abs(r - bg[0]) + Math.abs(g - bg[1]) + Math.abs(b - bg[2]) > 30) nonBackground += 1;
  }
  const mppMatch = status.textContent.match(/([0-9.]+) m\/px/);
  return JSON.stringify({
    ready: true,
    viewport: [window.innerWidth, window.innerHeight],
    dpr: window.devicePixelRatio,
    host: [Math.round(rect.width), Math.round(rect.height)],
    canvas: [canvas.width, canvas.height],
    mpp: mppMatch ? parseFloat(mppMatch[1]) : null,
    readout: readout ? readout.textContent : null,
    centerlinePixels: orange,
    selectionPixels: selection,
    footprintPixels: footprint,
    mapPixels: nonBackground,
    hasMap: status.textContent.includes('mapping_example'),
    centerlineChecked: document.querySelectorAll('.layer-toggle input')[2]?.checked ?? null,
  });
})()"""


def parse_readout(text: str | None):
    if not text:
        return None
    try:
        x_part, y_part = text.replace(" ", "").split("·")
        x_value = float(x_part.split("=")[1].replace("m", ""))
        y_value = float(y_part.split("=")[1].replace("m", ""))
        return [x_value, y_value]
    except Exception:
        return None


def wait_for_state(pipe: ChromePipe, session: str, timeout: float = 15.0) -> dict:
    deadline = time.time() + timeout
    last = {}
    while time.time() < deadline:
        raw = pipe.evaluate(STATE_EXPR, session)
        last = json.loads(raw)
        if last.get("ready") and last.get("hasMap") and last.get("mpp") is not None:
            dpr = last.get("dpr") or 1
            host = last["host"]
            canvas = last["canvas"]
            if (
                abs(host[0] * dpr - canvas[0]) <= dpr + 1
                and abs(host[1] * dpr - canvas[1]) <= dpr + 1
                and canvas[0] > 0
            ):
                return last
        time.sleep(0.2)
    raise CdpError(f"等待应用稳定超时，最后状态: {last}")


def dispatch_mouse(pipe: ChromePipe, session: str, event_type: str, x: float, y: float, button: str = "none", buttons: int = 0) -> None:
    click_count = 1 if event_type in ("mousePressed", "mouseReleased") else 0
    pipe.call(
        "Input.dispatchMouseEvent",
        {"type": event_type, "x": x, "y": y, "button": button, "buttons": buttons, "clickCount": click_count},
        session,
    )


def move_and_read(pipe: ChromePipe, session: str, x: float, y: float) -> list | None:
    dispatch_mouse(pipe, session, "mouseMoved", x, y)
    time.sleep(0.15)
    state = json.loads(pipe.evaluate(STATE_EXPR, session))
    return parse_readout(state.get("readout"))


def run_checks(pipe: ChromePipe, session: str, url: str, out_dir: Path) -> dict:
    report: dict = {
        "schema_version": 1,
        "type": "web_browser_check",
        "url": url,
        "viewport_requested": [VIEWPORT_WIDTH, VIEWPORT_HEIGHT],
        "checks": {},
        "screenshots": [],
    }
    checks = report["checks"]

    state0 = wait_for_state(pipe, session)
    checks["load_ok"] = True
    checks["viewport_actual"] = state0["viewport"]
    checks["dpr"] = state0["dpr"]
    checks["canvas_size_matches_host"] = state0["host"] == state0["canvas"]
    checks["initial_mpp"] = state0["mpp"]
    checks["initial_map_pixels"] = state0["mapPixels"]
    checks["initial_centerline_pixels"] = state0["centerlinePixels"]
    pipe.screenshot(session, out_dir / "browser_cdp_initial.png")
    report["screenshots"].append("browser_cdp_initial.png")

    anchor = (VIEWPORT_WIDTH / 2, VIEWPORT_HEIGHT / 2 - 60)
    world_before = move_and_read(pipe, session, *anchor)
    checks["cursor_world_before_zoom"] = world_before

    pipe.call(
        "Input.dispatchMouseEvent",
        {"type": "mouseWheel", "x": anchor[0], "y": anchor[1], "deltaX": 0, "deltaY": -480, "button": "none", "buttons": 0},
        session,
    )
    time.sleep(0.3)
    state1 = json.loads(pipe.evaluate(STATE_EXPR, session))
    checks["zoomed_mpp"] = state1["mpp"]
    checks["zoom_in_reduces_mpp"] = bool(state1["mpp"] and state0["mpp"] and state1["mpp"] < state0["mpp"])
    if state0["mpp"] and state1["mpp"]:
        checks["zoom_factor"] = state0["mpp"] / state1["mpp"]

    world_after_zoom = move_and_read(pipe, session, *anchor)
    checks["cursor_world_after_zoom"] = world_after_zoom
    if world_before and world_after_zoom:
        drift = max(abs(world_before[0] - world_after_zoom[0]), abs(world_before[1] - world_after_zoom[1]))
        checks["zoom_anchor_drift_m"] = drift
        checks["zoom_anchor_ok"] = drift <= 0.05
    else:
        checks["zoom_anchor_ok"] = False

    pipe.screenshot(session, out_dir / "browser_cdp_zoomed.png")
    report["screenshots"].append("browser_cdp_zoomed.png")

    # 拖拽平移：从 anchor 拖到 anchor+(100, 50)
    dispatch_mouse(pipe, session, "mousePressed", anchor[0], anchor[1], button="left", buttons=1)
    for step in range(1, 6):
        dispatch_mouse(
            pipe,
            session,
            "mouseMoved",
            anchor[0] + 20 * step,
            anchor[1] + 10 * step,
            button="left",
            buttons=1,
        )
        time.sleep(0.05)
    dispatch_mouse(pipe, session, "mouseReleased", anchor[0] + 100, anchor[1] + 50, button="left", buttons=0)
    time.sleep(0.2)

    world_after_pan = move_and_read(pipe, session, *anchor)
    checks["cursor_world_after_pan"] = world_after_pan
    mpp_now = state1["mpp"]
    if world_after_zoom and world_after_pan and mpp_now:
        expected = [-100 * mpp_now, 50 * mpp_now]
        actual = [
            world_after_pan[0] - world_after_zoom[0],
            world_after_pan[1] - world_after_zoom[1],
        ]
        error = max(abs(actual[0] - expected[0]), abs(actual[1] - expected[1]))
        checks["pan_expected_delta_m"] = [round(v, 3) for v in expected]
        checks["pan_actual_delta_m"] = [round(v, 3) for v in actual]
        checks["pan_error_m"] = error
        checks["pan_ok"] = error <= 2.0
    else:
        checks["pan_ok"] = False

    # 图层开关：中心线关闭 → 橙色像素消失；恢复 → 出现
    pipe.evaluate("document.querySelectorAll('.layer-toggle input')[2].click()", session)
    time.sleep(0.3)
    state_off = json.loads(pipe.evaluate(STATE_EXPR, session))
    checks["centerline_off_pixels"] = state_off["centerlinePixels"]
    checks["centerline_toggle_off_ok"] = state_off["centerlinePixels"] == 0 and state_off["mapPixels"] > 1000
    pipe.screenshot(session, out_dir / "browser_cdp_centerline_off.png")
    report["screenshots"].append("browser_cdp_centerline_off.png")

    pipe.evaluate("document.querySelectorAll('.layer-toggle input')[2].click()", session)
    time.sleep(0.3)
    state_on = json.loads(pipe.evaluate(STATE_EXPR, session))
    checks["centerline_on_pixels"] = state_on["centerlinePixels"]
    checks["centerline_toggle_on_ok"] = state_on["centerlinePixels"] > 0

    # 重置视图：比例恢复初始
    pipe.evaluate(
        "[...document.querySelectorAll('button')].find((b) => b.textContent.includes('重置视图')).click()",
        session,
    )
    time.sleep(0.3)
    state_reset = json.loads(pipe.evaluate(STATE_EXPR, session))
    checks["reset_mpp"] = state_reset["mpp"]
    if state0["mpp"] and state_reset["mpp"]:
        checks["reset_ok"] = abs(state_reset["mpp"] - state0["mpp"]) < 1e-6
    else:
        checks["reset_ok"] = False
    pipe.screenshot(session, out_dir / "browser_cdp_final.png")
    report["screenshots"].append("browser_cdp_final.png")

    # 高 DPI：以 deviceScaleFactor=2 重新加载，验证位图尺寸 = 宿主 CSS 尺寸 × DPR。
    pipe.call(
        "Emulation.setDeviceMetricsOverride",
        {"width": VIEWPORT_WIDTH, "height": VIEWPORT_HEIGHT, "deviceScaleFactor": 2, "mobile": False},
        session,
    )
    pipe.call("Page.reload", session=session)
    state_dpr2 = wait_for_state(pipe, session)
    checks["dpr2_device_scale"] = state_dpr2["dpr"]
    checks["dpr2_host"] = state_dpr2["host"]
    checks["dpr2_canvas"] = state_dpr2["canvas"]
    checks["dpr2_mpp"] = state_dpr2["mpp"]
    checks["dpr2_ok"] = (
        state_dpr2["dpr"] == 2
        and state_dpr2["canvas"] == [state_dpr2["host"][0] * 2, state_dpr2["host"][1] * 2]
        and state_dpr2["mapPixels"] > 1000
    )
    pipe.screenshot(session, out_dir / "browser_cdp_dpr2.png")
    report["screenshots"].append("browser_cdp_dpr2.png")

    # 恢复 DPR=1（后续无操作，仅保证浏览器状态一致）。
    pipe.call(
        "Emulation.setDeviceMetricsOverride",
        {"width": VIEWPORT_WIDTH, "height": VIEWPORT_HEIGHT, "deviceScaleFactor": 1, "mobile": False},
        session,
    )
    pipe.call("Page.reload", session=session)
    wait_for_state(pipe, session)

    return report


def fetch_json(url: str, timeout: float = 15.0):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def wait_for_condition(pipe: ChromePipe, session: str, expression: str, timeout: float = 15.0):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        last = pipe.evaluate(expression, session)
        if last:
            return last
        time.sleep(0.25)
    return last


def debug_state(pipe: ChromePipe, session: str) -> dict:
    raw = pipe.evaluate(
        "JSON.stringify(window.__plannerDebug ? window.__plannerDebug.getState() : null)", session
    )
    return json.loads(raw) if raw else {}


def find_connected_pair(snapshot: dict):
    lanelets = snapshot["lanelets"]
    by_id = {item["id"]: item for item in lanelets}
    for a in lanelets:
        a_centerline = a.get("centerline") or []
        if len(a_centerline) < 2:
            continue
        a_end = a_centerline[-1]
        for b in lanelets:
            if b["id"] == a["id"]:
                continue
            b_centerline = b.get("centerline") or []
            if len(b_centerline) < 2:
                continue
            b_start = b_centerline[0]
            if math.hypot(b_start["x_m"] - a_end["x_m"], b_start["y_m"] - a_end["y_m"]) < 0.5:
                return a["id"], b["id"], by_id
    return None, None, by_id


def centerline_midpoint(lanelet: dict):
    centerline = lanelet.get("centerline") or lanelet.get("left") or []
    if not centerline:
        return None
    point = centerline[len(centerline) // 2]
    return [point["x_m"], point["y_m"]]


def page_point_for_world(pipe: ChromePipe, session: str, x: float, y: float):
    expression = (
        "(() => { const d = window.__plannerDebug; const canvas = document.querySelector('canvas.map-canvas');"
        f" if (!d || !canvas) return null; const p = d.worldToScreen({x}, {y}); if (!p) return null;"
        " const rect = canvas.getBoundingClientRect(); return [rect.left + p[0], rect.top + p[1]]; })()"
    )
    return pipe.evaluate(expression, session)


def click_at(pipe: ChromePipe, session: str, x: float, y: float, settle: float = 0.4) -> None:
    dispatch_mouse(pipe, session, "mouseMoved", x, y)
    time.sleep(0.1)
    dispatch_mouse(pipe, session, "mousePressed", x, y, button="left", buttons=1)
    time.sleep(0.05)
    dispatch_mouse(pipe, session, "mouseReleased", x, y, button="left", buttons=0)
    time.sleep(settle)


def set_pose_fields_numeric(pipe: ChromePipe, session: str, block_index: int, x, y, yaw_deg) -> bool:
    """数值设置起/终位姿（NumberField 逐字段输入 + 回车提交；绕过画布命中）。"""
    for field_index, value in ((0, x), (1, y), (2, yaw_deg)):
        selector = (
            "document.querySelectorAll('.pose-block')[%d].querySelectorAll('input')[%d]"
            % (block_index, field_index)
        )
        set_script = (
            "(() => { const el = %s; if (!el) return false;"
            " const setter = Object.getOwnPropertyDescriptor(window.HTMLInputElement.prototype, 'value').set;"
            " setter.call(el, %s); el.dispatchEvent(new Event('input', { bubbles: true }));"
            " return true; })()" % (selector, json.dumps(str(value)))
        )
        if not pipe.evaluate(set_script, session):
            return False
        time.sleep(0.2)  # 等待 React 以新 text 重渲染后再提交
        commit_script = (
            "(() => { const el = %s; if (!el) return false;"
            " el.dispatchEvent(new KeyboardEvent('keydown', { key: 'Enter', bubbles: true }));"
            " return true; })()" % selector
        )
        if not pipe.evaluate(commit_script, session):
            return False
        time.sleep(0.25)
    return True


def click_button(pipe: ChromePipe, session: str, text: str, selector: str = ".inspector button") -> bool:
    expression = (
        "(() => { const buttons = [...document.querySelectorAll("
        f"'{selector}')]; const b = buttons.find((item) => item.textContent.includes('{text}'));"
        " if (!b) return false; b.click(); return true; })()"
    )
    result = pipe.evaluate(expression, session)
    time.sleep(0.3)
    return bool(result)


def dismiss_notice(pipe: ChromePipe, session: str) -> None:
    pipe.evaluate("(() => { const b = document.querySelector('.notice-close'); if (b) b.click(); })()", session)
    time.sleep(0.2)


def run_w2_checks(
    pipe: ChromePipe,
    session: str,
    snapshot_url: str,
    out_dir: Path,
    report: dict,
    route_pair: tuple[str, str] | None = None,
) -> bool:
    """W2 交互检查（需要 --api-target 代理到 planner_map_service）。"""
    checks = report.setdefault("w2_checks", {})
    ok_flags: list[bool] = []

    try:
        snapshot = fetch_json(snapshot_url)
    except Exception as error:  # noqa: BLE001
        checks["snapshot_fetch_ok"] = False
        checks["snapshot_fetch_error"] = str(error)
        return False
    checks["snapshot_fetch_ok"] = True
    lanelet_ids = {item["id"] for item in snapshot["lanelets"]}

    by_id = {item["id"]: item for item in snapshot["lanelets"]}
    if route_pair and all(item in by_id for item in route_pair):
        a_id, b_id = route_pair
    else:
        a_id, b_id, by_id = find_connected_pair(snapshot)
    checks["route_pair"] = [a_id, b_id]
    if a_id is None or b_id is None:
        checks["pair_found"] = False
        return False
    checks["pair_found"] = True

    online = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState(); return !!(s && s.service_online); })()",
        20.0,
    )
    checks["service_online"] = bool(online)
    ok_flags.append(bool(online))
    if not online:
        return False

    # 1) 点击选择 lanelet A：面板出现 ID、画布选中色像素增加
    state_before = json.loads(pipe.evaluate(STATE_EXPR, session))
    mid_a = centerline_midpoint(by_id[a_id])
    page_a = page_point_for_world(pipe, session, *mid_a)
    checks["click_point_a"] = page_a
    if not page_a:
        checks["selection_ok"] = False
        return False
    click_at(pipe, session, *page_a)
    selection = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        f" return s && s.selection && s.selection.id === '{a_id}' ? s.selection : null; }})()",
        10.0,
    )
    state_after = json.loads(pipe.evaluate(STATE_EXPR, session))
    panel_text = pipe.evaluate("(() => document.querySelector('.inspector').textContent)()", session) or ""
    checks["selection_id"] = selection.get("id") if selection else None
    checks["selection_source"] = selection.get("source") if selection else None
    checks["selection_panel_has_id"] = a_id in panel_text
    checks["selection_pixels_before"] = state_before.get("selectionPixels")
    checks["selection_pixels_after"] = state_after.get("selectionPixels")
    selection_ok = (
        bool(selection)
        and a_id in panel_text
        and state_after.get("selectionPixels", 0) > state_before.get("selectionPixels", 0)
    )
    checks["selection_ok"] = selection_ok
    ok_flags.append(selection_ok)
    pipe.screenshot(session, out_dir / "browser_cdp_w2_selection.png")
    report["screenshots"].append("browser_cdp_w2_selection.png")

    # 2) 拓扑查询（后继 following）：ids ⊆ 快照
    checks["topology_button_clicked"] = click_button(pipe, session, "后继")
    topology = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState(); return s && s.topology ? s.topology : null; })()",
        15.0,
    )
    topology_ids = topology.get("ids") if topology else None
    checks["topology_kind"] = topology.get("kind") if topology else None
    checks["topology_ids"] = topology_ids
    topology_ok = (
        bool(topology)
        and topology.get("kind") == "following"
        and isinstance(topology_ids, list)
        and all(item in lanelet_ids for item in topology_ids)
    )
    checks["topology_ok"] = topology_ok
    ok_flags.append(topology_ok)
    pipe.screenshot(session, out_dir / "browser_cdp_w2_topology.png")
    report["screenshots"].append("browser_cdp_w2_topology.png")

    # 3) 起终位姿 + 路由（A → B，本地连通对）
    dismiss_notice(pipe, session)
    # 相邻 lanelet 在默认视图下仅相距数像素（位姿标记拖拽半径 12 px），
    # 先以选择点为中心放大，保证点击目标与标记分离。
    zoom_page = page_point_for_world(pipe, session, *centerline_midpoint(by_id[a_id]))
    if zoom_page:
        for _ in range(2):
            pipe.call(
                "Input.dispatchMouseEvent",
                {
                    "type": "mouseWheel",
                    "x": zoom_page[0],
                    "y": zoom_page[1],
                    "deltaX": 0,
                    "deltaY": -900,
                    "button": "none",
                    "buttons": 0,
                },
                session,
            )
            time.sleep(0.3)
    checks["w2_zoom_mpp"] = json.loads(pipe.evaluate(STATE_EXPR, session)).get("mpp")
    click_button(pipe, session, "设起点", ".mode-button")
    start_page = page_point_for_world(pipe, session, *centerline_midpoint(by_id[a_id]))
    if start_page:
        click_at(pipe, session, *start_page)
    # 服务端（X-A1 后）要求起终车辆足迹位于 lanelet 可行驶域内：目标点取
    # B 中心线**距起点最近**的顶点（内点），避免 B 末端/地图边缘足迹出域；
    # 再放大一档，使最近顶点超出起点标记拖拽半径（12 px）可被点击。
    if zoom_page:
        pipe.call(
            "Input.dispatchMouseEvent",
            {
                "type": "mouseWheel",
                "x": zoom_page[0],
                "y": zoom_page[1],
                "deltaX": 0,
                "deltaY": -900,
                "button": "none",
                "buttons": 0,
            },
            session,
        )
        time.sleep(0.3)
    click_button(pipe, session, "设终点", ".mode-button")
    # 相邻 lanelet 的中心线顶点可能落在起点标记的拖拽半径内；按距起点升序
    # 尝试（优先内点），失败时按距离递增重试。
    a_mid = centerline_midpoint(by_id[a_id])
    b_centerline = by_id[b_id].get("centerline") or by_id[b_id].get("left") or []
    candidates = sorted(
        {(round(p["x_m"], 6), round(p["y_m"], 6)) for p in b_centerline},
        key=lambda p: math.hypot(p[0] - a_mid[0], p[1] - a_mid[1]),
    )
    goal_pose = None
    goal_attempts = []
    for candidate in candidates[:4]:
        goal_page = page_point_for_world(pipe, session, candidate[0], candidate[1])
        if not goal_page:
            continue
        click_at(pipe, session, *goal_page)
        state = debug_state(pipe, session)
        accepted = bool(state.get("goal_pose")) and state.get("route_to") == b_id
        goal_attempts.append(
            {
                "point": [candidate[0], candidate[1]],
                "page": goal_page,
                "goal_set": bool(state.get("goal_pose")),
                "route_to": state.get("route_to"),
            }
        )
        if accepted:
            goal_pose = state["goal_pose"]
            break
    poses = debug_state(pipe, session)
    checks["start_pose"] = poses.get("start_pose")
    checks["goal_pose"] = goal_pose
    checks["goal_attempts"] = goal_attempts
    dismiss_notice(pipe, session)
    checks["route_button_clicked"] = click_button(pipe, session, "计算路由")
    route_state = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState(); return s && s.route && s.route.length > 0 ? s : null; })()",
        25.0,
    )
    route_ids = route_state.get("route") if route_state else None
    checks["route_ids"] = route_ids
    checks["route_length_m"] = route_state.get("route_length_m") if route_state else None
    checks["route_inputs"] = (
        [route_state.get("route_from"), route_state.get("route_to")] if route_state else None
    )
    route_ok = (
        bool(route_state)
        and isinstance(route_ids, list)
        and a_id in route_ids
        and b_id in route_ids
        and all(item in lanelet_ids for item in route_ids)
    )
    checks["route_ok"] = route_ok
    ok_flags.append(route_ok)

    # 规划服务自 b5247df（约束规划到 lanelet 可行驶域）起要求起终车辆足迹位于
    # 可行驶域内；点击选点在短 lanelet/地图边缘可能把终点落到邻接 lanelet 或
    # 域外（规划 invalid_input）。路由检查完成后，将终点数值调整到 B 中心线
    # 距起点最近的内点（保留点击得到的 B 切向航向），保证可规划且 route_to 不变。
    interior = candidates[0] if candidates else None
    goal_adjust = {"target": interior, "applied": False, "matches": False}
    if interior and goal_pose is not None:
        yaw_deg = math.degrees(goal_pose.get("yaw_rad", 0.0))
        applied = set_pose_fields_numeric(pipe, session, 1, interior[0], interior[1], yaw_deg)
        time.sleep(0.3)
        current_pose = debug_state(pipe, session).get("goal_pose") or {}
        goal_adjust.update(
            {
                "applied": bool(applied),
                "pose": current_pose,
                "matches": abs((current_pose.get("x_m") or 1e9) - interior[0]) < 1e-3
                and abs((current_pose.get("y_m") or 1e9) - interior[1]) < 1e-3,
            }
        )
    checks["w2_goal_interior_adjust"] = goal_adjust
    ok_flags.append(bool(goal_adjust.get("matches")))
    pipe.screenshot(session, out_dir / "browser_cdp_w2_route.png")
    report["screenshots"].append("browser_cdp_w2_route.png")

    # 4) 窗口查询：拖拽矩形（mid_a ± 15 m），服务返回裁剪结果
    dismiss_notice(pipe, session)
    click_button(pipe, session, "窗口查询", ".mode-button")
    pad = 15.0
    center = centerline_midpoint(by_id[a_id])
    p1 = page_point_for_world(pipe, session, center[0] - pad, center[1] - pad)
    p2 = page_point_for_world(pipe, session, center[0] + pad, center[1] + pad)
    checks["window_drag_points"] = [p1, p2]
    if p1 and p2:
        dispatch_mouse(pipe, session, "mousePressed", p1[0], p1[1], button="left", buttons=1)
        for step in range(1, 6):
            x = p1[0] + (p2[0] - p1[0]) * step / 5
            y = p1[1] + (p2[1] - p1[1]) * step / 5
            dispatch_mouse(pipe, session, "mouseMoved", x, y, button="left", buttons=1)
            time.sleep(0.05)
        dispatch_mouse(pipe, session, "mouseReleased", p2[0], p2[1], button="left", buttons=0)
    window_state = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        " return s && s.window_result && s.window_result.loading === false ? s.window_result : null; })()",
        25.0,
    )
    window_lanelets = window_state.get("lanelets", []) if window_state else []
    checks["window_result"] = window_state
    window_ok = (
        bool(window_state)
        and window_state.get("loading") is False
        and window_state.get("approximate") is False
        and len(window_lanelets) > 0
        and a_id in window_lanelets
        and all(item in lanelet_ids for item in window_lanelets)
    )
    checks["window_ok"] = window_ok
    ok_flags.append(window_ok)
    pipe.screenshot(session, out_dir / "browser_cdp_w2_window.png")
    report["screenshots"].append("browser_cdp_w2_window.png")

    return all(ok_flags)


def scene_state(pipe: ChromePipe, session: str) -> dict:
    return debug_state(pipe, session).get("scene", {}) or {}


def rect_from_obstacle(obstacle: dict) -> tuple:
    return (
        float(obstacle.get("x_min_m")),
        float(obstacle.get("y_min_m")),
        float(obstacle.get("x_max_m")),
        float(obstacle.get("y_max_m")),
    )


def obstacle_matches(obstacle: dict, expected: tuple, tolerance: float = 1e-3) -> bool:
    if obstacle.get("type") != "rectangle":
        return False
    actual = rect_from_obstacle(obstacle)
    return all(abs(a - e) <= tolerance for a, e in zip(actual, expected))


def poses_match(a, b, tolerance: float = 1e-9) -> bool:
    if a is None and b is None:
        return True
    if a is None or b is None:
        return False
    return (
        abs(a["x_m"] - b["x_m"]) <= tolerance
        and abs(a["y_m"] - b["y_m"]) <= tolerance
        and abs(a["yaw_rad"] - b["yaw_rad"]) <= tolerance
    )


def drag_world(pipe: ChromePipe, session: str, from_world, to_world, in_canvas, steps: int = 5) -> bool:
    start = page_point_for_world(pipe, session, from_world[0], from_world[1])
    end = page_point_for_world(pipe, session, to_world[0], to_world[1])
    if not (in_canvas(start) and in_canvas(end)):
        return False
    dispatch_mouse(pipe, session, "mousePressed", start[0], start[1], button="left", buttons=1)
    for step in range(1, steps + 1):
        x = start[0] + (end[0] - start[0]) * step / steps
        y = start[1] + (end[1] - start[1]) * step / steps
        dispatch_mouse(pipe, session, "mouseMoved", x, y, button="left", buttons=1)
        time.sleep(0.05)
    dispatch_mouse(pipe, session, "mouseReleased", end[0], end[1], button="left", buttons=0)
    return True


def run_w3_checks(pipe: ChromePipe, session: str, out_dir: Path, report: dict) -> bool:
    """W3 场景编辑检查：障碍新建/移动/删除、足迹渲染、保存/重载一致、版本不匹配拒绝。"""
    checks = report.setdefault("w3_checks", {})
    ok_flags: list[bool] = []

    state = scene_state(pipe, session)
    start_pose = state.get("start")
    if not start_pose:
        checks["w3_precondition"] = False
        return False
    checks["w3_precondition"] = True
    mid = [float(start_pose["x_m"]), float(start_pose["y_m"])]

    canvas_rect = pipe.evaluate(
        "(() => { const c = document.querySelector('canvas.map-canvas'); const r = c.getBoundingClientRect();"
        " return {left: r.left, top: r.top, right: r.right, bottom: r.bottom}; })()",
        session,
    )

    def in_canvas(point) -> bool:
        if not point or not canvas_rect:
            return False
        return (
            canvas_rect["left"] + 4 <= point[0] <= canvas_rect["right"] - 4
            and canvas_rect["top"] + 4 <= point[1] <= canvas_rect["bottom"] - 4
        )

    # 1) 障碍模式：画布拖拽新建 20×10 m 矩形（越界时回退面板按钮）
    checks["w3_obstacle_mode"] = click_button(pipe, session, "障碍", ".mode-button")
    dragged = drag_world(pipe, session, (mid[0] + 25.0, mid[1] - 25.0), (mid[0] + 45.0, mid[1] - 15.0), in_canvas)
    checks["w3_obstacle_add_method"] = "canvas_drag" if dragged else "panel_button_fallback"
    if not dragged:
        click_button(pipe, session, "添加障碍（视口中心）")
    time.sleep(0.5)
    state = scene_state(pipe, session)
    obstacles = state.get("obstacles", [])
    checks["w3_obstacle_count"] = len(obstacles)
    if dragged:
        expected_rect = (mid[0] + 25.0, mid[1] - 25.0, mid[0] + 45.0, mid[1] - 15.0)
        add_ok = len(obstacles) == 1 and obstacle_matches(obstacles[0], expected_rect, 0.05)
    else:
        add_ok = len(obstacles) == 1
    checks["w3_obstacle_added"] = add_ok
    ok_flags.append(add_ok)

    # 2) 画布拖动障碍 +10 m（x 方向）
    move_ok = False
    if add_ok:
        x0, y0, x1, y1 = rect_from_obstacle(obstacles[0])
        center = ((x0 + x1) / 2, (y0 + y1) / 2)
        moved_drag = drag_world(pipe, session, center, (center[0] + 10.0, center[1]), in_canvas)
        time.sleep(0.4)
        if moved_drag:
            moved = scene_state(pipe, session).get("obstacles", [{}])[0]
            move_ok = obstacle_matches(moved, (x0 + 10.0, y0, x1 + 10.0, y1), 0.05)
            checks["w3_obstacle_rect_after_move"] = list(rect_from_obstacle(moved))
    checks["w3_obstacle_moved"] = move_ok
    ok_flags.append(move_ok)

    # 3) 足迹渲染（W2 已设起终位姿）：结构 + 起点足迹填充像素（清除后应归零，见步骤 5）
    state = scene_state(pipe, session)
    pixel_state = json.loads(pipe.evaluate(STATE_EXPR, session))
    footprints = state.get("footprints", [])
    checks["w3_footprint_kinds"] = footprints
    checks["w3_footprint_pixels"] = pixel_state.get("footprintPixels")
    footprint_ok = (
        "start" in footprints
        and "goal" in footprints
        and (pixel_state.get("footprintPixels") or 0) > 10
    )
    checks["w3_footprints_rendered"] = footprint_ok
    ok_flags.append(footprint_ok)

    # 4) 保存场景（下载 JSON）→ 与调试状态逐字段一致
    pipe.call(
        "Browser.setDownloadBehavior",
        {"behavior": "allow", "downloadPath": str(out_dir), "eventsEnabled": False},
    )
    checks["w3_export_clicked"] = click_button(pipe, session, "保存场景")
    download_path = out_dir / "planner_scene_mapping_example.json"
    saved_scene = None
    deadline = time.time() + 15
    while time.time() < deadline:
        if download_path.is_file():
            try:
                saved_scene = json.loads(download_path.read_text(encoding="utf-8"))
                break
            except Exception:
                saved_scene = None
        time.sleep(0.3)
    save_ok = False
    if saved_scene:
        state = scene_state(pipe, session)
        saved_obstacles = saved_scene.get("obstacles", [])
        save_ok = (
            saved_scene.get("schema_version") == 1
            and saved_scene.get("type") == "web_scene"
            and saved_scene.get("map_id") == state.get("map_id")
            and saved_scene.get("map_version") == state.get("map_version")
            and len(saved_obstacles) == len(state.get("obstacles", []))
            and all(
                obstacle_matches(a, rect_from_obstacle(b), 1e-6)
                for a, b in zip(saved_obstacles, state.get("obstacles", []))
            )
            and poses_match(saved_scene.get("start"), state.get("start"))
            and poses_match(saved_scene.get("goal"), state.get("goal"))
        )
        checks["w3_saved_obstacles"] = saved_obstacles
    checks["w3_saved_file"] = str(download_path) if saved_scene else None
    checks["w3_save_ok"] = save_ok
    ok_flags.append(save_ok)

    # 5) 清除场景 → 状态为空；足迹填充像素归零（差分证明渲染来源）
    checks["w3_clear_clicked"] = click_button(pipe, session, "清除场景")
    time.sleep(0.5)
    cleared = scene_state(pipe, session)
    clear_ok = (
        len(cleared.get("obstacles", [])) == 0
        and cleared.get("start") is None
        and cleared.get("goal") is None
    )
    checks["w3_cleared"] = clear_ok
    ok_flags.append(clear_ok)
    cleared_pixels = json.loads(pipe.evaluate(STATE_EXPR, session)).get("footprintPixels")
    checks["w3_footprint_pixels_after_clear"] = cleared_pixels
    # 相对容差：拓扑高亮等绿色要素的抗锯齿边缘可能落入填充色容差；深缩放
    # 下足迹像素显著放大，固定阈值不再适用，按清除前像素的 5%（下限 25）判定。
    before_pixels = checks.get("w3_footprint_pixels") or 0
    clear_limit = max(25, int(before_pixels * 0.05))
    footprint_cleared_ok = (cleared_pixels or 0) <= clear_limit
    checks["w3_footprint_clear_limit"] = clear_limit
    checks["w3_footprint_cleared"] = footprint_cleared_ok
    ok_flags.append(footprint_cleared_ok)

    # 6) 导入下载文件 → 与保存场景一致（障碍/位姿逐字段）
    import_ok = False
    if saved_scene:
        pipe.call("DOM.enable", session=session)
        document = pipe.call("DOM.getDocument", {}, session)
        node = pipe.call(
            "DOM.querySelector",
            {"nodeId": document["root"]["nodeId"], "selector": "input[data-testid=scene-import-input]"},
            session,
        )
        if node.get("nodeId"):
            pipe.call(
                "DOM.setFileInputFiles",
                {"files": [str(download_path)], "nodeId": node["nodeId"]},
                session,
            )
            deadline = time.time() + 10
            while time.time() < deadline:
                state = scene_state(pipe, session)
                if len(state.get("obstacles", [])) == len(saved_scene.get("obstacles", [])) and state.get("start"):
                    break
                time.sleep(0.3)
            state = scene_state(pipe, session)
            import_ok = (
                len(state.get("obstacles", [])) == len(saved_scene.get("obstacles", []))
                and all(
                    obstacle_matches(a, rect_from_obstacle(b), 1e-6)
                    for a, b in zip(saved_scene.get("obstacles", []), state.get("obstacles", []))
                )
                and poses_match(saved_scene.get("start"), state.get("start"))
                and poses_match(saved_scene.get("goal"), state.get("goal"))
            )
    checks["w3_import_restored"] = import_ok
    ok_flags.append(import_ok)
    pipe.screenshot(session, out_dir / "browser_cdp_w3_scene.png")
    report["screenshots"].append("browser_cdp_w3_scene.png")

    # 7) 版本不匹配：写入 map_version 不同的场景 → 恢复被拒绝且不套用
    mismatch_ok = False
    if saved_scene:
        # 等待导入触发的自动保存落盘，避免坏场景被随后覆盖。
        deadline = time.time() + 6
        while time.time() < deadline:
            if "已自动保存" in (scene_state(pipe, session).get("autosave_status") or ""):
                break
            time.sleep(0.3)
        bad_scene = json.loads(json.dumps(saved_scene))
        bad_scene["map_version"] = "0" * 64
        bad_literal = json.dumps(json.dumps(bad_scene))
        stored_bad = False
        for _ in range(3):
            pipe.evaluate(f"localStorage.setItem('planner.web_scene.v1', {bad_literal})", session)
            time.sleep(0.2)
            stored = pipe.evaluate("localStorage.getItem('planner.web_scene.v1')", session)
            try:
                stored_bad = bool(stored) and json.loads(stored).get("map_version") == "0" * 64
            except Exception:
                stored_bad = False
            if stored_bad:
                break
        checks["w3_bad_scene_written"] = stored_bad
        checks["w3_restore_clicked"] = click_button(pipe, session, "从本地恢复")
        time.sleep(0.5)
        state = scene_state(pipe, session)
        panel_text = pipe.evaluate("(() => document.querySelector('.inspector').textContent)()", session) or ""
        checks["w3_mismatch_state"] = {
            "warning": state.get("mismatch_warning"),
            "obstacles": len(state.get("obstacles", [])),
            "panel_has_mismatch": "不一致" in panel_text,
        }
        mismatch_ok = (
            stored_bad
            and bool(state.get("mismatch_warning"))
            and "不一致" in (state.get("mismatch_warning") or "")
            and "不一致" in panel_text
            and len(state.get("obstacles", [])) == len(saved_scene.get("obstacles", []))
            and poses_match(saved_scene.get("start"), state.get("start"))
        )
    checks["w3_mismatch_refused"] = mismatch_ok
    ok_flags.append(mismatch_ok)

    # 8) 单击选中障碍 → 面板删除
    delete_ok = False
    state = scene_state(pipe, session)
    if state.get("obstacles"):
        obstacle = state["obstacles"][0]
        x0, y0, x1, y1 = rect_from_obstacle(obstacle)
        center_page = page_point_for_world(pipe, session, (x0 + x1) / 2, (y0 + y1) / 2)
        if in_canvas(center_page):
            click_at(pipe, session, *center_page)
            time.sleep(0.3)
            selected = scene_state(pipe, session).get("selected_obstacle_id")
            checks["w3_obstacle_selected"] = selected
            if selected == obstacle["id"]:
                click_button(pipe, session, "删除障碍")
                time.sleep(0.4)
                delete_ok = len(scene_state(pipe, session).get("obstacles", [])) == 0
    checks["w3_obstacle_deleted"] = delete_ok
    ok_flags.append(delete_ok)

    return all(ok_flags)


def count_color_pixels(pipe: ChromePipe, session: str, rgb: tuple, tolerance: int = 8) -> int:
    expression = (
        "(() => { const c = document.querySelector('canvas.map-canvas'); if (!c) return -1;"
        " const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data; let n = 0;"
        f" for (let i = 0; i < d.length; i += 4) {{ if (Math.abs(d[i] - {rgb[0]}) < {tolerance}"
        f" && Math.abs(d[i + 1] - {rgb[1]}) < {tolerance} && Math.abs(d[i + 2] - {rgb[2]}) < {tolerance}) n += 1; }}"
        " return n; })()"
    )
    return int(pipe.evaluate(expression, session) or 0)


def select_playback_sample(pipe: ChromePipe, session: str, sample_id: str, settle: float = 0.6) -> bool:
    expression = (
        "(() => { const b = document.querySelector('[data-sample-id=\"" + sample_id + "\"]');"
        " if (!b) return false; b.click(); return true; })()"
    )
    clicked = bool(pipe.evaluate(expression, session))
    time.sleep(settle)
    return clicked


def playback_state(pipe: ChromePipe, session: str) -> dict:
    return debug_state(pipe, session).get("playback", {}) or {}


def toggle_playback_layer(pipe: ChromePipe, session: str, label: str) -> bool:
    expression = (
        "(() => { const labels = [...document.querySelectorAll('.inspector label.layer-toggle')];"
        f" const l = labels.find((x) => x.textContent.trim() === '{label}'); if (!l) return false;"
        " l.querySelector('input').click(); return true; })()"
    )
    result = bool(pipe.evaluate(expression, session))
    time.sleep(0.3)
    return result


def scroll_into_view(pipe: ChromePipe, session: str, selector: str, block: str = "center") -> None:
    pipe.evaluate(
        f"(() => {{ const el = document.querySelector('{selector}'); if (el) el.scrollIntoView({{ block: '{block}' }}); }})()",
        session,
    )
    time.sleep(0.3)


def set_slider_value(pipe: ChromePipe, session: str, selector: str, value: float) -> bool:
    expression = (
        f"(() => {{ const s = document.querySelector('{selector}'); if (!s) return false;"
        " const setter = Object.getOwnPropertyDescriptor(window.HTMLInputElement.prototype, 'value').set;"
        f" setter.call(s, '{value}'); s.dispatchEvent(new Event('input', {{ bubbles: true }})); return true; }})()"
    )
    result = bool(pipe.evaluate(expression, session))
    time.sleep(0.4)
    return result


def run_w4_checks(pipe: ChromePipe, session: str, out_dir: Path, report: dict) -> bool:
    """W4 固定回放检查（无需算法服务）：逐类加载、回放推进、v/κ 视图、碰撞点、过期警告。"""
    checks = report.setdefault("w4_checks", {})
    ok_flags: list[bool] = []

    # 1) 加载回放样例（SHA256 校验；合成地图切换）
    checks["w4_load_clicked"] = click_button(pipe, session, "回放样例", ".toolbar button")
    loaded = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        " const p = s && s.playback; return p && p.active && p.loaded && p.sample_count > 0 ? p : null; })()",
        30.0,
    )
    checks["w4_sample_count"] = loaded.get("sample_count") if loaded else 0
    checks["w4_issues"] = loaded.get("issues") if loaded else None
    checks["w4_map_file"] = loaded.get("map_file") if loaded else None
    load_ok = (
        bool(loaded)
        and loaded.get("sample_count") == 11
        and not loaded.get("issues")
        and loaded.get("map_file") == "map_snapshot_minimal.json"
    )
    checks["w4_load_ok"] = load_ok
    ok_flags.append(load_ok)
    if not load_ok:
        return False

    panel_text = pipe.evaluate("(() => document.querySelector('.inspector').textContent)()", session) or ""
    scene = scene_state(pipe, session)
    checks["w4_synthetic_banner"] = "非算法正确性证据" in panel_text
    checks["w4_map_switched"] = scene.get("map_id") == "map_synthetic_min"
    ok_flags.append(checks["w4_synthetic_banner"] and checks["w4_map_switched"])

    # 2) 逐类样例（结构 + 像素证据）
    forward_rgb = (56, 189, 248)
    reverse_rgb = (245, 158, 11)
    curve_rgb = (167, 139, 250)
    corridor_rgb = (12, 48, 50)
    trajectory_rgb = (96, 165, 250)
    # 重叠区填充 rgba(253,224,71,0.35) 叠加在障碍填充之上：约 (122,96,50)。
    overlap_rgb = (122, 96, 50)
    per_sample: dict = {}

    def record(sample_id: str, result: dict) -> None:
        per_sample[sample_id] = result

    select_playback_sample(pipe, session, "rs_path_open")
    state = playback_state(pipe, session)
    forward_px = count_color_pixels(pipe, session, forward_rgb)
    record("rs_path_open", {"segments": state.get("path_segments"), "points": state.get("path_points"), "forward_px": forward_px})
    ok_flags.append(state.get("path_segments") == 2 and state.get("path_points") == 8 and forward_px > 50)

    select_playback_sample(pipe, session, "hybrid_path_success")
    state = playback_state(pipe, session)
    forward_px = count_color_pixels(pipe, session, forward_rgb)
    record("hybrid_path_success", {"segments": state.get("path_segments"), "points": state.get("path_points"), "forward_px": forward_px})
    ok_flags.append(state.get("path_segments") == 1 and state.get("path_points") == 5 and forward_px > 50)

    select_playback_sample(pipe, session, "hybrid_path_reverse")
    state = playback_state(pipe, session)
    forward_px = count_color_pixels(pipe, session, forward_rgb)
    reverse_px = count_color_pixels(pipe, session, reverse_rgb)
    record(
        "hybrid_path_reverse",
        {"segments": state.get("path_segments"), "cusps": state.get("cusps"), "forward_px": forward_px, "reverse_px": reverse_px},
    )
    ok_flags.append(
        state.get("path_segments") == 2
        and state.get("cusps") == 1
        and forward_px > 30
        and reverse_px > 30
    )

    select_playback_sample(pipe, session, "hybrid_path_failure")
    state = playback_state(pipe, session)
    panel_text = pipe.evaluate("(() => document.querySelector('.inspector').textContent)()", session) or ""
    record("hybrid_path_failure", {"status": state.get("status"), "segments": state.get("path_segments"), "panel_has_status": "no_path" in panel_text})
    ok_flags.append(state.get("status") == "no_path" and state.get("path_segments") == 0 and "no_path" in panel_text)

    select_playback_sample(pipe, session, "bspline_curve_demo")
    state = playback_state(pipe, session)
    curve_px = count_color_pixels(pipe, session, curve_rgb)
    record("bspline_curve_demo", {"curve_points": state.get("curve_points"), "curve_px": curve_px})
    ok_flags.append(state.get("curve_points") == 200 and curve_px > 50)

    select_playback_sample(pipe, session, "polynomial_curve_demo")
    state = playback_state(pipe, session)
    curve_px = count_color_pixels(pipe, session, curve_rgb)
    record("polynomial_curve_demo", {"curve_points": state.get("curve_points"), "curve_px": curve_px})
    ok_flags.append(state.get("curve_points") == 200 and curve_px > 50)

    select_playback_sample(pipe, session, "corridor_hybrid_reverse")
    state = playback_state(pipe, session)
    corridor_px = count_color_pixels(pipe, session, corridor_rgb, 10)
    record("corridor_hybrid_reverse", {"polygons": state.get("corridor_polygons"), "corridor_px": corridor_px})
    ok_flags.append(state.get("corridor_polygons") == 3 and corridor_px > 50)
    scroll_into_view(pipe, session, "[data-testid=playback-panel]", "start")
    pipe.screenshot(session, out_dir / "browser_cdp_w4_corridor.png")
    report["screenshots"].append("browser_cdp_w4_corridor.png")

    select_playback_sample(pipe, session, "collision_demo")
    state = playback_state(pipe, session)
    overlap_px = count_color_pixels(pipe, session, overlap_rgb, 10)
    record("collision_demo", {"collision_visible": state.get("collision_visible"), "overlap_px": overlap_px})
    ok_flags.append(state.get("collision_visible") is True and overlap_px > 20)
    pipe.screenshot(session, out_dir / "browser_cdp_w4_collision.png")
    report["screenshots"].append("browser_cdp_w4_collision.png")

    select_playback_sample(pipe, session, "stale_result_demo")
    state = playback_state(pipe, session)
    panel_text = pipe.evaluate("(() => document.querySelector('.inspector').textContent)()", session) or ""
    record(
        "stale_result_demo",
        {
            "stale_warning": state.get("stale_warning"),
            "segments": state.get("path_segments"),
            "panel_has_stale": "过期" in panel_text,
        },
    )
    ok_flags.append(
        bool(state.get("stale_warning"))
        and "过期" in (state.get("stale_warning") or "")
        and "过期" in panel_text
        and state.get("path_segments") == 0
    )

    # 3) 时间回放推进 + 速度/曲率视图（Dftpav 轨迹）
    select_playback_sample(pipe, session, "dftpav_trajectory_demo")
    state = playback_state(pipe, session)
    trajectory_px = count_color_pixels(pipe, session, trajectory_rgb, 8)
    charts = pipe.evaluate(
        "(() => ({ velocity: !!document.querySelector('[data-testid=profile-velocity]'),"
        " curvature: !!document.querySelector('[data-testid=profile-curvature]'),"
        " slider: !!document.querySelector('[data-testid=playback-slider]') }))()",
        session,
    )
    record(
        "dftpav_trajectory_demo",
        {
            "trajectory_points": state.get("trajectory_points"),
            "duration_s": state.get("duration_s"),
            "profile_points": state.get("profile_points"),
            "trajectory_px": trajectory_px,
            "charts": charts,
        },
    )
    ok_flags.append(
        state.get("trajectory_points") == 8
        and abs((state.get("duration_s") or 0) - 7.5) < 1e-9
        and state.get("profile_points") == 8
        and trajectory_px > 20
        and charts.get("velocity")
        and charts.get("curvature")
        and charts.get("slider")
    )
    scroll_into_view(pipe, session, "[data-testid=profile-velocity]")
    pipe.screenshot(session, out_dir / "browser_cdp_w4_trajectory.png")
    report["screenshots"].append("browser_cdp_w4_trajectory.png")

    # 播放推进
    checks["w4_play_clicked"] = click_button(pipe, session, "播放", ".inspector button")
    time.sleep(1.4)
    advanced = playback_state(pipe, session)
    checks["w4_time_after_play"] = advanced.get("playback_time_s")
    play_ok = (advanced.get("playback_time_s") or 0) > 0.5
    click_button(pipe, session, "暂停", ".inspector button")
    time.sleep(0.2)
    paused = playback_state(pipe, session)
    checks["w4_paused"] = paused.get("playing") is False
    play_ok = play_ok and paused.get("playing") is False
    checks["w4_play_advance_ok"] = play_ok
    ok_flags.append(play_ok)

    # 进度拖拽 → 时刻与当前位姿联动
    checks["w4_seek_set"] = set_slider_value(pipe, session, "[data-testid=playback-slider]", 6.0)
    sought = playback_state(pipe, session)
    pose = sought.get("current_pose") or {}
    seek_ok = (
        abs((sought.get("playback_time_s") or -1) - 6.0) < 0.05
        and abs((pose.get("x_m") or -1) - 37.0) < 0.2
        and (pose.get("velocity_mps") or 0) < 0
    )
    checks["w4_seek_pose"] = {"time": sought.get("playback_time_s"), "x_m": pose.get("x_m"), "velocity_mps": pose.get("velocity_mps")}
    checks["w4_seek_ok"] = seek_ok
    ok_flags.append(seek_ok)

    # 4) 图层开关：关闭路径 → 路径几何消失；恢复
    select_playback_sample(pipe, session, "rs_path_open")
    checks["w4_layer_toggle_clicked"] = toggle_playback_layer(pipe, session, "路径")
    state = playback_state(pipe, session)
    off_ok = state.get("path_segments") == 0
    toggle_playback_layer(pipe, session, "路径")
    state = playback_state(pipe, session)
    on_ok = state.get("path_segments") == 2
    checks["w4_layer_toggle_ok"] = off_ok and on_ok
    ok_flags.append(off_ok and on_ok)

    checks["w4_per_sample"] = per_sample
    return all(ok_flags)


def _submit_filler_jobs(target: str, scene: dict, count: int, round_index: int) -> int:
    """直连提交填充任务占住服务队列（单工作线程按提交顺序执行），扩大取消窗口。"""
    start = scene.get("start")
    goal = scene.get("goal")
    if not start or not goal or not target:
        return 0
    submitted = 0
    for index in range(count):
        payload = {
            "request_id": f"cdp-filler-{round_index}-{index}",
            "map_id": scene.get("map_id"),
            "map_version": scene.get("map_version"),
            "start": start,
            "goal": goal,
            "vehicle_ref": scene.get("vehicle_source") or "tests/fixtures/v3/vehicles/default.json",
            "obstacles": scene.get("obstacles") or [],
            "algorithm_params": {"hybrid_backend": "kth_normalized"},
        }
        data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            f"{target.rstrip('/')}/plan/submit",
            data=data,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=15) as response:
            if response.status == 200:
                submitted += 1
    return submitted


def run_w5_checks(
    pipe: ChromePipe,
    session: str,
    out_dir: Path,
    report: dict,
    mode: str = "full",
    expect_ws: bool = False,
    plan_api_target: str | None = None,
) -> bool:
    """W5 规划服务检查：提交→进度→结果、比较、取消、陈旧事件、stale_map。"""
    checks = report.setdefault("w5_checks", {})
    ok_flags: list[bool] = []

    # 退出回放（W4 可能仍在合成地图上），恢复默认地图
    playback = playback_state(pipe, session)
    if playback.get("active"):
        click_button(pipe, session, "退出回放", ".inspector button")
        wait_for_condition(
            pipe,
            session,
            "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
            " return s && s.scene && s.scene.map_id === 'mapping_example' ? s : null; })()",
            20.0,
        )
    scene = scene_state(pipe, session)
    checks["w5_map_id"] = scene.get("map_id")
    checks["w5_poses_set"] = scene.get("start") is not None and scene.get("goal") is not None

    health = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        " const p = s && s.plan; return p && p.health === 'online' ? p : null; })()",
        20.0,
    )
    checks["w5_health_online"] = bool(health)
    ok_flags.append(bool(health))
    if not health:
        return False

    # stale_map 模式：提交应被服务拒绝并显式提示
    if mode == "stale_map":
        checks["w5_stale_map_submit_clicked"] = click_button(pipe, session, "提交规划", ".inspector button")
        time.sleep(1.0)
        state = plan_state(pipe, session)
        panel_text = pipe.evaluate("(() => document.querySelector('.inspector').textContent)()", session) or ""
        error_text = state.get("error") or ""
        runs = state.get("runs") or []
        checks["w5_stale_map_error"] = error_text
        checks["w5_stale_map_panel_has"] = "stale_map" in panel_text or "版本不匹配" in panel_text
        stale_ok = (
            "stale_map" in error_text
            and all(not run.get("job_id") for run in runs)
        )
        checks["w5_stale_map_blocked"] = stale_ok
        ok_flags.append(stale_ok)
        return all(ok_flags)

    # 提交（单后端）并采样进度（含事件推送写入的状态/阶段变迁历史）
    checks["w5_submit_clicked"] = click_button(pipe, session, "提交规划", ".inspector button")
    observed_statuses: list[str] = []
    observed_stages: list[str] = []
    deadline = time.time() + 20
    run = None
    while time.time() < deadline:
        state = plan_state(pipe, session)
        runs = state.get("runs") or []
        if runs:
            run = runs[0]
            for entry in run.get("history") or []:
                entry_status = str(entry.get("status") or "")
                entry_stage = str(entry.get("stage") or "")
                if entry_status:
                    observed_statuses.append(entry_status)
                if entry_stage:
                    observed_stages.append(entry_stage)
            if run.get("has_result"):
                break
        time.sleep(0.25)
    checks["w5_observed_statuses"] = sorted(set(observed_statuses))
    checks["w5_observed_stages"] = sorted(set(observed_stages))
    checks["w5_run_reason"] = run.get("reason") if run else None
    checks["w5_scene"] = {
        "start": scene.get("start"),
        "goal": scene.get("goal"),
        "obstacles": scene.get("obstacles"),
    }
    submit_ok = (
        run is not None
        and bool(run.get("job_id"))
        and bool(run.get("request_id"))
        and run.get("has_result") is True
        and run.get("status") in ("succeeded", "success")
    )
    checks["w5_submit_ok"] = submit_ok
    ok_flags.append(submit_ok)
    # 进度证据：至少观察到 queued/running（快速服务可能跳过 running；阶段另行记录）
    progress_ok = "queued" in observed_statuses or "running" in observed_statuses
    checks["w5_progress_observed"] = progress_ok
    ok_flags.append(progress_ok)
    state_ws = plan_state(pipe, session)
    checks["w5_push_transport"] = state_ws.get("push_transport")
    checks["w5_ws_state"] = state_ws.get("ws_state")
    checks["w5_ws_url"] = state_ws.get("ws_url")
    if expect_ws:
        ws_ok = state_ws.get("push_transport") == "websocket"
        checks["w5_ws_ok"] = ws_ok
        ok_flags.append(ws_ok)
    if not submit_ok:
        return False

    # 结果渲染（复用 W4 管线）
    state = plan_state(pipe, session)
    result_ok = (
        state.get("result_map_matches") is True
        and (state.get("path_points") or 0) > 0
        and (state.get("corridor_polygons") or 0) > 0
        and (state.get("trajectory_points") or 0) > 0
        and bool(run.get("validation"))
        and run["validation"].get("valid") is True
    )
    checks["w5_result_rendered"] = result_ok
    checks["w5_result_summary"] = {
        "path_points": state.get("path_points"),
        "corridor_polygons": state.get("corridor_polygons"),
        "trajectory_points": state.get("trajectory_points"),
        "validation": run.get("validation"),
        "duration_s": state.get("duration_s"),
    }
    ok_flags.append(result_ok)
    panel_text = pipe.evaluate("(() => document.querySelector('.inspector').textContent)()", session) or ""
    checks["w5_panel_labels"] = "请求 ID" in panel_text and "地图版本" in panel_text
    ok_flags.append(checks["w5_panel_labels"])
    pipe.evaluate(
        "(() => { const p = document.querySelector('[data-testid=plan-panel]');"
        " if (p) p.scrollIntoView({ block: 'start' }); return true; })()",
        session,
    )
    time.sleep(0.3)
    pipe.screenshot(session, out_dir / "browser_cdp_w5_result.png")
    report["screenshots"].append("browser_cdp_w5_result.png")

    # 结果轨迹回放：播放推进 + 拖拽联动 + v/κ 小图
    charts = pipe.evaluate(
        "(() => ({ velocity: !!document.querySelector('[data-testid=plan-profile-velocity]'),"
        " curvature: !!document.querySelector('[data-testid=plan-profile-curvature]'),"
        " slider: !!document.querySelector('[data-testid=plan-slider]') }))()",
        session,
    )
    checks["w5_charts"] = charts
    ok_flags.append(bool(charts.get("velocity") and charts.get("curvature") and charts.get("slider")))
    checks["w5_play_clicked"] = click_button(pipe, session, "播放", ".inspector button")
    time.sleep(1.2)
    advanced = plan_state(pipe, session)
    checks["w5_time_after_play"] = advanced.get("time_s")
    play_ok = (advanced.get("time_s") or 0) > 0.4
    click_button(pipe, session, "暂停", ".inspector button")
    time.sleep(0.2)
    checks["w5_play_ok"] = play_ok
    ok_flags.append(play_ok)
    checks["w5_seek_set"] = set_slider_value(pipe, session, "[data-testid=plan-slider]", 2.0)
    sought = plan_state(pipe, session)
    seek_ok = abs((sought.get("time_s") or -1) - 2.0) < 0.1
    checks["w5_seek_time"] = sought.get("time_s")
    checks["w5_seek_ok"] = seek_ok
    ok_flags.append(seek_ok)

    # 陈旧事件：旧 seq 被忽略、新 seq 被接受
    run_state = (plan_state(pipe, session).get("runs") or [{}])[0]
    job_id = run_state.get("job_id")
    latest_seq = plan_state(pipe, session).get("latest_seq")
    stale_result = pipe.evaluate(
        "(() => window.__plannerDebug.injectPlanEvent({ kind: 'job_update', job_seq: "
        + str(max(0, int(latest_seq or 0) - 1))
        + ", job_id: '"
        + str(job_id)
        + "', status: 'running', stage: 'stale-injected' }))()",
        session,
    )
    time.sleep(0.3)
    after_stale = plan_state(pipe, session)
    stale_run = (after_stale.get("runs") or [{}])[0]
    stale_ok = (
        stale_result is not None
        and stale_result.get("accepted") is False
        and (after_stale.get("stale_ignored") or 0) >= 1
        and stale_run.get("stage") != "stale-injected"
    )
    checks["w5_stale_event"] = {"inject_result": stale_result, "stale_ignored": after_stale.get("stale_ignored")}
    checks["w5_stale_event_ignored"] = stale_ok
    ok_flags.append(stale_ok)

    fresh_result = pipe.evaluate(
        "(() => window.__plannerDebug.injectPlanEvent({ kind: 'job_update', job_seq: "
        + str(int(latest_seq or 0) + 1)
        + ", job_id: '"
        + str(job_id)
        + "', status: 'running', stage: 'fresh-injected' }))()",
        session,
    )
    time.sleep(0.3)
    after_fresh = plan_state(pipe, session)
    fresh_run = (after_fresh.get("runs") or [{}])[0]
    fresh_ok = (
        fresh_result is not None
        and fresh_result.get("accepted") is True
        and fresh_run.get("stage") == "fresh-injected"
    )
    checks["w5_fresh_event"] = {"inject_result": fresh_result, "stage": fresh_run.get("stage")}
    checks["w5_fresh_event_accepted"] = fresh_ok
    ok_flags.append(fresh_ok)

    # 取消：快速服务可能在取消前完成 → 先用直连提交填充任务占住服务队列（单工作线程
    # 按提交顺序执行），再经 UI 提交被跟踪任务并立即取消；窗口仍不足时重试。
    cancel_ok = False
    cancel_attempts: list[dict] = []
    scene_now = scene_state(pipe, session)
    for attempt in range(3):
        compare_attempt = attempt == 2
        filler_count = 0
        try:
            filler_count = _submit_filler_jobs(
                plan_api_target or "",
                scene_now,
                count=(40, 60, 80)[attempt],
                round_index=attempt,
            )
        except Exception as error:  # noqa: BLE001
            checks["w5_cancel_filler_error"] = str(error)
        click_button(pipe, session, "对比提交" if compare_attempt else "提交规划", ".inspector button")
        clicked = False
        outcome = "no_run"
        deadline = time.time() + 6.0
        while time.time() < deadline:
            state = plan_state(pipe, session)
            runs = state.get("runs") or []
            if runs:
                active = next(
                    (run for run in runs if run.get("backend") == state.get("active_backend")),
                    runs[-1],
                )
                status = active.get("status")
                if status in ("queued", "running") and not clicked:
                    clicked = click_button(pipe, session, "取消任务", ".inspector button")
                    time.sleep(0.02)
                    continue
                if any(run.get("status") == "cancelled" for run in runs):
                    cancel_ok = True
                    outcome = "cancelled"
                    break
                if clicked and status in ("succeeded", "failed", "cancelled"):
                    outcome = f"finished_after_cancel_click:{status}"
                    break
                if not clicked and status in ("succeeded", "failed"):
                    outcome = "finished_before_cancel"
                    break
            time.sleep(0.04)
        cancel_attempts.append(
            {
                "attempt": attempt + 1,
                "compare": compare_attempt,
                "fillers": filler_count,
                "clicked": clicked,
                "outcome": outcome,
            }
        )
        if cancel_ok:
            break
    checks["w5_cancel_clicked"] = any(item["clicked"] for item in cancel_attempts)
    checks["w5_cancel_attempts"] = cancel_attempts
    checks["w5_cancelled"] = cancel_ok
    ok_flags.append(cancel_ok)

    # 对比提交（两个后端）→ 并排统计表
    checks["w5_compare_submit_clicked"] = click_button(pipe, session, "对比提交", ".inspector button")
    compare_ok = False
    deadline = time.time() + 25
    while time.time() < deadline:
        state = plan_state(pipe, session)
        runs = state.get("runs") or []
        with_results = [run for run in runs if run.get("has_result")]
        if len(with_results) >= 2:
            backends = sorted(run.get("backend") for run in runs)
            compare_ok = backends == ["kth_normalized", "ros_free"]
            break
        time.sleep(0.3)
    checks["w5_compare_runs"] = [
        {"backend": run.get("backend"), "status": run.get("status"), "has_result": run.get("has_result")}
        for run in (plan_state(pipe, session).get("runs") or [])
    ]
    checks["w5_compare_ok"] = compare_ok
    ok_flags.append(compare_ok)
    table_text = pipe.evaluate(
        "(() => { const t = document.querySelector('[data-testid=plan-comparison]'); return t ? t.textContent : null; })()",
        session,
    ) or ""
    table_ok = (
        "kth_normalized" in table_text
        and "ros_free" in table_text
        and "规划耗时" in table_text
        and "换向次数" in table_text
    )
    checks["w5_compare_table_ok"] = table_ok
    ok_flags.append(table_ok)
    pipe.evaluate(
        "(() => { const t = document.querySelector('[data-testid=plan-comparison]');"
        " if (t) t.scrollIntoView({ block: 'center' }); return true; })()",
        session,
    )
    time.sleep(0.3)
    pipe.screenshot(session, out_dir / "browser_cdp_w5_compare.png")
    report["screenshots"].append("browser_cdp_w5_compare.png")

    # 客户端阻止：回放合成地图（服务地图不匹配）时提交被阻止（按钮禁用 + 面板提示）
    checks["w5_block_playback_clicked"] = click_button(pipe, session, "回放样例", ".toolbar button")
    wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        " return s && s.playback && s.playback.active ? s : null; })()",
        30.0,
    )
    blocked_dom = pipe.evaluate(
        "(() => { const b = document.querySelector('[data-testid=plan-submit]');"
        " const t = document.querySelector('.inspector').textContent;"
        " return { disabled: b ? b.disabled : null, panel_has: t.includes('禁止提交') || t.includes('版本不匹配') }; })()",
        session,
    )
    click_button(pipe, session, "提交规划", ".inspector button")
    time.sleep(0.5)
    blocked = plan_state(pipe, session)
    blocked_ok = (blocked_dom.get("disabled") is True and blocked_dom.get("panel_has") is True) or (
        "版本不匹配" in (blocked.get("error") or "")
    )
    checks["w5_client_block"] = {
        "disabled": blocked_dom.get("disabled"),
        "panel_has": blocked_dom.get("panel_has"),
        "error": blocked.get("error"),
        "blocked": blocked_ok,
    }
    checks["w5_client_block_ok"] = blocked_ok
    ok_flags.append(blocked_ok)
    click_button(pipe, session, "退出回放", ".inspector button")
    time.sleep(0.5)

    return all(ok_flags)


def plan_state(pipe: ChromePipe, session: str) -> dict:
    return debug_state(pipe, session).get("plan", {}) or {}


# ---------------------------------------------------------------------------
# X-W1 栅格夹具检查（离线模式，无服务依赖）
# ---------------------------------------------------------------------------


def set_control_value(pipe: ChromePipe, session: str, selector: str, value) -> bool:
    """React 受控控件赋值（原生 setter + input/change 事件）。"""
    script = (
        "(() => { const element = document.querySelector(%s); if (!element) return false;"
        " const descriptor = Object.getOwnPropertyDescriptor(element.constructor.prototype, 'value');"
        " if (descriptor && descriptor.set) descriptor.set.call(element, %s); else element.value = %s;"
        " element.dispatchEvent(new Event('input', { bubbles: true }));"
        " element.dispatchEvent(new Event('change', { bubbles: true })); return true; })()"
        % (json.dumps(selector), json.dumps(value), json.dumps(value))
    )
    return bool(pipe.evaluate(script, session))


def grid_debug(pipe: ChromePipe, session: str) -> dict:
    raw = pipe.evaluate(
        "JSON.stringify(window.__plannerDebug ? window.__plannerDebug.getState().grid : null)",
        session,
    )
    return json.loads(raw) if raw else {}


def canvas_screen_point(pipe: ChromePipe, session: str, x: float, y: float):
    return pipe.evaluate(
        "(() => { const d = window.__plannerDebug; if (!d) return null;"
        f" return d.worldToScreen({x}, {y}); }})()",
        session,
    )


def canvas_pixel_at(pipe: ChromePipe, session: str, sx: float, sy: float):
    """画布内部像素采样（sx/sy 为相对画布的 CSS 像素）。"""
    script = (
        "(() => { const canvas = document.querySelector('canvas.map-canvas'); if (!canvas) return null;"
        " const rect = canvas.getBoundingClientRect(); const dpr = canvas.width / rect.width;"
        f" const x = Math.round({sx} * dpr); const y = Math.round({sy} * dpr);"
        " const data = canvas.getContext('2d').getImageData(x, y, 1, 1).data;"
        " return [data[0], data[1], data[2], data[3]]; })()"
    )
    return pipe.evaluate(script, session)


def click_grid_button(pipe: ChromePipe, session: str, label: str) -> bool:
    return bool(
        pipe.evaluate(
            "(() => { const buttons = [...document.querySelectorAll('.grid-actions button')];"
            f" const target = buttons.find((b) => b.textContent.includes({json.dumps(label)}));"
            " if (target) target.click(); return !!target; })()",
            session,
        )
    )


def run_grid_checks(pipe: ChromePipe, session: str, out_dir: Path, report: dict, base_url: str) -> bool:
    """X-W1 栅格夹具渲染检查：夹具加载、图例、旋转原点拾取、缓存复用、错误状态。"""
    checks: dict = {}
    report["grid_checks"] = checks
    all_ok = True

    def mark(key: str, condition: bool) -> bool:
        nonlocal all_ok
        checks[key] = bool(condition)
        all_ok = bool(condition) and all_ok
        return bool(condition)

    # 独立页面状态：W3 可能停留在障碍/窗口模式，画布单击会被交互模式截获而非拾取单元。
    navigate_app(pipe, session, f"{base_url}?grid=1")
    if not wait_for_condition(
        pipe, session, "document.querySelector('[data-testid=grid-panel]') !== null", timeout=15.0
    ):
        mark("grid_panel_present", False)
        return False
    mark("grid_panel_present", True)

    # 目录为异步加载：等待下拉出现完整目录后再读取（避免竞态导致的假失败）。
    wait_for_condition(
        pipe,
        session,
        "(() => { const s = document.querySelector('[data-testid=grid-file-select]');"
        " return s && s.options.length >= 12 ? true : null; })()",
        timeout=10.0,
    )
    options = pipe.evaluate(
        "(() => { const select = document.querySelector('[data-testid=grid-file-select]');"
        " return select ? [...select.options].map((option) => option.value) : null; })()",
        session,
    )
    checks["grid_catalog_options"] = options
    mark(
        "grid_catalog_ok",
        isinstance(options, list)
        and len(options) >= 12
        and "rotated_origin_ccw.json" in options
        and "snapshot_small.json" in options
        and "display_1024.json" in options,
    )

    legend = pipe.evaluate(
        "(() => { const n = document.querySelector('[data-testid=grid-legend]'); return n ? n.innerText : null; })()",
        session,
    )
    checks["grid_legend_text"] = legend
    mark(
        "grid_legend_ok",
        isinstance(legend, str) and all(label in legend for label in ("空闲", "占据", "未知")),
    )
    banner = pipe.evaluate(
        "(() => { const n = document.querySelector('.grid-synthetic-banner'); return n ? n.innerText : null; })()",
        session,
    )
    checks["grid_synthetic_banner"] = banner
    mark(
        "grid_synthetic_banner_ok",
        isinstance(banner, str) and "非算法正确性证据" in banner,
    )

    # 1) G1 旋转原点：加载、启用、拾取 (1,2) = unknown
    set_control_value(pipe, session, "[data-testid=grid-file-select]", "rotated_origin_ccw.json")
    ready = wait_for_condition(
        pipe,
        session,
        "window.__plannerDebug && window.__plannerDebug.getState().grid.status === 'ready'"
        " && window.__plannerDebug.getState().grid.kind === 'grid_view'",
        timeout=15.0,
    )
    mark("grid_g1_loaded", bool(ready))
    pipe.evaluate(
        "(() => { const box = document.querySelector('[data-testid=grid-enabled]');"
        " if (box && !box.checked) box.click(); return true; })()",
        session,
    )
    pipe.evaluate(
        "(() => { const box = document.querySelector('[data-testid=grid-pick-mode]');"
        " if (box && !box.checked) box.click(); return true; })()",
        session,
    )
    click_grid_button(pipe, session, "定位到栅格")
    time.sleep(0.5)

    point = page_point_for_world(pipe, session, 9.25, -3.75)
    if point:
        click_at(pipe, session, point[0], point[1], settle=0.5)
    picked = wait_for_condition(
        pipe,
        session,
        "(() => { const d = window.__plannerDebug; if (!d) return false;"
        " const cell = d.getState().grid.selected_cell;"
        " return !!cell && cell.row === 1 && cell.col === 2; })()",
        timeout=10.0,
    )
    selected = grid_debug(pipe, session).get("selected_cell") or {}
    checks["grid_g1_selected"] = selected
    mark("grid_g1_pick_ok", bool(picked) and selected.get("state") == 2)
    mark(
        "grid_g1_center_ok",
        abs(float(selected.get("center_x_m", 0.0)) - 9.25) < 1e-6
        and abs(float(selected.get("center_y_m", 0.0)) + 3.75) < 1e-6,
    )
    panel_rowcol = pipe.evaluate(
        "(() => { const n = document.querySelector('[data-testid=grid-cell-rowcol]'); return n ? n.innerText : null; })()",
        session,
    )
    panel_state = pipe.evaluate(
        "(() => { const n = document.querySelector('[data-testid=grid-cell-state]'); return n ? n.innerText : null; })()",
        session,
    )
    panel_center = pipe.evaluate(
        "(() => { const n = document.querySelector('[data-testid=grid-cell-center]'); return n ? n.innerText : null; })()",
        session,
    )
    checks["grid_g1_panel"] = {
        "rowcol": panel_rowcol,
        "state": panel_state,
        "center": panel_center,
    }
    mark(
        "grid_g1_panel_ok",
        "row 1" in (panel_rowcol or "")
        and "col 2" in (panel_rowcol or "")
        and "未知" in (panel_state or "")
        and "9.25" in (panel_center or ""),
    )

    # 三态像素抽查（颜色通道相对关系，避免与矢量要素颜色混淆）
    occupied_point = canvas_screen_point(pipe, session, 9.75, -4.75)
    free_point = canvas_screen_point(pipe, session, 9.75, -4.25)
    unknown_point = canvas_screen_point(pipe, session, 9.25, -3.75)
    occupied_px = canvas_pixel_at(pipe, session, *occupied_point) if occupied_point else None
    free_px = canvas_pixel_at(pipe, session, *free_point) if free_point else None
    unknown_px = canvas_pixel_at(pipe, session, *unknown_point) if unknown_point else None
    checks["grid_pixels"] = {"occupied": occupied_px, "free": free_px, "unknown": unknown_px}
    mark(
        "grid_pixels_ok",
        bool(occupied_px)
        and bool(free_px)
        and bool(unknown_px)
        and occupied_px[0] > occupied_px[1]
        and free_px[1] > free_px[0]
        and unknown_px[2] > unknown_px[0],
    )
    pipe.screenshot(session, out_dir / "grid_g1_rotated_ccw.png")
    report["screenshots"].append("grid_g1_rotated_ccw.png")

    # 2) 缩放/平移复用纹理缓存（重建次数不增加）
    before = grid_debug(pipe, session)
    rebuilds_before = before.get("texture_rebuilds")
    viewport_before = debug_state(pipe, session).get("viewport") or {}
    pipe.call(
        "Input.dispatchMouseEvent",
        {
            "type": "mouseWheel",
            "x": VIEWPORT_WIDTH / 2,
            "y": VIEWPORT_HEIGHT / 2,
            "deltaX": 0,
            "deltaY": -240,
            "button": "none",
            "buttons": 0,
        },
        session,
    )
    time.sleep(0.3)
    dispatch_mouse(pipe, session, "mousePressed", 700, 450, button="left", buttons=1)
    for step in range(1, 5):
        dispatch_mouse(pipe, session, "mouseMoved", 700 + 15 * step, 450 + 8 * step, button="left", buttons=1)
        time.sleep(0.04)
    dispatch_mouse(pipe, session, "mouseReleased", 760, 482, button="left", buttons=0)
    time.sleep(0.4)
    after = grid_debug(pipe, session)
    viewport_after = debug_state(pipe, session).get("viewport") or {}
    checks["grid_cache"] = {
        "rebuilds_before": rebuilds_before,
        "rebuilds_after": after.get("texture_rebuilds"),
        "scale_before": viewport_before.get("scale_px_per_m"),
        "scale_after": viewport_after.get("scale_px_per_m"),
        "center_before": [viewport_before.get("center_x_m"), viewport_before.get("center_y_m")],
        "center_after": [viewport_after.get("center_x_m"), viewport_after.get("center_y_m")],
    }
    mark(
        "grid_cache_ok",
        rebuilds_before is not None
        and after.get("texture_rebuilds") == rebuilds_before
        and viewport_after.get("scale_px_per_m") != viewport_before.get("scale_px_per_m")
        and viewport_after.get("center_x_m") != viewport_before.get("center_x_m"),
    )

    # 3) snapshot_small（base64 解码文档）拾取 (1,2) = unknown
    set_control_value(pipe, session, "[data-testid=grid-file-select]", "snapshot_small.json")
    ready = wait_for_condition(
        pipe,
        session,
        "window.__plannerDebug && window.__plannerDebug.getState().grid.status === 'ready'"
        " && window.__plannerDebug.getState().grid.kind === 'grid_snapshot'",
        timeout=15.0,
    )
    mark("grid_snapshot_loaded", bool(ready))
    click_grid_button(pipe, session, "定位到栅格")
    time.sleep(0.5)
    point = page_point_for_world(pipe, session, 0.25, 0.15)
    if point:
        click_at(pipe, session, point[0], point[1], settle=0.5)
    picked = wait_for_condition(
        pipe,
        session,
        "(() => { const d = window.__plannerDebug; if (!d) return false;"
        " const cell = d.getState().grid.selected_cell;"
        " return !!cell && cell.row === 1 && cell.col === 2 && cell.state === 2; })()",
        timeout=10.0,
    )
    selected = grid_debug(pipe, session).get("selected_cell") or {}
    checks["grid_snapshot_selected"] = selected
    mark(
        "grid_snapshot_pick_ok",
        bool(picked)
        and abs(float(selected.get("center_x_m", 0.0)) - 0.25) < 1e-6
        and abs(float(selected.get("center_y_m", 0.0)) - 0.15) < 1e-6,
    )
    pipe.screenshot(session, out_dir / "grid_snapshot_small.png")
    report["screenshots"].append("grid_snapshot_small.png")

    # 4) 透明度控制 + 关闭图层
    set_control_value(pipe, session, "[data-testid=grid-opacity]", "30")
    time.sleep(0.2)
    mark("grid_opacity_ok", abs(float(grid_debug(pipe, session).get("opacity", 1.0)) - 0.3) < 1e-6)
    pipe.evaluate(
        "(() => { const box = document.querySelector('[data-testid=grid-enabled]');"
        " if (box && box.checked) box.click(); return true; })()",
        session,
    )
    time.sleep(0.2)
    mark("grid_disable_ok", grid_debug(pipe, session).get("enabled") is False)

    # 5) 空数据可见状态（清空栅格）
    click_grid_button(pipe, session, "清空栅格")
    time.sleep(0.2)
    empty_visible = pipe.evaluate(
        "document.querySelector('[data-testid=grid-empty-state]') !== null", session
    )
    mark("grid_empty_state_ok", bool(empty_visible))

    # 6) 失败可见状态（加载不存在的夹具）
    pipe.call("Page.navigate", {"url": f"{base_url}?grid=1&gridFixture=missing_fixture.json"}, session)
    error_visible = wait_for_condition(
        pipe,
        session,
        "document.querySelector('[data-testid=grid-error-state]') !== null",
        timeout=15.0,
    )
    error_text = pipe.evaluate(
        "(() => { const node = document.querySelector('[data-testid=grid-error-state]');"
        " return node ? node.innerText : null; })()",
        session,
    )
    checks["grid_error_text"] = error_text
    mark(
        "grid_error_state_ok",
        bool(error_visible) and isinstance(error_text, str) and "missing_fixture.json" in error_text,
    )
    pipe.screenshot(session, out_dir / "grid_error_state.png")
    report["screenshots"].append("grid_error_state.png")

    return all_ok


# ---------------------------------------------------------------------------
# X-W2 在线栅格与规划实际场景检查
# ---------------------------------------------------------------------------


def grid_online_debug(pipe: ChromePipe, session: str) -> dict:
    return grid_debug(pipe, session).get("online", {}) or {}


def plan_scene_debug(pipe: ChromePipe, session: str) -> dict:
    raw = pipe.evaluate(
        "JSON.stringify(window.__plannerDebug ? window.__plannerDebug.getState().plan_scene : null)",
        session,
    )
    return json.loads(raw) if raw else {}


def count_online_grid_pixels(pipe: ChromePipe, session: str) -> dict:
    """在线栅格三态纹理像素计数（通道关系判定，容忍矢量覆盖与 alpha 合成）。"""
    expression = (
        "(() => { const c = document.querySelector('canvas.map-canvas'); if (!c) return null;"
        " const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;"
        " let free = 0, occupied = 0, unknown = 0;"
        " for (let i = 0; i < d.length; i += 4) {"
        "  const r = d[i], g = d[i + 1], b = d[i + 2];"
        "  if (g > r + 25 && g > b + 15 && g > 80) free += 1;"
        "  else if (r > g + 25 && r > b + 15 && r > 90) occupied += 1;"
        "  else if (b > r + 20 && b > g + 15 && b > 120) unknown += 1;"
        " } return { free, occupied, unknown }; })()"
    )
    return pipe.evaluate(expression, session) or {"free": 0, "occupied": 0, "unknown": 0}


def count_plan_scene_pixels(pipe: ChromePipe, session: str) -> int:
    """规划实际场景叠加层（占据=品红 rgba(217,70,239,0.80)）合成像素计数。"""
    expression = (
        "(() => { const c = document.querySelector('canvas.map-canvas'); if (!c) return -1;"
        " const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data; let n = 0;"
        " for (let i = 0; i < d.length; i += 4) {"
        "  const r = d[i], g = d[i + 1], b = d[i + 2];"
        "  if (r > 130 && b > 150 && g < 120 && r - g > 50 && b - g > 60) n += 1;"
        " } return n; })()"
    )
    return int(pipe.evaluate(expression, session) or 0)


def navigate_app(pipe: ChromePipe, session: str, url: str, timeout: float = 20.0) -> bool:
    pipe.call("Page.navigate", {"url": url}, session)
    return bool(
        wait_for_condition(
            pipe,
            session,
            "document.querySelector('canvas.map-canvas') !== null",
            timeout=timeout,
        )
    )


def run_xw2_checks(pipe: ChromePipe, session: str, out_dir: Path, report: dict, base_url: str) -> bool:
    """X-W2 全量检查（真实 map+plan 服务）：在线图层、乱序防护、规划快照展示与区分。"""
    checks: dict = {}
    report["xw2_checks"] = checks
    all_ok = True

    def mark(key: str, condition: bool) -> bool:
        nonlocal all_ok
        checks[key] = bool(condition)
        all_ok = bool(condition) and all_ok
        return bool(condition)

    # 1) 可行驶域在线图层（自动请求携带 request_seq）
    navigate_app(pipe, session, f"{base_url}?grid=1&gridSource=driveable")
    ready = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        " const g = s && s.grid; return g && g.source_kind === 'driveable' && g.status === 'ready' ? g : null; })()",
        timeout=25.0,
    )
    mark("xw2_driveable_ready", bool(ready))
    driveable = grid_online_debug(pipe, session)
    checks["xw2_driveable"] = driveable
    health = driveable.get("health") or {}
    mark("xw2_health_protocol_ok", bool(health.get("protocol_ok")))
    mark("xw2_health_scene_rules_ok", bool(health.get("scene_rules_ok")))
    mark("xw2_driveable_seq", int(driveable.get("request_seq") or 0) >= 1)
    mark("xw2_driveable_source", driveable.get("source") == "driveable")
    pixels = count_online_grid_pixels(pipe, session)
    checks["xw2_driveable_pixels"] = pixels
    mark("xw2_driveable_pixels_ok", (pixels.get("free", 0) + pixels.get("occupied", 0)) > 500)
    banner = pipe.evaluate(
        "(() => { const n = document.querySelector('[data-testid=grid-online-banner]');"
        " return n ? n.innerText : null; })()",
        session,
    )
    checks["xw2_driveable_banner"] = banner
    mark("xw2_driveable_service_badge", isinstance(banner, str) and "来自服务" in banner)
    degraded = bool(driveable.get("degraded"))
    degraded_text = pipe.evaluate(
        "(() => { const n = document.querySelector('[data-testid=grid-online-degraded]');"
        " return n ? n.innerText : null; })()",
        session,
    )
    checks["xw2_driveable_degraded"] = {"degraded": degraded, "text": degraded_text}
    mark(
        "xw2_degraded_display_ok",
        (not degraded and not degraded_text)
        or (degraded and isinstance(degraded_text, str) and "服务降级为实际" in degraded_text),
    )
    pipe.screenshot(session, out_dir / "xw2_driveable_online.png")
    report["screenshots"].append("xw2_driveable_online.png")

    # 2) 乱序注入：旧 request_seq 载荷不得覆盖新请求（协议 §6）
    payload_raw = pipe.evaluate("window.__plannerDebug.gridPayload('driveable')", session)
    if isinstance(payload_raw, str):
        checks["xw2_driveable_payload_sample"] = payload_raw[:400]
        try:
            parsed_payload = json.loads(payload_raw)
            checks["xw2_driveable_response_meta"] = {
                key: parsed_payload.get(key)
                for key in (
                    "schema_version", "type", "source", "map_id", "map_version", "window",
                    "requested_resolution_m", "effective_resolution_m", "resolution_m",
                    "degraded", "width", "height", "origin", "encoding", "request_seq",
                    "budget_cells",
                )
            }
            checks["xw2_driveable_response_meta"]["cells_base64_len"] = len(parsed_payload.get("cells") or "")
        except Exception as error:  # noqa: BLE001
            checks["xw2_driveable_response_meta"] = {"parse_error": str(error)}
    seq_before = int(driveable.get("request_seq") or 0)
    sample_before = driveable.get("sample_cell_state")
    checks["xw2_stale_payload_seq"] = seq_before
    click_button(pipe, session, "请求在线栅格", ".grid-actions button")
    refreshed = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        f" const g = s && s.grid; return g && g.status === 'ready' && g.online && g.online.request_seq > {seq_before} ? g : null; }})()",
        timeout=20.0,
    )
    mark("xw2_refresh_ready", bool(refreshed))
    after_refresh = grid_online_debug(pipe, session)
    new_seq = int(after_refresh.get("request_seq") or 0)
    new_sample = after_refresh.get("sample_cell_state")
    checks["xw2_refresh_seq"] = new_seq
    if isinstance(payload_raw, str) and payload_raw:
        try:
            stale_payload = json.loads(payload_raw)
            stale_payload["request_seq"] = seq_before
            injected = pipe.evaluate(
                "window.__plannerDebug.injectGridResponse('driveable', "
                + json.dumps(stale_payload)
                + ")",
                session,
            )
        except Exception as error:  # noqa: BLE001
            injected = {"decision": "error", "reason": str(error)}
    else:
        injected = {"decision": "missing_payload", "reason": "gridPayload 为空"}
    checks["xw2_stale_injection"] = injected
    mark(
        "xw2_stale_rejected",
        isinstance(injected, dict) and injected.get("decision") == "stale" and injected.get("apply") is False,
    )
    after_inject = grid_online_debug(pipe, session)
    checks["xw2_stale_after"] = {
        "request_seq": after_inject.get("request_seq"),
        "sample_cell_state": after_inject.get("sample_cell_state"),
        "stale_ignored": after_inject.get("stale_ignored"),
        "expected_seq": new_seq,
        "expected_sample": new_sample,
        "before_sample": sample_before,
    }
    mark(
        "xw2_stale_not_overwrite",
        int(after_inject.get("request_seq") or 0) == new_seq
        and after_inject.get("sample_cell_state") == new_sample
        and int(after_inject.get("stale_ignored") or 0) >= 1,
    )

    # 3) 规划场景预览（当前预览，独立 scene_id）
    set_control_value(pipe, session, "[data-testid=grid-source-select]", "planning_scene")
    preview_ready = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        " const g = s && s.grid; return g && g.source_kind === 'planning_scene' && g.status === 'ready' ? g : null; })()",
        timeout=25.0,
    )
    mark("xw2_preview_ready", bool(preview_ready))
    preview_state = grid_debug(pipe, session)
    preview_scene_id = preview_state.get("scene_id")
    checks["xw2_preview_scene_id"] = preview_scene_id
    mark(
        "xw2_preview_scene_id_ok",
        isinstance(preview_scene_id, str) and len(preview_scene_id) == 64 and all(c in "0123456789abcdef" for c in preview_scene_id),
    )
    mark("xw2_preview_scene_version", preview_state.get("source") == "planning_scene")
    scene_id_badge = pipe.evaluate(
        "(() => { const n = document.querySelector('[data-testid=grid-online-scene-id]');"
        " const panel = document.querySelector('[data-testid=grid-panel]');"
        " return { scene_id: n ? n.innerText : null, panel_has_preview: panel ? panel.innerText.includes('当前预览') : false }; })()",
        session,
    )
    checks["xw2_preview_panel"] = scene_id_badge
    mark("xw2_preview_panel_ok", bool(scene_id_badge.get("panel_has_preview")))
    preview_pixels = count_online_grid_pixels(pipe, session)
    checks["xw2_preview_pixels"] = preview_pixels
    mark("xw2_preview_pixels_ok", (preview_pixels.get("free", 0) + preview_pixels.get("occupied", 0)) > 500)
    preview_payload = pipe.evaluate("window.__plannerDebug.gridPayload('planning_scene')", session)
    if isinstance(preview_payload, str):
        checks["xw2_preview_payload_sample"] = preview_payload[:400]
        try:
            parsed_preview = json.loads(preview_payload)
            checks["xw2_preview_response_meta"] = {
                key: parsed_preview.get(key)
                for key in (
                    "schema_version", "type", "source", "scene_id", "scene_version", "map_id",
                    "map_version", "window", "requested_resolution_m", "effective_resolution_m",
                    "resolution_m", "degraded", "width", "height", "origin", "encoding",
                    "request_seq", "budget_cells",
                )
            }
        except Exception as error:  # noqa: BLE001
            checks["xw2_preview_response_meta"] = {"parse_error": str(error)}
    pipe.screenshot(session, out_dir / "xw2_preview_online.png")
    report["screenshots"].append("xw2_preview_online.png")

    # 4) 提交规划 → 结果 scene_id → GET /plan/scene/{scene_id} → 规划实际场景展示
    scene = scene_state(pipe, session)
    has_poses = scene.get("start") is not None and scene.get("goal") is not None
    mark("xw2_plan_snapshot_poses", has_poses)
    if has_poses:
        click_button(pipe, session, "提交规划", ".inspector button")
        snapshot_ready = wait_for_condition(
            pipe,
            session,
            "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
            " return s && s.plan_scene && s.plan_scene.status === 'ready' ? s.plan_scene : null; })()",
            timeout=30.0,
        )
        mark("xw2_plan_snapshot_ready", bool(snapshot_ready))
        plan_scene = plan_scene_debug(pipe, session)
        plan = plan_state(pipe, session)
        runs = plan.get("runs") or []
        run_scene_id = runs[0].get("scene_id") if runs else None
        checks["xw2_plan_scene"] = plan_scene
        checks["xw2_run_scene_id"] = run_scene_id
        mark(
            "xw2_plan_snapshot_scene_match",
            isinstance(run_scene_id, str) and plan_scene.get("scene_id") == run_scene_id,
        )
        panel_text = pipe.evaluate("(() => document.querySelector('.inspector').textContent)()", session) or ""
        mark(
            "xw2_plan_snapshot_panel",
            "规划实际场景" in panel_text and "历史" in panel_text and "scene_version" in panel_text,
        )
        # 定位到规划场景（快照窗口可能远小于当前视口），再统计叠加像素。
        click_button(pipe, session, "定位到场景", ".inspector button")
        time.sleep(0.6)
        overlay_pixels = count_plan_scene_pixels(pipe, session)
        checks["xw2_plan_snapshot_pixels"] = overlay_pixels
        mark("xw2_plan_snapshot_overlay_ok", overlay_pixels > 50)
        vs_preview = pipe.evaluate(
            "(() => { const n = document.querySelector('[data-testid=plan-scene-vs-preview]');"
            " return n ? n.innerText : null; })()",
            session,
        )
        checks["xw2_plan_scene_vs_preview"] = vs_preview
        mark(
            "xw2_plan_scene_vs_preview_ok",
            isinstance(vs_preview, str)
            and ("不同" in vs_preview or "一致" in vs_preview),
        )
        mark("xw2_plan_scene_history_badge", "历史" in (pipe.evaluate(
            "(() => { const n = document.querySelector('[data-testid=plan-scene-history-badge]');"
            " return n ? n.innerText : ''; })()",
            session,
        ) or ""))
        # 历史场景不覆盖当前预览：规划快照展示后，预览层 scene_id 必须保持不变。
        preview_after = grid_debug(pipe, session).get("online", {}).get("states", {}).get("planning_scene", {})
        checks["xw2_preview_after_plan_snapshot"] = preview_after.get("scene_id")
        mark(
            "xw2_plan_snapshot_not_overwrite_preview",
            preview_after.get("scene_id") == preview_scene_id
            and preview_after.get("status") == "ready",
        )
        pipe.screenshot(session, out_dir / "xw2_plan_scene_snapshot.png")
        report["screenshots"].append("xw2_plan_scene_snapshot.png")
    else:
        mark("xw2_plan_snapshot_ready", False)
        mark("xw2_plan_snapshot_scene_match", False)
        mark("xw2_plan_snapshot_panel", False)
        mark("xw2_plan_snapshot_overlay_ok", False)
        mark("xw2_plan_scene_vs_preview_ok", False)
        mark("xw2_plan_scene_history_badge", False)

    return all_ok


def run_xw2_mock_checks(
    pipe: ChromePipe,
    session: str,
    out_dir: Path,
    report: dict,
    base_url: str,
    mode: str,
) -> bool:
    """X-W2 mock 阶段：reorder（乱序响应）/ protocol（主版本不匹配）/ scene_rules（规则版本不匹配）。"""
    checks: dict = {}
    report["xw2_mock_checks"] = checks
    all_ok = True

    def mark(key: str, condition: bool) -> bool:
        nonlocal all_ok
        checks[key] = bool(condition)
        all_ok = bool(condition) and all_ok
        return bool(condition)

    checks["xw2_mock_mode"] = mode
    navigate_app(pipe, session, f"{base_url}?grid=1&gridSource=driveable")

    if mode == "reorder":
        # 首个请求（自动，seq=1）被 mock 延迟；点击后 seq=2 先返回。
        started = wait_for_condition(
            pipe,
            session,
            "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
            " const g = s && s.grid; return g && g.online && g.online.latest_seq >= 1 ? g : null; })()",
            timeout=20.0,
        )
        mark("xw2_reorder_started", bool(started))
        click_button(pipe, session, "请求在线栅格", ".grid-actions button")
        ready = wait_for_condition(
            pipe,
            session,
            "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
            " const g = s && s.grid;"
            " return g && g.status === 'ready' && g.online && g.online.request_seq >= 2 ? g : null; })()",
            timeout=20.0,
        )
        mark("xw2_reorder_second_ready", bool(ready))
        time.sleep(1.6)  # 等待延迟的首个响应到达
        final = grid_online_debug(pipe, session)
        checks["xw2_reorder_final"] = final
        mark(
            "xw2_reorder_final_seq",
            int(final.get("request_seq") or 0) == 2 and int(final.get("latest_seq") or 0) == 2,
        )
        mark(
            "xw2_reorder_stale_ignored",
            int((final.get("states") or {}).get("driveable", {}).get("stale_ignored") or 0) >= 1,
        )
        # 单元 (0,0) 编码 request_seq % 3：seq=2 → 2（unknown），证明展示的是第二个响应。
        mark("xw2_reorder_sample_cell", final.get("sample_cell_state") == 2)
        degraded_text = pipe.evaluate(
            "(() => { const n = document.querySelector('[data-testid=grid-online-degraded]');"
            " return n ? n.innerText : null; })()",
            session,
        )
        checks["xw2_reorder_degraded_text"] = degraded_text
        mark(
            "xw2_reorder_degraded_ok",
            isinstance(degraded_text, str) and "服务降级为实际" in degraded_text,
        )
        pipe.screenshot(session, out_dir / "xw2_reorder.png")
        report["screenshots"].append("xw2_reorder.png")
        return all_ok

    # 版本不匹配模式：等待健康字段到位后判定禁用/允许。
    health_ready = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        " const h = s && s.grid && s.grid.online && s.grid.online.health;"
        " return h && h.status === 'online' ? h : null; })()",
        timeout=20.0,
    )
    mark("xw2_mismatch_health_ready", bool(health_ready))
    health = grid_online_debug(pipe, session).get("health") or {}
    checks["xw2_mismatch_health"] = health

    if mode == "protocol":
        mark("xw2_mismatch_protocol_not_ok", health.get("protocol_ok") is False)
        disabled = wait_for_condition(
            pipe,
            session,
            "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
            " const g = s && s.grid; return g && g.status === 'disabled' ? g : null; })()",
            timeout=15.0,
        )
        mark("xw2_mismatch_layer_disabled", bool(disabled))
        state = grid_debug(pipe, session)
        mark("xw2_mismatch_no_request", (state.get("online") or {}).get("request_seq") in (None, 0))
        disabled_text = pipe.evaluate(
            "(() => { const n = document.querySelector('[data-testid=grid-online-disabled]');"
            " return n ? n.innerText : null; })()",
            session,
        )
        checks["xw2_mismatch_disabled_text"] = disabled_text
        mark(
            "xw2_mismatch_visible_state",
            isinstance(disabled_text, str) and "在线栅格已禁用" in disabled_text and "2.0" in disabled_text,
        )
        pipe.screenshot(session, out_dir / "xw2_protocol_mismatch.png")
        report["screenshots"].append("xw2_protocol_mismatch.png")
        return all_ok

    # scene_rules：可行驶域仍允许；规划场景预览禁用（规则版本不一致）。
    mark("xw2_mismatch_scene_rules_not_ok", health.get("scene_rules_ok") is False)
    mark("xw2_mismatch_protocol_ok", bool(health.get("protocol_ok")))
    driveable_ready = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        " const g = s && s.grid; return g && g.source_kind === 'driveable' && g.status === 'ready' ? g : null; })()",
        timeout=20.0,
    )
    mark("xw2_mismatch_driveable_allowed", bool(driveable_ready))
    set_control_value(pipe, session, "[data-testid=grid-source-select]", "planning_scene")
    preview_disabled = wait_for_condition(
        pipe,
        session,
        "(() => { const s = window.__plannerDebug && window.__plannerDebug.getState();"
        " const g = s && s.grid;"
        " return g && g.source_kind === 'planning_scene' && g.status === 'disabled' ? g : null; })()",
        timeout=15.0,
    )
    mark("xw2_mismatch_preview_disabled", bool(preview_disabled))
    disabled_text = pipe.evaluate(
        "(() => { const n = document.querySelector('[data-testid=grid-online-disabled]');"
        " return n ? n.innerText : null; })()",
        session,
    )
    checks["xw2_mismatch_disabled_text"] = disabled_text
    mark(
        "xw2_mismatch_visible_state",
        isinstance(disabled_text, str) and "scene_v2" in disabled_text,
    )
    pipe.screenshot(session, out_dir / "xw2_scene_rules_mismatch.png")
    report["screenshots"].append("xw2_scene_rules_mismatch.png")
    return all_ok


def main() -> int:
    parser = argparse.ArgumentParser(description="W1/W2 浏览器交互检查（CDP over pipe）")
    parser.add_argument("--output-dir", default=None, help="产物目录（默认 $PLANNER_RUN_DIR/outputs/browser_cdp）")
    parser.add_argument("--url", default=None, help="页面 URL（默认由脚本自建静态服务）")
    parser.add_argument(
        "--api-target",
        default=None,
        help="启用 W2 检查：把 /api 反向代理到该地址（如 http://127.0.0.1:8765）",
    )
    parser.add_argument(
        "--plan-api-target",
        default=None,
        help="启用 W5 检查：把 /plan-api 反向代理到规划服务（如 http://127.0.0.1:8766）",
    )
    parser.add_argument(
        "--expect-stale-map",
        action="store_true",
        help="W5 模式：规划服务地图版本与请求不一致，期望提交被 409 stale_map 拒绝",
    )
    parser.add_argument(
        "--expect-ws",
        action="store_true",
        help="W5：要求 /plan/events WebSocket 推送已连接（真实服务支持）",
    )
    parser.add_argument(
        "--xw2",
        action="store_true",
        help="X-W2 全量检查：在线栅格（可行驶域/预览）+ 规划实际场景快照（需 --api-target 与 --plan-api-target）",
    )
    parser.add_argument(
        "--grid-api-target",
        default=None,
        help="X-W2 mock 阶段：把 /api 指向 mock 栅格服务（跳过 W2/W3 检查）",
    )
    parser.add_argument(
        "--xw2-mock-mode",
        choices=["reorder", "protocol", "scene_rules"],
        default=None,
        help="X-W2 mock 阶段检查：乱序响应 / 协议主版本不匹配 / 场景规则版本不匹配",
    )
    parser.add_argument(
        "--route-pair",
        default=None,
        help="W2 路由检查使用的起终 lanelet ID，形如 42440,45260（缺省自动查找连通对）",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[3]
    dist_dir = repo_root / "Src" / "web" / "dist"
    run_dir = os.environ.get("PLANNER_RUN_DIR")
    out_dir = Path(args.output_dir) if args.output_dir else (
        Path(run_dir) / "outputs" / "browser_cdp" if run_dir else Path(tempfile.mkdtemp(prefix="web-cdp-out-"))
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    if not (dist_dir / "index.html").is_file():
        print("[cdp] dist/index.html 缺失；先执行 build --component web", file=sys.stderr)
        return 66

    try:
        chrome_bin = find_chrome()
    except CdpError as error:
        print(f"[cdp] {error}", file=sys.stderr)
        return 69

    route_pair: tuple[str, str] | None = None
    if args.route_pair:
        parts = [part.strip() for part in args.route_pair.split(",")]
        if len(parts) == 2 and all(parts):
            route_pair = (parts[0], parts[1])

    port = 18766
    StaticAndProxyHandler.api_target = args.grid_api_target or args.api_target
    StaticAndProxyHandler.plan_api_target = args.plan_api_target
    handler = functools.partial(StaticAndProxyHandler, directory=str(dist_dir))
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", port), handler)
    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()
    url = args.url or f"http://127.0.0.1:{port}/"
    # W5：把 /plan/events WebSocket 直连到规划服务（Python 代理不做 WS 升级）。
    if args.plan_api_target and "planWs=" not in url:
        ws_base = args.plan_api_target.replace("https://", "wss://").replace("http://", "ws://").rstrip("/")
        separator = "&" if "?" in url else "?"
        url = f"{url}{separator}planWs={ws_base}"
    pipe = None
    try:
        time.sleep(0.3)
        pipe = ChromePipe(chrome_bin, out_dir / "cdp_chrome.log")
        target_id = pipe.call("Target.createTarget", {"url": "about:blank"})["targetId"]
        session = pipe.call("Target.attachToTarget", {"targetId": target_id, "flatten": True})["sessionId"]
        pipe.call("Page.enable", session=session)
        pipe.call("Runtime.enable", session=session)
        pipe.call(
            "Emulation.setDeviceMetricsOverride",
            {"width": VIEWPORT_WIDTH, "height": VIEWPORT_HEIGHT, "deviceScaleFactor": 1, "mobile": False},
            session,
        )
        pipe.call("Page.navigate", {"url": url}, session)

        report = run_checks(pipe, session, url, out_dir)
        checks = report["checks"]
        w2_ok = True
        if args.api_target:
            w2_ok = run_w2_checks(
                pipe,
                session,
                f"http://127.0.0.1:{port}/maps/mapping_example.snapshot.json",
                out_dir,
                report,
                route_pair=route_pair,
            )
        w3_ok = True
        if args.api_target:
            w3_ok = run_w3_checks(pipe, session, out_dir, report)
        w4_ok = run_w4_checks(pipe, session, out_dir, report)
        w5_ok = True
        if args.plan_api_target:
            w5_ok = run_w5_checks(
                pipe,
                session,
                out_dir,
                report,
                mode="stale_map" if args.expect_stale_map else "full",
                expect_ws=args.expect_ws,
                plan_api_target=args.plan_api_target,
            )
        grid_ok = run_grid_checks(pipe, session, out_dir, report, f"http://127.0.0.1:{port}/")
        xw2_ok = True
        if args.xw2:
            xw2_ok = run_xw2_checks(pipe, session, out_dir, report, f"http://127.0.0.1:{port}/")
        elif args.xw2_mock_mode:
            xw2_ok = run_xw2_mock_checks(
                pipe, session, out_dir, report, f"http://127.0.0.1:{port}/", args.xw2_mock_mode
            )
        (out_dir / "browser_cdp_report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )

        print(f"[cdp] url={url}")
        print(
            f"[cdp] viewport={checks['viewport_actual']} dpr={checks['dpr']} "
            f"canvas_matches_host={checks['canvas_size_matches_host']}"
        )
        print(
            f"[cdp] initial mpp={checks['initial_mpp']} map_pixels={checks['initial_map_pixels']} "
            f"centerline_pixels={checks['initial_centerline_pixels']}"
        )
        print(
            f"[cdp] zoom: mpp {checks['initial_mpp']} -> {checks['zoomed_mpp']} "
            f"(factor {checks.get('zoom_factor')}) anchor_drift={checks.get('zoom_anchor_drift_m')} m ok={checks.get('zoom_anchor_ok')}"
        )
        print(
            f"[cdp] pan: expected={checks.get('pan_expected_delta_m')} actual={checks.get('pan_actual_delta_m')} "
            f"error={checks.get('pan_error_m')} m ok={checks.get('pan_ok')}"
        )
        print(
            f"[cdp] centerline toggle: off_pixels={checks['centerline_off_pixels']} on_pixels={checks['centerline_on_pixels']} "
            f"ok={checks['centerline_toggle_off_ok'] and checks['centerline_toggle_on_ok']}"
        )
        print(f"[cdp] reset: mpp={checks['reset_mpp']} ok={checks['reset_ok']}")
        print(
            f"[cdp] dpr2: host={checks.get('dpr2_host')} canvas={checks.get('dpr2_canvas')} "
            f"mpp={checks.get('dpr2_mpp')} ok={checks.get('dpr2_ok')}"
        )
        if args.api_target:
            w2 = report.get("w2_checks", {})
            print(
                f"[cdp] w2 selection: id={w2.get('selection_id')} source={w2.get('selection_source')} "
                f"pixels {w2.get('selection_pixels_before')}->{w2.get('selection_pixels_after')} ok={w2.get('selection_ok')}"
            )
            print(f"[cdp] w2 topology: kind={w2.get('topology_kind')} ids={w2.get('topology_ids')} ok={w2.get('topology_ok')}")
            print(
                f"[cdp] w2 route: pair={w2.get('route_pair')} ids={w2.get('route_ids')} "
                f"length={w2.get('route_length_m')} ok={w2.get('route_ok')}"
            )
            print(f"[cdp] w2 window: {w2.get('window_result')} ok={w2.get('window_ok')}")
            w3 = report.get("w3_checks", {})
            print(
                f"[cdp] w3 obstacle: method={w3.get('w3_obstacle_add_method')} count={w3.get('w3_obstacle_count')} "
                f"added={w3.get('w3_obstacle_added')} moved={w3.get('w3_obstacle_moved')} deleted={w3.get('w3_obstacle_deleted')}"
            )
            print(
                f"[cdp] w3 footprints: kinds={w3.get('w3_footprint_kinds')} pixels={w3.get('w3_footprint_pixels')} "
                f"ok={w3.get('w3_footprints_rendered')}"
            )
            print(
                f"[cdp] w3 scene: save={w3.get('w3_save_ok')} clear={w3.get('w3_cleared')} "
                f"import={w3.get('w3_import_restored')} mismatch_refused={w3.get('w3_mismatch_refused')}"
            )
        w4 = report.get("w4_checks", {})
        print(
            f"[cdp] w4 load: samples={w4.get('w4_sample_count')} issues={w4.get('w4_issues')} "
            f"map={w4.get('w4_map_file')} ok={w4.get('w4_load_ok')}"
        )
        for sample_id, result in (w4.get("w4_per_sample") or {}).items():
            print(f"[cdp] w4 {sample_id}: {json.dumps(result, ensure_ascii=False)}")
        print(
            f"[cdp] w4 playback: time_after_play={w4.get('w4_time_after_play')} paused={w4.get('w4_paused')} "
            f"seek={json.dumps(w4.get('w4_seek_pose'), ensure_ascii=False)} layer_toggle={w4.get('w4_layer_toggle_ok')}"
        )
        if args.plan_api_target:
            w5 = report.get("w5_checks", {})
            print(
                f"[cdp] w5 health={w5.get('w5_health_online')} submit={w5.get('w5_submit_ok')} "
                f"statuses={w5.get('w5_observed_statuses')} stages={w5.get('w5_observed_stages')} "
                f"push={w5.get('w5_push_transport')} reason={w5.get('w5_run_reason')}"
            )
            print(
                f"[cdp] w5 result: {json.dumps(w5.get('w5_result_summary'), ensure_ascii=False)} "
                f"panel_labels={w5.get('w5_panel_labels')}"
            )
            print(
                f"[cdp] w5 playback: play={w5.get('w5_play_ok')} seek={w5.get('w5_seek_time')} charts={w5.get('w5_charts')}"
            )
            print(
                f"[cdp] w5 stale: ignored={w5.get('w5_stale_event_ignored')} fresh_accepted={w5.get('w5_fresh_event_accepted')} "
                f"cancel={w5.get('w5_cancelled')}"
            )
            print(
                f"[cdp] w5 compare: runs={json.dumps(w5.get('w5_compare_runs'), ensure_ascii=False)} "
                f"table_ok={w5.get('w5_compare_table_ok')} client_block={w5.get('w5_client_block_ok')}"
            )
            if args.expect_stale_map:
                print(f"[cdp] w5 stale_map: {json.dumps(w5.get('w5_stale_map_error'), ensure_ascii=False)} blocked={w5.get('w5_stale_map_blocked')}")
        grid = report.get("grid_checks", {})
        print(
            f"[cdp] grid: catalog={len(grid.get('grid_catalog_options') or [])} legend={grid.get('grid_legend_ok')} "
            f"g1_pick={grid.get('grid_g1_pick_ok')} cache={grid.get('grid_cache_ok')} "
            f"snapshot={grid.get('grid_snapshot_pick_ok')} error_state={grid.get('grid_error_state_ok')}"
        )
        if args.xw2:
            xw2 = report.get("xw2_checks", {})
            print(
                f"[cdp] xw2 driveable: ready={xw2.get('xw2_driveable_ready')} seq={xw2.get('xw2_driveable_seq')} "
                f"degraded={json.dumps(xw2.get('xw2_driveable_degraded'), ensure_ascii=False)} "
                f"pixels={json.dumps(xw2.get('xw2_driveable_pixels'), ensure_ascii=False)}"
            )
            print(
                f"[cdp] xw2 stale: rejected={xw2.get('xw2_stale_rejected')} after={json.dumps(xw2.get('xw2_stale_after'), ensure_ascii=False)}"
            )
            print(
                f"[cdp] xw2 preview: ready={xw2.get('xw2_preview_ready')} scene_id={xw2.get('xw2_preview_scene_id')}"
            )
            print(
                f"[cdp] xw2 plan snapshot: ready={xw2.get('xw2_plan_snapshot_ready')} "
                f"match={xw2.get('xw2_plan_snapshot_scene_match')} overlay_pixels={xw2.get('xw2_plan_snapshot_pixels')} "
                f"vs_preview={xw2.get('xw2_plan_scene_vs_preview')}"
            )
        if args.xw2_mock_mode:
            mock = report.get("xw2_mock_checks", {})
            print(f"[cdp] xw2 mock mode={mock.get('xw2_mock_mode')} checks={json.dumps({k: v for k, v in mock.items() if isinstance(v, (bool, str, int, float))}, ensure_ascii=False)}")
        print(f"[cdp] report: {out_dir / 'browser_cdp_report.json'}")

        required_ok = [
            checks["canvas_size_matches_host"],
            checks.get("zoom_in_reduces_mpp", False),
            checks.get("zoom_anchor_ok", False),
            checks.get("pan_ok", False),
            checks.get("centerline_toggle_off_ok", False),
            checks.get("centerline_toggle_on_ok", False),
            checks.get("reset_ok", False),
            checks.get("dpr2_ok", False),
            checks["initial_map_pixels"] > 1000,
        ]
        if args.api_target:
            required_ok.extend(
                [
                    w2_ok,
                    bool(report.get("w2_checks", {}).get("selection_ok")),
                    bool(report.get("w2_checks", {}).get("topology_ok")),
                    bool(report.get("w2_checks", {}).get("route_ok")),
                    bool(report.get("w2_checks", {}).get("window_ok")),
                ]
            )
            required_ok.extend(
                [
                    w3_ok,
                    bool(report.get("w3_checks", {}).get("w3_obstacle_added")),
                    bool(report.get("w3_checks", {}).get("w3_obstacle_moved")),
                    bool(report.get("w3_checks", {}).get("w3_footprints_rendered")),
                    bool(report.get("w3_checks", {}).get("w3_footprint_cleared")),
                    bool(report.get("w3_checks", {}).get("w3_save_ok")),
                    bool(report.get("w3_checks", {}).get("w3_cleared")),
                    bool(report.get("w3_checks", {}).get("w3_import_restored")),
                    bool(report.get("w3_checks", {}).get("w3_mismatch_refused")),
                    bool(report.get("w3_checks", {}).get("w3_obstacle_deleted")),
                ]
            )
        required_ok.extend(
            [
                w4_ok,
                bool(report.get("w4_checks", {}).get("w4_load_ok")),
                bool(report.get("w4_checks", {}).get("w4_play_advance_ok")),
                bool(report.get("w4_checks", {}).get("w4_seek_ok")),
                bool(report.get("w4_checks", {}).get("w4_layer_toggle_ok")),
            ]
        )
        if args.plan_api_target:
            required_ok.append(w5_ok)
            required_ok.append(bool(report.get("w5_checks", {}).get("w5_health_online")))
            if args.expect_stale_map:
                required_ok.append(bool(report.get("w5_checks", {}).get("w5_stale_map_blocked")))
            else:
                required_ok.extend(
                    [
                        bool(report.get("w5_checks", {}).get("w5_submit_ok")),
                        bool(report.get("w5_checks", {}).get("w5_progress_observed")),
                        bool(report.get("w5_checks", {}).get("w5_result_rendered")),
                        bool(report.get("w5_checks", {}).get("w5_stale_event_ignored")),
                        bool(report.get("w5_checks", {}).get("w5_fresh_event_accepted")),
                        bool(report.get("w5_checks", {}).get("w5_cancelled")),
                        bool(report.get("w5_checks", {}).get("w5_compare_ok")),
                        bool(report.get("w5_checks", {}).get("w5_compare_table_ok")),
                        bool(report.get("w5_checks", {}).get("w5_client_block_ok")),
                    ]
                )
                if args.expect_ws:
                    required_ok.append(bool(report.get("w5_checks", {}).get("w5_ws_ok")))
        required_ok.extend(
            [
                grid_ok,
                bool(report.get("grid_checks", {}).get("grid_catalog_ok")),
                bool(report.get("grid_checks", {}).get("grid_legend_ok")),
                bool(report.get("grid_checks", {}).get("grid_synthetic_banner_ok")),
                bool(report.get("grid_checks", {}).get("grid_g1_pick_ok")),
                bool(report.get("grid_checks", {}).get("grid_g1_center_ok")),
                bool(report.get("grid_checks", {}).get("grid_g1_panel_ok")),
                bool(report.get("grid_checks", {}).get("grid_pixels_ok")),
                bool(report.get("grid_checks", {}).get("grid_cache_ok")),
                bool(report.get("grid_checks", {}).get("grid_snapshot_pick_ok")),
                bool(report.get("grid_checks", {}).get("grid_opacity_ok")),
                bool(report.get("grid_checks", {}).get("grid_disable_ok")),
                bool(report.get("grid_checks", {}).get("grid_empty_state_ok")),
                bool(report.get("grid_checks", {}).get("grid_error_state_ok")),
            ]
        )
        if args.xw2:
            xw2 = report.get("xw2_checks", {})
            required_ok.extend(
                [
                    xw2_ok,
                    bool(xw2.get("xw2_driveable_ready")),
                    bool(xw2.get("xw2_health_protocol_ok")),
                    bool(xw2.get("xw2_health_scene_rules_ok")),
                    bool(xw2.get("xw2_driveable_service_badge")),
                    bool(xw2.get("xw2_degraded_display_ok")),
                    bool(xw2.get("xw2_driveable_pixels_ok")),
                    bool(xw2.get("xw2_stale_rejected")),
                    bool(xw2.get("xw2_stale_not_overwrite")),
                    bool(xw2.get("xw2_preview_ready")),
                    bool(xw2.get("xw2_preview_scene_id_ok")),
                    bool(xw2.get("xw2_preview_panel_ok")),
                    bool(xw2.get("xw2_preview_pixels_ok")),
                    bool(xw2.get("xw2_plan_snapshot_ready")),
                    bool(xw2.get("xw2_plan_snapshot_scene_match")),
                    bool(xw2.get("xw2_plan_snapshot_panel")),
                    bool(xw2.get("xw2_plan_snapshot_overlay_ok")),
                    bool(xw2.get("xw2_plan_snapshot_not_overwrite_preview")),
                    bool(xw2.get("xw2_plan_scene_vs_preview_ok")),
                ]
            )
        if args.xw2_mock_mode:
            mock = report.get("xw2_mock_checks", {})
            required_ok.append(xw2_ok)
            if args.xw2_mock_mode == "reorder":
                required_ok.extend(
                    [
                        bool(mock.get("xw2_reorder_started")),
                        bool(mock.get("xw2_reorder_second_ready")),
                        bool(mock.get("xw2_reorder_final_seq")),
                        bool(mock.get("xw2_reorder_stale_ignored")),
                        bool(mock.get("xw2_reorder_sample_cell")),
                        bool(mock.get("xw2_reorder_degraded_ok")),
                    ]
                )
            elif args.xw2_mock_mode == "protocol":
                required_ok.extend(
                    [
                        bool(mock.get("xw2_mismatch_health_ready")),
                        bool(mock.get("xw2_mismatch_protocol_not_ok")),
                        bool(mock.get("xw2_mismatch_layer_disabled")),
                        bool(mock.get("xw2_mismatch_no_request")),
                        bool(mock.get("xw2_mismatch_visible_state")),
                    ]
                )
            else:
                required_ok.extend(
                    [
                        bool(mock.get("xw2_mismatch_health_ready")),
                        bool(mock.get("xw2_mismatch_scene_rules_not_ok")),
                        bool(mock.get("xw2_mismatch_protocol_ok")),
                        bool(mock.get("xw2_mismatch_driveable_allowed")),
                        bool(mock.get("xw2_mismatch_preview_disabled")),
                        bool(mock.get("xw2_mismatch_visible_state")),
                    ]
                )
        if not all(required_ok):
            print("[cdp] 检查未全部通过", file=sys.stderr)
            return 1
        print("[cdp] OK")
        return 0
    except CdpError as error:
        print(f"[cdp] 失败: {error}", file=sys.stderr)
        if pipe is not None:
            for line in pipe.recent_exceptions():
                print(f"[cdp] 页面异常: {line}", file=sys.stderr)
        return 1
    finally:
        if pipe is not None:
            pipe.close()
        httpd.shutdown()
        httpd.server_close()


if __name__ == "__main__":
    sys.exit(main())
