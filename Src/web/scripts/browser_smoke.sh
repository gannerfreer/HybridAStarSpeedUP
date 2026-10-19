#!/usr/bin/env bash
# W1 浏览器冒烟检查（回退方案）：把构建产物 Src/web/dist 通过本地 HTTP 提供，
# 用无头 Chrome 截图并导出 DOM，校验 React 实际渲染了地图信息。
#
# 注意：无头 `--screenshot` 捕获时会把视口重设为窗口大小，可能截到位图尺寸更新
# 前一帧（纵向拉伸）；权威浏览器证据请用 scripts/browser_cdp_check.py（CDP 固定视口）。
#
# 经统一入口运行（产物写入 $PLANNER_RUN_DIR/outputs/）：
#   python3 tools/task_runner.py exec --component web --workers 2 -- \
#     bash Src/web/scripts/browser_smoke.sh
#
# 先决条件：`python3 tools/task_runner.py build --component web` 已生成 dist/。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="$(dirname "$SCRIPT_DIR")"
DIST_DIR="$WEB_DIR/dist"
RUN_DIR="${PLANNER_RUN_DIR:?PLANNER_RUN_DIR not set（请经 tools/task_runner.py exec 运行）}"
OUT_DIR="$RUN_DIR/outputs"
PORT="${WEB_SMOKE_PORT:-18765}"
URL="http://127.0.0.1:$PORT/"

CHROME_BIN="${CHROME_BIN:-}"
if [[ -z "$CHROME_BIN" ]]; then
  for candidate in google-chrome google-chrome-stable chromium chromium-browser; do
    if command -v "$candidate" >/dev/null 2>&1; then
      CHROME_BIN="$candidate"
      break
    fi
  done
fi
if [[ -z "$CHROME_BIN" ]]; then
  echo "[browser-smoke] 未找到 chrome/chromium 可执行文件" >&2
  exit 69
fi

if [[ ! -f "$DIST_DIR/index.html" ]]; then
  echo "[browser-smoke] dist/index.html 缺失；先执行 build --component web" >&2
  exit 66
fi

mkdir -p "$OUT_DIR"
PROFILE_DIR="$(mktemp -d)"
SERVER_LOG="$OUT_DIR/browser_server.log"
CHROME_LOG="$OUT_DIR/browser_chrome.log"

cleanup() {
  if [[ -n "${SERVER_PID:-}" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  rm -rf "$PROFILE_DIR"
}
trap cleanup EXIT

python3 -m http.server "$PORT" --bind 127.0.0.1 --directory "$DIST_DIR" >"$SERVER_LOG" 2>&1 &
SERVER_PID=$!
sleep 1
if ! kill -0 "$SERVER_PID" 2>/dev/null; then
  echo "[browser-smoke] 静态服务器启动失败；见 $SERVER_LOG" >&2
  exit 70
fi

COMMON_ARGS=(
  --headless=new
  --no-sandbox
  --disable-gpu
  --disable-extensions
  --disable-background-networking
  --no-first-run
  --no-default-browser-check
  --no-proxy-server
  --user-data-dir="$PROFILE_DIR"
  --window-size=1440,900
  --virtual-time-budget=8000
  --hide-scrollbars
)

echo "[browser-smoke] chrome=$CHROME_BIN port=$PORT url=$URL"
# 单次调用同时产出截图与 DOM，保证两份证据来自同一页面状态。
"$CHROME_BIN" "${COMMON_ARGS[@]}" \
  --screenshot="$OUT_DIR/browser_smoke.png" \
  --dump-dom \
  "$URL" >"$OUT_DIR/browser_dom.html" 2>"$CHROME_LOG"

# DOM 校验：React 实际渲染 + 消费 v3 夹具来源信息。
if ! grep -q "mapping_example" "$OUT_DIR/browser_dom.html"; then
  echo "[browser-smoke] DOM 校验失败：未找到 mapping_example" >&2
  exit 1
fi
if ! grep -q "lanelets" "$OUT_DIR/browser_dom.html"; then
  echo "[browser-smoke] DOM 校验失败：状态栏未显示 lanelets 计数" >&2
  exit 1
fi
if ! grep -q "fixture_v3" "$OUT_DIR/browser_dom.html"; then
  echo "[browser-smoke] DOM 校验失败：状态栏未显示 fixture_v3 数据源" >&2
  exit 1
fi

echo "[browser-smoke] screenshot: $OUT_DIR/browser_smoke.png"
echo "[browser-smoke] dom: $OUT_DIR/browser_dom.html"
echo "[browser-smoke] chrome log: $CHROME_LOG"
echo "[browser-smoke] OK"
