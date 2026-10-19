#!/usr/bin/env bash
# W2–W5 + X-W2 集成冒烟：启动地图服务（8765）与规划服务（8766；真实二进制未就绪时用临时 mock，
# 日志中明确标注），依次运行
#   1. service_compare.py            —— 地图服务响应 vs v3 快照一致性对比
#   2. browser_cdp_check.py          —— 真浏览器 W1–W5 + X-W2 检查（/api 与 /plan-api 代理）
#   3. browser_cdp_check.py --expect-stale-map —— 规划服务地图版本不匹配 → 409 stale_map
#   4. browser_cdp_check.py --grid-api-target + --xw2-mock-mode —— X-W2 mock 阶段：
#        reorder（乱序响应不覆盖）/ protocol（协议主版本不匹配禁用）/ scene_rules（规则版本不匹配）
# 然后停止全部服务。产物全部写入 $PLANNER_RUN_DIR/outputs/。
#
# 经统一入口运行：
#   python3 tools/task_runner.py exec --component map --workers 4 -- \
#     bash Src/web/scripts/integration_smoke.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
RUN_DIR="${PLANNER_RUN_DIR:?PLANNER_RUN_DIR not set（请经 tools/task_runner.py exec 运行）}"
OUT_DIR="$RUN_DIR/outputs"
PORT="${WEB_SMOKE_SERVICE_PORT:-8765}"
PLAN_PORT="${WEB_SMOKE_PLAN_PORT:-8766}"
PLAN_STALE_PORT="${WEB_SMOKE_PLAN_STALE_PORT:-8767}"
GRID_MOCK_PORT="${WEB_SMOKE_GRID_MOCK_PORT:-8865}"
MAP_OSM="${REPO_ROOT}/tests/fixtures/v3/map/mapping_example.osm"
SNAPSHOT="${REPO_ROOT}/tests/fixtures/v3/map/mapping_example.snapshot.json"
ORIGIN_LAT=49.00345654351
ORIGIN_LON=8.42427590707

mkdir -p "$OUT_DIR"

# 定位地图服务二进制：PLANNER_BUILD_DIR → build/map → build/release
SERVICE_BIN=""
for candidate in "${PLANNER_BUILD_DIR:-}/Src/map/planner_map_service" \
                 "$REPO_ROOT/build/map/Src/map/planner_map_service" \
                 "$REPO_ROOT/build/release/Src/map/planner_map_service"; do
  if [[ -n "$candidate" && -x "$candidate" ]]; then
    SERVICE_BIN="$candidate"
    break
  fi
done
if [[ -z "$SERVICE_BIN" ]]; then
  echo "[w2-smoke] 未找到 planner_map_service 二进制（先构建 map 组件）" >&2
  exit 66
fi

# 定位规划服务二进制（W5）：不存在时使用临时 mock（非算法证据，日志标注）
PLAN_BIN=""
for candidate in "${PLANNER_BUILD_DIR:-}/Src/service/planner_plan_service" \
                 "$REPO_ROOT/build/release/Src/service/planner_plan_service" \
                 "$REPO_ROOT/build/service/Src/service/planner_plan_service" \
                 "$REPO_ROOT/build/map/Src/service/planner_plan_service"; do
  if [[ -n "$candidate" && -x "$candidate" ]]; then
    PLAN_BIN="$candidate"
    break
  fi
done

MAP_VERSION=$(python3 -c "import json; print(json.load(open('$SNAPSHOT'))['map_version'])")

"$SERVICE_BIN" --map "$MAP_OSM" --origin-lat "$ORIGIN_LAT" --origin-lon "$ORIGIN_LON" \
  --port "$PORT" --run-dir "$RUN_DIR" >"$OUT_DIR/service.log" 2>&1 &
SERVICE_PID=$!

if [[ -n "$PLAN_BIN" ]]; then
  echo "[w5-smoke] plan service: $PLAN_BIN (real)"
  "$PLAN_BIN" --map "$MAP_OSM" --origin-lat "$ORIGIN_LAT" --origin-lon "$ORIGIN_LON" \
    --port "$PLAN_PORT" --run-dir "$RUN_DIR" >"$OUT_DIR/plan_service.log" 2>&1 &
  PLAN_KIND="real"
else
  echo "[w5-smoke] plan service: mock_plan_service.py (interim; 非算法正确性证据)"
  python3 "$SCRIPT_DIR/mock_plan_service.py" --port "$PLAN_PORT" --map-version "$MAP_VERSION" \
    >"$OUT_DIR/plan_service.log" 2>&1 &
  PLAN_KIND="mock"
fi
PLAN_PID=$!

# 版本不匹配 mock（409 stale_map 演示）
python3 "$SCRIPT_DIR/mock_plan_service.py" --port "$PLAN_STALE_PORT" --map-version "stale-v0" \
  >"$OUT_DIR/plan_service_stale.log" 2>&1 &
PLAN_STALE_PID=$!

cleanup() {
  for pid in "$SERVICE_PID" "$PLAN_PID" "${PLAN_STALE_PID:-}" "${GRID_MOCK_PID:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

echo "[w2-smoke] map service=$SERVICE_BIN port=$PORT"
python3 - "$PORT" <<'PY'
import sys, time, urllib.request
port = int(sys.argv[1])
deadline = time.time() + 60
while time.time() < deadline:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/health", timeout=2) as response:
            if response.status == 200:
                print(f"[w2-smoke] map service healthy on port {port}")
                sys.exit(0)
    except Exception:
        time.sleep(0.5)
print("[w2-smoke] map service did not become healthy", file=sys.stderr)
sys.exit(1)
PY

python3 - "$PLAN_PORT" <<'PY'
import sys, time, urllib.request
port = int(sys.argv[1])
deadline = time.time() + 30
while time.time() < deadline:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/health", timeout=2) as response:
            if response.status == 200:
                print(f"[w5-smoke] plan service healthy on port {port}")
                sys.exit(0)
    except Exception:
        time.sleep(0.5)
print("[w5-smoke] plan service did not become healthy", file=sys.stderr)
sys.exit(1)
PY

python3 "$SCRIPT_DIR/service_compare.py" --base-url "http://127.0.0.1:$PORT" \
  --snapshot "$SNAPSHOT" --output-dir "$OUT_DIR"

# 复用对比脚本验证过的连通 lanelet 对作为浏览器路由检查的起终
PAIR=$(python3 - "$OUT_DIR/service_compare.json" <<'PY' || true
import json, sys
try:
    data = json.load(open(sys.argv[1]))
    pair = data.get("checks", {}).get("route_connected_pair", {}).get("detail", {}).get("pair", [])
    print(",".join(pair) if len(pair) == 2 else "")
except Exception:
    print("")
PY
)

CDP_ARGS=(--api-target "http://127.0.0.1:$PORT" --plan-api-target "http://127.0.0.1:$PLAN_PORT" \
  --xw2 --output-dir "$OUT_DIR/browser_cdp_w2")
if [[ -n "$PAIR" ]]; then
  CDP_ARGS+=(--route-pair "$PAIR")
fi
if [[ "$PLAN_KIND" == "real" ]]; then
  # 真实服务支持 /plan/events WebSocket 推送：要求推送通道连通
  CDP_ARGS+=(--expect-ws)
fi
python3 "$SCRIPT_DIR/browser_cdp_check.py" "${CDP_ARGS[@]}" || CDP_MAIN_RC=$?

# W5 陈旧防护（服务端 409 stale_map）：地图服务保持可用（避免客户端阻止），
# 规划服务指向版本不匹配的 mock。
python3 "$SCRIPT_DIR/browser_cdp_check.py" \
  --api-target "http://127.0.0.1:$PORT" \
  --plan-api-target "http://127.0.0.1:$PLAN_STALE_PORT" --expect-stale-map \
  --output-dir "$OUT_DIR/browser_cdp_w5_stale" || CDP_STALE_RC=$?

# 停止 stale mock，释放并发名额（后续 X-W2 mock 阶段峰值保持 map+plan+mock+chrome=4）。
if kill -0 "$PLAN_STALE_PID" 2>/dev/null; then
  kill "$PLAN_STALE_PID" 2>/dev/null || true
  wait "$PLAN_STALE_PID" 2>/dev/null || true
fi
PLAN_STALE_PID=""

# X-W2 mock 阶段：栅格服务替身（非算法证据）复现乱序与版本不匹配。
start_grid_mock() {
  python3 "$SCRIPT_DIR/mock_grid_service.py" --port "$GRID_MOCK_PORT" \
    --map-version "$MAP_VERSION" "$@" >"$OUT_DIR/mock_grid_service.log" 2>&1 &
  GRID_MOCK_PID=$!
  python3 - "$GRID_MOCK_PORT" <<'PY'
import sys, time, urllib.request
port = int(sys.argv[1])
deadline = time.time() + 20
while time.time() < deadline:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/health", timeout=2) as response:
            if response.status == 200:
                sys.exit(0)
    except Exception:
        time.sleep(0.3)
print("[xw2-smoke] mock grid service did not become healthy", file=sys.stderr)
sys.exit(1)
PY
}

stop_grid_mock() {
  if [[ -n "${GRID_MOCK_PID:-}" ]] && kill -0 "$GRID_MOCK_PID" 2>/dev/null; then
    kill "$GRID_MOCK_PID" 2>/dev/null || true
    wait "$GRID_MOCK_PID" 2>/dev/null || true
  fi
  GRID_MOCK_PID=""
}

# 1) 乱序响应：首个请求延迟 1.2 s；同时 max_cells=4096 触发服务端降级显示。
start_grid_mock --delay-first-grid-s 1.2 --max-cells 4096
python3 "$SCRIPT_DIR/browser_cdp_check.py" \
  --grid-api-target "http://127.0.0.1:$GRID_MOCK_PORT" --xw2-mock-mode reorder \
  --output-dir "$OUT_DIR/browser_cdp_xw2_reorder" || XW2_REORDER_RC=$?
stop_grid_mock

# 2) 协议主版本不匹配（grid_protocol_version=2.0）→ 在线图层禁用 + 可见状态。
start_grid_mock --grid-protocol-version 2.0
python3 "$SCRIPT_DIR/browser_cdp_check.py" \
  --grid-api-target "http://127.0.0.1:$GRID_MOCK_PORT" --xw2-mock-mode protocol \
  --output-dir "$OUT_DIR/browser_cdp_xw2_protocol" || XW2_PROTOCOL_RC=$?
stop_grid_mock

# 3) 场景组合规则版本不匹配（scene_rules_version=scene_v2）→ 预览禁用、可行驶域仍可用。
start_grid_mock --scene-rules-version scene_v2
python3 "$SCRIPT_DIR/browser_cdp_check.py" \
  --grid-api-target "http://127.0.0.1:$GRID_MOCK_PORT" --xw2-mock-mode scene_rules \
  --output-dir "$OUT_DIR/browser_cdp_xw2_scene_rules" || XW2_SCENE_RULES_RC=$?
stop_grid_mock

echo "[w2-smoke] plan service kind: $PLAN_KIND"
# §5.4：非关键阶段失败不终止后续阶段；末尾汇总首个非零退出码（证据均已保留）。
FIRST_RC=0
for rc in "${CDP_MAIN_RC:-0}" "${CDP_STALE_RC:-0}" "${XW2_REORDER_RC:-0}" "${XW2_PROTOCOL_RC:-0}" "${XW2_SCENE_RULES_RC:-0}"; do
  if [[ "$rc" -ne 0 && "$FIRST_RC" -eq 0 ]]; then
    FIRST_RC="$rc"
  fi
done
if [[ "$FIRST_RC" -ne 0 ]]; then
  echo "[w2-smoke] 阶段失败汇总：main=${CDP_MAIN_RC:-0} stale=${CDP_STALE_RC:-0} reorder=${XW2_REORDER_RC:-0} protocol=${XW2_PROTOCOL_RC:-0} scene_rules=${XW2_SCENE_RULES_RC:-0}" >&2
  exit "$FIRST_RC"
fi
echo "[w2-smoke] OK"
