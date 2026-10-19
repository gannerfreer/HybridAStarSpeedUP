#!/usr/bin/env bash
# X-B1 栅格基准套件编排：生成实例 → 独立 Python 参考 → C++ 基准运行器。
#
# 用法（必须经 tools/task_runner.py exec 调用：占 worker 名额、走监控门禁；
# 正式全矩阵属独占窗口阶段）：
#   run_suite.sh --build-dir <dir> --output-dir <dir> [--smoke] [--capacity]
#     [--capacity-gen] [--backends a,b] [--neighborhoods eight,four]
#     [--warmup 5] [--trials 30] [--corner-cut] [-- <额外 bench 参数>]
#
# 产物（全部在 --output-dir 内，绝不写到目录之外）：
#   scenario/instances.json、scenario/instances/*.u8、scenario/instances.sha256、
#   scenario/generation_summary.json
#   reference/reference.json、reference/fields/*.f64（逐单元 float64 参考场）
#   benchmark_metrics.csv、bench_summary.json、workers/<backend>/{metrics.csv,summary.json}
#   capacity_scenario/...（--capacity-gen 时：2048²/4096² 生成烟测；>10 MiB
#   按 §3.4 归档到 <repo>/artifacts/data/sha256/，运行目录只留 sidecar）
#
# 模式：
#   smoke    128² 子集（6 类 × seed 0）+ 默认 1–2 后端；
#   formal   128²/512²/1024² × 5 种子 × 6 类 = 90 实例 × 全部后端；
#   capacity 2048²/4096² 单次（种子 0），扩重复按 §3.3 自动评估。

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR=""
OUTPUT_DIR=""
MODE="formal"
BACKENDS=""
NEIGHBORHOODS="eight,four"
WARMUP=""
TRIALS=""
CAPACITY_GEN=0
CORNER_CUT=0
ARCHIVE_DIR="${REPO_ROOT}/artifacts/data/sha256"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --mode) MODE="$2"; shift 2 ;;
    --smoke) MODE="smoke"; shift ;;
    --capacity) MODE="capacity"; shift ;;
    --capacity-gen) CAPACITY_GEN=1; shift ;;
    --backends) BACKENDS="$2"; shift 2 ;;
    --neighborhoods) NEIGHBORHOODS="$2"; shift 2 ;;
    --warmup) WARMUP="$2"; shift 2 ;;
    --trials) TRIALS="$2"; shift 2 ;;
    --corner-cut) CORNER_CUT=1; shift ;;
    --) shift; EXTRA_ARGS=("$@"); break ;;
    *) echo "run_suite.sh: unknown argument: $1" >&2; exit 2 ;;
  esac
done

[[ -n "${BUILD_DIR}" ]] || { echo "run_suite.sh: --build-dir is required" >&2; exit 2; }
[[ -n "${OUTPUT_DIR}" ]] || { echo "run_suite.sh: --output-dir is required" >&2; exit 2; }

GEN=""
BENCH=""
BENCH_DIR=""
for candidate in "${BUILD_DIR}/benchmarks" \
                 "${BUILD_DIR}/Src/grid_search/benchmarks"; do
  if [[ -x "${candidate}/planner_grid_scenario_gen" && \
        -x "${candidate}/planner_grid_search_bench" ]]; then
    BENCH_DIR="${candidate}"
    break
  fi
done
if [[ -z "${BENCH_DIR}" ]]; then
  echo "run_suite.sh: cannot find benchmark binaries under ${BUILD_DIR}" \
       "(tried benchmarks/ and Src/grid_search/benchmarks/)" >&2
  exit 2
fi
GEN="${BENCH_DIR}/planner_grid_scenario_gen"
BENCH="${BENCH_DIR}/planner_grid_search_bench"

mkdir -p "${OUTPUT_DIR}/scenario" "${OUTPUT_DIR}/reference"

case "${MODE}" in
  smoke)
    GEN_ARGS=(--classes open,maze,corridor,deadend,unreachable,border_corner
              --scales 128 --seeds 0)
    [[ -n "${BACKENDS}" ]] || BACKENDS="vss2sn_astar,vss2sn_dijkstra"
    [[ -n "${WARMUP}" ]] || WARMUP=5
    [[ -n "${TRIALS}" ]] || TRIALS=30
    ;;
  formal)
    GEN_ARGS=()
    [[ -n "${WARMUP}" ]] || WARMUP=5
    [[ -n "${TRIALS}" ]] || TRIALS=30
    ;;
  capacity)
    GEN_ARGS=(--capacity)
    [[ -n "${WARMUP}" ]] || WARMUP=0
    [[ -n "${TRIALS}" ]] || TRIALS=1
    ;;
  *)
    echo "run_suite.sh: unknown mode: ${MODE}" >&2
    exit 2
    ;;
esac

# 实例生成（确定性；容量档 4096² 的 16 MiB 打包输入按 §3.4 归档）。
echo "== grid_scenario_gen v1 (${MODE}) =="
"${GEN}" --suite v1 --output-dir "${OUTPUT_DIR}/scenario" \
  --repo-root "${REPO_ROOT}" --archive-dir "${ARCHIVE_DIR}" \
  "${GEN_ARGS[@]}"

# 可选：容量档输入生成烟测（不运行容量搜索）。
if [[ "${CAPACITY_GEN}" -eq 1 ]]; then
  echo "== grid_scenario_gen v1 (capacity generation smoke) =="
  "${GEN}" --suite v1 --output-dir "${OUTPUT_DIR}/capacity_scenario" \
    --capacity --repo-root "${REPO_ROOT}" --archive-dir "${ARCHIVE_DIR}"
fi

# jps3d_jps 单列变体需要“允许对角穿角”参考：默认全后端（含 jps3d_jps）时启用。
if [[ "${CORNER_CUT}" -eq 0 ]]; then
  if [[ -z "${BACKENDS}" || ",${BACKENDS}," == *",jps3d_jps,"* ]]; then
    CORNER_CUT=1
  fi
fi

# 独立 Python 参考（逐单元距离场参考 + 点到点参考；容差见计划 §6.2）。
echo "== grid_bench_reference v1 =="
REF_ARGS=(--instances "${OUTPUT_DIR}/scenario/instances.json"
          --output "${OUTPUT_DIR}/reference/reference.json"
          --fields-dir "${OUTPUT_DIR}/reference/fields"
          --repo-root "${REPO_ROOT}"
          --neighborhoods "${NEIGHBORHOODS}")
if [[ "${CORNER_CUT}" -eq 1 ]]; then
  REF_ARGS+=(--corner-cut)
fi
python3 "${REPO_ROOT}/Src/grid_search/benchmarks/reference/grid_bench_reference.py" \
  "${REF_ARGS[@]}"

# C++ 基准运行器（每后端独立子进程；父进程合并 metrics 与 summary）。
echo "== planner_grid_search_bench (${MODE}) =="
BENCH_ARGS=(--suite v1 --output-dir "${OUTPUT_DIR}"
            --instances "${OUTPUT_DIR}/scenario/instances.json"
            --reference "${OUTPUT_DIR}/reference/reference.json"
            --repo-root "${REPO_ROOT}"
            --neighborhoods "${NEIGHBORHOODS}"
            --warmup "${WARMUP}" --trials "${TRIALS}")
if [[ -n "${BACKENDS}" ]]; then
  BENCH_ARGS+=(--backends "${BACKENDS}")
fi
if [[ "${MODE}" == "capacity" ]]; then
  BENCH_ARGS+=(--capacity --expand-repeats)
fi
"${BENCH}" "${BENCH_ARGS[@]}" "${EXTRA_ARGS[@]}"

echo "== done: ${OUTPUT_DIR} =="
