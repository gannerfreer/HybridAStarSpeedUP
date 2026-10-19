#!/usr/bin/env bash
# X-S3 开发期独立构建（不经过根 CMake；接线后由根项目构建）。
#
# 用法：
#   Src/grid_search/scripts/standalone_build.sh <build-dir> [cmake 额外参数...]
#
# 规则（AGENTS.md / 计划 §7）：
#   * 编译在命令内持全局编译锁 flock（.git/planner-runtime/build.lock）；
#   * 并行度固定 4（-j4 上限）；
#   * 本脚本必须通过 tools/task_runner.py exec 调用（占 worker 名额、走监控门禁）。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUILD_DIR="${1:?usage: standalone_build.sh <build-dir> [extra cmake args...]}"
shift || true

LOCK="${REPO_ROOT}/.git/planner-runtime/build.lock"
NLOHMANN_SRC="${REPO_ROOT}/build/release/_deps/nlohmann_json-src"
GTEST_SRC="${REPO_ROOT}/build/release/_deps/googletest-src"

run_build() {
  set -euo pipefail
  local args=()
  if [[ -d "${NLOHMANN_SRC}" ]]; then
    args+=("-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${NLOHMANN_SRC}")
  fi
  if [[ -d "${GTEST_SRC}" ]]; then
    args+=("-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=${GTEST_SRC}")
  fi
  cmake -S "${REPO_ROOT}/Src/grid_search" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release "${args[@]}" "$@"
  cmake --build "${BUILD_DIR}" --parallel 4
}

export REPO_ROOT BUILD_DIR NLOHMANN_SRC GTEST_SRC
export -f run_build
flock -w 1800 "${LOCK}" bash -c 'run_build "$@"' bash "$@"
