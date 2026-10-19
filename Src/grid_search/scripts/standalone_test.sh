#!/usr/bin/env bash
# X-S3 开发期独立测试（CTest 标签 grid_search）。
#
# 用法：
#   Src/grid_search/scripts/standalone_test.sh <build-dir> [workers] [ctest 额外参数...]
#
# 规则：通过 tools/task_runner.py exec 调用（占 worker 名额、走监控门禁）；
# 默认 1 个 ctest 并行 worker，不超过全局 4 worker 上限。
set -euo pipefail

BUILD_DIR="${1:?usage: standalone_test.sh <build-dir> [workers] [extra ctest args...]}"
WORKERS="${2:-1}"
shift 2 || true

cd "${BUILD_DIR}"
exec ctest -L grid_search --output-on-failure --parallel "${WORKERS}" "$@"
