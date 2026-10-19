#!/bin/bash
# Phase 4: 三状态 × 三代表例 callgrind（串行，E 核 12，budget 7200）
set -u
cd /home/gray/Planner/HybridAStarSpeedUP
mkdir -p profiling/out profiling/logs
export VALGRIND_LIB=$HOME/.local/valgrind/usr/lib/x86_64-linux-gnu/valgrind
VG=$HOME/.local/valgrind/usr/bin/valgrind.bin
WALL=profiling/wall_log.tsv
: > "$WALL"
printf "state\tcase\tsuite\twall_s\tfinish_time\trc\n" >> "$WALL"

run() { # state suite fixtures vehicle case
  local state=$1 suite=$2 fixtures=$3 vehicle=$4 case=$5
  local out="profiling/callgrind.${state}.${case}.out"
  local log="profiling/logs/${state}.${case}.log"
  local oob="profiling/out/${state}.${case}"
  mkdir -p "$oob"
  local t0 t1 rc
  t0=$(date +%s)
  taskset -c 12 "$VG" --tool=callgrind --callgrind-out-file="$out" \
    "./bin/bench-${state}-prof" --suite "$suite" \
    --fixtures-dir "tests/fixtures/${fixtures}" --vehicle "$vehicle" \
    --output-dir "$oob" --cold-only --warmup-runs 0 --measured-runs 1 --jobs 1 \
    --time-budget-s 7200 --backends ros_free --case-ids "$case" > "$log" 2>&1
  rc=$?
  t1=$(date +%s)
  printf "%s\t%s\t%s\t%s\t%s\t%s\n" "$state" "$case" "$suite" "$((t1-t0))" "$(date +%H:%M:%S)" "$rc" >> "$WALL"
  echo "[$(date +%H:%M:%S)] $state/$case rc=$rc wall=$((t1-t0))s"
}

echo "== 快例 pb-017 =="
for s in base hypot sincos; do run "$s" v4 v9 parkbench_vehicle.json pb-017; done
echo "== 中例 case_30 =="
for s in base hypot sincos; do run "$s" v3 v8 parking_bench.json case_30; done
echo "== 慢例 case_39 =="
for s in base hypot sincos; do run "$s" v3 v8 parking_bench.json case_39; done
echo "== done =="
date -u
