#!/bin/bash
# Phase 3 全量时间测试驱动：3 轮轮转交错（套件级），状态顺序轮换。
# 每轮每状态依次跑 v3、v4（每套件一个进程，冷启动单次实测）。
set -u
cd /home/gray/Planner/HybridAStarSpeedUP
mkdir -p timing
WALL=timing/wall_log.tsv
: > "$WALL"
printf "round\tstate\tsuite\tstart_time\twall_s\tload_before\tload_after\trc\n" >> "$WALL"

run_one() { # r state suite fixtures vehicle
  local r=$1 state=$2 suite=$3 fixtures=$4 vehicle=$5
  local out="timing/r${r}/${state}/${suite}"
  mkdir -p "$out"
  local la_before la_after t0 t1 rc
  la_before=$(cat /proc/loadavg | cut -d' ' -f1-3)
  t0=$(date +%s.%N)
  taskset -c 6 ./bin/bench-${state} --suite "${suite}" \
    --fixtures-dir "tests/fixtures/${fixtures}" --vehicle "${vehicle}" \
    --output-dir "$out" --cold-only --warmup-runs 0 --measured-runs 1 \
    --jobs 1 --time-budget-s 120 --backends ros_free > "$out/stdout.log" 2>&1
  rc=$?
  t1=$(date +%s.%N)
  la_after=$(cat /proc/loadavg | cut -d' ' -f1-3)
  local wall
  wall=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.3f", b-a}')
  printf "r%s\t%s\t%s\t%s\t%.3f\t%s\t%s\t%s\n" "$r" "$state" "$suite" \
    "$(date +%H:%M:%S)" "$wall" "$la_before" "$la_after" "$rc" >> "$WALL"
  echo "[$(date +%H:%M:%S)] r$r $state $suite rc=$rc wall=${wall}s load_before=$la_before load_after=$la_after"
}

run_suite() { # r state
  run_one "$1" "$2" v3 v8 parking_bench.json
  run_one "$1" "$2" v4 v9 parkbench_vehicle.json
}

echo "== R1: base -> hypot -> sincos =="
run_suite 1 base
run_suite 1 hypot
run_suite 1 sincos
echo "== R2: hypot -> sincos -> base =="
run_suite 2 hypot
run_suite 2 sincos
run_suite 2 base
echo "== R3: sincos -> base -> hypot =="
run_suite 3 sincos
run_suite 3 base
run_suite 3 hypot
echo "== done =="
date -u
