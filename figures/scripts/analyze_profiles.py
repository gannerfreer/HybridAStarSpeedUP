#!/usr/bin/env python3
"""Phase 4 归因：callgrind 剖面 → 7 桶（函数名匹配，三状态同一套映射）。

用法:
  analyze_profiles.py [--smoke <outfile> <stage> <case>]   # 快速单文件校验
  analyze_profiles.py                                      # 解析 9 个官方剖面并出 CSV

映射规则（按优先级；依据 T1 证据 PROFILING.md 的桶定义重建，脚本自包含）:
  1. 碰撞几何/碰撞检查       : 文件在 /Src/collision/ 下 或 函数名含 planner::collision::
  2. RS 距离求解/连接        : planner::rs:: / RsProvider / try_connection / reeds
  3. 势场 Dijkstra           : grid_potential / compute_grid_potential / potential / grid_cache_key
  4. libm hypot/三角函数     : 词元 hypot/sincos/sin/cos/tan(±a)/asin/acos/atan2/atan/
                              sqrt/fmod/floor/__ieee754_*（glibc 数学库符号）
  5. 内存分配                : malloc/free/operator new/delete/basic_string 系列
  6. 哈希/容器               : hashtable/_Rb_tree/unordered_/stl_vector/vector.tcc/
                              basic_json/nlohmann/emplace_back/push_back/_M_fill_assign 等
  7. 其它/未分类             : 其余（含 99.99% 阈值外残余）
"""
from __future__ import annotations

import csv
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

VG_BIN = Path.home() / ".local/valgrind/usr/bin"
VG_LIB = Path.home() / ".local/valgrind/usr/lib/x86_64-linux-gnu/valgrind"
ROOT = Path("/home/gray/Planner/HybridAStarSpeedUP")
PROF = ROOT / "profiling"
FIG_DATA = ROOT / "figures" / "data"

COLL = "碰撞几何/碰撞检查"
LIBM = "libm hypot/三角函数（多为碰撞几何内层）"
HASH = "哈希/容器"
POT = "势场 Dijkstra"
MEM = "内存分配"
RS = "RS 距离求解/连接"
OTHER = "其它/未分类"
ORDER = [COLL, LIBM, HASH, POT, MEM, RS, OTHER]

LIBM_RE = re.compile(
    r"(?<![A-Za-z0-9])(?:__)?(?:hypot|sincos|sin|cos|tan|asin|acos|atan2|atan|"
    r"sqrt|fmod|floor|__ieee754_[a-z0-9_]+)(?![A-Za-z0-9])")
RS_RE = re.compile(r"planner::rs::|RsProvider|try_connection|reeds", re.I)
POT_RE = re.compile(r"grid_potential|compute_grid_potential|potential|grid_cache_key")
MEM_RE = re.compile(
    r"malloc|sysmalloc|free\b|unlink_chunk|munmap_chunk|operator new|operator delete|"
    r"_Znwm|_ZdlPv|basic_string")
HASH_RE = re.compile(
    r"hashtable|_Rb_tree|unordered_|stl_vector|stl_map|stl_tree|vector\.tcc|"
    r"basic_json|nlohmann|emplace_back|push_back|_M_fill_assign|_M_realloc_insert|"
    r"__to_xstring|_M_fill_insert")

ROW_RE = re.compile(r"^\s*([\d,]+)\s+(\S.*?)\s*$")
TAG_RE = re.compile(r"\s+\[([^\]]*)\]\s*$")
TOTAL_RE = re.compile(r"^\s*([\d,]+)\s+PROGRAM TOTALS\s*$")


def split_payload(payload: str) -> tuple[str, str]:
    """payload = file:function [obj] → (file, function)。"""
    mtag = TAG_RE.search(payload)
    if mtag and ("/" in mtag.group(1) or "." in mtag.group(1)) and "(" not in mtag.group(1):
        payload = payload[:mtag.start()]
    head = payload.split("(", 1)[0]
    idx = head.find(":")
    if idx <= 0:
        idx = payload.find(":")
    if idx < 0:
        return "", payload
    return payload[:idx].strip(), payload[idx + 1:].strip()


def bucket_of(bfile: str, func: str) -> str:
    f = bfile or ""
    if "/Src/collision/" in f or "/planner/collision/" in f or "planner::collision::" in func:
        return COLL
    if RS_RE.search(func) or "/Src/rs/" in f:
        return RS
    if POT_RE.search(func):
        return POT
    if LIBM_RE.search(func) and "/Src/" not in f:
        return LIBM
    if MEM_RE.search(func):
        return MEM
    if HASH_RE.search(func) or f.endswith(("stl_vector.h", "hashtable.h", "hashtable_policy.h", "stl_tree.h", "stl_map.h", "json.hpp")):
        return HASH
    return OTHER


def annotate(out_file: Path) -> tuple[int, list[tuple[str, str, int]]]:
    env = {"VALGRIND_LIB": str(VG_LIB), "PATH": "/usr/bin:/bin:" + str(VG_BIN), "HOME": str(Path.home())}
    proc = subprocess.run(
        [str(VG_BIN / "callgrind_annotate"), "--threshold=99.99", "--show=Ir", str(out_file)],
        capture_output=True, text=True, env=env, check=True, timeout=600)
    total = 0
    rows: list[tuple[str, str, int]] = []
    for line in proc.stdout.splitlines():
        mt = TOTAL_RE.match(line)
        if mt:
            total = int(mt.group(1).replace(",", ""))
            continue
        m = ROW_RE.match(line)
        if not m:
            continue
        n = int(m.group(1).replace(",", ""))
        payload = m.group(2).strip()
        if payload in ("PROGRAM TOTALS", "Ir", "file:function") or payload.startswith("Events"):
            continue
        bfile, func = split_payload(payload)
        rows.append((bfile, func, n))
    return total, rows


def main() -> int:
    # smoke: 单文件
    if len(sys.argv) >= 3 and sys.argv[1] == "--smoke":
        total, rows = annotate(Path(sys.argv[2]))
        agg: dict[str, int] = defaultdict(int)
        nrows = 0
        for bfile, func, n in rows:
            agg[bucket_of(bfile, func)] += n
            nrows += 1
        print(f"total={total:,} rows={nrows}")
        s = 0
        for b in ORDER:
            print(f"  {b}: {agg[b]:,} ({agg[b]/total*100:.3f}%)")
            s += agg[b]
        print(f"  sum={s:,} residual={total-s:,} ({(total-s)/total*100:.4f}%)")
        return 0

    # 官方矩阵
    cases = {
        "case_30": ("v3", "v8", "parking_bench.json"),
        "case_39": ("v3", "v8", "parking_bench.json"),
        "pb-017": ("v4", "v9", "parkbench_vehicle.json"),
    }
    stages = [("base", "原始"), ("hypot", "+hypot"), ("sincos", "+hypot +sincos")]
    stage_label = dict(stages)

    per_ir: dict[tuple[str, str, str], int] = {}
    totals: dict[tuple[str, str], int] = {}
    func_rows: list[list] = []
    for stage, label in stages:
        for case in cases:
            out = PROF / f"callgrind.{stage}.{case}.out"
            total, rows = annotate(out)
            totals[(stage, case)] = total
            agg: dict[str, int] = defaultdict(int)
            for bfile, func, n in rows:
                b = bucket_of(bfile, func)
                agg[b] += n
                func_rows.append([stage, label, case, b, bfile, func, n])
            agg[OTHER] += total - sum(agg.values())
            for b in ORDER:
                per_ir[(stage, case, b)] = agg[b]
            print(f"[{stage}/{case}] total={total:,} "
                  + " ".join(f"{b.split('/')[0]}={agg[b]/total*100:.1f}%" for b in ORDER))

    FIG_DATA.mkdir(parents=True, exist_ok=True)
    with (FIG_DATA / "stages_totals.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["stage", "stage_label", "case", "total_ir"])
        for stage, label in stages:
            for case in cases:
                w.writerow([stage, label, case, totals[(stage, case)]])
            mean = sum(totals[(stage, c)] for c in cases) / len(cases)
            w.writerow([stage, label, "宏平均", round(mean)])

    with (FIG_DATA / "stages_buckets.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["stage", "stage_label", "case", "bucket", "ir", "share_pct", "total_ir"])
        for stage, label in stages:
            for case in cases:
                tot = totals[(stage, case)]
                for b in ORDER:
                    ir = per_ir[(stage, case, b)]
                    w.writerow([stage, label, case, b, ir, round(ir / tot * 100, 4), tot])
            for b in ORDER:
                mean_ir = sum(per_ir[(stage, c, b)] for c in cases) / len(cases)
                mean_tot = sum(totals[(stage, c)] for c in cases) / len(cases)
                w.writerow([stage, label, "宏平均", b, round(mean_ir),
                            round(mean_ir / mean_tot * 100, 4), round(mean_tot)])

    with (FIG_DATA / "functions_all.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["stage", "stage_label", "case", "bucket", "file", "function", "self_ir"])
        w.writerows(func_rows)

    print("CSV written to", FIG_DATA)
    return 0


if __name__ == "__main__":
    sys.exit(main())
